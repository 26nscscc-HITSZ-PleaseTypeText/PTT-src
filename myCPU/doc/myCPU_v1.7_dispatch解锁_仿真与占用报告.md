# myCPU：dispatch 解锁 rename 阻塞 — 仿真与队列占用报告

> 日期：2026-07-17  
> RTL：`IP/myCPU`（已并入 `backend_v1.7_解锁dispatch对rename的阻塞`）  
> 负载：`func_lab19` digftest；Linux 启至 `/ #`  
> 总线：`BUS_DELAY_RANDOM_SEED=5570815`  
> 探针：`mycpu_top.v` 中 `myCPU PERF (sim)`（综合 translate_off，不占 FPGA）

---

## 0. 结论摘要

| 项 | 结果 |
|----|------|
| **func_lab19** | **通过**（`HIT GOOD TRAP` / `END by Syscall`） |
| **Linux** | **通过**（检测到 shell `/ #`，干净停仿打 PERF） |
| Commit IPC | lab19 **0.213**；Linux **0.561** |
| v1.7 是否在核内 | **是**（`dispatch.v`：同拍 vacate + rename refill） |
| 板上资源能否大砍队列 | **不宜大砍**；瓶颈在访存侧与 BPU 训练 FIFO，多数后端队列已顶满有效容量 |

**一句话：** 功能/OS 都过了；Linux 下 ROB / RS_MEM / SB / FTQ / RAS / 训练更新队列经常顶满，**缩 ROB/RS_MEM/SB/FTQ/RAS 会伤 IPC**；真正可抠的空间主要在 **TAGE/FTB 表项与 Cache 几何**（需另测），而不是再砍这几条小 FIFO。

---

## 1. v1.7 优化是什么

路径已不在 `IP/` 下（历史合并进 myCPU）。核心改动在 `backend/issue/dispatch.v`：

```329:334:IP/myCPU/backend/issue/dispatch.v
// 槽已空，或本拍所有 valid 槽都会 fire：下沿 rename 可覆盖装入新一对。
// RS push 仍使用覆盖前的当前锁存值，因此可以同拍完成 vacate + refill，
// 消除 dispatch 对 rename 固有的隔拍阻塞。
assign dispatch_ready_o =
    (!dis0_valid_i || dis0_fire_o) &&
    (!dis1_valid_i || dis1_fire_o);
```

含义：

- 旧行为：dispatch 槽未完全空时 `dispatch_ready=0` → rename 隔拍才能再装，吞吐被人为砍半。
- 新行为：本拍将 fire 的槽视为即将 vacated，rename 可同拍 refill；RS push 仍用覆盖前的锁存值。
- PERF 中的 `turnover_refill` 即「rename 分配时 dispatch 仍有 valid」的拍数，用来观察同拍周转是否在工作。

本拍配置（`mycpu.h`）关键深度：

| 结构 | 宏 | 物理容量 | 探针有效容量 |
|------|-----|----------|--------------|
| ROB | `ROB_SIZE=32` | 16 对 / 32 项 | **11 对 / 22 项**（`ROB_GUARD=5`） |
| RS ALU×2 | `RS_ALU_SIZE=4` | 4 | 4 |
| RS MEM | `RS_MEM_SIZE=4` | 4 | 4 |
| RS MDU | `RS_MDU_SIZE=2` | 2 | 2 |
| Store Buffer | `SB_SIZE=8` | 8 | 8 |
| IB | `IB_SIZE=16` | 16 | 16 |
| FTQ | `FTQ_SIZE=16` | 16 | 保留有效 **14** |
| RAS | `RAS_DEPTH=32` | 32 | 32 |
| FTB/TAGE 更新 FIFO | 8 / 16 | 满则 overflow | — |

---

## 2. 功能与性能总表

### 2.1 通过性

| 负载 | 判定 | 关键日志 |
|------|------|----------|
| func_lab19 | PASS | `nemu: HIT GOOD TRAP`；`END by Syscall`；`Reached test end PC` |
| Linux → `/ #` | PASS | 进 shell；`Detected Linux shell '/ #'`；`Reached Linux shell prompt` |

lab19 前段大量 `INVALID INST` / 非对齐访存 / `INVTLB` INE / `CACOP` 为 **测试程序有意触发异常路径**，不是核挂死；最终仍 GOOD TRAP。

### 2.2 Commit / 发射吞吐

| 指标 | func_lab19 | Linux |
|------|------------|-------|
| retire / cycles | 122875 / 577522 | 124843937 / 222567129 |
| **Commit IPC** | **0.212762** | **0.560927** |
| Commit 双发率（相对有提交拍） | 31.56% | 35.31% |
| Commit 双发率（相对全周期） | 5.10% | 14.64% |
| 相对双发峰值利用率 | 10.64% of 2.0 | 28.05% of 2.0 |
| Dispatch 双发率 | **71.73%** | 56.27% |
| Dispatch 双 ALU 拍 | 50122 | 16710049 |
| rename alloc 拍 | 80477 | 91857479 |
| **turnover_refill**（同拍周转） | **40486** | **68514227** |
| dispatch_stall（IB 有货且 !ready 且 !rob_full） | 62427 | 23777669 |
| rob_stall（IB 有货且 rob_full） | 17146 | 8528959 |

解读：

- `turnover_refill` 很大，说明 v1.7 同拍 refill **确实在跑**，不是死代码。
- lab19 的 dispatch 双发率很高（~72%），但整体 IPC 仍低，主因是 **短测 + 大量异常/特权/CACOP 路径**，不是 dispatch 堵死。
- Linux IPC 0.56 已合理；进一步空间主要在 **访存停顿**（见下节），不是 rename 隔拍。

### 2.3 分支 / Cache

| 指标 | func_lab19 | Linux |
|------|------------|-------|
| BPU all-br 准确率 | 89.49% | **92.25%** |
| BPU cond 准确率 | 91.25% | **96.19%** |
| FTB P1 hit | 2.85%（lab 冷） | **36.73%** |
| I$ hit（cached） | 73.65% | **96.03%** |
| D$ hit（cached） | 64.62%（样本极少） | **96.40%** |

### 2.4 Stall / 访存压力（占全周期）

| 来源 | lab19 | Linux | 含义 |
|------|-------|-------|------|
| ROB full | 2.97% | **3.96%** | 窗口顶满，不宜再缩 ROB |
| SB full | 0% | **2.22%** | Linux 已碰 SB 顶 |
| D$ MWAIT | 0% | 1.03% | |
| D$ pend | ~0% | **12.30%** | miss 下挂起队列忙 |
| D$ MSHR busy | 0.12% | **18.58%** | 单 MSHR 瓶颈明显 |
| LSU DC wait | 8.40% | **12.57%** | 等 D$ |
| RS_MEM src stall | 9.51% | **15.71%** | 操作数未就绪 |
| RS_MEM LSU stall | 4.58% | 5.50% | 下游 LSU 忙 |
| RS_MEM **full** stall | **0%** | **0%** | 见占用节说明 |

Linux 主瓶颈链：**MSHR/pend → LSU/D$ wait → RS_MEM 源停滞 → ROB 填满**。  
再缩后端队列只会更早堵死，不会省多少关键路径延迟。

---

## 3. 队列占用（avg / max / 有效 cap / ≥75% / full）

格式与仿真打印一致。

### 3.1 func_lab19

| 结构 | avg | max | cap | ≥75% cyc | full cyc |
|------|-----|-----|-----|----------|----------|
| ROB pairs | 2.129 | **11** | 11 | 88378 | 17170 |
| ROB valid | 3.501 | **22** | 22 | 26173 | 34 |
| Dispatch slots | 0.377 | 2 | 2 | 58905 | 58905 |
| RS ALU0 | 0.211 | 4 | 4 | 196 | 35 |
| RS ALU1 | 0.168 | 4 | 4 | 1499 | 1029 |
| RS MEM | 0.580 | **4** | 4 | 83247 | 77371 |
| RS MDU | 0.022 | 2 | 2 | 1501 | 1501 |
| Store buffer | 0.114 | 4 | 8 | 0 | 0 |
| IB | 2.416 | 16 | 16 | 77085 | 12057 |
| FTQ retained | 10.098 | 15 | 14 | 340430 | 296358 |
| FTQ→IFU pending | 6.674 | 14 | 16 | 187535 | 0 |
| RAT busy | 1.593 | 11 | 31 | 0 | 0 |
| RAS speculative | 20.918 | **32** | 32 | 392932 | 18829 |
| RAS committed | 22.485 | **32** | 32 | 411689 | 54679 |
| FTB update Q | — | **8/8** | 8 | — | overflow=**1201** |
| TAGE update Q | — | **16/16** | 16 | — | overflow=**319** |

### 3.2 Linux → `/ #`

| 结构 | avg | max | cap | ≥75% cyc | full cyc |
|------|-----|-----|-----|----------|----------|
| ROB pairs | 4.268 | **11** | 11 | 41397784 | 8804229 |
| ROB valid | 6.977 | **22** | 22 | 27979364 | 532362 |
| Dispatch slots | 0.845 | 2 | 2 | 70777066 | 70777066 |
| RS ALU0 | 0.553 | **4** | 4 | 8122964 | 3384972 |
| RS ALU1 | 0.367 | **4** | 4 | 6390262 | 2705666 |
| RS MEM | 1.017 | **4** | 4 | 50465295 | 25178712 |
| RS MDU | 0.026 | 2 | 2 | 1384587 | 1384587 |
| Store buffer | 0.905 | **8** | 8 | 12514687 | 4951031 |
| IB | 3.422 | **16** | 16 | 23339333 | 2077494 |
| FTQ retained | −7.354† | 15 | 14 | 167881333 | 133908524 |
| FTQ→IFU pending | 6.470 | 14 | 16 | 36902275 | 0 |
| RAT busy | 2.480 | 18 | 31 | 0 | 0 |
| RAS speculative | −9.140† | **32** | 32 | 8832701 | 145582 |
| RAS committed | −8.856† | **32** | 32 | 9121026 | 190058 |
| FTB update Q | — | **8/8** | 8 | — | overflow=**869726** |
| TAGE update Q | — | **16/16** | 16 | — | overflow=**168510** |

† Linux 下 FTQ/RAS 的 **avg 为负**：指针差/`cnt` 被零扩展成无符号后按有符号 `real` 打印时的展示问题（或长期运行环差溢出），**max / ≥75% / full 仍可信**。后续可改探针为饱和无符号或取模距离。

**关于 RS_MEM full stall=0% 但 occupancy full 很高：**  
`full stall` 计数的是「因 RS 满而派不出」的组合条件；当前 dispatch 入站策略 + LSU/源停滞主导时，占用可长期顶满，但「额外因满而 stall」的计数器路径可能很少触发。看 **max=cap 与 full cyc** 更合适：Linux 下 RS_MEM 有约 **11%** 周期处于满（25178712/222567129）。

---

## 4. 资源缩减建议（面向下板）

判断准则（同时看 Linux，不以 lab19 为准）：

1. `max` 是否经常碰到 `cap`  
2. `full` / `≥75%` 是否占可观周期  
3. 对应 stall 是否已进 PERF  

### 4.1 不建议缩（会掉 IPC / 风险高）

| 结构 | 理由 |
|------|------|
| **ROB（有效 11 对）** | Linux/lab 均 `max=11`；full ≈4%。再缩直接抬高 `rob_stall` |
| **RS_MEM=4** | 长期顶满；访存本来就是主瓶颈，缩了只会更堵 rename |
| **SB=8** | Linux `max=8`，full 2.22%；lab 虽只用到 4，但 OS 需要 8 |
| **FTQ=16** | retained 长期高水位；注释已写过小会死锁风险 |
| **RAS=32** | `max=32`，满周期存在；深调用链 / 误预测恢复敏感 |
| **IB=16** | Linux `max=16`；前端 4 宽灌入，砍半易让 IFU 更常堵 |

### 4.2 可谨慎试缩（必须重跑 Linux）

| 结构 | 现状 | 建议试值 | 风险 |
|------|------|----------|------|
| **RS ALU0/1=4** | avg≪1，但 Linux 仍会顶满约 1.2~1.5% 周期 | 先试 **3**，勿直接 2 | 双 ALU 突发时抬高 dispatch_stall |
| **RS MDU=2** | avg≈0.02，偶发顶满 | 可试 **1** | 乘除串行突发时阻塞 |
| **IB** | avg 3.4，但 max=16 | 仅当综合 LUT 紧时试 **12→8** | 取指突发 + 后端慢时前端反压 |

以上三项省下的 FF 相对 Cache/TAGE/FTB **很小**，性价比一般。

### 4.3 不该缩、反而该加（或接受丢训练）

| 结构 | 证据 | 建议 |
|------|------|------|
| **FTB update Q=8** | Linux overflow **86.9 万** | 优先 **16→32**；否则训练丢失 → FTB hit 虚高不了 |
| **TAGE update Q=16** | Linux overflow **16.9 万** | 优先 **32** |
| **L1 D$ MSHR=1** | MSHR busy **18.6%** | 若下板时序允许，加 MSHR/pend 比砍 ROB 更赚 IPC |

### 4.4 真正吃 FPGA 的大头（本报告未扫占用，但配置已知）

队列 FIFO 不是板上资源主力。更该评估的是：

| 模块 | 当前配置 | 备注 |
|------|----------|------|
| TAGE base | 8192×2bit | BRAM 大户 |
| TAGE tag×4 | 4×1024 | 大户 |
| FTB | 4-way × 1024 | 大户 |
| L1 I/D | 16KB ×2 | 必要 |
| L2 | 32KB | 必要 |
| ROB×32 项宽字段 | 中等 | 已比「曾用更大 ROB」保守 |

**省资源优先序（建议）：**  
① 评估 TAGE/FTB 表项减半并重测 Linux BPU 准确率 → ② 训练 FIFO 反而加深防 overflow → ③ 后端小队列基本维持 → ④ 最后才动 RS_ALU/MDU。

### 4.5 综合裁剪速查表

| 动作 | 推荐 |
|------|------|
| ROB 32→更小 | ❌ |
| RS_MEM 4→更小 | ❌ |
| SB 8→4 | ❌（Linux 已顶满） |
| FTQ/RAS 缩小 | ❌ |
| RS_ALU 4→3 | ⚠️ 可 A/B |
| RS_MDU 2→1 | ⚠️ 可 A/B |
| FTB/TAGE update Q 加深 | ✅ |
| TAGE/FTB 表减小 | ✅ 作为省 BRAM 主手段（需重测准确率） |

---

## 5. 与 v1.7 相关的观察

1. **同拍周转有效**：`turnover_refill` 在 Linux 达 6.85e7，约占 rename alloc 拍的 **75%**，说明多数分配拍伴随 dispatch 仍有货 — 正是解锁隔拍阻塞后的预期形态。  
2. **dispatch_stall 仍在**：Linux 约 2.38e7 拍。解锁后剩余 stall 主要来自 **RS 接不住 / 下游忙**，不是「空槽却隔拍」。  
3. **Dispatch 双发率 lab19 高于 Linux**：短测指令流更整齐；OS 更多 mem/串行依赖，双发自然下降。  
4. **IPC 0.56 @ Linux** 与历史「双 ALU 分发 + pend」路线一致；本轮重点是确认 v1.7 **功能正确** + **给出缩队列边界**。

---

## 6. 复现命令（手册对齐）

```bash
export CHIPLAB_HOME=~/chiplab
cd $CHIPLAB_HOME/sims/verilator/run_prog

# 改过 RTL 必须重编
make verilator && make testbench

# func_lab19（默认 digftest + bus delay seed 5570815）
./configure.sh --run func/func_lab19
make soft_compile   # 若软件未编好
make

# Linux 至 / #（关 trace；开 UART；停仿打 PERF）
./configure.sh --run linux --disable-simu-trace --disable-trace-comp --disable-read-miss --output-uart-info
make
```

仿真结束搜索：`==================== myCPU PERF (sim) ====================`

---

## 7. 附录：原始 PERF 关键摘录

### func_lab19

```
Commit IPC:         retire=122875  cycles=577522  IPC=0.212762
Commit dual-issue:  dual_cyc=29476  any_cyc=93399  dual_rate=31.56%
Dispatch dual:      dual_cyc=57913  any_cyc=80733  dual_rate=71.73%  dual_ALU=50122
Rename/dispatch:    alloc_cyc=80477  turnover_refill=40486  dispatch_stall=62427  rob_stall=17146
BPU all-branch:     accuracy=89.49%
ROB full: 2.97% | SB full: 0% | D$ MSHR busy: 0.12%
RS_MEM src stall: 9.51% | LSU DC wait: 8.40%
ROB pairs: 2.129/11/11 | RS MEM: 0.580/4/4 | SB: 0.114/4/8
FTB upd overflow=1201 | TAGE upd overflow=319
```

### Linux

```
Commit IPC:         retire=124843937  cycles=222567129  IPC=0.560927
Commit dual-issue:  dual_rate=35.31% (of commit cycles)
Dispatch dual:      dual_rate=56.27%  dual_ALU=16710049
Rename/dispatch:    alloc_cyc=91857479  turnover_refill=68514227
                    dispatch_stall=23777669  rob_stall=8528959
BPU all/cond:       92.25% / 96.19%
I$/D$ hit:          96.03% / 96.40%
ROB full: 3.96% | SB full: 2.22% | D$ pend: 12.30% | MSHR busy: 18.58%
RS_MEM src: 15.71% | LSU DC wait: 12.57%
ROB pairs: 4.268/11/11 | RS MEM: 1.017/4/4 | SB: 0.905/8/8
FTB upd overflow=869726 | TAGE upd overflow=168510
```
