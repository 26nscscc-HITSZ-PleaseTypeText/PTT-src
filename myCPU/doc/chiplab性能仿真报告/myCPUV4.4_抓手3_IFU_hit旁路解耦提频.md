# myCPUV4.4：抓手 3（I$ hit→IFU 预译码→FTQ `blk_target` 解耦）— 性能仿真报告

> 日期：2026-07-27  
> **基线报告/CPU**：[`myCPUV4.3_抓手2_SB_done打拍提频.md`](myCPUV4.3_抓手2_SB_done打拍提频.md)（Linux 软门 IPC **0.703890**；55MHz WNS **+0.153**；70MHz WNS **−0.967** 垄断 `icache→ftq/blk_target`；75MHz WNS **−2.111** 垄断 `tage→ftq/blk_target`）  
> 本轮目标：切断 70MHz 绝对 WNS 族 `icache/req_paddr → I$ rdata → IFU 命中旁路预译码 → FTQ blk_target`；若仍 FAIL 且榜首落到 TAGE/FTB→FTQ，同轮兜底  
> 负载：`func_lab19` digftest（含 7 种子）；coremark / dhrystone digftest；Linux 软门 / digftest  
> seed：`5570815`（`BUS_DELAY_RANDOM_SEED`）  
> 原始 PERF 日志：`IP/myCPU/logs_v44/`  
> 时序报告：`IP/myCPU/logs_v44/reports_impl{55,70,75}/`（同源 `D:/chiplab_v41_sta/out/{55,70,75}mhz_v44*`）；IFU-only 中间态：`reports_impl70_ifuonly/`  
> PERF 口径：仿真结束 `myCPU PERF (sim)`（`mycpu_top.v` 中 `` `ifdef SYNTHESIS ``；仿真经 `-DSIMU`）  
> **等价性口径（本轮）：非逐拍等价**，以 Linux IPC A/B（相对 V4.3 **0.703890** 降幅 **&lt;0.5%**）+ 全量 digftest PASS 验收

---

## 0. 结论摘要

| 门控 | 结果 |
|------|------|
| **lab19 digftest** | **PASS**；retire=**122875**，IPC=**0.213014**；7 种子全过 |
| **coremark / dhrystone** | **PASS**（digftest）；IPC≈**0.201229** / **0.180171** |
| **Linux 软门** | **PASS**（`/ #`）；IPC = **0.703400**（相对 V4.3 0.703890：**−0.07%**） |
| **Linux digftest** | **PASS**（至 `/ #`，无 mismatch）；IPC = **0.703400** |
| **频率 55MHz（合规）** | **FAIL**：WNS=**−0.327**（V4.3 +0.153）；100/100 为 **D$ `req_paddr→valid/dirty_arr`**；同脚本重跑结果逐路径一致 |
| **频率 70MHz（主目标）** | **FAIL**：WNS=**−0.498**（V4.3 −0.967）；**`icache→ftq/blk_target` = 0**；榜首 **`dcache/req_paddr → mshr_line CE`** |
| **频率 75MHz（摸底）** | **FAIL**：WNS=**−1.108**（V4.3 −2.111）；**`tage/ftb→ftq` / `blk_target` = 0**；榜首 **`SB/tail → rs_mem`** |

**一句话：** 主案取消 I$ hit 旁路后，`icache→ftq/blk_target` 垄断族从 70 top100 消失；IFU-only 后榜首落到 **TAGE→`p1_diff`→FTQ `blk_pc` WE**，同轮以 **P0 `p1_diff` CE 解耦**（非裸寄存 `p1_target`、非 settle 扩窗）切开，最终 70/75 上前端 FTQ 捕获锥退出绝对 WNS。功能全绿，Linux IPC 降幅 **0.07%** 过门。70 WNS 从 −0.967 抬到 **−0.498**（估 Fmax ≈ **67.6MHz**），75 从 −2.111 抬到 **−1.108**，但 **70 仍未站稳**；55 因 D$ 阵列路径（约 18.4ns）两次确定性 FAIL。**下一刀继续频率线（D$/MSHR、SB→D$/rs_mem、FTQ→L1 TLB），不开 IPC 变体①。**

---

## 1. 本轮改动

### 1.1 方案选型（主案）

命中与未命中统一：预译码只吃 **已锁存** `if_rline`；`data_ok` 拍只写寄存器，**下一拍**再 `if_line_ready` / 预译码 / `predec_redirect`→FTQ。

路径级依据（V4.3@70MHz #0）：

```
icache req_paddr → tag/way_hit → I$ rdata
  → IFU if_rline_eff（命中旁路 ic_rline_i）
  → cut_inst → imm 目标加法 → predec_target
  → FTQ blk_target_reg
```

### 1.2 留树改动（主案）

| 项 | 留树 | 说明 |
|----|------|------|
| `frontend/ifu.v`：删 `if_rline_eff` 旁路 | ✅ | 切割/预译码恒用 `if_rline` |
| `frontend/ifu.v`：`if_line_ready` 去 `if_ic_hit` | ✅ | 仅 `!if_wait_data && \|if_rline`（或异常） |

代价：每块 I$ 返回后 IF 级多 1 拍再推 IB/预译码重定向。BPU `pc` 已跟 `predec_redirect`，晚一拍无需另改。

### 1.3 同轮兜底（已启用）

IFU-only@70（`reports_impl70_ifuonly/`，profile `v44-grip3-ifu-hit-reg-70`）：

| 项 | 值 |
|----|-----|
| WNS | **−0.746** |
| 榜首 | **TAGE `t2_meta` → FTQ `blk_pc` LUTRAM WE**（经 `p1_diff`→`p0_valid`） |
| `icache→ftq/blk_target` | 已退出 |

计划草稿写的是「BPU 侧 `p1_*` 再打一拍 + settle 扩至 2 拍」。实测瓶颈是 **`p0_valid` 组合吃 `!p1_diff` 驱动 `blk_pc` WE**，不是裸 `p1_target` 数据口。采用更轻、与现有 1 拍 settle 对齐的做法：

| 项 | 留树 | 说明 |
|----|------|------|
| `frontend/bpu.v`：`p0_valid_o` 不再组合看 `p1_diff` | ✅ | 切断 TAGE→`blk_pc` WE |
| `frontend/bpu.v`：`p0_wrote_r` / 锁存 P0 字段用 `p0_valid_o && !p1_diff` | ✅ | 与 FTQ「真正提交的 P0」对齐 |
| `frontend/ftq.v`：同拍 `p0_valid && p1_valid` 不推进 `bpu_ptr`、不置 `p0_wrote_r` | ✅ | 丢弃错误路径 P0；`blk_pc` WE 仍仅 `p0_valid` |
| PC | 不变 | 仍用组合 `p1_diff` 当拍纠正 |

**禁止项遵守：** 未裸寄存 FTQ `p1_target_i`；未单独扩 settle 而不改 BPU。

未改：FTQ 三写口结构；`dcache`/SB/LSU；变体①/②+③；方案 1.0.5 ENARDEN。

---

## 2. 功能结果

| 负载 | 判据 | 证据 |
|------|------|------|
| lab19 + 7 种子 | `Reached test end PC`；`All 7 seeds passed` | `logs_v44/v44_lab19_p1.log`；`v44_7seeds_p1.log` |
| coremark / dhrystone | digftest + Correct / end PC | `v44_coremark_p1.log`；`v44_dhrystone_p1.log` |
| Linux 软门 | UART `/ #` | `v44_linux_soft_p1.log` |
| Linux digftest | 无 mismatch + `/ #`（`TRACE_COMP`） | `v44_linux_dft_p1.log` |

（`*_p1.*` = 主案 IFU + P1 CE 兜底后的最终 RTL；早期 IFU-only 仿真日志为无 `_p1` 后缀。）

---

## 3. 性能对比

### 3.1 IPC（相对 V4.3 Linux 软门）

| 指标 | V4.3 | V4.4 | Δ |
|------|------|------|---|
| 软门 Commit IPC | 0.703890 | **0.703400** | **−0.07%** |
| digftest Commit IPC | 0.703890 | **0.703400** | −0.07% |
| lab19 IPC（seed0） | 0.213936 | **0.213014** | −0.43% |

验收：相对 V4.3 软门降幅 **&lt;0.5%** ✅。命中路径 +1 IF 拍在 Linux 上几乎被吸收。

### 3.2 频率 / 时序

隔离工程：`D:/chiplab_v41_sta/`。脚本：`logs_v44/launch_{55,70,75}mhz_v44.tcl`、`launch_70mhz_v44b.tcl`（最终 RTL）、`launch_55mhz_v44_retry.tcl`。

| 频点 | gate | WNS | 关键观察 |
|------|------|-----|----------|
| **55MHz** | **FAIL** | **−0.327** | 100/100 **D$ `req_paddr→valid/dirty_arr`**；retry 与首跑路径/Slack 完全一致（非偶然拥塞抖动） |
| **70MHz IFU-only** | **FAIL** | **−0.746** | `icache→ftq` 已清；64/100 **TAGE→`blk_pc` WE** |
| **70MHz 最终** | **FAIL** | **−0.498** | 前端 FTQ 捕获退出绝对榜首；#0 **`dcache→mshr_line CE`**；另有 SB→D$、FTQ→L1 TLB |
| **75MHz 最终** | **FAIL** | **−1.108** | **`blk_target` / tage→ftq = 0**；53/100 **SB→`rs_mem`**；38/100 FTQ→TLB |

相对 V4.3@70（WNS −0.967，96/100 `icache→ftq`）：本轮目标族切除成功，WNS 改善约 **0.47ns**。估 Fmax（70 括点：周期 14.286ns、WNS −0.498 → 需求 ≈14.784ns）≈ **67.6MHz**（V4.3 估 65–66）。

55 FAIL 与 70 估 Fmax 表面上矛盾：55 临界路径数据延迟 **18.38ns**（布线主导），70 临界路径约 **14.40ns**——同 RTL 不同频点 P&R 结果，**不能**用 70 线性外推否认 55 实测。V4.3@55 曾为 +0.153，本轮 D$ 阵列锥成为 55 垄断族，需单独按 D$ 刀处理或换布局策略，不归因于 IFU 旁路本身（该锥不在 55 top100）。

### 3.3 下一刀判定

| 候选 | 证据 | 判定 |
|------|------|------|
| **D$ `req_paddr`→MSHR/valid/dirty** | 70 #0；55 全垄断 | **下一刀优先** |
| **SB→D$ / SB→`rs_mem`** | 70 次席；75 垄断 | 同轮或紧随 D$ |
| **FTQ→L1 TLB / I$ `req_paddr`** | 70/75 大量 | 前端写出侧，非 `blk_target` 捕获 |
| hit→rs_mem / RF/select | 非当前绝对榜首 | 暂缓 |
| IPC 变体①（双 outstanding） | **70 未站稳** | **仍不开** |

---

## 4. 解读

- **优化：** 取消 I$ hit 组合旁路，把预译码/`blk_target` 推到寄存行下一拍；再用 P0/P1 同拍丢弃切断 TAGE→`blk_pc` WE，避免计划中的 settle 扩窗风险。  
- **等价性：** 非逐拍；I$ 返回后 IF 多 1 拍；同拍 P1 纠正时错误路径 P0 不进 FTQ 指针。digftest 全过，IPC 过门。  
- **提频：** 目标族与 75 的 TAGE→`blk_target` 垄断均切除，绝对 WNS 明显抬升，但 **未打开 70MHz**；新瓶颈在 **D$/SB/TLB**。  
- **教训：** IFU 切开后立刻冒出的是 `p0_valid` 使能锥而非 `p1_target` 数据锥——STA 驱动要读 **Destination 的 CE/WE**，不要按名字想当然打数据口。55 与 70 临界路径可完全不同，合规频点也要单独 STA。

---

## 5. 怎么测（复现）

```bash
export CHIPLAB_HOME=/home/dogandlamb/chiplab
export PATH=$CHIPLAB_HOME/toolchains/loongson-gnu-toolchain-8.3-x86_64-loongarch32r-linux-gnusf-v2.0/bin:$CHIPLAB_HOME/toolchains/nemu:$PATH
cd $CHIPLAB_HOME/sims/verilator/run_prog
sed -i 's/^BUS_DELAY_RANDOM_SEED=.*/BUS_DELAY_RANDOM_SEED=5570815/' Makefile_run

make verilator && make testbench

# lab19 / 7 种子
./configure.sh --run func/func_lab19
make simulation_run_prog
./run_7_seeds.sh

# coremark / dhrystone：换 case 必须 soft_compile + testbench
./configure.sh --run coremark --disable-simu-trace --output-uart-info
mkdir -p log/coremark_log && echo '0 00000000 1 00 00000000' > log/coremark_log/golden_trace.txt
make soft_compile && make testbench && make simulation_run_prog

# Linux 软门
./configure.sh --run linux --disable-simu-trace --disable-trace-comp --disable-read-miss --output-uart-info
make testbench && make simulation_run_prog

# Linux digftest：去掉 --disable-trace-comp；placeholder golden_trace
# STA：D:/chiplab_v41_sta/scripts/launch_{55,70,75}mhz_v44*.tcl
#      最终 70 用 launch_70mhz_v44b.tcl（含 P1 CE 兜底）
```

---

## 6. 文件清单

| 路径 | 用途 |
|------|------|
| `frontend/ifu.v` | 取消 hit 旁路；`if_line_ready` 去 `if_ic_hit` |
| `frontend/bpu.v` | `p0_valid` 与 `p1_diff` 解耦；`p0_wrote` 对齐 |
| `frontend/ftq.v` | 同拍 P1 丢弃 P0；`blk_pc` WE 不含 `p1_diff` |
| `logs_v44/` | 仿真与 STA 归档 |
| `logs_v44/reports_impl{55,70,75}/` | gate + timing_top100 等 |
| `logs_v44/reports_impl70_ifuonly/` | 兜底前中间态 |
| `logs_v44/reports_impl55_retry/` | 55 重跑（与首跑一致） |
| `D:/chiplab_v41_sta/scripts/launch_*mhz_v44*.tcl` | 隔离 STA 启动脚本 |
