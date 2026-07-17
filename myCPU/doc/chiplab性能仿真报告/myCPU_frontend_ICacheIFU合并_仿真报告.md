# myCPU：I$/IFU 流水线 + 训练 FIFO 加深 — 仿真报告

> 日期：2026-07-17  
> 基线：`myCPU_v1.7_dispatch解锁_仿真与占用报告.md`（seed `5570815`）  
> 本轮改动：cherry-pick `frontend_2.0(1.2+1.3)` 的 I$/IFU 吞吐；`FTB/TAGE_UPDATE_Q` 8/16 → **32/32**  
> 负载：`func_lab19` digftest；Linux → `/ #`

---

## 0. 结论摘要

| 项 | 结果 |
|----|------|
| **func_lab19** | **通过**（`END by Syscall` / digftest） |
| **Linux** | **通过**（`/ #`，干净停仿打 PERF） |
| lab19 IPC | 0.2128 → **0.2153**（+1.2%） |
| Linux IPC | 0.5609 → **0.5862**（**+4.5%**） |
| I$ 吞吐 | lab19 hit 73.7%→**79.9%**；access 明显增多（稳态 1 req/拍生效） |
| 训练 overflow | FTB Linux **87 万→37 万**（−57%）；TAGE 略降 |
| BPU 准确率 | 基本持平（Linux all 92.25%→92.17%） |
| 副作用 | Linux **ROB full 4.0%→10.1%**（前端更猛，窗口更常顶满） |

**一句话：** 功能/OS 都过；I$/IFU 合并显著抬高 Linux IPC；加深训练 FIFO 明显减 FTB 丢训练；下一步瓶颈更偏 **后端窗口/访存**，不是取指气泡。

---

## 1. 本轮 RTL 改动

### 1.1 `icache.v`（来自 frontend_2.0，保留 myCPU PERF）

- `req_window = IDLE || lookup_hit || RESP`
- LOOKUP hit / RESP 同拍可 `req_take` 启动下一查找 → 稳态约 **1 请求/拍**
- 保留 `ic_access_total` / `ic_hit_total` 仿真探针

### 1.2 `ifu.v`（I$ 接口移植 + 保留 `slot_pred_taken`）

并入：

- `ic_slot_free`：允许 data_ok 与 addr_ok 同拍替换
- `ftq_direct_req`：FTQ accept 同拍直发 I$
- `ic_rsp_line` + `if/pre_rsp_match`：按 cacheline 配对响应
- `pre_line_valid`：PRE 被反压时保留已返回行

**刻意保留（不采用对方简化逻辑）：**

- `slot_pred_taken`：仅块末 B/BL 或真实 `cond_br` 标 taken  
- 避免块末 ALU 被脏 FTB 标 taken 导致 Linux makecontext 静默跳指令

### 1.3 `mycpu.h`

| 宏 | 旧 | 新 |
|----|----|----|
| `FTB_UPDATE_Q_DEPTH` | 8 | **32** |
| `TAGE_UPDATE_Q_DEPTH` | 16 | **32** |

未改：`bpu.v` / `ftb.v` / `tage.v` / `ftq.v` 算法本体（仅吃宏深度）。

---

## 2. 功能结果

| 负载 | 判定 | 证据 |
|------|------|------|
| func_lab19 | PASS | digftest 全程一致；`END by Syscall`；retire=122875 |
| Linux | PASS | UART 至 `/ #`；`Detected Linux shell`；retire≈1.245e8 |

注意：lab19 若关 digftest 曾出现末尾 hang 探测器误报；**以 digftest 通过为准**。Linux 须 `make clean_all` 后重配、刷新 `obj/linux_obj`，否则会跑错镜像。

---

## 3. 性能对比（同 seed 5570815）

### 3.1 func_lab19

| 指标 | 基线 (v1.7) | 本轮 | Δ |
|------|-------------|------|---|
| Commit IPC | 0.212762 | **0.215277** | +1.18% |
| cycles | 577522 | 570775 | −1.2% |
| Dispatch 双发率 | 71.73% | **73.30%** | +1.6pt |
| Commit 双发率（有提交拍） | 31.56% | **41.35%** | +9.8pt |
| BPU all / cond | 89.49% / 91.25% | 89.46% / 91.21% | ≈ |
| I$ access / hit | 57177 / 73.65% | **75064 / 79.89%** | access↑、hit↑ |
| FTQ retained avg | 10.098 | 10.158 | ≈ |
| FTB upd overflow | 1201 | **779** | −35% |
| TAGE upd overflow | 319 | **245** | −23% |

lab19 绝对 IPC 仍低（短测 + 异常/CACOP 路径），但 I$ 与双发指标已体现取指加速。

### 3.2 Linux → `/ #`

| 指标 | 基线 (v1.7) | 本轮 | Δ |
|------|-------------|------|---|
| **Commit IPC** | 0.560927 | **0.586225** | **+4.51%** |
| cycles | 222567129 | 212381328 | −4.6% |
| retire | 124843937 | 124503350 | −0.3% |
| Dispatch 双发率 | 56.27% | **76.55%** | **+20.3pt** |
| dual_ALU 拍 | 16.7M | **27.6M** | +65% |
| BPU all / cond | 92.25% / 96.19% | 92.17% / 96.11% | −0.08 / −0.08pt |
| FTB P1 hit | 36.73% | **37.00%** | +0.27pt |
| I$ hit | 96.03% | **96.16%** | +0.13pt |
| I$ access | 63.8M | **69.4M** | +8.8% |
| D$ hit | 96.40% | 96.42% | ≈ |
| ROB full | 3.96% | **10.13%** | +6.2pt |
| SB full | 2.22% | 3.26% | +1.0pt |
| D$ MSHR busy | 18.58% | 19.38% | +0.8pt |
| RS_MEM src stall | 15.71% | 16.82% | +1.1pt |
| FTB upd overflow | 869726 | **369785** | **−57%** |
| TAGE upd overflow | 168510 | 161773 | −4% |
| FTB/TAGE Q max | 8/16 | **32/32** | 仍顶满 |

---

## 4. 占用与瓶颈解读

### 4.1 取指侧：合并目标达成

- I$ access↑、hit 率↑ → 流水线/同拍替换在干活。
- Dispatch 双发从 56%→77%：后端更容易吃满，说明前端少堵。
- FTQ retained 高水位依旧（Linux ≥75% 周期仍很多）——取指更快后，若后端/访存跟不上，FTQ 仍会积压；**不宜缩 FTQ**。

### 4.2 训练 FIFO：加深有效，但仍不够

| | 基线深 | 本轮深 | Linux overflow |
|--|--------|--------|----------------|
| FTB | 8 | 32 | 87万 → **37万** |
| TAGE | 16 | 32 | 17万 → 16万 |

FTB 收益大；TAGE 仍常顶满（训练突发 + 查询优先占读口）。若还要抠准确率，可再试 **TAGE 64**，或给训练口独立读端口（硬件更贵）。

### 4.3 后端压力上移（预期副作用）

前端更快后：

- ROB 更常满（4%→10%）→ rename `rob_stall` 升
- SB / MSHR / RS_MEM 源停滞略升

这不是回归，而是 **瓶颈从「取指气泡」转到「乱序窗口 + 访存」**。下一轮更该动：

1. ROB 有效容量（或减 `ROB_GUARD` 需极谨慎）  
2. D$ MSHR / pend  
3. RS_MEM 深度  

而不是回退 I$/IFU。

### 4.4 BPU 准确率略降

Linux all-branch −0.08pt，可忽略。略降可能来自：更激进取指 → 更多错误路径训练噪声；但 FTB hit 与 IPC 仍升，说明整体净收益为正。

---

## 5. 与「板上资源」建议的关系

| 结构 | 本轮后建议 |
|------|------------|
| I$/IFU 流水线 | **保留**（IPC 明确收益） |
| FTB/TAGE update Q=32 | **保留**；TAGE 可再加深试探 |
| ROB / RS_MEM / SB / FTQ / RAS | **仍不建议缩**（占用更高） |
| TAGE/FTB 表项 | 省 BRAM 仍应优先评估表几何，而非砍 update Q |

---

## 6. 复现

```bash
export CHIPLAB_HOME=~/chiplab
export PATH=$CHIPLAB_HOME/toolchains/loongson-gnu-toolchain-8.3-x86_64-loongarch32r-linux-gnusf-v2.0/bin:$PATH
cd $CHIPLAB_HOME/sims/verilator/run_prog

# lab19（digftest）
make clean_all
./configure.sh --run func/func_lab19
make verilator && make testbench && make soft_compile
make simulation_run_prog

# Linux（关 digftest；必须刷新 rom）
make clean_all
./configure.sh --run linux --disable-simu-trace --disable-trace-comp --disable-read-miss --output-uart-info
rm -rf obj/linux_obj
make soft_compile
make verilator && make testbench
make simulation_run_prog
```

---

## 7. 附录：本轮 PERF 摘录

### lab19

```
Commit IPC:         retire=122875  cycles=570775  IPC=0.215277
Dispatch dual:      dual_rate=73.30%  dual_ALU=51598
BPU all/cond:       89.46% / 91.21%
ICache:             access=75064  hit_rate=79.89%
FTB/TAGE overflow:  779 / 245  (depth 32/32)
```

### Linux

```
Commit IPC:         retire=124503350  cycles=212381328  IPC=0.586225
Dispatch dual:      dual_rate=76.55%  dual_ALU=27613129
BPU all/cond:       92.17% / 96.11%
FTB P1 hit:         37.00%
ICache:             access=69444232  hit_rate=96.16%
ROB full:           10.13% | SB full: 3.26% | MSHR busy: 19.38%
FTB/TAGE overflow:  369785 / 161773  (depth 32/32)
Reached Linux shell prompt (/ #).
```
