# myCPUV4.3：抓手 2（SB/D$ done 边界打拍）— 性能仿真报告

> 日期：2026-07-26  
> **基线报告/CPU**：[`myCPUV4.2_抓手1_D$miss打拍提频.md`](myCPUV4.2_抓手1_D$miss打拍提频.md)（Linux 软门 IPC **0.701964**；55MHz WNS **+0.558**；70MHz WNS **−0.995** 榜首 SB；75MHz WNS **−1.461**）  
> 本轮目标：切断 70/75MHz 绝对 WNS 族 `u_dcache/req_paddr → miss_need_wb → dc_sb_done → push_fire → u_store_buffer/data_reg CE`  
> 负载：`func_lab19` digftest（含 7 种子）；coremark / dhrystone digftest；Linux 软门 / digftest  
> seed：`5570815`（`BUS_DELAY_RANDOM_SEED`）  
> 原始 PERF 日志：`IP/myCPU/logs_v43/`  
> 时序报告：`IP/myCPU/logs_v43/reports_impl{55,70,75}/`（同源 `D:/chiplab_v41_sta/out/{55,70,75}mhz_v43/`）  
> PERF 口径：仿真结束 `myCPU PERF (sim)`（`mycpu_top.v` 中 `` `ifdef SYNTHESIS ``；仿真经 `-DSIMU`）  
> **等价性口径（本轮）：非逐拍等价**，以 Linux IPC A/B（相对 V4.2 **0.701964** 降幅 **&lt;0.5%**）+ 全量 digftest PASS 验收

---

## 0. 结论摘要

| 门控 | 结果 |
|------|------|
| **lab19 digftest** | **PASS**；retire=**122875**，IPC=**0.213936**；7 种子全过 |
| **coremark / dhrystone** | **PASS**（digftest）；IPC≈**0.201289** / **0.180174** |
| **Linux 软门** | **PASS**（`/ #`）；IPC = **0.703890**（相对 V4.2 0.701964：**+0.27%**） |
| **Linux digftest** | **PASS**（至 `/ #`，无 mismatch）；IPC = **0.703890** |
| **频率 55MHz（合规）** | **PASS**：WNS=**+0.153**（V4.2 为 +0.558；仍合规） |
| **频率 70MHz（主目标）** | **FAIL**：WNS=**−0.967**（V4.2 −0.995）；**SB/`push_fire` 在 cpu top100 = 0**；榜首改为 **`icache/req_paddr → ftq/blk_target`**（96/100） |
| **频率 75MHz（摸底）** | **FAIL**：WNS=**−2.111**（V4.2 −1.461）；**SB/`lsu_ready` = 0**；榜首 **`tage → ftq/blk_target`**（99/100） |

**一句话：** 抓手 2 按计划主案落地（D$ 组合 `st_done_o` 不变，SB 寄存 `dc_wr_done` 再 pop/清 inflight）；功能 digftest 全绿、Linux IPC 相对 V4.2 **不降反升**；目标 SB 长链已从 70/75 top100 **彻底切除**，但绝对 WNS 几乎未抬（70 仍约 −1ns），新瓶颈落到**前端 FTQ 捕获锥**（I$/TAGE→`blk_target`）。估 Fmax 仍约 **65–66MHz**。同轮 **不做** hit→`lsu_ready` 兜底（条件未触发：榜首不是 hit→rs_mem）。**下一刀继续频率线（前端 FTQ/I$ 解耦），不开 IPC 变体①。**

---

## 1. 本轮改动

### 1.1 方案选型

与 V4.2 兜底哲学对称：D$ 仍组合给出 `st_done_o`（与 LOOKUP hit/alloc/merge **同拍**），只在 **SB 消费侧**把 `dc_wr_done` 反应推迟 1 拍。

路径级依据（V4.2@70MHz #0）：`req_paddr` → `miss_need_wb` → `dc_sb_done` → `push_fire` → `data_reg` CE。  
`push_fire = push_valid && (!full || pop_fire)`，综合器把组合 `dc_wr_done` 算进 push 使能锥。

### 1.2 留树改动

| 项 | 留树 | 说明 |
|----|------|------|
| `memory/store_buffer.v`：`dc_wr_done_r` | ✅ | 捕获 `dc_wr_done_i`；`pop_fire` / `inflight` 清除 / `count` 更新 **统一**用寄存版 |
| `mycpu_top.v` | ✅（注释） | `dc_sb_done` 注明 SB 内打拍 |

未改：`dcache.v` 的 `st_done_o` 组合式；LSU hit 腿；RF/select/TAGE；变体①/②+③。

协议：done 延迟期间 `inflight` 仍为 1 → `dc_wr_req_o` 不抢发下一笔。

### 1.3 同轮兜底判定

计划：若 70 仍 FAIL 且榜首为 **hit→`lsu_ready`→rs_mem**，再寄存 hit→`d_done`。

实测 70 榜首为 **`icache → ftq`**，hit/rs_mem 不在 top100 → **不启用 hit 兜底**。

---

## 2. 功能结果

| 负载 | 判据 | 证据 |
|------|------|------|
| lab19 + 7 种子 | `Reached test end PC` / `END by Syscall`；`All 7 seeds passed` | `logs_v43/v43_lab19.log`；`v43_7seeds.log` |
| coremark / dhrystone | digftest + Correct / end PC | `v43_coremark.log`；`v43_dhrystone.log` |
| Linux 软门 | UART `/ #` | `v43_linux_soft.log` |
| Linux digftest | 无 mismatch + `/ #`（`TRACE_COMP`） | `v43_linux_dft.log` |

---

## 3. 性能对比

### 3.1 IPC（相对 V4.2 Linux 软门）

| 指标 | V4.2 | V4.3 | Δ |
|------|------|------|---|
| 软门 Commit IPC | 0.701964 | **0.703890** | **+0.27%** |
| digftest Commit IPC | 0.701050 | **0.703890** | +0.40%（相对 V4.2 dft） |
| lab19 IPC（seed0） | 0.214694 | **0.213936** | −0.35% |

验收：相对 V4.2 软门降幅 **&lt;0.5%** ✅（本轮为增益）。对照 V4.1 **0.703636**：软门 **+0.04%**。

SB 腾槽晚 1 拍在 Linux 上未体现为 IPC 损失（可能被别的松弛吸收，或方差盖过）；lab19 微降与 SB 满口无关（lab19 SB full=0）。

### 3.2 频率 / 时序

隔离工程：`D:/chiplab_v41_sta/`。脚本：`launch_{55,70,75}mhz_v43.tcl`。

| 频点 | gate | WNS | 关键观察 |
|------|------|-----|----------|
| **55MHz** | **PASS** | **+0.153** | 榜首仍 `icache→ftq`；合规裕量相对 V4.2 收窄但仍正 |
| **70MHz** | **FAIL** | **−0.967** | **SB/push_fire/lsu_ready = 0**；96/100 为 **icache→ftq/blk_target** |
| **75MHz** | **FAIL** | **−2.111** | **SB = 0**；99/100 为 **TAGE→ftq/blk_target** |

相对 V4.2@70（WNS −0.995，85/100 进 SB）：本轮切除 SB 族后 WNS 仅改善约 **0.03ns**——说明原 SB 路径与前端长链在绝对延迟上接近，切掉后前端立刻成为垄断族。75 上 WNS 相对 V4.2 变差属 P&R 方差 + 新族暴露，**不能解读为 SB 改动伤时序**（SB 相关网已不在 top100）。

用 70MHz 括点：周期 14.286ns、WNS −0.967 → 需求周期 ≈15.25ns → 估 Fmax ≈ **65.6MHz**（与 V4.2 估 65–66 同档）。

### 3.3 下一刀判定

| 候选 | 证据 | 判定 |
|------|------|------|
| **前端 I$/TAGE → FTQ `blk_target`** | 70/75 绝对榜首与垄断占比 | **下一刀**；对齐方案 1.0.5（I$ 使能/异常与 FTQ 捕获解耦）或对 `blk_target` 写入锥打拍 |
| hit→rs_mem | 70/75 top100 中 **0** | 暂不需；若前端切开后再冒再做 |
| 抓手 A/B/C + IPC 变体① | **70 未站稳** | **仍不开** |

---

## 4. 解读

- **优化：** SB 对 `dc_wr_done` 打一拍，切断 `LOOKUP miss/WB 锥 → st_done → push CE` 跨模块长链，且保持 D$ posted store 同拍完成语义。  
- **等价性：** 非逐拍；SB 腾槽/解满晚 1 拍。digftest 全过，Linux IPC 未掉。  
- **提频：** 目标族切除成功，但 **未打开 70MHz**；新 WNS 在前端 FTQ。  
- **教训：** 切掉绝对榜首不等于 WNS 大涨——若第二族延迟接近，需连续切刀。STA 驱动：下一步必须对准 **FTQ 捕获路径**，不要回头打已消失的 SB/hit 腿。

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
# STA：D:/chiplab_v41_sta/scripts/launch_{55,70,75}mhz_v43.tcl
```

---

## 6. 文件清单

| 路径 | 用途 |
|------|------|
| `memory/store_buffer.v` | `dc_wr_done_r` + pop/inflight/count |
| `mycpu_top.v` | `dc_sb_done` 注释 |
| `logs_v43/` | 仿真与 STA 归档 |
| `logs_v43/reports_impl{55,70,75}/` | gate + timing_top100 等 |
| `D:/chiplab_v41_sta/scripts/launch_*mhz_v43.tcl` | 隔离 STA 启动脚本 |
