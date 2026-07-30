# myCPUV4.2：抓手 1（D$ miss→LSU ready 长链切除）— 性能仿真报告

> 日期：2026-07-26  
> **基线报告/CPU**：[`myCPUV4.1_抓手0_CAM异常门控提频.md`](myCPUV4.1_抓手0_CAM异常门控提频.md)（Linux 软门 IPC **0.703636**；55MHz WNS **+0.331**；75MHz WNS **−2.577**）  
> 本轮目标：切断 75MHz 垄断瓶颈族 `u_dcache/req_paddr → dc_lsu_miss → lsu_ready → u_rs_mem`  
> 负载：`func_lab19` digftest（含 7 种子）；coremark / dhrystone digftest；Linux 软门 / digftest  
> seed：`5570815`（`BUS_DELAY_RANDOM_SEED`）  
> 原始 PERF 日志：`IP/myCPU/logs_v42/`  
> 时序报告：`IP/myCPU/logs_v42/reports_impl55/`、`reports_impl75/`（同源 `D:/chiplab_v41_sta/out/{55,75,70}mhz_v42/`）  
> PERF 口径：仿真结束 `myCPU PERF (sim)`（`mycpu_top.v` 中 `` `ifdef SYNTHESIS ``；仿真经 `-DSIMU`）  
> **等价性口径（本轮）：非逐拍等价**，以 Linux IPC A/B（相对 0.703636 降幅 **&lt;0.5%**）+ 全量 digftest PASS 验收

---

## 0. 结论摘要

| 门控 | 结果 |
|------|------|
| **lab19 digftest** | **PASS**；基线种子 retire=**122875**，IPC=**0.214694**；7 种子全过 |
| **coremark / dhrystone** | **PASS**（digftest）；IPC≈**0.201283** / **0.180153** |
| **Linux 软门** | **PASS**（`/ #`）；IPC = **0.701964**（相对 0.703636：**−0.24%**） |
| **Linux digftest** | **PASS**（至 `/ #`，无 mismatch）；IPC = **0.701050**（相对 0.703636：**−0.37%**） |
| **频率 55MHz（合规）** | **PASS**：WNS=**+0.558**（V4.1 为 +0.331） |
| **频率 70MHz（括点）** | **FAIL**：WNS=**−0.995**；榜首 **`req_paddr→store_buffer`**；`ld_miss` 仍为 0 |
| **频率 75MHz（摸底）** | **FAIL**：WNS=**−1.461**（V4.1 为 −2.577，改善 **+1.12ns**）；**miss 族退出 top100** |

**一句话：** 抓手 1 按计划兜底落地（D$ 组合 `ld_miss_o` 不变，LSU 寄存 `dc_miss→d_done` 腿）；功能 digftest 全绿、Linux IPC 降幅 &lt;0.5%；75MHz 上原 **miss→rs_mem** 垄断族已切除，WNS 从 −2.577 抬到 −1.461；70MHz 仍小负（−0.995），估 Fmax≈**65–66MHz**，新瓶颈以 **SB / hit→ready** 为主。

---

## 1. 本轮改动

### 1.1 方案选型（相对计划主案）

计划主案是在 `dcache.v` 寄存 `ld_miss_o`（T 拍判定、T+1 拍通知）。实测：

| 变体 | 现象 |
|------|------|
| D$ `ld_miss_r`（仅晚通知） | Linux digftest 早失败（`t0` @ `0xa04dbcf4`） |
| + `accept_ok` 挡住脉冲拍 | 软门可过；digftest 仍同点失败 |
| `S_LD_MISS`（次拍同拍分配+通知） | digftest 推进到 ~119M retire 后于 EENTRY 失败 |
| **LSU 寄存 `dc_miss→d_done`（计划兜底）** | **lab19/7seeds/coremark/dhrystone/Linux soft+dft 全 PASS** |

根因要点：晚通知若与 MSHR 分配不同拍，会破坏「移交/清 `d_req_sent`/占 miss 槽」原子性；即便挡住前端接受，仍有架构级错载。兜底方案保持 D$ 组合脉冲与 MSHR 分配同拍，只把 **腾空 DC 级 / 拉高 `lsu_ready`** 推迟一拍——STA 切点仍落在 `miss→ready→rs_mem`，协议更安全。

### 1.2 留树改动

| 项 | 留树 | 说明 |
|----|------|------|
| `backend/execute/lsu.v`：`dc_miss_done_r` | ✅ | `dc_missed && (d_drop \|\| m_has_free)` 寄存一拍后进入 `d_done` |
| `memory/dcache.v` | ✅（注释） | **仍** `assign ld_miss_o = lk_ld_alloc`（组合）；注释标明提频切点在 LSU |
| `mycpu_top.v` | ✅（注释） | `dc_lsu_miss` 注释同步 |

未改：`rs_mem` / SB / 命中腿（本轮不碰 load-to-use）。

---

## 2. 功能结果

| 负载 | 判据 | 证据 |
|------|------|------|
| lab19 + 7 种子 | `Reached test end PC` / `END by Syscall` | `logs_v42/v42_lab19_lsufb.log`；`v42_7seeds_lsufb.log` |
| coremark / dhrystone | digftest + Correct / end PC | `logs_v42/v42_coremark_retry.log`；`v42_dhrystone_lsufb.log` |
| Linux 软门 | UART `/ #` | `logs_v42/v42_linux_soft_lsufb.log` |
| Linux digftest | 无 mismatch + `/ #` | `logs_v42/v42_linux_dft_lsufb.log` |

注意：换 `RUN_SOFTWARE` / `TRACE_COMP` 后必须重编 `testbench`（`END_PC` 陷阱，见 V4.1 §5）；coremark 曾因误用 Linux TB 出现假 `Both Error`，重配后 PASS。

---

## 3. 性能对比（相对 V4.1，Linux）

| 指标 | V4.1 | V4.2 | Δ |
|------|------|------|---|
| 软门 Commit IPC | 0.703636 | **0.701964** | **−0.24%** |
| digftest Commit IPC | 0.703636 | **0.701050** | **−0.37%** |
| lab19 IPC（seed0） | 0.214694 | **0.214694** | 0（本负载几乎无 D$ miss） |

验收：IPC 降幅 **&lt;0.5%** ✅；非 bit-identical 属预期（miss 后 DC 级多占 1 拍）。

### 3.1 频率 / 时序

隔离工程：`D:/chiplab_v41_sta/`。策略同 V4.1（关增量综合 → 改 PLL → `vivado_full.tcl`）。

| 频点 | gate | WNS | 关键观察 |
|------|------|-----|----------|
| **55MHz** | **PASS** | **+0.558** | 榜首回到前端 `icache/req_paddr → ftq/blk_target`；合规裕量相对 V4.1 再扩大 |
| **70MHz** | **FAIL** | **−0.995** | 榜首 **`u_dcache/req_paddr → u_store_buffer/data` CE**；`ld_miss`=0；经 `lsu_ready` 仅 13/100 |
| **75MHz** | **FAIL** | **−1.461** | **ld_miss/dc_miss 路径在 cpu top100 = 0**；66/100 为 **hit/`ld_data_ok`→`lsu_ready`→`rs_mem`**；25/100 为 **`req_paddr→store_buffer`** |

相对 V4.1@75MHz（WNS −2.577，100/100 经 `dc_lsu_miss`）：本轮切除 miss 腿后 75 仍差约 **1.46ns**。用 70MHz 实测括点：周期 14.286ns、WNS −0.995 → 需求周期 ≈15.28ns → 估 Fmax ≈ **65.4MHz**（未跑 65 实测站稳）。

### 3.2 下一刀判定

| 候选 | 证据 | 判定 |
|------|------|------|
| **hit 腿**（`lk_ld_hit → ld_data_ok → lsu_ready → rs_mem`） | 75MHz cpu top **66/100** | **下一刀优先**；慎整段打拍（加 load-to-use）；优先收锥/复制寄存器/`lsu_ready` 扇出 |
| **store 腿**（`req_paddr → SB data CE`） | 70MHz 榜首；75MHz **25/100** | **与 hit 腿并列下一刀**；可考虑寄存 `st_done` alloc 腿或 SB 捕获切拍 |
| 前端 1.0.5 | 55MHz 榜首已回 FTQ/I$ | 抬合规裕量时做；非 75MHz 当前 WNS 主凶 |
| 抓手 A/B/C + IPC 变体 | 75 未站稳 | **等 ≥70MHz 站稳后再开** |

---

## 4. 解读

- **优化：** 在 LSU 把 miss 响应到 `d_done`/`lsu_ready` 的腿打一拍，切断跨模块 `D$ miss 判定 → RS_MEM CE` 长链，同时保持 D$→LSU miss 通知与 MSHR 分配同拍。  
- **等价性：** 非逐拍；每次 load miss 的 DC 级多留 1 拍。Linux IPC 微降（&lt;0.5%），digftest 全过。  
- **提频：** miss 垄断族已退出 75MHz top；WNS 改善约 1.1ns，但 hit/SB 腿立刻补位，75 仍负。55MHz 合规更宽裕。  
- **教训：** 软门 `/ #` ≠ digftest 正确性；推迟 `ld_miss_o` 而不保持分配/通知原子性会在中后期暴露 RF/异常入口错载。

---

## 5. 怎么测（复现）

```bash
export CHIPLAB_HOME=/home/dogandlamb/chiplab
export PATH=$CHIPLAB_HOME/toolchains/loongson-gnu-toolchain-8.3-x86_64-loongarch32r-linux-gnusf-v2.0/bin:$CHIPLAB_HOME/toolchains/nemu:$PATH
cd $CHIPLAB_HOME/sims/verilator/run_prog
sed -i 's/^BUS_DELAY_RANDOM_SEED=.*/BUS_DELAY_RANDOM_SEED=5570815/' Makefile_run

make verilator && make testbench

# lab19
./configure.sh --run func/func_lab19
make simulation_run_prog
# 7 种子：./run_7_seeds.sh

# coremark（换 case 必须 soft_compile + testbench）
./configure.sh --run coremark --disable-simu-trace --output-uart-info
mkdir -p log/coremark_log && echo '0 00000000 1 00 00000000' > log/coremark_log/golden_trace.txt
make soft_compile && make testbench && make simulation_run_prog

# Linux 软门
./configure.sh --run linux --disable-simu-trace --disable-trace-comp --disable-read-miss --output-uart-info
make testbench && make simulation_run_prog

# Linux digftest：去掉 --disable-trace-comp；placeholder golden_trace
# STA：D:/chiplab_v41_sta/scripts/launch_{55,70,75}mhz_v42.tcl
```

---

## 6. 文件清单

| 路径 | 用途 |
|------|------|
| `backend/execute/lsu.v` | `dc_miss_done_r` |
| `memory/dcache.v` / `mycpu_top.v` | 注释同步 |
| `logs_v42/` | 仿真与 STA 归档 |
| `logs_v42/reports_impl{55,70,75}/` | gate + timing_top100 等 |
