# myCPUV5.1：前端/LSU提速+IPC↑0.875957

> 日期：2026-07-31  
> **基线报告/CPU**：[`myCPUV5.0_稳定基线_仿真与实现报告.md`](myCPUV5.0_稳定基线_仿真与实现报告.md)；基线 Linux difftest IPC `0.734196`  
> 本轮改动：WIP 前端/LSU/D$/L2 微架构提速 + 关闭 `COMPETITION_BOOT_RAM_CACHE`（恢复 DMW MAT=0 真 UC）  
> 负载：`func_lab19` difftest；Linux **difftest**（本轮 **primary IPC**，非软门）  
> seed：终检 `5570815`；探针矩阵部分 run 为 Makefile 默认 `1818351`（见 §5）  
> 原始 PERF / 日志：`artifacts/v5.1/`（探针矩阵链到 `artifacts/v5.0/perf_matrix_{128k,32k}/`）  
> PERF 口径：仿真结束 `myCPU PERF (sim)`（`mycpu_top.v` `` `ifdef SYNTHESIS ``；`-DSIMU` → `mycpu.h` 定义 `SYNTHESIS`）  
> 日常探针选型：[`优化-仿真表.md`](../优化-仿真表.md)  
> **IPC 主对比源**：**Linux difftest**（与 V5.0 §4.2 同口径对齐；勿与软门混比）  
> **版本绑定**：chiplab 工作树相对 `02e8922` 的未提交 RTL；U-Boot `64ec2ccf`；Linux `17b50f62e`  
> **FPGA 工具口径**：Vivado 2023.2 / `fpga/nscscc-team` / `xc7a200tfbg676-2`；本轮性能实现 **`T202618123009783-perf`（cpu_clk = 70 MHz）** setup **FAIL**；正式 **65 MHz** 门（≥ +0.200 / TNS=0）**未重测**

---

## 0. 结论摘要

| 门控 | 结果 |
|------|------|
| **lab19 difftest** | PASS（seed `5570815`，`END by Syscall`）；**七 seed 7/7 PASS**（§3.4） |
| **C/性能短测** | A0 / inner_product / 四探针 / memcmp / CoreMark：**PASS**（矩阵见 §3.1；seed 见 §5） |
| **L2 Cache 维护专项** | PASS（`scripts/v5/test_l2_cacop.sh` → `L2 CACOP TEST PASS`） |
| **Linux 软门** | 未跑 |
| **Linux difftest** | PASS 到 `/ #`；IPC **`0.875957`**（相对 V5.0 `+19.31%`） |
| **U-Boot → Linux** | **有条件 PASS** 到 shell（`Processing /etc/profile... Done`）；IPC **`0.743280`**；cmdline 仍为 `earlycon`（与 V5.0 同款放宽，§3.5） |
| **70 MHz 性能实现** | **FAIL**：WNS `−0.141`、TNS `−0.923`、13 setup（`T202618123009783-perf`）；hold 干净 |
| **65 MHz 正式门** | **未重测**（仍按 V5.0 门槛 ≥ +0.200 / TNS=0；V5.0 65 MHz 曾 FAIL WNS `−0.072`） |

**一句话：** 正确性闸（lab19 七 seed + Linux difftest + uboot_linux 到 shell）已绿；Linux difftest IPC 自 V5.0 的 `0.734` 提升到 **`0.876`（+19%）**，主因是**误预测/冲刷下降带动 Commit dual-issue 36%→51%**（见 §4.2），而非再扩 L2；Branch 探针 MPKI 仍高（≈33）。板上 **70 MHz 性能实现仍 FAIL**，但相对 V5.0 `pref_70`（WNS≈`−0.530`）已收紧约 **0.39 ns**（见 §4.1）。

---

## 1. 本轮改动

相对 chiplab `02e8922`（V5.0 文档收尾）工作树 RTL（约 25 个核心文件，`+5501/−3756`）。

| 项 | 留树 | 说明 |
|----|------|------|
| 前端 BPU / TAGE / FTB / uBTB / FTQ | ✅ | 预测与训练路径加固；P0 fallback BTB |
| IFU linebuf / FTQ 直发 / `ic_invalidate` | ✅ | 响应复用；cacop 当拍清 PRE/IF/linebuf |
| I$ `refill_abort` | ✅ | miss 途中同 set cacop 丢弃迟到安装 |
| LSU fast_wb / hold_wb / STQ 前递 | ✅ | 命中旁路与 store→load 时序安全前递 |
| D$ resp_ready / store-merge 寄存 | ✅ | 命中响应握手；同行 merge 判定前移 |
| L2 D-refill streaming | ✅ | MRDATA 直通 D$ beat，少两拍回放 |
| L2 I-miss `im_line_stale` | ✅ | UC/维护写同行走时禁止把写前旧行装回 L2 |
| **关闭 `COMPETITION_BOOT_RAM_CACHE`** | ✅ | **正确性关键**：不再把 `0x1c0xxxxx` 上软件 MAT=0 提升为 cached；修复 lab19 `n73`/`0x700` |
| dual-OS / JTC P0 / 256KB L2 | ❌ | 继承 V5.0 不留树结论 |

**根因备忘（lab19 `0x700`）：** 自修改用例依赖 DMW MAT=0 UC store 写穿内存；MAT 提升使 store 只进 D$、I$ 从 L2 重填旧指令。关掉宏后 lab19/Linux 均 PASS。该改动只影响软件显式 UC 窗口，稳态 MAT=1 路径不变。

---

## 2. 功能结果

| 负载 | 判据 | 证据 |
|------|------|------|
| lab19 seed `5570815` | `END by Syscall`；retire=122875 | `artifacts/v5.1/lab19_5570815.log` |
| lab19 七 seed | **7/7 PASS**（retire 均为 122875） | `artifacts/v5.1/lab19_7seeds/` |
| L2 CACOP | dirty/clean/victim PASS | 本机 `test_l2_cacop.sh`；矩阵旁路 `perf_matrix_128k/run_l2_cacop.log` |
| memcmp | `END by Syscall`；无 mismatch 报告 | `artifacts/v5.0/perf_matrix_128k/run_memcmp.log` |
| inner_product / A0 / 四探针 | `END by Syscall` + PERF | `run_batch1.log` + `*.perf.txt` |
| CoreMark | `END by Syscall` + PERF | `run_coremark.log`（UART 分数日志被 null 污染，以 PERF/结束为准） |
| Linux difftest | 无 mismatch + `/ #` | `artifacts/v5.1/linux_difftest_5570815.log` |
| U-Boot → Linux | `bootelf` 注入 + `ChipLab: using bootargs…` + `Processing /etc/profile... Done` + shell 停仿 | `artifacts/v5.1/uboot_linux_5570815.log` |

软件绑定（与 V5.0 同树）：

| 仓库 | commit |
|------|--------|
| chiplab（文档 HEAD） | `02e8922`；**被测 RTL = 未提交 WIP** |
| la32r-U-Boot | `64ec2ccf` |
| la32r-Linux | `17b50f62e4d4a274ed19a620ba30ee30195d2332` |

---

## 3. 性能对比（相对 V5.0；**Linux difftest primary**）

同口径：V5.0 §4.2 Linux difftest ↔ 本轮 Linux difftest；seed 均为 `5570815`，停仿 `/ #`。

| 指标 | V5.0 | V5.1 | Δ |
|------|------|------|---|
| Commit IPC | 0.734196 | **0.875957** | **+19.31%** |
| retire / cycles | 122,370,475 / 166,672,802 | 121,546,164 / 138,758,121 | 指令 −0.67%；周期 −16.7% |
| Commit dual-issue | 36.26% | **50.96%** | +14.7 pp |
| Dispatch dual | （发版未单列本行） | 66.08% | — |
| BPU all accuracy | 95.72% | **98.62%** | +2.9 pp |
| Branch MPKI all | 8.867 | **2.871** | −67.6% |
| I$ hit | 95.90% | 96.42% | +0.52 pp |
| D$ hit | 83.88% | 83.45% | −0.43 pp |
| L2 hit | 56.02% | 52.27% | −3.75 pp |
| Flush / predec_redirect | 1.00% / 1.52% | 0.67% / 0.35% | 下降 |
| FTB update overflow | （发版有） | 44 | 低 |
| D$ MSHR busy / occ max | — | 25.46% / max=2 | 仍顶满 cap |
| IB empty / push_stall | 25.39% / 23.82% | 22.21% / 22.75% | 略降 |
| FTQ pending | — | 87.36% | 仍高 |

解读：IPC 大涨主要来自 **误预测/冲刷下降与双提交上升**；I$/D$/L2 hit 变化不大甚至 L2 略降，说明不是「又扩了一级 cache」。

### 3.1 固定性能探针对比

> 探针矩阵取自 `artifacts/v5.0/perf_matrix_128k/`（WIP @ 128KB L2）。  
> **注意：** 这些 run 的 `Makefile_run` seed 为 **`1818351`**；V5.0 §4.1 探针为 **`5570815`**。下表 Δ 为量级对照，严格同 seed 复测前不作发版签字。

| 探针 | V5.0 IPC (`5570815`) | V5.1 WIP IPC (`1818351`) | Δ（参考） | 关键伴随指标 | 结论 |
|------|----------------------|--------------------------|-----------|--------------|------|
| `perf_front_stream` | 0.551265 | 0.565623 | +2.6% | I$ 99.97%；IB push_stall 70% | 供指结构仍紧 |
| `perf_branch_mix` | 0.413587 | 0.499663 | +20.8% | Branch MPKI **33.1**；BPU all 88.8% | 间接/混合分支仍痛 |
| `perf_dcache_miss` | 0.746243 | 0.869137 | +16.5% | D$ hit 68.1%；L2 hit **94.2%**；ld miss lat avg **2.03** | L2 吃满本探针 |
| `perf_mshr_burst` | 0.432264 | 0.476248 | +10.2% | MSHR max=2；L2 hit 80.8% | 仍不足以为 dual-OS 开灯 |
| `fireye/A0`（A 面） | 0.957238 | **1.324774** | +38.4% | dual 89.1%；I$ 99.9% | 高上限吞吐大涨 |
| `inner_product` | 0.858718 | **1.101127** | +28.2% | D$ 97.1%；L2 89.8% | 同向 |
| CoreMark | 0.202427 | 0.217405 | +7.4% | dual 50.5%（V5.0 曾 ~5%） | 后端配对改善 |
| `memcmp` | 0.328507 | 0.354228 | +7.8% | D$ 90.9%；L2 66.9% | 可见性绿 |

### 3.2 L2 容量 A/B（同 WIP，临时 32KB vs 128KB）

数据：`artifacts/v5.1/perf_matrix_32k/` ↔ `perf_matrix_128k/`（测完已恢复 128KB）。

| 负载 | L2 | IPC | L2 hit | 说明 |
|------|----|-----|--------|------|
| `perf_dcache_miss` | 128KB | **0.869** | **94.2%** | |
| `perf_dcache_miss` | 32KB | 0.649 | 22.7% | IPC **+33.9%** @128KB |
| `perf_mshr_burst` | 128KB | **0.476** | **80.8%** | |
| `perf_mshr_burst` | 32KB | 0.385 | 28.5% | IPC **+23.7%** @128KB |

Linux 上 L2 hit 仍仅 ~52% → 探针「吃满」≠ 全系统吃满。

### 3.3 lab19 短测（正确性闸；IPC 仅参考）

| | V5.0 | V5.1（修 UC 后） |
|--|------|------------------|
| seed | 5570815 | 5570815 |
| 结果 | PASS | PASS |
| IPC | 0.214689 | 0.218994 |
| cycles / retire | 572339 / 122875 | 561088 / 122875 |

### 3.4 lab19 七 seed（本轮闭环）

证据：`artifacts/v5.1/lab19_7seeds/`（`summary.tsv` + 各 seed 日志 + `SHA256SUMS`）。  
口径：difftest on；`END by Syscall`；retire 恒为 122875。

| 名称 | seed | 周期 | 指令 | IPC | 结果 |
|------|------|------|------|-----|------|
| baseline | 5570815 | 561088 | 122875 | 0.218994 | PASS |
| short1 | 5592575 | 561972 | 122875 | 0.218650 | PASS |
| short2 | 2796287 | 561973 | 122875 | 0.218649 | PASS |
| normal1 | 5570561 | 561520 | 122875 | 0.218826 | PASS |
| normal2 | 2796769 | 561948 | 122875 | 0.218659 | PASS |
| normal3 | 7410228 | 561701 | 122875 | 0.218755 | PASS |
| normal4 | 1818351 | 561572 | 122875 | 0.218805 | PASS |

相对 V5.0 同七 seed：指令数不变，周期约 **−1.9%～−2.0%**，IPC 约 **+0.004**；总线随机延迟扫描下行为稳定。

### 3.5 U-Boot → Linux（本轮复验）

证据：`artifacts/v5.1/uboot_linux_5570815.log`（+ `.perf.txt` / `.summary.txt` / `.uart.real`）。  
配置：`--disable-trace-comp --disable-simu-trace --output-uart-info`；seed `5570815`；**须** `make testbench` 使 `STOP_ON_UBOOT_PROMPT=n` 与 `UART_CMD_SCRIPT` 进仿真器。

| 门控 | 结果 |
|------|------|
| UART 注入 `bootelf 0xa3000000` | PASS（`UARTSIM: saw u-boot@, inject…`） |
| `ChipLab: using bootargs: console=ttyS0,115200 rdinit=/init` | PASS |
| Linux 启动到 `Processing /etc/profile... Done` | PASS |
| 停仿 | `Reached shell prompt (/ #, u-boot@, or $ )` |
| Kernel command line | 仍打印 `earlycon`（与 V5.0 相同；U-Boot 侧 bootargs 已打出，仿真裁决放宽） |

| 指标 | V5.0 `final_uboot_linux` | V5.1 本轮 | 说明 |
|------|--------------------------|-----------|------|
| Commit IPC | 0.801302 | **0.743280** | 含 u-boot + `bootelf` 拷贝 + Linux；路径/停仿指令数不同，**勿与 Linux difftest primary 混比** |
| retire / cycles | 278,493,387 / 347,551,226 | 152,039,426 / 204,551,908 | 本轮更早到 shell |
| Commit dual-issue | （发版未单列） | **51.85%** | 与 Linux difftest dual≈51% 同向 |
| BPU all accuracy | — | 98.83% | |
| Branch MPKI all | — | 2.385 | |
| I$ / D$ / L2 hit | — | 97.33% / 84.10% / 45.97% | |

SHA-256：`077ab6fedb4b5e2ee689622c478ac60c44a618068bb74ecfbc95e05d5ee485b5`（整日志）。

---

## 4. 解读

- **优化：** 前端预测/供指（BPU/TAGE/FTQ/IFU）+ LSU/D$ 命中与序路径 + L2 填数流式；并用去掉 MAT 提升修复自修改 UC 语义。  
- **提升：** Linux difftest IPC **+19%**；A0/inner_product/E 面探针大幅上升；Branch MPKI（Linux）自 8.9 → 2.9；**Commit dual-issue 36.26%→50.96%（+14.7 pp）**；lab19 七 seed 全绿；uboot_linux 到 shell。  
- **不足：**  
  1. `perf_branch_mix` MPKI≈33 仍是分支专项短板；  
  2. IB `push_stall` / FTQ pending 在多负载上仍高；  
  3. MSHR cap=2 顶满，dual-OS 仍因历史 Linux 风险不开；  
  4. 软门未跑；**70 MHz 性能实现 FAIL**；正式 **65 MHz** 门未重测；uboot_linux cmdline 仍 `earlycon`。  
- **下一步（可验证）：**  
  1. 探针矩阵用 seed `5570815` 重跑，签字 Δ；  
  2. 针对 C 面（间接跳转/JTC）做小刀，以 `perf_branch_mix`+lab19 为闸；  
  3. 削 D$ hit/MSHR → RS_MEM/LSU 地址链（§4.1 最差路径），再跑 **65 MHz 两轮干净实现**（`check_timing_65mhz.sh`）。

### 4.1 FPGA 性能实现时序（`T202618123009783-perf`）

证据目录：仓库根 `T202618123009783-perf/`（`fpga/nscscc-team` 性能上板实现后导出）。工具：Vivado **2023.2**；设计 `soc_top`；器件 `7a200t-fbg676` / `-2`；状态 **Routed**（报告日 2026-07-31）。

| 项 | 本轮实测 |
|----|----------|
| build / 目标频 | `build_kind=perf`；请求 **70.0 MHz**；`cpu_clk` period **14.286 ns = 70.000 MHz**（sys 100 / DDR 200） |
| Design Timing | **constraints not met**：WNS **`−0.141 ns`**，TNS **`−0.923 ns`**，setup 失败端点 **13** / 129682 |
| Hold / Pulse | Design WHS **`+0.005`**、THS `0`、hold 失败 **0**；WPWS/TPWS 干净 |
| `cpu_clk` Intra | WNS `−0.141` / TNS `−0.923` / setup 失败 **13**；hold WHS `+0.060`、失败 **0** |
| check_timing | `unconstrained_internal_endpoints (0)`；`no_clock/loops/multiple_clock (0)`（I/O delay 缺失属板级既有告警） |
| 正式 65 MHz 门 | **未跑**；门槛仍为全设计与 `cpu_clk`：WNS ≥ **+0.200 ns**、TNS=**0**、hold 干净（`scripts/v5/check_timing_65mhz.sh`） |
| FPGA Linux / 网络 | 未测 |

**最差 setup 路径（`cpu_clk` → `cpu_clk`）：**

| | |
|--|--|
| Slack | **`−0.141 ns`（VIOLATED）** |
| Source → Dest | `u_dcache/req_paddr_reg[5]_rep__3` → `u_lsu/a_vaddr_reg[29]` |
| Datapath | **14.325 ns**（logic 3.813 ns / **26.6%**；route **10.512 ns / 73.4%**） |
| Logic Levels | **24**（CARRY4×8 + LUT/MUXF） |
| 路径语义 | D$ `req_paddr` → way/MSHR 命中 mux → `h_data`/`uc_rdata` → LSU `q_wdata`/`mem_fast_wb_data` → **RS_MEM `q_vaddr` CARRY 链** → LSU `a_vaddr` |

结论要点：

1. **70 MHz 未闭合**：13 个 setup 端点、WNS `−0.141`；瓶颈是 **D$ 命中回传 + RS_MEM 地址加法** 的跨模块长路径，且 **布线占 ~73%**（更像 floorplan/扇出，而非单点组合爆炸）。  
2. **相对 V5.0 同频改善**：V5.0 `reports_impl_pref_70MHZ` Path#1 约 **WNS `−0.530`**（`dcache.req_paddr`→`dcache.dirty_arr`）；本轮同为 70 MHz 已到 **`−0.141`（约 +0.39 ns）**，临界路径从 D$ 内部 dirty 更新迁到 **D$→LSU/RS_MEM 地址链**。  
3. **勿外推为 65 MHz PASS**：若仅按周期差粗算（14.286→15.385 ns）名义余量可观，但布局/布线会随约束变；正式门仍须 **65 MHz 两轮干净实现**。V5.0 65 MHz 曾为 WNS `−0.072` / TNS `−0.809` / 23 setup。  
4. Hold 侧本轮干净，不挡性能上板；当前阻塞是 **cpu_clk setup**。

| 产物 | SHA-256 |
|------|---------|
| `perf_timing_summary.rpt` | `eb0c376ec10f85af13300cb347904630ea1e593ddff6c0479a1c3a64a59878ac` |
| `perf_clock_timing_validation.txt` | `a26f4475f094fb0968d5af2b726721462599d977c65cf57870d275ac2ebb93d1` |
| `perf_clock_generated.txt` | `f0a4637b0551be6fc5c8725c63f7fdd2cca9fb381c1914d935d82be81bfdd25e` |

### 4.2 为什么双发率提高了？（Commit dual-issue 36%→51%）

口径：PERF 的 `Commit dual-issue` = **在至少退休 1 条的周期里，退休 2 条的比例**（不是对全部时钟）。Linux difftest 同 seed 下从 **36.26% → 50.96%（+14.7 pp）**，是 IPC +19% 的直接结构原因之一（双提交上升 ≈ 同样指令更少「半满」退休泡）。

**主因（按贡献排序）：**

1. **分支更准 → 冲刷更少 → 双退休窗口更长**  
   BPU all accuracy `95.72%→98.62%`，Branch MPKI `8.87→2.87`（−68%），Flush `1.00%→0.67%`，predec_redirect `1.52%→0.35%`。  
   误预测会清空 ROB/前端，恢复后常见「单条追赶」；少冲刷则更多周期里 ROB 头能同时退休一对。

2. **前端更能持续供指，双槽不那么常饿死**  
   IB empty `25.39%→22.21%`，FTQ/IFU/linebuf 与 P0 fallback BTB 改善冷/热路径；Dispatch dual 达到 **66%**，说明后端经常能配出 2-wide 包——提交端才有东西可双退。

3. **LSU/D$ 命中与前递变快，减少「等一条依赖」造成的单发泡**  
   fast_wb / STQ 前递 / D$ 命中握手改善后，load→ALU 对更容易同窗口就绪；A0 上 dual 甚至到 **89%**，说明在供指与分支都健康时，双发是后端常态。

**不是主因：** I$/D$/L2 hit 几乎持平或略降——双发率上升**不是**靠更大 cache 堆出来的。

**仍受限：** `perf_front_stream` dual 仅 ~8%（IB push_stall≈70%），`perf_branch_mix` MPKI≈33——说明双发收益高度依赖「预测准 + 供得上」；专项短板仍在。

---

## 5. 怎么测（复现）

```bash
export CHIPLAB_HOME=/home/dogandlamb/chiplab
export PATH=$CHIPLAB_HOME/toolchains/loongson-gnu-toolchain-8.3-x86_64-loongarch32r-linux-gnusf-v2.0/bin:$CHIPLAB_HOME/toolchains/nemu:$PATH
cd $CHIPLAB_HOME/sims/verilator/run_prog
make verilator && make testbench

# 正确性闸
./configure.sh --run func/func_lab19 --output-uart-info
make soft && make BUS_DELAY_RANDOM_SEED=5570815 run

# Linux difftest（primary）
./configure.sh --run linux --disable-simu-trace --output-uart-info
make soft && make BUS_DELAY_RANDOM_SEED=5570815 run

# 四探针（务必显式 seed，勿吃 Makefile 默认 1818351）
./configure.sh --run perf_front_stream,perf_branch_mix,perf_dcache_miss,perf_mshr_burst \
  --disable-trace-comp --output-uart-info
make soft && make BUS_DELAY_RANDOM_SEED=5570815 run

# U-Boot → Linux（configure 后必须重编 testbench，否则仍会在 u-boot@ 停仿）
./configure.sh --run uboot_linux --disable-trace-comp --disable-simu-trace --output-uart-info
make testbench && make soft && make BUS_DELAY_RANDOM_SEED=5570815 run
```

| 负载 | 配置 | 成功标志 |
|------|------|----------|
| lab19 | difftest on；seed `5570815` | `END by Syscall` |
| lab19 七 seed | `scripts/v5/run_lab19_7seeds.sh` 或本轮 `artifacts/v5.1/lab19_7seeds/` | 7/7 `END by Syscall` |
| 四探针 + A0/IP | `--disable-trace-comp`；seed `5570815` | 结束语 + PERF |
| Linux difftest | **不要** `--disable-trace-comp` | `/ #` + 无 mismatch |
| uboot_linux | `--disable-trace-comp`；**`make testbench`** | `bootelf` + profile Done + shell |
| L2 CACOP | `scripts/v5/test_l2_cacop.sh` | `L2 CACOP TEST PASS` |

### 证据路径与哈希

| 产物 | 路径 | SHA-256 |
|------|------|---------|
| lab19 日志 | `artifacts/v5.1/lab19_5570815.log` | `d0cab68daae0a2e084f45ca93ada6b376db12188f592cee0307047d2a8c74a3d` |
| lab19 七 seed | `artifacts/v5.1/lab19_7seeds/` | 见目录内 `SHA256SUMS` |
| Linux 日志 | `artifacts/v5.1/linux_difftest_5570815.log` | `ff3b25edaa603ab385fe7c727ad4e5a6bc10e9e328dd582b4f4325d88ecbddc8` |
| 探针 batch | `artifacts/v5.0/perf_matrix_128k/run_batch1.log` | `6871f18778b2686851433a761bb79fe06b3f5da093a5de26f72e2e7517d11927` |
| 探针 PERF 摘录 | `artifacts/v5.0/perf_matrix_128k/*.perf.txt` | （随矩阵） |
| U-Boot → Linux | `artifacts/v5.1/uboot_linux_5570815.log` | `077ab6fedb4b5e2ee689622c478ac60c44a618068bb74ecfbc95e05d5ee485b5` |
| FPGA 70 MHz 时序摘要 | `T202618123009783-perf/perf_timing_summary.rpt` | `eb0c376ec10f85af13300cb347904630ea1e593ddff6c0479a1c3a64a59878ac` |
| FPGA 70 MHz 频/slack 校验 | `T202618123009783-perf/perf_clock_timing_validation.txt` | `a26f4475f094fb0968d5af2b726721462599d977c65cf57870d275ac2ebb93d1` |

`artifacts/` 与 `T202618123009783-perf/` 默认不入 Git。

---

## 6. 必测性能指标清单（本轮覆盖）

已在 §3 / 附录覆盖：Commit IPC、dual、Dispatch dual、BPU/MPKI、I$/D$/L2 hit、MSHR、IB/FTQ、Flush、FTB/TAGE overflow、队列 occupancy。本轮已补 **七 seed** 与 **uboot_linux**；未覆盖项：软门 Linux、板上网络。

---

## 附录 A：lab19 PERF 全文（seed `5570815`）

```
==================== myCPU PERF (sim) ====================
IFU diag:           ftq_valid=472862 accept=83627 pre_stall=389235
  pre_wait_req=310318 pre_wait_if=90409 if_wait_data=199464 if_wait_ib=84975
  I$ outstanding=412228 req=64980 addr_ok=64947 data_ok=64946 mismatch=12/0
  I$ uncached=302 direct/pre/replay=0/302/0 first/last=1c000000/1c07d080
Commit IPC:         retire=122875  cycles=561088  IPC=0.218994
Commit dual-issue:  dual_cyc=35843  any_cyc=87032  dual_rate=41.18%  (of commit cycles)
  vs all cycles:    dual_rate=6.39%  IPC_util=10.95% of 2.0 peak
Dispatch dual:      dual_cyc=59854  any_cyc=82783  dual_rate=72.30%  dual_ALU=51389
Rename/dispatch:    alloc_cyc=82575  turnover_refill=53259  dispatch_stall=70946  rob_stall=15695
BPU all-branch:     total=22999  mispred=1946  accuracy=91.54%
BPU cond-branch:    total=17453  mispred=1537  accuracy=91.19%
FTB (P1 response):  resp=213868  hit=8101  hit_rate=3.79%
ICache (cached):    access=64645  hit=49608  hit_rate=76.74%
  I$ miss lat:      n=15036  avg=19.63  max=57  MPKI=122.368
DCache (cached):    access=71  hit=42  hit_rate=59.15%
  DCache load:      access=44  hit=21  hit_rate=47.73%
  DCache store:     access=27  hit=21  hit_rate=77.78%
  D$ ld miss lat:   n=20  avg=17.50  max=34  st_miss=6
  D$ CWF probe:     early_opp=20  beat1_lo=0  beat1_hi=0
L2 (line rd):       access=30024  hit=14799  miss=15225  hit_rate=49.29%
---- stall / mem ----
ROB full cycles:    15730  (2.80%)
SB  full cycles:    55  (0.01%)
SB nonempty/drain:  nonempty=35125  (6.26%)  wr_req=2843  (0.51%)
D$ MWAIT cycles:    244  (0.04%)
D$ pend cycles:     342  (0.06%)  push=4
D$ MSHR busy:       755  (0.13%)
D$ MSHR occupancy:  avg=0.001  max=1  cap=2
LSU store-order:    21356  (3.81%)
LSU DC wait:        36589  (6.52%)
LSU STQ:            full=0  (0.00%)  avg=0.120  max=6  cap=16
RS_MEM src stall:   65675  (11.70%)
RS_MEM LSU stall:   21070  (3.76%)
RS_MEM full stall:  0  (0.00%)
---- frontend / branch / D$ outstanding ----
Branch MPKI:        all=15.837  cond=12.509  (mispred*1000/retire)
RAS/predec:         ras_call=1280  ras_ret=418  flush=4465  predec_redirect=15986
IB frontend:        empty=389086  (69.34%)  push_stall=85361  (15.21%)  full_occ=7296
FTQ settle/pending: settle=0  (0.00%)  pending=473661  (84.42%)
FTQ P1 bypass:      offer=80288  accept=60040  corrected=676
D$ AXI OS:          N/A (dual-OS not in tree; dc_os_* absent)
Flush/redirect:     flush=4465  (0.80%)  predec_redirect=15986  (2.85%)
---- queue occupancy: avg / max / effective-cap / >=50% / >=75% / full ----
ROB pairs:          2.578 / 15 / 15 / 95991 / 36066 / 15730
ROB valid entries:  4.290 / 30 / 30 / 95785 / 20346 / 24
Dispatch slots:     0.401 / 2 / 2 / 153267 / 71589 / 71589
RS ALU0:            0.250 / 4 / 4 / 6389 / 678 / 119
RS ALU1:            0.181 / 4 / 4 / 26049 / 1946 / 1390
RS MEM:             0.605 / 4 / 4 / 91754 / 85284 / 69171
RS MDU:             0.027 / 2 / 2 / 12688 / 2407 / 2407
Store buffer:       0.126 / 8 / 8 / 2213 / 101 / 55
Instruction buffer: 2.878 / 16 / 16 / 100430 / 90935 / 7296
FTQ retained:       10.428 / 15 / 14 / 401426 / 343945 / 306307
FTQ to IFU pending: 6.565 / 14 / 16 / 257378 / 182464 / 0
RAT busy mappings:  1.726 / 13 / 31 / 0 / 0 / 0
RAS speculative:    22.203 / 32 / 32 / 394971 / 394665 / 54300
RAS committed:      22.207 / 32 / 32 / 394964 / 394659 / 54343
FTB update queue:   max=32/32  overflow=1277
  update traffic:   request=36563  enqueue=30372  tail_merge=4914  dequeue=30372  real_write=30372
FTB update filter:  commit_req=22999  skipped_hit=2422  sent=36563  predec_sent=15986
P0 fallback BTB:    query=556623  hit=64662  hit_rate=11.62%  taken=59910
  fallback types:   cond=8440  uncond=55309  call=421  ret=492
  fallback train:   update=38985  direction=17453  replacement=22274
P1 correction:      total=969  uBTB_hit=506  uBTB_miss=463  direction=555  target=83  length=427
Predecode train Q:  max=1/2  request=15986  direct=0  enqueue=15986  merge=0  dequeue=15986  overflow=0
TAGE update queue:  max=1/32  overflow=0  pipe_max=4
TAGE providers:     base=15473/17060  t0=239/353  t1=21/24  t2=14/14  t3=2/2
TAGE weak/alt:      weak=79/168  disagree=150  provider_better=47  alt_better=103
  weak disagree:    total=121  provider_better=38  alt_better=83
TAGE allocation:    success=1537  failure=0  provider_update_lost=0  train=17453
==========================================================
```

---

## 附录 B：Linux difftest PERF 全文（seed `5570815`，`/ #`）

```
==================== myCPU PERF (sim) ====================
IFU diag:           ftq_valid=115267793 accept=57388503 pre_stall=57879290
  pre_wait_req=29054647 pre_wait_if=30836428 if_wait_data=24557630 if_wait_ib=30663645
  I$ outstanding=86382677 req=55879469 addr_ok=55879469 data_ok=55879468 mismatch=27327/0
  I$ uncached=82 direct/pre/replay=0/82/0 first/last=1c000000/a07ebea0
Commit IPC:         retire=121546164  cycles=138758121  IPC=0.875957
Commit dual-issue:  dual_cyc=41032796  any_cyc=80513368  dual_rate=50.96%  (of commit cycles)
  vs all cycles:    dual_rate=29.57%  IPC_util=43.80% of 2.0 peak
Dispatch dual:      dual_cyc=52782302  any_cyc=79873374  dual_rate=66.08%  dual_ALU=19045671
Rename/dispatch:    alloc_cyc=78891531  turnover_refill=71588993  dispatch_stall=22470926  rob_stall=5741439
BPU all-branch:     total=25201789  mispred=348941  accuracy=98.62%
BPU cond-branch:    total=22757282  mispred=318450  accuracy=98.60%
FTB (P1 response):  resp=70456479  hit=32862368  hit_rate=46.64%
ICache (cached):    access=55879387  hit=53878043  hit_rate=96.42%
  I$ miss lat:      n=2001343  avg=13.24  max=172  MPKI=16.466
DCache (cached):    access=36794511  hit=30705205  hit_rate=83.45%
  DCache load:      access=16434321  hit=15734960  hit_rate=95.74%
  DCache store:     access=20360190  hit=14970245  hit_rate=73.53%
  D$ ld miss lat:   n=605260  avg=20.31  max=250  st_miss=5389945
  D$ CWF probe:     early_opp=440074  beat1_lo=821  beat1_hi=165186
L2 (line rd):       access=5868381  hit=3067533  miss=2800848  hit_rate=52.27%
---- stall / mem ----
ROB full cycles:    5821567  (4.20%)
SB  full cycles:    1513729  (1.09%)
SB nonempty/drain:  nonempty=64370467  (46.39%)  wr_req=21104874  (15.21%)
D$ MWAIT cycles:    2450143  (1.77%)
D$ pend cycles:     8267879  (5.96%)  push=414181
D$ MSHR busy:       35333896  (25.46%)
D$ MSHR occupancy:  avg=0.277  max=2  cap=2
LSU store-order:    314200  (0.23%)
LSU DC wait:        8693992  (6.27%)
LSU STQ:            full=0  (0.00%)  avg=1.273  max=15  cap=16
RS_MEM src stall:   31395382  (22.63%)
RS_MEM LSU stall:   2685438  (1.94%)
RS_MEM full stall:  0  (0.00%)
---- frontend / branch / D$ outstanding ----
Branch MPKI:        all=2.871  cond=2.620  (mispred*1000/retire)
RAS/predec:         ras_call=85365  ras_ret=75555  flush=928964  predec_redirect=486294
IB frontend:        empty=30823510  (22.21%)  push_stall=31565798  (22.75%)  full_occ=1540299
FTQ settle/pending: settle=0  (0.00%)  pending=121217169  (87.36%)
FTQ P1 bypass:      offer=45001618  accept=37781537  corrected=4652050
D$ AXI OS:          N/A (dual-OS not in tree; dc_os_* absent)
Flush/redirect:     flush=928964  (0.67%)  predec_redirect=486294  (0.35%)
---- queue occupancy: avg / max / effective-cap / >=50% / >=75% / full ----
ROB pairs:          8.262 / 15 / 15 / 91140798 / 41971635 / 5821567
ROB valid entries:  13.767 / 30 / 30 / 85030219 / 5936291 / 71579
Dispatch slots:     1.239 / 2 / 2 / 101456980 / 70461316 / 70461316
RS ALU0:            0.758 / 4 / 4 / 19676302 / 8228549 / 2834473
RS ALU1:            0.554 / 4 / 4 / 15235606 / 6615135 / 2248892
RS MEM:             1.356 / 4 / 4 / 50154088 / 43446463 / 19902273
RS MDU:             0.056 / 2 / 2 / 5273659 / 2486078 / 2486078
Store buffer:       0.724 / 8 / 8 / 3849423 / 2136616 / 1513729
Instruction buffer: 5.936 / 16 / 16 / 56012659 / 35222481 / 1540299
FTQ retained:       12.826 / 15 / 14 / 128248590 / 119593894 / 88461744
FTQ to IFU pending: 3.363 / 14 / 16 / 23971851 / 13510292 / 0
RAT busy mappings:  3.313 / 20 / 31 / 15540 / 0 / 0
RAS speculative:    9.726 / 32 / 32 / 22419992 / 4109825 / 136087
RAS committed:      9.718 / 32 / 32 / 22311476 / 4054387 / 129787
FTB update queue:   max=32/32  overflow=44
  update traffic:   request=2346309  enqueue=2147677  tail_merge=198588  dequeue=2147674  real_write=2147674
FTB update filter:  commit_req=25201789  skipped_hit=23341774  sent=2346309  predec_sent=486294
P0 fallback BTB:    query=137829157  hit=59073512  hit_rate=42.86%  taken=53834396
  fallback types:   cond=55484208  uncond=680113  call=1164193  ret=1744998
  fallback train:   update=25688083  direction=22757282  replacement=4545709
P1 correction:      total=8046783  uBTB_hit=519201  uBTB_miss=7527582  direction=5208257  target=8732  length=5967960
Predecode train Q:  max=2/2  request=486294  direct=0  enqueue=486294  merge=0  dequeue=486294  overflow=0
TAGE update queue:  max=1/32  overflow=0  pipe_max=4
TAGE providers:     base=5202742/5360515  t0=1896597/1932608  t1=1238530/1260227  t2=658986/669953  t3=13509216/13533979
TAGE weak/alt:      weak=90375/124573  disagree=724640  provider_better=675496  alt_better=49144
  weak disagree:    total=55082  provider_better=29207  alt_better=25875
TAGE allocation:    success=262815  failure=55635  provider_update_lost=22  train=22757282
==========================================================
```

---

## 附录 C：探针矩阵 PERF 关键行（seed `1818351`，WIP @ 128KB）

完整块见 `artifacts/v5.0/perf_matrix_128k/*.perf.txt`。摘要：

### fireye/A0
```
Commit IPC:         retire=899470  cycles=678961  IPC=1.324774
Commit dual-issue:  dual_cyc=423714  any_cyc=475756  dual_rate=89.06%
BPU all-branch:     total=221111  mispred=165  accuracy=99.93%
ICache (cached):    access=249414  hit=249201  hit_rate=99.91%
DCache (cached):    access=233221  hit=196999  hit_rate=84.47%
L2 (line rd):       access=32066  hit=28842  miss=3224  hit_rate=89.95%
Branch MPKI:        all=0.183  cond=0.181
IB frontend:        empty=13864  (2.04%)  push_stall=416307  (61.32%)
```

### c_prg/inner_product
```
Commit IPC:         retire=23086392  cycles=20966143  IPC=1.101127
Commit dual-issue:  dual_rate=65.59%
ICache hit_rate=99.97%  DCache hit_rate=97.07%  L2 hit_rate=89.82%
Branch MPKI:        all=0.073
```

### perf_front_stream
```
Commit IPC:         retire=2774428  cycles=4905086  IPC=0.565623
Commit dual-issue:  dual_rate=8.29%
ICache hit_rate=99.97%  L2 hit_rate=17.37%
IB push_stall=69.99%  FTQ pending=86.01%
```

### perf_branch_mix
```
Commit IPC:         retire=1524318  cycles=3050690  IPC=0.499663
BPU all-branch:     accuracy=88.79%
Branch MPKI:        all=33.096  cond=0.289
ICache hit_rate=99.98%  L2 hit_rate=17.01%
```

### perf_dcache_miss
```
Commit IPC:         retire=1542687  cycles=1774965  IPC=0.869137
DCache hit_rate=68.09%  L2 hit_rate=94.16%
D$ ld miss lat:     n=41084  avg=2.03  max=59
D$ MSHR busy:       16.51%  occupancy avg=0.190 max=2 cap=2
```

### perf_mshr_burst
```
Commit IPC:         retire=763187  cycles=1602498  IPC=0.476248
DCache hit_rate=71.32%  L2 hit_rate=80.78%
D$ ld miss lat:     avg=2.58  max=79
D$ MSHR busy:       14.53%  occupancy avg=0.203 max=2 cap=2
```

### coremark
```
Commit IPC:         retire=2141301  cycles=9849361  IPC=0.217405
Commit dual-issue:  dual_rate=50.48%
ICache hit_rate=99.77%  DCache hit_rate=98.76%  L2 hit_rate=40.36%
Branch MPKI:        all=2.740
```

### c_prg/memcmp
```
Commit IPC:         retire=981655  cycles=2771248  IPC=0.354228
DCache hit_rate=90.86%  L2 hit_rate=66.86%
Branch MPKI:        all=0.933
```

### L2=32KB 对照（同 WIP）
```
perf_dcache_miss:  IPC=0.649366  L2 hit_rate=22.68%
perf_mshr_burst:   IPC=0.384839  L2 hit_rate=28.46%
```

---

## 附录 D：U-Boot → Linux PERF 摘要（seed `5570815`）

完整块：`artifacts/v5.1/uboot_linux_5570815.perf.txt`。

```
Commit IPC:         retire=152039426  cycles=204551908  IPC=0.743280
Commit dual-issue:  dual_rate=51.85%  (of commit cycles)
Dispatch dual:      dual_rate=67.41%
BPU all-branch:     accuracy=98.83%
Branch MPKI:        all=2.385  cond=2.176
ICache hit_rate=97.33%  DCache hit_rate=84.10%  L2 hit_rate=45.97%
IB frontend:        empty=16.76%  push_stall=35.16%
```
