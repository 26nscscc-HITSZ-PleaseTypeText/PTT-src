# myCPUV4.0：55MHz 合规 WNS≥0（保 IPC）— 性能仿真报告

> 日期：2026-07-23  
> **基线报告/CPU**：`myCPUV3.4_store路径+后端前递+前端容量+扩大L2cache+55MHz.md`（Linux 软门 IPC **0.740188**；55MHz 曾用非官方 AggressiveExplore 摸底 WNS=+0.023）  
> 本轮主题：在龙芯杯 **4.3.3 / 4.3.4** 约束下，靠 myCPU RTL 收敛 **55MHz 官方综实 WNS≥0**，尽量保 IPC（底线 ≥0.70）  
> 负载：`func_lab19` digftest；Linux 软门 / digftest（分列）  
> seed：`5570815`（`BUS_DELAY_RANDOM_SEED`）  
> 原始 PERF 日志：`IP/myCPU/logs_v34/v34_55_*`  
> 时序报告：`IP/myCPU/logs_v34/reports_impl55_official_func/`、`reports_impl55_official_perf/`  
> PERF 口径：仿真结束 `myCPU PERF (sim)`（`mycpu_top.v` 中 `` `ifdef SYNTHESIS ``；仿真经 `-DSIMU`）

---

## 0. 结论摘要

| 门控 | 结果 |
|------|------|
| **lab19 digftest** | **PASS**（`END by Syscall`）；IPC = **0.214694** |
| **Linux 软门** | **PASS**（`/ #`）；IPC = **0.703636**（≥0.70；相对 V3.4 峰值 −4.9%） |
| **Linux digftest** | **PASS**（无 mismatch，至 `/ #`）；IPC = **0.703636**（与软门 bit-identical） |
| **频率（合规）** | **55MHz**：仅改 `clk_out1`；官方 `Flow_PerfOptimized_high` + `Performance_Explore`；功能 WNS=**+0.080** / 性能 WNS=**+0.083**；WPWS=0（`ddr_clk`=200） |

**一句话：** 三门绿；在 **禁止额外 AggressiveExplore / 禁改 xdc·策略 / 禁改 clk_out2** 的前提下，官方综实功能与性能均 **WNS≥0**；交付 Linux IPC **0.70**（相对 V3.4 的 0.74 有时序裁剪代价）。

---

## 1. 本轮改动

| 项 | 留树 / 回退 | 说明 |
|----|-------------|------|
| `LSU_DC_HIT_BYPASS=0`（hold） | ❌ 回退 | Linux hang（libm / commit-stall）；不作为交付路径 |
| `LSU_WB_PIPE=1` | ✅ | 顶层 `mem_wb→RS/ROB` 整总线打一拍，切断 D$ hit→RS 组合链 |
| `LSU_EARLY2_ENABLE=0` | ✅ | 与 WB_PIPE 配对关闭（避免 T 醒、T+1 数据） |
| `LSU_EARLY2_PIPE=0` | ✅ | 保持关闭 |
| `IFU_FTQ_DIRECT=1` + `mmu_i_direct_ok` | ✅ | 仅 DA/DMW/L1 CAM 命中时同拍 FTQ→I$；切断 FTQ→主 TLB→I$ ENARDEN |
| `dcache.front_ram_busy` | ✅ | 去掉与 `mshr_install` 互斥的 `ld/st/cacop_take`（STA 假路径，零 IPC） |
| 额外 `phys_opt AggressiveExplore` | ❌ 禁止交付 | 违 4.3.4；仅可私下摸底 |
| `clk_out2` / `soc_lite.xdc` / 综实策略 | ❌ 未改 | 遵守 4.3.3 / 4.3.4 |

交付宏：

```text
LSU_DC_HIT_BYPASS = 1
LSU_WB_PIPE       = 1
LSU_EARLY2_ENABLE = 0
LSU_EARLY2_PIPE   = 0
IFU_FTQ_DIRECT    = 1   // 且仅 mmu_i_direct_ok 时开火
L2_NSET           = 2048
FTB_NSET          = 2048
```

---

## 2. 功能结果

| 负载 | 判据 | 证据 |
|------|------|------|
| lab19 | `END by Syscall` | `logs_v34/v34_55_lab19_frambusy.log` |
| Linux 软门 | UART `/ #`，干净 PERF | `logs_v34/v34_55_linux_l1dir.log` |
| Linux digftest | 无 mismatch + `/ #` | `logs_v34/v34_55_ldiff_frambusy.log` |

---

## 3. 性能对比（相对基线 V3.4，Linux 软门）

| 指标 | V3.4（峰值） | V4.0（55MHz 合规交付） | Δ |
|------|-------------|------------------------|---|
| Commit IPC | 0.740188 | **0.703636** | −4.9% |
| Cycles（至 `/ #`） | 164.6M | 173.7M | +5.5% |
| 全实现频率 | 55MHz（非官方 AggressiveExplore 摸底） | **55MHz 官方综实 WNS≥0** | 合规 |

### 3.1 频率 / 时序（合规）

**约束：** 仅改 `clk_pll.clk_out1=55`；不改 `clk_out2`；不改 xdc/策略；提交 bit **WNS≥0**。

| 步骤 | 结果 |
|------|------|
| 官方功能 SoC | WNS=**+0.080**，WPWS=0；报告 `logs_v34/reports_impl55_official_func/` |
| 官方性能 SoC | WNS=**+0.083**，WPWS=0；bit `bit_impl55_official_perf/soc_top_perf_PASS_wns0083.bit`；报告 `logs_v34/reports_impl55_official_perf/` |
| WPWS−0.190（历史） | IDELAYCTRL 曾 ~183MHz；本工程 `clk_pll_ddr`@200、CLKOUT3 未用 → WPWS 已非负 |
| 脚本 | `impl_55mhz_official.tcl` / `impl_55mhz_official_perf_run.tcl`（Desktop chiplab） |

关键路径演进（性能 SoC）：V3.4 违约主因 D$→LSU→RS；本轮 WB_PIPE 切开后，再经 IFU `direct_ok` 与 `front_ram_busy` 假路径切除，官方 route **WNS=+0.083**。

---

## 4. 解读

- **优化：** 用 RTL 打拍 / 门控切断官方综实下的违约组合链，替代非合规 phys_opt。  
- **代价：** 相对 V3.4 峰值 IPC 约 −5%（0.74→0.70），仍满足 ≥0.70 底线。  
- **合规：** 功能与性能 Implementation 均 WNS≥0，未改 clk_out2 / xdc / 策略。  
- **下一步：** 在保持 WNS≥0 前提下尽量收回前端气泡，逼近 0.74；60MHz 仍需经验证的数据通路打拍。

---

## 5. 怎么测（复现）

```bash
export CHIPLAB_HOME=/home/dogandlamb/chiplab
export PATH=$CHIPLAB_HOME/toolchains/loongson-gnu-toolchain-8.3-x86_64-loongarch32r-linux-gnusf-v2.0/bin:$PATH
# lab19: run_prog；Linux 软门: run_prog（disable-trace-comp）；Linux digftest: run_prog3
cd $CHIPLAB_HOME/sims/verilator/run_prog
make verilator && make testbench && make simulation_run_prog
```

全实现（Windows Desktop）：`impl_55mhz_official*.tcl`；**勿**用 `impl55_closure.tcl` AggressiveExplore 作出板交付。

---

## 附录：PERF 摘录（V4.0 Linux 软门）

```
Commit IPC:         retire=122230172  cycles=173712099  IPC=0.703636
```

### 日志索引

| 门 | 日志 |
|----|------|
| lab19 | `logs_v34/v34_55_lab19_frambusy.log` |
| Linux 软门 | `logs_v34/v34_55_linux_l1dir.log` |
| Linux digftest | `logs_v34/v34_55_ldiff_frambusy.log` |
| 时序功能/性能 | `logs_v34/reports_impl55_official_{func,perf}/` |
