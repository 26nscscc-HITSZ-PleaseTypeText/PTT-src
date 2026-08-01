# myCPU V5.2：双 MEM issue 与 75 MHz 上板

> 日期：2026-08-01  
> **基线报告/CPU**：[`myCPUV5.1：前端LSU提速_IPC↑0.875957.md`](myCPUV5.1：前端LSU提速_IPC↑0.875957.md)；基线 Linux difftest IPC `0.875957`  
> 基线对比：V5.1 Linux difftest IPC `0.875957`；V5.0 `0.734196`  
> 主对比源：**Linux difftest**（seed `5570815`）  
> 板上证据：`T202618123009783-perf_2/`（75 MHz setup MET）  
> 仿真产物：`artifacts/v5.2/`  
> chiplab 工作树相对发布点 `02e8922`（WIP，未单独打 tag）

---

## 0. 结论摘要

| 门控 | 结果 |
|------|------|
| `func_lab19` + 七 seed | **PASS**（7/7，`END by Syscall`，retire=122875） |
| L2 CACOP 专项 | **PASS**（`L2 CACOP TEST PASS`） |
| C/探针短测 | **PASS**（A0 / inner_product / memcmp / CoreMark + 四 `perf_*`） |
| Linux difftest | **PASS** 到 `/ #`；IPC **`0.840564`**（相对 V5.1 `−4.04%`） |
| U-Boot → Linux | **有条件 PASS**（`bootelf` + `Processing /etc/profile... Done` + `/ #`）；IPC **`0.713575`**（相对 V5.1 `−4.00%`；cmdline=`earlycon`） |
| 75 MHz 实现 / CI | **PASS**：WNS **`+0.052`**，TNS 0；Functional 58/58；IPC ratio **1.935**；System counter ratio **4.438** |
| 65 MHz 正式门（旧脚本） | **未复测**；不以「正式 65 门」冒充 PASS |

**一句话：** 双 MEM 入队 + 访存路径时序刀在 **75 MHz 板上 MET**；仿真闸全绿。相对 V5.1，Linux/uboot Commit IPC 各约 **−4%**（dual-issue 51%→41%），换来更高频与 CI 性能分；lab19 曾因 `cmt_mem_bypass` 与 difftest 口不同步假报 `paddr=0`，已对齐 `*_eff`。

**CI 两比释义（勿与仿真 Commit IPC 混比）：**

- **IPC ratio** = `GEOMEAN(openla500_cpu_count / mycpu_cpu_count)` → 相对参考核的 **CPU 周期效率加速比**
- **System counter ratio** = `GEOMEAN(openla500_soc_count / mycpu_soc_count)` → 相对参考核的 **墙钟加速比**（≈ 周期效率 × 频率比；本轮约 `1.94 × 75/33 ≈ 4.44`）

---

## 1. 本轮改动（相对 V5.1）

| 项 | 文件 / 宏 | 说明 |
|----|-----------|------|
| 双 MEM issue | `rename.v` / `dispatch.v` / `rs_mem.v` | mem+mem 双入队；`push1_*` + `can_accept_two` |
| RS_MEM 容量 | `RS_MEM_SIZE` **4→8** | 覆盖短 LSU 反压；issue 扫描锥仍限 4 项（时序） |
| Load 快速旁路关 | `RS_MEM_LOAD_FAST_BYPASS=0` | 切断 D$→RS_MEM 组合长链 |
| SB 打拍 | `store_buffer.v` | drain 包寄存器；query 一拍返回 + resp 标签匹配 |
| LSU/WB 打拍 | `LSU_WB_PIPE=1`；commit `cmt_mem_bypass` | 常规 mem WB 寄存；提交同拍旁路 ROB |
| Difftest 对齐 | `mycpu_top.v` | Store/LoadEvent 改锁存 `cmt*_paddr/vaddr/result_eff`（修 bypass 拍 ROB 仍为 0） |
| Boot-RAM cache promote | `COMPETITION_BOOT_RAM_CACHE` **关** | 注释掉 define；UC boot-RAM 契约与 lab19 自修改一致 |
| IFU invalidate | `frontend/ifu.v` | 恢复同拍 gate（停 IB push / 清 PRE·IF / 拦 ftq_accept） |
| 继承 V5.1 | `fallback_btb.v` 等 | 未改 |

**lab19 根因（本轮）：** 非「真 store 基址为 0」。`cmt_mem_bypass` 让 store 在 ROB NBA 写回前提交；SB 已用 `*_eff`，difftest 仍读 `rob_cmt*_paddr`→`0`，表现为 `Both Error(0x700)`。对齐 `*_eff` 后七 seed 全绿。

---

## 2. 板上 / CI 证据（`perf_2` @ 75 MHz）

主证据目录：[`T202618123009783-perf_2/`](../../../../T202618123009783-perf_2/)（勿用同名 `T202618123009783-perf` / `(2)` 的 70 MHz FAIL 拷贝）。

| 项 | 值 |
|----|-----|
| 工具 / 设计 | Vivado 2023.2；`soc_top`；`7a200t-fbg676-2`；Routed |
| cpu_clk | period **13.333 ns = 75.000 MHz**（板上 actual **75.001875 MHz**） |
| Design Timing | **All constraints met**：WNS **`+0.052 ns`**，TNS **`0`**；hold WHS **`+0.050`** |
| CI Functional | **100.000000**（58/58 points） |
| CI IPC ratio | **1.934633120** |
| CI System counter ratio | **4.438258572** |
| Benchmarks | 20/20 complete；`failed_benchmarks=none` |
| Branch / Commit（CI） | `T202618123009783` / `d512e103…` |

SHA-256：

| 文件 | SHA-256 |
|------|---------|
| `perf_timing_summary.rpt` | `06b8c156e0697e4ad4109c3931c6cfebfe4ce449380d59d85349cc89d41c093b` |
| `perf_clock_timing_validation.txt` | `c2f3344954f7ee084259d17c94ad2ece11e10b0e72a726b7501d99374335a583` |
| `perf_score_summary.txt` | `09f0b7433e58b2ea3e38200261bcbad33d36aed754e1c7b22119063b41645a7b` |

---

## 3. 仿真矩阵

种子一律 **`BUS_DELAY_RANDOM_SEED=5570815`**（七 seed 表见下）。

### 3.1 门控总表

| 闸 | 配置要点 | 结果 | 证据 |
|----|----------|------|------|
| lab19 | difftest on | PASS；IPC 0.220106；cycles 558255 | `artifacts/v5.2/lab19_5570815.log` |
| lab19 七 seed | 见 §3.2 | **7/7 PASS** | `artifacts/v5.2/lab19_7seeds/` |
| L2 CACOP | `scripts/v5/test_l2_cacop.sh` | PASS | `artifacts/v5.2/l2_cacop.log` |
| 短测 | `--disable-trace-comp` | 8/8 到 test end | `artifacts/v5.2/short_matrix/run_all.log` |
| Linux | **不要**关 TRACE_COMP | `/ #`；IPC 0.840564 | `artifacts/v5.2/linux_difftest_5570815.log` |
| uboot_linux | 关 TRACE；configure 后 **`make testbench`** | profile Done + `/ #`；IPC 0.713575 | `artifacts/v5.2/uboot_linux_5570815.log` |

### 3.2 lab19 七 seed

| 名称 | seed | 结果 | retire | IPC |
|------|------|------|--------|-----|
| baseline | 5570815 | PASS | 122875 | 0.220106 |
| short1 | 5592575 | PASS | 122875 | 0.219614 |
| short2 | 2796287 | PASS | 122875 | 0.219613 |
| normal1 | 5570561 | PASS | 122875 | 0.219818 |
| normal2 | 2796769 | PASS | 122875 | 0.219652 |
| normal3 | 7410228 | PASS | 122875 | 0.219746 |
| normal4 | 1818351 | PASS | 122875 | 0.219838 |

相对 V5.1 baseline（IPC 0.218994 / 561088 cycles）本轮略快（558255 cycles）。

### 3.3 C / 探针短测（seed 5570815）

| 负载 | retire | cycles | IPC |
|------|--------|--------|-----|
| `fireye/A0` | 899035 | 824079 | 1.090957 |
| `c_prg/inner_product` | 23039375 | 20755024 | 1.110063 |
| `c_prg/memcmp` | 956178 | 2836824 | 0.337059 |
| CoreMark | 2036605 | 9994237 | 0.203778 |
| `perf_front_stream` | 2770357 | 5039693 | 0.549707 |
| `perf_branch_mix` | 1506669 | 3185616 | 0.472960 |
| `perf_dcache_miss` | 1533834 | 1909881 | 0.803104 |
| `perf_mshr_burst` | 759125 | 1751579 | 0.433395 |

### 3.4 Linux / uboot

| 负载 | V5.1 IPC | V5.2 IPC | Δ | 备注 |
|------|----------|----------|---|------|
| Linux difftest | 0.875957 | **0.840564** | **−4.04%** | retire 121.8M；无 mismatch |
| uboot_linux | 0.743280 | **0.713575** | **−4.00%** | `earlycon`；不与 difftest 混比 |

---

## 4. 性能解读

### 4.1 相对 V5.1（Linux difftest）

| 指标 | V5.1 | V5.2 | 解读 |
|------|------|------|------|
| Commit IPC | 0.875957 | 0.840564 | 时序刀代价；板上换 75 MHz |
| Commit dual-rate（of commit cycles） | 50.96% | 40.79% | 主回归项 |
| RS_MEM 占用 avg/max/cap | 1.356 / 4 / 4 | **2.823 / 8 / 8** | 双 MEM 入队生效 |
| Dual-load decode `mem_pair` | （V5.1 无此计数） | **10.55M** | rename 放行 mem+mem |
| Branch MPKI (all) | 2.871 | 2.898 | 基本持平 |
| ROB full | 4.20% | 9.37% | 后端更满，与 dual-MEM 一致 |
| `RS_MEM_LOAD_FAST_BYPASS` | （开） | **0** | 拉长 load→依赖唤醒 |

**dual-MEM 旁证：** Linux 上 `mem_pair≈10.5M`，RS_MEM 平均占用升至 ~2.8/8；正确性闸未因双入队失败。IPC 下降主要来自访存旁路/打拍与 dual-issue 率回落，而非前端误预测恶化。

### 4.2 75 MHz 与「正式 65 门」

- 竞赛/CI：**75 MHz setup MET（WNS +0.052）** → 本轮板上 **PASS**
- 仓库旧正式门（`check_timing_65mhz.sh`：65 MHz 且 WNS ≥ +0.200）**本轮未跑**；报告不声称该门 PASS

### 4.3 与 V5.0 探针粗比（口径提示）

短测 IPC 相对 V5.0 `perf_matrix_128k` 有涨有跌（A0/CoreMark 等受 seed、软件重建与后端改动影响）。**主对比以 Linux difftest 为准**；探针仅作结构压力旁证。

---

## 5. 复现命令与哈希

```bash
export CHIPLAB_HOME=~/chiplab
export PATH=$CHIPLAB_HOME/toolchains/loongson-gnu-toolchain-8.3-x86_64-loongarch32r-linux-gnusf-v2.0/bin:$CHIPLAB_HOME/toolchains/nemu:$PATH
cd $CHIPLAB_HOME/sims/verilator/run_prog

make verilator && make testbench

# lab19
./configure.sh --run func/func_lab19 --output-uart-info
make soft && make BUS_DELAY_RANDOM_SEED=5570815 run
# 七 seed：见 sims/verilator/run_prog/run_7_seeds.sh 或 artifacts/v5.2/lab19_7seeds/

# L2 CACOP
$CHIPLAB_HOME/scripts/v5/test_l2_cacop.sh

# 短测
./configure.sh --run fireye/A0,c_prg/inner_product,c_prg/memcmp,coremark,perf_front_stream,perf_branch_mix,perf_dcache_miss,perf_mshr_burst \
  --disable-trace-comp --disable-simu-trace --output-uart-info
make testbench && make soft && make BUS_DELAY_RANDOM_SEED=5570815 run

# Linux difftest（主 IPC）
./configure.sh --run linux --output-uart-info
make testbench && make soft && make BUS_DELAY_RANDOM_SEED=5570815 run

# U-Boot → Linux（configure 后必须重编 testbench）
./configure.sh --run uboot_linux --disable-trace-comp --disable-simu-trace --output-uart-info
make testbench && make soft && make BUS_DELAY_RANDOM_SEED=5570815 run
```

### 5.1 仿真产物 SHA-256

| 文件 | SHA-256 |
|------|---------|
| `lab19_5570815.log` | `d3727005a60535bc006d3ab94b75512be080239fc272592a65c0f946bf124af9` |
| `lab19_5570815.perf.txt` | `350bc83d98ba40de4a1f883ab35b79e2ea7d6dbed21146912bc5d9a70ef69545` |
| `lab19_7seeds/summary.tsv` | `e16b3cc8bf88e6d6923ab2225f169f4e6c217bbcd066c052c18ad43d32d8bbfd` |
| `l2_cacop.log` | `8368c90e0df7d14be67c9e12889c02d968ba2c8c1a490109583407627bc7ef58` |
| `short_matrix/run_all.log` | `a15b0c0242c1963bc9afbbf59829a4e0d2f4e9891242a6fa28d169b678259f19` |
| `linux_difftest_5570815.log` | `ca13841153b449ea49e8fe0cd8736a05f5377640154b283052c11a1156119e21` |
| `linux_difftest_5570815.perf.txt` | `f30e8dc8166898559013266efe6981302082dab8d1772665f1e78b4b64a3ab7f` |
| `uboot_linux_5570815.log` | `e1eedbcec5d71a39b52187431504f2edab3ad5b0004485bab31fd368df3c4934` |
| `uboot_linux_5570815.perf.txt` | `8908c80124d4c257b16f3f7d70608bd540f099a9a7829dea5493f41e25fefdb3` |

更全目录哈希见 `artifacts/v5.2/SHA256SUMS_main.txt` 与 `lab19_7seeds/SHA256SUMS`。
