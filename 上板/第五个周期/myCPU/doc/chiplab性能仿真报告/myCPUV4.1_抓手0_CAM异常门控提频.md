# myCPUV4.1：抓手 0（FTQ→主 TLB→I$ ENARDEN 切除）— 性能仿真报告

> 日期：2026-07-25  
> **基线报告/CPU**：[`myCPUV4.0_55MHz合规收敛.md`](myCPUV4.0_55MHz合规收敛.md)（Linux 软门 IPC **0.703636**；55MHz 官方 WNS≈**+0.08**）  
> 本轮改动：`ftq_direct_req` 异常门控改走 L1 微 TLB **纯 CAM** 口径（摘除主表 `|match0` / `s0_v` 独热腿）  
> 负载：`func_lab19` digftest（含 7 种子）；coremark / dhrystone digftest；Linux 软门 / digftest  
> seed：`5570815`（`BUS_DELAY_RANDOM_SEED`）  
> 原始 PERF 日志：`IP/myCPU/logs_v41/`  
> 时序报告：`IP/myCPU/logs_v41/reports_impl55/`、`logs_v41/reports_impl75/`（同源 `D:/chiplab_v41_sta/out/{55,75}mhz/`）  
> PERF 口径：仿真结束 `myCPU PERF (sim)`（`mycpu_top.v` 中 `` `ifdef SYNTHESIS ``；仿真经 `-DSIMU`）

---

## 0. 结论摘要

| 门控 | 结果 |
|------|------|
| **lab19 digftest** | **PASS**（`END by Syscall`）；基线种子 retire=**122875**，IPC=**0.214694**；7 种子全过 |
| **coremark / dhrystone** | **PASS**（digftest）；IPC=0.201283 / 0.180155 |
| **Linux 软门** | **PASS**（`/ #`）；IPC = **0.703636**（与 V4.0 **bit-identical**） |
| **Linux digftest** | **PASS**（`Difftest enabled`，无 mismatch，至 `/ #`）；IPC = **0.703636**（与软门 bit-identical） |
| **频率 55MHz（合规）** | **PASS**：WNS=**+0.331**，TNS=0；cpu top 中 **ENARDEN=0** |
| **频率 75MHz（摸底）** | **FAIL**：WNS=**−2.577**，TNS=−7996；新 WNS 族为 **D$→LSU→RS_MEM**（非 ENARDEN） |

**一句话：** 抓手 0 功能等价（Linux IPC 零变化）；55MHz 合规裕量 **+0.08→+0.331**，且 `ftq→ENARDEN` 退出榜首；75MHz 摸底失败，瓶颈已切到 **`u_dcache/req_paddr → u_rs_mem/s*_val` CE**（约 15.9ns，估 Fmax≈**63MHz**）。

---

## 1. 本轮改动

| 项 | 留树 | 说明 |
|----|------|------|
| `l1_tlb.v`：`cam_v_o` / `cam_plv_o` | ✅ | 纯 CAM `hit_v` / `hit_plv`，不含主表透传腿 |
| `tlb_manager.v`：`inst_direct_excp` | ✅ | ADEF + CAM 口径 PIF/PPI；完整 TLB 异常仍供 PRE |
| `mmu.v`：`i_direct_excp_o` | ✅ | 并入本地 PC 非对齐 ADEF |
| `mycpu_top.v` 穿线 | ✅ | `tlb_manager → mmu → ifu` |
| `ifu.v`：`ftq_direct_req` 门控 | ✅ | `!pre_excp_now` → `(mmu_i_direct_excp_i !== 1'b1)`；PRE 锁存不动 |

等价性：`ftq_direct_req` 已要求 `mmu_i_direct_ok_i=1`（`!need_tlb \|\| l1i_cam_hit`）。该前提下 TLBR 不可触发，PIF/PPI 的 CAM 口径与完整口径逐位相同。

---

## 2. 功能结果

| 负载 | 判据 | 证据 |
|------|------|------|
| lab19 | `END by Syscall` + digftest | `logs_v41/v41_lab19_difftest.log`；7 种子 `v41_lab19_7seeds.log` |
| coremark / dhrystone | digftest + Syscall | `logs_v41/v41_{coremark,dhrystone}.log` |
| Linux 软门 | UART `/ #`，干净 PERF | `logs_v41/v41_linux_soft.log` |
| Linux digftest | 无 mismatch + `/ #` | `logs_v41/v41_linux_dft.log` |

---

## 3. 性能对比（相对 V4.0，Linux 软门）

| 指标 | V4.0 | V4.1 | Δ |
|------|------|------|---|
| Commit IPC | 0.703636 | **0.703636** | **0**（bit-identical） |
| retire / cycles | 122230172 / 173712099 | 同左 | 0 |
| lab19 IPC（seed0） | 0.214694 | **0.214694** | 0 |

### 3.1 频率 / 时序

隔离工程：`D:/chiplab_v41_sta/`（勿碰 `chiplab_v5_55mhz`）。策略：官方 `Flow_PerfOptimized_high` + `Performance_Explore`；仅改 `clk_pll.CLKOUT1`。换频前关增量综合（`launch_*.tcl` 对齐 V5 模式）。

| 频点 | gate | WNS | 关键观察 |
|------|------|-----|----------|
| **55MHz** | **PASS** | **+0.331** | cpu 榜首：`u_ftq/bpu_ptr → u_icache/req_paddr_reg`（经主 TLB `pre_paddr`）；**ENARDEN=0** |
| **75MHz** | **FAIL** | **−2.577** | cpu 榜首族：**`u_dcache/req_paddr → u_rs_mem/s*_val` CE**（经 miss/wb→LSU `lsu_ready`）；ENARDEN≈0；估 Fmax≈**63MHz** |

对比旧 50MHz 报告：前 50 条全是 `bpu_ptr→ENARDEN`（~19.5ns）——该族在 55/75MHz top 中已退出主导。

未加跑 65/70MHz：计划约定仅在 75MHz WNS 为小负（\|WNS\|≤0.5）时酌情加跑；本轮 −2.577 超出该门槛。

### 3.2 下一刀判定

| 候选 | 证据 | 判定 |
|------|------|------|
| 方案 **1.0.5**（I$/翻译再解耦、mat 走 CAM） | 55MHz 仍见 FTQ→`req_paddr`/`blk_target` | 抬前端裕量时做；**非 75MHz 当前 WNS** |
| **D$→LSU→RS_MEM** 长链 | 75MHz top 全是 `u_dcache → u_rs_mem`（100 条 cpu 对） | **当前提频主瓶颈**；单独定位（与方案文档「抓手 0 后可能冒头」一致；类 V5 60MHz 的 D$ 族） |
| 抓手 A（RF）/ B（rs_alu）/ C（TAGE） | 75MHz WNS 未落此 | **本轮不启动** |
| IPC 变体合入 | 75MHz 未站稳 | **等频率线收敛后再合** |

---

## 4. 解读

- **优化：** 直发异常门控去掉主 TLB 32 路归约 / `s0_v` 独热，切断 `ftq→ENARDEN` 组合长链。  
- **等价性：** Linux / lab19 周期与 retire 与 V4.0 完全一致，符合「逐拍等价」设计。  
- **提频：** 55MHz 合规裕量扩大且 ENARDEN 退场——抓手 0 目标达成；75MHz 暴露后端访存唤醒链（D$ miss/wb → LSU ready → RS_MEM CE），约需再砍 **~2.6ns** 才摸到 75。  
- **下一步：** 针对 `req_paddr→rs_mem` 做路径拆解（打拍 / 假路径 / 收锥）；前端 1.0.5 作并行候选；站稳 ≥65–70MHz 后再开 IPC 线。

---

## 5. 怎么测（复现）

```bash
export CHIPLAB_HOME=/home/dogandlamb/chiplab
export PATH=$CHIPLAB_HOME/toolchains/loongson-gnu-toolchain-8.3-x86_64-loongarch32r-linux-gnusf-v2.0/bin:$CHIPLAB_HOME/toolchains/nemu:$PATH
cd $CHIPLAB_HOME/sims/verilator/run_prog

# lab19（TRACE_COMP=y；改 case 后须 make testbench）
./configure.sh --run func/func_lab19
make verilator && make testbench
rm -rf obj/func/func_lab19_obj && make soft_compile
make simulation_run_prog

# Linux 软门
make clean_all
./configure.sh --run linux --disable-simu-trace --disable-trace-comp --disable-read-miss --output-uart-info
rm -rf obj/linux_obj && make soft_compile
make verilator && make testbench
make simulation_run_prog

# Linux digftest：去掉 --disable-trace-comp；placeholder golden_trace（cmp_flag=0）
# STA：D:/chiplab_v41_sta/scripts/launch_{55,75}mhz.tcl（先关增量综合再改 PLL）
```

注意：换 `RUN_FUNC`/`RUN_C`/`TRACE_COMP` 后必须重编 `testbench`，否则 `END_PC` 错会把 Linux 早退。

---

## 附录：PERF / 日志索引

```
# Linux 软门 / digftest
Commit IPC:         retire=122230172  cycles=173712099  IPC=0.703636

# lab19 seed0
Commit IPC:         retire=122875  cycles=572327  IPC=0.214694

# 75MHz gate_status
wns=-2.577  automatic_gate=FAIL  gate_reason=NEGATIVE_WNS
```

| 门 | 日志 |
|----|------|
| lab19 / 7seeds | `logs_v41/v41_lab19_{difftest,7seeds}.log` |
| coremark / dhrystone | `logs_v41/v41_{coremark,dhrystone}.log` |
| Linux 软门 / digftest | `logs_v41/v41_linux_{soft,dft}.log` |
| 55MHz STA | `logs_v41/reports_impl55/`（WNS=+0.331 PASS） |
| 75MHz STA | `logs_v41/reports_impl75/`（WNS=−2.577 FAIL） |
