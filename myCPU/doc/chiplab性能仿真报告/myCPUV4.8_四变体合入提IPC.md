# myCPU V4.8：四变体合入提 IPC（基于 V4.4 抓手 3）

> 日期：2026-07-27  
> **基线报告/CPU**：[`myCPUV4.4_抓手3_IFU_hit旁路解耦提频.md`](myCPUV4.4_抓手3_IFU_hit旁路解耦提频.md)  
> **验收口径**：仅跑 `func_lab19` + Linux digftest（到 `/ #`），Difftest 未 mismatch 时取 `myCPU PERF (sim)` 的 `Commit IPC`。  
> seed：`BUS_DELAY_RANDOM_SEED=5570815`

---

## 0. 结论摘要（本轮最终状态）

| 门控 | 结果 |
|------|------|
| `func_lab19` digftest | PASS（`END by Syscall`）；IPC≈0.2143 |
| Linux digftest | PASS（到 `/ #`，无 mismatch） |
| **Linux digftest Commit IPC** | **0.798378** |

**与 V4.1 冷启动软门 IPC 目标对比**：V4.1 目标值 `0.798619`，本轮相差 **-0.000241（≈-0.03%）**，属于极小偏差；本轮正确性优先，未进一步激进开全组合能力（见第 4 节“偏离点”）。

---

## 1. 本轮合入内容（按成果来源归类）

### 1.1 冷启动前端（3.2：前端 FTQ-indexed RAS/TAGE checkpoint + predec RET 恢复）
合入到 [`IP/myCPU/frontend`](IP/myCPU/frontend/)（对应文件见下表），并保持 V4.4 抓手 3 的时序刀：

- `frontend/ras.v`：FTQ-indexed RAS checkpoint / restore  
- `frontend/tage.v`：TAGE hist checkpoint 与 `GHR_LEN=112` 对齐  
- `frontend/ubtb.v`：RET 描述符保留与 cold-start 语义  
- `frontend/inst_buffer.v`：IB（指令缓冲）实现差异合入  
- `frontend/ifu.v`：`predec_ras_*` + mid-block RET recovery 接线合入  
- `frontend/bpu.v` / `frontend/ftq.v`：RAS/TAGE checkpoint 与 predec redirect 指针修正合入  
- `mycpu.h`：`GHR_LEN 1136 → 112`

并保留 V4.4 关键时序/正确性语义：  
- **取消 I$ 命中旁路进入 predec**（`ifu.v`：不再使用 `if_rline_eff`）  
- **P0/P1 同拍纠正与 FTQ 写入解耦**（`bpu.v` `p0_valid_o` 不组合看 `p1_diff`；`ftq.v` 仍按 V4.4 P1 discard 逻辑）

### 1.2 4.0：jirl_target_cache + early uBTB train（并保持冷启动分工）
合入到 [`IP/myCPU/frontend`](IP/myCPU/frontend/) 的以下部分：
- `frontend/ifu.v`：`predec_block_pc_o` / `predec_branch_target_o`  
- `frontend/bpu.v`：JTC（jirl_target_cache）与早训练接口接线  
- `frontend/ubtb.v`：`update_early_i` 与 `taken[]` 行为  
- `frontend/ftq.v` / `mycpu_top.v`：`train_is_direct_b` / `cmt_is_direct_b` 等“direct-B”旗标传递

### 1.3 v3.1 双 outstanding（dc2/mem3）
本轮最终交付配置中 **未启用**（原因见第 4 节偏离点）。

---

## 2. 验收结果（日志关键摘录）

### 2.1 `func_lab19` digftest
日志：`IP/myCPU/logs_v48/v48_lab19_ship2.log`（与 `v48_lab19_final.log` 同口径）  
- `Reached test end PC.`  
- `Commit IPC: retire=122875 cycles=573444 IPC=0.214276`

### 2.2 Linux digftest（到 `/ #`）
日志：`IP/myCPU/logs_v48/v48_linux_ship2_dft.log`  
- `/ #Detected Linux shell '/ #'，stop for PERF dump.`  
- `Reached Linux shell prompt (/ #).`  
- `Commit IPC: retire=122027365 cycles=152844024 IPC=0.798378`

---

## 3. PERF 关键指标（从 Linux digftest PERF 摘录）

来自 `v48_linux_ship2_dft.log` 的主要性能/瓶颈信号：

- Branch MPKI：`all=3.251 / cond=2.968`  
- BPU accuracy：`~98.43%`（cond 与 all 相近）  
- FTB (P1 response) hit_rate：`43.97%`  
- I$ hit_rate：`96.32%`  
- D$ hit_rate：`83.48%`（load 命中率 `95.94%`，store 命中率 `73.20%`）  
- stall（主要来源）：
  - `ROB full cycles: 18251658 (11.92%)`
  - `RS_MEM src stall: 39240282 (25.64%)`
  - `LSU DC wait: 21378878 (13.97%)`
  - `D$ MSHR busy: 37134148 (24.26%)`

Occupancy（部分）：
- FTQ retained：`12.792 avg / max 15`  
- FTQ pending：`145219796 cycles (94.87% cap 以上)`（冷启动/长 warm-up 期间队列“待取”较多）

---

## 4. 偏离计划与原因（必须写清）

本轮“性能正确性优先”，因此以下两项在最终交付配置中 **未开到完整形态**：

### 4.1 JTC（jirl_target_cache）P0 选择被约束关闭
- 在 `IP/myCPU/frontend/bpu.v` 中强制：`JIRL_TC_P0_ENABLE=0`。  
- 原因：在“JTC P0 打开/与其它组合共同启用”时，Linux digftest 出现 CSR 不一致（difftest mismatch，`Code:0x700`），见：  
  - `IP/myCPU/logs_v48/v48_linux_jtcP0_enable_dft.log`（开启后 IPC 从 0.798378 回落到 ~0.797235，且无显式 mismatch 但偏离目标；JTC-only 更早出现 mismatch）。

因此本轮性能收益主要来自：
- 3.2 冷启动前端补强（FTQ-indexed RAS/TAGE checkpoint + predec RET 恢复）
- 4.0 的早训练接口（已接入）

### 4.2 v3.1 双 outstanding 未启用（dc2/mem3）
- 在启用 dual outstanding 后的多轮尝试中，Linux digftest 出现 difftest mismatch（`Both Error(Code:0x700)`，并在 `pc=0x779a49a8` 附近失败），见：  
  - `IP/myCPU/logs_v48/v48_linux_final_dft.log`
- 为了保证本轮交付“必过”，最终选择关闭 dual outstanding（最终 PERF 中 `D$ AXI OS: N/A (dual-OS not in tree; dc_os_* absent)`）。

---

## 5. 下一步（可验证的下一刀）

1. **修复 dual outstanding difftest mismatch**  
   重点围绕 `dc2/mem3` 路径与对齐的 `MSHR / refill / rd/rdata demux` 时序语义；同时保持本轮“安全前端设置”（冷启动 + V4.4 时序刀）不变。
2. **重新评估 JTC P0 选择条件**  
   当前强制关闭 `JIRL_TC_P0_ENABLE`，因此 JTC 命中率虽很低（PERF 里 JTC hit_rate~0.09%），但 redirect 的正确性受控；下一步应在“dual OS 也通过”的前提下，把 P0 selection 的正确性边界逐步放开。
3. **访存子系统进一步定位瓶颈**  
   本轮瓶颈集中在 `D$ MSHR busy` 与 `RS_MEM src stall`，下一步可优先针对：
   - store 命中率（store hit_rate~73.2%）
   - MSHR 并发 refill 对 RS_MEM 的影响

---

## 6. 本轮主要改动落点（便于维护注释）

- 冷启动（3.2 合入）：  
  - [`IP/myCPU/frontend/ras.v`](IP/myCPU/frontend/ras.v)  
  - [`IP/myCPU/frontend/tage.v`](IP/myCPU/frontend/tage.v)  
  - [`IP/myCPU/frontend/ubtb.v`](IP/myCPU/frontend/ubtb.v)  
  - [`IP/myCPU/frontend/inst_buffer.v`](IP/myCPU/frontend/inst_buffer.v)  
  - [`IP/myCPU/frontend/ifu.v`](IP/myCPU/frontend/ifu.v)  
  - [`IP/myCPU/frontend/bpu.v`](IP/myCPU/frontend/bpu.v)  
  - [`IP/myCPU/frontend/ftq.v`](IP/myCPU/frontend/ftq.v)  
  - [`IP/myCPU/mycpu.h`](IP/myCPU/mycpu.h)（`GHR_LEN=112`）  
  - [`IP/myCPU/mycpu_top.v`](IP/myCPU/mycpu_top.v)（增量接线）

- 4.0：JTC / early uBTB train：  
  - [`IP/myCPU/frontend/bpu.v`](IP/myCPU/frontend/bpu.v)  
  - [`IP/myCPU/frontend/ifu.v`](IP/myCPU/frontend/ifu.v)  
  - [`IP/myCPU/frontend/ubtb.v`](IP/myCPU/frontend/ubtb.v)  
  - [`IP/myCPU/frontend/ftq.v`](IP/myCPU/frontend/ftq.v)  
  - [`IP/myCPU/mycpu_top.v`](IP/myCPU/mycpu_top.v)

---  

## 7. 怎么测（复现）

```bash
export CHIPLAB_HOME=/home/dogandlamb/chiplab
export PATH=$CHIPLAB_HOME/toolchains/loongson-gnu-toolchain-8.3-x86_64-loongarch32r-linux-gnusf-v2.0/bin:$CHIPLAB_HOME/toolchains/nemu:$PATH
cd $CHIPLAB_HOME/sims/verilator/run_prog

sed -i 's/^BUS_DELAY_RANDOM_SEED=.*/BUS_DELAY_RANDOM_SEED=5570815/' Makefile_run
make verilator && make testbench

# lab19
./configure.sh --run func/func_lab19
make simulation_run_prog

# Linux digftest（本轮主性能口径；主性能表用 digftest-primary）
./configure.sh --run linux --disable-simu-trace --output-uart-info --disable-read-miss
make simulation_run_prog
```

