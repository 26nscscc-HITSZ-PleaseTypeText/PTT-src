# myCPUV3.4：store路径+后端前递+前端容量+扩大L2cache+55MHz

> 日期：2026-07-22  
> **基线报告/CPU**：`myCPUV2.3：资源缩减 + 50MHz 提频.md`（Linux 软门 IPC **0.569927**，50MHz WNS≥0）  
> 本轮改动：SB→D$ 整行合并、D$ hit-bypass + DC early2、TAGE 双口 + FTB 2048、L2 128KB、PERF u64 修复；55MHz 全实现收敛；60MHz 冲刺未交付  
> 负载：`func_lab19` digftest；Linux 软门 / digftest（分列）  
> seed：`5570815`（`BUS_DELAY_RANDOM_SEED`）  
> 原始 PERF 日志：`IP/myCPU/logs_v34/`、`/tmp/v34_*.log`  
> 时序报告：`/mnt/c/Users/h546d/Desktop/chiplab/reports_impl55/`（phys_opt 后）  
> PERF 口径：仿真结束 `myCPU PERF (sim)`（`mycpu_top.v` 中 ``ifdef SYNTHESIS`；仿真经 `-DSIMU`）

---

## 0. 结论摘要


| 门控                 | 结果                                                                 |
| ------------------ | ------------------------------------------------------------------ |
| **lab19 digftest** | **PASS**（`END by Syscall`）；IPC = **0.215413**                      |
| **Linux 软门**       | **PASS**（`/ #`）；IPC = **0.740188**（基线 0.569927，**+29.9%**）         |
| **Linux digftest** | **PASS**（无 mismatch，至 `/ #`）；IPC = **0.740188**（与软门 bit-identical） |
| **频率**             | **交付 55MHz**：全实现 + phys_opt **WNS=+0.023ns**；60MHz 冲刺未收敛（见 §3.1）   |


**一句话：** 功能门全绿；Linux 软门 IPC **0.57→0.74**（远超 ≥0.59 目标），主收益来自 hit-bypass/early2 与 L2 128KB；TAGE 双口把训练 overflow 打到 **0**；频率保底 **55MHz 已收敛**，60MHz 因关键路径（D$ hit→RS）与正确性约束未交付。

---

## 1. 本轮改动


| 项                        | 留树 / 回退 | 说明                                                                           |
| ------------------------ | ------- | ---------------------------------------------------------------------------- |
| P0 PERF u64→real         | ✅       | Verilator ITORD 32bit 截断致 FTQ/RAS 负均值；分段 `$itor` 修复（FTQ avg −7.67→**11.93**） |
| P1 SB→D$ 行合并             | ✅       | 泄流同行聚合 256b+32B strb；`st_merge` data-only 卫生化                                |
| P1 push_merge / SB16     | ❌       | 入队旁路合并与 SB16 导致 Linux panic，已回退                                              |
| P2 `LSU_DC_HIT_BYPASS`   | ✅       | D$ 命中当拍写回（跳 hold）                                                            |
| P2 DC 级 early2→三 RS      | ✅       | 命中限定早唤醒；lab19 IPC 0.2147→0.2154                                              |
| P3 TAGE 2R+1W            | ✅       | 查询∥训练；Linux/lab19 **overflow=0**                                             |
| P3 FTB NSET 2048         | ✅       | hit 37%→**40.7%**                                                            |
| P3 p1_target_diff 收紧     | ✅       | 仅双方 taken 比目标                                                                |
| P3 IB FWFT / 3.0 banking | ❌ 跳过    | IPC 已超额；未测 banking（按计划「回退即弃」）                                                |
| P4 L2 32KB→128KB         | ✅       | NSET 512→2048；IPC 再抬至 **0.74**                                               |
| P4 LOAD_MERGE            | ❌ 未上    | MSHR busy 仍~23%，但 IPC 已远超目标                                                  |
| P5 55MHz 全实现             | ✅       | 初布线 WNS=−0.099 → `phys_opt AggressiveExplore` → **+0.023**                   |
| P5 60MHz 冲刺              | ❌       | 见 §3.1；hit-bypass 关 + early2 打拍曾过 OOC，但 Linux digftest 错载，已回退                |
| 值预测                      | ❌ 跳过    | IPC 已 >0.59，按计划不做                                                            |


### 外部优化借鉴（`IP/优化/`*）

**采纳：** hit-bypass、early2 消费、TAGE 双口、p1_target_diff、st_merge data-only。  
**不采纳：** 投机 SB/砍 STQ、ROB64/RS 加大、`rob_data` 拆分、IB 3.0（未测且目标已达）。

---

## 2. 功能结果


| 负载             | 判据                 | 证据                                                    |
| -------------- | ------------------ | ----------------------------------------------------- |
| lab19          | `END by Syscall`   | `logs_v34/v34_final_lab19.log`（终验 IPC 0.215413）       |
| Linux 软门       | UART `/ #`，干净 PERF | `logs_v34/v34_p4_linux.log`；终验见 `v34_final_linux.log` |
| Linux digftest | 无 mismatch + `/ #` | `logs_v34/v34_ldiff.log`                              |


---

## 3. 性能对比（相对基线 V2.3，Linux 软门）


| 指标                  | 基线 V2.3     | V3.4                  | Δ                  |
| ------------------- | ----------- | --------------------- | ------------------ |
| Commit IPC          | 0.569927    | **0.740188**          | **+29.9%**         |
| Cycles（至 `/ #`）     | 219.1M      | 164.6M                | −24.9%             |
| Commit dual（of cmt） | —           | 36.08%                |                    |
| Dispatch dual       | —           | 77.05%                |                    |
| FTB hit             | 37%         | **40.68%**            | +3.7pt             |
| TAGE overflow       | ~159k       | **0**                 |                    |
| D$ store hit        | 78.35%      | 73.24%                | −5pt（访问结构变；IPC 仍升） |
| D$ MSHR busy        | 19.27%      | 22.80%                |                    |
| 全实现频率               | 50MHz WNS≥0 | **55MHz WNS=+0.023**  |                    |
| BRAM Tile           | ~54/365     | **79.5/365（21.8%）**   | L2×4 + FTB 加倍      |
| Slice LUT           | —           | 84696 / 133800（63.3%） |                    |


### 阶段爬坡（Linux 软门 IPC）


| 阶段                   | IPC          | 相对基线       |
| -------------------- | ------------ | ---------- |
| P0 探针修复              | 0.569927     | 0%         |
| P1 行合并               | 0.573726     | +0.7%      |
| P2 hit-bypass+early2 | 0.612124     | +7.4%      |
| P3 TAGE双口+FTB2048    | 0.630610     | +10.6%     |
| P4 L2 128KB          | **0.740188** | **+29.9%** |


### 3.1 频率专节（P5）


| 步骤                            | 结果                                                                                                             |
| ----------------------------- | -------------------------------------------------------------------------------------------------------------- |
| OOC 55MHz（18.18ns）            | WNS=**+1.150ns**                                                                                               |
| OOC 60MHz（16.67ns，无裁剪）        | WNS=**−0.360ns**                                                                                               |
| 全实现 55MHz                     | 初布线 WNS=**−0.099**（1 endpoint：`dcache/req_paddr`→`rs_mem/s0_ready`）                                            |
| `impl55_closure.tcl` phys_opt | WNS=**+0.023** / CPU_CLK_WNS=**+0.023** → **交付**                                                               |
| 60MHz 裁剪尝试                    | `LSU_DC_HIT_BYPASS=0` + `LSU_EARLY2_PIPE=1`：OOC WNS=+0.087；全实现 WNS=**−0.289**；**Linux digftest mismatch（已回退）** |


关键路径家族（55MHz 闭合后仍最紧）：D$ tag/hit → LSU `mem_wb_valid`（hit-bypass）→ RS ready/data，约 17.7ns，**~78% 布线**。  
60MHz 需再砍 ~1.5ns；打拍切断该路径会牺牲命中写回同拍语义，本轮安全裁剪引入功能回归，故 **最高可收敛频率 = 55MHz**。

脚本：`impl_55mhz.tcl` / `impl55_closure.tcl`；`impl_60mhz.tcl`（冲刺留档）。  
DCP：`soc_top_routed_physopt.dcp`（55MHz 闭合版）。

---

## 4. 解读

- **优化：** 后端前递（命中旁路 + early2）砍掉 load 命中路径多余拍；TAGE 双口消灭训练丢弃；L2×4 显著减 AXI 往返；SB 泄流行合并降部分 store miss 往返。  
- **提升：** IPC 0.57→0.74（+29.9%）；TAGE overflow 清零；FTB hit +3.7pt；频率 50→**55MHz** 收敛。  
- **不足：** store hit 未达 90%（行合并对冷写分配帮助有限）；60MHz 未交付（布线主导 + 裁剪正确性风险）；IB 3.0 / LOAD_MERGE / 值预测未合。  
- **下一步：** （1）若再冲 60MHz，优先对 D$→RS **数据通路**做经验证的打拍（勿只砍 early2）；每步 lab19+Linux digftest；（2）store 分配/写合并策略；（3）可选 FTB 双读口压 update overflow。

---

## 5. 怎么测（复现）

```bash
export CHIPLAB_HOME=/home/dogandlamb/chiplab
export PATH=$CHIPLAB_HOME/toolchains/loongson-gnu-toolchain-8.3-x86_64-loongarch32r-linux-gnusf-v2.0/bin:$PATH
# lab19: run_prog2；Linux 软门: run_prog；Linux digftest: run_prog3
cd $CHIPLAB_HOME/sims/verilator/run_prog
make verilator && make testbench && make simulation_run_prog
```

OOC：`IP/myCPU/scripts/run_ooc_synth.sh 18.18`（55M）/ `16.67`（60M）。  
全实现（Windows Desktop chiplab）：`impl_55mhz.tcl` → `impl55_closure.tcl`。

关键宏（交付默认）：

```text
LSU_DC_HIT_BYPASS = 1
LSU_EARLY2_PIPE   = 0
L2_NSET           = 2048   // 128KB
FTB_NSET          = 2048
```

---

## 附录：PERF 摘录（P4 Linux 软门，交付 IPC）

```
Commit IPC:         retire=121821876  cycles=164582321  IPC=0.740188
Commit dual-issue:  dual_cyc=32300272  any_cyc=89521604  dual_rate=36.08%
Dispatch dual:      dual_cyc=63371047  any_cyc=82245969  dual_rate=77.05%
FTB (P1 response):  hit_rate=40.68%
DCache store:       hit_rate=73.24%
D$ MSHR busy:       22.80%
TAGE update queue:  max=1/32  overflow=0
```

### 阶段日志索引


| 阶段            | lab19                        | Linux 软门 / 其它                                       |
| ------------- | ---------------------------- | --------------------------------------------------- |
| P0            | `logs_v34/v34_p0_lab19.log`  | `v34_p0_linux.log`                                  |
| P1            | `v34_p1_*`                   | IPC 0.5737                                          |
| P2            | `v34_p2_*`                   | IPC 0.6121                                          |
| P3            | `v34_p3_*`                   | IPC 0.6306                                          |
| P4            | `v34_p4_*` / `v34_ldiff.log` | IPC **0.7402**                                      |
| P5 OOC        | —                            | `v34_ooc55` +1.15；`v34_ooc60` −0.36                 |
| P5 55M 实现     | —                            | `impl55.log` / `impl55_closure` → WNS +0.023        |
| P5 60M 裁剪（回退） | `v34_p5_lab19_cut`           | soft hang + `v34_p5_ldiff_cut_FAIL`；impl WNS −0.289 |
| 终验            | `v34_final_lab19.log`        | `v34_final_linux.log`                               |


