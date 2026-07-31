# myCPUV4.9：功能合入 + 提 IPC + 70MHz — 性能仿真报告

> 日期：2026-07-28  
> **基线报告/CPU**：[`myCPUV4.8_四变体合入提IPC.md`](myCPUV4.8_四变体合入提IPC.md)（Linux digftest IPC **0.798378**；JTC P0=0；单 outstanding；当时 D$ 为 V4.8 raw CWF-lite）  
> 本轮改动：合入 `myCPU_修正bug` 功能包；修正并留树 **safe-CWF**；扩展 PERF / 优化-仿真表与探针程序；试 dual-OS / JTC P0 / 瘦身未留；STA 目标 70MHz  
> 负载：`func_lab19` digftest；Linux digftest（到 `/ #`）  
> seed：`5570815`（`BUS_DELAY_RANDOM_SEED`）  
> 原始 PERF 日志：`IP/myCPU/logs_v49/`（ship：`cwf_lab19/`）  
> PERF 口径：仿真结束 `myCPU PERF (sim)`（`-DSIMU` → `SYNTHESIS`；Vivado 勿定义）  
> 日常探针：[`优化-仿真表.md`](../优化-仿真表.md)  
> **IPC 主对比源**：**Linux digftest**（与 V4.8 ship 同口径；勿与软门混比）

---

## 0. 结论摘要

| 门控 | 结果 |
|------|------|
| **lab19 digftest** | **PASS**（`END by Syscall`）；IPC≈0.2147 |
| **Linux digftest** | **PASS**（到 `/ #`，无 mismatch） |
| **Linux digftest Commit IPC（ship）** | **0.734463**（相对 V4.8 **−8.0%**；相对本轮「关 CWF」中间态 **+0.1%**） |
| **70MHz STA** | **未过**：WNS=**−1.945 ns**，TNS≈−4463（`D:\chiplab_v5_55mhz\out\70mhz_v49`） |

**一句话：** 修正bug 功能包合入后 digftest 全绿；相对 V4.8 的主代价是关 raw CWF 后的 miss 延迟（Branch MPKI / 周期膨胀）；safe-CWF 正确性过门但 **几乎不回收 Linux IPC**；dual-OS / JTC P0 仍未留树；70MHz 时序失败。

---

## 0.1 变体合入盘点（相对用户列出的几棵树）

| 来源 | 留树？ | 说明 |
|------|--------|------|
| `myCPU_修正bug` | ✅ 功能包 | ALE/Index cacop、ALE>TLBR、`false_taken`、CSR 14-bit+掩码、STQ16、cacop IPE 等 |
| `3.2部分冷启动…`（后端顶层配 3.4） | ✅（经 V4.8） | FTQ-indexed RAS/TAGE checkpoint、predec RET、`GHR_LEN=112`、IB |
| `3.1保持inst_buffer,修改jirl` | ✅（经 V4.8） | IB 保留；jirl 相关前端改动 |
| `4.0时序+jirl` | ✅ 结构 / ❌ P0 | JTC + early uBTB train 已接线；`JIRL_TC_P0_ENABLE=0` |
| `myCPU_v3.1_双outstanding` | ❌ | 试过，Linux digftest `0x700`；回退 `L1_NMSHR=2`、无 dc2/mem3 |

---

## 1. 本轮改动

| 项 | 留树 | 说明 |
|----|------|------|
| ALE 挡 MMU + Index cacop 不翻译 | ✅ | `lsu.v` `a_no_trans` |
| ALE > TLBR | ✅ | `exception_Decoder.v` |
| IFU `false_taken` fallthrough | ✅ | 保留 V4.8 RAS/ret |
| cacop si12 + code→LSU + IPE@PLV3 | ✅ | decode / rename / dispatch / rs_mem |
| CSR 14-bit + 写掩码 | ✅ | TLBIDX / TLBRENTRY / DMW |
| `STQ_DEPTH=16` + commit→SB 延迟释放 | ✅ | |
| **safe-CWF**（无 store-merge 才 beat0 早回；早回用 `axi_ret_data`；否则 beat1+`refill_line_merged`） | ✅ | 修 raw CWF 漏 merge → lab19 INE@`0xe0000000`；修错误的 beat0/`refill_line_merged` 布局 |
| MSHR occupancy / miss lat / L2 / STQ / SB / CWF probe PERF | ✅ | `mycpu_top.v` dump |
| 优化-仿真表 + `perf_*` 探针 | ✅ | `doc/优化-仿真表.md`；`perf_front_stream` / `branch_mix` / `dcache_miss` / `mshr_burst` |
| dual-OS（OS=2, MSHR=4） | ❌ | Linux digftest `Both Error(Code:0x700)`；已回退 |
| JTC P0 | ❌ | 未开（V4.8 已知风险） |
| TAGE_UPD 32→16 / L1 扩容 | ❌ | 未单独绿门 |
| 抓手 0 / 3 | ✅ | CAM 异常门控；I$ hit 旁路切除（V4.4 线） |

### 探索未留树 / 曾试路径

| 探索 | 证据 | 处置 |
|------|------|------|
| dual-OS | `logs_v49/v49_linux_dualos_dft.log` 等 | 回退单 outstanding |
| raw CWF（无条件 beat0 + raw AXI） | lab19：`ecode` DUT INE vs REF SYS，PC=`0xe0000000` | 弃用 |
| CWF + 无条件 byte overlay | `logs_v49/examples_cwf_merge/` lab15/lab19 红 | 弃用 |
| 早期 safe-CWF（门控对、但 beat0 误用 `refill_line_merged`） | 曾 Linux digftest 早挂 | 修正早回数据源后重验绿 |
| TAGE Q=16 | lab19 绿、未单独 Linux | 保持 32 |

---

## 2. 功能结果

| 负载 | 判据 | 证据 |
|------|------|------|
| lab19（ship / safe-CWF） | `Difftest enabled` + `END by Syscall` | `logs_v49/cwf_lab19/lab19.log`；retire=122875，IPC=0.214698 |
| `c_prg/memcmp` digftest | 无 mismatch + PERF | `logs_v49/cwf_lab19/memcmp.log`；IPC≈0.3285 |
| Linux digftest（ship） | `/ #` + 无 mismatch | `logs_v49/cwf_lab19/linux_digftest.log` |
| Linux digftest（关 CWF 中间态） | `/ #` + 无 mismatch | `logs_v49/v49_linux_bugfix_dft.log`；IPC=0.733681 |

---

## 3. 性能（Linux digftest，primary）

| 指标 | V4.8 ship | V4.9 关 CWF | **V4.9 ship（safe-CWF）** | Δ vs V4.8 |
|------|-----------|-------------|---------------------------|-----------|
| Commit IPC | 0.798378 | 0.733681 | **0.734463** | **−8.0%** |
| Commit dual（of cmt） | 36.59% | 36.26% | 36.26% | ≈ |
| Dispatch dual | 77.71% | 76.00% | 76.00% | − |
| BPU all accuracy | 98.43% | 95.72% | 95.73% | − |
| Branch MPKI all / cond | 3.251 / 2.968 | 8.853 / 3.157 | **8.831 / 3.168** | all↑ |
| I$ hit | 96.33% | 95.91% | 95.91% | − |
| D$ hit 总 / ld / st | 83.47 / 95.95 / 73.17 | 83.87 / 95.96 / 73.63 | 83.87 / 95.95 / 73.64 | ≈ |
| D$ ld miss lat avg | （当时无此行） | — | **24.73**（max 264） | — |
| D$ CWF probe | — | — | early_opp≈469k；beat1_hi≈188k；beat1_lo≈7.8k | 早回机会多，高半字/merge 仍多等 beat1 |
| D$ MSHR busy | 24.26% | 22.42% | 22.45% | − |
| MSHR occ max/cap | — | — | **2 / 2** | 顶满；dual-OS 仍相关 |
| LSU DC wait | 13.97% | 13.00% | 12.99% | − |
| RS_MEM src stall | 25.66% | 23.81% | 23.70% | − |
| ROB full | 11.93% | 10.20% | 10.16% | − |
| IB empty | 20.10% | 25.24% | 25.32% | ↑ |
| Flush / predec_redirect | 0.64% / 0.47% | 1.00% / 1.53% | 0.99% / 1.53% | ↑ |
| JTC hit_rate | 0.09% | 0.09% | 0.09% | P0 关 |

### 解读要点

1. **相对 V4.8 −8% IPC**：周期从 ~153M → ~167M，retire 接近；主因是 miss 路径变慢后流水线气泡↑，并连带 Branch MPKI all、IB empty、redirect 变差（与「关 raw CWF」一致）。  
2. **safe-CWF vs 关 CWF**：IPC 0.733681 → 0.734463（**+0.11%**），统计噪声量级；`early_opp` 很大，但 Linux 负载大量高半字 / 有 merge 的 miss 仍走 beat1，**回收不明显**。  
3. **MSHR max=cap=2**：dual-OS 仍是下一刀候选，但必须先修 demux/`0x700`。  
4. **lab19 IPC≈0.215**：与 V4.8/关 CWF 同量级，短测不反映 Linux 访存延迟差。

---

## 4. Cache / 瘦身 / 探针基建

- **扩 L1/L2：** 未扩；store hit≈73.6% 仍有空间，优先 dual-OS/CWF 正确性。  
- **瘦身：** TAGE update Q PERF 仍偏低占用；保持 32。  
- **PERF 新增：** I$/D$ miss lat、CWF probe、L2 行读、LSU STQ、SB nonempty/drain、MSHR occupancy。  
- **手册：** [`优化-仿真表.md`](../优化-仿真表.md)（A–G 面 ↔ 分钟级仿真；Linux 仅候选合入）。

---

## 5. 频率（70MHz）

| 项 | 值 |
|----|-----|
| 工程 | `D:\chiplab_v5_55mhz` |
| 输出 | `out/70mhz_v49/`（`timing_summary.rpt` / `soc_top_routed.dcp`） |
| 结果 | **WNS=−1.945 ns**，**TNS≈−4462.8**；`write_bitstream` 仍 Complete（时序违例） |
| 日志 | `IP/myCPU/logs_v49/v49_vivado_70mhz.log` 末行 `V4.9 70MHz WNS=-1.945 ...` |

抓手 0/3 仍在树。下一刀按 STA 路径族（RF / rs_alu select / D$）一次一刀。

---

## 6. 下一步

1. **dual-OS 正确性**：addr-match demux + BYP 豁免；lab19 + `perf_mshr_burst` + Linux digftest 绿后再看 IPC。  
2. **CWF 收益**：当前 safe 门控过严或高半字占比高 → 评估「早回数据强制叠 merge、且禁止 beat0 与同拍 merge 竞态」的更强安全式；用 `perf_dcache_miss` 看 miss lat / IPC Δ。  
3. **70MHz**：按 WNS 路径收敛；可先稳住 60/65。  
4. **JTC P0**：仅在 dual-OS 绿后、且 `perf_branch_mix` 绿门再开。

---

## 7. 怎么测（复现）

```bash
export CHIPLAB_HOME=~/chiplab
export PATH=$CHIPLAB_HOME/toolchains/loongson-gnu-toolchain-8.3-x86_64-loongarch32r-linux-gnusf-v2.0/bin:$CHIPLAB_HOME/toolchains/nemu:$PATH
cd $CHIPLAB_HOME/sims/verilator/run_prog
sed -i 's/^BUS_DELAY_RANDOM_SEED=.*/BUS_DELAY_RANDOM_SEED=5570815/' Makefile_run
make verilator && make testbench

# lab19
./configure.sh --run func/func_lab19 --output-uart-info
make simulation_run_prog

# Linux digftest（本轮 primary IPC）
./configure.sh --run linux --disable-simu-trace --output-uart-info
make simulation_run_prog
# 成功：UART `/ #`，无 Both Error；摘 Commit IPC
```

CWF 专项：`c_prg/memcmp` + `perf_dcache_miss`（见优化-仿真表 E′）。

---

## 附录：关键日志

| 内容 | 路径 |
|------|------|
| lab19 ship | `IP/myCPU/logs_v49/cwf_lab19/lab19.log` |
| memcmp digftest | `IP/myCPU/logs_v49/cwf_lab19/memcmp.log` |
| Linux digftest ship | `IP/myCPU/logs_v49/cwf_lab19/linux_digftest.log` |
| Linux digftest 关 CWF | `IP/myCPU/logs_v49/v49_linux_bugfix_dft.log` |
| dual-OS fail | `IP/myCPU/logs_v49/v49_linux_dualos_dft.log` |
| 70MHz STA | `IP/myCPU/logs_v49/v49_vivado_70mhz.log` → `D:\chiplab_v5_55mhz\out\70mhz_v49\` |

### Linux digftest PERF 摘录（ship）

```
Commit IPC:         retire=122384311  cycles=166631091  IPC=0.734463
Commit dual-issue:  dual_rate=36.26%  (of commit cycles)
Dispatch dual:      dual_rate=76.00%  dual_ALU=25208006
BPU all-branch:     accuracy=95.73%
Branch MPKI:        all=8.831  cond=3.168
ICache (cached):    hit_rate=95.91%
DCache (cached):    hit_rate=83.87%  (ld 95.95% / st 73.64%)
D$ ld miss lat:     n=657126  avg=24.73  max=264
D$ CWF probe:       early_opp=469153  beat1_lo=7751  beat1_hi=187973
D$ MSHR busy:       22.45%   occupancy max=2/2
RS_MEM src stall:   23.70%
ROB full:           10.16%
```
