# myCPUV2.3：资源缩减 + 50MHz 提频— 性能仿真报告

> 日期：2026-07-21  
> **基线报告/CPU**：`myCPUV2.2_LSU_DCache_合入报告.md`（V2.2：Linux 软门 IPC **0.5699**，40MHz WNS +0.851，LUT 76.35% / Slice 91.27%）  
> 本轮改动：L1 tag/ROB/RAS/FTQ 存储 LUTRAM 化 + ROB 提交预译码 + TLB tlbsrch 专用查找口，PLL 40→50MHz 时序收敛；IPC 六项 A/B 均无收益全部回退；全量 36 文件注释审计统一（含 tage.v 乱码重写）  
> 负载：`func_lab19` digftest；Linux 软门 / digftest（分列）  
> seed：`5570815`（`BUS_DELAY_RANDOM_SEED`）  
> 原始 PERF 日志：`logs_v2.3/p3_final_lab19.log`、`logs_v2.3/p3_final_linux_soft.log`、`logs_v2.3/p4_final_linux_dft.log`、注释审计后终验 `logs_v2.3/p4_comment_audit_lab19.log` / `logs_v2.3/p4_final_linux_soft.log`；Vivado 报告 `reports_impl50/`（50MHz 全实现）  
> PERF 口径：仿真结束 `myCPU PERF (sim)`（`mycpu_top.v` 中 ``ifdef SYNTHESIS`；仿真经 `-DSIMU` 在 `mycpu.h` 定义 `SYNTHESIS`，Vivado 综合勿定义）

---

## 0. 结论摘要


| 门控                   | 结果                                                                                                       |
| -------------------- | -------------------------------------------------------------------------------------------------------- |
| **lab19 digftest**   | PASS（`END by Syscall`）；IPC = 0.2147（与 V2.2 逐位一致）                                                         |
| **Linux 软门**         | PASS（`/ #` 干净停仿）；IPC = **0.5699**（retire=124,881,985 / cycles=219,119,196，与 V2.2 逐位一致）                   |
| **Linux digftest**   | PASS（`Difftest enabled` 全程比对，无 mismatch，到 `/ #` 干净停仿）；IPC = 0.5699（与软门同 seed 逐位一致——NEMU 比对是纯观察者，不扰动 DUT） |
| **Vivado 50MHz 全实现** | **收敛**：cpu_clk WNS **+0.553ns**（P&R 直出即收敛）；全设计 WNS +0.038ns（sys_clk CDC 路径经一轮 post-route phys_opt 收平）    |


**一句话：** 功能三门全绿、行为逐位不变的前提下，CPU 频率 40→**50MHz**（cpu_clk 还余 0.55ns 裕量，折算上限 ≈51.4MHz），LUT −28.7%、FF −40.7%、Slice 占用 91.3%→68.9%；IPC 维持 0.5699（六项加深/精化 A/B 全部无收益，瓶颈确认为 D$ store miss 串行延迟，非队列容量）。

---

## 1. 本轮改动

### 1.1 留树改动（全部行为不变，difftest 逐位验证）


| 项                                                        | 留树  | 说明                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| -------------------------------------------------------- | --- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| I$/D$ tag_arr 拆 per-way 一维数组 + `ram_style="distributed"` | ✅   | 二维数组推断失败落成 FF+巨型 mux 是资源第一大户；改后 tag 走 LUTRAM，D$ −18.3k LUT / I$ −10.5k FF                                                                                                                                                                                                                                                                                                                                                                                                                          |
| ROB 提交预译码 + `inst[]` 阵列 SYNTHESIS 门控                     | ✅   | 提交侧仅存 dispatch 预译码的几个 bit；32b `inst[]` 只在仿真（difftest/debug）保留，同时把 `inst[head]` 读 mux 从关键路径起点摘除                                                                                                                                                                                                                                                                                                                                                                                                     |
| ROB 静态字段（pc/csr_num/tlb_op/cacop_code 等）奇偶双体 LUTRAM      | ✅   | 写一次读一次字段不必占 FF；result 类多写口字段保持 FF                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| RAS 双栈 / FTQ 载荷（blk_pc/target/meta）LUTRAM 化              | ✅   | 深度不变；RAS −2.3k LUT、FTQ −2.6k LUT                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| FTB `uq_target` RAMB18（利用率 5%）→ distributed              | ✅   | 释放 1 块 RAMB18，FTB LUT +0.7k（净赚）                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| TLB `tlbsrch` 专用查找口（srch 口）                              | ✅   | V2.3 提频主刀：tlbsrch 输入直接取 CSR.TLBEHI/ASID，不再挪用 s0 口；commit 逻辑与「TLB 查表→翻译→取指」关键路径完全解耦；l1_tlb 的 dis_refill 屏蔽随之删除。语义/时序拍数完全不变（found/index 每拍常备，提交拍采样）                                                                                                                                                                                                                                                                                                                                                  |
| PLL cpu_clk 40→50MHz（CLKOUT1 50.000）                     | ✅   | 全实现收敛（见 §3.3）                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 死信号清扫 + `cmt_csr_we/cmt_csr_wnum` 声明前移（消 Vivado 隐式声明告警）  | ✅   | 无逻辑变化                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| 全量 `.v/.h` 注释审计（36 文件）                                   | ✅   | 两路并行审计后逐条修复：tage.v 全文乱码重写（GBK 双重编码损坏，约 40 处）；9 个文件的「TODO: 实现…」脚手架改为「设计说明（已实现）」并修正与代码矛盾处（dispatch 全收或全停/双 ALU 路由、rs_mem 仅队头发射、rob 平铺存储、ctrl 组合 flush、regfile 单写口等旧描述）；改正事实性错误注释 14 处（ifu 预译码范围、ftb 借读口、store_buffer ll 等待、dcache ld_cancel 契约、exception_Decoder 优先级摘要、alu exe_pc 占位、fu_alu/rs_* early 唤醒"二期"、imm_generator jirl 目标、op_dec BL/JIRL 乱句、csr handler 五级流水残留称谓等）；inst_buffer 补标准文件头；删除 bpu.v 标注"调通后删除"的临时调试 $display 块（仿真专用，零逻辑影响）。改后 rebuild + lab19 digftest 快验 PASS，IPC=0.214678 逐位一致 |


### 1.2 探索未留树（A/B 结论均为 IPC 持平，已全部回退，无 .bak 残留）


| 探索项                                           | 软门结果                                    | 回退原因                                              |
| --------------------------------------------- | --------------------------------------- | ------------------------------------------------- |
| `L1_NMSHR` 2→4                                | IPC 0.5699 持平，MSHR busy 仍 19.27%        | AXI 读通道单 owner 串行化才是瓶颈，MSHR 数不增并行度，资源反增           |
| `RS_MEM_SIZE` 4→8（联合 FTB/TAGE_UPDATE_Q 32→64） | IPC 0.5700 持平                           | 反压只是从 RS_MEM 转移成 ROB full（9%→22%），真瓶颈在 D$ miss 延迟 |
| FTB/TAGE update queue 32→64                   | overflow 31万→26万 / 16万→14万，准确率与 IPC 均持平 | 溢出集中在查询独占读口的突发段，加深只延后丢弃                           |
| ROB 32→48/64                                  | 未实施（评审否决）                               | 早期实验已证明窗口加大在 miss 串行瓶颈下无收益，且 +5.5k FF 侵蚀时序裕量      |
| UC park 精化（parked UC 期间放行不冲突 cached 访问）       | IPC/周期数逐位不变                             | 探针实证：兜底旁路路径在 Linux 启动全程 0 次触发，park 场景本身极稀         |


---

## 2. 功能结果


| 负载                | 判据                                                 | 证据（日志关键字 / 路径）                                                      |
| ----------------- | -------------------------------------------------- | ------------------------------------------------------------------- |
| lab19             | `END by Syscall`，difftest 全程无 mismatch             | `logs_v23/p3_final_lab19.log`（IPC=0.214678，572,370 cycles）          |
| Linux 软门          | UART `/ #`，干净停仿打 PERF                              | `logs_v23/p3_final_linux_soft.log`（IPC=0.569927）                    |
| Linux digftest    | `Difftest enabled` 后全程无 mismatch，到 `/ #` 停仿，exit=0 | `logs_v23/p4_final_linux_dft.log`（IPC=0.569927，219,119,196 cycles）  |
| lab19（注释审计后终验）    | `END by Syscall`，无 mismatch，IPC 逐位一致               | `logs_v23/p4_comment_audit_lab19.log`（IPC=0.214678）                 |
| Linux 软门（注释审计后终验） | UART `/ #`，IPC/周期数与审计前逐位一致                         | `logs_v23/p4_final_linux_soft.log`（IPC=0.569927，219,119,196 cycles） |


---

## 3. 性能对比（相对基线 V2.2）

### 3.1 Linux 软门主对比（同 seed 5570815、同 `/ #` 停仿；V2.3 与 V2.2 逐位一致）

本轮定位是资源/频率轮：全部留树改动都以「行为不变」为红线，软门 IPC/周期数与 V2.2 完全一致（逐位），不存在性能回归。


| 指标                                  | V2.2 基线           | V2.3 本轮  | Δ       |
| ----------------------------------- | ----------------- | -------- | ------- |
| Commit IPC                          | 0.569927          | 0.569927 | 0（逐位一致） |
| Commit dual-issue（of commit cycles） | 35.24%            | 35.24%   | 0       |
| Dispatch dual / dual_ALU            | 75.66% / 27.68M   | 同左       | 0       |
| BPU all / cond                      | 92.19% / 96.13%   | 同左       | 0       |
| I$ hit                              | 96.09%            | 同左       | 0       |
| D$ hit（ld / st）                     | 95.92% / 78.35%   | 同左       | 0       |
| D$ MSHR busy                        | 19.27%            | 同左       | 0       |
| ROB full / RS_MEM src stall         | 8.96% / 19.26%    | 同左       | 0       |
| FTB / TAGE update overflow          | 313,039 / 159,006 | 同左       | 0       |


### 3.2 资源（Vivado 全实现，xc7a200t；V2.2@40MHz vs V2.3@50MHz）


| 资源              | V2.2            | V2.3               | Δ                   |
| --------------- | --------------- | ------------------ | ------------------- |
| Slice LUTs      | 102,771（76.35%） | **73,278（54.77%）** | **−29,493（−28.7%）** |
| Slice Registers | 69,723（25.90%）  | **41,335（15.35%）** | **−28,388（−40.7%）** |
| Slice 占用        | 30,714（91.27%）  | **23,040（68.88%）** | **−22.4 个百分点**      |
| Block RAM Tile  | 54.5（14.93%）    | 54（14.79%）         | −0.5                |


CPU 内部主要模块（LUT / FF）：


| 模块              | V2.2            | V2.3               | 主要来源                             |
| --------------- | --------------- | ------------------ | -------------------------------- |
| core_top 合计     | 91,157 / 55,924 | 61,707 / 27,542    | −29.4k LUT / −28.4k FF           |
| dcache          | 28,876 / 13,217 | **10,550 / 2,805** | tag LUTRAM 化（−18.3k LUT）         |
| icache          | 7,379 / 11,787  | **2,558 / 1,269**  | tag LUTRAM 化（−10.5k FF）          |
| rob             | 12,890 / 9,812  | 10,703 / 5,757     | inst[] 门控 + 静态字段 LUTRAM          |
| ftq             | 3,523 / 2,126   | 888 / 740          | 载荷 LUTRAM 化                      |
| ras             | 2,473 / 2,103   | 141 / 54           | 双栈 LUTRAM 化                      |
| bpu（含 ftb/tage） | 4,006 / 3,751   | 2,305 / 1,719      | uq_target 迁出 BRAM（ftb LUT 略增为代价） |


### 3.3 频率（Vivado 全实现）


| 项           | V2.2 @40MHz（25ns）                                                                                                     | V2.3 @50MHz（20ns）                                                                                                    |
| ----------- | --------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------- |
| cpu_clk WNS | +0.851ns（路径 24.15ns，上限 ≈41.4MHz）                                                                                      | **+0.553ns（路径 19.45ns，上限 ≈51.4MHz）**                                                                                 |
| 全设计 WNS     | +0.756ns                                                                                                              | **+0.038ns**（sys_clk AXI-CDC 路径 −0.206，一轮 post-route `phys_opt_design -directive AggressiveExplore` 收平；cpu_clk 无需干预） |
| 关键路径演变      | ROB head→commit ecode→cmt_tlb_op→tlb_manager s0 口 tlbsrch mux→TLB 查表→FTQ→IFU pre_excp→I$ req_paddr（29 级逻辑，Top50 全是此链） | 该链族被 srch 口整体消灭；新最长链为 D$ MSHR→RS_MEM 唤醒 / FTQ→I$ BRAM 使能，均有裕量                                                        |


myCPU 单核 OOC 综合（20ns 约束）演进：V2.2 基线 WNS +1.968 → Phase 1（tag LUTRAM+ROB 预译码）+2.219 → TLB srch 口 +3.888。

---

## 4. 解读（必写）

- **优化：** 本轮两条主线互相成就——(1) 资源线把 tag/ROB/RAS/FTQ 这些「写口少、读口少」的存储从 FF+mux 海改成 LUTRAM，Slice 占用从 91% 压回 69%，布线拥塞大幅缓解；(2) 频率线用「ROB 提交预译码」剪掉关键链起点、用「tlbsrch 专用查找口」剪掉中段的 s0 口 mux+全相联查表，把 29 级逻辑的巨链整族消灭。50MHz 一次全实现直接收敛（cpu_clk 无任何违例），只有 SoC 侧 sys_clk 的一条 AXI-CDC 路径差 0.2ns，phys_opt 一轮收平。
- **提升：** 频率 +25%（40→50MHz），同 IPC 下实际吞吐 +25%；LUT −28.7%、FF −40.7%、Slice −22 个百分点，为后续 ROB/FTB 扩容或 4 宽化留出实体空间。
- **不足：** IPC 线六项 A/B（MSHR4、RS_MEM8、双 Q64、ROB 扩、UC park 精化）全部持平，0.5699→0.587 的软门目标未达成。剖析一致指向：D$ store hit 78.35%（V1.5 是 97.16%）带来的 store miss 串行 MSHR 延迟是当前唯一硬瓶颈，属于 V2.2 dcache 行合并改版的结构性代价，非队列容量可解。另 PERF 探针中 FTQ/RAS 占用均值出现负数，是仿真探针加减计数在冲刷路径上不配平的已知缺陷，不影响硬件行为。
- **下一步：** (1) D$ store miss 路线：store 合并写缓冲（line-merge 前置到 SB→D$ 接口）或 store-allocate 策略回调，把 store hit 拉回 9 成以上，这是 IPC 唯一可验证的大刀；(2) 频率余量变现：cpu_clk 还有 0.55ns，可试 55MHz 全实现摸上限；(3) FTB hit 37% 仍低，BRAM 还剩 85%，可把 FTB 容量加深（如 NSET 1024→2048，仍走 BRAM）；(4) 修 PERF 占用探针的冲刷配平。

---

## 5. 怎么测（复现）

环境：`CHIPLAB_HOME`；seed `**5570815`**。改 RTL 后必须 `make verilator && make testbench`（不能只 `make run`）。换 case 前 `make clean_all` 再 `configure.sh`。


| 负载                  | 配置（典型）                                                                                                        | 成功标志                           | 性能用途                   |
| ------------------- | ------------------------------------------------------------------------------------------------------------- | ------------------------------ | ---------------------- |
| **lab19 硬门**        | `./configure.sh --run func/func_lab19`（digftest 开）                                                            | `END by Syscall` / HIT GOOD    | 功能正确；IPC 仅作短测参考        |
| **Linux 软门（性能主对比）** | `./configure.sh --run linux --disable-simu-trace --disable-trace-comp --disable-read-miss --output-uart-info` | UART `/ #`，干净停仿打 PERF          | **IPC/stall 与基线对比用这个** |
| **Linux digftest**  | 同上但**去掉** `--disable-trace-comp`                                                                              | 无 mismatch + 到 `/ #`（或报告明确写未到） | 正确性；IPC 勿与软门混比         |


```bash
export CHIPLAB_HOME=...
cd $CHIPLAB_HOME/sims/verilator/run_prog
make clean_all
./configure.sh --run <case> ...
make soft_compile
make verilator && make testbench
make simulation_run_prog
```

Vivado 50MHz 全实现（Windows 侧，工程 `fpga/nscscc-team/run_vivado/project/loongson.xpr`）：

```bash
# WSL 驱动（先把 IP/myCPU RTL 同步到 Windows 工程），脚本见 impl_50mhz.tcl：
#   clk_pll CLKOUT1_REQUESTED_OUT_FREQ=50.000 -> reset_target/generate_target/synth_ip
#   -> reset_run synth_1 -> launch_runs synth_1/impl_1 -> 报告输出 reports_impl50/
cmd.exe /c "vivado.bat -mode batch -source impl_50mhz.tcl"
# 若 sys_clk CDC 差零点几 ns：post-route phys_opt（impl50_closure.tcl）
cmd.exe /c "vivado.bat -mode batch -source impl50_closure.tcl"
```

摘数：日志中 `==================== myCPU PERF (sim) ====================` 至结束；完整 log 路径写入头注。

### 注意事项

- 同 seed、同停仿条件（`/ #` / Syscall）才可比 IPC  
- 软门 vs digftest **分列**，主性能表用软门  
- PERF 仅仿真；上板综合不依赖层次探针 / `final` dump  
- 探索项均已干净回退（无散落 `.bak`）；Phase 1 改动前的原文件快照集中存于 `IP/myCPU/.bak_v23/*.p1_pre`

---

## 6. 必测性能指标清单

探针来源：仿真结束 `myCPU PERF (sim)`。V2.3 与 V2.2 逐位一致，下表即两版共同值。

### 吞吐 / 提交


| 指标                       | 值                                                             | 备注                                    |
| ------------------------ | ------------------------------------------------------------- | ------------------------------------- |
| Commit IPC               | 0.569927                                                      | retire=124,881,985 cycles=219,119,196 |
| 双提交率                     | 35.24%（of commit cycles）/ 14.85%（of all）                      | IPC_util 28.50% of 2.0 peak           |
| Dispatch dual / dual_ALU | 75.66% / 27,681,031                                           | 前端供指充足                                |
| Rename/dispatch stall    | turnover_refill=64.78M dispatch_stall=32.84M rob_stall=19.38M |                                       |


### 预测 / 取指


| 指标                         | 值                 | 备注                    |
| -------------------------- | ----------------- | --------------------- |
| BPU all / cond 准确率         | 92.19% / 96.13%   | mispred 2.01M / 0.90M |
| FTB P1 hit                 | 36.96%            | 仍低，下一步加深候选            |
| I$ hit                     | 96.09%            | access=70.36M         |
| FTB / TAGE update overflow | 313,039 / 159,006 | Q64 A/B 证实加深无益        |


### 访存


| 指标                            | 值                            | 备注                |
| ----------------------------- | ---------------------------- | ----------------- |
| D$ hit（总/ld/st）               | 86.08% / 95.92% / **78.35%** | store 侧是主瓶颈       |
| D$ MSHR busy / pend / MWAIT   | 19.27% / 4.07% / 0.07%       | MSHR4 A/B 证实非项数问题 |
| LSU store-order / DC wait     | 0.25% / 12.11%               |                   |
| RS_MEM src / LSU / full stall | 19.26% / 7.96% / 0%          |                   |
| SB full                       | 0.54%                        |                   |


### 队列占用（avg / max / cap / ≥75% / full）


| 队列                | 值                            | 结论            |
| ----------------- | ---------------------------- | ------------- |
| ROB pairs / valid | 5.50/11/11 ・ 9.78/22/22      | full 9%，不砍不扩  |
| RS ALU0/1         | 0.68、0.50 / 4                | 富余            |
| RS MEM            | 1.28 / 4（full 34.9M cyc）     | 扩到 8 已 A/B 证伪 |
| RS MDU            | 0.03 / 2                     | 富余            |
| SB / IB           | 0.35/8 ・ 5.04/16             | 富余            |
| FTQ / RAS / RAT   | 探针均值出现负数（见 §4 不足），max/cap 正常 | 探针缺陷待修        |
| FTB/TAGE update Q | max=32/32 overflow>0         | 加深已证伪         |


---

## 附录：PERF 摘录

### lab19（digftest PASS）

```
END by Syscall
Commit IPC:         retire=122875  cycles=572370  IPC=0.214678
```

### Linux（软门；digftest 另列）

```
==================== myCPU PERF (sim) ====================
Commit IPC:         retire=124881985  cycles=219119196  IPC=0.569927
Commit dual-issue:  dual_cyc=32541533  any_cyc=92340452  dual_rate=35.24%  (of commit cycles)
  vs all cycles:    dual_rate=14.85%  IPC_util=28.50% of 2.0 peak
Dispatch dual:      dual_cyc=64668766  any_cyc=85476034  dual_rate=75.66%  dual_ALU=27681031
Rename/dispatch:    alloc_cyc=84273786  turnover_refill=64780845  dispatch_stall=32844416  rob_stall=19383518
BPU all-branch:     total=25752098  mispred=2011988  accuracy=92.19%
BPU cond-branch:    total=23163093  mispred=896769  accuracy=96.13%
FTB (P1 response):  resp=108054527  hit=39936747  hit_rate=36.96%
ICache (cached):    access=70355048  hit=67605528  hit_rate=96.09%
DCache (cached):    access=41046673  hit=35333550  hit_rate=86.08%
  DCache load:      access=18059804  hit=17323478  hit_rate=95.92%
  DCache store:     access=22986869  hit=18010072  hit_rate=78.35%
---- stall / mem ----
ROB full cycles:    19625778  (8.96%)
SB  full cycles:    1185699  (0.54%)
D$ MWAIT cycles:    148094  (0.07%)
D$ pend cycles:     8923625  (4.07%)  push=310817
D$ MSHR busy:       42225312  (19.27%)
LSU store-order:    538913  (0.25%)
LSU DC wait:        26525162  (12.11%)
RS_MEM src stall:   42196385  (19.26%)
RS_MEM LSU stall:   17451659  (7.96%)
RS_MEM full stall:  0  (0.00%)
---- queue occupancy: avg / max / effective-cap / >=75% cycles / full cycles ----
ROB pairs:          5.499 / 11 / 11 / 73049747 / 19625778
ROB valid entries:  9.784 / 22 / 22 / 61050367 / 848181
Dispatch slots:     0.957 / 2 / 2 / 90389174 / 90389174
RS ALU0:            0.679 / 4 / 4 / 13847577 / 5569663
RS ALU1:            0.496 / 4 / 4 / 10846811 / 4038290
RS MEM:             1.279 / 4 / 4 / 64960415 / 34925982
RS MDU:             0.030 / 2 / 2 / 1766509 / 1766509
Store buffer:       0.349 / 8 / 8 / 1837968 / 1185699
Instruction buffer: 5.039 / 16 / 16 / 50170816 / 1967939
FTQ retained:       -7.671 / 15 / 14 / 165423747 / 130689698   (探针负均值：冲刷不配平缺陷)
FTQ to IFU pending: 4.610 / 14 / 16 / 36569531 / 0
RAT busy mappings:  2.814 / 18 / 31 / 0 / 0
RAS speculative:    -9.157 / 32 / 32 / 9274719 / 131140        (同上)
RAS committed:      -8.845 / 32 / 32 / 9664291 / 190282        (同上)
FTB update queue:   max=32/32  overflow=313039
TAGE update queue:  max=32/32  overflow=159006  pipe_max=3
```

### Linux digftest

```
The first instruction of core 0 has commited. Difftest enabled.
#Detected Linux shell '/ #', stop for PERF dump.
total clock is 219119196
Commit IPC:         retire=124881985  cycles=219119196  IPC=0.569927
（全程无 mismatch，进程退出码 0；PERF 各行与软门逐位一致）
```

