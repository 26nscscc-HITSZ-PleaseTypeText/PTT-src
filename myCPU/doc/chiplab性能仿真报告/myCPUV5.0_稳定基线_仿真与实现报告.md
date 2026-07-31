# myCPU V5.0：稳定基线——仿真与实现报告

> 日期：2026-07-29  
> 本文件为 V5.0 **唯一主报告**（版本绑定、正确性、性能、实现时序、注释规范、证据哈希均在此）。  
> 基线：myCPU V4.9，Linux difftest IPC `0.734463`  
> seed：`5570815`；七 seed 见 §3.2  
> Vivado：2023.2；器件 `xc7a200t-fbg676-2`  
> 原始实现目录：`reports_impl_{func_63,pref_65,pref_70}MHZ/`（不入 Git）

---

## 0. 结论摘要

| 门控 | 结果 |
|------|------|
| RTL 完整性 | PASS：Verilator 11 条已解释 warning，无 WIDTH/隐式线网/缺口连接 |
| L2 CACOP 专项 | PASS：dirty writeback、clean invalidate、victim invalidate |
| `func_lab19` | PASS；七 seed 7/7 |
| `c_prg/memcmp` | PASS；difftest 无 mismatch |
| A0 / inner_product / CoreMark / 四探针 | PASS；四探针与基线精确一致 |
| Linux V5 difftest | PASS 到 `/ #`；IPC `0.734196`，相对 V4.9 `−0.036%` |
| U-Boot → Linux | 有条件 PASS：到 `/ #`（IPC `0.801302`）；仿真 cmdline=`earlycon` 已放宽 |
| 65 MHz 正式门 | **FAIL**：WNS `−0.072`、TNS `−0.809`、23 setup 失败；未达 +0.200 / TNS=0 |
| 63 MHz 功能布线 | **cpu_clk setup MET**（WNS `+0.074`）；全设计 1 个 DDR CDC hold 失败 |
| 70 MHz 性能布线 | FAIL（Path#1 ≈ `−0.530` ns）；仅作压力参考 |

V5.0 留树方向是稳定性与可复现性：不扩 L1/L2，不启用已知 Linux 风险的 dual
outstanding 或 JTC P0；保留前端 IB 时序解耦，并补齐 L2 下的 Cache 维护、Linux
DMA 一致性、U-Boot Cache 几何和三仓库版本绑定。

---

## 1. 版本绑定

chiplab、la32r-U-Boot、la32r-Linux 为三个独立仓库；本报告只记录源码 commit。
bitstream、DCP、波形等重型产物另存并用 SHA-256 校验（见 §6.5）。

### 1.1 V5.0 前基线

| 仓库 | 分支 | V5.0 前基线 | 说明 |
|------|------|-------------|------|
| chiplab | `v5.0` | `5dd5db86a2922c75d7b2f6363a05d2464fe51310` | 当前 myCPU/V4.9 相关源码的首次本地快照 |
| la32r-U-Boot | `v5.0` | `c7e65858aef008fc7b5d85ae900a1471dbd10f86` | 含启动地址、串口和 `bootelf` 的 pre-V5.0 既有修改 |
| la32r-Linux | `v5.0` | `09b2339a2d036f43fba8baef1301623d7352c491` | 含 CSR 工具链兼容与 `build_chiplab.sh` 的 pre-V5.0 既有修改 |

### 1.2 发布版本

| 仓库 | 发布 commit |
|------|---------------|
| chiplab | 被测功能 RTL `c3ef9a1`；收尾文档至 `02e8922`（及本合并提交） |
| la32r-U-Boot | `64ec2ccf`（uncached bootargs 暂存；含 V5 Cache 几何） |
| la32r-Linux | `17b50f62e4d4a274ed19a620ba30ee30195d2332` |

### 1.3 工具与 FPGA 目标

- Vivado：2023.2，正式证据不得混用 2019.2。
- `fpga/nscscc-team`：`xc7a200tfbg676-2`，稳定 65 MHz 性能上板目标（本轮未达正式门）。
- `fpga/loongson/2023.2`：U-Boot/Linux 启动与 MAC/DMA 一致性目标。

---

## 2. 留树改动

| 项 | 状态 | 说明 |
|----|------|------|
| 注释与接口清理 | 留树 | 注释描述当前契约；删除无消费者字段、垃圾信号和未实例化译码工具 |
| 译码直接比较 | 留树 | 删除稀疏 one-hot 展开；Verilator warning 收敛到 11 条已解释项 |
| IB 空队列直通解耦 | 留树 | RAM 写入口不再依赖 decoder/rename ready，保持冷启动同拍消费 |
| 128 KiB unified L2 | 留树 | 2 路 × 2048 组 × 32 B；256 KiB 候选因 Linux mismatch 回退 |
| L1→L2 地址型维护 | 留树 | D$ 完成 L1 写回后，请求 L2 对同一物理行写回并失效 |
| Linux Cache/DMA 适配 | 留树 | 正确报告 16 KiB/4-way/32 B；DMA 全程使用地址型写回失效 |
| U-Boot Cache 适配 | 留树 | 启动扫 128 组、4 路、32 B；`bootelf` 经 uncached `0x81f00000` 暂存 `bootargs` |
| `uboot_linux` harness | 留树 | 预装 ELF + UART `bootelf`；打包器拒绝 LOAD 段覆盖 ELF/参数区 |

`mycpu_top` 对 SoC 的外部接口没有变化；新增维护握手只存在于 `core_top` 内部。

最终 Verilator myCPU warning：**11**（`DECLFILENAME` 7 + `UNUSED` 4）。4 条
`UNUSED` 为 chiplab 保留端口 `bid`/`break_point`/`infor_flag`，以及 33×33 乘法
体系结构外高两位；不通过伪引用或 lint 屏蔽隐藏。

---

## 3. 功能正确性

### 3.1 静态检查与专项测试

| 项 | 结果 | 证据 |
|----|------|------|
| UTF-8 / 注释完整性 | PASS | `IP/myCPU` 文本严格 UTF-8；规范见 §8 |
| Verilator | PASS | 7 `DECLFILENAME` + 4 必要 `UNUSED`；无新增 Cache/L2 warning |
| L2 CACOP | PASS | `scripts/v5/test_l2_cacop.sh` → `L2 CACOP TEST PASS` |
| Linux 完整构建 | PASS | ELF32 LoongArch，入口 `0xa07ebe40` |
| U-Boot 完整构建 | PASS | ELF32 LoongArch，入口 `0xa0200000` |
| U-Boot/Linux 打包 | PASS | GOT 检查通过 |

### 3.2 `func_lab19` 七 seed（含 L2 维护后）

根因回顾：基线曾因测试程序 Cache 几何过期（2 路/16 B）在 PC `0x1c07c7a0` 触发
`0x700`；测试侧改为 `WAY=4`、`OFFSET=5`、`INDEX=7` 后全绿。L2 维护合入后终检：

| 名称 | seed | 周期 | 指令 | IPC |
|------|------|------|------|-----|
| baseline | 5570815 | 572339 | 122875 | 0.214689 |
| short1 | 5592575 | 573327 | 122875 | 0.214319 |
| short2 | 2796287 | 573328 | 122875 | 0.214319 |
| normal1 | 5570561 | 573153 | 122875 | 0.214384 |
| normal2 | 2796769 | 572509 | 122875 | 0.214625 |
| normal3 | 7410228 | 572610 | 122875 | 0.214588 |
| normal4 | 1818351 | 571882 | 122875 | 0.214861 |

相对维护前 baseline 只增加 23 周期（程序含 4 次地址型 D-Cache 维护）。

### 3.3 其余负载

| 负载 | 结果 | 周期 / 指令 / IPC | 证据 |
|------|------|-------------------|------|
| `c_prg/memcmp` | PASS | 2,911,060 / 956,303 / 0.328507 | `artifacts/v5.0/final_memcmp.log` |
| `fireye/A0` | PASS | 939,194 / 899,032 / 0.957238 | `artifacts/v5.0/final_short_matrix.log` |
| `c_prg/inner_product` | PASS | 26,828,747 / 23,038,335 / 0.858718 | 同上 |
| CoreMark | PASS | 10,042,207 / 2,032,818 / 0.202427 | `Correct operation validated`；310.320965 Iterations/Sec |

---

## 4. 性能

### 4.1 固定探针

| 探针 | V5 基线 IPC | 最终 IPC | Δ | 关键指标 / 决策 |
|------|-------------|----------|---|-----------------|
| `perf_front_stream` | 0.551265 | 0.551265 | 0.000% | I$ hit 99.97%，不扩 I$ |
| `perf_branch_mix` | 0.413587 | 0.413587 | 0.000% | Branch MPKI 56.758，JIRL 训练样本少，不开 JTC P0 |
| `perf_dcache_miss` | 0.746243 | 0.746243 | 0.000% | D$ hit 73.84%，L2 hit 94.27% |
| `perf_mshr_burst` | 0.432264 | 0.432264 | 0.000% | MSHR max=2、avg=0.196，不开 dual outstanding |

### 4.2 Linux 主对比

| 指标 | V4.9 | V5.0 | Δ / 门槛 |
|------|------|------|-----------|
| Commit IPC | 0.734463 | 0.734196 | `−0.036%`，满足 ≤0.5% 回归门槛 |
| 指令 | 122,384,311 | 122,370,475 | `−0.011%`；软件重建且 Cache 几何修正 |
| 周期 | 166,631,091 | 166,672,802 | `+0.025%` |

V5 内核正确打印 I/D Cache 均为 `16kB, 4-way, linesize 32 bytes`，difftest 全程无
mismatch 后到达 `/ #`。

---

## 5. Linux、U-Boot 与 L2 适配

myCPU 的 L2 是 CPU 内部 write-back unified Cache，而 Loongson SoC 的 MAC/DMA
直接访问 DDR，不会监听 L2。因此只清 L1 不足以保证一致性：CPU→设备前必须把
L1/L2 脏数据写回，设备→CPU 后必须让 L1/L2 的旧副本失效。

V5.0 把 Linux/U-Boot 的 DMA range 操作统一为 D-Cache 地址型 Hit Writeback+Invalidate。
D$ 先处理 L1；若 L1 脏，写入 L2 后再发 L2 maintenance；L2 命中脏行则写 DDR 后失效，
命中 clean 行直接失效，miss 直接完成，并清理同地址 victim-buffer 项。

软件镜像：

| 产物 | SHA-256 |
|------|---------|
| Linux `vmlinux` | `f5399d8dc80bfcec3903629dae7c0577a7b2ad79383ecfbbedcb75f48b734b60` |
| `uboot_linux/u-boot.bin` | `398e72837f4725e1100b2b1e48d28215bed86da7b671b4ea278c3a51f0e6736c` |
| 联合仿真日志 | `artifacts/v5.0/final_uboot_linux.log`（不入 Git） |

板上按 `Chiplab_for_vivado启动linux手册.md`：`setenv bootargs 'console=ttyS0,115200 rdinit=/init'`，
`tftpboot 0xa3000000 vmlinux`，`bootelf 0xa3000000`。`bootelf`（`64ec2ccf`）经
uncached `0x81f00000` 构造 `argv[0]+argv[1]`——**不改变你敲的命令**。

**仿真裁决（已放宽）：** 以到达 `/ #` 为通过；已知 `Kernel command line: earlycon`
与 U-Boot 打印的完整 bootargs 不一致，不阻塞 V5.0 仿真闭环。联合仿真 IPC
`0.801302`（347,551,226 周期 / 278,493,387 指令）。

---

## 6. nscscc-team 实现时序（63 / 65 / 70 MHz）

### 6.1 正式发布门槛

- 工具：Vivado 2023.2；器件：`xc7a200tfbg676-2`。
- `cpu_clk` = 65.000 MHz。
- 全设计与 `cpu_clk`：WNS ≥ **+0.200 ns**，TNS = **0**。
- setup/hold 失败端点均为 **0**；内部时序端点全部受约束。
- **同一 Git commit** 从两个干净工程各实现一轮，两轮均满足以上条件。

脉宽、CDC、I/O delay、methodology、DRC 不混入 setup/hold 数值门槛，但须在报告中
归类为已修复 / 器件 IP 固有 / 板级异步 / 明确待办。

复测流程（`fpga/nscscc-team/run_vivado`）：

```text
vivado -mode batch -source create_project.tcl
vivado -mode batch -source bit.tcl
vivado -mode batch -source collect_v5_reports.tcl -tclargs run1
# 第二轮重新 create_project，勿复用第一轮 DCP
./scripts/v5/check_timing_65mhz.sh reports_impl/v5_65mhz/run1 reports_impl/v5_65mhz/run2
```

### 6.2 历史与本轮总判定

| 目标 | 结果 | 说明 |
|------|------|------|
| 改 IB 前旧基线（65 MHz） | 参考 | WNS `−1.102`、TNS `−456.8`、1382 setup 失败；最差路径为 IFU→IB 反馈 |
| V5.0 正式 65 MHz 门 | **FAIL** | 单轮证据；WNS `−0.072`、TNS `−0.809`、23 setup 失败 |
| 功能上板 63 MHz（setup） | **cpu_clk setup MET** | WNS `+0.074`、TNS `0`；全设计 1 个 DDR CDC hold（非 cpu_clk） |
| 性能冲刺 70 MHz | **FAIL** | Path#1 ≈ `−0.530` ns |

相对旧基线，65 MHz 已显著改善（IB 反馈不再是第一名），但**尚未达到发布门槛**。

### 6.3 三套报告数字

| 目录 | cpu_clk | Design WNS / TNS / setup失败 | Hold | 最差 cpu_clk setup 路径 |
|------|---------|------------------------------|------|-------------------------|
| `reports_impl_func_63MHZ` | 15.833 ns ≈ **63.16 MHz** | **+0.074 / 0 / 0** | 全设计 WHS `−0.085`（1 端点，sys_clk→clk_pll_i）；**cpu_clk hold 干净** | `dcache.req_paddr` → `rs_mem.robid`，23 级，datapath 15.202 ns |
| `reports_impl_pref_65MHZ` | 15.385 ns = **65.00 MHz** | **−0.072 / −0.809 / 23**（QoR） | cpu_clk hold MET | `store_buffer.tail` → `rs_mem.s0_val` CE，24 级，datapath 15.137 ns |
| `reports_impl_pref_70MHZ` | clocks：**14.286 ns = 70 MHz** | Path#1 ≈ **−0.530** | cpu_clk hold MET | `dcache.req_paddr` → `dcache.dirty_arr`，21 级，datapath 14.651 ns |

证据完整性：

1. `pref_65MHZ` **缺少** `timing_summary.rpt`；数字由 `qor_assessment` +
   `design_analysis_timing` + `timing_setup_cpu_clk` 交叉确认。
2. `pref_70MHZ/timing_summary.rpt` 内部写的是 **65.000 MHz**，与同目录
   `clocks.rpt`（70 MHz）矛盾，疑似误放；**70 MHz 不以该 summary 为准**。
3. 未提供 `run_metadata.txt` / 双轮同 commit，故 `check_timing_65mhz.sh` 无法 PASS。

资源（Routed）：

| 项 | 63 MHz 功能 | 65 MHz 性能 |
|----|-------------|-------------|
| Slice LUTs | 78545 / 134600（58.35%） | 79405 / 134600（58.99%） |
| Slice Registers | 41572 / 269200（15.44%） | 41886 / 269200（15.56%） |
| Block RAM Tile | 84 / 365（23.01%） | 84 / 365（23.01%） |
| DSPs | 4 / 740 | 4 / 740 |

功能上板建议按 **63 MHz** 已布线结果使用；冲 65 MHz 优先削 SB→RS_MEM 与 D$
`req_paddr`/`dirty` 长路径（距 +0.200 约差 0.272 ns）。

### 6.4 正式门对照表（65 MHz）

| 正式门项 | 65 MHz 实测 | 门槛 |
|----------|-------------|------|
| Vivado | 2023.2 | 2023.2 |
| WNS | −0.072 ns | ≥ +0.200 ns |
| TNS | −0.809 ns | 0 ns |
| setup 失败端点 | 23 | 0 |
| hold（cpu_clk） | 干净 | 干净 |
| 两轮同 commit | 未提供 | 需要 |

### 6.5 实现报告关键文件 SHA-256

```
019dd15a35945f8482ec8920627f37ad0e877c975f29cf73fa4e64bc02925e6b  reports_impl_func_63MHZ/timing_summary.rpt
16e0ba4bca53f747ab52467eab6f18cc6c37e2a623b213527ff64d3a832e06e7  reports_impl_func_63MHZ/timing_setup_cpu_clk.rpt
e0f0ff0dfb1688e4d847eda542f93ff29f2b57089a4eb80b86037c5e9bc0d81b  reports_impl_func_63MHZ/timing_hold_violations.rpt
2cf0c836aaa10bacba5f30248cb0b4598034206db46ad34692bc613866b82242  reports_impl_func_63MHZ/qor_assessment.rpt
0de0f77cf3397dd4cb5e8320038d1694396872b0914126f587326804ada80478  reports_impl_func_63MHZ/clocks.rpt
b183e20908f30ab925ab1256b9f9b92eef1e18e2e47906fbffe0aff421b99141  reports_impl_func_63MHZ/utilization.rpt
c30c7c671fe117198466bd5df94761d9186961ca9d914ac26948a39aaee447c4  reports_impl_func_63MHZ/design_analysis_timing.rpt
864b183e5001e2cfefe1f0666331f3c164b6039861ec866de5119f25ed1297a6  reports_impl_pref_65MHZ/timing_setup_cpu_clk.rpt
687d7ffa5dc3969b61245634619072a675aba43979121b58b877f8a8304fe5e3  reports_impl_pref_65MHZ/qor_assessment.rpt
848553b29722f13c783231b1b131187059f633075936287a822026cfdaa88311  reports_impl_pref_65MHZ/clocks.rpt
ae965d56b7b25032a523e1df894babca9e95bf803248459991b68aefef9e4b69  reports_impl_pref_65MHZ/utilization.rpt
89c0b525a01a5fbd0514d918eb0906527c07b33e63153ab4d0d1a7cdf57c0d4b  reports_impl_pref_65MHZ/design_analysis_timing.rpt
58d3800a0f284257976ae962a7b471546f318d3e55bf43fa26914dd3a6aa19c6  reports_impl_pref_65MHZ/check_timing.rpt
4ec9f433c6e6f4b5b5db6bed1675bc8ddbc8a0eb7d8d2fb6fdf4e3c01af0f711  reports_impl_pref_70MHZ/timing_setup_cpu_clk.rpt
ea568424b09ae84c52254371a2e27c882467eb53b162b3e2060abe361b066161  reports_impl_pref_70MHZ/design_analysis_timing.rpt
1cabc276cdf4876b8a8db207515ddc420cf841b45363a0be0f63eeaf2ebe8724  reports_impl_pref_70MHZ/clocks.rpt
f6f65c1ff039ad2f5d7da66c0b36e1acfbcb15ec6acbc771c2677788db354c74  reports_impl_pref_70MHZ/qor_assessment.rpt
e099e753f087190fe3070dd093597b77160ebebf8f7a05c1bbbfae931c1608c1  reports_impl_pref_70MHZ/utilization.rpt
```

---

## 7. 探索未留树

| 候选 | 局部结果 | 回退原因 |
|------|----------|----------|
| 256 KiB L2 | MSHR 探针 IPC +0.738% | Linux difftest 晚期 mismatch（PC `0xa0337b7c`） |
| FTB 独立更新端口 | overflow 大幅下降、hit 上升 | branch 探针 IPC −0.047% |
| uBTB 目标步长 | 无误预测改善 | IPC 下降 |
| dual outstanding | V4.9 已见 MSHR 顶满 | 既有 Linux `0x700` 风险，V5 不重开 |
| JTC P0 | 有普通 JIRL 优化可能 | 样本少且有历史 Linux 回归 |

---

## 8. RTL 注释与完整性规范

适用于 `IP/myCPU` 下 Verilog/SystemVerilog。目标：注释描述当前实现，接口与不变量
可脱离版本考古独立理解。

### 8.1 文件头

每个主要模块应说明：职责与流水级位置；握手规则；主要状态/队列/Cache 几何；
flush/异常/回压/复位；关键不变量。辅助 RAM 可用简化头，但须说明读写时序。

### 8.2 正文注释

- 简体中文、全角标点；信号名/模块名/协议名/单位保留英文。
- 描述当前行为与原因，不在 RTL 记 `Vx.x`、合入来源或已回退方案。
- 单行 `// `；章节 `// ---------------- 名称 ----------------`。
- 时序写明优先级；组合写明默认、互斥与旁路；计数器注明单位与满/空。
- `TODO` 仅用于未实现且有验收条件的项。

### 8.3 完整性与 lint

- 禁止 `x & 0`、无输出 OR、`*_lint` 吸收 warning。
- 无用非契约信号删除；保留兼容端口须在端口处说明原因。
- 例化端口全部显式连接；不用的输出接语义明确的 `unused_*`。
- 禁止隐式网络；位宽转换须显式扩展/截断/切片并说明范围。
- 阻塞/非阻塞按组合/时序语义；RAM 推断例外局部说明。

### 8.4 验证要求

每批注释/完整性修改至少：`git diff --check`；Verilator 重编并比较 warning；
`func_lab19` seed `5570815` difftest；涉及握手/Cache/异常/前端时再跑专项探针。
注释修改不得改逻辑；行为修改须独立提交。

---

## 9. 下一步

1. **V5.0 基线已收尾**：仿真矩阵闭环；65 MHz 正式门本轮 FAIL；功能 63 MHz setup MET。
2. **冲 65 MHz（V5.0.1/V5.1）**：优先 SB→RS_MEM 与 D$ 长路径；两轮干净实现并补齐
   `timing_summary.rpt` + `run_metadata.txt`。
3. 清理 `pref_70MHZ` 内误放的 65 MHz `timing_summary.rpt`。
4. 板上 Linux/`/`# 与 DMA（同步当前 `IP/myCPU`，勿混 `chiplab_soc` 旧 L2）。
5. （非阻塞）仿真 earlycon / bootargs；dual-outstanding、FTB overflow、256 KiB L2 根因。
6. **WIP 正确性**：修复当前工作树 `func_lab19` `0x700` 后再谈合入；在此之前勿把 WIP IPC
   当作发版数字。
7. L2 性能：E 面 32→128 已证明显著；扩 256KB / write-allocate / L2 MSHR 仅在 lab19+Linux 绿后立项。

---

## 10. 复现与证据

| 内容 | 入口 / 路径 |
|------|-------------|
| L2 CACOP | `scripts/v5/test_l2_cacop.sh` |
| lab19 单 seed / 七 seed | `scripts/v5/check_lab19.sh` / `run_lab19_7seeds.sh` |
| warning 汇总 | `scripts/v5/summarize_mycpu_warnings.sh` |
| 65 MHz 判定脚本 | `scripts/v5/check_timing_65mhz.sh` |
| 原始实现目录 | `reports_impl_func_63MHZ/`、`reports_impl_pref_65MHZ/`、`reports_impl_pref_70MHZ/` |
| 基线四探针 | `artifacts/v5.0/baseline/perf_probes.log` |
| lab19 七 seed 日志 | `artifacts/v5.0/baseline/lab19_7seeds/` |
| V5 Linux difftest | `artifacts/v5.0/final_linux_v5_relocated_difftest.log` |
| 最终短矩阵 | `artifacts/v5.0/final_short_matrix.log` |
| U-Boot → Linux | `artifacts/v5.0/final_uboot_linux.log` |
| 256 KiB Linux 失败 | `artifacts/v5.0/candidate_l2_256k_linux_difftest.log` |
| **2026-07-31 全套 PERF（WIP）** | `artifacts/v5.0/perf_matrix_128k/` |
| **L2 32KB 对照** | `artifacts/v5.0/perf_matrix_32k/` |
| 优化-仿真表手册 | `IP/myCPU/doc/优化-仿真表.md` |

`artifacts/`、bitstream、DCP 和软件大镜像不入 Git；发布归档时另存并附 SHA-256。

---

## 11. 优化-仿真表全套 PERF 与 L2 复测（2026-07-31）

> 手册：[优化-仿真表.md](../优化-仿真表.md)。seed=`5570815`。  
> **被测对象**：chiplab 工作树未提交 RTL（LSU/D$/前端等 WIP），**不是** §0–§4 的发版快照。  
> 发版 V5.0 数字仍以 §3–§4 / `artifacts/v5.0/final_*` 为准。

### 11.1 L2 是否开启 / 是否吃满

| 问题 | 结论 |
|------|------|
| L2 开了吗？ | **是**。`mycpu_top` 无条件例化统一写回 L2：**128KB**（2×2048×32B）；无 bypass。 |
| 吃满了吗？ | **没有。** 探针 `perf_dcache_miss` L2 hit **94%**，但 Linux 发版日志仅 ~**56%**；无独立 L2 MSHR；行写 miss 不分配；主 FSM 单请求 + I-miss 引擎。 |
| 「提升很大」归因 | **两层**：历史 V3.4 **32→128KB**（Linux ~+17%）；本 WIP 相对 V5 发版探针的大跳变主要来自 **LSU/前端微架构**，不是又扩了 L2。 |

### 11.2 A–G 全套 PERF（WIP @ 128KB L2）

| 面 | 负载 | IPC | Commit dual | D$ hit | L2 hit | Branch MPKI | MSHR busy | ld miss lat avg |
|----|------|-----|-------------|--------|--------|-------------|-----------|-----------------|
| A | `fireye/A0` | **1.325** | 89.1% | 84.5% | 90.0% | 0.183 | 30.7% | 2.55 |
| A | `inner_product` | **1.101** | 65.6% | 97.1% | 89.8% | 0.073 | 2.1% | 2.94 |
| B | `perf_front_stream` | 0.566 | 8.3% | 97.0% | 17.4% | 0.094 | 0.1% | 25.1 |
| C | `perf_branch_mix` | 0.500 | 49.3% | 99.1% | 17.0% | **33.1** | 0.2% | 14.3 |
| D | CoreMark | 0.217 | 50.5% | 98.8% | 40.4% | 2.74 | 0.1% | 24.9 |
| E | `perf_dcache_miss` | **0.869** | 67.5% | 68.1% | **94.2%** | 0.179 | 16.5% | **2.03** |
| E′ | `memcmp`（difftest） | 0.354 | 36.9% | 90.9% | 66.9% | 0.933 | 3.1% | 2.45 |
| F | `perf_mshr_burst` | 0.476 | 55.2% | 71.3% | 80.8% | 0.297 | 14.5% | 2.58 |

相对 **V5 发版**同探针（§4.1）：A0 `0.957→1.325`（+38%）、`dcache_miss` `0.746→0.869`（+16%）、`front`/`branch` 小幅上升。  
**D 面**：A0 dual≈89% vs CoreMark dual≈50%（发版 CoreMark 曾仅 ~5%）——后端配对改善，但仍远低于 A0。  
**G 面**（occupancy）：E/A0 上 MSHR max 仍顶满 cap=2；SB/STQ 未长期顶满；F 面不足以为 dual-OS 开绿灯（且 WIP lab19 红）。

CoreMark UART：`Correct operation validated`（395.5 Iterations/Sec）。memcmp：到达 test end + difftest 无 mismatch 报告。

### 11.3 L2 容量 A/B（同 WIP，临时 `L2_NSET=512` → 32KB，测完已恢复 128KB）

| 负载 | L2 | IPC | L2 hit | MSHR busy | ld miss lat avg |
|------|----|-----|--------|-----------|-----------------|
| `perf_dcache_miss` | 128KB | **0.869** | **94.2%** | 16.5% | **2.03** |
| `perf_dcache_miss` | 32KB | 0.649 | 22.7% | 45.1% | 13.4 |
| `perf_mshr_burst` | 128KB | **0.476** | **80.8%** | 14.5% | **2.58** |
| `perf_mshr_burst` | 32KB | 0.385 | 28.5% | 36.9% | 17.2 |

解读：在默认 64KB 工作集探针上，**128KB 相对 32KB 的 IPC 增益约 +34%（E）/ +24%（F）**，与 V3.4「扩 L2 很赚」一致；L2 hit 从 ~23% 拉到 ~94% 是主因。  
**未吃满证据**：同一微架构下 Linux 发版 L2 hit 仅 ~56% → 工作集/替换/写策略仍有头寸；无「关 L2」旁路对照。

### 11.4 正确性与三仓库适配（本轮）

| 项 | 结果 |
|----|------|
| L2 CACOP 专项（WIP `l2cache`） | **PASS**（`scripts/v5/test_l2_cacop.sh`） |
| `func_lab19` difftest（WIP） | **FAIL** `Both Error(Code:0x700)`；日志见 `artifacts/v5.0/perf_matrix_128k/run_lab19_real.log` |
| `chiplab_soc` myCPU | 已用 chiplab **`02e8922` HEAD** 覆盖（128KB + `l2_cacop`）；**不含 WIP**。戳记：`chiplab_soc/.../SYNC_FROM_CHIPLAB.txt` |
| la32r-U-Boot | Kconfig 默认改为 32B/shift=5（无 prompt 符号原先忽略 defconfig）；`make la32rsoc_defconfig` 后 `.config` 为 32 |
| la32r-Linux | DMA 仍为 Hit-WB+Inv；增补 **scache 信息打印**（128KB/2-way/32B），不改 DMA 算法 |
| 手册 | `Chiplab_for_vivado启动linux与ucore手册.md` §0：禁止混旧 SoC 核 |

**剩余缺口（文档化，未改 RTL）**：Index/I$ cacop 不清 L2；Linux `flush_icache_range` 仅 `ibar`；无 `Cache_S`。

### 11.5 瓶颈解读（WIP）

1. **E 面已证明 128KB L2 价值很大**；再扩容量前必须先修 lab19。  
2. **C 面 Branch MPKI≈33** 仍是分支探针主痛（相对 L2）。  
3. **B 面** I$ hit≈100% 但 IB `push_stall` / FTQ pending 高 → 供指结构，不是 L2。  
4. **F 面** MSHR 顶满但 dual-OS 历史有 Linux 风险 → 暂不开。  
5. 发版路径继续用 §0 门控；WIP 大 IPC **不得**覆盖发版结论。
