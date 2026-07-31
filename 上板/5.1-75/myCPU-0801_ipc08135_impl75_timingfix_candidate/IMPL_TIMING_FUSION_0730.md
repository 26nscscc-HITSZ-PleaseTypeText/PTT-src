# 0730 实现时序违例处理与 IPC09 融合说明

## 交付版本

- 完整 CPU RTL：`D:\frontend\myCPU-0730_ipc09_impl_timing_fused`
- 性能与前端主线来源：`D:\frontend\myCPU-0729_ipc09_aggressive_final_80m`
- 实现报告一：`D:\report\reports_impl-0730-2-70`
- 实现报告二：`D:\report\reports_impl-0730-3-70`

三个来源目录均未覆盖。本目录包含完整 CPU，`core_top` 的 AXI、debug、中断
接口不变。

## 两份布局布线报告

| 实现版本 | 频率 | WNS | TNS | setup 失败端点 | hold |
|---|---:|---:|---:|---:|---:|
| `myCPU-0729_ipc1_candidate_v2` | 70 MHz | -0.130 ns | -2.261 ns | 66 | +0.051 ns |
| `myCPU-0729_ipc1_aggressive_final_80m` | 70 MHz | -0.065 ns | -0.120 ns | 2 | +0.052 ns |

两份报告均无 hold 和 pulse-width 违例。CPU setup 违例只有下面两类。

### 1. Store Buffer 到 RS_MEM

candidate_v2 的 66 个端点都属于同一逻辑锥：

`store_buffer tail`
`→ 逐项地址比较/逐字节合并`
`→ sb_query_partial`
`→ LSU a_go/ready`
`→ RS_MEM 操作数寄存器 CE/D`

最差路径为：

`u_store_buffer/tail_reg[1]`
`→ u_rs_mem/s1_val_reg[0][11]/CE`

数据路径 14.033 ns、19 级逻辑，其中布线占 11.148 ns。

最新 IPC09 主线已经包含该问题的修复：LSU 入口有一项 issue skid，
`lsu_ready_o` 只由本地 `q_valid` 寄存状态产生。Store Buffer 的组合查询只能决定
LSU 内部流水是否推进，不能再反向进入 RS_MEM 的 issue/CE 网络。本融合版保留该结构。

对应 RTL：

- `backend\execute\lsu.v` 中 `q_valid` 及 issue skid；
- `lsu_ready_o = !q_valid && !flush_i`。

### 2. DCache 到 ROB result

aggressive_final 的两个失败端点分别是：

1. `DCache req_paddr/tag/data select → ROB result_reg`，WNS -0.065 ns；
2. `DCache MSHR state/refill select → ROB result_reg`，WNS -0.056 ns。

它们都来自 `LSU_ROB_EARLY_COMPLETE=1`：虽然普通 LSU 写回总线已经寄存，
ROB 仍绕过该寄存器，直接接收 `mem_wb_*_raw`，把 DCache 命中和 MSHR
选择网络重新接到了 ROB 数据 RAM。

本融合版将：

`LSU_ROB_EARLY_COMPLETE = 0`

ROB 改为只接收现有的寄存 `mem_wb_*` 总线。RS 仍保留独立的
`mem_fast_wb_*` cached-load 同拍操作数旁路，因此切断的是体系结构 ROB 写入长线，
没有把所有 load-use 优化一起关闭。

## 与最新 IPC09 优化的融合内容

继续保留：

- 四 bank + FWFT 指令缓冲；
- 双 16 项 bank uBTB 与 2 位方向计数器；
- GHR 112 位，TAGE 历史长度 `11/23/53/112`；
- FTB、RAS、fallback BTB、P0 块长度快速路径；
- FTB 冷启动/训练过滤、FTQ/P1 bypass；
- 最新 store 的轻量 STQ 前递；
- DCache cached-load 快速依赖唤醒；
- L2 到 DCache refill 关键字直返；
- L1 TLB 流水回填；
- 不包含 ITTAGE 和独立 JTC。

## nscscc_perf 验证

20 项程序全部以 `status=FINISH` 结束。

| 指标 | IPC09 最新性能版 | 本融合时序版 |
|---|---:|---:|
| 20 项全程周期 | 5,878,136 | 5,954,913 |
| retired 指令 | 4,912,853 | 4,913,361 |
| 20 项全程 IPC | 0.835784 | 0.825094 |
| 18 项计算类周期 | 4,685,688 | 4,757,700 |
| 18 项计算类 IPC | 0.934281 | 0.920201 |
| CR1 合计 | 3,228,634 | 3,289,246 |

ROB load completion 延后一拍使计算类 IPC 相对下降约 1.51%，但仍高于 0.9。
这是消除真实 DCache 到 ROB 布线违例的性能/时序取舍。没有修改 MMIO、UART
语义或性能计数器。

仿真日志：

`D:\frontend\.codex_work\nscscc_cpu5\logs\ipc09_fused`

## 80 MHz 核级综合

目标器件：`xc7a200tfbg676-2`；目标周期：12.5 ns。

| 指标 | 结果 |
|---|---:|
| WNS | +0.245 ns |
| TNS | 0.000 ns |
| setup 失败端点 | 0 |
| Slice LUT | 65,458 |
| Slice Register | 28,862 |
| BRAM Tile | 81.5 |
| DSP | 4 |

综合后最差 500 条路径中：

- `DCache → ROB`：0 条；
- `Store Buffer → RS_MEM`：0 条。

综合报告：

`D:\frontend\.codex_work\nscscc_cpu5\vivado_ipc09_fused`

该结果证明 CPU RTL 的目标长逻辑锥已经切断，但 OOC 综合不能代替完整 SoC
布局布线。最终是否完全收敛，应以把本目录替换进比赛工程后的 routed
`timing_setup_violations.rpt` 和 `timing_summary.rpt` 为准。
