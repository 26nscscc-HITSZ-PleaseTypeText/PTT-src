# IPC1 aggressive / timing-safe candidate

## 版本定位

本目录是独立新版本，未覆盖保留版：

- 保留版：`D:\frontend\myCPU-0729_ipc1_candidate_v2`
- 本版：`D:\frontend\myCPU-0729_ipc1_aggressive_final_80m`

CPU 顶层 AXI、debug 和中断接口均未改变。ITTAGE/JTC 没有重新加入。

## 最终保留的优化

### 1. LSU 与 DCache 的多请求命中流水

- LSU 增加 4 项请求 token，按 ROB ID 保存返回所需的访存元数据。
- DCache 返回增加 ROB tag 以及 valid/ready 握手。
- 命中和 uncached 返回支持保持及反压，flush/cancel 会杀死错误路径请求。
- cached load 常见路径可达到每拍接收一个请求，不再等待前一条 load 完整写回后才接收下一条。

### 2. LSU issue skid，切断 Store Buffer 到 RS ready 的长组合路径

- LSU 前端增加一项 issue 暂存。
- `lsu_ready` 不再组合依赖 Store Buffer 查询结果。
- 常见无阻塞路径仍直接进入地址生成级；下游暂时阻塞时才使用 skid。

### 3. 正确预测分支后的双提交

- slot 0 是正确预测分支、slot 1 是普通指令时，允许两条同拍退休。
- 双分支、错误预测、特权/异常和需要冲刷的情况仍保持串行。
- 不增加 FTQ 第二训练口，也不放宽 JIRL 的保守校验。

### 4. TAGE 弱 provider 的 alternate 阈值

- `use_alt_on_na` 选择阈值由 12 调为 2。
- 该值经过 12/8/4/2/0 五档 20 项实测；2 的综合 CR1 最优。
- 没有增加预测表容量，也没有增加新的预测器级。

### 5. 两个 64 项直接索引 fallback BTB bank

- 原 64 项直接映射表改为 64 set × 2 way。
- 每个 bank 都只做一次直接索引和一次 tag 比较，不是大全相联结构。
- miss 时使用每 set 的轻量替换位；hit 时更新原 way。
- 仍只在主 uBTB miss 后使用，RET 目标继续由 RAS 优先覆盖。
- CoreMark 实测 P1 correction 从 26,588 降为 20,614。

## 已淘汰的激进实验

- ALU RS 满站同拍出队/补位：性能有小幅收益，但 12.5 ns OOC 综合 WNS 为
  `-0.067 ns`，已从本版撤销。
- 双 store 提交 skid：20 项周期逐项不变，未保留。
- 128 项单 bank fallback：收益极小且 CR1 反而略差。
- TAGE 本地 alternate chooser：不同用例互相伤害，总 CR1 退化，未保留。
- 5 位局部历史低置信度修正器：quick_sort 有收益，但 CoreMark、bubble_sort 和
  fireye_C0 的退化更大，未保留。
- UART LSR 提前执行：主要改变启动/打印轮询的全程 IPC，对计分 CR1 几乎无收益，
  且扩大 MMIO 语义风险，未保留。

## 仿真结果

`nscscc_perf` 20 项均以 `status=FINISH` 结束：

| 指标 | 保留版 candidate_v2 | 本版 |
|---|---:|---:|
| 20 项全程周期合计 | 6,521,183 | 6,093,934 |
| retired 指令合计 | 4,913,066 | 4,913,197 |
| 加权全程 IPC | 0.753401 | 0.806244 |
| CR1 合计 | 3,739,988 | 3,345,474 |

相对保留版：

- 全程减少 427,249 周期（6.55%）。
- IPC 相对提高约 7.01%。
- CR1 减少 394,514（10.55%）。

日志位于：

`D:\frontend\.codex_work\nscscc_cpu5\logs\ipc1_final80`

## 关于“IPC = 1”

这里的 0.806244 是从复位到测试结束的全程加权 IPC。它包含启动代码、串口打印和
UART LSR 轮询；Dhrystone、Stringsearch 的全程 IPC 尤其受串行 MMIO 等待影响，
而这些等待大多位于 CR1 计分窗口之外。多个实际计算内核已经超过 IPC 1，例如
CRC32、loop_induction、minmax_sequence 和 my_memcmp。

因此，不能在不修改外设语义或统计口径的前提下承诺整套全程 IPC 达到 1。本版选择
真实降低 CR1、保持体系结构语义的修改，不用缓存 MMIO 或修改计数器“做高”IPC。

## 时序说明

目标器件：`xc7a200tfbg676-2`，OOC 目标周期：12.5 ns（80 MHz）。

最终 OOC 综合结果：

| 指标 | 结果 |
|---|---:|
| WNS | +0.217 ns |
| Slice LUT | 64,531 |
| Slice Register | 28,085 |
| BRAM Tile | 81.5 |
| DSP | 4 |

最差路径为 BPU `pc_reg[10]` 到 `pc_reg[28]`，data path delay 为 12.237 ns。
报告位于：

`D:\frontend\.codex_work\nscscc_cpu5\vivado_ipc1_aggr16`

OOC 综合只能验证综合后的逻辑时序，不能替代完整工程 place/route；最终实现结果仍应
以比赛工程报告为准。
