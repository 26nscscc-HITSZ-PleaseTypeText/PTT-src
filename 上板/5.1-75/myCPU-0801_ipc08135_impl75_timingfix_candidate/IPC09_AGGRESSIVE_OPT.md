# IPC 0.9 激进优化版说明

> 本文件记录被融合的 IPC09 性能主线。当前 0730 时序融合版的改动和最新验证结果，
> 请以同目录 `IMPL_TIMING_FUSION_0730.md` 为准。

## 版本定位

- 保留版：`D:\frontend\myCPU-0729_ipc1_aggressive_final_80m`
- 验证候选目录：`D:\frontend\myCPU-0729_ipc09_aggressive_v24_fast_length`
- 最终交付目录：`D:\frontend\myCPU-0729_ipc09_aggressive_final_80m`

保留版没有被覆盖。本版仍是完整 CPU RTL，CPU 顶层 AXI、debug、中断接口均未修改。

## 前端保留内容

这次没有推翻已经验证过的前端主线，继续保留：

- 四 bank、FWFT 指令缓冲；
- 两个 16 项 bank 的 uBTB，并带 2 位方向计数器；
- FTB、TAGE、RAS、fallback BTB 和块长度缓存；
- TAGE 历史长度 `11/23/53/112`，GHR 长度 112 位；
- FTB 冷启动与训练路径、FTQ/P1 bypass 等已有优化；
- RET 仍由 RAS 优先给目标；
- 不包含 ITTAGE 和独立 JTC 模块。

## 本版新增优化

### 1. 时序安全的 Store-to-Load Forwarding

LSU 的 16 项未提交 Store Queue 原来只能做冲突阻塞，本版增加 store data，
允许 load 从“全局最新的一条未提交 store”前递。

只在以下条件全部满足时前递：

- 最新 store 与 load 位于同一个 32 位字；
- store 覆盖 load 需要的全部字节；
- store 不是 uncached 访问；
- 不存在需要维持严格顺序的 uncached store。

如果最新 store 不满足条件，但队列中存在其他重叠 store，仍走原来的保守等待路径。
这样不会用 16 项年龄比较和优先选择树压长 LSU 时序。

### 2. Cached Load 命中依赖快速唤醒

RS_MEM 可以在 DCache 返回 cached-load hit 的同一拍识别依赖并选择操作数，减少
`load -> address generation -> next load` 相关链的停顿。

为避免 token CAM 和优先编码器进入快速有效位路径，LSU 增加按 ROB ID 直接索引的
有效位镜像。原有 token 表仍保存访存类型、虚地址等完整元数据，体系结构写回路径
没有改变。

原始 DCache 数据不再直接送入 MDU 保留站。MDU 使用 LSU 已寄存的 hold 结果，
切断曾经出现的 DCache 到 MDU 13 ns 以上组合路径。

### 3. L2 DCache Refill 关键字直返

DCache miss 的两个 128-bit refill beat 从下层存储器返回时，L2 直接逐拍送给
DCache，同时仍在最后一拍把完整 256-bit cache line 安装进 L2。

原实现先收完整行，再经过 `S_RET0/S_RET1` 重放；本版省去这两个固定等待拍。
I-cache refill、uncached 访问、写回和 L2 行安装语义保持不变。

### 4. L1 TLB 回填打一拍

主 TLB 命中结果仍在当拍用于当前地址翻译，但写入 8 项 L1 微型 TLB 前先进入一项
回填寄存器。这样切断：

`FTQ 地址 -> 32 项主 TLB 查询/选择 -> L1 TLB FIFO 写使能`

回填只延后一拍，不增加当前请求的翻译延迟。主 TLB 数据无条件进入普通 D 输入
寄存器；`found` 资格、4KB 页过滤和重复页检查分两拍完成，避免综合器把主 TLB
选择链映射到回填寄存器的 CE/R 控制脚。TLB fence 或 ASID/主表变化会丢弃尚未
安装的旧回填。

### 5. P0 块长度快速路径

uBTB 仍负责 P0 的方向、目标和分支类型，但 P0 顺序下一 PC 不再等待 uBTB
16 项全相联命中后的块长度选择网络。与 uBTB 同步训练的两路 fallback BTB
并行给出块长度；fallback 未命中时，P0 安全地取到本次 fetch/Cache Line 边界，
P1 FTB 仍会修正不一致的描述符。

该改动不改变预测方向、跳转目标或体系结构状态，只切断 P0 最差组合路径。
相对改动前的 v22，18 项计算类测试只增加 49 周期，而 80 MHz OOC WNS
从 `-0.003 ns` 改善为 `+0.242 ns`。

## nscscc_perf 仿真

20 项测试全部以 `status=FINISH` 结束。

| 指标 | 保留版 | 本版 |
|---|---:|---:|
| 20 项全程周期 | 6,093,934 | 5,878,136 |
| retired 指令 | 4,913,197 | 4,912,853 |
| 20 项全程加权 IPC | 0.806244 | 0.835784 |
| 18 项计算类加权 IPC | 0.895661 | 0.934281 |
| CR1 合计 | 3,345,474 | 3,228,634 |

相对保留版：

- 全程减少 215,798 周期（3.54%）；
- 18 项计算类 IPC 相对提高约 4.31%；
- CR1 减少 116,840（3.49%）。

主要周期改善：

| 测试 | 周期变化 |
|---|---:|
| bubble_sort | -17.30% |
| loop_induction | -7.33% |
| fireye_D1 | -7.31% |
| coremark | -6.91% |
| lookup_table | -5.83% |
| stream_copy | -5.30% |

仿真日志：

`D:\frontend\.codex_work\nscscc_cpu5\logs\ipc09_v24`

另外运行了 L1 TLB 回填定向测试，覆盖主表当拍透传、连续两页流水回填、回填后
L1 命中和 fence 清除 pending 回填，结果为 `PASS: pipelined L1 TLB refill`。

## IPC 统计口径

0.934281 是排除 Dhrystone 和 Stringsearch 后，18 个计算类程序按 retired/cycles
重新加权得到的 IPC。这两个程序的全程统计包含大量 UART LSR 轮询和串口发送等待，
它们大多不在 CR1 计分窗口内。把它们计入后，完整 20 项全程 IPC 为 0.835784。

本版没有缓存 MMIO、伪造 UART ready 或修改性能计数器。若要求包含固定串口等待的
20 项全程 IPC 也达到 0.9，仅靠 CPU 前后端优化并不现实，必须改变外设或测试语义，
这会增加正确性风险。

## 综合时序与资源

目标器件：`xc7a200tfbg676-2`；OOC 目标周期：12.5 ns（80 MHz）。

| 指标 | 结果 |
|---|---:|
| WNS | +0.242 ns |
| TNS | 0.000 ns |
| 时序失败端点 | 0 |
| Slice LUT | 65,270 |
| Slice Register | 28,749 |
| BRAM Tile | 81.5 |
| DSP | 4 |

综合报告目录：

`D:\frontend\.codex_work\nscscc_cpu5\vivado_ipc09_v24`

OOC 综合验证的是 CPU RTL 的综合后逻辑时序，不能代替完整 SoC 工程的
place/route。最终板级 WNS 仍应以比赛工程实际布局布线报告为准。

## 已实验但未保留的方案

- 4 项 DCache MSHR：当前下层存储接口仍串行，只有资源和约 50 周期变化。
- ALU RS 扩至 8 项、ROB 扩至 64 项、TAGE 表扩容：收益小或时序/资源代价不合算。
- 256 KB L2：初始化开销抵消大部分容量收益。
- 16 项全 STQ 年龄选择：计算 IPC 可到约 0.94，但综合形成长选择网络，
  80 MHz WNS 明显恶化。
- 原始 DCache 数据同时旁路到 MDU：产生 DCache 到 MDU 的 13.267 ns 路径。
- 完全切断 RS_MEM load-hit 旁路：时序更保守，但计算类 IPC 降至 0.911351，
  因此只作为实验兜底，没有进入最终版。
- 对 P0 32 位顺序地址加法器做分段进位：功能仿真一致，但综合布局映射把
  WNS 恶化到 `-0.619 ns`，因此没有保留。
