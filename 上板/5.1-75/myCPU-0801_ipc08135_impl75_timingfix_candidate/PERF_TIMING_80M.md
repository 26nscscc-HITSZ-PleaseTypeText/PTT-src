# 80 MHz / 加权 IPC 0.75 优化版说明

## 1. 版本与目标

- 完整 CPU RTL：`D:\frontend\myCPU-0729_perf_timing_80m`
- 对照基线：`D:\frontend\myCPU-0729_frontend_timing_merge_all_path_cut`
- OOC 综合器件：`xc7a200tfbg676-2`
- 时钟约束：12.500 ns，即 80 MHz
- 性能目标：20 项 `nscscc_perf` 总退休指令数 / 总周期数不低于 0.75

本版本没有改变 `core_top` 与 SoC 的外部接口。前端保留四 bank Instruction
Buffer、已存条目的 FWFT 读出、112 位 GHR、uBTB/FTB/TAGE/RAS 主线以及
冷启动优化；没有实例化 ITTAGE 或普通 JIRL 目标缓存。

## 2. 最终验收结果

### 2.1 性能

仿真器为 Verilator。20 项测试均得到 `RESULT status=FINISH`。

加权 IPC 按下面的方式计算：

```text
加权 IPC = 20 项总退休指令数 / 20 项总周期数
         = 4,913,483 / 6,544,935
         = 0.750730603
```

| 汇总指标 | 对照基线 | 最终版本 | 变化 |
|---|---:|---:|---:|
| 总退休指令 | 4,888,925 | 4,913,483 | +24,558 |
| 总周期 | 14,329,010 | 6,544,935 | -54.32% |
| 加权 IPC | 0.341191 | **0.750731** | +120.03% |
| 20 项算术平均 IPC | 0.450176 | **0.792596** | +76.05% |
| CR1 合计 | 3,891,095 | **3,761,794** | -129,301 |

逐项结果：

| 测试 | 退休指令 | 周期 | IPC | CR1 |
|---|---:|---:|---:|---:|
| bitcount | 50,063 | 66,239 | 0.755793 | 23,049 |
| bubble_sort | 116,649 | 226,629 | 0.514713 | 180,722 |
| coremark | 406,483 | 583,388 | 0.696763 | 314,458 |
| crc32 | 195,419 | 154,238 | 1.266996 | 102,056 |
| dhrystone | 189,115 | 454,090 | 0.416470 | 3,805 |
| fireye_A0 | 550,854 | 764,235 | 0.720791 | 644,389 |
| fireye_B2 | 57,299 | 80,462 | 0.712125 | 36,112 |
| fireye_C0 | 159,469 | 196,719 | 0.810644 | 118,038 |
| fireye_D1 | 425,394 | 580,245 | 0.733128 | 186,422 |
| fireye_I2 | 304,812 | 254,767 | 1.196434 | 205,633 |
| inner_product | 471,055 | 638,602 | 0.737635 | 595,774 |
| lookup_table | 152,598 | 189,682 | 0.804494 | 144,511 |
| loop_induction | 551,328 | 473,789 | 1.163657 | 430,422 |
| minmax_sequence | 279,011 | 277,804 | 1.004345 | 233,409 |
| my_memcmp | 155,068 | 166,684 | 0.930311 | 119,683 |
| quick_sort | 163,304 | 238,852 | 0.683704 | 182,554 |
| select_sort | 98,113 | 113,627 | 0.863466 | 67,397 |
| sha | 207,530 | 249,707 | 0.831094 | 145,824 |
| stream_copy | 33,663 | 59,816 | 0.562776 | 9,621 |
| stringsearch | 346,256 | 775,360 | 0.446574 | 17,915 |

最终性能日志：

```text
D:\frontend\.codex_work\nscscc_cpu5\logs\perf80_fbcut
```

### 2.2 12.5 ns OOC 综合

最终源码使用 Vivado 2023.2、12.500 ns 时钟约束进行
`core_top` out-of-context 综合：

| 指标 | 最终值 |
|---|---:|
| WNS | **+0.228 ns** |
| TNS | **0.000 ns** |
| 失败端点 | **0** |
| 最差数据路径延迟 | 12.226 ns |
| 最差路径 | BPU PC → BPU next PC |

资源统计：

| 资源 | 使用量 | 器件占比 |
|---|---:|---:|
| Slice LUT | 63,964 | 47.52% |
| Slice Register | 27,851 | 10.35% |
| Block RAM Tile | 81.5 | 22.33% |
| DSP | 4 | 0.54% |

最终综合报告：

```text
D:\frontend\.codex_work\nscscc_cpu5\vivado_perf80_final_fbcut\timing_synth.rpt
D:\frontend\.codex_work\nscscc_cpu5\vivado_perf80_final_fbcut\utilization_synth.rpt
D:\frontend\.codex_work\nscscc_cpu5\vivado_perf80_final_fbcut\core_top_synth.dcp
```

OOC 报告使用未布局网表，route delay 是综合器估算值。它证明当前 RTL
在该综合模型下已经满足 80 MHz，但不能替代比赛工程的完整 place/route
报告。最终上板频率仍以实际工程的布局布线 WNS 为准。

## 3. 主要性能优化

### 3.1 片上 RAM 安全 cacheable 提升

启动代码在 DA/DMW/direct 等阶段可能以 MAT=0 访问片上 RAM。当前版本只把
物理地址 `0x1c000000~0x1c0fffff` 白名单提升为 cacheable，MMIO 和其他地址
仍严格服从软件 MAT 设置。取指和数据访问均覆盖该规则。

这是本轮 IPC 提升最大的来源。该地址范围对应当前 NSCSCC/chiplab 映射；
移植到其他 SoC 时必须重新核对。

修改文件：

- `priv/tlb_manager.v`
- `mycpu.h`

### 3.2 IFU 同 Cache Line 复用

IFU 增加一项物理行响应复用缓冲，保存：

```text
{物理行地址, 256-bit 行数据, 8-bit 字有效掩码}
```

连续 FTQ 块位于同一 32 字节行时，不重复访问 ICache/AXI。uncached 响应只把
真实返回的字位置为有效，防止回跳时误用补零数据。`cacop`、flush 和异常路径
会使相关状态失效。预译码仍只读取已寄存的 `if_rline`，没有重新引入
ICache data_ok → predecode 的组合旁路。

修改文件：

- `frontend/ifu.v`
- `mycpu_top.v`

### 3.3 四 bank IB 与存储条目 FWFT

Instruction Buffer 仍由四个 bank 构成，并保留对已存条目的组合读/FWFT 输出。
关闭的只有“IB 为空时，把 IFU 本拍 push 直接送到 decoder/rename 并同拍 pop”
这一条零周期空队列直通。

原直通会形成：

```text
IFU 行数据
  → 预译码/有效条数
  → IB 空队列直通
  → decoder/rename ready
  → IB/IFU 计数反馈
```

关闭后，空队列 push 先写入四 bank，下一拍再由正常 FWFT 读出，切断了曾达到
28~31 级逻辑的反馈链。load-use 快速通道补回了主要性能损失。

修改文件：

- `frontend/inst_buffer.v`
- `mycpu.h`

### 3.4 DCache load-hit 后连续接收提交 store

当当前访问确认是 cached load hit 时，本拍 RAM 查询口已经空闲，DCache 可以
同拍接收下一笔已提交 Store Buffer 请求并启动查询，避免固定 IDLE 气泡。
miss、uncached、cacop、同 set 冲突和未提交 store 顺序保护保持原语义。

修改文件：

- `memory/dcache.v`

### 3.5 LSU 写回隔离与选择性 load-use 快速通道

普通 LSU 写回总线打一拍后再广播到 Reservation Station，避免 DCache hit
直接进入全部唤醒和选择树。为了保留 load-use 性能，增加不写 RS 状态的
一次性快速候选：

- ALU0/ALU1：接收 LSU hold 或当前 DCache 数据 ready；
- MDU：只接收已寄存的 LSU hold 数据，并且只允许 FIFO 队头消费；
- MEM RS：不接快速通道，避免恢复 DCache → 地址生成器关键路径。

快速通道表达“数据已经可用”，不再表达“本拍赢得唯一正式写回口”。因此旧的
MSHR 仲裁状态不会进入 ALU/MDU 选择树；即使更老的 MSHR 返回占用正式写回口，
已准备好的 hold/DC 数据仍可供依赖指令做安全的提前唤醒。下一拍普通写回负责
持久化 RS ready/data。

ALU 普通候选和 load-use 候选按年龄公平选择，避免年轻 load-use 长期压制老项。

修改文件：

- `backend/execute/lsu.v`
- `backend/issue/rs_alu.v`
- `backend/issue/rs_mdu.v`
- `backend/execute/fu_alu.v`
- `mycpu_top.v`
- `mycpu.h`

### 3.6 ROB 早完成与读口转发解耦

ROB 仍在 LSU 原始完成拍记录 load/store 结果、地址、异常并置 complete，减少
退休等待；但四个 dispatch 读口只使用打一拍后的 LSU 总线做同拍转发。

新派发指令最早在下一拍才能发射，届时流水写回已经可用，因此该拆分没有引入
额外 load-use 等待，同时切断：

```text
DCache → ROB read forwarding → rename → RS
```

修改文件：

- `backend/commit/rob.v`
- `mycpu_top.v`

### 3.7 无需执行项由 ROB 静态字段完成

ROB 根据已有字段识别三类无需执行即可完成的项：

- 静态异常；
- DBAR/IBAR，屏障动作仍由提交端执行；
- 无寄存器、访存、分支和特权副作用的空操作。

rename 不再把最后一类送入执行单元，避免提前退休后的迟到写回污染复用 ROB
项。该实现不新增 `is_nop` 存储字段。

修改文件：

- `backend/commit/rob.v`
- `backend/rename/rename.v`

### 3.8 FTB 冷启动

FPGA bitstream 已初始化存储内容。FTB valid 上电直接为无效，不再串行清空
2048 个 set，也不在这段时间丢弃查询和训练。

修改文件：

- `frontend/ftb.v`
- `mycpu.h`

## 4. 关键时序修复

### 4.1 TLB 32 项最低命中编码器分组

原 s0/s1/srch 端口使用 32 项串行优先编码。现在精确改为两级结构：

```text
每 8 项局部最低命中 → 4 个 group 最低命中 → 最终索引
```

最低索引优先规则不变，20 项性能周期不变。该改动消除了
FTQ/iTLB refill clock-enable 的全局最差路径。

修改文件：

- `priv/tlb.v`

### 4.2 IFU 预译码到 TAGE 历史折叠切断

旧逻辑用 `predec_redirect` 组合取消 P1 结果，而同一 P1 valid 又进入
112 位 GHR 折叠、TAGE 索引和写转发碰撞检测，形成 IFU 行数据到 TAGE
寄存器的 23 级路径。

现在用于下一次查询的 P1 历史前递只由已寄存 P1 事件生成，不依赖 IFU
预译码取消。预译码重定向拍本来就不分配 P0，TAGE checkpoint 恢复在时钟
边沿具有更高优先级，因此被丢弃查询的语义不变。

修改文件：

- `frontend/bpu.v`

### 4.3 预译码训练到 uBTB 写使能切断

预译码训练已经先进入两项 FIFO，但旧的“无更新”多路器默认值仍选择 IFU
实时目标。虽然该拍写使能为 0，综合器仍把无效目标带入 uBTB 替换判断。

现在无更新分支使用常量 0；有效提交更新和 FIFO 出队更新完全不变。20 项
周期逐项不变。

修改文件：

- `frontend/bpu.v`

### 4.4 MSHR/当前 DCache 数据到执行单元切断

最终结构分成两条内部 ready 总线：

- ALU 使用 hold + 当前 DCache ready，但不依赖 MSHR 写回仲裁；
- MDU 只使用已寄存 hold，不直接消费 DCache tag/data 路径。

它消除了 MSHR → ALU 和 DCache request/tag → MDU 的长组合路径，同时把
加权 IPC 保持在 0.75 以上。

修改文件：

- `backend/execute/lsu.v`
- `mycpu_top.v`

### 4.5 FTQ backpressure 到 BPU next-PC 切断

原 `ftq_full` 同时门控组合式 fallback BTB 查询。ROB head 影响提交释放，
提交释放影响 FTQ full，随后又通过 fallback hit/块长进入 `PC + length`，
形成 26 级路径。

冻结拍的 PC 和 P0 分配本来就被禁止，因此只让组合式 fallback BTB 保持无害
查询；uBTB、FTB 和 TAGE 仍按原 `query_en` 节拍运行。这样既切断 full →
fallback length → next-PC，又保持 20 项周期逐项不变。

修改文件：

- `frontend/bpu.v`

## 5. 未采用的实验

- 全部 RS 直接使用原始 LSU 写回：IPC 可到约 0.768，但恢复
  DCache → MEM/LSU/ALU 长路径，不作为可综合版本。
- 完全保留空 IB 同拍直通：IPC 略高，但曾产生 IFU → IB `-5.115 ns`、
  IFU → ROB `-3.720 ns` 的长反馈路径。
- 所有预测器在 FTQ freeze 时继续查询：功能测试能结束，但改变 FTB/TAGE
  响应节拍，20 项周期明显增加，因此只保留无状态的 fallback 查询解耦。
- MDU 直接消费当前 DCache hit：能减少约 6.7k 总周期，但产生
  DCache tag/data → MDU 约 `-1.0 ns` 路径；最终改为 hold-only。
- `CACOP_NO_REFETCH`：收益接近零且增加体系结构语义风险，最终关闭。
- ITTAGE/JTC：当前竞赛测试普通 JIRL 收益过低，最终 RTL 不实例化这些模块。

## 6. 仿真与交付边界

- `nscscc_perf`：20/20 已完成，结果见第 2 节。
- 官方功能测试曾连续运行 120M 周期，停留于程序的 `idle_1s` 合法等待循环，
  未观察到随机跑飞；但该次运行没有到达测试平台的完整结束条件，因此本文不把
  它表述为“完整功能测试通过”。
- 所有统计计数器均位于 `synthesis translate_off/on` 区域，不进入综合资源。
- 最终仍应在实际比赛工程中重新跑功能仿真、综合和布局布线；若最终 P&R
  与 OOC 结果不同，应以实际工程报告为准。
