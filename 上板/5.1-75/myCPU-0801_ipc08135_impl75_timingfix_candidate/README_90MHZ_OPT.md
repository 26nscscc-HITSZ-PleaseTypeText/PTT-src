# myCPU-0731 IPC / 90 MHz 候选版说明

## 1. 版本定位

本目录是一份完整 CPU RTL，顶层仍为 `core_top`，没有修改 CPU 与
SoC、AXI、时钟复位及中断接口。它保留了原有双发射主线、四 bank
指令缓冲、FWFT、两 bank uBTB、TAGE、FTB、RAS 和原后端功能，并在
此基础上针对 90 MHz 综合关键路径做了小范围、可恢复的时序切断。

这个版本没有 ITTAGE 模块，也没有 JTC 模块。

本次本地验证目标器件为 `xc7a200tfbg676-2`，时钟约束为
`11.111 ns`。XDC 在 `synth_design` 前读入，避免了“无约束综合后再
创建时钟”造成的虚假时序结果。

## 2. 验证结论

| 项目 | 结果 |
|---|---:|
| `nscscc_perf` 完成数 | 20 / 20 |
| 仿真超时 | 0 |
| 20 项总周期 | 6,038,719 |
| 20 项总退休指令 | 4,913,835 |
| 20 项加权 IPC | 0.813721 |
| 18 项计算口径 IPC | 0.906472 |
| 90 MHz 综合 WNS | +0.120 ns |
| 90 MHz 综合 TNS | 0.000 ns |
| 失败端点 | 0 |

“18 项计算口径”沿用前面实验的统计方式，单列启动/系统行为占比较高
的 `dhrystone` 和 `stringsearch`。它不是 20 项官方总 IPC，因此不能
用 0.906472 替代 0.813721。

20 项结果如下：

| 测试 | 周期 | 退休指令 | IPC |
|---|---:|---:|---:|
| bitcount | 62,540 | 50,075 | 0.800688 |
| bubble_sort | 195,638 | 116,652 | 0.596265 |
| coremark | 527,253 | 406,785 | 0.771518 |
| crc32 | 147,564 | 195,443 | 1.324463 |
| dhrystone | 447,175 | 189,060 | 0.422787 |
| quick_sort | 234,640 | 163,325 | 0.696066 |
| select_sort | 101,747 | 97,990 | 0.963075 |
| sha | 222,108 | 207,563 | 0.934514 |
| stream_copy | 56,708 | 33,684 | 0.593990 |
| stringsearch | 761,253 | 346,253 | 0.454846 |
| fireye_A0 | 583,688 | 550,875 | 0.943783 |
| fireye_B2 | 75,960 | 57,308 | 0.754450 |
| fireye_C0 | 186,875 | 159,481 | 0.853410 |
| fireye_D1 | 546,356 | 425,406 | 0.778624 |
| fireye_I2 | 235,317 | 304,830 | 1.295402 |
| inner_product | 614,439 | 471,070 | 0.766667 |
| lookup_table | 169,805 | 152,607 | 0.898719 |
| loop_induction | 460,530 | 551,331 | 1.197166 |
| my_memcmp | 140,102 | 155,083 | 1.106929 |
| minmax_sequence | 269,021 | 279,014 | 1.037146 |

## 3. 前端保留与新增优化

### 3.1 四 bank IB + FWFT

`frontend/inst_buffer.v` 保留四 bank 组织和空队列 FWFT。占用状态改为
one-hot 编码，用移位完成每拍 `push-pop` 更新，去掉 rename-ready 到
IB 二进制加减计数器的长组合链。二进制 `count` 只作为仿真统计探针，
不参与运行时反压决策。

### 3.2 两 bank uBTB

`frontend/ubtb.v` 保留两个 16 项 bank，每次 P0 只查询一个 bank，
因此仍然只有 16 路并行比较，而不是 32 项大全相联。条件分支使用
2 位饱和计数器；RET 的目标仍由 RAS 覆盖。

uBTB tag 使用 `PC[21:2]` 共 20 位，覆盖 1 MiB 代码窗口，减少 P0 CAM
比较宽度。窗口外发生别名时只会造成可恢复的性能误预测，P1/提交会
纠正，不改变架构正确性。

### 3.3 TAGE 与 112 位 GHR

TAGE 历史长度保持 `11/23/53/112`，GHR 长度为 112 位。训练侧保留
必要的待写和上一拍写回旁路；查询侧去掉 tag RAM 同拍写回旁路，
直接使用 BRAM 查询结果，从而切断 TAGE 结果到下一次索引/冲突判断
的环路。该变化在 20 项测试中只有极小、可恢复的预测时序差异。

### 3.4 RAS / P1 控制重定时

`frontend/bpu.v` 用寄存后的 `p0_wrote_r` 推进 RAS 的 FTQ 镜像指针，
checkpoint ID 直接取当前分配指针。这样不再让 FTB/P1 组合结果直接
控制 RAS 指针寄存器 CE，同时保持 FTQ checkpoint 对齐。

BPU 的 PC 寄存器增加 `EXTRACT_ENABLE="no"`，防止 Vivado 把宽 PC
寄存器抽取成不利于当前关键路径的结构。

### 3.5 FTB 直接保存 fall-through 地址

这是本版最后一项 90 MHz 定向修改。旧实现从 FTB BRAM 读出 block
length 后，在查询路径计算：

```text
fall-through = block_pc + block_length * 4
```

该 32 位加法进入 BPU PC 寄存器 D 端，是上一版 90 MHz 约束综合的
最差路径。现在 FTB 在训练写入时同时保存完整 `fall-through`，查询
时直接从 BRAM 取出，移除了查询关键路径上的宽加法器。

这项修改不增加查询拍数，20 项周期与修改前逐项相同。代价是 FTB
entry 变宽，综合 BRAM 增加 6 块。

### 3.6 FTQ 提交目标读取

`frontend/ftq.v` 的提交训练目标直接用本地 `cmt_ptr` 选择，避免外部
query ID 再形成一条选择路径。`SIMU` 下保留 query ID 与 `cmt_ptr`
一致性断言，便于发现提交协议错误。

## 4. 后端时序/性能优化

### 4.1 选择性 load→store-data 快旁路

`RS_MEM_STORE_DATA_FAST_BYPASS=1`，但
`RS_MEM_LOAD_FAST_BYPASS=0`。load 结果可以提前唤醒 store payload，
不能提前唤醒 AGU base，因此既缩短常见 load→store 依赖，又不把
未经确认的数据送入地址生成路径。

### 4.2 Store Buffer 合并比较切断

`memory/store_buffer.v` 在入队时保存精确的 `merge_with_prev` 位，
记录当前项是否与物理前一项属于同一 cache line。排空时使用这个
邻接位，不再在多级合并链每一级重复做 27 位地址 tag 比较。

该位处理了 pop+push、满队列和同拍清除边界；只改变比较发生的时间，
不放宽实际合并条件。

### 4.3 Rename 配对分类切断

`backend/rename/rename.v` 的双发射资源配对只使用已经译码的静态类型，
不再让异常/无需执行判断进入 slot1 RAT/ROB 分配长路径。精确 CSR
drain 条件仍使用有效指令条件，因此只是更保守地阻止配对，不会放行
非法的 MEM+MEM、MDU+MDU 或 CSR 配对。

### 4.4 CSR 异常类别直接选择

`priv/csr_exception_commit_handler.v` 直接按照与异常编码器相同的优先级
生成 TLB、BADV、TLBEHI 和异常入口选择信号，避免 ROB 异常向量先编码
成 Ecode、随后再经过多组 Ecode 比较器。已对 16,384 种异常输入组合
做等价枚举，差异为 0。

## 5. 90 MHz 约束综合资源

| 资源 | 使用量 | 器件占比 |
|---|---:|---:|
| Slice LUT | 63,747 | 47.36% |
| Slice Register | 29,147 | 10.83% |
| Block RAM Tile | 87.5 | 23.97% |
| DSP | 4 | 0.54% |

与保存 fall-through 前的约束综合相比：

| 指标 | 修改前 | 当前 |
|---|---:|---:|
| WNS | +0.034 ns | +0.120 ns |
| TNS | 0.000 ns | 0.000 ns |
| LUT | 63,724 | 63,747 |
| Register | 29,117 | 29,147 |
| BRAM Tile | 81.5 | 87.5 |

原 FTB BRAM→BPU PC 的最差路径已经消失。当前综合最差路径转移到
FTQ/TLB refill 相关组合逻辑。

## 6. 报告与复现

本目录的 `reports_90m_ooc` 包含：

- `cpu_90m.xdc`
- `timing_synth.rpt`
- `timing_paths.rpt`
- `utilization_synth.rpt`

综合必须保证 `read_xdc cpu_90m.xdc` 位于 `synth_design` 之前。

本地使用的是 out-of-context 综合。它证明 RTL 在当前器件、当前
综合映射下满足 90 MHz，但不能替代完整 SoC 的 place/route。最终是否
稳定达到 90 MHz，应以你工程中的实现后 WNS/TNS、时钟偏斜、拥塞和
最差路径为准。

如果另一台电脑在 `synth_design` 偶发
`EXCEPTION ACCESS_VIOLATION`，先检查 Vivado 安装、自带 Tcl 文件读取、
TEMP 路径及 `hs_err_<pid>.dmp`；本地遇到过 Vivado 读取自身运行库
失败，而不是 RTL 语法错误。本版 RTL 已完成一次 0 error 的约束综合。
