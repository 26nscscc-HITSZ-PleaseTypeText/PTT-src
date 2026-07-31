# myCPU-0729 后端 + 优化前端合并说明

## 交付位置

- 合并版：`D:\frontend\myCPU-0729_frontend_timing_merge`
- 后端基线：`D:\frontend\myCPU-0729`
- 前端来源：`D:\frontend\myCPU_5_frontend_opt_hybrid_timing_fix_ras_cut_tage_t3_timing\myCPU`

两个源目录均未修改。

## 合并边界

合并版以 `myCPU-0729` 为整机基线，以下内容逐文件保持为
`myCPU-0729` 版本：

- `backend/` 全部模块；
- `memory/` 全部模块；
- `priv/` 全部模块；
- `mycpu.h` 全局配置；
- CPU 对外 AXI、调试和中断接口。

以下内容采用优化前端版本：

- `frontend/bpu.v`
- `frontend/ftq.v`
- `frontend/ifu.v`
- `frontend/inst_buffer.v`
- `frontend/ftb.v`
- `frontend/tage.v`
- `frontend/ubtb.v`
- `frontend/fallback_btb.v`
- `frontend/icache.v`
- `frontend/ras.v`

`mycpu_top.v` 只使用优化前端所需的内部连线版本，后端实例及其接口保持兼容。
两版 `core_top` 的 54 个对外端口完全一致。

`myCPU-0729/mycpu.h` 中的 JTC 宏被原样保留，但优化前端不再实例化 JTC，
因此这些未使用宏不会生成硬件资源。

## 保留的前端优化

1. 四 bank + FWFT Instruction Buffer；
2. Instruction Buffer 自然变空时不再清零 head/tail，切断 FWFT 到指针复位的长路径；
3. 两个 16 项 bank 的 uBTB，并保留 2 位方向计数器；
4. 64 项轻量 fallback BTB；
5. FTB 更新过滤、队尾同 PC 合并和统计；
6. P1 稳定描述符写入与注册 retry 槽；
7. RET/JIRL 作为真实控制流边界，RAS 目标比较不再影响 IB 入队数量；
8. GHR 112 位，TAGE 历史长度保持 11/23/53/112；
9. TAGE 查询碰撞寄存和 T3 训练写回切拍；
10. 删除低收益 JTC，保留 RAS 对 RET 的目标覆盖。

## 接口和结构检查

- Verilog 文件：41；
- 模块：48；
- 重名模块：0；
- `backend/`、`memory/`、`priv/` 与 `myCPU-0729` 哈希完全一致；
- `frontend/` 与优化前端版本哈希完全一致；
- Icarus Verilog 完整 `core_top` 编译通过；
- Verilator 完整 SoC 性能模型编译通过。

## nscscc_perf

原 `myCPU-0729` 与合并版使用同一测试平台、同一 MIF 和同一最大周期，
20 项测试均 `FINISH` 且 `led=ffff`。

| 指标 | myCPU-0729 | 合并版 | 变化 |
|---|---:|---:|---:|
| 20 项 CR1 合计 | 4,156,494 | 3,891,095 | -265,399（-6.3852%） |
| 仿真总周期 | 14,712,969 | 14,329,010 | -383,959（-2.6097%） |
| 加权 IPC | 0.332262 | 0.341191 | +2.6872% |
| 全分支准确率 | 95.7921% | 97.6033% | +1.8112 pp |
| 条件分支准确率 | 96.4455% | 96.6718% | +0.2263 pp |
| 全分支误预测 | 49,972 | 28,468 | -21,504 |
| IB empty 周期 | 8,503,655 | 8,243,398 | -260,257 |
| 预译码重定向 | 70,082 | 9,198 | -60,884 |

20 项 CR1 均改善，没有单项退化。

## 单项 CR1

| 测试 | myCPU-0729 | 合并版 | 变化 |
|---|---:|---:|---:|
| bitcount | 35,268 | 23,561 | -11,707 |
| bubble_sort | 193,597 | 191,672 | -1,925 |
| coremark | 325,853 | 319,409 | -6,444 |
| crc32 | 185,587 | 104,349 | -81,238 |
| dhrystone | 4,917 | 4,108 | -809 |
| quick_sort | 204,050 | 186,113 | -17,937 |
| select_sort | 73,560 | 68,408 | -5,152 |
| sha | 170,652 | 158,784 | -11,868 |
| stream_copy | 11,657 | 11,608 | -49 |
| stringsearch | 24,631 | 19,475 | -5,156 |
| fireye_A0 | 687,123 | 686,833 | -290 |
| fireye_B2 | 37,872 | 37,163 | -709 |
| fireye_C0 | 126,976 | 120,937 | -6,039 |
| fireye_D1 | 193,036 | 189,689 | -3,347 |
| fireye_I2 | 212,710 | 212,467 | -243 |
| inner_product | 606,810 | 606,489 | -321 |
| lookup_table | 160,107 | 145,598 | -14,509 |
| loop_induction | 490,727 | 441,999 | -48,728 |
| my_memcmp | 119,624 | 119,398 | -226 |
| minmax_sequence | 291,737 | 243,035 | -48,702 |

## 70 MHz OOC 综合

器件 `xc7a200tfbg676-2`，时钟约束 14.286 ns。完整 `core_top` 综合通过：

- 0 error；
- 0 critical warning；
- Slice LUT：65,323；
- Slice Register：27,695；
- BRAM Tile：81.5；
- DSP：4。

关键路径定点检查：

| 路径 | 结果 |
|---|---:|
| T2 训练计算 -> T3 写命令 | +6.144 ns |
| T3 写地址 -> TAGE RAM | +12.560 ns |
| 注册查询旁路 -> FTQ `blk_taken` | +5.987 ns |
| 注册查询旁路 -> BPU `pc_reg` | +5.433 ns |
| `t2_meta` -> FTQ `blk_taken` | 无时序路径 |
| `t2_meta` -> BPU `pc_reg` | 无时序路径 |
| `t2_meta` -> TAGE RAM | 无时序路径 |

OOC 全局 WNS 为 -1.513 ns，最差路径为
`IFU if_rline -> ROB complete`，不再是 TAGE 训练反馈。该路径跨越前端、译码和
后端，且 OOC 没有布局信息，最终数值仍以完整 SoC 布局布线为准。

## 综合布线关注点

1. 原 `TAGE t2_meta -> FTQ/BPU PC/TAGE RAM` 组合路径应已被 T3 和查询碰撞寄存切断；
2. 旧 `IFU/RAS -> IB count/head/tail` 路径不应回归；
3. 若前端下一条路径转移到 `FTQ ifu_ptr -> ICache req_paddr`，再考虑 FTQ 头部预取，
   不建议直接关闭 `IFU_FTQ_DIRECT`，因为会损失取指性能；
4. 最终 WNS、TNS、hold 和资源仍以完整 SoC 布局布线报告为准。
