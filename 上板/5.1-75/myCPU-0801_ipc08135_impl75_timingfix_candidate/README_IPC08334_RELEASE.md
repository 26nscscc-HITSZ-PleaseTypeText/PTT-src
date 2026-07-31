# IPC 0.833413 / 90 MHz OOC 候选版本

## 使用状态

- 只读候选目录：`D:\frontend\myCPU-0801_ipc08334_90m_candidate`
- 来源基线：`D:\frontend\myCPU-0731_ipc09_90m_candidate`
- 日期：2026-07-31
- 该目录是已经完成 20 项性能仿真和 90 MHz OOC 综合的固化副本，后续实验不要直接在此目录修改。

## 已验证结果

| 项目 | 结果 |
|---|---:|
| 20 项总周期 | 5,895,800 |
| 20 项退休指令 | 4,913,638 |
| 20 项加权 IPC | 0.833413 |
| 18 项计算阶段周期 | 4,697,482 |
| 18 项计算阶段退休指令 | 4,378,312 |
| 18 项计算阶段 IPC | 0.932055 |
| 20 项结束状态 | 全部 `FINISH`，`LED=ffff` |
| 90 MHz OOC WNS / TNS | +0.273 ns / 0 ns |
| Slice LUT | 66,540 |
| Slice Register | 29,503 |
| BRAM Tile | 87.5 |
| DSP | 4 |

相对原 0731 基线，20 项减少 142,919 周期，IPC 从 0.813721 提升到 0.833413；18 项计算 IPC 从 0.906472 提升到 0.932055。

## 主要改动

1. ROB 头部可在同周期消费已经寄存完成的 MEM writeback，减少 MEM 完成到提交的空拍。
2. Rename/Dispatch 支持一对 MEM 指令同时进入 RS_MEM。
3. RS_MEM 扩为 8 项存储、双入队；发射选择只扫描最老 4 项，保留排队吸收能力，同时切断全 8 项扫描造成的 ALU 到 LSU 长路径。
4. IFU 在取指寄存器边界预先保存对齐后的 4 条指令，而不是保存 256-bit 原始行后再由 `if_pc` 选择，消除了曾导致 OOC WNS -0.226 ns 的 IFU 组合反馈路径。
5. 加入的访存机会计数器均位于仿真专用区域，不进入综合网表。

## 与 75 MHz 实际布线违例的关系

用户提供的 0731 基线布线报告只有两个唯一 setup 失败端点：

- DCache `req_paddr` 到 `mshr_line` CE，WNS -0.056 ns。
- FTQ/IFU 指针到 L1 ITLB refill PPN，slack -0.020 ns。

本候选在更严格的 90 MHz OOC 中没有 setup 失败，最差路径已变成 Instruction Buffer 到 ROB 状态 RAM，slack +0.273 ns。FTQ 到 TLB 的同类路径在报告中为 +0.439 ns；DCache 的原失败路径没有进入最差 500 条路径。

但这不能替代完整 SoC 的 place/route：DCache 原路径并未通过一次新的 75 MHz 实际布线逐条确认。因此准确表述是“90 MHz OOC 已有余量，原两个路径均不再是 OOC 关键路径，预计更容易通过 75 MHz”，不能表述为“实际布线违例已确认全部修复”。

## 验证边界

20 项 `nscscc_perf` 已全部通过。当前没有获得一次完整官方功能平台的统一结束标志，所以不能声称“所有官方功能仿真全部通过”。

## 综合复现

在本目录执行：

```powershell
vivado -mode batch -source synth_90m.tcl -log vivado_synth_90m.log -journal vivado_synth_90m.jou
```

报告位于 `reports_90m_ooc`。

## 下一步优化顺序

1. 先实现“相邻两个普通 cached Load、同一 cache line”的双发射。当前 DCache 每路一次读出完整 256-bit cache line，同一行可由一次 tag/data lookup 提取两个 word；20 项统计有 309,500 次队头相邻同线机会，改动风险低于直接复制完整 DTLB/DCache 端口。
2. 第二阶段实现不同数据 bank 的两个普通 cached Load 双发，需要第二地址翻译通路、第二 DCache 读地址/标签通路和第二完成暂存通路。冲突、MMIO、异常、LL/SC、CACOP、store-order 风险全部自动退回单发。
3. 每一步先跑定向用例和 20 项性能仿真，再跑 90 MHz OOC；只有 IPC 增益且 WNS 非负才保留。最终仍需在另一台电脑做 75 MHz 完整 SoC place/route。
