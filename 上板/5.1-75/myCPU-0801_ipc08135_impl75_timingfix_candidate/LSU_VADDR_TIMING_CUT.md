# LSU 虚地址关键路径切分说明

## 版本

- 基线 CPU：`D:\frontend\myCPU-0729_frontend_timing_merge`
- 修改后 CPU：`D:\frontend\myCPU-0729_frontend_timing_merge_lsu_vaddr_cut`
- 输入布线报告：`D:\report\reports_impl-0730-1-70`
- 目标频率：70 MHz，时钟周期 14.286 ns

## 报告中的最差路径

布线后最差 CPU 路径为：

```text
u_cpu/u_lsu/a_base_reg[1]
    -> 32 位 base + imm 加法器
    -> DTLB 全相联查询、命中选择和 L1 TLB 选择
    -> u_cpu/u_lsu/d_paddr_reg[15]
```

报告数据：

| 项目 | 原布线结果 |
|---|---:|
| WNS | -0.906 ns |
| 数据路径延迟 | 15.200 ns |
| 逻辑级数 | 23 |
| 主要逻辑 | 9×CARRY4、9×LUT6，以及 TLB 选择逻辑 |

同一条结构还造成 `d_paddr` 其他位、`d_excp` 和 `d_uncached` 的多条违例，所以应切断共同的前半段，而不是只改某一个末端位。

## RTL 修改

只修改了：

```text
backend\execute\lsu.v
```

原流水：

```text
issue -> [锁存 base、imm] -> base+imm -> DTLB -> [锁存 paddr/异常]
```

修改后：

```text
issue -> base+imm -> [锁存 vaddr] -> DTLB -> [锁存 paddr/异常]
```

具体变化：

1. 删除 AGU 级的 `a_base`、`a_imm` 两组 32 位寄存器。
2. 增加一组 32 位 `a_vaddr` 寄存器。
3. 在原来的 issue→AGU 寄存器边界锁存 `issue_base_i + issue_imm_i`。
4. AGU→DC 这一拍只保留地址翻译、异常判断、uncached 判断和 store 对齐。

该修改没有增加流水级，没有改变 LSU、CPU 顶层或其他模块的端口，也没有改变访存指令的可见延迟。寄存器位数由 `base + imm` 的 64 位减少为 `vaddr` 的 32 位。

## 正确性与性能回归

- Icarus 全 CPU `core_top` 编译通过。
- Verilator 全 SoC 模型构建通过。
- `nscscc_perf` 20/20 全部正常结束。
- 20 项合计 cycles：14,329,010。
- 20 项合计 CR0：3,886,134。
- 20 项合计 CR1：3,891,095。
- 每一项测试的 cycles、CR0、CR1 均与基线版本逐项完全相同。
- RTL 文件 SHA-256 对比确认：除 `backend\execute\lsu.v` 外，其余 RTL 均未变化。

因此，这一版属于纯时序重定时：保留了基线的功能、IPC 和分数，不用性能换时序。

## 综合检查

使用 Vivado 2023.2、`xc7a200tfbg676-2`、70 MHz 对完整 `core_top` 做了相同条件的 OOC 综合。定向结果如下：

| 路径 | 基线余量 | 修改后余量 | 修改后状态 |
|---|---:|---:|---|
| issue/RS → AGU 边界数据寄存器 | +6.024 ns | +4.377 ns | 满足 |
| AGU 边界 → `d_paddr` | +2.569 ns | +4.153 ns | 满足 |
| `a_vaddr` → `d_excp` | — | +4.392 ns | 满足 |
| `a_vaddr` → `d_uncached` | — | +4.880 ns | 满足 |

地址加法被移动后，两侧都保留了明显正余量：

- 前半段新增 32 位加法，数据路径余量仍为 +4.377 ns。
- 包含 CE 在内的所有 `a_vaddr` 输入路径，最小余量为 +2.730 ns。
- 后半段 `a_vaddr → d_paddr` 的 OOC 余量提高 1.584 ns。
- 后半段数据延迟由 11.671 ns 降为 10.087 ns。
- 后半段逻辑级数由 24 降为 15。
- 后半段 CARRY4 数量由 9 降为 1；剩余 1 个属于翻译/选择逻辑，不再是 32 位虚地址加法链。

OOC 资源对比：

| 资源 | 基线 | 修改后 | 变化 |
|---|---:|---:|---:|
| Slice LUT | 65,323 | 65,273 | -50 |
| Flip-Flop | 27,695 | 27,652 | -43 |
| Block RAM Tile | 81.5 | 81.5 | 0 |
| DSP | 4 | 4 | 0 |

除直接减少 32 位寄存器外，综合器还消除了少量关联逻辑，因此最终 FF/LUT 降幅略大。

完整 OOC 的全局 WNS 仍为 -1.513 ns，路径是 `u_ifu/if_rline_reg[254] → u_rob/complete_reg[16]`，与基线 OOC 完全相同。它不是本次修改产生的路径，也不是输入的完整工程布线报告中的最差路径；因此不能用这一条 OOC 路径否定 LSU 定向切分结果。

## 后续布线验收重点

重新综合布线时应重点检查：

1. 原来的 `a_base_reg -> d_paddr_reg/d_excp_reg/d_uncached_reg` 路径应消失。
2. 新的 `a_vaddr_reg -> d_paddr_reg` 路径不应再包含 32 位地址加法的 CARRY4 链。
3. 新增的 issue/RS→`a_vaddr_reg` 地址加法路径必须满足 14.286 ns。
4. 如果 LSU 路径退出最差路径，下一候选很可能变为 DCache 请求地址到 `rs_mem` 唤醒/使能路径；应以新布线报告为准再优化。

注意：OOC 综合只能验证逻辑结构已经切开，不能替代完整工程的布局布线结果。
