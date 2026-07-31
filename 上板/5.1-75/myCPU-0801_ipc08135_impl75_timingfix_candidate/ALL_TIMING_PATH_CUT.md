# 70 MHz 时序路径集中修复说明

## 版本关系

- 输入版本：`D:\frontend\myCPU-0729_frontend_timing_merge_lsu_vaddr_cut`
- 输出版本：`D:\frontend\myCPU-0729_frontend_timing_merge_all_path_cut`
- 分析报告：`D:\report\reports_impl-0730-1-70`
- 目标周期：14.286 ns（70 MHz）

原实现报告为 WNS = -0.906 ns、TNS = -330.158 ns、1331 个 setup
失败端点。`timing_setup_violations.rpt` 给出的前 200 条代表性违例可归并为
三个根因，而不是 200 个互不相关的问题：

| 路径根因 | 前 200 条中的数量 | 本版处理 |
|---|---:|---|
| LSU `base + imm` 后继续经过 DTLB/TLB 组合翻译 | 150 | 沿用上一版 `a_vaddr` 提前寄存切分 |
| DCache 当前请求同行判断，经 CWF 返回影响 LSU ready/RS 写使能 | 40 | 同行判断提前到请求接受拍寄存 |
| FTQ 指针/PC 经主 TLB 透传腿进入 ICache 请求地址 | 10 | 新增独立 direct paddr/mat 快速锥 |

`design_analysis_timing.rpt` 较靠后的 DCache `valid_arr/dirty_arr` 动态写路径
也一并处理，避免前三类修复后它立即成为新的最差路径。

## RTL 修改

### 1. 保留 LSU 虚地址提前寄存

`backend/execute/lsu.v` 继续在 AGU 接受拍计算并寄存
`issue_base_i + issue_imm_i`。下一拍 DTLB 从寄存的 `a_vaddr` 启动，切断
原报告中 9 级 CARRY4 地址加法与 TLB 全相联查询串在同一周期的问题。

### 2. DCache 同行 MSHR 判断前移

文件：`memory/dcache.v`

原路径为：

```
req_paddr
  -> 与在飞 MSHR 做整行地址比较
  -> 同拍 store-merge / CWF early 判定
  -> ld_mshr_data_ok
  -> LSU 完成与 ready
  -> rs_mem 发射和条目写使能
```

本版在 DCache 接受 store 请求时，就把它与两个在飞 MSHR 的同行结果寄存为
`req_mshr_line_match_r`。MSHR 的物理行地址在 busy 期间不变，所以该前移与
原下一拍 LOOKUP 比较等价。LOOKUP、同拍 store merge、CWF 提前返回和
refill 数据叠层仍保持原行为，不增加等待拍。

这比简单把全部 MSHR 返回寄存一拍更合适：既切断跨模块长路径，也保留
load miss 的提前返回性能。

### 3. DCache valid/dirty 更新包寄存

文件：`memory/dcache.v`

原先 LOOKUP 的 tag hit、动态 way/set 选择直接驱动 4×128 项
`valid_arr/dirty_arr` 的分散 FF D 端，布线扇出很大。

本版把更新分成两个小包：

- `meta_install_*`：MSHR 安装产生的 valid/dirty 更新；
- `meta_front_*`：store hit 或 cacop 产生的更新。

先把 `{way, set, write-enable, value}` 收敛到局部寄存器，下一拍再落元数据
阵列。新请求最早在同一拍被接受、再下一拍才进入 LOOKUP，因此更新对下一次
查询仍按时可见。两个更新包同拍落同址时仍保持 front 更新优先，和原
`always` 块的赋值顺序一致。

### 4. IFU 同拍直发使用独立快速物理地址

涉及文件：

- `priv/l1_tlb.v`
- `priv/tlb_manager.v`
- `priv/mmu.v`
- `frontend/ifu.v`
- `mycpu_top.v`

旧版虽然用 `inst_direct_ok` 保证 FTQ 同拍直发只发生在 DA、DMW 或 L1 TLB
命中，但直发的数据仍复用了完整 `inst_paddr`。完整地址在 L1 miss 时包含
32 项主 TLB 透传腿，静态时序不会根据 `direct_ok` 推导两条腿功能互斥，
因此仍报告出 FTQ→主 TLB→ICache 的长路径。

本版从 L1 TLB 单独引出 CAM 命中项的 `ppn/mat`，构造：

- `inst_direct_paddr`
- `inst_direct_mat`

IFU 的 FTQ 同拍 ICache 请求只使用这组快速结果；PRE 级仍锁存完整
`inst_paddr/inst_mat`，所以 L1 miss、主 TLB 翻译和异常处理行为不变。
`IFU_FTQ_DIRECT` 仍保持开启，没有通过关闭快速取指换时序。

## 功能与性能回归

### RTL 编译

- Icarus Verilog：完整 `core_top` 编译通过；
- Verilator：完整 SoC 性能模型构建通过。

### nscscc_perf

20 项全部 `FINISH`，无超时；与输入版本逐项比较：

| 指标 | 输入版本 | 本版 | 差值 |
|---|---:|---:|---:|
| 20 项总 cycles | 14,329,010 | 14,329,010 | 0 |
| 20 项总 CR0 | 3,886,134 | 3,886,134 | 0 |
| 20 项总 CR1 | 3,891,095 | 3,891,095 | 0 |
| 不一致测试数 | 0 | 0 | 0 |

说明本版没有通过插入固定气泡换取时序，现有仿真性能完全保留。

## 针对性综合结果

使用 XC7A200T-2、14.286 ns 时钟对完整 `core_top` 进行综合，并按原实现报告的
源/终点单独查询：

| 原报告路径族 | 原 70 MHz 布线结果 | 本版综合结果 |
|---|---:|---:|
| DCache `req_paddr` → RS_MEM | -0.634 ns（代表路径） | +3.813 ns，MET |
| FTQ `bpu_ptr` → ICache `req_paddr` | -0.619 ns（代表路径） | +4.825 ns，MET |
| DCache `req_paddr` → `valid_arr/dirty_arr` | 约 -0.560 ns | No paths found |

其中 DCache → RS_MEM 的代表路径由 23 级、14.382 ns 缩短为 16 级、
10.233 ns；FTQ → ICache 代表路径为 16 级、9.221 ns。综合资源对比输入版本：

| 资源 | 输入版本 | 本版 | 差值 |
|---|---:|---:|---:|
| Slice LUT | 65,273 | 61,872 | -3,401 |
| Slice Register | 27,652 | 27,630 | -22 |
| Block RAM Tile | 81.5 | 81.5 | 0 |
| DSP | 4 | 4 | 0 |

综合后全局最差路径是 IFU 取回数据经过指令解码，影响 IB/RAT 控制的跨层路径；
输入版本的本地综合最差路径同样从 IFU 取回数据出发并穿过后端控制。该类未布局
跨层路径的线延迟占比很高，不是原实现报告列出的三个根因，不能用本地综合 WNS
代替最终布线结论。

## 综合和布线验收

最终 WNS/TNS 必须以完整 SoC 重新综合布线后的报告为准。建议重点检查：

1. 不再出现 `u_lsu/a_base_reg -> u_lsu/d_paddr/d_excp`；
2. DCache → RS_MEM 路径不再经过当前请求与 MSHR 的同拍整行比较；
3. FTQ → ICache 路径不再经过主 TLB；
4. 不再存在 DCache `req_paddr -> valid_arr/dirty_arr` 的直接路径。
