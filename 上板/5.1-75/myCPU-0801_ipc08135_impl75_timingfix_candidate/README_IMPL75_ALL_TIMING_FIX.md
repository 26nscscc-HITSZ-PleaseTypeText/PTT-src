# 0731-2 75 MHz 布线违例修复版

## 基线与范围

- 只读源基线：`D:/frontend/myCPU-0801_ipc08334_90m_candidate`
- 参考的异机布线报告：`D:/report/reports_impl-0731-2-75`
- 本目录在独立副本中修改，源基线未改动。
- 原报告在 75 MHz 下：setup WNS `-0.676 ns`，TNS `-68.913 ns`，546 个失败端点；hold WHS `+0.052 ns`，无 hold/pulse-width 违例。

原报告前 100 条设计分析路径归并为三类 RTL 根因：

1. ROB `head_reg` 经 slot1 commit/ARF 同拍写穿透，继续进入 rename/RS，最差 `-0.676 ns`。
2. DCache `req_paddr` 经 hit data、LSU load shaping 和 fast bypass 直接进入 RS/ALU，最差 `-0.361 ns`，另有 RS CE `-0.221 ns`。
3. DCache `req_paddr` 经 miss/allocation 控制进入 `mshr_line` CE，`-0.233 ns`。

## RTL 修改

### 1. 去掉功能 ARF 读口的同拍 commit 写穿透

文件：`backend/commit/regfile.v`

八个功能读口直接读取已提交寄存器阵列，不再由 `we0/we1/waddr/wdata` 组合写穿透。提交目的寄存器在该时钟沿之前仍由 RAT 标记为 busy，消费者本拍保留 ROB tag，下一拍读取已经写入 ARF 的值，因此该写穿透是冗余的。DIFFTEST 的沿对齐转发保持不变。

效果：从网表中移除 `u_rob/head_reg -> commit -> ARF -> rename/RS` 的跨模块组合路径。

### 2. 为原始 DCache hit fast bypass 增加局部寄存边界

文件：`mycpu_top.v`

LSU 内已经寄存的 hold 结果仍直接使用；原始 DCache hit 的 `valid/robid/data` 先进入 `mem_dc_fast_*_r`，然后再送 ALU0、ALU1 和 MEM reservation station。正常的 `mem_wb` 架构写回路径不变。

效果：把原来的单周期 `req_paddr -> DCache -> LSU -> RS/ALU` 长链拆成两段。

### 3. 去掉 `mshr_line` 对分配判定的 CE 依赖

文件：`memory/dcache.v`

`M_IDLE` 中每拍预装 `mshr_line`；idle MSHR 内容在架构上无效。真正发生 miss allocation 时只切换 MSHR 状态和元数据，不再用 `mshr_alloc` 控制 256-bit `mshr_line` 的写使能。

效果：新网表中 `req_paddr -> mshr_line` 定向查询结果为 `No timing paths found`。

## 验证结果

### Verilator 20 项性能仿真

- 20/20 均为 `FINISH`
- 20/20 LED 均为 `ffff`
- 全 20 项：retire `4,888,901`，cycles `6,009,475`，IPC `0.813532`
- 约定的 18 项计算集（排除 dhrystone、stringsearch）：retire `4,368,678`，cycles `4,823,095`，IPC `0.905783`

对源基线：全 20 项 IPC `0.833413 -> 0.813532`，下降约 2.39%；18 项计算 IPC `0.932055 -> 0.905783`。主要代价来自 raw DCache hit fast bypass 增加一拍。

仿真日志：`D:/frontend/.codex_work/nscscc_cpu5/logs/impl75_fix2`

注意：这里只能表述为 20 项性能仿真通过。尚未获得一次完整官方功能平台结束标志，不能表述为“所有官方功能仿真全部通过”。

### 本机 Vivado 90 MHz OOC

- WNS `+0.380 ns`
- TNS `0.000 ns`
- setup 失败端点 `0`
- LUT `66,287`
- Register `29,546`
- BRAM `87.5`
- DSP `4`

源基线同方法为 WNS `+0.273 ns`、LUT `66,540`、Register `29,503`、BRAM `87.5`、DSP `4`。

定向网表检查：

- 原 `u_rob/head_reg -> RS/ALU`：`No timing paths found`
- 原 `req_paddr -> mshr_line`：`No timing paths found`
- 新 `req_paddr -> mem_dc_fast_*_r`：最差裕量 `+2.104 ns`
- 新 `mem_dc_fast_*_r -> RS/ALU`：最差裕量 `+3.086 ns`

报告位于 `reports_90m_ooc/`。其中 `target_*.rpt` 为三类原始路径的定向检查。

## 仍需异机复核

OOC 综合不是完整 SoC place-and-route。应把本目录替换进原工程，继续以 75 MHz 跑一次完整布线并重新导出报告，确认在另一台机器的物理布局中 WNS/TNS 也已闭合。三类原始违例已由 RTL 寄存边界或依赖移除，不依赖本次 OOC 恰好采用的布局。
