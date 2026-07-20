# myCPU：LSU / DCache 合入报告

> 日期：2026-07-18（文档更新：2026-07-19）  
> **基线 CPU/报告**：[`chiplab性能仿真报告/myCPUV1.4_frontend_ICacheIFU合并_仿真报告.md`](chiplab性能仿真报告/myCPUV1.4_frontend_ICacheIFU合并_仿真报告.md)（Linux IPC **0.586**）→ 再叠 P1b 软门曾 **0.607**（无 park）→ 现树 digftest/软门 **~0.570**（UC park 代价）  
> 计划：`lsu_dcache_merge_plan` + digftest / Vivado PERF 收尾  
> 源：根目录 `dcache.v`/`lsu.v`（多 MSHR / pend / merge）+ `memory_v2.1` 投机 SB（探索未留）  
> 硬门：`func_lab19` digftest → `END by Syscall`  
> 软门：Linux → `/ #` + PERF（`--disable-trace-comp`）  
> digftest Linux：去掉 `--disable-trace-comp`；须到 `/ #`  
> 仿真：`CHIPLAB_HOME`，seed `5570815`  
> 原始日志：`/tmp/lab19_stq8_final.log`；`/tmp/linux_stq8_wide_dft.log`；软门对照 `/tmp/linux_stq4_soft.log` / `/tmp/linux_narrow_soft.log`  
> PERF 口径：`myCPU PERF (sim)`（`` `ifdef SYNTHESIS ``；仿真 `-DSIMU` → `mycpu.h` 定义 `SYNTHESIS`；Vivado **不**定义）

---

## 0. 一句话结论

**工作树 = P1b + flush 契约 + 精简 + UC park（宽龄门）+ `STQ_DEPTH=8`；`mem_*` overlap 在 `mycpu.h`。**

| 门控 | 结果 |
|------|------|
| lab19 digftest | **PASS**（`/tmp/lab19_stq8_final.log`，IPC **0.215**） |
| Linux digftest | **PASS** `/ #`（`/tmp/linux_stq8_wide_dft.log`，IPC **0.570**） |
| Linux 软门 | **PASS** `/ #`，IPC **0.570**（宽/窄 park、STQ 8/4 软门同量） |

**digftest 错载根因（二分）：**

| 配置 | digftest @ `account_user_time+0x6c` |
|------|-------------------------------------|
| 窄 park + STQ4 + 行合并 | **FAIL**（`s1` REF `0x78` / DUT 错指针） |
| 宽 park + STQ4 + 行合并 | **FAIL**（同点） |
| **宽 park + STQ8 + 行合并** | **PASS** `/ #` |

软门曾把 STQ 收到 4（IPC 无差），但 digftest 未复验 → **默认改回 `STQ_DEPTH=8`**。  
宽 park 保留（与窄软门 IPC 相同；窄另有 digftest 风险）。  
**未能回到 0.607：** 现 dcache 下无 park 软门也会 stall；park 完跑必需。

---

## 1. 这次合了什么（留在树上）

| 阶段 | 改动 | 是否留树 |
|------|------|----------|
| **P0–P1b** | 双 MSHR + pend + STQ + commit→SB | ✅ |
| **契约** | flush：`m_drop` + 仍回 `mshr_data_ok` | ✅ **关键** |
| **精简** | MSHR 行缓冲合并；`mem_*` → `mycpu.h` | ✅ |
| **UC park** | **宽**：比 DC 中 UC 更老的 AGU 让位 + `a_older_than_u` | ✅ |
| **STQ** | 软门可试 4；**digftest 需 8 → 默认 8** | ✅ |
| **探针极性** | 全树 `` `ifndef SYNTHESIS ``→`` `ifdef SYNTHESIS ``；`SIMU` 下定义 `SYNTHESIS` | ✅ |
| **P2/P3** | 投机 SB / hold early_wakeup | ❌ 已回退 |

### 1.1 digftest 错载与复验

- PC `0xa0337b7c` — `account_user_time+0x6c`：`ld.w $r24, $r23, 28`  
- REF `s1=0x78`，DUT 内核指针量级 → 错载 / 序破坏（非单纯 stall）  
- 复验：宽 park + **STQ=8** + 现 dcache 行合并 → `/ #`（与历史 `/tmp/linux_stq8_dft.log` 同 retire/IPC）

### 1.2 Vivado / 仿真探针极性

| 环境 | `SYNTHESIS` | `` `ifdef SYNTHESIS `` 探针/PERF |
|------|-------------|-------------------------------|
| Verilator（`-DSIMU`） | `mycpu.h` 内定义 | 仿真打表 |
| Vivado 综合 | **不定义** | 整段裁掉 |

---

## 2. 探索了什么（未留树 / 已回退）

| 配置 | lab19 | Linux |
|------|-------|-------|
| P1b + 契约（无 park） | PASS | 软门曾 **0.607**；现 dcache 下无 park 软门 stall |
| P2/P3 | PASS lab19 | 挂 ~0.8s |
| 窄 park digftest | PASS lab19 | **mismatch** |
| 宽 park + STQ4 digftest | PASS lab19 | **mismatch** 同点 |
| **宽 park + STQ8 digftest** | PASS | **PASS `/ #`，0.570** |

备份：`*.p1b_green.bak`、`lsu.v.narrow_park`、`dcache.v.contract.bak`、`dcache.v.merge_wip`、`*.p2p3_wip.bak`。

### 仿真坑

- 换 case 前 **`make clean_all`**  
- 改 RTL 必须 `make verilator && make testbench`  
- 软门缩队列后务必 **再跑 digftest**（STQ4 教训）  
- 注释勿以 `// Verilator` 开头  

---

## 3. 性能指标

条件：seed `5570815`；主对比用软门；digftest IPC 分列。

| 配置 | lab19 IPC | Linux IPC | store-order | 备注 |
|------|-----------|-----------|-------------|------|
| I$/IFU 基线 | ~0.215 | **0.586** | — | frontend 报告 |
| P1b 无 park | ~0.215 | **~0.607** | ~0.60% | 历史软门峰值 |
| 宽 park + STQ8（现树 digftest） | 0.215 | **0.570** | 0.25% | **功能基线** |
| 宽/窄 park + STQ4 软门 | 0.215 | **0.570** | 0.25% | 软门同 IPC；**digftest 不可用 STQ4** |

### 3.1 相对基线的分析

**优化：** 多 MSHR / pend / merge、flush 契约、STQ、行缓冲精简、`mem_*` 共用、宽 UC park。  

**提升：** 相对合入前访存链；无 park 峰值相对 I$/IFU **0.586→0.607**。  

**不足：** park → IPC **~0.570**（相对 0.586 约 −2.8%；相对 0.607 约 −6.1%）；MSHR busy ~19%；FTB/TAGE overflow 仍高；STQ 不能仅凭软门缩到 4。  

**下一步：** 改 park 机制追 IPC；STQ4 若再试须 digftest；勿砍 `L1_NMSHR=2`；P2 另开。

---

## 4. 板上资源

增量主要在 MSHR×2 行数据 + STQ×8 + LSU miss×2；相对 TAGE/FTB/Cache 很小。勿为省 LUT 砍双 MSHR。

---

## 5. 当前树

| 文件 | 状态 |
|------|------|
| `memory/dcache.v` | 多 MSHR + pend + 契约 + 行缓冲合并 |
| `backend/execute/lsu.v` | 多 miss + STQ + **宽** UC park |
| `memory/store_buffer.v` | commit 推 SB |
| `mycpu.h` | `STQ_DEPTH=8`；SIMU→SYNTHESIS；`mem_*` 三函数 |

---

## 附录 A：PERF 摘录

### lab19（`/tmp/lab19_stq8_final.log`）

```
Commit IPC:         retire=122875  cycles=572370  IPC=0.214678
Commit dual-issue:  dual_rate=41.31%
Dispatch dual:      dual_rate=73.41%  dual_ALU=51471
BPU all/cond:       89.46% / 91.21%
ICache:             access=74581  hit_rate=79.76%
ROB full:           3.53% | LSU store-order: 0.60% | LSU DC wait: 8.56%
RS_MEM src/LSU:     9.62% / 5.26%
D$ MSHR busy:       0.11%
FTB/TAGE overflow:  783 / 245
END by Syscall
```

### Linux digftest（宽 park + STQ8，`/tmp/linux_stq8_wide_dft.log`）

```
Commit IPC:         retire=124881985  cycles=219119196  IPC=0.569927
Commit dual-issue:  dual_rate=35.24%
Dispatch dual:      dual_rate=75.66%  dual_ALU=27681031
BPU all/cond:       92.19% / 96.13%
FTB P1 hit:         36.96%
ICache:             access=70355048  hit_rate=96.09%
DCache:             access=41046673  hit_rate=86.08%
ROB full:           8.96% | SB full: 0.54%
D$ MSHR busy:       19.27% | pend: 4.07% | LSU DC wait: 12.11%
LSU store-order:    0.25%
RS_MEM src/LSU:     19.26% / 7.96%
FTB/TAGE overflow:  313039 / 159006
Reached Linux shell prompt (/ #).
```

---

## 附录 B：阶段流水账

- **P0 / P1a**：宏 + `pend_can_progress`  
- **P1b + 契约**：Linux 软门峰值 0.607  
- **精简 + park + STQ**：digftest 完跑 0.570；STQ 默认 8  
