# myCPUV1.5：n49 异常修复 + L0 CSR 免 flush — 仿真报告

> 日期：2026-07-18  
> 基线：`myCPUV1.4_frontend_ICacheIFU合并_仿真报告.md`（I$/IFU 流水线 + update Q=32，seed `5570815`）  
> 来源：`IP/修功能测试n49_Bug的异常处理改动/` + `异常特权路径性能优化报告.md` Phase A1/A2  
> 负载：`func_lab19` digftest；Linux → `/ #`

---

## 0. 结论摘要

| 项 | 结果 |
|----|------|
| **func_lab19** | **通过**（`END by Syscall`，含 n49 TI） |
| **Linux** | **通过**（`/ #`） |
| lab19 IPC | 0.2153 → **0.2154**（≈持平） |
| Linux IPC | 0.5862 → **0.5871**（**+0.14%**） |
| 功能修复 | **已合入**：`timer_en` + TI 挂起冻结倒计时（上板 n49 根因） |
| 性能优化 | **已合入（安全版）**：TICLR/SAVE/TID 免 `FLUSH_REFETCH` + rename 排空 |
| 朴素免 flush | **不可行**：在途 `csrrd` 读旧 ESTAT → digftest 在 `0x1c0081d0` 炸（`t1=0x800`） |

**一句话：** 上板 n49 功能修复已进核；按报告做了 L0 CSR 免全核 refetch，并用 rename 串行堵住 CSR 读旁路洞。整体 IPC 几乎不动（L0 写在 lab19/Linux 稳态占比很小）；**收益主要在中断 handler 内「TICLR→下一条」气泡**，需上板 ILA 量化，仿真整段 IPC 不敏感。

---

## 1. 合入内容

### 1.1 功能修复（必做）

文件：`priv/csr_exception_commit_handler.v`（来自修功能目录）

| 点 | 行为 |
|----|------|
| `timer_en` | 对齐 open-la500；与可读 `TCFG.En` 分离 |
| 置 TI | `timer_en && TVAL==0` |
| oneshot / periodic | 到期停表 / 重装 InitVal |
| **TI 挂起冻结** | `estat_ti==1` 时不倒数；TICLR 后从已重装值走完下一完整周期 |
| 端口 | 去掉无用的 `flush_pipeline` 输出；`mycpu_top` 同步删接线 |

解决：OoO + CSR 全核冲刷使 `int_ex` 过长 → ertn 前再次置 TI → `230C` 级联风暴。

### 1.2 性能：Phase A1/A2（安全实现）

报告原意：TICLR（及 SAVE/TID）提交写 CSR 但 **不** `FLUSH_REFETCH`。

**朴素做法失败原因（已实测）：**

```
t1(r13) different at pc = 0x1c0081d0
  right=0x00000000  wrong=0x00000800   # DUT 仍读到 ESTAT.TI
```

本核 `csrrd` 在 **执行级**读 CSR。TICLR 提交免 flush 时，已在 ROB/流水线里的后续 `csrrd ESTAT` 仍可能带旧值退休。

**安全实现：**

| 模块 | 改动 |
|------|------|
| `mycpu.h` | `CSR_NUM_IS_L0_NOFLUSH`：TICLR / SAVE0~3 / TID |
| `rename.v` | L0 CSR 写单独进 ROB；置 `l0_csr_drain` 闸住后续 rename，直至 `l0_csr_commit` 或 flush |
| `commit.v` | L0 CSR 写：`csr_we=1`，**不**置 `selected_priv_flush` |
| `mycpu_top.v` | `l0_csr_commit_i = csr_we && L0` |

效果：提交时 ROB 中无更年轻指令 → 下一条 `csrrd` 在 TICLR 写回之后才 rename/执行 → 可见新值，且省掉一次全核 refetch。

**未做（报告后续）：** Phase B 冲刷/ICache 审计；去掉 TI 冻结；L0 双提。

---

## 2. 功能结果

| 负载 | 判定 | 说明 |
|------|------|------|
| func_lab19 | PASS | digftest 全程；retire=122875；含 n47~n81 异常/TI/cacop |
| Linux | PASS | 至 `/ #`；retire≈1.245e8 |

---

## 3. 性能对比（相对 I$/IFU 合并基线）

### 3.1 func_lab19

| 指标 | 基线 (I$/IFU) | 本轮 (n49+L0) | Δ |
|------|---------------|---------------|---|
| Commit IPC | 0.215277 | **0.215358** | +0.04% |
| cycles | 570775 | 570562 | −0.04% |
| Dispatch 双发 | 73.30% | 73.40% | ≈ |
| BPU all/cond | 89.46% / 91.21% | 89.48% / 91.23% | ≈ |
| I$ hit | 79.89% | 79.74% | ≈ |

### 3.2 Linux → `/ #`

| 指标 | 基线 (I$/IFU) | 本轮 (n49+L0) | Δ |
|------|---------------|---------------|---|
| **Commit IPC** | 0.586225 | **0.587069** | **+0.14%** |
| cycles | 212381328 | 212103733 | −0.13% |
| Dispatch 双发 | 76.55% | 76.58% | ≈ |
| BPU all/cond | 92.17% / 96.11% | 92.16% / 96.10% | ≈ |
| FTB P1 hit | 37.00% | 37.14% | +0.14pt |
| I$ hit | 96.16% | 96.14% | ≈ |
| ROB full | 10.13% | 10.10% | ≈ |
| FTB/TAGE overflow | 370k / 162k | 373k / 162k | ≈ |

整段 IPC **几乎无增益**是预期现象：L0 免 flush 只砍 handler 内个别 CSR 写后的 refetch；相对数亿拍 Linux 启动可忽略。lab19 同样以访存/异常路径为主，对 TICLR 气泡不敏感。

---

## 4. 如何理解「有没有提升」

| 度量 | 仿真结论 | 说明 |
|------|----------|------|
| 端到端 IPC | **基本持平** | 不适合作为本优化的主 KPI |
| digftest / OS | **不回退** | 安全版 A1 正确 |
| handler 内 TICLR→下一条 | 仿真未单独探针 | 报告建议上板 ILA 看 `wb_pc` 间隔；预期从「数百拍 refetch」降到「单提交量级」 |
| n49 上板 | 功能目录已验证过 scoreboard 越过 `30000030` | 本轮 RTL 含同一套 timer 修复 |

**瓶颈仍在：** ROB/访存（Linux ROB full ~10%，MSHR ~19%），与异常路径优化正交。

---

## 5. 实施备忘 / 回滚点

| 改动 | 文件 |
|------|------|
| 定时器语义 | `priv/csr_exception_commit_handler.v` |
| L0 宏 | `mycpu.h`（`CSR_NUM_IS_L0_NOFLUSH`） |
| 排空闸门 | `backend/rename/rename.v` |
| 免 flush | `backend/commit/commit.v` |
| 接线 | `mycpu_top.v`（去 `flush_pipeline`；接 `l0_csr_commit_i`） |

若仅要功能、不要 A1：保留 CSR handler，还原 rename/commit 的 L0 逻辑即可。

---

## 6. 复现

```bash
export CHIPLAB_HOME=~/chiplab
export PATH=$CHIPLAB_HOME/toolchains/loongson-gnu-toolchain-8.3-x86_64-loongarch32r-linux-gnusf-v2.0/bin:$PATH
cd $CHIPLAB_HOME/sims/verilator/run_prog

make clean_all
./configure.sh --run func/func_lab19
make verilator && make testbench && make soft_compile
make simulation_run_prog

make clean_all
./configure.sh --run linux --disable-simu-trace --disable-trace-comp --disable-read-miss --output-uart-info
rm -rf obj/linux_obj && make soft_compile
make verilator && make testbench
make simulation_run_prog
```

---

## 7. 附录 PERF 摘录

### lab19

```
END by Syscall
Commit IPC: retire=122875  cycles=570562  IPC=0.215358
BPU all/cond: 89.48% / 91.23%
ICache hit: 79.74%
```

### Linux

```
Detected Linux shell '/ #'
Commit IPC: retire=124519435  cycles=212103733  IPC=0.587069
Dispatch dual: 76.58%
BPU all/cond: 92.16% / 96.10%
ROB full: 10.10% | MSHR busy: 19.32%
```
