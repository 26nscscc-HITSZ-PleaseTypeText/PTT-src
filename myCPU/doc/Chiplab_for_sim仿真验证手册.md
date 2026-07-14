# Chiplab_for_sim仿真验证手册

> V1.4 2026.6.8 by dogandlamb

>该手册默认使用环境是在wsl下的chiplab，可以直接使用我打包好的WSL_chiplab_for_sim.tar文件导入到wsl中

> 换入自己的 CPU RTL 后，按本文顺序做功能 / 随机延迟 / 性能 / OS 验证。  
> 路径默认：`CHIPLAB_HOME=~/chiplab`。

---

## 0. 替换 CPU 前检查/WSL_chiplab_for_sim环境简介

| 项 | 说明 |
|----|------|
| RTL 目录 | 将你的核 Verilog 放入 `IP/myCPU/`（替换或覆盖原 myCPU 文件） |
| 总线宽度 | `IP/myCPU/mycpu.h` 中 `BR_BUS_WD`、`FS_TO_DS_BUS_WD` 等与流水线接口一致 |
| Linux 频率 | 使用 la32r v0.2 自编内核时，`chip/soc_demo/sim/config.h` 中 `` `define FREQ `` 保持 **33MHz** |

WSL_chiplab_for_sim环境默认myCPU是Openla500，把基本工具链、vmlinux、run_random都配置好了。同时还有Smartwave工具集成，可以让AI使用Smartwave查波形信号

但我没有配置git，为了方便AI记忆与回退代码，请自行叫AI配置git

---

## 1. 每次新开终端：添加环境变量、编译RTL、处理弱智问题

此步骤为设置环境变量（其实我已经设置好了，这部分命令输不输无所谓）：

```bash
export CHIPLAB_HOME=~/chiplab
export PATH=$CHIPLAB_HOME/toolchains/loongson-gnu-toolchain-8.3-x86_64-loongarch32r-linux-gnusf-v2.0/bin:$PATH
# 若开启 difftest，还需 NEMU：
# export PATH=$CHIPLAB_HOME/toolchains/nemu:$PATH
```

一定要执行以下命令来编译mycpu RTL，你直接用make是不行的！
而且要注意看有没有编译成功，也就是看有没有ERROR字样（ERROR没有高亮，一定仔细看）！若没成功，那仿真时就仍然是用上次编译成功的结果


```bash
cd ~/chiplab/sims/verilator/run_prog
export CHIPLAB_HOME=~/chiplab
# 改过 RTL 必须重编（不要只 make run）
make verilator && make testbench
```
弱智问题TM来了（-^-）:

(1)编译时如果报错：%Error-TIMESCALEMOD
那就在以下 3 个文件开头加入 `timescale 1ns / 1ps：
```bash
sims/verilator/testbench/simu_top.v
sims/verilator/testbench/difftest.v
chip/soc_demo/sim/soc_top.v
```

(2)如果有提示“Permission denied”或其他与权限有关的问题（大概率是因为AI执行用过终端后，权限是属于root的，要换回dogandlamb）
```bash
sudo chown -R "$USER:$USER" .
```

(3)如果执行make run（func_lab19）时，出现
```bash
[NEMU] ####### INIT HERE ########
[NEMU] TLB_ENTRY = 32
[NEMU] PALEN = 32
Start
The first instruction of core 0 has commited. Difftest enabled.
[NEMU] PC: 0x1c000000 [NEMU]: INVALID INST
[NEMU]: INVALID INST CODE : 0x 0
The first instruction of core 0 has commited. Difftest enabled.
make[1]: *** [../Makefile_run:23: simulation_run_prog] Segmentation fault (core dumped)
make[1]: Leaving directory '/home/dogandlamb/chiplab/sims/verilator/run_prog/tmp'
make: *** [Makefile:284: simulation_run_prog] Error 2
```
大概率是因为func_lab19没生成好，那就
```bash
make clean
make soft_compile
```
(4)如果执行soft_compile时出现以下问题：
```bash
dogandlamb@admin:~/chiplab/sims/verilator/run_prog$ make soft_compile
===================================================
COMPILING func...
===================================================
make[1]: Entering directory '/home/dogandlamb/chiplab/software/examples/func/func_lab19'
make all
make[2]: Entering directory '/home/dogandlamb/chiplab/software/examples/func/func_lab19'
mkdir -p ./obj
make ./obj/inst_ram.coe ./obj/test.s
make[3]: Entering directory '/home/dogandlamb/chiplab/software/examples/func/func_lab19'
loongarch32r-linux-gnusf-ld ./obj/start.o ./obj/init.o ./obj/libinst.a -T bin.lds -o ./obj/main.elf
loongarch32r-linux-gnusf-ld: ./obj/start.o: in function `inst_test':
(.text+0x10098): undefined reference to `n10_nor_test'
make[3]: *** [Makefile:43: obj/main.elf] Error 1
make[3]: Leaving directory '/home/dogandlamb/chiplab/software/examples/func/func_lab19'
make[2]: *** [Makefile:13: all] Error 2
make[2]: Leaving directory '/home/dogandlamb/chiplab/software/examples/func/func_lab19'
make[1]: *** [Makefile:7: default] Error 2
make[1]: Leaving directory '/home/dogandlamb/chiplab/software/examples/func/func_lab19'
make: *** [Makefile:248: soft_compile] Error 2
```
也是因为func_lab19没生成好，清理旧产物就好，也就是执行：
```bash
cd ~/chiplab/software/examples/func/func_lab19
make clean
make
```

---

## 2. 两种「随机种子」（勿混淆）

名字都叫种子，但接的是**两套不同硬件**；在 **Verilator 仿真**里，真正控制 AXI 背压的只有其中一个。

### 2.0 一句话区别

| | `` `define RANDOM_SEED ``（confreg） | `BUS_DELAY_RANDOM_SEED`（Makefile） |
|---|-------------------------------------|-----------------------------------|
| **配置位置** | `IP/CONFREG/confreg_sim.v`（仅一行宏） | `sims/verilator/run_prog/Makefile_run` |
| **Verilator 里是否生效** | **否**（`confreg_sim.v` 未引用该宏） | **是**（默认 `BUS_DELAY=y` 时） |

默认：`BUS_DELAY_RANDOM_SEED=5570815`，即 23 位 `0x5500FF` = `{7'b1010101, 16'h00FF}`。

### 2.1 必测 7 组种子（Verilator：改 `Makefile_run`）

| # | 模式 | `` `define RANDOM_SEED `` | `BUS_DELAY_RANDOM_SEED` |
|---|------|---------------------------|-------------------------|
| 1 | 基线，基本无随机延迟 | `{7'b1010101,16'h00FF}` | `5570815`（`0x5500FF`） |
| 2 | 短延迟 | `{7'b1010101,16'h55FF}` | `5592575`（`0x5555FF`） |
| 3 | 短延迟 | `{7'b0101010,16'hAAFF}` | `2796287`（`0x2AAAFF`） |
| 4 | 正常随机延迟 | `{7'b1010101,16'h0001}` | `5570561`（`0x550001`） |
| 5 | 正常随机延迟 | `{7'b0101010,16'hACE1}` | `2796769`（`0x2AACE1`） |
| 6 | 正常随机延迟 | `{7'b1110001,16'h1234}` | `7410228`（`0x711234`） |
| 7 | 正常随机延迟 | `{7'b0011011,16'hBEEF}` | `1818351`（`0x1BBEEF`） |

**单测某一组（以 func_lab19 为例）：**

```bash
cd $CHIPLAB_HOME/sims/verilator/run_prog

# 编辑 Makefile_run，例如：
#   BUS_DELAY_RANDOM_SEED=5570815

./configure.sh --run func/func_lab19
make run          # 已编过仿真器时只需 make run；改过 IP/*.v 才需 make
```

**批量跑 7 组（我已经保存为 `run_7_seeds.sh`）：**

```bash
#!/bin/bash
set -e
export CHIPLAB_HOME=~/chiplab
export PATH=$CHIPLAB_HOME/toolchains/loongson-gnu-toolchain-8.3-x86_64-loongarch32r-linux-gnusf-v2.0/bin:$PATH
RUN=$CHIPLAB_HOME/sims/verilator/run_prog
MK=$RUN/Makefile_run
CASE=func/func_lab19

# 名称, 十进制 BUS_DELAY_RANDOM_SEED
SEEDS=(
  "baseline:5570815"
  "short1:5592575"
  "short2:2796287"
  "normal1:5570561"
  "normal2:2796769"
  "normal3:7410228"
  "normal4:1818351"
)

cd "$RUN"
for item in "${SEEDS[@]}"; do
  name=${item%%:*}
  seed=${item##*:}
  echo "========== $name BUS_DELAY_RANDOM_SEED=$seed =========="
  sed -i "s/^BUS_DELAY_RANDOM_SEED=.*/BUS_DELAY_RANDOM_SEED=$seed/" "$MK"
  ./configure.sh --run "$CASE" --disable-trace-comp
  make run || { echo "FAIL seed $name ($seed)"; exit 1; }
done
echo "All 7 seeds passed."
```

直接执行以下命令即可

```bash
cd ~/chiplab/sims/verilator/run_prog
export CHIPLAB_HOME=~/chiplab
export PATH=$CHIPLAB_HOME/toolchains/loongson-gnu-toolchain-8.3-x86_64-loongarch32r-linux-gnusf-v2.0/bin:$PATH
./configure.sh --run func/func_lab19
make          # 完整 make，生成 output
./run_7_seeds.sh
```

---

## 3. 波形开关（func_lab19 示例）

### 开启波形

```bash
cd $CHIPLAB_HOME/sims/verilator/run_prog
sed -i 's/^DUMP_WAVEFORM=.*/DUMP_WAVEFORM=1/' Makefile_run
./configure.sh --run func/func_lab19
make
cd log/func/func_lab19_log
gtkwave simu_trace.fst    # 默认 configure 为 FST；若为 VCD 则用 simu_trace.vcd
```

若没打开波形，可以运行以下命令就打开了

```bash
cd ~/chiplab/sims/verilator/run_prog/log/func/func_lab19_log
gtkwave simu_trace.fst
```

### 关闭波形（加快仿真）

```bash
cd $CHIPLAB_HOME/sims/verilator/run_prog
sed -i 's/^DUMP_WAVEFORM=.*/DUMP_WAVEFORM=0/' Makefile_run
./configure.sh --run func/func_lab19
make
```

> 用 `Ctrl+C` 强杀仿真时，波形可能丢失；可设 `TIME_LIMIT` 估算运行时间。

---

## 4. 查看可运行用例

```bash
cd $CHIPLAB_HOME/sims/verilator/run_prog
./configure.sh --help
```

---

## 5. 功能测试 func（ISA / 流水线）

```bash
cd $CHIPLAB_HOME/sims/verilator/run_prog
./configure.sh --run func/func_lab19
make
# 等价：make verilator && make testbench && make soft_compile && make simulation_run_prog
```

- 日志：`log/func/func_lab19_log/`（`uart_output.txt`、`simu_trace.txt` 等）

---

## 6. 随机指令测试 run_random

**先准备用例**（网盘见 `docs/Simulation/verilator.md`，提取码 `sHJS`）：

- 下载 `random_res_*.tar.gz`，解压得到 `RES_cluster_*` / `RES_jump_*`
- 拷贝到 **`$CHIPLAB_HOME/software/examples/random_res/`**

**运行：**

```bash
cd $CHIPLAB_HOME/sims/verilator/run_random
make
# 结果汇总：log/*/run.log，log/pass.log、log/fail.log
```

---

## 7. 性能测试

注意：在重新./configure.sh之前一定要先make clean_all，避免造成冲突

### CoreMark

```bash
cd $CHIPLAB_HOME/sims/verilator/run_prog
./configure.sh --run coremark
make
```

### Dhrystone

```bash
cd $CHIPLAB_HOME/sims/verilator/run_prog
./configure.sh --run dhrystone
make
```

---

## 8. OS / RTOS

### Linux（推荐参数，耗时长）

```bash
export CHIPLAB_HOME=~/chiplab
export PATH=$CHIPLAB_HOME/toolchains/loongson-gnu-toolchain-8.3-x86_64-loongarch32r-linux-gnusf-v2.0/bin:$PATH

# 若更新过 vmlinux / initrd_d，先打包 rom
cd $CHIPLAB_HOME/software/examples/linux
make clean && make

cd $CHIPLAB_HOME/sims/verilator/run_prog
# 必须刷新，否则会沿用旧 rom
rm -rf obj/linux_obj
make soft_compile

# Makefile_run 建议：TIME_LIMIT=0  DUMP_WAVEFORM=0
./configure.sh --run linux --disable-simu-trace --disable-trace-comp --disable-read-miss --output-uart-info
make
# 或已编译过仿真器：make run
```

**成功标志：** 串口出现 `Run /init`、`/ #`；`Scan bootparam failed` / `Initrd not found` 可忽略（内置 initramfs 仍可用）。

**到 shell 后可试：** `ls /`、`mount`、`ps`、`uname -a`、`echo hello`（仿真网口通常无真实链路）。

### RT-Thread

```bash
cd $CHIPLAB_HOME/sims/verilator/run_prog
./configure.sh --run rtthread --output-uart-info
make
```

---

## 9. 自定义 C 程序

### 仓库已有 c_prg

```bash
cd $CHIPLAB_HOME/sims/verilator/run_prog
./configure.sh --run c_prg/memcmp && make
./configure.sh --run c_prg/inner_product && make
./configure.sh --run c_prg/lookup_table && make
./configure.sh --run c_prg/loop_induction && make
./configure.sh --run c_prg/minmax_sequence && make
./configure.sh --run c_prg/product_sequence && make
```

### 新建用例（参考 `software/examples/hello_world`）

1. 新建 `software/examples/my_program/`（`Makefile` + `main.c`）
2. 运行：

```bash
cd $CHIPLAB_HOME/sims/verilator/run_prog
./configure.sh --run my_program
make
```

---

## 10. 常用 Makefile 目标（run_prog）

| 命令 | 含义 |
|------|------|
| `make` | 编译 Verilator + testbench + 软件 + 仿真（需先 `configure.sh`） |
| `make verilator` | 仅编译 RTL |
| `make testbench` | 仅编译 C++ 仿真器 |
| `make soft_compile` | 仅编译当前 configure 的软件并更新 `obj/*_obj` |
| `make run` | 仅运行仿真（已编译过时） |
| `make clean_soft` | 清理软件 obj |

---

## 11. 换 CPU 后的推荐验证顺序

1. **基线种子** `BUS_DELAY_RANDOM_SEED=5570815` + `func/func_lab19`  
2. **7 组 `BUS_DELAY_RANDOM_SEED`**跑 func 或关键 lab  
3. **run_random**（有用例后）  
4. **coremark / dhrystone**  
5. **Linux** 进 `/ #`  
6. 需要时 **开波形** 查 FAIL 现场  

---

## 12. FPGA 上板（仿真通过后）

见 `docs/FPGA_run_linux/linux_run.md`：PMON/u-boot + TFTP 加载 `software/examples/linux/vmlinux`（建议 `strip` 后），`g console=ttyS0,115200 rdinit=/init`。内置 initramfs 的内核**不依赖**仿真里的 bootparam/initrd。
