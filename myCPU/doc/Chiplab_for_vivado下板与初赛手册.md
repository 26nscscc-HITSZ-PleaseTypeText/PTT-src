# Chiplab_for_vivado下板与初赛手册

> V1.1 2026.6.8 by dogandlamb

> V2.0 2026.7.13 by whalelll

## 1. 前言

这个手册是用来指导vivado仿真、上板验证的。myCPU应该先经过chiplab_for_sim的仿真验证（func_lab19与run_7_seeds.sh）与linux仿真后，再来此chiplab_for_vivado进行最后的功能测试与性能测试，及至最后的启动linux。

参考了chiplab的nscscc_readme.md

2607实验室的板子，对于上面的编号，56数码管上面灯有坏的，57按键损坏，58其中红色led损坏

## 2. 基于Vivado进行功能测试和性能测试的前仿真

1）【myCPU加入】首先确保已经替换`IP/myCPU`中的处理器核代码

2）【编译software】默认提供编译好的obj。

若希望重新编译性能测试：

```
cd $CHIPLAB_HOME/software/examples/nscscc_perf  #进入性能测试目录
make clean                                      #清除已有编译结果
make                                            #执行性能测试编译
```

重新编译功能测试：

```
cd $CHIPLAB_HOME/software/examples/nscscc_func   #进入功能测试目录
make clean                                       #清除已有编译结果
make                                             #执行功能测试编译
```

完成编译后，在software/examples/nscscc_func/obj目录下可以看到功能测试的编译结果

3）【创建Vivado工程】打开Vivado，在下方的控制台Tcl Console中，首先切换目录至`fpga/nscscc-team/run_vivado`，再调用create_project.tcl脚本。具体命令如下：

```
cd $CHIPLAB_HOME/fpga/nscscc-team/run_vivado
source create_project.tcl
```

脚本执行完成后，再手动添加myCPU代码至工程中

4）【执行Run Linter进行RTL分析】成功建立Vivado工程后可以先运行左侧的Run Linter进行语法检查。

5）【进行功能测试仿真】

首先修改 `$CHIPLAB_HOME/chip/soc_demo/nscscc-team/soc_config.vh`头文件，打开 `RUN_FUNC_TEST`宏，关闭`RUN_PERF_TEST`宏。该文件还存在两个可供调整的宏`SIMU_USE_PLL`和`SIMU_USE_DDR`，`SIMU_USE_PLL`为1时使用PLL产生时钟，为0时使用仿真时钟；`SIMU_USE_DDR`为1时使用DDR3作为内存，为0时使用仿真SRAM模型作为内存。两者均为0时仿真速度最快，为1时更符合上FPGA板的情况。`SIMU_USE_DDR`为1时仿真极慢，运行stream_copy测试程序大约需要15小时，建议仅在上板与仿真不一致且怀疑访存问题时打开该宏。

soc_config.vh修改后应为：

```bash
 `define RUN_FUNC_TEST
//`define RUN_PERF_TEST
`endif
```

完成宏的修改后在Vivado中点击Run Simulation。打开仿真界面后在控制台Tcl Console中执行下列命令，进行地址切换与调用tcl脚本执行仿真：

```
cd [get_property DIRECTORY [current_project]]
source ../run_func_test.tcl
```

功能测试包含58个测试点，应看到Vivado控制台打印信息`Number 8'd58 Functional Test Point PASS!!!`，证明功能测试前仿真通过。

5）【进行性能测试单个测试用例仿真】

首先修改 `$CHIPLAB_HOME/chip/soc_demo/nscscc-team/soc_config.vh`头文件，打开 `RUN_PERF_TEST`宏，关闭`RUN_FUNC_TEST`宏。

在性能测试中，还需要注意`$CHIPLAB_HOME/chip/soc_demo/nscscc-team/soc_config.vh`头文件中的`RUN_PERF_NO_DELAY`宏。打开该宏后，可以关闭内存的延时倍增，以加快仿真。性能测试分数提交应当是关闭`RUN_PERF_NO_DELAY`宏的分数。

soc_config.vh修改后应为：

```bash
// `define RUN_FUNC_TEST
`define RUN_PERF_TEST

`ifdef RUN_PERF_TEST
// `define RUN_PERF_NO_DELAY
`endif
```

在Vivado中点击Run Simulation。打开仿真界面后在控制台Tcl Console中执行下列命令，进行地址切换、更换内存初始化文件、重新开始仿真。

```
cd [get_property DIRECTORY [current_project]]
file copy -force ../../../../software/examples/nscscc_perf/obj/stream_copy/inst_data.bin ../inst_data.bin
restart
run all
```

例子中给出的是执行用时较短的 stream_copy 测试用例，执行其它性能测试用例，修改该字段即可。

如果性能仿真正确运行，在控制台Tcl Console里可以看到类似如下打印信息

```
Test begin!
…(不同程序有不同打印)
… PASS!... (不同程序有所不同)
…: Total Count =…(不同程序有所不同)
```

6）【进行性能测试全部测试用例仿真】
可直接使用tcl脚本对性能测试所有程序执行仿真，控制台命令如下：

```
cd [get_property DIRECTORY [current_project]]
source ../run_allbench.tcl
```

看到控制台输出stringsearch测试通过后，证明所有性能测试完成。

## 3. 基于Vivado进行综合实现

### 3.1 功能测试上板验证

1）功能/性能测试宏修改

首先修改 `chiplab/chip/soc_demo/nscscc-team/soc_config.vh`头文件，打开 `RUN_FUNC_TEST`宏，关闭`RUN_PERF_TEST`宏。

2）点击`Generate Bitstream`进行综合、实现、bit生成。

3）下载bit文件

打开Open Hardware Manager，连接好FPGA开发板后，选择Program Device，自动选择最新生成的比特文件。选择Program，等待下载完成。

4）通过JTAG下载bin文件

首先需要修改脚本`fpga/nscscc_team/run_vivado/jtag_axi_mater.tcl`第60行至第62行，选择需要下载的bin文件。这三行分别代表刚刚仿真中使用的bin文件、功能测试bin文件、性能测试bin文件。现在希望进行功能测试，因此打开第二行，将另外两行注释掉。

```
# set bin_file [open "../inst_data.bin" "rb"]
set bin_file [open "../../../../software/examples/nscscc_func/obj/main.bin" "rb"]
# set bin_file [open "../../../../software/examples/nscscc_perf/obj/allbench/inst_data.bin" "rb"]
```

完成修改后在Hardware Manager界面下方，Tcl Console中调用脚本进行bin文件下载，使用的命令如下。

```
cd [get_property DIRECTORY [current_project]]
source ../jtag_axi_master.tcl
```

脚本运行完成后便已经将bin文件下载至DDR3中。

5）观察实验现象

在FPGA上板验证时其结果正确与否的判断只有一种方法，func正确的执行行为是：

1.开始，单色LED全灭，双色LED灯一红一绿，数码管显示全0；

2.执行过程中，单色LED全灭，双色LED灯一红一绿，数码管高8位和低8位同步累加；

3.结束时，单色LED全灭，双色LED灯亮两绿，数码管高8位和低8位数值相同，对应测试功能点数目，龙芯杯功能测试应在数码管上出现3A 00 00 3A。

如果func执行过程中出错了，则数码管高8位和低8位第一次不同处即为测试出错的功能点编号，且最后的结果是单色LED全亮，双色LED灯亮两红，数码管高8位和低8位数值不同。

另外，可通过修改拨码开关switch值调整程序执行速度，从而看到完整的数码管数字递增。

***ps:如果只有bin文件，没有整个工程，参考下面，bin文件路径为D:/***

```
# 获取比特流中的 JTAG AXI Master
set axi_list [get_hw_axis]

if {[llength $axi_list] == 0} {
    error "没有找到 JTAG AXI Master，请确认 bit 文件包含 hw_axi 调试核"
}

set axi [lindex $axi_list 0]
puts "使用 JTAG AXI Master: $axi"

# 写32位寄存器
proc WriteReg {axi address data} {
    create_hw_axi_txn reg_write $axi \
        -address $address \
        -data $data \
        -type write

    run_hw_axi reg_write
    delete_hw_axi_txn reg_write
}

# bin文件路径
set bin_path "D:/main.bin"

if {![file exists $bin_path]} {
    error "找不到 bin 文件: $bin_path"
}

# 打开bin文件
set bin_file [open $bin_path "rb"]
fconfigure $bin_file -translation binary -encoding binary

# DDR3加载地址：0x1C000000
set addr_d 0x1C000000

# 复位处理器
puts "复位处理器..."
WriteReg $axi 80000000 00000000

# 每次通过AXI写入1024字节
set chunk_size 1024
set total_size 0

puts "开始将 $bin_path 下载到DDR3地址0x1C000000..."

while {1} {
    set data [read $bin_file $chunk_size]
    set data_len [string length $data]

    if {$data_len == 0} {
        break
    }

    # 最后不足4字节时补零
    set word_count [expr {($data_len + 3) / 4}]
    set padded_len [expr {$word_count * 4}]

    if {$padded_len > $data_len} {
        append data [string repeat "\x00" [expr {$padded_len - $data_len}]]
    }

    set temp_data {}

    # 将bin数据转换成Vivado AXI事务需要的数据格式
    for {set i 0} {$i < $padded_len} {incr i 4} {
        set word [string range $data $i [expr {$i + 3}]]
        binary scan $word cu* bytes

        lappend temp_data [format "%02X%02X%02X%02X" \
            [lindex $bytes 3] \
            [lindex $bytes 2] \
            [lindex $bytes 1] \
            [lindex $bytes 0]]
    }

    # Vivado的burst数据按高地址字在前排列
    set burst_data [join [lreverse $temp_data] "_"]
    set addr [format "%08X" $addr_d]

    create_hw_axi_txn burst_write $axi \
        -address $addr \
        -len $word_count \
        -type write \
        -data $burst_data

    run_hw_axi burst_write
    delete_hw_axi_txn burst_write

    incr addr_d $padded_len
    incr total_size $data_len

    puts [format "已下载 %d 字节，当前地址：0x%08X" \
        $total_size $addr_d]
}

close $bin_file

puts "bin文件下载完成，共写入 $total_size 字节"

# 解除处理器复位
puts "解除处理器复位..."
WriteReg $axi 40000000 00000000

puts "完成，处理器开始运行"
```

### 3.2 性能测试上板验证

1）功能/性能测试宏修改

首先修改 `chiplab/chip/soc_demo/nscscc-team/soc_config.vh`头文件，打开 `RUN_PERF_TEST`宏，关闭`RUN_FUNC_TEST`宏，关闭`RUN_PERF_NO_DELAY`宏。

2）点击`Generate Bitstream`进行综合、实现、bit生成。

3）下载bit文件

打开Open Hardware Manager，连接好FPGA开发板后，选择Program Device，自动选择最新生成的比特文件。选择Program，等待下载完成。

4）通过JTAG下载bin文件

首先需要修改脚本`fpga/nscscc_team/run_vivado/jtag_axi_mater.tcl`第60行至第62行，选择需要下载的bin文件。现在希望进行性能测试，因此打开第三行，将另外两行注释掉。

```
# set bin_file [open "../inst_data.bin" "rb"]
# set bin_file [open "../../../../software/examples/nscscc_func/obj/main.bin" "rb"]
set bin_file [open "../../../../software/examples/nscscc_perf/obj/allbench/inst_data.bin" "rb"]
```

完成修改后在Hardware Manager界面下方，Tcl Console中调用脚本进行bin文件下载，使用的命令如下。

```
cd [get_property DIRECTORY [current_project]]
source ../jtag_axi_master.tcl
```

下载 allbench 的 bin 文件后，在实验板上使用 8 个拨码开关的右侧 5 个选择运行哪个测试，随后按复位键，开始运行由拨码开关指定的测试。约定拨码开关拨上为 1，拨下为 0，则 5 个拨码开关与性能测试程序的对应关系如下表。（最左端按键拨上）

| 序号 | 运行的测试程序  | 拨码开关状态 |
| ---- | --------------- | ------------ |
| 1    | bitcount        | 5'b0_0001    |
| 2    | bubble_sort     | 5'b0_0010    |
| 3    | coremark        | 5'b0_0011    |
| 4    | crc32           | 5'b0_0100    |
| 5    | dhrystone       | 5'b0_0101    |
| 6    | quick_sort      | 5'b0_0110    |
| 7    | select_sort     | 5'b0_0111    |
| 8    | sha             | 5'b0_1000    |
| 9    | stream_copy     | 5'b0_1001    |
| 10   | stringsearch    | 5'b0_1010    |
| 11   | fireye_A0       | 5'b0_1011    |
| 12   | fireye_B2       | 5'b0_1100    |
| 13   | fireye_C0       | 5'b0_1101    |
| 14   | fireye_D1       | 5'b0_1110    |
| 15   | fireye_I2       | 5'b0_1111    |
| 16   | inner_product   | 5'b1_0000    |
| 17   | lookup_table    | 5'b1_0001    |
| 18   | loop_induction  | 5'b1_0010    |
| 19   | my_memcmp       | 5'b1_0011    |
| 20   | minmax_sequence | 5'b1_0100    |
| 其它 | 不运行性能测试  | 其它         |

5）数码管上显示的数字填到初赛提交包里就好

***ps:如果只有bin文件，没有整个工程，参考下面，bin文件路径为D:/***

```
get_hw_axis

get_hw_axisset bin_file [open "D:/inst_data.bin" "rb"]
fconfigure $bin_file -translation binary

create_hw_axi_txn cpu_reset [get_hw_axis hw_axi_1] -address 80000000 -data 00000000 -type write
run_hw_axi cpu_reset
delete_hw_axi_txn cpu_reset

set addr_d 469762048
set chunk_size 1024
set total_bytes 0

while {![eof $bin_file]} {
    set data [read $bin_file $chunk_size]
    set data_len [string length $data]

    if {$data_len == 0} {
        break
    }

    set remainder [expr {$data_len % 4}]
    if {$remainder != 0} {
        set padding [expr {4 - $remainder}]
        append data [binary format "x$padding"]
        set data_len [string length $data]
    }

    set temp_data [list]

    for {set i 0} {$i < $data_len} {incr i 4} {
        set bytes_data [string range $data $i [expr {$i + 3}]]

        binary scan $bytes_data B* binary_data
        set bytes [list]

        for {set j 0} {$j < 32} {incr j 8} {
            set byte [string range $binary_data $j [expr {$j + 7}]]
            lappend bytes [expr "0b$byte"]
        }

        lappend temp_data [format "%02X%02X%02X%02X" \
            [lindex $bytes 3] \
            [lindex $bytes 2] \
            [lindex $bytes 1] \
            [lindex $bytes 0]]
    }

    set temp_data [lreverse $temp_data]
    set burst_data [join $temp_data "_"]
    set addr [format "%08x" $addr_d]

    create_hw_axi_txn burst_write_txn [get_hw_axis hw_axi_1] \
        -address $addr \
        -len [expr {$data_len / 4}] \
        -type write \
        -data $burst_data

    run_hw_axi burst_write_txn
    delete_hw_axi_txn burst_write_txn

    incr addr_d $data_len
    incr total_bytes $data_len

    puts "Downloaded $total_bytes bytes"
}

close $bin_file

create_hw_axi_txn cpu_release [get_hw_axis hw_axi_1] -address 40000000 -data 00000000 -type write
run_hw_axi cpu_release
delete_hw_axi_txn cpu_release

puts "Performance bin download completed: $total_bytes bytes"
puts "CPU reset released, performance test started."
```

### 3.3 CPU 频率调整

性能测试统计的是 myCPU 运行性能测试程序实际花费的时间，其原理是：测试换 SoC_AXI_Lite 里设置了一
个固定 100MHz 的计时器，在运行性能测试程序的前后读取该计时器，其差值就是运行这一性能测试程序的所花费的实际时间。因而需要大家自行调整 SoC_AXI_Lite 里的 cpu_clk，使其为 myCPU 支持的最高频率，以获取最高性能分。调整 cpu_clk 的方法为：

双击Vivado工程中的`clk_pll`IP核，重新定制，在 `IP 定制界面->Output Clocks`修改 `clk_out1`的`Output Freq`。不允许修改`clk_out2`。完成修改后可重新进行综合、实现、bit生成。

调整 cpu_clk 后，一定要注意综合实现生成 bit 流文件后，Implementation 栏的 WNS 不允许为负值。

比赛统一约定，不允许 WNS 为负值！！！

### 3.4 综合、实现的优化参数不允许修改

大赛统一要求：不允许自行修改综合、实现、生成 Bit 流文件时候的参数，也不允许修改约束文件 soc_lite.xdc。

所以在生成 Bit 流文件准备上板时，必须按照发布包里的工程设置完成综合、实现的步骤。

在评审预赛作品时，会严格按照发布包里的环境进行综合、实现并查看上板情况，如果有时序违约，按照“预赛提交说明.pdf”，得分会记为 0 分。

希望大家将时间尽量花在有意义的事情上，比如设计 myCPU 之上运行的应用、系统等。

## 4.启动linux内核

我还没填呢，haha（-^-）
