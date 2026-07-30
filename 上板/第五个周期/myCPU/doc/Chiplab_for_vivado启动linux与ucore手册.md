# Chiplab_for_vivado启动linux与ucore手册

> V1.0 2026.6.8 by dogandlamb

> V2.0 2026.7.8 by whale//sssafridi

> V2.1 2026.7.11 by whale

> V3.0 2026.7.28 by dogandlamb

前置条件：

* flash 芯片正确放置 FPGA 开发板上。
* FPGA 开发板与电脑连接下载线、串口线、网线。

## 1.烧写控制Flash

下载[gitee.com/chenzes/chiplab-tools/releases/download/chiplab-tools/programmer_by_uart.bit](https://gitee.com/chenzes/chiplab-tools/releases/download/chiplab-tools/programmer_by_uart.bit)，这是一个比特流文件，同样使用vivado的Open HardWare Manager来烧录。

打开Open Hardware Manager，连接好FPGA开发板后，选择Program Device，选择刚刚下载的比特文件。选择Program，等待下载完成。记得对开发板上电。

## 2.串口配置及烧写PMON

下载串口软件（[ECOM](https://gitee.com/chenzes/chiplab-tools/releases/download/chiplab-tools/ECOMV280.zip)或[SecureCRT](https://gitee.com/chenzes/chiplab-tools/releases/download/chiplab-tools/SecureCRTPortable.zip)），下面讲解的是ecom这个串口软件

* 端口号看设备管理器，查看端口（COM和LPT），USB Serial  Port对应是哪个就是哪个端口
* 波特率选择230400，但是之后烧完成下一步之后，换为115200
* 传输协议选择Xmodem Send
* 打开文件选择下载的[gitee.com/chenzes/chiplab-tools/releases/download/pmon/gzrom.bin](https://gitee.com/chenzes/chiplab-tools/releases/download/pmon/gzrom.bin)这个bin文件（pmon烧这个，uboot也是烧相对应的bin文件）
* 剩下设置保持默认不变

接下来是烧写步骤

* 按reset键，等待不再输出
* 串口连接正常后根据提示，键盘输入 x 表示开始 xmodem 传输，会一直输出CCCC。。。。
* 按“发送文件”此按键

## 3.烧写设计cpu比特流文件

打开Open Hardware Manager，连接好FPGA开发板后，选择Program Device，选择生成的比特文件。选择Program，等待下载完成。记得对开发板上电。

这样即可启动PMON,正常应该是会有PMON>在串口工具上，可能需要等待一会儿。

## 4.1更改开发板网卡设置，tftp,注意关闭防火墙（PMON）

1. 请保证网线是电脑与开发板相连；电脑和开发板将使用以特网连接，具体操作如下：

* 打开电脑的控制面板→网络和共享中心→页面左边的“更改适配器设置”
* 右键“以太网”，选择“属性”→双击“Internet 协议版本4/IPv4”
* 在“常规”中勾选“使用下面的IP地址”→IP地址填写：10.249.10.114（任意？）；子网掩码填写255.255.255.0；默认网关空着
* 关闭控制面板，继续回到串口软件的PMON界面，输入"ifconfig dmfe0 10.249.10.113"  (前面三个数和上面指定的IP地址相同即可)
* 下载tftp，安装好之后将current directory 换为vmlinux所在目录，server interfaces选以特网，IP地址就是上面指定过的
* 回到串口软件的PMON界面，输入"load tftp://10.249.10.114/vmlinux"  （中间的IP地址还是一开始指定的）

2.等待显示”Entry address...",之后在串口上继续输入"`g console=ttyS0,115200 rdinit=sbin/init`““等待烧录即可，烧录完成即启动linux内核

## 4.2更改开发板网卡设置，tftp,注意关闭防火墙（U-boot）

1. 请保证网线是电脑与开发板相连；电脑和开发板将使用以特网连接，具体操作如下：

* 打开电脑的控制面板→网络和共享中心→页面左边的“更改适配器设置”
* 右键“以太网”，选择“属性”→双击“Internet 协议版本4/IPv4”
* 在“常规”中勾选“使用下面的IP地址”→IP地址填写：10.0.0.1（任意？）；子网掩码填写255.255.255.0；默认网关空着
* 关闭控制面板，继续回到串口软件的uboot界面，输入""  (前面三个数和上面指定的IP地址相同即可)

```
setenv ipaddr 10.0.0.2
setenv serverip 10.0.0.1
setenv netmask 255.255.255.0
```

* 使用ping 10.0.0.1
* 如果出现 `host 10.0.0.1 is alive`，说明网络已通，可以用 `tftpboot` 下载文件了。
* 如果仍然 `not alive`，直接在电脑上  **ping 10.0.0.2** （开发板的 IP），看是否有回复，同时关闭电脑的防火墙再试。
* 下载tftp，安装好之后将current directory 换为vmlinux所在目录，server interfaces选以特网，IP地址就是上面指定过的
* 回到串口软件的uboot界面
* 完成了网络配置，就可以使用uboot加载内核，使用命令

```
setenv bootcmd console=ttyS0,115200 rdinit=/init
```

* 可以配置内核的启动参数。 通过命令

```
tftpboot 0xa3000000 vmlinux(vmlinux需要跟文件名字一样，不一样就改成文件名字)
```

* 可以将内核加载到 `<span class="pre">0xa300_0000</span>` 开始的地址上，该地址并不是`<span class="pre">readelf</span>`显示的`<span class="pre">entry</span>`的入口地址。uboot会先把镜像加载到一段无数据的地址，运行时再根据elf段的信息加载到对应的位置上。
* 最后，使用命令即可成功运行linux内核！

```
bootelf 0xa3000000 bootcmd
```

## 5. 启动 ucore

前置：§1–§3 已完成（Flash / PMON 或 u-boot / CPU bit 流），网线与 TFTP 同 §4.1 或 §4.2。串口软件波特率与跑 Linux 相同，用 **115200**（烧完 PMON 之后；烧写 gzrom 阶段仍是 230400）。

内核文件（ELF，入口 `0xa0000000`，已内嵌 initrd）：

```
$CHIPLAB_HOME/ucore-loongarch32/obj/ucore-kernel-initrd
```

若尚无该文件：

```bash
export CHIPLAB_HOME=~/chiplab
export PATH=$CHIPLAB_HOME/toolchains/loongson-gnu-toolchain-8.3-x86_64-loongarch32r-linux-gnusf-v2.0/bin:$PATH
cd $CHIPLAB_HOME/ucore-loongarch32 && make -j$(nproc)
```

### 5.1 串口波特率

主机串口与跑 Linux 相同，用 **115200**。当前 `$CHIPLAB_HOME/ucore-loongarch32` 已按上板配置：内核**不改写**波特率分频，沿用 PMON/u-boot 的 115200。直接使用编好的 `obj/ucore-kernel-initrd` 即可（勿再把 `console.c` 里的 DLL 改成仿真用的 `1`，否则上板会乱码）。

### 5.2 PMON 加载

TFTP 当前目录设为 `ucore-kernel-initrd` 所在目录，开发板侧（IP 按 §4.1）：

```
ifconfig dmfe0 10.249.10.113
load tftp://10.249.10.114/ucore-kernel-initrd
g
```

无需 Linux 的 `console=... rdinit=...`。成功后串口应出现 ucore 启动日志，最后到用户 shell 提示符 `$ `。

### 5.3 U-boot 加载

网络按 §4.2 配好后：

```
tftpboot 0xa3000000 ucore-kernel-initrd
bootelf 0xa3000000
```

说明：`0xa3000000` 只是 TFTP 暂存地址（与加载 vmlinux 相同）；`bootelf` 会按 ELF 段装到链接地址 `0xa0000000` 并跳转。不需要 `bootcmd` / Linux 启动参数。

可试命令：`ls`、`pwd`、`cat test.txt`、`echo hi`、`hello` 等（见 `ucore-loongarch32/user/`）。
