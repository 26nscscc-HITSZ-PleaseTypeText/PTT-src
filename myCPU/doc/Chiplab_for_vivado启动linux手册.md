# Chiplab_for_vivado启动linux手册

> V1.0 2026.6.8 by dogandlamb

> V2.0 2026.7.8 by whale//sssafridi
> V2.1 2026.7.11 by whale

> V5.0 2026.7.28：补充 myCPU L2/DMA 适配、Vivado 2023.2 约束与 U-Boot 正确命令。

## 0. myCPU V5.0 前置适配

V5.0 的缓存结构为：L1 I$/D$ 各 16 KiB、4 路、128 组、32 B 行，统一
L2 为 128 KiB、2 路、2048 组、32 B 行。板上以太网、MMC 等 DMA 主设备
直接访问 DDR，不经过 CPU 的 L2，因此不能只维护 L1。

配套版本已做以下适配：

- `la32r-uboot`：D/I cache line size 改为 32 B，启动阶段遍历 128 组、4 路；
- `la32r-Linux`：按实际 L1 几何初始化 cache 描述符，DMA 同步始终采用地址型
  `Hit_Writeback_Inv_D`；
- `IP/myCPU`：D$ 完成地址型 CACOP 后继续等待 L2 对同一物理行写回并无效，
  同时清除 L2 victim buffer 同行。

缺少任意一层都可能表现为 TFTP/网卡偶发包错、DMA buffer 读到旧数据或文件
加载后 ELF 内容损坏。启动前应确认三个仓库都位于 `v5.0` 分支对应提交。

FPGA 工程固定使用 `fpga/loongson/2023.2` 和 Vivado 2023.2。65 MHz 上板前
还须按 `V5.0_65MHz实现检查清单.md` 完成两次干净实现并通过时序门槛。

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
setenv bootargs 'console=ttyS0,115200 rdinit=/init'
```

* 可以配置内核的启动参数。 通过命令

```
tftpboot 0xa3000000 vmlinux
```

* 可以将内核 ELF 临时加载到 `0xa3000000`。该地址不是 `readelf` 显示的
  内核入口；`bootelf` 会按 ELF program header 把各段放到链接地址后跳转。
* 最后，使用命令即可成功运行linux内核！

```
bootelf 0xa3000000
```

启动日志应至少满足：

```text
Primary instruction cache 16kB, 4-way, VIPT, linesize 32 bytes.
Primary data cache 16kB, 4-way, VIPT, no aliases, linesize 32 bytes
Kernel command line: console=ttyS0,115200 rdinit=/init
```

最终以出现 `/ #` 为启动成功。网络可用性还需在 shell 中做一次收发验证；仅到
U-Boot prompt 或只完成 `ping` 不能替代 Linux DMA 一致性验证。

Verilator 联合仿真不具备以太网，因此只把 ELF 预装到 `0xa3000000`，随后仍由
U-Boot 的 `bootelf` 读取同一个 `bootargs` 环境变量并动态建立 `0xa1f00000`
启动块。这样联合仿真与板卡使用同一启动 ABI；打包脚本会拒绝与 ELF 暂存地址或
启动参数块重叠的 Linux LOAD 段。
