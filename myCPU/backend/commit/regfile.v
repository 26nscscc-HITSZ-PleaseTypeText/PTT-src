`include "mycpu.h"

// ============================================================
// regfile 模块（ARF：体系结构寄存器堆，32x32）
// ------------------------------------------------------------
// 功能（新架构角色变化）：
// - 队列式重命名下，本寄存器堆只保存"已提交"的权威值（ARF）：
//   * 读口 ×8：rename 级 4 口（0~3，当前两条指令源操作数，RAT 不 busy 时用）
//              + 分发驻留 4 口（4~7，指令在 dispatch 队列等待时读 ARF 唤醒）
//   * 写口 ×2：commit 级双提交写回（槽 0 / 槽 1）
// - 未提交的推测值一律存在 ROB 中，与本堆无关；冲刷时本堆即为恢复基准，
//   不需要任何恢复动作 —— 这是队列式重命名"恢复简单"的根源。
// - 调试读口（chiplab rf_rdata）与 DIFFTEST 全寄存器快照口保留。
//
// 端口：
// - raddr0~3/rdata0~3 ：rename 级 4 读口（组合，r0 恒 0）
// - raddr4~7/rdata4~7 ：分发驻留 4 读口（组合，r0 恒 0；RAT busy 清后读权威值）
// - we0/waddr0/wdata0 ：commit 槽 0 写口
// - we1/waddr1/wdata1 ：commit 槽 1 写口（同地址时槽 1 更年轻，优先生效）
// - dbg_*             ：chiplab 调试读
// - diff_gpr_*        ：DIFFTEST 快照
// ============================================================
module regfile(
    input  wire        clk,

    // ---------------- rename 级读口 ×4（组合，0~3）----------------
    input  wire [ 4:0] raddr0,           // 槽 0 src0
    output wire [31:0] rdata0,
    input  wire [ 4:0] raddr1,           // 槽 0 src1
    output wire [31:0] rdata1,
    input  wire [ 4:0] raddr2,           // 槽 1 src0
    output wire [31:0] rdata2,
    input  wire [ 4:0] raddr3,           // 槽 1 src1
    output wire [31:0] rdata3,

    // ---------------- 分发驻留读口 ×4（组合，4~7）----------------
    // 指令在 dispatch 队列驻留、前序写它的指令提交后，用锁存源地址读 ARF 唤醒
    input  wire [ 4:0] raddr4,           // dis0 src0
    output wire [31:0] rdata4,
    input  wire [ 4:0] raddr5,           // dis0 src1
    output wire [31:0] rdata5,
    input  wire [ 4:0] raddr6,           // dis1 src0
    output wire [31:0] rdata6,
    input  wire [ 4:0] raddr7,           // dis1 src1
    output wire [31:0] rdata7,

    // ---------------- commit 写口 ×2 ----------------
    input  wire        we0,              // 槽 0 写使能
    input  wire [ 4:0] waddr0,
    input  wire [31:0] wdata0,
    input  wire        we1,              // 槽 1 写使能（同地址时优先于槽 0）
    input  wire [ 4:0] waddr1,
    input  wire [31:0] wdata1,

    // ---------------- 调试只读端口：core_top.rf_rdata 由本端口驱动 ----------------
    input  wire [ 4:0] dbg_raddr,
    output wire [31:0] dbg_rdata

`ifdef DIFFTEST_EN
    ,
    output wire [31:0] diff_gpr_0,
    output wire [31:0] diff_gpr_1,
    output wire [31:0] diff_gpr_2,
    output wire [31:0] diff_gpr_3,
    output wire [31:0] diff_gpr_4,
    output wire [31:0] diff_gpr_5,
    output wire [31:0] diff_gpr_6,
    output wire [31:0] diff_gpr_7,
    output wire [31:0] diff_gpr_8,
    output wire [31:0] diff_gpr_9,
    output wire [31:0] diff_gpr_10,
    output wire [31:0] diff_gpr_11,
    output wire [31:0] diff_gpr_12,
    output wire [31:0] diff_gpr_13,
    output wire [31:0] diff_gpr_14,
    output wire [31:0] diff_gpr_15,
    output wire [31:0] diff_gpr_16,
    output wire [31:0] diff_gpr_17,
    output wire [31:0] diff_gpr_18,
    output wire [31:0] diff_gpr_19,
    output wire [31:0] diff_gpr_20,
    output wire [31:0] diff_gpr_21,
    output wire [31:0] diff_gpr_22,
    output wire [31:0] diff_gpr_23,
    output wire [31:0] diff_gpr_24,
    output wire [31:0] diff_gpr_25,
    output wire [31:0] diff_gpr_26,
    output wire [31:0] diff_gpr_27,
    output wire [31:0] diff_gpr_28,
    output wire [31:0] diff_gpr_29,
    output wire [31:0] diff_gpr_30,
    output wire [31:0] diff_gpr_31
`endif

);
reg [31:0] rf[31:0];
integer i;

initial begin
    for (i = 0; i < 32; i = i + 1) begin
        rf[i] = 32'b0;
    end
end

// 双写口约定（已实现）：同拍 waddr0==waddr1 时槽 1（更年轻）优先生效。
always @(posedge clk) begin
    if (we1 && (waddr1 != 5'b0)) begin
        rf[waddr1] <= wdata1;
    end
    if (we0 && (waddr0 != 5'b0) && !(we1 && (waddr1 == waddr0))) begin
        rf[waddr0] <= wdata0;
    end
end

// 读口写穿透（已实现）：同拍写的值直接出现在读口（双写口都透，槽 1 优先），
//      与 RAT 的提交释放/重命名读时序约定对齐，避免"提交拍读到旧值"的窗口。
// 用 === 与 we===1'b1：地址或写使能含 X 时勿用 `==`/`&&` 产生 X 污染读口（进而污染 CSR 写回链）。
assign rdata0 = (raddr0 === 5'b0) ? 32'b0 :
                (((we1 === 1'b1) && (waddr1 === raddr0) && (waddr1 !== 5'b0)) ? wdata1 :
                 ((we0 === 1'b1) && (waddr0 === raddr0) && (waddr0 !== 5'b0)) ? wdata0 :
                 rf[raddr0]);

assign rdata1 = (raddr1 === 5'b0) ? 32'b0 :
                (((we1 === 1'b1) && (waddr1 === raddr1) && (waddr1 !== 5'b0)) ? wdata1 :
                 ((we0 === 1'b1) && (waddr0 === raddr1) && (waddr0 !== 5'b0)) ? wdata0 :
                 rf[raddr1]);

assign rdata2 = (raddr2 === 5'b0) ? 32'b0 :
                (((we1 === 1'b1) && (waddr1 === raddr2) && (waddr1 !== 5'b0)) ? wdata1 :
                 ((we0 === 1'b1) && (waddr0 === raddr2) && (waddr0 !== 5'b0)) ? wdata0 :
                 rf[raddr2]);

assign rdata3 = (raddr3 === 5'b0) ? 32'b0 :
                (((we1 === 1'b1) && (waddr1 === raddr3) && (waddr1 !== 5'b0)) ? wdata1 :
                 ((we0 === 1'b1) && (waddr0 === raddr3) && (waddr0 !== 5'b0)) ? wdata0 :
                 rf[raddr3]);

assign rdata4 = (raddr4 === 5'b0) ? 32'b0 :
                (((we1 === 1'b1) && (waddr1 === raddr4) && (waddr1 !== 5'b0)) ? wdata1 :
                 ((we0 === 1'b1) && (waddr0 === raddr4) && (waddr0 !== 5'b0)) ? wdata0 :
                 rf[raddr4]);

assign rdata5 = (raddr5 === 5'b0) ? 32'b0 :
                (((we1 === 1'b1) && (waddr1 === raddr5) && (waddr1 !== 5'b0)) ? wdata1 :
                 ((we0 === 1'b1) && (waddr0 === raddr5) && (waddr0 !== 5'b0)) ? wdata0 :
                 rf[raddr5]);

assign rdata6 = (raddr6 === 5'b0) ? 32'b0 :
                (((we1 === 1'b1) && (waddr1 === raddr6) && (waddr1 !== 5'b0)) ? wdata1 :
                 ((we0 === 1'b1) && (waddr0 === raddr6) && (waddr0 !== 5'b0)) ? wdata0 :
                 rf[raddr6]);

assign rdata7 = (raddr7 === 5'b0) ? 32'b0 :
                (((we1 === 1'b1) && (waddr1 === raddr7) && (waddr1 !== 5'b0)) ? wdata1 :
                 ((we0 === 1'b1) && (waddr0 === raddr7) && (waddr0 !== 5'b0)) ? wdata0 :
                 rf[raddr7]);

// 调试端口：直接读寄存器内容（不含写穿透），r0 恒 0
assign dbg_rdata = (dbg_raddr === 5'b0) ? 32'b0 : rf[dbg_raddr];

`ifdef DIFFTEST_EN
// difftest 快照同样带 we1 写穿透（双口都透，槽 1 优先），
//      否则双提交拍 NEMU 对比会差一拍。
// posedge 快照 + 写穿透，与 cmt_wdata/debug0_wb_rf_wdata 同拍对齐；勿直接组合读 rf
reg [31:0] diff_gpr_r [1:31];
integer dg;
always @(posedge clk) begin
    for (dg = 1; dg < 32; dg = dg + 1) begin
        if ((we1 === 1'b1) && (waddr1 === dg[4:0]) && (waddr1 !== 5'b0))
            diff_gpr_r[dg] <= wdata1;
        else if ((we0 === 1'b1) && (waddr0 === dg[4:0]) && (waddr0 !== 5'b0))
            diff_gpr_r[dg] <= wdata0;
        else
            diff_gpr_r[dg] <= rf[dg];
    end
end
assign diff_gpr_0  = 32'b0;
assign diff_gpr_1  = diff_gpr_r[1];
assign diff_gpr_2  = diff_gpr_r[2];
assign diff_gpr_3  = diff_gpr_r[3];
assign diff_gpr_4  = diff_gpr_r[4];
assign diff_gpr_5  = diff_gpr_r[5];
assign diff_gpr_6  = diff_gpr_r[6];
assign diff_gpr_7  = diff_gpr_r[7];
assign diff_gpr_8  = diff_gpr_r[8];
assign diff_gpr_9  = diff_gpr_r[9];
assign diff_gpr_10 = diff_gpr_r[10];
assign diff_gpr_11 = diff_gpr_r[11];
assign diff_gpr_12 = diff_gpr_r[12];
assign diff_gpr_13 = diff_gpr_r[13];
assign diff_gpr_14 = diff_gpr_r[14];
assign diff_gpr_15 = diff_gpr_r[15];
assign diff_gpr_16 = diff_gpr_r[16];
assign diff_gpr_17 = diff_gpr_r[17];
assign diff_gpr_18 = diff_gpr_r[18];
assign diff_gpr_19 = diff_gpr_r[19];
assign diff_gpr_20 = diff_gpr_r[20];
assign diff_gpr_21 = diff_gpr_r[21];
assign diff_gpr_22 = diff_gpr_r[22];
assign diff_gpr_23 = diff_gpr_r[23];
assign diff_gpr_24 = diff_gpr_r[24];
assign diff_gpr_25 = diff_gpr_r[25];
assign diff_gpr_26 = diff_gpr_r[26];
assign diff_gpr_27 = diff_gpr_r[27];
assign diff_gpr_28 = diff_gpr_r[28];
assign diff_gpr_29 = diff_gpr_r[29];
assign diff_gpr_30 = diff_gpr_r[30];
assign diff_gpr_31 = diff_gpr_r[31];
`endif

endmodule
