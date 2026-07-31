// ============================================================
// ctrl 模块（全局冲刷/重定向仲裁与分发）
// ------------------------------------------------------------
// 功能：
// - 接收 commit 级的冲刷请求（误预测/异常/ERTN/refetch），寄存一拍后向全核
//   广播 flush 脉冲与重定向 PC。所有恢复统一清空 RAT/RS/ROB/rename/
//   dispatch/IB/IFU/FTQ，并把 BPU 重定向到 flush_pc。
// - idle 指令支持：idle 提交后冻结取指，直到有中断到来才放行。
//
// 端口：
// - cmt_flush_*    ：commit 冲刷请求及目标 PC
// - flush_o/pc     ：全局冲刷广播（一拍脉冲）
// - idle_commit_i / has_int_i / fetch_stall_o ：idle 睡眠控制
// ============================================================
`include "mycpu.h"

module ctrl(
    input  wire                       clk,
    input  wire                       reset,

    // ---------------- commit 冲刷请求 ----------------
    input  wire                       cmt_flush_req_i,     // 提交级请求冲刷（一拍）
    input  wire [31:0]                cmt_flush_pc_i,      // 冲刷后取指目标

    // ---------------- idle 睡眠控制 ----------------
    input  wire                       idle_commit_i,       // idle 指令提交（随 flush 同拍）
    input  wire                       has_int_i,           // CSR 有未屏蔽中断

    // ---------------- 全局广播 ----------------
    output wire                       flush_o,             // 全局冲刷（一拍脉冲，接所有模块 flush_i）
    output wire [31:0]                flush_pc_o,          // 重定向 PC（接 bpu.flush_pc_i）
    output wire                       fetch_stall_o        // idle 睡眠期间冻结 BPU 取指
);

// commit 是唯一冲刷源。请求和目标寄存一拍后广播，以切断提交深组合链到全核
// flush 高扇出网络的路径。commit 在广播拍通过 flush_pending_i 闸住退休，保证
// 错误路径指令不会与流水线清空同拍提交。
reg idle_lock;

always @(posedge clk) begin
    if (reset) begin
        idle_lock <= 1'b0;
    end else if (has_int_i) begin
        idle_lock <= 1'b0;
    end else if (idle_commit_i) begin
        idle_lock <= 1'b1;
    end
end

// ---------------- 冲刷广播寄存器 ----------------
// T 拍提交冲刷指令并捕获请求；T+1 拍广播 flush，commit 同拍禁止继续退休；
// T+2 拍流水线已清空，前端从 flush_pc 重新填充。
reg        flush_r;
reg [31:0] flush_pc_r;
always @(posedge clk) begin
    if (reset) begin
        flush_r    <= 1'b0;
        flush_pc_r <= 32'b0;
    end else begin
        flush_r    <= cmt_flush_req_i;
        flush_pc_r <= cmt_flush_pc_i;
    end
end
assign flush_o    = flush_r;
assign flush_pc_o = flush_pc_r;
// idle 提交当拍即冻结取指，防止 idle_lock 尚未置位时取入后继指令；检测到未屏蔽
// 中断后解除锁定，后继指令在提交时承载中断异常。
assign fetch_stall_o = idle_lock | idle_commit_i;

endmodule
