// ============================================================
// ras 模块（Return Address Stack，返回地址栈，双栈结构）
// ------------------------------------------------------------
// 参考实现说明：
// - 前端推测栈（BPU 预测 CALL push / RET pop）+ 提交栈（commit 维护，恒正确）；
// - flush 时前端栈整体复制提交栈（指针+内容+计数一拍对拷）；
// - 同拍 flush 与 cmt_push/pop：先算提交栈新值再恢复（用 next 值）；
// - 栈满环形回绕覆盖最旧项（深调用链精度下降可接受）。
// ============================================================
`include "mycpu.h"

module ras(
    input  wire                clk,
    input  wire                reset,

    // ---------------- 冲刷恢复 ----------------
    input  wire                flush_i,            // 前端栈整体复制提交栈

    // ---------------- 前端推测栈 ----------------
    input  wire                spec_push_i,        // BPU 预测到 CALL
    input  wire [31:0]         spec_push_addr_i,   // 返回地址（call 块 fall_through）
    input  wire                spec_pop_i,         // BPU 预测到 RET
    output wire [31:0]         top_addr_o,         // 栈顶（RET 预测目标）
    output wire                empty_o,            // 栈空（空时 RET 退化用 FTB fall_through）

    // ---------------- 提交栈 ----------------
    input  wire                cmt_push_i,         // commit 提交 call
    input  wire [31:0]         cmt_push_addr_i,    // 真实返回地址（call PC+4）
    input  wire                cmt_pop_i           // commit 提交 ret
);

reg [31:0]        spec_stack [0:`RAS_DEPTH-1];
reg [31:0]        cmt_stack  [0:`RAS_DEPTH-1];
reg [`RAS_W-1:0]  spec_ptr,  cmt_ptr;     // 指向当前栈顶
reg [`RAS_W:0]    spec_cnt,  cmt_cnt;     // 计数（饱和在 DEPTH）

// 提交栈 next 值（flush 同拍先提交后恢复）
wire [`RAS_W-1:0] cmt_ptr_n = cmt_push_i ? (cmt_ptr + 1'b1)
                            : (cmt_pop_i && (cmt_cnt != 0)) ? (cmt_ptr - 1'b1)
                            : cmt_ptr;
wire [`RAS_W:0]   cmt_cnt_n = cmt_push_i ? ((cmt_cnt == `RAS_DEPTH) ? cmt_cnt : (cmt_cnt + 1'b1))
                            : (cmt_pop_i && (cmt_cnt != 0)) ? (cmt_cnt - 1'b1)
                            : cmt_cnt;

assign top_addr_o = spec_stack[spec_ptr];
assign empty_o    = (spec_cnt == 0);

integer k;
always @(posedge clk) begin
    if (reset) begin
        spec_ptr <= {`RAS_W{1'b0}};
        cmt_ptr  <= {`RAS_W{1'b0}};
        spec_cnt <= {(`RAS_W+1){1'b0}};
        cmt_cnt  <= {(`RAS_W+1){1'b0}};
    end else begin
        // ---- 提交栈 ----
        if (cmt_push_i) cmt_stack[cmt_ptr + 1'b1] <= cmt_push_addr_i;
        cmt_ptr <= cmt_ptr_n;
        cmt_cnt <= cmt_cnt_n;

        // ---- 前端栈 ----
        if (flush_i) begin
            // 整体复制提交栈（用本拍提交后的新值）
            for (k = 0; k < `RAS_DEPTH; k = k + 1)
                spec_stack[k] <= cmt_stack[k];
            if (cmt_push_i) spec_stack[cmt_ptr + 1'b1] <= cmt_push_addr_i;
            spec_ptr <= cmt_ptr_n;
            spec_cnt <= cmt_cnt_n;
        end else begin
            if (spec_push_i) begin
                spec_stack[spec_ptr + 1'b1] <= spec_push_addr_i;
                spec_ptr <= spec_ptr + 1'b1;
                if (spec_cnt != `RAS_DEPTH) spec_cnt <= spec_cnt + 1'b1;
            end else if (spec_pop_i && (spec_cnt != 0)) begin
                spec_ptr <= spec_ptr - 1'b1;
                spec_cnt <= spec_cnt - 1'b1;
            end
        end
    end
end

endmodule
