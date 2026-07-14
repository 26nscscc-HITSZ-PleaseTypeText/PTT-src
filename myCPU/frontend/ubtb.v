// ============================================================
// ubtb 模块（micro-BTB，当拍返回的小型分支目标缓冲）
// ------------------------------------------------------------
// 参考实现说明：
// - 16 项全相联（完整 32 位块 PC 做 tag），查询纯组合当拍返回；
// - 仅回填"实际发生跳转的向回分支"（target < block_pc，小循环），
//   命中即预测跳转（taken 恒 1）；
// - 替换：同 tag 原地更新 > 无效项 > 轮转替换。
// ============================================================
`include "mycpu.h"

module ubtb(
    input  wire                       clk,
    input  wire                       reset,

    // ---------------- 查询口（组合，当拍返回）----------------
    input  wire [31:0]                query_pc_i,        // 预测块起始 PC
    output wire                       hit_o,             // 命中
    output wire                       taken_o,           // 命中项的方向（uBTB 只存恒跳/强跳分支）
    output wire [31:0]                target_o,          // 跳转目标
    output wire [`BLK_LEN_W-1:0]      length_o,          // 块长（起始 PC 到分支指令的条数）
    output wire [`BR_TYPE_W-1:0]      br_type_o,

    // ---------------- 更新口（提交训练时回填）----------------
    input  wire                       update_valid_i,    // 本拍有训练
    input  wire [31:0]                update_block_pc_i, // 块起始 PC（作为 tag）
    input  wire                       update_taken_i,
    input  wire [31:0]                update_target_i,
    input  wire [`BLK_LEN_W-1:0]      update_length_i,
    input  wire [`BR_TYPE_W-1:0]      update_br_type_i
);

reg [`UBTB_SIZE-1:0]   valid;
reg [31:0]             tag    [0:`UBTB_SIZE-1];
reg [31:0]             target [0:`UBTB_SIZE-1];
reg [`BLK_LEN_W-1:0]   length [0:`UBTB_SIZE-1];
reg [`BR_TYPE_W-1:0]   btype  [0:`UBTB_SIZE-1];
reg [3:0]              repl_ptr;

// ---------------- 查询（全相联组合比较）----------------
wire [`UBTB_SIZE-1:0] q_hit;
genvar g;
generate
for (g = 0; g < `UBTB_SIZE; g = g + 1) begin : gen_qhit
    assign q_hit[g] = valid[g] && (tag[g] == query_pc_i);
end
endgenerate

reg [3:0]  q_idx;
integer qi;
always @(*) begin
    q_idx = 4'd0;
    for (qi = `UBTB_SIZE-1; qi >= 0; qi = qi - 1)
        if (q_hit[qi]) q_idx = qi[3:0];
end

assign hit_o     = |q_hit;
assign taken_o   = |q_hit;            // 只存跳转分支，命中即预测跳
assign target_o  = target[q_idx];
assign length_o  = length[q_idx];
assign br_type_o = btype[q_idx];

// ---------------- 更新（仅回填向回跳转的分支）----------------
wire do_fill = update_valid_i && update_taken_i && (update_target_i < update_block_pc_i);

wire [`UBTB_SIZE-1:0] u_hit;
generate
for (g = 0; g < `UBTB_SIZE; g = g + 1) begin : gen_uhit
    assign u_hit[g] = valid[g] && (tag[g] == update_block_pc_i);
end
endgenerate

reg [3:0]  u_idx;
reg        u_found;
reg [3:0]  inv_idx;
reg        inv_found;
integer ui;
always @(*) begin
    u_found = 1'b0;  u_idx = 4'd0;
    inv_found = 1'b0; inv_idx = 4'd0;
    for (ui = `UBTB_SIZE-1; ui >= 0; ui = ui - 1) begin
        if (u_hit[ui]) begin u_found = 1'b1; u_idx = ui[3:0]; end
        if (!valid[ui]) begin inv_found = 1'b1; inv_idx = ui[3:0]; end
    end
end

wire [3:0] fill_idx = u_found ? u_idx : inv_found ? inv_idx : repl_ptr;

always @(posedge clk) begin
    if (reset) begin
        valid    <= {`UBTB_SIZE{1'b0}};
        repl_ptr <= 4'd0;
    end else if (do_fill) begin
        valid[fill_idx]  <= 1'b1;
        tag[fill_idx]    <= update_block_pc_i;
        target[fill_idx] <= update_target_i;
        length[fill_idx] <= update_length_i;
        btype[fill_idx]  <= update_br_type_i;
        if (!u_found && !inv_found) repl_ptr <= repl_ptr + 4'd1;
    end
end

endmodule
