// ============================================================
// fu_alu 模块（ALU 执行单元，单周期，含分支判断；顶层例化 2 份）
// ------------------------------------------------------------
// 功能：
// - 接收 rs_alu 发射的指令，一拍完成：
//   * 普通算术逻辑运算（内部例化原有 alu.v 组合核，乘除已迁出到 fu_mdu）
//   * 分支：判方向（beq/bne/blt/bge/bltu/bgeu）、算目标
//     （条件分支/B/BL: pc+offs；jirl: src0+offs）、算链接值（pc+4）
// - 写回 ROB：结果 + 分支实际方向/实际目标（提交级据此判误预测）。
// - 发射拍对外广播提前唤醒（robid），缩短背靠背 RAW 延迟（二期接通）。
// - 二期预留执行级重定向输出（误预测当拍冲前端，配合 RAT 检查点）。
//
// 端口：
// - issue_*     ：rs_alu 发射口直连（组合进入，本模块内部锁存一拍执行）
// - wb_*        ：写回 ROB（含分支实际行为）
// - early_wakeup_* ：发射拍唤醒广播（已接到各 RS 的 early0/1）
// - ex_redirect_*  ：执行级重定向（二期，一期恒 0）
// ============================================================
`include "mycpu.h"

module fu_alu(
    input  wire                       clk,
    input  wire                       reset,
    input  wire                       flush_i,

    // ---------------- 发射入口（来自 rs_alu，组合）----------------
    input  wire                       issue_valid_i,
    input  wire [`ROB_W-1:0]          issue_robid_i,
    input  wire [31:0]                issue_pc_i,
    input  wire [`ALU_OP_NUM-1:0]     issue_alu_op_i,
    input  wire [`BR_OP_NUM-1:0]      issue_br_op_i,
    input  wire [31:0]                issue_src0_i,
    input  wire [31:0]                issue_src1_i,
    input  wire [31:0]                issue_imm_i,
    input  wire                       issue_use_imm_i,
    input  wire [31:0]                issue_br_offs_i,

    // ---------------- 写回 ROB ----------------
    output wire                       wb_valid_o,
    output wire [`ROB_W-1:0]          wb_robid_o,
    output wire [31:0]                wb_data_o,         // 运算结果 / 链接值(pc+4)
    output wire                       wb_br_taken_o,     // 分支实际方向（非分支恒 0）
    output wire [31:0]                wb_br_target_o,    // 分支实际目标（taken 时有效）

    // ---------------- 提前唤醒广播（发射拍，二期接入 RS）----------------
    output wire                       early_wakeup_valid_o,
    output wire [`ROB_W-1:0]          early_wakeup_robid_o,

    // ---------------- 二期：执行级重定向（一期输出恒 0）----------------
    output wire                       ex_redirect_valid_o,
    output wire [31:0]                ex_redirect_pc_o
);

reg                        ex_valid;
reg [`ROB_W-1:0]           ex_robid;
reg [31:0]                 ex_pc;
reg [`ALU_OP_NUM-1:0]      ex_alu_op;
reg [`BR_OP_NUM-1:0]       ex_br_op;
reg [31:0]                 ex_src0;
reg [31:0]                 ex_src1;
reg [31:0]                 ex_imm;
reg                        ex_use_imm;
reg [31:0]                 ex_br_offs;

wire                       is_link;   // 链接类（bl/jirl 写 rd = pc+4）
wire [31:0]                alu_result;
wire [31:0]                wb_data;


always @(posedge clk) begin
    if (reset || flush_i) begin
        ex_valid <= 1'b0;
    end
    else begin
        ex_valid <= issue_valid_i;
        if (issue_valid_i) begin
            ex_robid   <= issue_robid_i;
            ex_pc      <= issue_pc_i;
            ex_alu_op  <= issue_alu_op_i;
            ex_br_op   <= issue_br_op_i;
            ex_src0    <= issue_src0_i;
            ex_src1    <= issue_src1_i;
            ex_imm     <= issue_imm_i;
            ex_use_imm <= issue_use_imm_i;
            ex_br_offs <= issue_br_offs_i;
        end
    end
end

alu u_alu(
    .clk(clk),
    .reset(reset),
    .alu_op(ex_alu_op),
    .alu_src1(ex_src0),
    .alu_src2(ex_use_imm ? ex_imm : ex_src1),
    .exe_pc(ex_pc),
    .alu_result(alu_result),
    .alu_result_valid()
);

assign is_link = ex_br_op[`BR_OP_BL] | ex_br_op[`BR_OP_JIRL];  // 链接类（bl/jirl 写 rd = pc+4）
assign wb_data = is_link ? ex_pc + 32'd4 : alu_result;

assign wb_valid_o = ex_valid;
assign wb_robid_o = ex_robid;
assign wb_data_o = wb_data;


wire br_eq  = ex_src0 == ex_src1;
wire br_lts = $signed(ex_src0) < $signed(ex_src1);
wire br_ltu = ex_src0 < ex_src1;
wire is_branch = |ex_br_op;

wire br_taken = (ex_br_op[`BR_OP_BEQ]  &  br_eq)
              | (ex_br_op[`BR_OP_BNE]  & ~br_eq)
              | (ex_br_op[`BR_OP_BLT]  &  br_lts)
              | (ex_br_op[`BR_OP_BGE]  & ~br_lts)
              | (ex_br_op[`BR_OP_BLTU] &  br_ltu)
              | (ex_br_op[`BR_OP_BGEU] & ~br_ltu)
              |  ex_br_op[`BR_OP_B]
              |  ex_br_op[`BR_OP_BL]
              |  ex_br_op[`BR_OP_JIRL];

// n52 ADEF(jirl 目标非对齐)取指侧修法:jirl 目标 = rj+offs 原值(不掩码),链接 rd=pc+4 照写,
// 由目标取指(mmu i_vaddr[1:0]!=0)抬 ADEF、ERA=非对齐目标。前端需能承受非对齐 fetch PC(见探针)。
wire [31:0] br_target = ex_br_op[`BR_OP_JIRL]
                      ? (ex_src0 + ex_br_offs)
                      :  (ex_pc   + ex_br_offs);

assign wb_br_taken_o  = ex_valid & br_taken;
assign wb_br_target_o = (ex_valid && is_branch) ? br_target : 32'b0;

assign early_wakeup_valid_o = issue_valid_i && !flush_i && !reset;
assign early_wakeup_robid_o = issue_robid_i;

assign ex_redirect_valid_o  = 1'b0;
assign ex_redirect_pc_o     = 32'b0;

endmodule
