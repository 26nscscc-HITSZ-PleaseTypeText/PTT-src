`include "mycpu.h"

// ============================================================
// alu 模块（运算核，新架构中被 fu_alu.v 例化）
// ------------------------------------------------------------
// 方案 A：纯组合 ALU。
// - 只承担单周期组合运算：加减/逻辑/移位/比较/LUI（含 ANDN/ORN）；
// - 乘除职责已迁出到 fu_mdu（mul.v/div.v），本模块乘除位输出 0；
// - 端口保持框架原样，clk/reset/exe_pc 仅占位，方便现有 fu_alu 对接。
// ============================================================
module alu(
    input  wire                    clk,
    input  wire                    reset,
    input  wire [`ALU_OP_NUM-1:0]  alu_op,
    input  wire [31:0]             alu_src1,
    input  wire [31:0]             alu_src2,
    input  wire [31:0]             exe_pc,
    output wire [31:0]             alu_result,
    output wire                    alu_result_valid
);

wire op_add  = alu_op[`ALU_OP_ADD];
wire op_sub  = alu_op[`ALU_OP_SUB];
wire op_slt  = alu_op[`ALU_OP_SLT];
wire op_sltu = alu_op[`ALU_OP_SLTU];
wire op_and  = alu_op[`ALU_OP_AND];
wire op_nor  = alu_op[`ALU_OP_NOR];
wire op_or   = alu_op[`ALU_OP_OR];
wire op_xor  = alu_op[`ALU_OP_XOR];
wire op_sll  = alu_op[`ALU_OP_SLL];
wire op_srl  = alu_op[`ALU_OP_SRL];
wire op_sra  = alu_op[`ALU_OP_SRA];
wire op_lui  = alu_op[`ALU_OP_LUI];
wire op_andn = alu_op[`ALU_OP_ANDN];
wire op_orn  = alu_op[`ALU_OP_ORN];
wire op_pcadd = alu_op[`ALU_OP_PCADD];

wire [31:0] adder_a   = op_pcadd ? exe_pc : alu_src1;
wire [31:0] adder_b   = (op_sub | op_slt | op_sltu) ? ~alu_src2 : alu_src2;
wire        adder_cin = (op_sub | op_slt | op_sltu);
wire [31:0] adder_result;
wire        adder_cout;
assign {adder_cout, adder_result} = adder_a + adder_b + {31'b0, adder_cin};

wire [31:0] add_sub_result = adder_result;
wire [31:0] slt_result     = {31'b0, (alu_src1[31] & ~alu_src2[31])
                               | ((alu_src1[31] ~^ alu_src2[31]) & adder_result[31])};
wire [31:0] sltu_result    = {31'b0, ~adder_cout};
wire [31:0] and_result     = alu_src1 & alu_src2;
wire [31:0] or_result      = alu_src1 | alu_src2;
wire [31:0] nor_result     = ~or_result;
wire [31:0] xor_result     = alu_src1 ^ alu_src2;
wire [31:0] lui_result     = alu_src2;
wire [31:0] andn_result    = alu_src1 & ~alu_src2;
wire [31:0] orn_result     = alu_src1 | ~alu_src2;
wire [31:0] sll_result     = alu_src1 << alu_src2[4:0];
wire [63:0] sr64_result    = {{32{op_sra & alu_src1[31]}}, alu_src1} >> alu_src2[4:0];
wire [31:0] sr_result      = sr64_result[31:0];

assign alu_result = ({32{op_add | op_sub}} & add_sub_result)
                  | ({32{op_slt}}  & slt_result)
                  | ({32{op_sltu}} & sltu_result)
                  | ({32{op_and}}  & and_result)
                  | ({32{op_nor}}  & nor_result)
                  | ({32{op_or}}   & or_result)
                  | ({32{op_xor}}  & xor_result)
                  | ({32{op_lui}}  & lui_result)
                  | ({32{op_andn}} & andn_result)
                  | ({32{op_orn}}  & orn_result)
                  | ({32{op_pcadd}} & add_sub_result)
                  | ({32{op_sll}}  & sll_result)
                  | ({32{op_srl | op_sra}} & sr_result);

assign alu_result_valid = 1'b1;

wire alu_lint = clk | reset | (|exe_pc);

endmodule
