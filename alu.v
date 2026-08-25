`include "mycpu.h"

// ============================================================
// alu 模块（fu_alu 使用的纯组合整数运算核）
// ------------------------------------------------------------
// - 只承担单周期组合运算：加减/逻辑/移位/比较/LUI（含 ANDN/ORN）；
// - 乘除运算由 fu_mdu 执行；
// - PCADD 使用 exe_pc，其余加减运算使用 alu_src1。
// - 模块无状态，alu_result 始终为当前输入的组合结果。
// ============================================================
module alu(
    // 紧凑位序：{PCADD, ORN, ANDN, LUI..ADD}，去掉全局操作码中的 MDU 空洞。
    input  wire [14:0]             alu_op,
    input  wire [31:0]             alu_src1,
    input  wire [31:0]             alu_src2,
    input  wire [31:0]             exe_pc,
    output wire [31:0]             alu_result
);

wire op_add   = alu_op[0];
wire op_sub   = alu_op[1];
wire op_slt   = alu_op[2];
wire op_sltu  = alu_op[3];
wire op_and   = alu_op[4];
wire op_nor   = alu_op[5];
wire op_or    = alu_op[6];
wire op_xor   = alu_op[7];
wire op_sll   = alu_op[8];
wire op_srl   = alu_op[9];
wire op_sra   = alu_op[10];
wire op_lui   = alu_op[11];
wire op_andn  = alu_op[12];
wire op_orn   = alu_op[13];
wire op_pcadd = alu_op[14];

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
wire signed [31:0] signed_alu_src1 = alu_src1;
wire [31:0] sra_result = signed_alu_src1 >>> alu_src2[4:0];
wire [31:0] srl_result = alu_src1 >> alu_src2[4:0];
wire [31:0] sr_result  = ({32{op_sra}} & sra_result)
                       | ({32{op_srl}} & srl_result);

// The add/sub/PCADD result is the latency-sensitive producer for dependent
// memory addresses.  Select that carry-chain result directly instead of
// feeding it through the wide one-hot result OR used by non-arithmetic ops.
// ALU operation bits are one-hot, so this priority form is functionally
// identical while giving the arithmetic path a dedicated mux branch.
wire op_add_result = op_add | op_sub | op_pcadd;
wire [31:0] non_add_result = ({32{op_slt}}  & slt_result)
                  | ({32{op_sltu}} & sltu_result)
                  | ({32{op_and}}  & and_result)
                  | ({32{op_nor}}  & nor_result)
                  | ({32{op_or}}   & or_result)
                  | ({32{op_xor}}  & xor_result)
                  | ({32{op_lui}}  & lui_result)
                  | ({32{op_andn}} & andn_result)
                  | ({32{op_orn}}  & orn_result)
                  | ({32{op_sll}}  & sll_result)
                  | ({32{op_srl | op_sra}} & sr_result);
assign alu_result = op_add_result ? add_sub_result : non_add_result;

endmodule
