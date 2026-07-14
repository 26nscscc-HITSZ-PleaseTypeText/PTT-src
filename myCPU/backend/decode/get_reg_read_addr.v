`include "mycpu.h"

// ============================================================
// get_reg_read_addr 模块（源寄存器读地址生成）
// ------------------------------------------------------------
// 被 decoder.v 内部例化复用，生成 src0/src1 读地址：
// - 大多数指令：src0 读 rj，src1 读 rk/rd（store 数据、分支比较第二源）；
// - CSR 指令特例：csrwr 源在 rd；csrxchg mask=rj、写值=rd。
// 零改动复用（decoder 直接把两个输出当 src0_addr/src1_addr）。
// ============================================================
module get_reg_read_addr(
    input  wire        reset,
    input  wire [31:0] inst,
    input  wire        inst_add_w,
    input  wire        inst_addi_w,
    input  wire        inst_slti,
    input  wire        inst_sltui,
    input  wire        inst_andi,
    input  wire        inst_ori,
    input  wire        inst_xori,
    input  wire        inst_sub_w,
    input  wire        inst_ld_w,
    input  wire        inst_ld_h,
    input  wire        inst_ld_b,
    input  wire        inst_ld_hu,
    input  wire        inst_ld_bu,
    input  wire        inst_st_w,
    input  wire        inst_st_b,
    input  wire        inst_st_h,
    input  wire        inst_bne,
    input  wire        inst_slt,
    input  wire        inst_sltu,
    input  wire        inst_and,
    input  wire        inst_or,
    input  wire        inst_nor,
    input  wire        inst_xor,
    input  wire        inst_slli_w,
    input  wire        inst_srli_w,
    input  wire        inst_srai_w,
    input  wire        inst_sll_w,
    input  wire        inst_srl_w,
    input  wire        inst_sra_w,
    input  wire        inst_b,
    input  wire        inst_bl,
    input  wire        inst_beq,
    input  wire        inst_blt,
    input  wire        inst_bge,
    input  wire        inst_bltu,
    input  wire        inst_bgeu,
    input  wire        inst_jirl,
    input  wire        inst_lu12i_w,
    input  wire        inst_pcaddu12i,
    input  wire        inst_mul_w,
    input  wire        inst_mulh_w,
    input  wire        inst_mulh_wu,
    input  wire        inst_div_w,
    input  wire        inst_div_wu,
    input  wire        inst_mod_w,
    input  wire        inst_mod_wu,
    input  wire        inst_ertn,
    input  wire        inst_syscall,
    input  wire        inst_break,
    input  wire        inst_rdcntvl_w,
    input  wire        inst_rdcntvh_w,
    input  wire        inst_rdcntid,
    input  wire        inst_csrrd,
    input  wire        inst_csrwr,
    input  wire        inst_csrxchg,
    input  wire        inst_tlbsrch,
    input  wire        inst_tlbrd,
    input  wire        inst_tlbwr,
    input  wire        inst_tlbfill,
    input  wire        inst_invtlb_0,
    input  wire        inst_invtlb_1,
    input  wire        inst_invtlb_2,
    input  wire        inst_invtlb_3,
    input  wire        inst_invtlb_4,
    input  wire        inst_invtlb_5,
    input  wire        inst_invtlb_6,
    input  wire        inst_cacop,
    input  wire        inst_ll_w,
    input  wire        inst_sc_w,
    input  wire        inst_andn,
    input  wire        inst_orn,
    input  wire        inst_preld,
    input  wire        inst_cpucfg,
    output wire [ 4:0] rf_raddr1,
    output wire [ 4:0] rf_raddr2
);

wire [4:0] rd;
wire [4:0] rj;
wire [4:0] rk;
wire       need_rj;
wire       need_rk;
wire       src1_reg_is_rd;
wire       src2_reg_is_rd;
wire       grra_unused_inputs;

assign rd = inst[ 4: 0];
assign rj = inst[ 9: 5];
assign rk = inst[14:10];

//v1版本使用无意义，当时好像就是为了过仿真和编译
assign grra_unused_inputs = reset | inst_lu12i_w | inst_pcaddu12i | (|inst);

assign need_rj = inst_add_w  | inst_addi_w | inst_slti | inst_sltui
               | inst_andi   | inst_ori    | inst_xori | inst_sub_w
               | inst_ld_w   | inst_ld_h   | inst_ld_b | inst_ld_hu | inst_ld_bu
               | inst_st_w   | inst_st_b   | inst_st_h
               | inst_slt    | inst_sltu   | inst_and  | inst_or    | inst_nor
               | inst_xor    | inst_slli_w | inst_srli_w | inst_srai_w
               | inst_sll_w  | inst_srl_w  | inst_sra_w
               | inst_beq    | inst_bne    | inst_jirl
               | inst_blt    | inst_bge    | inst_bltu | inst_bgeu
               | inst_mul_w  | inst_mulh_w | inst_mulh_wu
               | inst_div_w  | inst_div_wu | inst_mod_w | inst_mod_wu
               | inst_invtlb_4 | inst_invtlb_5 | inst_invtlb_6
               | inst_csrxchg | inst_rdcntid | inst_cacop
               | inst_ll_w   | inst_sc_w   | inst_andn | inst_orn
               | inst_preld  | inst_cpucfg;

assign need_rk = inst_add_w | inst_sub_w | inst_slt | inst_sltu
               | inst_and   | inst_or    | inst_nor | inst_xor
               | inst_sll_w | inst_srl_w | inst_sra_w
               | inst_mul_w | inst_mulh_w | inst_mulh_wu
               | inst_invtlb_5 | inst_invtlb_6
               | inst_div_w | inst_div_wu | inst_mod_w | inst_mod_wu
               | inst_andn  | inst_orn;

assign src1_reg_is_rd = 1'b0;
assign src2_reg_is_rd = inst_st_w | inst_beq | inst_bne | inst_st_b | inst_st_h
                      | inst_blt | inst_bge | inst_bltu | inst_bgeu
                      | inst_csrwr | inst_csrxchg | inst_sc_w;

assign rf_raddr1 = src1_reg_is_rd ? rd
                 : (need_rj ? rj : (5'd0 | (5'd0 & {5{grra_unused_inputs}})));
assign rf_raddr2 = src2_reg_is_rd ? rd
                 : (need_rk ? rk : (5'd0 | (5'd0 & {5{grra_unused_inputs}})));

endmodule
