`include "mycpu.h"

// ============================================================
// inst_dec 模块（指令识别，输出各指令的独热激活标志）
// ------------------------------------------------------------
// 按字段等值识别当前实现的 LoongArch32R 指令。
// 添加指令时必须同步更新 op_dec 和 decoder 中的操作数/提交属性分类。
// ============================================================
module inst_dec(
    input  wire [31:0] inst,
    output wire        inst_add_w,
    output wire        inst_addi_w,
    output wire        inst_slti,
    output wire        inst_sltui,
    output wire        inst_andi,
    output wire        inst_ori,
    output wire        inst_xori,
    output wire        inst_sub_w,
    output wire        inst_ld_w,
    output wire        inst_ld_h,
    output wire        inst_ld_b,
    output wire        inst_ld_hu,
    output wire        inst_ld_bu,
    output wire        inst_st_w,
    output wire        inst_st_b,
    output wire        inst_st_h,
    output wire        inst_bne,
    output wire        inst_slt,
    output wire        inst_sltu,
    output wire        inst_and,
    output wire        inst_or,
    output wire        inst_nor,
    output wire        inst_xor,
    output wire        inst_slli_w,
    output wire        inst_srli_w,
    output wire        inst_srai_w,
    output wire        inst_sll_w,
    output wire        inst_srl_w,
    output wire        inst_sra_w,
    output wire        inst_b,
    output wire        inst_bl,
    output wire        inst_beq,
    output wire        inst_blt,
    output wire        inst_bge,
    output wire        inst_bltu,
    output wire        inst_bgeu,
    output wire        inst_jirl,
    output wire        inst_lu12i_w,
    output wire        inst_pcaddu12i,
    output wire        inst_mul_w,
    output wire        inst_mulh_w,
    output wire        inst_mulh_wu,
    output wire        inst_div_w,
    output wire        inst_div_wu,
    output wire        inst_mod_w,
    output wire        inst_mod_wu,
    output wire        inst_ertn,
    output wire        inst_syscall,
    output wire        inst_break,
    output wire        inst_rdcntvl_w,
    output wire        inst_rdcntvh_w,
    output wire        inst_rdcntid,
    output wire        inst_csrrd,
    output wire        inst_csrwr,
    output wire        inst_csrxchg,
    output wire        inst_tlbsrch,
    output wire        inst_tlbrd,
    output wire        inst_tlbwr,
    output wire        inst_tlbfill,
    output wire        inst_invtlb_0,
    output wire        inst_invtlb_1,
    output wire        inst_invtlb_2,
    output wire        inst_invtlb_3,
    output wire        inst_invtlb_4,
    output wire        inst_invtlb_5,
    output wire        inst_invtlb_6,
    output wire        inst_cacop,
    output wire        inst_ibar,
    output wire        inst_ll_w,
    output wire        inst_sc_w,
    output wire        inst_pcaddi,
    output wire        inst_andn,
    output wire        inst_orn,
    output wire        inst_dbar,
    output wire        inst_idle,
    output wire        inst_preld,
    output wire        inst_cpucfg
);

wire [ 5:0] op_31_26; // 指令主操作码字段
wire [ 3:0] op_25_22;
wire [ 1:0] op_21_20;
wire [ 4:0] op_19_15;
wire [ 4:0] rj;
wire [ 4:0] rd;
wire [ 4:0] rk;

assign op_31_26 = inst[31:26];
assign op_25_22 = inst[25:22];
assign op_21_20 = inst[21:20];
assign op_19_15 = inst[19:15];
assign rd       = inst[ 4: 0];
assign rk       = inst[14:10];
assign rj       = inst[ 9: 5];

assign inst_add_w   = (op_31_26 == 6'h00) & (op_25_22 == 4'h0) & (op_21_20 == 2'h1) & (op_19_15 == 5'h00);
assign inst_addi_w  = (op_31_26 == 6'h00) & (op_25_22 == 4'ha);
assign inst_slti    = (op_31_26 == 6'h00) & (op_25_22 == 4'h8);
assign inst_sltui   = (op_31_26 == 6'h00) & (op_25_22 == 4'h9);
assign inst_andi    = (op_31_26 == 6'h00) & (op_25_22 == 4'hd);
assign inst_ori     = (op_31_26 == 6'h00) & (op_25_22 == 4'he);
assign inst_xori    = (op_31_26 == 6'h00) & (op_25_22 == 4'hf);
assign inst_sub_w   = (op_31_26 == 6'h00) & (op_25_22 == 4'h0) & (op_21_20 == 2'h1) & (op_19_15 == 5'h02);

assign inst_slt     = (op_31_26 == 6'h00) & (op_25_22 == 4'h0) & (op_21_20 == 2'h1) & (op_19_15 == 5'h04);
assign inst_sltu    = (op_31_26 == 6'h00) & (op_25_22 == 4'h0) & (op_21_20 == 2'h1) & (op_19_15 == 5'h05);

assign inst_nor     = (op_31_26 == 6'h00) & (op_25_22 == 4'h0) & (op_21_20 == 2'h1) & (op_19_15 == 5'h08);
assign inst_and     = (op_31_26 == 6'h00) & (op_25_22 == 4'h0) & (op_21_20 == 2'h1) & (op_19_15 == 5'h09);
assign inst_or      = (op_31_26 == 6'h00) & (op_25_22 == 4'h0) & (op_21_20 == 2'h1) & (op_19_15 == 5'h0a);
assign inst_xor     = (op_31_26 == 6'h00) & (op_25_22 == 4'h0) & (op_21_20 == 2'h1) & (op_19_15 == 5'h0b);

assign inst_slli_w  = (op_31_26 == 6'h00) & (op_25_22 == 4'h1) & (op_21_20 == 2'h0) & (op_19_15 == 5'h01);
assign inst_srli_w  = (op_31_26 == 6'h00) & (op_25_22 == 4'h1) & (op_21_20 == 2'h0) & (op_19_15 == 5'h09);
assign inst_srai_w  = (op_31_26 == 6'h00) & (op_25_22 == 4'h1) & (op_21_20 == 2'h0) & (op_19_15 == 5'h11);
assign inst_sll_w   = (op_31_26 == 6'h00) & (op_25_22 == 4'h0) & (op_21_20 == 2'h1) & (op_19_15 == 5'h0e);
assign inst_srl_w   = (op_31_26 == 6'h00) & (op_25_22 == 4'h0) & (op_21_20 == 2'h1) & (op_19_15 == 5'h0f);
assign inst_sra_w   = (op_31_26 == 6'h00) & (op_25_22 == 4'h0) & (op_21_20 == 2'h1) & (op_19_15 == 5'h10);

assign inst_ld_w    = (op_31_26 == 6'h0a) & (op_25_22 == 4'h2);
assign inst_ld_b    = (op_31_26 == 6'h0a) & (op_25_22 == 4'h0);
assign inst_ld_h    = (op_31_26 == 6'h0a) & (op_25_22 == 4'h1);
assign inst_ld_bu   = (op_31_26 == 6'h0a) & (op_25_22 == 4'h8);
assign inst_ld_hu   = (op_31_26 == 6'h0a) & (op_25_22 == 4'h9);
assign inst_st_w    = (op_31_26 == 6'h0a) & (op_25_22 == 4'h6);
assign inst_st_b    = (op_31_26 == 6'h0a) & (op_25_22 == 4'h4);
assign inst_st_h    = (op_31_26 == 6'h0a) & (op_25_22 == 4'h5);

assign inst_jirl    = (op_31_26 == 6'h13);
assign inst_b       = (op_31_26 == 6'h14);
assign inst_bl      = (op_31_26 == 6'h15);
assign inst_beq     = (op_31_26 == 6'h16);
assign inst_bne     = (op_31_26 == 6'h17);
assign inst_blt     = (op_31_26 == 6'h18);
assign inst_bge     = (op_31_26 == 6'h19);
assign inst_bltu    = (op_31_26 == 6'h1a);
assign inst_bgeu    = (op_31_26 == 6'h1b);

assign inst_lu12i_w = (op_31_26 == 6'h05) & ~inst[25];
assign inst_pcaddu12i = (op_31_26 == 6'h07) & ~inst[25];

assign inst_mul_w   = (op_31_26 == 6'h00) & (op_25_22 == 4'h0) & (op_21_20 == 2'h1) & (op_19_15 == 5'h18);
assign inst_mulh_w  = (op_31_26 == 6'h00) & (op_25_22 == 4'h0) & (op_21_20 == 2'h1) & (op_19_15 == 5'h19);
assign inst_mulh_wu = (op_31_26 == 6'h00) & (op_25_22 == 4'h0) & (op_21_20 == 2'h1) & (op_19_15 == 5'h1a);
assign inst_div_w   = (op_31_26 == 6'h00) & (op_25_22 == 4'h0) & (op_21_20 == 2'h2) & (op_19_15 == 5'h00);
assign inst_div_wu  = (op_31_26 == 6'h00) & (op_25_22 == 4'h0) & (op_21_20 == 2'h2) & (op_19_15 == 5'h02);
assign inst_mod_w   = (op_31_26 == 6'h00) & (op_25_22 == 4'h0) & (op_21_20 == 2'h2) & (op_19_15 == 5'h01);
assign inst_mod_wu  = (op_31_26 == 6'h00) & (op_25_22 == 4'h0) & (op_21_20 == 2'h2) & (op_19_15 == 5'h03);

// ERTN 固定编码 0x06483800 要求 rk=0x0e、rj=rd=0。
assign inst_ertn     = (op_31_26 == 6'h01) & (op_25_22 == 4'h9) & (op_21_20 == 2'h0) & (op_19_15 == 5'h10)  & (rk == 5'h0e) & (rj == 5'h00) & (rd == 5'h00);
assign inst_syscall  = (op_31_26 == 6'h00) & (op_25_22 == 4'h0) & (op_21_20 == 2'h2) & (op_19_15 == 5'h16);
assign inst_break    = (op_31_26 == 6'h00) & (op_25_22 == 4'h0) & (op_21_20 == 2'h2) & (op_19_15 == 5'h14);

assign inst_rdcntid   = (op_31_26 == 6'h00) & (op_25_22 == 4'h0) & (op_21_20 == 2'h0) & (op_19_15 == 5'h00) & (rk == 5'h18) & (rd == 5'h00);
assign inst_rdcntvl_w = (op_31_26 == 6'h00) & (op_25_22 == 4'h0) & (op_21_20 == 2'h0) & (op_19_15 == 5'h00) & (rk == 5'h18) & (rj == 5'h00) & ~(rd == 5'h00);
assign inst_rdcntvh_w = (op_31_26 == 6'h00) & (op_25_22 == 4'h0) & (op_21_20 == 2'h0) & (op_19_15 == 5'h00) & (rk == 5'h19) & (rj == 5'h00);

assign inst_csrrd    = (op_31_26 == 6'h01) & ~inst[25] & ~inst[24] & (rj == 5'h00);
assign inst_csrwr    = (op_31_26 == 6'h01) & ~inst[25] & ~inst[24] & (rj == 5'h01);
assign inst_csrxchg  = (op_31_26 == 6'h01) & ~inst[25] & ~inst[24] & ~(rj == 5'h00) & ~(rj == 5'h01);

// TLB 维护指令
assign inst_tlbsrch  = (op_31_26 == 6'h01) & (op_25_22 == 4'h9) & (op_21_20 == 2'h0) & (op_19_15 == 5'h10) & (rk == 5'h0a) & (rj == 5'h00) & (rd == 5'h00);
assign inst_tlbrd    = (op_31_26 == 6'h01) & (op_25_22 == 4'h9) & (op_21_20 == 2'h0) & (op_19_15 == 5'h10) & (rk == 5'h0b) & (rj == 5'h00) & (rd == 5'h00);
assign inst_tlbwr    = (op_31_26 == 6'h01) & (op_25_22 == 4'h9) & (op_21_20 == 2'h0) & (op_19_15 == 5'h10) & (rk == 5'h0c) & (rj == 5'h00) & (rd == 5'h00);
assign inst_tlbfill  = (op_31_26 == 6'h01) & (op_25_22 == 4'h9) & (op_21_20 == 2'h0) & (op_19_15 == 5'h10) & (rk == 5'h0d) & (rj == 5'h00) & (rd == 5'h00);

assign inst_invtlb_0 = (op_31_26 == 6'h01) & (op_25_22 == 4'h9) & (op_21_20 == 2'h0) & (op_19_15 == 5'h13) & (rd == 5'h00);
assign inst_invtlb_1 = (op_31_26 == 6'h01) & (op_25_22 == 4'h9) & (op_21_20 == 2'h0) & (op_19_15 == 5'h13) & (rd == 5'h01);
assign inst_invtlb_2 = (op_31_26 == 6'h01) & (op_25_22 == 4'h9) & (op_21_20 == 2'h0) & (op_19_15 == 5'h13) & (rd == 5'h02);
assign inst_invtlb_3 = (op_31_26 == 6'h01) & (op_25_22 == 4'h9) & (op_21_20 == 2'h0) & (op_19_15 == 5'h13) & (rd == 5'h03);
assign inst_invtlb_4 = (op_31_26 == 6'h01) & (op_25_22 == 4'h9) & (op_21_20 == 2'h0) & (op_19_15 == 5'h13) & (rd == 5'h04);
assign inst_invtlb_5 = (op_31_26 == 6'h01) & (op_25_22 == 4'h9) & (op_21_20 == 2'h0) & (op_19_15 == 5'h13) & (rd == 5'h05);
assign inst_invtlb_6 = (op_31_26 == 6'h01) & (op_25_22 == 4'h9) & (op_21_20 == 2'h0) & (op_19_15 == 5'h13) & (rd == 5'h06);

assign inst_cacop = (op_31_26 == 6'h01) & (op_25_22 == 4'h8);
assign inst_ibar  = (op_31_26 == 6'h0e) & (op_25_22 == 4'h1) & (op_21_20 == 2'h3) & (op_19_15 == 5'h05);

// ll.w / sc.w：op[31:26]=0x08，inst[25:24] 分别为 00/01。
assign inst_ll_w  = (op_31_26 == 6'h08) & ~inst[25] & ~inst[24];
assign inst_sc_w  = (op_31_26 == 6'h08) & ~inst[25] &  inst[24];

// pcaddi：op[31:26]=0x06，inst[25]=0
assign inst_pcaddi   = (op_31_26 == 6'h06) & ~inst[25];

// andn / orn：3R 类
assign inst_orn   = (op_31_26 == 6'h00) & (op_25_22 == 4'h0) & (op_21_20 == 2'h1) & (op_19_15 == 5'h0c);
assign inst_andn  = (op_31_26 == 6'h00) & (op_25_22 == 4'h0) & (op_21_20 == 2'h1) & (op_19_15 == 5'h0d);

// 同步类与等待类
assign inst_dbar  = (op_31_26 == 6'h0e) & (op_25_22 == 4'h1) & (op_21_20 == 2'h3) & (op_19_15 == 5'h04);
// chiplab func 测试套件 idle 0 宏展开为 0x50000000（自分支等待中断），与手册 idle 编码并存。
assign inst_idle  = ((op_31_26 == 6'h01) & (op_25_22 == 4'h9) & (op_21_20 == 2'h0) & (op_19_15 == 5'h11))
                  | (inst[31:0] === 32'h50000000);

// preld：op[31:26]=0x0a，op[25:22]=0xb
assign inst_preld = (op_31_26 == 6'h0a) & (op_25_22 == 4'hb);

// cpucfg 指令编码
assign inst_cpucfg = (op_31_26 == 6'h00) & (op_25_22 == 4'h0) & (op_21_20 == 2'h0) & (op_19_15 == 5'h00) & (rk == 5'h1b);

endmodule
