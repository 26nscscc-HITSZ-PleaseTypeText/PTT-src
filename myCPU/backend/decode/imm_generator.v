`include "mycpu.h"

// ============================================================
// imm_generator 模块（立即数/分支偏移生成器）
// ------------------------------------------------------------
// 被 decoder.v 内部例化复用（原 IDport 已删除）。
// 根据指令类型选择并扩展不同位宽的立即数字段，alu_imm 进 RS 的 imm 字段，
// br_imm 进 RS 的 br_offs 字段（分支目标 = pc + br_offs 在 fu_alu 算）。
// 零改动复用。
// ============================================================
module imm_generator (
    input wire [31:0] inst,
    input wire inst_add_w,
    input wire inst_addi_w,
    input wire inst_slti,
    input wire inst_sltui,
    input wire inst_andi,
    input wire inst_ori,
    input wire inst_xori,
    input wire inst_sub_w,
    input wire inst_ld_w,
    input wire inst_ld_h,
    input wire inst_ld_b,
    input wire inst_ld_hu,
    input wire inst_ld_bu,
    input wire inst_st_w,
    input wire inst_st_b,
    input wire inst_st_h,
    input wire inst_bne,
    input wire inst_slt,
    input wire inst_sltu,
    input wire inst_and,
    input wire inst_or,
    input wire inst_nor,
    input wire inst_xor,
    input wire inst_slli_w,
    input wire inst_srli_w,
    input wire inst_srai_w,
    input wire inst_b,
    input wire inst_bl,
    input wire inst_beq,
    input wire inst_blt,
    input wire inst_bge,
    input wire inst_bltu,
    input wire inst_bgeu,
    input wire inst_jirl,
    input wire inst_lu12i_w,
    input wire inst_pcaddu12i,
    input wire inst_ll_w,
    input wire inst_sc_w,
    input wire inst_pcaddi,
    input wire inst_preld,
    output wire [31:0] alu_imm,            // 送 ALU 第二源用的立即数（扩展后的）
    output wire [31:0] br_imm              // 分支偏移（已对齐、扩展）
);
    
    wire [4:0 ] ui5;
    wire [11:0] i12;
    wire [13:0] i14;
    wire [15:0] i16;
    wire [19:0] i20;
    wire [25:0] i26;
    
    wire need_ui5;
    wire need_si12;
    wire need_si14_shl2;
    wire need_si16;
    wire need_si20;
    wire need_si20_shl2;
    wire need_si26;
    wire src2_is_4;
    
    
    assign need_ui5  = inst_slli_w | inst_srli_w | inst_srai_w;
    assign need_si12 = inst_addi_w | inst_slti | inst_sltui | inst_ld_w | inst_st_w | inst_st_b 
                     | inst_st_h | inst_ld_h | inst_ld_b | inst_ld_hu | inst_ld_bu | inst_preld;
    // ll.w / sc.w 使用 si14 << 2（14 位有符号偏移再左移 2 位作字偏移）
    assign need_si14_shl2 = inst_ll_w | inst_sc_w;
    assign need_si16 = inst_jirl | inst_beq | inst_bne 
                     | inst_blt | inst_bge | inst_bltu | inst_bgeu;
    assign need_si20 = inst_lu12i_w | inst_pcaddu12i;
    // pcaddi 使用 si20 << 2
    assign need_si20_shl2 = inst_pcaddi;
    assign need_si26 = inst_b | inst_bl;
    assign src2_is_4 = inst_jirl | inst_bl;

    // 勿对 inst 做归约或(|inst)：inst 在复位/取指边界可能含 X，会毒化 alu_imm 进而污染整级流水写回
    wire imm_unused_inputs;
    assign imm_unused_inputs = inst_add_w | inst_sub_w | inst_and | inst_or | inst_nor | inst_xor
                             | inst_slt | inst_sltu;
    
    assign ui5 = inst[14:10];
    assign i12 = inst[21:10];
    assign i14 = inst[23:10];
    assign i16 = inst[25:10];
    assign i20 = inst[24: 5];
    assign i26 = {inst[9:0], inst[25:10]};   // LoongArch B/BL: si26 = {offs[25:16], offs[15:0]}
    
    assign alu_imm = (src2_is_4 ? 32'h4 :
                     need_si20_shl2 ? {{10{i20[19]}}, i20[19:0], 2'b00} :
                     need_si20 ? {{12{i20[19]}}, i20[19:0], 12'b0} :
                     (inst_andi | inst_ori | inst_xori) ? {20'b0, i12[11:0]} :
                     need_si14_shl2 ? {{16{i14[13]}}, i14[13:0], 2'b00} :
                     need_si12 ? {{20{i12[11]}} , i12[11:0]} :
                     need_ui5  ? {27'b0 , ui5[4:0]} :
                     need_si26 ? {{ 4{i26[25]}} , i26[25:0] , 2'b00}:
                     need_si16 ? {{14{i16[15]}} , i16[15:0] , 2'b00}:
                     32'b0) | (32'b0 & {32{imm_unused_inputs}});

    assign br_imm = (need_si26 ? {{ 4{i26[25]}} , i26[25:0] , 2'b00}:
                    need_si16 ? {{14{i16[15]}} , i16[15:0] , 2'b00}:
                    32'b0) | (32'b0 & {32{imm_unused_inputs}});
    
endmodule
