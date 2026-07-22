`include "mycpu.h"

// ============================================================
// op_dec 模块（操作码生成：inst_* 独热 -> alu_op/br_op/mem_op/csr_op 等）
// ------------------------------------------------------------
// 被 decoder.v 内部例化复用，操作码编码体系（mycpu.h 独热宏）
// 不变，零改动。decoder.v 在其输出之上补充 futype 分类与 priv_vec 标记。
// 注意 inst_known（指令识别有效）输出供 decoder 生成 INE 异常，确认
//      新增指令时同步维护。
// ============================================================
module op_dec(
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
    input  wire        inst_csrrd,
    input  wire        inst_csrwr,
    input  wire        inst_csrxchg,
    input  wire        inst_rdcntvl_w,
    input  wire        inst_rdcntvh_w,
    input  wire        inst_rdcntid,
    input  wire        inst_ertn,
    input  wire        inst_syscall,
    input  wire        inst_break,
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
    input  wire        inst_ibar,
    input  wire        inst_ll_w,
    input  wire        inst_sc_w,
    input  wire        inst_pcaddi,
    input  wire        inst_andn,
    input  wire        inst_orn,
    input  wire        inst_dbar,
    input  wire        inst_idle,
    input  wire        inst_preld,
    input  wire        inst_cpucfg,
    output wire [`ALU_OP_NUM-1:0] alu_op,
    output wire [`BR_OP_NUM-1:0]  br_op,
    output wire [`MEM_OP_NUM-1:0] mem_op,
    output wire [`CSR_OP_NUM-1:0] csr_op,
    output wire [`WB_SRC_NUM-1:0] wb_src_op,
    output wire [`TLB_OP_NUM-1:0] tlb_op,
    output wire [`CACHE_OP_NUM-1:0] cache_op,
    output wire        inst_known
);

    assign alu_op[`ALU_OP_ADD] = inst_add_w | inst_addi_w | inst_jirl | inst_bl
                               | inst_ld_w | inst_ld_h | inst_ld_b | inst_ld_hu | inst_ld_bu
                               | inst_st_w | inst_st_b | inst_st_h
                               | inst_ll_w | inst_sc_w | inst_preld;
    assign alu_op[`ALU_OP_SUB] = inst_sub_w;
    assign alu_op[`ALU_OP_SLT] = inst_slt | inst_slti;
    assign alu_op[`ALU_OP_SLTU] = inst_sltu | inst_sltui;
    assign alu_op[`ALU_OP_AND] = inst_and | inst_andi;
    assign alu_op[`ALU_OP_NOR] = inst_nor;
    assign alu_op[`ALU_OP_OR] = inst_or | inst_ori;
    assign alu_op[`ALU_OP_XOR] = inst_xor | inst_xori;
    assign alu_op[`ALU_OP_SLL] = inst_slli_w | inst_sll_w;
    assign alu_op[`ALU_OP_SRL] = inst_srli_w | inst_srl_w;
    assign alu_op[`ALU_OP_SRA] = inst_srai_w | inst_sra_w;
    assign alu_op[`ALU_OP_LUI] = inst_lu12i_w;
    assign alu_op[`ALU_OP_MUL_W] = inst_mul_w;
    assign alu_op[`ALU_OP_MULH_W] = inst_mulh_w;
    assign alu_op[`ALU_OP_MULH_WU] = inst_mulh_wu;
    assign alu_op[`ALU_OP_DIV_W] = inst_div_w;
    assign alu_op[`ALU_OP_DIV_WU] = inst_div_wu;
    assign alu_op[`ALU_OP_MOD_W] = inst_mod_w;
    assign alu_op[`ALU_OP_MOD_WU] = inst_mod_wu;
    assign alu_op[`ALU_OP_ANDN] = inst_andn;
    assign alu_op[`ALU_OP_ORN]  = inst_orn;
    assign alu_op[`ALU_OP_PCADD] = inst_pcaddu12i | inst_pcaddi;

//分支跳转操作码生成
//inst_b 无条件跳转到目标地址，地址偏移值为 i26(offs26) 逻辑左移两位再符号拓展
//inst_bl 无条件跳转到目标地址，偏移值同上，同时将该指令的 pc+4 存到 r1
//inst_beq 将通用寄存器 rj 和 rd 的值比较，相等则跳转到目标地址
//inst_jirl 无条件跳转到目标地址，将 pc+4 存到 rd，目标地址为 i16(offs16) 逻辑左移两位符号拓展后加 rj 的值
//inst_bne 将通用寄存器 rj 和通用寄存器 rd 的值进行比较，如果两者不等则跳转到目标地址，否则不跳转。
//inst_blt 将通用寄存器 rj 和通用寄存器 rd 的值进行比较，如果 rj 的值小于 rd 的值（有符号比较），则跳转到目标地址，否则不跳转。
//inst_bge 将通用寄存器 rj 和通用寄存器 rd 的值进行比较，如果 rj 的值大于或等于 rd 的值（有符号比较），则跳转到目标地址，否则不跳转。
//inst_bltu 将通用寄存器 rj 和通用寄存器 rd 的值进行比较，如果 rj 的值小于 rd 的值（无符号比较），则跳转到目标地址，否则不跳转。
//inst_bgeu 将通用寄存器 rj 和通用寄存器 rd 的值进行比较，如果 rj 的值大于或等于 rd 的值（无符号比较），则跳转到目标地址，否则不跳转。
    assign br_op[`BR_OP_BEQ]  = inst_beq;
    assign br_op[`BR_OP_BNE]  = inst_bne;
    assign br_op[`BR_OP_JIRL] = inst_jirl;
    assign br_op[`BR_OP_BL]   = inst_bl;
    assign br_op[`BR_OP_B]    = inst_b;
    assign br_op[`BR_OP_BLT]  = inst_blt;
    assign br_op[`BR_OP_BGE]  = inst_bge;
    assign br_op[`BR_OP_BLTU] = inst_bltu;
    assign br_op[`BR_OP_BGEU] = inst_bgeu;

// mem_op 操作码生成（独热位定义见 mycpu.h 的 `MEM_OP_*`）
    assign mem_op[`MEM_OP_ST_W] = inst_st_w;
    assign mem_op[`MEM_OP_ST_B] = inst_st_b;
    assign mem_op[`MEM_OP_ST_H] = inst_st_h;
    assign mem_op[`MEM_OP_LD_W] = inst_ld_w;
    assign mem_op[`MEM_OP_LD_H] = inst_ld_h;
    assign mem_op[`MEM_OP_LD_B] = inst_ld_b;
    assign mem_op[`MEM_OP_LD_HU] = inst_ld_hu;
    assign mem_op[`MEM_OP_LD_BU] = inst_ld_bu;
    assign mem_op[`MEM_OP_LL_W]  = inst_ll_w;
    assign mem_op[`MEM_OP_SC_W]  = inst_sc_w;
// csr_op操作码生成
    assign csr_op[`CSR_OP_CSRRD] = inst_csrrd;
    assign csr_op[`CSR_OP_CSRWR] = inst_csrwr;
    assign csr_op[`CSR_OP_CSRXCHG] = inst_csrxchg;
// rdcnt_op操作码生成
    // assign rdcnt_op[`RDCNT_OP_RDCNTVL] = inst_rdcntvl_w;
    // assign rdcnt_op[`RDCNT_OP_RDCNTVH] = inst_rdcntvh_w;
    // assign rdcnt_op[`RDCNT_OP_RDCNTID] = inst_rdcntid;
// wb_src_op操作码生成
    assign wb_src_op[`WB_SRC_ALU] = inst_add_w | inst_addi_w | inst_slti | inst_sltui | inst_andi | inst_ori | inst_xori 
                                | inst_sub_w | inst_slli_w | inst_srli_w | inst_srai_w | inst_sll_w | inst_srl_w | inst_sra_w
                                | inst_lu12i_w | inst_pcaddu12i | inst_pcaddi
                                | inst_andn | inst_orn
                                | inst_mul_w | inst_mulh_w | inst_mulh_wu | inst_div_w | inst_div_wu | inst_mod_w | inst_mod_wu
                                | inst_sc_w  | inst_cpucfg;
    assign wb_src_op[`WB_SRC_MEM] = inst_ld_w | inst_ld_h | inst_ld_b | inst_ld_hu | inst_ld_bu | inst_ll_w;
    assign wb_src_op[`WB_SRC_CSR] = inst_csrrd | inst_csrwr | inst_csrxchg;
    assign wb_src_op[`WB_SRC_CNTVL] = inst_rdcntvl_w;
    assign wb_src_op[`WB_SRC_CNTVH] = inst_rdcntvh_w;
    assign wb_src_op[`WB_SRC_TID] = inst_rdcntid;
// tlb_op操作码生成
    assign tlb_op[`TLB_OP_TLBSRCH] = inst_tlbsrch;
    assign tlb_op[`TLB_OP_TLBRD] = inst_tlbrd;
    assign tlb_op[`TLB_OP_TLBWR] = inst_tlbwr;
    assign tlb_op[`TLB_OP_TLBFILL] = inst_tlbfill;
    assign tlb_op[`TLB_OP_INVTLB_0] = inst_invtlb_0;
    assign tlb_op[`TLB_OP_INVTLB_1] = inst_invtlb_1;
    assign tlb_op[`TLB_OP_INVTLB_2] = inst_invtlb_2;
    assign tlb_op[`TLB_OP_INVTLB_3] = inst_invtlb_3;
    assign tlb_op[`TLB_OP_INVTLB_4] = inst_invtlb_4;
    assign tlb_op[`TLB_OP_INVTLB_5] = inst_invtlb_5;
    assign tlb_op[`TLB_OP_INVTLB_6] = inst_invtlb_6;
//cache_op
    assign cache_op[`CACHE_OP_CACOP] = inst_cacop;
    assign cache_op[`CACHE_OP_IBAR] = inst_ibar;
// inst_known：指令识别信号，必须输出稳定 0/1（避免 reduction OR 遇到 X 传播，导致异常不触发）
    wire any_alu_op   = ((|alu_op)   === 1'b1);
    wire any_br_op    = ((|br_op)    === 1'b1);
    wire any_mem_op   = ((|mem_op)   === 1'b1);
    wire any_csr_op   = ((|csr_op)   === 1'b1);
    wire any_tlb_op   = ((|tlb_op)   === 1'b1);
    wire any_cache_op = ((|cache_op) === 1'b1);

    assign inst_known = any_alu_op | any_br_op | any_mem_op | any_csr_op | any_tlb_op | any_cache_op
                     | (inst_rdcntvl_w === 1'b1) | (inst_rdcntvh_w === 1'b1) | (inst_rdcntid === 1'b1)
                     | (inst_ertn === 1'b1) | (inst_syscall === 1'b1) | (inst_break === 1'b1)
                     | (inst_dbar === 1'b1)  | (inst_idle === 1'b1)
                     | (inst_preld === 1'b1) | (inst_cpucfg === 1'b1);

endmodule
