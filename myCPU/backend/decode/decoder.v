// ============================================================
// decoder 模块（单条指令译码器，纯组合，顶层例化 `MACHINE_WIDTH 份）
// ------------------------------------------------------------
// 功能：
// - 把一条 32bit 指令译成乱序后端所需的统一 uop 信息：
//   * FU 类型（决定进哪个保留站）：FU_ALU / FU_MEM / FU_MDU
//   * 各类操作码（alu_op/br_op/mem_op/csr_op/tlb_op/wb_src_op，沿用原独热体系）
//   * 源/目的寄存器（src0=rj 类，src1=rk/rd 类）、是否使用、是否写回
//   * 立即数与分支偏移
//   * 特权/特殊提交类标记 priv_vec（csr写/ertn/tlb维护/cacop/idle/ll/sc/ibar）
//   * 译码异常（INE/SYS/BRK/IPE）
// - 内部例化并复用旧五级流水的译码工具（inst_dec / op_dec /
//   imm_generator / get_reg_read_addr），这些模块的指令识别逻辑可整体复用。
//
// 端口：
// - inst_i / pc_i      ：指令与 PC
// - csr_plv_i          ：当前特权级（检测 IPE：用户态执行特权指令）
// - csr_llbit_i        ：LLBIT 当前值（sc.w 译码期决定行为，见 TODO）
// - 输出 uop 各字段（全组合）
// ============================================================
`include "mycpu.h"

module decoder(
    input  wire [31:0]                inst_i,
    input  wire [31:0]                pc_i,
    input  wire [1:0]                 csr_plv_i,         // 当前特权级（CRMD.PLV）
    input  wire                       csr_llbit_i,       // 当前 LLBIT（sc.w 行为选择）

    // ---------------- FU 类型与操作码 ----------------
    output wire [`FU_NUM-1:0]         futype_o,          // 独热：进哪个保留站
    output wire [`ALU_OP_NUM-1:0]     alu_op_o,
    output wire [`BR_OP_NUM-1:0]      br_op_o,
    output wire [`MEM_OP_NUM-1:0]     mem_op_o,
    output wire [`CSR_OP_NUM-1:0]     csr_op_o,
    output wire [`TLB_OP_NUM-1:0]     tlb_op_o,
    output wire [`WB_SRC_NUM-1:0]     wb_src_op_o,
    output wire [13:0]                csr_num_o,         // CSR 号（高位补 0 到 14 位）
    output wire [4:0]                 cacop_code_o,      // cacop 的 code 域（inst[4:0]）

    // ---------------- 寄存器与立即数 ----------------
    output wire [4:0]                 src0_addr_o,       // 源 0 读地址（rj 类）
    output wire [4:0]                 src1_addr_o,       // 源 1 读地址（rk/rd 类，store 数据/csr 写值）
    output wire                       use_src0_o,        // 是否真正使用源 0
    output wire                       use_src1_o,        // 是否真正使用源 1
    output wire [4:0]                 rd_addr_o,         // 目的寄存器
    output wire                       rf_we_o,           // 是否写回（r0 恒不写，内部处理）
    output wire [31:0]                imm_o,             // ALU 立即数（已扩展）
    output wire                       use_imm_o,         // ALU 第二操作数取 imm
    output wire [31:0]                br_offs_o,         // 分支偏移（已扩展对齐）

    // ---------------- 分支属性 ----------------
    output wire                       is_branch_o,       // 任意分支/跳转
    output wire [`BR_TYPE_W-1:0]      br_type_o,         // COND/UNCOND/CALL/RET

    // ---------------- 提交特殊处理类 ----------------
    output wire [`PRIV_NUM-1:0]       priv_vec_o,        // 独热特权/特殊类标记
    output wire                       is_nop_o,          // 真 NOP（andi r0,r0,0 等）/dbar/preld：不占 FU

    // ---------------- 访存属性 ----------------
    output wire                       is_load_o,
    output wire                       is_store_o,

    // ---------------- 译码异常 ----------------
    output wire [`EXCP_NUM-1:0]       excp_o             // INE/SYS/BRK/IPE 四位可能置位
);


wire inst_add_w;
wire inst_addi_w;
wire inst_slti;
wire inst_sltui;
wire inst_andi;
wire inst_ori;
wire inst_xori;
wire inst_sub_w;
wire inst_ld_w;
wire inst_ld_h;
wire inst_ld_b;
wire inst_ld_hu;
wire inst_ld_bu;
wire inst_st_w;
wire inst_st_b;
wire inst_st_h;
wire inst_bne;
wire inst_slt;
wire inst_sltu;
wire inst_and;
wire inst_or;
wire inst_nor;
wire inst_xor;
wire inst_slli_w;
wire inst_srli_w;
wire inst_srai_w;
wire inst_sll_w;
wire inst_srl_w;
wire inst_sra_w;
wire inst_b;
wire inst_bl;
wire inst_beq;
wire inst_blt;
wire inst_bge;
wire inst_bltu;
wire inst_bgeu;
wire inst_jirl;
wire inst_lu12i_w;
wire inst_pcaddu12i;
wire inst_mul_w;
wire inst_mulh_w;
wire inst_mulh_wu;
wire inst_div_w;
wire inst_div_wu;
wire inst_mod_w;
wire inst_mod_wu;
wire inst_ertn;
wire inst_syscall;
wire inst_break;
wire inst_rdcntvl_w;
wire inst_rdcntvh_w;
wire inst_rdcntid;
wire inst_csrrd;
wire inst_csrwr;
wire inst_csrxchg;
wire inst_tlbsrch;
wire inst_tlbrd;
wire inst_tlbwr;
wire inst_tlbfill;
wire inst_invtlb_0;
wire inst_invtlb_1;
wire inst_invtlb_2;
wire inst_invtlb_3;
wire inst_invtlb_4;
wire inst_invtlb_5;
wire inst_invtlb_6;
wire inst_cacop;
wire inst_ibar;
wire inst_ll_w;
wire inst_sc_w;
wire inst_pcaddi;
wire inst_andn;
wire inst_orn;
wire inst_dbar;
wire inst_idle;
wire inst_preld;
wire inst_cpucfg;

inst_dec u_inst_dec(
    .inst(inst_i),
    .inst_add_w(inst_add_w), .inst_addi_w(inst_addi_w), .inst_slti(inst_slti),
    .inst_sltui(inst_sltui), .inst_andi(inst_andi), .inst_ori(inst_ori),
    .inst_xori(inst_xori), .inst_sub_w(inst_sub_w), .inst_ld_w(inst_ld_w),
    .inst_ld_h(inst_ld_h), .inst_ld_b(inst_ld_b), .inst_ld_hu(inst_ld_hu),
    .inst_ld_bu(inst_ld_bu), .inst_st_w(inst_st_w), .inst_st_b(inst_st_b),
    .inst_st_h(inst_st_h), .inst_bne(inst_bne), .inst_slt(inst_slt),
    .inst_sltu(inst_sltu), .inst_and(inst_and), .inst_or(inst_or),
    .inst_nor(inst_nor), .inst_xor(inst_xor), .inst_slli_w(inst_slli_w),
    .inst_srli_w(inst_srli_w), .inst_srai_w(inst_srai_w), .inst_sll_w(inst_sll_w),
    .inst_srl_w(inst_srl_w), .inst_sra_w(inst_sra_w), .inst_b(inst_b),
    .inst_bl(inst_bl), .inst_beq(inst_beq), .inst_blt(inst_blt),
    .inst_bge(inst_bge), .inst_bltu(inst_bltu), .inst_bgeu(inst_bgeu),
    .inst_jirl(inst_jirl), .inst_lu12i_w(inst_lu12i_w),
    .inst_pcaddu12i(inst_pcaddu12i), .inst_mul_w(inst_mul_w),
    .inst_mulh_w(inst_mulh_w), .inst_mulh_wu(inst_mulh_wu),
    .inst_div_w(inst_div_w), .inst_div_wu(inst_div_wu), .inst_mod_w(inst_mod_w),
    .inst_mod_wu(inst_mod_wu), .inst_ertn(inst_ertn), .inst_syscall(inst_syscall),
    .inst_break(inst_break), .inst_rdcntvl_w(inst_rdcntvl_w),
    .inst_rdcntvh_w(inst_rdcntvh_w), .inst_rdcntid(inst_rdcntid),
    .inst_csrrd(inst_csrrd), .inst_csrwr(inst_csrwr), .inst_csrxchg(inst_csrxchg),
    .inst_tlbsrch(inst_tlbsrch), .inst_tlbrd(inst_tlbrd), .inst_tlbwr(inst_tlbwr),
    .inst_tlbfill(inst_tlbfill), .inst_invtlb_0(inst_invtlb_0),
    .inst_invtlb_1(inst_invtlb_1), .inst_invtlb_2(inst_invtlb_2),
    .inst_invtlb_3(inst_invtlb_3), .inst_invtlb_4(inst_invtlb_4),
    .inst_invtlb_5(inst_invtlb_5), .inst_invtlb_6(inst_invtlb_6),
    .inst_cacop(inst_cacop), .inst_ibar(inst_ibar), .inst_ll_w(inst_ll_w),
    .inst_sc_w(inst_sc_w), .inst_pcaddi(inst_pcaddi), .inst_andn(inst_andn),
    .inst_orn(inst_orn), .inst_dbar(inst_dbar), .inst_idle(inst_idle),
    .inst_preld(inst_preld), .inst_cpucfg(inst_cpucfg)
);

wire [`ALU_OP_NUM-1:0]   alu_op_raw;
wire [`BR_OP_NUM-1:0]    br_op_raw;
wire [`MEM_OP_NUM-1:0]   mem_op_raw;
wire [`CSR_OP_NUM-1:0]   csr_op_raw;
wire [`TLB_OP_NUM-1:0]   tlb_op_raw;
wire [`WB_SRC_NUM-1:0]   wb_src_op_raw;
wire [`CACHE_OP_NUM-1:0] cache_op_raw;
wire                     inst_known;

op_dec u_op_dec(
    .inst_add_w(inst_add_w), .inst_addi_w(inst_addi_w), .inst_slti(inst_slti),
    .inst_sltui(inst_sltui), .inst_andi(inst_andi), .inst_ori(inst_ori),
    .inst_xori(inst_xori), .inst_sub_w(inst_sub_w), .inst_ld_w(inst_ld_w),
    .inst_ld_h(inst_ld_h), .inst_ld_b(inst_ld_b), .inst_ld_hu(inst_ld_hu),
    .inst_ld_bu(inst_ld_bu), .inst_st_w(inst_st_w), .inst_st_b(inst_st_b),
    .inst_st_h(inst_st_h), .inst_bne(inst_bne), .inst_slt(inst_slt),
    .inst_sltu(inst_sltu), .inst_and(inst_and), .inst_or(inst_or),
    .inst_nor(inst_nor), .inst_xor(inst_xor), .inst_slli_w(inst_slli_w),
    .inst_srli_w(inst_srli_w), .inst_srai_w(inst_srai_w), .inst_sll_w(inst_sll_w),
    .inst_srl_w(inst_srl_w), .inst_sra_w(inst_sra_w), .inst_b(inst_b),
    .inst_bl(inst_bl), .inst_beq(inst_beq), .inst_blt(inst_blt),
    .inst_bge(inst_bge), .inst_bltu(inst_bltu), .inst_bgeu(inst_bgeu),
    .inst_jirl(inst_jirl), .inst_lu12i_w(inst_lu12i_w),
    .inst_pcaddu12i(inst_pcaddu12i), .inst_mul_w(inst_mul_w),
    .inst_mulh_w(inst_mulh_w), .inst_mulh_wu(inst_mulh_wu),
    .inst_div_w(inst_div_w), .inst_div_wu(inst_div_wu), .inst_mod_w(inst_mod_w),
    .inst_mod_wu(inst_mod_wu), .inst_csrrd(inst_csrrd), .inst_csrwr(inst_csrwr),
    .inst_csrxchg(inst_csrxchg), .inst_rdcntvl_w(inst_rdcntvl_w),
    .inst_rdcntvh_w(inst_rdcntvh_w), .inst_rdcntid(inst_rdcntid),
    .inst_ertn(inst_ertn), .inst_syscall(inst_syscall), .inst_break(inst_break),
    .inst_tlbsrch(inst_tlbsrch), .inst_tlbrd(inst_tlbrd), .inst_tlbwr(inst_tlbwr),
    .inst_tlbfill(inst_tlbfill), .inst_invtlb_0(inst_invtlb_0),
    .inst_invtlb_1(inst_invtlb_1), .inst_invtlb_2(inst_invtlb_2),
    .inst_invtlb_3(inst_invtlb_3), .inst_invtlb_4(inst_invtlb_4),
    .inst_invtlb_5(inst_invtlb_5), .inst_invtlb_6(inst_invtlb_6),
    .inst_cacop(inst_cacop), .inst_ibar(inst_ibar), .inst_ll_w(inst_ll_w),
    .inst_sc_w(inst_sc_w), .inst_pcaddi(inst_pcaddi), .inst_andn(inst_andn),
    .inst_orn(inst_orn), .inst_dbar(inst_dbar), .inst_idle(inst_idle),
    .inst_preld(inst_preld), .inst_cpucfg(inst_cpucfg),
    .alu_op(alu_op_raw), .br_op(br_op_raw), .mem_op(mem_op_raw),
    .csr_op(csr_op_raw), .wb_src_op(wb_src_op_raw), .tlb_op(tlb_op_raw),
    .cache_op(cache_op_raw), .inst_known(inst_known)
);

wire [31:0] imm_raw;
wire [31:0] br_offs_raw;

imm_generator u_imm_generator(
    .inst(inst_i),
    .inst_add_w(inst_add_w), .inst_addi_w(inst_addi_w), .inst_slti(inst_slti),
    .inst_sltui(inst_sltui), .inst_andi(inst_andi), .inst_ori(inst_ori),
    .inst_xori(inst_xori), .inst_sub_w(inst_sub_w), .inst_ld_w(inst_ld_w),
    .inst_ld_h(inst_ld_h), .inst_ld_b(inst_ld_b), .inst_ld_hu(inst_ld_hu),
    .inst_ld_bu(inst_ld_bu), .inst_st_w(inst_st_w), .inst_st_b(inst_st_b),
    .inst_st_h(inst_st_h), .inst_bne(inst_bne), .inst_slt(inst_slt),
    .inst_sltu(inst_sltu), .inst_and(inst_and), .inst_or(inst_or),
    .inst_nor(inst_nor), .inst_xor(inst_xor), .inst_slli_w(inst_slli_w),
    .inst_srli_w(inst_srli_w), .inst_srai_w(inst_srai_w), .inst_b(inst_b),
    .inst_bl(inst_bl), .inst_beq(inst_beq), .inst_blt(inst_blt),
    .inst_bge(inst_bge), .inst_bltu(inst_bltu), .inst_bgeu(inst_bgeu),
    .inst_jirl(inst_jirl), .inst_lu12i_w(inst_lu12i_w),
    .inst_pcaddu12i(inst_pcaddu12i), .inst_ll_w(inst_ll_w), .inst_sc_w(inst_sc_w),
    .inst_pcaddi(inst_pcaddi), .inst_preld(inst_preld),
    .alu_imm(imm_raw), .br_imm(br_offs_raw)
);

wire [4:0] src0_addr_raw;
wire [4:0] src1_addr_raw;

get_reg_read_addr u_get_reg_read_addr(
    .reset(1'b0), .inst(inst_i),
    .inst_add_w(inst_add_w), .inst_addi_w(inst_addi_w), .inst_slti(inst_slti),
    .inst_sltui(inst_sltui), .inst_andi(inst_andi), .inst_ori(inst_ori),
    .inst_xori(inst_xori), .inst_sub_w(inst_sub_w), .inst_ld_w(inst_ld_w),
    .inst_ld_h(inst_ld_h), .inst_ld_b(inst_ld_b), .inst_ld_hu(inst_ld_hu),
    .inst_ld_bu(inst_ld_bu), .inst_st_w(inst_st_w), .inst_st_b(inst_st_b),
    .inst_st_h(inst_st_h), .inst_bne(inst_bne), .inst_slt(inst_slt),
    .inst_sltu(inst_sltu), .inst_and(inst_and), .inst_or(inst_or),
    .inst_nor(inst_nor), .inst_xor(inst_xor), .inst_slli_w(inst_slli_w),
    .inst_srli_w(inst_srli_w), .inst_srai_w(inst_srai_w), .inst_sll_w(inst_sll_w),
    .inst_srl_w(inst_srl_w), .inst_sra_w(inst_sra_w), .inst_b(inst_b),
    .inst_bl(inst_bl), .inst_beq(inst_beq), .inst_blt(inst_blt),
    .inst_bge(inst_bge), .inst_bltu(inst_bltu), .inst_bgeu(inst_bgeu),
    .inst_jirl(inst_jirl), .inst_lu12i_w(inst_lu12i_w),
    .inst_pcaddu12i(inst_pcaddu12i), .inst_mul_w(inst_mul_w),
    .inst_mulh_w(inst_mulh_w), .inst_mulh_wu(inst_mulh_wu),
    .inst_div_w(inst_div_w), .inst_div_wu(inst_div_wu), .inst_mod_w(inst_mod_w),
    .inst_mod_wu(inst_mod_wu), .inst_ertn(inst_ertn), .inst_syscall(inst_syscall),
    .inst_break(inst_break), .inst_rdcntvl_w(inst_rdcntvl_w),
    .inst_rdcntvh_w(inst_rdcntvh_w), .inst_rdcntid(inst_rdcntid),
    .inst_csrrd(inst_csrrd), .inst_csrwr(inst_csrwr), .inst_csrxchg(inst_csrxchg),
    .inst_tlbsrch(inst_tlbsrch), .inst_tlbrd(inst_tlbrd), .inst_tlbwr(inst_tlbwr),
    .inst_tlbfill(inst_tlbfill), .inst_invtlb_0(inst_invtlb_0),
    .inst_invtlb_1(inst_invtlb_1), .inst_invtlb_2(inst_invtlb_2),
    .inst_invtlb_3(inst_invtlb_3), .inst_invtlb_4(inst_invtlb_4),
    .inst_invtlb_5(inst_invtlb_5), .inst_invtlb_6(inst_invtlb_6),
    .inst_cacop(inst_cacop), .inst_ll_w(inst_ll_w), .inst_sc_w(inst_sc_w),
    .inst_andn(inst_andn), .inst_orn(inst_orn), .inst_preld(inst_preld),
    .inst_cpucfg(inst_cpucfg), .rf_raddr1(src0_addr_raw), .rf_raddr2(src1_addr_raw)
);

wire [4:0] rd = inst_i[4:0];
wire [4:0] rj = inst_i[9:5];
wire inst_invtlb = inst_invtlb_0 | inst_invtlb_1 | inst_invtlb_2 | inst_invtlb_3 |
                   inst_invtlb_4 | inst_invtlb_5 | inst_invtlb_6;
wire inst_tlb_any = inst_tlbsrch | inst_tlbrd | inst_tlbwr | inst_tlbfill | inst_invtlb;
wire inst_muldiv = inst_mul_w | inst_mulh_w | inst_mulh_wu | inst_div_w | inst_div_wu |
                   inst_mod_w | inst_mod_wu;
wire inst_rdcnt_any = inst_rdcntvl_w | inst_rdcntvh_w | inst_rdcntid;
wire inst_csr_any = inst_csrrd | inst_csrwr | inst_csrxchg;
//如果 LLBIT=1，sc.w 正常作为 store-conditional 进入 MEM。
//如果 LLBIT=0，它不真正访存，而是退化成“写回 0 到 rd”的 ALU 类指令,但 PRIV_SC 仍然置位，因为 commit 阶段还需要处理 sc.w 的特殊提交语义。
wire sc_fail = inst_sc_w & ~csr_llbit_i;
wire sc_success = inst_sc_w & csr_llbit_i;

wire is_load_raw = inst_ld_w | inst_ld_h | inst_ld_b | inst_ld_hu | inst_ld_bu | inst_ll_w;
wire is_store_raw = inst_st_w | inst_st_h | inst_st_b | sc_success;

wire [`ALU_OP_NUM-1:0] sc_fail_alu_op = {{(`ALU_OP_NUM-1){1'b0}}, 1'b1};
assign alu_op_o = sc_fail ? sc_fail_alu_op : alu_op_raw;
assign br_op_o = br_op_raw;
assign mem_op_o = sc_fail ? {`MEM_OP_NUM{1'b0}} : mem_op_raw;
assign csr_op_o = csr_op_raw;
assign tlb_op_o = tlb_op_raw;
assign wb_src_op_o = wb_src_op_raw;
//FU 类型,这决定后面 dispatch 进哪个保留站。
wire fu_mdu = inst_muldiv | inst_csr_any | inst_rdcnt_any | inst_tlb_any | inst_cpucfg;
wire fu_mem = is_load_raw | is_store_raw | inst_cacop;
assign futype_o[`FU_ALU] = ~(fu_mdu | fu_mem);
assign futype_o[`FU_MEM] = fu_mem;
assign futype_o[`FU_MDU] = fu_mdu;

assign csr_num_o = {2'b00, inst_i[23:10]};
assign cacop_code_o = inst_i[4:0];
//判断地址是否有效，这两个信号后面 rename/dispatch 会用，决定是否需要查 RAT、是否等待源操作数。
assign src0_addr_o = src0_addr_raw;
assign src1_addr_o = src1_addr_raw;

wire use_src0_raw = inst_add_w | inst_addi_w | inst_slti | inst_sltui | inst_andi |
                    inst_ori | inst_xori | inst_sub_w | inst_ld_w | inst_ld_h |
                    inst_ld_b | inst_ld_hu | inst_ld_bu | inst_st_w | inst_st_b |
                    inst_st_h | inst_slt | inst_sltu | inst_and | inst_or |
                    inst_nor | inst_xor | inst_slli_w | inst_srli_w | inst_srai_w |
                    inst_sll_w | inst_srl_w | inst_sra_w | inst_beq | inst_bne |
                    inst_jirl | inst_blt | inst_bge | inst_bltu | inst_bgeu |
                    inst_mul_w | inst_mulh_w | inst_mulh_wu | inst_div_w | inst_div_wu |
                    inst_mod_w | inst_mod_wu | inst_invtlb_4 | inst_invtlb_5 |
                    inst_invtlb_6 | inst_csrxchg | inst_rdcntid | inst_cacop |
                    inst_ll_w | inst_sc_w | inst_andn | inst_orn | inst_preld | inst_cpucfg;
wire use_src1_raw = inst_add_w | inst_sub_w | inst_slt | inst_sltu | inst_and |
                    inst_or | inst_nor | inst_xor | inst_sll_w | inst_srl_w |
                    inst_sra_w | inst_mul_w | inst_mulh_w | inst_mulh_wu | inst_div_w |
                    inst_div_wu | inst_mod_w | inst_mod_wu | inst_invtlb_5 |
                    inst_invtlb_6 | inst_andn | inst_orn | inst_st_w | inst_st_b |
                    inst_st_h | inst_beq | inst_bne | inst_blt | inst_bge | inst_bltu |
                    inst_bgeu | inst_csrwr | inst_csrxchg | inst_sc_w;
assign use_src0_o = use_src0_raw & ~sc_fail;
assign use_src1_o = use_src1_raw & ~sc_fail;

assign rd_addr_o = inst_bl ? 5'd1 : inst_rdcntid ? rj : rd;
wire rf_we_raw = inst_add_w | inst_addi_w | inst_slti | inst_sltui | inst_andi |
                 inst_ori | inst_xori | inst_sub_w | inst_slt | inst_sltu |
                 inst_and | inst_or | inst_nor | inst_xor | inst_slli_w |
                 inst_srli_w | inst_srai_w | inst_sll_w | inst_srl_w | inst_sra_w |
                 inst_lu12i_w | inst_pcaddu12i | inst_pcaddi | inst_andn | inst_orn |
                 inst_mul_w | inst_mulh_w | inst_mulh_wu | inst_div_w | inst_div_wu |
                 inst_mod_w | inst_mod_wu | inst_ld_w | inst_ld_h | inst_ld_b |
                 inst_ld_hu | inst_ld_bu | inst_ll_w | inst_sc_w | inst_jirl | inst_bl |
                 inst_csrrd | inst_csrwr | inst_csrxchg | inst_rdcntvl_w |
                 inst_rdcntvh_w | inst_rdcntid | inst_cpucfg;
assign rf_we_o = rf_we_raw & (rd_addr_o != 5'd0);
//如果目的寄存器是 r0，强制不写回。这样可以避免乱序后端给 r0 分配无意义的重命名项。
assign imm_o = sc_fail ? 32'b0 : imm_raw;
assign br_offs_o = br_offs_raw;
//特殊处理 sc.w 失败时，立即数强制为 0。
wire use_imm_raw = inst_addi_w | inst_slti | inst_sltui | inst_andi | inst_ori |
                   inst_xori | inst_slli_w | inst_srli_w | inst_srai_w | inst_ld_w |
                   inst_ld_h | inst_ld_b | inst_ld_hu | inst_ld_bu | inst_st_w |
                   inst_st_b | inst_st_h | inst_lu12i_w | inst_pcaddu12i | inst_pcaddi |
                   inst_jirl | inst_bl | inst_ll_w | inst_sc_w | inst_preld;
assign use_imm_o = use_imm_raw | sc_fail;

assign is_branch_o = inst_beq | inst_bne | inst_blt | inst_bge | inst_bltu |
                     inst_bgeu | inst_b | inst_bl | inst_jirl;
//然后判断分支类型：这个给前端预测训练和 ROB 提交检查用。
assign br_type_o = (inst_jirl && (rd == 5'd0) && (rj == 5'd1)) ? `BR_TYPE_RET :
                   (inst_bl || (inst_jirl && (rd == 5'd1))) ? `BR_TYPE_CALL :
                   (inst_b || inst_jirl) ? `BR_TYPE_UNCOND : `BR_TYPE_COND;
//特权/特殊提交标记，这些一般不是普通执行完就算了，而是 commit 阶段要做特殊动作，比如刷新流水线、写 CSR、改 TLB、处理 LLBIT。
assign priv_vec_o[`PRIV_CSR_WR] = inst_csrwr | inst_csrxchg;
assign priv_vec_o[`PRIV_ERTN]   = inst_ertn;
assign priv_vec_o[`PRIV_TLB]    = inst_tlb_any;
assign priv_vec_o[`PRIV_CACOP]  = inst_cacop;
assign priv_vec_o[`PRIV_IDLE]   = inst_idle;
assign priv_vec_o[`PRIV_LL]     = inst_ll_w;
assign priv_vec_o[`PRIV_SC]     = inst_sc_w;
assign priv_vec_o[`PRIV_IBAR]   = inst_ibar | inst_dbar;
//这些被视为 is_nop_o。意思不是“完全不存在”，而是不需要进普通执行保留站，或者由提交/特殊逻辑处理。
assign is_nop_o = (inst_andi && (rd == 5'd0) && (rj == 5'd0) && (inst_i[21:10] == 12'b0)) |
                  inst_dbar | inst_ibar | inst_preld;
assign is_load_o = is_load_raw;
assign is_store_o = is_store_raw;

// IPE 特权指令集合。cacop 在手册上属特权，但 chiplab NEMU/func_lab19 n52
// TEST9 在 PLV3 下执行 cacop 且不注入 IPE（仅测「不应 ADEF」）；列入会导致
// ecode=IPE vs REF 无异常。仍经 priv_vec PRIV_CACOP 走单提交/落地。
wire privileged_inst = inst_csr_any | inst_tlb_any | inst_ertn | inst_idle;
//异常输出，不认识的指令：INE其他 TLB/访存异常现在 decode 阶段先置 0，后面 IFU/MMU/LSU 再产生。
assign excp_o[`EXCP_ADEF]   = 1'b0;
assign excp_o[`EXCP_TLBR_F] = 1'b0;
assign excp_o[`EXCP_PIF]    = 1'b0;
assign excp_o[`EXCP_PPI_F]  = 1'b0;
assign excp_o[`EXCP_INE]    = ~inst_known;
assign excp_o[`EXCP_SYS]    = inst_syscall;
assign excp_o[`EXCP_BRK]    = inst_break;
assign excp_o[`EXCP_IPE]    = privileged_inst & (csr_plv_i == 2'b11);
assign excp_o[`EXCP_ALE]    = 1'b0;
assign excp_o[`EXCP_ADEM]   = 1'b0;
assign excp_o[`EXCP_TLBR_M] = 1'b0;
assign excp_o[`EXCP_PIL]    = 1'b0;
assign excp_o[`EXCP_PIS]    = 1'b0;
assign excp_o[`EXCP_PPI_M]  = 1'b0;
assign excp_o[`EXCP_PME]    = 1'b0;

wire decoder_unused = |{pc_i, cache_op_raw};

endmodule
