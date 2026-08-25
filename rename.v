// ============================================================
// rename 模块（重命名级：查 RAT / 读 ARF / 分配 ROB / 锁存到分发级）
// ------------------------------------------------------------
// 功能：
// - 每拍从 IB 取最多 `MACHINE_WIDTH（2）条已译码指令，完成"队列式重命名"：
//   * 源操作数：查 RAT —— busy=0 直接读 ARF 拿值（ready=1）；
//                       busy=1 记下 ROB 编号（ready=0，到分发级再读 ROB/等唤醒）。
//   * 目的寄存器：向 RAT 写占用（ROB 编号即重命名标签）。
//   * 同拍 RAW 旁路：槽 1 的源 == 槽 0 的目的 -> 直接用槽 0 的 ROB 编号（表外旁路）。
// - 向 ROB 申请一对表项（恒成对分配：槽 0 -> {1'b0,tail}，槽 1 -> {1'b1,tail}，
//   即使只有 1 条有效也占一对，以保持奇偶槽位和双发射对齐）。
// - 把静态信息写入 ROB（pc/we/rd/futype/分支预测信息/priv/异常等），
//   把执行所需信息锁存到 rename/dispatch 流水寄存器。
// - NOP 消除：is_nop 指令只入 ROB（分配时即标完成），不送分发级。
//
// 端口：
// - ib0/ib1_*      ：来自 IB+decoder 的两槽指令（decoder 在顶层例化，组合插在中间）
// - ib0/ib1_ready  ：收走使能（反压 IB）
// - rat_*          ：RAT 查询/占用口
// - arf_raddr/rdata：ARF 4 读口（rename 级取源）
// - rob_alloc_*    ：ROB 成对分配口（静态信息）
// - rename↔dispatch：反压/发射反馈、驻留唤醒回读、流水寄存器输出、旁路查询地址
// - dispatch_ready_i / dis*_fire_i：分发级反压与发射反馈
// - rob_rrdy*/dis_rat_rbusy*/dis_arf_rdata*：驻留唤醒（ROB/RAT/ARF 回灌 rename）
// - dis0/dis1_*_o    ：流水寄存器输出 → dispatch（执行所需信息）
// - dis*_src*_addr_o ：驻留旁路查询地址 → RAT/ARF 口 4~7（不直接进 dispatch）
// - flush_i        ：全局冲刷（清空本级流水寄存器）
// ============================================================
`include "mycpu.h"

module rename(
    input  wire                       clk,
    input  wire                       reset,
    input  wire                       flush_i,
    // L0 CSR 写已提交（免 flush）：解除 rename 排空闸门
    input  wire                       l0_csr_commit_i,

    // =============== 槽 0 输入（IB pop0 + decoder0，组合送入） ===============
    input  wire                       ib0_valid_i,
    input  wire [31:0]                ib0_pc_i,
    input  wire [31:0]                ib0_inst_i,
    input  wire                       ib0_pred_taken_i,
    input  wire                       ib0_is_last_i,
    input  wire [`FTQ_W-1:0]          ib0_ftq_id_i,
    input  wire [`EXCP_NUM-1:0]       ib0_excp_i,        // 取指异常+译码异常（顶层已并好）
    // decoder0 输出
    input  wire [`FU_NUM-1:0]         ib0_futype_i,
    input  wire [`ALU_OP_NUM-1:0]     ib0_alu_op_i,
    input  wire [`BR_OP_NUM-1:0]      ib0_br_op_i,
    input  wire [`MEM_OP_NUM-1:0]     ib0_mem_op_i,
    input  wire [`CSR_OP_NUM-1:0]     ib0_csr_op_i,
    input  wire [`TLB_OP_NUM-1:0]     ib0_tlb_op_i,
    input  wire [`WB_SRC_NUM-1:0]     ib0_wb_src_op_i,
    input  wire [13:0]                ib0_csr_num_i,
    input  wire [4:0]                 ib0_cacop_code_i,
    input  wire [4:0]                 ib0_src0_addr_i,
    input  wire [4:0]                 ib0_src1_addr_i,
    input  wire                       ib0_use_src0_i,
    input  wire                       ib0_use_src1_i,
    input  wire [4:0]                 ib0_rd_addr_i,
    input  wire                       ib0_rf_we_i,
    input  wire [31:0]                ib0_imm_i,
    input  wire                       ib0_use_imm_i,
    input  wire [31:0]                ib0_br_offs_i,
    input  wire                       ib0_is_branch_i,
    input  wire [`BR_TYPE_W-1:0]      ib0_br_type_i,
    input  wire [`PRIV_NUM-1:0]       ib0_priv_vec_i,
    input  wire                       ib0_is_nop_i,
    input  wire                       ib0_is_load_i,
    input  wire                       ib0_is_store_i,
    output wire                       ib0_ready_o,       // 本拍收走槽 0

    // =============== 槽 1 输入（IB pop1 + decoder1） ===============
    input  wire                       ib1_valid_i,
    input  wire [31:0]                ib1_pc_i,
    input  wire [31:0]                ib1_inst_i,
    input  wire                       ib1_pred_taken_i,
    input  wire                       ib1_is_last_i,
    input  wire [`FTQ_W-1:0]          ib1_ftq_id_i,
    input  wire [`EXCP_NUM-1:0]       ib1_excp_i,
    input  wire [`FU_NUM-1:0]         ib1_futype_i,
    input  wire [`ALU_OP_NUM-1:0]     ib1_alu_op_i,
    input  wire [`BR_OP_NUM-1:0]      ib1_br_op_i,
    input  wire [`MEM_OP_NUM-1:0]     ib1_mem_op_i,
    input  wire [`CSR_OP_NUM-1:0]     ib1_csr_op_i,
    input  wire [`TLB_OP_NUM-1:0]     ib1_tlb_op_i,
    input  wire [`WB_SRC_NUM-1:0]     ib1_wb_src_op_i,
    input  wire [13:0]                ib1_csr_num_i,
    input  wire [4:0]                 ib1_cacop_code_i,
    input  wire [4:0]                 ib1_src0_addr_i,
    input  wire [4:0]                 ib1_src1_addr_i,
    input  wire                       ib1_use_src0_i,
    input  wire                       ib1_use_src1_i,
    input  wire [4:0]                 ib1_rd_addr_i,
    input  wire                       ib1_rf_we_i,
    input  wire [31:0]                ib1_imm_i,
    input  wire                       ib1_use_imm_i,
    input  wire [31:0]                ib1_br_offs_i,
    input  wire                       ib1_is_branch_i,
    input  wire [`BR_TYPE_W-1:0]      ib1_br_type_i,
    input  wire [`PRIV_NUM-1:0]       ib1_priv_vec_i,
    input  wire                       ib1_is_nop_i,
    input  wire                       ib1_is_load_i,
    input  wire                       ib1_is_store_i,
    output wire                       ib1_ready_o,

    input  wire                       ib2_valid_i,
    input  wire [31:0]                ib2_pc_i,
    input  wire [31:0]                ib2_inst_i,
    input  wire                       ib2_pred_taken_i,
    input  wire                       ib2_is_last_i,
    input  wire [`FTQ_W-1:0]          ib2_ftq_id_i,
    input  wire [`EXCP_NUM-1:0]       ib2_excp_i,
    input  wire [`FU_NUM-1:0]         ib2_futype_i,
    input  wire [`ALU_OP_NUM-1:0]     ib2_alu_op_i,
    input  wire [`BR_OP_NUM-1:0]      ib2_br_op_i,
    input  wire [`MEM_OP_NUM-1:0]     ib2_mem_op_i,
    input  wire [`CSR_OP_NUM-1:0]     ib2_csr_op_i,
    input  wire [`TLB_OP_NUM-1:0]     ib2_tlb_op_i,
    input  wire [`WB_SRC_NUM-1:0]     ib2_wb_src_op_i,
    input  wire [13:0]                ib2_csr_num_i,
    input  wire [4:0]                 ib2_cacop_code_i,
    input  wire [4:0]                 ib2_src0_addr_i,
    input  wire [4:0]                 ib2_src1_addr_i,
    input  wire                       ib2_use_src0_i,
    input  wire                       ib2_use_src1_i,
    input  wire [4:0]                 ib2_rd_addr_i,
    input  wire                       ib2_rf_we_i,
    input  wire [31:0]                ib2_imm_i,
    input  wire                       ib2_use_imm_i,
    input  wire [31:0]                ib2_br_offs_i,
    input  wire                       ib2_is_branch_i,
    input  wire [`BR_TYPE_W-1:0]      ib2_br_type_i,
    input  wire [`PRIV_NUM-1:0]       ib2_priv_vec_i,
    input  wire                       ib2_is_nop_i,
    input  wire                       ib2_is_load_i,
    input  wire                       ib2_is_store_i,
    output wire                       ib2_ready_o,

    // =============== RAT 接口 ===============
    output wire [4:0]                 rat_raddr0_o,      // 槽0 src0
    input  wire                       rat_rbusy0_i,
    input  wire [`ROB_W-1:0]          rat_rnum0_i,
    output wire [4:0]                 rat_raddr1_o,      // 槽0 src1
    input  wire                       rat_rbusy1_i,
    input  wire [`ROB_W-1:0]          rat_rnum1_i,
    output wire [4:0]                 rat_raddr2_o,      // 槽1 src0
    input  wire                       rat_rbusy2_i,
    input  wire [`ROB_W-1:0]          rat_rnum2_i,
    output wire [4:0]                 rat_raddr3_o,      // 槽1 src1
    input  wire                       rat_rbusy3_i,
    input  wire [`ROB_W-1:0]          rat_rnum3_i,

    output wire [4:0]                 rat_raddr4_o,
    input  wire                       rat_rbusy4_i,
    input  wire [`ROB_W-1:0]          rat_rnum4_i,
    output wire [4:0]                 rat_raddr5_o,
    input  wire                       rat_rbusy5_i,
    input  wire [`ROB_W-1:0]          rat_rnum5_i,

    output wire                       rat_wen0_o,        // 槽0 目的占用
    output wire [4:0]                 rat_waddr0_o,
    output wire [`ROB_W-1:0]          rat_wnum0_o,
    output wire                       rat_wen1_o,        // 槽1 目的占用
    output wire [4:0]                 rat_waddr1_o,
    output wire [`ROB_W-1:0]          rat_wnum1_o,
    output wire                       rat_wen2_o,
    output wire [4:0]                 rat_waddr2_o,
    output wire [`ROB_W-1:0]          rat_wnum2_o,

    // =============== ARF 4 读口 ===============
    output wire [4:0]                 arf_raddr0_o,
    input  wire [31:0]                arf_rdata0_i,
    output wire [4:0]                 arf_raddr1_o,
    input  wire [31:0]                arf_rdata1_i,
    output wire [4:0]                 arf_raddr2_o,
    input  wire [31:0]                arf_rdata2_i,
    output wire [4:0]                 arf_raddr3_o,
    input  wire [31:0]                arf_rdata3_i,
    output wire [4:0]                 arf_raddr4_o,
    input  wire [31:0]                arf_rdata4_i,
    output wire [4:0]                 arf_raddr5_o,
    input  wire [31:0]                arf_rdata5_i,

    // =============== ROB 成对分配口（静态信息） ===============
    output wire                       rob_alloc_en_o,        // 本拍分配一对
    input  wire [`ROB_PAIR_W-1:0]     rob_tail_i,            // 当前队尾（编号={奇偶位,tail}）
    input  wire                       rob_full_i,
    input  wire                       rob_tail_half_i,
    input  wire                       rob_full2_i,
    output wire                       rob_alloc_two_o,

    output wire                       rob_a0_valid_o,        // 槽 0 静态信息
    output wire [31:0]                rob_a0_pc_o,
    output wire [31:0]                rob_a0_inst_o,
    output wire                       rob_a0_rf_we_o,
    output wire [4:0]                 rob_a0_rd_o,
    output wire                       rob_a0_is_load_o,
    output wire                       rob_a0_is_store_o,
    output wire                       rob_a0_is_branch_o,
    output wire [`BR_TYPE_W-1:0]      rob_a0_br_type_o,
    output wire                       rob_a0_pred_taken_o,
    output wire                       rob_a0_is_last_o,
    output wire [`FTQ_W-1:0]          rob_a0_ftq_id_o,
    output wire [`PRIV_NUM-1:0]       rob_a0_priv_vec_o,
    output wire [13:0]                rob_a0_csr_num_o,
    output wire [`TLB_OP_NUM-1:0]     rob_a0_tlb_op_o,       // TLB 维护类型（提交级落地用）
    output wire [4:0]                 rob_a0_cacop_code_o,
    output wire [`EXCP_NUM-1:0]       rob_a0_excp_o,
    output wire                       rob_a0_is_nop_o,       // 分配时即标完成

    output wire                       rob_a1_valid_o,        // 槽 1 静态信息
    output wire [31:0]                rob_a1_pc_o,
    output wire [31:0]                rob_a1_inst_o,
    output wire                       rob_a1_rf_we_o,
    output wire [4:0]                 rob_a1_rd_o,
    output wire                       rob_a1_is_load_o,
    output wire                       rob_a1_is_store_o,
    output wire                       rob_a1_is_branch_o,
    output wire [`BR_TYPE_W-1:0]      rob_a1_br_type_o,
    output wire                       rob_a1_pred_taken_o,
    output wire                       rob_a1_is_last_o,
    output wire [`FTQ_W-1:0]          rob_a1_ftq_id_o,
    output wire [`PRIV_NUM-1:0]       rob_a1_priv_vec_o,
    output wire [13:0]                rob_a1_csr_num_o,
    output wire [`TLB_OP_NUM-1:0]     rob_a1_tlb_op_o,
    output wire [4:0]                 rob_a1_cacop_code_o,
    output wire [`EXCP_NUM-1:0]       rob_a1_excp_o,
    output wire                       rob_a1_is_nop_o,

    output wire                       rob_a2_valid_o,
    output wire [31:0]                rob_a2_pc_o,
    output wire [31:0]                rob_a2_inst_o,
    output wire                       rob_a2_rf_we_o,
    output wire [4:0]                 rob_a2_rd_o,
    output wire                       rob_a2_is_load_o,
    output wire                       rob_a2_is_store_o,
    output wire                       rob_a2_is_branch_o,
    output wire [`BR_TYPE_W-1:0]      rob_a2_br_type_o,
    output wire                       rob_a2_pred_taken_o,
    output wire                       rob_a2_is_last_o,
    output wire [`FTQ_W-1:0]          rob_a2_ftq_id_o,
    output wire [`PRIV_NUM-1:0]       rob_a2_priv_vec_o,
    output wire [13:0]                rob_a2_csr_num_o,
    output wire [`TLB_OP_NUM-1:0]     rob_a2_tlb_op_o,
    output wire [4:0]                 rob_a2_cacop_code_o,
    output wire [`EXCP_NUM-1:0]       rob_a2_excp_o,
    output wire                       rob_a2_is_nop_o,

    // =============== rename ↔ dispatch 接口 ===============
    // 反压与发射反馈（来自 dispatch）
    input  wire                       dispatch_ready_i,      // 分发级可接收新指令（两槽空，或本拍全部有效槽发射腾空）
    input  wire                       dis0_fire_i,           // 本拍槽 0 已入 RS
    input  wire                       dis1_fire_i,           // 本拍槽 1 已入 RS
    input  wire                       dis2_fire_i,

    // 驻留唤醒反馈（指令卡在 rename 流水寄存器时，从 ROB/RAT/ARF 回读操作数）
    // ROB 读口：robid 仍 busy 时取推测值；与 dispatch 共用 raddr=dis*_src*_robid
    input  wire                       rob_rrdy0_i,           // dis0 src0 对应 ROB 项已就绪
    input  wire [31:0]                rob_rdata0_i,
    input  wire                       rob_rrdy1_i,           // dis0 src1
    input  wire [31:0]                rob_rdata1_i,
    input  wire                       rob_rrdy2_i,           // dis1 src0
    input  wire [31:0]                rob_rdata2_i,
    input  wire                       rob_rrdy3_i,           // dis1 src1
    input  wire [31:0]                rob_rdata3_i,
    input  wire                       rob_rrdy4_i,
    input  wire [31:0]                rob_rdata4_i,
    input  wire                       rob_rrdy5_i,
    input  wire [31:0]                rob_rdata5_i,
    // ARF 旁路：RAT busy 清 0 → 前序已提交，ARF 为权威；查 RAT/ARF 口 4~7
    input  wire                       dis_rat_rbusy0_i,      // dis0 src0 当前 RAT busy
    input  wire                       dis_rat_rbusy1_i,      // dis0 src1
    input  wire                       dis_rat_rbusy2_i,      // dis1 src0
    input  wire                       dis_rat_rbusy3_i,      // dis1 src1
    input  wire [31:0]                dis_arf_rdata0_i,      // dis0 src0 ARF 权威值
    input  wire [31:0]                dis_arf_rdata1_i,      // dis0 src1
    input  wire [31:0]                dis_arf_rdata2_i,      // dis1 src0
    input  wire [31:0]                dis_arf_rdata3_i,      // dis1 src1
    input  wire                       dis_rat_rbusy4_i,
    input  wire                       dis_rat_rbusy5_i,
    input  wire [31:0]                dis_arf_rdata4_i,
    input  wire [31:0]                dis_arf_rdata5_i,

    // 流水寄存器输出 → dispatch（执行所需信息）
    output reg                        dis0_valid_o,
    output reg  [`ROB_W-1:0]          dis0_robid_o,
    output reg  [31:0]                dis0_pc_o,
    output reg  [`FU_NUM-1:0]         dis0_futype_o,
    output reg  [`ALU_OP_NUM-1:0]     dis0_alu_op_o,
    output reg  [`BR_OP_NUM-1:0]      dis0_br_op_o,
    output reg  [`MEM_OP_NUM-1:0]     dis0_mem_op_o,
    output reg  [`CSR_OP_NUM-1:0]     dis0_csr_op_o,
    output reg  [`TLB_OP_NUM-1:0]     dis0_tlb_op_o,
    output reg  [`WB_SRC_NUM-1:0]     dis0_wb_src_op_o,
    output reg  [13:0]                dis0_csr_num_o,
    output reg                        dis0_is_cacop_o,
    output reg  [4:3]                 dis0_cacop_op_o,
    output reg                        dis0_src0_ready_o,     // 1=val 有效；0=等 robid 唤醒
    output reg  [31:0]                dis0_src0_val_o,
    output reg  [`ROB_W-1:0]          dis0_src0_robid_o,
    output reg                        dis0_src1_ready_o,
    output reg  [31:0]                dis0_src1_val_o,
    output reg  [`ROB_W-1:0]          dis0_src1_robid_o,
    output reg  [31:0]                dis0_imm_o,
    output reg                        dis0_use_imm_o,
    output reg  [31:0]                dis0_br_offs_o,

    output reg                        dis1_valid_o,
    output reg  [`ROB_W-1:0]          dis1_robid_o,
    output reg  [31:0]                dis1_pc_o,
    output reg  [`FU_NUM-1:0]         dis1_futype_o,
    output reg  [`ALU_OP_NUM-1:0]     dis1_alu_op_o,
    output reg  [`BR_OP_NUM-1:0]      dis1_br_op_o,
    output reg  [`MEM_OP_NUM-1:0]     dis1_mem_op_o,
    output reg  [`CSR_OP_NUM-1:0]     dis1_csr_op_o,
    output reg  [`TLB_OP_NUM-1:0]     dis1_tlb_op_o,
    output reg  [`WB_SRC_NUM-1:0]     dis1_wb_src_op_o,
    output reg  [13:0]                dis1_csr_num_o,
    output reg                        dis1_is_cacop_o,
    output reg  [4:3]                 dis1_cacop_op_o,
    output reg                        dis1_src0_ready_o,
    output reg  [31:0]                dis1_src0_val_o,
    output reg  [`ROB_W-1:0]          dis1_src0_robid_o,
    output reg                        dis1_src1_ready_o,
    output reg  [31:0]                dis1_src1_val_o,
    output reg  [`ROB_W-1:0]          dis1_src1_robid_o,
    output reg  [31:0]                dis1_imm_o,
    output reg                        dis1_use_imm_o,
    output reg  [31:0]                dis1_br_offs_o,

    output reg                        dis2_valid_o,
    output reg  [`ROB_W-1:0]          dis2_robid_o,
    output reg  [31:0]                dis2_pc_o,
    output reg  [`FU_NUM-1:0]         dis2_futype_o,
    output reg  [`ALU_OP_NUM-1:0]     dis2_alu_op_o,
    output reg  [`BR_OP_NUM-1:0]      dis2_br_op_o,
    output reg  [`MEM_OP_NUM-1:0]     dis2_mem_op_o,
    output reg  [`CSR_OP_NUM-1:0]     dis2_csr_op_o,
    output reg  [`TLB_OP_NUM-1:0]     dis2_tlb_op_o,
    output reg  [`WB_SRC_NUM-1:0]     dis2_wb_src_op_o,
    output reg  [13:0]                dis2_csr_num_o,
    output reg                        dis2_is_cacop_o,
    output reg  [4:3]                 dis2_cacop_op_o,
    output reg                        dis2_src0_ready_o,
    output reg  [31:0]                dis2_src0_val_o,
    output reg  [`ROB_W-1:0]          dis2_src0_robid_o,
    output reg                        dis2_src1_ready_o,
    output reg  [31:0]                dis2_src1_val_o,
    output reg  [`ROB_W-1:0]          dis2_src1_robid_o,
    output reg  [31:0]                dis2_imm_o,
    output reg                        dis2_use_imm_o,
    output reg  [31:0]                dis2_br_offs_o,

    // 驻留旁路查询地址（锁存逻辑寄存器号 → RAT/ARF 口 4~7，供上述唤醒路径用）
    output wire [4:0]                 dis0_src0_addr_o,
    output wire [4:0]                 dis0_src1_addr_o,
    output wire [4:0]                 dis1_src0_addr_o,
    output wire [4:0]                 dis1_src1_addr_o,
    output wire [4:0]                 dis2_src0_addr_o,
    output wire [4:0]                 dis2_src1_addr_o
);

// 设计说明：
//
wire can_go;  // 本拍放行条件（组合）：IB 有效 && ROB 未满 && 分发级可接收 && !flush

wire [`ROB_W-1:0] robid0;
wire [`ROB_W-1:0] robid1;
wire [`ROB_W-1:0] robid2;
wire ib0_writes_rf;
wire ib1_writes_rf;
wire ib2_writes_rf;
wire ib1_src0_raw_from_ib0;
wire ib1_src1_raw_from_ib0;
wire ib2_src0_raw_from_ib0;
wire ib2_src1_raw_from_ib0;
wire ib2_src0_raw_from_ib1;
wire ib2_src1_raw_from_ib1;
wire src0_0_ready;
wire src0_1_ready;
wire src1_0_ready;
wire src1_1_ready;
wire src2_0_ready;
wire src2_1_ready;
wire [31:0] src0_0_val;
wire [31:0] src0_1_val;
wire [31:0] src1_0_val;
wire [31:0] src1_1_val;
wire [31:0] src2_0_val;
wire [31:0] src2_1_val;
wire [`ROB_W-1:0] src0_0_robid;
wire [`ROB_W-1:0] src0_1_robid;
wire [`ROB_W-1:0] src1_0_robid;
wire [`ROB_W-1:0] src1_1_robid;
wire [`ROB_W-1:0] src2_0_robid;
wire [`ROB_W-1:0] src2_1_robid;

reg        dis0_src0_use_r;
reg        dis0_src1_use_r;
reg        dis1_src0_use_r;
reg        dis1_src1_use_r;
reg        dis2_src0_use_r;
reg        dis2_src1_use_r;
reg [4:0]  dis0_src0_addr_r;
reg [4:0]  dis0_src1_addr_r;
reg [4:0]  dis1_src0_addr_r;
reg [4:0]  dis1_src1_addr_r;
reg [4:0]  dis2_src0_addr_r;
reg [4:0]  dis2_src1_addr_r;

assign dis0_src0_addr_o = dis0_src0_addr_r;
assign dis0_src1_addr_o = dis0_src1_addr_r;
assign dis1_src0_addr_o = dis1_src0_addr_r;
assign dis1_src1_addr_o = dis1_src1_addr_r;
assign dis2_src0_addr_o = dis2_src0_addr_r;
assign dis2_src1_addr_o = dis2_src1_addr_r;

// 第一步——本级放行条件（组合）：
//      can_go = (ib0_valid_i | ib1_valid_i) && !rob_full_i && dispatch_ready_i && !flush_i;
//      ib0_ready_o = can_go && ib0_valid_i;  ib1_ready_o = can_go && ib1_valid_i;
//      rob_alloc_en_o = can_go;   // 恒成对分配（哪怕只有槽 0 有效，也占一对槽位）
//      注：dispatch_ready_i 为 0 时整级停（IB 不弹出、ROB 不分配、流水寄存器保持）。
//      ready 在“槽空”或“本拍 valid 槽均 fire”时为 1，允许同拍 vacated+装入。
//      分发级每拍最多 1 条 MEM/MDU，双发两条同类会卡住直到能 vacated。
//
// Keep static exceptions/barriers/no-effect operations in the ROB so commit
// observes their architectural ordering, but do not send them to an
// execution unit.  ROB completion is reconstructed from the same saved
// static fields at the head, so a late FU write cannot corrupt a reused tag.
wire ib0_static_excp = |ib0_excp_i;
wire ib1_static_excp = |ib1_excp_i;
wire ib2_static_excp = |ib2_excp_i;
wire ib0_barrier = ib0_priv_vec_i[`PRIV_IBAR];
wire ib1_barrier = ib1_priv_vec_i[`PRIV_IBAR];
wire ib2_barrier = ib2_priv_vec_i[`PRIV_IBAR];
wire ib0_no_side_effect =
    !ib0_rf_we_i && !ib0_is_load_i && !ib0_is_store_i
    && !ib0_is_branch_i && !(|ib0_priv_vec_i);
wire ib1_no_side_effect =
    !ib1_rf_we_i && !ib1_is_load_i && !ib1_is_store_i
    && !ib1_is_branch_i && !(|ib1_priv_vec_i);
wire ib2_no_side_effect =
    !ib2_rf_we_i && !ib2_is_load_i && !ib2_is_store_i
    && !ib2_is_branch_i && !(|ib2_priv_vec_i);
wire ib0_no_exec =
    ib0_static_excp || ib0_barrier || ib0_no_side_effect;
wire ib1_no_exec =
    ib1_static_excp || ib1_barrier || ib1_no_side_effect;
wire ib2_no_exec =
    ib2_static_excp || ib2_barrier || ib2_no_side_effect;
wire ib0_eff_v = ib0_valid_i && !ib0_no_exec;
wire ib1_eff_v = ib1_valid_i && !ib1_no_exec;
wire ib2_eff_v = ib2_valid_i && !ib2_no_exec;
// L0 CSR 写：单独成对进入 ROB，并闸住后续 rename，直至提交（免 flush 时防在途 csrrd）
wire ib0_l0_csr_wr = ib0_eff_v && ib0_priv_vec_i[`PRIV_CSR_WR]
                  && `CSR_NUM_IS_L0_NOFLUSH(ib0_csr_num_i);
wire ib1_l0_csr_wr = ib1_eff_v && ib1_priv_vec_i[`PRIV_CSR_WR]
                  && `CSR_NUM_IS_L0_NOFLUSH(ib1_csr_num_i);
wire ib2_l0_csr_wr = ib2_eff_v && ib2_priv_vec_i[`PRIV_CSR_WR]
                  && `CSR_NUM_IS_L0_NOFLUSH(ib2_csr_num_i);
// Resource-pairing only needs the decoded instruction classes.  Do not put
// the exception/no-execute decision on the slot-1 RAT/ROB allocation path:
// for a no-execute entry the raw class can only make pairing more
// conservative, never admit an illegal pair.  The precise *_l0_csr_wr terms
// below are still used for the architectural drain state.
wire ib0_l0_csr_class = ib0_valid_i && ib0_priv_vec_i[`PRIV_CSR_WR]
                     && `CSR_NUM_IS_L0_NOFLUSH(ib0_csr_num_i);
wire ib1_l0_csr_class = ib1_valid_i && ib1_priv_vec_i[`PRIV_CSR_WR]
                     && `CSR_NUM_IS_L0_NOFLUSH(ib1_csr_num_i);
wire ib2_l0_csr_class = ib2_valid_i && ib2_priv_vec_i[`PRIV_CSR_WR]
                     && `CSR_NUM_IS_L0_NOFLUSH(ib2_csr_num_i);
reg l0_csr_drain_r;
wire both_slots_valid = ib0_valid_i && ib1_valid_i;
// RS_MEM has two enqueue ports in this experiment, so MEM/MEM bundles no
// longer need to be split across rename cycles.  MDU remains single-enqueue.
wire dual_issue_ok = !((both_slots_valid && ib0_futype_i[`FU_MDU] && ib1_futype_i[`FU_MDU]) ||
                       (ib0_l0_csr_class && ib1_valid_i) ||
                       (ib1_l0_csr_class && ib0_valid_i));

wire [1:0] three_alu_n = {1'b0, ib0_valid_i && ib0_futype_i[`FU_ALU]}
                       + {1'b0, ib1_valid_i && ib1_futype_i[`FU_ALU]}
                       + {1'b0, ib2_valid_i && ib2_futype_i[`FU_ALU]};
wire [1:0] three_mem_n = {1'b0, ib0_valid_i && ib0_futype_i[`FU_MEM]}
                       + {1'b0, ib1_valid_i && ib1_futype_i[`FU_MEM]}
                       + {1'b0, ib2_valid_i && ib2_futype_i[`FU_MEM]};
wire [1:0] three_mdu_n = {1'b0, ib0_valid_i && ib0_futype_i[`FU_MDU]}
                       + {1'b0, ib1_valid_i && ib1_futype_i[`FU_MDU]}
                       + {1'b0, ib2_valid_i && ib2_futype_i[`FU_MDU]};
wire three_issue_ok = dual_issue_ok && ib0_valid_i && ib1_valid_i && ib2_valid_i
                    && (three_alu_n <= 3) && (three_mem_n <= 2) && (three_mdu_n <= 1)
                    && !ib0_l0_csr_class && !ib1_l0_csr_class && !ib2_l0_csr_class;
// When tail_half is set, even[tail] is occupied and odd[tail] is the next
// free entry.  Filling that reserved odd slot needs no new ROB pair.  Wider
// prefixes are admitted only when the following pair(s) are also available.
wire take_slot1 = can_go && ib1_valid_i && dual_issue_ok && !rob_full_i;
// A three-entry prefix always consumes the last free slot of a second pair.
// Keep one complete guard pair after it even when allocation starts at an
// odd slot; otherwise tail can catch head with live entries still present.
wire take_slot2 = can_go && three_issue_ok && !rob_full2_i;

assign can_go = (ib0_valid_i | ib1_valid_i) && !rob_full_i && dispatch_ready_i
             && !flush_i && !l0_csr_drain_r;
assign ib0_ready_o = can_go && ib0_valid_i;
// 同拍 RAW（ib1 源 == ib0 目的）不再阻塞 ib1：
// 强制走 ROB 唤醒路径（src ready=0，tag=robid0，见下方第三步旁路），不信 ARF ready
assign ib1_ready_o = take_slot1;
assign ib2_ready_o = take_slot2;
assign rob_alloc_en_o = can_go;
assign rob_alloc_two_o = take_slot2;

wire l0_csr_alloc = can_go && (ib0_l0_csr_wr ||
                               (ib1_l0_csr_wr && take_slot1) ||
                               (ib2_l0_csr_wr && take_slot2));
always @(posedge clk) begin
    if (reset || flush_i)
        l0_csr_drain_r <= 1'b0;
    else if (l0_csr_commit_i)
        l0_csr_drain_r <= 1'b0;
    else if (l0_csr_alloc)
        l0_csr_drain_r <= 1'b1;
end

wire ib0_null_bubble = ib0_valid_i && (ib0_inst_i == 32'b0) && !(|ib0_excp_i);
wire ib1_null_bubble = ib1_valid_i && (ib1_inst_i == 32'b0) && !(|ib1_excp_i);
wire ib2_null_bubble = ib2_valid_i && (ib2_inst_i == 32'b0) && !(|ib2_excp_i);

wire [`ROB_PAIR_W-1:0] rob_tail_next =
    rob_tail_i + {{(`ROB_PAIR_W-1){1'b0}}, 1'b1};
assign robid0 = rob_tail_half_i ? {1'b1, rob_tail_i}
                                : {1'b0, rob_tail_i};
assign robid1 = rob_tail_half_i ? {1'b0, rob_tail_next}
                                : {1'b1, rob_tail_i};
assign robid2 = rob_tail_half_i ? {1'b1, rob_tail_next}
                                : {1'b0, rob_tail_next};
assign ib0_writes_rf = ib0_valid_i && ib0_rf_we_i && (ib0_rd_addr_i != 5'b0);
assign ib1_writes_rf = ib1_valid_i && ib1_rf_we_i && (ib1_rd_addr_i != 5'b0);
assign ib2_writes_rf = ib2_valid_i && ib2_rf_we_i && (ib2_rd_addr_i != 5'b0);

// 第二步——RAT/ARF 读地址（组合直连）：
//      rat_raddr0/1 = 槽0 的 src0/src1 地址；rat_raddr2/3 = 槽1 的；
//      arf_raddr0~3 与 RAT 查询使用同一组逻辑寄存器地址。
//
assign rat_raddr0_o = ib0_src0_addr_i;
assign rat_raddr1_o = ib0_src1_addr_i;
assign rat_raddr2_o = ib1_src0_addr_i;
assign rat_raddr3_o = ib1_src1_addr_i;
assign rat_raddr4_o = ib2_src0_addr_i;
assign rat_raddr5_o = ib2_src1_addr_i;

assign arf_raddr0_o = ib0_src0_addr_i;
assign arf_raddr1_o = ib0_src1_addr_i;
assign arf_raddr2_o = ib1_src0_addr_i;
assign arf_raddr3_o = ib1_src1_addr_i;
assign arf_raddr4_o = ib2_src0_addr_i;
assign arf_raddr5_o = ib2_src1_addr_i;

// 第三步——源操作数就绪判定（组合）：
//      槽0：src0_ready = !use_src0 || !rat_rbusy0_i;   值 = arf_rdata0_i；
//           标签 = rat_rnum0_i；（src1 同理）
//      槽1：要先做"同拍 RAW 旁路"——
//           槽1.src0 与槽0.rd 相同且槽0 写寄存器时：
//             src0_ready = 0；标签 = {1'b0, rob_tail_i}（槽 0 的新 ROB 编号）
//           否则与槽 0 同样查 RAT/ARF。
//      不使用的源（use_srcX=0）一律 ready=1、值给 0（保证保留站不等假数据）。
//
assign ib1_src0_raw_from_ib0 = ib1_use_src0_i && ib0_writes_rf &&
                               (ib1_src0_addr_i == ib0_rd_addr_i);
assign ib1_src1_raw_from_ib0 = ib1_use_src1_i && ib0_writes_rf &&
                               (ib1_src1_addr_i == ib0_rd_addr_i);
assign ib2_src0_raw_from_ib0 = ib2_use_src0_i && ib0_writes_rf &&
                               (ib2_src0_addr_i == ib0_rd_addr_i);
assign ib2_src1_raw_from_ib0 = ib2_use_src1_i && ib0_writes_rf &&
                               (ib2_src1_addr_i == ib0_rd_addr_i);
assign ib2_src0_raw_from_ib1 = ib2_use_src0_i && ib1_writes_rf &&
                               (ib2_src0_addr_i == ib1_rd_addr_i);
assign ib2_src1_raw_from_ib1 = ib2_use_src1_i && ib1_writes_rf &&
                               (ib2_src1_addr_i == ib1_rd_addr_i);

assign src0_0_ready = !ib0_use_src0_i || !rat_rbusy0_i;
assign src0_0_val = ib0_use_src0_i ? arf_rdata0_i : 32'b0;
assign src0_0_robid = rat_rnum0_i;

assign src0_1_ready = !ib0_use_src1_i || !rat_rbusy1_i;
assign src0_1_val = ib0_use_src1_i ? arf_rdata1_i : 32'b0;
assign src0_1_robid = rat_rnum1_i;

assign src1_0_ready = !ib1_use_src0_i ? 1'b1 :
                      ib1_src0_raw_from_ib0 ? 1'b0 :
                      !rat_rbusy2_i;
assign src1_0_val = ib1_use_src0_i ? arf_rdata2_i : 32'b0;
assign src1_0_robid = ib1_src0_raw_from_ib0 ? robid0 : rat_rnum2_i;

assign src1_1_ready = !ib1_use_src1_i ? 1'b1 :
                      ib1_src1_raw_from_ib0 ? 1'b0 :
                      !rat_rbusy3_i;
assign src1_1_val = ib1_use_src1_i ? arf_rdata3_i : 32'b0;
assign src1_1_robid = ib1_src1_raw_from_ib0 ? robid0 : rat_rnum3_i;

assign src2_0_ready = !ib2_use_src0_i ? 1'b1 :
                      (ib2_src0_raw_from_ib1 || ib2_src0_raw_from_ib0) ? 1'b0 :
                      !rat_rbusy4_i;
assign src2_0_val = ib2_use_src0_i ? arf_rdata4_i : 32'b0;
assign src2_0_robid = ib2_src0_raw_from_ib1 ? robid1 :
                      ib2_src0_raw_from_ib0 ? robid0 : rat_rnum4_i;
assign src2_1_ready = !ib2_use_src1_i ? 1'b1 :
                      (ib2_src1_raw_from_ib1 || ib2_src1_raw_from_ib0) ? 1'b0 :
                      !rat_rbusy5_i;
assign src2_1_val = ib2_use_src1_i ? arf_rdata5_i : 32'b0;
assign src2_1_robid = ib2_src1_raw_from_ib1 ? robid1 :
                      ib2_src1_raw_from_ib0 ? robid0 : rat_rnum5_i;

// Simulation-only probe for the first dual-load bottleneck.  VA[5] is the
// parity of the 32-byte DCache set and lies inside the page offset, so it is
// unchanged by address translation.
`ifdef SYNTHESIS
// synthesis translate_off
wire ren_ib0_plain_load = ib0_is_load_i && !ib0_mem_op_i[`MEM_OP_LL_W]
                        && !ib0_priv_vec_i[`PRIV_CACOP];
wire ren_ib1_plain_load = ib1_is_load_i && !ib1_mem_op_i[`MEM_OP_LL_W]
                        && !ib1_priv_vec_i[`PRIV_CACOP];
wire ren_dual_plain_load = both_slots_valid
                         && ren_ib0_plain_load && ren_ib1_plain_load;
wire ren_dual_load_indep = ren_dual_plain_load
                         && !ib1_src0_raw_from_ib0 && !ib1_src1_raw_from_ib0;
wire ren_dual_load_ready = ren_dual_load_indep
                         && src0_0_ready && src1_0_ready;
wire [31:0] ren_load_vaddr0 = src0_0_val + ib0_imm_i;
wire [31:0] ren_load_vaddr1 = src1_0_val + ib1_imm_i;
wire ren_dual_load_diff_bank = ren_dual_load_ready
                             && (ren_load_vaddr0[5] != ren_load_vaddr1[5]);
reg [63:0] ren_mem_pair_at_dispatch;
reg [63:0] ren_dual_plain_load_pair;
reg [63:0] ren_dual_load_independent;
reg [63:0] ren_dual_load_bases_ready;
reg [63:0] ren_dual_load_same_line;
reg [63:0] ren_dual_load_same_bank;
reg [63:0] ren_dual_load_diff_bank_count;
always @(posedge clk) begin
    if (reset) begin
        ren_mem_pair_at_dispatch       <= 64'd0;
        ren_dual_plain_load_pair       <= 64'd0;
        ren_dual_load_independent      <= 64'd0;
        ren_dual_load_bases_ready      <= 64'd0;
        ren_dual_load_same_line        <= 64'd0;
        ren_dual_load_same_bank        <= 64'd0;
        ren_dual_load_diff_bank_count  <= 64'd0;
    end else if (!flush_i && can_go) begin
        if (both_slots_valid && ib0_futype_i[`FU_MEM] && ib1_futype_i[`FU_MEM])
            ren_mem_pair_at_dispatch <= ren_mem_pair_at_dispatch + 64'd1;
        if (ren_dual_plain_load)
            ren_dual_plain_load_pair <= ren_dual_plain_load_pair + 64'd1;
        if (ren_dual_load_indep)
            ren_dual_load_independent <= ren_dual_load_independent + 64'd1;
        if (ren_dual_load_ready)
            ren_dual_load_bases_ready <= ren_dual_load_bases_ready + 64'd1;
        if (ren_dual_load_ready
            && (ren_load_vaddr0[31:5] == ren_load_vaddr1[31:5]))
            ren_dual_load_same_line <= ren_dual_load_same_line + 64'd1;
        if (ren_dual_load_ready && !ren_dual_load_diff_bank)
            ren_dual_load_same_bank <= ren_dual_load_same_bank + 64'd1;
        if (ren_dual_load_diff_bank)
            ren_dual_load_diff_bank_count <= ren_dual_load_diff_bank_count + 64'd1;
    end
end
// synthesis translate_on
`endif

// 第四步——RAT 占用写（组合输出，RAT 内部时序写）：
//      rat_wen0_o = can_go && ib0_valid_i && ib0_rf_we_i && (rd!=r0)；wnum0={1'b0,rob_tail_i}
//      rat_wen1_o 同理；wnum1={1'b1,rob_tail_i}
//      同拍 WAW（两槽 rd 相同）：RAT 内部已让槽 1 优先（更年轻），无需额外处理。
//
assign rat_wen0_o = can_go && ib0_writes_rf;
assign rat_waddr0_o = ib0_rd_addr_i;
assign rat_wnum0_o = robid0;
// WAW 同拍（两槽同 rd）：rat 内部槽 1 优先（更年轻），RAW 不再阻塞占用写
assign rat_wen1_o = take_slot1 && ib1_writes_rf;
assign rat_waddr1_o = ib1_rd_addr_i;
assign rat_wnum1_o = robid1;
assign rat_wen2_o = take_slot2 && ib2_writes_rf;
assign rat_waddr2_o = ib2_rd_addr_i;
assign rat_wnum2_o = robid2;

// 第五步——ROB 静态信息（组合直通 rob_a0_*/rob_a1_*）：
//      pc/inst/rf_we/rd/is_load/is_store/is_branch/br_type/pred_taken/
//      is_last/ftq_id/priv_vec/csr_num/cacop_code/excp/is_nop 全部从 ib*_ 输入透传。
//      ROB 在 alloc_en 时把这些锁进表项（valid 置位、complete 按 is_nop 置位）。
//
assign rob_a0_valid_o = can_go && ib0_valid_i;
assign rob_a0_pc_o = ib0_pc_i;
assign rob_a0_inst_o = ib0_inst_i;
assign rob_a0_rf_we_o = ib0_rf_we_i;
assign rob_a0_rd_o = ib0_rd_addr_i;
assign rob_a0_is_load_o = ib0_is_load_i;
assign rob_a0_is_store_o = ib0_is_store_i;
assign rob_a0_is_branch_o = ib0_is_branch_i;
assign rob_a0_br_type_o = ib0_br_type_i;
assign rob_a0_pred_taken_o = ib0_pred_taken_i;
assign rob_a0_is_last_o = ib0_is_last_i;
assign rob_a0_ftq_id_o = ib0_ftq_id_i;
assign rob_a0_priv_vec_o = ib0_priv_vec_i;
assign rob_a0_csr_num_o = ib0_csr_num_i;
assign rob_a0_tlb_op_o = ib0_tlb_op_i;
assign rob_a0_cacop_code_o = ib0_cacop_code_i;
assign rob_a0_excp_o = ib0_null_bubble ? {`EXCP_NUM{1'b0}} : ib0_excp_i;
assign rob_a0_is_nop_o = ib0_is_nop_i | ib0_null_bubble;

assign rob_a1_valid_o = take_slot1;
assign rob_a1_pc_o = ib1_pc_i;
assign rob_a1_inst_o = ib1_inst_i;
assign rob_a1_rf_we_o = ib1_rf_we_i;
assign rob_a1_rd_o = ib1_rd_addr_i;
assign rob_a1_is_load_o = ib1_is_load_i;
assign rob_a1_is_store_o = ib1_is_store_i;
assign rob_a1_is_branch_o = ib1_is_branch_i;
assign rob_a1_br_type_o = ib1_br_type_i;
assign rob_a1_pred_taken_o = ib1_pred_taken_i;
assign rob_a1_is_last_o = ib1_is_last_i;
assign rob_a1_ftq_id_o = ib1_ftq_id_i;
assign rob_a1_priv_vec_o = ib1_priv_vec_i;
assign rob_a1_csr_num_o = ib1_csr_num_i;
assign rob_a1_tlb_op_o = ib1_tlb_op_i;
assign rob_a1_cacop_code_o = ib1_cacop_code_i;
assign rob_a1_excp_o = ib1_null_bubble ? {`EXCP_NUM{1'b0}} : ib1_excp_i;
assign rob_a1_is_nop_o = ib1_is_nop_i | ib1_null_bubble;

assign rob_a2_valid_o = take_slot2 && ib2_valid_i;
assign rob_a2_pc_o = ib2_pc_i;
assign rob_a2_inst_o = ib2_inst_i;
assign rob_a2_rf_we_o = ib2_rf_we_i;
assign rob_a2_rd_o = ib2_rd_addr_i;
assign rob_a2_is_load_o = ib2_is_load_i;
assign rob_a2_is_store_o = ib2_is_store_i;
assign rob_a2_is_branch_o = ib2_is_branch_i;
assign rob_a2_br_type_o = ib2_br_type_i;
assign rob_a2_pred_taken_o = ib2_pred_taken_i;
assign rob_a2_is_last_o = ib2_is_last_i;
assign rob_a2_ftq_id_o = ib2_ftq_id_i;
assign rob_a2_priv_vec_o = ib2_priv_vec_i;
assign rob_a2_csr_num_o = ib2_csr_num_i;
assign rob_a2_tlb_op_o = ib2_tlb_op_i;
assign rob_a2_cacop_code_o = ib2_cacop_code_i;
assign rob_a2_excp_o = ib2_null_bubble ? {`EXCP_NUM{1'b0}} : ib2_excp_i;
assign rob_a2_is_nop_o = ib2_is_nop_i | ib2_null_bubble;

// 第六步——流水寄存器（时序，can_go 时锁存，flush_i 清 valid）：
//      dis*_valid <= can_go && ib*_valid && !ib*_is_nop（NOP 消除：不送分发级）；
//      其余字段照搬；dis*_robid <= {槽位, rob_tail_i}；
//      dispatch_ready_i=0 时保持（valid 不清，等分发级腾出位置）；
//      flush_i 时 dis0/dis1_valid 清 0（其余字段随意）。
//
// 坑点提示：
//      1. "恒成对分配"意味着 IB 只有 1 条时也消耗一对 ROB 槽（编号浪费一半），
//         代价是单条指令也占一对；好处是 ROB 编号奇偶位天然指示提交槽位。
//      2. is_nop 指令必须照常分配 ROB + 写 RAT？——NOP 不写寄存器（rf_we=0），
//         不会写 RAT；但 priv 类伪 NOP（dbar/ibar）带 priv_vec 要进 ROB 走提交流程。
//      3. flush_i 优先级最高：当拍 can_go 必须为 0（不得向 ROB/RAT 发任何写）。

always @(posedge clk) begin
    if (reset || flush_i) begin
        dis0_valid_o <= 1'b0;
        dis1_valid_o <= 1'b0;
        dis2_valid_o <= 1'b0;
    end else if (can_go) begin
        dis0_valid_o <= ib0_valid_i && !ib0_no_exec;
        dis0_robid_o <= robid0;
        dis0_pc_o <= ib0_pc_i;
        dis0_futype_o <= ib0_futype_i;
        dis0_alu_op_o <= ib0_alu_op_i;
        dis0_br_op_o <= ib0_br_op_i;
        dis0_mem_op_o <= ib0_mem_op_i;
        dis0_csr_op_o <= ib0_csr_op_i;
        dis0_tlb_op_o <= ib0_tlb_op_i;
        dis0_wb_src_op_o <= ib0_wb_src_op_i;
        dis0_csr_num_o <= ib0_csr_num_i;
        dis0_is_cacop_o <= ib0_priv_vec_i[`PRIV_CACOP];
        dis0_cacop_op_o <= ib0_cacop_code_i[4:3];
        dis0_src0_ready_o <= src0_0_ready;
        dis0_src0_val_o <= src0_0_val;
        dis0_src0_robid_o <= src0_0_robid;
        dis0_src1_ready_o <= src0_1_ready;
        dis0_src1_val_o <= src0_1_val;
        dis0_src1_robid_o <= src0_1_robid;
        dis0_imm_o <= ib0_imm_i;
        dis0_use_imm_o <= ib0_use_imm_i;
        dis0_br_offs_o <= ib0_br_offs_i;
        dis0_src0_use_r <= ib0_use_src0_i;
        dis0_src1_use_r <= ib0_use_src1_i;
        dis0_src0_addr_r <= ib0_src0_addr_i;
        dis0_src1_addr_r <= ib0_src1_addr_i;

        dis1_valid_o <= take_slot1 && !ib1_no_exec;
        dis1_robid_o <= robid1;
        dis1_pc_o <= ib1_pc_i;
        dis1_futype_o <= ib1_futype_i;
        dis1_alu_op_o <= ib1_alu_op_i;
        dis1_br_op_o <= ib1_br_op_i;
        dis1_mem_op_o <= ib1_mem_op_i;
        dis1_csr_op_o <= ib1_csr_op_i;
        dis1_tlb_op_o <= ib1_tlb_op_i;
        dis1_wb_src_op_o <= ib1_wb_src_op_i;
        dis1_csr_num_o <= ib1_csr_num_i;
        dis1_is_cacop_o <= ib1_priv_vec_i[`PRIV_CACOP];
        dis1_cacop_op_o <= ib1_cacop_code_i[4:3];
        dis1_src0_ready_o <= src1_0_ready;
        dis1_src0_val_o <= src1_0_val;
        dis1_src0_robid_o <= src1_0_robid;
        dis1_src1_ready_o <= src1_1_ready;
        dis1_src1_val_o <= src1_1_val;
        dis1_src1_robid_o <= src1_1_robid;
        dis1_imm_o <= ib1_imm_i;
        dis1_use_imm_o <= ib1_use_imm_i;
        dis1_br_offs_o <= ib1_br_offs_i;
        dis1_src0_use_r <= ib1_use_src0_i;
        dis1_src1_use_r <= ib1_use_src1_i;
        dis1_src0_addr_r <= ib1_src0_addr_i;
        dis1_src1_addr_r <= ib1_src1_addr_i;

        dis2_valid_o <= take_slot2 && !ib2_no_exec;
        dis2_robid_o <= robid2;
        dis2_pc_o <= ib2_pc_i;
        dis2_futype_o <= ib2_futype_i;
        dis2_alu_op_o <= ib2_alu_op_i;
        dis2_br_op_o <= ib2_br_op_i;
        dis2_mem_op_o <= ib2_mem_op_i;
        dis2_csr_op_o <= ib2_csr_op_i;
        dis2_tlb_op_o <= ib2_tlb_op_i;
        dis2_wb_src_op_o <= ib2_wb_src_op_i;
        dis2_csr_num_o <= ib2_csr_num_i;
        dis2_is_cacop_o <= ib2_priv_vec_i[`PRIV_CACOP];
        dis2_cacop_op_o <= ib2_cacop_code_i[4:3];
        dis2_src0_ready_o <= src2_0_ready;
        dis2_src0_val_o <= src2_0_val;
        dis2_src0_robid_o <= src2_0_robid;
        dis2_src1_ready_o <= src2_1_ready;
        dis2_src1_val_o <= src2_1_val;
        dis2_src1_robid_o <= src2_1_robid;
        dis2_imm_o <= ib2_imm_i;
        dis2_use_imm_o <= ib2_use_imm_i;
        dis2_br_offs_o <= ib2_br_offs_i;
        dis2_src0_use_r <= ib2_use_src0_i;
        dis2_src1_use_r <= ib2_use_src1_i;
        dis2_src0_addr_r <= ib2_src0_addr_i;
        dis2_src1_addr_r <= ib2_src1_addr_i;
    end else begin
        // 分发队列驻留：首次就绪时锁存操作数，避免 ROB 项复用后读到 ABA 错误值
        // （典型：mod.w 后 bne 等在 dispatch，下一条 mod.w 复用 robid 时误取新余数）
        if (dis0_valid_o && dis0_src0_use_r && !dis0_src0_ready_o) begin
            if (rob_rrdy0_i) begin
                dis0_src0_ready_o <= 1'b1;
                dis0_src0_val_o   <= rob_rdata0_i;
            end else if (!dis_rat_rbusy0_i) begin
                dis0_src0_ready_o <= 1'b1;
                dis0_src0_val_o   <= dis_arf_rdata0_i;
            end
        end
        if (dis0_valid_o && dis0_src1_use_r && !dis0_src1_ready_o) begin
            if (rob_rrdy1_i) begin
                dis0_src1_ready_o <= 1'b1;
                dis0_src1_val_o   <= rob_rdata1_i;
            end else if (!dis_rat_rbusy1_i) begin
                dis0_src1_ready_o <= 1'b1;
                dis0_src1_val_o   <= dis_arf_rdata1_i;
            end
        end
        if (dis1_valid_o && dis1_src0_use_r && !dis1_src0_ready_o) begin
            if (rob_rrdy2_i) begin
                dis1_src0_ready_o <= 1'b1;
                dis1_src0_val_o   <= rob_rdata2_i;
            end else if (!dis_rat_rbusy2_i) begin
                dis1_src0_ready_o <= 1'b1;
                dis1_src0_val_o   <= dis_arf_rdata2_i;
            end
        end
        if (dis1_valid_o && dis1_src1_use_r && !dis1_src1_ready_o) begin
            if (rob_rrdy3_i) begin
                dis1_src1_ready_o <= 1'b1;
                dis1_src1_val_o   <= rob_rdata3_i;
            end else if (!dis_rat_rbusy3_i) begin
                dis1_src1_ready_o <= 1'b1;
                dis1_src1_val_o   <= dis_arf_rdata3_i;
            end
        end
        if (dis2_valid_o && dis2_src0_use_r && !dis2_src0_ready_o) begin
            if (rob_rrdy4_i) begin
                dis2_src0_ready_o <= 1'b1;
                dis2_src0_val_o   <= rob_rdata4_i;
            end else if (!dis_rat_rbusy4_i) begin
                dis2_src0_ready_o <= 1'b1;
                dis2_src0_val_o   <= dis_arf_rdata4_i;
            end
        end
        if (dis2_valid_o && dis2_src1_use_r && !dis2_src1_ready_o) begin
            if (rob_rrdy5_i) begin
                dis2_src1_ready_o <= 1'b1;
                dis2_src1_val_o   <= rob_rdata5_i;
            end else if (!dis_rat_rbusy5_i) begin
                dis2_src1_ready_o <= 1'b1;
                dis2_src1_val_o   <= dis_arf_rdata5_i;
            end
        end

        if (dis0_fire_i)
            dis0_valid_o <= 1'b0;
        if (dis1_fire_i)
            dis1_valid_o <= 1'b0;
        if (dis2_fire_i)
            dis2_valid_o <= 1'b0;
    end
end

endmodule
