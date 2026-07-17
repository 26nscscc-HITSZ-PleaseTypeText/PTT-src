// ============================================================
// 顶层模块：core_top（乱序双发射 LA32R 处理器核）
// ------------------------------------------------------------
// 功能：
// - 组织"解耦前端 + 队列式重命名乱序后端"整体架构：
//   前端：BPU(uBTB+FTB+TAGE+RAS) -> FTQ -> IFU(+预译码) -> ICache -> IB
//   后端：decoder x2 -> rename(RAT+ARF/ROB) -> dispatch -> 4xRS
//         -> FU(ALU0/ALU1/LSU/MDU) -> ROB -> commit(双提交)
//   访存：LSU(AGU+DC) / store_buffer(提交后写) / DCache / L2 / AXI 桥
//   特权：csr_exception_commit_handler + tlb_manager(内含 32 项 TLB)
// - 重命名标签 = ROB 编号（mariver 队列式重命名，无独立 PRF/freelist）；
// - 一切恢复（误预测/异常/ertn/特权 refetch）走提交级统一冲刷（ctrl 广播）；
// - store 提交后经 store_buffer 按序写出；uncached load 仅在 ROB 头发出。
//
// 端口（与 chiplab/chip/soc_demo/sim/soc_top.v 中 core_top 一致）：
// - aclk / aresetn：CPU 时钟与低有效复位（内部同步为高有效 reset）。
// - intrpt[7:0]：8 位外部中断输入，直连 CSR ESTAT.IS[9:2]。
// - AXI4 Master：AR/R、AW/W/B 由 axi_line_bridge 驱动（cache 行突发读写）。
// - debug0_wb_*：提交槽 0 调试口（双提交校验走 DIFFTEST 的两个提交口）。
// ============================================================
`include "mycpu.h"

module core_top #(
    parameter TLBNUM = 32
)(
    input  wire        aclk,
    input  wire        aresetn,
    input  wire [7:0]  intrpt,

    // AR —— 读地址（Master → Slave）
    output wire [3:0]  arid,
    output wire [31:0] araddr,
    output wire [7:0]  arlen,
    output wire [2:0]  arsize,
    output wire [1:0]  arburst,
    output wire [1:0]  arlock,
    output wire [3:0]  arcache,
    output wire [2:0]  arprot,
    output wire        arvalid,
    input  wire        arready,

    // R —— 读数据（Slave → Master）
    input  wire [3:0]  rid,
    input  wire [31:0] rdata,
    input  wire [1:0]  rresp,
    input  wire        rlast,
    input  wire        rvalid,
    output wire        rready,

    // AW —— 写地址（Master → Slave）
    output wire [3:0]  awid,
    output wire [31:0] awaddr,
    output wire [7:0]  awlen,
    output wire [2:0]  awsize,
    output wire [1:0]  awburst,
    output wire [1:0]  awlock,
    output wire [3:0]  awcache,
    output wire [2:0]  awprot,
    output wire        awvalid,
    input  wire        awready,

    // W —— 写数据（Master → Slave）
    output wire [3:0]  wid,
    output wire [31:0] wdata,
    output wire [3:0]  wstrb,
    output wire        wlast,
    output wire        wvalid,
    input  wire        wready,

    // B —— 写响应（Slave → Master）
    input  wire [3:0]  bid,
    input  wire [1:0]  bresp,
    input  wire        bvalid,
    output wire        bready,

    // chiplab 调试用接口
    input  wire        break_point,
    input  wire        infor_flag,
    input  wire [ 4:0] reg_num,
    output wire        ws_valid,
    output wire [31:0] rf_rdata,

    output wire [31:0] debug0_wb_pc,
    output wire [ 3:0] debug0_wb_rf_wen,
    output wire [ 4:0] debug0_wb_rf_wnum,
    output wire [31:0] debug0_wb_rf_wdata,
    output wire [31:0] debug0_wb_inst

    `ifdef CPU_2CMT
    ,
    output wire [31:0] debug1_wb_pc,
    output wire [ 3:0] debug1_wb_rf_wen,
    output wire [ 4:0] debug1_wb_rf_wnum,
    output wire [31:0] debug1_wb_rf_wdata,
    output wire [31:0] debug1_wb_inst
    `endif
);

    // 把 aclk、aresetn 改为 clk、reset（同步高有效）
    wire clk = aclk;
    reg  reset;
    always @(posedge clk) reset <= ~aresetn;



//--------------------------------------------------
// 全局控制（ctrl）信号声明
//--------------------------------------------------
    wire                      flush;            // 全局冲刷（一拍脉冲，广播到所有模块）
    wire [31:0]               flush_pc;         // 冲刷后取指重定向 PC（接 BPU）
    wire                      fetch_stall;      // idle 睡眠期间冻结取指
    // commit -> ctrl 的冲刷请求
    wire                      cmt_flush_req;
    wire [`FLUSH_TYPE_W-1:0]  cmt_flush_type;
    wire [31:0]               cmt_flush_pc;
    wire                      cmt_idle;
    // 二期执行级重定向（一期 fu_alu 输出恒 0）
    wire                      alu0_exred_valid, alu1_exred_valid;
    wire [31:0]               alu0_exred_pc,    alu1_exred_pc;
    wire                      ex_redirect_req = alu0_exred_valid | alu1_exred_valid;
    wire [31:0]               ex_redirect_pc  = alu0_exred_valid ? alu0_exred_pc : alu1_exred_pc;
    // CSR 状态（前置声明，后方 CSR 分区驱动）
    wire                      csr_has_int;

//--------------------------------------------------
// ctrl：全局冲刷/重定向仲裁
//--------------------------------------------------
    ctrl u_ctrl(
        .clk              (clk),
        .reset            (reset),
        .cmt_flush_req_i  (cmt_flush_req),
        .cmt_flush_type_i (cmt_flush_type),
        .cmt_flush_pc_i   (cmt_flush_pc),
        .ex_redirect_req_i(ex_redirect_req),
        .ex_redirect_pc_i (ex_redirect_pc),
        .idle_commit_i    (cmt_idle),
        .has_int_i        (csr_has_int),
        .flush_o          (flush),
        .flush_pc_o       (flush_pc),
        .fetch_stall_o    (fetch_stall)
    );



//--------------------------------------------------
// 前端信号声明（BPU / FTQ / IFU / ICache / IB）
//--------------------------------------------------
    // BPU -> FTQ 预测块（P0 当拍 / P1 覆盖）
    wire                      bpu_p0_valid;
    wire [31:0]               bpu_p0_pc;
    wire [`BLK_LEN_W-1:0]     bpu_p0_length;
    wire                      bpu_p0_taken;
    wire [31:0]               bpu_p0_target;
    wire [`BR_TYPE_W-1:0]     bpu_p0_br_type;
    wire                      bpu_p1_valid;
    wire                      bpu_p1_meta_valid;
    wire [31:0]               bpu_p1_pc;
    wire [`BLK_LEN_W-1:0]     bpu_p1_length;
    wire                      bpu_p1_taken;
    wire [31:0]               bpu_p1_target;
    wire [`BR_TYPE_W-1:0]     bpu_p1_br_type;
    wire [`BPU_META_W-1:0]    bpu_p1_meta;
    // FTQ 状态与取块口
    wire                      ftq_full;
    wire                      ftq_ifu_valid;
    wire [31:0]               ftq_ifu_pc;
    wire [`BLK_LEN_W-1:0]     ftq_ifu_length;
    wire                      ftq_ifu_taken;
    wire [31:0]               ftq_ifu_target;
    wire [`FTQ_W-1:0]         ftq_ifu_id;
    wire                      ifu_ftq_accept;
    // FTQ -> BPU 训练包
    wire                      ftq_train_valid;
    wire [31:0]               ftq_train_pc;
    wire                      ftq_train_is_branch;
    wire                      ftq_train_taken;
    wire                      ftq_train_mispred;
    wire [31:0]               ftq_train_target;
    wire [`BR_TYPE_W-1:0]     ftq_train_br_type;
    wire [31:0]               ftq_train_fall_through;
    wire [`BPU_META_W-1:0]    ftq_train_meta;
    // IFU 预译码自重定向（-> BPU + FTQ）
    wire                      predec_redirect;
    wire                      predec_fixup_only;
    wire                      predec_update_pc;
    wire [31:0]               predec_redirect_pc;
    wire [`FTQ_W-1:0]         predec_redirect_id;
    wire [`BLK_LEN_W-1:0]     predec_length;
    wire                      predec_taken;
    wire [31:0]               predec_target;
    wire [`BR_TYPE_W-1:0]     predec_br_type;
    // IFU <-> MMU I 通道
    wire                      ifu_mmu_req;
    wire [31:0]               ifu_mmu_vaddr;
    wire [31:0]               mmu_i_paddr;
    wire [1:0]                mmu_i_mat;
    wire                      mmu_i_adef;
    wire [`TLB_EX_NUM-1:0]    mmu_i_tlb_ex;
    // IFU <-> ICache
    wire                      ifu_ic_req;
    wire [31:0]               ifu_ic_vaddr;
    wire [31:0]               ifu_ic_paddr;
    wire                      ifu_ic_uncached;
    wire                      ic_ifu_addr_ok;
    wire                      ic_ifu_data_ok;
    wire [`CACHE_LINE_BITS-1:0] ic_ifu_rline;
    wire                      ifu_ic_cancel;
    // IFU -> IB（4 槽）
    wire                      ifu_ib0_valid, ifu_ib1_valid, ifu_ib2_valid, ifu_ib3_valid;
    wire [31:0]               ifu_ib0_pc,    ifu_ib1_pc,    ifu_ib2_pc,    ifu_ib3_pc;
    wire [31:0]               ifu_ib0_inst,  ifu_ib1_inst,  ifu_ib2_inst,  ifu_ib3_inst;
    wire                      ifu_ib0_pred_taken, ifu_ib1_pred_taken, ifu_ib2_pred_taken, ifu_ib3_pred_taken;
    wire                      ifu_ib0_is_last, ifu_ib1_is_last, ifu_ib2_is_last, ifu_ib3_is_last;
    wire [`FTQ_W-1:0]         ifu_ib0_ftq_id, ifu_ib1_ftq_id, ifu_ib2_ftq_id, ifu_ib3_ftq_id;
    wire [`EXCP_NUM-1:0]      ifu_ib0_excp,  ifu_ib1_excp,  ifu_ib2_excp,  ifu_ib3_excp;
    wire                      ib_can_push;
    // IB -> 后端（2 槽）
    wire                      ib_pop0_valid, ib_pop1_valid;
    wire [31:0]               ib_pop0_pc,    ib_pop1_pc;
    wire [31:0]               ib_pop0_inst,  ib_pop1_inst;
    wire                      ib_pop0_pred_taken, ib_pop1_pred_taken;
    wire                      ib_pop0_is_last, ib_pop1_is_last;
    wire [`FTQ_W-1:0]         ib_pop0_ftq_id, ib_pop1_ftq_id;
    wire [`EXCP_NUM-1:0]      ib_pop0_excp,  ib_pop1_excp;
    wire                      ib_pop0_ready, ib_pop1_ready;
    // commit -> FTQ 提交/训练 与 -> BPU RAS 提交栈
    wire                      cmt_ftq_valid;
    wire [`FTQ_W-1:0]         cmt_ftq_id;
    wire                      cmt_ftq_is_last;
    wire [1:0]                cmt_ftq_release;
    wire                      cmt_ftq_is_branch;
    wire                      cmt_ftq_taken;
    wire                      cmt_ftq_mispred;
    wire [31:0]               cmt_ftq_target;
    wire [`BR_TYPE_W-1:0]     cmt_ftq_br_type;
    wire [31:0]               cmt_ftq_pc;
    wire [`FTQ_W-1:0]         cmt_ftq_query_id;
    wire [31:0]               ftq_cmt_blk_target;
    wire                      cmt_ras_call;
    wire                      cmt_ras_ret;
    wire [31:0]               cmt_ras_retaddr;

//--------------------------------------------------
// bpu：分支预测单元（内含 ubtb/ftb/tage/ras 四个子模块）
//--------------------------------------------------
    bpu u_bpu(
        .clk                  (clk),
        .reset                (reset),
        .flush_i              (flush),
        .flush_pc_i           (flush_pc),
        .predec_redirect_i    (predec_redirect),
        .predec_update_pc_i   (predec_update_pc),
        .predec_redirect_pc_i (predec_redirect_pc),
        // FTQ 满或 idle 睡眠都要冻结取指 PC
        .ftq_full_i           (ftq_full | fetch_stall),
        .p0_valid_o           (bpu_p0_valid),
        .p0_pc_o              (bpu_p0_pc),
        .p0_length_o          (bpu_p0_length),
        .p0_taken_o           (bpu_p0_taken),
        .p0_target_o          (bpu_p0_target),
        .p0_br_type_o         (bpu_p0_br_type),
        .p1_valid_o           (bpu_p1_valid),
        .p1_meta_valid_o      (bpu_p1_meta_valid),
        .p1_pc_o              (bpu_p1_pc),
        .p1_length_o          (bpu_p1_length),
        .p1_taken_o           (bpu_p1_taken),
        .p1_target_o          (bpu_p1_target),
        .p1_br_type_o         (bpu_p1_br_type),
        .p1_meta_o            (bpu_p1_meta),
        .train_valid_i        (ftq_train_valid),
        .train_pc_i           (ftq_train_pc),
        .train_is_branch_i    (ftq_train_is_branch),
        .train_taken_i        (ftq_train_taken),
        .train_mispred_i      (ftq_train_mispred),
        .train_target_i       (ftq_train_target),
        .train_br_type_i      (ftq_train_br_type),
        .train_fall_through_i (ftq_train_fall_through),
        .train_meta_i         (ftq_train_meta),
        .cmt_is_call_i        (cmt_ras_call),
        .cmt_is_ret_i         (cmt_ras_ret),
        .cmt_call_retaddr_i   (cmt_ras_retaddr)
    );

//--------------------------------------------------
// ftq：取指目标队列（BPU 与 IFU 解耦 + 训练元数据暂存）
//--------------------------------------------------
    ftq u_ftq(
        .clk                  (clk),
        .reset                (reset),
        .flush_i              (flush),
        .p0_valid_i           (bpu_p0_valid),
        .p0_pc_i              (bpu_p0_pc),
        .p0_length_i          (bpu_p0_length),
        .p0_taken_i           (bpu_p0_taken),
        .p0_target_i          (bpu_p0_target),
        .p0_br_type_i         (bpu_p0_br_type),
        .p1_valid_i           (bpu_p1_valid),
        .p1_meta_valid_i      (bpu_p1_meta_valid),
        .p1_pc_i              (bpu_p1_pc),
        .p1_length_i          (bpu_p1_length),
        .p1_taken_i           (bpu_p1_taken),
        .p1_target_i          (bpu_p1_target),
        .p1_br_type_i         (bpu_p1_br_type),
        .p1_meta_i            (bpu_p1_meta),
        .ftq_full_o           (ftq_full),
        .ifu_valid_o          (ftq_ifu_valid),
        .ifu_pc_o             (ftq_ifu_pc),
        .ifu_length_o         (ftq_ifu_length),
        .ifu_taken_o          (ftq_ifu_taken),
        .ifu_target_o         (ftq_ifu_target),
        .ifu_ftq_id_o         (ftq_ifu_id),
        .ifu_accept_i         (ifu_ftq_accept),
        .predec_redirect_i    (predec_redirect),
        .predec_fixup_only_i  (predec_fixup_only),
        .predec_redirect_id_i (predec_redirect_id),
        .predec_length_i      (predec_length),
        .predec_taken_i       (predec_taken),
        .predec_target_i      (predec_target),
        .predec_br_type_i     (predec_br_type),
        .cmt_valid_i          (cmt_ftq_valid),
        .cmt_ftq_id_i         (cmt_ftq_id),
        .cmt_is_last_i        (cmt_ftq_is_last),
        .cmt_release_i        (cmt_ftq_release),
        .cmt_is_branch_i      (cmt_ftq_is_branch),
        .cmt_taken_i          (cmt_ftq_taken),
        .cmt_mispred_i        (cmt_ftq_mispred),
        .cmt_target_i         (cmt_ftq_target),
        .cmt_br_type_i        (cmt_ftq_br_type),
        .cmt_pc_i             (cmt_ftq_pc),
        .cmt_query_id_i       (cmt_ftq_query_id),
        .cmt_blk_target_o     (ftq_cmt_blk_target),
        .train_valid_o        (ftq_train_valid),
        .train_pc_o           (ftq_train_pc),
        .train_is_branch_o    (ftq_train_is_branch),
        .train_taken_o        (ftq_train_taken),
        .train_mispred_o      (ftq_train_mispred),
        .train_target_o       (ftq_train_target),
        .train_br_type_o      (ftq_train_br_type),
        .train_fall_through_o (ftq_train_fall_through),
        .train_meta_o         (ftq_train_meta)
    );

//--------------------------------------------------
// ifu：取指单元（取块 -> 翻译 -> 取整行 -> 切割/预译码 -> 入 IB）
//--------------------------------------------------
    ifu u_ifu(
        .clk                  (clk),
        .reset                (reset),
        .flush_i              (flush),
        .ftq_valid_i          (ftq_ifu_valid),
        .ftq_pc_i             (ftq_ifu_pc),
        .ftq_length_i         (ftq_ifu_length),
        .ftq_taken_i          (ftq_ifu_taken),
        .ftq_target_i         (ftq_ifu_target),
        .ftq_ftq_id_i         (ftq_ifu_id),
        .ftq_accept_o         (ifu_ftq_accept),
        .mmu_i_req_o          (ifu_mmu_req),
        .mmu_i_vaddr_o        (ifu_mmu_vaddr),
        .mmu_i_paddr_i        (mmu_i_paddr),
        .mmu_i_mat_i          (mmu_i_mat),
        .mmu_i_excp_adef_i    (mmu_i_adef),
        .mmu_i_tlb_ex_i       (mmu_i_tlb_ex),
        .ic_req_o             (ifu_ic_req),
        .ic_vaddr_o           (ifu_ic_vaddr),
        .ic_paddr_o           (ifu_ic_paddr),
        .ic_uncached_o        (ifu_ic_uncached),
        .ic_addr_ok_i         (ic_ifu_addr_ok),
        .ic_data_ok_i         (ic_ifu_data_ok),
        .ic_rline_i           (ic_ifu_rline),
        .ic_cancel_o          (ifu_ic_cancel),
        .predec_redirect_o    (predec_redirect),
        .predec_fixup_only_o  (predec_fixup_only),
        .predec_update_pc_o   (predec_update_pc),
        .predec_redirect_pc_o (predec_redirect_pc),
        .predec_redirect_id_o (predec_redirect_id),
        .predec_length_o      (predec_length),
        .predec_taken_o       (predec_taken),
        .predec_target_o      (predec_target),
        .predec_br_type_o     (predec_br_type),
        .ib_push0_valid_o     (ifu_ib0_valid),
        .ib_push0_pc_o        (ifu_ib0_pc),
        .ib_push0_inst_o      (ifu_ib0_inst),
        .ib_push0_pred_taken_o(ifu_ib0_pred_taken),
        .ib_push0_is_last_o   (ifu_ib0_is_last),
        .ib_push0_ftq_id_o    (ifu_ib0_ftq_id),
        .ib_push0_excp_o      (ifu_ib0_excp),
        .ib_push1_valid_o     (ifu_ib1_valid),
        .ib_push1_pc_o        (ifu_ib1_pc),
        .ib_push1_inst_o      (ifu_ib1_inst),
        .ib_push1_pred_taken_o(ifu_ib1_pred_taken),
        .ib_push1_is_last_o   (ifu_ib1_is_last),
        .ib_push1_ftq_id_o    (ifu_ib1_ftq_id),
        .ib_push1_excp_o      (ifu_ib1_excp),
        .ib_push2_valid_o     (ifu_ib2_valid),
        .ib_push2_pc_o        (ifu_ib2_pc),
        .ib_push2_inst_o      (ifu_ib2_inst),
        .ib_push2_pred_taken_o(ifu_ib2_pred_taken),
        .ib_push2_is_last_o   (ifu_ib2_is_last),
        .ib_push2_ftq_id_o    (ifu_ib2_ftq_id),
        .ib_push2_excp_o      (ifu_ib2_excp),
        .ib_push3_valid_o     (ifu_ib3_valid),
        .ib_push3_pc_o        (ifu_ib3_pc),
        .ib_push3_inst_o      (ifu_ib3_inst),
        .ib_push3_pred_taken_o(ifu_ib3_pred_taken),
        .ib_push3_is_last_o   (ifu_ib3_is_last),
        .ib_push3_ftq_id_o    (ifu_ib3_ftq_id),
        .ib_push3_excp_o      (ifu_ib3_excp),
        .ib_can_push_i        (ib_can_push)
    );

//--------------------------------------------------
// icache：L1 指令缓存（整行取指；下层接 L2）
//--------------------------------------------------
    // ICache <-> L2 互联
    wire         ic_l2_rd_req;
    wire [2:0]   ic_l2_rd_type;
    wire [31:0]  ic_l2_rd_addr;
    wire         l2_ic_rd_rdy;
    wire         l2_ic_ret_valid;
    wire         l2_ic_ret_last;
    wire [127:0] l2_ic_ret_data;
    // commit -> I$ cache 维护
    wire         cmt_icacop_valid;
    wire [1:0]   cmt_icacop_op;
    wire [31:0]  cmt_icacop_addr;

    icache u_icache(
        .clk            (clk),
        .resetn         (aresetn),
        .ifu_req_i      (ifu_ic_req),
        .ifu_vaddr_i    (ifu_ic_vaddr),
        .ifu_paddr_i    (ifu_ic_paddr),
        .ifu_uncached_i (ifu_ic_uncached),
        .ifu_addr_ok_o  (ic_ifu_addr_ok),
        .ifu_data_ok_o  (ic_ifu_data_ok),
        .ifu_rline_o    (ic_ifu_rline),
        .ifu_cancel_i   (ifu_ic_cancel),
        .cacop_en_i     (cmt_icacop_valid),
        .cacop_op_i     (cmt_icacop_op),
        .cacop_addr_i   (cmt_icacop_addr),
        .axi_rd_req     (ic_l2_rd_req),
        .axi_rd_type    (ic_l2_rd_type),
        .axi_rd_addr    (ic_l2_rd_addr),
        .axi_rd_rdy     (l2_ic_rd_rdy),
        .axi_ret_valid  (l2_ic_ret_valid),
        .axi_ret_last   (l2_ic_ret_last),
        .axi_ret_data   (l2_ic_ret_data)
    );

//--------------------------------------------------
// inst_buffer：指令缓冲（前端 4 进 / 后端 2 出）
//--------------------------------------------------
    inst_buffer u_inst_buffer(
        .clk               (clk),
        .reset             (reset),
        .flush_i           (flush),
        .push0_valid_i     (ifu_ib0_valid),
        .push0_pc_i        (ifu_ib0_pc),
        .push0_inst_i      (ifu_ib0_inst),
        .push0_pred_taken_i(ifu_ib0_pred_taken),
        .push0_is_last_i   (ifu_ib0_is_last),
        .push0_ftq_id_i    (ifu_ib0_ftq_id),
        .push0_excp_i      (ifu_ib0_excp),
        .push1_valid_i     (ifu_ib1_valid),
        .push1_pc_i        (ifu_ib1_pc),
        .push1_inst_i      (ifu_ib1_inst),
        .push1_pred_taken_i(ifu_ib1_pred_taken),
        .push1_is_last_i   (ifu_ib1_is_last),
        .push1_ftq_id_i    (ifu_ib1_ftq_id),
        .push1_excp_i      (ifu_ib1_excp),
        .push2_valid_i     (ifu_ib2_valid),
        .push2_pc_i        (ifu_ib2_pc),
        .push2_inst_i      (ifu_ib2_inst),
        .push2_pred_taken_i(ifu_ib2_pred_taken),
        .push2_is_last_i   (ifu_ib2_is_last),
        .push2_ftq_id_i    (ifu_ib2_ftq_id),
        .push2_excp_i      (ifu_ib2_excp),
        .push3_valid_i     (ifu_ib3_valid),
        .push3_pc_i        (ifu_ib3_pc),
        .push3_inst_i      (ifu_ib3_inst),
        .push3_pred_taken_i(ifu_ib3_pred_taken),
        .push3_is_last_i   (ifu_ib3_is_last),
        .push3_ftq_id_i    (ifu_ib3_ftq_id),
        .push3_excp_i      (ifu_ib3_excp),
        .can_push_o        (ib_can_push),
        .pop0_valid_o      (ib_pop0_valid),
        .pop0_pc_o         (ib_pop0_pc),
        .pop0_inst_o       (ib_pop0_inst),
        .pop0_pred_taken_o (ib_pop0_pred_taken),
        .pop0_is_last_o    (ib_pop0_is_last),
        .pop0_ftq_id_o     (ib_pop0_ftq_id),
        .pop0_excp_o       (ib_pop0_excp),
        .pop0_ready_i      (ib_pop0_ready),
        .pop1_valid_o      (ib_pop1_valid),
        .pop1_pc_o         (ib_pop1_pc),
        .pop1_inst_o       (ib_pop1_inst),
        .pop1_pred_taken_o (ib_pop1_pred_taken),
        .pop1_is_last_o    (ib_pop1_is_last),
        .pop1_ftq_id_o     (ib_pop1_ftq_id),
        .pop1_excp_o       (ib_pop1_excp),
        .pop1_ready_i      (ib_pop1_ready)
    );



//--------------------------------------------------
// 译码信号声明（decoder x2，组合插在 IB 与 rename 之间）
//--------------------------------------------------
    // CSR 状态前置声明（CSR 分区驱动）
    wire [1:0]                csr_crmd_plv;
    wire                      csr_llbit;

    // decoder 槽 0 输出
    wire [`FU_NUM-1:0]        dec0_futype;
    wire [`ALU_OP_NUM-1:0]    dec0_alu_op;
    wire [`BR_OP_NUM-1:0]     dec0_br_op;
    wire [`MEM_OP_NUM-1:0]    dec0_mem_op;
    wire [`CSR_OP_NUM-1:0]    dec0_csr_op;
    wire [`TLB_OP_NUM-1:0]    dec0_tlb_op;
    wire [`WB_SRC_NUM-1:0]    dec0_wb_src_op;
    wire [13:0]               dec0_csr_num;
    wire [4:0]                dec0_cacop_code;
    wire [4:0]                dec0_src0_addr, dec0_src1_addr;
    wire                      dec0_use_src0,  dec0_use_src1;
    wire [4:0]                dec0_rd_addr;
    wire                      dec0_rf_we;
    wire [31:0]               dec0_imm;
    wire                      dec0_use_imm;
    wire [31:0]               dec0_br_offs;
    wire                      dec0_is_branch;
    wire [`BR_TYPE_W-1:0]     dec0_br_type;
    wire [`PRIV_NUM-1:0]      dec0_priv_vec;
    wire                      dec0_is_nop;
    wire                      dec0_is_load, dec0_is_store;
    wire [`EXCP_NUM-1:0]      dec0_excp;
    // decoder 槽 1 输出
    wire [`FU_NUM-1:0]        dec1_futype;
    wire [`ALU_OP_NUM-1:0]    dec1_alu_op;
    wire [`BR_OP_NUM-1:0]     dec1_br_op;
    wire [`MEM_OP_NUM-1:0]    dec1_mem_op;
    wire [`CSR_OP_NUM-1:0]    dec1_csr_op;
    wire [`TLB_OP_NUM-1:0]    dec1_tlb_op;
    wire [`WB_SRC_NUM-1:0]    dec1_wb_src_op;
    wire [13:0]               dec1_csr_num;
    wire [4:0]                dec1_cacop_code;
    wire [4:0]                dec1_src0_addr, dec1_src1_addr;
    wire                      dec1_use_src0,  dec1_use_src1;
    wire [4:0]                dec1_rd_addr;
    wire                      dec1_rf_we;
    wire [31:0]               dec1_imm;
    wire                      dec1_use_imm;
    wire [31:0]               dec1_br_offs;
    wire                      dec1_is_branch;
    wire [`BR_TYPE_W-1:0]     dec1_br_type;
    wire [`PRIV_NUM-1:0]      dec1_priv_vec;
    wire                      dec1_is_nop;
    wire                      dec1_is_load, dec1_is_store;
    wire [`EXCP_NUM-1:0]      dec1_excp;

    // 取指异常与译码异常合并后送 rename（一条指令的全部"静态"异常）
    // 取指 fault 时 IFU 常把 inst 置 0；译码会对 inst=0 再报 INE，合并后只剩 INE
    // 且优先级低于 ADEF，导致 commit 只看见 INE。此处抑制这类重复/气泡 INE。
    wire [`EXCP_NUM-1:0] ib0_fetch_excp = ib_pop0_excp & `FETCH_EXCP_MASK;
    wire [`EXCP_NUM-1:0] ib1_fetch_excp = ib_pop1_excp & `FETCH_EXCP_MASK;
    // INE 抑制只在“已有取指类异常”时生效（取指异常优先级高于 INE，同拍不重复触发）。
    // 【勿再用 inst==0 的 null_bubble 抑制】：n50 用一条真实的 0x00000000 非法指令测 INE；
    // 把全零指令当气泡吞掉 INE 会导致它被当 nop 提交、INE 永不触发 → n50 失败。
    // 真正的取指气泡由 ib_pop*_valid 门控(rename ib*_valid_i)冲掉，与 INE 无关，无需在此按位模式猜。
    wire                 dec0_suppress_ine = dec0_excp[`EXCP_INE] & (|ib0_fetch_excp);
    wire                 dec1_suppress_ine = dec1_excp[`EXCP_INE] & (|ib1_fetch_excp);
    wire [`EXCP_NUM-1:0] dec0_excp_ine_mask = {{(`EXCP_NUM-1){1'b0}}, dec0_suppress_ine} << `EXCP_INE;
    wire [`EXCP_NUM-1:0] dec1_excp_ine_mask = {{(`EXCP_NUM-1){1'b0}}, dec1_suppress_ine} << `EXCP_INE;
    wire [`EXCP_NUM-1:0] dec0_excp_eff = dec0_excp & ~dec0_excp_ine_mask;
    wire [`EXCP_NUM-1:0] dec1_excp_eff = dec1_excp & ~dec1_excp_ine_mask;
    wire [`EXCP_NUM-1:0]      rn_ib0_excp = ib_pop0_excp | dec0_excp_eff;
    wire [`EXCP_NUM-1:0]      rn_ib1_excp = ib_pop1_excp | dec1_excp_eff;

//--------------------------------------------------
// decoder x2：双发射译码（纯组合）
//--------------------------------------------------
    decoder u_decoder0(
        .inst_i        (ib_pop0_inst),
        .pc_i          (ib_pop0_pc),
        .csr_plv_i     (csr_crmd_plv),
        .csr_llbit_i   (csr_llbit),
        .futype_o      (dec0_futype),
        .alu_op_o      (dec0_alu_op),
        .br_op_o       (dec0_br_op),
        .mem_op_o      (dec0_mem_op),
        .csr_op_o      (dec0_csr_op),
        .tlb_op_o      (dec0_tlb_op),
        .wb_src_op_o   (dec0_wb_src_op),
        .csr_num_o     (dec0_csr_num),
        .cacop_code_o  (dec0_cacop_code),
        .src0_addr_o   (dec0_src0_addr),
        .src1_addr_o   (dec0_src1_addr),
        .use_src0_o    (dec0_use_src0),
        .use_src1_o    (dec0_use_src1),
        .rd_addr_o     (dec0_rd_addr),
        .rf_we_o       (dec0_rf_we),
        .imm_o         (dec0_imm),
        .use_imm_o     (dec0_use_imm),
        .br_offs_o     (dec0_br_offs),
        .is_branch_o   (dec0_is_branch),
        .br_type_o     (dec0_br_type),
        .priv_vec_o    (dec0_priv_vec),
        .is_nop_o      (dec0_is_nop),
        .is_load_o     (dec0_is_load),
        .is_store_o    (dec0_is_store),
        .excp_o        (dec0_excp)
    );

    decoder u_decoder1(
        .inst_i        (ib_pop1_inst),
        .pc_i          (ib_pop1_pc),
        .csr_plv_i     (csr_crmd_plv),
        .csr_llbit_i   (csr_llbit),
        .futype_o      (dec1_futype),
        .alu_op_o      (dec1_alu_op),
        .br_op_o       (dec1_br_op),
        .mem_op_o      (dec1_mem_op),
        .csr_op_o      (dec1_csr_op),
        .tlb_op_o      (dec1_tlb_op),
        .wb_src_op_o   (dec1_wb_src_op),
        .csr_num_o     (dec1_csr_num),
        .cacop_code_o  (dec1_cacop_code),
        .src0_addr_o   (dec1_src0_addr),
        .src1_addr_o   (dec1_src1_addr),
        .use_src0_o    (dec1_use_src0),
        .use_src1_o    (dec1_use_src1),
        .rd_addr_o     (dec1_rd_addr),
        .rf_we_o       (dec1_rf_we),
        .imm_o         (dec1_imm),
        .use_imm_o     (dec1_use_imm),
        .br_offs_o     (dec1_br_offs),
        .is_branch_o   (dec1_is_branch),
        .br_type_o     (dec1_br_type),
        .priv_vec_o    (dec1_priv_vec),
        .is_nop_o      (dec1_is_nop),
        .is_load_o     (dec1_is_load),
        .is_store_o    (dec1_is_store),
        .excp_o        (dec1_excp)
    );



//--------------------------------------------------
// 重命名/分发信号声明（rename / rat / regfile(ARF) / dispatch）
//--------------------------------------------------
    // rename <-> RAT
    wire [4:0]                rn_rat_raddr0, rn_rat_raddr1, rn_rat_raddr2, rn_rat_raddr3;
    wire                      rat_rbusy0, rat_rbusy1, rat_rbusy2, rat_rbusy3;
    wire [`ROB_W-1:0]         rat_rnum0, rat_rnum1, rat_rnum2, rat_rnum3;
    wire                      rn_rat_wen0, rn_rat_wen1;
    wire [4:0]                rn_rat_waddr0, rn_rat_waddr1;
    wire [`ROB_W-1:0]         rn_rat_wnum0, rn_rat_wnum1;
    // commit -> RAT 释放
    wire                      cmt_rat_en0, cmt_rat_en1;
    wire [4:0]                cmt_rat_addr0, cmt_rat_addr1;
    wire [`ROB_W-1:0]         cmt_rat_num0, cmt_rat_num1;
    // rename <-> ARF 读
    wire [4:0]                rn_arf_raddr0, rn_arf_raddr1, rn_arf_raddr2, rn_arf_raddr3;
    wire [31:0]               arf_rdata0, arf_rdata1, arf_rdata2, arf_rdata3;
    // commit -> ARF 写
    wire                      cmt_arf_we0, cmt_arf_we1;
    wire [4:0]                cmt_arf_waddr0, cmt_arf_waddr1;
    wire [31:0]               cmt_arf_wdata0, cmt_arf_wdata1;
    // rename <-> ROB 分配
    wire                      rn_rob_alloc_en;
    wire [`ROB_PAIR_W-1:0]    rob_tail;
    wire                      rob_full;
    wire                      rob_empty;
    wire                      rn_a0_valid, rn_a1_valid;
    wire [31:0]               rn_a0_pc, rn_a1_pc;
    wire [31:0]               rn_a0_inst, rn_a1_inst;
    wire                      rn_a0_rf_we, rn_a1_rf_we;
    wire [4:0]                rn_a0_rd, rn_a1_rd;
    wire [`FU_NUM-1:0]        rn_a0_futype, rn_a1_futype;
    wire                      rn_a0_is_load, rn_a1_is_load;
    wire                      rn_a0_is_store, rn_a1_is_store;
    wire                      rn_a0_is_branch, rn_a1_is_branch;
    wire [`BR_TYPE_W-1:0]     rn_a0_br_type, rn_a1_br_type;
    wire                      rn_a0_pred_taken, rn_a1_pred_taken;
    wire                      rn_a0_is_last, rn_a1_is_last;
    wire [`FTQ_W-1:0]         rn_a0_ftq_id, rn_a1_ftq_id;
    wire [`PRIV_NUM-1:0]      rn_a0_priv_vec, rn_a1_priv_vec;
    wire [13:0]               rn_a0_csr_num, rn_a1_csr_num;
    wire [`TLB_OP_NUM-1:0]    rn_a0_tlb_op, rn_a1_tlb_op;
    wire [4:0]                rn_a0_cacop_code, rn_a1_cacop_code;
    wire [`EXCP_NUM-1:0]      rn_a0_excp, rn_a1_excp;
    wire                      rn_a0_is_nop, rn_a1_is_nop;
    // rename -> dispatch 流水寄存器
    wire                      dis0_valid, dis1_valid;
    wire [`ROB_W-1:0]         dis0_robid, dis1_robid;
    wire [31:0]               dis0_pc, dis1_pc;
    wire [`FU_NUM-1:0]        dis0_futype, dis1_futype;
    wire [`ALU_OP_NUM-1:0]    dis0_alu_op, dis1_alu_op;
    wire [`BR_OP_NUM-1:0]     dis0_br_op, dis1_br_op;
    wire [`MEM_OP_NUM-1:0]    dis0_mem_op, dis1_mem_op;
    wire [`CSR_OP_NUM-1:0]    dis0_csr_op, dis1_csr_op;
    wire [`TLB_OP_NUM-1:0]    dis0_tlb_op, dis1_tlb_op;
    wire [`WB_SRC_NUM-1:0]    dis0_wb_src_op, dis1_wb_src_op;
    wire [13:0]               dis0_csr_num, dis1_csr_num;
    wire                      dis0_is_cacop, dis1_is_cacop;
    wire                      dis0_src0_ready, dis1_src0_ready;
    wire [31:0]               dis0_src0_val, dis1_src0_val;
    wire [`ROB_W-1:0]         dis0_src0_robid, dis1_src0_robid;
    wire                      dis0_src1_ready, dis1_src1_ready;
    wire [31:0]               dis0_src1_val, dis1_src1_val;
    wire [`ROB_W-1:0]         dis0_src1_robid, dis1_src1_robid;
    wire [31:0]               dis0_imm, dis1_imm;
    wire                      dis0_use_imm, dis1_use_imm;
    wire [31:0]               dis0_br_offs, dis1_br_offs;
    wire                      dispatch_ready;
    wire                      dis0_fire;
    wire                      dis1_fire;
    // 分发驻留旁路（非流水寄存器数据；用锁存源地址组合读 RAT/ARF 口 4~7 实时唤醒）
    wire                      dis_rat_rbusy0, dis_rat_rbusy1, dis_rat_rbusy2, dis_rat_rbusy3;
    wire [31:0]               dis_arf_rdata0, dis_arf_rdata1, dis_arf_rdata2, dis_arf_rdata3;
    wire [4:0]                dis0_src0_addr, dis0_src1_addr, dis1_src0_addr, dis1_src1_addr;
    // dispatch <-> ROB 操作数读口
    wire [`ROB_W-1:0]         dsp_rob_raddr0, dsp_rob_raddr1, dsp_rob_raddr2, dsp_rob_raddr3;
    wire                      rob_rrdy0, rob_rrdy1, rob_rrdy2, rob_rrdy3;
    wire [31:0]               rob_rdata0, rob_rdata1, rob_rdata2, rob_rdata3;

//--------------------------------------------------
// rename：重命名级（查 RAT / 读 ARF / 分配 ROB / 锁存到分发级）
//--------------------------------------------------
    rename u_rename(
        .clk               (clk),
        .reset             (reset),
        .flush_i           (flush),
        .l0_csr_commit_i   (cmt_csr_we && `CSR_NUM_IS_L0_NOFLUSH(cmt_csr_wnum[11:0])),
        // 槽 0
        .ib0_valid_i       (ib_pop0_valid),
        .ib0_pc_i          (ib_pop0_pc),
        .ib0_inst_i        (ib_pop0_inst),
        .ib0_pred_taken_i  (ib_pop0_pred_taken),
        .ib0_is_last_i     (ib_pop0_is_last),
        .ib0_ftq_id_i      (ib_pop0_ftq_id),
        .ib0_excp_i        (rn_ib0_excp),
        .ib0_futype_i      (dec0_futype),
        .ib0_alu_op_i      (dec0_alu_op),
        .ib0_br_op_i       (dec0_br_op),
        .ib0_mem_op_i      (dec0_mem_op),
        .ib0_csr_op_i      (dec0_csr_op),
        .ib0_tlb_op_i      (dec0_tlb_op),
        .ib0_wb_src_op_i   (dec0_wb_src_op),
        .ib0_csr_num_i     (dec0_csr_num),
        .ib0_cacop_code_i  (dec0_cacop_code),
        .ib0_src0_addr_i   (dec0_src0_addr),
        .ib0_src1_addr_i   (dec0_src1_addr),
        .ib0_use_src0_i    (dec0_use_src0),
        .ib0_use_src1_i    (dec0_use_src1),
        .ib0_rd_addr_i     (dec0_rd_addr),
        .ib0_rf_we_i       (dec0_rf_we),
        .ib0_imm_i         (dec0_imm),
        .ib0_use_imm_i     (dec0_use_imm),
        .ib0_br_offs_i     (dec0_br_offs),
        .ib0_is_branch_i   (dec0_is_branch),
        .ib0_br_type_i     (dec0_br_type),
        .ib0_priv_vec_i    (dec0_priv_vec),
        .ib0_is_nop_i      (dec0_is_nop),
        .ib0_is_load_i     (dec0_is_load),
        .ib0_is_store_i    (dec0_is_store),
        .ib0_ready_o       (ib_pop0_ready),
        // 槽 1
        .ib1_valid_i       (ib_pop1_valid),
        .ib1_pc_i          (ib_pop1_pc),
        .ib1_inst_i        (ib_pop1_inst),
        .ib1_pred_taken_i  (ib_pop1_pred_taken),
        .ib1_is_last_i     (ib_pop1_is_last),
        .ib1_ftq_id_i      (ib_pop1_ftq_id),
        .ib1_excp_i        (rn_ib1_excp),
        .ib1_futype_i      (dec1_futype),
        .ib1_alu_op_i      (dec1_alu_op),
        .ib1_br_op_i       (dec1_br_op),
        .ib1_mem_op_i      (dec1_mem_op),
        .ib1_csr_op_i      (dec1_csr_op),
        .ib1_tlb_op_i      (dec1_tlb_op),
        .ib1_wb_src_op_i   (dec1_wb_src_op),
        .ib1_csr_num_i     (dec1_csr_num),
        .ib1_cacop_code_i  (dec1_cacop_code),
        .ib1_src0_addr_i   (dec1_src0_addr),
        .ib1_src1_addr_i   (dec1_src1_addr),
        .ib1_use_src0_i    (dec1_use_src0),
        .ib1_use_src1_i    (dec1_use_src1),
        .ib1_rd_addr_i     (dec1_rd_addr),
        .ib1_rf_we_i       (dec1_rf_we),
        .ib1_imm_i         (dec1_imm),
        .ib1_use_imm_i     (dec1_use_imm),
        .ib1_br_offs_i     (dec1_br_offs),
        .ib1_is_branch_i   (dec1_is_branch),
        .ib1_br_type_i     (dec1_br_type),
        .ib1_priv_vec_i    (dec1_priv_vec),
        .ib1_is_nop_i      (dec1_is_nop),
        .ib1_is_load_i     (dec1_is_load),
        .ib1_is_store_i    (dec1_is_store),
        .ib1_ready_o       (ib_pop1_ready),
        // RAT
        .rat_raddr0_o      (rn_rat_raddr0),
        .rat_rbusy0_i      (rat_rbusy0),
        .rat_rnum0_i       (rat_rnum0),
        .rat_raddr1_o      (rn_rat_raddr1),
        .rat_rbusy1_i      (rat_rbusy1),
        .rat_rnum1_i       (rat_rnum1),
        .rat_raddr2_o      (rn_rat_raddr2),
        .rat_rbusy2_i      (rat_rbusy2),
        .rat_rnum2_i       (rat_rnum2),
        .rat_raddr3_o      (rn_rat_raddr3),
        .rat_rbusy3_i      (rat_rbusy3),
        .rat_rnum3_i       (rat_rnum3),
        .rat_wen0_o        (rn_rat_wen0),
        .rat_waddr0_o      (rn_rat_waddr0),
        .rat_wnum0_o       (rn_rat_wnum0),
        .rat_wen1_o        (rn_rat_wen1),
        .rat_waddr1_o      (rn_rat_waddr1),
        .rat_wnum1_o       (rn_rat_wnum1),
        // ARF 读
        .arf_raddr0_o      (rn_arf_raddr0),
        .arf_rdata0_i      (arf_rdata0),
        .arf_raddr1_o      (rn_arf_raddr1),
        .arf_rdata1_i      (arf_rdata1),
        .arf_raddr2_o      (rn_arf_raddr2),
        .arf_rdata2_i      (arf_rdata2),
        .arf_raddr3_o      (rn_arf_raddr3),
        .arf_rdata3_i      (arf_rdata3),
        // ROB 分配
        .rob_alloc_en_o    (rn_rob_alloc_en),
        .rob_tail_i        (rob_tail),
        .rob_full_i        (rob_full),
        .rob_a0_valid_o    (rn_a0_valid),
        .rob_a0_pc_o       (rn_a0_pc),
        .rob_a0_inst_o     (rn_a0_inst),
        .rob_a0_rf_we_o    (rn_a0_rf_we),
        .rob_a0_rd_o       (rn_a0_rd),
        .rob_a0_futype_o   (rn_a0_futype),
        .rob_a0_is_load_o  (rn_a0_is_load),
        .rob_a0_is_store_o (rn_a0_is_store),
        .rob_a0_is_branch_o(rn_a0_is_branch),
        .rob_a0_br_type_o  (rn_a0_br_type),
        .rob_a0_pred_taken_o(rn_a0_pred_taken),
        .rob_a0_is_last_o  (rn_a0_is_last),
        .rob_a0_ftq_id_o   (rn_a0_ftq_id),
        .rob_a0_priv_vec_o (rn_a0_priv_vec),
        .rob_a0_csr_num_o  (rn_a0_csr_num),
        .rob_a0_tlb_op_o   (rn_a0_tlb_op),
        .rob_a0_cacop_code_o(rn_a0_cacop_code),
        .rob_a0_excp_o     (rn_a0_excp),
        .rob_a0_is_nop_o   (rn_a0_is_nop),
        .rob_a1_valid_o    (rn_a1_valid),
        .rob_a1_pc_o       (rn_a1_pc),
        .rob_a1_inst_o     (rn_a1_inst),
        .rob_a1_rf_we_o    (rn_a1_rf_we),
        .rob_a1_rd_o       (rn_a1_rd),
        .rob_a1_futype_o   (rn_a1_futype),
        .rob_a1_is_load_o  (rn_a1_is_load),
        .rob_a1_is_store_o (rn_a1_is_store),
        .rob_a1_is_branch_o(rn_a1_is_branch),
        .rob_a1_br_type_o  (rn_a1_br_type),
        .rob_a1_pred_taken_o(rn_a1_pred_taken),
        .rob_a1_is_last_o  (rn_a1_is_last),
        .rob_a1_ftq_id_o   (rn_a1_ftq_id),
        .rob_a1_priv_vec_o (rn_a1_priv_vec),
        .rob_a1_csr_num_o  (rn_a1_csr_num),
        .rob_a1_tlb_op_o   (rn_a1_tlb_op),
        .rob_a1_cacop_code_o(rn_a1_cacop_code),
        .rob_a1_excp_o     (rn_a1_excp),
        .rob_a1_is_nop_o   (rn_a1_is_nop),
        // 分发级
        .dispatch_ready_i  (dispatch_ready),
        .dis0_fire_i       (dis0_fire),
        .dis1_fire_i       (dis1_fire),
        .dis0_valid_o      (dis0_valid),
        .dis0_robid_o      (dis0_robid),
        .dis0_pc_o         (dis0_pc),
        .dis0_futype_o     (dis0_futype),
        .dis0_alu_op_o     (dis0_alu_op),
        .dis0_br_op_o      (dis0_br_op),
        .dis0_mem_op_o     (dis0_mem_op),
        .dis0_csr_op_o     (dis0_csr_op),
        .dis0_tlb_op_o     (dis0_tlb_op),
        .dis0_wb_src_op_o  (dis0_wb_src_op),
        .dis0_csr_num_o    (dis0_csr_num),
        .dis0_is_cacop_o   (dis0_is_cacop),
        .dis0_src0_ready_o (dis0_src0_ready),
        .dis0_src0_val_o   (dis0_src0_val),
        .dis0_src0_robid_o   (dis0_src0_robid),
        .dis0_src1_ready_o   (dis0_src1_ready),
        .dis0_src1_val_o     (dis0_src1_val),
        .dis0_src1_robid_o   (dis0_src1_robid),
        .dis0_imm_o        (dis0_imm),
        .dis0_use_imm_o    (dis0_use_imm),
        .dis0_br_offs_o    (dis0_br_offs),
        .dis1_valid_o      (dis1_valid),
        .dis1_robid_o      (dis1_robid),
        .dis1_pc_o         (dis1_pc),
        .dis1_futype_o     (dis1_futype),
        .dis1_alu_op_o     (dis1_alu_op),
        .dis1_br_op_o      (dis1_br_op),
        .dis1_mem_op_o     (dis1_mem_op),
        .dis1_csr_op_o     (dis1_csr_op),
        .dis1_tlb_op_o     (dis1_tlb_op),
        .dis1_wb_src_op_o  (dis1_wb_src_op),
        .dis1_csr_num_o    (dis1_csr_num),
        .dis1_is_cacop_o   (dis1_is_cacop),
        .dis1_src0_ready_o (dis1_src0_ready),
        .dis1_src0_val_o   (dis1_src0_val),
        .dis1_src0_robid_o   (dis1_src0_robid),
        .dis1_src1_ready_o   (dis1_src1_ready),
        .dis1_src1_val_o     (dis1_src1_val),
        .dis1_src1_robid_o   (dis1_src1_robid),
        .dis1_imm_o        (dis1_imm),
        .dis1_use_imm_o    (dis1_use_imm),
        .dis1_br_offs_o    (dis1_br_offs),
        .rob_rrdy0_i       (rob_rrdy0),
        .rob_rdata0_i      (rob_rdata0),
        .rob_rrdy1_i       (rob_rrdy1),
        .rob_rdata1_i      (rob_rdata1),
        .rob_rrdy2_i       (rob_rrdy2),
        .rob_rdata2_i      (rob_rdata2),
        .rob_rrdy3_i       (rob_rrdy3),
        .rob_rdata3_i      (rob_rdata3),
        .dis_rat_rbusy0_i  (dis_rat_rbusy0),
        .dis_rat_rbusy1_i  (dis_rat_rbusy1),
        .dis_rat_rbusy2_i  (dis_rat_rbusy2),
        .dis_rat_rbusy3_i  (dis_rat_rbusy3),
        .dis_arf_rdata0_i  (dis_arf_rdata0),
        .dis_arf_rdata1_i  (dis_arf_rdata1),
        .dis_arf_rdata2_i  (dis_arf_rdata2),
        .dis_arf_rdata3_i  (dis_arf_rdata3),
        .dis0_src0_addr_o  (dis0_src0_addr),
        .dis0_src1_addr_o  (dis0_src1_addr),
        .dis1_src0_addr_o  (dis1_src0_addr),
        .dis1_src1_addr_o  (dis1_src1_addr)
    );

//--------------------------------------------------
// rat：寄存器别名表（32 x {busy, robid}）
//--------------------------------------------------
    rat u_rat(
        .clk         (clk),
        .reset       (reset),
        .flush_i     (flush),
        .raddr0_i    (rn_rat_raddr0),
        .rbusy0_o    (rat_rbusy0),
        .rnum0_o     (rat_rnum0),
        .raddr1_i    (rn_rat_raddr1),
        .rbusy1_o    (rat_rbusy1),
        .rnum1_o     (rat_rnum1),
        .raddr2_i    (rn_rat_raddr2),
        .rbusy2_o    (rat_rbusy2),
        .rnum2_o     (rat_rnum2),
        .raddr3_i    (rn_rat_raddr3),
        .rbusy3_o    (rat_rbusy3),
        .rnum3_o     (rat_rnum3),
        .raddr4_i    (dis0_src0_addr),
        .rbusy4_o    (dis_rat_rbusy0),
        .raddr5_i    (dis0_src1_addr),
        .rbusy5_o    (dis_rat_rbusy1),
        .raddr6_i    (dis1_src0_addr),
        .rbusy6_o    (dis_rat_rbusy2),
        .raddr7_i    (dis1_src1_addr),
        .rbusy7_o    (dis_rat_rbusy3),
        .wen0_i      (rn_rat_wen0),
        .waddr0_i    (rn_rat_waddr0),
        .wnum0_i     (rn_rat_wnum0),
        .wen1_i      (rn_rat_wen1),
        .waddr1_i    (rn_rat_waddr1),
        .wnum1_i     (rn_rat_wnum1),
        .cmt_en0_i   (cmt_rat_en0),
        .cmt_addr0_i (cmt_rat_addr0),
        .cmt_num0_i  (cmt_rat_num0),
        .cmt_en1_i   (cmt_rat_en1),
        .cmt_addr1_i (cmt_rat_addr1),
        .cmt_num1_i  (cmt_rat_num1)
    );

//--------------------------------------------------
// regfile：ARF 体系结构寄存器堆（4 读 2 写 + 调试/difftest）
//--------------------------------------------------
`ifdef DIFFTEST_EN
    wire [31:0] diff_gpr_0,  diff_gpr_1,  diff_gpr_2,  diff_gpr_3;
    wire [31:0] diff_gpr_4,  diff_gpr_5,  diff_gpr_6,  diff_gpr_7;
    wire [31:0] diff_gpr_8,  diff_gpr_9,  diff_gpr_10, diff_gpr_11;
    wire [31:0] diff_gpr_12, diff_gpr_13, diff_gpr_14, diff_gpr_15;
    wire [31:0] diff_gpr_16, diff_gpr_17, diff_gpr_18, diff_gpr_19;
    wire [31:0] diff_gpr_20, diff_gpr_21, diff_gpr_22, diff_gpr_23;
    wire [31:0] diff_gpr_24, diff_gpr_25, diff_gpr_26, diff_gpr_27;
    wire [31:0] diff_gpr_28, diff_gpr_29, diff_gpr_30, diff_gpr_31;
`endif

    regfile u_regfile(
        .clk       (clk),
        .raddr0    (rn_arf_raddr0),
        .rdata0    (arf_rdata0),
        .raddr1    (rn_arf_raddr1),
        .rdata1    (arf_rdata1),
        .raddr2    (rn_arf_raddr2),
        .rdata2    (arf_rdata2),
        .raddr3    (rn_arf_raddr3),
        .rdata3    (arf_rdata3),
        .raddr4    (dis0_src0_addr),
        .rdata4    (dis_arf_rdata0),
        .raddr5    (dis0_src1_addr),
        .rdata5    (dis_arf_rdata1),
        .raddr6    (dis1_src0_addr),
        .rdata6    (dis_arf_rdata2),
        .raddr7    (dis1_src1_addr),
        .rdata7    (dis_arf_rdata3),
        .we0       (cmt_arf_we0),
        .waddr0    (cmt_arf_waddr0),
        .wdata0    (cmt_arf_wdata0),
        .we1       (cmt_arf_we1),
        .waddr1    (cmt_arf_waddr1),
        .wdata1    (cmt_arf_wdata1),
        .dbg_raddr (reg_num),
        .dbg_rdata (rf_rdata)
`ifdef DIFFTEST_EN
        ,
        .diff_gpr_0 (diff_gpr_0),  .diff_gpr_1 (diff_gpr_1),
        .diff_gpr_2 (diff_gpr_2),  .diff_gpr_3 (diff_gpr_3),
        .diff_gpr_4 (diff_gpr_4),  .diff_gpr_5 (diff_gpr_5),
        .diff_gpr_6 (diff_gpr_6),  .diff_gpr_7 (diff_gpr_7),
        .diff_gpr_8 (diff_gpr_8),  .diff_gpr_9 (diff_gpr_9),
        .diff_gpr_10(diff_gpr_10), .diff_gpr_11(diff_gpr_11),
        .diff_gpr_12(diff_gpr_12), .diff_gpr_13(diff_gpr_13),
        .diff_gpr_14(diff_gpr_14), .diff_gpr_15(diff_gpr_15),
        .diff_gpr_16(diff_gpr_16), .diff_gpr_17(diff_gpr_17),
        .diff_gpr_18(diff_gpr_18), .diff_gpr_19(diff_gpr_19),
        .diff_gpr_20(diff_gpr_20), .diff_gpr_21(diff_gpr_21),
        .diff_gpr_22(diff_gpr_22), .diff_gpr_23(diff_gpr_23),
        .diff_gpr_24(diff_gpr_24), .diff_gpr_25(diff_gpr_25),
        .diff_gpr_26(diff_gpr_26), .diff_gpr_27(diff_gpr_27),
        .diff_gpr_28(diff_gpr_28), .diff_gpr_29(diff_gpr_29),
        .diff_gpr_30(diff_gpr_30), .diff_gpr_31(diff_gpr_31)
`endif
    );



//--------------------------------------------------
// 保留站/执行单元 信号声明（4xRS + 4xFU + 唤醒总线）
//--------------------------------------------------
    // 写回唤醒总线（4 路：alu0/alu1/lsu/mdu，广播到 4 个 RS + ROB）
    wire                      alu0_wb_valid, alu1_wb_valid, mem_wb_valid, mdu_wb_valid;
    wire [`ROB_W-1:0]         alu0_wb_robid, alu1_wb_robid, mem_wb_robid, mdu_wb_robid;
    wire [31:0]               alu0_wb_data,  alu1_wb_data,  mem_wb_data,  mdu_wb_data;
    wire                      alu0_wb_br_taken, alu1_wb_br_taken;
    wire [31:0]               alu0_wb_br_target, alu1_wb_br_target;
    wire [31:0]               mem_wb_paddr, mem_wb_vaddr;
    wire [3:0]                mem_wb_wstrb;
    wire [2:0]                mem_wb_size;
    wire                      mem_wb_uncached;
    wire [`EXCP_NUM-1:0]      mem_wb_excp;
    wire [31:0]               mdu_wb_data2;
    // 提前唤醒总线（3 路：alu0/alu1 发射拍 + lsu AGU 级；一期模块内恒 0）
    wire                      alu0_early_valid, alu1_early_valid, lsu_early_valid;
    wire [`ROB_W-1:0]         alu0_early_robid, alu1_early_robid, lsu_early_robid;
    // rs_alu0 入站/发射
    wire                      rsa0_can_accept;
    wire [`RS_ALU_OCC_W-1:0]  rsa0_occupancy;
    wire                      rsa0_push_valid;
    wire [`ROB_W-1:0]         rsa0_push_robid;
    wire [31:0]               rsa0_push_pc;
    wire [`ALU_OP_NUM-1:0]    rsa0_push_alu_op;
    wire [`BR_OP_NUM-1:0]     rsa0_push_br_op;
    wire                      rsa0_push_src0_ready, rsa0_push_src1_ready;
    wire [31:0]               rsa0_push_src0_val,   rsa0_push_src1_val;
    wire [`ROB_W-1:0]         rsa0_push_src0_robid, rsa0_push_src1_robid;
    wire [31:0]               rsa0_push_imm;
    wire                      rsa0_push_use_imm;
    wire [31:0]               rsa0_push_br_offs;
    wire                      rsa0_issue_valid;
    wire [`ROB_W-1:0]         rsa0_issue_robid;
    wire [31:0]               rsa0_issue_pc;
    wire [`ALU_OP_NUM-1:0]    rsa0_issue_alu_op;
    wire [`BR_OP_NUM-1:0]     rsa0_issue_br_op;
    wire [31:0]               rsa0_issue_src0, rsa0_issue_src1;
    wire [31:0]               rsa0_issue_imm;
    wire                      rsa0_issue_use_imm;
    wire [31:0]               rsa0_issue_br_offs;
    // rs_alu1 入站/发射
    wire                      rsa1_can_accept;
    wire [`RS_ALU_OCC_W-1:0]  rsa1_occupancy;
    wire                      rsa1_push_valid;
    wire [`ROB_W-1:0]         rsa1_push_robid;
    wire [31:0]               rsa1_push_pc;
    wire [`ALU_OP_NUM-1:0]    rsa1_push_alu_op;
    wire [`BR_OP_NUM-1:0]     rsa1_push_br_op;
    wire                      rsa1_push_src0_ready, rsa1_push_src1_ready;
    wire [31:0]               rsa1_push_src0_val,   rsa1_push_src1_val;
    wire [`ROB_W-1:0]         rsa1_push_src0_robid, rsa1_push_src1_robid;
    wire [31:0]               rsa1_push_imm;
    wire                      rsa1_push_use_imm;
    wire [31:0]               rsa1_push_br_offs;
    wire                      rsa1_issue_valid;
    wire [`ROB_W-1:0]         rsa1_issue_robid;
    wire [31:0]               rsa1_issue_pc;
    wire [`ALU_OP_NUM-1:0]    rsa1_issue_alu_op;
    wire [`BR_OP_NUM-1:0]     rsa1_issue_br_op;
    wire [31:0]               rsa1_issue_src0, rsa1_issue_src1;
    wire [31:0]               rsa1_issue_imm;
    wire                      rsa1_issue_use_imm;
    wire [31:0]               rsa1_issue_br_offs;
    // rs_mem 入站/发射
    wire                      rsm_can_accept;
    wire [`RS_MEM_OCC_W-1:0]  rsm_occupancy;
    wire                      rsm_push_valid;
    wire [`ROB_W-1:0]         rsm_push_robid;
    wire [31:0]               rsm_push_pc;
    wire [`MEM_OP_NUM-1:0]    rsm_push_mem_op;
    wire                      rsm_push_is_cacop;
    wire                      rsm_push_src0_ready, rsm_push_src1_ready;
    wire [31:0]               rsm_push_src0_val,   rsm_push_src1_val;
    wire [`ROB_W-1:0]         rsm_push_src0_robid, rsm_push_src1_robid;
    wire [31:0]               rsm_push_imm;
    wire                      rsm_issue_valid;
    wire [`ROB_W-1:0]         rsm_issue_robid;
    wire [31:0]               rsm_issue_pc;
    wire [`MEM_OP_NUM-1:0]    rsm_issue_mem_op;
    wire                      rsm_issue_is_cacop;
    wire [31:0]               rsm_issue_base, rsm_issue_wdata;
    wire [31:0]               rsm_issue_imm;
    wire                      lsu_ready;
    // rs_mdu 入站/发射
    wire                      rsd_can_accept;
    wire [1:0]                rsd_occupancy;
    wire                      rsd_push_valid;
    wire [`ROB_W-1:0]         rsd_push_robid;
    wire [`ALU_OP_NUM-1:0]    rsd_push_alu_op;
    wire [`CSR_OP_NUM-1:0]    rsd_push_csr_op;
    wire [13:0]               rsd_push_csr_num;
    wire [`TLB_OP_NUM-1:0]    rsd_push_tlb_op;
    wire [`WB_SRC_NUM-1:0]    rsd_push_wb_src_op;
    wire                      rsd_push_src0_ready, rsd_push_src1_ready;
    wire [31:0]               rsd_push_src0_val,   rsd_push_src1_val;
    wire [`ROB_W-1:0]         rsd_push_src0_robid, rsd_push_src1_robid;
    wire                      rsd_issue_valid;
    wire [`ROB_W-1:0]         rsd_issue_robid;
    wire [`ALU_OP_NUM-1:0]    rsd_issue_alu_op;
    wire [`CSR_OP_NUM-1:0]    rsd_issue_csr_op;
    wire [13:0]               rsd_issue_csr_num;
    wire [`TLB_OP_NUM-1:0]    rsd_issue_tlb_op;
    wire [`WB_SRC_NUM-1:0]    rsd_issue_wb_src_op;
    wire [31:0]               rsd_issue_src0, rsd_issue_src1;
    wire                      mdu_ready;

//--------------------------------------------------
// dispatch：分发级（读 ROB 补操作数 + 路由进 4 个保留站）
//--------------------------------------------------
    dispatch u_dispatch(
        .dis0_valid_i        (dis0_valid),
        .dis0_robid_i        (dis0_robid),
        .dis0_pc_i           (dis0_pc),
        .dis0_futype_i       (dis0_futype),
        .dis0_alu_op_i       (dis0_alu_op),
        .dis0_br_op_i        (dis0_br_op),
        .dis0_mem_op_i       (dis0_mem_op),
        .dis0_csr_op_i       (dis0_csr_op),
        .dis0_tlb_op_i       (dis0_tlb_op),
        .dis0_wb_src_op_i    (dis0_wb_src_op),
        .dis0_csr_num_i      (dis0_csr_num),
        .dis0_is_cacop_i     (dis0_is_cacop),
        .dis0_src0_ready_i   (dis0_src0_ready),
        .dis0_src0_val_i     (dis0_src0_val),
        .dis0_src0_robid_i     (dis0_src0_robid),
        .dis0_src1_ready_i     (dis0_src1_ready),
        .dis0_src1_val_i       (dis0_src1_val),
        .dis0_src1_robid_i     (dis0_src1_robid),
        .dis0_imm_i          (dis0_imm),
        .dis0_use_imm_i      (dis0_use_imm),
        .dis0_br_offs_i      (dis0_br_offs),
        .dis1_valid_i        (dis1_valid),
        .dis1_robid_i        (dis1_robid),
        .dis1_pc_i           (dis1_pc),
        .dis1_futype_i       (dis1_futype),
        .dis1_alu_op_i       (dis1_alu_op),
        .dis1_br_op_i        (dis1_br_op),
        .dis1_mem_op_i       (dis1_mem_op),
        .dis1_csr_op_i       (dis1_csr_op),
        .dis1_tlb_op_i       (dis1_tlb_op),
        .dis1_wb_src_op_i    (dis1_wb_src_op),
        .dis1_csr_num_i      (dis1_csr_num),
        .dis1_is_cacop_i     (dis1_is_cacop),
        .dis1_src0_ready_i   (dis1_src0_ready),
        .dis1_src0_val_i     (dis1_src0_val),
        .dis1_src0_robid_i     (dis1_src0_robid),
        .dis1_src1_ready_i     (dis1_src1_ready),
        .dis1_src1_val_i       (dis1_src1_val),
        .dis1_src1_robid_i     (dis1_src1_robid),
        .dis1_imm_i          (dis1_imm),
        .dis1_use_imm_i      (dis1_use_imm),
        .dis1_br_offs_i      (dis1_br_offs),
        .dispatch_ready_o    (dispatch_ready),
        .dis0_fire_o         (dis0_fire),
        .dis1_fire_o         (dis1_fire),
        .rob_raddr0_o        (dsp_rob_raddr0),
        .rob_rrdy0_i         (rob_rrdy0),
        .rob_rdata0_i        (rob_rdata0),
        .rob_raddr1_o        (dsp_rob_raddr1),
        .rob_rrdy1_i         (rob_rrdy1),
        .rob_rdata1_i        (rob_rdata1),
        .rob_raddr2_o        (dsp_rob_raddr2),
        .rob_rrdy2_i         (rob_rrdy2),
        .rob_rdata2_i        (rob_rdata2),
        .rob_raddr3_o        (dsp_rob_raddr3),
        .rob_rrdy3_i         (rob_rrdy3),
        .rob_rdata3_i        (rob_rdata3),
        .dis_rat_rbusy0_i    (dis_rat_rbusy0),
        .dis_rat_rbusy1_i    (dis_rat_rbusy1),
        .dis_rat_rbusy2_i    (dis_rat_rbusy2),
        .dis_rat_rbusy3_i    (dis_rat_rbusy3),
        .dis_arf_rdata0_i    (dis_arf_rdata0),
        .dis_arf_rdata1_i    (dis_arf_rdata1),
        .dis_arf_rdata2_i    (dis_arf_rdata2),
        .dis_arf_rdata3_i    (dis_arf_rdata3),
        .rs_alu0_can_accept_i(rsa0_can_accept),
        .rs_alu0_occupancy_i (rsa0_occupancy),
        .rs_alu0_push_valid_o(rsa0_push_valid),
        .rs_alu0_push_robid_o(rsa0_push_robid),
        .rs_alu0_push_pc_o   (rsa0_push_pc),
        .rs_alu0_push_alu_op_o(rsa0_push_alu_op),
        .rs_alu0_push_br_op_o(rsa0_push_br_op),
        .rs_alu0_push_src0_ready_o(rsa0_push_src0_ready),
        .rs_alu0_push_src0_val_o  (rsa0_push_src0_val),
        .rs_alu0_push_src0_robid_o(rsa0_push_src0_robid),
        .rs_alu0_push_src1_ready_o(rsa0_push_src1_ready),
        .rs_alu0_push_src1_val_o  (rsa0_push_src1_val),
        .rs_alu0_push_src1_robid_o(rsa0_push_src1_robid),
        .rs_alu0_push_imm_o  (rsa0_push_imm),
        .rs_alu0_push_use_imm_o(rsa0_push_use_imm),
        .rs_alu0_push_br_offs_o(rsa0_push_br_offs),
        .rs_alu1_can_accept_i(rsa1_can_accept),
        .rs_alu1_occupancy_i (rsa1_occupancy),
        .rs_alu1_push_valid_o(rsa1_push_valid),
        .rs_alu1_push_robid_o(rsa1_push_robid),
        .rs_alu1_push_pc_o   (rsa1_push_pc),
        .rs_alu1_push_alu_op_o(rsa1_push_alu_op),
        .rs_alu1_push_br_op_o(rsa1_push_br_op),
        .rs_alu1_push_src0_ready_o(rsa1_push_src0_ready),
        .rs_alu1_push_src0_val_o  (rsa1_push_src0_val),
        .rs_alu1_push_src0_robid_o(rsa1_push_src0_robid),
        .rs_alu1_push_src1_ready_o(rsa1_push_src1_ready),
        .rs_alu1_push_src1_val_o  (rsa1_push_src1_val),
        .rs_alu1_push_src1_robid_o(rsa1_push_src1_robid),
        .rs_alu1_push_imm_o  (rsa1_push_imm),
        .rs_alu1_push_use_imm_o(rsa1_push_use_imm),
        .rs_alu1_push_br_offs_o(rsa1_push_br_offs),
        .rs_mem_can_accept_i (rsm_can_accept),
        .rs_mem_push_valid_o (rsm_push_valid),
        .rs_mem_push_robid_o (rsm_push_robid),
        .rs_mem_push_pc_o    (rsm_push_pc),
        .rs_mem_push_mem_op_o(rsm_push_mem_op),
        .rs_mem_push_is_cacop_o(rsm_push_is_cacop),
        .rs_mem_push_src0_ready_o(rsm_push_src0_ready),
        .rs_mem_push_src0_val_o  (rsm_push_src0_val),
        .rs_mem_push_src0_robid_o(rsm_push_src0_robid),
        .rs_mem_push_src1_ready_o(rsm_push_src1_ready),
        .rs_mem_push_src1_val_o  (rsm_push_src1_val),
        .rs_mem_push_src1_robid_o(rsm_push_src1_robid),
        .rs_mem_push_imm_o   (rsm_push_imm),
        .rs_mdu_can_accept_i (rsd_can_accept),
        .rs_mdu_push_valid_o (rsd_push_valid),
        .rs_mdu_push_robid_o (rsd_push_robid),
        .rs_mdu_push_alu_op_o(rsd_push_alu_op),
        .rs_mdu_push_csr_op_o(rsd_push_csr_op),
        .rs_mdu_push_csr_num_o(rsd_push_csr_num),
        .rs_mdu_push_tlb_op_o(rsd_push_tlb_op),
        .rs_mdu_push_wb_src_op_o(rsd_push_wb_src_op),
        .rs_mdu_push_src0_ready_o(rsd_push_src0_ready),
        .rs_mdu_push_src0_val_o  (rsd_push_src0_val),
        .rs_mdu_push_src0_robid_o(rsd_push_src0_robid),
        .rs_mdu_push_src1_ready_o(rsd_push_src1_ready),
        .rs_mdu_push_src1_val_o  (rsd_push_src1_val),
        .rs_mdu_push_src1_robid_o(rsd_push_src1_robid)
    );

//--------------------------------------------------
// rs_alu x2：ALU 保留站（乱序发射）
//--------------------------------------------------
    rs_alu u_rs_alu0(
        .clk            (clk),
        .reset          (reset),
        .flush_i        (flush),
        .push_valid_i   (rsa0_push_valid),
        .push_robid_i   (rsa0_push_robid),
        .push_pc_i      (rsa0_push_pc),
        .push_alu_op_i  (rsa0_push_alu_op),
        .push_br_op_i   (rsa0_push_br_op),
        .push_src0_ready_i(rsa0_push_src0_ready),
        .push_src0_val_i  (rsa0_push_src0_val),
        .push_src0_robid_i(rsa0_push_src0_robid),
        .push_src1_ready_i(rsa0_push_src1_ready),
        .push_src1_val_i  (rsa0_push_src1_val),
        .push_src1_robid_i(rsa0_push_src1_robid),
        .push_imm_i     (rsa0_push_imm),
        .push_use_imm_i (rsa0_push_use_imm),
        .push_br_offs_i (rsa0_push_br_offs),
        .can_accept_o   (rsa0_can_accept),
        .occupancy_o    (rsa0_occupancy),
        .wb0_valid_i    (alu0_wb_valid),
        .wb0_robid_i    (alu0_wb_robid),
        .wb0_data_i     (alu0_wb_data),
        .wb1_valid_i    (alu1_wb_valid),
        .wb1_robid_i    (alu1_wb_robid),
        .wb1_data_i     (alu1_wb_data),
        .wb2_valid_i    (mem_wb_valid),
        .wb2_robid_i    (mem_wb_robid),
        .wb2_data_i     (mem_wb_data),
        .wb3_valid_i    (mdu_wb_valid),
        .wb3_robid_i    (mdu_wb_robid),
        .wb3_data_i     (mdu_wb_data),
        .early0_valid_i (alu0_early_valid),
        .early0_robid_i (alu0_early_robid),
        .early1_valid_i (alu1_early_valid),
        .early1_robid_i (alu1_early_robid),
        .early2_valid_i (lsu_early_valid),
        .early2_robid_i (lsu_early_robid),
        .issue_valid_o  (rsa0_issue_valid),
        .issue_robid_o  (rsa0_issue_robid),
        .issue_pc_o     (rsa0_issue_pc),
        .issue_alu_op_o (rsa0_issue_alu_op),
        .issue_br_op_o  (rsa0_issue_br_op),
        .issue_src0_o   (rsa0_issue_src0),
        .issue_src1_o   (rsa0_issue_src1),
        .issue_imm_o    (rsa0_issue_imm),
        .issue_use_imm_o(rsa0_issue_use_imm),
        .issue_br_offs_o(rsa0_issue_br_offs)
    );

    rs_alu u_rs_alu1(
        .clk            (clk),
        .reset          (reset),
        .flush_i        (flush),
        .push_valid_i   (rsa1_push_valid),
        .push_robid_i   (rsa1_push_robid),
        .push_pc_i      (rsa1_push_pc),
        .push_alu_op_i  (rsa1_push_alu_op),
        .push_br_op_i   (rsa1_push_br_op),
        .push_src0_ready_i(rsa1_push_src0_ready),
        .push_src0_val_i  (rsa1_push_src0_val),
        .push_src0_robid_i(rsa1_push_src0_robid),
        .push_src1_ready_i(rsa1_push_src1_ready),
        .push_src1_val_i  (rsa1_push_src1_val),
        .push_src1_robid_i(rsa1_push_src1_robid),
        .push_imm_i     (rsa1_push_imm),
        .push_use_imm_i (rsa1_push_use_imm),
        .push_br_offs_i (rsa1_push_br_offs),
        .can_accept_o   (rsa1_can_accept),
        .occupancy_o    (rsa1_occupancy),
        .wb0_valid_i    (alu0_wb_valid),
        .wb0_robid_i    (alu0_wb_robid),
        .wb0_data_i     (alu0_wb_data),
        .wb1_valid_i    (alu1_wb_valid),
        .wb1_robid_i    (alu1_wb_robid),
        .wb1_data_i     (alu1_wb_data),
        .wb2_valid_i    (mem_wb_valid),
        .wb2_robid_i    (mem_wb_robid),
        .wb2_data_i     (mem_wb_data),
        .wb3_valid_i    (mdu_wb_valid),
        .wb3_robid_i    (mdu_wb_robid),
        .wb3_data_i     (mdu_wb_data),
        .early0_valid_i (alu0_early_valid),
        .early0_robid_i (alu0_early_robid),
        .early1_valid_i (alu1_early_valid),
        .early1_robid_i (alu1_early_robid),
        .early2_valid_i (lsu_early_valid),
        .early2_robid_i (lsu_early_robid),
        .issue_valid_o  (rsa1_issue_valid),
        .issue_robid_o  (rsa1_issue_robid),
        .issue_pc_o     (rsa1_issue_pc),
        .issue_alu_op_o (rsa1_issue_alu_op),
        .issue_br_op_o  (rsa1_issue_br_op),
        .issue_src0_o   (rsa1_issue_src0),
        .issue_src1_o   (rsa1_issue_src1),
        .issue_imm_o    (rsa1_issue_imm),
        .issue_use_imm_o(rsa1_issue_use_imm),
        .issue_br_offs_o(rsa1_issue_br_offs)
    );

//--------------------------------------------------
// rs_mem / rs_mdu：顺序发射保留站
//--------------------------------------------------
    rs_mem u_rs_mem(
        .clk            (clk),
        .reset          (reset),
        .flush_i        (flush),
        .push_valid_i   (rsm_push_valid),
        .push_robid_i   (rsm_push_robid),
        .push_pc_i      (rsm_push_pc),
        .push_mem_op_i  (rsm_push_mem_op),
        .push_is_cacop_i(rsm_push_is_cacop),
        .push_src0_ready_i(rsm_push_src0_ready),
        .push_src0_val_i  (rsm_push_src0_val),
        .push_src0_robid_i(rsm_push_src0_robid),
        .push_src1_ready_i(rsm_push_src1_ready),
        .push_src1_val_i  (rsm_push_src1_val),
        .push_src1_robid_i(rsm_push_src1_robid),
        .push_imm_i     (rsm_push_imm),
        .can_accept_o   (rsm_can_accept),
        .occupancy_o    (rsm_occupancy),
        .wb0_valid_i    (alu0_wb_valid),
        .wb0_robid_i    (alu0_wb_robid),
        .wb0_data_i     (alu0_wb_data),
        .wb1_valid_i    (alu1_wb_valid),
        .wb1_robid_i    (alu1_wb_robid),
        .wb1_data_i     (alu1_wb_data),
        .wb2_valid_i    (mem_wb_valid),
        .wb2_robid_i    (mem_wb_robid),
        .wb2_data_i     (mem_wb_data),
        .wb3_valid_i    (mdu_wb_valid),
        .wb3_robid_i    (mdu_wb_robid),
        .wb3_data_i     (mdu_wb_data),
        .early0_valid_i (alu0_early_valid),
        .early0_robid_i (alu0_early_robid),
        .early1_valid_i (alu1_early_valid),
        .early1_robid_i (alu1_early_robid),
        .early2_valid_i (lsu_early_valid),
        .early2_robid_i (lsu_early_robid),
        .issue_valid_o  (rsm_issue_valid),
        .issue_robid_o  (rsm_issue_robid),
        .issue_pc_o     (rsm_issue_pc),
        .issue_mem_op_o (rsm_issue_mem_op),
        .issue_is_cacop_o(rsm_issue_is_cacop),
        .issue_base_o   (rsm_issue_base),
        .issue_wdata_o  (rsm_issue_wdata),
        .issue_imm_o    (rsm_issue_imm),
        .lsu_ready_i    (lsu_ready)
    );

    rs_mdu u_rs_mdu(
        .clk            (clk),
        .reset          (reset),
        .flush_i        (flush),
        .push_valid_i   (rsd_push_valid),
        .push_robid_i   (rsd_push_robid),
        .push_alu_op_i  (rsd_push_alu_op),
        .push_csr_op_i  (rsd_push_csr_op),
        .push_csr_num_i (rsd_push_csr_num),
        .push_tlb_op_i  (rsd_push_tlb_op),
        .push_wb_src_op_i(rsd_push_wb_src_op),
        .push_src0_ready_i(rsd_push_src0_ready),
        .push_src0_val_i  (rsd_push_src0_val),
        .push_src0_robid_i(rsd_push_src0_robid),
        .push_src1_ready_i(rsd_push_src1_ready),
        .push_src1_val_i  (rsd_push_src1_val),
        .push_src1_robid_i(rsd_push_src1_robid),
        .can_accept_o   (rsd_can_accept),
        .occupancy_o    (rsd_occupancy),
        .wb0_valid_i    (alu0_wb_valid),
        .wb0_robid_i    (alu0_wb_robid),
        .wb0_data_i     (alu0_wb_data),
        .wb1_valid_i    (alu1_wb_valid),
        .wb1_robid_i    (alu1_wb_robid),
        .wb1_data_i     (alu1_wb_data),
        .wb2_valid_i    (mem_wb_valid),
        .wb2_robid_i    (mem_wb_robid),
        .wb2_data_i     (mem_wb_data),
        .wb3_valid_i    (mdu_wb_valid),
        .wb3_robid_i    (mdu_wb_robid),
        .wb3_data_i     (mdu_wb_data),
        .early0_valid_i (alu0_early_valid),
        .early0_robid_i (alu0_early_robid),
        .early1_valid_i (alu1_early_valid),
        .early1_robid_i (alu1_early_robid),
        .early2_valid_i (lsu_early_valid),
        .early2_robid_i (lsu_early_robid),
        .issue_valid_o  (rsd_issue_valid),
        .issue_robid_o  (rsd_issue_robid),
        .issue_alu_op_o (rsd_issue_alu_op),
        .issue_csr_op_o (rsd_issue_csr_op),
        .issue_csr_num_o(rsd_issue_csr_num),
        .issue_tlb_op_o (rsd_issue_tlb_op),
        .issue_wb_src_op_o(rsd_issue_wb_src_op),
        .issue_src0_o   (rsd_issue_src0),
        .issue_src1_o   (rsd_issue_src1),
        .mdu_ready_i    (mdu_ready)
    );

//--------------------------------------------------
// fu_alu x2：单周期 ALU 执行单元（含分支判断）
//--------------------------------------------------
    fu_alu u_fu_alu0(
        .clk             (clk),
        .reset           (reset),
        .flush_i         (flush),
        .issue_valid_i   (rsa0_issue_valid),
        .issue_robid_i   (rsa0_issue_robid),
        .issue_pc_i      (rsa0_issue_pc),
        .issue_alu_op_i  (rsa0_issue_alu_op),
        .issue_br_op_i   (rsa0_issue_br_op),
        .issue_src0_i    (rsa0_issue_src0),
        .issue_src1_i    (rsa0_issue_src1),
        .issue_imm_i     (rsa0_issue_imm),
        .issue_use_imm_i (rsa0_issue_use_imm),
        .issue_br_offs_i (rsa0_issue_br_offs),
        .wb_valid_o      (alu0_wb_valid),
        .wb_robid_o      (alu0_wb_robid),
        .wb_data_o       (alu0_wb_data),
        .wb_br_taken_o   (alu0_wb_br_taken),
        .wb_br_target_o  (alu0_wb_br_target),
        .early_wakeup_valid_o(alu0_early_valid),
        .early_wakeup_robid_o(alu0_early_robid),
        .ex_redirect_valid_o (alu0_exred_valid),
        .ex_redirect_pc_o    (alu0_exred_pc)
    );

    fu_alu u_fu_alu1(
        .clk             (clk),
        .reset           (reset),
        .flush_i         (flush),
        .issue_valid_i   (rsa1_issue_valid),
        .issue_robid_i   (rsa1_issue_robid),
        .issue_pc_i      (rsa1_issue_pc),
        .issue_alu_op_i  (rsa1_issue_alu_op),
        .issue_br_op_i   (rsa1_issue_br_op),
        .issue_src0_i    (rsa1_issue_src0),
        .issue_src1_i    (rsa1_issue_src1),
        .issue_imm_i     (rsa1_issue_imm),
        .issue_use_imm_i (rsa1_issue_use_imm),
        .issue_br_offs_i (rsa1_issue_br_offs),
        .wb_valid_o      (alu1_wb_valid),
        .wb_robid_o      (alu1_wb_robid),
        .wb_data_o       (alu1_wb_data),
        .wb_br_taken_o   (alu1_wb_br_taken),
        .wb_br_target_o  (alu1_wb_br_target),
        .early_wakeup_valid_o(alu1_early_valid),
        .early_wakeup_robid_o(alu1_early_robid),
        .ex_redirect_valid_o (alu1_exred_valid),
        .ex_redirect_pc_o    (alu1_exred_pc)
    );

//--------------------------------------------------
// fu_mdu：乘除/CSR读/rdcnt/TLB杂项执行单元（内含 mul/div）
//--------------------------------------------------
    wire [13:0] mdu_csr_raddr;
    wire [31:0] csr_rvalue;
    wire [63:0] timer_64;
    wire [31:0] csr_tid;

    fu_mdu u_fu_mdu(
        .clk             (clk),
        .reset           (reset),
        .flush_i         (flush),
        .issue_valid_i   (rsd_issue_valid),
        .issue_robid_i   (rsd_issue_robid),
        .issue_alu_op_i  (rsd_issue_alu_op),
        .issue_csr_op_i  (rsd_issue_csr_op),
        .issue_csr_num_i (rsd_issue_csr_num),
        .issue_tlb_op_i  (rsd_issue_tlb_op),
        .issue_wb_src_op_i(rsd_issue_wb_src_op),
        .issue_src0_i    (rsd_issue_src0),
        .issue_src1_i    (rsd_issue_src1),
        .mdu_ready_o     (mdu_ready),
        .csr_raddr_o     (mdu_csr_raddr),
        .csr_rdata_i     (csr_rvalue),
        .timer_64_i      (timer_64),
        .csr_tid_i       (csr_tid),
        .wb_valid_o      (mdu_wb_valid),
        .wb_robid_o      (mdu_wb_robid),
        .wb_data_o       (mdu_wb_data),
        .wb_data2_o      (mdu_wb_data2)
    );

//--------------------------------------------------
// lsu：访存单元（AGU+DC 两级流水）
//--------------------------------------------------
    // LSU <-> MMU D 通道
    wire        lsu_mmu_req;
    wire [31:0] lsu_mmu_vaddr;
    wire        lsu_mmu_is_store;
    wire [31:0] mmu_d_paddr;
    wire [1:0]  mmu_d_mat;
    wire [`TLB_EX_NUM-1:0] mmu_d_tlb_ex;
    wire        mmu_d_adem;
    // LSU <-> DCache load 口（含非阻塞 miss 扩展通道）
    wire        lsu_dc_req;
    wire [31:0] lsu_dc_vaddr;
    wire [31:0] lsu_dc_paddr;
    wire [2:0]  lsu_dc_size;
    wire        lsu_dc_uncached;
    wire        dc_lsu_addr_ok;
    wire        dc_lsu_data_ok;
    wire [31:0] dc_lsu_rdata;
    wire        lsu_dc_cancel;
    wire        dc_lsu_miss;         // load 移入 MSHR（hit-under-miss）
    wire        dc_lsu_mshr_ok;      // MSHR 重填数据返回（CWF-lite 提前回）
    wire [31:0] dc_lsu_mshr_rdata;
    // LSU <-> store buffer 前递查询
    wire [31:0] lsu_sb_qpaddr;
    wire        lsu_sb_quncached;    // 查询来自 uncached load（设备写序保证）
    wire        sb_q_hit;
    wire [31:0] sb_q_data;
    wire        sb_q_partial;
    // ROB 队头（uncached load 许可 / store 序）
    // LSU 约定 head_robid 编码：MSB=槽0 是否仍未提交，低位=head 对指针。
    // rob.head_robid0_o 本身是真 robid ({0,head})，供 commit/RAT 用，不能改；
    // 这里用 cmt0_valid 拼装后只喂给 LSU。
    wire [`ROB_W-1:0] rob_head_robid0;
    wire        rob_cmt0_valid,    rob_cmt1_valid;
    wire [`ROB_W-1:0] lsu_rob_head_enc =
        {rob_cmt0_valid, rob_head_robid0[`ROB_PAIR_W-1:0]};
    wire        lsu_unc_inflight;

    lsu u_lsu(
        .clk              (clk),
        .reset            (reset),
        .flush_i          (flush),
        .issue_valid_i    (rsm_issue_valid),
        .issue_robid_i    (rsm_issue_robid),
        .issue_pc_i       (rsm_issue_pc),
        .issue_mem_op_i   (rsm_issue_mem_op),
        .issue_is_cacop_i (rsm_issue_is_cacop),
        .issue_base_i     (rsm_issue_base),
        .issue_wdata_i    (rsm_issue_wdata),
        .issue_imm_i      (rsm_issue_imm),
        .lsu_ready_o      (lsu_ready),
        .mmu_d_req_o      (lsu_mmu_req),
        .mmu_d_vaddr_o    (lsu_mmu_vaddr),
        .mmu_d_is_store_o (lsu_mmu_is_store),
        .mmu_d_paddr_i    (mmu_d_paddr),
        .mmu_d_mat_i      (mmu_d_mat),
        .mmu_d_tlb_ex_i   (mmu_d_tlb_ex),
        .mmu_d_excp_adem_i(mmu_d_adem),
        .dc_req_o         (lsu_dc_req),
        .dc_vaddr_o       (lsu_dc_vaddr),
        .dc_paddr_o       (lsu_dc_paddr),
        .dc_size_o        (lsu_dc_size),
        .dc_uncached_o    (lsu_dc_uncached),
        .dc_addr_ok_i     (dc_lsu_addr_ok),
        .dc_data_ok_i     (dc_lsu_data_ok),
        .dc_rdata_i       (dc_lsu_rdata),
        .dc_cancel_o      (lsu_dc_cancel),
        .dc_miss_i        (dc_lsu_miss),
        .dc_mshr_data_ok_i(dc_lsu_mshr_ok),
        .dc_mshr_rdata_i  (dc_lsu_mshr_rdata),
        .sb_query_paddr_o (lsu_sb_qpaddr),
        .sb_query_uncached_o(lsu_sb_quncached),
        .sb_query_hit_i   (sb_q_hit),
        .sb_query_data_i  (sb_q_data),
        .sb_query_partial_i(sb_q_partial),
        .rob_head_robid_i (lsu_rob_head_enc),
        .rob_head_valid_i (~rob_empty),
        .uncached_ld_inflight_o(lsu_unc_inflight),
        .wb_valid_o       (mem_wb_valid),
        .wb_robid_o       (mem_wb_robid),
        .wb_data_o        (mem_wb_data),
        .wb_paddr_o       (mem_wb_paddr),
        .wb_vaddr_o       (mem_wb_vaddr),
        .wb_wstrb_o       (mem_wb_wstrb),
        .wb_size_o        (mem_wb_size),
        .wb_uncached_o    (mem_wb_uncached),
        .wb_excp_o        (mem_wb_excp),
        .early_wakeup_valid_o(lsu_early_valid),
        .early_wakeup_robid_o(lsu_early_robid)
    );

//--------------------------------------------------
// store_buffer：提交后写缓冲（冲刷不清空！里面全是已提交 store）
//--------------------------------------------------
    // commit -> SB 入队
    wire        cmt_sb_push_valid;
    wire [31:0] cmt_sb_push_paddr;
    wire [31:0] cmt_sb_push_data;
    wire [3:0]  cmt_sb_push_wstrb;
    wire [2:0]  cmt_sb_push_size;
    wire        cmt_sb_push_uncached;
    wire        sb_full, sb_empty;
    // SB -> DCache store 写出
    wire        sb_dc_wr_req;
    wire [31:0] sb_dc_wr_paddr;
    wire [31:0] sb_dc_wr_data;
    wire [3:0]  sb_dc_wr_strb;
    wire [2:0]  sb_dc_wr_size;
    wire        sb_dc_wr_uncached;
    wire        dc_sb_addr_ok;
    wire        dc_sb_done;

    store_buffer u_store_buffer(
        .clk             (clk),
        .reset           (reset),
        .push_valid_i    (cmt_sb_push_valid),
        .push_paddr_i    (cmt_sb_push_paddr),
        .push_data_i     (cmt_sb_push_data),
        .push_wstrb_i    (cmt_sb_push_wstrb),
        .push_size_i     (cmt_sb_push_size),
        .push_uncached_i (cmt_sb_push_uncached),
        .sb_full_o       (sb_full),
        .sb_empty_o      (sb_empty),
        .dc_wr_req_o     (sb_dc_wr_req),
        .dc_wr_paddr_o   (sb_dc_wr_paddr),
        .dc_wr_data_o    (sb_dc_wr_data),
        .dc_wr_strb_o    (sb_dc_wr_strb),
        .dc_wr_size_o    (sb_dc_wr_size),
        .dc_wr_uncached_o(sb_dc_wr_uncached),
        .dc_wr_addr_ok_i (dc_sb_addr_ok),
        .dc_wr_done_i    (dc_sb_done),
        .query_paddr_i   (lsu_sb_qpaddr),
        .query_uncached_i(lsu_sb_quncached),
        .query_hit_o     (sb_q_hit),
        .query_data_o    (sb_q_data),
        .query_partial_o (sb_q_partial)
    );



//--------------------------------------------------
// ROB / commit 信号声明
//--------------------------------------------------
    // ROB 提交口（队头一对；cmt*_valid 已在 LSU 前声明）
    wire        rob_cmt0_complete, rob_cmt1_complete;
    wire [31:0] rob_cmt0_pc,       rob_cmt1_pc;
    wire [31:0] rob_cmt0_inst,     rob_cmt1_inst;
    wire        rob_cmt0_rf_we,    rob_cmt1_rf_we;
    wire [4:0]  rob_cmt0_rd,       rob_cmt1_rd;
    wire [31:0] rob_cmt0_result,   rob_cmt1_result;
    wire [31:0] rob_cmt0_result2,  rob_cmt1_result2;
    wire        rob_cmt0_is_load,  rob_cmt1_is_load;
    wire        rob_cmt0_is_store, rob_cmt1_is_store;
    wire [31:0] rob_cmt0_paddr,    rob_cmt1_paddr;
    wire [31:0] rob_cmt0_vaddr,    rob_cmt1_vaddr;
    wire [3:0]  rob_cmt0_wstrb,    rob_cmt1_wstrb;
    wire [2:0]  rob_cmt0_size,     rob_cmt1_size;
    wire        rob_cmt0_uncached, rob_cmt1_uncached;
    wire        rob_cmt0_is_branch,rob_cmt1_is_branch;
    wire [`BR_TYPE_W-1:0] rob_cmt0_br_type, rob_cmt1_br_type;
    wire        rob_cmt0_pred_taken, rob_cmt1_pred_taken;
    wire        rob_cmt0_br_taken,   rob_cmt1_br_taken;
    wire [31:0] rob_cmt0_br_target,  rob_cmt1_br_target;
    wire        rob_cmt0_is_last,  rob_cmt1_is_last;
    wire [`FTQ_W-1:0] rob_cmt0_ftq_id, rob_cmt1_ftq_id;
    wire [`PRIV_NUM-1:0] rob_cmt0_priv_vec, rob_cmt1_priv_vec;
    wire [13:0] rob_cmt0_csr_num,  rob_cmt1_csr_num;
    wire [`TLB_OP_NUM-1:0] rob_cmt0_tlb_op, rob_cmt1_tlb_op;
    wire [4:0]  rob_cmt0_cacop_code, rob_cmt1_cacop_code;
    wire [`EXCP_NUM-1:0] rob_cmt0_excp, rob_cmt1_excp;
    // commit -> ROB 推进
    wire        cmt_rob_pop, cmt_rob_clear0, cmt_rob_clear1;
    // commit -> CSR 提交
    wire        cmt_csr_valid;
    wire [31:0] cmt_csr_pc;
    wire        cmt_csr_ex;
    wire        cmt_csr_ertn;
    wire [31:0] cmt_csr_vaddr;
    wire        cmt_excp_int, cmt_excp_adef, cmt_excp_adem, cmt_excp_ipe;
    wire        cmt_excp_ale, cmt_excp_sys, cmt_excp_brk, cmt_excp_ine;
    wire [`TLB_EX_NUM-1:0] cmt_excp_tlb_vec;
    wire        cmt_csr_we;
    wire [13:0] cmt_csr_wnum;
    wire [31:0] cmt_csr_wmask;
    wire [31:0] cmt_csr_wvalue;
    wire        cmt_ll_set, cmt_sc_set;
    wire [27:0] cmt_lladdr;
    // commit -> TLB 维护
    wire [`TLB_OP_NUM-1:0] cmt_tlb_op;
    wire [9:0]  cmt_invtlb_asid;
    wire [18:0] cmt_invtlb_vpn;
    // commit -> D$ cacop
    wire        cmt_dcacop_valid;
    wire [1:0]  cmt_dcacop_op;
    wire [31:0] cmt_dcacop_addr;
    // CSR -> commit
    wire [31:0] csr_next_pc;
    wire [1:0]  csr_redirect;
    // commit 调试/difftest 口
    wire        cmt_dbg0_valid, cmt_dbg1_valid;
    wire [31:0] cmt_dbg0_pc,    cmt_dbg1_pc;
    wire [3:0]  cmt_dbg0_wen,   cmt_dbg1_wen;
    wire [4:0]  cmt_dbg0_wnum,  cmt_dbg1_wnum;
    wire [31:0] cmt_dbg0_wdata, cmt_dbg1_wdata;
    wire [31:0] cmt_dbg0_inst,  cmt_dbg1_inst;

//--------------------------------------------------
// rob：重排序缓冲（奇偶双体环形队列，队列式重命名核心）
//--------------------------------------------------
    rob u_rob(
        .clk              (clk),
        .reset            (reset),
        .flush_i          (flush),
        .alloc_en_i       (rn_rob_alloc_en),
        .rob_tail_o       (rob_tail),
        .rob_full_o       (rob_full),
        .rob_empty_o      (rob_empty),
        .a0_valid_i       (rn_a0_valid),
        .a0_pc_i          (rn_a0_pc),
        .a0_inst_i        (rn_a0_inst),
        .a0_rf_we_i       (rn_a0_rf_we),
        .a0_rd_i          (rn_a0_rd),
        .a0_futype_i      (rn_a0_futype),
        .a0_is_load_i     (rn_a0_is_load),
        .a0_is_store_i    (rn_a0_is_store),
        .a0_is_branch_i   (rn_a0_is_branch),
        .a0_br_type_i     (rn_a0_br_type),
        .a0_pred_taken_i  (rn_a0_pred_taken),
        .a0_is_last_i     (rn_a0_is_last),
        .a0_ftq_id_i      (rn_a0_ftq_id),
        .a0_priv_vec_i    (rn_a0_priv_vec),
        .a0_csr_num_i     (rn_a0_csr_num),
        .a0_tlb_op_i      (rn_a0_tlb_op),
        .a0_cacop_code_i  (rn_a0_cacop_code),
        .a0_excp_i        (rn_a0_excp),
        .a0_is_nop_i      (rn_a0_is_nop),
        .a1_valid_i       (rn_a1_valid),
        .a1_pc_i          (rn_a1_pc),
        .a1_inst_i        (rn_a1_inst),
        .a1_rf_we_i       (rn_a1_rf_we),
        .a1_rd_i          (rn_a1_rd),
        .a1_futype_i      (rn_a1_futype),
        .a1_is_load_i     (rn_a1_is_load),
        .a1_is_store_i    (rn_a1_is_store),
        .a1_is_branch_i   (rn_a1_is_branch),
        .a1_br_type_i     (rn_a1_br_type),
        .a1_pred_taken_i  (rn_a1_pred_taken),
        .a1_is_last_i     (rn_a1_is_last),
        .a1_ftq_id_i      (rn_a1_ftq_id),
        .a1_priv_vec_i    (rn_a1_priv_vec),
        .a1_csr_num_i     (rn_a1_csr_num),
        .a1_tlb_op_i      (rn_a1_tlb_op),
        .a1_cacop_code_i  (rn_a1_cacop_code),
        .a1_excp_i        (rn_a1_excp),
        .a1_is_nop_i      (rn_a1_is_nop),
        .raddr0_i         (dsp_rob_raddr0),
        .rrdy0_o          (rob_rrdy0),
        .rdata0_o         (rob_rdata0),
        .raddr1_i         (dsp_rob_raddr1),
        .rrdy1_o          (rob_rrdy1),
        .rdata1_o         (rob_rdata1),
        .raddr2_i         (dsp_rob_raddr2),
        .rrdy2_o          (rob_rrdy2),
        .rdata2_o         (rob_rdata2),
        .raddr3_i         (dsp_rob_raddr3),
        .rrdy3_o          (rob_rrdy3),
        .rdata3_o         (rob_rdata3),
        .alu0_wb_valid_i  (alu0_wb_valid),
        .alu0_wb_robid_i  (alu0_wb_robid),
        .alu0_wb_data_i   (alu0_wb_data),
        .alu0_wb_br_taken_i(alu0_wb_br_taken),
        .alu0_wb_br_target_i(alu0_wb_br_target),
        .alu1_wb_valid_i  (alu1_wb_valid),
        .alu1_wb_robid_i  (alu1_wb_robid),
        .alu1_wb_data_i   (alu1_wb_data),
        .alu1_wb_br_taken_i(alu1_wb_br_taken),
        .alu1_wb_br_target_i(alu1_wb_br_target),
        .mem_wb_valid_i   (mem_wb_valid),
        .mem_wb_robid_i   (mem_wb_robid),
        .mem_wb_data_i    (mem_wb_data),
        .mem_wb_paddr_i   (mem_wb_paddr),
        .mem_wb_vaddr_i   (mem_wb_vaddr),
        .mem_wb_wstrb_i   (mem_wb_wstrb),
        .mem_wb_size_i    (mem_wb_size),
        .mem_wb_uncached_i(mem_wb_uncached),
        .mem_wb_excp_i    (mem_wb_excp),
        .mdu_wb_valid_i   (mdu_wb_valid),
        .mdu_wb_robid_i   (mdu_wb_robid),
        .mdu_wb_data_i    (mdu_wb_data),
        .mdu_wb_data2_i   (mdu_wb_data2),
        .head_robid0_o    (rob_head_robid0),
        .cmt0_valid_o     (rob_cmt0_valid),
        .cmt0_complete_o  (rob_cmt0_complete),
        .cmt0_pc_o        (rob_cmt0_pc),
        .cmt0_inst_o      (rob_cmt0_inst),
        .cmt0_rf_we_o     (rob_cmt0_rf_we),
        .cmt0_rd_o        (rob_cmt0_rd),
        .cmt0_result_o    (rob_cmt0_result),
        .cmt0_result2_o   (rob_cmt0_result2),
        .cmt0_is_load_o   (rob_cmt0_is_load),
        .cmt0_is_store_o  (rob_cmt0_is_store),
        .cmt0_paddr_o     (rob_cmt0_paddr),
        .cmt0_vaddr_o     (rob_cmt0_vaddr),
        .cmt0_wstrb_o     (rob_cmt0_wstrb),
        .cmt0_size_o      (rob_cmt0_size),
        .cmt0_uncached_o  (rob_cmt0_uncached),
        .cmt0_is_branch_o (rob_cmt0_is_branch),
        .cmt0_br_type_o   (rob_cmt0_br_type),
        .cmt0_pred_taken_o(rob_cmt0_pred_taken),
        .cmt0_br_taken_o  (rob_cmt0_br_taken),
        .cmt0_br_target_o (rob_cmt0_br_target),
        .cmt0_is_last_o   (rob_cmt0_is_last),
        .cmt0_ftq_id_o    (rob_cmt0_ftq_id),
        .cmt0_priv_vec_o  (rob_cmt0_priv_vec),
        .cmt0_csr_num_o   (rob_cmt0_csr_num),
        .cmt0_tlb_op_o    (rob_cmt0_tlb_op),
        .cmt0_cacop_code_o(rob_cmt0_cacop_code),
        .cmt0_excp_o      (rob_cmt0_excp),
        .cmt1_valid_o     (rob_cmt1_valid),
        .cmt1_complete_o  (rob_cmt1_complete),
        .cmt1_pc_o        (rob_cmt1_pc),
        .cmt1_inst_o      (rob_cmt1_inst),
        .cmt1_rf_we_o     (rob_cmt1_rf_we),
        .cmt1_rd_o        (rob_cmt1_rd),
        .cmt1_result_o    (rob_cmt1_result),
        .cmt1_result2_o   (rob_cmt1_result2),
        .cmt1_is_load_o   (rob_cmt1_is_load),
        .cmt1_is_store_o  (rob_cmt1_is_store),
        .cmt1_paddr_o     (rob_cmt1_paddr),
        .cmt1_vaddr_o     (rob_cmt1_vaddr),
        .cmt1_wstrb_o     (rob_cmt1_wstrb),
        .cmt1_size_o      (rob_cmt1_size),
        .cmt1_uncached_o  (rob_cmt1_uncached),
        .cmt1_is_branch_o (rob_cmt1_is_branch),
        .cmt1_br_type_o   (rob_cmt1_br_type),
        .cmt1_pred_taken_o(rob_cmt1_pred_taken),
        .cmt1_br_taken_o  (rob_cmt1_br_taken),
        .cmt1_br_target_o (rob_cmt1_br_target),
        .cmt1_is_last_o   (rob_cmt1_is_last),
        .cmt1_ftq_id_o    (rob_cmt1_ftq_id),
        .cmt1_priv_vec_o  (rob_cmt1_priv_vec),
        .cmt1_csr_num_o   (rob_cmt1_csr_num),
        .cmt1_tlb_op_o    (rob_cmt1_tlb_op),
        .cmt1_cacop_code_o(rob_cmt1_cacop_code),
        .cmt1_excp_o      (rob_cmt1_excp),
        .cmt_pop_i        (cmt_rob_pop),
        .cmt_clear0_i     (cmt_rob_clear0),
        .cmt_clear1_i     (cmt_rob_clear1)
    );

//--------------------------------------------------
// commit：提交级（双提交仲裁 + 异常/特权/误预测处理）
//--------------------------------------------------
    commit u_commit(
        .clk               (clk),
        .reset             (reset),
        .flush_pending_i   (flush),   // ctrl 寄存器版 flush:广播拍闸住退休,防误提交
        .head_robid0_i     (rob_head_robid0),
        .cmt0_valid_i      (rob_cmt0_valid),
        .cmt0_complete_i   (rob_cmt0_complete),
        .cmt0_pc_i         (rob_cmt0_pc),
        .cmt0_inst_i       (rob_cmt0_inst),
        .cmt0_rf_we_i      (rob_cmt0_rf_we),
        .cmt0_rd_i         (rob_cmt0_rd),
        .cmt0_result_i     (rob_cmt0_result),
        .cmt0_result2_i    (rob_cmt0_result2),
        .cmt0_is_load_i    (rob_cmt0_is_load),
        .cmt0_is_store_i   (rob_cmt0_is_store),
        .cmt0_paddr_i      (rob_cmt0_paddr),
        .cmt0_vaddr_i      (rob_cmt0_vaddr),
        .cmt0_wstrb_i      (rob_cmt0_wstrb),
        .cmt0_size_i       (rob_cmt0_size),
        .cmt0_uncached_i   (rob_cmt0_uncached),
        .cmt0_is_branch_i  (rob_cmt0_is_branch),
        .cmt0_br_type_i    (rob_cmt0_br_type),
        .cmt0_pred_taken_i (rob_cmt0_pred_taken),
        .cmt0_br_taken_i   (rob_cmt0_br_taken),
        .cmt0_br_target_i  (rob_cmt0_br_target),
        .cmt0_is_last_i    (rob_cmt0_is_last),
        .cmt0_ftq_id_i     (rob_cmt0_ftq_id),
        .cmt0_priv_vec_i   (rob_cmt0_priv_vec),
        .cmt0_csr_num_i    (rob_cmt0_csr_num),
        .cmt0_tlb_op_i     (rob_cmt0_tlb_op),
        .cmt0_cacop_code_i (rob_cmt0_cacop_code),
        .cmt0_excp_i       (rob_cmt0_excp),
        .cmt1_valid_i      (rob_cmt1_valid),
        .cmt1_complete_i   (rob_cmt1_complete),
        .cmt1_pc_i         (rob_cmt1_pc),
        .cmt1_inst_i       (rob_cmt1_inst),
        .cmt1_rf_we_i      (rob_cmt1_rf_we),
        .cmt1_rd_i         (rob_cmt1_rd),
        .cmt1_result_i     (rob_cmt1_result),
        .cmt1_result2_i    (rob_cmt1_result2),
        .cmt1_is_load_i    (rob_cmt1_is_load),
        .cmt1_is_store_i   (rob_cmt1_is_store),
        .cmt1_paddr_i      (rob_cmt1_paddr),
        .cmt1_vaddr_i      (rob_cmt1_vaddr),
        .cmt1_wstrb_i      (rob_cmt1_wstrb),
        .cmt1_size_i       (rob_cmt1_size),
        .cmt1_uncached_i   (rob_cmt1_uncached),
        .cmt1_is_branch_i  (rob_cmt1_is_branch),
        .cmt1_br_type_i    (rob_cmt1_br_type),
        .cmt1_pred_taken_i (rob_cmt1_pred_taken),
        .cmt1_br_taken_i   (rob_cmt1_br_taken),
        .cmt1_br_target_i  (rob_cmt1_br_target),
        .cmt1_is_last_i    (rob_cmt1_is_last),
        .cmt1_ftq_id_i     (rob_cmt1_ftq_id),
        .cmt1_priv_vec_i   (rob_cmt1_priv_vec),
        .cmt1_csr_num_i    (rob_cmt1_csr_num),
        .cmt1_tlb_op_i     (rob_cmt1_tlb_op),
        .cmt1_cacop_code_i (rob_cmt1_cacop_code),
        .cmt1_excp_i       (rob_cmt1_excp),
        .rob_pop_o         (cmt_rob_pop),
        .rob_clear0_o      (cmt_rob_clear0),
        .rob_clear1_o      (cmt_rob_clear1),
        .arf_we0_o         (cmt_arf_we0),
        .arf_waddr0_o      (cmt_arf_waddr0),
        .arf_wdata0_o      (cmt_arf_wdata0),
        .arf_we1_o         (cmt_arf_we1),
        .arf_waddr1_o      (cmt_arf_waddr1),
        .arf_wdata1_o      (cmt_arf_wdata1),
        .rat_cmt_en0_o     (cmt_rat_en0),
        .rat_cmt_addr0_o   (cmt_rat_addr0),
        .rat_cmt_num0_o    (cmt_rat_num0),
        .rat_cmt_en1_o     (cmt_rat_en1),
        .rat_cmt_addr1_o   (cmt_rat_addr1),
        .rat_cmt_num1_o    (cmt_rat_num1),
        .sb_push_valid_o   (cmt_sb_push_valid),
        .sb_push_paddr_o   (cmt_sb_push_paddr),
        .sb_push_data_o    (cmt_sb_push_data),
        .sb_push_wstrb_o   (cmt_sb_push_wstrb),
        .sb_push_size_o    (cmt_sb_push_size),
        .sb_push_uncached_o(cmt_sb_push_uncached),
        .sb_full_i         (sb_full),
        .sb_empty_i        (sb_empty),
        .csr_cmt_valid_o   (cmt_csr_valid),
        .csr_cmt_pc_o      (cmt_csr_pc),
        .csr_cmt_ex_o      (cmt_csr_ex),
        .csr_cmt_ertn_o    (cmt_csr_ertn),
        .csr_cmt_vaddr_o   (cmt_csr_vaddr),
        .excp_int_o        (cmt_excp_int),
        .excp_adef_o       (cmt_excp_adef),
        .excp_adem_o       (cmt_excp_adem),
        .excp_ipe_o        (cmt_excp_ipe),
        .excp_ale_o        (cmt_excp_ale),
        .excp_sys_o        (cmt_excp_sys),
        .excp_brk_o        (cmt_excp_brk),
        .excp_ine_o        (cmt_excp_ine),
        .excp_tlb_vec_o    (cmt_excp_tlb_vec),
        .csr_we_o          (cmt_csr_we),
        .csr_wnum_o        (cmt_csr_wnum),
        .csr_wmask_o       (cmt_csr_wmask),
        .csr_wvalue_o      (cmt_csr_wvalue),
        .ll_set_o          (cmt_ll_set),
        .sc_set_o          (cmt_sc_set),
        .lladdr_o          (cmt_lladdr),
        .has_int_i         (csr_has_int),
        .csr_next_pc_i     (csr_next_pc),
        .csr_redirect_i    (csr_redirect),
        .tlb_op_cmt_o      (cmt_tlb_op),
        .invtlb_asid_o     (cmt_invtlb_asid),
        .invtlb_vpn_o      (cmt_invtlb_vpn),
        .icacop_valid_o    (cmt_icacop_valid),
        .icacop_op_o       (cmt_icacop_op),
        .icacop_addr_o     (cmt_icacop_addr),
        .dcacop_valid_o    (cmt_dcacop_valid),
        .dcacop_op_o       (cmt_dcacop_op),
        .dcacop_addr_o     (cmt_dcacop_addr),
        .ftq_cmt_valid_o   (cmt_ftq_valid),
        .ftq_cmt_id_o      (cmt_ftq_id),
        .ftq_cmt_is_last_o (cmt_ftq_is_last),
        .ftq_cmt_release_o (cmt_ftq_release),
        .ftq_cmt_is_branch_o(cmt_ftq_is_branch),
        .ftq_cmt_taken_o   (cmt_ftq_taken),
        .ftq_cmt_mispred_o (cmt_ftq_mispred),
        .ftq_cmt_target_o  (cmt_ftq_target),
        .ftq_cmt_br_type_o (cmt_ftq_br_type),
        .ftq_cmt_pc_o      (cmt_ftq_pc),
        .ftq_query_id_o    (cmt_ftq_query_id),
        .ftq_blk_target_i  (ftq_cmt_blk_target),
        .ras_cmt_call_o    (cmt_ras_call),
        .ras_cmt_ret_o     (cmt_ras_ret),
        .ras_cmt_retaddr_o (cmt_ras_retaddr),
        .flush_req_o       (cmt_flush_req),
        .flush_type_o      (cmt_flush_type),
        .flush_pc_o        (cmt_flush_pc),
        .idle_commit_o     (cmt_idle),
        .uncached_ld_inflight_i(lsu_unc_inflight),
        .debug0_valid_o    (cmt_dbg0_valid),
        .debug0_pc_o       (cmt_dbg0_pc),
        .debug0_rf_wen_o   (cmt_dbg0_wen),
        .debug0_rf_wnum_o  (cmt_dbg0_wnum),
        .debug0_rf_wdata_o (cmt_dbg0_wdata),
        .debug0_inst_o     (cmt_dbg0_inst),
        .debug1_valid_o    (cmt_dbg1_valid),
        .debug1_pc_o       (cmt_dbg1_pc),
        .debug1_rf_wen_o   (cmt_dbg1_wen),
        .debug1_rf_wnum_o  (cmt_dbg1_wnum),
        .debug1_rf_wdata_o (cmt_dbg1_wdata),
        .debug1_inst_o     (cmt_dbg1_inst)
    );



//--------------------------------------------------
// 地址翻译子系统（mmu + tlb_manager(内含主 tlb + I/D 两份 l1_tlb 微表)）
// l1_tlb 微表已在 tlb_manager 内部例化（二期加速：8 项组合命中 + fence 失效）
//--------------------------------------------------
    // CSR -> 翻译通路
    wire        csr_crmd_da, csr_crmd_pg;
    wire [1:0]  csr_crmd_datf, csr_crmd_datm;
    wire [9:0]  csr_asid;
    wire [31:0] csr_tlbidx, csr_tlbehi, csr_tlbelo0, csr_tlbelo1;
    wire [31:0] csr_dmw0, csr_dmw1;
    wire [4:0]  csr_rand_index;
    wire [7:0]  csr_estat_ecode;
    // mmu <-> tlb_manager
    wire        mmu_tlbm_inst_req;
    wire [31:0] mmu_tlbm_inst_vaddr;
    wire        mmu_tlbm_data_req;
    wire        mmu_tlbm_data_is_store;
    wire [31:0] mmu_tlbm_data_vaddr;
    wire [31:0] tlbm_inst_paddr;
    wire [1:0]  tlbm_inst_mat;
    wire        tlbm_inst_ex_adef;   // PLV3 取指越界（ADEF 特权子情形）
    wire        tlbm_inst_ex_tlbr, tlbm_inst_ex_pif, tlbm_inst_ex_ppi;
    wire [31:0] tlbm_data_paddr;
    wire [1:0]  tlbm_data_mat;
    wire        tlbm_data_ex_adem;   // PLV3 访存越界（ADEM）
    wire        tlbm_data_ex_tlbr, tlbm_data_ex_pil, tlbm_data_ex_pis;
    wire        tlbm_data_ex_ppi,  tlbm_data_ex_pme;
    // tlb_manager -> CSR（tlbsrch/tlbrd 回读）
    wire        tlbsrch_found;
    wire [4:0]  tlbsrch_index;
    wire [31:0] tlbrd_tlbidx, tlbrd_tlbehi, tlbrd_tlbelo0, tlbrd_tlbelo1;
    wire [9:0]  tlbrd_asid;
    // invtlb 类型编码（one-hot tlb_op -> 5bit op 号）
    wire [4:0]  cmt_invtlb_op = cmt_tlb_op[`TLB_OP_INVTLB_1] ? 5'd1 :
                                cmt_tlb_op[`TLB_OP_INVTLB_2] ? 5'd2 :
                                cmt_tlb_op[`TLB_OP_INVTLB_3] ? 5'd3 :
                                cmt_tlb_op[`TLB_OP_INVTLB_4] ? 5'd4 :
                                cmt_tlb_op[`TLB_OP_INVTLB_5] ? 5'd5 :
                                cmt_tlb_op[`TLB_OP_INVTLB_6] ? 5'd6 : 5'd0;

    mmu u_mmu(
        .i_req_i           (ifu_mmu_req),
        .i_vaddr_i         (ifu_mmu_vaddr),
        .i_paddr_o         (mmu_i_paddr),
        .i_mat_o           (mmu_i_mat),
        .i_excp_adef_o     (mmu_i_adef),
        .i_tlb_ex_o        (mmu_i_tlb_ex),
        .d_req_i           (lsu_mmu_req),
        .d_is_store_i      (lsu_mmu_is_store),
        .d_vaddr_i         (lsu_mmu_vaddr),
        .d_paddr_o         (mmu_d_paddr),
        .d_mat_o           (mmu_d_mat),
        .d_excp_adem_o     (mmu_d_adem),
        .d_tlb_ex_o        (mmu_d_tlb_ex),
        .tlbm_inst_req_o   (mmu_tlbm_inst_req),
        .tlbm_inst_vaddr_o (mmu_tlbm_inst_vaddr),
        .tlbm_data_req_o   (mmu_tlbm_data_req),
        .tlbm_data_is_store_o(mmu_tlbm_data_is_store),
        .tlbm_data_vaddr_o (mmu_tlbm_data_vaddr),
        .tlbm_inst_paddr_i (tlbm_inst_paddr),
        .tlbm_inst_mat_i   (tlbm_inst_mat),
        .tlbm_inst_ex_adef_i(tlbm_inst_ex_adef),
        .tlbm_inst_ex_tlbr_i(tlbm_inst_ex_tlbr),
        .tlbm_inst_ex_pif_i(tlbm_inst_ex_pif),
        .tlbm_inst_ex_ppi_i(tlbm_inst_ex_ppi),
        .tlbm_data_paddr_i (tlbm_data_paddr),
        .tlbm_data_mat_i   (tlbm_data_mat),
        .tlbm_data_ex_tlbr_i(tlbm_data_ex_tlbr),
        .tlbm_data_ex_pil_i(tlbm_data_ex_pil),
        .tlbm_data_ex_pis_i(tlbm_data_ex_pis),
        .tlbm_data_ex_ppi_i(tlbm_data_ex_ppi),
        .tlbm_data_ex_pme_i(tlbm_data_ex_pme),
        .tlbm_data_ex_adem_i(tlbm_data_ex_adem)
    );

    tlb_manager #(.TLBNUM(TLBNUM)) u_tlb_manager(
        .clk            (clk),
        .reset          (reset),
        .inst_req       (mmu_tlbm_inst_req),
        .inst_vaddr     (mmu_tlbm_inst_vaddr),
        .data_req       (mmu_tlbm_data_req),
        .data_is_store  (mmu_tlbm_data_is_store),
        .data_vaddr     (mmu_tlbm_data_vaddr),
        .csr_crmd_da    (csr_crmd_da),
        .csr_crmd_pg    (csr_crmd_pg),
        .csr_crmd_plv   (csr_crmd_plv),
        .csr_crmd_datf  (csr_crmd_datf),
        .csr_crmd_datm  (csr_crmd_datm),
        .csr_asid       (csr_asid),
        .csr_tlbidx     (csr_tlbidx),
        .csr_tlbehi     (csr_tlbehi),
        .csr_tlbelo0    (csr_tlbelo0),
        .csr_tlbelo1    (csr_tlbelo1),
        .csr_dmw0       (csr_dmw0),
        .csr_dmw1       (csr_dmw1),
        .csr_estat_ecode(csr_estat_ecode),
        .csr_rand_index (csr_rand_index),
        .tlb_op         (cmt_tlb_op),
        .invtlb_op      (cmt_invtlb_op),
        .invtlb_asid    (cmt_invtlb_asid),
        .invtlb_vpn     (cmt_invtlb_vpn),
        .inst_paddr     (tlbm_inst_paddr),
        .inst_mat       (tlbm_inst_mat),
        .inst_ex_adef   (tlbm_inst_ex_adef),
        .inst_ex_tlbr   (tlbm_inst_ex_tlbr),
        .inst_ex_pif    (tlbm_inst_ex_pif),
        .inst_ex_ppi    (tlbm_inst_ex_ppi),
        .data_paddr     (tlbm_data_paddr),
        .data_mat       (tlbm_data_mat),
        .data_ex_adem   (tlbm_data_ex_adem),
        .data_ex_tlbr   (tlbm_data_ex_tlbr),
        .data_ex_pil    (tlbm_data_ex_pil),
        .data_ex_pis    (tlbm_data_ex_pis),
        .data_ex_ppi    (tlbm_data_ex_ppi),
        .data_ex_pme    (tlbm_data_ex_pme),
        .tlbsrch_found  (tlbsrch_found),
        .tlbsrch_index  (tlbsrch_index),
        .tlbrd_tlbidx   (tlbrd_tlbidx),
        .tlbrd_tlbehi   (tlbrd_tlbehi),
        .tlbrd_tlbelo0  (tlbrd_tlbelo0),
        .tlbrd_tlbelo1  (tlbrd_tlbelo1),
        .tlbrd_asid     (tlbrd_asid)
    );



//--------------------------------------------------
// CSR：寄存器组与异常提交处理
//--------------------------------------------------
    wire [31:0] csr_crmd_live_unused;
    wire [27:0] csr_lladdr_unused;

`ifdef DIFFTEST_EN
    wire [31:0] diff_csr_crmd,   diff_csr_prmd,  diff_csr_euen,   diff_csr_ecfg;
    wire [31:0] diff_csr_estat,  diff_csr_era,   diff_csr_badv,   diff_csr_eentry;
    wire [31:0] diff_csr_tlbidx, diff_csr_tlbehi,diff_csr_tlbelo0,diff_csr_tlbelo1;
    wire [31:0] diff_csr_asid,   diff_csr_pgdl,  diff_csr_pgdh;
    wire [31:0] diff_csr_save0,  diff_csr_save1, diff_csr_save2,  diff_csr_save3;
    wire [31:0] diff_csr_tid,    diff_csr_tcfg,  diff_csr_tval,   diff_csr_ticlr;
    wire [31:0] diff_csr_llbctl, diff_csr_tlbrentry, diff_csr_dmw0, diff_csr_dmw1;
    wire [7:0]  diff_wb_ecode;
`endif

    csr_exception_commit_handler u_csr_exception_commit_handler(
        .clk            (clk),
        .reset          (reset),
        .csr_rnum       (mdu_csr_raddr[11:0]),     // fu_mdu 执行级读 CSR 旧值
        .csr_num        (cmt_csr_wnum[11:0]),      // commit 提交级写 CSR
        .csr_we         (cmt_csr_we),
        .csr_wmask      (cmt_csr_wmask),
        .csr_wvalue     (cmt_csr_wvalue),
        .tlbsrch_found  (tlbsrch_found),
        .tlbsrch_index  (tlbsrch_index),
        .tlbrd_tlbidx   (tlbrd_tlbidx),
        .tlbrd_tlbehi   (tlbrd_tlbehi),
        .tlbrd_tlbelo0  (tlbrd_tlbelo0),
        .tlbrd_tlbelo1  (tlbrd_tlbelo1),
        .tlbrd_asid     (tlbrd_asid),
        .hw_int_in      (intrpt),                  // 8 位硬中断 -> ESTAT.IS[9:2]
        .ipi_int_in     (1'b0),
        .wb_valid       (cmt_csr_valid),
        .wb_pc          (cmt_csr_pc),
        .wb_is_ertn     (cmt_csr_ertn),
        .wb_vaddr       (cmt_csr_vaddr),
        .wb_ex          (cmt_csr_ex),
        .wb_tlb_op      (cmt_tlb_op),
        .INT_valid      (cmt_excp_int),
        .ADEF_valid     (cmt_excp_adef),
        .ADEM_valid     (cmt_excp_adem),
        .IPE_valid      (cmt_excp_ipe),
        .TLB_EX_valid   (cmt_excp_tlb_vec),
        .ALE_valid      (cmt_excp_ale),
        .SYS_valid      (cmt_excp_sys),
        .BRK_valid      (cmt_excp_brk),
        .INE_valid      (cmt_excp_ine),
        .ll_set_in      (cmt_ll_set),
        .sc_set_in      (cmt_sc_set),
        .lladdr_in      (cmt_lladdr),
        .csr_next_pc    (csr_next_pc),
        .csr_redirect   (csr_redirect),
        .has_int        (csr_has_int),
        .csr_rvalue     (csr_rvalue),
        .csr_tid_out    (csr_tid),
        .csr_crmd_da_out(csr_crmd_da),
        .csr_crmd_pg_out(csr_crmd_pg),
        .csr_crmd_plv_out(csr_crmd_plv),
        .csr_asid_out   (csr_asid),
        .csr_tlbidx_out (csr_tlbidx),
        .csr_tlbehi_out (csr_tlbehi),
        .csr_tlbelo0_out(csr_tlbelo0),
        .csr_tlbelo1_out(csr_tlbelo1),
        .csr_dmw0_out   (csr_dmw0),
        .csr_dmw1_out   (csr_dmw1),
        .csr_rand_index_out(csr_rand_index),
        .csr_crmd_datf_out(csr_crmd_datf),
        .csr_crmd_datm_out(csr_crmd_datm),
        .csr_estat_ecode_out(csr_estat_ecode),
        .csr_crmd_out   (csr_crmd_live_unused),
        .csr_llbit_out  (csr_llbit),
        .csr_lladdr_out (csr_lladdr_unused),
        .diff_timer_64  (timer_64)
`ifdef DIFFTEST_EN
        ,
        .diff_csr_crmd      (diff_csr_crmd),
        .diff_csr_prmd      (diff_csr_prmd),
        .diff_csr_euen      (diff_csr_euen),
        .diff_csr_ecfg      (diff_csr_ecfg),
        .diff_csr_estat     (diff_csr_estat),
        .diff_csr_era       (diff_csr_era),
        .diff_csr_badv      (diff_csr_badv),
        .diff_csr_eentry    (diff_csr_eentry),
        .diff_csr_tlbidx    (diff_csr_tlbidx),
        .diff_csr_tlbehi    (diff_csr_tlbehi),
        .diff_csr_tlbelo0   (diff_csr_tlbelo0),
        .diff_csr_tlbelo1   (diff_csr_tlbelo1),
        .diff_csr_asid      (diff_csr_asid),
        .diff_csr_pgdl      (diff_csr_pgdl),
        .diff_csr_pgdh      (diff_csr_pgdh),
        .diff_csr_save0     (diff_csr_save0),
        .diff_csr_save1     (diff_csr_save1),
        .diff_csr_save2     (diff_csr_save2),
        .diff_csr_save3     (diff_csr_save3),
        .diff_csr_tid       (diff_csr_tid),
        .diff_csr_tcfg      (diff_csr_tcfg),
        .diff_csr_tval      (diff_csr_tval),
        .diff_csr_ticlr     (diff_csr_ticlr),
        .diff_csr_llbctl    (diff_csr_llbctl),
        .diff_csr_tlbrentry (diff_csr_tlbrentry),
        .diff_csr_dmw0      (diff_csr_dmw0),
        .diff_csr_dmw1      (diff_csr_dmw1),
        .diff_wb_ecode      (diff_wb_ecode)
`endif
    );



//--------------------------------------------------
// 数据缓存 + L2 + AXI 桥
//--------------------------------------------------
    // DCache <-> L2 互联
    wire         dc_l2_rd_req;
    wire [2:0]   dc_l2_rd_type;
    wire [31:0]  dc_l2_rd_addr;
    wire         l2_dc_rd_rdy;
    wire         l2_dc_ret_valid;
    wire         l2_dc_ret_last;
    wire [127:0] l2_dc_ret_data;
    wire         dc_l2_wr_req;
    wire [2:0]   dc_l2_wr_type;
    wire [31:0]  dc_l2_wr_addr;
    wire [15:0]  dc_l2_wr_strb;
    wire [127:0] dc_l2_wr_data;
    wire         dc_l2_wr_cacop;
    wire         l2_dc_wr_rdy;
    // L2 <-> AXI 桥互联（读口 0：D 侧/旁路，走桥 dc 通道 ARID=1）
    wire         l2_mem_rd_req;
    wire [2:0]   l2_mem_rd_type;
    wire [31:0]  l2_mem_rd_addr;
    wire         mem_l2_rd_rdy;
    wire         mem_l2_ret_valid;
    wire         mem_l2_ret_last;
    wire [127:0] mem_l2_ret_data;
    // L2 <-> AXI 桥互联（读口 1：I-miss 引擎，走桥 ic 通道 ARID=0）
    wire         l2_mem2_rd_req;
    wire [2:0]   l2_mem2_rd_type;
    wire [31:0]  l2_mem2_rd_addr;
    wire         mem2_l2_rd_rdy;
    wire         mem2_l2_ret_valid;
    wire         mem2_l2_ret_last;
    wire [127:0] mem2_l2_ret_data;
    wire         l2_mem_wr_req;
    wire [2:0]   l2_mem_wr_type;
    wire [31:0]  l2_mem_wr_addr;
    wire [15:0]  l2_mem_wr_strb;
    wire [127:0] l2_mem_wr_data;
    wire         mem_l2_wr_rdy;
    // AXI 桥 <-> 顶层 AXI 引脚
    wire         axi_awvalid_line;
    wire [31:0]  axi_awaddr_line;
    wire [2:0]   axi_awburst_line;
    wire [3:0]   axi_awlen_line;
    wire [2:0]   axi_awsize_line;
    wire         axi_wvalid_line;
    wire [31:0]  axi_wdata_line;
    wire [3:0]   axi_wstrb_line;
    wire         axi_wlast_line;
    wire         axi_bready_line;
    wire         axi_arvalid_line;
    wire [3:0]   axi_arid_line;
    wire [31:0]  axi_araddr_line;
    wire [2:0]   axi_arburst_line;
    wire [3:0]   axi_arlen_line;
    wire [2:0]   axi_arsize_line;
    wire         axi_rready_line;

    dcache u_dcache(
        .clk            (clk),
        .resetn         (aresetn),
        .ld_req_i       (lsu_dc_req),
        .ld_vaddr_i     (lsu_dc_vaddr),
        .ld_paddr_i     (lsu_dc_paddr),
        .ld_size_i      (lsu_dc_size),
        .ld_uncached_i  (lsu_dc_uncached),
        .ld_addr_ok_o   (dc_lsu_addr_ok),
        .ld_data_ok_o   (dc_lsu_data_ok),
        .ld_rdata_o     (dc_lsu_rdata),
        .ld_cancel_i    (lsu_dc_cancel),
        .ld_miss_o      (dc_lsu_miss),
        .ld_mshr_data_ok_o(dc_lsu_mshr_ok),
        .ld_mshr_rdata_o(dc_lsu_mshr_rdata),
        .st_req_i       (sb_dc_wr_req),
        .st_paddr_i     (sb_dc_wr_paddr),
        .st_data_i      (sb_dc_wr_data),
        .st_strb_i      (sb_dc_wr_strb),
        .st_size_i      (sb_dc_wr_size),
        .st_uncached_i  (sb_dc_wr_uncached),
        .st_addr_ok_o   (dc_sb_addr_ok),
        .st_done_o      (dc_sb_done),
        .cacop_en_i     (cmt_dcacop_valid),
        .cacop_op_i     (cmt_dcacop_op),
        .cacop_addr_i   (cmt_dcacop_addr),
        .axi_rd_req     (dc_l2_rd_req),
        .axi_rd_type    (dc_l2_rd_type),
        .axi_rd_addr    (dc_l2_rd_addr),
        .axi_rd_rdy     (l2_dc_rd_rdy),
        .axi_ret_valid  (l2_dc_ret_valid),
        .axi_ret_last   (l2_dc_ret_last),
        .axi_ret_data   (l2_dc_ret_data),
        .axi_wr_req     (dc_l2_wr_req),
        .axi_wr_type    (dc_l2_wr_type),
        .axi_wr_addr    (dc_l2_wr_addr),
        .axi_wr_strb    (dc_l2_wr_strb),
        .axi_wr_data    (dc_l2_wr_data),
        .axi_wr_cacop   (dc_l2_wr_cacop),
        .axi_wr_rdy     (l2_dc_wr_rdy)
    );

    l2cache u_l2cache(
        .clk           (clk),
        .resetn        (aresetn),
        .ic_rd_req     (ic_l2_rd_req),
        .ic_rd_type    (ic_l2_rd_type),
        .ic_rd_addr    (ic_l2_rd_addr),
        .ic_rd_rdy     (l2_ic_rd_rdy),
        .ic_ret_valid  (l2_ic_ret_valid),
        .ic_ret_last   (l2_ic_ret_last),
        .ic_ret_data   (l2_ic_ret_data),
        .dc_rd_req     (dc_l2_rd_req),
        .dc_rd_type    (dc_l2_rd_type),
        .dc_rd_addr    (dc_l2_rd_addr),
        .dc_rd_rdy     (l2_dc_rd_rdy),
        .dc_ret_valid  (l2_dc_ret_valid),
        .dc_ret_last   (l2_dc_ret_last),
        .dc_ret_data   (l2_dc_ret_data),
        .dc_wr_req     (dc_l2_wr_req),
        .dc_wr_type    (dc_l2_wr_type),
        .dc_wr_addr    (dc_l2_wr_addr),
        .dc_wr_strb    (dc_l2_wr_strb),
        .dc_wr_data    (dc_l2_wr_data),
        .dc_wr_cacop   (dc_l2_wr_cacop),
        .dc_wr_rdy     (l2_dc_wr_rdy),
        .mem_rd_req    (l2_mem_rd_req),
        .mem_rd_type   (l2_mem_rd_type),
        .mem_rd_addr   (l2_mem_rd_addr),
        .mem_rd_rdy    (mem_l2_rd_rdy),
        .mem_ret_valid (mem_l2_ret_valid),
        .mem_ret_last  (mem_l2_ret_last),
        .mem_ret_data  (mem_l2_ret_data),
        // I-miss 引擎专用读口（接桥 ic 通道，双 outstanding）
        .mem2_rd_req   (l2_mem2_rd_req),
        .mem2_rd_type  (l2_mem2_rd_type),
        .mem2_rd_addr  (l2_mem2_rd_addr),
        .mem2_rd_rdy   (mem2_l2_rd_rdy),
        .mem2_ret_valid(mem2_l2_ret_valid),
        .mem2_ret_last (mem2_l2_ret_last),
        .mem2_ret_data (mem2_l2_ret_data),
        .mem_wr_req    (l2_mem_wr_req),
        .mem_wr_type   (l2_mem_wr_type),
        .mem_wr_addr   (l2_mem_wr_addr),
        .mem_wr_strb   (l2_mem_wr_strb),
        .mem_wr_data   (l2_mem_wr_data),
        .mem_wr_rdy    (mem_l2_wr_rdy)
    );

    axi_line_bridge u_axi_line_bridge(
        .clk           (clk),
        .resetn        (aresetn),
        // ic 通道：L2 的 I-miss 引擎读口（ARID=0，与 dc 通道双 outstanding）
        .ic_rd_req     (l2_mem2_rd_req),
        .ic_rd_type    (l2_mem2_rd_type),
        .ic_rd_addr    (l2_mem2_rd_addr),
        .ic_rd_rdy     (mem2_l2_rd_rdy),
        .ic_ret_valid  (mem2_l2_ret_valid),
        .ic_ret_last   (mem2_l2_ret_last),
        .ic_ret_data   (mem2_l2_ret_data),
        .dc_rd_req     (l2_mem_rd_req),
        .dc_rd_type    (l2_mem_rd_type),
        .dc_rd_addr    (l2_mem_rd_addr),
        .dc_rd_rdy     (mem_l2_rd_rdy),
        .dc_ret_valid  (mem_l2_ret_valid),
        .dc_ret_last   (mem_l2_ret_last),
        .dc_ret_data   (mem_l2_ret_data),
        .dc_wr_req     (l2_mem_wr_req),
        .dc_wr_type    (l2_mem_wr_type),
        .dc_wr_addr    (l2_mem_wr_addr),
        .dc_wr_strb    (l2_mem_wr_strb),
        .dc_wr_data    (l2_mem_wr_data),
        .dc_wr_rdy     (mem_l2_wr_rdy),
        .axi_awvalid   (axi_awvalid_line),
        .axi_awaddr    (axi_awaddr_line),
        .axi_awburst   (axi_awburst_line),
        .axi_awlen     (axi_awlen_line),
        .axi_awsize    (axi_awsize_line),
        .axi_awready   (awready),
        .axi_wvalid    (axi_wvalid_line),
        .axi_wdata     (axi_wdata_line),
        .axi_wstrb     (axi_wstrb_line),
        .axi_wlast     (axi_wlast_line),
        .axi_wready    (wready),
        .axi_bvalid    (bvalid),
        .axi_bresp     (bresp),
        .axi_bready    (axi_bready_line),
        .axi_arvalid   (axi_arvalid_line),
        .axi_arid      (axi_arid_line),
        .axi_araddr    (axi_araddr_line),
        .axi_arburst   (axi_arburst_line),
        .axi_arlen     (axi_arlen_line),
        .axi_arsize    (axi_arsize_line),
        .axi_arready   (arready),
        .axi_rvalid    (rvalid),
        .axi_rid       (rid),
        .axi_rdata     (rdata),
        .axi_rresp     (rresp),
        .axi_rlast     (rlast),
        .axi_rready    (axi_rready_line)
    );

    // AXI 引脚固定属性（与 chiplab SoC 对接约定一致）
    // arid 由桥给出：0=取指（L2 I-miss 引擎）、1=数据——R 通道按 rid 分流
    assign arid    = axi_arid_line;
    assign araddr  = axi_araddr_line;
    assign arlen   = {4'b0000, axi_arlen_line};
    assign arsize  = axi_arsize_line;
    assign arburst = 2'b01;
    assign arlock  = 2'b00;
    assign arcache = 4'b0011;
    assign arprot  = 3'b000;
    assign arvalid = axi_arvalid_line;
    assign rready  = axi_rready_line;

    assign awid    = 4'b0000;
    assign awaddr  = axi_awaddr_line;
    assign awlen   = {4'b0000, axi_awlen_line};
    assign awsize  = axi_awsize_line;
    assign awburst = 2'b01;
    assign awlock  = 2'b00;
    assign awcache = 4'b0011;
    assign awprot  = 3'b000;
    assign awvalid = axi_awvalid_line;

    assign wid     = 4'b0000;
    assign wdata   = axi_wdata_line;
    assign wstrb   = axi_wstrb_line;
    assign wlast   = axi_wlast_line;
    assign wvalid  = axi_wvalid_line;

    assign bready  = axi_bready_line;



//--------------------------------------------------
// 调试：对齐测试平台的提交观测（debug0 接提交槽 0）
//--------------------------------------------------
    assign ws_valid           = cmt_dbg0_valid;
    assign debug0_wb_pc       = cmt_dbg0_pc;
    assign debug0_wb_rf_wen   = cmt_dbg0_wen;
    assign debug0_wb_rf_wnum  = cmt_dbg0_wnum;
    assign debug0_wb_rf_wdata = cmt_dbg0_wdata;
    assign debug0_wb_inst     = cmt_dbg0_inst;
    `ifdef CPU_2CMT
    assign debug1_wb_pc       = cmt_dbg1_pc;
    assign debug1_wb_rf_wen   = cmt_dbg1_wen;
    assign debug1_wb_rf_wnum  = cmt_dbg1_wnum;
    assign debug1_wb_rf_wdata = cmt_dbg1_wdata;
    assign debug1_wb_inst     = cmt_dbg1_inst;
    `endif

    // INE 定位探针（log_vcd u_cpu/* 可见）
    wire        dbg_rob_cmt0_valid    = rob_cmt0_valid;
    wire        dbg_rob_cmt0_complete = rob_cmt0_complete;
    wire [31:0] dbg_rob_cmt0_pc       = rob_cmt0_pc;
    wire [31:0] dbg_rob_cmt0_inst     = rob_cmt0_inst;
    wire [`EXCP_NUM-1:0] dbg_rob_cmt0_excp = rob_cmt0_excp;
    wire [31:0] dbg_ib_pop0_pc        = ib_pop0_pc;
    wire [31:0] dbg_ib_pop0_inst      = ib_pop0_inst;
    wire [`EXCP_NUM-1:0] dbg_ib_pop0_excp = ib_pop0_excp;
    wire [`EXCP_NUM-1:0] dbg_dec0_excp     = dec0_excp;
    wire [`EXCP_NUM-1:0] dbg_dec0_excp_eff = dec0_excp_eff;
    wire [`EXCP_NUM-1:0] dbg_rn_ib0_excp   = rn_ib0_excp;

    // Spyglass/lint：吸收暂未使用的观测信号
    wire mycpu_lint_sink;
    assign mycpu_lint_sink = break_point | infor_flag
                           | (|csr_crmd_live_unused) | (|csr_lladdr_unused)
                           | (|rsm_occupancy) | (|rsd_occupancy)
`ifndef CPU_2CMT
                           | cmt_dbg1_valid
`endif
                           | dbg_rob_cmt0_valid | dbg_rob_cmt0_complete
                           | (|dbg_rob_cmt0_excp) | (|dbg_ib_pop0_excp)
                           | (|dbg_dec0_excp) | (|dbg_rn_ib0_excp);



`ifdef DIFFTEST_EN
//--------------------------------------------------
// DIFFTEST DPI 接口（双提交：InstrCommit x2 / LoadEvent x2 / StoreEvent x1）
//--------------------------------------------------
    localparam [7:0] DIFFTEST_COREID = 8'd0;

    // Difftest 提交：csrrd ESTAT → csr_rstat/csr_data；rdcnt → is_CNTinst；tlbfill → index
    //      2. is_CNTinst/timer_64_value：rdcntvl/vh/id 提交拍锁存 timer
    //      3. 异常/ertn 走 ExcpEvent（excp_flush/ertn_flush），普通提交走 InstrCommit
    //      4. store/load 事件的 8 位类型掩码按 inst 编码生成（下面给了 st/ld 模板）
    //      5. 所有 cmt_* 信号 posedge 打一拍再送 DPI（与 NEMU 步进对齐）

    // 槽 0/1 提交指令的访存类型掩码（按指令编码判别，与旧实现同口径）
    wire dbg0_is_st_w = (cmt_dbg0_inst[31:26] == 6'h0a) && (cmt_dbg0_inst[25:22] == 4'h6);
    wire dbg0_is_st_h = (cmt_dbg0_inst[31:26] == 6'h0a) && (cmt_dbg0_inst[25:22] == 4'h5);
    wire dbg0_is_st_b = (cmt_dbg0_inst[31:26] == 6'h0a) && (cmt_dbg0_inst[25:22] == 4'h4);
    wire dbg0_is_ld_w = (cmt_dbg0_inst[31:26] == 6'h0a) && (cmt_dbg0_inst[25:22] == 4'h2);
    wire dbg0_is_ld_h = (cmt_dbg0_inst[31:26] == 6'h0a) && (cmt_dbg0_inst[25:22] == 4'h1);
    wire dbg0_is_ld_b = (cmt_dbg0_inst[31:26] == 6'h0a) && (cmt_dbg0_inst[25:22] == 4'h0);
    wire dbg0_is_ld_hu= (cmt_dbg0_inst[31:26] == 6'h0a) && (cmt_dbg0_inst[25:22] == 4'h9);
    wire dbg0_is_ld_bu= (cmt_dbg0_inst[31:26] == 6'h0a) && (cmt_dbg0_inst[25:22] == 4'h8);
    wire dbg0_is_ll_w = (cmt_dbg0_inst[31:29] == 3'b001) && (cmt_dbg0_inst[28:24] == 5'b00000);
    wire dbg0_is_sc_w = (cmt_dbg0_inst[31:29] == 3'b001) && (cmt_dbg0_inst[28:24] == 5'b00001);
    // rdcntvl/vh：提交时把 DUT 读到的半字同步进 NEMU（见 difftest.cpp is_CNTinst）
    wire dbg0_is_rdcntvl = (cmt_dbg0_inst[31:15] == 17'b0) && (cmt_dbg0_inst[14:10] == 5'h18)
                        && (cmt_dbg0_inst[9:5] == 5'h0) && (cmt_dbg0_inst[4:0] != 5'h0);
    wire dbg0_is_rdcntvh = (cmt_dbg0_inst[31:15] == 17'b0) && (cmt_dbg0_inst[14:10] == 5'h19)
                        && (cmt_dbg0_inst[9:5] == 5'h0);
    wire dbg0_is_cntinst = dbg0_is_rdcntvl | dbg0_is_rdcntvh;
    wire [63:0] dbg0_timer64 = dbg0_is_rdcntvh ? {cmt_dbg0_wdata, 32'b0}
                                               : {32'b0, cmt_dbg0_wdata};
    // tlbfill：须把提交拍使用的 rand_index 同步给 NEMU，否则两边写入不同表项
    wire dbg0_is_tlbfill = (cmt_dbg0_inst[31:10] == 22'h1920d) // 01_1001_00_10000_01101
                        && (cmt_dbg0_inst[9:0] == 10'b0);
    wire dbg1_is_st_w = (cmt_dbg1_inst[31:26] == 6'h0a) && (cmt_dbg1_inst[25:22] == 4'h6);
    wire dbg1_is_st_h = (cmt_dbg1_inst[31:26] == 6'h0a) && (cmt_dbg1_inst[25:22] == 4'h5);
    wire dbg1_is_st_b = (cmt_dbg1_inst[31:26] == 6'h0a) && (cmt_dbg1_inst[25:22] == 4'h4);
    wire dbg1_is_ld_w = (cmt_dbg1_inst[31:26] == 6'h0a) && (cmt_dbg1_inst[25:22] == 4'h2);
    wire dbg1_is_ld_h = (cmt_dbg1_inst[31:26] == 6'h0a) && (cmt_dbg1_inst[25:22] == 4'h1);
    wire dbg1_is_ld_b = (cmt_dbg1_inst[31:26] == 6'h0a) && (cmt_dbg1_inst[25:22] == 4'h0);
    wire dbg1_is_ld_hu= (cmt_dbg1_inst[31:26] == 6'h0a) && (cmt_dbg1_inst[25:22] == 4'h9);
    wire dbg1_is_ld_bu= (cmt_dbg1_inst[31:26] == 6'h0a) && (cmt_dbg1_inst[25:22] == 4'h8);
    wire dbg1_is_ll_w = (cmt_dbg1_inst[31:29] == 3'b001) && (cmt_dbg1_inst[28:24] == 5'b00000);
    wire dbg1_is_sc_w = (cmt_dbg1_inst[31:29] == 3'b001) && (cmt_dbg1_inst[28:24] == 5'b00001);
    wire dbg1_is_rdcntvl = (cmt_dbg1_inst[31:15] == 17'b0) && (cmt_dbg1_inst[14:10] == 5'h18)
                        && (cmt_dbg1_inst[9:5] == 5'h0) && (cmt_dbg1_inst[4:0] != 5'h0);
    wire dbg1_is_rdcntvh = (cmt_dbg1_inst[31:15] == 17'b0) && (cmt_dbg1_inst[14:10] == 5'h19)
                        && (cmt_dbg1_inst[9:5] == 5'h0);
    wire dbg1_is_cntinst = dbg1_is_rdcntvl | dbg1_is_rdcntvh;
    wire [63:0] dbg1_timer64 = dbg1_is_rdcntvh ? {cmt_dbg1_wdata, 32'b0}
                                               : {32'b0, cmt_dbg1_wdata};
    wire dbg1_is_tlbfill = (cmt_dbg1_inst[31:10] == 22'h1920d)
                        && (cmt_dbg1_inst[9:0] == 10'b0);

    // csrrd/csrwr/csrxchg ESTAT：digftest 用 csr_rstat/csr_data 同步 DUT ESTAT 进 NEMU
    // （手册要求三种 CSR 指令都要拉高；csr_data 用提交后建筑态 ESTAT，csrwr 时 wdata 是旧值）
    wire dbg0_is_csr_op = (cmt_dbg0_inst[31:24] == 8'h04);
    wire dbg1_is_csr_op = (cmt_dbg1_inst[31:24] == 8'h04);
    wire dbg0_is_csr_estat = dbg0_is_csr_op
                          && (cmt_dbg0_inst[23:10] == {2'b0, `CSR_ESTAT});
    wire dbg1_is_csr_estat = dbg1_is_csr_op
                          && (cmt_dbg1_inst[23:10] == {2'b0, `CSR_ESTAT});

    // 提交拍打一拍后送 DPI（槽 0）
    reg         cmt0_valid_r;
    reg  [31:0] cmt0_pc_r, cmt0_inst_r, cmt0_wdata_r;
    reg  [7:0]  cmt0_wdest_r;
    reg         cmt0_wen_r;
    reg  [7:0]  cmt0_ld_en_r, cmt0_st_en_r;
    reg  [31:0] cmt0_ld_paddr_r, cmt0_ld_vaddr_r;
    reg  [31:0] cmt0_st_paddr_r, cmt0_st_vaddr_r, cmt0_st_data_r;
    reg         cmt0_is_cnt_r;
    reg  [63:0] cmt0_timer64_r;
    reg         cmt0_is_tlbfill_r;
    reg  [4:0]  cmt0_tlbfill_idx_r;
    reg         cmt0_csr_rstat_r;
    reg  [31:0] cmt0_csr_data_r;
    reg         excp_flush_r, ertn_flush_r;
    reg  [31:0] excp_pc_r, excp_inst_r;
    reg  [7:0]  excp_ecode_r;
    // 槽 1
    reg         cmt1_valid_r;
    reg  [31:0] cmt1_pc_r, cmt1_inst_r, cmt1_wdata_r;
    reg  [7:0]  cmt1_wdest_r;
    reg         cmt1_wen_r;
    reg  [7:0]  cmt1_ld_en_r, cmt1_st_en_r;
    reg  [31:0] cmt1_ld_paddr_r, cmt1_ld_vaddr_r;
    reg  [31:0] cmt1_st_paddr_r, cmt1_st_vaddr_r, cmt1_st_data_r;
    reg         cmt1_is_cnt_r;
    reg  [63:0] cmt1_timer64_r;
    reg         cmt1_is_tlbfill_r;
    reg  [4:0]  cmt1_tlbfill_idx_r;
    reg         cmt1_csr_rstat_r;
    reg  [31:0] cmt1_csr_data_r;

    always @(posedge clk) begin
        if (reset) begin
            cmt0_valid_r <= 1'b0;  cmt1_valid_r <= 1'b0;
            excp_flush_r <= 1'b0;  ertn_flush_r <= 1'b0;
            cmt0_pc_r <= 32'b0;    cmt0_inst_r <= 32'b0;  cmt0_wdata_r <= 32'b0;
            cmt0_wdest_r <= 8'b0;  cmt0_wen_r <= 1'b0;
            cmt0_ld_en_r <= 8'b0;  cmt0_st_en_r <= 8'b0;
            cmt0_ld_paddr_r <= 32'b0; cmt0_ld_vaddr_r <= 32'b0;
            cmt0_st_paddr_r <= 32'b0; cmt0_st_vaddr_r <= 32'b0; cmt0_st_data_r <= 32'b0;
            cmt0_is_cnt_r <= 1'b0; cmt0_timer64_r <= 64'b0;
            cmt0_is_tlbfill_r <= 1'b0; cmt0_tlbfill_idx_r <= 5'b0;
            cmt0_csr_rstat_r <= 1'b0; cmt0_csr_data_r <= 32'b0;
            excp_pc_r <= 32'b0;    excp_inst_r <= 32'b0;  excp_ecode_r <= 8'b0;
            cmt1_pc_r <= 32'b0;    cmt1_inst_r <= 32'b0;  cmt1_wdata_r <= 32'b0;
            cmt1_wdest_r <= 8'b0;  cmt1_wen_r <= 1'b0;
            cmt1_ld_en_r <= 8'b0;  cmt1_st_en_r <= 8'b0;
            cmt1_ld_paddr_r <= 32'b0; cmt1_ld_vaddr_r <= 32'b0;
            cmt1_st_paddr_r <= 32'b0; cmt1_st_vaddr_r <= 32'b0; cmt1_st_data_r <= 32'b0;
            cmt1_is_cnt_r <= 1'b0; cmt1_timer64_r <= 64'b0;
            cmt1_is_tlbfill_r <= 1'b0; cmt1_tlbfill_idx_r <= 5'b0;
            cmt1_csr_rstat_r <= 1'b0; cmt1_csr_data_r <= 32'b0;
        end else begin
            cmt0_valid_r    <= cmt_dbg0_valid;
            cmt0_pc_r       <= cmt_dbg0_pc;
            cmt0_inst_r     <= cmt_dbg0_inst;
            cmt0_wen_r      <= cmt_dbg0_wen[0];
            cmt0_wdest_r    <= {3'b0, cmt_dbg0_wnum};
            cmt0_wdata_r    <= cmt_dbg0_wdata;
            cmt0_ld_en_r    <= (cmt_dbg0_valid && rob_cmt0_is_load)
                               ? {2'b0, dbg0_is_ll_w, dbg0_is_ld_w, dbg0_is_ld_hu,
                                  dbg0_is_ld_h, dbg0_is_ld_bu, dbg0_is_ld_b} : 8'b0;
            cmt0_ld_paddr_r <= rob_cmt0_paddr;
            cmt0_ld_vaddr_r <= rob_cmt0_vaddr;
            cmt0_st_en_r    <= (cmt_dbg0_valid && rob_cmt0_is_store)
                               ? {4'b0, dbg0_is_sc_w, dbg0_is_st_w, dbg0_is_st_h, dbg0_is_st_b} : 8'b0;
            cmt0_st_paddr_r <= rob_cmt0_paddr;
            cmt0_st_vaddr_r <= rob_cmt0_vaddr;
            cmt0_st_data_r  <= rob_cmt0_result;
            cmt0_is_cnt_r   <= cmt_dbg0_valid && dbg0_is_cntinst;
            cmt0_timer64_r  <= dbg0_timer64;
            cmt0_is_tlbfill_r  <= cmt_dbg0_valid && dbg0_is_tlbfill;
            cmt0_tlbfill_idx_r <= csr_rand_index;
            cmt0_csr_rstat_r <= cmt_dbg0_valid && dbg0_is_csr_estat;
            cmt0_csr_data_r  <= diff_csr_estat;
            excp_flush_r    <= cmt_flush_req && (cmt_flush_type == `FLUSH_EXCP);
            ertn_flush_r    <= cmt_flush_req && (cmt_flush_type == `FLUSH_ERTN);
            excp_pc_r       <= cmt_csr_pc;
            excp_inst_r     <= cmt_dbg0_inst;
            excp_ecode_r    <= diff_wb_ecode;
            cmt1_valid_r    <= cmt_dbg1_valid;
            cmt1_pc_r       <= cmt_dbg1_pc;
            cmt1_inst_r     <= cmt_dbg1_inst;
            cmt1_wen_r      <= cmt_dbg1_wen[0];
            cmt1_wdest_r    <= {3'b0, cmt_dbg1_wnum};
            cmt1_wdata_r    <= cmt_dbg1_wdata;
            cmt1_ld_en_r    <= (cmt_dbg1_valid && rob_cmt1_is_load)
                               ? {2'b0, dbg1_is_ll_w, dbg1_is_ld_w, dbg1_is_ld_hu,
                                  dbg1_is_ld_h, dbg1_is_ld_bu, dbg1_is_ld_b} : 8'b0;
            cmt1_ld_paddr_r <= rob_cmt1_paddr;
            cmt1_ld_vaddr_r <= rob_cmt1_vaddr;
            cmt1_st_en_r    <= (cmt_dbg1_valid && rob_cmt1_is_store)
                               ? {4'b0, dbg1_is_sc_w, dbg1_is_st_w, dbg1_is_st_h, dbg1_is_st_b} : 8'b0;
            cmt1_st_paddr_r <= rob_cmt1_paddr;
            cmt1_st_vaddr_r <= rob_cmt1_vaddr;
            cmt1_st_data_r  <= rob_cmt1_result;
            cmt1_is_cnt_r   <= cmt_dbg1_valid && dbg1_is_cntinst;
            cmt1_timer64_r  <= dbg1_timer64;
            cmt1_is_tlbfill_r  <= cmt_dbg1_valid && dbg1_is_tlbfill;
            cmt1_tlbfill_idx_r <= csr_rand_index;
            cmt1_csr_rstat_r <= cmt_dbg1_valid && dbg1_is_csr_estat;
            cmt1_csr_data_r  <= diff_csr_estat;
        end
    end

    // DifftestInstrCommit 的 DPI 带 WITH_EN(valid)：valid=0 时不会调用 C 侧，
    // 而 Difftest::step() 又要求 commit[] 从 index0 连续有效。
    // ROB 成对分配后可能出现「只退 odd 槽」(cmt1_head)：若直接绑到 index1，
    // 会留下空洞 → 本拍丢提交、且 stale commit[1] 会和下拍 index0 拼成假双提交
    // （func_lab19 开头表现为 this_pc: right=1c000008 wrong=1c000004）。
    // 出口处压成无空洞队列：仅槽1时并入 index0；双提交时保持 0/1 原序。
    // Store/LoadEvent 必须与打包后的 commit 下标对齐，否则会漏报 cmt1_head 的 store。
    wire        diff_cmt_slot1_only = cmt1_valid_r && !cmt0_valid_r;
    wire        diff_cmt0_v    = cmt0_valid_r | diff_cmt_slot1_only;
    wire        diff_cmt1_v    = cmt0_valid_r & cmt1_valid_r;
    wire [31:0] diff_cmt0_pc   = cmt0_valid_r ? cmt0_pc_r    : cmt1_pc_r;
    wire [31:0] diff_cmt0_inst = cmt0_valid_r ? cmt0_inst_r  : cmt1_inst_r;
    wire        diff_cmt0_wen  = cmt0_valid_r ? cmt0_wen_r   : cmt1_wen_r;
    wire [7:0]  diff_cmt0_wdest= cmt0_valid_r ? cmt0_wdest_r : cmt1_wdest_r;
    wire [31:0] diff_cmt0_wdata= cmt0_valid_r ? cmt0_wdata_r : cmt1_wdata_r;
    wire [7:0]  diff_cmt0_ld_en= cmt0_valid_r ? cmt0_ld_en_r : cmt1_ld_en_r;
    wire [31:0] diff_cmt0_ld_paddr = cmt0_valid_r ? cmt0_ld_paddr_r : cmt1_ld_paddr_r;
    wire [31:0] diff_cmt0_ld_vaddr = cmt0_valid_r ? cmt0_ld_vaddr_r : cmt1_ld_vaddr_r;
    wire [7:0]  diff_st0_en    = cmt0_valid_r ? cmt0_st_en_r : cmt1_st_en_r;
    wire [31:0] diff_st0_paddr = cmt0_valid_r ? cmt0_st_paddr_r : cmt1_st_paddr_r;
    wire [31:0] diff_st0_vaddr = cmt0_valid_r ? cmt0_st_vaddr_r : cmt1_st_vaddr_r;
    // LSU/ROB 中 store data 是按写选通复制的内存字（便于 SB/DCache）；
    // NEMU StoreEvent 要「落在目标字节道、其余为 0」：st.b@+0→0xa，st.h@+2→0x00f00000。
    // 用 vaddr[1:0] 生成字节掩码，从复制字里剥出与 REF 同口径的值。
    function automatic [31:0] diff_store_nbytes;
        input [7:0]  st_en;
        input [31:0] st_data;
        input [31:0] st_vaddr;
        reg   [31:0] mask;
        begin
            if (st_en[0])      mask = 32'h000000ff << {st_vaddr[1:0], 3'b0};
            else if (st_en[1]) mask = 32'h0000ffff << {st_vaddr[1], 4'b0};
            else               mask = 32'hffffffff;
            diff_store_nbytes = st_data & mask;
        end
    endfunction
    wire [31:0] diff_st0_data_raw = cmt0_valid_r ? cmt0_st_data_r : cmt1_st_data_r;
    wire [31:0] diff_st0_data = diff_store_nbytes(diff_st0_en, diff_st0_data_raw, diff_st0_vaddr);
    wire [7:0]  diff_st1_en   = diff_cmt1_v ? cmt1_st_en_r : 8'b0;
    wire [31:0] diff_st1_data = diff_store_nbytes(diff_st1_en, cmt1_st_data_r, cmt1_st_vaddr_r);
    wire        diff_cmt0_cnt = cmt0_valid_r ? cmt0_is_cnt_r : cmt1_is_cnt_r;
    wire [63:0] diff_cmt0_tmr = cmt0_valid_r ? cmt0_timer64_r : cmt1_timer64_r;
    wire        diff_cmt0_fill = cmt0_valid_r ? cmt0_is_tlbfill_r : cmt1_is_tlbfill_r;
    wire [4:0]  diff_cmt0_fidx = cmt0_valid_r ? cmt0_tlbfill_idx_r : cmt1_tlbfill_idx_r;
    wire        diff_cmt0_rstat = cmt0_valid_r ? cmt0_csr_rstat_r : cmt1_csr_rstat_r;
    wire [31:0] diff_cmt0_cdata = cmt0_valid_r ? cmt0_csr_data_r  : cmt1_csr_data_r;

    DifftestInstrCommit diff_instr_commit0(
        .clock          (clk),
        .coreid         (DIFFTEST_COREID),
        .index          (8'd0),
        .valid          (diff_cmt0_v),
        .pc             ({32'b0, diff_cmt0_pc}),
        .instr          (diff_cmt0_inst),
        .skip           (1'b0),
        .is_TLBFILL     (diff_cmt0_fill),
        .TLBFILL_index  (diff_cmt0_fidx),
        .is_CNTinst     (diff_cmt0_cnt),
        .timer_64_value (diff_cmt0_tmr),
        .wen            (diff_cmt0_wen),
        .wdest          (diff_cmt0_wdest),
        .wdata          ({32'b0, diff_cmt0_wdata}),
        .csr_rstat      (diff_cmt0_v & diff_cmt0_rstat),
        .csr_data       (diff_cmt0_cdata)
    );

    DifftestInstrCommit diff_instr_commit1(
        .clock          (clk),
        .coreid         (DIFFTEST_COREID),
        .index          (8'd1),
        .valid          (diff_cmt1_v),
        .pc             ({32'b0, cmt1_pc_r}),
        .instr          (cmt1_inst_r),
        .skip           (1'b0),
        .is_TLBFILL     (diff_cmt1_v & cmt1_is_tlbfill_r),
        .TLBFILL_index  (cmt1_tlbfill_idx_r),
        .is_CNTinst     (diff_cmt1_v & cmt1_is_cnt_r),
        .timer_64_value (cmt1_timer64_r),
        .wen            (cmt1_wen_r),
        .wdest          (cmt1_wdest_r),
        .wdata          ({32'b0, cmt1_wdata_r}),
        .csr_rstat      (diff_cmt1_v & cmt1_csr_rstat_r),
        .csr_data       (cmt1_csr_data_r)
    );

    DifftestExcpEvent diff_excp_event(
        .clock         (clk),
        .coreid        (DIFFTEST_COREID),
        .excp_valid    (excp_flush_r),
        .eret          (ertn_flush_r),
        .intrNo        (diff_csr_estat[12:2]),
        .cause         ({24'b0, excp_ecode_r}),
        .exceptionPC   ({32'b0, excp_pc_r}),
        .exceptionInst (excp_inst_r)
    );

    DifftestTrapEvent diff_trap_event(
        .clock    (clk),
        .coreid   (DIFFTEST_COREID),
        .valid    (1'b0),
        .code     (3'b0),
        .pc       (64'b0),
        .cycleCnt (64'b0),
        .instrCnt (64'b0)
    );

    DifftestStoreEvent diff_store_event0(
        .clock      (clk),
        .coreid     (DIFFTEST_COREID),
        .index      (8'd0),
        .valid      (diff_st0_en),
        .storePAddr ({32'b0, diff_st0_paddr}),
        .storeVAddr ({32'b0, diff_st0_vaddr}),
        .storeData  ({32'b0, diff_st0_data})
    );

    DifftestStoreEvent diff_store_event1(
        .clock      (clk),
        .coreid     (DIFFTEST_COREID),
        .index      (8'd1),
        .valid      (diff_st1_en),
        .storePAddr ({32'b0, cmt1_st_paddr_r}),
        .storeVAddr ({32'b0, cmt1_st_vaddr_r}),
        .storeData  ({32'b0, diff_st1_data})
    );

    DifftestLoadEvent diff_load_event0(
        .clock  (clk),
        .coreid (DIFFTEST_COREID),
        .index  (8'd0),
        .valid  (diff_cmt0_ld_en),
        .paddr  ({32'b0, diff_cmt0_ld_paddr}),
        .vaddr  ({32'b0, diff_cmt0_ld_vaddr})
    );

    DifftestLoadEvent diff_load_event1(
        .clock  (clk),
        .coreid (DIFFTEST_COREID),
        .index  (8'd1),
        .valid  (diff_cmt1_v ? cmt1_ld_en_r : 8'b0),
        .paddr  ({32'b0, cmt1_ld_paddr_r}),
        .vaddr  ({32'b0, cmt1_ld_vaddr_r})
    );

    DifftestCSRRegState diff_csr_state(
        .clock     (clk),
        .coreid    (DIFFTEST_COREID),
        .crmd      ({32'b0, diff_csr_crmd}),
        .prmd      ({32'b0, diff_csr_prmd}),
        .euen      ({32'b0, diff_csr_euen}),
        .ecfg      ({32'b0, diff_csr_ecfg}),
        .estat     ({32'b0, diff_csr_estat}),
        .era       ({32'b0, diff_csr_era}),
        .badv      ({32'b0, diff_csr_badv}),
        .eentry    ({32'b0, diff_csr_eentry}),
        .tlbidx    ({32'b0, diff_csr_tlbidx}),
        .tlbehi    ({32'b0, diff_csr_tlbehi}),
        .tlbelo0   ({32'b0, diff_csr_tlbelo0}),
        .tlbelo1   ({32'b0, diff_csr_tlbelo1}),
        .asid      ({32'b0, diff_csr_asid}),
        .pgdl      ({32'b0, diff_csr_pgdl}),
        .pgdh      ({32'b0, diff_csr_pgdh}),
        .save0     ({32'b0, diff_csr_save0}),
        .save1     ({32'b0, diff_csr_save1}),
        .save2     ({32'b0, diff_csr_save2}),
        .save3     ({32'b0, diff_csr_save3}),
        .tid       ({32'b0, diff_csr_tid}),
        .tcfg      ({32'b0, diff_csr_tcfg}),
        .tval      ({32'b0, diff_csr_tval}),
        .ticlr     ({32'b0, diff_csr_ticlr}),
        .llbctl    ({32'b0, diff_csr_llbctl}),
        .tlbrentry ({32'b0, diff_csr_tlbrentry}),
        .dmw0      ({32'b0, diff_csr_dmw0}),
        .dmw1      ({32'b0, diff_csr_dmw1})
    );

    DifftestGRegState diff_greg_state(
        .clock  (clk),
        .coreid (DIFFTEST_COREID),
        .gpr_0  ({32'b0, diff_gpr_0}),
        .gpr_1  ({32'b0, diff_gpr_1}),
        .gpr_2  ({32'b0, diff_gpr_2}),
        .gpr_3  ({32'b0, diff_gpr_3}),
        .gpr_4  ({32'b0, diff_gpr_4}),
        .gpr_5  ({32'b0, diff_gpr_5}),
        .gpr_6  ({32'b0, diff_gpr_6}),
        .gpr_7  ({32'b0, diff_gpr_7}),
        .gpr_8  ({32'b0, diff_gpr_8}),
        .gpr_9  ({32'b0, diff_gpr_9}),
        .gpr_10 ({32'b0, diff_gpr_10}),
        .gpr_11 ({32'b0, diff_gpr_11}),
        .gpr_12 ({32'b0, diff_gpr_12}),
        .gpr_13 ({32'b0, diff_gpr_13}),
        .gpr_14 ({32'b0, diff_gpr_14}),
        .gpr_15 ({32'b0, diff_gpr_15}),
        .gpr_16 ({32'b0, diff_gpr_16}),
        .gpr_17 ({32'b0, diff_gpr_17}),
        .gpr_18 ({32'b0, diff_gpr_18}),
        .gpr_19 ({32'b0, diff_gpr_19}),
        .gpr_20 ({32'b0, diff_gpr_20}),
        .gpr_21 ({32'b0, diff_gpr_21}),
        .gpr_22 ({32'b0, diff_gpr_22}),
        .gpr_23 ({32'b0, diff_gpr_23}),
        .gpr_24 ({32'b0, diff_gpr_24}),
        .gpr_25 ({32'b0, diff_gpr_25}),
        .gpr_26 ({32'b0, diff_gpr_26}),
        .gpr_27 ({32'b0, diff_gpr_27}),
        .gpr_28 ({32'b0, diff_gpr_28}),
        .gpr_29 ({32'b0, diff_gpr_29}),
        .gpr_30 ({32'b0, diff_gpr_30}),
        .gpr_31 ({32'b0, diff_gpr_31})
    );
`endif

`ifndef SYNTHESIS
// synthesis translate_off
// 性能仿真结束时打印前端/cache/IPC（无 digftest 时 testbench 的 inst_total 恒为 0）
reg [63:0] perf_cycle_count;
reg [63:0] perf_retire_count;
reg [63:0] stall_rob_full_cyc;
reg [63:0] stall_sb_full_cyc;
reg [63:0] perf_cmt_dual_cyc;     // 双提交拍数
reg [63:0] perf_cmt_any_cyc;      // 至少提交 1 条的拍数
reg [63:0] perf_dis_dual_cyc;     // 同拍双分发
reg [63:0] perf_dis_any_cyc;      // 至少分发 1 条
reg [63:0] perf_dis_dual_alu_cyc; // 同拍双 ALU 入站（alu0+alu1 push）
reg [63:0] perf_rn_alloc_cyc;      // rename 成功分配 ROB/装入 dispatch
reg [63:0] perf_turnover_refill_cyc; // dispatch 有旧指令时同拍腾空并 refill
reg [63:0] stall_dispatch_cyc;     // IB 有指令但 dispatch 无法整体腾空
reg [63:0] stall_rename_rob_cyc;   // IB 有指令但 ROB 达到安全满阈值

// ROB 有效项数不能只看 head/tail（每对可有 0/1/2 条有效指令），按分配与
// 提交事件维护；flush 只清当前占用，不清整次仿真的累计统计。
reg [5:0] probe_rob_entry_occ;
wire [1:0] probe_rob_alloc_n = rn_rob_alloc_en
                             ? ({1'b0, rn_a0_valid} + {1'b0, rn_a1_valid})
                             : 2'd0;
wire [1:0] probe_rob_clear_n = {1'b0, cmt_rob_clear0}
                             + {1'b0, cmt_rob_clear1};

always @(posedge clk) begin
    if (reset || flush)
        probe_rob_entry_occ <= 6'd0;
    else
        probe_rob_entry_occ <= probe_rob_entry_occ
                             + {4'd0, probe_rob_alloc_n}
                             - {4'd0, probe_rob_clear_n};
end

function automatic [5:0] perf_popcount32;
    input [31:0] value;
    integer pc_i;
    begin
        perf_popcount32 = 6'd0;
        for (pc_i = 0; pc_i < 32; pc_i = pc_i + 1)
            perf_popcount32 = perf_popcount32 + {5'd0, value[pc_i]};
    end
endfunction

// 统一队列占用采样。所有数组和层次探针均处于 translate_off 区域，
// 不参与综合，不消耗 FPGA LUT/FF/BRAM。
localparam PERF_OCC_N = 14;
reg [7:0]  perf_occ_now      [0:PERF_OCC_N-1];
reg [7:0]  perf_occ_cap      [0:PERF_OCC_N-1];
reg [7:0]  perf_occ_max      [0:PERF_OCC_N-1];
reg [63:0] perf_occ_sum      [0:PERF_OCC_N-1];
reg [63:0] perf_occ_high_cyc [0:PERF_OCC_N-1]; // >= 75% 有效容量
reg [63:0] perf_occ_full_cyc [0:PERF_OCC_N-1];
integer perf_occ_i;

always @(*) begin
    // ROB 物理 16 对，因 ROB_GUARD=5，实际最多允许 11 对/22 条在途。
    perf_occ_now[0]  = {{(8-`ROB_PAIR_W){1'b0}},
                        (u_rob.tail - u_rob.head)};
    perf_occ_cap[0]  = (1 << `ROB_PAIR_W) - `ROB_GUARD;
    perf_occ_now[1]  = {2'd0, probe_rob_entry_occ};
    perf_occ_cap[1]  = 2 * ((1 << `ROB_PAIR_W) - `ROB_GUARD);
    perf_occ_now[2]  = {7'd0, dis0_valid} + {7'd0, dis1_valid};
    perf_occ_cap[2]  = 8'd2;
    perf_occ_now[3]  = {{(8-`RS_ALU_OCC_W){1'b0}}, rsa0_occupancy};
    perf_occ_cap[3]  = `RS_ALU_SIZE;
    perf_occ_now[4]  = {{(8-`RS_ALU_OCC_W){1'b0}}, rsa1_occupancy};
    perf_occ_cap[4]  = `RS_ALU_SIZE;
    perf_occ_now[5]  = {{(8-`RS_MEM_OCC_W){1'b0}}, rsm_occupancy};
    perf_occ_cap[5]  = `RS_MEM_SIZE;
    perf_occ_now[6]  = {6'd0, rsd_occupancy};
    perf_occ_cap[6]  = `RS_MDU_SIZE;
    perf_occ_now[7]  = {{(7-`SB_W){1'b0}}, u_store_buffer.count};
    perf_occ_cap[7]  = `SB_SIZE;
    perf_occ_now[8]  = {{(7-`IB_W){1'b0}}, u_inst_buffer.count};
    perf_occ_cap[8]  = `IB_SIZE;
    perf_occ_now[9]  = {{(8-`FTQ_W){1'b0}},
                        (u_ftq.bpu_ptr - u_ftq.cmt_ptr)};
    // FTQ almost-full 在保留项达到 SIZE-2 时反压，故以 14 为有效容量。
    perf_occ_cap[9]  = `FTQ_SIZE - 2;
    perf_occ_now[10] = {{(8-`FTQ_W){1'b0}},
                        (u_ftq.bpu_ptr - u_ftq.ifu_ptr)};
    perf_occ_cap[10] = `FTQ_SIZE;
    perf_occ_now[11] = {2'd0, perf_popcount32(u_rat.busy)};
    perf_occ_cap[11] = 8'd31; // r0 永不 busy
    perf_occ_now[12] = {{(7-`RAS_W){1'b0}}, u_bpu.u_ras.spec_cnt};
    perf_occ_cap[12] = `RAS_DEPTH;
    perf_occ_now[13] = {{(7-`RAS_W){1'b0}}, u_bpu.u_ras.cmt_cnt};
    perf_occ_cap[13] = `RAS_DEPTH;
end

always @(posedge clk) begin
    if (reset) begin
        perf_cycle_count      <= 64'd0;
        perf_retire_count     <= 64'd0;
        stall_rob_full_cyc    <= 64'd0;
        stall_sb_full_cyc     <= 64'd0;
        perf_cmt_dual_cyc     <= 64'd0;
        perf_cmt_any_cyc      <= 64'd0;
        perf_dis_dual_cyc     <= 64'd0;
        perf_dis_any_cyc      <= 64'd0;
        perf_dis_dual_alu_cyc <= 64'd0;
        perf_rn_alloc_cyc      <= 64'd0;
        perf_turnover_refill_cyc <= 64'd0;
        stall_dispatch_cyc     <= 64'd0;
        stall_rename_rob_cyc   <= 64'd0;
        for (perf_occ_i = 0; perf_occ_i < PERF_OCC_N; perf_occ_i = perf_occ_i + 1) begin
            perf_occ_max[perf_occ_i]      <= 8'd0;
            perf_occ_sum[perf_occ_i]      <= 64'd0;
            perf_occ_high_cyc[perf_occ_i] <= 64'd0;
            perf_occ_full_cyc[perf_occ_i] <= 64'd0;
        end
    end else begin
        perf_cycle_count  <= perf_cycle_count + 64'd1;
        perf_retire_count <= perf_retire_count
                           + {63'd0, cmt_dbg0_valid}
                           + {63'd0, cmt_dbg1_valid};
        if (rob_full)
            stall_rob_full_cyc <= stall_rob_full_cyc + 64'd1;
        if (sb_full)
            stall_sb_full_cyc <= stall_sb_full_cyc + 64'd1;
        if (cmt_dbg0_valid && cmt_dbg1_valid)
            perf_cmt_dual_cyc <= perf_cmt_dual_cyc + 64'd1;
        if (cmt_dbg0_valid || cmt_dbg1_valid)
            perf_cmt_any_cyc <= perf_cmt_any_cyc + 64'd1;
        if (dis0_fire && dis1_fire)
            perf_dis_dual_cyc <= perf_dis_dual_cyc + 64'd1;
        if (dis0_fire || dis1_fire)
            perf_dis_any_cyc <= perf_dis_any_cyc + 64'd1;
        if (u_dispatch.rs_alu0_push_valid_o && u_dispatch.rs_alu1_push_valid_o)
            perf_dis_dual_alu_cyc <= perf_dis_dual_alu_cyc + 64'd1;
        if (rn_rob_alloc_en)
            perf_rn_alloc_cyc <= perf_rn_alloc_cyc + 64'd1;
        if (rn_rob_alloc_en && (dis0_valid || dis1_valid))
            perf_turnover_refill_cyc <= perf_turnover_refill_cyc + 64'd1;
        if ((ib_pop0_valid || ib_pop1_valid) && !dispatch_ready && !rob_full)
            stall_dispatch_cyc <= stall_dispatch_cyc + 64'd1;
        if ((ib_pop0_valid || ib_pop1_valid) && rob_full)
            stall_rename_rob_cyc <= stall_rename_rob_cyc + 64'd1;
        for (perf_occ_i = 0; perf_occ_i < PERF_OCC_N; perf_occ_i = perf_occ_i + 1) begin
            perf_occ_sum[perf_occ_i] <= perf_occ_sum[perf_occ_i]
                                      + {56'd0, perf_occ_now[perf_occ_i]};
            if (perf_occ_now[perf_occ_i] > perf_occ_max[perf_occ_i])
                perf_occ_max[perf_occ_i] <= perf_occ_now[perf_occ_i];
            if ((perf_occ_now[perf_occ_i] * 4) >= (perf_occ_cap[perf_occ_i] * 3))
                perf_occ_high_cyc[perf_occ_i] <= perf_occ_high_cyc[perf_occ_i] + 64'd1;
            if (perf_occ_now[perf_occ_i] >= perf_occ_cap[perf_occ_i])
                perf_occ_full_cyc[perf_occ_i] <= perf_occ_full_cyc[perf_occ_i] + 64'd1;
        end
    end
end

function automatic real perf_rate;
    input [63:0] num;
    input [63:0] den;
    begin
        if (den == 64'd0)
            perf_rate = 0.0;
        else
            perf_rate = 100.0 * num / den;
    end
endfunction

function automatic real perf_avg_occ;
    input [63:0] sum;
    input [63:0] cycles;
    begin
        if (cycles == 64'd0)
            perf_avg_occ = 0.0;
        else
            perf_avg_occ = 1.0 * sum / cycles;
    end
endfunction

function automatic real perf_ipc;
    input [63:0] insts;
    input [63:0] cycles;
    begin
        if (cycles == 64'd0)
            perf_ipc = 0.0;
        else
            perf_ipc = 1.0 * insts / cycles;
    end
endfunction

final begin
    $display("");
    $display("==================== myCPU PERF (sim) ====================");
    $display("Commit IPC:         retire=%0d  cycles=%0d  IPC=%.6f",
             perf_retire_count, perf_cycle_count,
             perf_ipc(perf_retire_count, perf_cycle_count));
    $display("Commit dual-issue:  dual_cyc=%0d  any_cyc=%0d  dual_rate=%.2f%%  (of commit cycles)",
             perf_cmt_dual_cyc, perf_cmt_any_cyc,
             perf_rate(perf_cmt_dual_cyc, perf_cmt_any_cyc));
    $display("  vs all cycles:    dual_rate=%.2f%%  IPC_util=%.2f%% of 2.0 peak",
             perf_rate(perf_cmt_dual_cyc, perf_cycle_count),
             perf_rate(perf_retire_count, perf_cycle_count * 2));
    $display("Dispatch dual:      dual_cyc=%0d  any_cyc=%0d  dual_rate=%.2f%%  dual_ALU=%0d",
             perf_dis_dual_cyc, perf_dis_any_cyc,
             perf_rate(perf_dis_dual_cyc, perf_dis_any_cyc),
             perf_dis_dual_alu_cyc);
    $display("Rename/dispatch:    alloc_cyc=%0d  turnover_refill=%0d  dispatch_stall=%0d  rob_stall=%0d",
             perf_rn_alloc_cyc, perf_turnover_refill_cyc,
             stall_dispatch_cyc, stall_rename_rob_cyc);
    $display("BPU all-branch:     total=%0d  mispred=%0d  accuracy=%.2f%%",
             u_bpu.commit_all_branch_count,
             u_bpu.commit_all_mispred_count,
             perf_rate(u_bpu.commit_all_branch_count - u_bpu.commit_all_mispred_count,
                       u_bpu.commit_all_branch_count));
    $display("BPU cond-branch:    total=%0d  mispred=%0d  accuracy=%.2f%%",
             u_bpu.commit_cond_branch_count,
             u_bpu.commit_cond_mispred_count,
             perf_rate(u_bpu.commit_cond_branch_count - u_bpu.commit_cond_mispred_count,
                       u_bpu.commit_cond_branch_count));
    $display("FTB (P1 response):  resp=%0d  hit=%0d  hit_rate=%.2f%%",
             u_bpu.u_ftb.ftb_response_total,
             u_bpu.u_ftb.ftb_hit_total,
             perf_rate(u_bpu.u_ftb.ftb_hit_total, u_bpu.u_ftb.ftb_response_total));
    $display("ICache (cached):    access=%0d  hit=%0d  hit_rate=%.2f%%",
             u_icache.ic_access_total,
             u_icache.ic_hit_total,
             perf_rate(u_icache.ic_hit_total, u_icache.ic_access_total));
    $display("DCache (cached):    access=%0d  hit=%0d  hit_rate=%.2f%%",
             u_dcache.dc_access_total,
             u_dcache.dc_hit_total,
             perf_rate(u_dcache.dc_hit_total, u_dcache.dc_access_total));
    $display("  DCache load:      access=%0d  hit=%0d  hit_rate=%.2f%%",
             u_dcache.dc_ld_access_total,
             u_dcache.dc_ld_hit_total,
             perf_rate(u_dcache.dc_ld_hit_total, u_dcache.dc_ld_access_total));
    $display("  DCache store:     access=%0d  hit=%0d  hit_rate=%.2f%%",
             u_dcache.dc_st_access_total,
             u_dcache.dc_st_hit_total,
             perf_rate(u_dcache.dc_st_hit_total, u_dcache.dc_st_access_total));
    $display("---- stall / mem ----");
    $display("ROB full cycles:    %0d  (%.2f%%)",
             stall_rob_full_cyc, perf_rate(stall_rob_full_cyc, perf_cycle_count));
    $display("SB  full cycles:    %0d  (%.2f%%)",
             stall_sb_full_cyc, perf_rate(stall_sb_full_cyc, perf_cycle_count));
    $display("D$ MWAIT cycles:    %0d  (%.2f%%)",
             u_dcache.dc_mwait_cycles, perf_rate(u_dcache.dc_mwait_cycles, perf_cycle_count));
    $display("D$ pend cycles:     %0d  (%.2f%%)  push=%0d",
             u_dcache.dc_pend_cycles, perf_rate(u_dcache.dc_pend_cycles, perf_cycle_count),
             u_dcache.dc_pend_push_total);
    $display("D$ MSHR busy:       %0d  (%.2f%%)",
             u_dcache.dc_mshr_busy_cycles, perf_rate(u_dcache.dc_mshr_busy_cycles, perf_cycle_count));
    $display("LSU store-order:    %0d  (%.2f%%)",
             u_lsu.lsu_store_order_stall_cyc,
             perf_rate(u_lsu.lsu_store_order_stall_cyc, perf_cycle_count));
    $display("LSU DC wait:        %0d  (%.2f%%)",
             u_lsu.lsu_dc_wait_cyc, perf_rate(u_lsu.lsu_dc_wait_cyc, perf_cycle_count));
    $display("RS_MEM src stall:   %0d  (%.2f%%)",
             u_rs_mem.rsm_src_stall_cyc, perf_rate(u_rs_mem.rsm_src_stall_cyc, perf_cycle_count));
    $display("RS_MEM LSU stall:   %0d  (%.2f%%)",
             u_rs_mem.rsm_lsu_stall_cyc, perf_rate(u_rs_mem.rsm_lsu_stall_cyc, perf_cycle_count));
    $display("RS_MEM full stall:  %0d  (%.2f%%)",
             u_rs_mem.rsm_full_stall_cyc, perf_rate(u_rs_mem.rsm_full_stall_cyc, perf_cycle_count));
    $display("---- queue occupancy: avg / max / effective-cap / >=75%% cycles / full cycles ----");
    $display("ROB pairs:          %.3f / %0d / %0d / %0d / %0d",
             perf_avg_occ(perf_occ_sum[0], perf_cycle_count), perf_occ_max[0], perf_occ_cap[0],
             perf_occ_high_cyc[0], perf_occ_full_cyc[0]);
    $display("ROB valid entries:  %.3f / %0d / %0d / %0d / %0d",
             perf_avg_occ(perf_occ_sum[1], perf_cycle_count), perf_occ_max[1], perf_occ_cap[1],
             perf_occ_high_cyc[1], perf_occ_full_cyc[1]);
    $display("Dispatch slots:     %.3f / %0d / %0d / %0d / %0d",
             perf_avg_occ(perf_occ_sum[2], perf_cycle_count), perf_occ_max[2], perf_occ_cap[2],
             perf_occ_high_cyc[2], perf_occ_full_cyc[2]);
    $display("RS ALU0:            %.3f / %0d / %0d / %0d / %0d",
             perf_avg_occ(perf_occ_sum[3], perf_cycle_count), perf_occ_max[3], perf_occ_cap[3],
             perf_occ_high_cyc[3], perf_occ_full_cyc[3]);
    $display("RS ALU1:            %.3f / %0d / %0d / %0d / %0d",
             perf_avg_occ(perf_occ_sum[4], perf_cycle_count), perf_occ_max[4], perf_occ_cap[4],
             perf_occ_high_cyc[4], perf_occ_full_cyc[4]);
    $display("RS MEM:             %.3f / %0d / %0d / %0d / %0d",
             perf_avg_occ(perf_occ_sum[5], perf_cycle_count), perf_occ_max[5], perf_occ_cap[5],
             perf_occ_high_cyc[5], perf_occ_full_cyc[5]);
    $display("RS MDU:             %.3f / %0d / %0d / %0d / %0d",
             perf_avg_occ(perf_occ_sum[6], perf_cycle_count), perf_occ_max[6], perf_occ_cap[6],
             perf_occ_high_cyc[6], perf_occ_full_cyc[6]);
    $display("Store buffer:       %.3f / %0d / %0d / %0d / %0d",
             perf_avg_occ(perf_occ_sum[7], perf_cycle_count), perf_occ_max[7], perf_occ_cap[7],
             perf_occ_high_cyc[7], perf_occ_full_cyc[7]);
    $display("Instruction buffer: %.3f / %0d / %0d / %0d / %0d",
             perf_avg_occ(perf_occ_sum[8], perf_cycle_count), perf_occ_max[8], perf_occ_cap[8],
             perf_occ_high_cyc[8], perf_occ_full_cyc[8]);
    $display("FTQ retained:       %.3f / %0d / %0d / %0d / %0d",
             perf_avg_occ(perf_occ_sum[9], perf_cycle_count), perf_occ_max[9], perf_occ_cap[9],
             perf_occ_high_cyc[9], perf_occ_full_cyc[9]);
    $display("FTQ to IFU pending: %.3f / %0d / %0d / %0d / %0d",
             perf_avg_occ(perf_occ_sum[10], perf_cycle_count), perf_occ_max[10], perf_occ_cap[10],
             perf_occ_high_cyc[10], perf_occ_full_cyc[10]);
    $display("RAT busy mappings:  %.3f / %0d / %0d / %0d / %0d",
             perf_avg_occ(perf_occ_sum[11], perf_cycle_count), perf_occ_max[11], perf_occ_cap[11],
             perf_occ_high_cyc[11], perf_occ_full_cyc[11]);
    $display("RAS speculative:    %.3f / %0d / %0d / %0d / %0d",
             perf_avg_occ(perf_occ_sum[12], perf_cycle_count), perf_occ_max[12], perf_occ_cap[12],
             perf_occ_high_cyc[12], perf_occ_full_cyc[12]);
    $display("RAS committed:      %.3f / %0d / %0d / %0d / %0d",
             perf_avg_occ(perf_occ_sum[13], perf_cycle_count), perf_occ_max[13], perf_occ_cap[13],
             perf_occ_high_cyc[13], perf_occ_full_cyc[13]);
    $display("FTB update queue:   max=%0d/%0d  overflow=%0d",
             u_bpu.u_ftb.ftb_update_queue_max_occupancy, `FTB_UPDATE_Q_DEPTH,
             u_bpu.u_ftb.ftb_update_overflow_count);
    $display("TAGE update queue:  max=%0d/%0d  overflow=%0d  pipe_max=%0d",
             u_bpu.u_tage.tage_update_queue_max_occupancy, `TAGE_UPDATE_Q_DEPTH,
             u_bpu.u_tage.tage_update_overflow_count,
             u_bpu.u_tage.tage_update_pipeline_max_pending_count);
    $display("==========================================================");
    $display("");
end
// synthesis translate_on
`endif

endmodule
