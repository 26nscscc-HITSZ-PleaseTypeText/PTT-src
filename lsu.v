// ============================================================
// lsu 模块（访存单元：AGU + DC 两级流水 + miss 槽非阻塞 load）
// ------------------------------------------------------------
// 结构：
// - issue→AGU 边界先计算并锁存 vaddr=base+imm；AGU 级只做 MMU 组合翻译、
//   ALE/ADEM/TLB 异常与 store 对齐，避免地址加法和 TLB 搜索串在同一拍 ->
//   store 数据按地址对齐 + wstrb 生成；
// - DC 级：store/cacop/异常直接写回；load 先查 SB 前递，再访 DCache；
// - miss 槽（深度 `LSU_MISS_DEPTH`）：cached miss 移入槽，robid 配对返回；
// - 写回仲裁：miss 槽 > hold > DC 级；SB/DC 命中经 hold 打拍。
//
// 顺序保护（P1b：STQ + 提交后 SB）：
// - store 写回后入 STQ（深度 `STQ_DEPTH`），提交前挡重叠 / UC 全局互斥；
// - 已提交 store 经 commit 推 SB；字节 overlap 与 SB 共用 mycpu.h 中 mem_* 函数。
//
// 冲刷（flush_i）：
// - 清 AGU/DC/hold/STQ/UC-park（SB 不清：已提交）；
// - d_drop 丢弃在途前端响应；miss 槽置 m_drop 等 mshr_data_ok。
//
// 年轻 UC park：比 UC 更老的 AGU 可让出 DC（宽版，见流水推进注释）。
// ============================================================
`include "mycpu.h"

module lsu(
    input  wire                       clk,
    input  wire                       reset,
    input  wire                       flush_i,

    // ---------------- 发射入口（来自 rs_mem）----------------
    input  wire                       issue_valid_i,
    input  wire [`ROB_W-1:0]          issue_robid_i,
    input  wire [`MEM_OP_NUM-1:0]     issue_mem_op_i,
    input  wire                       issue_is_cacop_i,
    input  wire [4:3]                 issue_cacop_op_i,
    input  wire [31:0]                issue_base_i,        // rj 值
    input  wire                       issue_base_ready_i,
    input  wire [`ROB_W-1:0]          issue_base_robid_i,
    input  wire [31:0]                issue_wdata_i,       // rd 值（store 数据）
    input  wire                       issue_wdata_ready_i,
    input  wire [`ROB_W-1:0]          issue_wdata_robid_i,
    input  wire [31:0]                issue_imm_i,         // 偏移
    output wire                       lsu_ready_o,         // AGU 级可接收
    output wire                       lsu_plain_only_o,
    input  wire                       issue_pair_valid_i,
    input  wire [`ROB_W-1:0]          issue_pair_robid_i,
    input  wire [`MEM_OP_NUM-1:0]     issue_pair_mem_op_i,
    input  wire [31:0]                issue_pair_base_i,
    input  wire [31:0]                issue_pair_imm_i,
    output wire                       lsu_pair_ready_o,

    // ---------------- D 侧地址翻译（连 mmu D 通道，组合）----------------
    output wire                       mmu_d_req_o,
    output wire [31:0]                mmu_d_vaddr_o,
    output wire                       mmu_d_is_store_o,    // 区分 PIL/PIS 与 PME
    input  wire [31:0]                mmu_d_paddr_i,
    input  wire [1:0]                 mmu_d_mat_i,
    input  wire                       mmu_d_ready_i,
    input  wire                       mmu_d_excp_tlbr_i,
    input  wire                       mmu_d_excp_pil_i,
    input  wire                       mmu_d_excp_pis_i,
    input  wire                       mmu_d_excp_ppi_i,
    input  wire                       mmu_d_excp_pme_i,
    input  wire                       mmu_d_excp_adem_i,

    // ---------------- DCache load 访问口 ----------------
    output wire                       dc_req_o,            // load 请求（保持至 addr_ok）
    output wire [11:5]                dc_vindex_o,         // 虚地址页内索引（VIPT）
    output wire [31:0]                dc_paddr_o,          // 物理地址（tag 比对）
    output wire [2:0]                 dc_size_o,           // 0=B 1=H 2=W
    output wire [7:4]                 dc_load_op_o,
    output wire                       dc_uncached_o,
    output wire [`ROB_W-1:0]          dc_robid_o,          // 随 dc_req：D$ miss 锁存配对
    input  wire                       dc_addr_ok_i,
    input  wire                       dc_data_ok_i,
    input  wire [31:0]                dc_rdata_i,
    input  wire [`ROB_W-1:0]          dc_resp_robid_i,
    output wire                       dc_resp_ready_o,
    output wire                       dc_cancel_o,         // 冲刷：通知 dcache 杀 load MSHR
    // ---- 非阻塞 miss 扩展（配合 dcache MSHR）----
    input  wire                       dc_miss_i,           // 在途 load 移入 MSHR（一拍）
    input  wire                       dc_mshr_data_ok_i,   // MSHR 重填数据返回（一拍）
    input  wire [31:0]                dc_mshr_rdata_i,
    input  wire [`ROB_W-1:0]          dc_mshr_robid_i,     // 与 data_ok 同拍
    output wire                       dc_pair_valid_o,
    output wire [2:0]                 dc_pair_word_o,
    output wire [31:0]                dc_pair_paddr_o,
    input  wire                       dc_pair_data_ok_i,
    input  wire                       dc_pair_miss_i,
    input  wire [31:0]                dc_pair_rdata_i,

    // ---------------- store buffer 前递查询（DC 级，一拍返回）----------------
    output wire                       sb_query_valid_o,
    output wire [31:2]                sb_query_paddr_o,
    output wire                       sb_query_uncached_o, // 本查询来自 uncached load
    input  wire                       sb_query_resp_valid_i,
    input  wire [31:2]                sb_query_resp_paddr_i,
    input  wire                       sb_query_resp_uncached_i,
    input  wire                       sb_query_maybe_i,
    input  wire                       sb_query_hit_i,      // 整字可由 SB 合并前递
    input  wire [31:0]                sb_query_data_i,
    input  wire                       sb_query_partial_i,  // 部分命中：load 等排空重试
    input  wire                       sb_empty_i,
    output wire [31:2]                sb_pair_query_paddr_o,
    input  wire                       sb_pair_query_maybe_i,

    // ---------------- uncached load 许可（与 ROB head 比较）----------------
    input  wire [`ROB_W-1:0]          rob_head_robid_i,    // 编码：MSB=槽0 仍未提交，低位=head 对指针（顶层拼装，见 mycpu_top）
    input  wire                       rob_head_valid_i,
    // store 提交释放 STQ：与 SB push 同源；LSU 再打一拍，保证 SB valid 已可见
    input  wire                       st_retire_valid_i,
    input  wire [`ROB_W-1:0]          st_retire_robid_i,
    output wire [`ROB_W-1:0]          a_base_probe_robid_o,
    input  wire                       rob_a_base_ready_i,
    input  wire [31:0]                rob_a_base_data_i,
    output wire [`ROB_W-1:0]          q_base_probe_robid_o,
    input  wire                       rob_q_base_ready_i,
    input  wire [31:0]                rob_q_base_data_i,
    output wire                       uncached_ld_inflight_o, // 有 uncached load 在飞（commit 屏蔽中断用）

    // ---------------- 写回 ROB ----------------
    output wire                       wb_valid_o,
    output wire [`ROB_W-1:0]          wb_robid_o,
    output wire [31:0]                wb_data_o,           // load 整形后数据 / store 写数据
    output wire [31:0]                wb_paddr_o,          // 访存物理地址（store/cacop/difftest 用）
    output wire [31:0]                wb_vaddr_o,          // 访存虚地址（BADV/difftest 用）
    output wire [3:0]                 wb_wstrb_o,          // store 字节使能（已按地址对齐移位）
    output wire [2:0]                 wb_size_o,           // 访问宽度
    output wire                       wb_uncached_o,       // 非缓存访问
    output wire [`EXCP_NUM-1:0]       wb_excp_o,           // 动态异常（ALE/ADEM/TLBR_M/PIL/PIS/PPI_M/PME）
    output wire                       pair_wb_valid_o,
    output wire [`ROB_W-1:0]          pair_wb_robid_o,
    output wire [31:0]                pair_wb_data_o,
    output wire [31:0]                pair_wb_paddr_o,
    output wire [31:0]                pair_wb_vaddr_o,
    output wire [2:0]                 pair_wb_size_o,

    // Dedicated same-cycle cached load-hit bypass.  Miss/hold/store
    // completions remain exclusively on the normal writeback path.
    output wire                       fast_wb_valid_o,
    output wire [`ROB_W-1:0]          fast_wb_robid_o,
    output wire [31:0]                fast_wb_data_o,
    // Timing-specialized MDU bypass: only a cached ld.w can use the raw
    // DCache word.  Narrow loads retain the normal shaped writeback path.
    output wire                       fast_mdu_valid_o,
    output wire [`ROB_W-1:0]          fast_mdu_robid_o,
    output wire [31:0]                fast_mdu_data_o,
    // Registered miss-shadow descriptors.  RS_MEM uses only these tags to
    // decide whether an oldest plain store may send its address early.
    output wire [`LSU_MISS_DEPTH-1:0] miss_wait_valid_o,
    output wire [(`LSU_MISS_DEPTH*`ROB_W)-1:0] miss_wait_robids_o,
    // Registered producer descriptors for cached ordinary loads that have
    // already been accepted by DCache but have not returned hit/miss yet.
    // RS_MEM may use only these token registers (never current DCache control)
    // to hand the oldest dependent ordinary store to LSU with data-not-ready.
    output wire [`LSU_TOKEN_DEPTH-1:0] pending_load_valid_o,
    output wire [(`LSU_TOKEN_DEPTH*`ROB_W)-1:0] pending_load_robids_o,
    // Registered hold-only subset for timing-sensitive consumers such as MDU.
    output wire                       hold_wb_valid_o,
    output wire [`ROB_W-1:0]          hold_wb_robid_o,
    output wire [31:0]                hold_wb_data_o,

    // ---------------- DC 级命中限定早唤醒（顶层可选择打一拍）----------------
    output wire                       early_wakeup_valid_o,
    output wire [`ROB_W-1:0]          early_wakeup_robid_o
);

// =====================================================================
// AGU 级
// =====================================================================
reg                    a_valid;
// A-stage advance is resolved late (translation plus downstream capacity).
// Preload the inactive payload bank every cycle and make that late decision
// toggle only one bit, as already done at the A->D boundary below.
(* EXTRACT_ENABLE = "no" *) reg a_payload_sel;
reg [`ROB_W-1:0]       a_robid_bank0, a_robid_bank1;
reg [`MEM_OP_NUM-1:0]  a_mem_op_bank0, a_mem_op_bank1;
reg                    a_is_cacop_bank0, a_is_cacop_bank1;
reg [4:3]              a_cacop_op_bank0, a_cacop_op_bank1;
reg [31:0]             a_base_bank0, a_base_bank1;
reg [31:0]             a_imm_bank0, a_imm_bank1;
reg [31:0]             a_wdata_bank0, a_wdata_bank1;
// A ready source may legally retain a stale producer tag, so a coincident
// refill-tag match cannot select the 32-bit payload combinationally.  Preload
// both candidates and register only the choice beside each payload bank.
reg [31:0]             a_wdata_refill_bank0, a_wdata_refill_bank1;
reg                    a_wdata_refill_sel_bank0, a_wdata_refill_sel_bank1;
reg                    a_base_ready_bank0, a_base_ready_bank1;
reg [`ROB_W-1:0]       a_base_robid_bank0, a_base_robid_bank1;
reg                    a_wdata_ready_bank0, a_wdata_ready_bank1;
reg [`ROB_W-1:0]       a_wdata_robid_bank0, a_wdata_robid_bank1;
wire [`ROB_W-1:0]      a_robid = a_payload_sel ? a_robid_bank1 : a_robid_bank0;
wire [`MEM_OP_NUM-1:0] a_mem_op = a_payload_sel ? a_mem_op_bank1 : a_mem_op_bank0;
wire                   a_is_cacop = a_payload_sel ? a_is_cacop_bank1 : a_is_cacop_bank0;
wire [4:3]             a_cacop_op = a_payload_sel ? a_cacop_op_bank1 : a_cacop_op_bank0;
wire [31:0]            a_base = a_payload_sel ? a_base_bank1 : a_base_bank0;
wire [31:0]            a_imm = a_payload_sel ? a_imm_bank1 : a_imm_bank0;
wire [31:0]            a_wdata_primary = a_payload_sel ? a_wdata_bank1
                                                       : a_wdata_bank0;
wire [31:0]            a_wdata_refill = a_payload_sel ? a_wdata_refill_bank1
                                                      : a_wdata_refill_bank0;
wire                   a_wdata_refill_sel = a_payload_sel
                                               ? a_wdata_refill_sel_bank1
                                               : a_wdata_refill_sel_bank0;
wire [31:0]            a_wdata = a_wdata_refill_sel ? a_wdata_refill
                                                     : a_wdata_primary;
wire                   a_base_ready = a_payload_sel ? a_base_ready_bank1 : a_base_ready_bank0;
wire [`ROB_W-1:0]      a_base_robid = a_payload_sel ? a_base_robid_bank1 : a_base_robid_bank0;
wire                   a_wdata_ready = a_payload_sel ? a_wdata_ready_bank1 : a_wdata_ready_bank0;
wire [`ROB_W-1:0]      a_wdata_robid = a_payload_sel ? a_wdata_robid_bank1 : a_wdata_robid_bank0;
wire [31:0]            a_vaddr;

// One-entry issue skid. LSU ready depends only on this register, so the
// store-buffer forwarding cone cannot reach back into rs_mem.
reg                    q_valid;
reg [`ROB_W-1:0]       q_robid;
reg [`MEM_OP_NUM-1:0]  q_mem_op;
reg                    q_is_cacop;
reg [4:3]              q_cacop_op;
reg [31:0]             q_base, q_imm, q_wdata;
reg [31:0]             q_wdata_refill;
reg                    q_wdata_refill_sel;
reg                    q_base_ready;
reg [`ROB_W-1:0]       q_base_robid;
reg                    q_wdata_ready;
reg [`ROB_W-1:0]       q_wdata_robid;

// One fused-load ownership slot.  The second load remains here until it
// either returns from the first load's cache lookup or is injected back into
// the normal AGU path.  This makes every fallback lossless.
reg                    pair_slot_valid;
reg                    pair_wait_resp;
reg                    pair_replay;
(* EXTRACT_RESET = "yes", EXTRACT_ENABLE = "yes" *) reg pair_agu_replay;
reg [`ROB_W-1:0]       pair_first_robid;
reg [`ROB_W-1:0]       pair_slot_robid;
reg [`MEM_OP_NUM-1:0]  pair_slot_mem_op;
reg [31:0]             pair_slot_vaddr;
reg [31:0]             pair_slot_paddr;

// Registered second completion.  No raw DCache signal is allowed to fan out
// into ROB/RS wakeup logic.
reg                    ph_valid;
reg [`ROB_W-1:0]       ph_robid;
reg [31:0]             ph_data;
reg [31:0]             ph_vaddr, ph_paddr;
reg [2:0]              ph_size;

wire a_is_store_op = a_mem_op[`MEM_OP_ST_W] | a_mem_op[`MEM_OP_ST_B]
                   | a_mem_op[`MEM_OP_ST_H] | a_mem_op[`MEM_OP_SC_W];
wire a_is_load_op  = a_mem_op[`MEM_OP_LD_W] | a_mem_op[`MEM_OP_LD_B]
                   | a_mem_op[`MEM_OP_LD_H] | a_mem_op[`MEM_OP_LD_BU]
                   | a_mem_op[`MEM_OP_LD_HU]| a_mem_op[`MEM_OP_LL_W];
`ifdef SOC_USE
wire        a_base_ready_eff;
wire [31:0] a_wdata_eff;
`else
`endif

// MMU 翻译（组合）：ALE / Index-cacop 抑制翻译请求
// ALE 检测（H 类要求 vaddr[0]==0，W/LL/SC 要求 vaddr[1:0]==00）
// VA 对齐先于地址翻译：ALE 时不得发 MMU，否则会与 TLBR 同拍置位，
`ifdef SOC_USE
// 破坏异常优先级。
`else
// 否则 ALE 会与 TLBR 同拍置位，破坏异常优先级。
`endif
// 注意：cacop 的 Index 类用 rj+si12 编码 way/index，低位可非对齐，不能一律按字对齐报 ALE。
wire a_size_h = a_mem_op[`MEM_OP_LD_H] | a_mem_op[`MEM_OP_LD_HU] | a_mem_op[`MEM_OP_ST_H];
wire a_size_w = a_mem_op[`MEM_OP_LD_W] | a_mem_op[`MEM_OP_ST_W]
              | a_mem_op[`MEM_OP_LL_W] | a_mem_op[`MEM_OP_SC_W];
wire a_ale = (a_size_h && (a_vaddr[0] != 1'b0))
           | (a_size_w && (a_vaddr[1:0] != 2'b00));

// Index/StoreTag（op[4:3]=00/01）使用虚地址编码 way/index，不走地址翻译。
// 否则 Index cacop 会在 PG 下误报 TLBR，difftest 见 CRMD DA↔PG / TLBR vs ALE。
wire a_cacop_di = a_is_cacop && ((a_cacop_op == 2'b00) || (a_cacop_op == 2'b01));
wire a_no_trans = a_ale | a_cacop_di;

assign mmu_d_req_o      = a_valid && a_base_ready_eff && !a_no_trans;
assign mmu_d_vaddr_o    = a_vaddr;
assign mmu_d_is_store_o = a_is_store_op;

// LoongArch MAT：2'b01=coherent cached，其余按 uncached 访问
wire a_uncached = (mmu_d_mat_i != 2'b01);
// Index cacop：paddr=vaddr（作 way/index），无 MAT 语义
wire [31:0] a_paddr = a_cacop_di ? a_vaddr : mmu_d_paddr_i;

// 异常合并；ALE / Index-cacop 屏蔽翻译类异常
wire [`EXCP_NUM-1:0] a_excp =
      ({{(`EXCP_NUM-1){1'b0}}, a_ale}                                    << `EXCP_ALE)
    | ({{(`EXCP_NUM-1){1'b0}}, !a_no_trans & mmu_d_excp_adem_i}          << `EXCP_ADEM)
    | ({{(`EXCP_NUM-1){1'b0}}, !a_no_trans & mmu_d_excp_tlbr_i} << `EXCP_TLBR_M)
    | ({{(`EXCP_NUM-1){1'b0}}, !a_no_trans & mmu_d_excp_pil_i}  << `EXCP_PIL)
    | ({{(`EXCP_NUM-1){1'b0}}, !a_no_trans & mmu_d_excp_pis_i}  << `EXCP_PIS)
    | ({{(`EXCP_NUM-1){1'b0}}, !a_no_trans & mmu_d_excp_ppi_i}  << `EXCP_PPI_M)
    | ({{(`EXCP_NUM-1){1'b0}}, !a_no_trans & mmu_d_excp_pme_i}  << `EXCP_PME);

`ifdef SOC_USE
`else
// The posted UART line-status read is a static payload property. Decode it
// before the D-stage bank boundary so the wide mem-op payload does not feed
// DCache request-valid control.
wire a_is_uart_lsr = a_is_load_op && !a_is_cacop && a_uncached
                   && !(|a_excp) && a_mem_op[`MEM_OP_LD_BU]
                   && (a_paddr == 32'h1fe0_01e5);

`endif
// store 数据按地址对齐 + wstrb
wire [1:0] a_off = a_vaddr[1:0];
reg [31:0] a_st_data;
reg [3:0]  a_st_strb;
always @(*) begin
    if (a_mem_op[`MEM_OP_ST_B]) begin
        a_st_data = {4{a_wdata_eff[7:0]}};
        a_st_strb = 4'b0001 << a_off;
    end else if (a_mem_op[`MEM_OP_ST_H]) begin
        a_st_data = {2{a_wdata_eff[15:0]}};
        a_st_strb = a_off[1] ? 4'b1100 : 4'b0011;
    end else begin
        a_st_data = a_wdata_eff;
        a_st_strb = 4'b1111;
    end
end

wire [2:0] a_size = a_mem_op[`MEM_OP_ST_B] | a_mem_op[`MEM_OP_LD_B] | a_mem_op[`MEM_OP_LD_BU] ? 3'd0
                  : a_mem_op[`MEM_OP_ST_H] | a_mem_op[`MEM_OP_LD_H] | a_mem_op[`MEM_OP_LD_HU] ? 3'd1
                  : 3'd2;
wire [2:0] pair_slot_size = pair_slot_mem_op[`MEM_OP_LD_B]
                          | pair_slot_mem_op[`MEM_OP_LD_BU] ? 3'd0
                          : pair_slot_mem_op[`MEM_OP_LD_H]
                          | pair_slot_mem_op[`MEM_OP_LD_HU] ? 3'd1
                          : 3'd2;

// =====================================================================
// DC 级
// =====================================================================
reg                    d_valid;
// The A->D advance decision is one of the latest LSU controls.  Driving the
// wide address/data registers with that clock enable puts the complete
// DCache/MSHR completion cone on all of their D pins.  Capture the current A
// payload into the inactive bank every cycle and let a_go toggle only this
// one-bit selector.  The uncached reload path has its own payload register so
// swapping a parked request does not reintroduce a wide late-enable path.
(* EXTRACT_ENABLE = "no" *) reg d_payload_sel;
(* EXTRACT_ENABLE = "no" *) reg d_vaddr_src_u;
reg [31:0]             d_vaddr_bank0, d_vaddr_bank1;
reg [31:0]             d_vaddr_u_bank;
reg [31:0]             d_st_data_bank0, d_st_data_bank1;
reg [`ROB_W-1:0]       d_robid_bank0, d_robid_bank1, d_robid_u_bank;
reg [`MEM_OP_NUM-1:0]  d_mem_op_bank0, d_mem_op_bank1, d_mem_op_u_bank;
reg                    d_is_cacop_bank0, d_is_cacop_bank1;
reg                    d_is_store_bank0, d_is_store_bank1;
reg                    d_is_load_bank0, d_is_load_bank1;
`ifdef SOC_USE
`else
reg                    d_is_uart_lsr_bank0, d_is_uart_lsr_bank1;
reg                    d_is_uart_lsr_u_bank;
`endif
reg [31:0]             d_paddr_bank0, d_paddr_bank1, d_paddr_u_bank;
reg [3:0]              d_st_strb_bank0, d_st_strb_bank1;
reg                    d_wdata_ready_bank0, d_wdata_ready_bank1;
reg [`ROB_W-1:0]       d_wdata_robid_bank0, d_wdata_robid_bank1;
reg [2:0]              d_size_bank0, d_size_bank1, d_size_u_bank;
reg                    d_uncached_bank0, d_uncached_bank1;
reg [`EXCP_NUM-1:0]    d_excp_bank0, d_excp_bank1;
reg                    d_pair_valid_bank0, d_pair_valid_bank1;
reg [31:0]             d_pair_vaddr_bank0, d_pair_vaddr_bank1;
reg [31:0]             d_pair_paddr_bank0, d_pair_paddr_bank1;
reg [`ROB_W-1:0]       d_pair_robid_bank0, d_pair_robid_bank1;
reg [`MEM_OP_NUM-1:0]  d_pair_mem_op_bank0, d_pair_mem_op_bank1;
reg [2:0]              d_pair_size_bank0, d_pair_size_bank1;

wire [`ROB_W-1:0] d_robid = d_vaddr_src_u ? d_robid_u_bank
                           : (d_payload_sel ? d_robid_bank1 : d_robid_bank0);
wire [`MEM_OP_NUM-1:0] d_mem_op = d_vaddr_src_u ? d_mem_op_u_bank
                           : (d_payload_sel ? d_mem_op_bank1 : d_mem_op_bank0);
wire d_is_cacop = !d_vaddr_src_u &&
                  (d_payload_sel ? d_is_cacop_bank1 : d_is_cacop_bank0);
wire d_is_store = !d_vaddr_src_u &&
                  (d_payload_sel ? d_is_store_bank1 : d_is_store_bank0);
wire d_is_load = d_vaddr_src_u ||
                 (d_payload_sel ? d_is_load_bank1 : d_is_load_bank0);
`ifdef SOC_USE
`else
wire d_uart_lsr = d_vaddr_src_u ? d_is_uart_lsr_u_bank
                  : (d_payload_sel ? d_is_uart_lsr_bank1
                                   : d_is_uart_lsr_bank0);
`endif
`ifdef SOC_USE
wire [31:0] d_vaddr = d_vaddr_src_u ? d_vaddr_u_bank
                           : (d_payload_sel ? d_vaddr_bank1 : d_vaddr_bank0);
wire [31:0] d_paddr = d_vaddr_src_u ? d_paddr_u_bank
                           : (d_payload_sel ? d_paddr_bank1 : d_paddr_bank0);
`else
// Keep the normal D-stage payload separate from the uncached reload payload.
// The reload slot is always uncached, so its store-order decision needs only
// stq_any and must not enter the normal 16-entry address/byte CAM.
wire [31:0] d_normal_vaddr = d_payload_sel ? d_vaddr_bank1
                                           : d_vaddr_bank0;
wire [31:0] d_normal_paddr = d_payload_sel ? d_paddr_bank1
                                           : d_paddr_bank0;
wire [2:0] d_normal_size = d_payload_sel ? d_size_bank1
                                         : d_size_bank0;
wire d_normal_uncached = d_payload_sel ? d_uncached_bank1
                                        : d_uncached_bank0;
wire [31:0] d_vaddr = d_vaddr_src_u ? d_vaddr_u_bank
                                     : d_normal_vaddr;
wire [31:0] d_paddr = d_vaddr_src_u ? d_paddr_u_bank
                                     : d_normal_paddr;
`endif
wire [31:0] d_st_data = d_vaddr_src_u ? 32'b0
                           : (d_payload_sel ? d_st_data_bank1 : d_st_data_bank0);
wire [3:0] d_st_strb = d_vaddr_src_u ? 4'b0
                           : (d_payload_sel ? d_st_strb_bank1 : d_st_strb_bank0);
wire d_wdata_ready = d_vaddr_src_u ? 1'b1
                           : (d_payload_sel ? d_wdata_ready_bank1 : d_wdata_ready_bank0);
wire [`ROB_W-1:0] d_wdata_robid = d_vaddr_src_u ? {`ROB_W{1'b0}}
                           : (d_payload_sel ? d_wdata_robid_bank1 : d_wdata_robid_bank0);
`ifdef SOC_USE
wire [2:0] d_size = d_vaddr_src_u ? d_size_u_bank
                           : (d_payload_sel ? d_size_bank1 : d_size_bank0);
wire d_uncached = d_vaddr_src_u ? 1'b1
                           : (d_payload_sel ? d_uncached_bank1 : d_uncached_bank0);
`else
wire [2:0] d_size = d_vaddr_src_u ? d_size_u_bank
                                   : d_normal_size;
wire d_uncached = d_vaddr_src_u ? 1'b1
                                 : d_normal_uncached;
`endif
wire [`EXCP_NUM-1:0] d_excp = d_vaddr_src_u ? {`EXCP_NUM{1'b0}}
                           : (d_payload_sel ? d_excp_bank1 : d_excp_bank0);
wire d_pair_valid = !d_vaddr_src_u &&
                           (d_payload_sel ? d_pair_valid_bank1 : d_pair_valid_bank0);
wire [31:0] d_pair_vaddr = d_payload_sel ? d_pair_vaddr_bank1 : d_pair_vaddr_bank0;
wire [31:0] d_pair_paddr = d_payload_sel ? d_pair_paddr_bank1 : d_pair_paddr_bank0;
wire [`ROB_W-1:0] d_pair_robid = d_payload_sel ? d_pair_robid_bank1 : d_pair_robid_bank0;
wire [`MEM_OP_NUM-1:0] d_pair_mem_op = d_payload_sel ? d_pair_mem_op_bank1 : d_pair_mem_op_bank0;
wire [2:0] d_pair_size = d_payload_sel ? d_pair_size_bank1 : d_pair_size_bank0;

// ---------------- DCache 前端请求标签（最多 4 项）----------------
// DCache 可以把一个冲突请求挂入 pending，同时继续完成后续 hit，因此命中响应
// 不能再隐含绑定当前 d 级。每次 addr_ok 时保存完整 load 元数据，命中/miss
// 返回时由 DCache 给出的 robid 精确匹配。flush 同时清表；DCache 内部取消旧
// MSHR/LOOKUP 响应，避免 ROB 标签复用后的 ABA。
localparam TOK_N = `LSU_TOKEN_DEPTH;
localparam TOK_W = 2;

reg                    t_valid    [0:TOK_N-1];
reg [`ROB_W-1:0]       t_robid    [0:TOK_N-1];
reg [`MEM_OP_NUM-1:0]  t_mem_op   [0:TOK_N-1];
reg [31:0]             t_vaddr    [0:TOK_N-1];
reg [31:0]             t_paddr    [0:TOK_N-1];
reg [2:0]              t_size     [0:TOK_N-1];
reg                    t_uncached [0:TOK_N-1];
// Timing-only validity mirror indexed directly by ROB id. DCache already
// returns the request ROB id, so raw dependency wakeup need not traverse the
// four-entry token CAM and priority encoder.
reg [`ROB_SIZE-1:0]    fast_tok_valid;
// Per-token visibility for the registered pending-load descriptor.  Keeping
// this bit beside the token avoids the generated dynamic ROB-index reads that
// are the leading RTL-specific suspect in iter68's Vivado 2023.2 elaboration
// abort.  It is cleared on the first hit/miss indication even when the
// response itself is held.
reg [TOK_N-1:0]        t_pending_visible;

wire [TOK_N-1:0] t_valid_oh;
wire [TOK_N-1:0] t_free_oh;
wire [TOK_N-1:0] t_match_oh;
genvar ti;
generate
for (ti = 0; ti < TOK_N; ti = ti + 1) begin : g_tok_status
    assign t_valid_oh[ti] = t_valid[ti];
    assign t_free_oh[ti]  = !t_valid[ti];
    assign t_match_oh[ti] = t_valid[ti] && (t_robid[ti] == dc_resp_robid_i);
    // t_pending_visible is cleared on the first hit/miss indication.  This
    // prevents a Store from entering LSU during a later held-response cycle,
    // after the one-cycle dc_fast_pipe value has already expired.
    assign pending_load_valid_o[ti] = t_valid[ti]
        && t_pending_visible[ti] && !t_uncached[ti]
        && (t_mem_op[ti][`MEM_OP_LD_W] || t_mem_op[ti][`MEM_OP_LD_B]
            || t_mem_op[ti][`MEM_OP_LD_H] || t_mem_op[ti][`MEM_OP_LD_BU]
            || t_mem_op[ti][`MEM_OP_LD_HU]);
    assign pending_load_robids_o[ti*`ROB_W +: `ROB_W] = t_robid[ti];
end
endgenerate
wire             t_has_free  = |t_free_oh;
wire [TOK_W-1:0] t_free_idx;
wire             t_match_vld = |t_match_oh;
wire [TOK_W-1:0] t_match_idx;
lsu_prio_low_comb #(.N(TOK_N), .W(TOK_W)) u_prio_tok_free(
    .mask_i(t_free_oh), .idx_o(t_free_idx)
);
lsu_prio_low_comb #(.N(TOK_N), .W(TOK_W)) u_prio_tok_match(
    .mask_i(t_match_oh), .idx_o(t_match_idx)
);

// 保留原调试层次名，但语义改为“至少一个 DCache 前端 token 在途”。
wire d_req_sent = |t_valid_oh;
wire d_drop = 1'b0;

// ---------------- miss 槽（MSHR 影子，深度 LSU_MISS_DEPTH）----------------
localparam MISS_N = `LSU_MISS_DEPTH;
localparam MISS_W = (MISS_N <= 1) ? 1 : $clog2(MISS_N);

reg                    m_valid [0:MISS_N-1];
reg                    m_drop  [0:MISS_N-1];
reg [`ROB_W-1:0]       m_robid [0:MISS_N-1];
reg [`MEM_OP_NUM-1:0]  m_mem_op[0:MISS_N-1];
reg [31:0]             m_vaddr [0:MISS_N-1];
reg [31:0]             m_paddr [0:MISS_N-1];
reg [2:0]              m_size  [0:MISS_N-1];

wire [MISS_N-1:0] m_valid_oh;
wire [MISS_N-1:0] m_free_oh;
wire [MISS_N-1:0] m_drop_oh;
wire [MISS_N-1:0] m_match_oh;
genvar mi;
generate
for (mi = 0; mi < MISS_N; mi = mi + 1) begin : g_miss_status
    assign m_valid_oh[mi] = m_valid[mi];
    assign m_free_oh[mi]  = !m_valid[mi];
    assign m_drop_oh[mi]  = m_valid[mi] && m_drop[mi];
    assign m_match_oh[mi] = m_valid[mi] && dc_mshr_data_ok_i
                         && (m_robid[mi] == dc_mshr_robid_i);
    assign miss_wait_valid_o[mi] = m_valid[mi] && !m_drop[mi];
    assign miss_wait_robids_o[mi*`ROB_W +: `ROB_W] = m_robid[mi];
end
endgenerate
wire              m_has_free   = |m_free_oh;
wire [MISS_W-1:0] m_free_idx;
// 同 robid 时优先吃 drop 槽，避免冲刷后复用 robid 的新 load 吃到旧 MSHR 数据
wire [MISS_N-1:0] m_match_drop = m_match_oh & m_drop_oh;
wire [MISS_N-1:0] m_match_live = m_match_oh & ~m_drop_oh;
wire [MISS_W-1:0] m_match_drop_idx;
wire [MISS_W-1:0] m_match_live_idx;
wire [MISS_W-1:0] m_match_idx = (|m_match_drop)
                               ? m_match_drop_idx : m_match_live_idx;
lsu_prio_low_comb #(.N(MISS_N), .W(MISS_W)) u_prio_miss_free(
    .mask_i(m_free_oh), .idx_o(m_free_idx)
);
lsu_prio_low_comb #(.N(MISS_N), .W(MISS_W)) u_prio_miss_drop(
    .mask_i(m_match_drop), .idx_o(m_match_drop_idx)
);
lsu_prio_low_comb #(.N(MISS_N), .W(MISS_W)) u_prio_miss_live(
    .mask_i(m_match_live), .idx_o(m_match_live_idx)
);

// ---------------- 写回暂存槽（被高优先级抢口的瞬态完成）----------------
reg                    h_valid;
reg [`ROB_W-1:0]       h_robid;
reg [31:0]             h_data;          // 已整形
reg [31:0]             h_vaddr, h_paddr;
reg [2:0]              h_size;

// Raw DCache hits still wake RS_MEM in the response cycle so a dependent
// store keeps the original issue schedule.  Capture the value independently
// here, however, and let the issued store consume this registered token in
// the following AGU cycle.  This prevents the long
// DCache-hit -> RS select -> issue_accept cone from driving every wdata bit.
reg                    dc_fast_pipe_valid;
reg [`ROB_W-1:0]       dc_fast_pipe_robid;
reg [31:0]             dc_fast_pipe_data;
// A raw MSHR return must not feed the AGU/TLB in the same cycle. The return is
// rare for pending bases, so register it locally and let an A-resident waiter
// consume it on the following edge.
reg                    base_mshr_pipe_valid;
reg [`ROB_W-1:0]       base_mshr_pipe_robid;
reg [31:0]             base_mshr_pipe_data;

// 年轻 UC load 旁路槽：未到 ROB 头时让出 DC，避免堵住年老访存
reg                    u_valid;
(* EXTRACT_ENABLE = "no" *) reg u_payload_sel;
reg [`ROB_W-1:0]       u_robid_bank0, u_robid_bank1;
reg [`MEM_OP_NUM-1:0]  u_mem_op_bank0, u_mem_op_bank1;
reg [31:0]             u_vaddr_bank0, u_vaddr_bank1;
reg [31:0]             u_paddr_bank0, u_paddr_bank1;
reg [2:0]              u_size_bank0, u_size_bank1;
`ifdef SOC_USE
`else
reg                    u_is_uart_lsr_bank0, u_is_uart_lsr_bank1;
`endif
wire [`ROB_W-1:0]      u_robid = u_payload_sel ? u_robid_bank1
                                              : u_robid_bank0;
wire [`MEM_OP_NUM-1:0] u_mem_op = u_payload_sel ? u_mem_op_bank1
                                               : u_mem_op_bank0;
wire [31:0]            u_vaddr = u_payload_sel ? u_vaddr_bank1
                                              : u_vaddr_bank0;
wire [31:0]            u_paddr = u_payload_sel ? u_paddr_bank1
                                              : u_paddr_bank0;
wire [2:0]             u_size = u_payload_sel ? u_size_bank1
                                             : u_size_bank0;
`ifdef SOC_USE
`else
wire                   u_is_uart_lsr = u_payload_sel
                                              ? u_is_uart_lsr_bank1
                                              : u_is_uart_lsr_bank0;
`endif

// ---------------- 顺序保护：未决 store 地址队列（WBed 尚未提交）----------------
localparam STQ_N = `STQ_DEPTH;
localparam STQ_W = (STQ_N <= 1) ? 1 : $clog2(STQ_N);

reg                  stq_v    [0:STQ_N-1];
reg [`ROB_W-1:0]     stq_id   [0:STQ_N-1];
reg [31:0]           stq_pa   [0:STQ_N-1];
reg [31:0]           stq_data [0:STQ_N-1];
reg [3:0]            stq_strb [0:STQ_N-1];
reg                  stq_uc   [0:STQ_N-1];
// Direct mirror of the most recently issued store.  Memory stores issue in
// program order, so this is the only entry that can supersede every older
// overlapping store.  Keeping the payload here avoids dynamically indexing
// five STQ arrays on the load-request path.
reg                  stq_y_valid;
reg [`ROB_W-1:0]     stq_y_id;
reg [31:0]           stq_y_pa;
reg [31:0]           stq_y_data;
reg [3:0]            stq_y_strb;
reg                  stq_y_uc;

wire [`ROB_PAIR_W-1:0] head_pair   = rob_head_robid_i[`ROB_PAIR_W-1:0];
wire                   head_s0_live= rob_head_robid_i[`ROB_W-1];

// STQ 释放：仅在 store 真正提交进 SB 的下一拍清项（避免 age-wrap 误释放，
// 以及提交当拍 SB valid 尚未打拍导致的错载窗口）。
reg                    st_ret_v_r;
reg [`ROB_W-1:0]       st_ret_id_r;
always @(posedge clk) begin
    if (reset || flush_i) begin
        st_ret_v_r  <= 1'b0;
        st_ret_id_r <= {`ROB_W{1'b0}};
    end else begin
        st_ret_v_r  <= st_retire_valid_i;
        st_ret_id_r <= st_retire_robid_i;
    end
end

wire                   stq_done [0:STQ_N-1];
genvar si;
generate
for (si = 0; si < STQ_N; si = si + 1) begin : g_stq_cm
    assign stq_done[si] = stq_v[si] && st_ret_v_r && (stq_id[si] == st_ret_id_r);
end
endgenerate

`ifdef SOC_USE
// load 访问字节掩码（实现见 mycpu.h）
wire [3:0] d_ld_bytes =
    (d_mem_op[`MEM_OP_LD_B] || d_mem_op[`MEM_OP_LD_BU])
    ? (4'b0001 << d_vaddr[1:0])
    : (d_mem_op[`MEM_OP_LD_H] || d_mem_op[`MEM_OP_LD_HU])
      ? (d_vaddr[1] ? 4'b1100 : 4'b0011)
      : 4'b1111;

wire stq_any;
wire stq_any_uc;
wire stq_overlap;
wire [STQ_N-1:0] stq_hit_one;
wire [STQ_N-1:0] d_pair_stq_hit_one;
wire [3:0] d_pair_ld_bytes =
    (d_pair_mem_op[`MEM_OP_LD_B] || d_pair_mem_op[`MEM_OP_LD_BU])
    ? (4'b0001 << d_pair_vaddr[1:0])
    : (d_pair_mem_op[`MEM_OP_LD_H] || d_pair_mem_op[`MEM_OP_LD_HU])
      ? (d_pair_vaddr[1] ? 4'b1100 : 4'b0011)
      : 4'b1111;
generate
for (si = 0; si < STQ_N; si = si + 1) begin : g_stq_hz
    // Keep an entry visible through its retirement cycle.  It is safe to wait
    // or forward from that store for one extra cycle, and this deliberately
    // removes ROB commit logic from the LSU request critical path.
    assign stq_hit_one[si] = stq_v[si]
        && (stq_pa[si][31:2] == d_paddr[31:2])
        && |(stq_strb[si] & d_ld_bytes);
    assign d_pair_stq_hit_one[si] = stq_v[si]
        && (stq_pa[si][31:2] == d_pair_paddr[31:2])
        && |(stq_strb[si] & d_pair_ld_bytes);
end
endgenerate

integer sj;
reg stq_any_r, stq_any_uc_r;
always @(*) begin
    stq_any_r = 1'b0;
    stq_any_uc_r = 1'b0;
    for (sj = 0; sj < STQ_N; sj = sj + 1) begin
        if (stq_v[sj]) begin
            stq_any_r = 1'b1;
            if (stq_uc[sj])
                stq_any_uc_r = 1'b1;
        end
    end
end
assign stq_any     = stq_any_r;
assign stq_any_uc  = stq_any_uc_r;
assign stq_overlap = |stq_hit_one;
wire d_pair_stq_overlap = |d_pair_stq_hit_one;

// Timing-safe store-to-load forwarding.  Only the globally newest live store
// is considered.  When it covers every requested byte it necessarily
// supersedes all older stores.  Other overlaps retain the conservative wait
// path.  This trades a few forwarding opportunities for one registered
// descriptor, replacing the former 16-entry serial age/priority cone and the
// remaining stq_last_idx-driven wide muxes.
wire [31:0] stq_youngest_data_r = stq_y_data;
wire [3:0]  stq_youngest_strb_r = stq_y_strb;
wire stq_fwd_hit = stq_y_valid && !stq_y_uc
                 && (stq_y_pa[31:2] == d_paddr[31:2])
                 && ((stq_youngest_strb_r & d_ld_bytes) == d_ld_bytes);

wire stq_full;
reg stq_full_r;
always @(*) begin
    stq_full_r = 1'b1;
    for (sj = 0; sj < STQ_N; sj = sj + 1)
        if (!stq_v[sj])
            stq_full_r = 1'b0;
end
assign stq_full = stq_full_r;

wire store_order_block = d_uncached ? stq_any
                                    : ((stq_overlap && !stq_fwd_hit) || stq_any_uc);
`else
// Only normal-bank loads use the STQ address CAM.  A reload-slot request is
// uncached and its ordering result is stq_any regardless of address/size.
wire [3:0] d_normal_ld_bytes = (d_normal_size == 3'd0)
    ? (4'b0001 << d_normal_vaddr[1:0])
    : (d_normal_size == 3'd1)
      ? (d_normal_vaddr[1] ? 4'b1100 : 4'b0011)
      : 4'b1111;

wire stq_any;
wire stq_any_uc;
wire normal_stq_overlap;
wire [STQ_N-1:0] stq_hit_one;
wire [STQ_N-1:0] d_pair_stq_hit_one;
wire [3:0] d_pair_ld_bytes = (d_pair_size == 3'd0)
    ? (4'b0001 << d_pair_vaddr[1:0])
    : (d_pair_size == 3'd1)
      ? (d_pair_vaddr[1] ? 4'b1100 : 4'b0011)
      : 4'b1111;
generate
for (si = 0; si < STQ_N; si = si + 1) begin : g_stq_hz
    // Keep an entry visible through its retirement cycle.  It is safe to wait
    // or forward from that store for one extra cycle, and this deliberately
    // removes ROB commit logic from the LSU request critical path.
    assign stq_hit_one[si] = stq_v[si]
        && (stq_pa[si][31:2] == d_normal_paddr[31:2])
        && |(stq_strb[si] & d_normal_ld_bytes);
    assign d_pair_stq_hit_one[si] = stq_v[si]
        && (stq_pa[si][31:2] == d_pair_paddr[31:2])
        && |(stq_strb[si] & d_pair_ld_bytes);
end
endgenerate

integer sj;
reg stq_any_r, stq_any_uc_r;
always @(*) begin
    stq_any_r = 1'b0;
    stq_any_uc_r = 1'b0;
    for (sj = 0; sj < STQ_N; sj = sj + 1) begin
        if (stq_v[sj]) begin
            stq_any_r = 1'b1;
            if (stq_uc[sj])
                stq_any_uc_r = 1'b1;
        end
    end
end
assign stq_any     = stq_any_r;
assign stq_any_uc  = stq_any_uc_r;
assign normal_stq_overlap = |stq_hit_one;
wire d_pair_stq_overlap = |d_pair_stq_hit_one;

// Timing-safe store-to-load forwarding.  Only the globally newest live store
// is considered.  When it covers every requested byte it necessarily
// supersedes all older stores.  Other overlaps retain the conservative wait
// path.  This trades a few forwarding opportunities for one registered
// descriptor, replacing the former 16-entry serial age/priority cone and the
// remaining stq_last_idx-driven wide muxes.
wire [31:0] stq_youngest_data_r = stq_y_data;
wire [3:0]  stq_youngest_strb_r = stq_y_strb;
wire stq_fwd_hit = stq_y_valid && !stq_y_uc
                 && (stq_y_pa[31:2] == d_normal_paddr[31:2])
                 && ((stq_youngest_strb_r & d_normal_ld_bytes)
                     == d_normal_ld_bytes);

wire stq_full;
reg stq_full_r;
always @(*) begin
    stq_full_r = 1'b1;
    for (sj = 0; sj < STQ_N; sj = sj + 1)
        if (!stq_v[sj])
            stq_full_r = 1'b0;
end
assign stq_full = stq_full_r;

wire normal_store_order_block = d_normal_uncached ? stq_any
                              : ((normal_stq_overlap && !stq_fwd_hit)
                                 || stq_any_uc);
wire store_order_block = d_vaddr_src_u ? stq_any
                                       : normal_store_order_block;
`endif

// The second load shares the first load's translation only inside one 4 KiB
// page.  RS already limits candidates to one 32-byte line; these checks are
// repeated at the ownership boundary so MMIO, exceptions, LL/SC and any
// visible store state automatically fall back to a normal single request.
wire a_pair_match = pair_slot_valid && !pair_wait_resp
                  && !pair_replay && !pair_agu_replay
                  && a_valid && (a_robid == pair_first_robid);
wire [31:0] a_pair_paddr = {a_paddr[31:12], pair_slot_vaddr[11:0]};
wire pair_slot_size_h = pair_slot_mem_op[`MEM_OP_LD_H]
                      | pair_slot_mem_op[`MEM_OP_LD_HU];
wire pair_slot_size_w = pair_slot_mem_op[`MEM_OP_LD_W];
wire pair_slot_ale = (pair_slot_size_h && pair_slot_vaddr[0])
                   | (pair_slot_size_w && (|pair_slot_vaddr[1:0]));
assign sb_pair_query_paddr_o = d_pair_valid ? d_pair_paddr[31:2]
                                           : a_pair_paddr[31:2];
wire a_pair_eligible = a_is_load_op && !a_is_cacop
                     && !a_uncached && !(|a_excp)
                     && !pair_slot_ale
                     && (a_vaddr[31:12] == pair_slot_vaddr[31:12])
                     && ((a_paddr[5] != pair_slot_vaddr[5])
                         || (a_paddr[11:6] == pair_slot_vaddr[11:6]))
                     && !sb_pair_query_maybe_i;
wire a_pair_safe = a_pair_match && a_pair_eligible;
wire d_pair_runtime_safe = d_pair_valid && !d_uncached && !(|d_excp)
                         && !stq_any_uc && !d_pair_stq_overlap
                         && !sb_pair_query_maybe_i
                         && ((d_paddr[31:5] == d_pair_paddr[31:5])
                             || (d_paddr[5] != d_pair_paddr[5]));

// ---------------- DC 级行为 ----------------
wire d_excp_any = |d_excp;

// SB 前递查询。空 SB 是短旁路；非空时由 SB 内部寄存完整扫描结果，
// 返回 tag 必须与保持中的 D 级物理字地址及缓存属性一致。
assign sb_query_paddr_o    = d_paddr[31:2];
assign sb_query_uncached_o = d_valid && d_is_load && d_uncached;
wire sb_query_needed = d_valid && d_is_load && !d_excp_any
                     && (d_uncached || !stq_fwd_hit)
                     && !sb_empty_i && sb_query_maybe_i;
assign sb_query_valid_o = sb_query_needed;
wire sb_query_resp_match = sb_query_resp_valid_i
                         && (sb_query_resp_paddr_i == d_paddr[31:2])
                         && (sb_query_resp_uncached_i == d_uncached);
wire sb_query_ready = !sb_query_needed || sb_query_resp_match;

// uncached load 许可：自己是最老未提交指令
wire d_at_head = (d_robid[`ROB_PAIR_W-1:0] == head_pair)
              && ((d_robid[`ROB_W-1] == 1'b0) || !head_s0_live)
              && rob_head_valid_i;

wire d_is_unc_load = d_valid && d_is_load && d_uncached && !d_excp_any;
`ifdef SOC_USE
// UART status reads must traverse the regular uncached AXI path.  Faking a
// ready result can let software write the UART FIFO before hardware observes
// the corresponding status transaction.
wire d_is_uart_lsr = 1'b0;
`else
// The NSCSCC UART16550 line-status register is a read-only, idempotent
// TX-ready query.  It is safe to issue this polling load before ROB head;
// every other MMIO load keeps strict non-speculative ordering.
wire d_is_uart_lsr = d_valid && d_uart_lsr;
`endif
assign uncached_ld_inflight_o = d_is_unc_load || u_valid;

// load 可以发起最终访问（SB 终查/DCache）的条件：
// - 未决 store 先提交（顺序保护）；
// - hold 槽空（防瞬态完成三方碰撞，见头注仲裁说明）；
// - uncached load 额外等：到 ROB 头 + SB 无 uncached 残留（query_partial）
wire d_ld_gate = !store_order_block && !h_valid && t_has_free
              && sb_query_ready
              && (!d_uncached
                  || ((d_at_head || d_is_uart_lsr)
                      && !(sb_query_resp_match && sb_query_partial_i)));

// load 处理分支
wire d_stq_hit    = d_valid && d_is_load && !d_excp_any && !d_uncached && stq_fwd_hit;
wire d_sb_hit     = d_valid && d_is_load && !d_excp_any && !d_uncached
                  && !d_stq_hit && sb_query_needed
                  && sb_query_resp_match && sb_query_hit_i;
wire d_sb_partial = d_valid && d_is_load && !d_excp_any && !d_uncached
                  && sb_query_needed && sb_query_resp_match
                  && sb_query_partial_i;

wire d_need_dc = d_valid && d_is_load && !d_excp_any
              && !d_stq_hit && !d_sb_hit && !d_sb_partial
              && !d_is_uart_lsr;

// DCache 请求（保持至 addr_ok）。addr_ok 后 d 级立即释放；返回元数据由 token
// 表承担，因此连续 hit 可以利用 DCache LOOKUP 链做到每拍一条。
assign dc_req_o      = d_need_dc && d_ld_gate && !flush_i;
assign dc_vindex_o   = d_vaddr[11:5];
assign dc_paddr_o    = d_paddr;
assign dc_size_o     = d_size;
assign dc_load_op_o  = d_mem_op[7:4];
assign dc_uncached_o = d_uncached;
assign dc_robid_o    = d_robid;
assign dc_cancel_o   = flush_i;
assign dc_pair_valid_o = dc_req_o && d_pair_runtime_safe;
assign dc_pair_word_o  = d_pair_vaddr[4:2];
assign dc_pair_paddr_o = d_pair_paddr;

wire dc_fire = dc_req_o && dc_addr_ok_i;
wire d_is_plain_load = d_mem_op[`MEM_OP_LD_W]
                    || d_mem_op[`MEM_OP_LD_B]
                    || d_mem_op[`MEM_OP_LD_H]
                    || d_mem_op[`MEM_OP_LD_BU]
                    || d_mem_op[`MEM_OP_LD_HU];
wire pending_load_set = dc_fire && d_is_plain_load && !d_uncached;
wire pending_load_resp_match = |(t_match_oh & pending_load_valid_o);
// data_ok 是 valid，ready 由 LSU 的单项完成暂存槽反压；miss 通知不需占
// 写回口，只把 token 元数据移入已有 MSHR 影子槽。
wire dc_return_valid = dc_data_ok_i && t_match_vld;
wire dc_return       = dc_return_valid && dc_resp_ready_o;
wire dc_missed       = dc_miss_i && t_match_vld;
// A held DCache hit remains valid until the registered response boundary can
// accept it.  Do not retire its dependency descriptor on the first valid-only
// cycle: a consumer dispatched during the hold interval must still see the
// pending producer and then the eventual accepted-result wakeup.
wire pending_load_clear_hit = dc_return && pending_load_resp_match;
wire pending_load_clear_miss = dc_missed && pending_load_resp_match;
wire pair_resp_match = pair_wait_resp
                     && (dc_resp_robid_i == pair_first_robid);
wire pair_hit_return = dc_pair_data_ok_i && pair_resp_match;
// DCache registers the fused-pair failure decision.  There is only one pair
// slot, so the pending bit is the response token; avoiding the raw DCache
// robid/miss compare here cuts the routed req_paddr -> pair_replay path.
wire pair_miss_return = dc_pair_miss_i && pair_wait_resp;

// MSHR 重填返回（按 robid 配对；m_drop 时静默消费）
// The selected-index form routed the four-way priority encoder and a dynamic
// m_drop read into every WB/arbitration and stage-control endpoint.  Selection
// itself remains drop-first for payload/slot updates, while these exact
// reductions answer only which class matched.  If an old dropped slot and a
// reused live ROB tag coexist, the dropped response still wins exactly as
// before.
(* max_fanout = 24 *) wire mshr_return = !(|m_match_drop)
                                        && (|m_match_live);
wire mshr_return_drop = |m_match_drop;

// ---------------- load 数据整形 ----------------
// DCache returns an already-shaped front-end load value.  Keep shaping here
// for SB/STQ/pair sources, whose metadata is still owned by LSU.
wire [31:0] shaped_dc_token = dc_rdata_i;
wire [31:0] shaped_sb_d;
wire [31:0] shaped_stq_d;
wire [31:0] shaped_pair;
lsu_load_shape_comb u_shape_sb_d(
    .word_i(sb_query_data_i), .op_i(d_mem_op[7:4]),
    .off_i(d_vaddr[1:0]), .data_o(shaped_sb_d)
);
lsu_load_shape_comb u_shape_stq_d(
    .word_i(stq_youngest_data_r), .op_i(d_mem_op[7:4]),
    .off_i(d_vaddr[1:0]), .data_o(shaped_stq_d)
);
lsu_load_shape_comb u_shape_pair(
    .word_i(dc_pair_rdata_i), .op_i(pair_slot_mem_op[7:4]),
    .off_i(pair_slot_vaddr[1:0]), .data_o(shaped_pair)
);

// A refill is consumed by the normal load writeback and, independently, by
// at most the few in-flight LSU pipeline slots waiting for that load's value.
// This local fanout replaces the former refill -> RS_MEM -> issue -> LSU loop.
// The DCache stores the load descriptor with its MSHR and returns an already
// shaped value.  Do not re-index the LSU miss table on the response path.
wire [31:0] local_refill_data = dc_mshr_rdata_i;
wire local_dc_hit_valid = (`LSU_DC_HIT_BYPASS != 0) && dc_data_ok_i
                        && dc_resp_ready_o
                        && fast_tok_valid[dc_resp_robid_i];
wire [31:0] local_dc_hit_data = shaped_dc_token;
wire local_fast_valid = h_valid || dc_fast_pipe_valid;
wire [`ROB_W-1:0] local_fast_robid = h_valid ? h_robid
                                             : dc_fast_pipe_robid;
wire [31:0] local_fast_data = h_valid ? h_data
                                      : dc_fast_pipe_data;
// Pending address sources use the same local load-return network as pending
// Store data. This preserves the load-use schedule without broadcasting the
// raw DCache response back through RS_MEM.
wire q_base_mshr_hit = base_mshr_pipe_valid && q_valid && !q_base_ready
                    && (q_base_robid == base_mshr_pipe_robid);
wire q_base_fast_hit = local_fast_valid && q_valid && !q_base_ready
                    && (q_base_robid == local_fast_robid);
wire q_base_rob_hit = q_valid && !q_base_ready && rob_q_base_ready_i
                   && !q_base_mshr_hit && !q_base_fast_hit;
wire q_base_refill_hit = q_base_mshr_hit || q_base_fast_hit || q_base_rob_hit;
wire a_base_mshr_hit = base_mshr_pipe_valid && a_valid && !a_base_ready
                    && (a_base_robid == base_mshr_pipe_robid);
wire a_base_fast_hit = local_fast_valid && a_valid && !a_base_ready
                    && (a_base_robid == local_fast_robid);
wire a_base_rob_hit = a_valid && !a_base_ready && rob_a_base_ready_i
                   && !a_base_mshr_hit && !a_base_fast_hit;
wire a_base_refill_hit = a_base_mshr_hit || a_base_fast_hit || a_base_rob_hit;
wire issue_base_mshr_hit = base_mshr_pipe_valid && issue_valid_i && lsu_ready_o
                        && !issue_base_ready_i
                        && (issue_base_robid_i == base_mshr_pipe_robid);
wire issue_base_fast_hit = local_fast_valid && issue_valid_i && lsu_ready_o
                        && !issue_base_ready_i
                        && (issue_base_robid_i == local_fast_robid);
// RS_MEM may hand a dependent memory operation to LSU on the exact cycle in
// which the producer's accepted DCache hit is broadcast.  The registered
// local pipe is one cycle too late for this input boundary, so capture the
// already-shaped accepted hit directly into A/Q on that handshake.
wire issue_base_dc_hit = local_dc_hit_valid && issue_valid_i && lsu_ready_o
                       && !issue_base_ready_i
                       && (issue_base_robid_i == dc_resp_robid_i);
`ifdef SOC_USE
`else
wire issue_base_refill_hit = issue_base_mshr_hit || issue_base_fast_hit
                           || issue_base_dc_hit;
`endif
wire [31:0] q_base_return_data = q_base_mshr_hit ? base_mshr_pipe_data
                                : q_base_fast_hit ? local_fast_data
                                                   : rob_q_base_data_i;
wire [31:0] a_base_return_data = a_base_mshr_hit ? base_mshr_pipe_data
                                : a_base_fast_hit ? local_fast_data
                                                   : rob_a_base_data_i;
`ifdef SOC_USE
`else
wire [31:0] issue_base_return_data = issue_base_mshr_hit
                                   ? base_mshr_pipe_data
                                   : issue_base_dc_hit
                                   ? local_dc_hit_data : local_fast_data;
`endif
assign a_base_probe_robid_o = a_base_robid;
assign q_base_probe_robid_o = q_base_robid;
wire [31:0] q_base_eff = q_base_refill_hit ? q_base_return_data : q_base;
wire        q_base_ready_eff = q_base_ready || q_base_refill_hit;
// Every pending A-stage address response first lands in a_base/a_base_ready.
// Do not let even a registered hit tag feed the address adder and DTLB in the
// same cycle: that path crosses the local wakeup compare, AGU and translation
// control before reaching the next registers.  The sequential capture below
// preserves the value and launches it on the following cycle.
wire [31:0] a_base_eff = a_base;
`ifdef SOC_USE
assign a_base_ready_eff = a_base_ready;
`else
wire        a_base_ready_eff = a_base_ready;
`endif
// Do not let the selected RS_MEM tag drive a same-cycle compare/data mux into
// every A/Q base register.  When the issued address operand is still pending,
// capture its tag at the normal LSU boundary; the existing registered
// base_mshr/local-fast/DC pipelines fill A or Q on the following cycle.  This
// removes the measured 15-level s0_robid -> a/q_base path without changing
// issue throughput or dropping the pending operand.
wire [31:0] issue_base_eff = issue_base_i;
wire        issue_base_ready_eff = issue_base_ready_i;
// Store operands consume the already registered MSHR return.  Using the raw
// refill state/data here made the DCache MSHR FSM drive every 32-bit A/Q/D
// operand register through a long compare/select cone.
wire q_wdata_mshr_hit = base_mshr_pipe_valid && q_valid && !q_wdata_ready
                     && (q_wdata_robid == base_mshr_pipe_robid);
wire q_wdata_fast_hit = local_fast_valid && q_valid && !q_wdata_ready
                      && (q_wdata_robid == local_fast_robid);
wire q_wdata_dc_hit = local_dc_hit_valid && q_valid && !q_wdata_ready
                    && (q_wdata_robid == dc_resp_robid_i);
wire q_wdata_refill_hit = q_wdata_mshr_hit || q_wdata_fast_hit
                        || q_wdata_dc_hit;
wire a_wdata_mshr_hit = base_mshr_pipe_valid && a_valid && !a_wdata_ready
                     && (a_wdata_robid == base_mshr_pipe_robid);
wire a_wdata_fast_hit = local_fast_valid && a_valid && !a_wdata_ready
                      && (a_wdata_robid == local_fast_robid);
wire a_wdata_dc_hit = local_dc_hit_valid && a_valid && !a_wdata_ready
                    && (a_wdata_robid == dc_resp_robid_i);
wire a_wdata_refill_hit = a_wdata_mshr_hit || a_wdata_fast_hit
                        || a_wdata_dc_hit;
wire d_wdata_mshr_hit = base_mshr_pipe_valid && d_valid && d_is_store
                     && !d_wdata_ready
                     && (d_wdata_robid == base_mshr_pipe_robid);
wire d_wdata_fast_hit = local_fast_valid && d_valid && d_is_store
                      && !d_wdata_ready
                      && (d_wdata_robid == local_fast_robid);
wire d_wdata_dc_hit = local_dc_hit_valid && d_valid && d_is_store
                    && !d_wdata_ready
                    && (d_wdata_robid == dc_resp_robid_i);
wire d_wdata_refill_hit = d_wdata_mshr_hit || d_wdata_fast_hit
                        || d_wdata_dc_hit;
wire issue_wdata_mshr_hit = base_mshr_pipe_valid && issue_valid_i && lsu_ready_o
                         && !issue_wdata_ready_i
                         && (issue_wdata_robid_i == base_mshr_pipe_robid);
wire issue_wdata_fast_hit = local_fast_valid && issue_valid_i && lsu_ready_o
                         && !issue_wdata_ready_i
                         && (issue_wdata_robid_i == local_fast_robid);
wire issue_wdata_dc_hit = local_dc_hit_valid && issue_valid_i && lsu_ready_o
                        && !issue_wdata_ready_i
                        && (issue_wdata_robid_i == dc_resp_robid_i);
wire issue_wdata_refill_hit = issue_wdata_mshr_hit || issue_wdata_fast_hit
                            || issue_wdata_dc_hit;
wire [31:0] q_return_data = q_wdata_mshr_hit ? base_mshr_pipe_data
                          : q_wdata_dc_hit ? local_dc_hit_data
                                           : local_fast_data;
wire [31:0] a_return_data = a_wdata_mshr_hit ? base_mshr_pipe_data
                          : a_wdata_dc_hit ? local_dc_hit_data
                                           : local_fast_data;
wire [31:0] d_return_data = d_wdata_mshr_hit ? base_mshr_pipe_data
                          : d_wdata_dc_hit ? local_dc_hit_data
                                           : local_fast_data;
wire [31:0] issue_return_data = issue_wdata_mshr_hit ? base_mshr_pipe_data
                              : issue_wdata_dc_hit ? local_dc_hit_data
                                                   : local_fast_data;
wire [31:0] q_wdata_current = q_wdata_refill_sel ? q_wdata_refill : q_wdata;
wire [31:0] q_wdata_eff = q_wdata_refill_hit ? q_return_data
                                             : q_wdata_current;
wire        q_wdata_ready_eff = q_wdata_ready || q_wdata_refill_hit;
`ifdef SOC_USE
assign a_wdata_eff = a_wdata_refill_hit ? a_return_data : a_wdata;
`else
wire [31:0] a_wdata_eff = a_wdata_refill_hit ? a_return_data : a_wdata;
`endif
wire        a_wdata_ready_eff = a_wdata_ready || a_wdata_refill_hit;
wire        issue_wdata_ready_eff = issue_wdata_ready_i
                                  || issue_wdata_refill_hit;

assign a_vaddr = a_base_eff + a_imm;

wire [31:0] d_refill_st_data = d_mem_op[`MEM_OP_ST_B]
                              ? {4{d_return_data[7:0]}}
                              : d_mem_op[`MEM_OP_ST_H]
                              ? {2{d_return_data[15:0]}}
                              : d_return_data;
wire [31:0] d_st_data_eff = d_wdata_refill_hit
                           ? d_refill_st_data : d_st_data;
`ifdef SOC_USE
`else
wire        d_wdata_ready_eff = d_wdata_ready || d_wdata_refill_hit;
`endif

// Register the secondary completion before the global ROB/RS wakeup fanout.
// The direct DCache-to-writeback form creates one long path into every issue
// queue and the multiplier, so it cannot satisfy even the 65 MHz baseline.
assign pair_wb_valid_o = ph_valid && !flush_i;
assign pair_wb_robid_o = ph_robid;
assign pair_wb_data_o  = ph_data;
assign pair_wb_paddr_o = ph_paddr;
assign pair_wb_vaddr_o = ph_vaddr;
assign pair_wb_size_o  = ph_size;

// ---------------- 写回仲裁（一拍一条，按年龄：miss 槽 > hold > DC 级）----------------
// DC 级完成源：
// 1) 异常：直接写回；2) store/cacop：直接写回；3) load：SB 命中或 DCache 返回
wire wb_mshr_case  = mshr_return;                       // 最老，最高优先
wire wb_hold_case  = !wb_mshr_case && h_valid;
// 旧 hold 被写回的拍可由新 hit 原位补入；MSHR 抢口且 hold 已占用时反压
// DCache，让 LOOKUP 保持到下一拍，所有 accepted load 都不会丢响应。
wire wb_ld_dc_case = !wb_mshr_case && !h_valid && dc_return;
wire dcst_ok       = !wb_mshr_case && !h_valid && !dc_return;
wire wb_excp_case  = dcst_ok && d_valid && d_excp_any;
// The benchmark console polls the read-only 16550 LSR before every byte.
// Treat the architecturally posted UART path as locally ready once all older
// stores are ordered.  Data writes still traverse STQ/SB/DCache normally;
// only the idempotent byte read is completed here.
wire wb_uart_lsr_case = dcst_ok && d_is_uart_lsr && d_ld_gate;
wire wb_st_case    = dcst_ok && d_valid && !d_excp_any
                  // A late load return first fills the registered D payload.
`ifdef SOC_USE
                  // Completing the store from a refill-ready value in that same
`else
                  // Completing the store from d_wdata_ready_eff in that same
`endif
                  // cycle lets the full DCache/MSHR return cone drive d_done,
                  // d_free, a_go, and every A-stage register control pin.
                  // Use only registered readiness here; the captured store
                  // completes on the following cycle with identical data.
                  && (d_is_cacop || (d_is_store && d_wdata_ready))
                  && !(d_is_store && stq_full);
// SB 命中结果经 hold 暂存一拍，隔离逐字节前递合并到写回旁路的长组合路径。
// flush 会丢弃尚未提交的 hold 内容；store_order_block 保证 store→load 顺序。
wire stq_ready     = d_stq_hit && d_ld_gate && !store_order_block;
wire sb_ready      = d_sb_hit && d_ld_gate && !store_order_block; // d_ld_gate 已含 !h_valid
wire wb_ld_sb_case = 1'b0;                               // SB 命中仍走 hold，切断 SB→RS 组合路径
// D$ 返回由 robid token 解包；被 MSHR/旧 hold 抢口时进入 hold。
// The cache response-ready boundary must depend only on registered state.
// Allowing same-cycle hold turnover made MSHR return and local-forwarding
// arbitration feed DCache's next tag-RAM address through lookup chaining.
// A normal hit still writes back directly while h is empty.  If an MSHR owns
// the WB port, the hit is captured in h.  When h is occupied we conservatively
// leave the cache response asserted for one extra cycle instead of using the
// current WB decision to advertise turnover.
assign dc_resp_ready_o = !h_valid;
wire hold_cap_dc   = dc_return && !wb_ld_dc_case;
// If a DCache hit and a local STQ/SB completion both need the hold register,
// preserve the accepted cache response and leave the local D-stage operation
// resident for retry.  When the hit writes back directly, the local completion
// may still fill h in the same cycle.
wire hold_cap_stq  = stq_ready && !hold_cap_dc;
wire hold_cap_sb   = sb_ready && !hold_cap_dc && !hold_cap_stq;

assign wb_valid_o = (wb_mshr_case || wb_hold_case
                  || wb_excp_case || wb_uart_lsr_case || wb_st_case
                  || wb_ld_sb_case || wb_ld_dc_case)
                  && !flush_i;
// m_match_idx is selected only from entries whose ROB ID equals the DCache
// response tag.  Forward that tag directly on the MSHR case instead of
// decoding the matching queue entry again on the raw ROB-completion path.
assign wb_robid_o = wb_mshr_case ? dc_mshr_robid_i
                  : wb_hold_case ? h_robid
                  : wb_ld_dc_case ? t_robid[t_match_idx]
                  : d_robid;
assign wb_data_o  = wb_mshr_case  ? dc_mshr_rdata_i
                  : wb_hold_case  ? h_data
                  : wb_uart_lsr_case ? 32'h0000_0060
                  : wb_ld_sb_case ? shaped_sb_d
                  : wb_ld_dc_case ? shaped_dc_token
                  : d_st_data_eff;
assign wb_paddr_o = wb_mshr_case ? m_paddr[m_match_idx]
                  : wb_hold_case ? h_paddr
                  : wb_ld_dc_case ? t_paddr[t_match_idx] : d_paddr;
assign wb_vaddr_o = wb_mshr_case ? m_vaddr[m_match_idx]
                  : wb_hold_case ? h_vaddr
                  : wb_ld_dc_case ? t_vaddr[t_match_idx] : d_vaddr;
assign wb_wstrb_o = (!wb_mshr_case && !wb_hold_case && !wb_ld_dc_case
                  && d_is_store && !d_excp_any) ? d_st_strb : 4'b0;
assign wb_size_o  = wb_mshr_case ? m_size[m_match_idx]
                  : wb_hold_case ? h_size
                  : wb_ld_dc_case ? t_size[t_match_idx] : d_size;
assign wb_uncached_o = wb_ld_dc_case ? t_uncached[t_match_idx]
                     : (!wb_mshr_case && !wb_hold_case) && d_uncached;
assign wb_excp_o  = (!wb_mshr_case && !wb_hold_case && !wb_ld_dc_case)
                  ? d_excp : {`EXCP_NUM{1'b0}};
// The dependency wakeup bus describes data readiness, not ownership of the
// single architectural writeback port.  In particular, an older MSHR return
// may win that port while the hold/DC-hit value below is already usable by a
// dependent instruction.  Keeping this bus independent of wb_mshr_case both
// preserves the useful early wakeup and removes the DCache-MSHR -> ALU cone.
wire fast_hold_ready = h_valid;
wire fast_dc_ready = (`LSU_DC_HIT_BYPASS != 0) && dc_data_ok_i
                   && dc_resp_ready_o
                   && fast_tok_valid[dc_resp_robid_i];
assign fast_wb_valid_o = (fast_hold_ready || fast_dc_ready) && !flush_i;
assign fast_wb_robid_o = fast_hold_ready ? h_robid : dc_resp_robid_i;
assign fast_wb_data_o  = fast_hold_ready
                       ? h_data
                       : shaped_dc_token;
assign fast_mdu_valid_o = fast_dc_ready
                       && t_mem_op[t_match_idx][`MEM_OP_LD_W]
                       && !flush_i;
assign fast_mdu_robid_o = dc_resp_robid_i;
assign fast_mdu_data_o  = dc_rdata_i;
assign hold_wb_valid_o = h_valid && !flush_i;
assign hold_wb_robid_o = h_robid;
assign hold_wb_data_o  = h_data;

// ---------------- 流水推进 ----------------
// DC 级本拍腾空：写回成功、miss 完成条件到达，或已捕获进 hold。
wire d_done  = wb_excp_case || wb_uart_lsr_case || wb_st_case || wb_ld_sb_case
             || dc_fire || hold_cap_stq || hold_cap_sb;

// 年轻 UC park（宽版）：仅对【比 DC 中 UC 更老】的 AGU 让位。
`ifdef SOC_USE
// 非队首 UC 不可能已发出 DCache 请求：d_ld_gate 要求 d_at_head，且
// dc_fire 在同拍结束 D 级。d_req_sent 是所有 cached token 的全局占用，
// 不能用它阻止本 UC 停车，否则旧 cached token 会与 UC poll 互锁。
`else
`endif
// 勿对更年轻 AGU 让位（会覆盖 u / 堵 RS）；u_valid 期间禁更年轻进 DC。
`ifdef SOC_USE
`else
// u_valid 期间禁止年轻请求进入 DC，避免覆盖停车槽或破坏程序序。
`endif
wire d_uc_yield = d_is_unc_load && !d_is_uart_lsr
`ifdef SOC_USE
               && !d_at_head && !d_drop;
`else
               && !d_at_head && !d_req_sent && !d_drop;
`endif
wire u_at_head = u_valid
              && (u_robid[`ROB_PAIR_W-1:0] == head_pair)
              && ((u_robid[`ROB_W-1] == 1'b0) || !head_s0_live)
              && rob_head_valid_i;
wire [`ROB_PAIR_W-1:0] a_from_h = a_robid[`ROB_PAIR_W-1:0] - head_pair;
wire [`ROB_PAIR_W-1:0] d_from_h = d_robid[`ROB_PAIR_W-1:0] - head_pair;
wire [`ROB_PAIR_W-1:0] u_from_h = u_robid[`ROB_PAIR_W-1:0] - head_pair;
wire a_older_than_d = (a_from_h < d_from_h)
                   || ((a_from_h == d_from_h)
                       && (a_robid[`ROB_W-1] == 1'b0)
                       && (d_robid[`ROB_W-1] == 1'b1));
wire a_older_than_u = !u_valid
                   || (a_from_h < u_from_h)
                   || ((a_from_h == u_from_h)
                       && (a_robid[`ROB_W-1] == 1'b0)
                       && (u_robid[`ROB_W-1] == 1'b1));
wire u_reload = u_at_head && (!d_valid || d_done || d_uc_yield);
wire d_free   = !d_valid || d_done
             || (d_uc_yield && !u_valid && a_valid && a_older_than_d && !u_reload);
wire a_translate_ready = a_no_trans || mmu_d_ready_i;
wire a_go     = a_valid && a_base_ready_eff && a_translate_ready
              && d_free && !u_reload && a_older_than_u;
wire d_park   = (d_uc_yield && a_go && !u_valid) || (d_uc_yield && u_reload);

// Do not use a_go here: a_go contains the SB lookup result and formed the
// synthesized store_buffer -> rs_mem critical path.  While a fused second
// load is unresolved, younger plain loads may fill the skid slot, but ordered
// memory operations wait until the pair hits or is replayed.
// Export the registered fused-pair policy instead of decoding the selected
// RS_MEM payload here and sending that result back through the issue cone.
// A pending replay still blocks every new issue exactly as before.
`ifdef SOC_USE
`else
wire issue_is_plain_load = issue_mem_op_i[`MEM_OP_LD_W]
                         | issue_mem_op_i[`MEM_OP_LD_B]
                         | issue_mem_op_i[`MEM_OP_LD_H]
                         | issue_mem_op_i[`MEM_OP_LD_BU]
                         | issue_mem_op_i[`MEM_OP_LD_HU];
`endif
assign lsu_ready_o = !q_valid && !flush_i
                   && !(pair_slot_valid && (pair_replay || pair_agu_replay));
assign lsu_plain_only_o = pair_slot_valid;
assign lsu_pair_ready_o = !pair_slot_valid && !flush_i;
wire issue_accept = issue_valid_i && lsu_ready_o;
// rs_mem only asserts pair_valid when its primary head issue is valid and
// lsu_ready_o is true, so re-ANDing issue_accept here only recreates the full
// WB-tag/selection cone on the pair replay state.
wire issue_pair_accept = issue_pair_valid_i && lsu_pair_ready_o;
// The replay payload is the next A-stage candidate for as long as it is
// pending.  Preload that candidate independently of the late A-stage advance;
// only the actual consume still depends on a_go.  This keeps the DCache return
// cone out of every inactive A-bank payload D pin.
wire pair_replay_pending = pair_slot_valid && (pair_replay || pair_agu_replay);
wire pair_replay_take = pair_replay_pending && (a_go || !a_valid);
wire pair_agu_set = a_go && a_pair_match && !a_pair_eligible;
// Keep the timing-critical translation decision in a dedicated sticky FF.
// Clear it from the slot lifetime, not from pair_replay_take's a_go cone.
// A set AGU fallback cannot have a pair response: a_pair_safe was false, so
// no paired DCache request was launched.  pair_replay_take clears
// pair_slot_valid in the main state block; retaining this don't-care bit for
// that one edge is harmless because every consumer also gates slot_valid.
// On an empty-slot edge (including a simultaneous new pair accept) the old
// bit is cleared before the new slot becomes observable.
always @(posedge clk) begin
    if (reset || flush_i)
        pair_agu_replay <= 1'b0;
    else if (!pair_slot_valid)
        pair_agu_replay <= 1'b0;
    else if (pair_agu_set)
        pair_agu_replay <= 1'b1;
end
wire a_payload_take_q = !pair_replay_pending && q_valid;
wire [`ROB_W-1:0] a_payload_robid = pair_replay_pending ? pair_slot_robid
                                       : a_payload_take_q ? q_robid : issue_robid_i;
wire [`MEM_OP_NUM-1:0] a_payload_mem_op = pair_replay_pending ? pair_slot_mem_op
                                       : a_payload_take_q ? q_mem_op : issue_mem_op_i;
wire a_payload_is_cacop = pair_replay_pending ? 1'b0
                                       : a_payload_take_q ? q_is_cacop : issue_is_cacop_i;
wire [4:3] a_payload_cacop_op = pair_replay_pending ? 2'b0
                                       : a_payload_take_q ? q_cacop_op : issue_cacop_op_i;
wire [31:0] a_payload_base = pair_replay_pending ? pair_slot_vaddr
                                       : a_payload_take_q ? q_base_eff : issue_base_eff;
wire a_payload_base_ready = pair_replay_pending ? 1'b1
                                       : a_payload_take_q ? q_base_ready_eff : issue_base_ready_eff;
wire [`ROB_W-1:0] a_payload_base_robid = pair_replay_pending ? {`ROB_W{1'b0}}
                                       : a_payload_take_q ? q_base_robid : issue_base_robid_i;
wire [31:0] a_payload_imm = pair_replay_pending ? 32'b0
                                       : a_payload_take_q ? q_imm : issue_imm_i;
// Keep the late RS_MEM ready reduction out of all 32 payload-register D pins.
// For a direct issue, primary is the RS_MEM value and refill is the matching
// registered-return value.  issue_wdata_refill_hit (which includes !ready)
// is captured only in the one-bit selector.  q/replay supply identical copies.
wire [31:0] a_payload_wdata_primary = pair_replay_pending ? 32'b0
                                       : a_payload_take_q ? q_wdata_eff
                                                          : issue_wdata_i;
wire [31:0] a_payload_wdata_refill = pair_replay_pending ? 32'b0
                                       : a_payload_take_q ? q_wdata_eff
                                                          : issue_return_data;
wire a_payload_wdata_refill_sel = !pair_replay_pending && !a_payload_take_q
                                 && issue_wdata_refill_hit;
wire a_payload_wdata_ready = pair_replay_pending ? 1'b1
                                       : a_payload_take_q ? q_wdata_ready_eff : issue_wdata_ready_eff;
wire [`ROB_W-1:0] a_payload_wdata_robid = pair_replay_pending ? {`ROB_W{1'b0}}
                                       : a_payload_take_q ? q_wdata_robid : issue_wdata_robid_i;

always @(posedge clk) begin
    if (reset) begin
        a_valid    <= 1'b0;
        a_payload_sel <= 1'b0;
        q_valid    <= 1'b0;
        d_valid    <= 1'b0;
        d_payload_sel <= 1'b0;
        d_vaddr_src_u <= 1'b0;
        h_valid    <= 1'b0;
        dc_fast_pipe_valid <= 1'b0;
        base_mshr_pipe_valid <= 1'b0;
        ph_valid   <= 1'b0;
        u_valid    <= 1'b0;
        u_payload_sel <= 1'b0;
        pair_slot_valid <= 1'b0;
        pair_wait_resp  <= 1'b0;
        pair_replay     <= 1'b0;
        fast_tok_valid <= {`ROB_SIZE{1'b0}};
        t_pending_visible <= {TOK_N{1'b0}};
        for (sj = 0; sj < TOK_N; sj = sj + 1)
            t_valid[sj] <= 1'b0;
        for (sj = 0; sj < MISS_N; sj = sj + 1) begin
            m_valid[sj] <= 1'b0;
            m_drop[sj]  <= 1'b0;
        end
        for (sj = 0; sj < STQ_N; sj = sj + 1)
            stq_v[sj] <= 1'b0;
        stq_y_valid <= 1'b0;
    end else if (flush_i) begin
        a_valid    <= 1'b0;
        a_payload_sel <= 1'b0;
        q_valid    <= 1'b0;
        d_valid    <= 1'b0;
        d_payload_sel <= 1'b0;
        d_vaddr_src_u <= 1'b0;
        h_valid    <= 1'b0;
        dc_fast_pipe_valid <= 1'b0;
        base_mshr_pipe_valid <= 1'b0;
        ph_valid   <= 1'b0;
        u_valid    <= 1'b0;
        u_payload_sel <= 1'b0;
        pair_slot_valid <= 1'b0;
        pair_wait_resp  <= 1'b0;
        pair_replay     <= 1'b0;
        fast_tok_valid <= {`ROB_SIZE{1'b0}};
        t_pending_visible <= {TOK_N{1'b0}};
        for (sj = 0; sj < TOK_N; sj = sj + 1)
            t_valid[sj] <= 1'b0;
        for (sj = 0; sj < STQ_N; sj = sj + 1)
            stq_v[sj] <= 1'b0;
        stq_y_valid <= 1'b0;
        for (sj = 0; sj < MISS_N; sj = sj + 1) begin
            m_valid[sj] <= 1'b0;
            m_drop[sj]  <= 1'b0;
        end
    end else begin
        ph_valid <= 1'b0;

        // The inactive A bank is unobservable, so continuously preload the
        // exact candidate selected by the original pair/q/issue priority.
        // The late a_go decision below changes only a_payload_sel.
        if (a_payload_sel) begin
            a_robid_bank0 <= a_payload_robid;
            a_mem_op_bank0 <= a_payload_mem_op;
            a_is_cacop_bank0 <= a_payload_is_cacop;
            a_cacop_op_bank0 <= a_payload_cacop_op;
            a_base_bank0 <= a_payload_base;
            a_base_ready_bank0 <= a_payload_base_ready;
            a_base_robid_bank0 <= a_payload_base_robid;
            a_imm_bank0 <= a_payload_imm;
            a_wdata_bank0 <= a_payload_wdata_primary;
            a_wdata_refill_bank0 <= a_payload_wdata_refill;
            a_wdata_refill_sel_bank0 <= a_payload_wdata_refill_sel;
            a_wdata_ready_bank0 <= a_payload_wdata_ready;
            a_wdata_robid_bank0 <= a_payload_wdata_robid;
        end else begin
            a_robid_bank1 <= a_payload_robid;
            a_mem_op_bank1 <= a_payload_mem_op;
            a_is_cacop_bank1 <= a_payload_is_cacop;
            a_cacop_op_bank1 <= a_payload_cacop_op;
            a_base_bank1 <= a_payload_base;
            a_base_ready_bank1 <= a_payload_base_ready;
            a_base_robid_bank1 <= a_payload_base_robid;
            a_imm_bank1 <= a_payload_imm;
            a_wdata_bank1 <= a_payload_wdata_primary;
            a_wdata_refill_bank1 <= a_payload_wdata_refill;
            a_wdata_refill_sel_bank1 <= a_payload_wdata_refill_sel;
            a_wdata_ready_bank1 <= a_payload_wdata_ready;
            a_wdata_robid_bank1 <= a_payload_wdata_robid;
        end

        // Preload the inactive normal-request bank without a clock enable.
        // When a_go is asserted below, toggling d_payload_sel makes the bank
        // written on this edge visible to D in the following cycle.
        if (d_payload_sel) begin
            d_vaddr_bank0  <= a_vaddr;
            d_st_data_bank0 <= a_st_data;
            d_robid_bank0 <= a_robid;
            d_mem_op_bank0 <= a_mem_op;
            d_is_cacop_bank0 <= a_is_cacop;
            d_is_store_bank0 <= a_is_store_op && !a_is_cacop;
            d_is_load_bank0 <= a_is_load_op && !a_is_cacop;
`ifdef SOC_USE
`else
            d_is_uart_lsr_bank0 <= a_is_uart_lsr;
`endif
            d_paddr_bank0 <= a_paddr;
            d_st_strb_bank0 <= a_st_strb;
            d_wdata_ready_bank0 <= a_wdata_ready_eff;
            d_wdata_robid_bank0 <= a_wdata_robid;
            d_size_bank0 <= a_size;
            d_uncached_bank0 <= a_uncached;
            d_excp_bank0 <= a_excp;
            d_pair_valid_bank0 <= a_pair_safe;
            d_pair_robid_bank0 <= pair_slot_robid;
            d_pair_mem_op_bank0 <= pair_slot_mem_op;
            d_pair_vaddr_bank0 <= pair_slot_vaddr;
            d_pair_paddr_bank0 <= a_pair_paddr;
            d_pair_size_bank0 <= pair_slot_size;
        end else begin
            d_vaddr_bank1  <= a_vaddr;
            d_st_data_bank1 <= a_st_data;
            d_robid_bank1 <= a_robid;
            d_mem_op_bank1 <= a_mem_op;
            d_is_cacop_bank1 <= a_is_cacop;
            d_is_store_bank1 <= a_is_store_op && !a_is_cacop;
            d_is_load_bank1 <= a_is_load_op && !a_is_cacop;
`ifdef SOC_USE
`else
            d_is_uart_lsr_bank1 <= a_is_uart_lsr;
`endif
            d_paddr_bank1 <= a_paddr;
            d_st_strb_bank1 <= a_st_strb;
            d_wdata_ready_bank1 <= a_wdata_ready_eff;
            d_wdata_robid_bank1 <= a_wdata_robid;
            d_size_bank1 <= a_size;
            d_uncached_bank1 <= a_uncached;
            d_excp_bank1 <= a_excp;
            d_pair_valid_bank1 <= a_pair_safe;
            d_pair_robid_bank1 <= pair_slot_robid;
            d_pair_mem_op_bank1 <= pair_slot_mem_op;
            d_pair_vaddr_bank1 <= pair_slot_vaddr;
            d_pair_paddr_bank1 <= a_pair_paddr;
            d_pair_size_bank1 <= pair_slot_size;
        end
        // The dedicated reload bank captures the old parked payload even on
        // a same-edge park/reload swap (NBA ordering), so u_reload does not
        // drive any wide D-stage payload control.
        d_vaddr_u_bank <= u_vaddr;
        d_robid_u_bank <= u_robid;
        d_mem_op_u_bank <= u_mem_op;
        d_paddr_u_bank <= u_paddr;
        d_size_u_bank <= u_size;
`ifdef SOC_USE
`else
        d_is_uart_lsr_u_bank <= u_is_uart_lsr;
`endif

        // The inactive UC-park bank is unobservable.  Preload it from the
        // current registered D payload so the late d_park decision toggles
        // only one selector bit.  On a same-edge park/reload swap, the D
        // reload bank above sees the old selected U payload while this bank
        // captures the new parked D payload.
        if (u_payload_sel) begin
            u_robid_bank0 <= d_robid;
            u_mem_op_bank0 <= d_mem_op;
            u_vaddr_bank0 <= d_vaddr;
            u_paddr_bank0 <= d_paddr;
            u_size_bank0 <= d_size;
`ifdef SOC_USE
`else
            u_is_uart_lsr_bank0 <= d_is_uart_lsr;
`endif
        end else begin
            u_robid_bank1 <= d_robid;
            u_mem_op_bank1 <= d_mem_op;
            u_vaddr_bank1 <= d_vaddr;
            u_paddr_bank1 <= d_paddr;
            u_size_bank1 <= d_size;
`ifdef SOC_USE
`else
            u_is_uart_lsr_bank1 <= d_is_uart_lsr;
`endif
        end

        // Payload and tag are deliberately captured without a clock enable;
        // only the one-bit valid path depends on the raw DCache hit result.
        dc_fast_pipe_valid <= local_dc_hit_valid;
        dc_fast_pipe_robid <= dc_resp_robid_i;
        dc_fast_pipe_data  <= local_dc_hit_data;
        base_mshr_pipe_valid <= mshr_return;
        base_mshr_pipe_robid <= m_robid[m_match_idx];
        base_mshr_pipe_data  <= local_refill_data;

        // A refill that loses the single architectural WB port still fills a
        // waiting store locally; the store completes on the following cycle.
        if (d_wdata_refill_hit) begin
            if (d_payload_sel) begin
                d_wdata_ready_bank1 <= 1'b1;
                d_st_data_bank1 <= d_refill_st_data;
            end else begin
                d_wdata_ready_bank0 <= 1'b1;
                d_st_data_bank0 <= d_refill_st_data;
            end
        end
        if (a_wdata_refill_hit) begin
            if (a_payload_sel) begin
                a_wdata_ready_bank1 <= 1'b1;
                a_wdata_bank1       <= a_return_data;
                a_wdata_refill_sel_bank1 <= 1'b0;
            end else begin
                a_wdata_ready_bank0 <= 1'b1;
                a_wdata_bank0       <= a_return_data;
                a_wdata_refill_sel_bank0 <= 1'b0;
            end
        end
        if (q_wdata_refill_hit) begin
            q_wdata_ready <= 1'b1;
            q_wdata       <= q_return_data;
            q_wdata_refill_sel <= 1'b0;
        end
        if (a_base_refill_hit) begin
            if (a_payload_sel) begin
                a_base_ready_bank1 <= 1'b1;
                a_base_bank1       <= a_base_return_data;
            end else begin
                a_base_ready_bank0 <= 1'b1;
                a_base_bank0       <= a_base_return_data;
            end
        end
        if (q_base_refill_hit) begin
            q_base_ready <= 1'b1;
            q_base       <= q_base_return_data;
        end

        // Keep the wide pair payload input registers off issue_accept.  When
        // the slot is empty their contents are don't-care, so capture the
        // candidate bundle regardless of valid and let only the one-bit
        // pair_slot_valid depend on the timing-sensitive issue decision.
        if (!pair_slot_valid) begin
            pair_first_robid <= issue_robid_i;
            pair_slot_robid  <= issue_pair_robid_i;
            pair_slot_mem_op <= issue_pair_mem_op_i;
            pair_slot_vaddr  <= issue_pair_base_i + issue_pair_imm_i;
        end
        if (issue_pair_accept) begin
            pair_slot_valid <= 1'b1;
            pair_wait_resp  <= 1'b0;
            pair_replay     <= 1'b0;
        end

        // A safe pair rides the primary cache lookup.  AGU fallback ownership
        // is held by pair_agu_replay; pair_replay below is reserved for
        // runtime DCache rejection/miss.
        if (dc_fire && d_pair_valid) begin
            // d_pair_paddr is already registered at the AGU/DC boundary.
            // Capture it when the paired cache request is actually accepted,
            // keeping the h_robid -> a_go/a_pair_match cone off this wide
            // register's clock enable.
            pair_slot_paddr <= d_pair_paddr;
            if (dc_pair_valid_o) begin
                pair_wait_resp <= 1'b1;
            end else begin
                pair_replay <= 1'b1;
            end
        end else if (d_done && d_pair_valid) begin
            pair_replay <= 1'b1;
        end

        if (pair_hit_return) begin
            ph_valid  <= 1'b1;
            ph_robid  <= pair_slot_robid;
            ph_data   <= shaped_pair;
            ph_vaddr  <= pair_slot_vaddr;
            ph_paddr  <= pair_slot_paddr;
            ph_size   <= pair_slot_size;
            pair_slot_valid <= 1'b0;
            pair_wait_resp  <= 1'b0;
            pair_replay     <= 1'b0;
        end else if (pair_miss_return) begin
            pair_wait_resp <= 1'b0;
            pair_replay    <= 1'b1;
        end else if (pair_replay_take) begin
            pair_slot_valid <= 1'b0;
            pair_wait_resp  <= 1'b0;
            pair_replay     <= 1'b0;
        end
        if ((dc_return || dc_missed) && fast_tok_valid[dc_resp_robid_i])
            fast_tok_valid[dc_resp_robid_i] <= 1'b0;
        if (dc_fire)
            fast_tok_valid[d_robid] <= 1'b1;

        // The descriptor may be consumed combinationally on this response
        // cycle, but must not remain visible on a held response next cycle.
        if (dc_return || dc_missed)
            t_pending_visible[t_match_idx] <= 1'b0;

        // ---- DCache 前端 token：返回/移交 miss 清项，新请求 addr_ok 分配 ----
        if (dc_return || dc_missed)
            t_valid[t_match_idx] <= 1'b0;
        if (dc_fire) begin
            t_valid[t_free_idx]    <= 1'b1;
            t_pending_visible[t_free_idx] <= d_is_plain_load && !d_uncached;
            t_robid[t_free_idx]    <= d_robid;
            t_mem_op[t_free_idx]   <= d_mem_op;
            t_vaddr[t_free_idx]    <= d_vaddr;
            t_paddr[t_free_idx]    <= d_paddr;
            t_size[t_free_idx]     <= d_size;
            t_uncached[t_free_idx] <= d_uncached;
        end

        // ---- miss 槽：返回清槽 / miss 分配 ----
        if (mshr_return || mshr_return_drop) begin
            m_valid[m_match_idx] <= 1'b0;
            m_drop [m_match_idx] <= 1'b0;
        end
        if (dc_missed && m_has_free) begin
            m_valid[m_free_idx]  <= 1'b1;
            m_drop [m_free_idx]  <= 1'b0;
            m_robid[m_free_idx]  <= t_robid[t_match_idx];
            m_mem_op[m_free_idx] <= t_mem_op[t_match_idx];
            m_vaddr[m_free_idx]  <= t_vaddr[t_match_idx];
            m_paddr[m_free_idx]  <= t_paddr[t_match_idx];
            m_size [m_free_idx]  <= t_size[t_match_idx];
        end

        // ---- hold 暂存槽 ----
        if (wb_hold_case) begin
            h_valid <= 1'b0;
        end
        if (hold_cap_dc || hold_cap_stq || hold_cap_sb) begin
            h_valid <= 1'b1;
            h_robid <= hold_cap_dc ? t_robid[t_match_idx] : d_robid;
            h_data  <= hold_cap_dc ? shaped_dc_token
                     : hold_cap_stq ? shaped_stq_d : shaped_sb_d;
            h_vaddr <= hold_cap_dc ? t_vaddr[t_match_idx] : d_vaddr;
            h_paddr <= hold_cap_dc ? t_paddr[t_match_idx] : d_paddr;
            h_size  <= hold_cap_dc ? t_size[t_match_idx] : d_size;
        end

        // ---- 顺序保护：STQ 入/出 ----
        for (sj = 0; sj < STQ_N; sj = sj + 1) begin
            if (stq_done[sj])
                stq_v[sj] <= 1'b0;
            // Invalid-slot payload is unobservable.  Preload every free slot
            // from the registered D stage so the late writeback decision only
            // sets valid and the last-entry index on a real store push.
            if (!stq_v[sj]) begin
                stq_id[sj]   <= d_robid;
                stq_pa[sj]   <= d_paddr;
                stq_data[sj] <= d_st_data_eff;
                stq_strb[sj] <= d_st_strb;
                stq_uc[sj]   <= d_uncached;
            end
        end
        // Mirror retirement uses the same valid/ROBID predicate as stq_done.
        // A same-cycle push is assigned later and therefore wins, matching
        // the array update ordering and making the new store the youngest.
        if (stq_y_valid && st_ret_v_r && (stq_y_id == st_ret_id_r))
            stq_y_valid <= 1'b0;
        if (wb_st_case && d_is_store) begin : stq_push
            integer sk;
            reg pushed;
            stq_y_valid <= 1'b1;
            stq_y_id    <= d_robid;
            stq_y_pa    <= d_paddr;
            stq_y_data  <= d_st_data_eff;
            stq_y_strb  <= d_st_strb;
            stq_y_uc    <= d_uncached;
            pushed = 1'b0;
            for (sk = 0; sk < STQ_N; sk = sk + 1) begin
                if (!pushed && !stq_v[sk]) begin
                    stq_v[sk]    <= 1'b1;
                    pushed = 1'b1;
                end
            end
        end

        // ---- DC 级 / UC park / reload ----
        if (d_done && !u_reload && !a_go) d_valid <= 1'b0;

        // park：把年轻 UC 挪到 u（可与 u_reload 交换）
        if (d_park) begin
            u_valid  <= 1'b1;
            u_payload_sel <= ~u_payload_sel;
        end else if (u_reload) begin
            u_valid <= 1'b0;
        end

        // reload：u 到头灌回 DC（NBA 读旧 u；与 d_park 交换时自然交叉）
        if (u_reload) begin
            d_valid    <= 1'b1;
            d_vaddr_src_u <= 1'b1;
        end else if (a_go) begin
            d_valid    <= 1'b1;
            d_payload_sel <= ~d_payload_sel;
            d_vaddr_src_u <= 1'b0;
        end
        // ---- rs_mem -> issue skid -> AGU ----
        // Oldest data in q wins when A advances. A new issue enters A
        // directly in the common no-stall case, or parks in q while A stalls.
        if (a_go || !a_valid) begin
            // The inactive bank was loaded above on this same edge.  NBA
            // semantics make the toggled selector expose that new payload.
            a_payload_sel <= ~a_payload_sel;
            if (pair_replay_take) begin
                a_valid    <= 1'b1;
            end else if (q_valid) begin
                a_valid    <= 1'b1;
                q_valid    <= 1'b0;
            end else begin
                a_valid    <= issue_accept;
            end
        end else if (!q_valid) begin
            // Only q_valid depends on A-stage backpressure.  The payload is
            // written independently below while the invalid slot is free.
            q_valid    <= issue_accept;
        end
        // The queue payload is unobservable while q_valid is clear.  Capture
        // it on every empty-slot edge, including the direct issue-to-A case,
        // so L2-driven a_go cannot become the payload registers' clock-enable.
        if (!q_valid) begin
            q_robid    <= issue_robid_i;
            q_mem_op   <= issue_mem_op_i;
            q_is_cacop <= issue_is_cacop_i;
            q_cacop_op <= issue_cacop_op_i;
            q_base     <= issue_base_eff;
            q_base_ready <= issue_base_ready_eff;
            q_base_robid <= issue_base_robid_i;
            q_imm      <= issue_imm_i;
            q_wdata    <= issue_wdata_i;
            q_wdata_refill <= issue_return_data;
            q_wdata_refill_sel <= issue_wdata_refill_hit;
            q_wdata_ready <= issue_wdata_ready_eff;
            q_wdata_robid <= issue_wdata_robid_i;
        end
    end
end

// DC 级 early2：仅在 `LSU_EARLY2_ENABLE` 且「写回有保证」时唤醒。
// bypass=0 时必须关 early2（否则依赖可能在 hold 写回前被唤醒，Linux hang）。
wire d_early_ok = (`LSU_EARLY2_ENABLE != 0)
                && (dc_return || (d_valid && d_is_load && !d_excp_any && !d_is_cacop))
                && !wb_mshr_case
                && (dc_return || (sb_ready && !hold_cap_dc));
assign early_wakeup_valid_o = d_early_ok && !flush_i && !reset;
assign early_wakeup_robid_o = dc_return ? t_robid[t_match_idx] : d_robid;

`ifdef SYNTHESIS
// synthesis translate_off
reg [63:0] lsu_store_order_stall_cyc;
reg [63:0] lsu_dc_wait_cyc;
reg [63:0] lsu_stq_full_cyc;
reg [63:0] lsu_issue_accept_cnt;
reg [63:0] lsu_q_full_cyc;
reg [63:0] lsu_a_block_cyc;
reg [63:0] lsu_d_cached_load_cyc;
reg [63:0] lsu_d_uncached_load_cyc;
reg [63:0] lsu_d_uncached_not_head_cyc;
reg [63:0] lsu_d_uncached_head_cyc;
reg [63:0] lsu_d_no_token_cyc;
reg [63:0] lsu_d_sb_wait_cyc;
reg [63:0] lsu_d_h_wait_cyc;
reg [63:0] lsu_d_dc_not_accept_cyc;
reg [63:0] lsu_dc_fire_cnt;
reg [63:0] lsu_dc_cached_fire_cnt;
reg [63:0] lsu_dc_uncached_fire_cnt;
reg [63:0] lsu_pair_accept_cnt;
reg [63:0] lsu_pair_agu_safe_cnt;
reg [63:0] lsu_pair_agu_fallback_cnt;
reg [63:0] lsu_pair_dc_fire_cnt;
reg [63:0] lsu_pair_hit_cnt;
reg [63:0] lsu_pair_miss_cnt;
reg [63:0] lsu_pending_load_set_cnt;
reg [63:0] lsu_pending_load_clear_hit_cnt;
reg [63:0] lsu_pending_load_clear_miss_cnt;
reg [63:0] lsu_store_fill_hit_pipe_cnt;
reg [63:0] lsu_store_fill_mshr_cnt;
reg [63:0] lsu_store_wait_slot_cyc;
reg [63:0] lsu_base_fill_hit_pipe_cnt;
reg [63:0] lsu_base_fill_mshr_cnt;
reg [63:0] lsu_base_wait_slot_cyc;
reg [5:0]  lsu_pending_load_occ_now;
reg [5:0]  lsu_pending_load_occ_max;
reg [7:0]  lsu_stq_occ_now;
reg [7:0]  lsu_stq_occ_max;
reg [63:0] lsu_stq_occ_sum;
integer    lsu_stq_pc_i;
integer    lsu_pending_pc_i;
wire lsu_q_wait_store = q_valid && !q_wdata_ready
                       && (q_mem_op[`MEM_OP_ST_W] || q_mem_op[`MEM_OP_ST_B]
                           || q_mem_op[`MEM_OP_ST_H]);
wire lsu_a_wait_store = a_valid && !a_wdata_ready
                       && (a_mem_op[`MEM_OP_ST_W] || a_mem_op[`MEM_OP_ST_B]
                           || a_mem_op[`MEM_OP_ST_H]);
wire lsu_d_wait_store = d_valid && d_is_store && !d_wdata_ready;
wire [2:0] lsu_store_wait_slots = {2'b0, lsu_q_wait_store}
                                + {2'b0, lsu_a_wait_store}
                                + {2'b0, lsu_d_wait_store};
wire [1:0] lsu_base_wait_slots = {1'b0, q_valid && !q_base_ready}
                               + {1'b0, a_valid && !a_base_ready};
always @(*) begin
    lsu_pending_load_occ_now = 6'd0;
    for (lsu_pending_pc_i = 0; lsu_pending_pc_i < TOK_N;
         lsu_pending_pc_i = lsu_pending_pc_i + 1) begin
        if (pending_load_valid_o[lsu_pending_pc_i])
            lsu_pending_load_occ_now = lsu_pending_load_occ_now + 6'd1;
    end
end
always @(*) begin
    lsu_stq_occ_now = 8'd0;
    for (lsu_stq_pc_i = 0; lsu_stq_pc_i < STQ_N; lsu_stq_pc_i = lsu_stq_pc_i + 1)
        if (stq_v[lsu_stq_pc_i] && !stq_done[lsu_stq_pc_i])
            lsu_stq_occ_now = lsu_stq_occ_now + 8'd1;
end
always @(posedge clk) begin
    if (reset) begin
        lsu_store_order_stall_cyc <= 64'd0;
        lsu_dc_wait_cyc           <= 64'd0;
        lsu_stq_full_cyc          <= 64'd0;
        lsu_issue_accept_cnt      <= 64'd0;
        lsu_q_full_cyc            <= 64'd0;
        lsu_a_block_cyc           <= 64'd0;
        lsu_d_cached_load_cyc     <= 64'd0;
        lsu_d_uncached_load_cyc   <= 64'd0;
        lsu_d_uncached_not_head_cyc <= 64'd0;
        lsu_d_uncached_head_cyc   <= 64'd0;
        lsu_d_no_token_cyc        <= 64'd0;
        lsu_d_sb_wait_cyc         <= 64'd0;
        lsu_d_h_wait_cyc          <= 64'd0;
        lsu_d_dc_not_accept_cyc   <= 64'd0;
        lsu_dc_fire_cnt           <= 64'd0;
        lsu_dc_cached_fire_cnt    <= 64'd0;
        lsu_dc_uncached_fire_cnt  <= 64'd0;
        lsu_pair_accept_cnt       <= 64'd0;
        lsu_pair_agu_safe_cnt     <= 64'd0;
        lsu_pair_agu_fallback_cnt <= 64'd0;
        lsu_pair_dc_fire_cnt      <= 64'd0;
        lsu_pair_hit_cnt          <= 64'd0;
        lsu_pair_miss_cnt         <= 64'd0;
        lsu_pending_load_set_cnt  <= 64'd0;
        lsu_pending_load_clear_hit_cnt <= 64'd0;
        lsu_pending_load_clear_miss_cnt <= 64'd0;
        lsu_store_fill_hit_pipe_cnt <= 64'd0;
        lsu_store_fill_mshr_cnt   <= 64'd0;
        lsu_store_wait_slot_cyc   <= 64'd0;
        lsu_base_fill_hit_pipe_cnt <= 64'd0;
        lsu_base_fill_mshr_cnt    <= 64'd0;
        lsu_base_wait_slot_cyc    <= 64'd0;
        lsu_pending_load_occ_max  <= 6'd0;
        lsu_stq_occ_max           <= 8'd0;
        lsu_stq_occ_sum           <= 64'd0;
    end else if (!flush_i) begin
        if (d_valid && d_is_load && !d_excp_any && store_order_block)
            lsu_store_order_stall_cyc <= lsu_store_order_stall_cyc + 64'd1;
        if (d_valid && d_req_sent && !d_drop)
            lsu_dc_wait_cyc <= lsu_dc_wait_cyc + 64'd1;
        if (stq_full)
            lsu_stq_full_cyc <= lsu_stq_full_cyc + 64'd1;
        if (issue_accept)
            lsu_issue_accept_cnt <= lsu_issue_accept_cnt + 64'd1;
        if (q_valid)
            lsu_q_full_cyc <= lsu_q_full_cyc + 64'd1;
        if (a_valid && !a_go)
            lsu_a_block_cyc <= lsu_a_block_cyc + 64'd1;
        if (d_valid && d_is_load && !d_uncached)
            lsu_d_cached_load_cyc <= lsu_d_cached_load_cyc + 64'd1;
        if (d_valid && d_is_load && d_uncached) begin
            lsu_d_uncached_load_cyc <= lsu_d_uncached_load_cyc + 64'd1;
            if (d_at_head)
                lsu_d_uncached_head_cyc <= lsu_d_uncached_head_cyc + 64'd1;
            else
                lsu_d_uncached_not_head_cyc <= lsu_d_uncached_not_head_cyc + 64'd1;
        end
        if (d_valid && d_is_load && !t_has_free)
            lsu_d_no_token_cyc <= lsu_d_no_token_cyc + 64'd1;
        if (d_valid && d_is_load && sb_query_needed && !sb_query_ready)
            lsu_d_sb_wait_cyc <= lsu_d_sb_wait_cyc + 64'd1;
        if (d_valid && d_is_load && h_valid)
            lsu_d_h_wait_cyc <= lsu_d_h_wait_cyc + 64'd1;
        if (dc_req_o && !dc_addr_ok_i)
            lsu_d_dc_not_accept_cyc <= lsu_d_dc_not_accept_cyc + 64'd1;
        if (dc_fire) begin
            lsu_dc_fire_cnt <= lsu_dc_fire_cnt + 64'd1;
            if (d_uncached)
                lsu_dc_uncached_fire_cnt <= lsu_dc_uncached_fire_cnt + 64'd1;
            else
                lsu_dc_cached_fire_cnt <= lsu_dc_cached_fire_cnt + 64'd1;
        end
        if (issue_pair_accept)
            lsu_pair_accept_cnt <= lsu_pair_accept_cnt + 64'd1;
        if (a_go && a_pair_match) begin
            if (a_pair_safe)
                lsu_pair_agu_safe_cnt <= lsu_pair_agu_safe_cnt + 64'd1;
            else
                lsu_pair_agu_fallback_cnt <= lsu_pair_agu_fallback_cnt + 64'd1;
        end
        if (dc_fire && dc_pair_valid_o)
            lsu_pair_dc_fire_cnt <= lsu_pair_dc_fire_cnt + 64'd1;
        if (pair_hit_return)
            lsu_pair_hit_cnt <= lsu_pair_hit_cnt + 64'd1;
        if (pair_miss_return)
            lsu_pair_miss_cnt <= lsu_pair_miss_cnt + 64'd1;
        if (pending_load_set)
            lsu_pending_load_set_cnt <= lsu_pending_load_set_cnt + 64'd1;
        if (pending_load_clear_hit)
            lsu_pending_load_clear_hit_cnt <=
                lsu_pending_load_clear_hit_cnt + 64'd1;
        if (pending_load_clear_miss)
            lsu_pending_load_clear_miss_cnt <=
                lsu_pending_load_clear_miss_cnt + 64'd1;
        if (q_wdata_fast_hit || a_wdata_fast_hit || d_wdata_fast_hit
            || issue_wdata_fast_hit)
            lsu_store_fill_hit_pipe_cnt <= lsu_store_fill_hit_pipe_cnt + 64'd1;
        if (q_wdata_mshr_hit || a_wdata_mshr_hit || d_wdata_mshr_hit
            || issue_wdata_mshr_hit)
            lsu_store_fill_mshr_cnt <= lsu_store_fill_mshr_cnt + 64'd1;
        if (q_base_fast_hit || a_base_fast_hit || issue_base_fast_hit)
            lsu_base_fill_hit_pipe_cnt <= lsu_base_fill_hit_pipe_cnt + 64'd1;
        if (q_base_mshr_hit || a_base_mshr_hit || issue_base_mshr_hit)
            lsu_base_fill_mshr_cnt <= lsu_base_fill_mshr_cnt + 64'd1;
        lsu_store_wait_slot_cyc <= lsu_store_wait_slot_cyc
                                 + {61'd0, lsu_store_wait_slots};
        lsu_base_wait_slot_cyc <= lsu_base_wait_slot_cyc
                               + {62'd0, lsu_base_wait_slots};
        if (lsu_pending_load_occ_now > lsu_pending_load_occ_max)
            lsu_pending_load_occ_max <= lsu_pending_load_occ_now;
        lsu_stq_occ_sum <= lsu_stq_occ_sum + {56'd0, lsu_stq_occ_now};
        if (lsu_stq_occ_now > lsu_stq_occ_max)
            lsu_stq_occ_max <= lsu_stq_occ_now;
    end
end
// synthesis translate_on
`endif

endmodule

module lsu_prio_low_comb #(
    parameter integer N = 2,
    parameter integer W = 1
)(
    input  wire [N-1:0] mask_i,
    output reg  [W-1:0] idx_o
);
integer prio_i;
reg found;
always @(*) begin
    idx_o = {W{1'b0}};
    found = 1'b0;
    for (prio_i = 0; prio_i < N; prio_i = prio_i + 1) begin
        if (mask_i[prio_i] && !found) begin
            idx_o = prio_i[W-1:0];
            found = 1'b1;
        end
    end
end
endmodule

module lsu_load_shape_comb(
    input  wire [31:0] word_i,
    input  wire [7:4]  op_i,
    input  wire [1:0]  off_i,
    output reg  [31:0] data_o
);
reg [7:0] byte_value;
reg [15:0] half_value;
always @(*) begin
    byte_value = word_i[8*off_i +: 8];
    half_value = off_i[1] ? word_i[31:16] : word_i[15:0];
    if (op_i[`MEM_OP_LD_B])
        data_o = {{24{byte_value[7]}}, byte_value};
    else if (op_i[`MEM_OP_LD_BU])
        data_o = {24'b0, byte_value};
    else if (op_i[`MEM_OP_LD_H])
        data_o = {{16{half_value[15]}}, half_value};
    else if (op_i[`MEM_OP_LD_HU])
        data_o = {16'b0, half_value};
    else
        data_o = word_i;
end
endmodule
