// ============================================================
// lsu 模块（访存单元：AGU + DC 两级流水 + miss 槽非阻塞 load）
// ------------------------------------------------------------
// 结构：
// - AGU 级：vaddr=base+imm -> MMU 组合翻译 -> ALE/ADEM/TLB 异常 ->
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
    input  wire [31:0]                issue_wdata_i,       // rd 值（store 数据）
    input  wire [31:0]                issue_imm_i,         // 偏移
    output wire                       lsu_ready_o,         // AGU 级可接收

    // ---------------- D 侧地址翻译（连 mmu D 通道，组合）----------------
    output wire                       mmu_d_req_o,
    output wire [31:0]                mmu_d_vaddr_o,
    output wire                       mmu_d_is_store_o,    // 区分 PIL/PIS 与 PME
    input  wire [31:0]                mmu_d_paddr_i,
    input  wire [1:0]                 mmu_d_mat_i,
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
    output wire                       dc_uncached_o,
    output wire [`ROB_W-1:0]          dc_robid_o,          // 随 dc_req：D$ miss 锁存配对
    input  wire                       dc_addr_ok_i,
    input  wire                       dc_data_ok_i,
    input  wire [31:0]                dc_rdata_i,
    output wire                       dc_cancel_o,         // 冲刷：通知 dcache 杀 load MSHR
    // ---- 非阻塞 miss 扩展（配合 dcache MSHR）----
    input  wire                       dc_miss_i,           // 在途 load 移入 MSHR（一拍）
    input  wire                       dc_mshr_data_ok_i,   // MSHR 重填数据返回（一拍）
    input  wire [31:0]                dc_mshr_rdata_i,
    input  wire [`ROB_W-1:0]          dc_mshr_robid_i,     // 与 data_ok 同拍

    // ---------------- store buffer 前递查询（DC 级组合）----------------
    output wire [31:2]                sb_query_paddr_o,
    output wire                       sb_query_uncached_o, // 本查询来自 uncached load
    input  wire                       sb_query_hit_i,      // 整字可由 SB 合并前递
    input  wire [31:0]                sb_query_data_i,
    input  wire                       sb_query_partial_i,  // 部分命中：load 等排空重试

    // ---------------- uncached load 许可（与 ROB head 比较）----------------
    input  wire [`ROB_W-1:0]          rob_head_robid_i,    // 编码：MSB=槽0 仍未提交，低位=head 对指针（顶层拼装，见 mycpu_top）
    input  wire                       rob_head_valid_i,
    // store 提交释放 STQ：与 SB push 同源；LSU 再打一拍，保证 SB valid 已可见
    input  wire                       st_retire_valid_i,
    input  wire [`ROB_W-1:0]          st_retire_robid_i,
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

    // ---------------- DC 级命中限定早唤醒（顶层可选择打一拍）----------------
    output wire                       early_wakeup_valid_o,
    output wire [`ROB_W-1:0]          early_wakeup_robid_o
);

// =====================================================================
// AGU 级
// =====================================================================
reg                    a_valid;
reg [`ROB_W-1:0]       a_robid;
reg [`MEM_OP_NUM-1:0]  a_mem_op;
reg                    a_is_cacop;
reg [4:3]              a_cacop_op;
reg [31:0]             a_base, a_wdata, a_imm;

wire [31:0] a_vaddr = a_base + a_imm;

wire a_is_store_op = a_mem_op[`MEM_OP_ST_W] | a_mem_op[`MEM_OP_ST_B]
                   | a_mem_op[`MEM_OP_ST_H] | a_mem_op[`MEM_OP_SC_W];
wire a_is_load_op  = a_mem_op[`MEM_OP_LD_W] | a_mem_op[`MEM_OP_LD_B]
                   | a_mem_op[`MEM_OP_LD_H] | a_mem_op[`MEM_OP_LD_BU]
                   | a_mem_op[`MEM_OP_LD_HU]| a_mem_op[`MEM_OP_LL_W];

// MMU 翻译（组合）：ALE / Index-cacop 抑制翻译请求
// ALE 检测（H 类要求 vaddr[0]==0，W/LL/SC 要求 vaddr[1:0]==00）
// VA 对齐先于地址翻译：ALE 时不得发 MMU，否则会与 TLBR 同拍置位，
// 否则 ALE 会与 TLBR 同拍置位，破坏异常优先级。
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

assign mmu_d_req_o      = a_valid && !a_no_trans;
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

// store 数据按地址对齐 + wstrb
wire [1:0] a_off = a_vaddr[1:0];
reg [31:0] a_st_data;
reg [3:0]  a_st_strb;
always @(*) begin
    if (a_mem_op[`MEM_OP_ST_B]) begin
        a_st_data = {4{a_wdata[7:0]}};
        a_st_strb = 4'b0001 << a_off;
    end else if (a_mem_op[`MEM_OP_ST_H]) begin
        a_st_data = {2{a_wdata[15:0]}};
        a_st_strb = a_off[1] ? 4'b1100 : 4'b0011;
    end else begin
        a_st_data = a_wdata;
        a_st_strb = 4'b1111;
    end
end

wire [2:0] a_size = a_mem_op[`MEM_OP_ST_B] | a_mem_op[`MEM_OP_LD_B] | a_mem_op[`MEM_OP_LD_BU] ? 3'd0
                  : a_mem_op[`MEM_OP_ST_H] | a_mem_op[`MEM_OP_LD_H] | a_mem_op[`MEM_OP_LD_HU] ? 3'd1
                  : 3'd2;

// =====================================================================
// DC 级
// =====================================================================
reg                    d_valid;
reg [`ROB_W-1:0]       d_robid;
reg [`MEM_OP_NUM-1:0]  d_mem_op;
reg                    d_is_cacop;
reg                    d_is_store, d_is_load;
reg [31:0]             d_vaddr, d_paddr;
reg [31:0]             d_st_data;
reg [3:0]              d_st_strb;
reg [2:0]              d_size;
reg                    d_uncached;
reg [`EXCP_NUM-1:0]    d_excp;
reg                    d_req_sent;      // DCache 已收下请求，等响应（data_ok 或 miss）
reg                    d_drop;          // 冲刷后丢弃下一个前端响应

// ---------------- miss 槽（MSHR 影子，深度 LSU_MISS_DEPTH）----------------
localparam MISS_N = `LSU_MISS_DEPTH;
localparam MISS_W = (MISS_N <= 1) ? 1 : $clog2(MISS_N);

function automatic [MISS_W-1:0] lsu_miss_prio_low;
    input [MISS_N-1:0] mask;
    integer k;
    reg found;
    begin
        lsu_miss_prio_low = {MISS_W{1'b0}};
        found = 1'b0;
        for (k = 0; k < MISS_N; k = k + 1) begin
            if (mask[k] && !found) begin
                lsu_miss_prio_low = k[MISS_W-1:0];
                found = 1'b1;
            end
        end
    end
endfunction

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
end
endgenerate
wire              m_has_free   = |m_free_oh;
wire [MISS_W-1:0] m_free_idx   = lsu_miss_prio_low(m_free_oh);
// 同 robid 时优先吃 drop 槽，避免冲刷后复用 robid 的新 load 吃到旧 MSHR 数据
wire [MISS_N-1:0] m_match_drop = m_match_oh & m_drop_oh;
wire [MISS_N-1:0] m_match_live = m_match_oh & ~m_drop_oh;
wire              m_match_vld  = |m_match_oh;
wire [MISS_W-1:0] m_match_idx  = (|m_match_drop)
                               ? lsu_miss_prio_low(m_match_drop)
                               : lsu_miss_prio_low(m_match_live);

// ---------------- 写回暂存槽（被高优先级抢口的瞬态完成）----------------
reg                    h_valid;
reg [`ROB_W-1:0]       h_robid;
reg [31:0]             h_data;          // 已整形
reg [31:0]             h_vaddr, h_paddr;
reg [2:0]              h_size;

// 年轻 UC load 旁路槽：未到 ROB 头时让出 DC，避免堵住年老访存
reg                    u_valid;
reg [`ROB_W-1:0]       u_robid;
reg [`MEM_OP_NUM-1:0]  u_mem_op;
reg [31:0]             u_vaddr, u_paddr;
reg [2:0]              u_size;

// ---------------- 顺序保护：未决 store 地址队列（WBed 尚未提交）----------------
localparam STQ_N = `STQ_DEPTH;
localparam STQ_W = (STQ_N <= 1) ? 1 : $clog2(STQ_N);

reg                  stq_v    [0:STQ_N-1];
reg [`ROB_W-1:0]     stq_id   [0:STQ_N-1];
reg [31:0]           stq_pa   [0:STQ_N-1];
reg [3:0]            stq_strb [0:STQ_N-1];
reg                  stq_uc   [0:STQ_N-1];

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

// load 访问字节掩码（实现见 mycpu.h）
wire [3:0] d_ld_bytes = mem_load_byte_mask(d_mem_op[7:4], d_vaddr[1:0]);

wire stq_any;
wire stq_any_uc;
wire stq_overlap;
wire [STQ_N-1:0] stq_hit_one;
generate
for (si = 0; si < STQ_N; si = si + 1) begin : g_stq_hz
    assign stq_hit_one[si] = stq_v[si] && !stq_done[si]
        && mem_st_ld_overlap(stq_pa[si][31:2], stq_strb[si], d_paddr[31:2], d_ld_bytes);
end
endgenerate

integer sj;
reg stq_any_r, stq_any_uc_r;
always @(*) begin
    stq_any_r = 1'b0;
    stq_any_uc_r = 1'b0;
    for (sj = 0; sj < STQ_N; sj = sj + 1) begin
        if (stq_v[sj] && !stq_done[sj]) begin
            stq_any_r = 1'b1;
            if (stq_uc[sj])
                stq_any_uc_r = 1'b1;
        end
    end
end
assign stq_any     = stq_any_r;
assign stq_any_uc  = stq_any_uc_r;
assign stq_overlap = |stq_hit_one;

wire stq_full;
reg stq_full_r;
always @(*) begin
    stq_full_r = 1'b1;
    for (sj = 0; sj < STQ_N; sj = sj + 1)
        if (!stq_v[sj] || stq_done[sj])
            stq_full_r = 1'b0;
end
assign stq_full = stq_full_r;

wire store_order_block = d_uncached ? stq_any : (stq_overlap || stq_any_uc);

// ---------------- DC 级行为 ----------------
wire d_excp_any = |d_excp;

// SB 前递查询（DC 级持续驱动）
assign sb_query_paddr_o    = d_paddr[31:2];
assign sb_query_uncached_o = d_valid && d_is_load && d_uncached;

// uncached load 许可：自己是最老未提交指令
wire d_at_head = (d_robid[`ROB_PAIR_W-1:0] == head_pair)
              && ((d_robid[`ROB_W-1] == 1'b0) || !head_s0_live)
              && rob_head_valid_i;

wire d_is_unc_load = d_valid && d_is_load && d_uncached && !d_excp_any;
assign uncached_ld_inflight_o = d_is_unc_load || u_valid;

// load 可以发起最终访问（SB 终查/DCache）的条件：
// - 未决 store 先提交（顺序保护）；
// - hold 槽空（防瞬态完成三方碰撞，见头注仲裁说明）；
// - uncached load 额外等：到 ROB 头 + SB 无 uncached 残留（query_partial）
wire d_ld_gate = !store_order_block && !h_valid
              && (!d_uncached || (d_at_head && !sb_query_partial_i));

// load 处理分支
wire d_sb_hit     = d_valid && d_is_load && !d_excp_any && !d_uncached && sb_query_hit_i;
wire d_sb_partial = d_valid && d_is_load && !d_excp_any && !d_uncached && sb_query_partial_i;

wire d_need_dc = d_valid && d_is_load && !d_excp_any && !d_sb_hit && !d_sb_partial;

// DCache 请求（保持至 addr_ok）
assign dc_req_o      = d_need_dc && d_ld_gate && !d_req_sent && !d_drop && !flush_i;
assign dc_vindex_o   = d_vaddr[11:5];
assign dc_paddr_o    = d_paddr;
assign dc_size_o     = d_size;
assign dc_uncached_o = d_uncached;
assign dc_robid_o    = d_robid;
assign dc_cancel_o   = flush_i;

wire dc_fire   = dc_req_o && dc_addr_ok_i;
// 前端响应二选一：数据返回（命中/uncached）或 miss 移交
wire dc_return = d_req_sent && dc_data_ok_i && !d_drop;
wire dc_return_drop = d_req_sent && dc_data_ok_i && d_drop;
wire dc_missed = d_req_sent && dc_miss_i;

// D$ miss 通知与 MSHR 分配保持同拍；完成条件寄存一拍，隔离
// req_paddr→miss→lsu_ready→rs_mem 组合路径。
reg dc_miss_done_r;
always @(posedge clk) begin
    if (reset)
        dc_miss_done_r <= 1'b0;
    else if (flush_i)
        dc_miss_done_r <= 1'b0;
    else
        dc_miss_done_r <= dc_missed && (d_drop || m_has_free);
end

// MSHR 重填返回（按 robid 配对；m_drop 时静默消费）
wire mshr_return      = m_match_vld && !m_drop[m_match_idx];
wire mshr_return_drop = m_match_vld &&  m_drop[m_match_idx];

// ---------------- load 数据整形 ----------------
function [31:0] shape_load;
    input [31:0] word;
    input [7:4] op;
    input [1:0] off;
    reg [7:0]  b;
    reg [15:0] h;
    begin
        b = word[8*off +: 8];
        h = off[1] ? word[31:16] : word[15:0];
        if (op[`MEM_OP_LD_B])       shape_load = {{24{b[7]}}, b};
        else if (op[`MEM_OP_LD_BU]) shape_load = {24'b0, b};
        else if (op[`MEM_OP_LD_H])  shape_load = {{16{h[15]}}, h};
        else if (op[`MEM_OP_LD_HU]) shape_load = {16'b0, h};
        else                        shape_load = word;
    end
endfunction

// ---------------- 写回仲裁（一拍一条，按年龄：miss 槽 > hold > DC 级）----------------
// DC 级完成源：
// 1) 异常：直接写回；2) store/cacop：直接写回；3) load：SB 命中或 DCache 返回
wire wb_mshr_case  = mshr_return;                       // 最老，最高优先
wire wb_hold_case  = !wb_mshr_case && h_valid;
wire dcst_ok       = !wb_mshr_case && !h_valid;         // DC 级静态源可用口
wire wb_excp_case  = dcst_ok && d_valid && d_excp_any;
wire wb_st_case    = dcst_ok && d_valid && !d_excp_any && (d_is_store || d_is_cacop)
                  && !(d_is_store && stq_full);
// SB 命中结果经 hold 暂存一拍，隔离逐字节前递合并到写回旁路的长组合路径。
// flush 会丢弃尚未提交的 hold 内容；store_order_block 保证 store→load 顺序。
wire sb_ready      = d_sb_hit && d_ld_gate && !store_order_block; // d_ld_gate 已含 !h_valid
wire wb_ld_sb_case = 1'b0;                               // SB 命中仍走 hold，切断 SB→RS 组合路径
// 启用 `LSU_DC_HIT_BYPASS` 时 D$ 命中同拍写回；MSHR 抢口时仍进入 hold。
wire wb_ld_dc_case = (`LSU_DC_HIT_BYPASS != 0) && dcst_ok && dc_return;
wire hold_cap_dc   = dc_return && ((`LSU_DC_HIT_BYPASS != 0) ? wb_mshr_case : 1'b1);
// 所有就绪 SB 命中都进 hold(与 hold_cap_dc 互斥:同一 d 级 load 不会既 SB 命中又 DC 返回)
wire hold_cap_sb   = sb_ready && !hold_cap_dc;

assign wb_valid_o = (wb_mshr_case || wb_hold_case
                  || wb_excp_case || wb_st_case || wb_ld_sb_case || wb_ld_dc_case)
                  && !flush_i;
assign wb_robid_o = wb_mshr_case ? m_robid[m_match_idx]
                  : wb_hold_case ? h_robid
                  : d_robid;
assign wb_data_o  = wb_mshr_case  ? shape_load(dc_mshr_rdata_i, m_mem_op[m_match_idx][7:4], m_vaddr[m_match_idx][1:0])
                  : wb_hold_case  ? h_data
                  : wb_ld_sb_case ? shape_load(sb_query_data_i, d_mem_op[7:4], d_vaddr[1:0])
                  : wb_ld_dc_case ? shape_load(dc_rdata_i,      d_mem_op[7:4], d_vaddr[1:0])
                  : d_st_data;
assign wb_paddr_o = wb_mshr_case ? m_paddr[m_match_idx] : wb_hold_case ? h_paddr : d_paddr;
assign wb_vaddr_o = wb_mshr_case ? m_vaddr[m_match_idx] : wb_hold_case ? h_vaddr : d_vaddr;
assign wb_wstrb_o = (!wb_mshr_case && !wb_hold_case && d_is_store && !d_excp_any) ? d_st_strb : 4'b0;
assign wb_size_o  = wb_mshr_case ? m_size[m_match_idx] : wb_hold_case ? h_size : d_size;
assign wb_uncached_o = (!wb_mshr_case && !wb_hold_case) && d_uncached;
assign wb_excp_o  = (!wb_mshr_case && !wb_hold_case) ? d_excp : {`EXCP_NUM{1'b0}};

// ---------------- 流水推进 ----------------
// DC 级本拍腾空：写回成功、miss 完成条件到达，或已捕获进 hold。
wire d_done  = wb_excp_case || wb_st_case || wb_ld_sb_case || wb_ld_dc_case
             || dc_miss_done_r || dc_return_drop
             || hold_cap_dc || hold_cap_sb;

// 年轻 UC park（宽版）：仅对【比 DC 中 UC 更老】的 AGU 让位。
// 勿对更年轻 AGU 让位（会覆盖 u / 堵 RS）；u_valid 期间禁更年轻进 DC。
// u_valid 期间禁止年轻请求进入 DC，避免覆盖停车槽或破坏程序序。
wire d_uc_yield = d_is_unc_load && !d_at_head && !d_req_sent && !d_drop;
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
wire a_go     = a_valid && d_free && !u_reload && a_older_than_u;
wire d_park   = (d_uc_yield && a_go && !u_valid) || (d_uc_yield && u_reload);

assign lsu_ready_o = (!a_valid || a_go) && !flush_i;

always @(posedge clk) begin
    if (reset) begin
        a_valid    <= 1'b0;
        d_valid    <= 1'b0;
        d_drop     <= 1'b0;
        d_req_sent <= 1'b0;
        h_valid    <= 1'b0;
        u_valid    <= 1'b0;
        for (sj = 0; sj < MISS_N; sj = sj + 1) begin
            m_valid[sj] <= 1'b0;
            m_drop[sj]  <= 1'b0;
        end
        for (sj = 0; sj < STQ_N; sj = sj + 1)
            stq_v[sj] <= 1'b0;
    end else if (flush_i) begin
        a_valid    <= 1'b0;
        d_valid    <= 1'b0;
        h_valid    <= 1'b0;
        u_valid    <= 1'b0;
        for (sj = 0; sj < STQ_N; sj = sj + 1)
            stq_v[sj] <= 1'b0;
        d_drop     <= d_req_sent && !(dc_data_ok_i || dc_miss_i);
        d_req_sent <= 1'b0;
        for (sj = 0; sj < MISS_N; sj = sj + 1) begin
            if (m_match_vld && (sj[MISS_W-1:0] == m_match_idx)) begin
                m_valid[sj] <= 1'b0;
                m_drop[sj]  <= 1'b0;
            end else if (m_valid[sj]) begin
                m_drop[sj]  <= 1'b1;
            end
        end
    end else begin
        // ---- 前端 drop 配对（冲刷遗留的在途响应）----
        if (d_drop && (dc_data_ok_i || dc_miss_i)) begin
            d_drop <= 1'b0;
        end

        // ---- miss 槽：返回清槽 / miss 分配 ----
        if (mshr_return || mshr_return_drop) begin
            m_valid[m_match_idx] <= 1'b0;
            m_drop [m_match_idx] <= 1'b0;
        end
        if (dc_missed && !d_drop && m_has_free) begin
            m_valid[m_free_idx]  <= 1'b1;
            m_drop [m_free_idx]  <= 1'b0;
            m_robid[m_free_idx]  <= d_robid;
            m_mem_op[m_free_idx] <= d_mem_op;
            m_vaddr[m_free_idx]  <= d_vaddr;
            m_paddr[m_free_idx]  <= d_paddr;
            m_size [m_free_idx]  <= d_size;
        end

        // ---- hold 暂存槽 ----
        if (wb_hold_case) begin
            h_valid <= 1'b0;
        end
        if (hold_cap_dc || hold_cap_sb) begin
            h_valid <= 1'b1;
            h_robid <= d_robid;
            h_data  <= hold_cap_dc ? shape_load(dc_rdata_i,      d_mem_op[7:4], d_vaddr[1:0])
                                   : shape_load(sb_query_data_i, d_mem_op[7:4], d_vaddr[1:0]);
            h_vaddr <= d_vaddr;
            h_paddr <= d_paddr;
            h_size  <= d_size;
        end

        // ---- 顺序保护：STQ 入/出 ----
        for (sj = 0; sj < STQ_N; sj = sj + 1) begin
            if (stq_done[sj])
                stq_v[sj] <= 1'b0;
        end
        if (wb_st_case && d_is_store) begin : stq_push
            integer sk;
            reg pushed;
            pushed = 1'b0;
            for (sk = 0; sk < STQ_N; sk = sk + 1) begin
                if (!pushed && (!stq_v[sk] || stq_done[sk])) begin
                    stq_v[sk]    <= 1'b1;
                    stq_id[sk]   <= d_robid;
                    stq_pa[sk]   <= d_paddr;
                    stq_strb[sk] <= d_st_strb;
                    stq_uc[sk]   <= d_uncached;
                    pushed = 1'b1;
                end
            end
        end

        // ---- DC 级 / UC park / reload ----
        if (d_done && !u_reload && !a_go) d_valid <= 1'b0;
        if (dc_fire) d_req_sent <= 1'b1;
        if (dc_return || dc_return_drop || dc_missed) d_req_sent <= 1'b0;

        // park：把年轻 UC 挪到 u（可与 u_reload 交换）
        if (d_park) begin
            u_valid  <= 1'b1;
            u_robid  <= d_robid;
            u_mem_op <= d_mem_op;
            u_vaddr  <= d_vaddr;
            u_paddr  <= d_paddr;
            u_size   <= d_size;
        end else if (u_reload) begin
            u_valid <= 1'b0;
        end

        // reload：u 到头灌回 DC（NBA 读旧 u；与 d_park 交换时自然交叉）
        if (u_reload) begin
            d_valid    <= 1'b1;
            d_robid    <= u_robid;
            d_mem_op   <= u_mem_op;
            d_is_cacop <= 1'b0;
            d_is_store <= 1'b0;
            d_is_load  <= 1'b1;
            d_vaddr    <= u_vaddr;
            d_paddr    <= u_paddr;
            d_size     <= u_size;
            d_uncached <= 1'b1;
            d_excp     <= {`EXCP_NUM{1'b0}};
            d_req_sent <= 1'b0;
        end else if (a_go) begin
            d_valid    <= 1'b1;
            d_robid    <= a_robid;
            d_mem_op   <= a_mem_op;
            d_is_cacop <= a_is_cacop;
            d_is_store <= a_is_store_op && !a_is_cacop;
            d_is_load  <= a_is_load_op  && !a_is_cacop;
            d_vaddr    <= a_vaddr;
            d_paddr    <= a_paddr;
            d_st_data  <= a_st_data;
            d_st_strb  <= a_st_strb;
            d_size     <= a_size;
            d_uncached <= a_uncached;
            d_excp     <= a_excp;
            d_req_sent <= 1'b0;
        end
        if (a_go && !issue_valid_i) a_valid <= 1'b0;

        // ---- 发射 -> AGU ----
        if (issue_valid_i && lsu_ready_o) begin
            a_valid    <= 1'b1;
            a_robid    <= issue_robid_i;
            a_mem_op   <= issue_mem_op_i;
            a_is_cacop    <= issue_is_cacop_i;
            a_cacop_op    <= issue_cacop_op_i;
            a_base        <= issue_base_i;
            a_wdata    <= issue_wdata_i;
            a_imm      <= issue_imm_i;
        end
    end
end

// DC 级 early2：仅在 `LSU_EARLY2_ENABLE` 且「写回有保证」时唤醒。
// bypass=0 时必须关 early2（否则依赖可能在 hold 写回前被唤醒，Linux hang）。
wire d_early_ok = (`LSU_EARLY2_ENABLE != 0)
                && d_valid && d_is_load && !d_excp_any && !d_is_cacop
                && !wb_mshr_case
                && (dc_return || (sb_ready && !hold_cap_dc));
assign early_wakeup_valid_o = d_early_ok && !flush_i && !reset;
assign early_wakeup_robid_o = d_robid;

`ifdef SYNTHESIS
// synthesis translate_off
reg [63:0] lsu_store_order_stall_cyc;
reg [63:0] lsu_dc_wait_cyc;
reg [63:0] lsu_stq_full_cyc;
reg [7:0]  lsu_stq_occ_now;
reg [7:0]  lsu_stq_occ_max;
reg [63:0] lsu_stq_occ_sum;
integer    lsu_stq_pc_i;
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
        lsu_stq_occ_max           <= 8'd0;
        lsu_stq_occ_sum           <= 64'd0;
    end else if (!flush_i) begin
        if (d_valid && d_is_load && !d_excp_any && store_order_block)
            lsu_store_order_stall_cyc <= lsu_store_order_stall_cyc + 64'd1;
        if (d_valid && d_req_sent && !d_drop)
            lsu_dc_wait_cyc <= lsu_dc_wait_cyc + 64'd1;
        if (stq_full)
            lsu_stq_full_cyc <= lsu_stq_full_cyc + 64'd1;
        lsu_stq_occ_sum <= lsu_stq_occ_sum + {56'd0, lsu_stq_occ_now};
        if (lsu_stq_occ_now > lsu_stq_occ_max)
            lsu_stq_occ_max <= lsu_stq_occ_now;
        if (dc_missed && !d_drop && !m_has_free)
            $error("[%0t] LSU: miss with full miss-slots", $time);
    end
end
// synthesis translate_on
`endif

endmodule
