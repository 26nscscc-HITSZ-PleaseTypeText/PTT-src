// ============================================================
// rs_mem 模块（访存保留站，`RS_MEM_SIZE 项，FIFO + 有限 load 越过）
// ------------------------------------------------------------
// 功能：
// - 缓存等待操作数的访存类指令（load/store/ll/sc/cacop）。
// - 默认按程序序发射；例外：普通 load 可越过前方**未就绪的普通 load**。
// - store / ll / sc / cacop 仍为序屏障：前方有未发射的屏障项时不可越过。
//   （无地址消歧时不能让 load 越过未知/未发 store。）
// - 越过时在发射拍把选中项与队头交换，再按 head 出队，保持 FIFO 紧凑。
// - 唤醒机制与 rs_alu 相同（4 路写回总线 + early0/1/2 提前唤醒）。
//
// 端口：与 rs_alu 同构，差异：
// - bundle 为 mem_op/is_cacop/imm（无 br 相关）
// - 发射口对接 lsu，lsu_ready_i 反压（LSU 两级流水可能停顿）
// ============================================================
`include "mycpu.h"

module rs_mem(
    input  wire                       clk,
    input  wire                       reset,
    input  wire                       flush_i,

`ifdef SOC_USE
    // Translation attributes used to prove that a bypassed load is cached.
    // A non-head request may never enter LSU when its MAT is unknown or UC.
    input  wire                       csr_crmd_da_i,
    input  wire                       csr_crmd_pg_i,
    input  wire [1:0]                 csr_crmd_plv_i,
    input  wire [1:0]                 csr_crmd_datm_i,
    input  wire [2:0]                 csr_dmw0_vseg_i,
    input  wire [1:0]                 csr_dmw0_mat_i,
    input  wire                       csr_dmw0_plv3_i,
    input  wire                       csr_dmw0_plv0_i,
    input  wire [2:0]                 csr_dmw1_vseg_i,
    input  wire [1:0]                 csr_dmw1_mat_i,
    input  wire                       csr_dmw1_plv3_i,
    input  wire                       csr_dmw1_plv0_i,

`else
`endif
    // ---------------- 入站（dispatch）----------------
    input  wire                       push_valid_i,
    input  wire [`ROB_W-1:0]          push_robid_i,
    input  wire [`MEM_OP_NUM-1:0]     push_mem_op_i,
    input  wire                       push_is_cacop_i,
    input  wire [4:3]                 push_cacop_op_i,
    input  wire                       push_src0_ready_i,   // src0 = 基址 rj
    input  wire [31:0]                push_src0_val_i,
    input  wire [`ROB_W-1:0]          push_src0_robid_i,
    input  wire [31:0]                push_src0_key_val_i,
    input  wire                       push_src0_rob_wait_i,
    input  wire                       push_src1_ready_i,   // src1 = store 数据 rd
    input  wire [31:0]                push_src1_val_i,
    input  wire [`ROB_W-1:0]          push_src1_robid_i,
    input  wire [31:0]                push_imm_i,          // si12/si14 偏移

    input  wire                       push1_valid_i,
    input  wire [`ROB_W-1:0]          push1_robid_i,
    input  wire [`MEM_OP_NUM-1:0]     push1_mem_op_i,
    input  wire                       push1_is_cacop_i,
    input  wire [4:3]                 push1_cacop_op_i,
    input  wire                       push1_src0_ready_i,
    input  wire [31:0]                push1_src0_val_i,
    input  wire [`ROB_W-1:0]          push1_src0_robid_i,
    input  wire [31:0]                push1_src0_key_val_i,
    input  wire                       push1_src0_rob_wait_i,
    input  wire                       push1_src1_ready_i,
    input  wire [31:0]                push1_src1_val_i,
    input  wire [`ROB_W-1:0]          push1_src1_robid_i,
    input  wire [31:0]                push1_imm_i,

    output wire                       can_accept_o,
    output wire                       can_accept_two_o,
    output wire [`RS_MEM_OCC_W-1:0]   occupancy_o,

    // ---------------- 写回唤醒总线 ×4 ----------------
    input  wire                       wb0_valid_i,
    input  wire [`ROB_W-1:0]          wb0_robid_i,
    input  wire [31:0]                wb0_data_i,
    input  wire                       wb1_valid_i,
    input  wire [`ROB_W-1:0]          wb1_robid_i,
    input  wire [31:0]                wb1_data_i,
    input  wire                       wb2_valid_i,
    input  wire [`ROB_W-1:0]          wb2_robid_i,
    input  wire [31:0]                wb2_data_i,
    input  wire                       fast2_valid_i,
    input  wire [`ROB_W-1:0]          fast2_robid_i,
    input  wire [31:0]                fast2_data_i,
    // Registered cached-load miss shadows from LSU.  These tags allow only
    // the oldest plain store to launch its ready address while its data waits
    // locally in LSU; no refill data/control is broadcast back into this RS.
    input  wire [`LSU_MISS_DEPTH-1:0] miss_wait_valid_i,
    input  wire [(`LSU_MISS_DEPTH*`ROB_W)-1:0] miss_wait_robids_i,
    // Registered per-ROB promise from LSU: a cached ordinary load request is
    // already in DCache and will complete through hit-pipe or MSHR.  This mask
    // is intentionally independent of the current raw DCache response.
    input  wire [`LSU_TOKEN_DEPTH-1:0] pending_load_valid_i,
    input  wire [(`LSU_TOKEN_DEPTH*`ROB_W)-1:0] pending_load_robids_i,
    input  wire                       wb3_valid_i,
    input  wire [`ROB_W-1:0]          wb3_robid_i,
    input  wire [31:0]                wb3_data_i,
    input  wire                       wb4_valid_i,
    input  wire [`ROB_W-1:0]          wb4_robid_i,
    input  wire [31:0]                wb4_data_i,
    input  wire                       wb5_valid_i,
    input  wire [`ROB_W-1:0]          wb5_robid_i,
    input  wire [31:0]                wb5_data_i,

    // ---------------- 提前唤醒总线 ×3（early0/1=ALU；early2=LSU DC 命中）----------------
    input  wire                       early0_valid_i,
    input  wire [`ROB_W-1:0]          early0_robid_i,
    input  wire                       early1_valid_i,
    input  wire [`ROB_W-1:0]          early1_robid_i,
    input  wire                       early2_valid_i,
    input  wire [`ROB_W-1:0]          early2_robid_i,

    // ---------------- 发射口（到 lsu）----------------
    output wire                       issue_valid_o,
    output wire [`ROB_W-1:0]          issue_robid_o,
    output wire [`MEM_OP_NUM-1:0]     issue_mem_op_o,
    output wire                       issue_is_cacop_o,
    output wire [4:3]                 issue_cacop_op_o,
    output wire [31:0]                issue_base_o,        // 基址（src0 捕获值）
    output wire                       issue_base_ready_o,
    output wire [`ROB_W-1:0]          issue_base_robid_o,
    output wire [31:0]                issue_wdata_o,       // store 数据（src1 捕获值）
    output wire                       issue_wdata_ready_o,
    output wire [`ROB_W-1:0]          issue_wdata_robid_o,
    output wire [31:0]                issue_imm_o,
    input  wire                       lsu_ready_i,         // LSU 本拍可接收（AGU 级空闲）
    input  wire                       lsu_plain_only_i,
    output wire                       issue_pair_valid_o,
    output wire [`ROB_W-1:0]          issue_pair_robid_o,
    output wire [`MEM_OP_NUM-1:0]     issue_pair_mem_op_o,
    output wire [31:0]                issue_pair_base_o,
    output wire [31:0]                issue_pair_imm_o,
    input  wire                       lsu_pair_ready_i
);

// 设计说明：
//
// 存储结构：
//      与 rs_alu 类似（valid/robid/op/双源/imm），但组织成 FIFO：
//      head/tail 指针（4 项 -> 2bit）；入站写 tail 项 tail++，发射出队 head++。
//
// 唤醒与数据捕获：与 rs_alu 完全相同（4 路 wb 总线逐项逐源比较捕获，
//      入站同拍旁路同样要做）。
//
// 发射（队头优先 + 有限 load 越过，见头注）：
//      默认发队头：issue = valid[head] && 双源 ready && lsu_ready_i；
//      队头是未就绪普通 load 时，可从其后连续的普通 load 中选一条就绪的，
//      发射拍与队头【交换】后按 head 出队（保持 FIFO 紧凑）；
//      store/ll/sc/cacop 是序屏障——不可被越过，自身也不越过别人；
//      store 即使 src1（数据）未就绪也不能让位给后面的 load ——
//      无地址消歧时这是内存序正确性的底线。
//
// 冲刷：flush_i 清空 head/tail/valid。
//
// 接口约束：lsu_ready_i 为 0 时必须保持队头；early2 只表示 D$ 命中 load
// 的提前唤醒。没有地址消歧和违例恢复时，load 不得越过 store。

reg                     valid [0:`RS_MEM_SIZE-1];
reg [`ROB_W-1:0]        robid [0:`RS_MEM_SIZE-1];
reg [`MEM_OP_NUM-1:0]   mem_op [0:`RS_MEM_SIZE-1];
// Static memory-order classes are decoded once at insertion and travel with
// the entry.  They keep the wide mem_op register away from both the ordinary
// oldest-ready scan and the isolated fused-load selector.
reg                     ord_barrier [0:`RS_MEM_SIZE-1];
reg                     plain_load [0:`RS_MEM_SIZE-1];
reg                     plain_store [0:`RS_MEM_SIZE-1];
reg                     is_cacop [0:`RS_MEM_SIZE-1];
reg [4:3]               cacop_op [0:`RS_MEM_SIZE-1];
reg                     s0_ready [0:`RS_MEM_SIZE-1];
reg                     s0_val_valid [0:`RS_MEM_SIZE-1];
reg                     s0_birth_pending [0:`RS_MEM_SIZE-1];
reg [2:0]               s0_birth_sel [0:`RS_MEM_SIZE-1];
reg                     s0_alu0_pred [0:`RS_MEM_SIZE-1];
reg                     s0_alu1_pred [0:`RS_MEM_SIZE-1];
reg                     s0_mem_pred [0:`RS_MEM_SIZE-1];
reg [31:0]              s0_val [0:`RS_MEM_SIZE-1];
reg [`ROB_W-1:0]        s0_robid [0:`RS_MEM_SIZE-1];
reg                     s1_ready [0:`RS_MEM_SIZE-1];
reg                     s1_val_valid [0:`RS_MEM_SIZE-1];
reg                     s1_birth_pending [0:`RS_MEM_SIZE-1];
reg [2:0]               s1_birth_sel [0:`RS_MEM_SIZE-1];
reg                     s1_alu0_pred [0:`RS_MEM_SIZE-1];
reg                     s1_alu1_pred [0:`RS_MEM_SIZE-1];
reg                     s1_mem_pred [0:`RS_MEM_SIZE-1];
reg [31:0]              s1_val [0:`RS_MEM_SIZE-1];
reg [`ROB_W-1:0]        s1_robid [0:`RS_MEM_SIZE-1];
reg [31:0]              imm [0:`RS_MEM_SIZE-1];
// Registered address key used only by fused-pair eligibility.  Keeping the
// carry chain out of pair selection prevents base data from driving every
// pair-compaction payload CE in the same cycle.
reg [26:0]              pair_vaddr_key [0:`RS_MEM_SIZE-1];
// Refresh pair keys from registered s0_val the cycle after a WB install.
// Fused-pair select already requires stored_ready (registered val_valid), so
// removing the ALU/WB carry from this D-pin does not delay single-issue; it
// only keeps the key aligned one edge after the operand becomes visible.
reg                     s0_key_refresh [0:`RS_MEM_SIZE-1];
// ========== Timing optimization: Pre-computed issue base values ==========
// Register the final base value for each entry to cut the critical path from
// s0_ready -> swap_idx selection -> dynamic s0_val mux -> issue_base_o.
// This adds one cycle of latency but eliminates 6-7ns of combinational delay.
reg [31:0]              precomp_base [0:`RS_MEM_SIZE-1];
// ==========================================================================
reg [`RS_MEM_IDX_W-1:0] head;
reg [`RS_MEM_IDX_W-1:0] tail;
reg [`RS_MEM_OCC_W-1:0] count;
`ifdef SOC_USE
localparam [`RS_MEM_OCC_W-1:0] RS_MEM_COUNT_MAX = `RS_MEM_SIZE;
// Valid bits remain stale after dequeue; count is the architectural empty
// predicate for every issue fast path.
wire queue_nonempty = (count != {`RS_MEM_OCC_W{1'b0}});
`endif
// Exact mirror of the two source tags at the logical queue head.  The mirror
// is advanced with the ring pointer (including pair compaction and the
// deferred non-head swap), so timing-sensitive head wake/pending compares do
// not first traverse the dynamically indexed source-tag arrays.
reg [`ROB_W-1:0]        head_s0_robid_q;
reg [`ROB_W-1:0]        head_s1_robid_q;
// Low count bits are one.  Maintaining this mask beside count removes the
// variable (1 << count) barrel-shift from the issue priority/data-select cone.
reg [`RS_MEM_SIZE-1:0]  scan_count_mask;

`ifdef SOC_USE
// The ordinary bypass scan is allowed only for requests whose cacheability is
// already determined in this stage. TLB MAT is unavailable until LSU/MMU, so
// TLB-mapped accesses remain ordered. This prevents a younger UC/MMIO load
// from consuming LSU ownership ahead of an older stalled cached load.
wire scan_da_mode = (csr_crmd_da_i === 1'b1) && (csr_crmd_pg_i === 1'b0);
wire scan_pg_mode = (csr_crmd_da_i === 1'b0) && (csr_crmd_pg_i === 1'b1);
wire scan_dmw0_plv_ok = ((csr_crmd_plv_i === 2'b00) &&
                         (csr_dmw0_plv0_i === 1'b1)) ||
                        ((csr_crmd_plv_i === 2'b11) &&
                         (csr_dmw0_plv3_i === 1'b1));
wire scan_dmw1_plv_ok = ((csr_crmd_plv_i === 2'b00) &&
                         (csr_dmw1_plv0_i === 1'b1)) ||
                        ((csr_crmd_plv_i === 2'b11) &&
                         (csr_dmw1_plv3_i === 1'b1));
wire [`RS_MEM_SIZE-1:0] scan_known_cached_phys;
genvar sc;
generate
for (sc = 0; sc < `RS_MEM_SIZE; sc = sc + 1) begin : g_scan_known_cached
    wire entry_dmw0_hit = scan_pg_mode
                       && (pair_vaddr_key[sc][26:24] === csr_dmw0_vseg_i)
                       && scan_dmw0_plv_ok;
    wire entry_dmw1_hit = scan_pg_mode
                       && (pair_vaddr_key[sc][26:24] === csr_dmw1_vseg_i)
                       && scan_dmw1_plv_ok;
    // Match tlb_manager's MAT priority exactly: DA, then DMW0, then DMW1,
    // then the TLB. DA cacheability is independent of the address. A DMW key,
    // however, is valid only after src0 reaches the registered value state.
    // An early wakeup may make an entry executable, but must not classify a
    // non-head request from its old key. The TLB case remains unknown here.
    assign scan_known_cached_phys[sc] = scan_da_mode
        ? (csr_crmd_datm_i === 2'b01)
        : (s0_val_valid[sc] &&
           (entry_dmw0_hit ? (csr_dmw0_mat_i === 2'b01) :
            entry_dmw1_hit ? (csr_dmw1_mat_i === 2'b01) :
                             1'b0));
end
endgenerate

`else
`endif
// Mirror every WB data port for one cycle.  A newborn entry records only the
// matching port number at its insertion edge, then selects this local data on
// its first visible cycle.  No stored ROB tag participates in that selection.
reg [31:0]              wb0d_data, wb1d_data, wb2d_data, wb3d_data, wb4d_data, wb5d_data;

// Defer a non-head issue's head-entry relocation through one fixed buffer.
// This keeps the current WB tag away from dynamically indexed metadata D pins.
reg                      swap_pending;
reg [`RS_MEM_IDX_W-1:0]  swap_idx_r;
reg [`ROB_W-1:0]         swap_robid;
reg [`MEM_OP_NUM-1:0]    swap_mem_op;
reg                      swap_ord_barrier;
reg                      swap_plain_load;
reg                      swap_plain_store;
reg                      swap_is_cacop;
reg [4:3]                swap_cacop_op;
reg                      swap_s0_ready, swap_s0_val_valid;
reg                      swap_s0_alu0_pred, swap_s0_alu1_pred, swap_s0_mem_pred;
reg [31:0]               swap_s0_val;
reg [`ROB_W-1:0]         swap_s0_robid;
reg                      swap_s1_ready, swap_s1_val_valid;
reg                      swap_s1_alu0_pred, swap_s1_alu1_pred, swap_s1_mem_pred;
reg [31:0]               swap_s1_val;
reg [`ROB_W-1:0]         swap_s1_robid;
reg [31:0]               swap_imm;
reg [26:0]               swap_pair_vaddr_key;

integer i;
wire issue_fire;
reg  [`RS_MEM_IDX_W-1:0] issue_idx;
reg                      issue_sel_valid;
(* max_fanout = 32 *) reg [`RS_MEM_IDX_W-1:0] base_issue_idx;
reg  [`RS_MEM_OCC_W-1:0] base_issue_age;
reg                      base_issue_valid;
wire                     issue_need_swap;

wire            s0_wb_match [0:`RS_MEM_SIZE-1];
wire            s1_wb_match [0:`RS_MEM_SIZE-1];
wire            s0_wbhit [0:`RS_MEM_SIZE-1];
wire            s1_wbhit [0:`RS_MEM_SIZE-1];
wire            s0_birth_wbhit [0:`RS_MEM_SIZE-1];
wire            s1_birth_wbhit [0:`RS_MEM_SIZE-1];
`ifdef SOC_USE
`else
wire            s0_issue_wbhit [0:`RS_MEM_SIZE-1];
wire            s1_issue_wbhit [0:`RS_MEM_SIZE-1];
`endif
wire            s0_alu0_earlyhit [0:`RS_MEM_SIZE-1];
wire            s1_alu0_earlyhit [0:`RS_MEM_SIZE-1];
wire            s0_alu1_earlyhit [0:`RS_MEM_SIZE-1];
wire            s1_alu1_earlyhit [0:`RS_MEM_SIZE-1];
wire            s0_mem_earlyhit [0:`RS_MEM_SIZE-1];
wire            s1_mem_earlyhit [0:`RS_MEM_SIZE-1];
wire            s0_earlyhit [0:`RS_MEM_SIZE-1];
wire            s1_earlyhit [0:`RS_MEM_SIZE-1];
wire [31:0]     s0_wbdat [0:`RS_MEM_SIZE-1];
wire [31:0]     s1_wbdat [0:`RS_MEM_SIZE-1];
wire [31:0]     issue_base_candidate [0:`RS_MEM_SIZE-1];
wire [31:0]     s0_birth_wbdat [0:`RS_MEM_SIZE-1];
wire [31:0]     s1_birth_wbdat [0:`RS_MEM_SIZE-1];
`ifdef SOC_USE
`else
wire [31:0]     s0_issue_wbdat [0:`RS_MEM_SIZE-1];
wire [31:0]     s1_issue_wbdat [0:`RS_MEM_SIZE-1];
`endif
wire [31:0]     s0_key_mid_data [0:`RS_MEM_SIZE-1];
(* keep = "true" *) wire [31:0] s0_key_mid_vaddr [0:`RS_MEM_SIZE-1];
wire            s0_wb0_direct_hit [0:`RS_MEM_SIZE-1];
(* keep = "true" *) wire [31:0] s0_wb0_vaddr [0:`RS_MEM_SIZE-1];
wire            s0_wb1_direct_hit [0:`RS_MEM_SIZE-1];
(* keep = "true" *) wire [31:0] s0_wb1_vaddr [0:`RS_MEM_SIZE-1];
wire            s0_wb5_direct_hit [0:`RS_MEM_SIZE-1];
(* keep = "true" *) wire [31:0] s0_wb5_vaddr [0:`RS_MEM_SIZE-1];
wire [26:0]     s0_wb_key [0:`RS_MEM_SIZE-1];
wire [26:0]     s0_ref_key [0:`RS_MEM_SIZE-1];
wire            entry_ready [0:`RS_MEM_SIZE-1];
wire            entry_stored_ready [0:`RS_MEM_SIZE-1];
wire            is_ord_barrier [0:`RS_MEM_SIZE-1]; // store/ll/sc/cacop：不可被越过
wire            is_plain_load [0:`RS_MEM_SIZE-1];
wire [`RS_MEM_SIZE-1:0] pair_stored_ready_flat;
wire [`RS_MEM_SIZE-1:0] pair_plain_load_flat;
wire [`RS_MEM_SIZE*`ROB_W-1:0] pair_robid_flat;
wire [`RS_MEM_SIZE*`MEM_OP_NUM-1:0] pair_mem_op_flat;
wire [`RS_MEM_SIZE*32-1:0] pair_base_flat;
wire [`RS_MEM_SIZE*32-1:0] pair_imm_flat;
wire [`RS_MEM_SIZE*27-1:0] pair_vaddr_key_flat;
genvar gw;
generate
for (gw = 0; gw < `RS_MEM_SIZE; gw = gw + 1) begin : g_wake
    assign s0_birth_wbhit[gw] = s0_birth_pending[gw];
    assign s1_birth_wbhit[gw] = s1_birth_pending[gw];
    assign s0_wb_match[gw] = (wb0_valid_i && (wb0_robid_i == s0_robid[gw])) ||
                             (wb1_valid_i && (wb1_robid_i == s0_robid[gw])) ||
                             (wb2_valid_i && (wb2_robid_i == s0_robid[gw])) ||
                             (wb3_valid_i && (wb3_robid_i == s0_robid[gw])) ||
                             (wb4_valid_i && (wb4_robid_i == s0_robid[gw])) ||
                             (wb5_valid_i && (wb5_robid_i == s0_robid[gw])) ||
                             s0_birth_wbhit[gw];
    assign s1_wb_match[gw] = (wb0_valid_i && (wb0_robid_i == s1_robid[gw])) ||
                             (wb1_valid_i && (wb1_robid_i == s1_robid[gw])) ||
                             (wb2_valid_i && (wb2_robid_i == s1_robid[gw])) ||
                             (wb3_valid_i && (wb3_robid_i == s1_robid[gw])) ||
                             (wb4_valid_i && (wb4_robid_i == s1_robid[gw])) ||
                             (wb5_valid_i && (wb5_robid_i == s1_robid[gw])) ||
                             s1_birth_wbhit[gw];
    // val_valid 冻结真值；仅 early 时允许 WB 再捕获/旁路
    assign s0_wbhit[gw] = !s0_val_valid[gw] && s0_wb_match[gw];
    assign s1_wbhit[gw] = !s1_val_valid[gw] && s1_wb_match[gw];
    // The locally registered MDU port (wb3) may capture into the queue this
    // cycle, but does not also traverse selection plus the address adder.
    // All other ports retain their same-cycle issue bypass.
    // ALU1 is captured locally into s*_val this cycle and becomes issuable
    // on the next.  Removing wb1 from same-cycle issue cuts tag -> FIFO scan
    // -> swap/address paths; all other established bypasses are retained.
`ifdef SOC_USE
    // A birth hit is copied from the delayed WB mirror into s*_val and
    // becomes issuable on the following cycle. ALU0/ALU1/LSU WB retain the
    // dedicated head/next-head overrides below.
`else
    // A birth hit is first copied from the delayed WB mirror into s*_val and
    // becomes issuable on the following cycle.  Keeping birth_pending out of
    // issue selection avoids another high-fanout tag -> priority -> payload
    // path.  ALU0/ALU1/LSU WB retain dedicated head/next-head overrides.
    assign s0_issue_wbhit[gw] = !s0_val_valid[gw] &&
                             s0_birth_wbhit[gw];
    assign s1_issue_wbhit[gw] = !s1_val_valid[gw] &&
                             s1_birth_wbhit[gw];
`endif
    assign s0_alu1_earlyhit[gw] = !s0_val_valid[gw] && early1_valid_i &&
                                  (early1_robid_i == s0_robid[gw]);
    assign s1_alu1_earlyhit[gw] = !s1_val_valid[gw] && early1_valid_i &&
                                  (early1_robid_i == s1_robid[gw]);
    assign s0_alu0_earlyhit[gw] = !s0_val_valid[gw] && early0_valid_i &&
                                  (early0_robid_i == s0_robid[gw]);
    assign s1_alu0_earlyhit[gw] = !s1_val_valid[gw] && early0_valid_i &&
                                  (early0_robid_i == s1_robid[gw]);
    assign s0_mem_earlyhit[gw] = !s0_val_valid[gw] && early2_valid_i &&
                                 (early2_robid_i == s0_robid[gw]);
    assign s1_mem_earlyhit[gw] = !s1_val_valid[gw] && early2_valid_i &&
                                 (early2_robid_i == s1_robid[gw]);
    assign s0_earlyhit[gw] = !s0_ready[gw] && !s0_wbhit[gw] &&
                             ((early0_valid_i && (early0_robid_i == s0_robid[gw])) ||
                              (early1_valid_i && (early1_robid_i == s0_robid[gw])) ||
                              (early2_valid_i && (early2_robid_i == s0_robid[gw])));
    assign s1_earlyhit[gw] = !s1_ready[gw] && !s1_wbhit[gw] &&
                             ((early0_valid_i && (early0_robid_i == s1_robid[gw])) ||
                              (early1_valid_i && (early1_robid_i == s1_robid[gw])) ||
                              (early2_valid_i && (early2_robid_i == s1_robid[gw])));
    assign s0_birth_wbdat[gw] = (s0_birth_sel[gw] == 3'd0) ? wb0d_data :
                                (s0_birth_sel[gw] == 3'd1) ? wb1d_data :
                                (s0_birth_sel[gw] == 3'd2) ? wb2d_data :
                                (s0_birth_sel[gw] == 3'd3) ? wb3d_data :
                                (s0_birth_sel[gw] == 3'd4) ? wb4d_data : wb5d_data;
    assign s1_birth_wbdat[gw] = (s1_birth_sel[gw] == 3'd0) ? wb0d_data :
                                (s1_birth_sel[gw] == 3'd1) ? wb1d_data :
                                (s1_birth_sel[gw] == 3'd2) ? wb2d_data :
                                (s1_birth_sel[gw] == 3'd3) ? wb3d_data :
                                (s1_birth_sel[gw] == 3'd4) ? wb4d_data : wb5d_data;
    assign s0_wbdat[gw] = s0_birth_wbhit[gw] ? s0_birth_wbdat[gw] :
                          (wb0_valid_i && (wb0_robid_i == s0_robid[gw])) ? wb0_data_i :
                          (wb1_valid_i && (wb1_robid_i == s0_robid[gw])) ? wb1_data_i :
                          (wb2_valid_i && (wb2_robid_i == s0_robid[gw])) ? wb2_data_i :
                          (wb3_valid_i && (wb3_robid_i == s0_robid[gw])) ? wb3_data_i :
                          (wb4_valid_i && (wb4_robid_i == s0_robid[gw])) ? wb4_data_i :
                          (wb5_valid_i && (wb5_robid_i == s0_robid[gw])) ? wb5_data_i : 32'b0;
    assign s1_wbdat[gw] = s1_birth_wbhit[gw] ? s1_birth_wbdat[gw] :
                          (wb0_valid_i && (wb0_robid_i == s1_robid[gw])) ? wb0_data_i :
                          (wb1_valid_i && (wb1_robid_i == s1_robid[gw])) ? wb1_data_i :
                          (wb2_valid_i && (wb2_robid_i == s1_robid[gw])) ? wb2_data_i :
                          (wb3_valid_i && (wb3_robid_i == s1_robid[gw])) ? wb3_data_i :
                          (wb4_valid_i && (wb4_robid_i == s1_robid[gw])) ? wb4_data_i :
                          (wb5_valid_i && (wb5_robid_i == s1_robid[gw])) ? wb5_data_i : 32'b0;
    // Resolve each entry's same-cycle base bypass beside issue selection.
    // The final issue_idx mux therefore sees already-bypassed data, while the
    // original wb0 > wb1 > wb2 > wb5 > stored priority remains unchanged.
    assign issue_base_candidate[gw] =
        (!s0_val_valid[gw] && wb0_valid_i &&
         (wb0_robid_i == s0_robid[gw])) ? wb0_data_i :
        (!s0_val_valid[gw] && wb1_valid_i &&
         (wb1_robid_i == s0_robid[gw])) ? wb1_data_i :
        (!s0_val_valid[gw] && wb2_valid_i &&
         (wb2_robid_i == s0_robid[gw])) ? wb2_data_i :
        (!s0_val_valid[gw] && wb5_valid_i &&
         (wb5_robid_i == s0_robid[gw])) ? wb5_data_i :
        precomp_base[gw];
`ifdef SOC_USE
`else
    assign s0_issue_wbdat[gw] = s0_birth_wbhit[gw] ? s0_birth_wbdat[gw] : 32'b0;
    assign s1_issue_wbdat[gw] = s1_birth_wbhit[gw] ? s1_birth_wbdat[gw] : 32'b0;
`endif
    // The generic s0_wbdat mux must still contain every source for operand
    // capture.  The pair-key cone is separate: ALU0/ALU1/ALU2 have dedicated
    // adders, so remove them structurally from this middle-priority data mux.
    // This prevents STA from traversing a logically unsensitizable direct-WB
    // result through the generic carry chain (incl. a false ALU1->mid path).
    assign s0_key_mid_data[gw] = s0_birth_wbhit[gw] ? s0_birth_wbdat[gw] :
                          (wb2_valid_i && (wb2_robid_i == s0_robid[gw])) ? wb2_data_i :
                          (wb3_valid_i && (wb3_robid_i == s0_robid[gw])) ? wb3_data_i :
                          (wb4_valid_i && (wb4_robid_i == s0_robid[gw])) ? wb4_data_i : 32'b0;
    wire [5:0] s0_mid_lo = {1'b0, s0_key_mid_data[gw][4:0]} + {1'b0, imm[gw][4:0]};
    wire [26:0] s0_mid_key = s0_key_mid_data[gw][31:5] + imm[gw][31:5] + {26'b0, s0_mid_lo[5]};
    assign s0_key_mid_vaddr[gw] = {s0_mid_key, s0_mid_lo[4:0]};
    // ALU0 is the measured long producer: producer carry -> six-source WB
    // mux -> address carry -> relocation mux.  Preserve a parallel ALU0-only
    // address adder and select its registered key after the carry chain.
    // Birth data has higher priority in s0_wbdat and therefore excludes this
    // direct branch, preserving the original arbitration exactly.
    assign s0_wb0_direct_hit[gw] = !s0_birth_wbhit[gw]
                                 && !s0_val_valid[gw]
                                 && wb0_valid_i
                                 && (wb0_robid_i == s0_robid[gw]);
    // Split [4:0]+[31:5] add so the pair-key bus only waits on a 27-bit
    // carry after a cheap 5-bit sum — identical to (base+imm)[31:5].
    wire [5:0] s0_wb0_lo = {1'b0, wb0_data_i[4:0]} + {1'b0, imm[gw][4:0]};
    wire [26:0] s0_wb0_key = wb0_data_i[31:5] + imm[gw][31:5] + {26'b0, s0_wb0_lo[5]};
    assign s0_wb0_vaddr[gw] = {s0_wb0_key, s0_wb0_lo[4:0]};
    // ALU1 is the remaining high-fanout producer on the pair-key path.  Its
    // direct adder is selected only when the existing higher-priority birth
    // and ALU0 sources miss, exactly matching s0_wbdat arbitration.
    assign s0_wb1_direct_hit[gw] = !s0_birth_wbhit[gw]
                                 && !s0_val_valid[gw]
                                 && wb1_valid_i
                                 && (wb1_robid_i == s0_robid[gw])
                                 && !(wb0_valid_i && (wb0_robid_i == s0_robid[gw]));
    wire [5:0] s0_wb1_lo = {1'b0, wb1_data_i[4:0]} + {1'b0, imm[gw][4:0]};
    wire [26:0] s0_wb1_key = wb1_data_i[31:5] + imm[gw][31:5] + {26'b0, s0_wb1_lo[5]};
    assign s0_wb1_vaddr[gw] = {s0_wb1_key, s0_wb1_lo[4:0]};
    // ALU2 is the remaining measured long producer after the ALU0 branch was
    // isolated.  It is last in the original WB priority, so the direct path
    // is enabled only when no birth or earlier WB source matches this entry.
    // This changes neither the update edge nor simultaneous-WB arbitration.
    assign s0_wb5_direct_hit[gw] = !s0_birth_wbhit[gw]
                                 && !s0_val_valid[gw]
                                 && wb5_valid_i
                                 && (wb5_robid_i == s0_robid[gw])
                                 && !(wb0_valid_i && (wb0_robid_i == s0_robid[gw]))
                                 && !(wb1_valid_i && (wb1_robid_i == s0_robid[gw]))
                                 && !(wb2_valid_i && (wb2_robid_i == s0_robid[gw]))
                                 && !(wb3_valid_i && (wb3_robid_i == s0_robid[gw]))
                                 && !(wb4_valid_i && (wb4_robid_i == s0_robid[gw]));
    wire [5:0] s0_wb5_lo = {1'b0, wb5_data_i[4:0]} + {1'b0, imm[gw][4:0]};
    wire [26:0] s0_wb5_key = wb5_data_i[31:5] + imm[gw][31:5] + {26'b0, s0_wb5_lo[5]};
    assign s0_wb5_vaddr[gw] = {s0_wb5_key, s0_wb5_lo[4:0]};
    assign s0_wb_key[gw] = s0_wb0_direct_hit[gw]
                         ? s0_wb0_key
                         : s0_wb1_direct_hit[gw]
                         ? s0_wb1_key
                         : s0_wb5_direct_hit[gw]
                         ? s0_wb5_key
                         : s0_mid_key;
    // 仅 early 的 ready 不够：必须已有真值或本拍 WB 可旁路
    assign entry_ready[gw] = valid[gw] &&
                             ((s0_ready[gw] && s0_val_valid[gw]) ||
                              s0_alu0_pred[gw] || s0_alu1_pred[gw] ||
                              s0_mem_pred[gw]) &&
                             ((s1_ready[gw] && s1_val_valid[gw]) ||
                              s1_alu0_pred[gw] || s1_alu1_pred[gw] ||
                              s1_mem_pred[gw]);
    // Pair acceptance is deliberately based only on values already stored in
    // the queue.  Ordinary single issue still consumes same-cycle WB bypasses;
    // excluding them here keeps the ALU1 tag out of the LSU pair-control cone.
    assign entry_stored_ready[gw] = valid[gw] &&
                                    s0_ready[gw] && s0_val_valid[gw] &&
                                    s1_ready[gw] && s1_val_valid[gw];
    assign is_ord_barrier[gw] = ord_barrier[gw];
    assign is_plain_load[gw] = plain_load[gw];
    assign pair_stored_ready_flat[gw] = entry_stored_ready[gw];
    assign pair_plain_load_flat[gw] = is_plain_load[gw];
    assign pair_robid_flat[gw*`ROB_W +: `ROB_W] = robid[gw];
    assign pair_mem_op_flat[gw*`MEM_OP_NUM +: `MEM_OP_NUM] = mem_op[gw];
    assign pair_base_flat[gw*32 +: 32] = s0_val[gw];
    assign pair_imm_flat[gw*32 +: 32] = imm[gw];
    assign pair_vaddr_key_flat[gw*27 +: 27] = pair_vaddr_key[gw];
    // Registered-base key used by the one-cycle deferred refresh (no WB cone).
    wire [5:0] s0_ref_lo = {1'b0, s0_val[gw][4:0]} + {1'b0, imm[gw][4:0]};
    assign s0_ref_key[gw] = s0_val[gw][31:5] + imm[gw][31:5] + {26'b0, s0_ref_lo[5]};
end
endgenerate

// A producer may complete while the buffered head waits for its install
// edge.  Snoop the same buses here so the one-cycle relocation cannot lose a
// wakeup pulse.
wire swap_s0_wb_match = (wb0_valid_i && (wb0_robid_i == swap_s0_robid)) ||
                        (wb1_valid_i && (wb1_robid_i == swap_s0_robid)) ||
                        (wb2_valid_i && (wb2_robid_i == swap_s0_robid)) ||
                        (wb3_valid_i && (wb3_robid_i == swap_s0_robid)) ||
                        (wb4_valid_i && (wb4_robid_i == swap_s0_robid)) ||
                        (wb5_valid_i && (wb5_robid_i == swap_s0_robid));
wire swap_s1_wb_match = (wb0_valid_i && (wb0_robid_i == swap_s1_robid)) ||
                        (wb1_valid_i && (wb1_robid_i == swap_s1_robid)) ||
                        (wb2_valid_i && (wb2_robid_i == swap_s1_robid)) ||
                        (wb3_valid_i && (wb3_robid_i == swap_s1_robid)) ||
                        (wb4_valid_i && (wb4_robid_i == swap_s1_robid)) ||
                        (wb5_valid_i && (wb5_robid_i == swap_s1_robid));
wire swap_s0_wbhit = swap_pending && !swap_s0_val_valid && swap_s0_wb_match;
wire swap_s1_wbhit = swap_pending && !swap_s1_val_valid && swap_s1_wb_match;
wire [31:0] swap_s0_wbdat = (wb0_valid_i && (wb0_robid_i == swap_s0_robid)) ? wb0_data_i :
                            (wb1_valid_i && (wb1_robid_i == swap_s0_robid)) ? wb1_data_i :
                            (wb2_valid_i && (wb2_robid_i == swap_s0_robid)) ? wb2_data_i :
                            (wb3_valid_i && (wb3_robid_i == swap_s0_robid)) ? wb3_data_i :
                            (wb4_valid_i && (wb4_robid_i == swap_s0_robid)) ? wb4_data_i : wb5_data_i;
wire [31:0] swap_s1_wbdat = (wb0_valid_i && (wb0_robid_i == swap_s1_robid)) ? wb0_data_i :
                            (wb1_valid_i && (wb1_robid_i == swap_s1_robid)) ? wb1_data_i :
                            (wb2_valid_i && (wb2_robid_i == swap_s1_robid)) ? wb2_data_i :
                            (wb3_valid_i && (wb3_robid_i == swap_s1_robid)) ? wb3_data_i :
                             (wb4_valid_i && (wb4_robid_i == swap_s1_robid)) ? wb4_data_i : wb5_data_i;
// Mirror entry pair-key isolation: ALU0/1/2 get dedicated address adders so
// STA cannot force their carries through the generic 6:1 WB mux.
wire        swap_s0_wb0_direct = swap_pending && !swap_s0_val_valid &&
                                 wb0_valid_i && (wb0_robid_i == swap_s0_robid);
wire        swap_s0_wb1_direct = swap_pending && !swap_s0_val_valid &&
                                 wb1_valid_i && (wb1_robid_i == swap_s0_robid) &&
                                 !(wb0_valid_i && (wb0_robid_i == swap_s0_robid));
wire        swap_s0_wb5_direct = swap_pending && !swap_s0_val_valid &&
                                 wb5_valid_i && (wb5_robid_i == swap_s0_robid) &&
                                 !(wb0_valid_i && (wb0_robid_i == swap_s0_robid)) &&
                                 !(wb1_valid_i && (wb1_robid_i == swap_s0_robid)) &&
                                 !(wb2_valid_i && (wb2_robid_i == swap_s0_robid)) &&
                                 !(wb3_valid_i && (wb3_robid_i == swap_s0_robid)) &&
                                 !(wb4_valid_i && (wb4_robid_i == swap_s0_robid));
wire [31:0] swap_s0_mid_data  =
    (wb2_valid_i && (wb2_robid_i == swap_s0_robid)) ? wb2_data_i :
    (wb3_valid_i && (wb3_robid_i == swap_s0_robid)) ? wb3_data_i :
    (wb4_valid_i && (wb4_robid_i == swap_s0_robid)) ? wb4_data_i : 32'b0;
wire [5:0]  swap_s0_wb0_lo = {1'b0, wb0_data_i[4:0]} + {1'b0, swap_imm[4:0]};
wire [26:0] swap_s0_wb0_key = wb0_data_i[31:5] + swap_imm[31:5] + {26'b0, swap_s0_wb0_lo[5]};
wire [5:0]  swap_s0_wb1_lo = {1'b0, wb1_data_i[4:0]} + {1'b0, swap_imm[4:0]};
wire [26:0] swap_s0_wb1_key = wb1_data_i[31:5] + swap_imm[31:5] + {26'b0, swap_s0_wb1_lo[5]};
wire [5:0]  swap_s0_wb5_lo = {1'b0, wb5_data_i[4:0]} + {1'b0, swap_imm[4:0]};
wire [26:0] swap_s0_wb5_key = wb5_data_i[31:5] + swap_imm[31:5] + {26'b0, swap_s0_wb5_lo[5]};
wire [5:0]  swap_s0_mid_lo = {1'b0, swap_s0_mid_data[4:0]} + {1'b0, swap_imm[4:0]};
wire [26:0] swap_s0_mid_key = swap_s0_mid_data[31:5] + swap_imm[31:5] + {26'b0, swap_s0_mid_lo[5]};
wire [26:0] swap_s0_wb_key  = swap_s0_wb0_direct ? swap_s0_wb0_key :
                              swap_s0_wb1_direct ? swap_s0_wb1_key :
                              swap_s0_wb5_direct ? swap_s0_wb5_key :
                                                   swap_s0_mid_key;
wire swap_s0_earlyhit = swap_pending && !swap_s0_ready && !swap_s0_wbhit &&
                        ((early0_valid_i && (early0_robid_i == swap_s0_robid)) ||
                         (early1_valid_i && (early1_robid_i == swap_s0_robid)) ||
                         (early2_valid_i && (early2_robid_i == swap_s0_robid)));
wire swap_s1_earlyhit = swap_pending && !swap_s1_ready && !swap_s1_wbhit &&
                        ((early0_valid_i && (early0_robid_i == swap_s1_robid)) ||
                         (early1_valid_i && (early1_robid_i == swap_s1_robid)) ||
                         (early2_valid_i && (early2_robid_i == swap_s1_robid)));
wire swap_s0_alu0_earlyhit = swap_s0_earlyhit && early0_valid_i &&
                             (early0_robid_i == swap_s0_robid);
wire swap_s1_alu0_earlyhit = swap_s1_earlyhit && early0_valid_i &&
                             (early0_robid_i == swap_s1_robid);
wire swap_s0_alu1_earlyhit = swap_s0_earlyhit && early1_valid_i &&
                             (early1_robid_i == swap_s0_robid);
wire swap_s1_alu1_earlyhit = swap_s1_earlyhit && early1_valid_i &&
                             (early1_robid_i == swap_s1_robid);
wire swap_s0_mem_earlyhit = swap_s0_earlyhit && early2_valid_i &&
                            (early2_robid_i == swap_s0_robid);
wire swap_s1_mem_earlyhit = swap_s1_earlyhit && early2_valid_i &&
                            (early2_robid_i == swap_s1_robid);

// Preserve the common ALU0/ALU1/load -> oldest memory-operation zero-bubble
// cases without feeding their tags into the four-entry bypass priority scan.
wire head_s0_wb0_hit = !s0_val_valid[head] && wb0_valid_i &&
                       (wb0_robid_i == head_s0_robid_q);
wire head_s1_wb0_hit = !s1_val_valid[head] && wb0_valid_i &&
                       (wb0_robid_i == head_s1_robid_q);
wire head_s0_wb1_hit = !s0_val_valid[head] && wb1_valid_i &&
                       (wb1_robid_i == head_s0_robid_q);
wire head_s1_wb1_hit = !s1_val_valid[head] && wb1_valid_i &&
                       (wb1_robid_i == head_s1_robid_q);
wire head_s0_wb2_hit = !s0_val_valid[head] && wb2_valid_i &&
                       (wb2_robid_i == head_s0_robid_q);
wire head_s1_wb2_hit = !s1_val_valid[head] && wb2_valid_i &&
                       (wb2_robid_i == head_s1_robid_q);
// iter71 deliberately removes every raw DCache response from the functional
// RS_MEM issue cone. A dependent head entry is handed to LSU with its source
// ROB tag; LSU resolves the value from its local hit-pipe/MSHR state.
wire head_s0_fast_hit = 1'b0;
`ifdef SOC_USE
wire head_direct_ready =
                          queue_nonempty &&
                          valid[head] &&
                      (head_s0_wb0_hit || head_s1_wb0_hit ||
                       head_s0_wb1_hit || head_s1_wb1_hit ||
                       head_s0_wb2_hit || head_s1_wb2_hit ||
                       head_s0_fast_hit) &&
                      ((s0_ready[head] && s0_val_valid[head]) ||
                       s0_alu0_pred[head] || s0_alu1_pred[head] ||
                       s0_mem_pred[head] ||
                       head_s0_wb0_hit || head_s0_wb1_hit ||
                       head_s0_wb2_hit || head_s0_fast_hit) &&
                      ((s1_ready[head] && s1_val_valid[head]) ||
                       s1_alu0_pred[head] || s1_alu1_pred[head] ||
                       s1_mem_pred[head] ||
                       head_s1_wb0_hit || head_s1_wb1_hit ||
                       head_s1_wb2_hit);
`else
wire head_direct_ready = valid[head] &&
                      (head_s0_wb0_hit || head_s1_wb0_hit ||
                       head_s0_wb1_hit || head_s1_wb1_hit ||
                       head_s0_wb2_hit || head_s1_wb2_hit ||
                       head_s0_fast_hit) &&
                      ((s0_ready[head] && s0_val_valid[head]) ||
                       s0_alu0_pred[head] || s0_alu1_pred[head] ||
                       s0_mem_pred[head] ||
                       head_s0_wb0_hit || head_s0_wb1_hit ||
                       head_s0_wb2_hit || head_s0_fast_hit) &&
                      ((s1_ready[head] && s1_val_valid[head]) ||
                       s1_alu0_pred[head] || s1_alu1_pred[head] ||
                       s1_mem_pred[head] ||
                       head_s1_wb0_hit || head_s1_wb1_hit ||
                       head_s1_wb2_hit);
`endif

// A physical-head ordinary Load/Store may leave the RS when a missing source
// is guaranteed by an LSU front-token or MSHR descriptor. Both descriptor
// sets are registered, so this path cannot recreate DCache -> RS_MEM timing.
// LL/SC/CACOP and every non-head entry remain on the conservative path.
wire [`LSU_MISS_DEPTH-1:0] head_s0_miss_match;
wire [`LSU_MISS_DEPTH-1:0] head_s1_miss_match;
genvar mw;
generate
for (mw = 0; mw < `LSU_MISS_DEPTH; mw = mw + 1) begin : g_head_local_miss
    assign head_s0_miss_match[mw] = miss_wait_valid_i[mw]
        && !s0_val_valid[head]
        && (miss_wait_robids_i[mw*`ROB_W +: `ROB_W] == head_s0_robid_q);
    assign head_s1_miss_match[mw] = miss_wait_valid_i[mw]
        && !s1_val_valid[head]
        && (miss_wait_robids_i[mw*`ROB_W +: `ROB_W] == head_s1_robid_q);
end
endgenerate
wire head_is_plain_load = plain_load[head];
wire head_is_plain_store = plain_store[head];
wire head_base_present = (s0_ready[head] && s0_val_valid[head])
                      || s0_alu0_pred[head] || s0_alu1_pred[head]
                      || s0_mem_pred[head]
                      || head_s0_wb0_hit || head_s0_wb1_hit
                      || head_s0_wb2_hit;
wire head_data_present = (s1_ready[head] && s1_val_valid[head])
                      || s1_alu0_pred[head] || s1_alu1_pred[head]
                      || s1_mem_pred[head]
                      || head_s1_wb0_hit || head_s1_wb1_hit
                      || head_s1_wb2_hit;
wire [`LSU_TOKEN_DEPTH-1:0] head_s0_token_match;
wire [`LSU_TOKEN_DEPTH-1:0] head_s1_token_match;
genvar tw;
generate
for (tw = 0; tw < `LSU_TOKEN_DEPTH; tw = tw + 1) begin : g_head_local_token
    assign head_s0_token_match[tw] = pending_load_valid_i[tw]
        && !s0_val_valid[head]
        && (pending_load_robids_i[tw*`ROB_W +: `ROB_W] == head_s0_robid_q);
    assign head_s1_token_match[tw] = pending_load_valid_i[tw]
        && !s1_val_valid[head]
        && (pending_load_robids_i[tw*`ROB_W +: `ROB_W] == head_s1_robid_q);
end
endgenerate
wire head_s0_pending_match = |head_s0_token_match;
wire head_s1_pending_match = |head_s1_token_match;
wire head_base_registered_producer = head_s0_pending_match
                                    || (|head_s0_miss_match);
wire head_data_registered_producer = head_s1_pending_match
                                    || (|head_s1_miss_match);
`ifdef SOC_USE
wire head_local_wait_ready =
                           queue_nonempty &&
                           valid[head] && !is_cacop[head]
                           && (head_is_plain_load || head_is_plain_store)
                           && (head_base_present
                               || head_base_registered_producer)
                           && (head_data_present
                               || (head_is_plain_store
                                   && head_data_registered_producer))
                           && (!head_base_present
                               || (head_is_plain_store
                                   && !head_data_present));
`else
wire head_local_wait_ready = valid[head] && !is_cacop[head]
                           && (head_is_plain_load || head_is_plain_store)
                           && (head_base_present
                               || head_base_registered_producer)
                           && (head_data_present
                               || (head_is_plain_store
                                   && head_data_registered_producer))
                           && (!head_base_present
                               || (head_is_plain_store
                                   && !head_data_present));
`endif

wire [`RS_MEM_IDX_W-1:0] head1_idx =
    head + {{(`RS_MEM_IDX_W-1){1'b0}}, 1'b1};
wire [`RS_MEM_IDX_W-1:0] head2_idx =
    head + {{(`RS_MEM_IDX_W-2){1'b0}}, 2'd2};
`ifdef SOC_USE
wire head1_known_cached = scan_known_cached_phys[head1_idx];
`else
`endif
wire head1_s0_wb2_hit = !s0_val_valid[head1_idx] && wb2_valid_i &&
                        (wb2_robid_i == s0_robid[head1_idx]);
wire head1_s1_wb2_hit = !s1_val_valid[head1_idx] && wb2_valid_i &&
                        (wb2_robid_i == s1_robid[head1_idx]);
wire head1_s0_wb1_hit = !s0_val_valid[head1_idx] && wb1_valid_i &&
                        (wb1_robid_i == s0_robid[head1_idx]);
wire head1_s1_wb1_hit = !s1_val_valid[head1_idx] && wb1_valid_i &&
                        (wb1_robid_i == s1_robid[head1_idx]);
// Keep raw DCache wakeup on the queue head only.  A head+1 dependency is
// captured by the registered WB bus on the next edge; allowing it to steer
// the head1 selector made the cache tag traverse both selection networks.
wire head1_s0_fast_hit = 1'b0;
wire head1_s1_fast_hit = 1'b0;
wire head1_s0_refill_hit = 1'b0;
wire head1_s1_refill_hit = 1'b0;
wire head1_direct_ready = (count > {{(`RS_MEM_OCC_W-1){1'b0}}, 1'b1}) &&
                       valid[head] && !is_ord_barrier[head] &&
`ifdef SOC_USE
                       scan_known_cached_phys[head] &&
`else
`endif
                       !entry_ready[head] && !head_direct_ready &&
                       valid[head1_idx] && !is_ord_barrier[head1_idx] &&
`ifdef SOC_USE
                       head1_known_cached &&
`else
`endif
                       (head1_s0_wb1_hit || head1_s1_wb1_hit ||
                        head1_s0_wb2_hit || head1_s1_wb2_hit ||
                        head1_s0_fast_hit || head1_s1_fast_hit ||
                        head1_s0_refill_hit || head1_s1_refill_hit) &&
                       ((s0_ready[head1_idx] && s0_val_valid[head1_idx]) ||
                        s0_alu0_pred[head1_idx] || s0_alu1_pred[head1_idx] ||
                        s0_mem_pred[head1_idx] ||
                        head1_s0_wb1_hit || head1_s0_wb2_hit ||
                        head1_s0_fast_hit || head1_s0_refill_hit) &&
                       ((s1_ready[head1_idx] && s1_val_valid[head1_idx]) ||
                        s1_alu0_pred[head1_idx] || s1_alu1_pred[head1_idx] ||
                        s1_mem_pred[head1_idx] ||
                        head1_s1_wb1_hit || head1_s1_wb2_hit ||
                        head1_s1_fast_hit || head1_s1_refill_hit);

// 年龄序扫描：
// - 普通 load 可越过前方未就绪的普通 load
// - store/ll/sc/cacop 仅在其已是队内最老项时发射（不可越过未发 load）
// - 前方有未就绪屏障则停止
// Rotate physical queue vectors into age order, then use a balanced priority
// tree. This preserves the full-window oldest-ready policy while avoiding an
// eight-deep procedural priority chain into issue_idx and the LSU.
wire [`RS_MEM_SIZE-1:0] scan_ready_phys;
wire [`RS_MEM_SIZE-1:0] scan_barrier_phys;
wire [`RS_MEM_SIZE-1:0] scan_valid_phys;
genvar sv;
generate
for (sv = 0; sv < `RS_MEM_SIZE; sv = sv + 1) begin : g_scan_vectors
    assign scan_ready_phys[sv]   = entry_ready[sv];
    assign scan_barrier_phys[sv] = is_ord_barrier[sv];
    assign scan_valid_phys[sv]   = valid[sv];
end
endgenerate

wire [(2*`RS_MEM_SIZE)-1:0] scan_ready_double =
    {scan_ready_phys, scan_ready_phys} >> head;
wire [(2*`RS_MEM_SIZE)-1:0] scan_barrier_double =
    {scan_barrier_phys, scan_barrier_phys} >> head;
wire [(2*`RS_MEM_SIZE)-1:0] scan_valid_double =
    {scan_valid_phys, scan_valid_phys} >> head;
`ifdef SOC_USE
wire [(2*`RS_MEM_SIZE)-1:0] scan_known_cached_double =
    {scan_known_cached_phys, scan_known_cached_phys} >> head;
`else
`endif
wire [`RS_MEM_SIZE-1:0] scan_ready_age =
    scan_ready_double[`RS_MEM_SIZE-1:0];
wire [`RS_MEM_SIZE-1:0] scan_barrier_age =
    scan_barrier_double[`RS_MEM_SIZE-1:0];
wire [`RS_MEM_SIZE-1:0] scan_valid_age =
    scan_valid_double[`RS_MEM_SIZE-1:0];
`ifdef SOC_USE
wire [`RS_MEM_SIZE-1:0] scan_known_cached_age =
    scan_known_cached_double[`RS_MEM_SIZE-1:0];
`else
`endif
wire [`RS_MEM_SIZE-1:0] scan_live_age =
    scan_valid_age & scan_count_mask;
wire [`RS_MEM_SIZE-1:0] scan_blocker_age =
    scan_live_age & scan_barrier_age;
`ifdef SOC_USE
wire [`RS_MEM_SIZE-1:0] scan_unknown_mat_age =
    scan_live_age & ~scan_known_cached_age;
`else
`endif
wire [`RS_MEM_SIZE-1:0] scan_candidate_age;
`ifdef SOC_USE
localparam integer MEM_ISSUE_SCAN_LIMIT =
    (`MEM_ISSUE_SCAN_N < 1) ? 1 :
    ((`MEM_ISSUE_SCAN_N > `RS_MEM_SIZE) ? `RS_MEM_SIZE : `MEM_ISSUE_SCAN_N);
`else
`endif
assign scan_candidate_age[0] = scan_live_age[0] && scan_ready_age[0];
genvar sa;
generate
for (sa = 1; sa < `RS_MEM_SIZE; sa = sa + 1) begin : g_scan_candidates
    // A non-head ordering operation is ineligible and stops younger loads.
`ifdef SOC_USE
    assign scan_candidate_age[sa] =
                                  (sa < MEM_ISSUE_SCAN_LIMIT) &&
                                  scan_known_cached_age[sa] &&
                                  !(|scan_unknown_mat_age[sa:0]) &&
                                  scan_live_age[sa]
                                  && scan_ready_age[sa]
                                  && !(|scan_blocker_age[sa:0]);
`else
    assign scan_candidate_age[sa] = scan_live_age[sa]
                                  && scan_ready_age[sa]
                                  && !(|scan_blocker_age[sa:0]);
`endif
end
endgenerate

wire scan_tree_valid = |scan_candidate_age;
reg [`RS_MEM_IDX_W-1:0] scan_tree_age;
always @(*) begin
    // RS_MEM_SIZE is eight; group 8 -> 4 -> 2 for a shallow encoder.
    if (|scan_candidate_age[3:0]) begin
        if (|scan_candidate_age[1:0])
            scan_tree_age = scan_candidate_age[0] ? 3'd0 : 3'd1;
        else
            scan_tree_age = scan_candidate_age[2] ? 3'd2 : 3'd3;
    end else begin
        if (|scan_candidate_age[5:4])
            scan_tree_age = scan_candidate_age[4] ? 3'd4 : 3'd5;
        else
            scan_tree_age = scan_candidate_age[6] ? 3'd6 : 3'd7;
    end
end

always @(*) begin
    issue_idx = head;
    issue_sel_valid = 1'b0;
    base_issue_idx = head + scan_tree_age;
`ifdef SOC_USE
    base_issue_age = {{(`RS_MEM_OCC_W-`RS_MEM_IDX_W){1'b0}}, scan_tree_age};
`else
    base_issue_age = scan_tree_age;
`endif
    base_issue_valid = scan_tree_valid;
    if (head_local_wait_ready) begin
        base_issue_idx = head;
        base_issue_age = {`RS_MEM_OCC_W{1'b0}};
        base_issue_valid = 1'b1;
    end else if (head_direct_ready) begin
        base_issue_idx = head;
        base_issue_age = {`RS_MEM_OCC_W{1'b0}};
        base_issue_valid = 1'b1;
`ifdef SOC_USE
    end else if ((MEM_ISSUE_SCAN_LIMIT > 1) && head1_direct_ready &&
`else
    end else if (head1_direct_ready &&
`endif
                 (!base_issue_valid ||
                  (base_issue_age > {{(`RS_MEM_OCC_W-1){1'b0}}, 1'b1}))) begin
        base_issue_idx = head1_idx;
        base_issue_age = {{(`RS_MEM_OCC_W-1){1'b0}}, 1'b1};
        base_issue_valid = 1'b1;
    end
    issue_idx = base_issue_idx;
    issue_sel_valid = base_issue_valid;
end

assign issue_need_swap = issue_fire && (issue_idx != head);

// push 口的唤醒命中/旁路（参数是端口信号，非数组变址；同样彻底内联）
// push 同理带 !push_srcX_ready 门控：ready-from-ARF 的入站操作数直接取 push_srcX_val，
// 不允许被 tag=0 的误命中改写。
// Newborn entries consume a same-edge WB from the local delayed mirrors on
// their first visible cycle.  The compare only feeds a small registered port
// selector; it never feeds the wide queue-data D directly.
wire push0_ord_barrier = push_is_cacop_i
                       || push_mem_op_i[`MEM_OP_ST_W]
                       || push_mem_op_i[`MEM_OP_ST_B]
                       || push_mem_op_i[`MEM_OP_ST_H]
                       || push_mem_op_i[`MEM_OP_SC_W]
                       || push_mem_op_i[`MEM_OP_LL_W];
wire push0_plain_load = !push_is_cacop_i
                      && (push_mem_op_i[`MEM_OP_LD_W]
                       || push_mem_op_i[`MEM_OP_LD_B]
                       || push_mem_op_i[`MEM_OP_LD_H]
                       || push_mem_op_i[`MEM_OP_LD_BU]
                       || push_mem_op_i[`MEM_OP_LD_HU]);
wire push0_plain_store = push_mem_op_i[`MEM_OP_ST_W]
                       || push_mem_op_i[`MEM_OP_ST_B]
                       || push_mem_op_i[`MEM_OP_ST_H];
wire push1_ord_barrier = push1_is_cacop_i
                       || push1_mem_op_i[`MEM_OP_ST_W]
                       || push1_mem_op_i[`MEM_OP_ST_B]
                       || push1_mem_op_i[`MEM_OP_ST_H]
                       || push1_mem_op_i[`MEM_OP_SC_W]
                       || push1_mem_op_i[`MEM_OP_LL_W];
wire push1_plain_load = !push1_is_cacop_i
                      && (push1_mem_op_i[`MEM_OP_LD_W]
                       || push1_mem_op_i[`MEM_OP_LD_B]
                       || push1_mem_op_i[`MEM_OP_LD_H]
                       || push1_mem_op_i[`MEM_OP_LD_BU]
                       || push1_mem_op_i[`MEM_OP_LD_HU]);
wire push1_plain_store = push1_mem_op_i[`MEM_OP_ST_W]
                       || push1_mem_op_i[`MEM_OP_ST_B]
                       || push1_mem_op_i[`MEM_OP_ST_H];
wire        push_s0_wbhit = !push_src0_ready_i &&
                           ((wb0_valid_i && (wb0_robid_i == push_src0_robid_i)) ||
                            (wb1_valid_i && (wb1_robid_i == push_src0_robid_i)) ||
                            (wb2_valid_i && (wb2_robid_i == push_src0_robid_i)) ||
                            (wb3_valid_i && (wb3_robid_i == push_src0_robid_i)) ||
                            (wb4_valid_i && (wb4_robid_i == push_src0_robid_i)) ||
                            (wb5_valid_i && (wb5_robid_i == push_src0_robid_i)));
wire        push_s1_wbhit = !push_src1_ready_i &&
                           ((wb0_valid_i && (wb0_robid_i == push_src1_robid_i)) ||
                            (wb1_valid_i && (wb1_robid_i == push_src1_robid_i)) ||
                            (wb2_valid_i && (wb2_robid_i == push_src1_robid_i)) ||
                            (wb3_valid_i && (wb3_robid_i == push_src1_robid_i)) ||
                            (wb4_valid_i && (wb4_robid_i == push_src1_robid_i)) ||
                            (wb5_valid_i && (wb5_robid_i == push_src1_robid_i)));
wire [2:0]  push_s0_wbsel = (wb0_valid_i && (wb0_robid_i == push_src0_robid_i)) ? 3'd0 :
                            (wb1_valid_i && (wb1_robid_i == push_src0_robid_i)) ? 3'd1 :
                            (wb2_valid_i && (wb2_robid_i == push_src0_robid_i)) ? 3'd2 :
                            (wb3_valid_i && (wb3_robid_i == push_src0_robid_i)) ? 3'd3 :
                            (wb4_valid_i && (wb4_robid_i == push_src0_robid_i)) ? 3'd4 : 3'd5;
wire [2:0]  push_s1_wbsel = (wb0_valid_i && (wb0_robid_i == push_src1_robid_i)) ? 3'd0 :
                            (wb1_valid_i && (wb1_robid_i == push_src1_robid_i)) ? 3'd1 :
                            (wb2_valid_i && (wb2_robid_i == push_src1_robid_i)) ? 3'd2 :
                            (wb3_valid_i && (wb3_robid_i == push_src1_robid_i)) ? 3'd3 :
                            (wb4_valid_i && (wb4_robid_i == push_src1_robid_i)) ? 3'd4 : 3'd5;
`ifdef SOC_USE
`else
wire        push_s0_early = !push_src0_ready_i && !push_s0_wbhit &&
                           ((early0_valid_i && (early0_robid_i == push_src0_robid_i)) ||
                            (early1_valid_i && (early1_robid_i == push_src0_robid_i)));
wire        push_s1_early = !push_src1_ready_i && !push_s1_wbhit &&
                           ((early0_valid_i && (early0_robid_i == push_src1_robid_i)) ||
                            (early1_valid_i && (early1_robid_i == push_src1_robid_i)));
`endif
wire        push_s0_alu1_pred = !push_src0_ready_i && !push_s0_wbhit &&
                                early1_valid_i &&
                                (early1_robid_i == push_src0_robid_i);
wire        push_s1_alu1_pred = !push_src1_ready_i && !push_s1_wbhit &&
                                early1_valid_i &&
                                (early1_robid_i == push_src1_robid_i);
wire        push_s0_alu0_pred = !push_src0_ready_i && !push_s0_wbhit &&
                                early0_valid_i &&
                                (early0_robid_i == push_src0_robid_i);
wire        push_s1_alu0_pred = !push_src1_ready_i && !push_s1_wbhit &&
                                early0_valid_i &&
                                (early0_robid_i == push_src1_robid_i);
wire        push_s0_mem_pred = !push_src0_ready_i && !push_s0_wbhit &&
                               early2_valid_i &&
                               (early2_robid_i == push_src0_robid_i);
wire        push_s1_mem_pred = !push_src1_ready_i && !push_s1_wbhit &&
                               early2_valid_i &&
                               (early2_robid_i == push_src1_robid_i);
`ifdef SOC_USE
`else
wire [31:0] push_s0_wbdat = 32'b0;
wire [31:0] push_s1_wbdat = 32'b0;

`endif
wire        push1_s0_wbhit = !push1_src0_ready_i &&
                            ((wb0_valid_i && (wb0_robid_i == push1_src0_robid_i)) ||
                             (wb1_valid_i && (wb1_robid_i == push1_src0_robid_i)) ||
                             (wb2_valid_i && (wb2_robid_i == push1_src0_robid_i)) ||
                             (wb3_valid_i && (wb3_robid_i == push1_src0_robid_i)) ||
                             (wb4_valid_i && (wb4_robid_i == push1_src0_robid_i)) ||
                             (wb5_valid_i && (wb5_robid_i == push1_src0_robid_i)));
wire        push1_s1_wbhit = !push1_src1_ready_i &&
                            ((wb0_valid_i && (wb0_robid_i == push1_src1_robid_i)) ||
                             (wb1_valid_i && (wb1_robid_i == push1_src1_robid_i)) ||
                             (wb2_valid_i && (wb2_robid_i == push1_src1_robid_i)) ||
                             (wb3_valid_i && (wb3_robid_i == push1_src1_robid_i)) ||
                             (wb4_valid_i && (wb4_robid_i == push1_src1_robid_i)) ||
                             (wb5_valid_i && (wb5_robid_i == push1_src1_robid_i)));
wire [2:0]  push1_s0_wbsel = (wb0_valid_i && (wb0_robid_i == push1_src0_robid_i)) ? 3'd0 :
                             (wb1_valid_i && (wb1_robid_i == push1_src0_robid_i)) ? 3'd1 :
                             (wb2_valid_i && (wb2_robid_i == push1_src0_robid_i)) ? 3'd2 :
                             (wb3_valid_i && (wb3_robid_i == push1_src0_robid_i)) ? 3'd3 :
                             (wb4_valid_i && (wb4_robid_i == push1_src0_robid_i)) ? 3'd4 : 3'd5;
wire [2:0]  push1_s1_wbsel = (wb0_valid_i && (wb0_robid_i == push1_src1_robid_i)) ? 3'd0 :
                             (wb1_valid_i && (wb1_robid_i == push1_src1_robid_i)) ? 3'd1 :
                             (wb2_valid_i && (wb2_robid_i == push1_src1_robid_i)) ? 3'd2 :
                             (wb3_valid_i && (wb3_robid_i == push1_src1_robid_i)) ? 3'd3 :
                             (wb4_valid_i && (wb4_robid_i == push1_src1_robid_i)) ? 3'd4 : 3'd5;
`ifdef SOC_USE
`else
wire        push1_s0_early = !push1_src0_ready_i && !push1_s0_wbhit &&
                           ((early0_valid_i && (early0_robid_i == push1_src0_robid_i)) ||
                            (early1_valid_i && (early1_robid_i == push1_src0_robid_i)));
wire        push1_s1_early = !push1_src1_ready_i && !push1_s1_wbhit &&
                           ((early0_valid_i && (early0_robid_i == push1_src1_robid_i)) ||
                            (early1_valid_i && (early1_robid_i == push1_src1_robid_i)));
`endif
wire        push1_s0_alu1_pred = !push1_src0_ready_i && !push1_s0_wbhit &&
                                 early1_valid_i &&
                                 (early1_robid_i == push1_src0_robid_i);
wire        push1_s1_alu1_pred = !push1_src1_ready_i && !push1_s1_wbhit &&
                                 early1_valid_i &&
                                 (early1_robid_i == push1_src1_robid_i);
wire        push1_s0_alu0_pred = !push1_src0_ready_i && !push1_s0_wbhit &&
                                 early0_valid_i &&
                                 (early0_robid_i == push1_src0_robid_i);
wire        push1_s1_alu0_pred = !push1_src1_ready_i && !push1_s1_wbhit &&
                                 early0_valid_i &&
                                 (early0_robid_i == push1_src1_robid_i);
wire        push1_s0_mem_pred = !push1_src0_ready_i && !push1_s0_wbhit &&
                                early2_valid_i &&
                                (early2_robid_i == push1_src0_robid_i);
wire        push1_s1_mem_pred = !push1_src1_ready_i && !push1_s1_wbhit &&
                                early2_valid_i &&
                                (early2_robid_i == push1_src1_robid_i);
`ifdef SOC_USE
`else
wire [31:0] push1_s0_wbdat = 32'b0;
wire [31:0] push1_s1_wbdat = 32'b0;
`endif
// Preserve the ROB bypass priority while keeping ALU2 data out of the generic
// ROB/dispatch path.  ALU0 and ALU1 suppress the ALU2 branch exactly as they
// do in rob.v; lower-priority producers do not.
// The ROB timing-copy port excludes all three ALU result branches.  Restore
// those branches here with separate adders, retaining ROB's ALU0 > ALU1 >
// ALU2 priority.  This keeps a just-produced ALU result out of the generic
// ROB-read mux and carry chain while preserving the key captured at push.
wire push0_wb0_key_hit = push_src0_rob_wait_i && wb0_valid_i
                       && (wb0_robid_i == push_src0_robid_i);
wire push0_wb1_key_hit = push_src0_rob_wait_i && wb1_valid_i
                       && (wb1_robid_i == push_src0_robid_i)
                       && !(wb0_valid_i && (wb0_robid_i == push_src0_robid_i));
wire push0_wb5_key_hit = push_src0_rob_wait_i && wb5_valid_i
                       && (wb5_robid_i == push_src0_robid_i)
                       && !(wb0_valid_i && (wb0_robid_i == push_src0_robid_i))
                       && !(wb1_valid_i && (wb1_robid_i == push_src0_robid_i));
wire push1_wb0_key_hit = push1_src0_rob_wait_i && wb0_valid_i
                       && (wb0_robid_i == push1_src0_robid_i);
wire push1_wb1_key_hit = push1_src0_rob_wait_i && wb1_valid_i
                       && (wb1_robid_i == push1_src0_robid_i)
                       && !(wb0_valid_i && (wb0_robid_i == push1_src0_robid_i));
wire push1_wb5_key_hit = push1_src0_rob_wait_i && wb5_valid_i
                       && (wb5_robid_i == push1_src0_robid_i)
                       && !(wb0_valid_i && (wb0_robid_i == push1_src0_robid_i))
                       && !(wb1_valid_i && (wb1_robid_i == push1_src0_robid_i));
(* keep = "true" *) wire [31:0] push0_generic_vaddr = push_src0_key_val_i + push_imm_i;
(* keep = "true" *) wire [31:0] push1_generic_vaddr = push1_src0_key_val_i + push1_imm_i;
(* keep = "true" *) wire [31:0] push0_wb0_vaddr = wb0_data_i + push_imm_i;
(* keep = "true" *) wire [31:0] push0_wb1_vaddr = wb1_data_i + push_imm_i;
(* keep = "true" *) wire [31:0] push0_wb5_vaddr = wb5_data_i + push_imm_i;
(* keep = "true" *) wire [31:0] push1_wb0_vaddr = wb0_data_i + push1_imm_i;
(* keep = "true" *) wire [31:0] push1_wb1_vaddr = wb1_data_i + push1_imm_i;
(* keep = "true" *) wire [31:0] push1_wb5_vaddr = wb5_data_i + push1_imm_i;
wire [26:0] push0_key = push0_wb0_key_hit ? push0_wb0_vaddr[31:5]
                      : push0_wb1_key_hit ? push0_wb1_vaddr[31:5]
                      : push0_wb5_key_hit ? push0_wb5_vaddr[31:5]
                                           : push0_generic_vaddr[31:5];
wire [26:0] push1_key = push1_wb0_key_hit ? push1_wb0_vaddr[31:5]
                      : push1_wb1_key_hit ? push1_wb1_vaddr[31:5]
                      : push1_wb5_key_hit ? push1_wb5_vaddr[31:5]
                                           : push1_generic_vaddr[31:5];

assign occupancy_o = count;
assign can_accept_o = !swap_pending && (count != `RS_MEM_SIZE);
assign can_accept_two_o = !swap_pending && (count <= (`RS_MEM_SIZE - 2));
wire push0_fire = push_valid_i && can_accept_o;
wire push1_fire = push1_valid_i && can_accept_two_o && push0_fire;
// Apply LSU's registered fused-pair policy without selecting and decoding a
// second copy of mem_op through issue_idx.  By construction, every non-head
// scan candidate is on the non-barrier path; the complete opcode set makes
// that candidate a plain load.  Only an age-zero/head issue needs the already
// local head opcode check.  This is exactly equivalent to decoding the final
// selected mem_op, but removes that mux from issue_valid -> LSU payload CEs.
wire issue_is_plain_load = (base_issue_age != {`RS_MEM_OCC_W{1'b0}})
                         || head_is_plain_load;
`ifdef SOC_USE
assign issue_valid_o =
                    queue_nonempty &&
                    !swap_pending && issue_sel_valid && lsu_ready_i
                    && (!lsu_plain_only_i || issue_is_plain_load);
`else
assign issue_valid_o = !swap_pending && issue_sel_valid && lsu_ready_i
                    && (!lsu_plain_only_i || issue_is_plain_load);
`endif
assign issue_fire = issue_valid_o;

assign issue_robid_o = robid[issue_idx];
assign issue_mem_op_o = mem_op[issue_idx];
assign issue_is_cacop_o = is_cacop[issue_idx];
assign issue_cacop_op_o = cacop_op[issue_idx];
wire issue_local_wait = (issue_idx == head) && head_local_wait_ready;
wire issue_base_wait = issue_local_wait && !head_base_present;
wire issue_store_wait = issue_local_wait && head_is_plain_store
                     && !head_data_present;
// issue_idx now drives only the final 8:1 data selection; WB tag compares and
// bypass priority are computed in parallel for every entry above.
assign issue_base_o = issue_base_candidate[issue_idx];
assign issue_base_ready_o = !issue_base_wait;
assign issue_base_robid_o = s0_robid[issue_idx];
// Source-1 never consumes the raw DCache response in iter69.  A dependent
// plain Store is released by the registered pending/miss descriptor and LSU
// fills its data locally; all other operations wait for a registered WB.
// Every operation that consumes source 1 is an ordering entry and the scan
// permits it to issue only from the physical head.  Non-head issues are plain
// loads and ignore source 1.  Keep Store data off the oldest/issue_idx cone.
assign issue_wdata_o = head_s1_wb0_hit ? wb0_data_i
                     : head_s1_wb1_hit ? wb1_data_i
                     : head_s1_wb2_hit ? wb2_data_i
                     : s1_alu0_pred[head] ? wb0_data_i
                     : s1_alu1_pred[head] ? wb1_data_i
                     : s1_mem_pred[head] ? wb2_data_i
                     : s1_val[head];
assign issue_wdata_ready_o = !issue_store_wait;
assign issue_wdata_robid_o = head_s1_robid_q;
assign issue_imm_o = imm[issue_idx];

// Fuse two ready plain loads from the ordered load-only window.  The isolated
// selector returns the physical second entry so dequeue can compact the ring.
wire [`RS_MEM_IDX_W-1:0] issue_pair_idx;
wire issue_pair_candidate;
wire [`RS_MEM_SIZE-1:0] issue_pair_grant;
wire [`ROB_W-1:0] issue_pair_robid_iso;
wire [`MEM_OP_NUM-1:0] issue_pair_mem_op_iso;
wire [31:0] issue_pair_base_iso, issue_pair_imm_iso;
rs_mem_pair_select u_pair_select(
    .head_i(head),
    .count_i(count),
    .stored_ready_i(pair_stored_ready_flat),
    .plain_load_i(pair_plain_load_flat),
    .robid_i(pair_robid_flat),
    .mem_op_i(pair_mem_op_flat),
    .base_i(pair_base_flat),
    .imm_i(pair_imm_flat),
    .vaddr_key_i(pair_vaddr_key_flat),
`ifdef SOC_USE
    .known_cached_i(scan_known_cached_phys),
`else
`endif
    .candidate_o(issue_pair_candidate),
    .pair_grant_o(issue_pair_grant),
    .pair_idx_o(issue_pair_idx),
    .pair_robid_o(issue_pair_robid_iso),
    .pair_mem_op_o(issue_pair_mem_op_iso),
    .pair_base_o(issue_pair_base_iso),
    .pair_imm_o(issue_pair_imm_iso)
);
`ifdef SOC_USE
wire pair_issue_enabled = (MEM_ISSUE_SCAN_LIMIT > 1);
assign issue_pair_valid_o  =
                           pair_issue_enabled &&
                           !swap_pending && lsu_ready_i && issue_pair_candidate;
`else
assign issue_pair_valid_o  = !swap_pending && lsu_ready_i && issue_pair_candidate;
`endif
assign issue_pair_robid_o  = issue_pair_robid_iso;
assign issue_pair_mem_op_o = issue_pair_mem_op_iso;
assign issue_pair_base_o   = issue_pair_base_iso;
assign issue_pair_imm_o    = issue_pair_imm_iso;
// Keep the LSU acceptance term separate from the selector's global
// candidate reduction.  A pair_grant bit is asserted only for the selected
// candidate, so per-entry relocation can use accept && pair_grant directly
// without routing candidate through every payload register CE.
`ifdef SOC_USE
wire issue_pair_accept =
                         pair_issue_enabled &&
                         !swap_pending && lsu_ready_i && lsu_pair_ready_i;
`else
wire issue_pair_accept = !swap_pending && lsu_ready_i && lsu_pair_ready_i;
`endif
wire issue_pair_fire = issue_pair_candidate && issue_pair_accept;
// Compute both possible post-issue indices from the registered head.  The
// late pair decision now selects between precomputed values instead of
// entering the address adder that feeds the queue read muxes.
wire [`RS_MEM_IDX_W-1:0] head_after_idx = issue_pair_fire ? head2_idx
                                        : issue_fire ? head1_idx : head;
wire [`ROB_W-1:0] head_after_s0_robid = issue_pair_fire
                                      ? s0_robid[head2_idx]
                                      : s0_robid[head1_idx];
wire [`ROB_W-1:0] head_after_s1_robid = issue_pair_fire
                                      ? s1_robid[head2_idx]
                                      : s1_robid[head1_idx];
wire head_after_from_push0 = push0_fire && (tail == head_after_idx);
wire head_after_from_pair_move = issue_pair_fire
                               && (issue_pair_idx == head_after_idx)
                               && (issue_pair_idx != head1_idx);
wire head_after_from_swap_move = issue_fire && !issue_pair_fire
                               && issue_need_swap
                               && (issue_idx == head_after_idx);

always @(posedge clk) begin
    wb0d_data <= wb0_data_i;
    wb1d_data <= wb1_data_i;
    wb2d_data <= wb2_data_i;
    wb3d_data <= wb3_data_i;
    wb4d_data <= wb4_data_i;
    wb5d_data <= wb5_data_i;
end

always @(posedge clk) begin
    if (reset || flush_i) begin
        head <= {`RS_MEM_IDX_W{1'b0}};
        tail <= {`RS_MEM_IDX_W{1'b0}};
        count <= {`RS_MEM_OCC_W{1'b0}};
        scan_count_mask <= {`RS_MEM_SIZE{1'b0}};
        swap_pending <= 1'b0;
        swap_pair_vaddr_key <= 27'b0;
        head_s0_robid_q <= {`ROB_W{1'b0}};
        head_s1_robid_q <= {`ROB_W{1'b0}};
        for (i = 0; i < `RS_MEM_SIZE; i = i + 1) begin
            valid[i] <= 1'b0;
            ord_barrier[i] <= 1'b0;
            plain_load[i] <= 1'b0;
            plain_store[i] <= 1'b0;
            s0_birth_pending[i] <= 1'b0;
            s1_birth_pending[i] <= 1'b0;
            s0_alu0_pred[i] <= 1'b0;
            s0_alu1_pred[i] <= 1'b0;
            s0_mem_pred[i] <= 1'b0;
            s1_alu0_pred[i] <= 1'b0;
            s1_alu1_pred[i] <= 1'b0;
            s1_mem_pred[i] <= 1'b0;
            pair_vaddr_key[i] <= 27'b0;
            s0_key_refresh[i] <= 1'b0;
            precomp_base[i] <= 32'b0;
        end
    end else begin
        for (i = 0; i < `RS_MEM_SIZE; i = i + 1) begin
            // Do not gate every operand CE with dynamic issue/head indices.
            // A removed entry may receive one harmless wakeup assignment;
            // the later dequeue/swap/push NBAs in this same always block win
            // for all live architectural state.  This cuts the internal
            // s*_robid -> unrelated s*_val CE timing cone.
            if (valid[i]) begin
                s0_birth_pending[i] <= 1'b0;
                s1_birth_pending[i] <= 1'b0;
                if (s0_wbhit[i]) begin
                    s0_ready[i]     <= 1'b1;
                    s0_val_valid[i] <= 1'b1;
                    s0_val[i]       <= s0_wbdat[i];
                    s0_alu0_pred[i] <= 1'b0;
                    s0_alu1_pred[i] <= 1'b0;
                    s0_mem_pred[i]  <= 1'b0;
                    // Defer key update off the ALU/WB carry cone; refresh below
                    // uses the newly registered s0_val on the next edge.
                    s0_key_refresh[i] <= 1'b1;
                    // ========== Timing optimization: Update pre-computed base ==========
                    precomp_base[i] <= s0_wbdat[i];
                    // ===================================================================
                end else if (s0_key_refresh[i]) begin
                    pair_vaddr_key[i] <= s0_ref_key[i];
                    s0_key_refresh[i] <= 1'b0;
                end else if (s0_earlyhit[i]) begin
                    s0_alu0_pred[i] <= s0_alu0_earlyhit[i];
                    s0_alu1_pred[i] <= s0_alu1_earlyhit[i];
                    s0_mem_pred[i]  <= s0_mem_earlyhit[i];
                    // ========== Timing optimization: Update pre-computed base ==========
                    // When early-wakeup happens, precompute what the final value will be
                    precomp_base[i] <= s0_alu0_earlyhit[i] ? wb0_data_i :
                                       s0_alu1_earlyhit[i] ? wb1_data_i :
                                       s0_mem_earlyhit[i]  ? wb2_data_i : s0_val[i];
                    // ===================================================================
                end
                if (s1_wbhit[i]) begin
                    s1_ready[i]     <= 1'b1;
                    s1_val_valid[i] <= 1'b1;
                    s1_val[i]       <= s1_wbdat[i];
                    s1_alu0_pred[i] <= 1'b0;
                    s1_alu1_pred[i] <= 1'b0;
                    s1_mem_pred[i]  <= 1'b0;
                end else if (s1_earlyhit[i]) begin
                    s1_alu0_pred[i] <= s1_alu0_earlyhit[i];
                    s1_alu1_pred[i] <= s1_alu1_earlyhit[i];
                    s1_mem_pred[i]  <= s1_mem_earlyhit[i];
                end
            end
        end

        // Compute the post-edge logical-head tag with the same write priority
        // as the queue arrays below.  Push write-through covers empty/refill
        // turnover; pair and swap overrides cover the two cases where the
        // post-advance head slot is rewritten on this edge or the next.
        if (head_after_from_push0) begin
            head_s0_robid_q <= push_src0_robid_i;
            head_s1_robid_q <= push_src1_robid_i;
        end else if (head_after_from_pair_move) begin
            head_s0_robid_q <= s0_robid[head1_idx];
            head_s1_robid_q <= s1_robid[head1_idx];
        end else if (head_after_from_swap_move) begin
            head_s0_robid_q <= head_s0_robid_q;
            head_s1_robid_q <= head_s1_robid_q;
        end else if (swap_pending && (swap_idx_r == head)) begin
            head_s0_robid_q <= swap_s0_robid;
            head_s1_robid_q <= swap_s1_robid;
        end else if (issue_pair_fire || issue_fire) begin
            head_s0_robid_q <= head_after_s0_robid;
            head_s1_robid_q <= head_after_s1_robid;
        end

        if (swap_pending) begin
            robid[swap_idx_r]        <= swap_robid;
            mem_op[swap_idx_r]       <= swap_mem_op;
            ord_barrier[swap_idx_r]  <= swap_ord_barrier;
            plain_load[swap_idx_r]   <= swap_plain_load;
            plain_store[swap_idx_r]  <= swap_plain_store;
            is_cacop[swap_idx_r]     <= swap_is_cacop;
            cacop_op[swap_idx_r]     <= swap_cacop_op;
            s0_ready[swap_idx_r]     <= swap_s0_ready || swap_s0_wbhit;
            s0_val_valid[swap_idx_r] <= swap_s0_val_valid || swap_s0_wbhit;
            s0_alu0_pred[swap_idx_r] <=
                (swap_s0_alu0_pred || swap_s0_alu0_earlyhit) && !swap_s0_wbhit;
            s0_alu1_pred[swap_idx_r] <=
                (swap_s0_alu1_pred || swap_s0_alu1_earlyhit) && !swap_s0_wbhit;
            s0_mem_pred[swap_idx_r] <=
                (swap_s0_mem_pred || swap_s0_mem_earlyhit) && !swap_s0_wbhit;
            s0_val[swap_idx_r]       <= swap_s0_wbhit ? swap_s0_wbdat : swap_s0_val;
            s0_robid[swap_idx_r]     <= swap_s0_robid;
            // ========== Timing optimization: Update pre-computed base ==========
            precomp_base[swap_idx_r] <= swap_s0_wbhit ? swap_s0_wbdat : swap_s0_val;
            // ===================================================================
            s1_ready[swap_idx_r]     <= swap_s1_ready || swap_s1_wbhit;
            s1_val_valid[swap_idx_r] <= swap_s1_val_valid || swap_s1_wbhit;
            s1_alu0_pred[swap_idx_r] <=
                (swap_s1_alu0_pred || swap_s1_alu0_earlyhit) && !swap_s1_wbhit;
            s1_alu1_pred[swap_idx_r] <=
                (swap_s1_alu1_pred || swap_s1_alu1_earlyhit) && !swap_s1_wbhit;
            s1_mem_pred[swap_idx_r] <=
                (swap_s1_mem_pred || swap_s1_mem_earlyhit) && !swap_s1_wbhit;
            s1_val[swap_idx_r]       <= swap_s1_wbhit ? swap_s1_wbdat : swap_s1_val;
            s1_robid[swap_idx_r]     <= swap_s1_robid;
            imm[swap_idx_r]          <= swap_imm;
            // Avoid swap_s0_wb_key (ALU carry).  Refresh from the installed
            // registered s0_val when it is or becomes valid.
            if (swap_s0_wbhit || swap_s0_val_valid) begin
                s0_key_refresh[swap_idx_r] <= 1'b1;
            end else begin
                pair_vaddr_key[swap_idx_r] <= swap_pair_vaddr_key;
                s0_key_refresh[swap_idx_r] <= 1'b0;
            end
            valid[swap_idx_r]        <= 1'b1;
            s0_birth_pending[swap_idx_r] <= 1'b0;
            s1_birth_pending[swap_idx_r] <= 1'b0;
            swap_pending             <= 1'b0;
        end

        // Array relocation is needed only for a genuine non-head issue. The
        // local pending-source path is the physical head and only advances it.
        if (issue_fire && !issue_pair_fire) begin
            if (issue_need_swap) begin
                // 未就绪队头挪到被越过槽，保持年龄序；本拍发射项按已位于 head 出队
                swap_pending      <= 1'b1;
                swap_idx_r        <= issue_idx;
                swap_robid        <= robid[head];
                swap_mem_op       <= mem_op[head];
                swap_ord_barrier  <= ord_barrier[head];
                swap_plain_load   <= plain_load[head];
                swap_plain_store  <= plain_store[head];
                swap_is_cacop     <= is_cacop[head];
                swap_cacop_op     <= cacop_op[head];
                swap_s0_ready     <= s0_ready[head] || s0_wbhit[head];
                swap_s0_val_valid <= s0_val_valid[head] || s0_wbhit[head];
                swap_s0_alu0_pred <=
                    (s0_alu0_pred[head] || s0_alu0_earlyhit[head]) && !s0_wbhit[head];
                swap_s0_alu1_pred <=
                    (s0_alu1_pred[head] || s0_alu1_earlyhit[head]) && !s0_wbhit[head];
                swap_s0_mem_pred <=
                    (s0_mem_pred[head] || s0_mem_earlyhit[head]) && !s0_wbhit[head];
                swap_s0_val       <= s0_wbhit[head] ? s0_wbdat[head] : s0_val[head];
                swap_s0_robid     <= head_s0_robid_q;
                swap_s1_ready     <= s1_ready[head] || s1_wbhit[head];
                swap_s1_val_valid <= s1_val_valid[head] || s1_wbhit[head];
                swap_s1_alu0_pred <=
                    (s1_alu0_pred[head] || s1_alu0_earlyhit[head]) && !s1_wbhit[head];
                swap_s1_alu1_pred <=
                    (s1_alu1_pred[head] || s1_alu1_earlyhit[head]) && !s1_wbhit[head];
                swap_s1_mem_pred <=
                    (s1_mem_pred[head] || s1_mem_earlyhit[head]) && !s1_wbhit[head];
                swap_s1_val       <= s1_wbhit[head] ? s1_wbdat[head] : s1_val[head];
                swap_s1_robid     <= head_s1_robid_q;
                swap_imm          <= imm[head];
                swap_pair_vaddr_key <= pair_vaddr_key[head];
            end
            head <= head + {{(`RS_MEM_IDX_W-1){1'b0}}, 1'b1};
        end

`ifdef SOC_USE
        // Pair compaction is controlled by the stored-ready pair lane. The
        // per-entry grant keeps candidate reduction off payload clock enables.
`else
        // Pair compaction is controlled only by the stored-ready pair lane.
        // Keeping it outside the ordinary/refill issue branch prevents the
        // DCache refill decision from becoming a payload-array CE input.
`endif
        if (issue_pair_accept) begin
                // For a non-adjacent pair, move the skipped head+1 entry into
                // the removed second load's slot.  Advancing by two then
                // leaves a contiguous live ring without a hole or lost tail
                // entry.  Loads inside this window are already allowed to
                // issue out of order; ordering barriers terminate selection.
                for (i = 0; i < `RS_MEM_SIZE; i = i + 1) begin
`ifdef SOC_USE
                    if (issue_pair_grant[i] &&
                        (i[`RS_MEM_IDX_W-1:0] != head1_idx)) begin
`else
                    if (issue_pair_grant[i] && (i != head1_idx)) begin
`endif
                    valid[i] <= valid[head1_idx];
                    robid[i] <= robid[head1_idx];
                    mem_op[i] <= mem_op[head1_idx];
                    ord_barrier[i] <= ord_barrier[head1_idx];
                    plain_load[i] <= plain_load[head1_idx];
                    plain_store[i] <= plain_store[head1_idx];
                    is_cacop[i] <= is_cacop[head1_idx];
                    cacop_op[i] <= cacop_op[head1_idx];
                    s0_ready[i] <=
                        s0_ready[head1_idx] || s0_wbhit[head1_idx];
                    s0_val_valid[i] <=
                        s0_val_valid[head1_idx] || s0_wbhit[head1_idx];
                    s0_birth_pending[i] <= 1'b0;
                    s0_alu0_pred[i] <=
                        (s0_alu0_pred[head1_idx] || s0_alu0_earlyhit[head1_idx])
                        && !s0_wbhit[head1_idx];
                    s0_alu1_pred[i] <=
                        (s0_alu1_pred[head1_idx] || s0_alu1_earlyhit[head1_idx])
                        && !s0_wbhit[head1_idx];
                    s0_mem_pred[i] <=
                        (s0_mem_pred[head1_idx] || s0_mem_earlyhit[head1_idx])
                        && !s0_wbhit[head1_idx];
                    s0_val[i] <= s0_wbhit[head1_idx]
                                              ? s0_wbdat[head1_idx]
                                              : s0_val[head1_idx];
                    s0_robid[i] <= s0_robid[head1_idx];
                    precomp_base[i] <= s0_wbhit[head1_idx]
                                              ? s0_wbdat[head1_idx]
                                              : s0_val[head1_idx];
                    s1_ready[i] <=
                        s1_ready[head1_idx] || s1_wbhit[head1_idx];
                    s1_val_valid[i] <=
                        s1_val_valid[head1_idx] || s1_wbhit[head1_idx];
                    s1_birth_pending[i] <= 1'b0;
                    s1_alu0_pred[i] <=
                        (s1_alu0_pred[head1_idx] || s1_alu0_earlyhit[head1_idx])
                        && !s1_wbhit[head1_idx];
                    s1_alu1_pred[i] <=
                        (s1_alu1_pred[head1_idx] || s1_alu1_earlyhit[head1_idx])
                        && !s1_wbhit[head1_idx];
                    s1_mem_pred[i] <=
                        (s1_mem_pred[head1_idx] || s1_mem_earlyhit[head1_idx])
                        && !s1_wbhit[head1_idx];
                    s1_val[i] <= s1_wbhit[head1_idx]
                                              ? s1_wbdat[head1_idx]
                                              : s1_val[head1_idx];
                    s1_robid[i] <= s1_robid[head1_idx];
                    imm[i] <= imm[head1_idx];
                    if (s0_wbhit[head1_idx]) begin
                        s0_key_refresh[i] <= 1'b1;
                    end else begin
                        pair_vaddr_key[i] <= pair_vaddr_key[head1_idx];
                        s0_key_refresh[i] <= s0_key_refresh[head1_idx];
                    end
                    end
                end
                if (issue_pair_candidate)
                    head <= head + 2'd2;
        end

        if (push0_fire) begin
            valid[tail] <= 1'b1;
            robid[tail] <= push_robid_i;
            mem_op[tail] <= push_mem_op_i;
            ord_barrier[tail] <= push0_ord_barrier;
            plain_load[tail] <= push0_plain_load;
            plain_store[tail] <= push0_plain_store;
            is_cacop[tail] <= push_is_cacop_i;
            cacop_op[tail] <= push_cacop_op_i;
            s0_ready[tail] <= push_src0_ready_i;
            s0_val_valid[tail] <= push_src0_ready_i;
            s0_birth_pending[tail] <= push_s0_wbhit;
            s0_birth_sel[tail] <= push_s0_wbsel;
            s0_alu0_pred[tail] <= push_s0_alu0_pred;
            s0_alu1_pred[tail] <= push_s0_alu1_pred;
            s0_mem_pred[tail] <= push_s0_mem_pred;
            s0_val[tail] <= push_src0_ready_i ? push_src0_val_i : 32'b0;
            s0_robid[tail] <= push_src0_robid_i;
            precomp_base[tail] <= push_src0_ready_i ? push_src0_val_i : 32'b0;
            s1_ready[tail] <= push_src1_ready_i;
            s1_val_valid[tail] <= push_src1_ready_i;
            s1_birth_pending[tail] <= push_s1_wbhit;
            s1_birth_sel[tail] <= push_s1_wbsel;
            s1_alu0_pred[tail] <= push_s1_alu0_pred;
            s1_alu1_pred[tail] <= push_s1_alu1_pred;
            s1_mem_pred[tail] <= push_s1_mem_pred;
            s1_val[tail] <= push_src1_ready_i ? push_src1_val_i : 32'b0;
            s1_robid[tail] <= push_src1_robid_i;
            imm[tail] <= push_imm_i;
            pair_vaddr_key[tail] <= push_src0_ready_i
                                  ? push0_key : 27'b0;
            s0_key_refresh[tail] <= 1'b0;
        end

        if (push1_fire) begin
            valid[tail + {{(`RS_MEM_IDX_W-1){1'b0}}, 1'b1}] <= 1'b1;
            robid[tail + {{(`RS_MEM_IDX_W-1){1'b0}}, 1'b1}] <= push1_robid_i;
            mem_op[tail + {{(`RS_MEM_IDX_W-1){1'b0}}, 1'b1}] <= push1_mem_op_i;
            ord_barrier[tail + {{(`RS_MEM_IDX_W-1){1'b0}}, 1'b1}] <=
                push1_ord_barrier;
            plain_load[tail + {{(`RS_MEM_IDX_W-1){1'b0}}, 1'b1}] <=
                push1_plain_load;
            plain_store[tail + {{(`RS_MEM_IDX_W-1){1'b0}}, 1'b1}] <=
                push1_plain_store;
            is_cacop[tail + {{(`RS_MEM_IDX_W-1){1'b0}}, 1'b1}] <= push1_is_cacop_i;
            cacop_op[tail + {{(`RS_MEM_IDX_W-1){1'b0}}, 1'b1}] <= push1_cacop_op_i;
            s0_ready[tail + {{(`RS_MEM_IDX_W-1){1'b0}}, 1'b1}] <=
                push1_src0_ready_i || push1_s0_wbhit;
            s0_val_valid[tail + {{(`RS_MEM_IDX_W-1){1'b0}}, 1'b1}] <=
                push1_src0_ready_i;
            s0_birth_pending[tail + {{(`RS_MEM_IDX_W-1){1'b0}}, 1'b1}] <=
                push1_s0_wbhit;
            s0_birth_sel[tail + {{(`RS_MEM_IDX_W-1){1'b0}}, 1'b1}] <=
                push1_s0_wbsel;
            s0_alu0_pred[tail + {{(`RS_MEM_IDX_W-1){1'b0}}, 1'b1}] <=
                push1_s0_alu0_pred;
            s0_alu1_pred[tail + {{(`RS_MEM_IDX_W-1){1'b0}}, 1'b1}] <=
                push1_s0_alu1_pred;
            s0_mem_pred[tail + {{(`RS_MEM_IDX_W-1){1'b0}}, 1'b1}] <=
                push1_s0_mem_pred;
            s0_val[tail + {{(`RS_MEM_IDX_W-1){1'b0}}, 1'b1}] <=
                push1_src0_ready_i ? push1_src0_val_i : 32'b0;
            s0_robid[tail + {{(`RS_MEM_IDX_W-1){1'b0}}, 1'b1}] <= push1_src0_robid_i;
            precomp_base[tail + {{(`RS_MEM_IDX_W-1){1'b0}}, 1'b1}] <=
                push1_src0_ready_i ? push1_src0_val_i : 32'b0;
            s1_ready[tail + {{(`RS_MEM_IDX_W-1){1'b0}}, 1'b1}] <=
                push1_src1_ready_i || push1_s1_wbhit;
            s1_val_valid[tail + {{(`RS_MEM_IDX_W-1){1'b0}}, 1'b1}] <=
                push1_src1_ready_i;
            s1_birth_pending[tail + {{(`RS_MEM_IDX_W-1){1'b0}}, 1'b1}] <=
                push1_s1_wbhit;
            s1_birth_sel[tail + {{(`RS_MEM_IDX_W-1){1'b0}}, 1'b1}] <=
                push1_s1_wbsel;
            s1_alu0_pred[tail + {{(`RS_MEM_IDX_W-1){1'b0}}, 1'b1}] <=
                push1_s1_alu0_pred;
            s1_alu1_pred[tail + {{(`RS_MEM_IDX_W-1){1'b0}}, 1'b1}] <=
                push1_s1_alu1_pred;
            s1_mem_pred[tail + {{(`RS_MEM_IDX_W-1){1'b0}}, 1'b1}] <=
                push1_s1_mem_pred;
            s1_val[tail + {{(`RS_MEM_IDX_W-1){1'b0}}, 1'b1}] <=
                push1_src1_ready_i ? push1_src1_val_i : 32'b0;
            s1_robid[tail + {{(`RS_MEM_IDX_W-1){1'b0}}, 1'b1}] <= push1_src1_robid_i;
            imm[tail + {{(`RS_MEM_IDX_W-1){1'b0}}, 1'b1}] <= push1_imm_i;
            pair_vaddr_key[tail + {{(`RS_MEM_IDX_W-1){1'b0}}, 1'b1}] <=
                push1_src0_ready_i ? push1_key : 27'b0;
            s0_key_refresh[tail + {{(`RS_MEM_IDX_W-1){1'b0}}, 1'b1}] <= 1'b0;
        end

        if (push1_fire)
            tail <= tail + 2'd2;
        else if (push0_fire)
            tail <= tail + {{(`RS_MEM_IDX_W-1){1'b0}}, 1'b1};

        case ({push1_fire, push0_fire, issue_fire, issue_pair_fire})
            4'b1100: begin
                count <= count + 3'd2;
                scan_count_mask <= (scan_count_mask << 2) | {{(`RS_MEM_SIZE-2){1'b0}}, 2'b11};
            end
            4'b1110, 4'b0100: begin
                count <= count + 3'd1;
                scan_count_mask <= (scan_count_mask << 1) | {{(`RS_MEM_SIZE-1){1'b0}}, 1'b1};
            end
            4'b0111, 4'b0010: begin
                count <= count - 3'd1;
                scan_count_mask <= scan_count_mask >> 1;
            end
            4'b0011: begin
                count <= count - 3'd2;
                scan_count_mask <= scan_count_mask >> 2;
            end
            default: begin
                count <= count;
                scan_count_mask <= scan_count_mask;
            end
        endcase

    end
end

`ifdef SOC_USE
`ifdef SIMU
// Dequeued entries retain stale valid bits; count must remain the sole empty
// predicate and issue must never fire after the queue drains.
always @(posedge clk) begin
    if (!reset && !flush_i) begin
        if (issue_fire && !queue_nonempty)
            $error("RS_MEM issued with count=0 (robid=%0d)", issue_robid_o);
        if (count > RS_MEM_COUNT_MAX)
            $error("RS_MEM count out of range: count=%0d max=%0d",
                   count, RS_MEM_COUNT_MAX);
    end
end
`endif
`endif
`ifdef SYNTHESIS
// synthesis translate_off
reg [63:0] rsm_full_stall_cyc;
reg [63:0] rsm_src_stall_cyc;
reg [63:0] rsm_lsu_stall_cyc;
reg [63:0] rsm_dual_ready_opp;
reg [63:0] rsm_dual_ready_same_line;
reg [63:0] rsm_dual_ready_diff_bank;
reg [63:0] rsm_dual_ready_same_bank_diff_line;
reg [63:0] rsm_head_pair_same_line;
reg [63:0] rsm_pending_store_issue_cnt;
reg [63:0] rsm_miss_store_issue_cnt;
reg [63:0] rsm_raw_store_issue_cnt;
reg [63:0] rsm_pending_response_issue_cnt;
reg [63:0] rsm_pending_store_candidate_cyc;
reg [63:0] rsm_pending_base_issue_cnt;
reg [63:0] rsm_pending_base_candidate_cyc;
reg [`RS_MEM_IDX_W-1:0] rsm_pair_scan_idx;
reg [`RS_MEM_IDX_W-1:0] rsm_pair_second_idx;
reg rsm_pair_second_valid;
reg rsm_pair_scan_stop;
reg [31:0] rsm_pair_vaddr0;
reg [31:0] rsm_pair_vaddr1;
integer rsm_pair_age;
always @(*) begin
    rsm_pair_second_idx = head;
    rsm_pair_second_valid = 1'b0;
    rsm_pair_scan_stop = 1'b0;
    for (rsm_pair_age = 0; rsm_pair_age < `RS_MEM_SIZE;
         rsm_pair_age = rsm_pair_age + 1) begin
        rsm_pair_scan_idx = head + rsm_pair_age[`RS_MEM_IDX_W-1:0];
        if (!rsm_pair_second_valid && !rsm_pair_scan_stop
            && (rsm_pair_age[`RS_MEM_OCC_W-1:0] < count)
            && valid[rsm_pair_scan_idx]) begin
            if (is_ord_barrier[rsm_pair_scan_idx]) begin
                rsm_pair_scan_stop = 1'b1;
            end else if ((rsm_pair_scan_idx != issue_idx)
                         && entry_ready[rsm_pair_scan_idx]) begin
                rsm_pair_second_idx = rsm_pair_scan_idx;
                rsm_pair_second_valid = 1'b1;
            end
        end
    end
    rsm_pair_vaddr0 = issue_base_o + issue_imm_o;
    rsm_pair_vaddr1 = (s0_wbhit[rsm_pair_second_idx]
                       ? s0_wbdat[rsm_pair_second_idx]
                       : s0_val[rsm_pair_second_idx])
                    + imm[rsm_pair_second_idx];
end
always @(posedge clk) begin
    if (reset) begin
        rsm_full_stall_cyc <= 64'd0;
        rsm_src_stall_cyc  <= 64'd0;
        rsm_lsu_stall_cyc  <= 64'd0;
        rsm_dual_ready_opp  <= 64'd0;
        rsm_dual_ready_same_line <= 64'd0;
        rsm_dual_ready_diff_bank <= 64'd0;
        rsm_dual_ready_same_bank_diff_line <= 64'd0;
        rsm_head_pair_same_line <= 64'd0;
        rsm_pending_store_issue_cnt <= 64'd0;
        rsm_miss_store_issue_cnt <= 64'd0;
        rsm_raw_store_issue_cnt <= 64'd0;
        rsm_pending_response_issue_cnt <= 64'd0;
        rsm_pending_store_candidate_cyc <= 64'd0;
        rsm_pending_base_issue_cnt <= 64'd0;
        rsm_pending_base_candidate_cyc <= 64'd0;
    end else if (!flush_i) begin
        if (!can_accept_o && push_valid_i)
            rsm_full_stall_cyc <= rsm_full_stall_cyc + 64'd1;
        if ((count != {`RS_MEM_OCC_W{1'b0}}) && !issue_sel_valid)
            rsm_src_stall_cyc <= rsm_src_stall_cyc + 64'd1;
        if (issue_sel_valid && !lsu_ready_i)
            rsm_lsu_stall_cyc <= rsm_lsu_stall_cyc + 64'd1;
        if (issue_fire && !is_ord_barrier[issue_idx]
            && rsm_pair_second_valid) begin
            rsm_dual_ready_opp <= rsm_dual_ready_opp + 64'd1;
            if (rsm_pair_vaddr0[31:5] == rsm_pair_vaddr1[31:5])
                rsm_dual_ready_same_line <= rsm_dual_ready_same_line + 64'd1;
            else if (rsm_pair_vaddr0[5] != rsm_pair_vaddr1[5])
                rsm_dual_ready_diff_bank <= rsm_dual_ready_diff_bank + 64'd1;
            else
                rsm_dual_ready_same_bank_diff_line <=
                    rsm_dual_ready_same_bank_diff_line + 64'd1;
        end
        if (issue_fire && (issue_idx == head)
            && (count >= {{(`RS_MEM_OCC_W-2){1'b0}}, 2'd2})
            && valid[head + {{(`RS_MEM_IDX_W-1){1'b0}}, 1'b1}]
            && entry_ready[head + {{(`RS_MEM_IDX_W-1){1'b0}}, 1'b1}]
            && !is_ord_barrier[head]
            && !is_ord_barrier[head + {{(`RS_MEM_IDX_W-1){1'b0}}, 1'b1}]
            && (((issue_base_o + issue_imm_o) >> 5)
                == (((s0_wbhit[head + {{(`RS_MEM_IDX_W-1){1'b0}}, 1'b1}]
                     ? s0_wbdat[head + {{(`RS_MEM_IDX_W-1){1'b0}}, 1'b1}]
                     : s0_val[head + {{(`RS_MEM_IDX_W-1){1'b0}}, 1'b1}])
                    + imm[head + {{(`RS_MEM_IDX_W-1){1'b0}}, 1'b1}]) >> 5)))
            rsm_head_pair_same_line <= rsm_head_pair_same_line + 64'd1;
        if (head_local_wait_ready && head_is_plain_store
            && !head_data_present)
            rsm_pending_store_candidate_cyc <=
                rsm_pending_store_candidate_cyc + 64'd1;
        if (head_local_wait_ready && !head_base_present)
            rsm_pending_base_candidate_cyc <=
                rsm_pending_base_candidate_cyc + 64'd1;
        if (issue_fire && issue_base_wait)
            rsm_pending_base_issue_cnt <= rsm_pending_base_issue_cnt + 64'd1;
        if (issue_fire && issue_store_wait && head_s1_pending_match)
            rsm_pending_store_issue_cnt <= rsm_pending_store_issue_cnt + 64'd1;
        if (issue_fire && issue_store_wait && !head_s1_pending_match
            && (|head_s1_miss_match))
            rsm_miss_store_issue_cnt <= rsm_miss_store_issue_cnt + 64'd1;
        // Exact, mutually interpretable coverage: this is a real pending-path
        // issue that coincides with the raw response which iter68 preferred.
        // It is simulation-only and therefore cannot recreate the timing cone.
        if (issue_fire && issue_store_wait && head_s1_pending_match
            && fast2_valid_i && !s1_val_valid[head]
            && (fast2_robid_i == head_s1_robid_q))
            rsm_pending_response_issue_cnt <=
                rsm_pending_response_issue_cnt + 64'd1;
    end
end
// synthesis translate_on
`endif

endmodule
