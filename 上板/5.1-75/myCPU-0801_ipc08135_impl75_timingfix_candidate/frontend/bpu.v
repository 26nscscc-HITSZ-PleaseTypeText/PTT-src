// ============================================================
// bpu 模块（分支预测单元顶层）
// ------------------------------------------------------------
// 当前结构与约束：
// - P0 当拍：uBTB 命中则用其块，否则顺序取满至行边界（最多 4 条）；
// - P1 次拍：FTB/TAGE 1 拍延迟结果返回，与上一拍 P0 块比较，不同则覆盖 FTQ；
// - PC 更新优先级：flush > predec > P1 覆盖 > ftq_full 冻结 > P0 顺序；
// - 训练：FTB 全分支、TAGE 仅 COND、uBTB 仅向回跳（模块内过滤）；
// - RAS 双栈：P1 预测 CALL/RET 维护推测栈，flush 复制提交栈；
// - 预译码可提前训练 uBTB/FTB，并通过单项 skid 与优先级更高的提交训练仲裁。
// ============================================================
`include "mycpu.h"

module bpu(
    input  wire                       clk,
    input  wire                       reset,

    input  wire                       flush_i,
    input  wire [31:0]                flush_pc_i,
    input  wire                       predec_redirect_i,
    input  wire                       predec_update_pc_i,
    input  wire [31:0]                predec_redirect_pc_i,
    input  wire [`FTQ_W-1:0]          predec_redirect_id_i,
    input  wire                       predec_taken_i,
    // 预译码早期训练描述符
    input  wire [31:0]                predec_block_pc_i,
    input  wire [`BLK_LEN_W-1:0]      predec_length_i,
    input  wire [31:0]                predec_branch_target_i,
    input  wire [`BR_TYPE_W-1:0]      predec_br_type_i,
    input  wire                       predec_ras_call_i,
    input  wire                       predec_ras_ret_i,
    input  wire [31:0]                predec_ras_retaddr_i,
    input  wire [`FTQ_W-1:0]          ras_checkpoint_query_id_i,
    output wire [31:0]                ras_checkpoint_top_o,
    output wire                       ras_checkpoint_nonempty_o,

    input  wire                       ftq_full_i,

    output wire                       p0_valid_o,
    output wire [31:0]                p0_pc_o,
    output wire [`BLK_LEN_W-1:0]      p0_length_o,
    output wire                       p0_taken_o,
    output wire [31:0]                p0_target_o,

    output wire                       p1_valid_o,
    output wire                       p1_meta_valid_o,
    output wire                       p1_desc_valid_o,
    output wire                       p0_retry_pending_o,
    output wire [`BLK_LEN_W-1:0]      p1_length_o,
    output wire                       p1_taken_o,
    output wire [31:0]                p1_target_o,
    output wire [`BPU_META_W-1:0]     p1_meta_o,

    input  wire                       train_valid_i,
    input  wire [31:0]                train_pc_i,
    input  wire                       train_is_branch_i,
    input  wire                       train_taken_i,
    input  wire                       train_mispred_i,
    input  wire [31:0]                train_target_i,
    input  wire [`BR_TYPE_W-1:0]      train_br_type_i,
    input  wire [`BLK_LEN_W+1:2]      train_fall_through_i, // 顺序出口的块内字偏移
    input  wire [`BPU_META_W-1:0]     train_meta_i,

    input  wire                       cmt_is_call_i,
    input  wire                       cmt_is_ret_i,
    input  wire [31:0]                cmt_call_retaddr_i,
    input  wire                       cmt_hist_valid_i,
    input  wire                       cmt_hist_taken_i
);

localparam META_TAGE_VALID_BIT = 38;
localparam META_FTB_RESP_BIT   = 39;
localparam META_FTB_HIT_BIT    = 40;
localparam META_FTB_WAY_LSB    = 41;
localparam META_FTQ_ID_LSB     = 55;

// ---------------- 取指 PC ----------------
// P1 correction is a late signal.  Mapping the hold mux onto the FDRE clock
// enable gives it the shorter CE setup requirement; keep it on the D input so
// the same logic receives the normal data setup budget.
(* EXTRACT_ENABLE = "no" *) reg [31:0] pc;
reg [31:0] pc_r;
reg        flush_r;
reg        ftq_full_r;
reg        ftq_freeze_r;
reg        p0_wrote_r;
reg [`BLK_LEN_W-1:0] p0_length_r;
reg        p0_taken_r;
reg [31:0] p0_target_r;
reg        p0_ubtb_hit_r;
reg        p0_fallback_hit_r;
reg        p0_fallback_predict_r;
reg [`BR_TYPE_W-1:0] p0_btype_r;
reg        p0_retry_pending_r;
wire                       ubtb_hit;
wire                       p0_use_ubtb =
    (`P0_DIRECT_PREDICTOR_ONLY == 0) && ubtb_hit;
wire                       fallback_use;
wire                       fallback_predict;
wire [`BR_TYPE_W-1:0]      p0_btype_c;

initial begin
    pc = 32'h1c000000;
    p0_retry_pending_r = 1'b0;
end

// FTQ almost-full（留 2 槽）采用两拍冻结：首拍照常查询和写入，
// 连续两拍满才冻结 PC/查询——满边界不丢 P1 覆盖、不浪费取指拍
wire ftq_freeze = ftq_full_i && ftq_full_r;

always @(posedge clk) begin
    flush_r      <= flush_i;
    ftq_full_r   <= ftq_full_i;
    ftq_freeze_r <= ftq_freeze;
    if (flush_i)
        p0_wrote_r <= 1'b0;
    else
        // 与 FTQ 一致：仅“真正提交的 P0”置 wrote（同拍 p1_diff 则丢弃）
        p0_wrote_r <= p0_valid_o && !p1_diff;
    // Capture every issued P0 descriptor.  When a simultaneous P1 correction
    // cancels that P0, p0_wrote_r remains clear, so none of these payload
    // registers can be consumed; the retry overwrites them on its next issue.
    // Keeping p1_diff out of this enable prevents the FTB/TAGE P1 result from
    // crossing the complete predictor and ending at all P0 register CEs.
    if (p0_valid_o) begin
        p0_length_r <= p0_length_o;
        p0_taken_r  <= p0_taken_o;
        p0_target_r <= p0_target_o;
        p0_ubtb_hit_r <= p0_use_ubtb;
        p0_fallback_hit_r <= fallback_use;
        p0_fallback_predict_r <= fallback_predict;
        p0_btype_r    <= p0_btype_c;
    end
    pc_r <= pc;
end

// ---------------- 子模块：uBTB / FTB / TAGE / RAS ----------------
wire                       ubtb_taken;
wire [31:0]                ubtb_target;
wire [`BLK_LEN_W-1:0]      ubtb_length;
wire [`BR_TYPE_W-1:0]      ubtb_btype;
wire                       fallback_hit;
wire                       fallback_taken;
wire [31:0]                fallback_target;
wire [`BLK_LEN_W-1:0]      fallback_length;
wire [`BR_TYPE_W-1:0]      fallback_btype;
wire                       fallback_strong_taken;
wire                       fallback_static_direct;
assign fallback_use = (`P0_DIRECT_PREDICTOR_ONLY != 0)
                    ? fallback_hit : (!ubtb_hit && fallback_hit);
// The direct-mapped fallback is allowed to steer P0 only for descriptors
// that are safe or highly confident.  All hits may still supply block length.
assign fallback_predict =
    fallback_use &&
    (((fallback_btype == `BR_TYPE_COND)
      && ((`P0_DIRECT_PREDICTOR_ONLY != 0)
          ? fallback_taken : fallback_strong_taken)) ||
     (((fallback_btype == `BR_TYPE_UNCOND) ||
       (fallback_btype == `BR_TYPE_CALL)) && fallback_static_direct) ||
     ((fallback_btype == `BR_TYPE_RET) && !ras_empty));

wire                       ftb_hit;
wire                       ftb_resp_valid;
wire [1:0]                 ftb_hit_way;
wire [31:0]                ftb_target;
wire [31:0]                ftb_fall;
wire [`BR_TYPE_W-1:0]      ftb_btype;

wire                       tage_taken;
wire                       tage_resp_valid;
wire [`BPU_META_W-1:0]     tage_meta;

wire [31:0]                ras_top;
wire                       ras_empty;
wire                       ras_spec_push;
wire                       ras_spec_pop;
wire [31:0]                ras_spec_push_addr;
wire                       ras_checkpoint_save;
wire [`FTQ_W-1:0]          ras_checkpoint_id;
reg  [`FTQ_W-1:0]          ras_ftq_alloc_ptr;
wire                       p1_diff;
wire                       p1_hist_update_valid;
wire                       p1_hist_update_taken;

initial begin
    ras_ftq_alloc_ptr = {`FTQ_W{1'b0}};
end

// 镜像 FTQ 分配指针，使每个稳定预测块保存进入该块前的推测 RAS 状态。
always @(posedge clk) begin
    if (reset || flush_i)
        ras_ftq_alloc_ptr <= {`FTQ_W{1'b0}};
    else if (predec_redirect_i)
        ras_ftq_alloc_ptr <= predec_redirect_id_i + 1'b1;
    // 与 FTQ bpu_ptr 保持一致，仅在真正接纳 P0 时递增。
    // Retimed to the registered "P0 really allocated" decision.  Checkpoint
    // save is already at P1, so this removes the late FTB correction cone.
    else if (p0_wrote_r)
        ras_ftq_alloc_ptr <= ras_ftq_alloc_ptr + 1'b1;
end

// Keep predictor lookups independent of FTQ backpressure.  While frozen the
// PC is held and p0_valid_o is suppressed, so these responses are discarded;
// allowing the harmless lookup prevents ftq_full (which depends on the ROB
// release side) from entering fallback-hit/block-length/next-PC logic.
wire predictor_query_en = ~flush_i;
wire query_en = predictor_query_en && ~ftq_freeze;

wire [`BLK_LEN_W-1:0] ubtb_train_len =
    train_fall_through_i - train_pc_i[`BLK_LEN_W+1:2];

// IFU predecode can provide a trustworthy descriptor before commit.  Commit
// remains lossless and has priority; a two-entry FIFO absorbs consecutive
// collisions.  Repeated updates to the newest queued block are coalesced.
localparam PREDEC_TRAIN_Q_DEPTH = 2;
localparam PREDEC_TRAIN_Q_PTR_W = 1;
localparam PREDEC_TRAIN_Q_CNT_W = 2;

reg [31:0]                 predec_q_pc [0:PREDEC_TRAIN_Q_DEPTH-1];
reg [31:0]                 predec_q_target [0:PREDEC_TRAIN_Q_DEPTH-1];
reg [`BLK_LEN_W-1:0]       predec_q_length [0:PREDEC_TRAIN_Q_DEPTH-1];
reg                        predec_q_taken [0:PREDEC_TRAIN_Q_DEPTH-1];
reg [`BR_TYPE_W-1:0]       predec_q_btype [0:PREDEC_TRAIN_Q_DEPTH-1];
reg [PREDEC_TRAIN_Q_PTR_W-1:0] predec_q_rptr;
reg [PREDEC_TRAIN_Q_PTR_W-1:0] predec_q_wptr;
reg [PREDEC_TRAIN_Q_CNT_W-1:0] predec_q_count;

wire commit_btb_update = train_valid_i && train_is_branch_i;
wire predec_btb_update = predec_redirect_i;
wire predec_q_empty =
    (predec_q_count == {PREDEC_TRAIN_Q_CNT_W{1'b0}});
wire predec_q_full = (predec_q_count == PREDEC_TRAIN_Q_DEPTH);
wire service_predec_pending = !predec_q_empty && !commit_btb_update;
// Always register IFU predecode training in the small FIFO before it reaches
// uBTB/fallback/FTB write enables.  The former empty-queue direct service made
// IFU tail/predecode logic drive hundreds of predictor array CEs in one cycle.
wire service_predec_current = 1'b0;
wire btb_update_early_raw = service_predec_pending || service_predec_current;
wire btb_update_valid_raw = commit_btb_update || btb_update_early_raw;
wire [31:0] btb_update_pc_raw =
    commit_btb_update      ? train_pc_i :
    service_predec_pending ? predec_q_pc[predec_q_rptr] : 32'b0;
wire [31:0] btb_update_target_raw =
    commit_btb_update      ? train_target_i :
    service_predec_pending ? predec_q_target[predec_q_rptr] :
                             32'b0;
wire [`BLK_LEN_W-1:0] btb_update_length_raw =
    commit_btb_update      ? ubtb_train_len :
    service_predec_pending ? predec_q_length[predec_q_rptr] :
                             {`BLK_LEN_W{1'b0}};
wire btb_update_taken_raw =
    commit_btb_update      ? train_taken_i :
    service_predec_pending ? predec_q_taken[predec_q_rptr] : 1'b0;
wire [`BR_TYPE_W-1:0] btb_update_btype_raw =
    commit_btb_update      ? train_br_type_i :
    service_predec_pending ? predec_q_btype[predec_q_rptr] :
                             {`BR_TYPE_W{1'b0}};

// All predictor writes cross this register boundary.  It removes both the
// predecode FIFO read pointer and the commit/FTQ metadata mux from hundreds
// of uBTB/fallback/FTB RAM write-enables without reducing update throughput.
reg btb_update_valid;
reg btb_update_early;
reg [31:0] btb_update_pc;
reg [31:0] btb_update_target;
reg [`BLK_LEN_W-1:0] btb_update_length;
reg btb_update_taken;
reg [`BR_TYPE_W-1:0] btb_update_btype;
wire [`BLK_LEN_W+1:2] btb_update_fall_through =
    btb_update_pc[`BLK_LEN_W+1:2] + btb_update_length;

wire predec_q_dequeue = service_predec_pending;
wire predec_needs_queue = predec_btb_update && !service_predec_current;
wire [PREDEC_TRAIN_Q_PTR_W-1:0] predec_q_tail_ptr =
    predec_q_wptr - {{(PREDEC_TRAIN_Q_PTR_W-1){1'b0}}, 1'b1};
wire predec_matches_tail =
    !predec_q_empty &&
    (predec_q_pc[predec_q_tail_ptr] == predec_block_pc_i);
wire predec_tail_is_dequeue =
    predec_q_dequeue &&
    (predec_q_count == {{(PREDEC_TRAIN_Q_CNT_W-1){1'b0}}, 1'b1});
wire predec_q_merge_tail =
    predec_needs_queue && predec_matches_tail && !predec_tail_is_dequeue;
wire predec_q_enqueue =
    predec_needs_queue && !predec_q_merge_tail &&
    (!predec_q_full || predec_q_dequeue);
wire predec_q_overflow =
    predec_needs_queue && !predec_q_merge_tail &&
    predec_q_full && !predec_q_dequeue;
wire [PREDEC_TRAIN_Q_CNT_W-1:0] predec_q_count_next =
    (predec_q_enqueue && !predec_q_dequeue) ?
        predec_q_count + {{(PREDEC_TRAIN_Q_CNT_W-1){1'b0}}, 1'b1} :
    (!predec_q_enqueue && predec_q_dequeue) ?
        predec_q_count - {{(PREDEC_TRAIN_Q_CNT_W-1){1'b0}}, 1'b1} :
        predec_q_count;

// FTB entries describe the static branch position/type/target.  Rewriting an
// already-hit conditional branch or RET on every commit only consumes update
// bandwidth; uBTB still receives every commit below to train its direction
// counter.  Keep all misses, early-predecode fills and potentially dynamic
// UNCOND/CALL targets.
wire commit_ftb_known_hit =
    train_meta_i[META_FTB_RESP_BIT] && train_meta_i[META_FTB_HIT_BIT];
wire commit_ftb_static_descriptor =
    (train_br_type_i == `BR_TYPE_COND) ||
    (train_br_type_i == `BR_TYPE_RET);
wire ftb_commit_update_filtered_raw =
    commit_btb_update && commit_ftb_known_hit && commit_ftb_static_descriptor;
reg ftb_commit_update_filtered;
wire ftb_update_valid = btb_update_valid && !ftb_commit_update_filtered;

always @(posedge clk) begin
    if (reset) begin
        btb_update_valid  <= 1'b0;
        btb_update_early  <= 1'b0;
        btb_update_pc     <= 32'b0;
        btb_update_target <= 32'b0;
        btb_update_length <= {`BLK_LEN_W{1'b0}};
        btb_update_taken  <= 1'b0;
        btb_update_btype  <= {`BR_TYPE_W{1'b0}};
        ftb_commit_update_filtered <= 1'b0;
    end else begin
        btb_update_valid  <= btb_update_valid_raw;
        btb_update_early  <= btb_update_early_raw;
        btb_update_pc     <= btb_update_pc_raw;
        btb_update_target <= btb_update_target_raw;
        btb_update_length <= btb_update_length_raw;
        btb_update_taken  <= btb_update_taken_raw;
        btb_update_btype  <= btb_update_btype_raw;
        ftb_commit_update_filtered <= ftb_commit_update_filtered_raw;
    end
end

always @(posedge clk) begin
    if (reset) begin
        predec_q_rptr  <= {PREDEC_TRAIN_Q_PTR_W{1'b0}};
        predec_q_wptr  <= {PREDEC_TRAIN_Q_PTR_W{1'b0}};
        predec_q_count <= {PREDEC_TRAIN_Q_CNT_W{1'b0}};
    end else begin
        if (predec_q_merge_tail) begin
            predec_q_pc[predec_q_tail_ptr]     <= predec_block_pc_i;
            predec_q_target[predec_q_tail_ptr] <= predec_branch_target_i;
            predec_q_length[predec_q_tail_ptr] <= predec_length_i;
            predec_q_taken[predec_q_tail_ptr]  <= predec_taken_i;
            predec_q_btype[predec_q_tail_ptr]  <= predec_br_type_i;
        end else if (predec_q_enqueue) begin
            predec_q_pc[predec_q_wptr]     <= predec_block_pc_i;
            predec_q_target[predec_q_wptr] <= predec_branch_target_i;
            predec_q_length[predec_q_wptr] <= predec_length_i;
            predec_q_taken[predec_q_wptr]  <= predec_taken_i;
            predec_q_btype[predec_q_wptr]  <= predec_br_type_i;
            predec_q_wptr <= predec_q_wptr +
                {{(PREDEC_TRAIN_Q_PTR_W-1){1'b0}}, 1'b1};
        end
        if (predec_q_dequeue)
            predec_q_rptr <= predec_q_rptr +
                {{(PREDEC_TRAIN_Q_PTR_W-1){1'b0}}, 1'b1};
        predec_q_count <= predec_q_count_next;
    end
end

ubtb u_ubtb(
    .clk               (clk),
    .reset             (reset),
    .query_valid_i     (query_en),
    .query_pc_i        (pc),
    .hit_o             (ubtb_hit),
    .taken_o           (ubtb_taken),
    .target_o          (ubtb_target),
    .length_o          (ubtb_length),
    .br_type_o         (ubtb_btype),
    .update_valid_i    (btb_update_valid),
    .update_block_pc_i (btb_update_pc),
    .update_taken_i    (btb_update_taken),
    .update_target_i   (btb_update_target),
    .update_length_i   (btb_update_length),
    .update_br_type_i  (btb_update_btype),
    .update_early_i    (btb_update_early)
);

fallback_btb u_fallback_btb(
    .clk               (clk),
    .reset             (reset),
    // This small two-way table also supplies P0's fast block length.  Keep
    // its lookup independent of the fully-associative uBTB hit so the
    // sequential next-PC path never waits for the uBTB length mux.
    .query_valid_i     (predictor_query_en),
    .query_pc_i        (pc),
    .hit_o             (fallback_hit),
    .taken_o           (fallback_taken),
    .target_o          (fallback_target),
    .length_o          (fallback_length),
    .br_type_o         (fallback_btype),
    .strong_taken_o    (fallback_strong_taken),
    .static_direct_o   (fallback_static_direct),
    // Training remains independent of FTB filtering.
    .update_valid_i    (btb_update_valid),
    .update_block_pc_i (btb_update_pc),
    .update_taken_i    (btb_update_taken),
    .update_target_i   (btb_update_target),
    .update_length_i   (btb_update_length),
    .update_br_type_i  (btb_update_btype),
    .update_early_i    (btb_update_early)
);

ftb u_ftb(
    .clk                 (clk),
    .reset               (reset),
    .query_valid_i       (query_en),
    .query_pc_i          (pc),
    .hit_o               (ftb_hit),
    .resp_valid_o        (ftb_resp_valid),
    .hit_way_o           (ftb_hit_way),
    .jump_target_o       (ftb_target),
    .fall_through_o      (ftb_fall),
    .br_type_o           (ftb_btype),
    .update_valid_i      (ftb_update_valid),
    .update_block_pc_i   (btb_update_pc[31:2]),
    .update_jump_target_i(btb_update_target),
    .update_fall_through_i(btb_update_fall_through),
    .update_br_type_i    (btb_update_btype)
);

tage u_tage(
    .clk             (clk),
    .reset           (reset),
    .flush_i         (flush_i),
    .query_valid_i   (query_en),
    .query_pc_i      (pc[21:2]),
    .taken_o         (tage_taken),
    .resp_valid_o    (tage_resp_valid),
    .meta_o          (tage_meta),
    .train_valid_i   (train_valid_i && train_is_branch_i && (train_br_type_i == `BR_TYPE_COND)),
    .train_pc_i      (train_pc_i[21:2]),
    .train_taken_i   (train_taken_i),
    .train_mispred_i (train_mispred_i),
    .train_meta_i    (train_meta_i),
    .hist_checkpoint_save_i(ras_checkpoint_save),
    .hist_checkpoint_id_i(ras_checkpoint_id),
    .hist_restore_i  (predec_redirect_i && !flush_i),
    .hist_restore_id_i(predec_redirect_id_i),
    .hist_restore_append_i(predec_br_type_i == `BR_TYPE_COND),
    .hist_restore_taken_i(predec_taken_i),
    .hist_update_valid_i(p1_hist_update_valid),
    .hist_update_taken_i(p1_hist_update_taken),
    .cmt_hist_valid_i(cmt_hist_valid_i),
    .cmt_hist_taken_i(cmt_hist_taken_i)
);

ras u_ras(
    .clk              (clk),
    .reset            (reset),
    .flush_i          (flush_i),
    .spec_push_i      (ras_spec_push),
    .spec_push_addr_i (ras_spec_push_addr),
    .spec_pop_i       (ras_spec_pop),
    .top_addr_o       (ras_top),
    .empty_o          (ras_empty),
    .checkpoint_save_i(ras_checkpoint_save),
    .checkpoint_id_i  (ras_checkpoint_id),
    .checkpoint_query_id_i(ras_checkpoint_query_id_i),
    .checkpoint_top_addr_o(ras_checkpoint_top_o),
    .checkpoint_nonempty_o(ras_checkpoint_nonempty_o),
    .restore_i        (predec_redirect_i && !flush_i),
    .restore_id_i     (predec_redirect_id_i),
    .restore_push_i   (predec_ras_call_i),
    .restore_pop_i    (predec_ras_ret_i),
    .restore_push_addr_i(predec_ras_retaddr_i),
    .cmt_push_i       (cmt_is_call_i),
    .cmt_push_addr_i  (cmt_call_retaddr_i),
    .cmt_pop_i        (cmt_is_ret_i)
);

// ---------------- P0 基础块 ----------------
wire [3:0] words_to_eol = `CACHE_LINE_WORDS - {1'b0, pc[`CACHE_LINE_W-1:2]};
wire [`BLK_LEN_W-1:0] base_len = (words_to_eol > `FETCH_WIDTH) ? `FETCH_WIDTH
                                 : words_to_eol[`BLK_LEN_W-1:0];

// The uBTB still owns direction/target selection, but its 16-way length mux
// is deliberately excluded from next-PC generation.  The larger fallback
// table is trained by the same updates and provides a short two-way length
// lookup; on a miss, fetching to the line/fetch boundary is always safe and
// P1/FTB will correct the descriptor if necessary.
wire fast_length_hit = fallback_hit && (fallback_length <= base_len);
wire [`BLK_LEN_W-1:0] p0_len_raw =
    fast_length_hit ? fallback_length : base_len;
wire [`BLK_LEN_W-1:0] p0_len_c = (p0_len_raw === 3'd0 || p0_len_raw === 3'bx || p0_len_raw === 3'dz)
                                 ? 3'd1 : p0_len_raw;
wire                  p0_taken_c =
    p0_use_ubtb ? ubtb_taken :
    fallback_predict && fallback_taken;
wire [`BR_TYPE_W-1:0] p0_selected_btype =
    p0_use_ubtb       ? ubtb_btype :
    fallback_predict  ? fallback_btype :
                        `BR_TYPE_COND;
wire [31:0]           p0_target_c =
    ((p0_use_ubtb || fallback_predict) &&
     (p0_selected_btype == `BR_TYPE_RET) && !ras_empty) ? ras_top :
    p0_use_ubtb                                          ? ubtb_target :
    fallback_predict                                     ? fallback_target :
                                                           32'b0;
assign p0_btype_c = p0_selected_btype;

// P1 覆盖拍必须压掉 P0：此拍的 pc 是被 P1 否定的错误路径延续，
// 若照写会在 FTQ 中留下一个"元数据不跳、取指流却已跳走"的幽灵块，
// 提交级误预测检查察觉不到（pred_taken=0 且非分支），导致错误路径静默提交。
// p0_valid 不组合依赖 p1_diff，以切断 TAGE 到 blk_pc 写使能的时序长路径。
// 同拍冲突改由 FTQ：仍可写 LUTRAM，但不推进 bpu_ptr / 不置 p0_wrote（等价丢弃该 P0）。
// PC 仍用组合 p1_diff 当拍纠正（见下方 PC 更新）。
assign p0_valid_o   = query_en && !(predec_redirect_i && predec_update_pc_i);
assign p0_pc_o      = pc;
assign p0_length_o  = p0_len_c;
assign p0_taken_o   = p0_taken_c;
assign p0_target_o  = p0_target_c;

wire [31:0] p0_next = p0_taken_c ? p0_target_c : (pc + {27'b0, p0_len_c, 2'b00});

// ---------------- P1 覆盖块 ----------------
wire [`BLK_LEN_W-1:0] p1_len_raw =
    ftb_fall[`BLK_LEN_W+1:2] - pc_r[`BLK_LEN_W+1:2];
wire [`BLK_LEN_W-1:0] p1_len_mid   = (p1_len_raw === 3'd0) ? 3'd1 :
                                   (p1_len_raw > `FETCH_WIDTH) ? `FETCH_WIDTH : p1_len_raw;
wire [`BLK_LEN_W-1:0] p1_len_c   = (p1_len_mid === 3'bx || p1_len_mid === 3'dz) ? 3'd1 : p1_len_mid;

wire p1_taken_c = (ftb_btype == `BR_TYPE_COND)
                ? tage_taken : 1'b1;
wire [31:0] p1_target_c = (ftb_btype == `BR_TYPE_RET && !ras_empty) ? ras_top :
                          (ftb_btype == `BR_TYPE_RET) ? ftb_fall : ftb_target;

wire p1_result_valid = p0_wrote_r && !ftq_freeze_r && !flush_r && !flush_i &&
                       !predec_redirect_i && tage_resp_valid;

// 目标仅在双方都 taken 时比较，避免不跳转路径的无效目标触发伪覆盖。
wire p1_path_comparable = p1_result_valid && ftb_resp_valid && ftb_hit;
wire p1_direction_diff  = p1_path_comparable && (p1_taken_c != p0_taken_r);
wire p1_target_diff     = p1_path_comparable && p1_taken_c && p0_taken_r &&
                          (p1_target_c != p0_target_r);
wire p1_block_len_diff  = p1_path_comparable && (p1_len_c != p0_length_r);
assign p1_diff = p1_direction_diff || p1_target_diff || p1_block_len_diff;

reg [`BPU_META_W-1:0] p1_meta_pack;
always @(*) begin
    p1_meta_pack = tage_meta;
    p1_meta_pack[META_TAGE_VALID_BIT] = tage_resp_valid;
    p1_meta_pack[META_FTB_RESP_BIT]   = ftb_resp_valid;
    p1_meta_pack[META_FTB_HIT_BIT]    = ftb_hit;
    p1_meta_pack[META_FTB_WAY_LSB +: 2] = ftb_hit_way;
    p1_meta_pack[META_FTQ_ID_LSB +: `FTQ_W] = ras_checkpoint_id;
end

assign p1_valid_o   = p1_diff;
assign p1_meta_valid_o = p1_result_valid; // 保持每次 P1 结果写 meta（训练覆盖优于收窄）
// Settle one descriptor for every allocated P0 block.  FTB hits use the P1
// descriptor; misses retain the registered P0 descriptor.  FTQ can therefore
// write its descriptor array only at this fixed P1 point, independent of
// p1_diff, instead of multiplexing P0 and correction writes with TAGE control.
wire p1_desc_use_ftb = ftb_resp_valid && ftb_hit;
assign p1_desc_valid_o = p1_result_valid;
assign p1_length_o  = p1_desc_use_ftb ? p1_len_c    : p0_length_r;
assign p1_taken_o   = p1_desc_use_ftb ? p1_taken_c  : p0_taken_r;
assign p1_target_o  = p1_desc_use_ftb ? p1_target_c : p0_target_r;
assign p1_meta_o    = p1_meta_pack;

// A correction may coincide with a speculative P0 for the wrong path.  FTQ
// reserves that P0 slot immediately and overwrites it with the corrected P0
// on the next available query cycle.  Export the cancellation as registered
// state so p1_diff no longer drives FTQ pointer/array controls combinationally.
always @(posedge clk) begin
    if (reset || flush_i || predec_redirect_i)
        p0_retry_pending_r <= 1'b0;
    else if (p1_diff && p0_valid_o)
        p0_retry_pending_r <= 1'b1;
    else if (p0_retry_pending_r && p0_valid_o)
        p0_retry_pending_r <= 1'b0;
end
assign p0_retry_pending_o = p0_retry_pending_r;

wire [31:0] p1_next = p1_taken_c ? p1_target_c : ftb_fall;

// 每个稳定的 P1 预测都更新一次推测 RAS，而非仅在 P1 修正 P0 时更新。
// FTB 未命中时，FTQ 保留 P0 的 uBTB 描述符，因此用该描述符执行同样的更新。
wire p1_ras_settle = p1_result_valid && !reset;
wire p1_ftb_branch_valid = p1_ras_settle && ftb_resp_valid && ftb_hit;
wire p1_p0_branch_valid = p1_ras_settle && !p1_ftb_branch_valid &&
                          (p0_ubtb_hit_r || p0_fallback_predict_r);
wire p1_ras_event_valid = p1_ftb_branch_valid || p1_p0_branch_valid;
wire [`BR_TYPE_W-1:0] p1_ras_btype =
    p1_ftb_branch_valid ? ftb_btype : p0_btype_r;
wire [31:0] p1_ras_fall_through =
    p1_ftb_branch_valid ? ftb_fall
                        : (pc_r + {27'b0, p0_length_r, 2'b00});
wire p1_ras_push = p1_ras_event_valid && (p1_ras_btype == `BR_TYPE_CALL);
wire p1_ras_pop  = p1_ras_event_valid && (p1_ras_btype == `BR_TYPE_RET);

// 预译码通过检查点端口恢复并压栈，因此 P1 不会重复更新 IFU 稍后识别出的 BL。
// 每个已分配块都保存动作前状态，即使 FTQ 满导致 FTB/TAGE 响应未写入；该块仍可能
// 到达 IFU 并请求预译码回滚，检查点不能处于未初始化状态。
assign ras_checkpoint_save = p0_wrote_r && !flush_r && !flush_i &&
                             !predec_redirect_i && !reset;
assign ras_checkpoint_id   = ras_ftq_alloc_ptr;
assign ras_spec_push       = p1_ras_push;
assign ras_spec_push_addr  = p1_ras_fall_through;
assign ras_spec_pop        = p1_ras_pop;
// History forwarding is used only to form the next TAGE query index.  Keep it
// independent of IFU predecode cancellation: a redirect suppresses the current
// P0 allocation, and TAGE restores the checkpoint with higher sequential
// priority, so that cycle's query response is discarded.  Using the registered
// P1 event here cuts the IF-line -> predecoder -> folded-history timing cone.
wire p1_hist_settle = p0_wrote_r && !ftq_freeze_r && !flush_r &&
                      !flush_i && tage_resp_valid && !reset;
wire p1_hist_ftb_branch_valid =
    p1_hist_settle && ftb_resp_valid && ftb_hit;
wire p1_hist_p0_branch_valid =
    p1_hist_settle && !p1_hist_ftb_branch_valid &&
    (p0_ubtb_hit_r || p0_fallback_predict_r);
wire p1_hist_event_valid =
    p1_hist_ftb_branch_valid || p1_hist_p0_branch_valid;
wire [`BR_TYPE_W-1:0] p1_hist_btype =
    p1_hist_ftb_branch_valid ? ftb_btype : p0_btype_r;

assign p1_hist_update_valid = p1_hist_event_valid &&
                              (p1_hist_btype == `BR_TYPE_COND);
assign p1_hist_update_taken =
    p1_hist_ftb_branch_valid ? p1_taken_c : p0_taken_r;


`ifdef SYNTHESIS
// synthesis translate_off
reg [63:0] commit_cond_branch_count;
reg [63:0] commit_cond_mispred_count;
// mycpu_top 使用：训练口全分支与条件分支精度。
reg [63:0] commit_all_branch_count;
reg [63:0] commit_all_mispred_count;
// P1 correction来源统计：uBTB命中/未命中，以及方向、目标、块长差异。
reg [63:0] p1_correction_count;
reg [63:0] p1_correction_from_ubtb_hit_count;
reg [63:0] p1_correction_from_ubtb_miss_count;
reg [63:0] p1_correction_direction_count;
reg [63:0] p1_correction_target_count;
reg [63:0] p1_correction_length_count;
reg [63:0] ftb_commit_update_request_count;
reg [63:0] ftb_commit_update_filtered_count;
reg [63:0] ftb_update_sent_count;
reg [63:0] ftb_predec_update_sent_count;
reg [63:0] predec_train_request_count;
reg [63:0] predec_train_direct_count;
reg [63:0] predec_train_enqueue_count;
reg [63:0] predec_train_merge_count;
reg [63:0] predec_train_dequeue_count;
reg [63:0] predec_train_overflow_count;
reg [63:0] predec_train_queue_max_occupancy;

wire stat_commit_cond = train_valid_i && train_is_branch_i && (train_br_type_i == `BR_TYPE_COND);
wire stat_commit_br   = train_valid_i && train_is_branch_i;

always @(posedge clk) begin
    if (reset) begin
        commit_cond_branch_count<= 64'd0;
        commit_cond_mispred_count <= 64'd0;
        commit_all_branch_count <= 64'd0;
        commit_all_mispred_count <= 64'd0;
        p1_correction_count <= 64'd0;
        p1_correction_from_ubtb_hit_count <= 64'd0;
        p1_correction_from_ubtb_miss_count <= 64'd0;
        p1_correction_direction_count <= 64'd0;
        p1_correction_target_count <= 64'd0;
        p1_correction_length_count <= 64'd0;
        ftb_commit_update_request_count <= 64'd0;
        ftb_commit_update_filtered_count <= 64'd0;
        ftb_update_sent_count <= 64'd0;
        ftb_predec_update_sent_count <= 64'd0;
        predec_train_request_count <= 64'd0;
        predec_train_direct_count <= 64'd0;
        predec_train_enqueue_count <= 64'd0;
        predec_train_merge_count <= 64'd0;
        predec_train_dequeue_count <= 64'd0;
        predec_train_overflow_count <= 64'd0;
        predec_train_queue_max_occupancy <= 64'd0;
    end else begin
        if (predec_btb_update)
            predec_train_request_count <= predec_train_request_count + 64'd1;
        if (service_predec_current)
            predec_train_direct_count <= predec_train_direct_count + 64'd1;
        if (predec_q_enqueue)
            predec_train_enqueue_count <= predec_train_enqueue_count + 64'd1;
        if (predec_q_merge_tail)
            predec_train_merge_count <= predec_train_merge_count + 64'd1;
        if (predec_q_dequeue)
            predec_train_dequeue_count <= predec_train_dequeue_count + 64'd1;
        if (predec_q_overflow)
            predec_train_overflow_count <= predec_train_overflow_count + 64'd1;
        if ({{(64-PREDEC_TRAIN_Q_CNT_W){1'b0}}, predec_q_count_next} >
            predec_train_queue_max_occupancy)
            predec_train_queue_max_occupancy <=
                {{(64-PREDEC_TRAIN_Q_CNT_W){1'b0}}, predec_q_count_next};
        if (commit_btb_update)
            ftb_commit_update_request_count <=
                ftb_commit_update_request_count + 64'd1;
        if (ftb_commit_update_filtered)
            ftb_commit_update_filtered_count <=
                ftb_commit_update_filtered_count + 64'd1;
        if (ftb_update_valid)
            ftb_update_sent_count <= ftb_update_sent_count + 64'd1;
        if (ftb_update_valid && btb_update_early)
            ftb_predec_update_sent_count <=
                ftb_predec_update_sent_count + 64'd1;
        if (stat_commit_br) begin
            commit_all_branch_count <= commit_all_branch_count + 64'd1;
            if (train_mispred_i)
                commit_all_mispred_count <= commit_all_mispred_count + 64'd1;
        end
        if (stat_commit_cond) begin
            commit_cond_branch_count <= commit_cond_branch_count + 64'd1;
            if (train_mispred_i)
                commit_cond_mispred_count <= commit_cond_mispred_count + 64'd1;
        end
        if (p1_diff) begin
            p1_correction_count <= p1_correction_count + 64'd1;
            if (p0_ubtb_hit_r)
                p1_correction_from_ubtb_hit_count <=
                    p1_correction_from_ubtb_hit_count + 64'd1;
            else
                p1_correction_from_ubtb_miss_count <=
                    p1_correction_from_ubtb_miss_count + 64'd1;
            if (p1_direction_diff)
                p1_correction_direction_count <=
                    p1_correction_direction_count + 64'd1;
            if (p1_target_diff)
                p1_correction_target_count <=
                    p1_correction_target_count + 64'd1;
            if (p1_block_len_diff)
                p1_correction_length_count <=
                    p1_correction_length_count + 64'd1;
        end
    end
end
// synthesis translate_on
`endif

// ---------------- PC 更新 ----------------
always @(posedge clk) begin
    if (reset)
        pc <= 32'h1c000000;
    else if (flush_i)
        pc <= flush_pc_i;
    // flush 仅在到达拍装载重取地址；下一拍发出该块后必须按正常预测推进，
    // 否则同一块会连续两拍写入 FTQ 并被重复提交。
    else if (predec_redirect_i && predec_update_pc_i)
        pc <= predec_redirect_pc_i;
    else if (p1_diff)
        pc <= p1_next;
    else if (!ftq_freeze)
        pc <= p0_next;
end

endmodule
