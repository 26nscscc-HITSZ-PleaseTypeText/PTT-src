// ============================================================
// bpu 模块（分支预测单元顶层）
// ------------------------------------------------------------
// 当前结构与约束：
// - P0 当拍：uBTB 命中则用其块，否则顺序取满至行边界（最多 4 条）；
// - P1 次拍：FTB/TAGE 1 拍延迟结果返回，与上一拍 P0 块比较，不同则覆盖 FTQ；
// - PC 更新优先级：flush > predec > P1 覆盖 > ftq_full 冻结 > P0 顺序；
// - 训练：FTB 全分支、TAGE 仅 COND、uBTB 仅向回跳（模块内过滤）；
// - RAS 双栈：P1 预测 CALL/RET 维护推测栈，flush 复制提交栈；
// - JTC 仅记录普通 JIRL；预译码可提前训练 uBTB/FTB，并通过单项 skid
//   与优先级更高的提交训练仲裁。
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
    input  wire                       train_is_direct_b_i,  // JTC 不训练直接跳转 B
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
reg [31:0] pc;
reg [31:0] pc_r;
reg        flush_r;
reg        ftq_full_r;
reg        ftq_freeze_r;
reg        p0_wrote_r;
reg [`BLK_LEN_W-1:0] p0_length_r;
reg        p0_taken_r;
reg [31:0] p0_target_r;
reg        p0_ubtb_hit_r;
reg [`BR_TYPE_W-1:0] p0_btype_r;
wire                       ubtb_hit;
wire                       jtc_hit;
wire                       p0_jtc_select;
wire [`BR_TYPE_W-1:0]      p0_btype_c;

initial begin
    pc = 32'h1c000000;
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
    if (p0_valid_o && !p1_diff) begin
        p0_length_r <= p0_length_o;
        p0_taken_r  <= p0_taken_o;
        p0_target_r <= p0_target_o;
        p0_ubtb_hit_r <= ubtb_hit;
        p0_btype_r    <= p0_btype_c;
    end
    pc_r <= pc;
end

// ---------------- 子模块：uBTB / JTC / FTB / TAGE / RAS ----------------
wire                       ubtb_taken;
wire [31:0]                ubtb_target;
wire [`BLK_LEN_W-1:0]      ubtb_length;
wire [`BR_TYPE_W-1:0]      ubtb_btype;
wire [31:0]                jtc_target;
wire [`BLK_LEN_W-1:0]      jtc_length;

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
    else if (p0_valid_o && !p1_diff)
        ras_ftq_alloc_ptr <= ras_ftq_alloc_ptr + 1'b1;
end

wire query_en = ~ftq_freeze && ~flush_i;

wire [`BLK_LEN_W-1:0] ubtb_train_len =
    train_fall_through_i - train_pc_i[`BLK_LEN_W+1:2];

// IFU 预译码可在提交前给出可信描述符。
// 一拍 skid 在与单口 commit 训练冲突时保留；commit 无损优先。
// skid 已占时再来的第二拍预译码直接丢弃（不堵前端）。
reg                        predec_train_pending;
reg [31:0]                 predec_train_pc_r;
reg [31:0]                 predec_train_target_r;
reg [`BLK_LEN_W-1:0]       predec_train_length_r;
reg                        predec_train_taken_r;
reg [`BR_TYPE_W-1:0]       predec_train_btype_r;

wire commit_btb_update = train_valid_i && train_is_branch_i;
wire predec_btb_update = predec_redirect_i;
wire service_predec_pending = predec_train_pending && !commit_btb_update;
wire service_predec_current = !predec_train_pending &&
                              predec_btb_update && !commit_btb_update;
// 预译码早期训练与 JTC 的 P0 选择相互独立。
wire btb_update_early = service_predec_pending || service_predec_current;
wire btb_update_valid = commit_btb_update || btb_update_early;
wire [31:0] btb_update_pc =
    commit_btb_update      ? train_pc_i :
    service_predec_pending ? predec_train_pc_r : predec_block_pc_i;
wire [31:0] btb_update_target =
    commit_btb_update      ? train_target_i :
    service_predec_pending ? predec_train_target_r :
                             predec_branch_target_i;
wire [`BLK_LEN_W-1:0] btb_update_length =
    commit_btb_update      ? ubtb_train_len :
    service_predec_pending ? predec_train_length_r : predec_length_i;
wire btb_update_taken =
    commit_btb_update      ? train_taken_i :
    service_predec_pending ? predec_train_taken_r : predec_taken_i;
wire [`BR_TYPE_W-1:0] btb_update_btype =
    commit_btb_update      ? train_br_type_i :
    service_predec_pending ? predec_train_btype_r : predec_br_type_i;
wire [`BLK_LEN_W+1:2] btb_update_fall_through =
    btb_update_pc[`BLK_LEN_W+1:2] + btb_update_length;

always @(posedge clk) begin
    if (reset) begin
        predec_train_pending <= 1'b0;
    end else if (commit_btb_update) begin
        if (predec_btb_update && !predec_train_pending) begin
            predec_train_pending  <= 1'b1;
            predec_train_pc_r     <= predec_block_pc_i;
            predec_train_target_r <= predec_branch_target_i;
            predec_train_length_r <= predec_length_i;
            predec_train_taken_r  <= predec_taken_i;
            predec_train_btype_r  <= predec_br_type_i;
        end
    end else if (predec_train_pending) begin
        if (predec_btb_update) begin
            predec_train_pending  <= 1'b1;
            predec_train_pc_r     <= predec_block_pc_i;
            predec_train_target_r <= predec_branch_target_i;
            predec_train_length_r <= predec_length_i;
            predec_train_taken_r  <= predec_taken_i;
            predec_train_btype_r  <= predec_br_type_i;
        end else begin
            predec_train_pending <= 1'b0;
        end
    end
end

ubtb u_ubtb(
    .clk               (clk),
    .reset             (reset),
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

// 普通 JIRL 目标缓存，仅由 commit 训练，并排除直接跳转 B。
jirl_target_cache u_jirl_target_cache(
    .clk               (clk),
    .reset             (reset),
    .query_valid_i     (query_en),
    .query_pc_i        (pc[31:2]),
    .hit_o             (jtc_hit),
    .target_o          (jtc_target),
    .length_o          (jtc_length),
    .update_valid_i    (commit_btb_update &&
                        (train_br_type_i == `BR_TYPE_UNCOND) &&
                        !train_is_direct_b_i),
    .update_block_pc_i (train_pc_i[31:2]),
    .update_target_i   (train_target_i),
    .update_length_i   (ubtb_train_len)
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
    .update_valid_i      (btb_update_valid),
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

// uBTB 未命中时，JTC 命中可把 P0 预测为跳向缓存目标。该选择默认关闭：
// 当前间接跳恢复路径尚未满足全量 Linux difftest，启用前必须重新完成该回归。
`ifndef JIRL_TC_P0_ENABLE
`define JIRL_TC_P0_ENABLE 0
`endif
assign p0_jtc_select = (`JIRL_TC_P0_ENABLE != 0) && !ubtb_hit && jtc_hit &&
                       (jtc_length != {`BLK_LEN_W{1'b0}}) &&
                       (jtc_length <= base_len);
wire [`BLK_LEN_W-1:0] p0_len_raw =
    (ubtb_hit && (ubtb_length <= base_len)) ? ubtb_length :
    p0_jtc_select                            ? jtc_length : base_len;
wire [`BLK_LEN_W-1:0] p0_len_c = (p0_len_raw === 3'd0 || p0_len_raw === 3'bx || p0_len_raw === 3'dz)
                                 ? 3'd1 : p0_len_raw;
wire                  p0_taken_c = ubtb_hit ? ubtb_taken : p0_jtc_select;
wire [31:0]           p0_target_c =
    (ubtb_hit && (ubtb_btype == `BR_TYPE_RET) && !ras_empty) ? ras_top :
    ubtb_hit                                                   ? ubtb_target :
                                                                 jtc_target;
assign p0_btype_c = ubtb_hit ? ubtb_btype :
                      p0_jtc_select ? `BR_TYPE_UNCOND : `BR_TYPE_COND;

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

wire p1_taken_c = (ftb_btype == `BR_TYPE_COND) ? tage_taken : 1'b1;
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
assign p1_length_o  = p1_len_c;
assign p1_taken_o   = p1_taken_c;
assign p1_target_o  = p1_target_c;
assign p1_meta_o    = p1_meta_pack;

wire [31:0] p1_next = p1_taken_c ? p1_target_c : ftb_fall;

// 每个稳定的 P1 预测都更新一次推测 RAS，而非仅在 P1 修正 P0 时更新。
// FTB 未命中时，FTQ 保留 P0 的 uBTB 描述符，因此用该描述符执行同样的更新。
wire p1_ras_settle = p1_result_valid && !reset;
wire p1_ftb_branch_valid = p1_ras_settle && ftb_resp_valid && ftb_hit;
wire p1_ubtb_branch_valid = p1_ras_settle && !p1_ftb_branch_valid && p0_ubtb_hit_r;
wire p1_ras_event_valid = p1_ftb_branch_valid || p1_ubtb_branch_valid;
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
assign ras_checkpoint_id   = ras_ftq_alloc_ptr - 1'b1;
assign ras_spec_push       = p1_ras_push;
assign ras_spec_push_addr  = p1_ras_fall_through;
assign ras_spec_pop        = p1_ras_pop;
assign p1_hist_update_valid = p1_ras_event_valid &&
                              (p1_ras_btype == `BR_TYPE_COND);
assign p1_hist_update_taken = p1_ftb_branch_valid ? p1_taken_c : p0_taken_r;


`ifdef SYNTHESIS
// synthesis translate_off
reg [63:0] commit_cond_branch_count;
reg [63:0] commit_cond_mispred_count;
// mycpu_top 使用：训练口全分支与条件分支精度。
reg [63:0] commit_all_branch_count;
reg [63:0] commit_all_mispred_count;

wire stat_commit_cond = train_valid_i && train_is_branch_i && (train_br_type_i == `BR_TYPE_COND);
wire stat_commit_br   = train_valid_i && train_is_branch_i;

always @(posedge clk) begin
    if (reset) begin
        commit_cond_branch_count<= 64'd0;
        commit_cond_mispred_count <= 64'd0;
        commit_all_branch_count <= 64'd0;
        commit_all_mispred_count <= 64'd0;
    end else begin
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

// ============================================================
// jirl_target_cache 模块（普通 JIRL 目标缓存）
// ------------------------------------------------------------
// 直接映射、组合读，使其可参与 BPU P0 预测。目标连续观察两次后才可用，
// 以一次预热为代价降低多态或快速变化的间接目标造成的错误预测。
// RAS 处理 CALL/RET；JTC 只处理 BR_TYPE_UNCOND 且非 direct_b 的普通 JIRL。
// ============================================================
module jirl_target_cache(
    input  wire                       clk,
    input  wire                       reset,
    input  wire                       query_valid_i,
    input  wire [31:2]                query_pc_i,
    output wire                       hit_o,
    output wire [31:0]                target_o,
    output wire [`BLK_LEN_W-1:0]      length_o,
    input  wire                       update_valid_i,
    input  wire [31:2]                update_block_pc_i,
    input  wire [31:0]                update_target_i,
    input  wire [`BLK_LEN_W-1:0]      update_length_i
);

localparam JTC_TAG_W = 32 - 2 - `JIRL_TC_INDEX_W;

reg [`JIRL_TC_SIZE-1:0] valid;
(* ram_style = "distributed" *)
reg [JTC_TAG_W-1:0] tag [0:`JIRL_TC_SIZE-1];
(* ram_style = "distributed" *)
reg [31:0] target [0:`JIRL_TC_SIZE-1];
reg [`BLK_LEN_W-1:0] length [0:`JIRL_TC_SIZE-1];
reg [1:0] confidence [0:`JIRL_TC_SIZE-1];

wire [`JIRL_TC_INDEX_W-1:0] q_idx =
    query_pc_i[2 +: `JIRL_TC_INDEX_W];
wire [JTC_TAG_W-1:0] q_tag =
    query_pc_i[31 -: JTC_TAG_W];
wire q_tag_hit = valid[q_idx] && (tag[q_idx] == q_tag);

assign hit_o = query_valid_i && q_tag_hit && confidence[q_idx][1];
assign target_o = target[q_idx];
assign length_o = length[q_idx];

wire [`JIRL_TC_INDEX_W-1:0] u_idx =
    update_block_pc_i[2 +: `JIRL_TC_INDEX_W];
wire [JTC_TAG_W-1:0] u_tag =
    update_block_pc_i[31 -: JTC_TAG_W];
wire u_tag_hit = valid[u_idx] && (tag[u_idx] == u_tag);
wire u_target_same = u_tag_hit && (target[u_idx] == update_target_i);

always @(posedge clk) begin
    if (reset) begin
        valid <= {`JIRL_TC_SIZE{1'b0}};
    end else if (update_valid_i) begin
        valid[u_idx]  <= 1'b1;
        tag[u_idx]    <= u_tag;
        target[u_idx] <= update_target_i;
        length[u_idx] <= update_length_i;
        if (!u_tag_hit || !u_target_same)
            confidence[u_idx] <= 2'd1;
        else if (confidence[u_idx] != 2'd3)
            confidence[u_idx] <= confidence[u_idx] + 2'd1;
    end
end

`ifdef SYNTHESIS
// synthesis translate_off
// 性能计数：JTC 查询命中、未命中与训练次数。
reg [63:0] jtc_query_hit;
reg [63:0] jtc_query_miss;
reg [63:0] jtc_train_updates;
always @(posedge clk) begin
    if (reset) begin
        jtc_query_hit     <= 64'd0;
        jtc_query_miss    <= 64'd0;
        jtc_train_updates <= 64'd0;
    end else begin
        if (query_valid_i) begin
            if (hit_o)
                jtc_query_hit  <= jtc_query_hit + 64'd1;
            else
                jtc_query_miss <= jtc_query_miss + 64'd1;
        end
        if (update_valid_i)
            jtc_train_updates <= jtc_train_updates + 64'd1;
    end
end
// synthesis translate_on
`endif

endmodule
