// ============================================================
// ftq 模块（Fetch Target Queue，取指目标队列）
// ------------------------------------------------------------
// 当前结构与约束：
// - 三指针环形队列（bpu_ptr 写 / ifu_ptr 取 / cmt_ptr 提交释放）；
// - P0 当拍写 bpu_ptr；P1 次拍覆盖 bpu_ptr-1（含 meta）；
// - "P1 安定"约定：块写入次拍内不发给 IFU（保证覆盖发生在取走前），
//   即 ifu_ptr 指向上一拍刚写入的块时 ifu_valid 压低一拍；
// - 预译码重定向：修正出错块 + bpu_ptr 回退到 id+1（丢弃其后推测块）；
// - 提交：release 数推进 cmt_ptr；分支提交产生训练包（寄存一拍送 BPU，
//   flush 不清在途训练包——它来自已提交的正确信息）。
// - P0 与 P1 同拍时不推进 bpu_ptr，以丢弃错误路径 P0；blk_pc LUTRAM 的
//   写使能仅跟随 p0_valid，不经过 P1 修正长路径。
// ============================================================
`include "mycpu.h"

module ftq(
    input  wire                       clk,
    input  wire                       reset,
    input  wire                       flush_i,             // 全局冲刷：三指针清零、全部作废

    // ---------------- BPU 写入口 ----------------
    input  wire                       p0_valid_i,          // 当拍基础块写入 bpu_ptr
    input  wire [31:0]                p0_pc_i,
    input  wire [`BLK_LEN_W-1:0]      p0_length_i,
    input  wire                       p0_taken_i,
    input  wire [31:0]                p0_target_i,

    input  wire                       p1_valid_i,          // 覆盖 bpu_ptr-1 处的块（主预测修正）
    input  wire                       p1_meta_valid_i,
    input  wire                       p1_desc_valid_i,     // P1 安定描述符有效；不依赖方向是否修正
    input  wire                       p0_retry_pending_i,  // 已寄存：上一拍修正取消了同拍 P0
    input  wire [`BLK_LEN_W-1:0]      p1_length_i,
    input  wire                       p1_taken_i,
    input  wire [31:0]                p1_target_i,
    input  wire [`BPU_META_W-1:0]     p1_meta_i,           // BPU 训练元数据，存入该块

    output wire                       ftq_full_o,          // 满反压 BPU

    // ---------------- IFU 取块口 ----------------
    output wire                       ifu_valid_o,         // 有块可取
    output wire [31:0]                ifu_pc_o,            // 块起始 PC
    output wire [`BLK_LEN_W-1:0]      ifu_length_o,        // 块长
    output wire                       ifu_taken_o,         // 块末预测跳转
    output wire [31:0]                ifu_target_o,        // 预测目标（仅 taken 时有意义）
    output wire [`FTQ_W-1:0]          ifu_ftq_id_o,        // 该块的 FTQ 编号（随指令流入后端）
    input  wire                       ifu_accept_i,        // IFU 收下本块（ifu_ptr++）

    // ---------------- IFU 预译码重定向 ----------------
    input  wire                       predec_redirect_i,    // 预译码发现漏预测的直接跳转
    input  wire                       predec_fixup_only_i,  // 1=仅修正块元数据（cond 截断），不回滚指针
    input  wire [`FTQ_W-1:0]          predec_redirect_id_i, // 出错块的 FTQ 编号
    input  wire [`BLK_LEN_W-1:0]      predec_length_i,      // 修正后的块长（截断到跳转指令）
    input  wire                       predec_taken_i,       // 修正后的方向（恒 1）
    input  wire [31:0]                predec_target_i,      // 修正后的目标

    // ---------------- 提交信息入口（来自 commit）----------------
    input  wire                       cmt_valid_i,          // 本拍有指令提交
    input  wire [`FTQ_W-1:0]          cmt_ftq_id_i,         // 提交指令所在块编号
    input  wire [1:0]                 cmt_release_i,        // 本拍释放的 FTQ 块数（0/1/2，双提交可结束两块）
    input  wire                       cmt_is_branch_i,      // 提交的是分支
    input  wire                       cmt_taken_i,          // 实际方向
    input  wire                       cmt_mispred_i,        // 是否误预测
    input  wire [31:0]                cmt_target_i,         // 实际跳转目标
    input  wire [`BR_TYPE_W-1:0]      cmt_br_type_i,
    input  wire [`BLK_LEN_W+1:2]      cmt_pc_word_i,        // 分支 PC 的块内字偏移

    // ---------------- commit 误预测比对查询口（组合）----------------
    input  wire [`FTQ_W-1:0]          cmt_query_id_i,       // commit 用提交分支的块编号查预测目标
    output wire [31:0]                cmt_blk_target_o,     // 该块当时的预测跳转目标

    // ---------------- 训练包出口（送 BPU）----------------
    output wire                       train_valid_o,
    output wire [31:0]                train_pc_o,           // 块起始 PC（FTB/uBTB 用块 PC 训练）
    output wire                       train_is_branch_o,
    output wire                       train_taken_o,
    output wire                       train_mispred_o,
    output wire [31:0]                train_target_o,
    output wire [`BR_TYPE_W-1:0]      train_br_type_o,
    output wire [`BLK_LEN_W+1:2]      train_fall_through_o, // 顺序出口的块内字偏移
    output wire [`BPU_META_W-1:0]     train_meta_o          // 暂存的 meta 原样回送
);

// ---------------- 存储 ----------------
// 多写口小字段（P0/P1/predec 三写）使用触发器；
// 宽载荷 blk_pc / blk_meta 使用单写口 LUTRAM（见下方专用写块）：
// - blk_pc 只由 P0 写；P1 仅修正上一拍已写块的预测字段，不改变块起始 PC；
// - blk_meta 只由 P1 写；P0 清 meta_set，P1 置 meta_set，读侧在位图为 0 时返回 0。
reg [`BLK_LEN_W-1:0]  blk_len   [0:`FTQ_SIZE-1];
reg                   blk_taken [0:`FTQ_SIZE-1];
reg [31:0]            blk_target[0:`FTQ_SIZE-1];
(* ram_style = "distributed" *) reg [31:0]            blk_pc   [0:`FTQ_SIZE-1];
(* ram_style = "distributed" *) reg [`BPU_META_W-1:0] blk_meta [0:`FTQ_SIZE-1];
reg [`FTQ_SIZE-1:0]   blk_meta_set;   // 该槽 meta 已由 P1 写入（读 0 语义门控）

reg [`FTQ_W-1:0] bpu_ptr, ifu_ptr, cmt_ptr;
reg              p0_wrote_r;     // 上一拍 P0 写入过（"P1 安定"判定）

integer ftq_i;
initial begin
    for (ftq_i = 0; ftq_i < `FTQ_SIZE; ftq_i = ftq_i + 1) begin
        blk_pc[ftq_i]     = 32'b0;
        blk_len[ftq_i]    = 3'd4;
        blk_taken[ftq_i]  = 1'b0;
        blk_target[ftq_i] = 32'b0;
        blk_meta[ftq_i]   = {`BPU_META_W{1'b0}};
    end
    blk_meta_set = {`FTQ_SIZE{1'b0}};
    bpu_ptr    = {`FTQ_W{1'b0}};
    ifu_ptr    = {`FTQ_W{1'b0}};
    cmt_ptr    = {`FTQ_W{1'b0}};
    p0_wrote_r = 1'b0;
end

wire [`FTQ_W-1:0] bpu_prev = bpu_ptr - {{(`FTQ_W-1){1'b0}}, 1'b1};

// ---------------- 满/空 ----------------
// almost-full（预留 2 槽）：配合 BPU 的两拍冻结——
// full 首拍 P0 仍可写最后 1 个可用槽，连续两拍满才真正冻结，
// 避免满边界抖动吞掉 P1 覆盖/浪费取指拍
assign ftq_full_o = ((bpu_ptr + {{(`FTQ_W-1){1'b0}}, 1'b1}) == cmt_ptr)
                 || ((bpu_ptr + {{(`FTQ_W-2){1'b0}}, 2'd2}) == cmt_ptr);

// ---------------- IFU 取块口 ----------------
// The P1 response for the newest P0 arrives in this cycle.  Release the head
// immediately when that response is valid; corrected fields bypass the array.
//
// Deliberately do not gate this control with predec_redirect_i.  IFU already
// rejects a new block while predecode redirects, and keeping redirect out of
// this expression cuts the IFU-predecode -> FTQ-valid -> MMU/TLB feedback cone.
wire head_is_newest = (ifu_ptr == bpu_prev);
wire head_retry_blocked = head_is_newest && p0_retry_pending_i;
wire head_blk_settling_raw = head_is_newest && p0_wrote_r &&
                             !p0_retry_pending_i;
// p0_wrote_r is the local, registered P1 schedule marker.  Do not use
// p1_meta_valid_i to control ifu_valid_o: that signal is suppressed by an IFU
// predecode redirect and would recreate the same cross-module feedback path.
// Redirect/flush already blocks acceptance.  The complete settled descriptor
// is selected below from the fixed P1 schedule, not from p1_valid_i.
wire head_p1_bypass = head_blk_settling_raw;
wire head_p1_correction_bypass = head_p1_bypass && p1_valid_i;
wire head_blk_settling = head_blk_settling_raw && !head_p1_bypass;
assign ifu_valid_o  = (ifu_ptr != bpu_ptr) && !head_blk_settling &&
                      !head_retry_blocked;
assign ifu_pc_o     = blk_pc[ifu_ptr];
// The newest block is released on its fixed P1 settle cycle.  BPU supplies a
// complete settled descriptor on both FTB hit and miss, so this bypass is no
// longer limited to corrections.
assign ifu_length_o = head_p1_bypass ? p1_length_i : blk_len[ifu_ptr];
assign ifu_taken_o  = head_p1_bypass ? p1_taken_i  : blk_taken[ifu_ptr];
assign ifu_target_o = head_p1_bypass ? p1_target_i : blk_target[ifu_ptr];
assign ifu_ftq_id_o = ifu_ptr;

// ---------------- commit 查询口 ----------------
assign cmt_blk_target_o = blk_target[cmt_query_id_i];

// cmt_ptr 只随提交释放推进，predec 重定向不改变它。重定向块 R 的指令
// 本拍才进入 IB；R 之后被 squash 的推测块尚未提交，也不会再到达提交侧。
wire [`FTQ_W-1:0] cmt_adv = (|cmt_release_i)
                          ? {{(`FTQ_W-2){1'b0}}, cmt_release_i}
                          : {`FTQ_W{1'b0}};

// ---------------- LUTRAM 载荷写口（单写口，与原写入同拍同条件）----------------
wire ftq_run      = !reset && !flush_i;
wire blk_pc_wr    = ftq_run && !predec_redirect_i && p0_valid_i;   // 块 PC 只随 P0 写入
wire blk_meta_wr  = ftq_run && p1_meta_valid_i;
wire [`FTQ_W-1:0] blk_pc_waddr =
    p0_retry_pending_i ? bpu_prev : bpu_ptr;
// Descriptor storage has one normal write point: the fixed P1 settle cycle.
// p1_desc_valid_i depends only on the registered response schedule, not on the
// TAGE direction decision.  Predecode redirects are mutually exclusive.
wire blk_p1_desc_wr = ftq_run && !predec_redirect_i && p1_desc_valid_i;
wire blk_predec_desc_wr = ftq_run && predec_redirect_i;

always @(posedge clk) begin
    if (blk_pc_wr)
        blk_pc[blk_pc_waddr] <= p0_pc_i;
end
always @(posedge clk) begin
    if (blk_meta_wr)
        blk_meta[bpu_prev] <= p1_meta_i;
end

// Prediction descriptor storage no longer has a P0/P1 dual-write mux.  In
// particular, p1_valid_i cannot select the D input of all 32 target bits.
always @(posedge clk) begin
    if (blk_predec_desc_wr) begin
        blk_len[predec_redirect_id_i]    <= predec_length_i;
        blk_taken[predec_redirect_id_i]  <= predec_taken_i;
        blk_target[predec_redirect_id_i] <= predec_target_i;
    end else if (blk_p1_desc_wr) begin
        blk_len[bpu_prev]    <= p1_length_i;
        blk_taken[bpu_prev]  <= p1_taken_i;
        blk_target[bpu_prev] <= p1_target_i;
    end
end

// meta 有效位图：P0 清（对应原"写 0 清空"），P1 写置位
always @(posedge clk) begin
    if (reset)
        blk_meta_set <= {`FTQ_SIZE{1'b0}};
    else begin
        if (blk_pc_wr)   blk_meta_set[blk_pc_waddr] <= 1'b0;
        if (blk_meta_wr) blk_meta_set[bpu_prev] <= 1'b1;
    end
end

// meta 读口（位图为 0 给全 0，等价原"P0 清零后未被 P1 覆盖"取值）
wire [`BPU_META_W-1:0] blk_meta_cmt_rd =
    blk_meta_set[cmt_ftq_id_i] ? blk_meta[cmt_ftq_id_i] : {`BPU_META_W{1'b0}};

// ---------------- 指针与块写入 ----------------
always @(posedge clk) begin
    if (reset) begin
        bpu_ptr    <= {`FTQ_W{1'b0}};
        ifu_ptr    <= {`FTQ_W{1'b0}};
        cmt_ptr    <= {`FTQ_W{1'b0}};
        p0_wrote_r <= 1'b0;
    end else if (flush_i) begin
        bpu_ptr    <= {`FTQ_W{1'b0}};
        ifu_ptr    <= {`FTQ_W{1'b0}};
        cmt_ptr    <= {`FTQ_W{1'b0}};
        p0_wrote_r <= 1'b0;
    end else begin
        // predec 优先于 P0：同拍禁止写入 fall-through 块污染 ifu_ptr 指向的槽
        if (predec_redirect_i) begin
            if (!predec_fixup_only_i) begin
                bpu_ptr    <= predec_redirect_id_i + 1'b1;
                // 丢弃 redirect 之后的推测块。IFU 两级流水最多领先 redirect 块
                // 2 个（错误路径块已被 IFU predec_kill 丢弃），且 ifu_ptr 环序上
                // 恒不落后于 redirect_id+1，故无条件回卷（原 '>' 比较回绕处漏判）
                ifu_ptr <= predec_redirect_id_i + 1'b1;
            end
            p0_wrote_r <= 1'b0;
        end else begin
            // Pointer allocation never depends on combinational p1_valid_i.
            // A correction-cycle P0 reserves the tail normally; on the next
            // accepted P0, registered retry state overwrites that slot and
            // suppresses a second increment.
            if (p0_valid_i) begin
                if (!p0_retry_pending_i)
                    bpu_ptr <= bpu_ptr + 1'b1;
                p0_wrote_r <= 1'b1;
            end else begin
                p0_wrote_r <= 1'b0;
            end
        end
        // IFU 取走
        if (ifu_accept_i && ifu_valid_o)
            ifu_ptr <= ifu_ptr + 1'b1;

        // 提交释放（与 predec 独立，避免 cond 截断同拍吞掉 release）
        if (|cmt_adv)
            cmt_ptr <= cmt_ptr + cmt_adv;
    end
end

// ---------------- 训练包（寄存一拍；flush 不清——已提交信息恒正确）----------------
reg                   train_valid_r;
reg [31:0]            train_pc_r;
reg                   train_is_branch_r, train_taken_r, train_mispred_r;
reg [31:0]            train_target_r;
reg [`BR_TYPE_W-1:0]  train_btype_r;
reg [`BLK_LEN_W+1:2]  train_ft_r;
reg [`BPU_META_W-1:0] train_meta_r;

always @(posedge clk) begin
    if (reset) begin
        train_valid_r <= 1'b0;
    end else begin
        train_valid_r     <= cmt_valid_i && cmt_is_branch_i;
        train_pc_r        <= blk_pc[cmt_ftq_id_i];
        train_is_branch_r <= cmt_is_branch_i;
        train_taken_r     <= cmt_taken_i;
        train_mispred_r   <= cmt_mispred_i;
        train_target_r    <= cmt_target_i;
        train_btype_r     <= cmt_br_type_i;
        train_ft_r        <= cmt_pc_word_i + 1'b1;
        train_meta_r      <= blk_meta_cmt_rd;
    end
end

assign train_valid_o        = train_valid_r;
assign train_pc_o           = train_pc_r;
assign train_is_branch_o    = train_is_branch_r;
assign train_taken_o        = train_taken_r;
assign train_mispred_o      = train_mispred_r;
assign train_target_o       = train_target_r;
assign train_br_type_o      = train_btype_r;
assign train_fall_through_o = train_ft_r;
assign train_meta_o         = train_meta_r;

`ifdef SYNTHESIS
// synthesis translate_off
reg [63:0] p1_meta_saved_count;
reg [63:0] p1_correction_count;
reg [63:0] commit_cond_branch_count;
reg [63:0] commit_cond_mispred_count;
reg [63:0] commit_cond_meta_valid_count;
reg [63:0] commit_cond_meta_invalid_count;
reg [63:0] p1_head_bypass_offer_count;
reg [63:0] p1_head_bypass_accept_count;
reg [63:0] p1_head_correction_bypass_count;

localparam META_TAGE_VALID_BIT = 38;

always @(posedge clk) begin
    if (reset) begin
        p1_meta_saved_count       <= 64'd0;
        p1_correction_count       <= 64'd0;
        commit_cond_branch_count  <= 64'd0;
        commit_cond_mispred_count <= 64'd0;
        commit_cond_meta_valid_count <= 64'd0;
        commit_cond_meta_invalid_count <= 64'd0;
        p1_head_bypass_offer_count <= 64'd0;
        p1_head_bypass_accept_count <= 64'd0;
        p1_head_correction_bypass_count <= 64'd0;
    end else begin
        if (p1_meta_valid_i)
            p1_meta_saved_count <= p1_meta_saved_count + 64'd1;
        if (p1_valid_i)
            p1_correction_count <= p1_correction_count + 64'd1;
        if (cmt_valid_i && cmt_is_branch_i && (cmt_br_type_i == `BR_TYPE_COND)) begin
            commit_cond_branch_count <= commit_cond_branch_count + 64'd1;
            if (blk_meta_cmt_rd[META_TAGE_VALID_BIT])
                commit_cond_meta_valid_count <= commit_cond_meta_valid_count + 64'd1;
            else
                commit_cond_meta_invalid_count <= commit_cond_meta_invalid_count + 64'd1;
        end
        if (cmt_valid_i && cmt_is_branch_i && (cmt_br_type_i == `BR_TYPE_COND) && cmt_mispred_i)
            commit_cond_mispred_count <= commit_cond_mispred_count + 64'd1;
        if (head_p1_bypass)
            p1_head_bypass_offer_count <= p1_head_bypass_offer_count + 64'd1;
        if (head_p1_bypass && ifu_accept_i && ifu_valid_o)
            p1_head_bypass_accept_count <= p1_head_bypass_accept_count + 64'd1;
        if (head_p1_correction_bypass && ifu_accept_i && ifu_valid_o)
            p1_head_correction_bypass_count <=
                p1_head_correction_bypass_count + 64'd1;
    end
end
// synthesis translate_on
`endif

endmodule
