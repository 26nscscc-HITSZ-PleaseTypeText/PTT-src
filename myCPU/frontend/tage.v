// ============================================================
// tage 模块（TAGE 条件分支方向预测器）
// ------------------------------------------------------------
// 功能/结构：
// - 基础表 8192×2bit（bimodal）+ 4 个标记表 1024×{tag12,ctr3,u2}；
//   全部推断 BRAM（真双读口 2R+1W：查询口 q_* 与训练口 t_* 独立），
//   查询仍 1 拍延迟返回，训练读表可与查询同拍并行；
// - GHR 单份、训练到达时移入实际方向（不做检查点回滚——训练走队列，
//   入队时快照当拍 GHR，出队训练用快照重算索引，与查询解耦）；
// - 索引/标签用历史折叠（GHR 按各表历史长度 fold 到 10/12 位再与 PC 异或）；
// - meta 打包（低位起）：
//   {prov_useful(2), prov_ctr(3), prov_idx(10), prov_id(2), prov_valid(1),
//    alt_taken(1), base_ctr(2), base_idx(13), hits(4)}，另加 tage_valid@38、
//   prov_tag(12)@43（44b 有效装入 64b `BPU_META_W；训练按位解包定位 provider）；
// - 训练走「更新 FIFO + 3 级小流水」：满则丢弃计 overflow；因 2R 不占查询读口，
//   FIFO 非空即可每拍出队读表；随后 provider 原地更新 ctr/useful；误预测时向
//   更长历史表分配新项，无位可分则把更长历史表的 useful 清 0（腾位）；
// - 写旁路：BRAM 同址同拍写转发进读寄存器；查询响应另对 T2 写口做组合旁路。
// ============================================================
`include "mycpu.h"

module tage(
    input  wire                       clk,
    input  wire                       reset,
    input  wire                       flush_i,

    // ---------------- 查询口（1 拍延迟返回）----------------
    input  wire                       query_valid_i,
    input  wire [21:2]                query_pc_i,         // 预测索引使用的 PC 位

    output wire                       taken_o,            // 方向预测（晚查询 1 拍）
    output wire                       resp_valid_o,
    output wire [`BPU_META_W-1:0]     meta_o,             // 训练回带信息

    // ---------------- 训练口（提交级 BPU 训练，经内部 FIFO 缓冲）----------------
    input  wire                       train_valid_i,
    input  wire [21:2]                train_pc_i,
    input  wire                       train_taken_i,      // 实际方向
    input  wire                       train_mispred_i,    // 该分支发生误预测
    input  wire [`BPU_META_W-1:0]     train_meta_i,       // 查询时的 meta 原样回传

    // 按 FTQ 编号保存的推测历史检查点
    input  wire                       hist_checkpoint_save_i,
    input  wire [`FTQ_W-1:0]          hist_checkpoint_id_i,
    input  wire                       hist_restore_i,
    input  wire [`FTQ_W-1:0]          hist_restore_id_i,
    input  wire                       hist_restore_append_i,
    input  wire                       hist_restore_taken_i,
    input  wire                       hist_update_valid_i,
    input  wire                       hist_update_taken_i,

    // 后端冲刷时用于恢复的精确提交历史
    input  wire                       cmt_hist_valid_i,
    input  wire                       cmt_hist_taken_i
);

localparam BASE_IDXW = 13;          // 8192
localparam TIDXW     = 10;          // 1024
localparam TENTRY_W  = 1 + `TAGE_TAG_W + 3 + 2;   // {valid, tag, ctr, useful} = 18
localparam META_RAW_W              = 38;
localparam META_HITS_LSB           = 0;
localparam META_BASE_IDX_LSB       = 4;
localparam META_BASE_CTR_LSB       = 17;
localparam META_ALT_TAKEN_BIT      = 19;
localparam META_PROVIDER_VALID_BIT = 20;
localparam META_PROVIDER_ID_LSB    = 21;
localparam META_PROVIDER_IDX_LSB   = 23;
localparam META_PROVIDER_CTR_LSB   = 33;
localparam META_PROVIDER_U_LSB     = 36;
localparam META_TAGE_VALID_BIT     = 38;
localparam META_PROVIDER_TAG_LSB   = 43;
localparam META_PROVIDER_TAG_W     = `TAGE_TAG_W;
localparam META_FTQ_ID_LSB         = 55;
localparam TAGE_UPDATE_Q_DEPTH = `TAGE_UPDATE_Q_DEPTH;
localparam TAGE_UPDATE_Q_PTR_W =
    (TAGE_UPDATE_Q_DEPTH <= 1) ? 1 : $clog2(TAGE_UPDATE_Q_DEPTH);
localparam TAGE_UPDATE_Q_CNT_W = $clog2(TAGE_UPDATE_Q_DEPTH + 1);
localparam [TAGE_UPDATE_Q_CNT_W-1:0] TAGE_UPDATE_Q_DEPTH_C = TAGE_UPDATE_Q_DEPTH;

// ---------------- 推测 / 提交 GHR ----------------
reg [`GHR_LEN-1:0] spec_ghr;
reg [`GHR_LEN-1:0] commit_ghr;
(* ram_style = "distributed" *) reg [`GHR_LEN-1:0]
    hist_checkpoint [0:`FTQ_SIZE-1];

function [`GHR_LEN-1:0] hist_shift;
    input [`GHR_LEN-1:0] h;
    input taken;
    begin
        hist_shift = (h << 1) | {{(`GHR_LEN-1){1'b0}}, taken};
    end
endfunction

// P1 结算当前块与下一块查询同拍发生；将本拍预测结果前递到历史折叠网络。
wire [`GHR_LEN-1:0] query_ghr =
    hist_update_valid_i ? hist_shift(spec_ghr, hist_update_taken_i) : spec_ghr;

// ---------------- 历史折叠函数 ----------------
function [TIDXW-1:0] fold10;
    input [`GHR_LEN-1:0] h;
    input integer len;
    integer i;
    begin
        fold10 = {TIDXW{1'b0}};
        for (i = 0; i < len; i = i + 1)
            fold10[i % TIDXW] = fold10[i % TIDXW] ^ h[i];
    end
endfunction

function [`TAGE_TAG_W-1:0] fold12;
    input [`GHR_LEN-1:0] h;
    input integer len;
    integer i;
    begin
        fold12 = {`TAGE_TAG_W{1'b0}};
        for (i = 0; i < len; i = i + 1)
            fold12[i % `TAGE_TAG_W] = fold12[i % `TAGE_TAG_W] ^ h[i];
    end
endfunction

// 索引/标签生成：查询用 query_pc + 当前 ghr；训练用入队快照的 ghr（_h 版本）
function [TIDXW-1:0] tidx;
    input [21:2] pc;
    input integer t;
    begin
        tidx = fold10(query_ghr, (t == 0) ? `TAGE_HIST_LEN0 :
                           (t == 1) ? `TAGE_HIST_LEN1 :
                           (t == 2) ? `TAGE_HIST_LEN2 : `TAGE_HIST_LEN3)
             ^ pc[2 +: TIDXW] ^ pc[12 +: TIDXW];
    end
endfunction

function [TIDXW-1:0] tidx_h;
    input [21:2] pc;
    input integer t;
    input [`GHR_LEN-1:0] h;
    begin
        tidx_h = fold10(h, (t == 0) ? `TAGE_HIST_LEN0 :
                           (t == 1) ? `TAGE_HIST_LEN1 :
                           (t == 2) ? `TAGE_HIST_LEN2 : `TAGE_HIST_LEN3)
               ^ pc[2 +: TIDXW] ^ pc[12 +: TIDXW];
    end
endfunction

function [`TAGE_TAG_W-1:0] ttag;
    input [13:2] pc;
    input integer t;
    begin
        ttag = fold12(query_ghr, (t == 0) ? `TAGE_HIST_LEN0 :
                           (t == 1) ? `TAGE_HIST_LEN1 :
                           (t == 2) ? `TAGE_HIST_LEN2 : `TAGE_HIST_LEN3)
             ^ pc[2 +: `TAGE_TAG_W];
    end
endfunction

function [`TAGE_TAG_W-1:0] ttag_h;
    input [13:2] pc;
    input integer t;
    input [`GHR_LEN-1:0] h;
    begin
        ttag_h = fold12(h, (t == 0) ? `TAGE_HIST_LEN0 :
                           (t == 1) ? `TAGE_HIST_LEN1 :
                           (t == 2) ? `TAGE_HIST_LEN2 : `TAGE_HIST_LEN3)
               ^ pc[2 +: `TAGE_TAG_W];
    end
endfunction

// ---------------- 训练流水线寄存器（T0 读表 / T1 / T2 计算 / T3 写回）----------------
reg        t0_valid;
reg [21:2] t0_pc;
reg        t0_taken, t0_mispred;
reg [`GHR_LEN-1:0] t0_ghr;
reg [`BPU_META_W-1:0] t0_meta;
reg        t1_valid;
reg        t1_taken, t1_mispred;
reg [`BPU_META_W-1:0] t1_meta;
reg [TIDXW-1:0]      t1_alloc_idx [0:3];
reg [`TAGE_TAG_W-1:0] t1_alloc_tag [0:3];
reg [TENTRY_W-1:0]   t1_rd_entry [0:3];
reg        t2_valid;
reg        t2_taken, t2_mispred;
reg [`BPU_META_W-1:0] t2_meta;
reg [TIDXW-1:0]      t2_alloc_idx [0:3];
reg [`TAGE_TAG_W-1:0] t2_alloc_tag [0:3];
reg [TENTRY_W-1:0]   t2_rd_entry [0:3];

reg [21:2]               uq_pc      [0:TAGE_UPDATE_Q_DEPTH-1];
reg                      uq_taken   [0:TAGE_UPDATE_Q_DEPTH-1];
reg                      uq_mispred [0:TAGE_UPDATE_Q_DEPTH-1];
reg [`BPU_META_W-1:0]    uq_meta    [0:TAGE_UPDATE_Q_DEPTH-1];
reg [`GHR_LEN-1:0]       uq_ghr     [0:TAGE_UPDATE_Q_DEPTH-1];
reg [TAGE_UPDATE_Q_PTR_W-1:0] uq_rptr, uq_wptr;
reg [TAGE_UPDATE_Q_CNT_W-1:0] uq_count;

wire tage_update_queue_empty = (uq_count == {TAGE_UPDATE_Q_CNT_W{1'b0}});
wire tage_update_queue_full  = (uq_count == TAGE_UPDATE_Q_DEPTH_C);
// 2R+1W：训练读口独立，FIFO 非空即可出队，不再等查询空闲拍
wire train_read_grant        = !tage_update_queue_empty;
wire tage_update_enqueue     = train_valid_i && (!tage_update_queue_full || train_read_grant);
wire tage_update_overflow    = train_valid_i && tage_update_queue_full && !train_read_grant;
wire tage_update_dequeue     = train_read_grant;
wire [TAGE_UPDATE_Q_CNT_W-1:0] uq_count_next =
    (tage_update_enqueue && !tage_update_dequeue) ? (uq_count + {{(TAGE_UPDATE_Q_CNT_W-1){1'b0}},1'b1}) :
    (!tage_update_enqueue && tage_update_dequeue) ? (uq_count - {{(TAGE_UPDATE_Q_CNT_W-1){1'b0}},1'b1}) :
                                                    uq_count;
wire [63:0] uq_count_next_64 = {{(64-TAGE_UPDATE_Q_CNT_W){1'b0}}, uq_count_next};

wire [21:2]            uq_head_pc      = uq_pc[uq_rptr];
wire                   uq_head_taken   = uq_taken[uq_rptr];
wire                   uq_head_mispred = uq_mispred[uq_rptr];
wire [`BPU_META_W-1:0] uq_head_meta    = uq_meta[uq_rptr];
wire [`GHR_LEN-1:0]    uq_head_ghr     = uq_ghr[uq_rptr];
wire                   uq_head_meta_valid = uq_head_meta[META_TAGE_VALID_BIT];
wire                   uq_head_pvalid  = uq_head_meta[META_PROVIDER_VALID_BIT];
wire [1:0]             uq_head_pid     = uq_head_meta[META_PROVIDER_ID_LSB +: 2];
wire [TIDXW-1:0]       uq_head_pidx    = uq_head_meta[META_PROVIDER_IDX_LSB +: TIDXW];
wire [`FTQ_W-1:0]      train_ftq_id    = train_meta_i[META_FTQ_ID_LSB +: `FTQ_W];

`ifdef SYNTHESIS
// synthesis translate_off
initial begin
    if (TAGE_UPDATE_Q_DEPTH < 2)
        $fatal(1, "TAGE_UPDATE_Q_DEPTH must be >= 2");
    if ((TAGE_UPDATE_Q_DEPTH & (TAGE_UPDATE_Q_DEPTH - 1)) != 0)
        $fatal(1, "TAGE_UPDATE_Q_DEPTH must be a power of two");
end
// synthesis translate_on
`endif

// ---------------- 基础表（bimodal，2R+1W）----------------
wire [BASE_IDXW-1:0] q_base_idx = query_pc_i[2 +: BASE_IDXW];
wire [1:0] base_q_rdata;
reg        base_we_calc;
reg [BASE_IDXW-1:0] base_waddr_calc;
reg [1:0]  base_wdata_calc;
reg        t3_base_we;
reg [BASE_IDXW-1:0] t3_base_waddr;
reg [1:0]  t3_base_wdata;

tage_base_ram u_base(
    .clk(clk),
    .q_raddr(q_base_idx), .q_rdata(base_q_rdata),
    .we(t3_base_we), .waddr(t3_base_waddr), .wdata(t3_base_wdata)
);

// ---------------- 4 个标记表（2R+1W）----------------
wire [TIDXW-1:0] q_idx [0:3];
wire [`TAGE_TAG_W-1:0] q_tag [0:3];
wire [TENTRY_W-1:0] t_q_rdata [0:3];
wire [TENTRY_W-1:0] t_train_rdata [0:3];
reg  [3:0] t_we_calc;
reg  [TIDXW-1:0] t_waddr_calc [0:3];
reg  [TENTRY_W-1:0] t_wdata_calc [0:3];
// The RAM write port is driven only by these registers.  This prevents the
// T2 allocation/provider network from directly reaching the RAM bypass
// registers and BRAM write controls in the same cycle.
reg  [3:0] t3_t_we;
reg  [TIDXW-1:0] t3_t_waddr [0:3];
reg  [TENTRY_W-1:0] t3_t_wdata [0:3];
// A just-committed command is retained for one more cycle.  A training read
// may have been issued before that write reached the duplicated RAMs, so T2
// needs both the pending and the just-committed commands for correct RAW
// forwarding.
reg  [3:0] t3_last_t_we;
reg  [TIDXW-1:0] t3_last_t_waddr [0:3];
reg  [TENTRY_W-1:0] t3_last_t_wdata [0:3];

genvar g;
generate
for (g = 0; g < 4; g = g + 1) begin : gen_ttab
    localparam [1:0] TABLE_ID = g[1:0];
    wire provider_read_sel = uq_head_meta_valid && uq_head_pvalid && (uq_head_pid == TABLE_ID);
    wire [TIDXW-1:0] train_raddr = provider_read_sel ? uq_head_pidx : tidx_h(uq_head_pc, g, uq_head_ghr);
    assign q_idx[g] = tidx(query_pc_i, g);
    assign q_tag[g] = ttag(query_pc_i[13:2], g);
    tage_tag_ram u_ttab(
        .clk(clk),
        .q_raddr(q_idx[g]),
        .q_rdata(t_q_rdata[g]),
        .t_raddr(train_raddr),
        .t_rdata(t_train_rdata[g]),
        .we(t3_t_we[g]),
        .waddr(t3_t_waddr[g]),
        .wdata(t3_t_wdata[g])
    );
end
endgenerate

// ---------------- 查询响应（相对查询晚 1 拍）----------------
reg        q_valid_r;
reg [`TAGE_TAG_W-1:0] q_tag_r [0:3];
reg [TIDXW-1:0]       q_idx_r [0:3];
reg [BASE_IDXW-1:0]   q_bidx_r;
reg                   q_base_fwd_valid_r;
reg [1:0]             q_base_fwd_data_r;
integer qk;
always @(posedge clk) begin
    if (reset) begin
        q_valid_r             <= 1'b0;
        q_base_fwd_valid_r    <= 1'b0;
    end else begin
        q_valid_r <= query_valid_i;
        for (qk = 0; qk < 4; qk = qk + 1) begin
            q_tag_r[qk] <= q_tag[qk];
            q_idx_r[qk] <= q_idx[qk];
        end
        q_bidx_r <= q_base_idx;
        q_base_fwd_valid_r <=
            t3_base_we && (t3_base_waddr == q_base_idx);
        q_base_fwd_data_r <= t3_base_wdata;
    end
end

// The query port intentionally uses the BRAM result directly.  A same-edge
// training write may therefore be visible one query later; prediction remains
// recoverable, while removing the TAGE-result -> next-index -> collision loop.
wire [3:0] thit;
wire [TENTRY_W-1:0] t_q_entry_eff [0:3];
generate
for (g = 0; g < 4; g = g + 1) begin : gen_thit
    assign t_q_entry_eff[g] = t_q_rdata[g];
    assign thit[g] = q_valid_r
                    && t_q_entry_eff[g][TENTRY_W-1]
                    && (t_q_entry_eff[g][TENTRY_W-2 -: `TAGE_TAG_W] == q_tag_r[g]);
end
endgenerate

// provider = 命中的最长历史表（表号大者优先）
wire       prov_valid = |thit;
wire [1:0] prov_id    = thit[3] ? 2'd3 : thit[2] ? 2'd2 : thit[1] ? 2'd1 : 2'd0;
wire [2:0] prov_ctr   = t_q_entry_eff[prov_id][4:2];
wire [1:0] prov_u     = t_q_entry_eff[prov_id][1:0];
wire [`TAGE_TAG_W-1:0] prov_tag = t_q_entry_eff[prov_id][TENTRY_W-2 -: `TAGE_TAG_W];
wire [1:0] base_q_eff =
    q_base_fwd_valid_r ? q_base_fwd_data_r : base_q_rdata;
wire       base_taken = base_q_eff[1];

// The alternate provider is the second-longest matching tagged table, not
// unconditionally the bimodal base table.
reg        alt_valid;
reg [1:0]  alt_id;
reg        alt_taken;
always @(*) begin
    alt_valid = 1'b0;
    alt_id    = 2'd0;
    alt_taken = base_taken;
    case (prov_id)
        2'd3: begin
            if (thit[2]) begin
                alt_valid = 1'b1; alt_id = 2'd2;
                alt_taken = t_q_entry_eff[2][4];
            end else if (thit[1]) begin
                alt_valid = 1'b1; alt_id = 2'd1;
                alt_taken = t_q_entry_eff[1][4];
            end else if (thit[0]) begin
                alt_valid = 1'b1; alt_id = 2'd0;
                alt_taken = t_q_entry_eff[0][4];
            end
        end
        2'd2: begin
            if (thit[1]) begin
                alt_valid = 1'b1; alt_id = 2'd1;
                alt_taken = t_q_entry_eff[1][4];
            end else if (thit[0]) begin
                alt_valid = 1'b1; alt_id = 2'd0;
                alt_taken = t_q_entry_eff[0][4];
            end
        end
        2'd1: begin
            if (thit[0]) begin
                alt_valid = 1'b1; alt_id = 2'd0;
                alt_taken = t_q_entry_eff[0][4];
            end
        end
        default: begin
            alt_valid = 1'b0;
            alt_id    = 2'd0;
            alt_taken = base_taken;
        end
    endcase
end

// Weak, newly allocated providers may defer to the alternate.  The global
// counter changes only when the two predictions disagree.
reg [3:0] use_alt_on_na;
wire prov_weak = (prov_ctr == 3'd3) || (prov_ctr == 3'd4);
wire use_alt_prediction = prov_valid && prov_weak && (prov_u == 2'b00) &&
                          (use_alt_on_na >= 4'd2);
wire tage_selected_taken = use_alt_prediction ? alt_taken : prov_ctr[2];
wire tage_final_taken = prov_valid ? tage_selected_taken : base_taken;

assign resp_valid_o = q_valid_r;
assign taken_o = q_valid_r && tage_final_taken;

// meta 打包：{prov_useful(2), prov_ctr(3), prov_idx(10), prov_id(2), prov_valid(1),
//             alt_taken(1), base_ctr(2), base_idx(13), hits(4)} 共 38 位，
wire [`BPU_META_W-1:0] meta_raw =
    { {(`BPU_META_W-META_RAW_W){1'b0}},
      prov_u, prov_ctr, q_idx_r[prov_id], prov_id, prov_valid,
      alt_taken, base_q_eff, q_bidx_r, thit };
wire [`BPU_META_W-1:0] meta_provider_tag =
    { {(`BPU_META_W-META_PROVIDER_TAG_W){1'b0}}, prov_tag } << META_PROVIDER_TAG_LSB;
assign meta_o = q_valid_r ? (meta_raw | meta_provider_tag)
                          : {`BPU_META_W{1'b0}};

// meta 解包（训练 T2 拍使用）
wire [BASE_IDXW-1:0]  m_base_idx = t2_meta[META_BASE_IDX_LSB +: BASE_IDXW];
wire [1:0]            m_base_ctr = t2_meta[META_BASE_CTR_LSB +: 2];
wire                  m_alt      = t2_meta[META_ALT_TAKEN_BIT];
wire                  m_pvalid   = t2_meta[META_PROVIDER_VALID_BIT];
wire [1:0]            m_pid      = t2_meta[META_PROVIDER_ID_LSB +: 2];
wire [TIDXW-1:0]      m_pidx     = t2_meta[META_PROVIDER_IDX_LSB +: TIDXW];
wire [2:0]            m_pctr     = t2_meta[META_PROVIDER_CTR_LSB +: 3];
wire [1:0]            m_pu       = t2_meta[META_PROVIDER_U_LSB +: 2];
wire [`TAGE_TAG_W-1:0] m_ptag    = t2_meta[META_PROVIDER_TAG_LSB +: META_PROVIDER_TAG_W];

// ---------------- 训练写回 ----------------
// T2 table data can predate one or two registered write commands.  Select
// the newest matching command before provider validation and allocation.
wire [TIDXW-1:0] t2_read_idx [0:3];
wire [TENTRY_W-1:0] t2_rd_entry_eff [0:3];
generate
for (g = 0; g < 4; g = g + 1) begin : gen_t2_raw_forward
    localparam [1:0] TABLE_ID = g[1:0];
    assign t2_read_idx[g] =
        (m_pvalid && (m_pid == TABLE_ID)) ? m_pidx : t2_alloc_idx[g];
    assign t2_rd_entry_eff[g] =
        (t3_t_we[g] && (t3_t_waddr[g] == t2_read_idx[g]))
        ? t3_t_wdata[g]
        : (t3_last_t_we[g] && (t3_last_t_waddr[g] == t2_read_idx[g]))
          ? t3_last_t_wdata[g]
          : t2_rd_entry[g];
end
endgenerate

// 饱和计数器
function [1:0] sat2;
    input [1:0] c;
    input taken;
    begin
        sat2 = taken ? ((c == 2'd3) ? 2'd3 : c + 2'd1)
                     : ((c == 2'd0) ? 2'd0 : c - 2'd1);
    end
endfunction
function [2:0] sat3;
    input [2:0] c;
    input taken;
    begin
        sat3 = taken ? ((c == 3'd7) ? 3'd7 : c + 3'd1)
                     : ((c == 3'd0) ? 3'd0 : c - 3'd1);
    end
endfunction

// 分配候选 = 比 provider 历史更长的表中 空项或 useful==0 的项
wire [3:0] alloc_cand;
generate
for (g = 0; g < 4; g = g + 1) begin : gen_alloc
    if (g == 0) begin : gen_first
        assign alloc_cand[g] = (!t2_rd_entry_eff[g][TENTRY_W-1]
                             || (t2_rd_entry_eff[g][1:0] == 2'b00))
                            && !m_pvalid;
    end else begin : gen_longer
        assign alloc_cand[g] = (!t2_rd_entry_eff[g][TENTRY_W-1]
                             || (t2_rd_entry_eff[g][1:0] == 2'b00))
                            && (!m_pvalid || (m_pid < g[1:0]));
    end
end
endgenerate
wire alloc_any = |alloc_cand;
wire [1:0] alloc_sel = alloc_cand[0] ? 2'd0 : alloc_cand[1] ? 2'd1 :
                       alloc_cand[2] ? 2'd2 : 2'd3;

// provider 预测与 alt 不同时才动 useful：预测对 -> useful++，错 -> useful--
wire t1_prov_pred  = m_pctr[2];
wire t1_prov_corr  = (t1_prov_pred == t2_taken);
wire t1_useful_chg = m_pvalid && (t1_prov_pred != m_alt);
wire [1:0] t1_u_new = !t1_useful_chg ? m_pu : sat2(m_pu, t1_prov_corr);
wire train_prov_weak = (m_pctr == 3'd3) || (m_pctr == 3'd4);
wire train_alt_better = m_pvalid && train_prov_weak &&
                        (t1_prov_pred != m_alt) &&
                        (m_alt == t2_taken);
wire train_prov_better = m_pvalid && train_prov_weak &&
                         (t1_prov_pred != m_alt) &&
                         (t1_prov_pred == t2_taken);
wire provider_entry_valid = t2_rd_entry_eff[m_pid][TENTRY_W-1];
wire provider_tag_match = provider_entry_valid &&
                          (t2_rd_entry_eff[m_pid][TENTRY_W-2 -: `TAGE_TAG_W] == m_ptag);

integer tk;
always @(*) begin
    base_we_calc    = 1'b0;
    base_waddr_calc = m_base_idx;
    base_wdata_calc = sat2(m_base_ctr, t2_taken);
    t_we_calc       = 4'b0;
    for (tk = 0; tk < 4; tk = tk + 1) begin
        t_waddr_calc[tk] = t2_alloc_idx[tk];
        t_wdata_calc[tk] = {1'b1, t2_alloc_tag[tk],
                            t2_taken ? 3'd4 : 3'd3, 2'b00};
    end
    if (t2_valid) begin
        // 基础表恒训练
        base_we_calc = 1'b1;
        if (m_pvalid && provider_tag_match) begin
            // provider 原地更新（ctr + useful；tag 比对通过才写，防队列期间被换项）
            t_we_calc[m_pid]    = 1'b1;
            t_waddr_calc[m_pid] = m_pidx;
            t_wdata_calc[m_pid] = {1'b1,
                                   m_ptag,
                                   sat3(m_pctr, t2_taken), t1_u_new};
        end
        // 误预测且有分配候选（且候选不是 provider 自身）：分配新项
        if (t2_mispred && alloc_any && !(m_pvalid && (alloc_sel == m_pid))) begin
            t_we_calc[alloc_sel] = 1'b1;
        end else if (t2_mispred && !alloc_any) begin
            // 无可分配项：把比 provider 更长历史表的 useful 清 0（腾位）
            for (tk = 0; tk < 4; tk = tk + 1) begin
                if (!m_pvalid || (tk[1:0] > m_pid)) begin
                    if (t2_rd_entry_eff[tk][1:0] != 2'b00) begin
                        t_we_calc[tk]    = 1'b1;
                        t_waddr_calc[tk] = t2_alloc_idx[tk];
                        t_wdata_calc[tk] = {t2_rd_entry_eff[tk][TENTRY_W-1],
                                            t2_rd_entry_eff[tk][TENTRY_W-2 -: `TAGE_TAG_W],
                                            t2_rd_entry_eff[tk][4:2], 2'b00};
                    end
                end
            end
        end
    end
end

integer pk;
always @(posedge clk) begin
    if (reset) begin
        t0_valid     <= 1'b0;
        t1_valid     <= 1'b0;
        t2_valid     <= 1'b0;
        t3_base_we   <= 1'b0;
        t3_t_we      <= 4'b0;
        t3_last_t_we <= 4'b0;
        t3_base_waddr <= {BASE_IDXW{1'b0}};
        t3_base_wdata <= 2'b0;
        for (pk = 0; pk < 4; pk = pk + 1) begin
            t3_t_waddr[pk]      <= {TIDXW{1'b0}};
            t3_t_wdata[pk]      <= {TENTRY_W{1'b0}};
            t3_last_t_waddr[pk] <= {TIDXW{1'b0}};
            t3_last_t_wdata[pk] <= {TENTRY_W{1'b0}};
        end
        use_alt_on_na <= 4'd0;
        spec_ghr     <= {`GHR_LEN{1'b0}};
        commit_ghr   <= {`GHR_LEN{1'b0}};
        uq_rptr      <= {TAGE_UPDATE_Q_PTR_W{1'b0}};
        uq_wptr      <= {TAGE_UPDATE_Q_PTR_W{1'b0}};
        uq_count     <= {TAGE_UPDATE_Q_CNT_W{1'b0}};
    end else begin
        // T3 write command.  The RAM modules see only registered controls;
        // throughput remains one completed training command per cycle.
        t3_base_we    <= base_we_calc;
        t3_base_waddr <= base_waddr_calc;
        t3_base_wdata <= base_wdata_calc;
        t3_last_t_we  <= t3_t_we;
        t3_t_we       <= t_we_calc;
        for (pk = 0; pk < 4; pk = pk + 1) begin
            t3_last_t_waddr[pk] <= t3_t_waddr[pk];
            t3_last_t_wdata[pk] <= t3_t_wdata[pk];
            t3_t_waddr[pk]      <= t_waddr_calc[pk];
            t3_t_wdata[pk]      <= t_wdata_calc[pk];
        end

        if (t2_valid && train_alt_better && (use_alt_on_na != 4'hf))
            use_alt_on_na <= use_alt_on_na + 4'd1;
        else if (t2_valid && train_prov_better &&
                 (use_alt_on_na != 4'h0))
            use_alt_on_na <= use_alt_on_na - 4'd1;

        if (cmt_hist_valid_i)
            commit_ghr <= hist_shift(commit_ghr, cmt_hist_taken_i);

        if (flush_i) begin
            spec_ghr <= cmt_hist_valid_i
                      ? hist_shift(commit_ghr, cmt_hist_taken_i)
                      : commit_ghr;
        end else if (hist_restore_i) begin
            spec_ghr <= hist_restore_append_i
                      ? hist_shift(hist_checkpoint[hist_restore_id_i],
                                   hist_restore_taken_i)
                      : hist_checkpoint[hist_restore_id_i];
        end else if (hist_update_valid_i) begin
            spec_ghr <= hist_shift(spec_ghr, hist_update_taken_i);
        end

        // T0：出队项进读表级（用入队快照 GHR 重算读地址，已在组合段完成，
        //     此处仅锁存出队 bundle；meta 无效项直接旁落不进流水）
        if (tage_update_enqueue) begin
            uq_pc[uq_wptr]      <= train_pc_i;
            uq_taken[uq_wptr]   <= train_taken_i;
            uq_mispred[uq_wptr] <= train_mispred_i;
            uq_meta[uq_wptr]    <= train_meta_i;
            uq_ghr[uq_wptr]     <= hist_checkpoint[train_ftq_id];
            uq_wptr             <= uq_wptr + {{(TAGE_UPDATE_Q_PTR_W-1){1'b0}},1'b1};
        end
        if (tage_update_dequeue)
            uq_rptr <= uq_rptr + {{(TAGE_UPDATE_Q_PTR_W-1){1'b0}},1'b1};
        uq_count <= uq_count_next;

        t0_valid     <= train_read_grant && uq_head_meta_valid;
        if (train_read_grant) begin
            t0_pc      <= uq_head_pc;
            t0_taken   <= uq_head_taken;
            t0_mispred <= uq_head_mispred;
            t0_meta    <= uq_head_meta;
            t0_ghr     <= uq_head_ghr;
        end
        // T1：锁存 T0 读出的 4 路表项与分配用 idx/tag（快照 GHR 版本）
        t1_valid <= t0_valid;
        if (t0_valid) begin
            t1_taken   <= t0_taken;
            t1_mispred <= t0_mispred;
            t1_meta    <= t0_meta;
            for (pk = 0; pk < 4; pk = pk + 1) begin
                t1_alloc_idx[pk]  <= tidx_h(t0_pc, pk, t0_ghr);
                t1_alloc_tag[pk]  <= ttag_h(t0_pc[13:2], pk, t0_ghr);
                t1_rd_entry[pk]   <= t_train_rdata[pk];
            end
        end
        t2_valid <= t1_valid;
        if (t1_valid) begin
            t2_taken    <= t1_taken;
            t2_mispred  <= t1_mispred;
            t2_meta     <= t1_meta;
            for (pk = 0; pk < 4; pk = pk + 1) begin
                t2_alloc_idx[pk] <= t1_alloc_idx[pk];
                t2_alloc_tag[pk] <= t1_alloc_tag[pk];
                t2_rd_entry[pk]  <= t1_rd_entry[pk];
            end
        end
    end
end

// 在 P1 更新前保存历史；非阻塞赋值保证同拍保存和移位读取旧的 spec_ghr。
always @(posedge clk) begin
    if (!reset && hist_checkpoint_save_i && !hist_restore_i && !flush_i)
        hist_checkpoint[hist_checkpoint_id_i] <= spec_ghr;
end

`ifdef SYNTHESIS
// synthesis translate_off
wire [2:0] tage_update_pipeline_pending_w = {2'b0, t0_valid}
                                          + {2'b0, t1_valid}
                                          + {2'b0, t2_valid}
                                          + {2'b0, t3_base_we};

reg [63:0] tage_update_overflow_count;
reg [63:0] tage_update_queue_max_occupancy;
reg [63:0] tage_update_pipeline_max_pending_count;
reg [63:0] tage_train_count;
reg [63:0] tage_provider_base_count;
reg [63:0] tage_provider_base_correct_count;
reg [63:0] tage_provider_t0_count;
reg [63:0] tage_provider_t0_correct_count;
reg [63:0] tage_provider_t1_count;
reg [63:0] tage_provider_t1_correct_count;
reg [63:0] tage_provider_t2_count;
reg [63:0] tage_provider_t2_correct_count;
reg [63:0] tage_provider_t3_count;
reg [63:0] tage_provider_t3_correct_count;
reg [63:0] tage_weak_provider_count;
reg [63:0] tage_weak_provider_correct_count;
reg [63:0] tage_weak_disagree_count;
reg [63:0] tage_weak_provider_better_count;
reg [63:0] tage_weak_alt_better_count;
reg [63:0] tage_provider_alt_disagree_count;
reg [63:0] tage_provider_better_count;
reg [63:0] tage_alt_better_count;
reg [63:0] tage_allocation_success_count;
reg [63:0] tage_allocation_failure_count;
reg [63:0] tage_provider_update_lost_count;

wire stat_provider_correct = (m_pctr[2] == t2_taken);
wire stat_base_correct = (m_base_ctr[1] == t2_taken);
wire stat_allocation_success =
    t2_mispred && alloc_any &&
    !(m_pvalid && (alloc_sel == m_pid));

always @(posedge clk) begin
    if (reset) begin
        tage_update_overflow_count     <= 64'd0;
        tage_update_queue_max_occupancy <= 64'd0;
        tage_update_pipeline_max_pending_count <= 64'd0;
        tage_train_count <= 64'd0;
        tage_provider_base_count <= 64'd0;
        tage_provider_base_correct_count <= 64'd0;
        tage_provider_t0_count <= 64'd0;
        tage_provider_t0_correct_count <= 64'd0;
        tage_provider_t1_count <= 64'd0;
        tage_provider_t1_correct_count <= 64'd0;
        tage_provider_t2_count <= 64'd0;
        tage_provider_t2_correct_count <= 64'd0;
        tage_provider_t3_count <= 64'd0;
        tage_provider_t3_correct_count <= 64'd0;
        tage_weak_provider_count <= 64'd0;
        tage_weak_provider_correct_count <= 64'd0;
        tage_weak_disagree_count <= 64'd0;
        tage_weak_provider_better_count <= 64'd0;
        tage_weak_alt_better_count <= 64'd0;
        tage_provider_alt_disagree_count <= 64'd0;
        tage_provider_better_count <= 64'd0;
        tage_alt_better_count <= 64'd0;
        tage_allocation_success_count <= 64'd0;
        tage_allocation_failure_count <= 64'd0;
        tage_provider_update_lost_count <= 64'd0;
    end else begin
        if (tage_update_overflow)
            tage_update_overflow_count <= tage_update_overflow_count + 64'd1;
        if (uq_count_next_64 > tage_update_queue_max_occupancy)
            tage_update_queue_max_occupancy <= uq_count_next_64;
        if ({61'd0, tage_update_pipeline_pending_w} > tage_update_pipeline_max_pending_count)
            tage_update_pipeline_max_pending_count <= {61'd0, tage_update_pipeline_pending_w};
        if (t2_valid) begin
            tage_train_count <= tage_train_count + 64'd1;
            if (!m_pvalid) begin
                tage_provider_base_count <= tage_provider_base_count + 64'd1;
                if (stat_base_correct)
                    tage_provider_base_correct_count <=
                        tage_provider_base_correct_count + 64'd1;
            end else begin
                case (m_pid)
                    2'd0: begin
                        tage_provider_t0_count <= tage_provider_t0_count + 64'd1;
                        if (stat_provider_correct)
                            tage_provider_t0_correct_count <=
                                tage_provider_t0_correct_count + 64'd1;
                    end
                    2'd1: begin
                        tage_provider_t1_count <= tage_provider_t1_count + 64'd1;
                        if (stat_provider_correct)
                            tage_provider_t1_correct_count <=
                                tage_provider_t1_correct_count + 64'd1;
                    end
                    2'd2: begin
                        tage_provider_t2_count <= tage_provider_t2_count + 64'd1;
                        if (stat_provider_correct)
                            tage_provider_t2_correct_count <=
                                tage_provider_t2_correct_count + 64'd1;
                    end
                    default: begin
                        tage_provider_t3_count <= tage_provider_t3_count + 64'd1;
                        if (stat_provider_correct)
                            tage_provider_t3_correct_count <=
                                tage_provider_t3_correct_count + 64'd1;
                    end
                endcase
                if (train_prov_weak) begin
                    tage_weak_provider_count <=
                        tage_weak_provider_count + 64'd1;
                    if (stat_provider_correct)
                        tage_weak_provider_correct_count <=
                            tage_weak_provider_correct_count + 64'd1;
                    if (m_pctr[2] != m_alt) begin
                        tage_weak_disagree_count <=
                            tage_weak_disagree_count + 64'd1;
                        if (stat_provider_correct)
                            tage_weak_provider_better_count <=
                                tage_weak_provider_better_count + 64'd1;
                        else
                            tage_weak_alt_better_count <=
                                tage_weak_alt_better_count + 64'd1;
                    end
                end
                if (m_pctr[2] != m_alt) begin
                    tage_provider_alt_disagree_count <=
                        tage_provider_alt_disagree_count + 64'd1;
                    if (stat_provider_correct)
                        tage_provider_better_count <=
                            tage_provider_better_count + 64'd1;
                    else
                        tage_alt_better_count <= tage_alt_better_count + 64'd1;
                end
                if (!provider_tag_match)
                    tage_provider_update_lost_count <=
                        tage_provider_update_lost_count + 64'd1;
            end
            if (stat_allocation_success)
                tage_allocation_success_count <=
                    tage_allocation_success_count + 64'd1;
            if (t2_mispred && !alloc_any)
                tage_allocation_failure_count <=
                    tage_allocation_failure_count + 64'd1;
        end
    end
end
// synthesis translate_on
`endif

endmodule

// ------------------------------------------------------------
// tage_base_ram / tage_tag_ram：真双读口同步 RAM（2R+1W，推断 BRAM）
// 同址同拍写转发进对应读寄存器，避免训练写与查询/训练读 RAW 旧值。
// ------------------------------------------------------------
module tage_base_ram(
    input  wire        clk,
    input  wire [12:0] q_raddr,
    output wire [1:0]  q_rdata,
    input  wire        we,
    input  wire [12:0] waddr,
    input  wire [1:0]  wdata
);
// 两份保持一致的 BRAM 副本实现双读口，写入同时广播到两份副本。
// Read-during-write forwarding is implemented explicitly in the parent TAGE
// pipeline using the pending and last-committed T3 commands.  Keeping it out
// of these RAM wrappers prevents Vivado from folding the T2 command network
// back into an internal bypass-valid register.
(* ram_style = "block" *) reg [1:0] q_mem [0:`TAGE_BASE_DEPTH-1];
reg [1:0] q_rdata_ram;
integer i;
initial begin
    for (i = 0; i < `TAGE_BASE_DEPTH; i = i + 1) begin
        q_mem[i] = 2'b01;
    end
end
always @(posedge clk) begin
    q_rdata_ram <= q_mem[q_raddr];
    if (we)
        q_mem[waddr] <= wdata;
end
assign q_rdata = q_rdata_ram;
endmodule

module tage_tag_ram #(
    parameter ENTRY_W = 1 + `TAGE_TAG_W + 3 + 2
)(
    input  wire        clk,
    input  wire [9:0] q_raddr,
    output wire [ENTRY_W-1:0] q_rdata,
    input  wire [9:0] t_raddr,
    output wire [ENTRY_W-1:0] t_rdata,
    input  wire        we,
    input  wire [9:0]  waddr,
    input  wire [ENTRY_W-1:0] wdata
);
(* ram_style = "block" *) reg [ENTRY_W-1:0] q_mem [0:`TAGE_TAG_DEPTH-1];
(* ram_style = "block" *) reg [ENTRY_W-1:0] t_mem [0:`TAGE_TAG_DEPTH-1];
reg [ENTRY_W-1:0] q_rdata_ram, t_rdata_ram;
integer i;
initial begin
    for (i = 0; i < `TAGE_TAG_DEPTH; i = i + 1) begin
        q_mem[i] = {ENTRY_W{1'b0}};
        t_mem[i] = {ENTRY_W{1'b0}};
    end
end
always @(posedge clk) begin
    q_rdata_ram <= q_mem[q_raddr];
    t_rdata_ram <= t_mem[t_raddr];
    if (we) begin
        q_mem[waddr] <= wdata;
        t_mem[waddr] <= wdata;
    end
end
assign q_rdata = q_rdata_ram;
assign t_rdata = t_rdata_ram;
endmodule
