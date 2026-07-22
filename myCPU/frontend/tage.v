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

    // ---------------- 查询口（1 拍延迟返回）----------------
    input  wire                       query_valid_i,
    input  wire [31:0]                query_pc_i,         // 预测块起始 PC

    output wire                       taken_o,            // 方向预测（晚查询 1 拍）
    output wire                       resp_valid_o,
    output wire [`BPU_META_W-1:0]     meta_o,             // 训练回带信息

    // ---------------- 训练口（提交级 BPU 训练，经内部 FIFO 缓冲）----------------
    input  wire                       train_valid_i,
    input  wire [31:0]                train_pc_i,
    input  wire                       train_taken_i,      // 实际方向
    input  wire                       train_mispred_i,    // 该分支发生误预测
    input  wire [`BPU_META_W-1:0]     train_meta_i        // 查询时的 meta 原样回传
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
localparam TAGE_UPDATE_Q_DEPTH = `TAGE_UPDATE_Q_DEPTH;
localparam TAGE_UPDATE_Q_PTR_W =
    (TAGE_UPDATE_Q_DEPTH <= 1) ? 1 : $clog2(TAGE_UPDATE_Q_DEPTH);
localparam TAGE_UPDATE_Q_CNT_W = $clog2(TAGE_UPDATE_Q_DEPTH + 1);
localparam [TAGE_UPDATE_Q_CNT_W-1:0] TAGE_UPDATE_Q_DEPTH_C = TAGE_UPDATE_Q_DEPTH;

// ---------------- GHR ----------------
reg [`GHR_LEN-1:0] ghr;

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
    input [31:0] pc;
    input integer t;
    begin
        tidx = fold10(ghr, (t == 0) ? `TAGE_HIST_LEN0 :
                           (t == 1) ? `TAGE_HIST_LEN1 :
                           (t == 2) ? `TAGE_HIST_LEN2 : `TAGE_HIST_LEN3)
             ^ pc[2 +: TIDXW] ^ pc[12 +: TIDXW];
    end
endfunction

function [TIDXW-1:0] tidx_h;
    input [31:0] pc;
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
    input [31:0] pc;
    input integer t;
    begin
        ttag = fold12(ghr, (t == 0) ? `TAGE_HIST_LEN0 :
                           (t == 1) ? `TAGE_HIST_LEN1 :
                           (t == 2) ? `TAGE_HIST_LEN2 : `TAGE_HIST_LEN3)
             ^ pc[2 +: `TAGE_TAG_W];
    end
endfunction

function [`TAGE_TAG_W-1:0] ttag_h;
    input [31:0] pc;
    input integer t;
    input [`GHR_LEN-1:0] h;
    begin
        ttag_h = fold12(h, (t == 0) ? `TAGE_HIST_LEN0 :
                           (t == 1) ? `TAGE_HIST_LEN1 :
                           (t == 2) ? `TAGE_HIST_LEN2 : `TAGE_HIST_LEN3)
               ^ pc[2 +: `TAGE_TAG_W];
    end
endfunction

// ---------------- 训练流水线寄存器（T0 读表 / T1 / T2 写回）----------------
reg        t0_valid;
reg        t0_ghr_valid;
reg [31:0] t0_pc;
reg        t0_taken, t0_mispred;
reg        t0_ghr_taken;
reg [`GHR_LEN-1:0] t0_ghr;
reg [`BPU_META_W-1:0] t0_meta;
reg        t1_valid;
reg        t1_taken, t1_mispred;
reg [`BPU_META_W-1:0] t1_meta;
reg [TIDXW-1:0]      t1_alloc_idx [0:3];
reg [`TAGE_TAG_W-1:0] t1_alloc_tag [0:3];
reg [TENTRY_W-1:0]   t1_rd_entry [0:3];
reg [BASE_IDXW-1:0]  t1_base_idx;
reg        t2_valid;
reg        t2_taken, t2_mispred;
reg [`BPU_META_W-1:0] t2_meta;
reg [TIDXW-1:0]      t2_alloc_idx [0:3];
reg [`TAGE_TAG_W-1:0] t2_alloc_tag [0:3];
reg [TENTRY_W-1:0]   t2_rd_entry [0:3];
reg [BASE_IDXW-1:0]  t2_base_idx;

reg [31:0]               uq_pc      [0:TAGE_UPDATE_Q_DEPTH-1];
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

wire [31:0]            uq_head_pc      = uq_pc[uq_rptr];
wire                   uq_head_taken   = uq_taken[uq_rptr];
wire                   uq_head_mispred = uq_mispred[uq_rptr];
wire [`BPU_META_W-1:0] uq_head_meta    = uq_meta[uq_rptr];
wire [`GHR_LEN-1:0]    uq_head_ghr     = uq_ghr[uq_rptr];
wire                   uq_head_meta_valid = uq_head_meta[META_TAGE_VALID_BIT];
wire                   uq_head_pvalid  = uq_head_meta[META_PROVIDER_VALID_BIT];
wire [1:0]             uq_head_pid     = uq_head_meta[META_PROVIDER_ID_LSB +: 2];
wire [TIDXW-1:0]       uq_head_pidx    = uq_head_meta[META_PROVIDER_IDX_LSB +: TIDXW];

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
wire [1:0] base_train_rdata;
reg        base_we;
reg [BASE_IDXW-1:0] base_waddr;
reg [1:0]  base_wdata;

tage_base_ram u_base(
    .clk(clk),
    .q_raddr(q_base_idx), .q_rdata(base_q_rdata),
    .t_raddr(uq_head_pc[2 +: BASE_IDXW]), .t_rdata(base_train_rdata),
    .we(base_we), .waddr(base_waddr), .wdata(base_wdata)
);

// ---------------- 4 个标记表（2R+1W）----------------
wire [TIDXW-1:0] q_idx [0:3];
wire [`TAGE_TAG_W-1:0] q_tag [0:3];
wire [TENTRY_W-1:0] t_q_rdata [0:3];
wire [TENTRY_W-1:0] t_train_rdata [0:3];
reg  [3:0] t_we;
reg  [TIDXW-1:0] t_waddr [0:3];
reg  [TENTRY_W-1:0] t_wdata [0:3];

wire                  t0_meta_valid = t0_meta[META_TAGE_VALID_BIT];
wire                  t0_pvalid     = t0_meta[META_PROVIDER_VALID_BIT];
wire [1:0]            t0_pid        = t0_meta[META_PROVIDER_ID_LSB +: 2];
wire [TIDXW-1:0]      t0_pidx       = t0_meta[META_PROVIDER_IDX_LSB +: TIDXW];
wire [`TAGE_TAG_W-1:0] t0_ptag      = t0_meta[META_PROVIDER_TAG_LSB +: META_PROVIDER_TAG_W];

genvar g;
generate
for (g = 0; g < 4; g = g + 1) begin : gen_ttab
    localparam [1:0] TABLE_ID = g[1:0];
    wire provider_read_sel = uq_head_meta_valid && uq_head_pvalid && (uq_head_pid == TABLE_ID);
    wire [TIDXW-1:0] train_raddr = provider_read_sel ? uq_head_pidx : tidx_h(uq_head_pc, g, uq_head_ghr);
    assign q_idx[g] = tidx(query_pc_i, g);
    assign q_tag[g] = ttag(query_pc_i, g);
    tage_tag_ram u_ttab(
        .clk(clk),
        .q_raddr(q_idx[g]),
        .q_rdata(t_q_rdata[g]),
        .t_raddr(train_raddr),
        .t_rdata(t_train_rdata[g]),
        .we(t_we[g]),
        .waddr(t_waddr[g]),
        .wdata(t_wdata[g])
    );
end
endgenerate

// ---------------- 查询响应（相对查询晚 1 拍）----------------
reg        q_valid_r;
reg [`TAGE_TAG_W-1:0] q_tag_r [0:3];
reg [TIDXW-1:0]       q_idx_r [0:3];
reg [BASE_IDXW-1:0]   q_bidx_r;
integer qk;
always @(posedge clk) begin
    q_valid_r <= query_valid_i;
    for (qk = 0; qk < 4; qk = qk + 1) begin
        q_tag_r[qk] <= q_tag[qk];
        q_idx_r[qk] <= q_idx[qk];
    end
    q_bidx_r <= q_base_idx;
end

// 查询侧组合写旁路：T2 写回与查询响应同拍命中同一索引时，用新值
wire [3:0] thit;
wire [TENTRY_W-1:0] t_q_entry_eff [0:3];
generate
for (g = 0; g < 4; g = g + 1) begin : gen_thit
    assign t_q_entry_eff[g] = (t_we[g] && (t_waddr[g] == q_idx_r[g])) ? t_wdata[g] : t_q_rdata[g];
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
wire [1:0] base_q_eff = (base_we && (base_waddr == q_bidx_r)) ? base_wdata : base_q_rdata;
wire       base_taken = base_q_eff[1];

assign resp_valid_o = q_valid_r;
assign taken_o = q_valid_r && (prov_valid ? prov_ctr[2] : base_taken);

// meta 打包：{prov_useful(2), prov_ctr(3), prov_idx(10), prov_id(2), prov_valid(1),
//             alt_taken(1), base_ctr(2), base_idx(13), hits(4)} 共 38 位，
wire [`BPU_META_W-1:0] meta_raw =
    { {(`BPU_META_W-META_RAW_W){1'b0}},
      prov_u, prov_ctr, q_idx_r[prov_id], prov_id, prov_valid,
      base_taken, base_q_eff, q_bidx_r, thit };
wire [`BPU_META_W-1:0] meta_provider_tag =
    { {(`BPU_META_W-META_PROVIDER_TAG_W){1'b0}}, prov_tag } << META_PROVIDER_TAG_LSB;
assign meta_o = q_valid_r ? (meta_raw | meta_provider_tag) : {`BPU_META_W{1'b0}};

// meta 解包（训练 T2 拍使用）
wire                  train_meta_valid = train_meta_i[META_TAGE_VALID_BIT];
wire [3:0]            m_hits     = t2_meta[META_HITS_LSB +: 4];
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
    assign alloc_cand[g] = (!t2_rd_entry[g][TENTRY_W-1]
                         || (t2_rd_entry[g][1:0] == 2'b00))
                        && (!m_pvalid || (g[1:0] > m_pid));
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
wire provider_entry_valid = t2_rd_entry[m_pid][TENTRY_W-1];
wire provider_tag_match = provider_entry_valid &&
                          (t2_rd_entry[m_pid][TENTRY_W-2 -: `TAGE_TAG_W] == m_ptag);

integer tk;
always @(*) begin
    base_we    = 1'b0;
    base_waddr = m_base_idx;
    base_wdata = sat2(m_base_ctr, t2_taken);
    t_we       = 4'b0;
    for (tk = 0; tk < 4; tk = tk + 1) begin
        t_waddr[tk] = t2_alloc_idx[tk];
        t_wdata[tk] = {1'b1, t2_alloc_tag[tk],
                       t2_taken ? 3'd4 : 3'd3, 2'b00};
    end
        if (t2_valid) begin
        // 基础表恒训练
        base_we = 1'b1;
        if (m_pvalid && provider_tag_match) begin
            // provider 原地更新（ctr + useful；tag 比对通过才写，防队列期间被换项）
            t_we[m_pid]    = 1'b1;
            t_waddr[m_pid] = m_pidx;
            t_wdata[m_pid] = {1'b1,
                              m_ptag,
                              sat3(m_pctr, t2_taken), t1_u_new};
        end
        // 误预测且有分配候选（且候选不是 provider 自身）：分配新项
        if (t2_mispred && alloc_any && !(m_pvalid && (alloc_sel == m_pid))) begin
            t_we[alloc_sel] = 1'b1;
        end else if (t2_mispred && !alloc_any) begin
            // 无可分配项：把比 provider 更长历史表的 useful 清 0（腾位）
            for (tk = 0; tk < 4; tk = tk + 1) begin
                if (!m_pvalid || (tk[1:0] > m_pid)) begin
                    if (t2_rd_entry[tk][1:0] != 2'b00) begin
                        t_we[tk]    = 1'b1;
                        t_waddr[tk] = t2_alloc_idx[tk];
                        t_wdata[tk] = {t2_rd_entry[tk][TENTRY_W-1],
                                       t2_rd_entry[tk][TENTRY_W-2 -: `TAGE_TAG_W],
                                       t2_rd_entry[tk][4:2], 2'b00};
                    end
                end
            end
        end
    end
end

reg [`TAGE_TAG_W-1:0] t1_rd_tag [0:3];  // lint 吸收用途

integer pk;
always @(posedge clk) begin
    if (reset) begin
        t0_valid     <= 1'b0;
        t0_ghr_valid <= 1'b0;
        t1_valid     <= 1'b0;
        t2_valid     <= 1'b0;
        ghr          <= {`GHR_LEN{1'b0}};
        uq_rptr      <= {TAGE_UPDATE_Q_PTR_W{1'b0}};
        uq_wptr      <= {TAGE_UPDATE_Q_PTR_W{1'b0}};
        uq_count     <= {TAGE_UPDATE_Q_CNT_W{1'b0}};
    end else begin
        // T0：出队项进读表级（用入队快照 GHR 重算读地址，已在组合段完成，
        //     此处仅锁存出队 bundle；meta 无效项直接旁落不进流水）
        if (tage_update_enqueue) begin
            uq_pc[uq_wptr]      <= train_pc_i;
            uq_taken[uq_wptr]   <= train_taken_i;
            uq_mispred[uq_wptr] <= train_mispred_i;
            uq_meta[uq_wptr]    <= train_meta_i;
            uq_ghr[uq_wptr]     <= ghr;
            uq_wptr             <= uq_wptr + {{(TAGE_UPDATE_Q_PTR_W-1){1'b0}},1'b1};
        end
        if (tage_update_dequeue)
            uq_rptr <= uq_rptr + {{(TAGE_UPDATE_Q_PTR_W-1){1'b0}},1'b1};
        uq_count <= uq_count_next;

        t0_valid     <= train_read_grant && uq_head_meta_valid;
        t0_ghr_valid <= train_valid_i;
        t0_ghr_taken <= train_taken_i;
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
            t1_base_idx <= t0_pc[2 +: BASE_IDXW];
            for (pk = 0; pk < 4; pk = pk + 1) begin
                t1_alloc_idx[pk]  <= tidx_h(t0_pc, pk, t0_ghr);
                t1_alloc_tag[pk]  <= ttag_h(t0_pc, pk, t0_ghr);
                t1_rd_entry[pk]   <= t_train_rdata[pk];
                t1_rd_tag[pk]     <= t_train_rdata[pk][TENTRY_W-2 -: `TAGE_TAG_W];
            end
        end
        t2_valid <= t1_valid;
        if (t1_valid) begin
            t2_taken    <= t1_taken;
            t2_mispred  <= t1_mispred;
            t2_meta     <= t1_meta;
            t2_base_idx <= t1_base_idx;
            for (pk = 0; pk < 4; pk = pk + 1) begin
                t2_alloc_idx[pk] <= t1_alloc_idx[pk];
                t2_alloc_tag[pk] <= t1_alloc_tag[pk];
                t2_rd_entry[pk]  <= t1_rd_entry[pk];
            end
        end
        if (train_valid_i)
            ghr <= {ghr[`GHR_LEN-2:0], train_taken_i};
    end
end

// lint 吸收
wire tage_lint = (|m_hits) | t0_mispred | (|t0_ptag) | (|base_train_rdata)
               | t0_meta_valid | t0_pvalid | (|t0_pid) | (|t0_pidx)
               | train_meta_valid;

`ifdef SYNTHESIS
// synthesis translate_off
reg tage_useful_clear_fire;
integer stat_tk;
always @(*) begin
    tage_useful_clear_fire = 1'b0;
    if (t2_valid && t2_mispred && !alloc_any) begin
        for (stat_tk = 0; stat_tk < 4; stat_tk = stat_tk + 1) begin
            if ((!m_pvalid || (stat_tk[1:0] > m_pid)) &&
                (t2_rd_entry[stat_tk][1:0] != 2'b00)) begin
                tage_useful_clear_fire = 1'b1;
            end
        end
    end
end

wire stat_provider_update_fire =
    t2_valid && m_pvalid && t_we[m_pid];
wire stat_allocation_fire =
    t2_valid && t2_mispred && alloc_any && !(m_pvalid && (alloc_sel == m_pid)) && t_we[alloc_sel];
wire tage_update_complete = t2_valid || (train_read_grant && !uq_head_meta_valid);
wire [1:0] tage_update_pipeline_pending_w = {1'b0, t0_valid}
                                          + {1'b0, t1_valid}
                                          + {1'b0, t2_valid};

reg [63:0] tage_query_total;
reg [63:0] tage_query_suppressed_by_train;
reg [63:0] tage_response_total;
reg [63:0] tage_train_total;                 // training request count
reg [63:0] tage_mispred_train_total;
reg [63:0] tage_train_request_count;
reg [63:0] tage_train_meta_valid_count;
reg [63:0] tage_train_meta_invalid_count;
reg [63:0] tage_train_accepted_count;
reg [63:0] tage_base_update_count;
reg [63:0] tage_provider_update_count;
reg [63:0] tage_allocation_count;
reg [63:0] tage_useful_clear_count;
reg [63:0] tage_provider_tag_check_count;
reg [63:0] tage_provider_tag_match_count;
reg [63:0] tage_provider_tag_mismatch_count;
reg [63:0] tage_update_request_count;
reg [63:0] tage_update_enqueue_count;
reg [63:0] tage_update_dequeue_count;
reg [63:0] tage_update_complete_count;
reg [63:0] tage_update_overflow_count;
reg [63:0] tage_update_queue_pending_count;
reg [63:0] tage_update_queue_max_occupancy;
reg [63:0] tage_update_pipeline_pending_count;
reg [63:0] tage_update_pipeline_max_pending_count;
reg [63:0] tage_query_while_update_arrives_count;
reg [63:0] tage_train_read_grant_count;
reg [63:0] tage_train_wait_cycle_count;
reg [63:0] tage_train_max_wait_cycle_count;
reg [63:0] tage_train_wait_current_count;

always @(posedge clk) begin
    if (reset) begin
        tage_query_total               <= 64'd0;
        tage_query_suppressed_by_train <= 64'd0;
        tage_response_total            <= 64'd0;
        tage_train_total               <= 64'd0;
        tage_mispred_train_total       <= 64'd0;
        tage_train_request_count       <= 64'd0;
        tage_train_meta_valid_count    <= 64'd0;
        tage_train_meta_invalid_count  <= 64'd0;
        tage_train_accepted_count      <= 64'd0;
        tage_base_update_count         <= 64'd0;
        tage_provider_update_count     <= 64'd0;
        tage_allocation_count          <= 64'd0;
        tage_useful_clear_count        <= 64'd0;
        tage_provider_tag_check_count  <= 64'd0;
        tage_provider_tag_match_count  <= 64'd0;
        tage_provider_tag_mismatch_count <= 64'd0;
        tage_update_request_count      <= 64'd0;
        tage_update_enqueue_count      <= 64'd0;
        tage_update_dequeue_count      <= 64'd0;
        tage_update_complete_count     <= 64'd0;
        tage_update_overflow_count     <= 64'd0;
        tage_update_queue_pending_count <= 64'd0;
        tage_update_queue_max_occupancy <= 64'd0;
        tage_update_pipeline_pending_count <= 64'd0;
        tage_update_pipeline_max_pending_count <= 64'd0;
        tage_query_while_update_arrives_count <= 64'd0;
        tage_train_read_grant_count <= 64'd0;
        tage_train_wait_cycle_count <= 64'd0;
        tage_train_max_wait_cycle_count <= 64'd0;
        tage_train_wait_current_count <= 64'd0;
    end else begin
        if (query_valid_i)
            tage_query_total <= tage_query_total + 64'd1;
        if (q_valid_r)
            tage_response_total <= tage_response_total + 64'd1;
        if (train_valid_i) begin
            tage_train_total <= tage_train_total + 64'd1;
            tage_train_request_count <= tage_train_request_count + 64'd1;
            tage_update_request_count <= tage_update_request_count + 64'd1;
            if (train_meta_i[META_TAGE_VALID_BIT])
                tage_train_meta_valid_count <= tage_train_meta_valid_count + 64'd1;
            else
                tage_train_meta_invalid_count <= tage_train_meta_invalid_count + 64'd1;
        end
        if (query_valid_i && train_valid_i)
            tage_query_while_update_arrives_count <= tage_query_while_update_arrives_count + 64'd1;
        if (tage_update_enqueue)
            tage_update_enqueue_count <= tage_update_enqueue_count + 64'd1;
        if (tage_update_dequeue)
            tage_update_dequeue_count <= tage_update_dequeue_count + 64'd1;
        if (tage_update_complete)
            tage_update_complete_count <= tage_update_complete_count + 64'd1;
        if (tage_update_overflow) begin
            tage_update_overflow_count <= tage_update_overflow_count + 64'd1;
            // 与 FTB 一致：仿真只累计，不 $error/$stop（队列满时丢新训练）
        end
        if (uq_count_next_64 > tage_update_queue_max_occupancy)
            tage_update_queue_max_occupancy <= uq_count_next_64;
        tage_update_queue_pending_count <= uq_count_next_64;
        tage_update_pipeline_pending_count <= {62'd0, tage_update_pipeline_pending_w};
        if ({62'd0, tage_update_pipeline_pending_w} > tage_update_pipeline_max_pending_count)
            tage_update_pipeline_max_pending_count <= {62'd0, tage_update_pipeline_pending_w};
        if (train_read_grant)
            tage_train_read_grant_count <= tage_train_read_grant_count + 64'd1;
        if (!tage_update_queue_empty && !train_read_grant) begin
            tage_train_wait_cycle_count <= tage_train_wait_cycle_count + 64'd1;
            tage_train_wait_current_count <= tage_train_wait_current_count + 64'd1;
            if ((tage_train_wait_current_count + 64'd1) > tage_train_max_wait_cycle_count)
                tage_train_max_wait_cycle_count <= tage_train_wait_current_count + 64'd1;
        end else if (train_read_grant || tage_update_queue_empty) begin
            tage_train_wait_current_count <= 64'd0;
        end
        if (train_valid_i && train_mispred_i)
            tage_mispred_train_total <= tage_mispred_train_total + 64'd1;
        if (t0_valid)
            tage_train_accepted_count <= tage_train_accepted_count + 64'd1;
        if (base_we)
            tage_base_update_count <= tage_base_update_count + 64'd1;
        if (stat_provider_update_fire)
            tage_provider_update_count <= tage_provider_update_count + 64'd1;
        if (stat_allocation_fire)
            tage_allocation_count <= tage_allocation_count + 64'd1;
        if (tage_useful_clear_fire)
            tage_useful_clear_count <= tage_useful_clear_count + 64'd1;
        if (t2_valid && m_pvalid) begin
            tage_provider_tag_check_count <= tage_provider_tag_check_count + 64'd1;
            if (provider_tag_match)
                tage_provider_tag_match_count <= tage_provider_tag_match_count + 64'd1;
            else
                tage_provider_tag_mismatch_count <= tage_provider_tag_mismatch_count + 64'd1;
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
    output reg  [1:0]  q_rdata,
    input  wire [12:0] t_raddr,
    output reg  [1:0]  t_rdata,
    input  wire        we,
    input  wire [12:0] waddr,
    input  wire [1:0]  wdata
);
reg [1:0] mem [0:`TAGE_BASE_DEPTH-1];
integer i;
initial begin
    for (i = 0; i < `TAGE_BASE_DEPTH; i = i + 1) mem[i] = 2'b01;  // 弱不跳初值
end
always @(posedge clk) begin
    q_rdata <= (we && (waddr == q_raddr)) ? wdata : mem[q_raddr];
    t_rdata <= (we && (waddr == t_raddr)) ? wdata : mem[t_raddr];
    if (we) mem[waddr] <= wdata;
end
endmodule

module tage_tag_ram #(
    parameter ENTRY_W = 1 + `TAGE_TAG_W + 3 + 2
)(
    input  wire        clk,
    input  wire [9:0] q_raddr,
    output reg  [ENTRY_W-1:0] q_rdata,
    input  wire [9:0] t_raddr,
    output reg  [ENTRY_W-1:0] t_rdata,
    input  wire        we,
    input  wire [9:0]  waddr,
    input  wire [ENTRY_W-1:0] wdata
);
reg [ENTRY_W-1:0] mem [0:`TAGE_TAG_DEPTH-1];
integer i;
initial begin
    for (i = 0; i < `TAGE_TAG_DEPTH; i = i + 1) mem[i] = {ENTRY_W{1'b0}};
end
always @(posedge clk) begin
    q_rdata <= (we && (waddr == q_raddr)) ? wdata : mem[q_raddr];
    t_rdata <= (we && (waddr == t_raddr)) ? wdata : mem[t_raddr];
    if (we) mem[waddr] <= wdata;
end
endmodule

