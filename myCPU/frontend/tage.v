// ============================================================
// tage 模块（TAGE 条件分支方向预测器）
// ------------------------------------------------------------
// 参考实现说明：
// - 基础表 8192×2bit（bimodal）+ 4 个标记表 1024×{tag12,ctr3,u2}，
//   全部推断 BRAM（1R+1W），查询 1 拍延迟；
// - GHR 一期仅提交训练时移入实际方向（永远正确，精度略保守）；
// - 索引/标记哈希：GHR 异或折叠（fold 10/12 位）^ PC 位段；
// - meta 打包（44b 用 64b 容器）：
//   {prov_useful(2), prov_ctr(3), prov_idx(10), prov_id(2), prov_valid(1),
//    alt_taken(1), base_ctr(2), base_idx(13), hits(4)} —— 训练免重算 provider；
// - 训练 2 级小流水：T0 借查询读口读 4 表分配候选 + 基础表旧值，
//   T1 写：基础表/provider 计数、useful、误预测分配（useful==0 项，
//   无空位则将更长表 useful 清 0 腾位）。
// ============================================================
`include "mycpu.h"

module tage(
    input  wire                       clk,
    input  wire                       reset,

    // ---------------- 查询口（1 拍延迟）----------------
    input  wire                       query_valid_i,
    input  wire [31:0]                query_pc_i,         // 预测块起始 PC

    output wire                       taken_o,            // 方向预测（晚 1 拍）
    output wire [`BPU_META_W-1:0]     meta_o,             // 训练元数据

    // ---------------- 训练口（提交级，仅条件分支）----------------
    input  wire                       train_valid_i,
    input  wire [31:0]                train_pc_i,
    input  wire                       train_taken_i,      // 实际方向
    input  wire                       train_mispred_i,    // 是否误预测
    input  wire [`BPU_META_W-1:0]     train_meta_i        // 预测时的 meta 原样回传
);

localparam BASE_IDXW = 13;          // 8192
localparam TIDXW     = 10;          // 1024
localparam TENTRY_W  = `TAGE_TAG_W + 3 + 2;   // {tag, ctr, useful} = 17

// ---------------- GHR ----------------
reg [`GHR_LEN-1:0] ghr;

// ---------------- 折叠哈希 ----------------
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

// 各表索引/标记（查询用 query_pc，训练分配用 train_pc，共用函数）
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

// ---------------- 训练流水寄存器（T0 借读口）----------------
reg        t0_valid;
reg [31:0] t0_pc;
reg        t0_taken, t0_mispred;
reg [`BPU_META_W-1:0] t0_meta;
reg        t1_valid;
reg        t1_taken, t1_mispred;
reg [`BPU_META_W-1:0] t1_meta;
reg [TIDXW-1:0]      t1_alloc_idx [0:3];
reg [`TAGE_TAG_W-1:0] t1_alloc_tag [0:3];
reg [TENTRY_W-1:0]   t1_rd_entry [0:3];
reg [BASE_IDXW-1:0]  t1_base_idx;

wire rd_steal = t0_valid;

// ---------------- 基础表 ----------------
wire [BASE_IDXW-1:0] q_base_idx = query_pc_i[2 +: BASE_IDXW];
wire [BASE_IDXW-1:0] base_raddr = rd_steal ? t0_pc[2 +: BASE_IDXW] : q_base_idx;
wire [1:0] base_rdata;
reg        base_we;
reg [BASE_IDXW-1:0] base_waddr;
reg [1:0]  base_wdata;

tage_base_ram u_base(
    .clk(clk), .raddr(base_raddr), .rdata(base_rdata),
    .we(base_we), .waddr(base_waddr), .wdata(base_wdata)
);

// ---------------- 4 个标记表 ----------------
wire [TIDXW-1:0] q_idx [0:3];
wire [`TAGE_TAG_W-1:0] q_tag [0:3];
wire [TENTRY_W-1:0] t_rdata [0:3];
reg  [3:0] t_we;
reg  [TIDXW-1:0] t_waddr [0:3];
reg  [TENTRY_W-1:0] t_wdata [0:3];

genvar g;
generate
for (g = 0; g < 4; g = g + 1) begin : gen_ttab
    assign q_idx[g] = tidx(query_pc_i, g);
    assign q_tag[g] = ttag(query_pc_i, g);
    tage_tag_ram u_ttab(
        .clk(clk),
        .raddr(rd_steal ? tidx(t0_pc, g) : q_idx[g]),
        .rdata(t_rdata[g]),
        .we(t_we[g]),
        .waddr(t_waddr[g]),
        .wdata(t_wdata[g])
    );
end
endgenerate

// ---------------- 查询合成（晚 1 拍）----------------
reg        q_valid_r;
reg [`TAGE_TAG_W-1:0] q_tag_r [0:3];
reg [TIDXW-1:0]       q_idx_r [0:3];
reg [BASE_IDXW-1:0]   q_bidx_r;
integer qk;
always @(posedge clk) begin
    q_valid_r <= query_valid_i && !rd_steal;
    for (qk = 0; qk < 4; qk = qk + 1) begin
        q_tag_r[qk] <= q_tag[qk];
        q_idx_r[qk] <= q_idx[qk];
    end
    q_bidx_r <= q_base_idx;
end

wire [3:0] thit;
generate
for (g = 0; g < 4; g = g + 1) begin : gen_thit
    assign thit[g] = q_valid_r && (t_rdata[g][TENTRY_W-1 -: `TAGE_TAG_W] == q_tag_r[g]);
end
endgenerate

// provider = 命中表中历史最长者
wire       prov_valid = |thit;
wire [1:0] prov_id    = thit[3] ? 2'd3 : thit[2] ? 2'd2 : thit[1] ? 2'd1 : 2'd0;
wire [2:0] prov_ctr   = t_rdata[prov_id][4:2];
wire [1:0] prov_u     = t_rdata[prov_id][1:0];
wire       base_taken = base_rdata[1];

assign taken_o = prov_valid ? prov_ctr[2] : base_taken;

// meta 打包：{prov_useful(2), prov_ctr(3), prov_idx(10), prov_id(2), prov_valid(1),
//             alt_taken(1), base_ctr(2), base_idx(13), hits(4)}  共 38 位
assign meta_o = { {(`BPU_META_W-38){1'b0}},
                  prov_u, prov_ctr, q_idx_r[prov_id], prov_id, prov_valid,
                  base_taken, base_rdata, q_bidx_r, thit };

// meta 解包（训练端）
wire [3:0]            m_hits     = t1_meta[3:0];
wire [BASE_IDXW-1:0]  m_base_idx = t1_meta[4 +: BASE_IDXW];
wire [1:0]            m_base_ctr = t1_meta[17 +: 2];
wire                  m_alt      = t1_meta[19];
wire                  m_pvalid   = t1_meta[20];
wire [1:0]            m_pid      = t1_meta[21 +: 2];
wire [TIDXW-1:0]      m_pidx     = t1_meta[23 +: TIDXW];
wire [2:0]            m_pctr     = t1_meta[33 +: 3];
wire [1:0]            m_pu       = t1_meta[36 +: 2];

// ---------------- 训练 ----------------
// 饱和计数
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

// T1 拍：分配候选 = provider 更长的表中 useful==0 者（T0 读出）
wire [3:0] alloc_cand;
generate
for (g = 0; g < 4; g = g + 1) begin : gen_alloc
    assign alloc_cand[g] = (t1_rd_entry[g][1:0] == 2'b00)
                        && (!m_pvalid || (g[1:0] > m_pid));
end
endgenerate
wire alloc_any = |alloc_cand;
wire [1:0] alloc_sel = alloc_cand[0] ? 2'd0 : alloc_cand[1] ? 2'd1 :
                       alloc_cand[2] ? 2'd2 : 2'd3;

// provider 预测正确且与 alt 不同 -> useful++；错 -> useful--
wire t1_prov_pred  = m_pctr[2];
wire t1_prov_corr  = (t1_prov_pred == t1_taken);
wire t1_useful_chg = m_pvalid && (t1_prov_pred != m_alt);
wire [1:0] t1_u_new = !t1_useful_chg ? m_pu : sat2(m_pu, t1_prov_corr);

integer tk;
always @(*) begin
    base_we    = 1'b0;
    base_waddr = m_base_idx;
    base_wdata = sat2(m_base_ctr, t1_taken);
    t_we       = 4'b0;
    for (tk = 0; tk < 4; tk = tk + 1) begin
        t_waddr[tk] = t1_alloc_idx[tk];
        t_wdata[tk] = {t1_alloc_tag[tk], t1_taken ? 3'd4 : 3'd3, 2'b00};
    end
        if (t1_valid) begin
        // 基础表恒训练
        base_we = 1'b1;
        if (m_pvalid) begin
            // provider 原地更新（ctr + useful）
            t_we[m_pid]    = 1'b1;
            t_waddr[m_pid] = m_pidx;
            t_wdata[m_pid] = {t1_rd_entry[m_pid][TENTRY_W-1 -: `TAGE_TAG_W],
                              sat3(m_pctr, t1_taken), t1_u_new};
        end
        // 误预测：向更长表分配（与 provider 更新不同表，无冲突）
        if (t1_mispred && alloc_any && !(m_pvalid && (alloc_sel == m_pid))) begin
            t_we[alloc_sel] = 1'b1;
        end else if (t1_mispred && !alloc_any) begin
            // 无 useful==0 空位：更长历史表 useful 清 0 腾位
            for (tk = 0; tk < 4; tk = tk + 1) begin
                if (!m_pvalid || (tk[1:0] > m_pid)) begin
                    if (t1_rd_entry[tk][1:0] != 2'b00) begin
                        t_we[tk]    = 1'b1;
                        t_waddr[tk] = t1_alloc_idx[tk];
                        t_wdata[tk] = {t1_rd_entry[tk][TENTRY_W-1 -: `TAGE_TAG_W],
                                       t1_rd_entry[tk][4:2], 2'b00};
                    end
                end
            end
        end
    end
end

reg [`TAGE_TAG_W-1:0] t1_rd_tag [0:3];  // lint 吸收用

integer pk;
always @(posedge clk) begin
    if (reset) begin
        t0_valid <= 1'b0;
        t1_valid <= 1'b0;
        ghr      <= {`GHR_LEN{1'b0}};
    end else begin
        // T0：捕获训练（借读口；读地址按"训练前 GHR"算，与 GHR 移位同拍安全：
        //     ghr 在 T0 捕获拍尚未移位，T0 读用旧 GHR，T1 写 idx 也用 T0 算好的）
        t0_valid <= train_valid_i;
        if (train_valid_i) begin
            t0_pc      <= train_pc_i;
            t0_taken   <= train_taken_i;
            t0_mispred <= train_mispred_i;
            t0_meta    <= train_meta_i;
        end
        // T1：锁存 T0 读出与分配哈希；GHR 移位
        t1_valid <= t0_valid;
        if (t0_valid) begin
            t1_taken   <= t0_taken;
            t1_mispred <= t0_mispred;
            t1_meta    <= t0_meta;
            t1_base_idx <= t0_pc[2 +: BASE_IDXW];
            for (pk = 0; pk < 4; pk = pk + 1) begin
                t1_alloc_idx[pk]  <= tidx(t0_pc, pk);
                t1_alloc_tag[pk]  <= ttag(t0_pc, pk);
                t1_rd_entry[pk]   <= t_rdata[pk];
                t1_rd_tag[pk]     <= t_rdata[pk][TENTRY_W-1 -: `TAGE_TAG_W];
            end
            ghr <= {ghr[`GHR_LEN-2:0], t0_taken};
        end
    end
end

// lint 吸收
wire tage_lint = (|m_hits) | t0_mispred;

endmodule

// ------------------------------------------------------------
// tage_base_ram / tage_tag_ram：简单双口 RAM 模板（推断 BRAM）
// ------------------------------------------------------------
module tage_base_ram(
    input  wire        clk,
    input  wire [12:0] raddr,
    output reg  [1:0]  rdata,
    input  wire        we,
    input  wire [12:0] waddr,
    input  wire [1:0]  wdata
);
reg [1:0] mem [0:`TAGE_BASE_DEPTH-1];
integer i;
initial begin
    for (i = 0; i < `TAGE_BASE_DEPTH; i = i + 1) mem[i] = 2'b01;  // 弱不跳
end
always @(posedge clk) begin
    rdata <= mem[raddr];
    if (we) mem[waddr] <= wdata;
end
endmodule

module tage_tag_ram(
    input  wire        clk,
    input  wire [9:0]  raddr,
    output reg  [16:0] rdata,
    input  wire        we,
    input  wire [9:0]  waddr,
    input  wire [16:0] wdata
);
reg [16:0] mem [0:`TAGE_TAG_DEPTH-1];
integer i;
initial begin
    for (i = 0; i < `TAGE_TAG_DEPTH; i = i + 1) mem[i] = 17'b0;
end
always @(posedge clk) begin
    rdata <= mem[raddr];
    if (we) mem[waddr] <= wdata;
end
endmodule