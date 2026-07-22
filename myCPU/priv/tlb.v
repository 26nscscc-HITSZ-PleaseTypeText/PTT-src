`include "mycpu.h"

// ============================================================
// tlb 模块（主 TLB：TLBNUM 项全相联，双查找口 + 读写/无效化口）
// ------------------------------------------------------------
// 功能（新架构下完全复用，端口不变，已通过 79 个功能点验证）：
// - s0 口：取指翻译查询（经 tlb_manager 内 I 侧 l1_tlb 微表转发）
// - s1 口：访存翻译查询（经 D 侧 l1_tlb 微表转发）
// - srch 口（V2.3 提频新增）：tlbsrch 专用查找口，输入直接来自
//   CSR.TLBEHI/CSR.ASID 寄存器，只输出 found/index（体系结构语义，
//   不读页属性）。tlbsrch 不再挪用 s0 口，commit 提交逻辑从
//   「s0 mux → 全相联比较 → 翻译 → 取指」关键路径上整体摘除；
// - 读写口/INVTLB：由 tlb_manager 在 commit 提交 TLB 维护指令时驱动
//   （提交拍一拍脉冲，伴随 FLUSH_REFETCH 冲刷，无在途查询竞争）
// - 双页结构（odd/even ppn）：一个表项覆盖相邻两页，写入时同时给出
//   两页的 {ppn, plv, mat, d, v}，查找按 va 的页内奇偶位选边
//
// 参数：TLBNUM 表项数（模块默认 16，顶层 core_top 例化为 32）。
//
// 二期优化落点说明：
// 1) TLBNUM 项全相联比较链是时序热点——已按优先方案在 tlb_manager 里加
//    l1_tlb 微表缓存（8 项组合命中 + fence 整表失效）；
//    微表只缓存 4KB 页翻译，大页恒走本模块透传。
// 2) 大页（PS!=12）匹配完整性（跑 Linux 前已核对）：
//    - match：PS=12 时全 19 位 vppn 精确比较；PS!=12（4MB 大页）时只比较
//      vppn[18:9]（即 va[31:22]），低位属于大页页内偏移，正确；
//    - 奇偶页选择：PS=12 用 va_bit12（va[12]），大页用 vppn[8]（va[21]），
//      与 tlb_manager 的 paddr 拼接规则 {ppn[19:10], va[21:0]} 配套；
//    - INVTLB 的 VPN 匹配（inv_cond_vpn）与查找口同规则，大页同样只比
//      高 10 位，不会漏无效化。
// ============================================================
module tlb
#(
    parameter TLBNUM = 16
)
(
    input  wire                         clk,
    input  wire                         reset,

    // 查找端口0（用于取指）
    input  wire [18:0]                  s0_vppn,
    input  wire                         s0_va_bit12,
    input  wire [9:0]                   s0_asid,
    output wire                         s0_found,
    output wire [$clog2(TLBNUM)-1:0]    s0_index,
    output wire [19:0]                  s0_ppn,
    output wire [5:0]                   s0_ps,
    output wire [1:0]                   s0_plv,
    output wire [1:0]                   s0_mat,
    output wire                         s0_d,
    output wire                         s0_v,

    // 查找端口1（用于访存 load/store）
    input  wire [18:0]                  s1_vppn,
    input  wire                         s1_va_bit12,
    input  wire [9:0]                   s1_asid,
    output wire                         s1_found,
    output wire [$clog2(TLBNUM)-1:0]    s1_index,
    output wire [19:0]                  s1_ppn,
    output wire [5:0]                   s1_ps,
    output wire [1:0]                   s1_plv,
    output wire [1:0]                   s1_mat,
    output wire                         s1_d,
    output wire                         s1_v,

    // tlbsrch 专用查找口（输入恒为 CSR.TLBEHI/ASID 寄存器，输出仅 found/index；
    // 匹配规则与 s0/s1 完全一致：E 有效 + 按 PS 比较 vppn + (ASID 相符或 G)）
    input  wire [18:0]                  srch_vppn,
    input  wire [9:0]                   srch_asid,
    output wire                         srch_found,
    output wire [$clog2(TLBNUM)-1:0]    srch_index,

    // INVTLB 指令接口（用于 TLB 无效化）
    input  wire                         invtlb_valid,
    input  wire [4:0]                   invtlb_op,
    input  wire [9:0]                   invtlb_asid,      // 【新增】独立ASID参数
    input  wire [18:0]                  invtlb_vpn,       // 【新增】独立VPN参数

    // 写端口
    input  wire                         we,
    input  wire [$clog2(TLBNUM)-1:0]    w_index,
    input  wire                         w_e,
    input  wire [18:0]                  w_vppn,
    input  wire [5:0]                   w_ps,
    input  wire [9:0]                   w_asid,
    input  wire                         w_g,
    input  wire [19:0]                  w_ppn0,
    input  wire [1:0]                   w_plv0,
    input  wire [1:0]                   w_mat0,
    input  wire                         w_d0,
    input  wire                         w_v0,
    input  wire [19:0]                  w_ppn1,
    input  wire [1:0]                   w_plv1,
    input  wire [1:0]                   w_mat1,
    input  wire                         w_d1,
    input  wire                         w_v1,

    // 读端口
    input  wire [$clog2(TLBNUM)-1:0]    r_index,
    output wire                         r_e,
    output wire [18:0]                  r_vppn,
    output wire [5:0]                   r_ps,
    output wire [9:0]                   r_asid,
    output wire                         r_g,
    output wire [19:0]                  r_ppn0,
    output wire [1:0]                   r_plv0,
    output wire [1:0]                   r_mat0,
    output wire                         r_d0,
    output wire                         r_v0,
    output wire [19:0]                  r_ppn1,
    output wire [1:0]                   r_plv1,
    output wire [1:0]                   r_mat1,
    output wire                         r_d1,
    output wire                         r_v1
);

localparam IDXW   = $clog2(TLBNUM);
localparam PS_4KB = 6'd12;
localparam [TLBNUM-1:0] ALL_ONE = {TLBNUM{1'b1}};

reg [18:0] tlb_vppn [TLBNUM-1:0];
reg        tlb_e    [TLBNUM-1:0];
reg [9:0]  tlb_asid [TLBNUM-1:0];
reg        tlb_g    [TLBNUM-1:0];
reg [5:0]  tlb_ps   [TLBNUM-1:0];
reg [19:0] tlb_ppn0 [TLBNUM-1:0];
reg [1:0]  tlb_plv0 [TLBNUM-1:0];
reg [1:0]  tlb_mat0 [TLBNUM-1:0];
reg        tlb_d0   [TLBNUM-1:0];
reg        tlb_v0   [TLBNUM-1:0];
reg [19:0] tlb_ppn1 [TLBNUM-1:0];
reg [1:0]  tlb_plv1 [TLBNUM-1:0];
reg [1:0]  tlb_mat1 [TLBNUM-1:0];
reg        tlb_d1   [TLBNUM-1:0];
reg        tlb_v1   [TLBNUM-1:0];

wire [TLBNUM-1:0] match0;
wire [TLBNUM-1:0] match1;
wire [TLBNUM-1:0] match_srch;
wire [TLBNUM-1:0] s0_odd_page_hit;
wire [TLBNUM-1:0] s1_odd_page_hit;

wire [TLBNUM-1:0] inv_cond_nonglobal;
wire [TLBNUM-1:0] inv_cond_global;
wire [TLBNUM-1:0] inv_cond_asid;
wire [TLBNUM-1:0] inv_cond_vpn;
wire [TLBNUM-1:0] inv_match;

reg [IDXW-1:0] s0_index_r;
reg [IDXW-1:0] s1_index_r;
reg            s0_index_hit;
reg            s1_index_hit;

wire s0_sel_odd;
wire s1_sel_odd;

genvar i;
generate
    for (i = 0; i < TLBNUM; i = i + 1) begin: gen_tlb_lookup
        assign s0_odd_page_hit[i] = (tlb_ps[i] == PS_4KB) ? s0_va_bit12 : s0_vppn[8];
        assign s1_odd_page_hit[i] = (tlb_ps[i] == PS_4KB) ? s1_va_bit12 : s1_vppn[8];

        assign match0[i] = ((tlb_ps[i] == PS_4KB) ? (s0_vppn == tlb_vppn[i]) : (s0_vppn[18:9] == tlb_vppn[i][18:9])) &&
                           ((s0_asid == tlb_asid[i]) || tlb_g[i]) &&
                           tlb_e[i];
        assign match1[i] = ((tlb_ps[i] == PS_4KB) ? (s1_vppn == tlb_vppn[i]) : (s1_vppn[18:9] == tlb_vppn[i][18:9])) &&
                           ((s1_asid == tlb_asid[i]) || tlb_g[i]) &&
                           tlb_e[i];
        assign match_srch[i] = ((tlb_ps[i] == PS_4KB) ? (srch_vppn == tlb_vppn[i]) : (srch_vppn[18:9] == tlb_vppn[i][18:9])) &&
                           ((srch_asid == tlb_asid[i]) || tlb_g[i]) &&
                           tlb_e[i];

        assign inv_cond_nonglobal[i] = ~tlb_g[i];
        assign inv_cond_global[i]    =  tlb_g[i];
        assign inv_cond_asid[i]      = (invtlb_asid == tlb_asid[i]);  // 【修正】使用独立ASID参数
        // 【修正】INVTLB的VPN匹配使用独立的invtlb_vpn参数
        assign inv_cond_vpn[i]       = (tlb_ps[i] == PS_4KB) ? (invtlb_vpn == tlb_vppn[i]) : (invtlb_vpn[18:9] == tlb_vppn[i][18:9]);
    end
endgenerate

assign inv_match = ((invtlb_op == 5'h0) || (invtlb_op == 5'h1)) ? ALL_ONE :
                   (invtlb_op == 5'h2)                          ? inv_cond_global :
                   (invtlb_op == 5'h3)                          ? inv_cond_nonglobal :
                   (invtlb_op == 5'h4)                          ? (inv_cond_nonglobal & inv_cond_asid) :
                   (invtlb_op == 5'h5)                          ? (inv_cond_nonglobal & inv_cond_asid & inv_cond_vpn) :
                   (invtlb_op == 5'h6)                          ? ((inv_cond_global | inv_cond_asid) & inv_cond_vpn) :
                                                                   {TLBNUM{1'b0}};

assign s0_found = |match0;
assign s1_found = |match1;

integer idx0;
always @(*) begin
    s0_index_r   = {IDXW{1'b0}};
    s0_index_hit = 1'b0;
    for (idx0 = 0; idx0 < TLBNUM; idx0 = idx0 + 1) begin
        if (match0[idx0] && !s0_index_hit) begin
            s0_index_r   = idx0[IDXW-1:0];
            s0_index_hit = 1'b1;
        end
    end
end

integer idx1;
always @(*) begin
    s1_index_r   = {IDXW{1'b0}};
    s1_index_hit = 1'b0;
    for (idx1 = 0; idx1 < TLBNUM; idx1 = idx1 + 1) begin
        if (match1[idx1] && !s1_index_hit) begin
            s1_index_r   = idx1[IDXW-1:0];
            s1_index_hit = 1'b1;
        end
    end
end

// srch 口 index 编码（与 s0/s1 相同的最低命中项优先规则）
reg [IDXW-1:0] srch_index_r;
reg            srch_index_hit;
integer idxs;
always @(*) begin
    srch_index_r   = {IDXW{1'b0}};
    srch_index_hit = 1'b0;
    for (idxs = 0; idxs < TLBNUM; idxs = idxs + 1) begin
        if (match_srch[idxs] && !srch_index_hit) begin
            srch_index_r   = idxs[IDXW-1:0];
            srch_index_hit = 1'b1;
        end
    end
end

assign s0_index = s0_index_r;
assign s1_index = s1_index_r;
assign srch_found = |match_srch;
assign srch_index = srch_index_r;

assign s0_sel_odd = s0_odd_page_hit[s0_index_r];
assign s1_sel_odd = s1_odd_page_hit[s1_index_r];

assign s0_ps  = s0_found ? tlb_ps[s0_index_r] : 6'b0;
assign s0_ppn = s0_found ? (s0_sel_odd ? tlb_ppn1[s0_index_r] : tlb_ppn0[s0_index_r]) : 20'b0;
assign s0_plv = s0_found ? (s0_sel_odd ? tlb_plv1[s0_index_r] : tlb_plv0[s0_index_r]) : 2'b0;
assign s0_mat = s0_found ? (s0_sel_odd ? tlb_mat1[s0_index_r] : tlb_mat0[s0_index_r]) : 2'b0;
assign s0_d   = s0_found ? (s0_sel_odd ? tlb_d1[s0_index_r]   : tlb_d0[s0_index_r])   : 1'b0;
assign s0_v   = s0_found ? (s0_sel_odd ? tlb_v1[s0_index_r]   : tlb_v0[s0_index_r])   : 1'b0;

assign s1_ps  = s1_found ? tlb_ps[s1_index_r] : 6'b0;
assign s1_ppn = s1_found ? (s1_sel_odd ? tlb_ppn1[s1_index_r] : tlb_ppn0[s1_index_r]) : 20'b0;
assign s1_plv = s1_found ? (s1_sel_odd ? tlb_plv1[s1_index_r] : tlb_plv0[s1_index_r]) : 2'b0;
assign s1_mat = s1_found ? (s1_sel_odd ? tlb_mat1[s1_index_r] : tlb_mat0[s1_index_r]) : 2'b0;
assign s1_d   = s1_found ? (s1_sel_odd ? tlb_d1[s1_index_r]   : tlb_d0[s1_index_r])   : 1'b0;
assign s1_v   = s1_found ? (s1_sel_odd ? tlb_v1[s1_index_r]   : tlb_v0[s1_index_r])   : 1'b0;

assign r_e    = tlb_e[r_index];
assign r_vppn = tlb_vppn[r_index];
assign r_ps   = tlb_ps[r_index];
assign r_asid = tlb_asid[r_index];
assign r_g    = tlb_g[r_index];
assign r_ppn0 = tlb_ppn0[r_index];
assign r_plv0 = tlb_plv0[r_index];
assign r_mat0 = tlb_mat0[r_index];
assign r_d0   = tlb_d0[r_index];
assign r_v0   = tlb_v0[r_index];
assign r_ppn1 = tlb_ppn1[r_index];
assign r_plv1 = tlb_plv1[r_index];
assign r_mat1 = tlb_mat1[r_index];
assign r_d1   = tlb_d1[r_index];
assign r_v1   = tlb_v1[r_index];

integer widx;
always @(posedge clk) begin
    if (reset) begin
        for (widx = 0; widx < TLBNUM; widx = widx + 1) begin
            tlb_vppn[widx] = 19'b0;
            tlb_e[widx]    = 1'b0;
            tlb_asid[widx] = 10'b0;
            tlb_g[widx]    = 1'b0;
            tlb_ps[widx]   = PS_4KB;
            tlb_ppn0[widx] = 20'b0;
            tlb_plv0[widx] = 2'b0;
            tlb_mat0[widx] = 2'b0;
            tlb_d0[widx]   = 1'b0;
            tlb_v0[widx]   = 1'b0;
            tlb_ppn1[widx] = 20'b0;
            tlb_plv1[widx] = 2'b0;
            tlb_mat1[widx] = 2'b0;
            tlb_d1[widx]   = 1'b0;
            tlb_v1[widx]   = 1'b0;
        end
    end else begin
        if (we) begin
            tlb_vppn[w_index] <= w_vppn;
            tlb_e[w_index]    <= w_e;
            tlb_asid[w_index] <= w_asid;
            tlb_g[w_index]    <= w_g;
            tlb_ps[w_index]   <= w_ps;
            tlb_ppn0[w_index] <= w_ppn0;
            tlb_plv0[w_index] <= w_plv0;
            tlb_mat0[w_index] <= w_mat0;
            tlb_d0[w_index]   <= w_d0;
            tlb_v0[w_index]   <= w_v0;
            tlb_ppn1[w_index] <= w_ppn1;
            tlb_plv1[w_index] <= w_plv1;
            tlb_mat1[w_index] <= w_mat1;
            tlb_d1[w_index]   <= w_d1;
            tlb_v1[w_index]   <= w_v1;
        end

        if (invtlb_valid) begin
            for (widx = 0; widx < TLBNUM; widx = widx + 1) begin
                if (!((we == 1'b1) && (w_index == widx[IDXW-1:0])) && inv_match[widx]) begin
                    tlb_e[widx] = 1'b0;
                end
            end
        end
    end
end

endmodule
