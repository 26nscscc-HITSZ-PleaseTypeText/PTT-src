`include "mycpu.h"

// ============================================================
// tlb 模块（主 TLB：TLBNUM 项全相联，双查找口 + 读写/无效化口）
// ------------------------------------------------------------
// 功能：
// - s0 口：取指翻译查询（经 tlb_manager 内 I 侧 l1_tlb 微表转发）
// - s1 口：访存翻译查询（经 D 侧 l1_tlb 微表转发）
// - srch 口：tlbsrch 专用查找口，输入来自 CSR.TLBEHI/CSR.ASID，只输出
//   found/index，不读取页属性，也不占用 s0 取指查询口；
// - 读写口/INVTLB：由 tlb_manager 在 commit 提交 TLB 维护指令时驱动
//   （提交拍一拍脉冲，伴随 FLUSH_REFETCH 冲刷，无在途查询竞争）
// - 双页结构（odd/even ppn）：一个表项覆盖相邻两页，写入时同时给出
//   两页的 {ppn, plv, mat, d, v}，查找按 va 的页内奇偶位选边
//
// 参数：TLBNUM 表项数（模块默认 16，顶层 core_top 例化为 32）。
//
// 大页规则：
// - tlb_manager 中的 I/D L1 微表只缓存 4KB 页，大页始终查询本模块；
// - PS!=12 时按 4MB 页处理：
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
    input  wire [9:0]                   invtlb_asid,
    input  wire [18:0]                  invtlb_vpn,

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

`ifdef SOC_USE
reg [IDXW-1:0] s0_index_r;
reg            s0_index_hit;
`else
`endif
reg [IDXW-1:0] s1_index_r;
reg            s1_index_hit;
localparam GROUP_NUM = 4;
localparam GROUP_SIZE = TLBNUM / GROUP_NUM;
localparam GROUP_IDX_W = $clog2(GROUP_SIZE);
reg [GROUP_NUM-1:0] s0_group_hit;
reg [GROUP_NUM-1:0] s1_group_hit;
reg [GROUP_NUM-1:0] srch_group_hit;
reg [GROUP_IDX_W-1:0] s0_group_idx [0:GROUP_NUM-1];
reg [GROUP_IDX_W-1:0] s1_group_idx [0:GROUP_NUM-1];
reg [GROUP_IDX_W-1:0] srch_group_idx [0:GROUP_NUM-1];

wire [5:0]  s0_group_ps  [0:GROUP_NUM-1];
wire [19:0] s0_group_ppn [0:GROUP_NUM-1];
wire [1:0]  s0_group_plv [0:GROUP_NUM-1];
wire [1:0]  s0_group_mat [0:GROUP_NUM-1];
wire        s0_group_d   [0:GROUP_NUM-1];
wire        s0_group_v   [0:GROUP_NUM-1];
wire        s0_group_odd [0:GROUP_NUM-1];
`ifdef SOC_USE
`else
wire [5:0]  s1_group_ps  [0:GROUP_NUM-1];
wire [19:0] s1_group_ppn [0:GROUP_NUM-1];
wire [1:0]  s1_group_plv [0:GROUP_NUM-1];
wire [1:0]  s1_group_mat [0:GROUP_NUM-1];
wire        s1_group_d   [0:GROUP_NUM-1];
wire        s1_group_v   [0:GROUP_NUM-1];
wire        s1_group_odd [0:GROUP_NUM-1];
`endif

`ifdef SOC_USE
wire s0_sel_odd;
`else
`endif
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

// Select each lookup payload locally inside its eight-entry priority group,
// then choose the lowest matching group. This avoids encoding a global index
// only to decode it through a 32-entry payload mux.
genvar sg;
generate
    for (sg = 0; sg < GROUP_NUM; sg = sg + 1) begin: gen_group_payload
        wire [IDXW-1:0] s0_selected_idx = sg*GROUP_SIZE + s0_group_idx[sg];
`ifdef SOC_USE
`else
        wire [IDXW-1:0] s1_selected_idx = sg*GROUP_SIZE + s1_group_idx[sg];
`endif
        assign s0_group_odd[sg] = s0_odd_page_hit[s0_selected_idx];
        assign s0_group_ps[sg]  = tlb_ps[s0_selected_idx];
        assign s0_group_ppn[sg] = s0_group_odd[sg]
                                 ? tlb_ppn1[s0_selected_idx] : tlb_ppn0[s0_selected_idx];
        assign s0_group_plv[sg] = s0_group_odd[sg]
                                 ? tlb_plv1[s0_selected_idx] : tlb_plv0[s0_selected_idx];
        assign s0_group_mat[sg] = s0_group_odd[sg]
                                 ? tlb_mat1[s0_selected_idx] : tlb_mat0[s0_selected_idx];
        assign s0_group_d[sg]   = s0_group_odd[sg]
                                 ? tlb_d1[s0_selected_idx] : tlb_d0[s0_selected_idx];
        assign s0_group_v[sg]   = s0_group_odd[sg]
                                 ? tlb_v1[s0_selected_idx] : tlb_v0[s0_selected_idx];
`ifdef SOC_USE
`else
        assign s1_group_odd[sg] = s1_odd_page_hit[s1_selected_idx];
        assign s1_group_ps[sg]  = tlb_ps[s1_selected_idx];
        assign s1_group_ppn[sg] = s1_group_odd[sg]
                                 ? tlb_ppn1[s1_selected_idx] : tlb_ppn0[s1_selected_idx];
        assign s1_group_plv[sg] = s1_group_odd[sg]
                                 ? tlb_plv1[s1_selected_idx] : tlb_plv0[s1_selected_idx];
        assign s1_group_mat[sg] = s1_group_odd[sg]
                                 ? tlb_mat1[s1_selected_idx] : tlb_mat0[s1_selected_idx];
        assign s1_group_d[sg]   = s1_group_odd[sg]
                                 ? tlb_d1[s1_selected_idx] : tlb_d0[s1_selected_idx];
        assign s1_group_v[sg]   = s1_group_odd[sg]
                                 ? tlb_v1[s1_selected_idx] : tlb_v0[s1_selected_idx];
`endif
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

// Two-level exact priority encoder.  Each quarter is encoded independently,
// then the lowest matching quarter wins.  This preserves the original
// lowest-index rule while avoiding a TLBNUM-entry serial priority chain.
integer grp0;
integer ent0;
always @(*) begin
    s0_group_hit = {GROUP_NUM{1'b0}};
    for (grp0 = 0; grp0 < GROUP_NUM; grp0 = grp0 + 1) begin
        s0_group_idx[grp0] = {GROUP_IDX_W{1'b0}};
        for (ent0 = 0; ent0 < GROUP_SIZE; ent0 = ent0 + 1) begin
            if (match0[grp0*GROUP_SIZE+ent0] && !s0_group_hit[grp0]) begin
                s0_group_idx[grp0] = ent0[GROUP_IDX_W-1:0];
                s0_group_hit[grp0] = 1'b1;
            end
        end
    end
end

integer grp1;
integer ent1;
always @(*) begin
    s1_group_hit = {GROUP_NUM{1'b0}};
    for (grp1 = 0; grp1 < GROUP_NUM; grp1 = grp1 + 1) begin
        s1_group_idx[grp1] = {GROUP_IDX_W{1'b0}};
        for (ent1 = 0; ent1 < GROUP_SIZE; ent1 = ent1 + 1) begin
            if (match1[grp1*GROUP_SIZE+ent1] && !s1_group_hit[grp1]) begin
                s1_group_idx[grp1] = ent1[GROUP_IDX_W-1:0];
                s1_group_hit[grp1] = 1'b1;
            end
        end
    end
end

// srch 口 index 编码（与 s0/s1 相同的最低命中项优先规则）
reg [IDXW-1:0] srch_index_r;
reg            srch_index_hit;
integer grps;
integer ents;
always @(*) begin
    srch_group_hit = {GROUP_NUM{1'b0}};
    for (grps = 0; grps < GROUP_NUM; grps = grps + 1) begin
        srch_group_idx[grps] = {GROUP_IDX_W{1'b0}};
        for (ents = 0; ents < GROUP_SIZE; ents = ents + 1) begin
            if (match_srch[grps*GROUP_SIZE+ents] && !srch_group_hit[grps]) begin
                srch_group_idx[grps] = ents[GROUP_IDX_W-1:0];
                srch_group_hit[grps] = 1'b1;
            end
        end
    end
end

`ifdef SOC_USE
integer pick0;
`else
`endif
integer pick1;
integer picks;
always @(*) begin
`ifdef SOC_USE
    s0_index_r     = {IDXW{1'b0}};
    s0_index_hit   = 1'b0;
`else
`endif
    s1_index_r    = {IDXW{1'b0}};
    srch_index_r   = {IDXW{1'b0}};
    s1_index_hit   = 1'b0;
    srch_index_hit = 1'b0;
`ifdef SOC_USE
    for (pick0 = 0; pick0 < GROUP_NUM; pick0 = pick0 + 1) begin
        if (s0_group_hit[pick0] && !s0_index_hit) begin
            s0_index_r   = {pick0[1:0], s0_group_idx[pick0]};
            s0_index_hit = 1'b1;
        end
    end
`else
`endif
    for (pick1 = 0; pick1 < GROUP_NUM; pick1 = pick1 + 1) begin
        if (s1_group_hit[pick1] && !s1_index_hit) begin
            s1_index_r   = {pick1[1:0], s1_group_idx[pick1]};
            s1_index_hit = 1'b1;
        end
    end
    for (picks = 0; picks < GROUP_NUM; picks = picks + 1) begin
        if (srch_group_hit[picks] && !srch_index_hit) begin
            srch_index_r   = {picks[1:0], srch_group_idx[picks]};
            srch_index_hit = 1'b1;
        end
    end
end

assign srch_found = |match_srch;
assign srch_index = srch_index_r;

`ifdef SOC_USE
assign s0_sel_odd = s0_odd_page_hit[s0_index_r];
`else
`endif
assign s1_sel_odd = s1_odd_page_hit[s1_index_r];

`ifdef SOC_USE
assign s0_ps  = s0_found ? tlb_ps[s0_index_r] : 6'b0;
assign s0_ppn = s0_found ? (s0_sel_odd ? tlb_ppn1[s0_index_r] : tlb_ppn0[s0_index_r]) : 20'b0;
assign s0_plv = s0_found ? (s0_sel_odd ? tlb_plv1[s0_index_r] : tlb_plv0[s0_index_r]) : 2'b0;
assign s0_mat = s0_found ? (s0_sel_odd ? tlb_mat1[s0_index_r] : tlb_mat0[s0_index_r]) : 2'b0;
assign s0_d   = s0_found ? (s0_sel_odd ? tlb_d1[s0_index_r] : tlb_d0[s0_index_r]) : 1'b0;
assign s0_v   = s0_found ? (s0_sel_odd ? tlb_v1[s0_index_r] : tlb_v0[s0_index_r]) : 1'b0;
assign s1_ps  = s1_found ? tlb_ps[s1_index_r] : 6'b0;
assign s1_ppn = s1_found ? (s1_sel_odd ? tlb_ppn1[s1_index_r] : tlb_ppn0[s1_index_r]) : 20'b0;
assign s1_plv = s1_found ? (s1_sel_odd ? tlb_plv1[s1_index_r] : tlb_plv0[s1_index_r]) : 2'b0;
assign s1_mat = s1_found ? (s1_sel_odd ? tlb_mat1[s1_index_r] : tlb_mat0[s1_index_r]) : 2'b0;
assign s1_d   = s1_found ? (s1_sel_odd ? tlb_d1[s1_index_r] : tlb_d0[s1_index_r]) : 1'b0;
assign s1_v   = s1_found ? (s1_sel_odd ? tlb_v1[s1_index_r] : tlb_v0[s1_index_r]) : 1'b0;
`else
assign s0_ps  = s0_group_hit[0] ? s0_group_ps[0]
              : s0_group_hit[1] ? s0_group_ps[1]
              : s0_group_hit[2] ? s0_group_ps[2]
              : s0_group_hit[3] ? s0_group_ps[3] : 6'b0;
assign s0_ppn = s0_group_hit[0] ? s0_group_ppn[0]
              : s0_group_hit[1] ? s0_group_ppn[1]
              : s0_group_hit[2] ? s0_group_ppn[2]
              : s0_group_hit[3] ? s0_group_ppn[3] : 20'b0;
assign s0_plv = s0_group_hit[0] ? s0_group_plv[0]
              : s0_group_hit[1] ? s0_group_plv[1]
              : s0_group_hit[2] ? s0_group_plv[2]
              : s0_group_hit[3] ? s0_group_plv[3] : 2'b0;
assign s0_mat = s0_group_hit[0] ? s0_group_mat[0]
              : s0_group_hit[1] ? s0_group_mat[1]
              : s0_group_hit[2] ? s0_group_mat[2]
              : s0_group_hit[3] ? s0_group_mat[3] : 2'b0;
assign s0_d   = s0_group_hit[0] ? s0_group_d[0]
              : s0_group_hit[1] ? s0_group_d[1]
              : s0_group_hit[2] ? s0_group_d[2]
              : s0_group_hit[3] ? s0_group_d[3] : 1'b0;
assign s0_v   = s0_group_hit[0] ? s0_group_v[0]
              : s0_group_hit[1] ? s0_group_v[1]
              : s0_group_hit[2] ? s0_group_v[2]
              : s0_group_hit[3] ? s0_group_v[3] : 1'b0;

assign s1_ps  = s1_group_hit[0] ? s1_group_ps[0]
              : s1_group_hit[1] ? s1_group_ps[1]
              : s1_group_hit[2] ? s1_group_ps[2]
              : s1_group_hit[3] ? s1_group_ps[3] : 6'b0;
assign s1_ppn = s1_group_hit[0] ? s1_group_ppn[0]
              : s1_group_hit[1] ? s1_group_ppn[1]
              : s1_group_hit[2] ? s1_group_ppn[2]
              : s1_group_hit[3] ? s1_group_ppn[3] : 20'b0;
assign s1_plv = s1_group_hit[0] ? s1_group_plv[0]
              : s1_group_hit[1] ? s1_group_plv[1]
              : s1_group_hit[2] ? s1_group_plv[2]
              : s1_group_hit[3] ? s1_group_plv[3] : 2'b0;
assign s1_mat = s1_group_hit[0] ? s1_group_mat[0]
              : s1_group_hit[1] ? s1_group_mat[1]
              : s1_group_hit[2] ? s1_group_mat[2]
              : s1_group_hit[3] ? s1_group_mat[3] : 2'b0;
assign s1_d   = s1_group_hit[0] ? s1_group_d[0]
              : s1_group_hit[1] ? s1_group_d[1]
              : s1_group_hit[2] ? s1_group_d[2]
              : s1_group_hit[3] ? s1_group_d[3] : 1'b0;
assign s1_v   = s1_group_hit[0] ? s1_group_v[0]
              : s1_group_hit[1] ? s1_group_v[1]
              : s1_group_hit[2] ? s1_group_v[2]
              : s1_group_hit[3] ? s1_group_v[3] : 1'b0;
`endif

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
            tlb_vppn[widx] <= 19'b0;
            tlb_e[widx]    <= 1'b0;
            tlb_asid[widx] <= 10'b0;
            tlb_g[widx]    <= 1'b0;
            tlb_ps[widx]   <= PS_4KB;
            tlb_ppn0[widx] <= 20'b0;
            tlb_plv0[widx] <= 2'b0;
            tlb_mat0[widx] <= 2'b0;
            tlb_d0[widx]   <= 1'b0;
            tlb_v0[widx]   <= 1'b0;
            tlb_ppn1[widx] <= 20'b0;
            tlb_plv1[widx] <= 2'b0;
            tlb_mat1[widx] <= 2'b0;
            tlb_d1[widx]   <= 1'b0;
            tlb_v1[widx]   <= 1'b0;
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
                    tlb_e[widx] <= 1'b0;
                end
            end
        end
    end
end

endmodule
