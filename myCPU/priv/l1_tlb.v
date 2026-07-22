// ============================================================
// l1_tlb 模块（L1 微型 TLB，I 侧 / D 侧各例化一份）
// ------------------------------------------------------------
// 功能：
// - 主 TLB（tlb.v，TLBNUM 项全相联，顶层例化 32 项）的小容量缓存
//   （ENTRY_NUM=8 项），目的：
//   主 TLB 全相联比较链很长，是取指/访存关键路径的时序瓶颈；
//   L1 微表只比较 8 项，命中时组合路径显著缩短（mariver 实证：L1 仅 3 项
//   已有可观收益）。
// - 对软件完全透明（体系结构只看得到主 TLB）；本表只是"翻译结果缓存"。
// - 工作方式：
//   * 查询：组合比较 vaddr[31:12]（4KB 页号粒度，含 va_bit12 奇偶页）与各
//     有效项，命中当拍给出缓存的 {ppn, mat, v, d, plv}，ps 恒为 4KB；
//   * 未命中：查询透传主 TLB（主表查找本就是组合的），结果原样返回；
//     同拍主表命中且为 4KB 页时，posedge 把结果回填进本表（FIFO 替换）；
//   * 大页项（ps!=12）不缓存，一律走透传 —— 大页的 ppn/偏移拼接位数随 ps
//     变化，缓存展开不值得（Linux 大页占比极低）；
//   * fence_i：整表一拍失效（TLB 写/无效化/ASID 切换后旧翻译可能失效，
//     一刀切最简单且绝对安全——失效频率极低，性能无损）。
//     （V2.3 起 tlbsrch 走主表专用 srch 口，s0/s1 口不再被挪用，
//      原 dis_refill 屏蔽机制随之删除。）
//
// 坑点（保留框架提示，均已按此实现）：
// 1. 本表只是缓存，任何"主表内容可能变化"的时刻（fence_i）必须失效，
//    宁可多失效。
// 2. 取指口与访存口各例化一份本模块（I 侧连 tlb.v 的 s0 口、D 侧连 s1 口），
//    两份互不相干。
//
// 端口：
// - 上游（tlb_manager 翻译通路）：req/vaddr 进，found/ppn/页属性 出（组合）
// - 下游（主 TLB s0/s1 口）：查询恒透传（命中拍透传结果不被使用，无副作用）
// - fence_i：维护
// ============================================================
`include "mycpu.h"

module l1_tlb #(
    parameter ENTRY_NUM = 8          // 微表项数（4~8，时序与命中率折中）
)(
    input  wire          clk,
    input  wire          reset,

    // ---------------- 维护 ----------------
    input  wire          fence_i,          // TLB 维护指令提交（tlbwr/tlbfill/invtlb/asid 变化）-> 整表失效

    // ---------------- 上游查询口（tlb_manager，组合）----------------
    input  wire          req_valid_i,
    input  wire [31:0]   vaddr_i,
    output wire          found_o,          // 翻译命中（本表命中 或 主表命中）
    output wire [19:0]   ppn_o,            // 物理页号（4KB 页：paddr = {ppn, vaddr[11:0]}）
    output wire [5:0]    ps_o,             // 有效页大小（本表命中恒 4KB；透传时跟随主表）
    output wire [1:0]    mat_o,            // 页存储访问类型
    output wire          v_o,              // 页有效位
    output wire          d_o,              // 页脏位
    output wire [1:0]    plv_o,            // 页特权等级

    // ---------------- 下游主 TLB 查询口（连 tlb.v 的 s0 或 s1 口）----------------
    output wire [18:0]   tlb_vppn_o,       // 主表查询 vppn（vaddr[31:13]）
    output wire          tlb_va_bit12_o,   // vaddr[12]（双页选择）
    input  wire          tlb_found_i,
    input  wire [19:0]   tlb_ppn_i,
    input  wire [5:0]    tlb_ps_i,         // 主表命中项页大小（回填过滤大页用）
    input  wire [1:0]    tlb_mat_i,
    input  wire          tlb_v_i,
    input  wire          tlb_d_i,
    input  wire [1:0]    tlb_plv_i
);

localparam PS_4KB  = 6'd12;
localparam PTR_W   = $clog2(ENTRY_NUM);

// ------------------------------------------------------------
// 存储：ENTRY_NUM 项 {valid, key=vaddr[31:12], ppn, mat, v, d, plv}
// key 按 4KB 页号（含奇偶页位）——主表返回的属性已经按 va_bit12 选好页，
// 因此本表以单页为粒度缓存，不需要保存双页结构。
// ------------------------------------------------------------
reg [ENTRY_NUM-1:0] e_valid;
reg [19:0]          e_key  [0:ENTRY_NUM-1];   // vaddr[31:12]
reg [19:0]          e_ppn  [0:ENTRY_NUM-1];
reg [1:0]           e_mat  [0:ENTRY_NUM-1];
reg                 e_v    [0:ENTRY_NUM-1];
reg                 e_d    [0:ENTRY_NUM-1];
reg [1:0]           e_plv  [0:ENTRY_NUM-1];
reg [PTR_W-1:0]     fifo_ptr;                 // FIFO 替换指针

wire [19:0] q_key = vaddr_i[31:12];

// ---------------- 查询（组合，8 项并行比较）----------------
wire [ENTRY_NUM-1:0] e_hit;
genvar gi;
generate
for (gi = 0; gi < ENTRY_NUM; gi = gi + 1) begin : gen_l1_hit
    assign e_hit[gi] = e_valid[gi] && (e_key[gi] == q_key);
end
endgenerate

wire l1_hit = (req_valid_i === 1'b1) && (|e_hit);

// 命中项属性选择（独热 hit 向量按位与-或归并；填表逻辑保证同键至多一项）
integer si;
reg [19:0] hit_ppn;
reg [1:0]  hit_mat;
reg        hit_v;
reg        hit_d;
reg [1:0]  hit_plv;
always @(*) begin
    hit_ppn = 20'b0;
    hit_mat = 2'b0;
    hit_v   = 1'b0;
    hit_d   = 1'b0;
    hit_plv = 2'b0;
    for (si = 0; si < ENTRY_NUM; si = si + 1) begin
        if (e_hit[si]) begin
            hit_ppn = hit_ppn | e_ppn[si];
            hit_mat = hit_mat | e_mat[si];
            hit_v   = hit_v   | e_v[si];
            hit_d   = hit_d   | e_d[si];
            hit_plv = hit_plv | e_plv[si];
        end
    end
end

// ---------------- 下游透传（查询恒转发，命中拍转发结果不被使用）----------------
assign tlb_vppn_o     = vaddr_i[31:13];
assign tlb_va_bit12_o = vaddr_i[12];

// ---------------- 上游输出（命中走本表，否则透传主表）----------------
assign found_o = l1_hit ? 1'b1 : ((req_valid_i === 1'b1) && (tlb_found_i === 1'b1));
assign ppn_o   = l1_hit ? hit_ppn : tlb_ppn_i;
assign ps_o    = l1_hit ? PS_4KB  : tlb_ps_i;
assign mat_o   = l1_hit ? hit_mat : tlb_mat_i;
assign v_o     = l1_hit ? hit_v   : tlb_v_i;
assign d_o     = l1_hit ? hit_d   : tlb_d_i;
assign plv_o   = l1_hit ? hit_plv : tlb_plv_i;

// ---------------- 回填（FIFO 替换）----------------
// 条件：真实查询 miss + 主表命中 + 4KB 页 + 未被 fence 屏蔽。
// 回填在 posedge 落表，下一拍同地址查询即命中——同地址连续查询不会重复回填。
wire refill_en = (req_valid_i === 1'b1) && !l1_hit
              && (tlb_found_i === 1'b1) && (tlb_ps_i == PS_4KB)
              && (fence_i !== 1'b1);

integer ri;
always @(posedge clk) begin
    if (reset) begin
        e_valid  <= {ENTRY_NUM{1'b0}};
        fifo_ptr <= {PTR_W{1'b0}};
        for (ri = 0; ri < ENTRY_NUM; ri = ri + 1) begin
            e_key[ri] <= 20'b0;
            e_ppn[ri] <= 20'b0;
            e_mat[ri] <= 2'b0;
            e_v[ri]   <= 1'b0;
            e_d[ri]   <= 1'b0;
            e_plv[ri] <= 2'b0;
        end
    end else if (fence_i) begin
        // 整表失效：主表内容/ASID 已变，旧翻译一律作废（宁可多失效）
        e_valid  <= {ENTRY_NUM{1'b0}};
        fifo_ptr <= {PTR_W{1'b0}};
    end else if (refill_en) begin
        e_valid[fifo_ptr] <= 1'b1;
        e_key[fifo_ptr]   <= q_key;
        e_ppn[fifo_ptr]   <= tlb_ppn_i;
        e_mat[fifo_ptr]   <= tlb_mat_i;
        e_v[fifo_ptr]     <= tlb_v_i;
        e_d[fifo_ptr]     <= tlb_d_i;
        e_plv[fifo_ptr]   <= tlb_plv_i;
        fifo_ptr          <= fifo_ptr + {{(PTR_W-1){1'b0}}, 1'b1};
    end
end

endmodule
