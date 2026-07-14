// ============================================================
// ftb 模块（Fetch Target Buffer，取指目标缓冲）
// ------------------------------------------------------------
// 参考实现说明：
// - 4 路 × 1024 组，推断 BRAM（1R+1W 简单双口），查询 1 拍延迟；
// - 条目 {valid, tag(20), br_type(2), len(3), target(32)}；
//   fall_through 不存全宽：由 len 重建（= 块PC + 4*len）；
// - 更新走内部 2 级小流水：U0 借用查询读口读出组内 4 路（该拍查询作废，
//   预测器允许偶发 miss），U1 比较命中路原地更新 / victim 轮转分配；
// - 复位逐组清 valid（1024 拍）。
// ============================================================
`include "mycpu.h"

module ftb(
    input  wire                       clk,
    input  wire                       reset,

    // ---------------- 查询口（BRAM，1 拍延迟）----------------
    input  wire                       query_valid_i,
    input  wire [31:0]                query_pc_i,          // 块起始 PC

    output wire                       hit_o,               // （相对查询晚 1 拍）
    output wire [31:0]                jump_target_o,       // 块内分支的跳转目标
    output wire [31:0]                fall_through_o,      // 块顺序出口地址（start_pc + 4*块长）
    output wire [`BR_TYPE_W-1:0]      br_type_o,           // 分支类型

    // ---------------- 更新口（提交训练）----------------
    input  wire                       update_valid_i,
    input  wire [31:0]                update_block_pc_i,   // 块起始 PC
    input  wire [31:0]                update_jump_target_i,
    input  wire [31:0]                update_fall_through_i,
    input  wire [`BR_TYPE_W-1:0]      update_br_type_i,
    input  wire                       update_alloc_i       // 1=新分配，0=仅更新
);

localparam TAGW    = 20;                          // pc[31:12]
localparam ENTRY_W = 1 + TAGW + `BR_TYPE_W + `BLK_LEN_W + 32;  // 58

// ---------------- 更新流水 U0/U1 ----------------
reg                   u0_valid;
reg [31:0]            u0_pc, u0_target, u0_ft;
reg [`BR_TYPE_W-1:0]  u0_btype;
reg                   u0_alloc;
reg                   u1_valid;
reg [31:0]            u1_pc, u1_target;
reg [`BR_TYPE_W-1:0]  u1_btype;
reg [`BLK_LEN_W-1:0]  u1_len;

wire [`FTB_INDEX_W-1:0] q_index = query_pc_i[2 +: `FTB_INDEX_W];
wire [`FTB_INDEX_W-1:0] u0_index= u0_pc[2 +: `FTB_INDEX_W];
wire [`FTB_INDEX_W-1:0] u1_index= u1_pc[2 +: `FTB_INDEX_W];

// 读口仲裁：U0 借口
wire                     rd_steal = u0_valid;
wire [`FTB_INDEX_W-1:0]  rd_index = rd_steal ? u0_index : q_index;

// ---------------- 初始化（复位逐组清 valid）----------------
reg                     initing;
reg [`FTB_INDEX_W-1:0]  init_set;

// ---------------- 4 路 BRAM ----------------
wire [ENTRY_W-1:0] way_rdata [0:`FTB_NWAY-1];
reg  [`FTB_NWAY-1:0] way_we;
reg  [ENTRY_W-1:0] way_wdata;
wire [`FTB_INDEX_W-1:0] wr_index = initing ? init_set : u1_index;

genvar g;
generate
for (g = 0; g < `FTB_NWAY; g = g + 1) begin : gen_ftb_way
    ftb_way_ram u_way(
        .clk   (clk),
        .raddr (rd_index),
        .rdata (way_rdata[g]),
        .we    (way_we[g] | initing),
        .waddr (wr_index),
        .wdata (initing ? {ENTRY_W{1'b0}} : way_wdata)
    );
end
endgenerate

// ---------------- 查询结果（晚 1 拍）----------------
reg        q_valid_r;
reg [31:0] q_pc_r;
always @(posedge clk) begin
    q_valid_r <= query_valid_i && !rd_steal && !initing;
    q_pc_r    <= query_pc_i;
end

wire [TAGW-1:0] q_tag_r = q_pc_r[31:12];
wire [`FTB_NWAY-1:0] q_hit;
generate
for (g = 0; g < `FTB_NWAY; g = g + 1) begin : gen_ftb_hit
    assign q_hit[g] = q_valid_r && way_rdata[g][ENTRY_W-1]
                   && (way_rdata[g][ENTRY_W-2 -: TAGW] == q_tag_r);
end
endgenerate

reg [1:0] q_way;
integer qi;
always @(*) begin
    q_way = 2'd0;
    for (qi = `FTB_NWAY-1; qi >= 0; qi = qi - 1)
        if (q_hit[qi]) q_way = qi[1:0];
end

wire [ENTRY_W-1:0] q_entry = way_rdata[q_way];
wire [`BLK_LEN_W-1:0] q_len = q_entry[32 +: `BLK_LEN_W];

assign hit_o          = |q_hit;
assign jump_target_o  = q_entry[31:0];
assign fall_through_o = q_pc_r + {27'b0, q_len, 2'b00};
assign br_type_o      = q_entry[32+`BLK_LEN_W +: `BR_TYPE_W];

// ---------------- 更新流水 ----------------
// U1 拍：U0 读出的 4 路与 u1 tag 比较
wire [TAGW-1:0] u1_tag = u1_pc[31:12];
wire [`FTB_NWAY-1:0] u_hit;
generate
for (g = 0; g < `FTB_NWAY; g = g + 1) begin : gen_ftb_uhit
    assign u_hit[g] = way_rdata[g][ENTRY_W-1]
                   && (way_rdata[g][ENTRY_W-2 -: TAGW] == u1_tag);
end
endgenerate

reg [1:0] u_way;
reg       u_found;
reg [1:0] u_inv_way;
reg       u_inv_found;
integer uj;
always @(*) begin
    u_found = 1'b0;  u_way = 2'd0;
    u_inv_found = 1'b0; u_inv_way = 2'd0;
    for (uj = `FTB_NWAY-1; uj >= 0; uj = uj - 1) begin
        if (u_hit[uj]) begin u_found = 1'b1; u_way = uj[1:0]; end
        if (!way_rdata[uj][ENTRY_W-1]) begin u_inv_found = 1'b1; u_inv_way = uj[1:0]; end
    end
end

reg [1:0] victim_rr;
wire [1:0] wr_way = u_found ? u_way : u_inv_found ? u_inv_way : victim_rr;

always @(*) begin
    way_we    = {`FTB_NWAY{1'b0}};
    way_wdata = {1'b1, u1_tag, u1_btype, u1_len, u1_target};
    if (u1_valid) way_we[wr_way] = 1'b1;
end

always @(posedge clk) begin
    if (reset) begin
        u0_valid  <= 1'b0;
        u1_valid  <= 1'b0;
        victim_rr <= 2'd0;
        initing   <= 1'b1;
        init_set  <= {`FTB_INDEX_W{1'b0}};
    end else if (initing) begin
        init_set <= init_set + 1'b1;
        if (init_set == {`FTB_INDEX_W{1'b1}}) initing <= 1'b0;
    end else begin
        // U0：捕获更新请求（借读口）
        u0_valid <= update_valid_i;
        if (update_valid_i) begin
            u0_pc     <= update_block_pc_i;
            u0_target <= update_jump_target_i;
            u0_ft     <= update_fall_through_i;
            u0_btype  <= update_br_type_i;
            u0_alloc  <= update_alloc_i;
        end
        // U1：写入
        u1_valid <= u0_valid;
        if (u0_valid) begin
            u1_pc     <= u0_pc;
            u1_target <= u0_target;
            u1_btype  <= u0_btype;
            u1_len    <= (u0_ft - u0_pc) >> 2;      // 块长（1~4）
        end
        if (u1_valid && !u_found && !u_inv_found)
            victim_rr <= victim_rr + 2'd1;
    end
end

// lint 吸收（alloc 标志当前未区分语义：命中即原地更新，未命中即分配）
wire ftb_lint = u0_alloc;

endmodule

// ------------------------------------------------------------
// ftb_way_ram：简单双口 RAM 模板（1R + 1W，推断 BRAM）
// ------------------------------------------------------------
module ftb_way_ram(
    input  wire                      clk,
    input  wire [`FTB_INDEX_W-1:0]   raddr,
    output reg  [57:0]               rdata,
    input  wire                      we,
    input  wire [`FTB_INDEX_W-1:0]   waddr,
    input  wire [57:0]               wdata
);
reg [57:0] mem [0:`FTB_NSET-1];
always @(posedge clk) begin
    rdata <= mem[raddr];
    if (we) mem[waddr] <= wdata;
end
endmodule