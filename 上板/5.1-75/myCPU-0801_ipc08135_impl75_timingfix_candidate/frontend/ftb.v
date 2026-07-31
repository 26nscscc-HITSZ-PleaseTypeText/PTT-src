// ============================================================
// ftb 模块（Fetch Target Buffer，取指目标缓冲）
// ------------------------------------------------------------
// 当前结构与约束：
// - 4 路 × 2048 组，推断 BRAM（1R+1W 简单双口），查询 1 拍延迟；
// - 条目 {valid, tag(19), br_type(2), len(3), target(32)}；
//   fall_through 不存全宽：由 len 重建（= 块PC + 4*len）；
// - 更新走「训练 FIFO + 内部 2 级小流水」：训练请求先入小队列（深度
//   `FTB_UPDATE_Q_DEPTH，满则丢弃计 overflow），查询优先占读口，U0 只在
//   无查询的空闲拍出队借读口读出组内 4 路，U1 比较命中路原地更新 /
//   victim 轮转分配（查询永不被作废）；
// - 复位逐组清 valid（2048 拍）。
// ============================================================
`include "mycpu.h"

module ftb(
    input  wire                       clk,
    input  wire                       reset,

    // ---------------- 查询口（BRAM，1 拍延迟）----------------
    input  wire                       query_valid_i,
    input  wire [31:0]                query_pc_i,          // 块起始 PC

    output wire                       hit_o,               // （相对查询晚 1 拍）
    output wire                       resp_valid_o,
    output wire [1:0]                 hit_way_o,
    output wire [31:0]                jump_target_o,       // 块内分支的跳转目标
    output wire [31:0]                fall_through_o,      // 块顺序出口地址（start_pc + 4*块长）
    output wire [`BR_TYPE_W-1:0]      br_type_o,           // 分支类型

    // ---------------- 更新口（提交训练）----------------
    input  wire                       update_valid_i,
    input  wire [31:2]                update_block_pc_i,   // 块起始 PC 的字地址
    input  wire [31:0]                update_jump_target_i,
    input  wire [`BLK_LEN_W+1:2]      update_fall_through_i, // 顺序出口的块内字偏移
    input  wire [`BR_TYPE_W-1:0]      update_br_type_i
);

localparam TAGW    = 32 - 2 - `FTB_INDEX_W;        // pc[31:(2+INDEX)]；2048 组时为 19
localparam TARGET_LSB = 0;
localparam FALL_LSB   = TARGET_LSB + 32;
localparam LEN_LSB    = FALL_LSB + 32;
localparam BTYPE_LSB  = LEN_LSB + `BLK_LEN_W;
localparam ENTRY_W =
    1 + TAGW + `BR_TYPE_W + `BLK_LEN_W + 32 + 32;
localparam FTB_UPDATE_Q_DEPTH = `FTB_UPDATE_Q_DEPTH;
localparam FTB_UPDATE_Q_PTR_W =
    (FTB_UPDATE_Q_DEPTH <= 1) ? 1 : $clog2(FTB_UPDATE_Q_DEPTH);
localparam FTB_UPDATE_Q_CNT_W = $clog2(FTB_UPDATE_Q_DEPTH + 1);
localparam [FTB_UPDATE_Q_CNT_W-1:0] FTB_UPDATE_Q_DEPTH_C = FTB_UPDATE_Q_DEPTH;

// ---------------- 更新流水 U0/U1 ----------------
reg                   u0_valid;
reg [31:2]            u0_pc_word;
reg [31:0]            u0_target;
reg [`BLK_LEN_W-1:0]  u0_ft_wordoff;
reg [`BR_TYPE_W-1:0]  u0_btype;
reg                   u1_valid;
reg [31:2]            u1_pc_word;
reg [31:0]            u1_target;
reg [31:0]            u1_fall;
reg [`BR_TYPE_W-1:0]  u1_btype;
reg [`BLK_LEN_W-1:0]  u1_len;

wire [`BLK_LEN_W-1:0] u0_len =
    u0_ft_wordoff - u0_pc_word[`BLK_LEN_W+1:2];
wire [31:2] u0_fall_word =
    u0_pc_word + {{(30-`BLK_LEN_W){1'b0}}, u0_len};

wire [`FTB_INDEX_W-1:0] q_index = query_pc_i[2 +: `FTB_INDEX_W];
wire [`FTB_INDEX_W-1:0] u1_index= u1_pc_word[2 +: `FTB_INDEX_W];

// 读口仲裁：查询优先，训练请求进入小 FIFO 后在空闲周期借口
reg                     initing;
reg [`FTB_INDEX_W-1:0]  init_set;

// 训练 FIFO 载荷：强制分布式 RAM——32 项小队列若被推断成 RAMB18
// 利用率仅 ~6%，且 BRAM 读延迟约束会打断"出队拍借读口"的异步读用法
(* ram_style = "distributed" *) reg [31:2]           uq_pc_word [0:FTB_UPDATE_Q_DEPTH-1];
(* ram_style = "distributed" *) reg [31:0]           uq_target  [0:FTB_UPDATE_Q_DEPTH-1];
(* ram_style = "distributed" *) reg [`BLK_LEN_W-1:0] uq_ft_wordoff[0:FTB_UPDATE_Q_DEPTH-1];
(* ram_style = "distributed" *) reg [`BR_TYPE_W-1:0] uq_btype   [0:FTB_UPDATE_Q_DEPTH-1];
reg [FTB_UPDATE_Q_PTR_W-1:0] uq_rptr, uq_wptr;
reg [FTB_UPDATE_Q_CNT_W-1:0] uq_count;

wire update_queue_empty = (uq_count == {FTB_UPDATE_Q_CNT_W{1'b0}});
wire update_queue_full  = (uq_count == FTB_UPDATE_Q_DEPTH_C);
wire service_update     = !initing && !query_valid_i && !update_queue_empty;
wire update_accept      = !initing && update_valid_i;
wire [FTB_UPDATE_Q_PTR_W-1:0] uq_tail_ptr =
    uq_wptr - {{(FTB_UPDATE_Q_PTR_W-1){1'b0}}, 1'b1};
wire update_matches_tail =
    !update_queue_empty &&
    (uq_pc_word[uq_tail_ptr] == update_block_pc_i);
// If the queue contains only one entry and that entry is leaving now, U0
// samples its old payload on this edge.  Do not overwrite it as a "merge";
// enqueue the new request into the newly freed slot instead.
wire update_tail_is_dequeue =
    service_update &&
    (uq_count == {{(FTB_UPDATE_Q_CNT_W-1){1'b0}}, 1'b1});
wire update_merge_tail =
    update_accept && update_matches_tail && !update_tail_is_dequeue;
wire update_enqueue =
    update_accept && !update_merge_tail &&
    (!update_queue_full || service_update);
wire update_overflow =
    update_accept && !update_merge_tail &&
    update_queue_full && !service_update;
wire update_dequeue     = service_update;
wire [FTB_UPDATE_Q_CNT_W-1:0] uq_count_next =
    (update_enqueue && !update_dequeue) ? (uq_count + {{(FTB_UPDATE_Q_CNT_W-1){1'b0}},1'b1}) :
    (!update_enqueue && update_dequeue) ? (uq_count - {{(FTB_UPDATE_Q_CNT_W-1){1'b0}},1'b1}) :
                                          uq_count;
wire [63:0] uq_count_next_64 = {{(64-FTB_UPDATE_Q_CNT_W){1'b0}}, uq_count_next};

wire [`FTB_INDEX_W-1:0] service_index = uq_pc_word[uq_rptr][2 +: `FTB_INDEX_W];
wire [`FTB_INDEX_W-1:0] rd_index = service_update ? service_index : q_index;


`ifdef SYNTHESIS
// synthesis translate_off
initial begin
    if (FTB_UPDATE_Q_DEPTH < 2)
        $fatal(1, "FTB_UPDATE_Q_DEPTH must be >= 2");
    if ((FTB_UPDATE_Q_DEPTH & (FTB_UPDATE_Q_DEPTH - 1)) != 0)
        $fatal(1, "FTB_UPDATE_Q_DEPTH must be a power of two");
end
// synthesis translate_on
`endif


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
    q_valid_r <= query_valid_i && !initing;
    q_pc_r    <= query_pc_i;
end

wire [TAGW-1:0] q_tag_r = q_pc_r[31 -: TAGW];
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

wire [`BLK_LEN_W-1:0] q_len =
    way_rdata[q_way][LEN_LSB +: `BLK_LEN_W];

assign hit_o          = |q_hit;
assign resp_valid_o   = q_valid_r;
assign hit_way_o      = q_way;
assign jump_target_o  = way_rdata[q_way][TARGET_LSB +: 32];
assign fall_through_o = way_rdata[q_way][FALL_LSB +: 32];
assign br_type_o      = way_rdata[q_way][BTYPE_LSB +: `BR_TYPE_W];

// ---------------- 更新流水 ----------------
// U1 拍：U0 读出的 4 路与 u1 tag 比较
wire [TAGW-1:0] u1_tag = u1_pc_word[31 -: TAGW];
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
    way_wdata = {1'b1, u1_tag, u1_btype, u1_len,
                 u1_fall, u1_target};
    if (u1_valid) way_we[wr_way] = 1'b1;
end

always @(posedge clk) begin
    if (reset) begin
        u0_valid  <= 1'b0;
        u1_valid  <= 1'b0;
        victim_rr <= 2'd0;
`ifdef FTB_POWERUP_INIT
        initing   <= 1'b0;
`else
        initing   <= 1'b1;
`endif
        init_set  <= {`FTB_INDEX_W{1'b0}};
        uq_rptr   <= {FTB_UPDATE_Q_PTR_W{1'b0}};
        uq_wptr   <= {FTB_UPDATE_Q_PTR_W{1'b0}};
        uq_count  <= {FTB_UPDATE_Q_CNT_W{1'b0}};
    end else if (initing) begin
        u0_valid <= 1'b0;
        u1_valid <= 1'b0;
        init_set <= init_set + 1'b1;
        if (init_set == {`FTB_INDEX_W{1'b1}}) initing <= 1'b0;
    end else begin
        // 更新请求先进入 FIFO；空闲周期再借读口进入 U0
        if (update_merge_tail) begin
            // Same block is already the newest pending request.  Retain its
            // queue position but replace the payload so indirect targets
            // (ordinary JIRL/CALL) still train with the latest observation.
            uq_pc_word[uq_tail_ptr]    <= update_block_pc_i;
            uq_target[uq_tail_ptr]     <= update_jump_target_i;
            uq_ft_wordoff[uq_tail_ptr] <= update_fall_through_i;
            uq_btype[uq_tail_ptr]      <= update_br_type_i;
        end else if (update_enqueue) begin
            uq_pc_word[uq_wptr]  <= update_block_pc_i;
            uq_target[uq_wptr]   <= update_jump_target_i;
            uq_ft_wordoff[uq_wptr] <= update_fall_through_i;
            uq_btype[uq_wptr]    <= update_br_type_i;
            uq_wptr              <= uq_wptr + {{(FTB_UPDATE_Q_PTR_W-1){1'b0}},1'b1};
        end

        if (update_dequeue) begin
            uq_rptr <= uq_rptr + {{(FTB_UPDATE_Q_PTR_W-1){1'b0}},1'b1};
        end
        uq_count <= uq_count_next;

        u0_valid <= update_dequeue;
        if (update_dequeue) begin
            u0_pc_word    <= uq_pc_word[uq_rptr];
            u0_target     <= uq_target[uq_rptr];
            u0_ft_wordoff <= uq_ft_wordoff[uq_rptr];
            u0_btype      <= uq_btype[uq_rptr];
        end
        // U1：写入
        u1_valid <= u0_valid;
        if (u0_valid) begin
            u1_pc_word<= u0_pc_word;
            u1_target <= u0_target;
            u1_fall   <= {u0_fall_word, 2'b00};
            u1_btype  <= u0_btype;
            u1_len    <= u0_len;                    // 块长（1~4 条指令）
        end
        if (u1_valid && !u_found && !u_inv_found)
            victim_rr <= victim_rr + 2'd1;
    end
end

`ifdef SYNTHESIS
// synthesis translate_off
reg [63:0] ftb_query_total;
reg [63:0] ftb_response_total;
reg [63:0] ftb_hit_total;
reg [63:0] ftb_train_total;
reg [63:0] ftb_update_request_count;
reg [63:0] ftb_update_enqueue_count;
reg [63:0] ftb_update_dequeue_count;
reg [63:0] ftb_update_write_count;
reg [63:0] ftb_update_overflow_count;
reg [63:0] ftb_update_tail_merge_count;
reg [63:0] ftb_update_queue_max_occupancy;
reg [63:0] ftb_query_while_update_arrives_count;
reg [63:0] ftb_update_service_idle_cycle_count;

always @(posedge clk) begin
    if (reset) begin
        ftb_query_total               <= 64'd0;
        ftb_response_total            <= 64'd0;
        ftb_hit_total                 <= 64'd0;
        ftb_train_total               <= 64'd0;
        ftb_update_request_count      <= 64'd0;
        ftb_update_enqueue_count      <= 64'd0;
        ftb_update_dequeue_count      <= 64'd0;
        ftb_update_write_count        <= 64'd0;
        ftb_update_overflow_count     <= 64'd0;
        ftb_update_tail_merge_count   <= 64'd0;
        ftb_update_queue_max_occupancy <= 64'd0;
        ftb_query_while_update_arrives_count <= 64'd0;
        ftb_update_service_idle_cycle_count  <= 64'd0;
    end else begin
        if (query_valid_i)
            ftb_query_total <= ftb_query_total + 64'd1;
        if (q_valid_r)
            ftb_response_total <= ftb_response_total + 64'd1;
        if (hit_o)
            ftb_hit_total <= ftb_hit_total + 64'd1;
        if (update_valid_i)
            ftb_train_total <= ftb_train_total + 64'd1;
        if (update_accept)
            ftb_update_request_count <= ftb_update_request_count + 64'd1;
        if (update_enqueue)
            ftb_update_enqueue_count <= ftb_update_enqueue_count + 64'd1;
        if (update_dequeue)
            ftb_update_dequeue_count <= ftb_update_dequeue_count + 64'd1;
        if (u1_valid)
            ftb_update_write_count <= ftb_update_write_count + 64'd1;
        if (update_overflow)
            ftb_update_overflow_count <= ftb_update_overflow_count + 64'd1;
        if (update_merge_tail)
            ftb_update_tail_merge_count <=
                ftb_update_tail_merge_count + 64'd1;
        if (uq_count_next_64 > ftb_update_queue_max_occupancy)
            ftb_update_queue_max_occupancy <= uq_count_next_64;
        if (query_valid_i && update_valid_i && !initing)
            ftb_query_while_update_arrives_count <= ftb_query_while_update_arrives_count + 64'd1;
        if (update_dequeue)
            ftb_update_service_idle_cycle_count <= ftb_update_service_idle_cycle_count + 64'd1;
    end
end
// synthesis translate_on
`endif

endmodule

// ------------------------------------------------------------
// ftb_way_ram：简单双口 RAM 模板（1R + 1W，推断 BRAM）
// ------------------------------------------------------------
module ftb_way_ram #(
    parameter ENTRY_W =
        1 + (32 - 2 - `FTB_INDEX_W) + `BR_TYPE_W +
        `BLK_LEN_W + 32 + 32
)(
    input  wire                      clk,
    input  wire [`FTB_INDEX_W-1:0]   raddr,
    output reg  [ENTRY_W-1:0]        rdata,
    input  wire                      we,
    input  wire [`FTB_INDEX_W-1:0]   waddr,
    input  wire [ENTRY_W-1:0]        wdata
);
reg [ENTRY_W-1:0] mem [0:`FTB_NSET-1];
`ifdef FTB_POWERUP_INIT
integer init_i;
initial begin
    for (init_i = 0; init_i < `FTB_NSET; init_i = init_i + 1)
        mem[init_i] = {ENTRY_W{1'b0}};
end
`endif
always @(posedge clk) begin
    rdata <= mem[raddr];
    if (we) mem[waddr] <= wdata;
end
endmodule
