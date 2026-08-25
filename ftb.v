// ============================================================
// ftb 模块（Fetch Target Buffer，取指目标缓冲）
// ------------------------------------------------------------
// 当前结构与约束：
`ifdef SOC_USE
// - 4 路 × 2048 组，推断 BRAM（1R+1W 简单双口），查询 1 拍延迟；
`else
// - 4 路 × 2048 组，推断 BRAM（查询口 A + 训练读写口 B），查询 1 拍延迟；
`endif
// - 条目 {valid, tag(19), br_type(2), len(3), target(32)}；
//   fall_through 不存全宽：由 len 重建（= 块PC + 4*len）；
`ifdef SOC_USE
// - 更新走「训练 FIFO + 内部 2 级小流水」：训练请求先入小队列（深度
//   `FTB_UPDATE_Q_DEPTH，满则丢弃计 overflow），查询优先占读口，U0 只在
//   无查询的空闲拍出队借读口读出组内 4 路，U1 比较命中路原地更新 /
//   victim 轮转分配（查询永不被作废）；
`else
// - 更新走 32 深训练 FIFO；训练口 B 交替执行读组和下一拍写回，查询口 A
//   不再阻塞训练。查询与训练写同址时允许查询看到旧项；FTB 仅提供预测，
//   下一次查询即可看到训练后的新项，不在查询关键路径上增加宽旁路；
`endif
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

    // ---------------- 每路原始载荷（与 hit_way_o 同拍）----------------
    // 命中优先编码 -> 宽 MUX -> 目标比较是 P1 覆盖判定的最长串联链。导出各路
    // 原始字段，使 bpu 可以在“选路”之外并行地按路比较，再用 1 bit 独热选择
    // 结果。语义与先选后比完全相同：独热位只保留与 hit_way_o 相同的最低命中路。
    output wire [`FTB_NWAY-1:0]                 way_hit_onehot_o,
    output wire [(`FTB_NWAY*32)-1:0]            way_jump_target_o,
    output wire [(`FTB_NWAY*32)-1:0]            way_fall_through_o,
    output wire [(`FTB_NWAY*`BR_TYPE_W)-1:0]    way_br_type_o,

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

`ifdef SOC_USE
// ---------------- 更新流水 U0/U1 ----------------
`else
// ---------------- 更新流水 U0 ----------------
`endif
reg                   u0_valid;
reg [31:2]            u0_pc_word;
reg [31:0]            u0_target;
reg [`BLK_LEN_W-1:0]  u0_ft_wordoff;
reg [`BR_TYPE_W-1:0]  u0_btype;
`ifdef SOC_USE
reg                   u1_valid;
reg [31:2]            u1_pc_word;
reg [31:0]            u1_target;
reg [31:0]            u1_fall;
reg [`BR_TYPE_W-1:0]  u1_btype;
reg [`BLK_LEN_W-1:0]  u1_len;
`else
`endif

wire [`BLK_LEN_W-1:0] u0_len =
    u0_ft_wordoff - u0_pc_word[`BLK_LEN_W+1:2];
wire [31:2] u0_fall_word =
    u0_pc_word + {{(30-`BLK_LEN_W){1'b0}}, u0_len};

wire [`FTB_INDEX_W-1:0] q_index = query_pc_i[2 +: `FTB_INDEX_W];
`ifdef SOC_USE
wire [`FTB_INDEX_W-1:0] u1_index= u1_pc_word[2 +: `FTB_INDEX_W];
`else
wire [`FTB_INDEX_W-1:0] u0_index= u0_pc_word[2 +: `FTB_INDEX_W];
`endif

`ifdef SOC_USE
// 读口仲裁：查询优先，训练请求进入小 FIFO 后在空闲周期借口
`else
// 查询始终走 A 口；B 口在一次训练读与其下一拍写之间交替。
`endif
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
`ifdef SOC_USE
wire service_update     = !initing && !query_valid_i && !update_queue_empty;
`else
// u0_valid=1 的拍由 B 口写回；其余拍可以从 FIFO 发起下一次训练读。
wire service_update     = !initing && !u0_valid && !update_queue_empty;
`endif
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
`ifdef SOC_USE
wire [`FTB_INDEX_W-1:0] rd_index = service_update ? service_index : q_index;
`else
wire [`FTB_INDEX_W-1:0] update_port_addr =
    initing ? init_set : service_update ? service_index : u0_index;
`endif


// ---------------- 4 路 BRAM ----------------
`ifdef SOC_USE
wire [ENTRY_W-1:0] way_rdata [0:`FTB_NWAY-1];
`else
wire [ENTRY_W-1:0] query_way_rdata  [0:`FTB_NWAY-1];
wire [ENTRY_W-1:0] update_way_rdata [0:`FTB_NWAY-1];
`endif
reg  [`FTB_NWAY-1:0] way_we;
reg  [ENTRY_W-1:0] way_wdata;
`ifdef SOC_USE
wire [`FTB_INDEX_W-1:0] wr_index = initing ? init_set : u1_index;
`else
`endif

genvar g;
generate
for (g = 0; g < `FTB_NWAY; g = g + 1) begin : gen_ftb_way
    ftb_way_ram u_way(
`ifdef SOC_USE
        .clk   (clk),
        .raddr (rd_index),
        .rdata (way_rdata[g]),
        .we    (way_we[g] | initing),
        .waddr (wr_index),
        .wdata (initing ? {ENTRY_W{1'b0}} : way_wdata)
`else
        .clk     (clk),
        .q_raddr (q_index),
        .q_rdata (query_way_rdata[g]),
        .u_en    (initing | service_update | way_we[g]),
        .u_we    (initing | way_we[g]),
        .u_addr  (update_port_addr),
        .u_rdata (update_way_rdata[g]),
        .u_wdata (initing ? {ENTRY_W{1'b0}} : way_wdata)
`endif
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
`ifdef SOC_USE
    assign q_hit[g] = q_valid_r && way_rdata[g][ENTRY_W-1]
                   && (way_rdata[g][ENTRY_W-2 -: TAGW] == q_tag_r);
`else
    assign q_hit[g] = q_valid_r && query_way_rdata[g][ENTRY_W-1]
                   && (query_way_rdata[g][ENTRY_W-2 -: TAGW] == q_tag_r);
`endif
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
`ifdef SOC_USE
    way_rdata[q_way][LEN_LSB +: `BLK_LEN_W];
`else
    query_way_rdata[q_way][LEN_LSB +: `BLK_LEN_W];
`endif

assign hit_o          = |q_hit;
assign resp_valid_o   = q_valid_r;
assign hit_way_o      = q_way;
`ifdef SOC_USE
assign jump_target_o  = way_rdata[q_way][TARGET_LSB +: 32];
assign fall_through_o = way_rdata[q_way][FALL_LSB +: 32];
assign br_type_o      = way_rdata[q_way][BTYPE_LSB +: `BR_TYPE_W];
`else
assign jump_target_o  = query_way_rdata[q_way][TARGET_LSB +: 32];
assign fall_through_o = query_way_rdata[q_way][FALL_LSB +: 32];
assign br_type_o      = query_way_rdata[q_way][BTYPE_LSB +: `BR_TYPE_W];
`endif

// 独热化命中：只保留最低命中路，与 q_way 的优先级完全一致（q_way 的降序循环
// 使最低命中路最后写入）。按构造同一 tag 不会同时存在于多路——写回优先命中路，
// 否则取最低无效路——所以掩码在功能上是恒等的；显式写出以保证等价性可证。
wire [`FTB_NWAY-1:0] q_hit_lower_mask;
generate
`ifdef SOC_USE
for (g = 0; g < `FTB_NWAY; g = g + 1) begin : gen_ftb_way_outputs
`else
for (g = 0; g < `FTB_NWAY; g = g + 1) begin : gen_ftb_hit_onehot
`endif
    if (g == 0) begin : g_lowest
        assign q_hit_lower_mask[g] = 1'b0;
    end else begin : g_upper
        assign q_hit_lower_mask[g] = |q_hit[g-1:0];
    end
    assign way_hit_onehot_o[g] = q_hit[g] && !q_hit_lower_mask[g];
`ifdef SOC_USE
    assign way_jump_target_o[g*32 +: 32] =
        way_rdata[g][TARGET_LSB +: 32];
    assign way_fall_through_o[g*32 +: 32] =
        way_rdata[g][FALL_LSB +: 32];
    assign way_br_type_o[g*`BR_TYPE_W +: `BR_TYPE_W] =
        way_rdata[g][BTYPE_LSB +: `BR_TYPE_W];
`else
    assign way_jump_target_o[g*32 +: 32] =
        query_way_rdata[g][TARGET_LSB +: 32];
    assign way_fall_through_o[g*32 +: 32] =
        query_way_rdata[g][FALL_LSB +: 32];
    assign way_br_type_o[g*`BR_TYPE_W +: `BR_TYPE_W] =
        query_way_rdata[g][BTYPE_LSB +: `BR_TYPE_W];
`endif
end
endgenerate

`ifdef SOC_USE
// ---------------- 更新流水 ----------------
// U1 拍：U0 读出的 4 路与 u1 tag 比较
wire [TAGW-1:0] u1_tag = u1_pc_word[31 -: TAGW];
`else
// ---------------- 更新写回 ----------------
// B 口上一拍读出的 4 路与同拍锁存的 U0 payload 对齐；本拍写回。
wire [TAGW-1:0] u0_tag = u0_pc_word[31 -: TAGW];
`endif
wire [`FTB_NWAY-1:0] u_hit;
generate
for (g = 0; g < `FTB_NWAY; g = g + 1) begin : gen_ftb_uhit
`ifdef SOC_USE
    assign u_hit[g] = way_rdata[g][ENTRY_W-1]
                   && (way_rdata[g][ENTRY_W-2 -: TAGW] == u1_tag);
`else
    assign u_hit[g] = u0_valid && update_way_rdata[g][ENTRY_W-1]
                   && (update_way_rdata[g][ENTRY_W-2 -: TAGW] == u0_tag);
`endif
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
`ifdef SOC_USE
        if (!way_rdata[uj][ENTRY_W-1]) begin u_inv_found = 1'b1; u_inv_way = uj[1:0]; end
`else
        if (!update_way_rdata[uj][ENTRY_W-1]) begin
            u_inv_found = 1'b1;
            u_inv_way = uj[1:0];
        end
`endif
    end
end

reg [1:0] victim_rr;
wire [1:0] wr_way = u_found ? u_way : u_inv_found ? u_inv_way : victim_rr;

always @(*) begin
    way_we    = {`FTB_NWAY{1'b0}};
`ifdef SOC_USE
    way_wdata = {1'b1, u1_tag, u1_btype, u1_len,
                 u1_fall, u1_target};
    if (u1_valid) way_we[wr_way] = 1'b1;
`else
    way_wdata = {1'b1, u0_tag, u0_btype, u0_len,
                 {u0_fall_word, 2'b00}, u0_target};
    if (u0_valid) way_we[wr_way] = 1'b1;
`endif
end

always @(posedge clk) begin
    if (reset) begin
        u0_valid  <= 1'b0;
`ifdef SOC_USE
        u1_valid  <= 1'b0;
`else
`endif
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
`ifdef SOC_USE
        u1_valid <= 1'b0;
`else
`endif
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
`ifdef SOC_USE
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
`else
        if (u0_valid && !u_found && !u_inv_found)
`endif
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
`ifdef SOC_USE
        if (u1_valid)
`else
        if (u0_valid)
`endif
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
`ifdef SOC_USE
// ftb_way_ram：简单双口 RAM 模板（1R + 1W，推断 BRAM）
`else
// ftb_way_ram：真双口 RAM 模板（A=查询只读，B=训练读写，推断 BRAM）
`endif
// ------------------------------------------------------------
module ftb_way_ram #(
    parameter ENTRY_W =
        1 + (32 - 2 - `FTB_INDEX_W) + `BR_TYPE_W +
        `BLK_LEN_W + 32 + 32
)(
    input  wire                      clk,
`ifdef SOC_USE
    input  wire [`FTB_INDEX_W-1:0]   raddr,
    output reg  [ENTRY_W-1:0]        rdata,
    input  wire                      we,
    input  wire [`FTB_INDEX_W-1:0]   waddr,
    input  wire [ENTRY_W-1:0]        wdata
`else
    input  wire [`FTB_INDEX_W-1:0]   q_raddr,
    output reg  [ENTRY_W-1:0]        q_rdata,
    input  wire                      u_en,
    input  wire                      u_we,
    input  wire [`FTB_INDEX_W-1:0]   u_addr,
    output reg  [ENTRY_W-1:0]        u_rdata,
    input  wire [ENTRY_W-1:0]        u_wdata
`endif
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
`ifdef SOC_USE
    rdata <= mem[raddr];
    if (we) mem[waddr] <= wdata;
`else
    q_rdata <= mem[q_raddr];
    if (u_en) begin
        if (u_we) mem[u_addr] <= u_wdata;
        else      u_rdata <= mem[u_addr];
    end
`endif
end
endmodule
