`include "mycpu.h"

module inst_buffer(
    input  wire                       clk,
    input  wire                       reset,
    input  wire                       flush_i,

    input  wire                       push0_valid_i,
    input  wire [31:0]                push0_pc_i,
    input  wire [31:0]                push0_inst_i,
    input  wire                       push0_pred_taken_i,
    input  wire                       push0_is_last_i,
    input  wire [`FTQ_W-1:0]          push0_ftq_id_i,
    input  wire [`EXCP_NUM-1:0]       push0_excp_i,

    input  wire                       push1_valid_i,
    input  wire [31:0]                push1_pc_i,
    input  wire [31:0]                push1_inst_i,
    input  wire                       push1_pred_taken_i,
    input  wire                       push1_is_last_i,
    input  wire [`FTQ_W-1:0]          push1_ftq_id_i,
    input  wire [`EXCP_NUM-1:0]       push1_excp_i,

    input  wire                       push2_valid_i,
    input  wire [31:0]                push2_pc_i,
    input  wire [31:0]                push2_inst_i,
    input  wire                       push2_pred_taken_i,
    input  wire                       push2_is_last_i,
    input  wire [`FTQ_W-1:0]          push2_ftq_id_i,
    input  wire [`EXCP_NUM-1:0]       push2_excp_i,

    input  wire                       push3_valid_i,
    input  wire [31:0]                push3_pc_i,
    input  wire [31:0]                push3_inst_i,
    input  wire                       push3_pred_taken_i,
    input  wire                       push3_is_last_i,
    input  wire [`FTQ_W-1:0]          push3_ftq_id_i,
    input  wire [`EXCP_NUM-1:0]       push3_excp_i,

    output wire                       can_push_o,

    output wire                       pop0_valid_o,
    output wire [31:0]                pop0_pc_o,
    output wire [31:0]                pop0_inst_o,
    output wire                       pop0_pred_taken_o,
    output wire                       pop0_is_last_o,
    output wire [`FTQ_W-1:0]          pop0_ftq_id_o,
    output wire [`EXCP_NUM-1:0]       pop0_excp_o,
    input  wire                       pop0_ready_i,

    output wire                       pop1_valid_o,
    output wire [31:0]                pop1_pc_o,
    output wire [31:0]                pop1_inst_o,
    output wire                       pop1_pred_taken_o,
    output wire                       pop1_is_last_o,
    output wire [`FTQ_W-1:0]          pop1_ftq_id_o,
    output wire [`EXCP_NUM-1:0]       pop1_excp_o,
    input  wire                       pop1_ready_i
);

localparam integer ENTRY_W = `EXCP_NUM + `FTQ_W + 1 + 1 + 32 + 32;
localparam integer BANK_NUM   = 4;
localparam integer BANK_SEL_W = 2;
localparam integer BANK_DEPTH = `IB_SIZE / BANK_NUM;
localparam integer ROW_W      = `IB_W - BANK_SEL_W;

(* ram_style = "distributed" *) reg [ENTRY_W-1:0] bank0 [0:BANK_DEPTH-1];
(* ram_style = "distributed" *) reg [ENTRY_W-1:0] bank1 [0:BANK_DEPTH-1];
(* ram_style = "distributed" *) reg [ENTRY_W-1:0] bank2 [0:BANK_DEPTH-1];
(* ram_style = "distributed" *) reg [ENTRY_W-1:0] bank3 [0:BANK_DEPTH-1];
reg [`IB_W-1:0]   head;
reg [`IB_W-1:0]   tail;
// One-hot occupancy removes the decoder/rename-ready -> binary add/subtract
// cone from the IB state update.  count is retained only as a decoded probe
// for the simulation statistics in mycpu_top.
reg [`IB_SIZE:0]  count_oh;
reg [`IB_W:0]     count;
integer count_decode_i;
always @(*) begin
    count = {(`IB_W+1){1'b0}};
    for (count_decode_i = 0; count_decode_i <= `IB_SIZE;
         count_decode_i = count_decode_i + 1)
        if (count_oh[count_decode_i])
            count = count_decode_i[`IB_W:0];
end

wire [ENTRY_W-1:0] push0_entry = {push0_excp_i, push0_ftq_id_i, push0_is_last_i,
                                  push0_pred_taken_i, push0_inst_i, push0_pc_i};
wire [ENTRY_W-1:0] push1_entry = {push1_excp_i, push1_ftq_id_i, push1_is_last_i,
                                  push1_pred_taken_i, push1_inst_i, push1_pc_i};
wire [ENTRY_W-1:0] push2_entry = {push2_excp_i, push2_ftq_id_i, push2_is_last_i,
                                  push2_pred_taken_i, push2_inst_i, push2_pc_i};
wire [ENTRY_W-1:0] push3_entry = {push3_excp_i, push3_ftq_id_i, push3_is_last_i,
                                  push3_pred_taken_i, push3_inst_i, push3_pc_i};

wire [2:0] push_n = {2'b0, push0_valid_i} + {2'b0, push1_valid_i}
                  + {2'b0, push2_valid_i} + {2'b0, push3_valid_i};
`ifdef IB_DISABLE_EMPTY_FALLTHROUGH
wire ib_empty_fwft_en = 1'b0;
`else
wire ib_empty_fwft_en = 1'b1;
`endif
wire stored_pop0_valid_c = !count_oh[0];
wire stored_pop1_valid_c = !(count_oh[0] || count_oh[1]);
wire ft_active = ib_empty_fwft_en && count_oh[0]
              && push0_valid_i && can_push_o && !flush_i;
wire pop0_valid_c = ft_active ? (push_n >= 3'd1) : stored_pop0_valid_c;
`ifdef IB_SINGLE_EMPTY_FALLTHROUGH
wire pop1_valid_c = ft_active ? 1'b0 : stored_pop1_valid_c;
`else
wire pop1_valid_c = ft_active ? (push_n >= 3'd2) : stored_pop1_valid_c;
`endif
wire pop0_fire = (pop0_ready_i === 1'b1) && pop0_valid_c;
wire pop1_fire = pop0_fire && (pop1_ready_i === 1'b1) && pop1_valid_c;
wire [1:0] pop_n = {1'b0, pop0_fire} + {1'b0, pop1_fire};

wire [1:0] ft_consume_count = {1'b0, ft_active && pop0_fire}
                            + {1'b0, ft_active && pop1_fire};
wire [1:0] stored_pop_count = ft_active ? 2'd0 : pop_n;
wire [2:0] residual_push_count = ft_active ? (push_n - {1'b0, ft_consume_count}) : push_n;
wire [`IB_W-1:0] push_inc              = {{(`IB_W-3){1'b0}}, push_n};
wire [`IB_W-1:0] ft_consume_inc        = {{(`IB_W-2){1'b0}}, ft_consume_count};
wire [`IB_W-1:0] stored_pop_inc        = {{(`IB_W-2){1'b0}}, stored_pop_count};
wire [2:0] accepted_push_n = can_push_o ? residual_push_count : 3'd0;

reg [`IB_SIZE:0] count_oh_next;
always @(*) begin
    count_oh_next = count_oh;
    if (ft_active) begin
        case (residual_push_count)
            3'd0: count_oh_next = {{`IB_SIZE{1'b0}}, 1'b1};
            3'd1: count_oh_next = {{(`IB_SIZE-1){1'b0}}, 2'b10};
            3'd2: count_oh_next = {{(`IB_SIZE-2){1'b0}}, 3'b100};
            3'd3: count_oh_next = {{(`IB_SIZE-3){1'b0}}, 4'b1000};
            default:
                count_oh_next = {{(`IB_SIZE-4){1'b0}}, 5'b10000};
        endcase
    end else begin
        case ({accepted_push_n, stored_pop_count})
            {3'd0,2'd1}: count_oh_next = count_oh >> 1;
            {3'd0,2'd2}: count_oh_next = count_oh >> 2;
            {3'd1,2'd0}: count_oh_next = count_oh << 1;
            {3'd1,2'd2}: count_oh_next = count_oh >> 1;
            {3'd2,2'd0}: count_oh_next = count_oh << 2;
            {3'd2,2'd1}: count_oh_next = count_oh << 1;
            {3'd3,2'd0}: count_oh_next = count_oh << 3;
            {3'd3,2'd1}: count_oh_next = count_oh << 2;
            {3'd3,2'd2}: count_oh_next = count_oh << 1;
            {3'd4,2'd0}: count_oh_next = count_oh << 4;
            {3'd4,2'd1}: count_oh_next = count_oh << 3;
            {3'd4,2'd2}: count_oh_next = count_oh << 2;
            default:     count_oh_next = count_oh;
        endcase
    end
end

// can_push_o 只能依赖寄存器 count，不能依赖 IFU 的 push valid，否则会与
// ib_can_push_i 构成组合环。预留每拍最大 4 条的空间；count 为 13..16 时会
// 保守反压，push_cnt_n 仍负责门控计数更新，保证不会溢出。
assign can_push_o = |count_oh[`IB_SIZE-4:0];

wire [ROW_W-1:0]  head_row = head[`IB_W-1:BANK_SEL_W];
wire [ROW_W-1:0]  head_row_plus1 = head_row + {{(ROW_W-1){1'b0}}, 1'b1};
wire [ROW_W-1:0]  tail_row = tail[`IB_W-1:BANK_SEL_W];
wire [ROW_W-1:0]  tail_row_plus1 = tail_row + {{(ROW_W-1){1'b0}}, 1'b1};
wire [ROW_W-1:0]  bank0_raddr = (head[1:0] == 2'd3) ? head_row_plus1 : head_row;
wire [ROW_W-1:0]  bank1_raddr = head_row;
wire [ROW_W-1:0]  bank2_raddr = head_row;
wire [ROW_W-1:0]  bank3_raddr = head_row;
wire [ENTRY_W-1:0] bank0_rdata = bank0[bank0_raddr];
wire [ENTRY_W-1:0] bank1_rdata = bank1[bank1_raddr];
wire [ENTRY_W-1:0] bank2_rdata = bank2[bank2_raddr];
wire [ENTRY_W-1:0] bank3_rdata = bank3[bank3_raddr];

reg [ENTRY_W-1:0] stored_pop0_entry_c;
reg [ENTRY_W-1:0] stored_pop1_entry_c;
always @(*) begin
    case (head[1:0])
        2'd0: begin
            stored_pop0_entry_c = bank0_rdata;
            stored_pop1_entry_c = bank1_rdata;
        end
        2'd1: begin
            stored_pop0_entry_c = bank1_rdata;
            stored_pop1_entry_c = bank2_rdata;
        end
        2'd2: begin
            stored_pop0_entry_c = bank2_rdata;
            stored_pop1_entry_c = bank3_rdata;
        end
        default: begin
            stored_pop0_entry_c = bank3_rdata;
            stored_pop1_entry_c = bank0_rdata;
        end
    endcase
end

wire [ENTRY_W-1:0] pop0_entry_c = ft_active ? push0_entry : stored_pop0_entry_c;
wire [ENTRY_W-1:0] pop1_entry_c = ft_active ? push1_entry : stored_pop1_entry_c;

// 空队列直通时仍把全部入口按原顺序写入 RAM，再由 head 跳过同拍已消费项。
// RAM 写数据和写使能因此只依赖 IFU，不经过 decoder/rename ready 反馈；既保留
// 冷启动零气泡，也切断 IFU→IB→decoder→rename→IB 写口的长组合路径。
wire wr0_valid = push0_valid_i;
wire wr1_valid = push1_valid_i;
wire wr2_valid = push2_valid_i;
wire wr3_valid = push3_valid_i;

// 出队数据与 valid 必须使用同一拍的 head 组合读结果；若数据额外打一拍，
// push 后首个 valid 会与上一拍旧数据错配，并导致真正的首条指令被跳过。
assign {pop0_excp_o, pop0_ftq_id_o, pop0_is_last_o, pop0_pred_taken_o,
        pop0_inst_o, pop0_pc_o} = pop0_entry_c;
assign {pop1_excp_o, pop1_ftq_id_o, pop1_is_last_o, pop1_pred_taken_o,
        pop1_inst_o, pop1_pc_o} = pop1_entry_c;
assign pop0_valid_o = pop0_valid_c;
assign pop1_valid_o = pop1_valid_c;

integer i;
initial begin
    head = {`IB_W{1'b0}};
    tail = {`IB_W{1'b0}};
    count_oh = {{`IB_SIZE{1'b0}}, 1'b1};
    for (i = 0; i < BANK_DEPTH; i = i + 1) begin
        bank0[i] = {ENTRY_W{1'b0}};
        bank1[i] = {ENTRY_W{1'b0}};
        bank2[i] = {ENTRY_W{1'b0}};
        bank3[i] = {ENTRY_W{1'b0}};
    end
end



always @(posedge clk) begin
    if (reset || flush_i) begin
        head <= {`IB_W{1'b0}};
        tail <= {`IB_W{1'b0}};
        count_oh <= {{`IB_SIZE{1'b0}}, 1'b1};
    end else begin
        if (can_push_o) begin
            case (tail[1:0])
                2'd0: begin
                    if (wr0_valid) bank0[tail_row]       <= push0_entry;
                    if (wr1_valid) bank1[tail_row]       <= push1_entry;
                    if (wr2_valid) bank2[tail_row]       <= push2_entry;
                    if (wr3_valid) bank3[tail_row]       <= push3_entry;
                end
                2'd1: begin
                    if (wr0_valid) bank1[tail_row]       <= push0_entry;
                    if (wr1_valid) bank2[tail_row]       <= push1_entry;
                    if (wr2_valid) bank3[tail_row]       <= push2_entry;
                    if (wr3_valid) bank0[tail_row_plus1] <= push3_entry;
                end
                2'd2: begin
                    if (wr0_valid) bank2[tail_row]       <= push0_entry;
                    if (wr1_valid) bank3[tail_row]       <= push1_entry;
                    if (wr2_valid) bank0[tail_row_plus1] <= push2_entry;
                    if (wr3_valid) bank1[tail_row_plus1] <= push3_entry;
                end
                default: begin
                    if (wr0_valid) bank3[tail_row]       <= push0_entry;
                    if (wr1_valid) bank0[tail_row_plus1] <= push1_entry;
                    if (wr2_valid) bank1[tail_row_plus1] <= push2_entry;
                    if (wr3_valid) bank2[tail_row_plus1] <= push3_entry;
                end
            endcase
        end

        // Do not reset head/tail when the queue naturally becomes empty.
        // Ring-buffer invariants already guarantee head_next == tail_next in
        // that case.  Resetting both to zero made the FWFT push-valid path run
        // through decode/rename ready and back into every pointer reset pin.
        if (ft_active) begin
            head <= head + ft_consume_inc;
            tail <= tail + push_inc;
        end else begin
            head <= head + stored_pop_inc;
            if (can_push_o)
                tail <= tail + push_inc;
        end
        count_oh <= count_oh_next;
    end
end

endmodule
