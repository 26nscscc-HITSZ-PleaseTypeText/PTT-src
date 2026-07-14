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

localparam ENTRY_W = `EXCP_NUM + `FTQ_W + 1 + 1 + 32 + 32;

reg [ENTRY_W-1:0] mem [0:`IB_SIZE-1];
reg [`IB_W-1:0]   head;
reg [`IB_W-1:0]   tail;
reg [`IB_W:0]     count;

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
wire pop0_valid_c = (count != {(`IB_W+1){1'b0}});
wire pop1_valid_c = (count >= {{(`IB_W-1){1'b0}}, 2'd2});
wire pop0_fire = (pop0_ready_i === 1'b1) && pop0_valid_c;
wire pop1_fire = pop0_fire && (pop1_ready_i === 1'b1) && pop1_valid_c;
wire [1:0] pop_n = {1'b0, pop0_fire} + {1'b0, pop1_fire};

wire [`IB_W:0] push_cnt_n = can_push_o ? {2'b0, push_n} : {(`IB_W+1){1'b0}};
wire [`IB_W:0] count_next   = count + push_cnt_n - {{(`IB_W-1){1'b0}}, pop_n};
wire           ib_empty_next = (count_next == {(`IB_W+1){1'b0}});

// 断组合环:原式 can_push_o 依赖 push_n,而 push_n 来自 ifu 的 pushN_valid,
// ifu 的 push 又依赖 can_push_o(ib_can_push_i),构成纯组合闭环 → 综合被迫加
// false_path、phys_opt 崩溃、时序分析失效。改为仅依据寄存器 count 与最大推入
// 宽度(4条/拍)判断:留够 4 个空位就允许推,与 push_n 解耦。count 更新已由
// line85 push_cnt_n 的 can_push_o 门控保证正确,溢出不可能。代价:count∈{13..16}
// 时略保守(偶发 1 拍前端气泡),IPC 影响可忽略(IB 出口 2 条/拍,极少贴满)。
assign can_push_o = (count <= (`IB_SIZE - 4));

wire [`IB_W-1:0] head_plus1 = head + {{(`IB_W-1){1'b0}}, 1'b1};
wire [`IB_W-1:0] tail_plus1 = tail + {{(`IB_W-1){1'b0}}, 1'b1};
wire [`IB_W-1:0] tail_plus2 = tail + {{(`IB_W-2){1'b0}}, 2'd2};
wire [`IB_W-1:0] tail_plus3 = tail + {{(`IB_W-2){1'b0}}, 2'd3};

// 出队数据必须与 head 同拍组合读出。原实现将 mem[head] 打一拍进
// pop0_entry_r，但 valid（count!=0）是组合的：push 当拍 count 已非 0、
// 而 entry_r 里还是旧数据 —— rename 会拿着"旧 PC/旧指令"配上"新 valid"
// 消费一条幽灵指令，真正的新指令则被 head+1 静默丢弃（曾表现为
// idle_1s 入口 lu12i/addi 对被吞、ld.w 用到陈旧 ARF 基址 -> 假 ALE）。
assign {pop0_excp_o, pop0_ftq_id_o, pop0_is_last_o, pop0_pred_taken_o,
        pop0_inst_o, pop0_pc_o} = mem[head];
assign {pop1_excp_o, pop1_ftq_id_o, pop1_is_last_o, pop1_pred_taken_o,
        pop1_inst_o, pop1_pc_o} = mem[head_plus1];
assign pop0_valid_o = pop0_valid_c;
assign pop1_valid_o = pop1_valid_c;

integer i;
initial begin
    head = {`IB_W{1'b0}};
    tail = {`IB_W{1'b0}};
    count = {(`IB_W+1){1'b0}};
    for (i = 0; i < `IB_SIZE; i = i + 1)
        mem[i] = {ENTRY_W{1'b0}};
end

always @(posedge clk) begin
    if (reset || flush_i) begin
        head <= {`IB_W{1'b0}};
        tail <= {`IB_W{1'b0}};
        count <= {(`IB_W+1){1'b0}};
    end else begin
        if (can_push_o) begin
            if (push0_valid_i) mem[tail]      <= push0_entry;
            if (push1_valid_i) mem[tail_plus1] <= push1_entry;
            if (push2_valid_i) mem[tail_plus2] <= push2_entry;
            if (push3_valid_i) mem[tail_plus3] <= push3_entry;
        end

        if (ib_empty_next) begin
            head <= {`IB_W{1'b0}};
            if (can_push_o)
                tail <= {1'b0, push_n[`IB_W-2:0]};
            else
                tail <= {`IB_W{1'b0}};
        end else begin
            head <= head + {{(`IB_W-2){1'b0}}, pop_n};
            if (can_push_o)
                tail <= tail + {1'b0, push_n[`IB_W-2:0]};
        end
        count <= count_next;
    end
end

endmodule
