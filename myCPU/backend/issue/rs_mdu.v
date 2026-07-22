// ============================================================
// rs_mdu 模块（乘除/杂项保留站，`RS_MDU_SIZE=2 项，FIFO 顺序发射）
// ------------------------------------------------------------
// 功能：
// - 缓存乘/除/取模、CSR 读类（csrrd/csrwr/csrxchg 的读旧值+算新值）、
//   rdcnt 类、TLB 维护类（执行级只打包 invtlb 的 asid/vpn）指令。
// - FIFO 严格顺序发射（MDU 非流水多周期，且 CSR 读必须按序——
//   虽然 CSR 写会触发 refetch 冲刷保证了顺序，但保守起见仍按序发射）。
// - 容量 2 项即可（这类指令频率低，mariver 同款配置）。
//
// 端口：与 rs_mem 同构，差异：
// - bundle 为 alu_op（乘除位）/csr_op/csr_num/tlb_op/wb_src_op
// - 发射口对接 fu_mdu，mdu_ready_i 反压（MDU 多周期忙）
// ============================================================
`include "mycpu.h"

module rs_mdu(
    input  wire                       clk,
    input  wire                       reset,
    input  wire                       flush_i,

    // ---------------- 入站（dispatch）----------------
    input  wire                       push_valid_i,
    input  wire [`ROB_W-1:0]          push_robid_i,
    input  wire [`ALU_OP_NUM-1:0]     push_alu_op_i,       // 乘除位有效
    input  wire [`CSR_OP_NUM-1:0]     push_csr_op_i,
    input  wire [13:0]                push_csr_num_i,
    input  wire [`TLB_OP_NUM-1:0]     push_tlb_op_i,
    input  wire [`WB_SRC_NUM-1:0]     push_wb_src_op_i,    // rdcnt 类选择
    input  wire                       push_src0_ready_i,   // src0 = rj（csrxchg 的 mask / invtlb 的 asid / 乘除源1）
    input  wire [31:0]                push_src0_val_i,
    input  wire [`ROB_W-1:0]          push_src0_robid_i,
    input  wire                       push_src1_ready_i,   // src1 = rk/rd（csr 写值 / invtlb 的 va / 乘除源2）
    input  wire [31:0]                push_src1_val_i,
    input  wire [`ROB_W-1:0]          push_src1_robid_i,

    output wire                       can_accept_o,
    output wire [1:0]                 occupancy_o,

    // ---------------- 写回唤醒总线 ×4 ----------------
    input  wire                       wb0_valid_i,
    input  wire [`ROB_W-1:0]          wb0_robid_i,
    input  wire [31:0]                wb0_data_i,
    input  wire                       wb1_valid_i,
    input  wire [`ROB_W-1:0]          wb1_robid_i,
    input  wire [31:0]                wb1_data_i,
    input  wire                       wb2_valid_i,
    input  wire [`ROB_W-1:0]          wb2_robid_i,
    input  wire [31:0]                wb2_data_i,
    input  wire                       wb3_valid_i,
    input  wire [`ROB_W-1:0]          wb3_robid_i,
    input  wire [31:0]                wb3_data_i,

    // ---------------- 提前唤醒总线 ×3（early0/1/2 均已接通；early2=LSU DC 命中）----------------
    input  wire                       early0_valid_i,
    input  wire [`ROB_W-1:0]          early0_robid_i,
    input  wire                       early1_valid_i,
    input  wire [`ROB_W-1:0]          early1_robid_i,
    input  wire                       early2_valid_i,
    input  wire [`ROB_W-1:0]          early2_robid_i,

    // ---------------- 发射口（到 fu_mdu）----------------
    output wire                       issue_valid_o,
    output wire [`ROB_W-1:0]          issue_robid_o,
    output wire [`ALU_OP_NUM-1:0]     issue_alu_op_o,
    output wire [`CSR_OP_NUM-1:0]     issue_csr_op_o,
    output wire [13:0]                issue_csr_num_o,
    output wire [`TLB_OP_NUM-1:0]     issue_tlb_op_o,
    output wire [`WB_SRC_NUM-1:0]     issue_wb_src_op_o,
    output wire [31:0]                issue_src0_o,
    output wire [31:0]                issue_src1_o,
    input  wire                       mdu_ready_i          // MDU 空闲可接收
);

// 设计说明（已实现，参考 mariver station.v 的 MDU 保留站部分）：
//      结构与 rs_mem 同构（head/tail FIFO + 唤醒捕获 + 队头发射），
//      容量 2 项、bundle 字段不同、反压来自 mdu_ready_i。
//
// 坑点提示：
//      1. csrxchg 同时用 rj(mask) 和 rd(写值) 两个源，都要等唤醒。
//      2. invtlb 的 asid 来自 rj[9:0]、va 来自 rk —— 也是双源指令。
//      3. rdcnt 类无源操作数（use_src=0，rename 已置 ready=1），入站即可发射。

reg                     valid [0:`RS_MDU_SIZE-1];
reg [`ROB_W-1:0]        robid [0:`RS_MDU_SIZE-1];
reg [`ALU_OP_NUM-1:0]   alu_op [0:`RS_MDU_SIZE-1];
reg [`CSR_OP_NUM-1:0]   csr_op [0:`RS_MDU_SIZE-1];
reg [13:0]              csr_num [0:`RS_MDU_SIZE-1];
reg [`TLB_OP_NUM-1:0]   tlb_op [0:`RS_MDU_SIZE-1];
reg [`WB_SRC_NUM-1:0]   wb_src_op [0:`RS_MDU_SIZE-1];
reg                     s0_ready [0:`RS_MDU_SIZE-1];
reg                     s0_val_valid [0:`RS_MDU_SIZE-1];
reg [31:0]              s0_val [0:`RS_MDU_SIZE-1];
reg [`ROB_W-1:0]        s0_robid [0:`RS_MDU_SIZE-1];
reg                     s1_ready [0:`RS_MDU_SIZE-1];
reg                     s1_val_valid [0:`RS_MDU_SIZE-1];
reg [31:0]              s1_val [0:`RS_MDU_SIZE-1];
reg [`ROB_W-1:0]        s1_robid [0:`RS_MDU_SIZE-1];
reg                     head;
reg                     tail;
reg [1:0]               count;

integer i;
wire head_ready;
wire issue_fire;

wire            s0_wb_match [0:`RS_MDU_SIZE-1];
wire            s1_wb_match [0:`RS_MDU_SIZE-1];
wire            s0_wbhit [0:`RS_MDU_SIZE-1];
wire            s1_wbhit [0:`RS_MDU_SIZE-1];
wire            s0_earlyhit [0:`RS_MDU_SIZE-1];
wire            s1_earlyhit [0:`RS_MDU_SIZE-1];
wire [31:0]     s0_wbdat [0:`RS_MDU_SIZE-1];
wire [31:0]     s1_wbdat [0:`RS_MDU_SIZE-1];
genvar gw;
generate
for (gw = 0; gw < `RS_MDU_SIZE; gw = gw + 1) begin : g_wake
    assign s0_wb_match[gw] = (wb0_valid_i && (wb0_robid_i == s0_robid[gw])) ||
                             (wb1_valid_i && (wb1_robid_i == s0_robid[gw])) ||
                             (wb2_valid_i && (wb2_robid_i == s0_robid[gw])) ||
                             (wb3_valid_i && (wb3_robid_i == s0_robid[gw]));
    assign s1_wb_match[gw] = (wb0_valid_i && (wb0_robid_i == s1_robid[gw])) ||
                             (wb1_valid_i && (wb1_robid_i == s1_robid[gw])) ||
                             (wb2_valid_i && (wb2_robid_i == s1_robid[gw])) ||
                             (wb3_valid_i && (wb3_robid_i == s1_robid[gw]));
    assign s0_wbhit[gw] = !s0_val_valid[gw] && s0_wb_match[gw];
    assign s1_wbhit[gw] = !s1_val_valid[gw] && s1_wb_match[gw];
    assign s0_earlyhit[gw] = !s0_ready[gw] && !s0_wbhit[gw] &&
                             ((early0_valid_i && (early0_robid_i == s0_robid[gw])) ||
                              (early1_valid_i && (early1_robid_i == s0_robid[gw])) ||
                              (early2_valid_i && (early2_robid_i == s0_robid[gw])));
    assign s1_earlyhit[gw] = !s1_ready[gw] && !s1_wbhit[gw] &&
                             ((early0_valid_i && (early0_robid_i == s1_robid[gw])) ||
                              (early1_valid_i && (early1_robid_i == s1_robid[gw])) ||
                              (early2_valid_i && (early2_robid_i == s1_robid[gw])));
    assign s0_wbdat[gw] = (wb0_valid_i && (wb0_robid_i == s0_robid[gw])) ? wb0_data_i :
                          (wb1_valid_i && (wb1_robid_i == s0_robid[gw])) ? wb1_data_i :
                          (wb2_valid_i && (wb2_robid_i == s0_robid[gw])) ? wb2_data_i :
                          (wb3_valid_i && (wb3_robid_i == s0_robid[gw])) ? wb3_data_i : 32'b0;
    assign s1_wbdat[gw] = (wb0_valid_i && (wb0_robid_i == s1_robid[gw])) ? wb0_data_i :
                          (wb1_valid_i && (wb1_robid_i == s1_robid[gw])) ? wb1_data_i :
                          (wb2_valid_i && (wb2_robid_i == s1_robid[gw])) ? wb2_data_i :
                          (wb3_valid_i && (wb3_robid_i == s1_robid[gw])) ? wb3_data_i : 32'b0;
end
endgenerate

wire        push_s0_wbhit = !push_src0_ready_i &&
                           ((wb0_valid_i && (wb0_robid_i == push_src0_robid_i)) ||
                            (wb1_valid_i && (wb1_robid_i == push_src0_robid_i)) ||
                            (wb2_valid_i && (wb2_robid_i == push_src0_robid_i)) ||
                            (wb3_valid_i && (wb3_robid_i == push_src0_robid_i)));
wire        push_s1_wbhit = !push_src1_ready_i &&
                           ((wb0_valid_i && (wb0_robid_i == push_src1_robid_i)) ||
                            (wb1_valid_i && (wb1_robid_i == push_src1_robid_i)) ||
                            (wb2_valid_i && (wb2_robid_i == push_src1_robid_i)) ||
                            (wb3_valid_i && (wb3_robid_i == push_src1_robid_i)));
wire        push_s0_early = !push_src0_ready_i && !push_s0_wbhit &&
                           ((early0_valid_i && (early0_robid_i == push_src0_robid_i)) ||
                            (early1_valid_i && (early1_robid_i == push_src0_robid_i)));
wire        push_s1_early = !push_src1_ready_i && !push_s1_wbhit &&
                           ((early0_valid_i && (early0_robid_i == push_src1_robid_i)) ||
                            (early1_valid_i && (early1_robid_i == push_src1_robid_i)));
wire [31:0] push_s0_wbdat = (wb0_valid_i && (wb0_robid_i == push_src0_robid_i)) ? wb0_data_i :
                            (wb1_valid_i && (wb1_robid_i == push_src0_robid_i)) ? wb1_data_i :
                            (wb2_valid_i && (wb2_robid_i == push_src0_robid_i)) ? wb2_data_i :
                            (wb3_valid_i && (wb3_robid_i == push_src0_robid_i)) ? wb3_data_i : 32'b0;
wire [31:0] push_s1_wbdat = (wb0_valid_i && (wb0_robid_i == push_src1_robid_i)) ? wb0_data_i :
                            (wb1_valid_i && (wb1_robid_i == push_src1_robid_i)) ? wb1_data_i :
                            (wb2_valid_i && (wb2_robid_i == push_src1_robid_i)) ? wb2_data_i :
                            (wb3_valid_i && (wb3_robid_i == push_src1_robid_i)) ? wb3_data_i : 32'b0;

assign occupancy_o = count;
assign can_accept_o = (count != `RS_MDU_SIZE);
assign head_ready = (count != 2'b0) && valid[head] &&
                    ((s0_ready[head] && s0_val_valid[head]) || s0_wbhit[head]) &&
                    ((s1_ready[head] && s1_val_valid[head]) || s1_wbhit[head]);
assign issue_valid_o = head_ready && mdu_ready_i;
assign issue_fire = issue_valid_o;

assign issue_robid_o = robid[head];
assign issue_alu_op_o = alu_op[head];
assign issue_csr_op_o = csr_op[head];
assign issue_csr_num_o = csr_num[head];
assign issue_tlb_op_o = tlb_op[head];
assign issue_wb_src_op_o = wb_src_op[head];
assign issue_src0_o = s0_wbhit[head] ? s0_wbdat[head] : s0_val[head];
assign issue_src1_o = s1_wbhit[head] ? s1_wbdat[head] : s1_val[head];

always @(posedge clk) begin
    if (reset || flush_i) begin
        head <= 1'b0;
        tail <= 1'b0;
        count <= 2'b0;
        for (i = 0; i < `RS_MDU_SIZE; i = i + 1) begin
            valid[i] <= 1'b0;
        end
    end else begin
        for (i = 0; i < `RS_MDU_SIZE; i = i + 1) begin
            if (valid[i] && !(issue_fire && (i[0] == head))) begin
                if (s0_wbhit[i]) begin
                    s0_ready[i]     <= 1'b1;
                    s0_val_valid[i] <= 1'b1;
                    s0_val[i]       <= s0_wbdat[i];
                end else if (s0_earlyhit[i]) begin
                    s0_ready[i]     <= 1'b1;
                end
                if (s1_wbhit[i]) begin
                    s1_ready[i]     <= 1'b1;
                    s1_val_valid[i] <= 1'b1;
                    s1_val[i]       <= s1_wbdat[i];
                end else if (s1_earlyhit[i]) begin
                    s1_ready[i]     <= 1'b1;
                end
            end
        end

        if (issue_fire) begin
            valid[head] <= 1'b0;
            head <= head + 1'b1;
        end

        if (push_valid_i && can_accept_o) begin
            valid[tail] <= 1'b1;
            robid[tail] <= push_robid_i;
            alu_op[tail] <= push_alu_op_i;
            csr_op[tail] <= push_csr_op_i;
            csr_num[tail] <= push_csr_num_i;
            tlb_op[tail] <= push_tlb_op_i;
            wb_src_op[tail] <= push_wb_src_op_i;
            s0_ready[tail] <= push_src0_ready_i || push_s0_wbhit || push_s0_early;
            s0_val_valid[tail] <= push_src0_ready_i || push_s0_wbhit;
            s0_val[tail] <= push_s0_wbhit ? push_s0_wbdat :
                            push_src0_ready_i ? push_src0_val_i : 32'b0;
            s0_robid[tail] <= push_src0_robid_i;
            s1_ready[tail] <= push_src1_ready_i || push_s1_wbhit || push_s1_early;
            s1_val_valid[tail] <= push_src1_ready_i || push_s1_wbhit;
            s1_val[tail] <= push_s1_wbhit ? push_s1_wbdat :
                            push_src1_ready_i ? push_src1_val_i : 32'b0;
            s1_robid[tail] <= push_src1_robid_i;
            tail <= tail + 1'b1;
        end

        case ({push_valid_i && can_accept_o, issue_fire})
            2'b10: count <= count + 2'b01;
            2'b01: count <= count - 2'b01;
            default: count <= count;
        endcase
    end
end

endmodule
