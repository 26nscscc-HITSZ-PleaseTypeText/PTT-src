// ============================================================
// fu_mdu 模块（乘除/CSR读/rdcnt/TLB维护打包 杂项执行单元，多周期非流水）
// ------------------------------------------------------------
// 功能：
// - 处理低频/多周期/需要特殊端口的指令（mariver 哲学："脏活累活都给 MDU 干"）：
//   * 乘法 mul/mulh/mulh_u    ：例化 mul.v（DSP 流水，固定 3 拍）
//   * 除法 div/div_u/mod/mod_u：例化 div.v（CLZ 快速除法，常见 5~10 拍）
//   * CSR 类 csrrd/csrwr/csrxchg：读 CSR 旧值作为写回结果（rd <- old CSR）；
//     csrwr/csrxchg 同时把"要写入 CSR 的新值"算好放 result2（提交时真正写入）
//   * rdcntvl/rdcntvh/rdcntid ：读 64 位稳定计数器/TID
//   * invtlb：把 asid（src0[9:0]）与 va 的 vppn（src1[31:13]）打包进 result2，
//     提交级取出执行 TLB 无效化（TLB 真正落地永远在提交级）
//   * tlbsrch/tlbrd/tlbwr/tlbfill：执行级无事可做，直接标完成（提交级落地）
// - 非流水：同时只处理一条（busy 反压 rs_mdu）。
//
// 端口：
// - issue_*       ：rs_mdu 发射口直连
// - mdu_ready_o   ：空闲可接收
// - csr_raddr/rdata：CSR 读口（连 csr_exception_commit_handler 的读端口，组合）
// - timer_64_i / csr_tid_i ：稳定计数器与 TID（rdcnt 用）
// - wb_*          ：写回 ROB（data=写回值，data2=CSR新值/invtlb打包）
// ============================================================
`include "mycpu.h"

module fu_mdu(
    input  wire                       clk,
    input  wire                       reset,
    input  wire                       flush_i,

    // ---------------- 发射入口（来自 rs_mdu）----------------
    input  wire                       issue_valid_i,
    input  wire [`ROB_W-1:0]          issue_robid_i,
    input  wire [`ALU_OP_NUM-1:0]     issue_alu_op_i,     // 乘除位有效
    input  wire [`CSR_OP_NUM-1:0]     issue_csr_op_i,
    input  wire [13:0]                issue_csr_num_i,
    input  wire [`TLB_OP_NUM-1:0]     issue_tlb_op_i,
    input  wire [`WB_SRC_NUM-1:0]     issue_wb_src_op_i,  // rdcnt 类选择
    input  wire [31:0]                issue_src0_i,       // rj 值
    input  wire [31:0]                issue_src1_i,       // rk/rd 值
    output wire                       mdu_ready_o,        // 空闲（rs_mdu 发射条件）

    // ---------------- CSR 读口（组合读 csr handler）----------------
    output wire [13:0]                csr_raddr_o,
    input  wire [31:0]                csr_rdata_i,

    // ---------------- 计时器 ----------------
    input  wire [63:0]                timer_64_i,
    input  wire [31:0]                csr_tid_i,

    // ---------------- 写回 ROB ----------------
    output wire                       wb_valid_o,
    output wire [`ROB_W-1:0]          wb_robid_o,
    output wire [31:0]                wb_data_o,          // 写回 rd 的值
    output wire [31:0]                wb_data2_o          // CSR 新值 / invtlb {asid,vppn} 打包
);

localparam S_IDLE = 2'd0;
localparam S_BUSY = 2'd1;
localparam S_WB   = 2'd2;

reg [1:0]              state;
reg [`ROB_W-1:0]       r_robid;
reg [`ALU_OP_NUM-1:0]  r_alu_op;
reg                    r_result_is_quotient;
reg [13:0]             r_csr_num;
reg [31:0]             r_data;
reg [31:0]             r_data2;
reg                    r_is_mul;
reg [1:0]              mul_flush_wait;

wire issue_is_mul = issue_alu_op_i[`ALU_OP_MUL_W]
                  | issue_alu_op_i[`ALU_OP_MULH_W]
                  | issue_alu_op_i[`ALU_OP_MULH_WU];
wire issue_is_div = issue_alu_op_i[`ALU_OP_DIV_W]
                  | issue_alu_op_i[`ALU_OP_DIV_WU]
                  | issue_alu_op_i[`ALU_OP_MOD_W]
                  | issue_alu_op_i[`ALU_OP_MOD_WU];
wire issue_is_csr = |issue_csr_op_i;
wire issue_is_rdcnt = issue_wb_src_op_i[`WB_SRC_CNTVL]
                    | issue_wb_src_op_i[`WB_SRC_CNTVH]
                    | issue_wb_src_op_i[`WB_SRC_TID];
wire issue_is_invtlb = issue_tlb_op_i[`TLB_OP_INVTLB_0]
                     | issue_tlb_op_i[`TLB_OP_INVTLB_1]
                     | issue_tlb_op_i[`TLB_OP_INVTLB_2]
                     | issue_tlb_op_i[`TLB_OP_INVTLB_3]
                     | issue_tlb_op_i[`TLB_OP_INVTLB_4]
                     | issue_tlb_op_i[`TLB_OP_INVTLB_5]
                     | issue_tlb_op_i[`TLB_OP_INVTLB_6];
wire issue_is_tlb = |issue_tlb_op_i;
wire issue_is_cpucfg = issue_wb_src_op_i[`WB_SRC_ALU]
                     & ~issue_is_mul & ~issue_is_div
                     & ~issue_is_csr & ~issue_is_rdcnt & ~issue_is_tlb;

wire accept = (state == S_IDLE) && (mul_flush_wait == 2'b0)
            && issue_valid_i && !flush_i;

assign mdu_ready_o = (state == S_IDLE) && (mul_flush_wait == 2'b0) && !flush_i;

function [31:0] cpucfg_value;
    input [31:0] index;
    begin
        case (index[13:0])
            14'h0001: cpucfg_value = 32'h0001_f1f4;
            14'h0002: cpucfg_value = 32'h0000_0000;
            14'h000a: cpucfg_value = 32'h0000_0005;
            14'h000b: cpucfg_value = 32'h0408_0001;
            14'h000c: cpucfg_value = 32'h0408_0001;
            14'h000d: cpucfg_value = 32'h0000_0000;
            default:  cpucfg_value = 32'h0000_0000;
        endcase
    end
endfunction

wire        mul_start = accept && issue_is_mul;
wire [63:0] mul_result;
wire        mul_done;

mul u_mul(
    .clk        (clk),
    .reset      (reset),
    .valid_i    (mul_start),
    .a_i        (issue_src0_i),
    .b_i        (issue_src1_i),
    .is_signed_i(issue_alu_op_i[`ALU_OP_MUL_W] | issue_alu_op_i[`ALU_OP_MULH_W]),
    .result_o   (mul_result),
    .done_o     (mul_done)
);

wire        div_start = accept && issue_is_div;
wire [31:0] div_quotient;
wire [31:0] div_remainder;
wire        div_done;
wire        div_busy;

div u_div(
    .clk        (clk),
    .reset      (reset),
    .flush_i    (flush_i),
    .valid_i    (div_start),
    .dividend_i (issue_src0_i),
    .divisor_i  (issue_src1_i),
    .is_signed_i(issue_alu_op_i[`ALU_OP_DIV_W] | issue_alu_op_i[`ALU_OP_MOD_W]),
    .quotient_o (div_quotient),
    .remainder_o(div_remainder),
    .done_o     (div_done),
    .busy_o     (div_busy)
);

assign csr_raddr_o = (state == S_IDLE) ? issue_csr_num_i : r_csr_num;

wire [31:0] csr_old = csr_rdata_i;
wire [31:0] fast_data = issue_is_csr                  ? csr_old
                      : issue_wb_src_op_i[`WB_SRC_CNTVL] ? timer_64_i[31:0]
                      : issue_wb_src_op_i[`WB_SRC_CNTVH] ? timer_64_i[63:32]
                      : issue_wb_src_op_i[`WB_SRC_TID]   ? csr_tid_i
                      : issue_is_cpucfg               ? cpucfg_value(issue_src0_i)
                      : 32'b0;
wire [31:0] fast_data2 = issue_csr_op_i[`CSR_OP_CSRWR]   ? issue_src1_i
                       : issue_csr_op_i[`CSR_OP_CSRXCHG] ? ((csr_old & ~issue_src0_i) | (issue_src1_i & issue_src0_i))
                       : issue_is_invtlb                 ? {issue_src1_i[31:13], 3'b0, issue_src0_i[9:0]}
                       : 32'b0;

always @(posedge clk) begin
    if (reset) begin
        state <= S_IDLE;
        mul_flush_wait <= 2'b0;
        r_is_mul <= 1'b0;
        r_result_is_quotient <= 1'b0;
    end else if (flush_i) begin
        state <= S_IDLE;
        r_is_mul <= 1'b0;
        if ((state == S_BUSY) && r_is_mul) begin
            mul_flush_wait <= 2'd3;
        end
    end else begin
        if (mul_flush_wait != 2'b0) begin
            mul_flush_wait <= mul_flush_wait - 2'b01;
        end

        case (state)
            S_IDLE: begin
                if (accept) begin
                    r_robid   <= issue_robid_i;
                    r_alu_op  <= issue_alu_op_i;
                    r_csr_num <= issue_csr_num_i;
                    r_is_mul  <= issue_is_mul;
                    r_result_is_quotient <= issue_alu_op_i[`ALU_OP_DIV_W]
                                          | issue_alu_op_i[`ALU_OP_DIV_WU];
                    if (issue_is_mul || issue_is_div) begin
                        state <= S_BUSY;
                    end else begin
                        r_data  <= fast_data;
                        r_data2 <= fast_data2;
                        state   <= S_WB;
                    end
                end
            end
            S_BUSY: begin
                if (mul_done && r_is_mul) begin
                    r_data  <= r_alu_op[`ALU_OP_MUL_W] ? mul_result[31:0] : mul_result[63:32];
                    r_data2 <= 32'b0;
                    state   <= S_WB;
                end else if (div_done && !r_is_mul) begin
                    r_data  <= r_result_is_quotient ? div_quotient : div_remainder;
                    r_data2 <= 32'b0;
                    state   <= S_WB;
                end
            end
            S_WB: begin
                state <= S_IDLE;
                r_is_mul <= 1'b0;
            end
            default: begin
                state <= S_IDLE;
                r_is_mul <= 1'b0;
            end
        endcase
    end
end

assign wb_valid_o = (state == S_WB) && !flush_i;
assign wb_robid_o = r_robid;
assign wb_data_o  = r_data;
assign wb_data2_o = r_data2;

wire fu_mdu_lint = div_busy;

endmodule
