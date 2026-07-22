// ============================================================
// rs_alu 模块（ALU 保留站，`RS_ALU_SIZE 项，乱序发射，顶层例化 2 份）
// ------------------------------------------------------------
// 功能：
// - 缓存等待操作数的 ALU/分支指令；监听 4 路写回唤醒总线捕获操作数；
//   每拍从"两个操作数都就绪"的项中选出"最老的"一条发射给 fu_alu。
// - 乱序发射是本架构 IPC 的最大来源（mariver 实证 +17%）：
//   后进入的指令只要操作数先就绪就可以先执行。
// - "唤醒"与"数据"分离的概念（重要！）：
//   * 唤醒（wakeup）：得知"某 robid 的结果即将/已经可用"，置 ready 位；
//   * 数据捕获（capture）：从写回总线上把 32bit 数据真正存进本项。
//   写回唤醒两者同拍完成；提前唤醒（early0/1，已接通）两者分离——
//   唤醒早 1 拍置 ready，数据在下拍生产者写回时由 WB 旁路补齐。
//
// 端口：
// - push_*          ：dispatch 入站口（一拍最多 1 条）
// - can_accept/occupancy ：空位信息
// - wb0~3_*         ：4 路写回唤醒总线（alu0/alu1/mem/mdu）
// - early0~2_*      ：提前唤醒（alu0/alu1 发射拍；early2=LSU DC 命中限定）
// - issue_*         ：发射口（到 fu_alu，ALU 恒可接收）
// - flush_i         ：全局冲刷清空
//
// early 语义：只置 ready、不带数据；下一拍生产者写回时由 WB 旁路补数。
// 用 val_valid 区分「ARF/WB 已捕获真值」与「仅 early 唤醒」，避免
// ready 置位后门控死 WB 捕获，也避免 tag don't-care 误覆盖 ARF 值。
// ============================================================
`include "mycpu.h"

module rs_alu(
    input  wire                       clk,
    input  wire                       reset,
    input  wire                       flush_i,

    // ---------------- 入站（dispatch）----------------
    input  wire                       push_valid_i,
    input  wire [`ROB_W-1:0]          push_robid_i,
    input  wire [31:0]                push_pc_i,
    input  wire [`ALU_OP_NUM-1:0]     push_alu_op_i,
    input  wire [`BR_OP_NUM-1:0]      push_br_op_i,
    input  wire                       push_src0_ready_i,
    input  wire [31:0]                push_src0_val_i,
    input  wire [`ROB_W-1:0]          push_src0_robid_i,
    input  wire                       push_src1_ready_i,
    input  wire [31:0]                push_src1_val_i,
    input  wire [`ROB_W-1:0]          push_src1_robid_i,
    input  wire [31:0]                push_imm_i,
    input  wire                       push_use_imm_i,
    input  wire [31:0]                push_br_offs_i,

    output wire                       can_accept_o,      // 有空位
    output wire [`RS_ALU_OCC_W-1:0]   occupancy_o,       // 当前占用项数（dispatch 负载均衡）

    // ---------------- 写回唤醒总线 ×4（带数据）----------------
    input  wire                       wb0_valid_i,       // fu_alu0 写回
    input  wire [`ROB_W-1:0]          wb0_robid_i,
    input  wire [31:0]                wb0_data_i,
    input  wire                       wb1_valid_i,       // fu_alu1 写回
    input  wire [`ROB_W-1:0]          wb1_robid_i,
    input  wire [31:0]                wb1_data_i,
    input  wire                       wb2_valid_i,       // lsu 写回
    input  wire [`ROB_W-1:0]          wb2_robid_i,
    input  wire [31:0]                wb2_data_i,
    input  wire                       wb3_valid_i,       // fu_mdu 写回
    input  wire [`ROB_W-1:0]          wb3_robid_i,
    input  wire [31:0]                wb3_data_i,

    // ---------------- 提前唤醒总线 ×3（无数据；early0/1/2 均已接通）----------------
    input  wire                       early0_valid_i,    // fu_alu0 发射拍唤醒
    input  wire [`ROB_W-1:0]          early0_robid_i,
    input  wire                       early1_valid_i,    // fu_alu1 发射拍唤醒
    input  wire [`ROB_W-1:0]          early1_robid_i,
    input  wire                       early2_valid_i,    // lsu DC 命中限定唤醒（V3.4）
    input  wire [`ROB_W-1:0]          early2_robid_i,

    // ---------------- 发射口（到 fu_alu，组合）----------------
    output wire                       issue_valid_o,
    output wire [`ROB_W-1:0]          issue_robid_o,
    output wire [31:0]                issue_pc_o,
    output wire [`ALU_OP_NUM-1:0]     issue_alu_op_o,
    output wire [`BR_OP_NUM-1:0]      issue_br_op_o,
    output wire [31:0]                issue_src0_o,      // 已捕获的最终操作数值
    output wire [31:0]                issue_src1_o,
    output wire [31:0]                issue_imm_o,
    output wire                       issue_use_imm_o,
    output wire [31:0]                issue_br_offs_o
);

// 设计说明（已实现，参考 mariver station.v 的 ALU 保留站部分：
//      prior 时间戳选择 388~398 行、唤醒 659~666 行、旁路 479~533 行）
//
// 存储结构（每项，全 reg）：
//      valid, robid[`ROB_W], pc[32], alu_op[`ALU_OP_NUM], br_op[`BR_OP_NUM],
//      s0_ready/s0_val_valid, s0_val[32], s0_robid[`ROB_W]（s1 同理），
//      imm[32], use_imm, br_offs[32],
//      prior[1:0]   // 年龄时间戳：新入站项=3，有项发射后其余项-1（数值小=老）
//
// 入站：
//      can_accept_o = 有 valid=0 的空项；occupancy_o = valid 计数。
//      push 时写第一个空项，prior 置最大（最年轻）。
//
// 唤醒与数据捕获（每项每源监听 4 路 wb 总线 + early0/1）：
//      wb 命中：置 sX_ready 并捕获数据；early 命中：只置 ready（val_valid=0），
//      数据在下拍 WB 旁路补齐。
//      同拍入站+唤醒：push 数据进来的同拍总线上恰有它等的 robid ——
//      入站数据要先过一遍同样的旁路比较再写入（mariver 的 bypassin 逻辑），
//      否则错过唤醒永远等不到（经典死锁坑！）。
//
// 发射选择（组合，oldest-first）：
//      cand[i] = valid[i] && s0_ready[i] && s1_ready[i]（发射拍若 val 未捕获，
//      从 WB 总线旁路取数）；从 cand 中选 prior 最小（最老）的一项发射。
//      发射成功（ALU 恒接收）当拍清该项 valid；比发射项年轻的项 prior-1。
//
reg                     valid [0:`RS_ALU_SIZE-1];
reg [`ROB_W-1:0]        robid [0:`RS_ALU_SIZE-1];
reg [31:0]              pc [0:`RS_ALU_SIZE-1];
reg [`ALU_OP_NUM-1:0]   alu_op [0:`RS_ALU_SIZE-1];
reg [`BR_OP_NUM-1:0]    br_op [0:`RS_ALU_SIZE-1];
reg                     s0_ready [0:`RS_ALU_SIZE-1];
reg                     s0_val_valid [0:`RS_ALU_SIZE-1];
reg [31:0]              s0_val [0:`RS_ALU_SIZE-1];
reg [`ROB_W-1:0]        s0_robid [0:`RS_ALU_SIZE-1];
reg                     s1_ready [0:`RS_ALU_SIZE-1];
reg                     s1_val_valid [0:`RS_ALU_SIZE-1];
reg [31:0]              s1_val [0:`RS_ALU_SIZE-1];
reg [`ROB_W-1:0]        s1_robid [0:`RS_ALU_SIZE-1];
reg [31:0]              imm [0:`RS_ALU_SIZE-1];
reg                     use_imm [0:`RS_ALU_SIZE-1];
reg [31:0]              br_offs [0:`RS_ALU_SIZE-1];
reg [`RS_ALU_IDX_W-1:0] prior [0:`RS_ALU_SIZE-1];

integer i;
reg [`RS_ALU_IDX_W-1:0] free_idx;
reg [`RS_ALU_IDX_W-1:0] issue_idx;
reg                     issue_sel_valid;

// 每项唤醒命中/旁路数据用【generate 预计算 wire 数组】而非 function 调用：
// xsim 在 continuous assign 里对"带可变下标的 function"存在求值缺陷（见 rob.v），
// 会返回上一次求值下标的旧值。改为纯数组变址读（s0_wbhit[idx] 等）即可规避。
wire            s0_wb_match [0:`RS_ALU_SIZE-1];
wire            s1_wb_match [0:`RS_ALU_SIZE-1];
wire            s0_wbhit [0:`RS_ALU_SIZE-1];
wire            s1_wbhit [0:`RS_ALU_SIZE-1];
wire            s0_earlyhit [0:`RS_ALU_SIZE-1];
wire            s1_earlyhit [0:`RS_ALU_SIZE-1];
wire [31:0]     s0_wbdat [0:`RS_ALU_SIZE-1];
wire [31:0]     s1_wbdat [0:`RS_ALU_SIZE-1];
genvar gw;
generate
for (gw = 0; gw < `RS_ALU_SIZE; gw = gw + 1) begin : g_wake
    // val_valid=1：ARF/WB 已有真值，冻结（防 tag don't-care 被 wb0=robid0 误覆盖）。
    // val_valid=0：尚未捕获数据（含仅 early 唤醒），允许 WB 匹配捕获/旁路。
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
    // early0/1=ALU 发射拍；early2=LSU DC 命中限定（见 lsu early_wakeup）
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

// push 口：WB 优先于 early；已带 ready 的源不再匹配 tag
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

// occupancy：按项累加，深度随 RS_ALU_SIZE 变化
reg [`RS_ALU_OCC_W-1:0] occ_cnt;
always @(*) begin
    occ_cnt = {`RS_ALU_OCC_W{1'b0}};
    for (i = 0; i < `RS_ALU_SIZE; i = i + 1)
        occ_cnt = occ_cnt + {{(`RS_ALU_OCC_W-1){1'b0}}, valid[i]};
end
assign occupancy_o = occ_cnt;
assign can_accept_o = (occupancy_o != `RS_ALU_SIZE);
wire [`RS_ALU_OCC_W-1:0] push_prior_next = occupancy_o
                                         - {{(`RS_ALU_OCC_W-1){1'b0}}, issue_sel_valid};

always @(*) begin : find_free
    integer fi;
    free_idx = {`RS_ALU_IDX_W{1'b0}};
    for (fi = 0; fi < `RS_ALU_SIZE; fi = fi + 1) begin
        if (!valid[fi]) begin
            free_idx = fi[`RS_ALU_IDX_W-1:0];
            disable find_free;
        end
    end
end

always @(*) begin
    issue_idx = {`RS_ALU_IDX_W{1'b0}};
    issue_sel_valid = 1'b0;
    for (i = 0; i < `RS_ALU_SIZE; i = i + 1) begin
        // 可发：已有真值，或本拍 WB 旁路可补。仅 early 的 ready 不够（无数据）。
        if (valid[i] &&
            ((s0_ready[i] && s0_val_valid[i]) || s0_wbhit[i]) &&
            ((s1_ready[i] && s1_val_valid[i]) || s1_wbhit[i]) &&
            (!issue_sel_valid || (prior[i] < prior[issue_idx]))) begin
            issue_idx = i[`RS_ALU_IDX_W-1:0];
            issue_sel_valid = 1'b1;
        end
    end
end

assign issue_valid_o = issue_sel_valid;
assign issue_robid_o = robid[issue_idx];
assign issue_pc_o = pc[issue_idx];
assign issue_alu_op_o = alu_op[issue_idx];
assign issue_br_op_o = br_op[issue_idx];
// 真值已捕获 → 用 s_val；仅 early 唤醒（val_valid=0）→ 本拍 WB 旁路补数
wire s0_issue_wb = s0_wbhit[issue_idx];
wire s1_issue_wb = s1_wbhit[issue_idx];
assign issue_src0_o = s0_issue_wb ? s0_wbdat[issue_idx] : s0_val[issue_idx];
assign issue_src1_o = s1_issue_wb ? s1_wbdat[issue_idx] : s1_val[issue_idx];
assign issue_imm_o = imm[issue_idx];
assign issue_use_imm_o = use_imm[issue_idx];
assign issue_br_offs_o = br_offs[issue_idx];

always @(posedge clk) begin
    if (reset || flush_i) begin
        for (i = 0; i < `RS_ALU_SIZE; i = i + 1) begin
            valid[i] <= 1'b0;
            prior[i] <= {`RS_ALU_IDX_W{1'b0}};
        end
    end else begin
        if (issue_sel_valid) begin
            valid[issue_idx] <= 1'b0;
            for (i = 0; i < `RS_ALU_SIZE; i = i + 1) begin
                if (valid[i] && (i[`RS_ALU_IDX_W-1:0] != issue_idx) && (prior[i] > prior[issue_idx])) begin
                    prior[i] <= prior[i] - {{(`RS_ALU_IDX_W-1){1'b0}}, 1'b1};
                end
            end
        end

        for (i = 0; i < `RS_ALU_SIZE; i = i + 1) begin
            if (valid[i] && !(issue_sel_valid && (i[`RS_ALU_IDX_W-1:0] == issue_idx))) begin
                // WB 优先：补齐数据；early 仅置 ready（下一拍再靠 WB 旁路）
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

        if (push_valid_i && can_accept_o) begin
            valid[free_idx] <= 1'b1;
            robid[free_idx] <= push_robid_i;
            pc[free_idx] <= push_pc_i;
            alu_op[free_idx] <= push_alu_op_i;
            br_op[free_idx] <= push_br_op_i;
            s0_ready[free_idx] <= push_src0_ready_i || push_s0_wbhit || push_s0_early;
            s0_val_valid[free_idx] <= push_src0_ready_i || push_s0_wbhit;
            s0_val[free_idx] <= push_s0_wbhit ? push_s0_wbdat :
                                push_src0_ready_i ? push_src0_val_i : 32'b0;
            s0_robid[free_idx] <= push_src0_robid_i;
            s1_ready[free_idx] <= push_src1_ready_i || push_s1_wbhit || push_s1_early;
            s1_val_valid[free_idx] <= push_src1_ready_i || push_s1_wbhit;
            s1_val[free_idx] <= push_s1_wbhit ? push_s1_wbdat :
                                push_src1_ready_i ? push_src1_val_i : 32'b0;
            s1_robid[free_idx] <= push_src1_robid_i;
            imm[free_idx] <= push_imm_i;
            use_imm[free_idx] <= push_use_imm_i;
            br_offs[free_idx] <= push_br_offs_i;
            prior[free_idx] <= push_prior_next[`RS_ALU_IDX_W-1:0];
        end
    end
end

endmodule
