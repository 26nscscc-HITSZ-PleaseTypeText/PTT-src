// ============================================================
// rs_mem 模块（访存保留站，`RS_MEM_SIZE 项，FIFO + 有限 load 越过）
// ------------------------------------------------------------
// 功能：
// - 缓存等待操作数的访存类指令（load/store/ll/sc/cacop）。
// - 默认按程序序发射；例外：普通 load 可越过前方**未就绪的普通 load**。
// - store / ll / sc / cacop 仍为序屏障：前方有未发射的屏障项时不可越过。
//   （无地址消歧时不能让 load 越过未知/未发 store。）
// - 越过时在发射拍把选中项与队头交换，再按 head 出队，保持 FIFO 紧凑。
// - 唤醒机制与 rs_alu 相同（4 路写回总线 + early0/1/2 提前唤醒）。
//
// 端口：与 rs_alu 同构，差异：
// - bundle 为 mem_op/is_cacop/imm（无 br 相关）
// - 发射口对接 lsu，lsu_ready_i 反压（LSU 两级流水可能停顿）
// ============================================================
`include "mycpu.h"

module rs_mem(
    input  wire                       clk,
    input  wire                       reset,
    input  wire                       flush_i,

    // ---------------- 入站（dispatch）----------------
    input  wire                       push_valid_i,
    input  wire [`ROB_W-1:0]          push_robid_i,
    input  wire [31:0]                push_pc_i,
    input  wire [`MEM_OP_NUM-1:0]     push_mem_op_i,
    input  wire                       push_is_cacop_i,
    input  wire                       push_src0_ready_i,   // src0 = 基址 rj
    input  wire [31:0]                push_src0_val_i,
    input  wire [`ROB_W-1:0]          push_src0_robid_i,
    input  wire                       push_src1_ready_i,   // src1 = store 数据 rd
    input  wire [31:0]                push_src1_val_i,
    input  wire [`ROB_W-1:0]          push_src1_robid_i,
    input  wire [31:0]                push_imm_i,          // si12/si14 偏移

    output wire                       can_accept_o,
    output wire [`RS_MEM_OCC_W-1:0]   occupancy_o,

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

    // ---------------- 提前唤醒总线 ×3（early0/1=ALU；early2=LSU DC 命中，V3.4）----------------
    input  wire                       early0_valid_i,
    input  wire [`ROB_W-1:0]          early0_robid_i,
    input  wire                       early1_valid_i,
    input  wire [`ROB_W-1:0]          early1_robid_i,
    input  wire                       early2_valid_i,
    input  wire [`ROB_W-1:0]          early2_robid_i,

    // ---------------- 发射口（到 lsu）----------------
    output wire                       issue_valid_o,
    output wire [`ROB_W-1:0]          issue_robid_o,
    output wire [31:0]                issue_pc_o,
    output wire [`MEM_OP_NUM-1:0]     issue_mem_op_o,
    output wire                       issue_is_cacop_o,
    output wire [31:0]                issue_base_o,        // 基址（src0 捕获值）
    output wire [31:0]                issue_wdata_o,       // store 数据（src1 捕获值）
    output wire [31:0]                issue_imm_o,
    input  wire                       lsu_ready_i          // LSU 本拍可接收（AGU 级空闲）
);

// 设计说明（已实现，参考 mariver station.v 的 MU 保留站部分 + 有限 load 越过）
//
// 存储结构：
//      与 rs_alu 类似（valid/robid/op/双源/imm），但组织成 FIFO：
//      head/tail 指针（4 项 -> 2bit）；入站写 tail 项 tail++，发射出队 head++。
//
// 唤醒与数据捕获：与 rs_alu 完全相同（4 路 wb 总线逐项逐源比较捕获，
//      入站同拍旁路同样要做）。
//
// 发射（队头优先 + 有限 load 越过，见头注）：
//      默认发队头：issue = valid[head] && 双源 ready && lsu_ready_i；
//      队头是未就绪普通 load 时，可从其后连续的普通 load 中选一条就绪的，
//      发射拍与队头【交换】后按 head 出队（保持 FIFO 紧凑）；
//      store/ll/sc/cacop 是序屏障——不可被越过，自身也不越过别人；
//      store 即使 src1（数据）未就绪也不能让位给后面的 load ——
//      无地址消歧时这是内存序正确性的底线。
//
// 冲刷：flush_i 清空 head/tail/valid。
//
// 坑点提示：
//      1. lsu_ready_i 为 0 时队头保持，不要丢发射（脉冲式发射 + not-ready 丢失
//         是常见 bug，发射条件里与上 lsu_ready 即可避免）。
//      2. load 提前唤醒（V3.4：lsu DC 命中限定 early2）已接入；唤醒的是依赖
//         load 结果的指令，本站自身的操作数捕获逻辑不变。
//      3. 若想 load 越过前面的 store，必须加 store 地址比较（内存消歧）+
//         违例恢复，工作量大，务必先评估收益。

reg                     valid [0:`RS_MEM_SIZE-1];
reg [`ROB_W-1:0]        robid [0:`RS_MEM_SIZE-1];
reg [31:0]              pc [0:`RS_MEM_SIZE-1];
reg [`MEM_OP_NUM-1:0]   mem_op [0:`RS_MEM_SIZE-1];
reg                     is_cacop [0:`RS_MEM_SIZE-1];
reg                     s0_ready [0:`RS_MEM_SIZE-1];
reg                     s0_val_valid [0:`RS_MEM_SIZE-1];
reg [31:0]              s0_val [0:`RS_MEM_SIZE-1];
reg [`ROB_W-1:0]        s0_robid [0:`RS_MEM_SIZE-1];
reg                     s1_ready [0:`RS_MEM_SIZE-1];
reg                     s1_val_valid [0:`RS_MEM_SIZE-1];
reg [31:0]              s1_val [0:`RS_MEM_SIZE-1];
reg [`ROB_W-1:0]        s1_robid [0:`RS_MEM_SIZE-1];
reg [31:0]              imm [0:`RS_MEM_SIZE-1];
reg [`RS_MEM_IDX_W-1:0] head;
reg [`RS_MEM_IDX_W-1:0] tail;
reg [`RS_MEM_OCC_W-1:0] count;

integer i;
integer a;
wire issue_fire;
reg  [`RS_MEM_IDX_W-1:0] issue_idx;
reg                      issue_sel_valid;
reg                      scan_stop;
reg  [`RS_MEM_IDX_W-1:0] scan_idx;
wire                     issue_need_swap;

wire            s0_wb_match [0:`RS_MEM_SIZE-1];
wire            s1_wb_match [0:`RS_MEM_SIZE-1];
wire            s0_wbhit [0:`RS_MEM_SIZE-1];
wire            s1_wbhit [0:`RS_MEM_SIZE-1];
wire            s0_earlyhit [0:`RS_MEM_SIZE-1];
wire            s1_earlyhit [0:`RS_MEM_SIZE-1];
wire [31:0]     s0_wbdat [0:`RS_MEM_SIZE-1];
wire [31:0]     s1_wbdat [0:`RS_MEM_SIZE-1];
wire            entry_ready [0:`RS_MEM_SIZE-1];
wire            is_ord_barrier [0:`RS_MEM_SIZE-1]; // store/ll/sc/cacop：不可被越过
genvar gw;
generate
for (gw = 0; gw < `RS_MEM_SIZE; gw = gw + 1) begin : g_wake
    assign s0_wb_match[gw] = (wb0_valid_i && (wb0_robid_i == s0_robid[gw])) ||
                             (wb1_valid_i && (wb1_robid_i == s0_robid[gw])) ||
                             (wb2_valid_i && (wb2_robid_i == s0_robid[gw])) ||
                             (wb3_valid_i && (wb3_robid_i == s0_robid[gw]));
    assign s1_wb_match[gw] = (wb0_valid_i && (wb0_robid_i == s1_robid[gw])) ||
                             (wb1_valid_i && (wb1_robid_i == s1_robid[gw])) ||
                             (wb2_valid_i && (wb2_robid_i == s1_robid[gw])) ||
                             (wb3_valid_i && (wb3_robid_i == s1_robid[gw]));
    // val_valid 冻结真值；仅 early 时允许 WB 再捕获/旁路
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
    // 仅 early 的 ready 不够：必须已有真值或本拍 WB 可旁路
    assign entry_ready[gw] = valid[gw] &&
                             ((s0_ready[gw] && s0_val_valid[gw]) || s0_wbhit[gw]) &&
                             ((s1_ready[gw] && s1_val_valid[gw]) || s1_wbhit[gw]);
    assign is_ord_barrier[gw] = is_cacop[gw]
                             || mem_op[gw][`MEM_OP_ST_W] || mem_op[gw][`MEM_OP_ST_B]
                             || mem_op[gw][`MEM_OP_ST_H] || mem_op[gw][`MEM_OP_SC_W]
                             || mem_op[gw][`MEM_OP_LL_W];
end
endgenerate

// 年龄序扫描：
// - 普通 load 可越过前方未就绪的普通 load
// - store/ll/sc/cacop 仅在其已是队内最老项时发射（不可越过未发 load）
// - 前方有未就绪屏障则停止
always @(*) begin
    issue_idx = head;
    issue_sel_valid = 1'b0;
    scan_stop = 1'b0;
    for (a = 0; a < `RS_MEM_SIZE; a = a + 1) begin
        scan_idx = head + a[`RS_MEM_IDX_W-1:0];
        if (!issue_sel_valid && !scan_stop && (a[`RS_MEM_OCC_W-1:0] < count) && valid[scan_idx]) begin
            if (entry_ready[scan_idx]) begin
                if (!is_ord_barrier[scan_idx] || (a == 0)) begin
                    issue_idx = scan_idx;
                    issue_sel_valid = 1'b1;
                end else begin
                    // 更年轻的就绪屏障：不能越过前方未发 load，停扫
                    scan_stop = 1'b1;
                end
            end else if (is_ord_barrier[scan_idx]) begin
                scan_stop = 1'b1;
            end
        end
    end
end

assign issue_need_swap = issue_fire && (issue_idx != head);

// push 口的唤醒命中/旁路（参数是端口信号，非数组变址；同样内联以彻底去除 function）
// push 同理带 !push_srcX_ready 门控：ready-from-ARF 的入站操作数直接取 push_srcX_val，
// 不允许被 tag=0 的误命中改写。
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
assign can_accept_o = (count != `RS_MEM_SIZE);
assign issue_valid_o = issue_sel_valid && lsu_ready_i;
assign issue_fire = issue_valid_o;

assign issue_robid_o = robid[issue_idx];
assign issue_pc_o = pc[issue_idx];
assign issue_mem_op_o = mem_op[issue_idx];
assign issue_is_cacop_o = is_cacop[issue_idx];
assign issue_base_o = s0_wbhit[issue_idx] ? s0_wbdat[issue_idx] : s0_val[issue_idx];
assign issue_wdata_o = s1_wbhit[issue_idx] ? s1_wbdat[issue_idx] : s1_val[issue_idx];
assign issue_imm_o = imm[issue_idx];

always @(posedge clk) begin
    if (reset || flush_i) begin
        head <= {`RS_MEM_IDX_W{1'b0}};
        tail <= {`RS_MEM_IDX_W{1'b0}};
        count <= {`RS_MEM_OCC_W{1'b0}};
        for (i = 0; i < `RS_MEM_SIZE; i = i + 1) begin
            valid[i] <= 1'b0;
        end
    end else begin
        for (i = 0; i < `RS_MEM_SIZE; i = i + 1) begin
            // 发射项出队；swap 时 head 内容写入 issue_idx（见下），两边都跳过常规唤醒
            if (valid[i]
                && !(issue_fire && (i[`RS_MEM_IDX_W-1:0] == issue_idx))
                && !(issue_need_swap && (i[`RS_MEM_IDX_W-1:0] == head))) begin
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
            if (issue_need_swap) begin
                // 未就绪队头挪到被越过槽，保持年龄序；本拍发射项按已位于 head 出队
                robid[issue_idx]     <= robid[head];
                pc[issue_idx]        <= pc[head];
                mem_op[issue_idx]    <= mem_op[head];
                is_cacop[issue_idx]  <= is_cacop[head];
                s0_ready[issue_idx]  <= s0_ready[head] || s0_wbhit[head] || s0_earlyhit[head];
                s0_val_valid[issue_idx] <= s0_val_valid[head] || s0_wbhit[head];
                s0_val[issue_idx]    <= s0_wbhit[head] ? s0_wbdat[head] : s0_val[head];
                s0_robid[issue_idx]  <= s0_robid[head];
                s1_ready[issue_idx]  <= s1_ready[head] || s1_wbhit[head] || s1_earlyhit[head];
                s1_val_valid[issue_idx] <= s1_val_valid[head] || s1_wbhit[head];
                s1_val[issue_idx]    <= s1_wbhit[head] ? s1_wbdat[head] : s1_val[head];
                s1_robid[issue_idx]  <= s1_robid[head];
                imm[issue_idx]       <= imm[head];
                valid[issue_idx]     <= 1'b1;
            end
            valid[head] <= 1'b0;
            head <= head + {{(`RS_MEM_IDX_W-1){1'b0}}, 1'b1};
        end

        if (push_valid_i && can_accept_o) begin
            valid[tail] <= 1'b1;
            robid[tail] <= push_robid_i;
            pc[tail] <= push_pc_i;
            mem_op[tail] <= push_mem_op_i;
            is_cacop[tail] <= push_is_cacop_i;
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
            imm[tail] <= push_imm_i;
            tail <= tail + {{(`RS_MEM_IDX_W-1){1'b0}}, 1'b1};
        end

        case ({push_valid_i && can_accept_o, issue_fire})
            2'b10: count <= count + {{(`RS_MEM_OCC_W-1){1'b0}}, 1'b1};
            2'b01: count <= count - {{(`RS_MEM_OCC_W-1){1'b0}}, 1'b1};
            default: count <= count;
        endcase
    end
end

`ifdef SYNTHESIS
// synthesis translate_off
reg [63:0] rsm_full_stall_cyc;
reg [63:0] rsm_src_stall_cyc;
reg [63:0] rsm_lsu_stall_cyc;
always @(posedge clk) begin
    if (reset) begin
        rsm_full_stall_cyc <= 64'd0;
        rsm_src_stall_cyc  <= 64'd0;
        rsm_lsu_stall_cyc  <= 64'd0;
    end else if (!flush_i) begin
        if (!can_accept_o && push_valid_i)
            rsm_full_stall_cyc <= rsm_full_stall_cyc + 64'd1;
        if ((count != {`RS_MEM_OCC_W{1'b0}}) && !issue_sel_valid)
            rsm_src_stall_cyc <= rsm_src_stall_cyc + 64'd1;
        if (issue_sel_valid && !lsu_ready_i)
            rsm_lsu_stall_cyc <= rsm_lsu_stall_cyc + 64'd1;
    end
end
// synthesis translate_on
`endif

endmodule
