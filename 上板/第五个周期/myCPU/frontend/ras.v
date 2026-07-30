// ============================================================
// ras 模块（Return Address Stack，返回地址栈，双栈结构）
// ------------------------------------------------------------
// 实现说明：
// - 前端推测栈（BPU 预测 CALL push / RET pop）+ 提交栈（commit 维护，恒正确）；
// - flush 时前端栈视图恢复为提交栈（指针/计数对拷 + 覆盖位图清零）；
// - 同拍 flush 与 cmt_push/pop：先算提交栈新值再恢复（用 next 值）；
// - 栈满环形回绕覆盖最旧项（深调用链精度下降可接受）。
//
// 存储实现（LUTRAM 化，行为与"整栈一拍对拷"版等价或更优）：
// - 两个栈体均为 1 写口 + 异步读的分布式 RAM（拍拷贝会强制全 FF，弃用）；
// - spec_ovl 位图（FF）标记"该项已被推测栈覆盖"：置位读 spec 体，
//   否则读提交体；flush 只清位图 + 拷指针/计数，一拍完成；
// - 唯一行为差异：flush 后提交栈在该槽再次 push 时，未覆盖槽的推测读
//   会看到新提交值而非旧快照——该情形仅出现在"BPU 漏识别 call"的
//   错位场景，两种取值都是纯预测提示，且新提交值更准（RAS 只影响
//   预测准确率，不影响正确性）。
// ============================================================
`include "mycpu.h"

module ras(
    input  wire                clk,
    input  wire                reset,

    // ---------------- 冲刷恢复 ----------------
    input  wire                flush_i,            // 前端栈视图恢复为提交栈

    // ---------------- 前端推测栈 ----------------
    input  wire                spec_push_i,        // BPU 预测到 CALL
    input  wire [31:0]         spec_push_addr_i,   // 返回地址（call 块 fall_through）
    input  wire                spec_pop_i,         // BPU 预测到 RET
    output wire [31:0]         top_addr_o,         // 栈顶（RET 预测目标）
    output wire                empty_o,            // 栈空（空时 RET 退化用 FTB fall_through）
    // ---------------- FTQ checkpoints / predecode recovery ----------------
    input  wire                checkpoint_save_i,
    input  wire [`FTQ_W-1:0]   checkpoint_id_i,
    input  wire [`FTQ_W-1:0]   checkpoint_query_id_i,
    output wire [31:0]         checkpoint_top_addr_o,
    output wire                checkpoint_nonempty_o,
    input  wire                restore_i,
    input  wire [`FTQ_W-1:0]   restore_id_i,
    input  wire                restore_push_i,
    input  wire                restore_pop_i,
    input  wire [31:0]         restore_push_addr_i,

    // ---------------- 提交栈 ----------------
    input  wire                cmt_push_i,         // commit 提交 call
    input  wire [31:0]         cmt_push_addr_i,    // 真实返回地址（call PC+4）
    input  wire                cmt_pop_i           // commit 提交 ret
);

(* ram_style = "distributed" *) reg [31:0] spec_stack [0:`RAS_DEPTH-1];
(* ram_style = "distributed" *) reg [31:0] cmt_stack  [0:`RAS_DEPTH-1];
reg [`RAS_DEPTH-1:0] spec_ovl;              // 推测覆盖位图（flush 一拍清零）
reg [`RAS_W-1:0]  spec_ptr,  cmt_ptr;       // 指向当前栈顶
reg [`RAS_W:0]    spec_cnt,  cmt_cnt;       // 计数（饱和在 DEPTH）
(* ram_style = "distributed" *) reg [`RAS_W-1:0] checkpoint_ptr [0:`FTQ_SIZE-1];
(* ram_style = "distributed" *) reg [`RAS_W:0]   checkpoint_cnt [0:`FTQ_SIZE-1];
(* ram_style = "distributed" *) reg [31:0]
    checkpoint_top [0:`FTQ_SIZE-1];
reg [`FTQ_SIZE-1:0] checkpoint_nonempty;

// 提交栈 next 值（flush 同拍先提交后恢复）
wire [`RAS_W-1:0] cmt_ptr_n = cmt_push_i ? (cmt_ptr + 1'b1)
                            : (cmt_pop_i && (cmt_cnt != 0)) ? (cmt_ptr - 1'b1)
                            : cmt_ptr;
wire [`RAS_W:0]   cmt_cnt_n = cmt_push_i ? ((cmt_cnt == `RAS_DEPTH) ? cmt_cnt : (cmt_cnt + 1'b1))
                            : (cmt_pop_i && (cmt_cnt != 0)) ? (cmt_cnt - 1'b1)
                            : cmt_cnt;

// 栈顶读：推测覆盖过读 spec 体，否则读提交体（异步读，LUTRAM 多读口由综合复制）
assign top_addr_o = spec_ovl[spec_ptr] ? spec_stack[spec_ptr] : cmt_stack[spec_ptr];
assign empty_o    = (spec_cnt == 0);
assign checkpoint_top_addr_o = checkpoint_top[checkpoint_query_id_i];
assign checkpoint_nonempty_o = checkpoint_nonempty[checkpoint_query_id_i];

wire [`RAS_W-1:0] spec_wr_idx = spec_ptr + 1'b1;
wire [`RAS_W-1:0] cmt_wr_idx  = cmt_ptr + 1'b1;
wire [`RAS_W-1:0] restore_ptr = checkpoint_ptr[restore_id_i];
wire [`RAS_W:0]   restore_cnt = checkpoint_cnt[restore_id_i];
wire [`RAS_W-1:0] restore_wr_idx = restore_ptr + 1'b1;
wire [`RAS_W:0]   restore_cnt_push = (restore_cnt == `RAS_DEPTH)
                                   ? restore_cnt : (restore_cnt + 1'b1);
wire              restore_can_pop = (restore_cnt != 0);
wire [`RAS_W-1:0] restore_pop_ptr = restore_can_pop
                                  ? (restore_ptr - 1'b1) : restore_ptr;
wire [`RAS_W:0]   restore_cnt_pop = restore_can_pop
                                  ? (restore_cnt - 1'b1) : restore_cnt;

// 栈体写口（各自唯一；提交写不受 flush 影响）
always @(posedge clk) begin
    if (!reset && cmt_push_i)
        cmt_stack[cmt_wr_idx] <= cmt_push_addr_i;
end
always @(posedge clk) begin
    if (!reset && !flush_i) begin
        if (restore_i && restore_push_i)
            spec_stack[restore_wr_idx] <= restore_push_addr_i;
        else if (!restore_i && spec_push_i)
            spec_stack[spec_wr_idx] <= spec_push_addr_i;
    end
end

// Capture the state before the corresponding P1 block changes the RAS.
always @(posedge clk) begin
    if (reset) begin
        checkpoint_nonempty <= {`FTQ_SIZE{1'b0}};
    end else if (checkpoint_save_i && !restore_i && !flush_i) begin
        checkpoint_ptr[checkpoint_id_i] <= spec_ptr;
        checkpoint_cnt[checkpoint_id_i] <= spec_cnt;
        checkpoint_top[checkpoint_id_i] <= top_addr_o;
        checkpoint_nonempty[checkpoint_id_i] <= (spec_cnt != 0);
    end
end

always @(posedge clk) begin
    if (reset) begin
        spec_ptr <= {`RAS_W{1'b0}};
        cmt_ptr  <= {`RAS_W{1'b0}};
        spec_cnt <= {(`RAS_W+1){1'b0}};
        cmt_cnt  <= {(`RAS_W+1){1'b0}};
        spec_ovl <= {`RAS_DEPTH{1'b0}};
    end else begin
        // ---- 提交栈指针/计数 ----
        cmt_ptr <= cmt_ptr_n;
        cmt_cnt <= cmt_cnt_n;

        // ---- 前端栈视图 ----
        if (flush_i) begin
            // 恢复为提交栈（本拍提交后的新值）；清覆盖位图即"视图对拷"
            spec_ovl <= {`RAS_DEPTH{1'b0}};
            spec_ptr <= cmt_ptr_n;
            spec_cnt <= cmt_cnt_n;
        end else if (restore_i) begin
            // 丢弃更年轻的推测动作；新识别的 BL 在块前状态上压入体系结构 PC+4。
            if (restore_push_i) begin
                spec_ovl[restore_wr_idx] <= 1'b1;
                spec_ptr <= restore_wr_idx;
                spec_cnt <= restore_cnt_push;
            end else if (restore_pop_i) begin
                spec_ptr <= restore_pop_ptr;
                spec_cnt <= restore_cnt_pop;
            end else begin
                spec_ptr <= restore_ptr;
                spec_cnt <= restore_cnt;
            end
        end else begin
            if (spec_push_i) begin
                spec_ovl[spec_wr_idx] <= 1'b1;
                spec_ptr <= spec_wr_idx;
                if (spec_cnt != `RAS_DEPTH) spec_cnt <= spec_cnt + 1'b1;
            end else if (spec_pop_i && (spec_cnt != 0)) begin
                spec_ptr <= spec_ptr - 1'b1;
                spec_cnt <= spec_cnt - 1'b1;
            end
        end
    end
end


endmodule
