// ============================================================
// ifu 模块（取指单元，含预译码）
// ------------------------------------------------------------
// 参考实现说明：
// - 2 段流水：PRE（收 FTQ 块 + MMU 翻译 + 发 ICache）/ IF（等返回 + 切割 + 入 IB）；
//   最多 2 块在途；I$ 严格单请求在途（ic_outstanding），过期应答自然丢弃，
//   数据在 PRE 滞留期间返回被弃时由 if_replay_req 重发兜底；
// - 取指异常整块占位（inst=0，不打 ICache）；
// - 预译码仅识别 B/BL 直接跳转，漏预测时截断块并全量 predec_redirect；
// - ic_cancel_o 恒 0（I$ 忽略 cancel，本模块自行丢弃）。
// ============================================================
`include "mycpu.h"

module ifu(
    input  wire                       clk,
    input  wire                       reset,
    input  wire                       flush_i,

    input  wire                       ftq_valid_i,
    input  wire [31:0]                ftq_pc_i,
    input  wire [`BLK_LEN_W-1:0]      ftq_length_i,
    input  wire                       ftq_taken_i,
    input  wire [31:0]                ftq_target_i,
    input  wire [`FTQ_W-1:0]          ftq_ftq_id_i,
    output wire                       ftq_accept_o,

    output wire                       mmu_i_req_o,
    output wire [31:0]                mmu_i_vaddr_o,
    input  wire [31:0]                mmu_i_paddr_i,
    input  wire [1:0]                 mmu_i_mat_i,
    input  wire                       mmu_i_excp_adef_i,
    input  wire [`TLB_EX_NUM-1:0]     mmu_i_tlb_ex_i,

    output wire                       ic_req_o,
    output wire [31:0]                ic_vaddr_o,
    output wire [31:0]                ic_paddr_o,
    output wire                       ic_uncached_o,
    input  wire                       ic_addr_ok_i,
    input  wire                       ic_data_ok_i,
    input  wire [`CACHE_LINE_BITS-1:0] ic_rline_i,
    output wire                       ic_cancel_o,

    output wire                       predec_redirect_o,
    output wire                       predec_fixup_only_o,
    output wire                       predec_update_pc_o,
    output wire [31:0]                predec_redirect_pc_o,
    output wire [`FTQ_W-1:0]          predec_redirect_id_o,
    output wire [`BLK_LEN_W-1:0]      predec_length_o,
    output wire                       predec_taken_o,
    output wire [31:0]                predec_target_o,
    output wire [`BR_TYPE_W-1:0]      predec_br_type_o,

    output wire                       ib_push0_valid_o,
    output wire [31:0]                ib_push0_pc_o,
    output wire [31:0]                ib_push0_inst_o,
    output wire                       ib_push0_pred_taken_o,
    output wire                       ib_push0_is_last_o,
    output wire [`FTQ_W-1:0]          ib_push0_ftq_id_o,
    output wire [`EXCP_NUM-1:0]       ib_push0_excp_o,

    output wire                       ib_push1_valid_o,
    output wire [31:0]                ib_push1_pc_o,
    output wire [31:0]                ib_push1_inst_o,
    output wire                       ib_push1_pred_taken_o,
    output wire                       ib_push1_is_last_o,
    output wire [`FTQ_W-1:0]          ib_push1_ftq_id_o,
    output wire [`EXCP_NUM-1:0]       ib_push1_excp_o,

    output wire                       ib_push2_valid_o,
    output wire [31:0]                ib_push2_pc_o,
    output wire [31:0]                ib_push2_inst_o,
    output wire                       ib_push2_pred_taken_o,
    output wire                       ib_push2_is_last_o,
    output wire [`FTQ_W-1:0]          ib_push2_ftq_id_o,
    output wire [`EXCP_NUM-1:0]       ib_push2_excp_o,

    output wire                       ib_push3_valid_o,
    output wire [31:0]                ib_push3_pc_o,
    output wire [31:0]                ib_push3_inst_o,
    output wire                       ib_push3_pred_taken_o,
    output wire                       ib_push3_is_last_o,
    output wire [`FTQ_W-1:0]          ib_push3_ftq_id_o,
    output wire [`EXCP_NUM-1:0]       ib_push3_excp_o,

    input  wire                       ib_can_push_i
);

assign ic_cancel_o = 1'b0;

function [`EXCP_NUM-1:0] pack_if_excp;
    input adef;
    input [`TLB_EX_NUM-1:0] tlb_ex;
    begin
        pack_if_excp = {(`EXCP_NUM){1'b0}};
        if (adef)
            pack_if_excp[`EXCP_ADEF] = 1'b1;
        if (tlb_ex[`TLB_EX_TLBR])
            pack_if_excp[`EXCP_TLBR_F] = 1'b1;
        if (tlb_ex[`TLB_EX_PIF])
            pack_if_excp[`EXCP_PIF] = 1'b1;
        if (tlb_ex[`TLB_EX_PPI])
            pack_if_excp[`EXCP_PPI_F] = 1'b1;
    end
endfunction

// ---------------- PRE 级 ----------------
reg        pre_v;
reg [31:0] pre_pc;
reg [`BLK_LEN_W-1:0] pre_len;
reg        pre_taken;
reg [31:0] pre_target;
reg [`FTQ_W-1:0] pre_id;
reg [31:0] pre_paddr;
reg        pre_uncached;
reg        pre_excp;
reg [`EXCP_NUM-1:0] pre_excp_vec;
reg        pre_ic_sent;

initial begin
    pre_v       = 1'b0;
    pre_ic_sent = 1'b0;
end

// ---------------- IF 级 ----------------
reg        if_v;
reg [31:0] if_pc;
reg [`BLK_LEN_W-1:0] if_len;
reg        if_taken;
reg [31:0] if_target;
reg [`FTQ_W-1:0] if_id;
reg        if_excp;
reg [`EXCP_NUM-1:0] if_excp_vec;
reg        if_wait_data;
reg [31:0] if_paddr;
reg        if_uncached;
reg [`CACHE_LINE_BITS-1:0] if_rline;

initial begin
    if_rline = {`CACHE_LINE_BITS{1'b0}};
end

reg ic_outstanding;

initial begin
    ic_outstanding = 1'b0;
end

// I$ 严格单请求在途：addr_ok 置位、data_ok 清零。
// flush/predec 丢弃 PRE/IF 块时【不】清该标志：在途应答返回前禁止发新请求，
// 旧应答返回时无人处于等待态（新请求尚未发出），自然被丢弃——
// 从机制上杜绝“旧应答喂错新块”，也无需 drop_rsp 标记。
always @(posedge clk) begin
    if (reset) begin
        ic_outstanding <= 1'b0;
    end else begin
        if (ic_req_o && (ic_addr_ok_i === 1'b1))
            ic_outstanding <= 1'b1;
        if (ic_data_ok_i === 1'b1)
            ic_outstanding <= 1'b0;
    end
end

wire pre_excp_now = mmu_i_excp_adef_i || |mmu_i_tlb_ex_i;
wire [`EXCP_NUM-1:0] pre_excp_vec_now = pack_if_excp(mmu_i_excp_adef_i, mmu_i_tlb_ex_i);

function [`BLK_LEN_W-1:0] safe_blk_len;
    input [`BLK_LEN_W-1:0] len;
    begin
        if (len === 3'd0 || len === 3'bx || len === 3'dz)
            safe_blk_len = 3'd1;
        else
            safe_blk_len = len;
    end
endfunction

wire [`BLK_LEN_W-1:0] if_len_eff = safe_blk_len(if_len);
wire ib_can_push = (ib_can_push_i === 1'b1);
// if_wait_data=1 时唯一在途请求必属本块（PRE 被 !ic_outstanding/!if_replay_req
// 挡住，发不出新请求），data_ok 可直接消费
wire if_ic_hit   = (if_wait_data === 1'b1) && (ic_data_ok_i === 1'b1);
wire if_line_ready = (if_excp === 1'b1)
                  || if_ic_hit
                  || ((if_wait_data !== 1'b1) && (if_v === 1'b1) && (|if_rline));
wire if_ready_go = (if_v === 1'b1) && ib_can_push && if_line_ready;

// PRE/IF 两级流水：最多 2 块在途。
// 仅当 PRE 占用且本拍无法前进（IF 占用未走 / PRE 请求未发出）时反压 FTQ；
// predec 全量重定向当拍不接新块（FTQ 正回滚 ifu_ptr/bpu_ptr），
// 且 PRE 中的错误路径块被丢弃（见 predec_kill）。
wire pre_done     = (pre_v === 1'b1) && (pre_excp || (pre_ic_sent === 1'b1));
wire if_allow_in  = (if_v !== 1'b1) || if_ready_go;
wire pre_ready_go = pre_done && if_allow_in;
wire predec_kill  = predec_redirect_o && (predec_fixup_only_o !== 1'b1);
wire pre_to_if    = pre_ready_go && !predec_kill;
assign ftq_accept_o = (ftq_valid_i === 1'b1)
                   && ((pre_v !== 1'b1) || pre_ready_go)
                   && !predec_kill && (flush_i !== 1'b1);

// 时序解耦(100MHz 攻坚 step1):MMU I 通道的 req/vaddr 原本用 ftq_accept_o 选择,
// 而 ftq_accept_o 组合依赖 pre_ready_go→if_ready_go→ic_data_ok_i(icache 命中),
// 使得 "icache tag 命中 → 接受下一块 → 组合翻译下一 PC → 主 TLB/L1 TLB 回填"
// 全挤在一拍(综合关键路径 36 级 LUT / -12ns @100MHz)。
// 改用 ftq_valid_i(FTQ 指针比较,纯寄存器浅逻辑,无 icache 依赖)做选择:
//   * 翻译结果只在 ftq_accept_o=1 时被 pre_paddr/pre_excp 锁存,而 accept=1 蕴含
//     ftq_valid_i=1 且 vaddr=ftq_pc_i —— 被锁存的值与原来逐位相同,功能等价;
//   * 唯一行为差异:accept=0&&pre_v&&ftq_valid 的停顿拍会提前把 ftq_pc 的翻译
//     回填进 L1 TLB(纯缓存预取,地址真实,无正确性影响)。
// 这样把 icache 命中链从 TLB 翻译/回填的建立路径上彻底摘除。
assign mmu_i_req_o   = (ftq_valid_i || (pre_v === 1'b1)) && (flush_i !== 1'b1);
assign mmu_i_vaddr_o = (ftq_valid_i === 1'b1) ? ftq_pc_i : pre_pc;

// IF 等数据但请求已丢（数据在 PRE 滞留期间返回被弃）时重放
wire if_replay_req = (if_v === 1'b1) && (if_wait_data === 1'b1) && (ic_outstanding !== 1'b1)
                   && (if_excp !== 1'b1) && (flush_i !== 1'b1);

// PRE 发请求的前提：I$ 无在途请求、IF 不在重放、本块非错误路径
wire pre_ic_req = (pre_v === 1'b1) && !pre_excp && (pre_ic_sent !== 1'b1)
               && (ic_outstanding !== 1'b1) && !if_replay_req
               && !predec_kill && (flush_i !== 1'b1);

assign ic_req_o      = pre_ic_req || if_replay_req;
assign ic_vaddr_o    = if_replay_req ? if_pc : pre_pc;
assign ic_paddr_o    = if_replay_req ? if_paddr : pre_paddr;
assign ic_uncached_o = if_replay_req ? if_uncached : pre_uncached;

always @(posedge clk) begin
    if (reset || flush_i) begin
        pre_v        <= 1'b0;
        pre_ic_sent  <= 1'b0;
        if_v         <= 1'b0;
        if_wait_data <= 1'b0;
        if_rline     <= {`CACHE_LINE_BITS{1'b0}};
        if_pc        <= 32'b0;
    end else begin
        // ---- PRE 级：接收新块 > 前进/被杀 > 记录请求已发 ----
        // ftq_accept_o 已保证：PRE 空或本拍 pre_ready_go（成对进出）
        if (ftq_accept_o) begin
            pre_v        <= 1'b1;
            pre_pc       <= ftq_pc_i;
            pre_len      <= safe_blk_len(ftq_length_i);
            pre_taken    <= ftq_taken_i;
            pre_target   <= ftq_target_i;
            pre_id       <= ftq_ftq_id_i;
            pre_paddr    <= mmu_i_paddr_i;
            pre_uncached <= (mmu_i_mat_i != 2'd1);
            pre_excp     <= pre_excp_now;
            pre_excp_vec <= pre_excp_vec_now;
            pre_ic_sent  <= 1'b0;
        end else if (pre_to_if || predec_kill) begin
            pre_v       <= 1'b0;
            pre_ic_sent <= 1'b0;
        end else if (pre_ic_req && (ic_addr_ok_i === 1'b1)) begin
            pre_ic_sent <= 1'b1;
        end

        // ---- IF 级：装载 > 弹出 > 等数据 ----
        if (pre_to_if) begin
            if_v         <= 1'b1;
            if_pc        <= pre_pc;
            if_len       <= pre_len;
            if_taken     <= pre_taken;
            if_target    <= pre_target;
            if_id        <= pre_id;
            if_excp      <= pre_excp;
            if_excp_vec  <= pre_excp_vec;
            if_paddr     <= pre_paddr;
            if_uncached  <= pre_uncached;
            if (pre_excp) begin
                if_wait_data <= 1'b0;
            end else if (ic_data_ok_i === 1'b1) begin
                // 过渡拍返回的 data_ok 必属本块（唯一在途请求）
                if_rline     <= ic_rline_i;
                if_wait_data <= 1'b0;
            end else begin
                if_wait_data <= 1'b1;
            end
        end else if (if_ready_go === 1'b1) begin
            if_v         <= 1'b0;
            if_wait_data <= 1'b0;
        end else if (if_v && if_wait_data && (ic_data_ok_i === 1'b1)) begin
            if_rline     <= ic_rline_i;
            if_wait_data <= 1'b0;
        end
    end
end

// ---------------- 指令切割 ----------------
wire [`CACHE_LINE_W-1:0] line_off = if_pc[`CACHE_LINE_W-1:2];
wire [`CACHE_LINE_BITS-1:0] if_rline_eff = (if_v && if_wait_data && ic_data_ok_i)
                                           ? ic_rline_i : if_rline;

wire [31:0] cut_inst [0:3];
wire [31:0] cut_pc   [0:3];
genvar gi;
generate
for (gi = 0; gi < 4; gi = gi + 1) begin : gen_cut
    assign cut_inst[gi] = if_excp ? 32'b0 :
                          if_rline_eff[(line_off + gi) * 32 +: 32];
    assign cut_pc[gi]   = if_pc + {29'b0, gi[1:0], 2'b00};
end
endgenerate

// ---------------- 预译码：B(010100) / BL(010101) ----------------
wire [5:0] opc0 = cut_inst[0][31:26];
wire [5:0] opc1 = cut_inst[1][31:26];
wire [5:0] opc2 = cut_inst[2][31:26];
wire [5:0] opc3 = cut_inst[3][31:26];

function is_direct_br;
    input [5:0] opc;
    begin
        is_direct_br = (opc == 6'b010100) || (opc == 6'b010101);
    end
endfunction

function is_cond_br;
    input [5:0] opc;
    begin
        is_cond_br = (opc >= 6'b010110) && (opc <= 6'b011011);
    end
endfunction

function [31:0] br_target;
    input [31:0] pc;
    input [31:0] inst;
    begin
        br_target = pc + {{4{inst[9]}}, inst[9:0], inst[25:10], 2'b00}; // si26 << 2, MSB=inst[9]
    end
endfunction

wire [3:0] imm_br;
wire [3:0] cond_br;
assign imm_br[0] = is_direct_br(opc0);
assign imm_br[1] = is_direct_br(opc1);
assign imm_br[2] = is_direct_br(opc2);
assign imm_br[3] = is_direct_br(opc3);
assign cond_br[0] = is_cond_br(opc0);
assign cond_br[1] = is_cond_br(opc1);
assign cond_br[2] = is_cond_br(opc2);
assign cond_br[3] = is_cond_br(opc3);

wire [31:0] imm_tgt [0:3];
assign imm_tgt[0] = br_target(cut_pc[0], cut_inst[0]);
assign imm_tgt[1] = br_target(cut_pc[1], cut_inst[1]);
assign imm_tgt[2] = br_target(cut_pc[2], cut_inst[2]);
assign imm_tgt[3] = br_target(cut_pc[3], cut_inst[3]);

// 截断块内最早的关键分支，两类都做【全量】重定向（FTQ 回滚 bpu/ifu 指针 +
// BPU 改 PC，predec_fixup_only 恒 0）：
// - 漏预测的直接跳转（B/BL）：重定向到立即数目标；
// - 块中部的条件分支：重定向到分支后一条指令（截断块的 fall-through），
//   不跳过任何指令；预测不跳且实际不跳时无缝衔接，其余情形由提交级
//   误预测冲刷纠正。保证"每块至多一条分支且在块末"的 FTQ 训练不变式。
// （旧 fixup_only 模式只改元数据不改 PC，会跳过截断点到原块尾之间的指令，
//   属功能错误，已弃用。）
reg [1:0] predec_idx;
reg       predec_found;
reg       predec_is_direct;
integer pi;
always @(*) begin
    predec_found     = 1'b0;
    predec_idx       = 2'd0;
    predec_is_direct = 1'b0;
    for (pi = 0; pi < 4; pi = pi + 1) begin
        if (!predec_found && (pi < if_len_eff)) begin
            if (imm_br[pi] && ((pi < if_len_eff - 1) || !if_taken)) begin
                predec_found     = 1'b1;
                predec_idx       = pi[1:0];
                predec_is_direct = 1'b1;
            end else if (cond_br[pi] && (pi < if_len_eff - 1)) begin
                predec_found     = 1'b1;
                predec_idx       = pi[1:0];
                predec_is_direct = 1'b0;
            end
        end
    end
end

wire [`BLK_LEN_W-1:0] out_len = predec_found ? (predec_idx + 1'b1) : if_len_eff;
wire push_en = if_ready_go;

// pred_taken 只标在推送块末指令：直接 B/BL 恒 1；cond 截断恒 0（前端实际
// 走 fall-through，若实际 taken 由提交级 br_taken!=pred_taken 冲刷，
// 不依赖目标比对，避免目标撞车漏检）；未截断块末用 FTQ 块级 taken
function slot_pred_taken;
    input [1:0] idx;
    input [`BLK_LEN_W-1:0] olen;
    begin
        slot_pred_taken = 1'b0;
        if ((idx + 1'b1) == olen[`BLK_LEN_W-1:0]) begin
            if (imm_br[idx])
                slot_pred_taken = 1'b1;
            else if (predec_found && !predec_is_direct)
                slot_pred_taken = 1'b0;
            else if (if_taken && (cond_br[idx] ||
                     (!predec_found && ((idx + 1'b1) == if_len_eff))))
                slot_pred_taken = 1'b1;
        end
    end
endfunction

assign predec_redirect_o    = push_en && predec_found && !if_excp;
assign predec_fixup_only_o  = 1'b0;   // 两类截断均全量重定向（回滚指针 + 改 PC）
assign predec_update_pc_o   = 1'b1;
// 直接跳转：去立即数目标；cond 截断：去截断块 fall-through（不跳过指令）
assign predec_redirect_pc_o = predec_is_direct ? imm_tgt[predec_idx]
                                               : (if_pc + {27'b0, out_len, 2'b00});
assign predec_redirect_id_o = if_id;
assign predec_length_o      = out_len;
// cond 截断块按"不跳"出口（前端实际走 fall-through），taken=0；
// target 与 redirect_pc 一致（pred_taken=0 时提交级不比对 target，仅记录）
assign predec_taken_o       = predec_is_direct;
assign predec_target_o      = predec_redirect_pc_o;
assign predec_br_type_o     = predec_is_direct ?
                              ((cut_inst[predec_idx][31:26] == 6'b010101) ? `BR_TYPE_CALL : `BR_TYPE_UNCOND) :
                              `BR_TYPE_COND;

// ---------------- IB 输出 ----------------
assign ib_push0_valid_o     = push_en && (out_len >= 3'd1);
assign ib_push0_pc_o        = cut_pc[0];
assign ib_push0_inst_o      = cut_inst[0];
assign ib_push0_pred_taken_o= push_en && (out_len >= 3'd1) && slot_pred_taken(2'd0, out_len);
assign ib_push0_is_last_o   = (out_len == 3'd1);
assign ib_push0_ftq_id_o    = if_id;
assign ib_push0_excp_o      = if_excp_vec;

assign ib_push1_valid_o     = push_en && (out_len >= 3'd2);
assign ib_push1_pc_o        = cut_pc[1];
assign ib_push1_inst_o      = cut_inst[1];
assign ib_push1_pred_taken_o= push_en && (out_len >= 3'd2) && slot_pred_taken(2'd1, out_len);
assign ib_push1_is_last_o   = (out_len == 3'd2);
assign ib_push1_ftq_id_o    = if_id;
assign ib_push1_excp_o      = if_excp_vec;

assign ib_push2_valid_o     = push_en && (out_len >= 3'd3);
assign ib_push2_pc_o        = cut_pc[2];
assign ib_push2_inst_o      = cut_inst[2];
assign ib_push2_pred_taken_o= push_en && (out_len >= 3'd3) && slot_pred_taken(2'd2, out_len);
assign ib_push2_is_last_o   = (out_len == 3'd3);
assign ib_push2_ftq_id_o    = if_id;
assign ib_push2_excp_o      = if_excp_vec;

assign ib_push3_valid_o     = push_en && (out_len >= 3'd4);
assign ib_push3_pc_o        = cut_pc[3];
assign ib_push3_inst_o      = cut_inst[3];
assign ib_push3_pred_taken_o= push_en && (out_len >= 3'd4) && slot_pred_taken(2'd3, out_len);
assign ib_push3_is_last_o   = (out_len >= 3'd4);
assign ib_push3_ftq_id_o    = if_id;
assign ib_push3_excp_o      = if_excp_vec;

endmodule
