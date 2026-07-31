// ============================================================
// ifu 模块（取指单元，含预译码）
// ------------------------------------------------------------
// 当前实现：
// - 2 段流水：PRE（收 FTQ 块 + MMU 翻译 + 发 ICache）/ IF（等返回 + 切割 + 入 IB）；
//   最多 2 块在途；I$ 保持一个逻辑请求在途，但允许 data_ok 返回旧请求与
//   addr_ok 接受新请求同拍发生；FTQ 新块在进入 PRE 的同拍可直发 I$，
//   数据在 PRE 滞留期间返回被弃时由 if_replay_req 重发兜底；
// - 取指异常整块占位（inst=0，不打 ICache）；
// - 预译码识别两类截断并全量 predec_redirect：漏预测的 B/BL 直接跳转
//   （重定向到立即数目标）、块中部的条件分支（截断并重定向到 fall-through）；
// - pred_taken：保留 slot_pred_taken（仅块末真实 cond/B/BL），禁止块末 ALU
//   被标 taken（Linux makecontext 静默跳过指令回归防护）。
// - 预译码只读取已锁存的 if_rline；data_ok 拍写入行数据，下一拍才允许
//   IF 级前进，避免把 I$ 命中数据到分支目标的组合路径接入 FTQ。
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
    input  wire [31:0]                ras_checkpoint_top_i,
    input  wire                       ras_checkpoint_nonempty_i,

    output wire                       mmu_i_req_o,
    output wire [31:0]                mmu_i_vaddr_o,
    input  wire [31:0]                mmu_i_paddr_i,
    input  wire [1:0]                 mmu_i_mat_i,
    input  wire                       mmu_i_excp_adef_i,
    input  wire [`TLB_EX_NUM-1:0]     mmu_i_tlb_ex_i,
    input  wire                       mmu_i_direct_ok_i, // 1: 翻译不依赖主 TLB（DA/DMW/L1 CAM）
    input  wire [31:0]                mmu_i_direct_paddr_i,
    input  wire [1:0]                 mmu_i_direct_mat_i,
    // 仅包含可直发路径上的异常；PRE 仍锁存完整 adef/tlb_ex 结果。
    input  wire                       mmu_i_direct_excp_i,

    output wire                       ic_req_o,
    output wire [11:5]                ic_vindex_o,
    output wire [31:0]                ic_paddr_o,
    output wire                       ic_uncached_o,
    input  wire                       ic_addr_ok_i,
    input  wire                       ic_data_ok_i,
    input  wire [`CACHE_LINE_BITS-1:0] ic_rline_i,
    input  wire                       ic_invalidate_i,

    output wire                       predec_redirect_o,
    output wire                       predec_fixup_only_o,
    output wire                       predec_update_pc_o,
    output wire [31:0]                predec_redirect_pc_o,
    output wire [`FTQ_W-1:0]          predec_redirect_id_o,
    output wire [`BLK_LEN_W-1:0]      predec_length_o,
    output wire                       predec_taken_o,
    output wire [31:0]                predec_target_o,
    // 预译码训练信息：块 PC、分支目标和分支类型
    output wire [31:0]                predec_block_pc_o,
    output wire [31:0]                predec_branch_target_o,
    output wire [`BR_TYPE_W-1:0]      predec_br_type_o,
    output wire                       predec_ras_call_o,
    output wire                       predec_ras_ret_o,
    output wire [31:0]                predec_ras_retaddr_o,

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

function [`EXCP_NUM-1:0] pack_if_excp;
    input adef;
    input [5:3] tlb_ex;
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
reg        pre_line_valid;
reg [`CACHE_LINE_BITS-1:0] pre_rline;

initial begin
    pre_v       = 1'b0;
    pre_ic_sent = 1'b0;
    pre_line_valid = 1'b0;
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
reg [31:5] ic_rsp_line;
reg [31:5] ic_rsp_pline;
reg [2:0]  ic_rsp_word;
reg        ic_rsp_uncached;
reg        ic_rsp_killed;

// One-entry physical response reuse buffer.  Reuse is checked only after the
// FTQ block has entered PRE, so the tag comparison is kept out of the P0/FTQ
// request path.  Both cached and uncached responses are retained: the latter
// is especially important during the reset-time data-copy loop, where one
// 32-byte fetch response would otherwise be refetched several times.
reg        linebuf_valid;
reg [31:5] linebuf_pline;
reg [7:0]  linebuf_word_valid;
reg [`CACHE_LINE_BITS-1:0] linebuf_data;

initial begin
    ic_outstanding = 1'b0;
    ic_rsp_line    = 27'b0;
    ic_rsp_pline   = 27'b0;
    ic_rsp_word    = 3'b0;
    ic_rsp_uncached = 1'b0;
    ic_rsp_killed  = 1'b0;
    linebuf_valid  = 1'b0;
end

// I$ 单逻辑请求在途：addr_ok 置位、data_ok 清零；若旧响应和新接受同拍，
// 新请求替换旧请求，标志必须保持为 1。
// flush/predec 丢弃 PRE/IF 块时【不】清该标志：在途应答返回前禁止发新请求，
// 旧应答返回时无人处于等待态（新请求尚未发出），自然被丢弃——
// 从机制上杜绝“旧应答喂错新块”，也无需 drop_rsp 标记。
always @(posedge clk) begin
    if (reset) begin
        ic_outstanding <= 1'b0;
        ic_rsp_line    <= 27'b0;
        ic_rsp_pline   <= 27'b0;
        ic_rsp_word    <= 3'b0;
        ic_rsp_uncached <= 1'b0;
        ic_rsp_killed  <= 1'b0;
    end else begin
        case ({(ic_req_o && (ic_addr_ok_i === 1'b1)), (ic_data_ok_i === 1'b1)})
            2'b10: ic_outstanding <= 1'b1;
            2'b01: ic_outstanding <= 1'b0;
            2'b11: ic_outstanding <= 1'b1; // 同拍完成旧请求并接受新请求
            default: ic_outstanding <= ic_outstanding;
        endcase
        // 同拍 data_ok + addr_ok 时，data_ok 属于旧 ic_rsp_line；时钟沿后
        // tracker 被新请求替换，供下一拍响应配对。
        // A flush/cache-maintenance operation invalidates the response that
        // was already in flight.  It may return later, but must not refill
        // the reuse buffer after the invalidation point.
        if ((flush_i === 1'b1) || (ic_invalidate_i === 1'b1))
            ic_rsp_killed <= ic_outstanding
                          || (ic_req_o && (ic_addr_ok_i === 1'b1));
        else if (ic_req_o && (ic_addr_ok_i === 1'b1))
            ic_rsp_killed <= 1'b0;

        if (ic_req_o && (ic_addr_ok_i === 1'b1)) begin
            ic_rsp_line <= ic_vline;
            ic_rsp_pline <= ic_paddr_o[31:5];
            ic_rsp_word <= ic_paddr_o[4:2];
            ic_rsp_uncached <= ic_uncached_o;
        end
    end
end

always @(posedge clk) begin
    if (reset || (flush_i === 1'b1)
              || (ic_invalidate_i === 1'b1)) begin
        linebuf_valid <= 1'b0;
    end else if ((ic_data_ok_i === 1'b1)
              && (ic_rsp_killed !== 1'b1)) begin
        linebuf_valid <= 1'b1;
        linebuf_pline <= ic_rsp_pline;
        // An uncached I$ response contains only the requested word through
        // word 7; the lower words are zero-filled by icache.v.  Track that
        // coverage explicitly so a later backward branch can never consume
        // those zeros as instructions.
        linebuf_word_valid <= ic_rsp_uncached
                            ? (8'hff << ic_rsp_word) : 8'hff;
        linebuf_data  <= ic_rline_i;
    end
end

wire pre_excp_now = mmu_i_excp_adef_i || |mmu_i_tlb_ex_i;
wire [`EXCP_NUM-1:0] pre_excp_vec_now = pack_if_excp(mmu_i_excp_adef_i, mmu_i_tlb_ex_i[5:3]);

function [`BLK_LEN_W-1:0] safe_blk_len;
    input [`BLK_LEN_W-1:0] len;
    begin
        if (len === 3'd0 || len === 3'bx || len === 3'dz)
            safe_blk_len = 3'd1;
        else
            safe_blk_len = len;
    end
endfunction

wire [`BLK_LEN_W-1:0] ftq_len_eff = safe_blk_len(ftq_length_i);
wire [`BLK_LEN_W-1:0] if_len_eff  = safe_blk_len(if_len);
wire ib_can_push = (ib_can_push_i === 1'b1);
wire if_rsp_match  = (ic_rsp_line == if_pc[31:5]);
wire pre_rsp_match = (ic_rsp_line == pre_pc[31:5]);
wire pre_rsp_reuse = (pre_v === 1'b1) && !pre_excp
                  && (pre_ic_sent !== 1'b1)
                  && (ic_data_ok_i === 1'b1)
                  && (ic_rsp_killed !== 1'b1)
                  && (pre_paddr[31:5] == ic_rsp_pline)
                  && (!ic_rsp_uncached
                      || (pre_paddr[4:2] >= ic_rsp_word))
                  && (ic_invalidate_i !== 1'b1);
wire pre_linebuf_hit = (pre_v === 1'b1) && !pre_excp
                    && (pre_ic_sent !== 1'b1)
                    && linebuf_valid
                    && (pre_paddr[31:5] == linebuf_pline)
                    && linebuf_word_valid[pre_paddr[4:2]]
                    && (ic_invalidate_i !== 1'b1);
// data_ok 拍只锁存 if_rline，下一拍 !wait && |if_rline 才允许 IF 前进。
wire if_line_ready = (if_excp === 1'b1)
                  || ((if_wait_data !== 1'b1) && (if_v === 1'b1) && (|if_rline));
// Invalidate must combinationally stop IB push: register clear is next edge.
wire if_ready_go = (if_v === 1'b1) && ib_can_push && if_line_ready
                && (ic_invalidate_i !== 1'b1);
wire pre_done     = (pre_v === 1'b1)
                 && (pre_excp || (pre_ic_sent === 1'b1)
                     || pre_rsp_reuse || pre_linebuf_hit);
wire if_allow_in  = (if_v !== 1'b1) || if_ready_go;
wire pre_ready_go = pre_done && if_allow_in
                 && (ic_invalidate_i !== 1'b1);
wire predec_kill  = predec_redirect_o && (predec_fixup_only_o !== 1'b1);
wire pre_to_if    = pre_ready_go && !predec_kill;
assign ftq_accept_o = (ftq_valid_i === 1'b1)
                   && ((pre_v !== 1'b1) || pre_ready_go)
                   && !predec_kill && (flush_i !== 1'b1)
                   && (ic_invalidate_i !== 1'b1);

// MMU 请求由 FTQ valid 驱动，避免 ftq_accept_o 上的 I$ 返回链进入地址翻译。
// 翻译结果只会在 ftq_accept_o 成立时锁存；停顿期间提前查询真实 FTQ PC
// 只可能回填 L1 TLB，不改变体系结构状态。
assign mmu_i_req_o   = (ftq_valid_i || (pre_v === 1'b1)) && (flush_i !== 1'b1);
assign mmu_i_vaddr_o = (ftq_valid_i === 1'b1) ? ftq_pc_i : pre_pc;

// IF 等数据但请求已丢（数据在 PRE 滞留期间返回被弃）时重放
wire if_replay_req = (if_v === 1'b1) && (if_wait_data === 1'b1) && (ic_outstanding !== 1'b1)
                   && (if_excp !== 1'b1) && (flush_i !== 1'b1);

// data_ok 本拍返回时，旧请求将在时钟沿完成，可同拍用 addr_ok 替换为新请求。
wire ic_slot_free = (ic_outstanding !== 1'b1) || (ic_data_ok_i === 1'b1);

// PRE 发请求的前提：I$ 有空槽、IF 不在重放、本块非错误路径
wire pre_ic_req = (pre_v === 1'b1) && !pre_excp && (pre_ic_sent !== 1'b1)
               && ic_slot_free && !if_replay_req
               && !pre_rsp_reuse && !pre_linebuf_hit
               && !predec_kill && (flush_i !== 1'b1);

// PRE 正在前进（或为空）时，ftq_accept_o 接收的新块可直接发 I$，避免
// “先写 PRE、下一拍再发请求”的固定气泡。若 cache 当前不能接受，块仍正常
// 锁存进 PRE，pre_ic_sent=0，之后由 pre_ic_req 保持重试。
// 直发仅覆盖 DA/DMW/L1 CAM 命中且无直发异常的请求；主 TLB 命中结果走
// PRE→pre_ic_req，以隔离 FTQ 到主表查找的长组合路径。
wire ftq_direct_req = (`IFU_FTQ_DIRECT != 0)
                   && (ftq_accept_o === 1'b1) && (mmu_i_direct_excp_i !== 1'b1)
                   && (mmu_i_direct_ok_i === 1'b1)
                   // Uncached responses are partial lines and much slower
                   // than one PRE cycle.  Let PRE consult the physical
                   // response buffer before issuing them; keep same-cycle
                   // FTQ direct issue only for ordinary cacheable fetches.
                   && (mmu_i_direct_mat_i == 2'd1)
                   && ic_slot_free && !if_replay_req
                   && (flush_i !== 1'b1);
wire ftq_direct_fire = ftq_direct_req && (ic_addr_ok_i === 1'b1);

assign ic_req_o      = if_replay_req || pre_ic_req || ftq_direct_req;
wire [31:5] ic_vline = if_replay_req ? if_pc[31:5]
                      : pre_ic_req   ? pre_pc[31:5] : ftq_pc_i[31:5];
assign ic_vindex_o   = ic_vline[11:5];
assign ic_paddr_o    = if_replay_req ? if_paddr
                     : pre_ic_req    ? pre_paddr : mmu_i_direct_paddr_i;
assign ic_uncached_o = if_replay_req ? if_uncached
                     : pre_ic_req    ? pre_uncached : (mmu_i_direct_mat_i != 2'd1);

always @(posedge clk) begin
    // I$ cacop 与流水线 flush 可能差一拍：invalidate 当拍必须丢掉 PRE/IF
    // 已锁存的行数据，否则自修改代码仍可能从 if_rline/linebuf 吃到旧指令。
    if (reset || flush_i || (ic_invalidate_i === 1'b1)) begin
        pre_v        <= 1'b0;
        pre_ic_sent  <= 1'b0;
        pre_line_valid <= 1'b0;
        pre_rline      <= {`CACHE_LINE_BITS{1'b0}};
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
            pre_len      <= ftq_len_eff;
            pre_taken    <= ftq_taken_i;
            pre_target   <= ftq_target_i;
            pre_id       <= ftq_ftq_id_i;
            pre_paddr    <= mmu_i_paddr_i;
            pre_uncached <= (mmu_i_mat_i != 2'd1);
            pre_excp     <= pre_excp_now;
            pre_excp_vec <= pre_excp_vec_now;
            pre_ic_sent  <= ftq_direct_fire;
            pre_line_valid <= 1'b0;
        end else if (pre_to_if || predec_kill) begin
            pre_v       <= 1'b0;
            pre_ic_sent <= 1'b0;
            pre_line_valid <= 1'b0;
        end else begin
            if (pre_rsp_reuse) begin
                pre_ic_sent    <= 1'b1;
                pre_line_valid <= 1'b1;
                pre_rline      <= ic_rline_i;
            end else if (pre_linebuf_hit) begin
                pre_ic_sent    <= 1'b1;
                pre_line_valid <= 1'b1;
                pre_rline      <= linebuf_data;
            end else if (pre_ic_req && (ic_addr_ok_i === 1'b1)) begin
                pre_ic_sent <= 1'b1;
            end
            // IF/IB 反压可能在 PRE 被占用时遇到 I$ 返回；保留该行，避免丢弃后重放。
            if (pre_v && pre_ic_sent && (ic_data_ok_i === 1'b1) && pre_rsp_match
                && (ic_rsp_killed !== 1'b1)) begin
                pre_rline      <= ic_rline_i;
                pre_line_valid <= 1'b1;
            end
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
            end else if (pre_line_valid) begin
                if_rline     <= pre_rline;
                if_wait_data <= 1'b0;
            end else if (pre_rsp_reuse) begin
                if_rline     <= ic_rline_i;
                if_wait_data <= 1'b0;
            end else if (pre_linebuf_hit) begin
                if_rline     <= linebuf_data;
                if_wait_data <= 1'b0;
            end else if ((ic_data_ok_i === 1'b1) && pre_rsp_match
                         && (ic_rsp_killed !== 1'b1)) begin
                // 只消费与 PRE 同一 cache line 的响应；旧路径应答即使与
                // 新请求同拍返回，也不会被错误配给新块。
                if_rline     <= ic_rline_i;
                if_wait_data <= 1'b0;
            end else begin
                if_wait_data <= 1'b1;
            end
        end else if (if_ready_go === 1'b1) begin
            if_v         <= 1'b0;
            if_wait_data <= 1'b0;
        end else if (if_v && if_wait_data && (ic_data_ok_i === 1'b1)
                     && if_rsp_match && (ic_rsp_killed !== 1'b1)) begin
            if_rline     <= ic_rline_i;
            if_wait_data <= 1'b0;
        end
    end
end

// ---------------- 指令切割 ----------------
// 命中与 miss 统一使用寄存后的 if_rline，不从 ic_rline_i 组合旁路。
wire [2:0] line_off = if_pc[`CACHE_LINE_W-1:2];

wire [31:0] cut_inst [0:3];
wire [31:0] cut_pc   [0:3];
genvar gi;
generate
for (gi = 0; gi < 4; gi = gi + 1) begin : gen_cut
    assign cut_inst[gi] = if_excp ? 32'b0 :
                          if_rline[({29'b0, line_off} + gi) * 32 +: 32];
    assign cut_pc[gi]   = if_pc + {28'b0, gi[1:0], 2'b00};
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
    input [25:0] inst;
    begin
        br_target = pc + {{4{inst[9]}}, inst[9:0], inst[25:10], 2'b00}; // si26 << 2, MSB=inst[9]
    end
endfunction

// 条件分支立即数目标，供前端预测器训练使用。
function [31:0] cond_target;
    input [31:0] pc;
    input [25:10] inst;
    begin
        cond_target = pc + {{14{inst[25]}}, inst[25:10], 2'b00};
    end
endfunction

wire [3:0] imm_br;
wire [3:0] cond_br;
wire [3:0] jirl_br;
wire [3:0] ret_br;
assign imm_br[0] = is_direct_br(opc0);
assign imm_br[1] = is_direct_br(opc1);
assign imm_br[2] = is_direct_br(opc2);
assign imm_br[3] = is_direct_br(opc3);
assign cond_br[0] = is_cond_br(opc0);
assign cond_br[1] = is_cond_br(opc1);
assign cond_br[2] = is_cond_br(opc2);
assign cond_br[3] = is_cond_br(opc3);
assign jirl_br[0] = (opc0 == 6'h13);
assign jirl_br[1] = (opc1 == 6'h13);
assign jirl_br[2] = (opc2 == 6'h13);
assign jirl_br[3] = (opc3 == 6'h13);
// 与后端译码保持一致：JIRL rd=$r0、rj=$r1 识别为返回。
assign ret_br[0] = jirl_br[0] && (cut_inst[0][4:0] == 5'd0) &&
                   (cut_inst[0][9:5] == 5'd1);
assign ret_br[1] = jirl_br[1] && (cut_inst[1][4:0] == 5'd0) &&
                   (cut_inst[1][9:5] == 5'd1);
assign ret_br[2] = jirl_br[2] && (cut_inst[2][4:0] == 5'd0) &&
                   (cut_inst[2][9:5] == 5'd1);
assign ret_br[3] = jirl_br[3] && (cut_inst[3][4:0] == 5'd0) &&
                   (cut_inst[3][9:5] == 5'd1);

wire [31:0] imm_tgt [0:3];
assign imm_tgt[0] = br_target(cut_pc[0], cut_inst[0][25:0]);
assign imm_tgt[1] = br_target(cut_pc[1], cut_inst[1][25:0]);
assign imm_tgt[2] = br_target(cut_pc[2], cut_inst[2][25:0]);
assign imm_tgt[3] = br_target(cut_pc[3], cut_inst[3][25:0]);

wire [31:0] cond_tgt [0:3];
assign cond_tgt[0] = cond_target(cut_pc[0], cut_inst[0][25:10]);
assign cond_tgt[1] = cond_target(cut_pc[1], cut_inst[1][25:10]);
assign cond_tgt[2] = cond_target(cut_pc[2], cut_inst[2][25:10]);
assign cond_tgt[3] = cond_target(cut_pc[3], cut_inst[3][25:10]);

// 截断块内最早的关键分支，两类都做【全量】重定向（FTQ 回滚 bpu/ifu 指针 +
// BPU 改 PC，predec_fixup_only 恒 0）：
// - 漏预测的直接跳转（B/BL）：重定向到立即数目标；
// - 块中部的条件分支：重定向到分支后一条指令（截断块的 fall-through），
//   不跳过任何指令；预测不跳且实际不跳时无缝衔接，其余情形由提交级
//   误预测冲刷纠正。保证"每块至多一条分支且在块末"的 FTQ 训练不变式。
// predec_fixup_only 恒为 0；若只改元数据而不改 PC，会跳过截断点到块尾的指令。
reg [1:0] predec_idx;
reg       predec_found;
reg       predec_is_direct;
reg       predec_is_ret;
reg       block_has_br;
integer pi;
wire [31:0] if_len_u = {29'b0, if_len_eff};
always @(*) begin
    predec_found     = 1'b0;
    predec_idx       = 2'd0;
    predec_is_direct = 1'b0;
    predec_is_ret    = 1'b0;
    block_has_br     = 1'b0;
    for (pi = 0; pi < 4; pi = pi + 1) begin
        if (pi < if_len_u) begin
            // A JIRL is also a real control-flow boundary.  In particular, a
            // correctly predicted RET must not be mistaken for a dirty taken
            // FTB entry merely because no predecode redirect is required.
            if (imm_br[pi] || cond_br[pi] || jirl_br[pi])
                block_has_br = 1'b1;
            if (!predec_found) begin
                // Find the first architectural control-flow instruction
                // independently of whether its predicted target is correct.
                // out_len and all IB push controls are derived from this
                // structural boundary and therefore do not depend on the
                // asynchronous RAS checkpoint read.
                if (imm_br[pi]) begin
                    predec_found     = 1'b1;
                    predec_idx       = pi[1:0];
                    predec_is_direct = 1'b1;
                end else if (ret_br[pi]) begin
                    predec_found     = 1'b1;
                    predec_idx       = pi[1:0];
                    predec_is_ret    = 1'b1;
                end else if (cond_br[pi] || jirl_br[pi]) begin
                    predec_found     = 1'b1;
                    predec_idx       = pi[1:0];
                end
            end
        end
    end
end

wire predec_before_end = predec_found &&
                         ({30'b0, predec_idx} < (if_len_u - 32'd1));
// Target validation is intentionally kept on the redirect-only side of the
// predecoder.  It may update BPU/FTQ state, but it cannot change the number of
// instructions presented to the four-bank FWFT instruction buffer.
wire predec_needs_redirect =
    predec_found &&
    (predec_is_direct
        ? (predec_before_end || !if_taken)
        : predec_is_ret
            ? (ras_checkpoint_nonempty_i &&
               (predec_before_end || !if_taken ||
                (if_target != ras_checkpoint_top_i)))
            : (cond_br[predec_idx] && predec_before_end));

wire [`BLK_LEN_W-1:0] out_len = predec_found ? (predec_idx + 1'b1) : if_len_eff;
wire push_en = if_ready_go;
// FTB/uBTB 脏命中：块标 taken 但无一分支。BPU 已跳到 target，IFU 又禁给 ALU
// 标 pred_taken → 提交级看不到 (!branch && pred_taken)，错误路径（如 random
// TLBR 入口 0x200 后跳进 0x3cc 空洞）会以 INE 静默覆盖 ESTAT。
wire false_taken = if_taken && !predec_found && !block_has_br;

// pred_taken 只标在推送块末指令：直接 B/BL 恒 1；cond 截断恒 0（前端实际
// 走 fall-through，若实际 taken 由提交级 br_taken!=pred_taken 冲刷，
// 不依赖目标比对，避免目标撞车漏检）；未截断块末仅真实 cond 分支用 FTQ taken。
// 禁止对块末 ALU 标 pred_taken：否则 taken 脏 FTB 会令 BPU 跳到 target，而 IFU
// 仍推送块内 ALU；双提交又不检槽1 的 (!branch&&pred_taken) → 静默跳过后续指令
// （Linux makecontext: 2e68/2e6c 后跳到 2ea8）。假 taken 改由 false_taken 重定向纠错。
function slot_pred_taken;
    input [1:0] idx;
    input [`BLK_LEN_W-1:0] olen;
    begin
        slot_pred_taken = 1'b0;
        if ((idx + 1'b1) == olen[`BLK_LEN_W-1:0]) begin
            if (imm_br[idx])
                slot_pred_taken = 1'b1;
            // RET is architecturally taken even when the speculative RAS is
            // empty.  The backend may still repair its target, but the IB
            // metadata must not depend on the RAS checkpoint lookup.
            else if (predec_found && predec_is_ret &&
                     (idx == predec_idx))
                slot_pred_taken = 1'b1;
            else if (predec_found && !predec_is_direct &&
                     !predec_is_ret && cond_br[idx] &&
                     (idx == predec_idx) && predec_before_end)
                slot_pred_taken = 1'b0;
            else if (if_taken && (cond_br[idx] || jirl_br[idx]))
                slot_pred_taken = 1'b1;
        end
    end
endfunction

assign predec_redirect_o    = push_en && !if_excp &&
                              (predec_needs_redirect || false_taken);
assign predec_fixup_only_o  = 1'b0;   // 两类截断均全量重定向（回滚指针 + 改 PC）
assign predec_update_pc_o   = 1'b1;
// 直接跳转：去立即数目标；cond/ret 截断 / 假 taken：去截断块 fall-through
assign predec_redirect_pc_o = predec_is_direct ? imm_tgt[predec_idx] :
                              predec_is_ret    ? ras_checkpoint_top_i :
                              (if_pc + {27'b0, out_len, 2'b00});
assign predec_redirect_id_o = if_id;
assign predec_length_o      = out_len;
// cond/ret 截断 / 假 taken 按"不跳"出口，taken=0；
// target 与 redirect_pc 一致（pred_taken=0 时提交级不比对 target，仅记录）
assign predec_taken_o       = predec_needs_redirect && predec_is_direct;
assign predec_target_o      = predec_redirect_pc_o;
assign predec_block_pc_o    = if_pc;
assign predec_branch_target_o =
    predec_is_direct ? imm_tgt[predec_idx] :
    predec_is_ret    ? ras_checkpoint_top_i :
                       cond_tgt[predec_idx];
assign predec_br_type_o     = predec_is_ret ? `BR_TYPE_RET :
                              predec_is_direct ?
                              ((cut_inst[predec_idx][31:26] == 6'b010101) ? `BR_TYPE_CALL : `BR_TYPE_UNCOND) :
                              `BR_TYPE_COND;
assign predec_ras_call_o    = predec_redirect_o && (predec_br_type_o == `BR_TYPE_CALL);
assign predec_ras_ret_o     = predec_redirect_o && (predec_br_type_o == `BR_TYPE_RET);
assign predec_ras_retaddr_o = cut_pc[predec_idx] + 32'd4;

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

`ifdef SYNTHESIS
// synthesis translate_off
reg [63:0] diag_cycle;
reg [63:0] diag_ftq_valid;
reg [63:0] diag_ftq_accept;
reg [63:0] diag_ftq_pre_stall;
reg [63:0] diag_pre_wait_req;
reg [63:0] diag_pre_wait_if;
reg [63:0] diag_if_wait_data;
reg [63:0] diag_if_wait_ib;
reg [63:0] diag_ic_outstanding;
reg [63:0] diag_ic_req;
reg [63:0] diag_ic_addr_ok;
reg [63:0] diag_ic_data_ok;
reg [63:0] diag_ic_uncached;
reg [63:0] diag_ic_uncached_direct;
reg [63:0] diag_ic_uncached_pre;
reg [63:0] diag_ic_uncached_replay;
reg [63:0] diag_pre_rsp_mismatch;
reg [63:0] diag_if_rsp_mismatch;
reg [31:0] diag_first_uncached_pc;
reg [31:0] diag_last_uncached_pc;

always @(posedge clk) begin
    if (reset) begin
        diag_cycle            <= 64'd0;
        diag_ftq_valid        <= 64'd0;
        diag_ftq_accept       <= 64'd0;
        diag_ftq_pre_stall    <= 64'd0;
        diag_pre_wait_req     <= 64'd0;
        diag_pre_wait_if      <= 64'd0;
        diag_if_wait_data     <= 64'd0;
        diag_if_wait_ib       <= 64'd0;
        diag_ic_outstanding   <= 64'd0;
        diag_ic_req           <= 64'd0;
        diag_ic_addr_ok       <= 64'd0;
        diag_ic_data_ok       <= 64'd0;
        diag_ic_uncached      <= 64'd0;
        diag_ic_uncached_direct <= 64'd0;
        diag_ic_uncached_pre  <= 64'd0;
        diag_ic_uncached_replay <= 64'd0;
        diag_pre_rsp_mismatch <= 64'd0;
        diag_if_rsp_mismatch  <= 64'd0;
        diag_first_uncached_pc <= 32'd0;
        diag_last_uncached_pc <= 32'd0;
    end else begin
        diag_cycle          <= diag_cycle + 64'd1;
        diag_ftq_valid      <= diag_ftq_valid + {63'd0, ftq_valid_i};
        diag_ftq_accept     <= diag_ftq_accept + {63'd0, ftq_accept_o};
        diag_ftq_pre_stall  <= diag_ftq_pre_stall
                             + {63'd0, (ftq_valid_i && !ftq_accept_o)};
        diag_pre_wait_req   <= diag_pre_wait_req
                             + {63'd0, (pre_v && !pre_excp && !pre_ic_sent)};
        diag_pre_wait_if    <= diag_pre_wait_if
                             + {63'd0, (pre_done && !if_allow_in)};
        diag_if_wait_data   <= diag_if_wait_data
                             + {63'd0, (if_v && if_wait_data)};
        diag_if_wait_ib     <= diag_if_wait_ib
                             + {63'd0, (if_v && !ib_can_push)};
        diag_ic_outstanding <= diag_ic_outstanding + {63'd0, ic_outstanding};
        diag_ic_req         <= diag_ic_req + {63'd0, ic_req_o};
        diag_ic_addr_ok     <= diag_ic_addr_ok
                             + {63'd0, (ic_req_o && ic_addr_ok_i)};
        diag_ic_data_ok     <= diag_ic_data_ok + {63'd0, ic_data_ok_i};
        diag_ic_uncached    <= diag_ic_uncached
                             + {63'd0, (ic_req_o && ic_addr_ok_i
                                      && ic_uncached_o)};
        diag_ic_uncached_direct <= diag_ic_uncached_direct
                             + {63'd0, (ftq_direct_req && ic_addr_ok_i
                                      && (mmu_i_direct_mat_i != 2'd1))};
        diag_ic_uncached_pre <= diag_ic_uncached_pre
                             + {63'd0, (pre_ic_req && ic_addr_ok_i
                                      && pre_uncached)};
        diag_ic_uncached_replay <= diag_ic_uncached_replay
                             + {63'd0, (if_replay_req && ic_addr_ok_i
                                      && if_uncached)};
        if (ic_req_o && ic_addr_ok_i && ic_uncached_o) begin
            if (diag_ic_uncached == 64'd0)
                diag_first_uncached_pc <= {ic_vline, 5'd0};
            diag_last_uncached_pc <= {ic_vline, 5'd0};
        end
        diag_pre_rsp_mismatch <= diag_pre_rsp_mismatch
                             + {63'd0, (ic_data_ok_i && pre_v
                                      && pre_ic_sent && !pre_rsp_match)};
        diag_if_rsp_mismatch <= diag_if_rsp_mismatch
                             + {63'd0, (ic_data_ok_i && if_v
                                      && if_wait_data && !if_rsp_match)};
    end
end

final begin
    $display("IFU diag: cycle=%0d ftq_valid=%0d accept=%0d pre_stall=%0d",
             diag_cycle, diag_ftq_valid, diag_ftq_accept, diag_ftq_pre_stall);
    $display("  pre_wait_req=%0d pre_wait_if=%0d if_wait_data=%0d if_wait_ib=%0d",
             diag_pre_wait_req, diag_pre_wait_if, diag_if_wait_data,
             diag_if_wait_ib);
    $display("  ic_outstanding=%0d req=%0d addr_ok=%0d data_ok=%0d",
             diag_ic_outstanding, diag_ic_req, diag_ic_addr_ok,
             diag_ic_data_ok);
    $display("  ic_uncached=%0d direct/pre/replay=%0d/%0d/%0d first/last=%08x/%08x",
             diag_ic_uncached, diag_ic_uncached_direct,
             diag_ic_uncached_pre, diag_ic_uncached_replay,
             diag_first_uncached_pc, diag_last_uncached_pc);
    $display("  rsp_mismatch: pre=%0d if=%0d",
             diag_pre_rsp_mismatch, diag_if_rsp_mismatch);
end
// synthesis translate_on
`endif

endmodule
