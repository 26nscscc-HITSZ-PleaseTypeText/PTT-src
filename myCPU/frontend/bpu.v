// ============================================================
// bpu 模块（分支预测单元顶层）
// ------------------------------------------------------------
// 参考实现说明：
// - P0 当拍：uBTB 命中则用其块，否则顺序取满至行边界（最多 4 条）；
// - P1 次拍：FTB/TAGE 1 拍延迟结果返回，与上一拍 P0 块比较，不同则覆盖 FTQ；
// - PC 更新优先级：flush > predec > P1 覆盖 > ftq_full 冻结 > P0 顺序；
// - 训练：FTB 全分支、TAGE 仅 COND、uBTB 仅向回跳（模块内过滤）；
// - RAS 双栈：P1 预测 CALL/RET 维护推测栈，flush 复制提交栈。
// ============================================================
`include "mycpu.h"

module bpu(
    input  wire                       clk,
    input  wire                       reset,

    input  wire                       flush_i,
    input  wire [31:0]                flush_pc_i,
    input  wire                       predec_redirect_i,
    input  wire                       predec_update_pc_i,
    input  wire [31:0]                predec_redirect_pc_i,

    input  wire                       ftq_full_i,

    output wire                       p0_valid_o,
    output wire [31:0]                p0_pc_o,
    output wire [`BLK_LEN_W-1:0]      p0_length_o,
    output wire                       p0_taken_o,
    output wire [31:0]                p0_target_o,
    output wire [`BR_TYPE_W-1:0]      p0_br_type_o,

    output wire                       p1_valid_o,
    output wire [31:0]                p1_pc_o,
    output wire [`BLK_LEN_W-1:0]      p1_length_o,
    output wire                       p1_taken_o,
    output wire [31:0]                p1_target_o,
    output wire [`BR_TYPE_W-1:0]      p1_br_type_o,
    output wire [`BPU_META_W-1:0]     p1_meta_o,

    input  wire                       train_valid_i,
    input  wire [31:0]                train_pc_i,
    input  wire                       train_is_branch_i,
    input  wire                       train_taken_i,
    input  wire                       train_mispred_i,
    input  wire [31:0]                train_target_i,
    input  wire [`BR_TYPE_W-1:0]      train_br_type_i,
    input  wire [31:0]                train_fall_through_i,
    input  wire [`BPU_META_W-1:0]     train_meta_i,

    input  wire                       cmt_is_call_i,
    input  wire                       cmt_is_ret_i,
    input  wire [31:0]                cmt_call_retaddr_i
);

// ---------------- 取指 PC ----------------
reg [31:0] pc;
reg [31:0] pc_r;
reg        flush_r;
reg [31:0] flush_pc_r;
reg        ftq_full_r;
reg        ftq_freeze_r;
reg        p0_wrote_r;
reg [31:0] p0_pc_r;
reg [`BLK_LEN_W-1:0] p0_length_r;
reg        p0_taken_r;
reg [31:0] p0_target_r;

initial begin
    pc          = 32'h1c000000;
    flush_pc_r  = 32'h1c000000;
end

// 满洋式两拍冻结：FTQ almost-full（留 2 槽）首拍照常查询/写入，
// 连续两拍满才冻结 PC/查询——满边界不丢 P1 覆盖、不浪费取指拍
wire ftq_freeze = ftq_full_i && ftq_full_r;

always @(posedge clk) begin
    if (flush_i)
        flush_pc_r <= flush_pc_i;
    flush_r      <= flush_i;
    ftq_full_r   <= ftq_full_i;
    ftq_freeze_r <= ftq_freeze;
    if (flush_i)
        p0_wrote_r <= 1'b0;
    else
        p0_wrote_r <= p0_valid_o;
    if (p0_valid_o) begin
        p0_pc_r     <= p0_pc_o;
        p0_length_r <= p0_length_o;
        p0_taken_r  <= p0_taken_o;
        p0_target_r <= p0_target_o;
    end
    pc_r <= pc;
end

// ---------------- 子模块：uBTB / FTB / TAGE / RAS ----------------
wire                       ubtb_hit;
wire                       ubtb_taken;
wire [31:0]                ubtb_target;
wire [`BLK_LEN_W-1:0]      ubtb_length;
wire [`BR_TYPE_W-1:0]      ubtb_btype;

wire                       ftb_hit;
wire [31:0]                ftb_target;
wire [31:0]                ftb_fall;
wire [`BR_TYPE_W-1:0]      ftb_btype;

wire                       tage_taken;
wire [`BPU_META_W-1:0]     tage_meta;

wire [31:0]                ras_top;
wire                       ras_empty;

wire query_en = ~ftq_freeze && ~flush_i;

wire [`BLK_LEN_W-1:0] ubtb_train_len = (train_fall_through_i - train_pc_i) >> 2;

ubtb u_ubtb(
    .clk               (clk),
    .reset             (reset),
    .query_pc_i        (pc),
    .hit_o             (ubtb_hit),
    .taken_o           (ubtb_taken),
    .target_o          (ubtb_target),
    .length_o          (ubtb_length),
    .br_type_o         (ubtb_btype),
    .update_valid_i    (train_valid_i && train_is_branch_i),
    .update_block_pc_i (train_pc_i),
    .update_taken_i    (train_taken_i),
    .update_target_i   (train_target_i),
    .update_length_i   (ubtb_train_len),
    .update_br_type_i  (train_br_type_i)
);

ftb u_ftb(
    .clk                 (clk),
    .reset               (reset),
    .query_valid_i       (query_en),
    .query_pc_i          (pc),
    .hit_o               (ftb_hit),
    .jump_target_o       (ftb_target),
    .fall_through_o      (ftb_fall),
    .br_type_o           (ftb_btype),
    .update_valid_i      (train_valid_i && train_is_branch_i),
    .update_block_pc_i   (train_pc_i),
    .update_jump_target_i(train_target_i),
    .update_fall_through_i(train_fall_through_i),
    .update_br_type_i    (train_br_type_i),
    .update_alloc_i      (train_mispred_i)
);

tage u_tage(
    .clk             (clk),
    .reset           (reset),
    .query_valid_i   (query_en),
    .query_pc_i      (pc),
    .taken_o         (tage_taken),
    .meta_o          (tage_meta),
    .train_valid_i   (train_valid_i && train_is_branch_i && (train_br_type_i == `BR_TYPE_COND)),
    .train_pc_i      (train_pc_i),
    .train_taken_i   (train_taken_i),
    .train_mispred_i (train_mispred_i),
    .train_meta_i    (train_meta_i)
);

ras u_ras(
    .clk              (clk),
    .reset            (reset),
    .flush_i          (flush_i),
    .spec_push_i      (ras_spec_push),
    .spec_push_addr_i (ftb_fall),
    .spec_pop_i       (ras_spec_pop),
    .top_addr_o       (ras_top),
    .empty_o          (ras_empty),
    .cmt_push_i       (cmt_is_call_i),
    .cmt_push_addr_i  (cmt_call_retaddr_i),
    .cmt_pop_i        (cmt_is_ret_i)
);

// ---------------- P0 基础块 ----------------
wire [3:0] words_to_eol = `CACHE_LINE_WORDS - {1'b0, pc[`CACHE_LINE_W-1:2]};
wire [`BLK_LEN_W-1:0] base_len = (words_to_eol > `FETCH_WIDTH) ? `FETCH_WIDTH
                                 : words_to_eol[`BLK_LEN_W-1:0];

wire [`BLK_LEN_W-1:0] p0_len_raw = (ubtb_hit && (ubtb_length <= base_len)) ? ubtb_length : base_len;
wire [`BLK_LEN_W-1:0] p0_len_c = (p0_len_raw === 3'd0 || p0_len_raw === 3'bx || p0_len_raw === 3'dz)
                                 ? 3'd1 : p0_len_raw;
wire                  p0_taken_c = ubtb_hit && ubtb_taken;
wire [31:0]           p0_target_c = ubtb_target;
wire [`BR_TYPE_W-1:0] p0_btype_c = ubtb_hit ? ubtb_btype : `BR_TYPE_COND;

// P1 覆盖拍必须压掉 P0：此拍的 pc 是被 P1 否定的错误路径延续，
// 若照写会在 FTQ 中留下一个"元数据不跳、取指流却已跳走"的幽灵块，
// 提交级误预测检查察觉不到（pred_taken=0 且非分支），导致错误路径静默提交
assign p0_valid_o   = query_en && !p1_diff && !(predec_redirect_i && predec_update_pc_i);
assign p0_pc_o      = pc;
assign p0_length_o  = p0_len_c;
assign p0_taken_o   = p0_taken_c;
assign p0_target_o  = p0_target_c;
assign p0_br_type_o = p0_btype_c;

wire [31:0] p0_next = p0_taken_c ? p0_target_c : (pc + {27'b0, p0_len_c, 2'b00});

// ---------------- P1 覆盖块 ----------------
wire [`BLK_LEN_W-1:0] p1_len_raw = (ftb_fall - pc_r) >> 2;
wire [`BLK_LEN_W-1:0] p1_len_mid   = (p1_len_raw === 3'd0) ? 3'd1 :
                                   (p1_len_raw > `FETCH_WIDTH) ? `FETCH_WIDTH : p1_len_raw;
wire [`BLK_LEN_W-1:0] p1_len_c   = (p1_len_mid === 3'bx || p1_len_mid === 3'dz) ? 3'd1 : p1_len_mid;

wire p1_taken_c = (ftb_btype == `BR_TYPE_COND) ? tage_taken : 1'b1;
wire [31:0] p1_target_c = (ftb_btype == `BR_TYPE_RET && !ras_empty) ? ras_top :
                          (ftb_btype == `BR_TYPE_RET) ? ftb_fall : ftb_target;

wire p1_diff = ftb_hit && p0_wrote_r && !ftq_freeze_r && !flush_r && !flush_i &&
               ((p1_len_c != p0_length_r) || (p1_taken_c != p0_taken_r) ||
                (p1_target_c != p0_target_r));

assign p1_valid_o   = p1_diff;
assign p1_pc_o      = pc_r;
assign p1_length_o  = p1_len_c;
assign p1_taken_o   = p1_taken_c;
assign p1_target_o  = p1_target_c;
assign p1_br_type_o = ftb_btype;
assign p1_meta_o    = tage_meta;

wire [31:0] p1_next = p1_taken_c ? p1_target_c : ftb_fall;

wire ras_spec_push = p1_diff && (ftb_btype == `BR_TYPE_CALL);
wire ras_spec_pop  = p1_diff && (ftb_btype == `BR_TYPE_RET) && !ras_empty;

// ---------------- PC 更新 ----------------
always @(posedge clk) begin
    if (reset)
        pc <= 32'h1c000000;
    else if (flush_i)
        pc <= flush_pc_i;
    // 注意:不要在 flush 的下一拍(flush_r)再次把 pc 钉回 flush_pc_r。
    // flush_i 当拍 query_en=0(无 P0),pc 已置 flush_pc_i;下一拍(flush_r)query_en=1
    // 正常发出 flush_pc 的 P0 块并写入 FTQ——这是正确的重取指块。若此拍 posedge 再用
    // flush_r 把 pc 钉回 flush_pc_r,则 pc 在 flush_r 拍与其后一拍连续两拍都等于 flush_pc,
    // 而 query_en 两拍都为 1 → 同一块被取指/分配/提交两次(FLUSH_REFETCH 双取)。
    // 表现:csrwr/csrxchg(每条都触发 FLUSH_REFETCH)之后那条指令被提交两次
    //   → n47/n48/n50 的 confreg store 双写("numbers unequal")、
    //     n49/n51 的成功计数器 addi r26,r26,1 自增 2("Occurred in number")。
    // 修复:删掉 flush_r 的 pc 钉回,让 flush_r 拍取指后 pc 正常推进到 p0_next。
    else if (predec_redirect_i && predec_update_pc_i)
        pc <= predec_redirect_pc_i;
    else if (p1_diff)
        pc <= p1_next;
    else if (!ftq_freeze)
        pc <= p0_next;
end

// synthesis translate_off
// 临时调试：捕获 PC 离开 1c 代码段的时刻（调通后删除）
always @(posedge clk) begin
    if (!reset && (pc[31:24] == 8'h1c)) begin
        if (flush_i && (flush_pc_i[31:24] != 8'h1c))
            $display("[%0t] BPU_DBG flush -> %h", $time, flush_pc_i);
        else if (!flush_i && flush_r && (flush_pc_r[31:24] != 8'h1c))
            $display("[%0t] BPU_DBG flush_r -> %h", $time, flush_pc_r);
        else if (!flush_i && !flush_r && predec_redirect_i && predec_update_pc_i && (predec_redirect_pc_i[31:24] != 8'h1c))
            $display("[%0t] BPU_DBG predec -> %h (pc=%h)", $time, predec_redirect_pc_i, pc);
        else if (!flush_i && !flush_r && !(predec_redirect_i && predec_update_pc_i) && p1_diff && (p1_next[31:24] != 8'h1c))
            $display("[%0t] BPU_DBG p1 -> %h (pc_r=%h taken=%b tgt=%h fall=%h btype=%b rasE=%b)", $time,
                     p1_next, pc_r, p1_taken_c, ftb_target, ftb_fall, ftb_btype, ras_empty);
        else if (!flush_i && !flush_r && !(predec_redirect_i && predec_update_pc_i) && !p1_diff && !ftq_freeze && (p0_next[31:24] != 8'h1c))
            $display("[%0t] BPU_DBG p0 -> %h (pc=%h ubtb=%b taken=%b tgt=%h)", $time,
                     p0_next, pc, ubtb_hit, p0_taken_c, p0_target_c);
    end
end
// synthesis translate_on

endmodule
