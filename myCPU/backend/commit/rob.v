// ============================================================
// rob 模块（重排序缓冲：奇偶双体环形队列，队列式重命名的核心）
// ------------------------------------------------------------
// 功能：
// - `ROB_SIZE=32 项 = 16 个"槽位对" × 2 路（奇偶分体）：
//   每拍恒分配一对（槽 0 -> {1'b0,tail}，槽 1 -> {1'b1,tail}），
//   ROB 编号同时承担三重角色：重命名标签 + 程序序 + 结果存储索引。
// - head/tail 环形指针；满判据 head == tail + `ROB_GUARD（保留 5 对安全
//   间距：保证"已提交但结果仍可能被 dispatch 读取"的表项不被新分配覆盖）。
// - 4 个 FU 写回口各写各的（无端口冲突）；提交口读队头一对；
//   dispatch 4 个读口取未就绪源操作数的值（带同拍写回旁路）。
// - NOP 消除：分配时 is_nop 直接标 complete，不经执行。
//
// 端口分组：
// - alloc_*  ：rename 成对分配（静态信息）
// - raddr/rrdy/rdata ×4 ：dispatch 操作数读口
// - alu0/alu1/mem/mdu wb ：4 路 FU 写回口
// - cmt0/cmt1_* ：队头一对的全部信息（送 commit 仲裁）
// - cmt_pop/clear ：提交推进控制
// ============================================================
`include "mycpu.h"

module rob(
    input  wire                       clk,
    input  wire                       reset,
    input  wire                       flush_i,            // 全局冲刷：head/tail/valid 全清

    // =============== 分配口（rename，恒成对） ===============
    input  wire                       alloc_en_i,         // 本拍分配一对
    output wire [`ROB_PAIR_W-1:0]     rob_tail_o,         // 当前队尾对指针
    output wire                       rob_full_o,         // 满（含安全间距）
    output wire                       rob_empty_o,

    // ---- 槽 0 静态信息 ----
    input  wire                       a0_valid_i,
    input  wire [31:0]                a0_pc_i,
    input  wire [31:0]                a0_inst_i,
    input  wire                       a0_rf_we_i,
    input  wire [4:0]                 a0_rd_i,
    input  wire                       a0_is_load_i,
    input  wire                       a0_is_store_i,
    input  wire                       a0_is_branch_i,
    input  wire [`BR_TYPE_W-1:0]      a0_br_type_i,
    input  wire                       a0_pred_taken_i,
    input  wire                       a0_is_last_i,
    input  wire [`FTQ_W-1:0]          a0_ftq_id_i,
    input  wire [`PRIV_NUM-1:0]       a0_priv_vec_i,
    input  wire [13:0]                a0_csr_num_i,
    input  wire [`TLB_OP_NUM-1:0]     a0_tlb_op_i,        // TLB 维护类型（提交级落地）
    input  wire [4:0]                 a0_cacop_code_i,
    input  wire [`EXCP_NUM-1:0]       a0_excp_i,          // 静态异常（取指/译码）
    input  wire                       a0_is_nop_i,

    // ---- 槽 1 静态信息 ----
    input  wire                       a1_valid_i,
    input  wire [31:0]                a1_pc_i,
    input  wire [31:0]                a1_inst_i,
    input  wire                       a1_rf_we_i,
    input  wire [4:0]                 a1_rd_i,
    input  wire                       a1_is_load_i,
    input  wire                       a1_is_store_i,
    input  wire                       a1_is_branch_i,
    input  wire [`BR_TYPE_W-1:0]      a1_br_type_i,
    input  wire                       a1_pred_taken_i,
    input  wire                       a1_is_last_i,
    input  wire [`FTQ_W-1:0]          a1_ftq_id_i,
    input  wire [`PRIV_NUM-1:0]       a1_priv_vec_i,
    input  wire [13:0]                a1_csr_num_i,
    input  wire [`TLB_OP_NUM-1:0]     a1_tlb_op_i,
    input  wire [4:0]                 a1_cacop_code_i,
    input  wire [`EXCP_NUM-1:0]       a1_excp_i,
    input  wire                       a1_is_nop_i,

    // =============== dispatch 操作数读口 ×4（组合，带同拍写回旁路） ===============
    input  wire [`ROB_W-1:0]          raddr0_i,
    output wire                       rrdy0_o,            // 该编号已写回（含本拍正在写回）
    output wire [31:0]                rdata0_o,
    input  wire [`ROB_W-1:0]          raddr1_i,
    output wire                       rrdy1_o,
    output wire [31:0]                rdata1_o,
    input  wire [`ROB_W-1:0]          raddr2_i,
    output wire                       rrdy2_o,
    output wire [31:0]                rdata2_o,
    input  wire [`ROB_W-1:0]          raddr3_i,
    output wire                       rrdy3_o,
    output wire [31:0]                rdata3_o,

    // =============== FU 写回口 ×4 ===============
    // ---- fu_alu0 ----
    input  wire                       alu0_wb_valid_i,
    input  wire [`ROB_W-1:0]          alu0_wb_robid_i,
    input  wire [31:0]                alu0_wb_data_i,
    input  wire                       alu0_wb_br_taken_i,
    input  wire [31:0]                alu0_wb_br_target_i,
    // ---- fu_alu1 ----
    input  wire                       alu1_wb_valid_i,
    input  wire [`ROB_W-1:0]          alu1_wb_robid_i,
    input  wire [31:0]                alu1_wb_data_i,
    input  wire                       alu1_wb_br_taken_i,
    input  wire [31:0]                alu1_wb_br_target_i,
    // ---- lsu ----
    input  wire                       mem_wb_valid_i,
    input  wire [`ROB_W-1:0]          mem_wb_robid_i,
    input  wire [31:0]                mem_wb_data_i,
    input  wire [31:0]                mem_wb_paddr_i,
    input  wire [31:0]                mem_wb_vaddr_i,
    input  wire [3:0]                 mem_wb_wstrb_i,
    input  wire [2:0]                 mem_wb_size_i,
    input  wire                       mem_wb_uncached_i,
    input  wire [`EXCP_NUM-1:0]       mem_wb_excp_i,      // 动态异常（与静态按位或）
    // The raw memory completion above updates ROB state immediately.  ROB
    // read ports use this registered copy for same-cycle operand forwarding,
    // because a newly dispatched consumer cannot issue before the next cycle.
    input  wire                       mem_fwd_valid_i,
    input  wire [`ROB_W-1:0]          mem_fwd_robid_i,
    input  wire [31:0]                mem_fwd_data_i,
    // ---- fu_mdu ----
    input  wire                       mdu_wb_valid_i,
    input  wire [`ROB_W-1:0]          mdu_wb_robid_i,
    input  wire [31:0]                mdu_wb_data_i,
    input  wire [31:0]                mdu_wb_data2_i,     // CSR 新值 / invtlb {vppn,asid} 打包

    // =============== 提交口：队头一对的全部信息 ===============
    output wire [`ROB_W-1:0]          head_robid0_o,      // 队头槽 0 编号（uncached load 许可比较用）
    // ---- 槽 0 ----
    output wire                       cmt0_valid_o,
    output wire                       cmt0_complete_o,
    output wire [31:0]                cmt0_pc_o,
    output wire [31:0]                cmt0_inst_o,        // 仅仿真视图有效（difftest/调试）；综合恒 0
    output wire                       cmt0_inst_is_b0_o,  // 预译码：inst==0x50000000（套件 `b 0` 空转）
    output wire                       cmt0_is_direct_b_o, // 预译码：inst[31:26]==010100（直接 B）
    output wire                       cmt0_rf_we_o,
    output wire [4:0]                 cmt0_rd_o,
    output wire [31:0]                cmt0_result_o,      // 写回 ARF 的值
    output wire [31:0]                cmt0_result2_o,     // CSR 新值/invtlb 打包/…
    output wire                       cmt0_is_load_o,
    output wire                       cmt0_is_store_o,
    output wire [31:0]                cmt0_paddr_o,
    output wire [31:0]                cmt0_vaddr_o,
    output wire [3:0]                 cmt0_wstrb_o,
    output wire [2:0]                 cmt0_size_o,
    output wire                       cmt0_uncached_o,
    output wire                       cmt0_is_branch_o,
    output wire [`BR_TYPE_W-1:0]      cmt0_br_type_o,
    output wire                       cmt0_pred_taken_o,
    output wire                       cmt0_br_taken_o,    // 实际方向（FU 写回）
    output wire [31:0]                cmt0_br_target_o,   // 实际目标
    output wire                       cmt0_is_last_o,
    output wire [`FTQ_W-1:0]          cmt0_ftq_id_o,
    output wire [`PRIV_NUM-1:0]       cmt0_priv_vec_o,
    output wire [13:0]                cmt0_csr_num_o,
    output wire [`TLB_OP_NUM-1:0]     cmt0_tlb_op_o,
    output wire [4:0]                 cmt0_cacop_code_o,
    output wire [`EXCP_NUM-1:0]       cmt0_excp_o,        // 静态|动态 合并后
    // ---- 槽 1 ----
    output wire                       cmt1_valid_o,
    output wire                       cmt1_complete_o,
    output wire [31:0]                cmt1_pc_o,
    output wire [31:0]                cmt1_inst_o,        // 仅仿真视图有效；综合恒 0
    output wire                       cmt1_inst_is_b0_o,
    output wire                       cmt1_is_direct_b_o,
    output wire                       cmt1_rf_we_o,
    output wire [4:0]                 cmt1_rd_o,
    output wire [31:0]                cmt1_result_o,
    output wire [31:0]                cmt1_result2_o,
    output wire                       cmt1_is_load_o,
    output wire                       cmt1_is_store_o,
    output wire [31:0]                cmt1_paddr_o,
    output wire [31:0]                cmt1_vaddr_o,
    output wire [3:0]                 cmt1_wstrb_o,
    output wire [2:0]                 cmt1_size_o,
    output wire                       cmt1_uncached_o,
    output wire                       cmt1_is_branch_o,
    output wire [`BR_TYPE_W-1:0]      cmt1_br_type_o,
    output wire                       cmt1_pred_taken_o,
    output wire                       cmt1_br_taken_o,
    output wire [31:0]                cmt1_br_target_o,
    output wire                       cmt1_is_last_o,
    output wire [`FTQ_W-1:0]          cmt1_ftq_id_o,
    output wire [`PRIV_NUM-1:0]       cmt1_priv_vec_o,
    output wire [13:0]                cmt1_csr_num_o,
    output wire [`TLB_OP_NUM-1:0]     cmt1_tlb_op_o,
    output wire [4:0]                 cmt1_cacop_code_o,
    output wire [`EXCP_NUM-1:0]       cmt1_excp_o,

    // ---- 提交推进控制（commit 仲裁结果）----
    input  wire                       cmt_pop_i,          // head++（队头对处理完毕）
    input  wire                       cmt_clear0_i,       // 槽 0 已提交（清 valid，可早于 pop）
    input  wire                       cmt_clear1_i        // 槽 1 已提交
);

// 设计说明：
//
// 指针与满判据：
//      reg [`ROB_PAIR_W-1:0] head, tail;
//      rob_full_o  = (head == trunc(tail + ROB_GUARD)); // 保留安全间距，环形截断
//      rob_empty_o = (head == tail);
//      为什么留间距：提交后（clear/pop）该表项的 result 仍可能在同拍/下拍被
//      dispatch 读口用旧编号读取（rename 在它提交前一拍刚查到这个标签）；
//      留 GUARD 对间距保证新分配不会立即覆盖刚提交项的结果。删掉间距会出现
//      “读到新指令结果”的错误，因此该安全间距属于接口不变式。
//
// 存储结构（编号={奇偶,对指针}）：
//      静态区（分配写、提交读一次）：按【奇偶双体 LUTRAM】存放（sta0_*/sta1_*，
//        每体 dispatch 单写口）：pc、csr_num、tlb_op、cacop_code、ftq_id、
//        br_type 及提交预译码位（is_direct_b/is_idle/csr 字段选择等）；
//        32b inst[] 阵列仅 `ifdef SIMU 保留（difftest/debug 用），
//        综合网表不含 inst 存储与 inst[head] 读 mux。
//      动态区（写回写/需冲刷/多写口）：保持 FF：valid、complete、result、
//        result2、paddr/vaddr、wstrb、size、uncached、br_taken、br_target、
//        excp 等。
//
// 分配（alloc_en_i）：
//      把 a0_* 写入 {1'b0,tail} 项、a1_* 写入 {1'b1,tail} 项；tail++；
//      complete 初值 = is_nop || (|excp_static)（NOP 与带静态异常项分配即完成）；
//      a*_valid=0 的槽也要写（valid=0 占位），保证队头判断简单。
//
// FU 写回（4 路并行，互不冲突）：
//      alu0/alu1：result<=data；br_taken/br_target 锁存；complete<=1
//      mem      ：result<=data；paddr/vaddr/wstrb/size/uncached/excp_dynamic 锁存；complete<=1
//      mdu      ：result<=data；result2<=data2；complete<=1
//
// dispatch 读口（组合 + 同拍写回旁路）：
//      rrdy = complete[raddr] | (任一 wb 口本拍 valid 且 robid==raddr)
//      rdata = 本拍写回旁路优先，否则读 result[raddr]
//      （旁路必不可少：写回与 dispatch 同拍时，不旁路会让指令在 RS 里
//        错过唤醒、又读不到 ROB 值，死等。）
//
// 提交口：
//      cmt0_* = 表项 {1'b0,head} 的全部字段；cmt1_* = {1'b1,head}；
//      cmt*_excp_o = excp_static | excp_dynamic；
//      head_robid0_o = {1'b0, head}（lsu 的 uncached 许可比较用）。
//      cmt_clear0/1_i：清对应项 valid（提交完成）；cmt_pop_i：head++。
//
// 冲刷：flush_i 时 head/tail 清 0、valid/complete 全清（一拍完成）。
//
// 坑点提示：
//      1. 同拍"分配写静态区"与"写回写动态区"作用于不同表项（GUARD 保证），
//         不会冲突；但代码里要分开两个 always 块写清楚。
//      2. complete 位在分配时必须清 0（is_nop/静态异常除外）——上一轮用过的
//         旧值残留会导致指令"未执行就提交"，查死人。
//      3. 双发射槽 1 无效时照样占位（valid=0），提交仲裁会跳过它。

reg [`ROB_PAIR_W-1:0] head;
reg [`ROB_PAIR_W-1:0] tail;

// ---------------- 多写口 / 需冲刷字段：保持触发器 ----------------
// valid/complete：分配+提交+冲刷多点写；动态区：分配清 0 + FU 写回（两写点）。
reg                       valid [0:`ROB_SIZE-1];
reg                       complete [0:`ROB_SIZE-1];

reg [31:0]                result [0:`ROB_SIZE-1];
reg [31:0]                result2 [0:`ROB_SIZE-1];
reg [31:0]                paddr [0:`ROB_SIZE-1];
reg [31:0]                vaddr [0:`ROB_SIZE-1];
reg [3:0]                 wstrb [0:`ROB_SIZE-1];
reg [2:0]                 size [0:`ROB_SIZE-1];
reg                       uncached [0:`ROB_SIZE-1];
reg                       br_taken [0:`ROB_SIZE-1];
reg [31:0]                br_target [0:`ROB_SIZE-1];
reg [`EXCP_NUM-1:0]       excp_dynamic [0:`ROB_SIZE-1];

integer i;

wire [`ROB_W-1:0] alloc0_idx = {1'b0, tail};
wire [`ROB_W-1:0] alloc1_idx = {1'b1, tail};
wire [`ROB_W-1:0] head0_idx  = {1'b0, head};
wire [`ROB_W-1:0] head1_idx  = {1'b1, head};

// ---------------- 静态区：奇偶双体 LUTRAM（写一次读一次） ----------------
// 分配拍恒成对写（偶体<=槽0、奇体<=槽1，各自单写口 @tail），提交只读 @head，
// 满足分布式 RAM 1W1R 异步读模型；冲刷无需清内容（valid=0 即屏蔽陈旧值）。
// ROB 不保存 futype；执行类型只用于前级路由，提交所需类别使用独立位记录。
// inst 32b 仅 difftest/调试需要：只在仿真视图保留（见下 `ifdef SIMU 段），
// 综合视图用分配拍预译码的 2 bit（is_b0 / direct_b）替代提交侧全部真实用途，
// 同时把 inst[head] 的 32b 读 mux 从"提交异常链"关键路径起点上摘除。
localparam STA_PC_LSB    = 0;
localparam STA_B0_LSB    = STA_PC_LSB    + 32;             // inst==0x50000000
localparam STA_DIRB_LSB  = STA_B0_LSB    + 1;              // inst[31:26]==010100
localparam STA_RFWE_LSB  = STA_DIRB_LSB  + 1;
localparam STA_RD_LSB    = STA_RFWE_LSB  + 1;
localparam STA_ISLD_LSB  = STA_RD_LSB    + 5;
localparam STA_ISST_LSB  = STA_ISLD_LSB  + 1;
localparam STA_ISBR_LSB  = STA_ISST_LSB  + 1;
localparam STA_BRT_LSB   = STA_ISBR_LSB  + 1;
localparam STA_PRDT_LSB  = STA_BRT_LSB   + `BR_TYPE_W;
localparam STA_LAST_LSB  = STA_PRDT_LSB  + 1;
localparam STA_FTQ_LSB   = STA_LAST_LSB  + 1;
localparam STA_PRIV_LSB  = STA_FTQ_LSB   + `FTQ_W;
localparam STA_CSRN_LSB  = STA_PRIV_LSB  + `PRIV_NUM;
localparam STA_TLBOP_LSB = STA_CSRN_LSB  + 14;
localparam STA_CACOP_LSB = STA_TLBOP_LSB + `TLB_OP_NUM;
localparam STA_EXCP_LSB  = STA_CACOP_LSB + 5;
localparam STA_W         = STA_EXCP_LSB  + `EXCP_NUM;

(* ram_style = "distributed" *) reg [STA_W-1:0] sta_even [0:`ROB_SIZE/2-1];
(* ram_style = "distributed" *) reg [STA_W-1:0] sta_odd  [0:`ROB_SIZE/2-1];

wire a0_inst_is_b0  = (a0_inst_i == 32'h50000000);
wire a1_inst_is_b0  = (a1_inst_i == 32'h50000000);
wire a0_is_direct_b = (a0_inst_i[31:26] == 6'b010100);
wire a1_is_direct_b = (a1_inst_i[31:26] == 6'b010100);

wire [STA_W-1:0] sta_pack0 = {a0_excp_i, a0_cacop_code_i, a0_tlb_op_i, a0_csr_num_i,
                              a0_priv_vec_i, a0_ftq_id_i, a0_is_last_i, a0_pred_taken_i,
                              a0_br_type_i, a0_is_branch_i, a0_is_store_i, a0_is_load_i,
                              a0_rd_i, a0_rf_we_i, a0_is_direct_b, a0_inst_is_b0, a0_pc_i};
wire [STA_W-1:0] sta_pack1 = {a1_excp_i, a1_cacop_code_i, a1_tlb_op_i, a1_csr_num_i,
                              a1_priv_vec_i, a1_ftq_id_i, a1_is_last_i, a1_pred_taken_i,
                              a1_br_type_i, a1_is_branch_i, a1_is_store_i, a1_is_load_i,
                              a1_rd_i, a1_rf_we_i, a1_is_direct_b, a1_inst_is_b0, a1_pc_i};

wire alloc_fire = !reset && !flush_i && alloc_en_i && !rob_full_o;

always @(posedge clk) begin
    if (alloc_fire) begin
        sta_even[tail] <= sta_pack0;
        sta_odd[tail]  <= sta_pack1;
    end
end

wire [STA_W-1:0] sta_h0 = sta_even[head];
wire [STA_W-1:0] sta_h1 = sta_odd[head];

// Static no-execute entries do not need a per-entry completion FF write.
// Derive their readiness only after the saved ROB static row reaches head:
//   1) fetch/decode exception;
//   2) DBAR/IBAR (the commit stage still performs the barrier action);
//   3) no RF/memory/branch/privileged side effect.
// This removes the IFU->decode->ROB complete allocation path and also avoids
// storing is_nop in the static RAM.
wire cmt0_static_excp = |sta_h0[STA_EXCP_LSB +: `EXCP_NUM];
wire cmt1_static_excp = |sta_h1[STA_EXCP_LSB +: `EXCP_NUM];
wire cmt0_barrier =
    sta_h0[STA_PRIV_LSB + `PRIV_IBAR];
wire cmt1_barrier =
    sta_h1[STA_PRIV_LSB + `PRIV_IBAR];
wire cmt0_no_side_effect =
    !sta_h0[STA_RFWE_LSB]
    && !sta_h0[STA_ISLD_LSB]
    && !sta_h0[STA_ISST_LSB]
    && !sta_h0[STA_ISBR_LSB]
    && !(|sta_h0[STA_PRIV_LSB +: `PRIV_NUM]);
wire cmt1_no_side_effect =
    !sta_h1[STA_RFWE_LSB]
    && !sta_h1[STA_ISLD_LSB]
    && !sta_h1[STA_ISST_LSB]
    && !sta_h1[STA_ISBR_LSB]
    && !(|sta_h1[STA_PRIV_LSB +: `PRIV_NUM]);
wire cmt0_static_done =
    cmt0_static_excp || cmt0_barrier || cmt0_no_side_effect;
wire cmt1_static_done =
    cmt1_static_excp || cmt1_barrier || cmt1_no_side_effect;

assign rob_tail_o = tail;
// 满判据必须按 ROB_PAIR_W 位宽环形加：`ROB_GUARD` 是无宽度十进制字面量，
// 若写成 (head == tail+ROB_GUARD)，右边被扩成 32 位（如 11+5=16），与 4 位
// head 比较永远对不上 wrap 后的 0，SB 阻塞 head 时 tail 会绕回覆盖尚未提交项
// （linux pagetable_init：ae24@0080 被 OVW → digftest 看见 @00c0）。
wire [`ROB_PAIR_W-1:0] rob_full_mark = tail + `ROB_GUARD;
assign rob_full_o = (head == rob_full_mark);
assign rob_empty_o = (head == tail);
assign head_robid0_o = head0_idx;

// 读口 ready/data【不得】用 valid 门控：
// RAT 的 busy/tag 相对提交晚一拍——生产者提交当拍，消费者 rename 仍拿到
// "busy=1, tag=生产者" 的旧视图，下一拍 dispatch 读 ROB 时表项已 pop。
// 若此处再检查 valid，该操作数永远等不到唤醒，直到 robid 被新指令复用后
// 捕获错误数据（ABA）。complete/result 在 pop 后保留、重新分配时清除，
// 故 pop 后一拍窗口内仍可安全读出正确值。
// 注意：读口用【每口内联组合】而非 function 调用——xsim 在 continuous assign
// 里对"带可变下标的 function"存在求值/敏感表缺陷，会返回上一次求值下标的旧值
// （表现为 raddr=13 却读出 result[9]）。内联后按 raddr 直接索引，杜绝此问题。
`define ROB_RDPORT(P) \
    wire wbhit``P = (alu0_wb_valid_i && (alu0_wb_robid_i == raddr``P``_i)) || \
                    (alu1_wb_valid_i && (alu1_wb_robid_i == raddr``P``_i)) || \
                    (mem_fwd_valid_i && (mem_fwd_robid_i == raddr``P``_i)) || \
                    (mdu_wb_valid_i  && (mdu_wb_robid_i  == raddr``P``_i)); \
    wire [31:0] wbdat``P = (alu0_wb_valid_i && (alu0_wb_robid_i == raddr``P``_i)) ? alu0_wb_data_i : \
                           (alu1_wb_valid_i && (alu1_wb_robid_i == raddr``P``_i)) ? alu1_wb_data_i : \
                           (mem_fwd_valid_i && (mem_fwd_robid_i == raddr``P``_i)) ? mem_fwd_data_i : \
                           (mdu_wb_valid_i  && (mdu_wb_robid_i  == raddr``P``_i)) ? mdu_wb_data_i  : 32'b0; \
    assign rrdy``P``_o  = complete[raddr``P``_i] | wbhit``P; \
    assign rdata``P``_o = wbhit``P ? wbdat``P : result[raddr``P``_i];
`ROB_RDPORT(0)
`ROB_RDPORT(1)
`ROB_RDPORT(2)
`ROB_RDPORT(3)
`undef ROB_RDPORT

assign cmt0_valid_o = valid[head0_idx];
assign cmt0_complete_o = complete[head0_idx] || cmt0_static_done;
assign cmt0_pc_o = sta_h0[STA_PC_LSB +: 32];
assign cmt0_inst_is_b0_o = sta_h0[STA_B0_LSB];
assign cmt0_is_direct_b_o = sta_h0[STA_DIRB_LSB];
assign cmt0_rf_we_o = sta_h0[STA_RFWE_LSB];
assign cmt0_rd_o = sta_h0[STA_RD_LSB +: 5];
assign cmt0_result_o = result[head0_idx];
assign cmt0_result2_o = result2[head0_idx];
assign cmt0_is_load_o = sta_h0[STA_ISLD_LSB];
assign cmt0_is_store_o = sta_h0[STA_ISST_LSB];
assign cmt0_paddr_o = paddr[head0_idx];
assign cmt0_vaddr_o = vaddr[head0_idx];
assign cmt0_wstrb_o = wstrb[head0_idx];
assign cmt0_size_o = size[head0_idx];
assign cmt0_uncached_o = uncached[head0_idx];
assign cmt0_is_branch_o = sta_h0[STA_ISBR_LSB];
assign cmt0_br_type_o = sta_h0[STA_BRT_LSB +: `BR_TYPE_W];
assign cmt0_pred_taken_o = sta_h0[STA_PRDT_LSB];
assign cmt0_br_taken_o = br_taken[head0_idx];
assign cmt0_br_target_o = br_target[head0_idx];
assign cmt0_is_last_o = sta_h0[STA_LAST_LSB];
assign cmt0_ftq_id_o = sta_h0[STA_FTQ_LSB +: `FTQ_W];
assign cmt0_priv_vec_o = sta_h0[STA_PRIV_LSB +: `PRIV_NUM];
assign cmt0_csr_num_o = sta_h0[STA_CSRN_LSB +: 14];
assign cmt0_tlb_op_o = sta_h0[STA_TLBOP_LSB +: `TLB_OP_NUM];
assign cmt0_cacop_code_o = sta_h0[STA_CACOP_LSB +: 5];
assign cmt0_excp_o = sta_h0[STA_EXCP_LSB +: `EXCP_NUM] | excp_dynamic[head0_idx];

assign cmt1_valid_o = valid[head1_idx];
assign cmt1_complete_o = complete[head1_idx] || cmt1_static_done;
assign cmt1_pc_o = sta_h1[STA_PC_LSB +: 32];
assign cmt1_inst_is_b0_o = sta_h1[STA_B0_LSB];
assign cmt1_is_direct_b_o = sta_h1[STA_DIRB_LSB];
assign cmt1_rf_we_o = sta_h1[STA_RFWE_LSB];
assign cmt1_rd_o = sta_h1[STA_RD_LSB +: 5];
assign cmt1_result_o = result[head1_idx];
assign cmt1_result2_o = result2[head1_idx];
assign cmt1_is_load_o = sta_h1[STA_ISLD_LSB];
assign cmt1_is_store_o = sta_h1[STA_ISST_LSB];
assign cmt1_paddr_o = paddr[head1_idx];
assign cmt1_vaddr_o = vaddr[head1_idx];
assign cmt1_wstrb_o = wstrb[head1_idx];
assign cmt1_size_o = size[head1_idx];
assign cmt1_uncached_o = uncached[head1_idx];
assign cmt1_is_branch_o = sta_h1[STA_ISBR_LSB];
assign cmt1_br_type_o = sta_h1[STA_BRT_LSB +: `BR_TYPE_W];
assign cmt1_pred_taken_o = sta_h1[STA_PRDT_LSB];
assign cmt1_br_taken_o = br_taken[head1_idx];
assign cmt1_br_target_o = br_target[head1_idx];
assign cmt1_is_last_o = sta_h1[STA_LAST_LSB];
assign cmt1_ftq_id_o = sta_h1[STA_FTQ_LSB +: `FTQ_W];
assign cmt1_priv_vec_o = sta_h1[STA_PRIV_LSB +: `PRIV_NUM];
assign cmt1_csr_num_o = sta_h1[STA_CSRN_LSB +: 14];
assign cmt1_tlb_op_o = sta_h1[STA_TLBOP_LSB +: `TLB_OP_NUM];
assign cmt1_cacop_code_o = sta_h1[STA_CACOP_LSB +: 5];
assign cmt1_excp_o = sta_h1[STA_EXCP_LSB +: `EXCP_NUM] | excp_dynamic[head1_idx];

// inst 32b 仅仿真视图保留（difftest/调试观测）；综合视图恒 0，
// 提交侧真实用途（idle `b 0` 判定 / 槽1 直接 B 判定）已由预译码位承担。
`ifdef SIMU
reg [31:0] inst [0:`ROB_SIZE-1];
always @(posedge clk) begin
    if (alloc_fire) begin
        inst[alloc0_idx] <= a0_inst_i;
        inst[alloc1_idx] <= a1_inst_i;
    end
end
assign cmt0_inst_o = inst[head0_idx];
assign cmt1_inst_o = inst[head1_idx];
`else
assign cmt0_inst_o = 32'b0;
assign cmt1_inst_o = 32'b0;
`endif

always @(posedge clk) begin
    if (reset || flush_i) begin
        head <= {`ROB_PAIR_W{1'b0}};
        tail <= {`ROB_PAIR_W{1'b0}};
        for (i = 0; i < `ROB_SIZE; i = i + 1) begin
            valid[i] <= 1'b0;
            complete[i] <= 1'b0;
            excp_dynamic[i] <= {`EXCP_NUM{1'b0}};
        end
    end else begin
        // pop 只清 valid，complete/result 保留至该项被重新分配（alloc 时覆写）。
        // 原因：RAT 的 busy 视图比提交晚一拍——生产者提交当拍 rename 仍拿到
        // 旧标签；下一拍 dispatch 仍可能读取该 robid，因此 complete/result
        // 必须保留到重新分配，否则消费者会错过唤醒并在 robid 复用后捕获错误数据。
        if (cmt_clear0_i) begin
            valid[head0_idx]   <= 1'b0;
        end
        if (cmt_clear1_i) begin
            valid[head1_idx]   <= 1'b0;
        end
        if (cmt_pop_i) begin
            head <= head + 1'b1;
        end

        if (alloc_en_i && !rob_full_o) begin
            // 静态字段写已移至 sta_even/sta_odd LUTRAM（alloc_fire 同拍同条件）
            valid[alloc0_idx] <= a0_valid_i;
            // Static no-execute completion is derived from sta_h* only at
            // the ROB head.  Allocation therefore has no decode->complete
            // timing dependency and always clears a reused dynamic bit.
            complete[alloc0_idx] <= 1'b0;
            result[alloc0_idx] <= 32'b0;
            result2[alloc0_idx] <= 32'b0;
            paddr[alloc0_idx] <= 32'b0;
            vaddr[alloc0_idx] <= 32'b0;
            wstrb[alloc0_idx] <= 4'b0;
            size[alloc0_idx] <= 3'b0;
            uncached[alloc0_idx] <= 1'b0;
            br_taken[alloc0_idx] <= 1'b0;
            br_target[alloc0_idx] <= 32'b0;
            excp_dynamic[alloc0_idx] <= {`EXCP_NUM{1'b0}};

            valid[alloc1_idx] <= a1_valid_i;
            complete[alloc1_idx] <= 1'b0;
            result[alloc1_idx] <= 32'b0;
            result2[alloc1_idx] <= 32'b0;
            paddr[alloc1_idx] <= 32'b0;
            vaddr[alloc1_idx] <= 32'b0;
            wstrb[alloc1_idx] <= 4'b0;
            size[alloc1_idx] <= 3'b0;
            uncached[alloc1_idx] <= 1'b0;
            br_taken[alloc1_idx] <= 1'b0;
            br_target[alloc1_idx] <= 32'b0;
            excp_dynamic[alloc1_idx] <= {`EXCP_NUM{1'b0}};

            tail <= tail + 1'b1;
        end

        if (alu0_wb_valid_i) begin
            result[alu0_wb_robid_i] <= alu0_wb_data_i;
            br_taken[alu0_wb_robid_i] <= alu0_wb_br_taken_i;
            br_target[alu0_wb_robid_i] <= alu0_wb_br_target_i;
            complete[alu0_wb_robid_i] <= 1'b1;
        end
        if (alu1_wb_valid_i) begin
            result[alu1_wb_robid_i] <= alu1_wb_data_i;
            br_taken[alu1_wb_robid_i] <= alu1_wb_br_taken_i;
            br_target[alu1_wb_robid_i] <= alu1_wb_br_target_i;
            complete[alu1_wb_robid_i] <= 1'b1;
        end
        if (mem_wb_valid_i) begin
            result[mem_wb_robid_i] <= mem_wb_data_i;
            paddr[mem_wb_robid_i] <= mem_wb_paddr_i;
            vaddr[mem_wb_robid_i] <= mem_wb_vaddr_i;
            wstrb[mem_wb_robid_i] <= mem_wb_wstrb_i;
            size[mem_wb_robid_i] <= mem_wb_size_i;
            uncached[mem_wb_robid_i] <= mem_wb_uncached_i;
            excp_dynamic[mem_wb_robid_i] <= mem_wb_excp_i;
            complete[mem_wb_robid_i] <= 1'b1;
        end
        if (mdu_wb_valid_i) begin
            result[mdu_wb_robid_i] <= mdu_wb_data_i;
            result2[mdu_wb_robid_i] <= mdu_wb_data2_i;
            complete[mdu_wb_robid_i] <= 1'b1;
        end
    end
end

endmodule
