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
// - raddr/rrdy/rdata ×6 ：dispatch 操作数读口
// - base_probe_* ×2 ：LSU 遗漏唤醒恢复读口
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
    input  wire                       alloc_en_i,         // 本拍至少分配一对
    input  wire                       alloc_two_i,        // 第三条占用 tail+1 偶槽
    output wire [`ROB_PAIR_W-1:0]     rob_tail_o,         // 当前队尾对指针
    output wire                       rob_full_o,         // 不能分配一对
    output wire                       rob_full2_o,        // 不能分配连续两对
    output wire                       rob_tail_half_o,    // even[tail] 已占，odd[tail] 为下一空位
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

    input  wire                       a2_valid_i,
    input  wire [31:0]                a2_pc_i,
    input  wire [31:0]                a2_inst_i,
    input  wire                       a2_rf_we_i,
    input  wire [4:0]                 a2_rd_i,
    input  wire                       a2_is_load_i,
    input  wire                       a2_is_store_i,
    input  wire                       a2_is_branch_i,
    input  wire [`BR_TYPE_W-1:0]      a2_br_type_i,
    input  wire                       a2_pred_taken_i,
    input  wire                       a2_is_last_i,
    input  wire [`FTQ_W-1:0]          a2_ftq_id_i,
    input  wire [`PRIV_NUM-1:0]       a2_priv_vec_i,
    input  wire [13:0]                a2_csr_num_i,
    input  wire [`TLB_OP_NUM-1:0]     a2_tlb_op_i,
    input  wire [4:0]                 a2_cacop_code_i,
    input  wire [`EXCP_NUM-1:0]       a2_excp_i,
    input  wire                       a2_is_nop_i,

    // =============== dispatch 操作数读口 ×4（组合，带同拍写回旁路） ===============
    input  wire [`ROB_W-1:0]          raddr0_i,
    output wire                       rrdy0_o,            // 该编号已写回（含本拍正在写回）
    output wire [31:0]                rdata0_o,
    output wire [31:0]                rdata0_noa2_o,
    input  wire [`ROB_W-1:0]          raddr1_i,
    output wire                       rrdy1_o,
    output wire [31:0]                rdata1_o,
    input  wire [`ROB_W-1:0]          raddr2_i,
    output wire                       rrdy2_o,
    output wire [31:0]                rdata2_o,
    output wire [31:0]                rdata2_noa2_o,
    input  wire [`ROB_W-1:0]          raddr3_i,
    output wire                       rrdy3_o,
    output wire [31:0]                rdata3_o,
    input  wire [`ROB_W-1:0]          raddr4_i,
    output wire                       rrdy4_o,
    output wire [31:0]                rdata4_o,
    output wire [31:0]                rdata4_noa2_o,
    input  wire [`ROB_W-1:0]          raddr5_i,
    output wire                       rrdy5_o,
    output wire [31:0]                rdata5_o,

    // =============== LSU base recovery probe ×2 ===============
    input  wire [`ROB_W-1:0]          base_probe0_robid_i,
    output wire                       base_probe0_ready_o,
    output wire [31:0]                base_probe0_data_o,
    input  wire [`ROB_W-1:0]          base_probe1_robid_i,
    output wire                       base_probe1_ready_o,
    output wire [31:0]                base_probe1_data_o,

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
    // ---- fu_alu2 ----
    input  wire                       alu2_wb_valid_i,
    input  wire [`ROB_W-1:0]          alu2_wb_robid_i,
    input  wire [31:0]                alu2_wb_data_i,
    input  wire                       alu2_wb_br_taken_i,
    input  wire [31:0]                alu2_wb_br_target_i,
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
    // The raw memory completion above updates the completion bit and memory
    // metadata immediately.  Load data itself is written through this
    // registered copy, cutting the DCache-tag/data -> wide ROB result-array
    // path.  Read and commit ports bypass the same copy, so the split does
    // not add a retirement or consumer-visible cycle.
    input  wire                       mem_fwd_valid_i,
    input  wire [`ROB_W-1:0]          mem_fwd_robid_i,
    input  wire [31:0]                mem_fwd_data_i,
    input  wire [31:0]                mem_fwd_paddr_i,
    input  wire [31:0]                mem_fwd_vaddr_i,
    input  wire [3:0]                 mem_fwd_wstrb_i,
    input  wire [2:0]                 mem_fwd_size_i,
    input  wire                       mem_fwd_uncached_i,
    input  wire [`EXCP_NUM-1:0]       mem_fwd_excp_i,
    input  wire                       mem2_wb_valid_i,
    input  wire [`ROB_W-1:0]          mem2_wb_robid_i,
    input  wire [31:0]                mem2_wb_data_i,
    input  wire [31:0]                mem2_wb_paddr_i,
    input  wire [31:0]                mem2_wb_vaddr_i,
    input  wire [2:0]                 mem2_wb_size_i,
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
    output wire                       cmt0_has_excp_fast_o,
    output wire                       cmt0_has_priv_fast_o,
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
    output wire                       cmt1_has_excp_fast_o,
    output wire                       cmt1_has_priv_fast_o,

    output wire                       quad_simple_ready_o,
    output wire [`ROB_W-1:0]          cmt2_robid_o,
    output wire                       cmt2_rf_we_o,
    output wire [4:0]                 cmt2_rd_o,
    output wire [31:0]                cmt2_result_o,
    output wire [31:0]                cmt2_pc_o,
    output wire [31:0]                cmt2_inst_o,
    output wire                       cmt2_is_load_o,
    output wire [31:0]                cmt2_paddr_o,
    output wire [31:0]                cmt2_vaddr_o,
    output wire [`ROB_W-1:0]          cmt3_robid_o,
    output wire                       cmt3_rf_we_o,
    output wire [4:0]                 cmt3_rd_o,
    output wire [31:0]                cmt3_result_o,
    output wire [31:0]                cmt3_pc_o,
    output wire [31:0]                cmt3_inst_o,
    output wire                       cmt3_is_load_o,
    output wire [31:0]                cmt3_paddr_o,
    output wire [31:0]                cmt3_vaddr_o,
    input  wire                       cmt_pop2_i,

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
reg                    tail_half;
// Keep same-cycle copies of the commit head next to each static metadata
// lane.  Each copy is intentionally narrow and only feeds one packed static
// read lane, reducing cross-lane address fanout while preserving the original
// architectural pointer everywhere else.
(* keep = "true" *) reg [`ROB_PAIR_W-1:0] head_static_even_q;
(* keep = "true" *) reg [`ROB_PAIR_W-1:0] head_static_odd_q;

// ---------------- 多写口状态和单写口动态 payload banks ----------------
// valid/complete/excp_dynamic 保持触发器实现。宽 payload 按写回源拆分，
// 每个 bank 只有一个写入源；小型 owner 表指示每个 ROB tag 的最新 bank。
reg                       valid [0:`ROB_SIZE-1];
// The ROB id MSB is the static even/odd lane selector.  Keep completion in
// two 16-entry banks so allocation and the six completion writers only
// decode the pair index inside one lane instead of steering one 32-entry
// multi-write array.  Reads below select the lane with the same ROB id MSB;
// no completion state or cycle boundary changes.
reg                       complete_even [0:(`ROB_SIZE/2)-1];
reg                       complete_odd  [0:(`ROB_SIZE/2)-1];
// Raw LSU completion must remain visible at the first post-response edge for
// IPC, but writing the shared multi-port complete array put the full L2/MSHR
// cone behind every FU/allocation priority level.  Keep the one-cycle shadow
// as a registered tag instead of a 32-bit decoded one-hot: consumers compare
// against this short local register and the registered mem_fwd bus installs
// the persistent complete bit on the following edge.
reg                       mem_complete_shadow_valid;
reg [`ROB_W-1:0]          mem_complete_shadow_robid;

localparam [2:0] RESULT_SRC_ALU0 = 3'd0;
localparam [2:0] RESULT_SRC_ALU1 = 3'd1;
localparam [2:0] RESULT_SRC_ALU2 = 3'd2;
localparam [2:0] RESULT_SRC_MEM  = 3'd3;
localparam [2:0] RESULT_SRC_MEM2 = 3'd4;
localparam [2:0] RESULT_SRC_MDU  = 3'd5;
(* ram_style = "distributed" *) reg [31:0] result_alu0 [0:`ROB_SIZE-1];
(* ram_style = "distributed" *) reg [31:0] result_alu1 [0:`ROB_SIZE-1];
(* ram_style = "distributed" *) reg [31:0] result_alu2 [0:`ROB_SIZE-1];
(* ram_style = "distributed" *) reg [31:0] result_mem  [0:`ROB_SIZE-1];
(* ram_style = "distributed" *) reg [31:0] result_mem2 [0:`ROB_SIZE-1];
(* ram_style = "distributed" *) reg [31:0] result_mdu  [0:`ROB_SIZE-1];
reg [2:0]                 result_owner [0:`ROB_SIZE-1];

(* ram_style = "distributed" *) reg [31:0] result2_mdu [0:`ROB_SIZE-1];

localparam MEM_PADDR_LSB = 0;
localparam MEM_VADDR_LSB = MEM_PADDR_LSB + 32;
localparam MEM_WSTRB_LSB = MEM_VADDR_LSB + 32;
localparam MEM_SIZE_LSB  = MEM_WSTRB_LSB + 4;
localparam MEM_UNC_LSB   = MEM_SIZE_LSB + 3;
localparam MEM_PAYLOAD_W = MEM_UNC_LSB + 1;
(* ram_style = "distributed" *) reg [MEM_PAYLOAD_W-1:0]
    mem_payload_fwd  [0:`ROB_SIZE-1];
(* ram_style = "distributed" *) reg [MEM_PAYLOAD_W-1:0]
    mem_payload_mem2 [0:`ROB_SIZE-1];
reg                       mem_payload_owner [0:`ROB_SIZE-1];

localparam BR_TAKEN_LSB = 0;
localparam BR_TARGET_LSB = BR_TAKEN_LSB + 1;
localparam BR_PAYLOAD_W = BR_TARGET_LSB + 32;
localparam [1:0] BR_SRC_ALU0 = 2'd0;
localparam [1:0] BR_SRC_ALU1 = 2'd1;
localparam [1:0] BR_SRC_ALU2 = 2'd2;
localparam [1:0] BR_SRC_ZERO = 2'd3;
(* ram_style = "distributed" *) reg [BR_PAYLOAD_W-1:0]
    br_payload_alu0 [0:`ROB_SIZE-1];
(* ram_style = "distributed" *) reg [BR_PAYLOAD_W-1:0]
    br_payload_alu1 [0:`ROB_SIZE-1];
(* ram_style = "distributed" *) reg [BR_PAYLOAD_W-1:0]
    br_payload_alu2 [0:`ROB_SIZE-1];
reg [1:0]                 br_payload_owner [0:`ROB_SIZE-1];
reg [`EXCP_NUM-1:0]       excp_dynamic [0:`ROB_SIZE-1];
reg                       excp_dynamic_any [0:`ROB_SIZE-1];
reg [`FTQ_W-1:0]          head_ftq0_stay_q;
reg [`FTQ_W-1:0]          head_ftq1_stay_q;
reg [`FTQ_W-1:0]          head_ftq0_advance_q;
reg [`FTQ_W-1:0]          head_ftq1_advance_q;
reg                       head_ftq_advance_sel_q;
wire [`FTQ_W-1:0]         head_ftq0_q = head_ftq_advance_sel_q
                                           ? head_ftq0_advance_q
                                           : head_ftq0_stay_q;
wire [`FTQ_W-1:0]         head_ftq1_q = head_ftq_advance_sel_q
                                           ? head_ftq1_advance_q
                                           : head_ftq1_stay_q;

integer i;
integer ci;

wire [`ROB_PAIR_W-1:0] alloc2_pair = tail + {{(`ROB_PAIR_W-1){1'b0}}, 1'b1};
wire [`ROB_W-1:0] alloc0_idx = tail_half ? {1'b1, tail}
                                         : {1'b0, tail};
wire [`ROB_W-1:0] alloc1_idx = tail_half ? {1'b0, alloc2_pair}
                                         : {1'b1, tail};
wire [`ROB_W-1:0] alloc2_idx = tail_half ? {1'b1, alloc2_pair}
                                         : {1'b0, alloc2_pair};
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
localparam STA_HAS_EXCP_LSB = STA_EXCP_LSB + `EXCP_NUM;
localparam STA_DYN_EXCP_LSB = STA_HAS_EXCP_LSB + 1;
localparam STA_HAS_PRIV_LSB = STA_DYN_EXCP_LSB + 1;
localparam STA_W         = STA_HAS_PRIV_LSB + 1;

// Consecutive allocation pair addresses always have opposite bit 0 values.
// Split each physical lane on that bit so every array has at most one write
// per cycle while retaining asynchronous reads and the original cycle timing.
(* ram_style = "distributed" *) reg [STA_W-1:0] sta_even_p0 [0:`ROB_SIZE/4-1];
(* ram_style = "distributed" *) reg [STA_W-1:0] sta_even_p1 [0:`ROB_SIZE/4-1];
(* ram_style = "distributed" *) reg [STA_W-1:0] sta_odd_p0  [0:`ROB_SIZE/4-1];
(* ram_style = "distributed" *) reg [STA_W-1:0] sta_odd_p1  [0:`ROB_SIZE/4-1];

wire a0_inst_is_b0  = (a0_inst_i == 32'h50000000);
wire a1_inst_is_b0  = (a1_inst_i == 32'h50000000);
wire a0_is_direct_b = (a0_inst_i[31:26] == 6'b010100);
wire a1_is_direct_b = (a1_inst_i[31:26] == 6'b010100);
wire a2_inst_is_b0  = (a2_inst_i == 32'h50000000);
wire a2_is_direct_b = (a2_inst_i[31:26] == 6'b010100);
wire a0_static_excp_any = |a0_excp_i;
wire a1_static_excp_any = |a1_excp_i;
wire a2_static_excp_any = |a2_excp_i;
wire a0_dynamic_excp_enable = a0_is_load_i || a0_is_store_i
                            || a0_priv_vec_i[`PRIV_CACOP];
wire a1_dynamic_excp_enable = a1_is_load_i || a1_is_store_i
                            || a1_priv_vec_i[`PRIV_CACOP];
wire a2_dynamic_excp_enable = a2_is_load_i || a2_is_store_i
                            || a2_priv_vec_i[`PRIV_CACOP];
wire a0_has_priv = |a0_priv_vec_i;
wire a1_has_priv = |a1_priv_vec_i;
wire a2_has_priv = |a2_priv_vec_i;

wire [STA_W-1:0] sta_pack0 = {a0_has_priv, a0_dynamic_excp_enable,
                              a0_static_excp_any, a0_excp_i, a0_cacop_code_i, a0_tlb_op_i, a0_csr_num_i,
                              a0_priv_vec_i, a0_ftq_id_i, a0_is_last_i, a0_pred_taken_i,
                              a0_br_type_i, a0_is_branch_i, a0_is_store_i, a0_is_load_i,
                              a0_rd_i, a0_rf_we_i, a0_is_direct_b, a0_inst_is_b0, a0_pc_i};
wire [STA_W-1:0] sta_pack1 = {a1_has_priv, a1_dynamic_excp_enable,
                              a1_static_excp_any, a1_excp_i, a1_cacop_code_i, a1_tlb_op_i, a1_csr_num_i,
                              a1_priv_vec_i, a1_ftq_id_i, a1_is_last_i, a1_pred_taken_i,
                              a1_br_type_i, a1_is_branch_i, a1_is_store_i, a1_is_load_i,
                              a1_rd_i, a1_rf_we_i, a1_is_direct_b, a1_inst_is_b0, a1_pc_i};
wire [STA_W-1:0] sta_pack2 = {a2_has_priv, a2_dynamic_excp_enable,
                              a2_static_excp_any, a2_excp_i, a2_cacop_code_i, a2_tlb_op_i, a2_csr_num_i,
                              a2_priv_vec_i, a2_ftq_id_i, a2_is_last_i, a2_pred_taken_i,
                              a2_br_type_i, a2_is_branch_i, a2_is_store_i, a2_is_load_i,
                              a2_rd_i, a2_rf_we_i, a2_is_direct_b, a2_inst_is_b0, a2_pc_i};

// Keep independent physical copies of the allocation predicate.  A single
// merged driver previously reached 3500+ static-bank and ROB-state loads.
// Only the two wide static banks need explicit local replication guidance.
(* keep = "true", max_fanout = 128 *) wire alloc_sta_even_fire =
    !reset && !flush_i && alloc_en_i
    && (tail_half
        ? (a2_valid_i ? !rob_full2_o : (!a1_valid_i || !rob_full_o))
        : (a2_valid_i ? !rob_full2_o : !rob_full_o));
(* keep = "true", max_fanout = 128 *) wire alloc_sta_odd_fire =
    !reset && !flush_i && alloc_en_i
    && (tail_half
        ? (a2_valid_i ? !rob_full2_o : (!a1_valid_i || !rob_full_o))
        : (a2_valid_i ? !rob_full2_o : !rob_full_o));
(* keep = "true" *) wire alloc_state_fire =
    !reset && !flush_i && alloc_en_i
    && (tail_half
        ? (a2_valid_i ? !rob_full2_o : (!a1_valid_i || !rob_full_o))
        : (a2_valid_i ? !rob_full2_o : !rob_full_o));
(* keep = "true" *) wire alloc_head_fire =
    !reset && !flush_i && alloc_en_i
    && (tail_half
        ? (a2_valid_i ? !rob_full2_o : (!a1_valid_i || !rob_full_o))
        : (a2_valid_i ? !rob_full2_o : !rob_full_o));

wire even_tail_we = alloc_sta_even_fire && !tail_half;
wire even_next_we = alloc_sta_even_fire &&
                    ((!tail_half && a2_valid_i) || (tail_half && a1_valid_i));
wire [STA_W-1:0] even_next_wdata = tail_half ? sta_pack1 : sta_pack2;
wire even_p0_we = (even_tail_we && !tail[0]) ||
                  (even_next_we && !alloc2_pair[0]);
wire even_p1_we = (even_tail_we && tail[0]) ||
                  (even_next_we && alloc2_pair[0]);
wire [`ROB_PAIR_W-2:0] even_p0_waddr =
    (even_tail_we && !tail[0]) ? tail[`ROB_PAIR_W-1:1] :
                                 alloc2_pair[`ROB_PAIR_W-1:1];
wire [`ROB_PAIR_W-2:0] even_p1_waddr =
    (even_tail_we && tail[0]) ? tail[`ROB_PAIR_W-1:1] :
                                alloc2_pair[`ROB_PAIR_W-1:1];
wire [STA_W-1:0] even_p0_wdata =
    (even_tail_we && !tail[0]) ? sta_pack0 : even_next_wdata;
wire [STA_W-1:0] even_p1_wdata =
    (even_tail_we && tail[0]) ? sta_pack0 : even_next_wdata;

wire odd_tail_we = alloc_sta_odd_fire &&
                   ((!tail_half && a1_valid_i) || tail_half);
wire odd_next_we = alloc_sta_odd_fire && tail_half && a2_valid_i;
wire [STA_W-1:0] odd_tail_wdata = tail_half ? sta_pack0 : sta_pack1;
wire odd_p0_we = (odd_tail_we && !tail[0]) ||
                 (odd_next_we && !alloc2_pair[0]);
wire odd_p1_we = (odd_tail_we && tail[0]) ||
                 (odd_next_we && alloc2_pair[0]);
wire [`ROB_PAIR_W-2:0] odd_p0_waddr =
    (odd_tail_we && !tail[0]) ? tail[`ROB_PAIR_W-1:1] :
                                alloc2_pair[`ROB_PAIR_W-1:1];
wire [`ROB_PAIR_W-2:0] odd_p1_waddr =
    (odd_tail_we && tail[0]) ? tail[`ROB_PAIR_W-1:1] :
                               alloc2_pair[`ROB_PAIR_W-1:1];
wire [STA_W-1:0] odd_p0_wdata =
    (odd_tail_we && !tail[0]) ? odd_tail_wdata : sta_pack2;
wire [STA_W-1:0] odd_p1_wdata =
    (odd_tail_we && tail[0]) ? odd_tail_wdata : sta_pack2;

always @(posedge clk) begin
    if (even_p0_we)
        sta_even_p0[even_p0_waddr] <= even_p0_wdata;
end

always @(posedge clk) begin
    if (even_p1_we)
        sta_even_p1[even_p1_waddr] <= even_p1_wdata;
end

always @(posedge clk) begin
    if (odd_p0_we)
        sta_odd_p0[odd_p0_waddr] <= odd_p0_wdata;
end

always @(posedge clk) begin
    if (odd_p1_we)
        sta_odd_p1[odd_p1_waddr] <= odd_p1_wdata;
end

wire [STA_W-1:0] sta_h0 = head_static_even_q[0]
    ? sta_even_p1[head_static_even_q[`ROB_PAIR_W-1:1]]
    : sta_even_p0[head_static_even_q[`ROB_PAIR_W-1:1]];
wire [STA_W-1:0] sta_h1 = head_static_odd_q[0]
    ? sta_odd_p1[head_static_odd_q[`ROB_PAIR_W-1:1]]
    : sta_odd_p0[head_static_odd_q[`ROB_PAIR_W-1:1]];

// One shared next-head expression drives both the architectural pointer and
// its static-read replica.  Keeping the guard identical is important: when
// an open odd slot is filled while the current pair is being retired, the
// pair is revisited and neither pointer advances on that edge.
wire head_pop_advance = cmt_pop_i &&
                        !(alloc_head_fire && tail_half && (head == tail));
wire [`ROB_PAIR_W-1:0] head_pop_next =
    head + (cmt_pop2_i ? {{(`ROB_PAIR_W-2){1'b0}}, 2'd2}
                        : {{(`ROB_PAIR_W-1){1'b0}}, 1'b1});

// The commit-to-FTQ redirect loop is too long when it begins at the packed
// static RAM's asynchronous head read.  Pre-read both possible next-head
// pairs and register only the late stay/advance decision.  This keeps
// alloc_head_fire out of all eight FTQ-ID data D pins without changing the
// edge at which a newly allocated odd slot becomes visible.
wire head_will_advance = head_pop_advance;
wire [`ROB_PAIR_W-1:0] head_advance_pair =
    head + (cmt_pop2_i ? {{(`ROB_PAIR_W-2){1'b0}}, 2'd2}
                         : {{(`ROB_PAIR_W-1){1'b0}}, 1'b1});
wire [STA_W-1:0] sta_advance0 = head_advance_pair[0]
    ? sta_even_p1[head_advance_pair[`ROB_PAIR_W-1:1]]
    : sta_even_p0[head_advance_pair[`ROB_PAIR_W-1:1]];
wire [STA_W-1:0] sta_advance1 = head_advance_pair[0]
    ? sta_odd_p1[head_advance_pair[`ROB_PAIR_W-1:1]]
    : sta_odd_p0[head_advance_pair[`ROB_PAIR_W-1:1]];
// These are physical slot-availability facts from registered queue state.
// If dispatch does not actually write an available slot, that slot remains
// invalid and its speculative candidate is unobservable.  Full/full2 gates
// prevent a wrapped allocation tag from aliasing a live head entry.
wire ftq_alloc0_slot_available = tail_half || !rob_full_o;
wire ftq_alloc1_slot_available = !rob_full_o;
wire ftq_alloc2_slot_available = !rob_full2_o;

wire stay_ftq0_fwd_a0 = ftq_alloc0_slot_available && !tail_half &&
                         (head == tail);
wire stay_ftq0_fwd_a2 = ftq_alloc2_slot_available && !tail_half && a2_valid_i &&
                         (head == alloc2_pair);
wire stay_ftq0_fwd_a1 = ftq_alloc1_slot_available && tail_half && a1_valid_i &&
                         (head == alloc2_pair);
wire stay_ftq1_fwd_a1 = ftq_alloc1_slot_available && !tail_half && a1_valid_i &&
                         (head == tail);
wire stay_ftq1_fwd_a0 = ftq_alloc0_slot_available && tail_half &&
                         (head == tail);
wire stay_ftq1_fwd_a2 = ftq_alloc2_slot_available && tail_half && a2_valid_i &&
                         (head == alloc2_pair);

wire advance_ftq0_fwd_a0 = ftq_alloc0_slot_available && !tail_half &&
                            (head_advance_pair == tail);
wire advance_ftq0_fwd_a2 = ftq_alloc2_slot_available && !tail_half && a2_valid_i &&
                            (head_advance_pair == alloc2_pair);
wire advance_ftq0_fwd_a1 = ftq_alloc1_slot_available && tail_half && a1_valid_i &&
                            (head_advance_pair == alloc2_pair);
wire advance_ftq1_fwd_a1 = ftq_alloc1_slot_available && !tail_half && a1_valid_i &&
                            (head_advance_pair == tail);
wire advance_ftq1_fwd_a0 = ftq_alloc0_slot_available && tail_half &&
                            (head_advance_pair == tail);
wire advance_ftq1_fwd_a2 = ftq_alloc2_slot_available && tail_half && a2_valid_i &&
                            (head_advance_pair == alloc2_pair);

wire [`FTQ_W-1:0] head_ftq0_stay_next =
    stay_ftq0_fwd_a0 ? a0_ftq_id_i :
    stay_ftq0_fwd_a2 ? a2_ftq_id_i :
    stay_ftq0_fwd_a1 ? a1_ftq_id_i :
    sta_h0[STA_FTQ_LSB +: `FTQ_W];
wire [`FTQ_W-1:0] head_ftq1_stay_next =
    stay_ftq1_fwd_a1 ? a1_ftq_id_i :
    stay_ftq1_fwd_a0 ? a0_ftq_id_i :
    stay_ftq1_fwd_a2 ? a2_ftq_id_i :
    sta_h1[STA_FTQ_LSB +: `FTQ_W];
wire [`FTQ_W-1:0] head_ftq0_advance_next =
    advance_ftq0_fwd_a0 ? a0_ftq_id_i :
    advance_ftq0_fwd_a2 ? a2_ftq_id_i :
    advance_ftq0_fwd_a1 ? a1_ftq_id_i :
    sta_advance0[STA_FTQ_LSB +: `FTQ_W];
wire [`FTQ_W-1:0] head_ftq1_advance_next =
    advance_ftq1_fwd_a1 ? a1_ftq_id_i :
    advance_ftq1_fwd_a0 ? a0_ftq_id_i :
    advance_ftq1_fwd_a2 ? a2_ftq_id_i :
    sta_advance1[STA_FTQ_LSB +: `FTQ_W];

always @(posedge clk) begin
    if (reset || flush_i) begin
        head_ftq0_stay_q <= {`FTQ_W{1'b0}};
        head_ftq1_stay_q <= {`FTQ_W{1'b0}};
        head_ftq0_advance_q <= {`FTQ_W{1'b0}};
        head_ftq1_advance_q <= {`FTQ_W{1'b0}};
        head_ftq_advance_sel_q <= 1'b0;
    end else begin
        head_ftq0_stay_q <= head_ftq0_stay_next;
        head_ftq1_stay_q <= head_ftq1_stay_next;
        head_ftq0_advance_q <= head_ftq0_advance_next;
        head_ftq1_advance_q <= head_ftq1_advance_next;
        head_ftq_advance_sel_q <= head_will_advance;
    end
end

// Static no-execute entries do not need a per-entry completion FF write.
// Derive their readiness only after the saved ROB static row reaches head:
//   1) fetch/decode exception;
//   2) DBAR/IBAR (the commit stage still performs the barrier action);
//   3) no RF/memory/branch/privileged side effect.
// This removes the IFU->decode->ROB complete allocation path and also avoids
// storing is_nop in the static RAM.
wire cmt0_static_excp = sta_h0[STA_HAS_EXCP_LSB];
wire cmt1_static_excp = sta_h1[STA_HAS_EXCP_LSB];
wire cmt0_barrier =
    sta_h0[STA_PRIV_LSB + `PRIV_IBAR];
wire cmt1_barrier =
    sta_h1[STA_PRIV_LSB + `PRIV_IBAR];
wire cmt0_no_side_effect =
    !sta_h0[STA_RFWE_LSB]
    && !sta_h0[STA_ISLD_LSB]
    && !sta_h0[STA_ISST_LSB]
    && !sta_h0[STA_ISBR_LSB]
    && !sta_h0[STA_HAS_PRIV_LSB];
wire cmt1_no_side_effect =
    !sta_h1[STA_RFWE_LSB]
    && !sta_h1[STA_ISLD_LSB]
    && !sta_h1[STA_ISST_LSB]
    && !sta_h1[STA_ISBR_LSB]
    && !sta_h1[STA_HAS_PRIV_LSB];
wire cmt0_static_done =
    cmt0_static_excp || cmt0_barrier || cmt0_no_side_effect;
wire cmt1_static_done =
    cmt1_static_excp || cmt1_barrier || cmt1_no_side_effect;

wire [`ROB_PAIR_W-1:0] quad_npair = head + 1'b1;
wire [`ROB_W-1:0] quad_n0_idx = {1'b0, quad_npair};
wire [`ROB_W-1:0] quad_n1_idx = {1'b1, quad_npair};

// Each view is an asynchronous read of the writer-owned banks followed by a
// narrow owner select.  Keeping the expressions inline avoids simulator
// sensitivity issues previously seen with variable-index array functions.
`define ROB_RESULT_VIEW(N, A) \
    wire [2:0] result_owner_``N = result_owner[A]; \
    wire [31:0] result_view_``N = \
        (result_owner_``N == RESULT_SRC_ALU0) ? result_alu0[A] : \
        (result_owner_``N == RESULT_SRC_ALU1) ? result_alu1[A] : \
        (result_owner_``N == RESULT_SRC_ALU2) ? result_alu2[A] : \
        (result_owner_``N == RESULT_SRC_MEM)  ? result_mem[A]  : \
        (result_owner_``N == RESULT_SRC_MEM2) ? result_mem2[A] : result_mdu[A];
`ROB_RESULT_VIEW(c0, head0_idx)
`ROB_RESULT_VIEW(c1, head1_idx)
`ROB_RESULT_VIEW(q0, quad_n0_idx)
`ROB_RESULT_VIEW(q1, quad_n1_idx)
`ROB_RESULT_VIEW(r0, raddr0_i)
`ROB_RESULT_VIEW(r1, raddr1_i)
`ROB_RESULT_VIEW(r2, raddr2_i)
`ROB_RESULT_VIEW(r3, raddr3_i)
`ROB_RESULT_VIEW(r4, raddr4_i)
`ROB_RESULT_VIEW(r5, raddr5_i)
`ROB_RESULT_VIEW(bp0, base_probe0_robid_i)
`ROB_RESULT_VIEW(bp1, base_probe1_robid_i)
`undef ROB_RESULT_VIEW

`define ROB_MEM_VIEW(N, A) \
    wire [MEM_PAYLOAD_W-1:0] mem_view_``N = mem_payload_owner[A] \
        ? mem_payload_mem2[A] : mem_payload_fwd[A];
`ROB_MEM_VIEW(c0, head0_idx)
`ROB_MEM_VIEW(c1, head1_idx)
`ROB_MEM_VIEW(q0, quad_n0_idx)
`ROB_MEM_VIEW(q1, quad_n1_idx)
`undef ROB_MEM_VIEW

`define ROB_BRANCH_VIEW(N, A) \
    wire [1:0] br_owner_``N = br_payload_owner[A]; \
    wire [BR_PAYLOAD_W-1:0] br_view_``N = \
        (br_owner_``N == BR_SRC_ALU0) ? br_payload_alu0[A] : \
        (br_owner_``N == BR_SRC_ALU1) ? br_payload_alu1[A] : \
        (br_owner_``N == BR_SRC_ALU2) ? br_payload_alu2[A] : \
        {BR_PAYLOAD_W{1'b0}};
`ROB_BRANCH_VIEW(c0, head0_idx)
`ROB_BRANCH_VIEW(c1, head1_idx)
`undef ROB_BRANCH_VIEW

wire [STA_W-1:0] quad_n0_sta = quad_npair[0]
    ? sta_even_p1[quad_npair[`ROB_PAIR_W-1:1]]
    : sta_even_p0[quad_npair[`ROB_PAIR_W-1:1]];
wire [STA_W-1:0] quad_n1_sta = quad_npair[0]
    ? sta_odd_p1[quad_npair[`ROB_PAIR_W-1:1]]
    : sta_odd_p0[quad_npair[`ROB_PAIR_W-1:1]];
wire quad_n0_static_done =
    (|quad_n0_sta[STA_EXCP_LSB +: `EXCP_NUM]) ||
    (!quad_n0_sta[STA_RFWE_LSB] && !quad_n0_sta[STA_ISLD_LSB] &&
     !quad_n0_sta[STA_ISST_LSB] && !quad_n0_sta[STA_ISBR_LSB] &&
     !(|quad_n0_sta[STA_PRIV_LSB +: `PRIV_NUM]));
wire quad_n1_static_done =
    (|quad_n1_sta[STA_EXCP_LSB +: `EXCP_NUM]) ||
    (!quad_n1_sta[STA_RFWE_LSB] && !quad_n1_sta[STA_ISLD_LSB] &&
     !quad_n1_sta[STA_ISST_LSB] && !quad_n1_sta[STA_ISBR_LSB] &&
     !(|quad_n1_sta[STA_PRIV_LSB +: `PRIV_NUM]));
wire quad_n0_complete = quad_n0_idx[`ROB_W-1]
                      ? complete_odd[quad_n0_idx[`ROB_PAIR_W-1:0]]
                      : complete_even[quad_n0_idx[`ROB_PAIR_W-1:0]];
wire quad_n1_complete = quad_n1_idx[`ROB_W-1]
                      ? complete_odd[quad_n1_idx[`ROB_PAIR_W-1:0]]
                      : complete_even[quad_n1_idx[`ROB_PAIR_W-1:0]];
wire quad_n0_ready = valid[quad_n0_idx] &&
                     (quad_n0_complete
                      || (mem_complete_shadow_valid &&
                          (mem_complete_shadow_robid == quad_n0_idx))
                      || quad_n0_static_done);
wire quad_n1_ready = valid[quad_n1_idx] &&
                     (quad_n1_complete
                      || (mem_complete_shadow_valid &&
                          (mem_complete_shadow_robid == quad_n1_idx))
                      || quad_n1_static_done);
wire quad_n0_mem_fwd_hit = mem_fwd_valid_i && (mem_fwd_robid_i == quad_n0_idx);
wire quad_n1_mem_fwd_hit = mem_fwd_valid_i && (mem_fwd_robid_i == quad_n1_idx);
wire quad_n0_is_mem = quad_n0_sta[STA_ISLD_LSB] || quad_n0_sta[STA_ISST_LSB]
                   || quad_n0_sta[STA_PRIV_LSB + `PRIV_CACOP];
wire quad_n1_is_mem = quad_n1_sta[STA_ISLD_LSB] || quad_n1_sta[STA_ISST_LSB]
                   || quad_n1_sta[STA_PRIV_LSB + `PRIV_CACOP];
wire [`EXCP_NUM-1:0] quad_n0_excp_eff = {`EXCP_NUM{quad_n0_is_mem}} &
    (excp_dynamic[quad_n0_idx] |
     ({`EXCP_NUM{quad_n0_mem_fwd_hit}} & mem_fwd_excp_i));
wire [`EXCP_NUM-1:0] quad_n1_excp_eff = {`EXCP_NUM{quad_n1_is_mem}} &
    (excp_dynamic[quad_n1_idx] |
     ({`EXCP_NUM{quad_n1_mem_fwd_hit}} & mem_fwd_excp_i));
wire quad_n0_simple = quad_n0_ready &&
                      !quad_n0_sta[STA_ISST_LSB] && !quad_n0_sta[STA_ISBR_LSB] &&
                      !(|quad_n0_sta[STA_PRIV_LSB +: `PRIV_NUM]) &&
                      !(|quad_n0_sta[STA_EXCP_LSB +: `EXCP_NUM]) &&
                      !(|quad_n0_excp_eff) &&
                      !quad_n0_sta[STA_LAST_LSB];
wire quad_n1_simple = quad_n1_ready &&
                      !quad_n1_sta[STA_ISST_LSB] && !quad_n1_sta[STA_ISBR_LSB] &&
                      !(|quad_n1_sta[STA_PRIV_LSB +: `PRIV_NUM]) &&
                      !(|quad_n1_sta[STA_EXCP_LSB +: `EXCP_NUM]) &&
                      !(|quad_n1_excp_eff) &&
                      !quad_n1_sta[STA_LAST_LSB];
assign quad_simple_ready_o = quad_n0_simple && quad_n1_simple;
assign cmt2_robid_o = quad_n0_idx;
assign cmt2_rf_we_o = quad_n0_sta[STA_RFWE_LSB];
assign cmt2_rd_o = quad_n0_sta[STA_RD_LSB +: 5];
assign cmt2_result_o = quad_n0_mem_fwd_hit ? mem_fwd_data_i : result_view_q0;
assign cmt2_pc_o = quad_n0_sta[STA_PC_LSB +: 32];
assign cmt2_is_load_o = quad_n0_sta[STA_ISLD_LSB];
assign cmt2_paddr_o = quad_n0_mem_fwd_hit ? mem_fwd_paddr_i
                                         : mem_view_q0[MEM_PADDR_LSB +: 32];
assign cmt2_vaddr_o = quad_n0_mem_fwd_hit ? mem_fwd_vaddr_i
                                         : mem_view_q0[MEM_VADDR_LSB +: 32];
assign cmt3_robid_o = quad_n1_idx;
assign cmt3_rf_we_o = quad_n1_sta[STA_RFWE_LSB];
assign cmt3_rd_o = quad_n1_sta[STA_RD_LSB +: 5];
assign cmt3_result_o = quad_n1_mem_fwd_hit ? mem_fwd_data_i : result_view_q1;
assign cmt3_pc_o = quad_n1_sta[STA_PC_LSB +: 32];
assign cmt3_is_load_o = quad_n1_sta[STA_ISLD_LSB];
assign cmt3_paddr_o = quad_n1_mem_fwd_hit ? mem_fwd_paddr_i
                                         : mem_view_q1[MEM_PADDR_LSB +: 32];
assign cmt3_vaddr_o = quad_n1_mem_fwd_hit ? mem_fwd_vaddr_i
                                         : mem_view_q1[MEM_VADDR_LSB +: 32];

`ifdef SYNTHESIS
// synthesis translate_off
// Opportunity probe for a pair-aligned four-retire fast path.  It observes
// the pair after the current commit pair and deliberately separates raw
// readiness from the subset with no branch/memory/privileged/FTQ side effect.
wire [`ROB_PAIR_W-1:0] probe_npair = head + 1'b1;
wire [`ROB_W-1:0] probe_n0_idx = {1'b0, probe_npair};
wire [`ROB_W-1:0] probe_n1_idx = {1'b1, probe_npair};
wire [STA_W-1:0] probe_n0_sta = quad_n0_sta;
wire [STA_W-1:0] probe_n1_sta = quad_n1_sta;
wire probe_n0_static_done =
    (|probe_n0_sta[STA_EXCP_LSB +: `EXCP_NUM]) ||
    (!probe_n0_sta[STA_RFWE_LSB] && !probe_n0_sta[STA_ISLD_LSB] &&
     !probe_n0_sta[STA_ISST_LSB] && !probe_n0_sta[STA_ISBR_LSB] &&
     !(|probe_n0_sta[STA_PRIV_LSB +: `PRIV_NUM]));
wire probe_n1_static_done =
    (|probe_n1_sta[STA_EXCP_LSB +: `EXCP_NUM]) ||
    (!probe_n1_sta[STA_RFWE_LSB] && !probe_n1_sta[STA_ISLD_LSB] &&
     !probe_n1_sta[STA_ISST_LSB] && !probe_n1_sta[STA_ISBR_LSB] &&
     !(|probe_n1_sta[STA_PRIV_LSB +: `PRIV_NUM]));
wire probe_n0_complete = probe_n0_idx[`ROB_W-1]
                       ? complete_odd[probe_n0_idx[`ROB_PAIR_W-1:0]]
                       : complete_even[probe_n0_idx[`ROB_PAIR_W-1:0]];
wire probe_n1_complete = probe_n1_idx[`ROB_W-1]
                       ? complete_odd[probe_n1_idx[`ROB_PAIR_W-1:0]]
                       : complete_even[probe_n1_idx[`ROB_PAIR_W-1:0]];
wire probe_n0_ready = valid[probe_n0_idx] &&
                      (probe_n0_complete
                       || (mem_complete_shadow_valid &&
                           (mem_complete_shadow_robid == probe_n0_idx))
                       || probe_n0_static_done);
wire probe_n1_ready = valid[probe_n1_idx] &&
                      (probe_n1_complete
                       || (mem_complete_shadow_valid &&
                           (mem_complete_shadow_robid == probe_n1_idx))
                       || probe_n1_static_done);
wire probe_n0_simple = probe_n0_ready &&
                       !probe_n0_sta[STA_ISST_LSB] && !probe_n0_sta[STA_ISBR_LSB] &&
                       !(|probe_n0_sta[STA_PRIV_LSB +: `PRIV_NUM]) &&
                       !(|probe_n0_sta[STA_EXCP_LSB +: `EXCP_NUM]) &&
                       !((probe_n0_sta[STA_ISLD_LSB] || probe_n0_sta[STA_ISST_LSB]
                          || probe_n0_sta[STA_PRIV_LSB + `PRIV_CACOP]) &&
                         (|excp_dynamic[probe_n0_idx])) &&
                       !probe_n0_sta[STA_LAST_LSB];
wire probe_n1_simple = probe_n1_ready &&
                       !probe_n1_sta[STA_ISST_LSB] && !probe_n1_sta[STA_ISBR_LSB] &&
                       !(|probe_n1_sta[STA_PRIV_LSB +: `PRIV_NUM]) &&
                       !(|probe_n1_sta[STA_EXCP_LSB +: `EXCP_NUM]) &&
                       !((probe_n1_sta[STA_ISLD_LSB] || probe_n1_sta[STA_ISST_LSB]
                          || probe_n1_sta[STA_PRIV_LSB + `PRIV_CACOP]) &&
                         (|excp_dynamic[probe_n1_idx])) &&
                       !probe_n1_sta[STA_LAST_LSB];
reg [63:0] rob_quad_ready_opp;
reg [63:0] rob_quad_simple_opp;
always @(posedge clk) begin
    if (reset) begin
        rob_quad_ready_opp <= 64'd0;
        rob_quad_simple_opp <= 64'd0;
    end else if (cmt_clear0_i && cmt_clear1_i) begin
        if (probe_n0_ready && probe_n1_ready)
            rob_quad_ready_opp <= rob_quad_ready_opp + 64'd1;
        if (probe_n0_simple && probe_n1_simple)
            rob_quad_simple_opp <= rob_quad_simple_opp + 64'd1;
    end
end
// synthesis translate_on
`endif

assign rob_tail_o = tail;
assign rob_tail_half_o = tail_half;
// 满判据必须按 ROB_PAIR_W 位宽环形加：`ROB_GUARD` 是无宽度十进制字面量，
// 若写成 (head == tail+ROB_GUARD)，右边被扩成 32 位（如 11+5=16），与 4 位
// head 比较永远对不上 wrap 后的 0，SB 阻塞 head 时 tail 会绕回覆盖尚未提交项
// （linux pagetable_init：ae24@0080 被 OVW → digftest 看见 @00c0）。
wire [`ROB_PAIR_W-1:0] rob_full_mark = tail + `ROB_GUARD;
assign rob_full_o = (head == rob_full_mark);
wire [`ROB_PAIR_W-1:0] rob_full2_mark = tail + `ROB_GUARD + 1'b1;
assign rob_full2_o = rob_full_o || (head == rob_full2_mark);
assign rob_empty_o = (head == tail) && !tail_half;
assign head_robid0_o = head0_idx;

// 读口 ready/data【不得】用 valid 门控：
// RAT 的 busy/tag 相对提交晚一拍——生产者提交当拍，消费者 rename 仍拿到
// "busy=1, tag=生产者" 的旧视图，下一拍 dispatch 读 ROB 时表项已 pop。
// 若此处再检查 valid，该操作数永远等不到唤醒，直到 robid 被新指令复用后
// 捕获错误数据（ABA）。complete/result 在 pop 后保留、重新分配时清除，
// 故 pop 后一拍窗口内仍可安全读出正确值。
// 注意：读口使用每口内联组合——xsim 对 continuous assign 中带可变下标的
// 子程序存在求值/敏感表缺陷，会返回上一次求值下标的旧值
// （表现为 raddr=13 却读出 result[9]）。内联后按 raddr 直接索引，杜绝此问题。
`define ROB_RDPORT(P) \
    wire wbhit``P = (alu0_wb_valid_i && (alu0_wb_robid_i == raddr``P``_i)) || \
                    (alu1_wb_valid_i && (alu1_wb_robid_i == raddr``P``_i)) || \
                    (alu2_wb_valid_i && (alu2_wb_robid_i == raddr``P``_i)) || \
                    (mem_fwd_valid_i && (mem_fwd_robid_i == raddr``P``_i)) || \
                    (mem2_wb_valid_i && (mem2_wb_robid_i == raddr``P``_i)) || \
                    (mdu_wb_valid_i  && (mdu_wb_robid_i  == raddr``P``_i)); \
    wire [31:0] wbdat``P = (alu0_wb_valid_i && (alu0_wb_robid_i == raddr``P``_i)) ? alu0_wb_data_i : \
                           (alu1_wb_valid_i && (alu1_wb_robid_i == raddr``P``_i)) ? alu1_wb_data_i : \
                           (alu2_wb_valid_i && (alu2_wb_robid_i == raddr``P``_i)) ? alu2_wb_data_i : \
                           (mem_fwd_valid_i && (mem_fwd_robid_i == raddr``P``_i)) ? mem_fwd_data_i : \
                           (mem2_wb_valid_i && (mem2_wb_robid_i == raddr``P``_i)) ? mem2_wb_data_i : \
                           (mdu_wb_valid_i  && (mdu_wb_robid_i  == raddr``P``_i)) ? mdu_wb_data_i  : 32'b0; \
    wire mem_shadow_hit``P = mem_complete_shadow_valid && \
                              (mem_complete_shadow_robid == raddr``P``_i); \
    wire complete_hit``P = raddr``P``_i[`ROB_W-1] \
                         ? complete_odd[raddr``P``_i[`ROB_PAIR_W-1:0]] \
                         : complete_even[raddr``P``_i[`ROB_PAIR_W-1:0]]; \
    assign rrdy``P``_o  = complete_hit``P | mem_shadow_hit``P | wbhit``P; \
    assign rdata``P``_o = wbhit``P ? wbdat``P : result_view_r``P;
`ROB_RDPORT(0)
`ROB_RDPORT(1)
`ROB_RDPORT(2)
`ROB_RDPORT(3)
`ROB_RDPORT(4)
`ROB_RDPORT(5)
`undef ROB_RDPORT

// LSU recovery probes preserve the former top-level hierarchical-read
// behavior exactly.  They intentionally include only the two memory forward
// buses in the same-cycle data bypass; ALU/MDU completion remains visible on
// the following cycle, as before.
`define ROB_BASE_PROBE(P) \
    wire base_probe_complete``P = base_probe``P``_robid_i[`ROB_W-1] \
        ? complete_odd[base_probe``P``_robid_i[`ROB_PAIR_W-1:0]] \
        : complete_even[base_probe``P``_robid_i[`ROB_PAIR_W-1:0]]; \
    wire base_probe_shadow``P = mem_complete_shadow_valid && \
        (mem_complete_shadow_robid == base_probe``P``_robid_i); \
    wire base_probe_mem_fwd``P = mem_fwd_valid_i && \
        (mem_fwd_robid_i == base_probe``P``_robid_i); \
    wire base_probe_mem2_fwd``P = mem2_wb_valid_i && \
        (mem2_wb_robid_i == base_probe``P``_robid_i); \
    assign base_probe``P``_ready_o = base_probe_complete``P \
        || base_probe_shadow``P || base_probe_mem_fwd``P \
        || base_probe_mem2_fwd``P; \
    assign base_probe``P``_data_o = base_probe_mem_fwd``P \
        ? mem_fwd_data_i : base_probe_mem2_fwd``P \
        ? mem2_wb_data_i : result_view_bp``P;
`ROB_BASE_PROBE(0)
`ROB_BASE_PROBE(1)
`undef ROB_BASE_PROBE

// Pair-key timing copies exist only for the three src0 read ports.  They omit
// all same-cycle ALU data branches.  RS_MEM restores ALU0/ALU1/ALU2 through
// three priority-matched direct adders, keeping ALU carry chains out of this
// generic ROB-read path.
`define ROB_NOA2_RDPORT(P) \
    wire wbhit_noa2_``P = (mem_fwd_valid_i && (mem_fwd_robid_i == raddr``P``_i)) || \
                          (mem2_wb_valid_i && (mem2_wb_robid_i == raddr``P``_i)) || \
                          (mdu_wb_valid_i  && (mdu_wb_robid_i  == raddr``P``_i)); \
    wire [31:0] wbdat_noa2_``P = \
        (mem_fwd_valid_i && (mem_fwd_robid_i == raddr``P``_i)) ? mem_fwd_data_i : \
        (mem2_wb_valid_i && (mem2_wb_robid_i == raddr``P``_i)) ? mem2_wb_data_i : \
        (mdu_wb_valid_i  && (mdu_wb_robid_i  == raddr``P``_i)) ? mdu_wb_data_i : 32'b0; \
    assign rdata``P``_noa2_o = wbhit_noa2_``P \
                              ? wbdat_noa2_``P : result_view_r``P;
`ROB_NOA2_RDPORT(0)
`ROB_NOA2_RDPORT(2)
`ROB_NOA2_RDPORT(4)
`undef ROB_NOA2_RDPORT

assign cmt0_valid_o = valid[head0_idx];
assign cmt0_complete_o = complete_even[head]
                       || (mem_complete_shadow_valid &&
                           (mem_complete_shadow_robid == head0_idx))
                       || cmt0_static_done;
assign cmt0_pc_o = sta_h0[STA_PC_LSB +: 32];
assign cmt0_inst_is_b0_o = sta_h0[STA_B0_LSB];
assign cmt0_is_direct_b_o = sta_h0[STA_DIRB_LSB];
assign cmt0_rf_we_o = sta_h0[STA_RFWE_LSB];
assign cmt0_rd_o = sta_h0[STA_RD_LSB +: 5];
wire cmt0_mem_fwd_hit = mem_fwd_valid_i && (mem_fwd_robid_i == head0_idx);
wire cmt1_mem_fwd_hit = mem_fwd_valid_i && (mem_fwd_robid_i == head1_idx);
assign cmt0_result_o = cmt0_mem_fwd_hit ? mem_fwd_data_i : result_view_c0;
assign cmt0_result2_o = result2_mdu[head0_idx];
assign cmt0_is_load_o = sta_h0[STA_ISLD_LSB];
assign cmt0_is_store_o = sta_h0[STA_ISST_LSB];
assign cmt0_paddr_o = mem_view_c0[MEM_PADDR_LSB +: 32];
assign cmt0_vaddr_o = mem_view_c0[MEM_VADDR_LSB +: 32];
assign cmt0_wstrb_o = mem_view_c0[MEM_WSTRB_LSB +: 4];
assign cmt0_size_o = mem_view_c0[MEM_SIZE_LSB +: 3];
assign cmt0_uncached_o = mem_view_c0[MEM_UNC_LSB];
assign cmt0_is_branch_o = sta_h0[STA_ISBR_LSB];
assign cmt0_br_type_o = sta_h0[STA_BRT_LSB +: `BR_TYPE_W];
assign cmt0_pred_taken_o = sta_h0[STA_PRDT_LSB];
assign cmt0_br_taken_o = br_view_c0[BR_TAKEN_LSB];
assign cmt0_br_target_o = br_view_c0[BR_TARGET_LSB +: 32];
assign cmt0_is_last_o = sta_h0[STA_LAST_LSB];
assign cmt0_ftq_id_o = head_ftq0_q;
assign cmt0_priv_vec_o = sta_h0[STA_PRIV_LSB +: `PRIV_NUM];
assign cmt0_csr_num_o = sta_h0[STA_CSRN_LSB +: 14];
assign cmt0_tlb_op_o = sta_h0[STA_TLBOP_LSB +: `TLB_OP_NUM];
assign cmt0_cacop_code_o = sta_h0[STA_CACOP_LSB +: 5];
assign cmt0_excp_o = sta_h0[STA_EXCP_LSB +: `EXCP_NUM] |
    ({`EXCP_NUM{sta_h0[STA_ISLD_LSB] || sta_h0[STA_ISST_LSB]
                || sta_h0[STA_PRIV_LSB + `PRIV_CACOP]}} &
     excp_dynamic[head0_idx]);
assign cmt0_has_excp_fast_o = sta_h0[STA_HAS_EXCP_LSB] |
    (sta_h0[STA_DYN_EXCP_LSB] && excp_dynamic_any[head0_idx]);
assign cmt0_has_priv_fast_o = sta_h0[STA_HAS_PRIV_LSB];

assign cmt1_valid_o = valid[head1_idx];
assign cmt1_complete_o = complete_odd[head]
                       || (mem_complete_shadow_valid &&
                           (mem_complete_shadow_robid == head1_idx))
                       || cmt1_static_done;
assign cmt1_pc_o = sta_h1[STA_PC_LSB +: 32];
assign cmt1_inst_is_b0_o = sta_h1[STA_B0_LSB];
assign cmt1_is_direct_b_o = sta_h1[STA_DIRB_LSB];
assign cmt1_rf_we_o = sta_h1[STA_RFWE_LSB];
assign cmt1_rd_o = sta_h1[STA_RD_LSB +: 5];
assign cmt1_result_o = cmt1_mem_fwd_hit ? mem_fwd_data_i : result_view_c1;
assign cmt1_result2_o = result2_mdu[head1_idx];
assign cmt1_is_load_o = sta_h1[STA_ISLD_LSB];
assign cmt1_is_store_o = sta_h1[STA_ISST_LSB];
assign cmt1_paddr_o = mem_view_c1[MEM_PADDR_LSB +: 32];
assign cmt1_vaddr_o = mem_view_c1[MEM_VADDR_LSB +: 32];
assign cmt1_wstrb_o = mem_view_c1[MEM_WSTRB_LSB +: 4];
assign cmt1_size_o = mem_view_c1[MEM_SIZE_LSB +: 3];
assign cmt1_uncached_o = mem_view_c1[MEM_UNC_LSB];
assign cmt1_is_branch_o = sta_h1[STA_ISBR_LSB];
assign cmt1_br_type_o = sta_h1[STA_BRT_LSB +: `BR_TYPE_W];
assign cmt1_pred_taken_o = sta_h1[STA_PRDT_LSB];
assign cmt1_br_taken_o = br_view_c1[BR_TAKEN_LSB];
assign cmt1_br_target_o = br_view_c1[BR_TARGET_LSB +: 32];
assign cmt1_is_last_o = sta_h1[STA_LAST_LSB];
assign cmt1_ftq_id_o = head_ftq1_q;
assign cmt1_priv_vec_o = sta_h1[STA_PRIV_LSB +: `PRIV_NUM];
assign cmt1_csr_num_o = sta_h1[STA_CSRN_LSB +: 14];
assign cmt1_tlb_op_o = sta_h1[STA_TLBOP_LSB +: `TLB_OP_NUM];
assign cmt1_cacop_code_o = sta_h1[STA_CACOP_LSB +: 5];
assign cmt1_excp_o = sta_h1[STA_EXCP_LSB +: `EXCP_NUM] |
    ({`EXCP_NUM{sta_h1[STA_ISLD_LSB] || sta_h1[STA_ISST_LSB]
                || sta_h1[STA_PRIV_LSB + `PRIV_CACOP]}} &
     excp_dynamic[head1_idx]);
assign cmt1_has_excp_fast_o = sta_h1[STA_HAS_EXCP_LSB] |
    (sta_h1[STA_DYN_EXCP_LSB] && excp_dynamic_any[head1_idx]);
assign cmt1_has_priv_fast_o = sta_h1[STA_HAS_PRIV_LSB];

// inst 32b 仅仿真视图保留（difftest/调试观测）；综合视图恒 0，
// 提交侧真实用途（idle `b 0` 判定 / 槽1 直接 B 判定）已由预译码位承担。
`ifdef SIMU
reg [31:0] inst [0:`ROB_SIZE-1];
always @(posedge clk) begin
    if (alloc_state_fire) begin
        inst[alloc0_idx] <= a0_inst_i;
        if (a1_valid_i)
            inst[alloc1_idx] <= a1_inst_i;
        if (a2_valid_i)
            inst[alloc2_idx] <= a2_inst_i;
    end
end
assign cmt0_inst_o = inst[head0_idx];
assign cmt1_inst_o = inst[head1_idx];
assign cmt2_inst_o = inst[quad_n0_idx];
assign cmt3_inst_o = inst[quad_n1_idx];
`else
assign cmt0_inst_o = 32'b0;
assign cmt1_inst_o = 32'b0;
assign cmt2_inst_o = 32'b0;
assign cmt3_inst_o = 32'b0;
`endif

// Wide payload storage has no architectural reset requirement: valid and
// complete gate every consumer, and a reused tag keeps its retired value until
// the matching writer installs a new owner.  Blocking writes during reset or
// flush matches the original enclosing sequential block.
always @(posedge clk) begin
    if (!reset && !flush_i) begin
        if (alu0_wb_valid_i) begin
            result_alu0[alu0_wb_robid_i] <= alu0_wb_data_i;
            br_payload_alu0[alu0_wb_robid_i] <=
                {alu0_wb_br_target_i, alu0_wb_br_taken_i};
        end
        if (alu1_wb_valid_i) begin
            result_alu1[alu1_wb_robid_i] <= alu1_wb_data_i;
            br_payload_alu1[alu1_wb_robid_i] <=
                {alu1_wb_br_target_i, alu1_wb_br_taken_i};
        end
        if (alu2_wb_valid_i) begin
            result_alu2[alu2_wb_robid_i] <= alu2_wb_data_i;
            br_payload_alu2[alu2_wb_robid_i] <=
                {alu2_wb_br_target_i, alu2_wb_br_taken_i};
        end
        if (mem_fwd_valid_i) begin
            result_mem[mem_fwd_robid_i] <= mem_fwd_data_i;
            mem_payload_fwd[mem_fwd_robid_i] <=
                {mem_fwd_uncached_i, mem_fwd_size_i, mem_fwd_wstrb_i,
                 mem_fwd_vaddr_i, mem_fwd_paddr_i};
        end
        if (mem2_wb_valid_i) begin
            result_mem2[mem2_wb_robid_i] <= mem2_wb_data_i;
            mem_payload_mem2[mem2_wb_robid_i] <=
                {1'b0, mem2_wb_size_i, 4'b0,
                 mem2_wb_vaddr_i, mem2_wb_paddr_i};
        end
        if (mdu_wb_valid_i) begin
            result_mdu[mdu_wb_robid_i] <= mdu_wb_data_i;
            result2_mdu[mdu_wb_robid_i] <= mdu_wb_data2_i;
        end
    end
end

always @(posedge clk) begin
    if (reset || flush_i) begin
        head <= {`ROB_PAIR_W{1'b0}};
        head_static_even_q <= {`ROB_PAIR_W{1'b0}};
        head_static_odd_q <= {`ROB_PAIR_W{1'b0}};
        tail <= {`ROB_PAIR_W{1'b0}};
        tail_half <= 1'b0;
        mem_complete_shadow_valid <= 1'b0;
        mem_complete_shadow_robid <= {`ROB_W{1'b0}};
        for (i = 0; i < `ROB_SIZE; i = i + 1) begin
            valid[i] <= 1'b0;
            excp_dynamic[i] <= {`EXCP_NUM{1'b0}};
            excp_dynamic_any[i] <= 1'b0;
        end
        for (ci = 0; ci < (`ROB_SIZE/2); ci = ci + 1) begin
            complete_even[ci] <= 1'b0;
            complete_odd[ci]  <= 1'b0;
        end
    end else begin
        // A new raw completion replaces the prior one-cycle pulse.  At this
        // same edge mem_fwd installs the prior pulse into complete[].
        mem_complete_shadow_valid <= mem_wb_valid_i;
        mem_complete_shadow_robid <= mem_wb_robid_i;
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
        if (cmt_pop2_i) begin
            valid[quad_n0_idx] <= 1'b0;
            valid[quad_n1_idx] <= 1'b0;
        end
        if (cmt_pop_i) begin
            // Do not skip a younger instruction that fills an open odd slot
            // in this same cycle.  The already-retired even valid is cleared,
            // and the pair is revisited next cycle for its new odd entry.
            if (head_pop_advance) begin
                head <= head_pop_next;
                head_static_even_q <= head_pop_next;
                head_static_odd_q <= head_pop_next;
            end
            // A serializing final instruction may leave no younger allocation
            // to fill the odd slot.  Seal that partial pair as it retires.
            if (!alloc_head_fire && tail_half && (head == tail)) begin
                tail <= tail + 1'b1;
                tail_half <= 1'b0;
            end
        end

        // Pre-clear only tags that the registered head/tail/full state proves
        // are allocable.  The odd half of a partial tail is always available;
        // the following slots respect the same one-pair guard used by dispatch.
        // ALU writeback appears later in this block and wins any impossible
        // same-tag collision, matching the old allocation-before-WB priority.
        if (tail_half || !rob_full_o)
            br_payload_owner[alloc0_idx] <= BR_SRC_ZERO;
        if (!rob_full_o)
            br_payload_owner[alloc1_idx] <= BR_SRC_ZERO;
        if (!rob_full2_o)
            br_payload_owner[alloc2_idx] <= BR_SRC_ZERO;

        if (alloc_state_fire) begin
            // Static fields are written to the pair-index-banked LUTRAMs.
            valid[alloc0_idx] <= a0_valid_i;
            // Static no-execute completion is derived from sta_h* only at
            // the ROB head.  Allocation therefore has no decode->complete
            // timing dependency and always clears a reused dynamic bit.
            if (alloc0_idx[`ROB_W-1])
                complete_odd[alloc0_idx[`ROB_PAIR_W-1:0]] <= 1'b0;
            else
                complete_even[alloc0_idx[`ROB_PAIR_W-1:0]] <= 1'b0;
            // Wide dynamic payload fields are don't-care until the matching
            // FU sets complete and writes them.  Retaining their old value
            // removes three allocation-side write ports from every array;
            // the ROB guard still preserves a retired producer for delayed
            // dispatch reads until its tag can safely be reused.

            if (a1_valid_i) begin
                valid[alloc1_idx] <= 1'b1;
                if (alloc1_idx[`ROB_W-1])
                    complete_odd[alloc1_idx[`ROB_PAIR_W-1:0]] <= 1'b0;
                else
                    complete_even[alloc1_idx[`ROB_PAIR_W-1:0]] <= 1'b0;
            end

            if (a2_valid_i) begin
                valid[alloc2_idx] <= 1'b1;
                if (alloc2_idx[`ROB_W-1])
                    complete_odd[alloc2_idx[`ROB_PAIR_W-1:0]] <= 1'b0;
                else
                    complete_even[alloc2_idx[`ROB_PAIR_W-1:0]] <= 1'b0;

            end

            // Advance a consecutive next-free pointer by the accepted prefix
            // length.  One and three entries toggle half alignment; two keep
            // it.  No invalid hole is consumed or exposed to commit.
            if (a2_valid_i) begin
                tail <= tail + (tail_half ? 2'd2 : 1'd1);
                tail_half <= !tail_half;
            end else if (a1_valid_i) begin
                tail <= tail + 1'b1;
                tail_half <= tail_half;
            end else begin
                tail <= tail + (tail_half ? 1'd1 : 1'd0);
                tail_half <= !tail_half;
            end
        end

        if (alu0_wb_valid_i) begin
            result_owner[alu0_wb_robid_i] <= RESULT_SRC_ALU0;
            br_payload_owner[alu0_wb_robid_i] <= BR_SRC_ALU0;
            if (alu0_wb_robid_i[`ROB_W-1])
                complete_odd[alu0_wb_robid_i[`ROB_PAIR_W-1:0]] <= 1'b1;
            else
                complete_even[alu0_wb_robid_i[`ROB_PAIR_W-1:0]] <= 1'b1;
        end
        if (alu1_wb_valid_i) begin
            result_owner[alu1_wb_robid_i] <= RESULT_SRC_ALU1;
            br_payload_owner[alu1_wb_robid_i] <= BR_SRC_ALU1;
            if (alu1_wb_robid_i[`ROB_W-1])
                complete_odd[alu1_wb_robid_i[`ROB_PAIR_W-1:0]] <= 1'b1;
            else
                complete_even[alu1_wb_robid_i[`ROB_PAIR_W-1:0]] <= 1'b1;
        end
        if (alu2_wb_valid_i) begin
            result_owner[alu2_wb_robid_i] <= RESULT_SRC_ALU2;
            br_payload_owner[alu2_wb_robid_i] <= BR_SRC_ALU2;
            if (alu2_wb_robid_i[`ROB_W-1])
                complete_odd[alu2_wb_robid_i[`ROB_PAIR_W-1:0]] <= 1'b1;
            else
                complete_even[alu2_wb_robid_i[`ROB_PAIR_W-1:0]] <= 1'b1;
        end
        // The registered LSU forward bus is aligned with the raw completion
        // captured one cycle earlier.  Commit and ROB read ports bypass it
        // above while this persistent copy is installed.
        if (mem_fwd_valid_i) begin
            if (mem_fwd_robid_i[`ROB_W-1])
                complete_odd[mem_fwd_robid_i[`ROB_PAIR_W-1:0]] <= 1'b1;
            else
                complete_even[mem_fwd_robid_i[`ROB_PAIR_W-1:0]] <= 1'b1;
            result_owner[mem_fwd_robid_i] <= RESULT_SRC_MEM;
            mem_payload_owner[mem_fwd_robid_i] <= 1'b0;
            excp_dynamic[mem_fwd_robid_i] <= mem_fwd_excp_i;
            excp_dynamic_any[mem_fwd_robid_i] <= |mem_fwd_excp_i;
        end
        if (mem2_wb_valid_i) begin
            result_owner[mem2_wb_robid_i] <= RESULT_SRC_MEM2;
            mem_payload_owner[mem2_wb_robid_i] <= 1'b1;
            excp_dynamic[mem2_wb_robid_i] <= {`EXCP_NUM{1'b0}};
            excp_dynamic_any[mem2_wb_robid_i] <= 1'b0;
            if (mem2_wb_robid_i[`ROB_W-1])
                complete_odd[mem2_wb_robid_i[`ROB_PAIR_W-1:0]] <= 1'b1;
            else
                complete_even[mem2_wb_robid_i[`ROB_PAIR_W-1:0]] <= 1'b1;
        end
        if (mdu_wb_valid_i) begin
            result_owner[mdu_wb_robid_i] <= RESULT_SRC_MDU;
            if (mdu_wb_robid_i[`ROB_W-1])
                complete_odd[mdu_wb_robid_i[`ROB_PAIR_W-1:0]] <= 1'b1;
            else
                complete_even[mdu_wb_robid_i[`ROB_PAIR_W-1:0]] <= 1'b1;
        end
    end
end

endmodule
