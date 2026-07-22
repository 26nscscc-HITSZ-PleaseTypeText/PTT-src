// ============================================================
// commit 模块（提交级：双提交仲裁 + 异常/特权/误预测处理 + 训练/冲刷源头）
// ------------------------------------------------------------
// 功能（全核"唯一让体系结构状态改变"的地方）：
// - 每拍检查 ROB 队头一对指令，按规则提交 0/1/2 条：
//   * 写 ARF（×2 写口）+ 释放 RAT 占用
//   * store -> 推入 store buffer（提交后写）
//   * 分支 -> 与预测比对，误预测则发起 FLUSH_MISPRED 冲刷
//   * 异常/中断 -> 经 csr_exception_commit_handler 触发入口跳转（FLUSH_EXCP）
//   * 特权类（csr写/ertn/tlb维护/cacop/ll/sc/ibar/idle）-> 单提交 +
//     落地各自效果 + FLUSH_REFETCH/ERTN 冲刷（"遇事不决清空流水线"）
//   * 向 FTQ 回送训练信息、维护 RAS 提交栈
// - 双提交规则（槽 0 永远先于槽 1，槽 1 提交以槽 0 成功提交为前提）。
//
// 端口分组：
// - cmt0/cmt1_*  ：ROB 队头一对（rob.v 提交口直连）
// - rob_pop/clear：提交推进
// - arf_* / rat_*：写回与释放
// - sb_*         ：store 入队
// - csr_*        ：对接 csr_exception_commit_handler 的提交接口
// - tlb/cacop_*  ：TLB 维护与 cache 维护落地
// - ftq_* / ras_*：BPU 训练与 RAS 提交栈
// - flush_*      ：冲刷请求（送 ctrl）
// - debug0/1_*   ：调试/difftest 对齐接口
// ============================================================
`include "mycpu.h"

module commit(
    input  wire                       clk,
    input  wire                       reset,

    // 100MHz 攻坚:全局 flush 已在 ctrl 打一拍。flush 生效的那一拍(广播拍),
    // 提交级必须闸住一切退休,否则错误路径的下一条指令会在流水被清空的同拍误提交。
    // 接 ctrl 的寄存器版 flush(=顶层 flush)。
    input  wire                       flush_pending_i,

    // =============== ROB 队头一对（rob.v 提交口直连） ===============
    input  wire [`ROB_W-1:0]          head_robid0_i,
    // ---- 槽 0 ----
    input  wire                       cmt0_valid_i,
    input  wire                       cmt0_complete_i,
    input  wire [31:0]                cmt0_pc_i,
    input  wire [31:0]                cmt0_inst_i,        // 仅 debug/difftest 观测（综合视图恒 0）
    input  wire                       cmt0_inst_is_b0_i,  // ROB 预译码：inst==0x50000000
    input  wire                       cmt0_is_direct_b_i, // ROB 预译码：inst[31:26]==010100
    input  wire                       cmt0_rf_we_i,
    input  wire [4:0]                 cmt0_rd_i,
    input  wire [31:0]                cmt0_result_i,
    input  wire [31:0]                cmt0_result2_i,
    input  wire                       cmt0_is_load_i,
    input  wire                       cmt0_is_store_i,
    input  wire [31:0]                cmt0_paddr_i,
    input  wire [31:0]                cmt0_vaddr_i,
    input  wire [3:0]                 cmt0_wstrb_i,
    input  wire [2:0]                 cmt0_size_i,
    input  wire                       cmt0_uncached_i,
    input  wire                       cmt0_is_branch_i,
    input  wire [`BR_TYPE_W-1:0]      cmt0_br_type_i,
    input  wire                       cmt0_pred_taken_i,
    input  wire                       cmt0_br_taken_i,
    input  wire [31:0]                cmt0_br_target_i,
    input  wire                       cmt0_is_last_i,
    input  wire [`FTQ_W-1:0]          cmt0_ftq_id_i,
    input  wire [`PRIV_NUM-1:0]       cmt0_priv_vec_i,
    input  wire [13:0]                cmt0_csr_num_i,
    input  wire [`TLB_OP_NUM-1:0]     cmt0_tlb_op_i,
    input  wire [4:0]                 cmt0_cacop_code_i,
    input  wire [`EXCP_NUM-1:0]       cmt0_excp_i,
    // ---- 槽 1 ----
    input  wire                       cmt1_valid_i,
    input  wire                       cmt1_complete_i,
    input  wire [31:0]                cmt1_pc_i,
    input  wire [31:0]                cmt1_inst_i,        // 仅 debug/difftest 观测（综合视图恒 0）
    input  wire                       cmt1_inst_is_b0_i,
    input  wire                       cmt1_is_direct_b_i,
    input  wire                       cmt1_rf_we_i,
    input  wire [4:0]                 cmt1_rd_i,
    input  wire [31:0]                cmt1_result_i,
    input  wire [31:0]                cmt1_result2_i,
    input  wire                       cmt1_is_load_i,
    input  wire                       cmt1_is_store_i,
    input  wire [31:0]                cmt1_paddr_i,
    input  wire [31:0]                cmt1_vaddr_i,
    input  wire [3:0]                 cmt1_wstrb_i,
    input  wire [2:0]                 cmt1_size_i,
    input  wire                       cmt1_uncached_i,
    input  wire                       cmt1_is_branch_i,
    input  wire [`BR_TYPE_W-1:0]      cmt1_br_type_i,
    input  wire                       cmt1_pred_taken_i,
    input  wire                       cmt1_br_taken_i,
    input  wire [31:0]                cmt1_br_target_i,
    input  wire                       cmt1_is_last_i,
    input  wire [`FTQ_W-1:0]          cmt1_ftq_id_i,
    input  wire [`PRIV_NUM-1:0]       cmt1_priv_vec_i,
    input  wire [13:0]                cmt1_csr_num_i,
    input  wire [`TLB_OP_NUM-1:0]     cmt1_tlb_op_i,
    input  wire [4:0]                 cmt1_cacop_code_i,
    input  wire [`EXCP_NUM-1:0]       cmt1_excp_i,

    // ---- 提交推进 ----
    output wire                       rob_pop_o,
    output wire                       rob_clear0_o,
    output wire                       rob_clear1_o,

    // =============== ARF 写口 ×2 ===============
    output wire                       arf_we0_o,
    output wire [4:0]                 arf_waddr0_o,
    output wire [31:0]                arf_wdata0_o,
    output wire                       arf_we1_o,
    output wire [4:0]                 arf_waddr1_o,
    output wire [31:0]                arf_wdata1_o,

    // =============== RAT 释放口 ×2 ===============
    output wire                       rat_cmt_en0_o,
    output wire [4:0]                 rat_cmt_addr0_o,
    output wire [`ROB_W-1:0]          rat_cmt_num0_o,
    output wire                       rat_cmt_en1_o,
    output wire [4:0]                 rat_cmt_addr1_o,
    output wire [`ROB_W-1:0]          rat_cmt_num1_o,

    // =============== store buffer：提交入队（单写口） ===============
    output wire                       sb_push_valid_o,
    output wire [31:0]                sb_push_paddr_o,
    output wire [31:0]                sb_push_data_o,
    output wire [3:0]                 sb_push_wstrb_o,
    output wire [2:0]                 sb_push_size_o,
    output wire                       sb_push_uncached_o,
    input  wire                       sb_full_i,
    input  wire                       sb_empty_i,          // 已提交写排空（ibar/cacop）

    // =============== CSR / 异常提交（对接 csr_exception_commit_handler） ===============
    output wire                       csr_cmt_valid_o,     // 本拍有指令提交（接 wb_valid）
    output wire [31:0]                csr_cmt_pc_o,        // 提交 PC（异常 ERA）
    output wire                       csr_cmt_ex_o,        // 异常提交（接 wb_ex）
    output wire                       csr_cmt_ertn_o,      // ertn 提交
    output wire [31:0]                csr_cmt_vaddr_o,     // BADV（取指异常=PC，访存异常=访存 vaddr）
    output wire                       excp_int_o,          // 各异常类型 valid（对接 handler 分立端口）
    output wire                       excp_adef_o,
    output wire                       excp_adem_o,
    output wire                       excp_ipe_o,
    output wire                       excp_ale_o,
    output wire                       excp_sys_o,
    output wire                       excp_brk_o,
    output wire                       excp_ine_o,
    output wire [`TLB_EX_NUM-1:0]     excp_tlb_vec_o,      // TLB 异常向量（取指/访存合并格式）
    output wire                       csr_we_o,            // CSR 写提交（csrwr/csrxchg）
    output wire [13:0]                csr_wnum_o,
    output wire [31:0]                csr_wmask_o,         // csrxchg 已在 MDU 合成新值 -> 全 1
    output wire [31:0]                csr_wvalue_o,        // = result2
    output wire                       ll_set_o,            // ll.w 提交置 LLBIT
    output wire                       sc_set_o,            // sc.w 提交清 LLBIT
    output wire [27:0]                lladdr_o,            // LL 地址高位
    input  wire                       has_int_i,           // CSR 有待处理中断
    input  wire [31:0]                csr_next_pc_i,       // 异常入口/ERA（handler 算好）
    input  wire [1:0]                 csr_redirect_i,      // `CSR_REDIRECT_EX / _ERTN

    // =============== TLB 维护落地（对接 tlb_manager / csr handler） ===============
    output wire [`TLB_OP_NUM-1:0]     tlb_op_cmt_o,        // 提交拍有效一拍
    output wire [9:0]                 invtlb_asid_o,       // 由 result2 解包
    output wire [18:0]                invtlb_vpn_o,

    // =============== cache 维护落地（直接驱动 I$/D$ cacop 口） ===============
    output wire                       icacop_valid_o,
    output wire [1:0]                 icacop_op_o,         // cacop_code[4:3]
    output wire [31:0]                icacop_addr_o,       // ROB paddr
    output wire                       dcacop_valid_o,
    output wire [1:0]                 dcacop_op_o,
    output wire [31:0]                dcacop_addr_o,

    // =============== FTQ 训练 / 查询 ===============
    output wire                       ftq_cmt_valid_o,     // 本拍有指令提交（推进 cmt_ptr/产生训练）
    output wire [`FTQ_W-1:0]          ftq_cmt_id_o,
    output wire                       ftq_cmt_is_last_o,
    output wire [1:0]                 ftq_cmt_release_o,   // 本拍释放 FTQ 块数（0/1/2）
    output wire                       ftq_cmt_is_branch_o,
    output wire                       ftq_cmt_taken_o,
    output wire                       ftq_cmt_mispred_o,
    output wire [31:0]                ftq_cmt_target_o,    // 实际目标
    output wire [`BR_TYPE_W-1:0]      ftq_cmt_br_type_o,
    output wire [31:0]                ftq_cmt_pc_o,        // 分支指令 PC
    output wire [`FTQ_W-1:0]          ftq_query_id_o,      // 查该块的预测目标（误预测比对用）
    input  wire [31:0]                ftq_blk_target_i,    // FTQ 返回的预测目标

    // =============== RAS 提交栈维护（连 bpu） ===============
    output wire                       ras_cmt_call_o,
    output wire                       ras_cmt_ret_o,
    output wire [31:0]                ras_cmt_retaddr_o,   // call 的 PC+4

    // =============== 冲刷请求（送 ctrl） ===============
    output wire                       flush_req_o,
    output wire [`FLUSH_TYPE_W-1:0]   flush_type_o,
    output wire [31:0]                flush_pc_o,
    output wire                       idle_commit_o,       // idle 提交（ctrl 进入睡眠）

    // =============== LSU 状态 ===============
    input  wire                       uncached_ld_inflight_i, // 在飞 uncached load（屏蔽中断）

    // =============== 调试 / difftest 提交口 ===============
    output wire                       debug0_valid_o,
    output wire [31:0]                debug0_pc_o,
    output wire [3:0]                 debug0_rf_wen_o,
    output wire [4:0]                 debug0_rf_wnum_o,
    output wire [31:0]                debug0_rf_wdata_o,
    output wire [31:0]                debug0_inst_o,
    output wire                       debug1_valid_o,
    output wire [31:0]                debug1_pc_o,
    output wire [3:0]                 debug1_rf_wen_o,
    output wire [4:0]                 debug1_rf_wnum_o,
    output wire [31:0]                debug1_rf_wdata_o,
    output wire [31:0]                debug1_inst_o
);

// 设计说明（已实现，参考 mariver commit.v 的 can0/can1/flush 体系 +
//      团队赛报告 2.4.7 节的单/双提交规则。这是全核逻辑最"杂"的模块，
//      但每条规则都很直白，逐条写即可）
//
// 第一步——基础可提交判定：
//      can0 = cmt0_valid_i && cmt0_complete_i;
//      can1 = can0提交成功 && cmt1_valid_i && cmt1_complete_i && 槽1无单提交限制;
//      （槽 0 无效但槽 1 有效的情况：恒成对分配下槽 0 无效=空泡，跳过即可，
//        此时 can1 仅需自身 complete。）
//
// 第二步——中断附着（最高优先级，在一切提交动作之前判断）：
//      int_take = has_int_i && cmt0_valid_i && !uncached_ld_inflight_i;
//      命中时：槽 0 指令不提交效果（不写 ARF/SB），按 INT 异常走异常流程，
//      ERA=cmt0_pc_i，flush_type=FLUSH_EXCP，flush_pc=csr_next_pc_i。
//      （uncached load 在飞时禁止附着：外设读副作用已发出，必须等它提交。）
//
// 第三步——槽 0 异常检查：
//      cmt0_excp_i 非 0（按 EXCP_* 优先级取最高位）：不提交效果，
//      向 csr handler 发对应异常 valid（excp_*_o 端口逐位翻译；
//      取指侧 TLB 异常翻 TLB_EX_TLBR/PIF/PPI，访存侧翻 TLBR/PIL/PIS/PPI/PME，
//      合成 excp_tlb_vec_o）；csr_cmt_vaddr_o：取指类=pc，访存类=cmt0_vaddr_i；
//      flush_type=FLUSH_EXCP，flush_pc=csr_next_pc_i（handler 组合给出
//      EENTRY/TLBRENTRY）。同拍 rob_clear0+pop（该指令"以异常方式退休"）。
//
// 第四步——槽 0 特权类（priv_vec 非 0 且无异常）：单提交 + 落地 + 冲刷：
//      PRIV_CSR_WR：csr_we_o=1（wnum/wvalue=result2/wmask=~0），FLUSH_REFETCH pc+4
//      PRIV_ERTN  ：csr_cmt_ertn_o=1，FLUSH_ERTN，flush_pc=csr_next_pc_i（ERA）
//      PRIV_TLB   ：tlb_op_cmt_o = cmt0_tlb_op_i（ROB 静态区已存，直通发一拍）；
//                   invtlb_asid/vpn 从 result2 解包；FLUSH_REFETCH pc+4
//      PRIV_CACOP ：cacop_code[2:0]==0 -> icacop_*；==1 -> dcacop_*（op=code[4:3]）
//                   与 ibar 一样等 sb_empty（D$ 前端 cacop 优先于 SB 写，不排空
//                   会把尚未写回 D$ 的 dirty 老数据刷出，n78 Hit-WB 会挂）；
//                   FLUSH_REFETCH pc+4
//      PRIV_LL    ：ll_set_o=1，lladdr_o=cmt0_paddr_i[31:4]；FLUSH_REFETCH pc+4
//      PRIV_SC    ：sc_set_o=1（清 LLBIT）；若译码期定性为真 store 则照常入 SB；
//                   FLUSH_REFETCH pc+4
//      PRIV_IBAR  ：等 sb_empty_i 才提交（不空则本拍不提交，下拍重试）；
//                   FLUSH_REFETCH pc+4
//      PRIV_IDLE  ：idle_commit_o=1 + FLUSH_REFETCH pc+4（ctrl 冻结取指等中断）
//
// 第五步——槽 0 分支误预测检查（无异常无特权时）：
//      ftq_query_id_o = cmt0_ftq_id_i（组合查 FTQ 存的预测目标）；
//      mispred =  (cmt0_br_taken_i != cmt0_pred_taken_i)
//              || (cmt0_br_taken_i && cmt0_pred_taken_i
//                  && cmt0_br_target_i != ftq_blk_target_i)     // 方向对但目标错
//              || (!cmt0_is_branch_i && cmt0_pred_taken_i);     // FTB 脏项把非分支预测成跳
//      命中：本条正常提交（写 ARF），但 flush_type=FLUSH_MISPRED，
//      flush_pc = br_taken ? br_target : pc+4；本拍只提交槽 0。
//
// 第六步——槽 1 的单提交限制（出现任一情况槽 1 留到下一拍）：
//      槽1 有异常 / 槽1 priv / 槽1 CALL·RET·jirl 或误预测 /
//      槽0 也是分支（同拍双分支仍禁，单训练口）/
//      槽0 冲刷 / 两槽都是 store（SB 单写口）。
//      槽1 为 COND 或直接 B、与槽0 同 ftq_id、且预测正确时，允许与非分支槽0 双提
//      （复用现有单路 FTQ 查询口比对目标）。
//      【jirl 为何单提，非 ISA 硬禁】见下方 cmt1_is_direct_b 注释；日后若要给
//      jirl 双提，需独立类型/更严间接目标校验，勿再仅靠 BR_TYPE_UNCOND。
//      未来改进：FTQ 加第二读口后，可放开「不同 ftq_id」的槽1 正确分支双提。
//
// 第七步——常规提交动作（can0/can1 各自独立做）：
//      写 ARF：arf_we=rf_we && rd!=0，wdata=result；
//      释放 RAT：rat_cmt_en=同 arf_we，num=对应 robid
//                （robid0={1'b0,head}，robid1={1'b1,head}，head 取自 head_robid0_i）；
//      store 入 SB：is_store && !uncached 限制无（uncached store 也走 SB），
//                   sb_full_i 时该 store 槽不能提交（本拍整体停）；
//      FTQ 训练：每提交一条发 ftq_cmt_valid（双提交时两条都要送——FTQ 训练口
//                一拍一条，槽 1 的训练信息建议寄存一拍错峰送出，或约定每拍只
//                训练一条分支（槽1 分支已单提交，天然满足））；
//      RAS 提交栈：br_type==CALL -> ras_cmt_call（retaddr=pc+4）；RET -> ras_cmt_ret。
//
// 第八步——rob_pop/clear 生成：
//      rob_clear0_o=本拍槽0提交（含异常退休）；rob_clear1_o=槽1 提交；
//      rob_pop_o=队头一对全部处理完（两槽都 clear 过/无效）——注意槽 1 延迟
//      提交的场景：槽 0 先 clear，下一拍槽 1 提交后才 pop。
//
// 第九步——debug/difftest：
//      debug0_* = 槽 0 提交信息（valid 仅在真正提交效果时置位）；
//      debug1_* = 槽 1。chiplab 的 debug0_wb_* 只接槽 0；双提交核用 DIFFTEST
//      校验两个口（顶层已按此连线）。
//
// 坑点提示：
//      1. 冲刷请求 flush_req_o 必须严格一拍脉冲，且发出当拍 ROB 的 clear/pop
//         也要同拍完成（flush 会清 ROB，时序上别留半提交状态）。
//      2. 异常指令"不写 ARF 不入 SB"，但 debug/difftest 的异常事件要报
//         （ExcpEvent），否则 NEMU 对不上。
//      3. 双提交两条都写同一寄存器（WAW）：ARF 两写口同地址时槽 1 优先
//         （regfile.v 内部已有约定，见其写口说明）。
//      4. mispred 但 br_taken 与 pred 一致、目标也一致 -> 不冲刷（正确预测），
//         这正是 BPU 命中的收益路径，确保比较逻辑别写反。

wire [`ROB_W-1:0] cmt1_robid = {1'b1, head_robid0_i[`ROB_PAIR_W-1:0]};

// flush 广播拍(flush_pending_i=1)闸住全部退休:cmt*_ready/int_take 三个退休根
// 全部置 0 → arf/csr 写、rob pop/clear、flush_req、ftq 训练、sb push 全随之为 0。
// 这样错误路径指令在流水被清空的同拍不会误提交;同时 cmt_flush_req 回 0 保证 flush 单拍。
assign cmt0_ready = !flush_pending_i && cmt0_valid_i && (cmt0_complete_i === 1'b1);
assign cmt1_ready = !flush_pending_i && cmt1_valid_i && (cmt1_complete_i === 1'b1);
wire cmt0_has_excp = |cmt0_excp_i;
wire cmt1_has_excp = |cmt1_excp_i;
wire cmt0_has_priv = |cmt0_priv_vec_i;
wire cmt1_has_priv = |cmt1_priv_vec_i;

// L0 CSR 写免 FLUSH_REFETCH（Phase A1/A2）。rename 侧已对 L0 串行排空，
// 保证提交时 ROB 中无更年轻指令，故不会有在途 csrrd 读到旧 CSR。
wire cmt0_csr_nofush = cmt0_priv_vec_i[`PRIV_CSR_WR]
                    && `CSR_NUM_IS_L0_NOFLUSH(cmt0_csr_num_i[11:0]);
wire cmt1_csr_nofush = cmt1_priv_vec_i[`PRIV_CSR_WR]
                    && `CSR_NUM_IS_L0_NOFLUSH(cmt1_csr_num_i[11:0]);

// 真 idle（非套件 `b 0`/0x50000000）：即使已有 has_int 也先退休完成，
// FLUSH 到 pc+4；中断挂到后继 nop。这样 ERA/s4/NEMU 一致为 idle+4。
// 若在 idle 上 int_take，NEMU（尤其 tcfg InitVal=0）会把 ERA 记成 idle 自身。
// inst 比对已由 ROB 分配拍预译码为 inst_is_b0（提交侧不再读 32b inst）。
wire cmt0_true_idle = cmt0_priv_vec_i[`PRIV_IDLE] && !cmt0_inst_is_b0_i;
wire int_take = !flush_pending_i && has_int_i && cmt0_valid_i && !uncached_ld_inflight_i
                && !cmt0_true_idle;
wire cmt0_ibar_block = cmt0_ready && cmt0_priv_vec_i[`PRIV_IBAR] && !sb_empty_i;
wire cmt1_ibar_block = cmt1_ready && cmt1_priv_vec_i[`PRIV_IBAR] && !sb_empty_i;
// D$/I$ cacop 提交前排空 SB：否则 store 还在 SB、cacop 已按旧脏行写回（n78）。
wire cmt0_cacop_block = cmt0_ready && cmt0_priv_vec_i[`PRIV_CACOP] && !sb_empty_i;
wire cmt1_cacop_block = cmt1_ready && cmt1_priv_vec_i[`PRIV_CACOP] && !sb_empty_i;
// store 提交入 SB：SB 满时该 store 槽不能提交；单写口时槽1 与槽0 双 store 串行
wire cmt0_store_push = cmt0_ready && cmt0_is_store_i && !cmt0_has_excp;
wire cmt1_store_push = cmt1_ready && cmt1_is_store_i && !cmt1_has_excp;
wire cmt0_store_block = cmt0_store_push && sb_full_i;
wire cmt1_store_block = cmt1_store_push && (sb_full_i || (cmt0_effect && cmt0_is_store_i));

wire cmt0_mispred = cmt0_ready && !cmt0_has_excp && !cmt0_has_priv &&
                    ((cmt0_br_taken_i != cmt0_pred_taken_i) ||
                     (cmt0_br_taken_i && cmt0_pred_taken_i &&
                      (cmt0_br_target_i != ftq_blk_target_i)) ||
                     (!cmt0_is_branch_i && cmt0_pred_taken_i));
// 槽1 mispred：头槽路径（query 落在槽1）与「同 ftq_id 双提」共用本路目标。
// 不同 id 时不能信本路目标做双提放行——见下 cmt1_same_ftq。
wire cmt1_mispred = cmt1_ready && !cmt1_has_excp && !cmt1_has_priv &&
                    ((cmt1_br_taken_i != cmt1_pred_taken_i) ||
                     (cmt1_br_taken_i && cmt1_pred_taken_i &&
                      (cmt1_br_target_i != ftq_blk_target_i)) ||
                     (!cmt1_is_branch_i && cmt1_pred_taken_i));
wire cmt1_mispred_head = (!cmt0_valid_i) && cmt1_mispred;

// 同块才复用单路 FTQ 目标；跨块槽1 分支/脏预测仍单提，留给下拍头槽。
// 未来：FTQ 第二读口后可去掉 cmt1_same_ftq 门槛，支持不同 id 双提。
wire cmt1_same_ftq = cmt0_valid_i && (cmt0_ftq_id_i == cmt1_ftq_id_i);

// soft 双提：COND，或真正的直接 B（opc=010100）。jirl 必须单提（标记点）：
//   1) decoder 把非 CALL/RET 的 jirl 也标成 BR_TYPE_UNCOND，与直接 B 共用类型；
//      若只看 br_type 会把间接跳误放进 soft。
//   2) jirl 目标 = rj+offs，依赖寄存器；现有「方向 + FTQ 目标相等」在同块单口
//      比对下，间接跳更易“碰巧判对”而不冲刷，错路径可残留（曾卡在异常向量）。
//   3) jirl 的 CALL/RET 变体还要动 RAS，本就约定单提；普通 jirl 与它们仅编码之差。
//   4) IFU 对 B/cond 有块末截断不变式，jirl 不在同一套预译码截断路径里。
// 以后若放开：ROB 独立 is_jirl / 间接目标专用校验，再允许 soft，勿仅靠 UNCOND。
// 直接 B 判定已由 ROB 分配拍预译码（cmt1_is_direct_b_i），提交侧不读 32b inst。
wire cmt1_br_soft = cmt1_is_branch_i &&
                    ((cmt1_br_type_i == `BR_TYPE_COND) ||
                     ((cmt1_br_type_i == `BR_TYPE_UNCOND) && cmt1_is_direct_b_i));
wire cmt1_br_ok   = cmt1_br_soft && cmt1_same_ftq && !cmt1_mispred;
wire cmt1_br_hard = cmt1_is_branch_i && !cmt1_br_ok;

wire cmt0_retire = int_take ||
                   (cmt0_ready && (cmt0_has_excp ||
                    (!cmt0_store_block && !cmt0_ibar_block && !cmt0_cacop_block)));
wire cmt0_effect = cmt0_ready && !int_take && !cmt0_has_excp &&
                   !cmt0_store_block && !cmt0_ibar_block && !cmt0_cacop_block;
wire cmt0_flush = int_take || (cmt0_ready && cmt0_has_excp) ||
                  (cmt0_effect && cmt0_has_priv && !cmt0_csr_nofush) || cmt0_mispred;

wire cmt1_head_retire = (!cmt0_valid_i) &&
                        (cmt1_ready && (cmt1_has_excp ||
                         (!cmt1_store_block && !cmt1_ibar_block && !cmt1_cacop_block)));
wire cmt1_head_effect = (!cmt0_valid_i) && cmt1_ready && !cmt1_has_excp &&
                        !cmt1_store_block && !cmt1_ibar_block && !cmt1_cacop_block;
wire cmt1_head_flush = (!cmt0_valid_i) &&
                       ((cmt1_ready && cmt1_has_excp) ||
                        (cmt1_head_effect && cmt1_has_priv && !cmt1_csr_nofush) ||
                        cmt1_mispred_head);

wire cmt0_taken_br = cmt0_effect && cmt0_is_branch_i && cmt0_br_taken_i && !cmt0_is_last_i;

// cmt1_br_hard：CALL/RET、跨块分支、或误预测；cmt1_mispred：含脏预测（非分支 pred）
// cmt0_is_branch：同拍双分支仍禁（单 FTQ 训练口）
// 双 store：SB 单写口，槽1 经 cmt1_store_block 串行
wire cmt1_single_limit = cmt1_has_excp || cmt1_has_priv || cmt1_br_hard || cmt1_mispred ||
                         cmt0_is_branch_i || cmt0_taken_br;
wire cmt1_dual_effect = cmt0_effect && !cmt0_flush && cmt1_ready &&
                        !cmt1_single_limit && !cmt1_store_block &&
                        !cmt1_ibar_block && !cmt1_cacop_block;
wire cmt1_retire = cmt0_valid_i ? cmt1_dual_effect : cmt1_head_retire;
wire cmt1_effect = cmt0_valid_i ? cmt1_dual_effect : cmt1_head_effect;

wire take_slot1_for_csr = !cmt0_retire && cmt1_head_retire;
// 训练选源：双提的槽1 分支，或头槽仅槽1（仅 FTQ/RAS/误预测路径用）
wire br_sel_cmt1 = (cmt1_dual_effect && cmt1_is_branch_i) || take_slot1_for_csr;
// CSR/异常 ERA 只用「头槽/中断附着」槽的 PC，不能跟 br_sel 绑到双提槽1 分支 PC
wire [31:0] csr_sel_pc = take_slot1_for_csr ? cmt1_pc_i : cmt0_pc_i;
wire [31:0] sel_pc = br_sel_cmt1 ? cmt1_pc_i : cmt0_pc_i;
wire sel_inst_is_b0 = take_slot1_for_csr ? cmt1_inst_is_b0_i : cmt0_inst_is_b0_i;
wire [31:0] sel_result = take_slot1_for_csr ? cmt1_result_i : cmt0_result_i;
wire [31:0] sel_result2 = take_slot1_for_csr ? cmt1_result2_i : cmt0_result2_i;
wire [31:0] sel_paddr = take_slot1_for_csr ? cmt1_paddr_i : cmt0_paddr_i;
wire [31:0] sel_vaddr = take_slot1_for_csr ? cmt1_vaddr_i : cmt0_vaddr_i;
wire [4:0]  sel_cacop_code = take_slot1_for_csr ? cmt1_cacop_code_i : cmt0_cacop_code_i;
wire [13:0] sel_csr_num = take_slot1_for_csr ? cmt1_csr_num_i : cmt0_csr_num_i;
wire [`TLB_OP_NUM-1:0] sel_tlb_op = take_slot1_for_csr ? cmt1_tlb_op_i : cmt0_tlb_op_i;
wire [`PRIV_NUM-1:0] sel_priv = take_slot1_for_csr ? cmt1_priv_vec_i : cmt0_priv_vec_i;
wire [`EXCP_NUM-1:0] sel_excp = take_slot1_for_csr ? cmt1_excp_i : cmt0_excp_i;
wire sel_is_branch = br_sel_cmt1 ? cmt1_is_branch_i : cmt0_is_branch_i;
wire sel_pred_taken = br_sel_cmt1 ? cmt1_pred_taken_i : cmt0_pred_taken_i;
wire sel_br_taken = br_sel_cmt1 ? cmt1_br_taken_i : cmt0_br_taken_i;
wire [31:0] sel_br_target = br_sel_cmt1 ? cmt1_br_target_i : cmt0_br_target_i;
wire [`BR_TYPE_W-1:0] sel_br_type = br_sel_cmt1 ? cmt1_br_type_i : cmt0_br_type_i;
wire [`FTQ_W-1:0] sel_ftq_id = br_sel_cmt1 ? cmt1_ftq_id_i : cmt0_ftq_id_i;
wire sel_is_last = take_slot1_for_csr ? cmt1_is_last_i : cmt0_is_last_i;

wire selected_effect = take_slot1_for_csr ? cmt1_head_effect : cmt0_effect;
wire selected_excp_take = int_take || (take_slot1_for_csr ? (cmt1_ready && cmt1_has_excp)
                                                          : (cmt0_ready && cmt0_has_excp));
wire selected_csr_nofush = take_slot1_for_csr ? cmt1_csr_nofush : cmt0_csr_nofush;
wire selected_priv_flush = selected_effect && (|sel_priv) && !selected_csr_nofush;
wire selected_mispred = take_slot1_for_csr ? cmt1_mispred_head : cmt0_mispred;

wire mem_excp = sel_excp[`EXCP_ADEM] | sel_excp[`EXCP_ALE] |
                sel_excp[`EXCP_TLBR_M] | sel_excp[`EXCP_PIL] |
                sel_excp[`EXCP_PIS] | sel_excp[`EXCP_PPI_M] |
                sel_excp[`EXCP_PME];

assign rob_clear0_o = cmt0_retire;
assign rob_clear1_o = cmt1_retire;
assign rob_pop_o = (cmt0_valid_i || cmt1_valid_i) &&
                   ((cmt0_valid_i === 1'b1) ? rob_clear0_o : 1'b1) &&
                   ((cmt1_valid_i === 1'b1) ? rob_clear1_o : 1'b1);

assign arf_we0_o = cmt0_effect && cmt0_rf_we_i && (cmt0_rd_i != 5'b0);
assign arf_waddr0_o = cmt0_rd_i;
// sc.w 成功走 LSU，ROB result 是 store data（SB 仍用它入队）；ARF/差分要写回 1。
// 失败走 ALU 且 result=0，is_store=0，保持 result。
assign arf_wdata0_o = (cmt0_effect && cmt0_priv_vec_i[`PRIV_SC] && cmt0_is_store_i)
                      ? 32'd1 : cmt0_result_i;
assign arf_we1_o = cmt1_effect && cmt1_rf_we_i && (cmt1_rd_i != 5'b0);
assign arf_waddr1_o = cmt1_rd_i;
assign arf_wdata1_o = (cmt1_effect && cmt1_priv_vec_i[`PRIV_SC] && cmt1_is_store_i)
                      ? 32'd1 : cmt1_result_i;

assign rat_cmt_en0_o = arf_we0_o;
assign rat_cmt_addr0_o = cmt0_rd_i;
assign rat_cmt_num0_o = head_robid0_i;
assign rat_cmt_en1_o = arf_we1_o;
assign rat_cmt_addr1_o = cmt1_rd_i;
assign rat_cmt_num1_o = cmt1_robid;

// SB：提交 store 入队（单写口；槽0 优先）
wire cmt0_sb_push = cmt0_effect && cmt0_is_store_i;
wire cmt1_sb_push = cmt1_effect && cmt1_is_store_i && !cmt0_sb_push;
assign sb_push_valid_o    = cmt0_sb_push || cmt1_sb_push;
assign sb_push_paddr_o    = cmt0_sb_push ? cmt0_paddr_i    : cmt1_paddr_i;
assign sb_push_data_o     = cmt0_sb_push ? cmt0_result_i   : cmt1_result_i;
assign sb_push_wstrb_o    = cmt0_sb_push ? cmt0_wstrb_i    : cmt1_wstrb_i;
assign sb_push_size_o     = cmt0_sb_push ? cmt0_size_i     : cmt1_size_i;
assign sb_push_uncached_o = cmt0_sb_push ? cmt0_uncached_i : cmt1_uncached_i;

assign csr_cmt_valid_o = cmt0_retire || cmt1_retire;
assign csr_cmt_pc_o = csr_sel_pc;
assign csr_cmt_ex_o = selected_excp_take;
assign csr_cmt_ertn_o = selected_effect && sel_priv[`PRIV_ERTN];
// BADV/ERA 的地址来源:访存类异常用 sel_vaddr(访存 vaddr);取指类(含取指侧 ADEF、TLB 重填)用 csr_sel_pc。
// 取指侧 ADEF(jirl 目标非对齐)的错误地址 == 出错取指指令自身 PC(=非对齐目标 0x227f9789),恰为 csr_sel_pc;
// ROB 对取指异常气泡的 vaddr 字段未写(=0),故不能用 sel_vaddr。csr handler 对 ADEF 用 wb_vaddr 记 ERA/BADV,
// 而这里 ADEF 走 csr_sel_pc 分支 → csr_cmt_vaddr_o=fault PC → ERA/BADV 正确,adef_ex `bne r27,ERA` 通过。
assign csr_cmt_vaddr_o = mem_excp ? sel_vaddr : csr_sel_pc;
assign excp_int_o = int_take;
assign excp_adef_o = sel_excp[`EXCP_ADEF];
assign excp_adem_o = sel_excp[`EXCP_ADEM];
assign excp_ipe_o = sel_excp[`EXCP_IPE];
assign excp_ale_o = sel_excp[`EXCP_ALE];
assign excp_sys_o = sel_excp[`EXCP_SYS];
assign excp_brk_o = sel_excp[`EXCP_BRK];
assign excp_ine_o = sel_excp[`EXCP_INE];
assign excp_tlb_vec_o[`TLB_EX_TLBR] = sel_excp[`EXCP_TLBR_F] | sel_excp[`EXCP_TLBR_M];
assign excp_tlb_vec_o[`TLB_EX_PIF]  = sel_excp[`EXCP_PIF];
assign excp_tlb_vec_o[`TLB_EX_PPI]  = sel_excp[`EXCP_PPI_F] | sel_excp[`EXCP_PPI_M];
assign excp_tlb_vec_o[`TLB_EX_PIL]  = sel_excp[`EXCP_PIL];
assign excp_tlb_vec_o[`TLB_EX_PIS]  = sel_excp[`EXCP_PIS];
assign excp_tlb_vec_o[`TLB_EX_PME]  = sel_excp[`EXCP_PME];

assign csr_we_o = selected_effect && sel_priv[`PRIV_CSR_WR];
assign csr_wnum_o = sel_csr_num;
assign csr_wmask_o = 32'hffff_ffff;
assign csr_wvalue_o = sel_result2;
assign ll_set_o = selected_effect && sel_priv[`PRIV_LL];
assign sc_set_o = selected_effect && sel_priv[`PRIV_SC];
assign lladdr_o = sel_paddr[31:4];

assign tlb_op_cmt_o = (selected_effect && sel_priv[`PRIV_TLB]) ? sel_tlb_op : {`TLB_OP_NUM{1'b0}};
assign invtlb_vpn_o = sel_result2[31:13];
assign invtlb_asid_o = sel_result2[9:0];

assign icacop_valid_o = selected_effect && sel_priv[`PRIV_CACOP] && (sel_cacop_code[2:0] == 3'b000);
assign dcacop_valid_o = selected_effect && sel_priv[`PRIV_CACOP] && (sel_cacop_code[2:0] == 3'b001);
assign icacop_op_o = sel_cacop_code[4:3];
assign dcacop_op_o = sel_cacop_code[4:3];
assign icacop_addr_o = sel_paddr;
assign dcacop_addr_o = sel_paddr;

wire ftq_s0_last = cmt0_effect && (cmt0_is_last_i ||
                    (cmt0_is_branch_i && cmt0_br_taken_i));
wire ftq_s1_last = cmt1_effect && (cmt1_is_last_i ||
                    (cmt1_is_branch_i && cmt1_br_taken_i));

assign ftq_cmt_valid_o = selected_effect || cmt1_dual_effect;
assign ftq_cmt_id_o = sel_ftq_id;
assign ftq_cmt_is_last_o = ftq_s0_last || ftq_s1_last;
assign ftq_cmt_release_o = {1'b0, ftq_s0_last} + {1'b0, ftq_s1_last};
assign ftq_cmt_is_branch_o = sel_is_branch;
assign ftq_cmt_taken_o = sel_br_taken;
assign ftq_cmt_mispred_o = selected_mispred;
assign ftq_cmt_target_o = sel_br_target;
assign ftq_cmt_br_type_o = sel_br_type;
assign ftq_cmt_pc_o = sel_pc;
// 有槽0 时 query 用槽0 id（与槽1 双提同 id 时目标一致）；仅槽1 头槽时用槽1 id
assign ftq_query_id_o = cmt0_valid_i ? cmt0_ftq_id_i : cmt1_ftq_id_i;

// RAS：仅在真正 effect 的分支上更新（双提槽1 正确 COND/UNCOND 时走 br_sel）
wire br_retired = br_sel_cmt1 ? (cmt1_effect && cmt1_is_branch_i)
                              : (cmt0_effect && cmt0_is_branch_i);
assign ras_cmt_call_o = br_retired && (sel_br_type == `BR_TYPE_CALL);
assign ras_cmt_ret_o  = br_retired && (sel_br_type == `BR_TYPE_RET);
assign ras_cmt_retaddr_o = sel_pc + 32'd4;

assign flush_req_o = selected_excp_take || selected_priv_flush || selected_mispred;
assign flush_type_o = selected_excp_take ? `FLUSH_EXCP :
                      (selected_effect && sel_priv[`PRIV_ERTN]) ? `FLUSH_ERTN :
                      selected_priv_flush ? `FLUSH_REFETCH :
                      selected_mispred ? `FLUSH_MISPRED :
                      `FLUSH_NONE;
// 真 idle：FLUSH 到 pc+4 + ctrl 冻取指；唤醒后取后继并 int_take（ERA=后继）。
// 套件 `b 0`(0x50000000)：仍 FLUSH 回自身，int_take 可附着其上（n49 ERA=循环 PC）。
assign flush_pc_o = selected_excp_take ? csr_next_pc_i :
                    (selected_effect && sel_priv[`PRIV_ERTN]) ? csr_next_pc_i :
                    (selected_priv_flush && sel_priv[`PRIV_IDLE] &&
                     !sel_inst_is_b0) ? (sel_pc + 32'd4) :
                    (selected_priv_flush && sel_priv[`PRIV_IDLE]) ? sel_pc :
                    selected_priv_flush ? (sel_pc + 32'd4) :
                    selected_mispred ? (sel_br_taken ? sel_br_target : (sel_pc + 32'd4)) :
                    32'b0;
assign idle_commit_o = selected_effect && sel_priv[`PRIV_IDLE];

assign debug0_valid_o = cmt0_effect;
assign debug0_pc_o = cmt0_pc_i;
assign debug0_rf_wen_o = arf_we0_o ? 4'hf : 4'h0;
assign debug0_rf_wnum_o = cmt0_rd_i;
assign debug0_rf_wdata_o = arf_wdata0_o;
assign debug0_inst_o = cmt0_inst_i;
assign debug1_valid_o = cmt1_effect;
assign debug1_pc_o = cmt1_pc_i;
assign debug1_rf_wen_o = arf_we1_o ? 4'hf : 4'h0;
assign debug1_rf_wnum_o = cmt1_rd_i;
assign debug1_rf_wdata_o = arf_wdata1_o;
assign debug1_inst_o = cmt1_inst_i;

endmodule
