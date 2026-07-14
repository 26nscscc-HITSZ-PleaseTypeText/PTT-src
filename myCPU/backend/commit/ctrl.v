// ============================================================
// ctrl 模块（全局冲刷/重定向仲裁与分发）
// ------------------------------------------------------------
// 功能：
// - 接收 commit 级的冲刷请求（误预测/异常/ertn/refetch），仲裁后向全核
//   广播一拍 flush 脉冲与重定向 PC。本架构（一期）所有恢复都走
//   "提交级统一冲刷"：RAT/RS/ROB/rename/dispatch/IB/IFU/FTQ 全清，
//   BPU 的 PC 重定向到 flush_pc —— 天然正确，无需任何部分恢复逻辑。
// - idle 指令支持：idle 提交后冻结取指，直到有中断到来才放行。
// - 二期预留：执行级分支重定向入口（fu_alu 算出误预测当拍冲前端，
//   需配合 RAT 检查点，见 TODO）。
//
// 端口：
// - cmt_flush_*    ：commit 冲刷请求（类型 + 目标 PC）
// - ex_redirect_*  ：二期执行级重定向入口（一期悬空）
// - flush_o/pc     ：全局冲刷广播（一拍脉冲）
// - idle_commit_i / has_int_i / fetch_stall_o ：idle 睡眠控制
// ============================================================
`include "mycpu.h"

module ctrl(
    input  wire                       clk,
    input  wire                       reset,

    // ---------------- commit 冲刷请求 ----------------
    input  wire                       cmt_flush_req_i,     // 提交级请求冲刷（一拍）
    input  wire [`FLUSH_TYPE_W-1:0]   cmt_flush_type_i,    // FLUSH_MISPRED/EXCP/ERTN/REFETCH
    input  wire [31:0]                cmt_flush_pc_i,      // 冲刷后取指目标

    // ---------------- 二期：执行级分支重定向（预留，一期接 0）----------------
    input  wire                       ex_redirect_req_i,
    input  wire [31:0]                ex_redirect_pc_i,

    // ---------------- idle 睡眠控制 ----------------
    input  wire                       idle_commit_i,       // idle 指令提交（随 flush 同拍）
    input  wire                       has_int_i,           // CSR 有未屏蔽中断

    // ---------------- 全局广播 ----------------
    output wire                       flush_o,             // 全局冲刷（一拍脉冲，接所有模块 flush_i）
    output wire [31:0]                flush_pc_o,          // 重定向 PC（接 bpu.flush_pc_i）
    output wire                       fetch_stall_o        // idle 睡眠期间冻结 BPU 取指
);

//TODO: 实现全局控制（一期非常简单，几十行；参考：团队赛 ctrl.sv 的 flush 仲裁部分）
//
//TODO: 冲刷广播：
//      flush_o    = cmt_flush_req_i /*| ex_redirect_req_i（二期）*/;
//      flush_pc_o = cmt_flush_req_i ? cmt_flush_pc_i : ex_redirect_pc_i;
//      一期 commit 是唯一冲刷源，无需优先级仲裁；打不打一拍寄存看时序
//      （flush 扇出很大，建议在顶层对 flush 做一级寄存复制再分发，
//       commit 请求时序上留有余量——但注意打拍后 commit 侧要保证请求只发一拍）。
//
//TODO: idle 睡眠：
//      reg idle_lock;
//      idle_commit_i（伴随 FLUSH_REFETCH 冲刷）-> idle_lock <= 1（冻结取指）；
//      idle_lock && has_int_i -> idle_lock <= 0（中断到来，放行取指，
//        中断本身会在第一条新指令提交时附着触发异常入口）。
//      fetch_stall_o = idle_lock;（bpu 内 PC 保持、不产生新块；
//        实现上可直接与 ftq_full 一起作为 BPU 的停止条件，顶层已并联）
//
//TODO: 二期升级路径（执行级重定向，写给后来人）：
//      1. fu_alu 解析出分支误预测当拍 -> ex_redirect_req 冲前端（FTQ/IFU/IB 清空，
//         BPU 重定向），后端"分支之后的指令"需要选择性作废：
//         ROB tail 回滚到分支处 + RS 中比分支年轻的项作废（按 robid 年龄判断）。
//      2. RAT 恢复是难点：方案 A）分支重命名时做 RAT 快照（checkpoint，4~8 份），
//         误预测恢复快照；方案 B）等流水线排空后用"提交态重建"。先做 A 的 1 份
//         快照（同时只允许 1 条在飞分支投机）就能拿到大部分收益。
//      3. 在此之前，一期的提交级冲刷已保证功能完全正确，可放心先跑分。

reg idle_lock;

always @(posedge clk) begin
    if (reset) begin
        idle_lock <= 1'b0;
    end else if (has_int_i) begin
        idle_lock <= 1'b0;
    end else if (idle_commit_i) begin
        idle_lock <= 1'b1;
    end
end

// ============================================================
// 100MHz 攻坚:全局 flush 打一拍(参考团队赛 core_top 的 top_backend_flush_r)
// ------------------------------------------------------------
// 原来 flush_o/flush_pc_o 是组合信号,由提交级深链
//   rob.excp → csr_estat_ecode → commit 误预测比较 → flush_req(组合)
// 直接扇出到 17 个模块(fanout 1212),形成 34-36 级 LUT 的关键路径。
// 现把 flush/flush_pc 在本模块打一拍:
//   * 生成段:...→ cmt_flush_req_i(组合)→ flush 寄存器 D 端(短)
//   * 广播段:flush 寄存器 Q 端 → 各模块 flush_i(短)
// 巨扇出组合链被寄存器一分为二。
//
// 正确性(配合 commit 的 flush_pending 握手,见 commit.v):
//   T   拍:提交级发现队头 N 需冲刷 → cmt_flush_req_i=1(组合);N 的架构副作用
//           (写 ARF/CSR 异常入口、ROB 弹出 N)在 T 拍正常生效;flush 仍=0。
//           flush_r/flush_pc_r 在 T→T+1 沿捕获 1/目标 PC。
//   T+1 拍:flush=1 广播,各模块在 T+1→T+2 沿清空;flush_pc 重定向 BPU。
//           commit 收到 flush_pending=flush=1 → 闸住一切退休(cmt*_ready/int_take
//           全 0),故错误路径的 N+1 不会误提交;cmt_flush_req_i 回 0 → flush 单拍脉冲。
//   T+2 拍:ROB 空,前端已从 flush_pc 重填。
// 代价:每次冲刷多 1 拍气泡(IPC 微降),换取关键路径腰斩。
reg        flush_r;
reg [31:0] flush_pc_r;
always @(posedge clk) begin
    if (reset) begin
        flush_r    <= 1'b0;
        flush_pc_r <= 32'b0;
    end else begin
        flush_r    <= cmt_flush_req_i | ex_redirect_req_i;
        flush_pc_r <= cmt_flush_req_i ? cmt_flush_pc_i : ex_redirect_pc_i;
    end
end
assign flush_o    = flush_r;
assign flush_pc_o = flush_pc_r;
// idle 提交当拍即冻结取指（组合叠加 idle_commit_i），否则 idle 的 FLUSH_REFETCH
// 把 PC 打到 pc+4 后，idle_lock 要到下一拍才生效——中间这一拍 FTQ 两拍冻结尚未成立，
// pc+4 会被取指并提交（n49：pc+4=csrwr TCFG=0 提前关掉定时器 → 中断永不触发 → 死锁）。
// 组合提前一拍冻结，使两拍 ftq_freeze 覆盖 idle 提交拍，pc+4 停在 PC 寄存器但不取指；
// 定时器到期 has_int 置起后放行，pc+4 再入流水时携带 has_int → 提交拍 int_take 进异常入口。
assign fetch_stall_o = idle_lock | idle_commit_i;

endmodule
