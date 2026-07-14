// ============================================================
// dispatch 模块（分发级：读 ROB 补操作数 + 按 FU 类型路由进保留站）
// ------------------------------------------------------------
// 功能：
// - 接收 rename 流水寄存器中的两条指令，做两件事：
//   1) 操作数补全：源操作数 ready=0 的，用其 ROB 编号读 ROB ——
//      若该编号的指令已写回（ROB ready），直接取到值（ready 变 1）；
//      仍未写回的，带着编号进保留站等唤醒。
//   2) 按 futype 路由进 4 个保留站：
//      FU_ALU -> rs_alu0 / rs_alu1（双 ALU 负载均衡：选剩余容量大的）
//      FU_MEM -> rs_mem（一拍最多进 1 条）
//      FU_MDU -> rs_mdu（一拍最多进 1 条）
// - 本级纯组合（rename 的流水寄存器就是本级的输入寄存器）。
//
// 端口：
// - dis0/dis1_*      ：rename 流水寄存器输出（两槽指令）
// - rob_raddr/rrdy/rdata ：ROB 4 个操作数读口
// - rs_alu0/1、rs_mem、rs_mdu 的入站口（bundle 见各 RS 模块）
// - rs_*_can_accept / occupancy ：各 RS 空位信息
// - dispatch_ready_o ：分发级两槽均已空（rename 可接收新一对）
// - dis0/1_fire_o    ：本拍该槽已入站（rename 逐槽清 valid）
// ============================================================
`include "mycpu.h"

module dispatch(
    // =============== 槽 0 输入（rename 流水寄存器） ===============
    input  wire                       dis0_valid_i,
    input  wire [`ROB_W-1:0]          dis0_robid_i,
    input  wire [31:0]                dis0_pc_i,
    input  wire [`FU_NUM-1:0]         dis0_futype_i,
    input  wire [`ALU_OP_NUM-1:0]     dis0_alu_op_i,
    input  wire [`BR_OP_NUM-1:0]      dis0_br_op_i,
    input  wire [`MEM_OP_NUM-1:0]     dis0_mem_op_i,
    input  wire [`CSR_OP_NUM-1:0]     dis0_csr_op_i,
    input  wire [`TLB_OP_NUM-1:0]     dis0_tlb_op_i,
    input  wire [`WB_SRC_NUM-1:0]     dis0_wb_src_op_i,
    input  wire [13:0]                dis0_csr_num_i,
    input  wire                       dis0_is_cacop_i,
    input  wire                       dis0_src0_ready_i,
    input  wire [31:0]                dis0_src0_val_i,
    input  wire [`ROB_W-1:0]          dis0_src0_robid_i,
    input  wire                       dis0_src1_ready_i,
    input  wire [31:0]                dis0_src1_val_i,
    input  wire [`ROB_W-1:0]          dis0_src1_robid_i,
    input  wire [31:0]                dis0_imm_i,
    input  wire                       dis0_use_imm_i,
    input  wire [31:0]                dis0_br_offs_i,

    // =============== 槽 1 输入 ===============
    input  wire                       dis1_valid_i,
    input  wire [`ROB_W-1:0]          dis1_robid_i,
    input  wire [31:0]                dis1_pc_i,
    input  wire [`FU_NUM-1:0]         dis1_futype_i,
    input  wire [`ALU_OP_NUM-1:0]     dis1_alu_op_i,
    input  wire [`BR_OP_NUM-1:0]      dis1_br_op_i,
    input  wire [`MEM_OP_NUM-1:0]     dis1_mem_op_i,
    input  wire [`CSR_OP_NUM-1:0]     dis1_csr_op_i,
    input  wire [`TLB_OP_NUM-1:0]     dis1_tlb_op_i,
    input  wire [`WB_SRC_NUM-1:0]     dis1_wb_src_op_i,
    input  wire [13:0]                dis1_csr_num_i,
    input  wire                       dis1_is_cacop_i,
    input  wire                       dis1_src0_ready_i,
    input  wire [31:0]                dis1_src0_val_i,
    input  wire [`ROB_W-1:0]          dis1_src0_robid_i,
    input  wire                       dis1_src1_ready_i,
    input  wire [31:0]                dis1_src1_val_i,
    input  wire [`ROB_W-1:0]          dis1_src1_robid_i,
    input  wire [31:0]                dis1_imm_i,
    input  wire                       dis1_use_imm_i,
    input  wire [31:0]                dis1_br_offs_i,

    output wire                       dispatch_ready_o,    // 两槽均已空（rename 可写新一对）
    output wire                       dis0_fire_o,         // 本拍槽 0 入站
    output wire                       dis1_fire_o,         // 本拍槽 1 入站

    // =============== ROB 操作数读口 ×4（组合） ===============
    output wire [`ROB_W-1:0]          rob_raddr0_o,        // 槽0 src0
    input  wire                       rob_rrdy0_i,         // 该编号已写回
    input  wire [31:0]                rob_rdata0_i,
    output wire [`ROB_W-1:0]          rob_raddr1_o,        // 槽0 src1
    input  wire                       rob_rrdy1_i,
    input  wire [31:0]                rob_rdata1_i,
    output wire [`ROB_W-1:0]          rob_raddr2_o,        // 槽1 src0
    input  wire                       rob_rrdy2_i,
    input  wire [31:0]                rob_rdata2_i,
    output wire [`ROB_W-1:0]          rob_raddr3_o,        // 槽1 src1
    input  wire                       rob_rrdy3_i,
    input  wire [31:0]                rob_rdata3_i,

    // =============== 分发驻留旁路 ===============
    // 指令在本级等待时，用锁存源地址实时查 RAT/ARF（口 4~7）。
    // RAT busy 清 0 说明前序写它的指令已提交，此时 ARF 即权威值，可直接唤醒；
    // 好处：驻留期间前序 ROB 项可能被更年轻指令复用（ABA），改读 ARF 可绕开旧 tag。
    input  wire                       dis_rat_rbusy0_i,      // dis0 src0 当前 RAT busy
    input  wire                       dis_rat_rbusy1_i,      // dis0 src1
    input  wire                       dis_rat_rbusy2_i,      // dis1 src0
    input  wire                       dis_rat_rbusy3_i,      // dis1 src1
    input  wire [31:0]                dis_arf_rdata0_i,      // dis0 src0 ARF 权威值
    input  wire [31:0]                dis_arf_rdata1_i,      // dis0 src1
    input  wire [31:0]                dis_arf_rdata2_i,      // dis1 src0
    input  wire [31:0]                dis_arf_rdata3_i,      // dis1 src1

    // =============== rs_alu0 入站口 ===============
    input  wire                       rs_alu0_can_accept_i,
    input  wire [2:0]                 rs_alu0_occupancy_i,   // 占用项数（负载均衡用）
    output wire                       rs_alu0_push_valid_o,
    output wire [`ROB_W-1:0]          rs_alu0_push_robid_o,
    output wire [31:0]                rs_alu0_push_pc_o,
    output wire [`ALU_OP_NUM-1:0]     rs_alu0_push_alu_op_o,
    output wire [`BR_OP_NUM-1:0]      rs_alu0_push_br_op_o,
    output wire                       rs_alu0_push_src0_ready_o,
    output wire [31:0]                rs_alu0_push_src0_val_o,
    output wire [`ROB_W-1:0]          rs_alu0_push_src0_robid_o,
    output wire                       rs_alu0_push_src1_ready_o,
    output wire [31:0]                rs_alu0_push_src1_val_o,
    output wire [`ROB_W-1:0]          rs_alu0_push_src1_robid_o,
    output wire [31:0]                rs_alu0_push_imm_o,
    output wire                       rs_alu0_push_use_imm_o,
    output wire [31:0]                rs_alu0_push_br_offs_o,

    // =============== rs_alu1 入站口 ===============
    input  wire                       rs_alu1_can_accept_i,
    input  wire [2:0]                 rs_alu1_occupancy_i,
    output wire                       rs_alu1_push_valid_o,
    output wire [`ROB_W-1:0]          rs_alu1_push_robid_o,
    output wire [31:0]                rs_alu1_push_pc_o,
    output wire [`ALU_OP_NUM-1:0]     rs_alu1_push_alu_op_o,
    output wire [`BR_OP_NUM-1:0]      rs_alu1_push_br_op_o,
    output wire                       rs_alu1_push_src0_ready_o,
    output wire [31:0]                rs_alu1_push_src0_val_o,
    output wire [`ROB_W-1:0]          rs_alu1_push_src0_robid_o,
    output wire                       rs_alu1_push_src1_ready_o,
    output wire [31:0]                rs_alu1_push_src1_val_o,
    output wire [`ROB_W-1:0]          rs_alu1_push_src1_robid_o,
    output wire [31:0]                rs_alu1_push_imm_o,
    output wire                       rs_alu1_push_use_imm_o,
    output wire [31:0]                rs_alu1_push_br_offs_o,

    // =============== rs_mem 入站口 ===============
    input  wire                       rs_mem_can_accept_i,
    output wire                       rs_mem_push_valid_o,
    output wire [`ROB_W-1:0]          rs_mem_push_robid_o,
    output wire [31:0]                rs_mem_push_pc_o,
    output wire [`MEM_OP_NUM-1:0]     rs_mem_push_mem_op_o,
    output wire                       rs_mem_push_is_cacop_o,
    output wire                       rs_mem_push_src0_ready_o,
    output wire [31:0]                rs_mem_push_src0_val_o,
    output wire [`ROB_W-1:0]          rs_mem_push_src0_robid_o,
    output wire                       rs_mem_push_src1_ready_o,
    output wire [31:0]                rs_mem_push_src1_val_o,
    output wire [`ROB_W-1:0]          rs_mem_push_src1_robid_o,
    output wire [31:0]                rs_mem_push_imm_o,

    // =============== rs_mdu 入站口 ===============
    input  wire                       rs_mdu_can_accept_i,
    output wire                       rs_mdu_push_valid_o,
    output wire [`ROB_W-1:0]          rs_mdu_push_robid_o,
    output wire [`ALU_OP_NUM-1:0]     rs_mdu_push_alu_op_o,    // 乘除位有效
    output wire [`CSR_OP_NUM-1:0]     rs_mdu_push_csr_op_o,
    output wire [13:0]                rs_mdu_push_csr_num_o,
    output wire [`TLB_OP_NUM-1:0]     rs_mdu_push_tlb_op_o,
    output wire [`WB_SRC_NUM-1:0]     rs_mdu_push_wb_src_op_o,
    output wire                       rs_mdu_push_src0_ready_o,
    output wire [31:0]                rs_mdu_push_src0_val_o,
    output wire [`ROB_W-1:0]          rs_mdu_push_src0_robid_o,
    output wire                       rs_mdu_push_src1_ready_o,
    output wire [31:0]                rs_mdu_push_src1_val_o,
    output wire [`ROB_W-1:0]          rs_mdu_push_src1_robid_o
);

//TODO: 实现分发逻辑（纯组合；参考：mariver dispatch.v 137~243 行）
//
//TODO: 第一步——读 ROB 补操作数：
//      rob_raddr0~3 直接接 4 个源的 robid；
//      最终就绪/值（以槽0 src0 为例）：
//        ready = dis0_src0_ready_i | rob_rrdy0_i;
//        val   = dis0_src0_ready_i ? dis0_src0_val_i : rob_rdata0_i;
//        robid 原样带走（没 ready 的在 RS 里等唤醒）。
//      注意 ROB 读口内部已带"同拍 FU 写回旁路"（rob.v 实现），这里无需再旁路。
//
//TODO: 第二步——路由决策（组合）：
//      slot0_is_alu/mem/mdu = dis0_futype_i 独热位；slot1 同理。
//      ALU 负载均衡（两条都是 ALU 时）：槽0 进占用少的那个 RS，槽1 进另一个；
//      只有一条 ALU 时：进占用少且 can_accept 的 RS（mariver alucap 比较）。
//      MEM/MDU：每拍每站最多 1 条；两条同类时本拍只发槽 0（槽 1 等下一拍，
//      见第三步的"全收或全停"约定——一期从简）。
//
//TODO: 第三步——dispatch_ready_o（一期"全收或全停"约定，最简单且无半发状态）：
//      dispatch_ready_o = 所有 valid 槽都能在本拍入站：
//        - 两条 ALU：rs_alu0_can_accept && rs_alu1_can_accept
//        - 一条 ALU：rs_alu0/1 至少一个 can_accept
//        - MEM/MDU 槽：对应 RS can_accept，且两槽不同类（两条同 MEM/同 MDU -> not ready）
//      ready=0 时所有 push_valid 必须为 0（rename 流水寄存器保持，下拍重试）。
//      //二期优化 TODO：mask 部分分发（mariver mask0/mask1 机制——先发能发的，
//        记录已发标记，剩下的下拍再发；能提升 RS 利用率，但要在 rename 流水
//        寄存器加 per-slot issued 标记，建议框架跑通后再做）。
//
//TODO: 第四步——push 信号拼装：
//      把对应槽的字段（含第一步补全后的 src ready/val）按路由结果接到
//      rs_*_push_* 上；未选中的 RS push_valid=0。
//
//TODO: 坑点提示：
//      1. 本级为纯组合，dispatch_ready_o 会反压 rename 的 can_go ——
//         不要在该环路里引入 RS 的 issue 信号（用"项数<容量"判 can_accept，
//         不要用"本拍发射后会空出一项"做超前判断，避免组合环）。
//      2. 两槽指令路由进两个 ALU RS 时，槽 0（更老）建议进占用多的站也可以——
//         顺序无关紧要，RS 内部按 robid 年龄选择发射；负载均衡只为吞吐。

wire dis0_is_alu;
wire dis0_is_mem;
wire dis0_is_mdu;
wire dis1_is_alu;
wire dis1_is_mem;
wire dis1_is_mdu;

wire dis0_src0_ready;
wire dis0_src1_ready;
wire dis1_src0_ready;
wire dis1_src1_ready;
wire [31:0] dis0_src0_val;
wire [31:0] dis0_src1_val;
wire [31:0] dis1_src0_val;
wire [31:0] dis1_src1_val;

wire two_alu;
wire two_mem;
wire two_mdu;
wire single_alu_to_alu0;
wire slot0_to_alu0;
wire slot0_to_alu1;
wire slot1_to_alu0;
wire slot1_to_alu1;
wire slot0_to_mem;
wire slot1_to_mem;
wire slot0_to_mdu;
wire slot1_to_mdu;
wire dis0_can_dispatch;
wire dis1_can_dispatch;
wire dis0_rs_ok;
wire dis1_rs_ok;
wire dis0_will_take_alu0;
wire dis0_will_take_alu1;
wire dis0_will_take_mem;
wire dis0_will_take_mdu;

assign rob_raddr0_o = dis0_src0_robid_i;
assign rob_raddr1_o = dis0_src1_robid_i;
assign rob_raddr2_o = dis1_src0_robid_i;
assign rob_raddr3_o = dis1_src1_robid_i;

// 已锁存 ready/val 优先；否则 RAT 不 busy 读 ARF；再否则读 ROB（防 robid 复用 ABA）
assign dis0_src0_ready = dis0_src0_ready_i | !dis_rat_rbusy0_i | rob_rrdy0_i;
assign dis0_src1_ready = dis0_src1_ready_i | !dis_rat_rbusy1_i | rob_rrdy1_i;
assign dis1_src0_ready = dis1_src0_ready_i | !dis_rat_rbusy2_i | rob_rrdy2_i;
assign dis1_src1_ready = dis1_src1_ready_i | !dis_rat_rbusy3_i | rob_rrdy3_i;

// 已锁存 ready/val 优先（rename 驻留 wakeup 可能新于 ARF 提交）；未锁存且 !busy 读 ARF；
// busy 时 rob_rrdy 优先于旧锁存（防 robid ABA）；!busy 必须优先于 rob_rrdy（防 RAT 释放后误读旧 ROB）
assign dis0_src0_val = dis0_src0_ready_i ? dis0_src0_val_i :
                       !dis_rat_rbusy0_i ? dis_arf_rdata0_i :
                       rob_rrdy0_i ? rob_rdata0_i : 32'b0;
assign dis0_src1_val = dis0_src1_ready_i ? dis0_src1_val_i :
                       !dis_rat_rbusy1_i ? dis_arf_rdata1_i :
                       rob_rrdy1_i ? rob_rdata1_i : 32'b0;
assign dis1_src0_val = dis1_src0_ready_i ? dis1_src0_val_i :
                       !dis_rat_rbusy2_i ? dis_arf_rdata2_i :
                       rob_rrdy2_i ? rob_rdata2_i : 32'b0;
assign dis1_src1_val = dis1_src1_ready_i ? dis1_src1_val_i :
                       !dis_rat_rbusy3_i ? dis_arf_rdata3_i :
                       rob_rrdy3_i ? rob_rdata3_i : 32'b0;

assign dis0_ops_ready = dis0_src0_ready && dis0_src1_ready;
assign dis1_ops_ready = dis1_src0_ready && dis1_src1_ready;

assign dis0_is_alu = dis0_valid_i && dis0_futype_i[`FU_ALU];
assign dis0_is_mem = dis0_valid_i && dis0_futype_i[`FU_MEM];
assign dis0_is_mdu = dis0_valid_i && dis0_futype_i[`FU_MDU];
assign dis1_is_alu = dis1_valid_i && dis1_futype_i[`FU_ALU];
assign dis1_is_mem = dis1_valid_i && dis1_futype_i[`FU_MEM];
assign dis1_is_mdu = dis1_valid_i && dis1_futype_i[`FU_MDU];

assign two_alu = dis0_is_alu && dis1_is_alu;
assign two_mem = dis0_is_mem && dis1_is_mem;
assign two_mdu = dis0_is_mdu && dis1_is_mdu;

assign single_alu_to_alu0 = rs_alu0_can_accept_i &&
                            (!rs_alu1_can_accept_i ||
                             (rs_alu0_occupancy_i <= rs_alu1_occupancy_i));

assign slot0_to_alu0 = dis0_is_alu &&
                       (two_alu ? (rs_alu0_occupancy_i <= rs_alu1_occupancy_i)
                                : single_alu_to_alu0);
assign slot0_to_alu1 = dis0_is_alu &&
                       (two_alu ? (rs_alu0_occupancy_i > rs_alu1_occupancy_i)
                                : !single_alu_to_alu0);
assign slot1_to_alu0 = dis1_is_alu &&
                       (two_alu ? (rs_alu0_occupancy_i > rs_alu1_occupancy_i)
                                : single_alu_to_alu0);
assign slot1_to_alu1 = dis1_is_alu &&
                       (two_alu ? (rs_alu0_occupancy_i <= rs_alu1_occupancy_i)
                                : !single_alu_to_alu0);
assign slot0_to_mem = dis0_is_mem;
assign slot1_to_mem = dis1_is_mem;
assign slot0_to_mdu = dis0_is_mdu;
assign slot1_to_mdu = dis1_is_mdu;

assign dis0_will_take_alu0 = dis0_valid_i && dis0_ops_ready && dis0_is_alu && slot0_to_alu0;
assign dis0_will_take_alu1 = dis0_valid_i && dis0_ops_ready && dis0_is_alu && slot0_to_alu1;
assign dis0_will_take_mem  = dis0_valid_i && dis0_ops_ready && dis0_is_mem;
assign dis0_will_take_mdu  = dis0_valid_i && dis0_ops_ready && dis0_is_mdu;

assign dis0_rs_ok = dis0_is_alu ? (slot0_to_alu0 ? rs_alu0_can_accept_i : rs_alu1_can_accept_i) :
                    dis0_is_mem ? rs_mem_can_accept_i :
                    dis0_is_mdu ? rs_mdu_can_accept_i :
                    1'b0;
assign dis1_rs_ok = dis1_is_alu ? (slot1_to_alu0 ? (rs_alu0_can_accept_i && !dis0_will_take_alu0)
                                                   : (rs_alu1_can_accept_i && !dis0_will_take_alu1)) :
                    dis1_is_mem ? (rs_mem_can_accept_i && !dis0_will_take_mem) :
                    dis1_is_mdu ? (rs_mdu_can_accept_i && !dis0_will_take_mdu) :
                    1'b0;

assign dis0_can_dispatch = dis0_valid_i && dis0_ops_ready && dis0_rs_ok;
assign dis1_can_dispatch = dis1_valid_i && dis1_ops_ready && dis1_rs_ok;

assign dis0_fire_o = dis0_can_dispatch;
// 槽 1 只能在槽 0 已空后入站，避免同 rename 对内乱序
assign dis1_fire_o = dis1_can_dispatch && !dis0_valid_i;
assign dispatch_ready_o = !dis0_valid_i && !dis1_valid_i;

wire rs_alu0_from_slot1 = slot1_to_alu0 && dis1_fire_o;
wire rs_alu1_from_slot1 = slot1_to_alu1 && dis1_fire_o;
wire rs_mem_from_slot1  = slot1_to_mem && dis1_fire_o;
wire rs_mdu_from_slot1  = slot1_to_mdu && dis1_fire_o;

assign rs_alu0_push_valid_o = (slot0_to_alu0 && dis0_fire_o) || (slot1_to_alu0 && dis1_fire_o);
assign rs_alu0_push_robid_o = rs_alu0_from_slot1 ? dis1_robid_i : dis0_robid_i;
assign rs_alu0_push_pc_o = rs_alu0_from_slot1 ? dis1_pc_i : dis0_pc_i;
assign rs_alu0_push_alu_op_o = rs_alu0_from_slot1 ? dis1_alu_op_i : dis0_alu_op_i;
assign rs_alu0_push_br_op_o = rs_alu0_from_slot1 ? dis1_br_op_i : dis0_br_op_i;
assign rs_alu0_push_src0_ready_o = rs_alu0_from_slot1 ? dis1_src0_ready : dis0_src0_ready;
assign rs_alu0_push_src0_val_o = rs_alu0_from_slot1 ? dis1_src0_val : dis0_src0_val;
assign rs_alu0_push_src0_robid_o = rs_alu0_from_slot1 ? dis1_src0_robid_i : dis0_src0_robid_i;
assign rs_alu0_push_src1_ready_o = rs_alu0_from_slot1 ? dis1_src1_ready : dis0_src1_ready;
assign rs_alu0_push_src1_val_o = rs_alu0_from_slot1 ? dis1_src1_val : dis0_src1_val;
assign rs_alu0_push_src1_robid_o = rs_alu0_from_slot1 ? dis1_src1_robid_i : dis0_src1_robid_i;
assign rs_alu0_push_imm_o = rs_alu0_from_slot1 ? dis1_imm_i : dis0_imm_i;
assign rs_alu0_push_use_imm_o = rs_alu0_from_slot1 ? dis1_use_imm_i : dis0_use_imm_i;
assign rs_alu0_push_br_offs_o = rs_alu0_from_slot1 ? dis1_br_offs_i : dis0_br_offs_i;

assign rs_alu1_push_valid_o = (slot0_to_alu1 && dis0_fire_o) || (slot1_to_alu1 && dis1_fire_o);
assign rs_alu1_push_robid_o = rs_alu1_from_slot1 ? dis1_robid_i : dis0_robid_i;
assign rs_alu1_push_pc_o = rs_alu1_from_slot1 ? dis1_pc_i : dis0_pc_i;
assign rs_alu1_push_alu_op_o = rs_alu1_from_slot1 ? dis1_alu_op_i : dis0_alu_op_i;
assign rs_alu1_push_br_op_o = rs_alu1_from_slot1 ? dis1_br_op_i : dis0_br_op_i;
assign rs_alu1_push_src0_ready_o = rs_alu1_from_slot1 ? dis1_src0_ready : dis0_src0_ready;
assign rs_alu1_push_src0_val_o = rs_alu1_from_slot1 ? dis1_src0_val : dis0_src0_val;
assign rs_alu1_push_src0_robid_o = rs_alu1_from_slot1 ? dis1_src0_robid_i : dis0_src0_robid_i;
assign rs_alu1_push_src1_ready_o = rs_alu1_from_slot1 ? dis1_src1_ready : dis0_src1_ready;
assign rs_alu1_push_src1_val_o = rs_alu1_from_slot1 ? dis1_src1_val : dis0_src1_val;
assign rs_alu1_push_src1_robid_o = rs_alu1_from_slot1 ? dis1_src1_robid_i : dis0_src1_robid_i;
assign rs_alu1_push_imm_o = rs_alu1_from_slot1 ? dis1_imm_i : dis0_imm_i;
assign rs_alu1_push_use_imm_o = rs_alu1_from_slot1 ? dis1_use_imm_i : dis0_use_imm_i;
assign rs_alu1_push_br_offs_o = rs_alu1_from_slot1 ? dis1_br_offs_i : dis0_br_offs_i;

assign rs_mem_push_valid_o = (slot0_to_mem && dis0_fire_o) || (slot1_to_mem && dis1_fire_o);
assign rs_mem_push_robid_o = rs_mem_from_slot1 ? dis1_robid_i : dis0_robid_i;
assign rs_mem_push_pc_o = rs_mem_from_slot1 ? dis1_pc_i : dis0_pc_i;
assign rs_mem_push_mem_op_o = rs_mem_from_slot1 ? dis1_mem_op_i : dis0_mem_op_i;
assign rs_mem_push_is_cacop_o = rs_mem_from_slot1 ? dis1_is_cacop_i : dis0_is_cacop_i;
assign rs_mem_push_src0_ready_o = rs_mem_from_slot1 ? dis1_src0_ready : dis0_src0_ready;
assign rs_mem_push_src0_val_o = rs_mem_from_slot1 ? dis1_src0_val : dis0_src0_val;
assign rs_mem_push_src0_robid_o = rs_mem_from_slot1 ? dis1_src0_robid_i : dis0_src0_robid_i;
assign rs_mem_push_src1_ready_o = rs_mem_from_slot1 ? dis1_src1_ready : dis0_src1_ready;
assign rs_mem_push_src1_val_o = rs_mem_from_slot1 ? dis1_src1_val : dis0_src1_val;
assign rs_mem_push_src1_robid_o = rs_mem_from_slot1 ? dis1_src1_robid_i : dis0_src1_robid_i;
assign rs_mem_push_imm_o = rs_mem_from_slot1 ? dis1_imm_i : dis0_imm_i;

assign rs_mdu_push_valid_o = (slot0_to_mdu && dis0_fire_o) || (slot1_to_mdu && dis1_fire_o);
assign rs_mdu_push_robid_o = rs_mdu_from_slot1 ? dis1_robid_i : dis0_robid_i;
assign rs_mdu_push_alu_op_o = rs_mdu_from_slot1 ? dis1_alu_op_i : dis0_alu_op_i;
assign rs_mdu_push_csr_op_o = rs_mdu_from_slot1 ? dis1_csr_op_i : dis0_csr_op_i;
assign rs_mdu_push_csr_num_o = rs_mdu_from_slot1 ? dis1_csr_num_i : dis0_csr_num_i;
assign rs_mdu_push_tlb_op_o = rs_mdu_from_slot1 ? dis1_tlb_op_i : dis0_tlb_op_i;
assign rs_mdu_push_wb_src_op_o = rs_mdu_from_slot1 ? dis1_wb_src_op_i : dis0_wb_src_op_i;
assign rs_mdu_push_src0_ready_o = rs_mdu_from_slot1 ? dis1_src0_ready : dis0_src0_ready;
assign rs_mdu_push_src0_val_o = rs_mdu_from_slot1 ? dis1_src0_val : dis0_src0_val;
assign rs_mdu_push_src0_robid_o = rs_mdu_from_slot1 ? dis1_src0_robid_i : dis0_src0_robid_i;
assign rs_mdu_push_src1_ready_o = rs_mdu_from_slot1 ? dis1_src1_ready : dis0_src1_ready;
assign rs_mdu_push_src1_val_o = rs_mdu_from_slot1 ? dis1_src1_val : dis0_src1_val;
assign rs_mdu_push_src1_robid_o = rs_mdu_from_slot1 ? dis1_src1_robid_i : dis0_src1_robid_i;

endmodule
