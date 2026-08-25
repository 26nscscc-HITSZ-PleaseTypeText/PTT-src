// ============================================================
// dispatch 模块（分发级：读 ROB 补操作数 + 按 FU 类型路由进保留站）
// ------------------------------------------------------------
// 功能：
// - 接收 rename 流水寄存器中的两条指令，做两件事：
//   1) 操作数补全：源操作数 ready=0 的，用其 ROB 编号读 ROB ——
//      若该编号的指令已写回（ROB ready），直接取到值（ready 变 1）；
//      仍未写回的，带着编号进保留站等唤醒。
//   2) 按 futype 路由进 4 个保留站：
//      FU_ALU -> rs_alu0 / rs_alu1
//         * 双 ALU 且两站皆空：槽0→alu0、槽1→alu1，同拍双发；
//         * 否则按 occupancy 负载均衡选站；
//      FU_MEM -> rs_mem（一拍最多进 1 条）
//      FU_MDU -> rs_mdu（一拍最多进 1 条）
// - 入站不卡源就绪：未齐则带 robid 进 RS，由 wakeup 补齐（避免堵 rename）。
// - 槽1 不可越过未发出的槽0；可与槽0 同拍双发。
// - 本级纯组合（rename 的流水寄存器就是本级的输入寄存器）。
//
// 端口：
// - dis0/dis1_*      ：rename 流水寄存器输出（两槽指令）
// - rob_raddr/rrdy/rdata ：ROB 4 个操作数读口
// - rs_alu0/1、rs_mem、rs_mdu 的入站口（bundle 见各 RS 模块）
// - rs_*_can_accept / occupancy ：各 RS 空位信息
// - dispatch_ready_o ：两槽已空，或本拍将全部 vacated（均 fire）——rename 可装新一对
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
    input  wire [4:3]                 dis0_cacop_op_i,
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
    input  wire [4:3]                 dis1_cacop_op_i,
    input  wire                       dis1_src0_ready_i,
    input  wire [31:0]                dis1_src0_val_i,
    input  wire [`ROB_W-1:0]          dis1_src0_robid_i,
    input  wire                       dis1_src1_ready_i,
    input  wire [31:0]                dis1_src1_val_i,
    input  wire [`ROB_W-1:0]          dis1_src1_robid_i,
    input  wire [31:0]                dis1_imm_i,
    input  wire                       dis1_use_imm_i,
    input  wire [31:0]                dis1_br_offs_i,

    input  wire                       dis2_valid_i,
    input  wire [`ROB_W-1:0]          dis2_robid_i,
    input  wire [31:0]                dis2_pc_i,
    input  wire [`FU_NUM-1:0]         dis2_futype_i,
    input  wire [`ALU_OP_NUM-1:0]     dis2_alu_op_i,
    input  wire [`BR_OP_NUM-1:0]      dis2_br_op_i,
    input  wire [`MEM_OP_NUM-1:0]     dis2_mem_op_i,
    input  wire [`CSR_OP_NUM-1:0]     dis2_csr_op_i,
    input  wire [`TLB_OP_NUM-1:0]     dis2_tlb_op_i,
    input  wire [`WB_SRC_NUM-1:0]     dis2_wb_src_op_i,
    input  wire [13:0]                dis2_csr_num_i,
    input  wire                       dis2_is_cacop_i,
    input  wire [4:3]                 dis2_cacop_op_i,
    input  wire                       dis2_src0_ready_i,
    input  wire [31:0]                dis2_src0_val_i,
    input  wire [`ROB_W-1:0]          dis2_src0_robid_i,
    input  wire                       dis2_src1_ready_i,
    input  wire [31:0]                dis2_src1_val_i,
    input  wire [`ROB_W-1:0]          dis2_src1_robid_i,
    input  wire [31:0]                dis2_imm_i,
    input  wire                       dis2_use_imm_i,
    input  wire [31:0]                dis2_br_offs_i,

    output wire                       dispatch_ready_o,    // 空或本拍全部 vacated（可写新一对）
    output wire                       dis0_fire_o,         // 本拍槽 0 入站
    output wire                       dis1_fire_o,         // 本拍槽 1 入站
    output wire                       dis2_fire_o,

    // =============== ROB 操作数读口 ×4（组合） ===============
    output wire [`ROB_W-1:0]          rob_raddr0_o,        // 槽0 src0
    input  wire                       rob_rrdy0_i,         // 该编号已写回
    input  wire [31:0]                rob_rdata0_i,
    input  wire [31:0]                rob_rdata0_noa2_i,
    output wire [`ROB_W-1:0]          rob_raddr1_o,        // 槽0 src1
    input  wire                       rob_rrdy1_i,
    input  wire [31:0]                rob_rdata1_i,
    output wire [`ROB_W-1:0]          rob_raddr2_o,        // 槽1 src0
    input  wire                       rob_rrdy2_i,
    input  wire [31:0]                rob_rdata2_i,
    input  wire [31:0]                rob_rdata2_noa2_i,
    output wire [`ROB_W-1:0]          rob_raddr3_o,        // 槽1 src1
    input  wire                       rob_rrdy3_i,
    input  wire [31:0]                rob_rdata3_i,
    output wire [`ROB_W-1:0]          rob_raddr4_o,
    input  wire                       rob_rrdy4_i,
    input  wire [31:0]                rob_rdata4_i,
    input  wire [31:0]                rob_rdata4_noa2_i,
    output wire [`ROB_W-1:0]          rob_raddr5_o,
    input  wire                       rob_rrdy5_i,
    input  wire [31:0]                rob_rdata5_i,

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
    input  wire                       dis_rat_rbusy4_i,
    input  wire                       dis_rat_rbusy5_i,
    input  wire [31:0]                dis_arf_rdata4_i,
    input  wire [31:0]                dis_arf_rdata5_i,

    // =============== rs_alu0 入站口 ===============
    input  wire                       rs_alu0_can_accept_i,
    input  wire [`RS_ALU_OCC_W-1:0]   rs_alu0_occupancy_i,   // 占用项数（负载均衡用）
    output wire                       rs_alu0_push_valid_o,
    output wire [`ROB_W-1:0]          rs_alu0_push_robid_o,
    output wire [31:0]                rs_alu0_push_pc_o,
    output wire [14:0]                rs_alu0_push_alu_op_o, // {PCADD,ORN,ANDN,LUI..ADD}
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
    input  wire [`RS_ALU_OCC_W-1:0]   rs_alu1_occupancy_i,
    output wire                       rs_alu1_push_valid_o,
    output wire [`ROB_W-1:0]          rs_alu1_push_robid_o,
    output wire [31:0]                rs_alu1_push_pc_o,
    output wire [14:0]                rs_alu1_push_alu_op_o,
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

    input  wire                       rs_alu2_can_accept_i,
    input  wire [`RS_ALU_OCC_W-1:0]   rs_alu2_occupancy_i,
    output wire                       rs_alu2_push_valid_o,
    output wire [`ROB_W-1:0]          rs_alu2_push_robid_o,
    output wire [31:0]                rs_alu2_push_pc_o,
    output wire [14:0]                rs_alu2_push_alu_op_o,
    output wire [`BR_OP_NUM-1:0]      rs_alu2_push_br_op_o,
    output wire                       rs_alu2_push_src0_ready_o,
    output wire [31:0]                rs_alu2_push_src0_val_o,
    output wire [`ROB_W-1:0]          rs_alu2_push_src0_robid_o,
    output wire                       rs_alu2_push_src1_ready_o,
    output wire [31:0]                rs_alu2_push_src1_val_o,
    output wire [`ROB_W-1:0]          rs_alu2_push_src1_robid_o,
    output wire [31:0]                rs_alu2_push_imm_o,
    output wire                       rs_alu2_push_use_imm_o,
    output wire [31:0]                rs_alu2_push_br_offs_o,

    // =============== rs_mem 入站口 ===============
    input  wire                       rs_mem_can_accept_i,
    input  wire                       rs_mem_can_accept_two_i,
    output wire                       rs_mem_push_valid_o,
    output wire [`ROB_W-1:0]          rs_mem_push_robid_o,
    output wire [`MEM_OP_NUM-1:0]     rs_mem_push_mem_op_o,
    output wire                       rs_mem_push_is_cacop_o,
    output wire [4:3]                 rs_mem_push_cacop_op_o,
    output wire                       rs_mem_push_src0_ready_o,
    output wire [31:0]                rs_mem_push_src0_val_o,
    output wire [`ROB_W-1:0]          rs_mem_push_src0_robid_o,
    output wire [31:0]                rs_mem_push_src0_key_val_o,
    output wire                       rs_mem_push_src0_rob_wait_o,
    output wire                       rs_mem_push_src1_ready_o,
    output wire [31:0]                rs_mem_push_src1_val_o,
    output wire [`ROB_W-1:0]          rs_mem_push_src1_robid_o,
    output wire [31:0]                rs_mem_push_imm_o,
    output wire                       rs_mem_push1_valid_o,
    output wire [`ROB_W-1:0]          rs_mem_push1_robid_o,
    output wire [`MEM_OP_NUM-1:0]     rs_mem_push1_mem_op_o,
    output wire                       rs_mem_push1_is_cacop_o,
    output wire [4:3]                 rs_mem_push1_cacop_op_o,
    output wire                       rs_mem_push1_src0_ready_o,
    output wire [31:0]                rs_mem_push1_src0_val_o,
    output wire [`ROB_W-1:0]          rs_mem_push1_src0_robid_o,
    output wire [31:0]                rs_mem_push1_src0_key_val_o,
    output wire                       rs_mem_push1_src0_rob_wait_o,
    output wire                       rs_mem_push1_src1_ready_o,
    output wire [31:0]                rs_mem_push1_src1_val_o,
    output wire [`ROB_W-1:0]          rs_mem_push1_src1_robid_o,
    output wire [31:0]                rs_mem_push1_imm_o,

    // =============== rs_mdu 入站口 ===============
    input  wire                       rs_mdu_can_accept_i,
    output wire                       rs_mdu_push_valid_o,
    output wire [`ROB_W-1:0]          rs_mdu_push_robid_o,
    output wire [18:12]               rs_mdu_push_alu_op_o,    // MUL..MOD_WU
    output wire [`CSR_OP_NUM-1:0]     rs_mdu_push_csr_op_o,
    output wire [13:0]                rs_mdu_push_csr_num_o,
    output wire [`TLB_OP_NUM-1:0]     rs_mdu_push_tlb_op_o,
    output wire [`WB_SRC_NUM-1:0]     rs_mdu_push_wb_src_op_o, // {TID,CNTVH,CNTVL,ALU}
    output wire                       rs_mdu_push_src0_ready_o,
    output wire [31:0]                rs_mdu_push_src0_val_o,
    output wire [`ROB_W-1:0]          rs_mdu_push_src0_robid_o,
    output wire                       rs_mdu_push_src1_ready_o,
    output wire [31:0]                rs_mdu_push_src1_val_o,
    output wire [`ROB_W-1:0]          rs_mdu_push_src1_robid_o
);

// 设计说明（纯组合）：
//
// 第一步——读 ROB 补操作数：
//      rob_raddr0~3 直接接 4 个源的 robid；
//      最终就绪/值（以槽0 src0 为例）：
//        ready = dis0_src0_ready_i | rob_rrdy0_i;
//        val   = dis0_src0_ready_i ? dis0_src0_val_i : rob_rdata0_i;
//        robid 原样带走（没 ready 的在 RS 里等唤醒）。
//      注意 ROB 读口内部已带"同拍 FU 写回旁路"（rob.v 实现），这里无需再旁路。
//
// 第二步——路由决策（组合）：
//      slot0_is_alu/mem/mdu = dis0_futype_i 独热位；slot1 同理。
//      双 ALU 且两站都能收：固定程序序 槽0→rs_alu0、槽1→rs_alu1 同拍双压；
//      只有一条 ALU：进占用少且 can_accept 的站（alucap 比较）。
//      MEM/MDU：每拍每站最多 1 条；两条同类时本拍只发槽 0，槽 1 下拍再发
//      （per-slot fire，见第三步）。
//
// 第三步——per-slot fire 与 dispatch_ready_o：
//      dis0_fire/dis1_fire 按槽独立判定（dis1 不得越过未发出的 dis0，保序）；
//      dispatch_ready_o = 所有 valid 槽本拍都 fire（或槽本就空）——
//      支持同拍 vacate + refill，消除对 rename 的固有隔拍阻塞。
//      入站不要求源就绪：未齐则带 robid 进 RS 等唤醒，避免槽0 相关阻塞拖死 rename。
//
// 第四步——push 信号拼装：
//      把对应槽的字段（含第一步补全后的 src ready/val）按路由结果接到
//      rs_*_push_* 上；未选中的 RS push_valid=0。
//
// 坑点提示：
//      本级为纯组合，dispatch_ready_o 会反压 rename 的 can_go ——
//      不要在该环路里引入 RS 的 issue 信号（用"项数<容量"判 can_accept，
//      不要用"本拍发射后会空出一项"做超前判断，避免组合环）。

wire dis0_is_alu;
wire dis0_is_mem;
wire dis0_is_mdu;
wire dis1_is_alu;
wire dis1_is_mem;
wire dis1_is_mdu;
wire dis2_is_alu;
wire dis2_is_mem;
wire dis2_is_mdu;

wire dis0_src0_ready;
wire dis0_src1_ready;
wire dis1_src0_ready;
wire dis1_src1_ready;
wire dis2_src0_ready;
wire dis2_src1_ready;
wire [31:0] dis0_src0_val;
wire [31:0] dis0_src1_val;
wire [31:0] dis1_src0_val;
wire [31:0] dis1_src1_val;
wire [31:0] dis2_src0_val;
wire [31:0] dis0_src0_key_val;
wire [31:0] dis1_src0_key_val;
wire [31:0] dis2_src0_key_val;
wire [31:0] dis2_src1_val;

wire two_alu;
wire single_alu_to_alu0;
wire slot0_to_alu0;
wire slot0_to_alu1;
wire slot1_to_alu0;
wire slot1_to_alu1;
wire slot2_to_alu0;
wire slot2_to_alu1;
wire slot0_to_alu2;
wire slot1_to_alu2;
wire slot2_to_alu2;
wire slot0_to_mem;
wire slot1_to_mem;
wire slot2_to_mem;
wire slot0_to_mdu;
wire slot1_to_mdu;
wire slot2_to_mdu;
wire dis0_can_dispatch;
wire dis1_can_dispatch;
wire dis2_can_dispatch;
wire dis0_rs_ok;
wire dis1_rs_ok;
wire dis2_rs_ok;
wire dis0_will_take_alu0;
wire dis0_will_take_alu1;
wire dis0_will_take_mem;
wire dis0_will_take_mdu;
wire dis1_will_take_alu0;
wire dis1_will_take_alu1;
wire dis0_will_take_alu2;
wire dis1_will_take_alu2;
wire dis1_will_take_mem;
wire dis1_will_take_mdu;

assign rob_raddr0_o = dis0_src0_robid_i;
assign rob_raddr1_o = dis0_src1_robid_i;
assign rob_raddr2_o = dis1_src0_robid_i;
assign rob_raddr3_o = dis1_src1_robid_i;
assign rob_raddr4_o = dis2_src0_robid_i;
assign rob_raddr5_o = dis2_src1_robid_i;

// 已锁存 ready/val 优先；否则 RAT 不 busy 读 ARF；再否则读 ROB（防 robid 复用 ABA）
assign dis0_src0_ready = dis0_src0_ready_i | !dis_rat_rbusy0_i | rob_rrdy0_i;
assign dis0_src1_ready = dis0_src1_ready_i | !dis_rat_rbusy1_i | rob_rrdy1_i;
assign dis1_src0_ready = dis1_src0_ready_i | !dis_rat_rbusy2_i | rob_rrdy2_i;
assign dis1_src1_ready = dis1_src1_ready_i | !dis_rat_rbusy3_i | rob_rrdy3_i;
assign dis2_src0_ready = dis2_src0_ready_i | !dis_rat_rbusy4_i | rob_rrdy4_i;
assign dis2_src1_ready = dis2_src1_ready_i | !dis_rat_rbusy5_i | rob_rrdy5_i;

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
assign dis2_src0_val = dis2_src0_ready_i ? dis2_src0_val_i :
                       !dis_rat_rbusy4_i ? dis_arf_rdata4_i :
                       rob_rrdy4_i ? rob_rdata4_i : 32'b0;
// Pair-key-only values use the ROB read mux with ALU2's data branch removed.
// RS_MEM reintroduces a matching ALU2 value through a direct parallel adder.
`ifdef SOC_USE
assign dis0_src0_key_val = dis0_src0_ready_i ? dis0_src0_val_i :
                           !dis_rat_rbusy0_i ? dis_arf_rdata0_i :
                           rob_rrdy0_i ? rob_rdata0_noa2_i : 32'b0;
assign dis1_src0_key_val = dis1_src0_ready_i ? dis1_src0_val_i :
                           !dis_rat_rbusy2_i ? dis_arf_rdata2_i :
                           rob_rrdy2_i ? rob_rdata2_noa2_i : 32'b0;
assign dis2_src0_key_val = dis2_src0_ready_i ? dis2_src0_val_i :
                           !dis_rat_rbusy4_i ? dis_arf_rdata4_i :
                           rob_rrdy4_i ? rob_rdata4_noa2_i : 32'b0;
`else
// The RS_MEM write itself is already gated by the corresponding src0-ready
// signal.  Selecting ROB data unconditionally in this last branch is therefore
// bit/cycle equivalent, while keeping rob_rrdy out of the address carry chain.
assign dis0_src0_key_val = dis0_src0_ready_i ? dis0_src0_val_i :
                           !dis_rat_rbusy0_i ? dis_arf_rdata0_i :
                                              rob_rdata0_noa2_i;
assign dis1_src0_key_val = dis1_src0_ready_i ? dis1_src0_val_i :
                           !dis_rat_rbusy2_i ? dis_arf_rdata2_i :
                                              rob_rdata2_noa2_i;
assign dis2_src0_key_val = dis2_src0_ready_i ? dis2_src0_val_i :
                           !dis_rat_rbusy4_i ? dis_arf_rdata4_i :
                                              rob_rdata4_noa2_i;
`endif
assign dis2_src1_val = dis2_src1_ready_i ? dis2_src1_val_i :
                       !dis_rat_rbusy5_i ? dis_arf_rdata5_i :
                       rob_rrdy5_i ? rob_rdata5_i : 32'b0;

assign dis0_is_alu = dis0_valid_i && dis0_futype_i[`FU_ALU];
assign dis0_is_mem = dis0_valid_i && dis0_futype_i[`FU_MEM];
assign dis0_is_mdu = dis0_valid_i && dis0_futype_i[`FU_MDU];
assign dis1_is_alu = dis1_valid_i && dis1_futype_i[`FU_ALU];
assign dis1_is_mem = dis1_valid_i && dis1_futype_i[`FU_MEM];
assign dis1_is_mdu = dis1_valid_i && dis1_futype_i[`FU_MDU];
assign dis2_is_alu = dis2_valid_i && dis2_futype_i[`FU_ALU];
assign dis2_is_mem = dis2_valid_i && dis2_futype_i[`FU_MEM];
assign dis2_is_mdu = dis2_valid_i && dis2_futype_i[`FU_MDU];

assign two_alu = (dis0_is_alu && dis1_is_alu) ||
                 (dis0_is_alu && dis2_is_alu) ||
                 (dis1_is_alu && dis2_is_alu);
wire three_alu = dis0_is_alu && dis1_is_alu && dis2_is_alu;

// 双 ALU 且两站都有空：固定程序序 槽0→rs_alu0、槽1→rs_alu1（同拍各压一条）
wire dual_alu_ok = two_alu && !three_alu
                 && rs_alu0_can_accept_i && rs_alu1_can_accept_i;
wire triple_alu_ok = three_alu && rs_alu0_can_accept_i
                   && rs_alu1_can_accept_i && rs_alu2_can_accept_i;

assign single_alu_to_alu0 = rs_alu0_can_accept_i &&
                            (!rs_alu1_can_accept_i ||
                             (rs_alu0_occupancy_i <= rs_alu1_occupancy_i));

assign slot0_to_alu0 = dis0_is_alu && (three_alu ? 1'b1 :
                                       dual_alu_ok ? 1'b1 : single_alu_to_alu0);
assign slot0_to_alu1 = dis0_is_alu && !three_alu &&
                       (dual_alu_ok ? 1'b0 : !single_alu_to_alu0);
assign slot1_to_alu0 = dis1_is_alu && !three_alu &&
                       (dual_alu_ok ? !dis0_is_alu : single_alu_to_alu0);
assign slot1_to_alu1 = dis1_is_alu && (three_alu ? 1'b1 :
                                       dual_alu_ok ? dis0_is_alu : !single_alu_to_alu0);
assign slot2_to_alu0 = dis2_is_alu &&
                       !three_alu &&
                       (dual_alu_ok ? !(dis0_is_alu || dis1_is_alu) : single_alu_to_alu0);
assign slot2_to_alu1 = dis2_is_alu &&
                       !three_alu &&
                       (dual_alu_ok ?  (dis0_is_alu || dis1_is_alu) : !single_alu_to_alu0);
assign slot0_to_alu2 = 1'b0;
assign slot1_to_alu2 = 1'b0;
assign slot2_to_alu2 = three_alu;
assign slot0_to_mem = dis0_is_mem;
assign slot1_to_mem = dis1_is_mem;
assign slot2_to_mem = dis2_is_mem;
assign slot0_to_mdu = dis0_is_mdu;
assign slot1_to_mdu = dis1_is_mdu;
assign slot2_to_mdu = dis2_is_mdu;

// 入站不再要求源就绪：未齐则带 robid 进 RS，由 wakeup 补齐。
// 避免槽0 因相关/长延迟占住 dis，拖死 rename（dispatch_ready）。
assign dis0_will_take_alu0 = dis0_valid_i && dis0_is_alu && slot0_to_alu0;
assign dis0_will_take_alu1 = dis0_valid_i && dis0_is_alu && slot0_to_alu1;
assign dis0_will_take_alu2 = dis0_valid_i && dis0_is_alu && slot0_to_alu2;
assign dis0_will_take_mem  = dis0_valid_i && dis0_is_mem;
assign dis0_will_take_mdu  = dis0_valid_i && dis0_is_mdu;
assign dis1_will_take_alu0 = dis1_valid_i && dis1_is_alu && slot1_to_alu0;
assign dis1_will_take_alu1 = dis1_valid_i && dis1_is_alu && slot1_to_alu1;
assign dis1_will_take_alu2 = dis1_valid_i && dis1_is_alu && slot1_to_alu2;
assign dis1_will_take_mem  = dis1_valid_i && dis1_is_mem;
assign dis1_will_take_mdu  = dis1_valid_i && dis1_is_mdu;

assign dis0_rs_ok = dis0_is_alu ? (slot0_to_alu0 ? rs_alu0_can_accept_i :
                                   slot0_to_alu1 ? rs_alu1_can_accept_i : rs_alu2_can_accept_i) :
                    dis0_is_mem ? rs_mem_can_accept_i :
                    dis0_is_mdu ? rs_mdu_can_accept_i :
                    1'b0;
assign dis1_rs_ok = dis1_is_alu ? (slot1_to_alu0 ? (rs_alu0_can_accept_i && !dis0_will_take_alu0) :
                                   slot1_to_alu1 ? (rs_alu1_can_accept_i && !dis0_will_take_alu1) :
                                                   (rs_alu2_can_accept_i && !dis0_will_take_alu2)) :
                    dis1_is_mem ? (dis0_will_take_mem ? rs_mem_can_accept_two_i
                                                       : rs_mem_can_accept_i) :
                    dis1_is_mdu ? (rs_mdu_can_accept_i && !dis0_will_take_mdu) :
                    1'b0;
assign dis2_rs_ok = dis2_is_alu ?
                        (slot2_to_alu0 ? (rs_alu0_can_accept_i && !dis0_will_take_alu0 && !dis1_will_take_alu0)
                         : slot2_to_alu1 ? (rs_alu1_can_accept_i && !dis0_will_take_alu1 && !dis1_will_take_alu1)
                         : (rs_alu2_can_accept_i && !dis0_will_take_alu2 && !dis1_will_take_alu2)) :
                    dis2_is_mem ? ((dis0_will_take_mem || dis1_will_take_mem)
                                   ? rs_mem_can_accept_two_i : rs_mem_can_accept_i) :
                    dis2_is_mdu ? (rs_mdu_can_accept_i && !dis0_will_take_mdu && !dis1_will_take_mdu) :
                    1'b0;

assign dis0_can_dispatch = dis0_valid_i && dis0_rs_ok;
assign dis1_can_dispatch = dis1_valid_i && dis1_rs_ok;
assign dis2_can_dispatch = dis2_valid_i && dis2_rs_ok;

assign dis0_fire_o = dis0_can_dispatch;
// 禁止槽1越过未发出的槽0；允许与 dis0 同拍双发（如 dual_alu_ok → 两站各一条）
assign dis1_fire_o = dis1_can_dispatch && (!dis0_valid_i || dis0_fire_o);
assign dis2_fire_o = dis2_can_dispatch
                   && (!dis0_valid_i || dis0_fire_o)
                   && (!dis1_valid_i || dis1_fire_o);
// 槽已空，或本拍所有 valid 槽都会 fire：下沿 rename 可覆盖装入新一对。
// RS push 仍使用覆盖前的当前锁存值，因此可以同拍完成 vacate + refill，
// 消除 dispatch 对 rename 固有的隔拍阻塞。
assign dispatch_ready_o =
    (!dis0_valid_i || dis0_fire_o) &&
    (!dis1_valid_i || dis1_fire_o) &&
    (!dis2_valid_i || dis2_fire_o);

wire rs_alu0_from_slot1 = slot1_to_alu0 && dis1_fire_o;
wire rs_alu1_from_slot1 = slot1_to_alu1 && dis1_fire_o;
wire rs_mem_from_slot1  = slot1_to_mem && dis1_fire_o;
wire rs_mdu_from_slot1  = slot1_to_mdu && dis1_fire_o;
wire rs_alu0_from_slot2 = slot2_to_alu0 && dis2_fire_o;
wire rs_alu1_from_slot2 = slot2_to_alu1 && dis2_fire_o;
wire rs_mem_from_slot2  = slot2_to_mem && dis2_fire_o;
wire rs_mdu_from_slot2  = slot2_to_mdu && dis2_fire_o;

assign rs_alu0_push_valid_o = (slot0_to_alu0 && dis0_fire_o)
                           || (slot1_to_alu0 && dis1_fire_o)
                           || (slot2_to_alu0 && dis2_fire_o);
assign rs_alu0_push_robid_o = rs_alu0_from_slot2 ? dis2_robid_i :
                              rs_alu0_from_slot1 ? dis1_robid_i : dis0_robid_i;
assign rs_alu0_push_pc_o = rs_alu0_from_slot2 ? dis2_pc_i :
                           rs_alu0_from_slot1 ? dis1_pc_i : dis0_pc_i;
assign rs_alu0_push_alu_op_o = rs_alu0_from_slot2
                             ? {dis2_alu_op_i[21:19], dis2_alu_op_i[11:0]}
                             : rs_alu0_from_slot1
                             ? {dis1_alu_op_i[21:19], dis1_alu_op_i[11:0]}
                             : {dis0_alu_op_i[21:19], dis0_alu_op_i[11:0]};
assign rs_alu0_push_br_op_o = rs_alu0_from_slot2 ? dis2_br_op_i : rs_alu0_from_slot1 ? dis1_br_op_i : dis0_br_op_i;
assign rs_alu0_push_src0_ready_o = rs_alu0_from_slot2 ? dis2_src0_ready : rs_alu0_from_slot1 ? dis1_src0_ready : dis0_src0_ready;
assign rs_alu0_push_src0_val_o = rs_alu0_from_slot2 ? dis2_src0_val : rs_alu0_from_slot1 ? dis1_src0_val : dis0_src0_val;
assign rs_alu0_push_src0_robid_o = rs_alu0_from_slot2 ? dis2_src0_robid_i : rs_alu0_from_slot1 ? dis1_src0_robid_i : dis0_src0_robid_i;
assign rs_alu0_push_src1_ready_o = rs_alu0_from_slot2 ? dis2_src1_ready : rs_alu0_from_slot1 ? dis1_src1_ready : dis0_src1_ready;
assign rs_alu0_push_src1_val_o = rs_alu0_from_slot2 ? dis2_src1_val : rs_alu0_from_slot1 ? dis1_src1_val : dis0_src1_val;
assign rs_alu0_push_src1_robid_o = rs_alu0_from_slot2 ? dis2_src1_robid_i : rs_alu0_from_slot1 ? dis1_src1_robid_i : dis0_src1_robid_i;
assign rs_alu0_push_imm_o = rs_alu0_from_slot2 ? dis2_imm_i : rs_alu0_from_slot1 ? dis1_imm_i : dis0_imm_i;
assign rs_alu0_push_use_imm_o = rs_alu0_from_slot2 ? dis2_use_imm_i : rs_alu0_from_slot1 ? dis1_use_imm_i : dis0_use_imm_i;
assign rs_alu0_push_br_offs_o = rs_alu0_from_slot2 ? dis2_br_offs_i : rs_alu0_from_slot1 ? dis1_br_offs_i : dis0_br_offs_i;

assign rs_alu1_push_valid_o = (slot0_to_alu1 && dis0_fire_o)
                           || (slot1_to_alu1 && dis1_fire_o)
                           || (slot2_to_alu1 && dis2_fire_o);
assign rs_alu1_push_robid_o = rs_alu1_from_slot2 ? dis2_robid_i : rs_alu1_from_slot1 ? dis1_robid_i : dis0_robid_i;
assign rs_alu1_push_pc_o = rs_alu1_from_slot2 ? dis2_pc_i : rs_alu1_from_slot1 ? dis1_pc_i : dis0_pc_i;
assign rs_alu1_push_alu_op_o = rs_alu1_from_slot2
                             ? {dis2_alu_op_i[21:19], dis2_alu_op_i[11:0]}
                             : rs_alu1_from_slot1
                             ? {dis1_alu_op_i[21:19], dis1_alu_op_i[11:0]}
                             : {dis0_alu_op_i[21:19], dis0_alu_op_i[11:0]};
assign rs_alu1_push_br_op_o = rs_alu1_from_slot2 ? dis2_br_op_i : rs_alu1_from_slot1 ? dis1_br_op_i : dis0_br_op_i;
assign rs_alu1_push_src0_ready_o = rs_alu1_from_slot2 ? dis2_src0_ready : rs_alu1_from_slot1 ? dis1_src0_ready : dis0_src0_ready;
assign rs_alu1_push_src0_val_o = rs_alu1_from_slot2 ? dis2_src0_val : rs_alu1_from_slot1 ? dis1_src0_val : dis0_src0_val;
assign rs_alu1_push_src0_robid_o = rs_alu1_from_slot2 ? dis2_src0_robid_i : rs_alu1_from_slot1 ? dis1_src0_robid_i : dis0_src0_robid_i;
assign rs_alu1_push_src1_ready_o = rs_alu1_from_slot2 ? dis2_src1_ready : rs_alu1_from_slot1 ? dis1_src1_ready : dis0_src1_ready;
assign rs_alu1_push_src1_val_o = rs_alu1_from_slot2 ? dis2_src1_val : rs_alu1_from_slot1 ? dis1_src1_val : dis0_src1_val;
assign rs_alu1_push_src1_robid_o = rs_alu1_from_slot2 ? dis2_src1_robid_i : rs_alu1_from_slot1 ? dis1_src1_robid_i : dis0_src1_robid_i;
assign rs_alu1_push_imm_o = rs_alu1_from_slot2 ? dis2_imm_i : rs_alu1_from_slot1 ? dis1_imm_i : dis0_imm_i;
assign rs_alu1_push_use_imm_o = rs_alu1_from_slot2 ? dis2_use_imm_i : rs_alu1_from_slot1 ? dis1_use_imm_i : dis0_use_imm_i;
assign rs_alu1_push_br_offs_o = rs_alu1_from_slot2 ? dis2_br_offs_i : rs_alu1_from_slot1 ? dis1_br_offs_i : dis0_br_offs_i;

// Keep the common one/two-ALU routing unchanged.  ALU2 is dedicated to the
// youngest instruction of an all-ALU triple, making the added timing cone
// local to the new path.
assign rs_alu2_push_valid_o = slot2_to_alu2 && dis2_fire_o && triple_alu_ok;
assign rs_alu2_push_robid_o = dis2_robid_i;
assign rs_alu2_push_pc_o = dis2_pc_i;
assign rs_alu2_push_alu_op_o = {dis2_alu_op_i[21:19], dis2_alu_op_i[11:0]};
assign rs_alu2_push_br_op_o = dis2_br_op_i;
assign rs_alu2_push_src0_ready_o = dis2_src0_ready;
assign rs_alu2_push_src0_val_o = dis2_src0_val;
assign rs_alu2_push_src0_robid_o = dis2_src0_robid_i;
assign rs_alu2_push_src1_ready_o = dis2_src1_ready;
assign rs_alu2_push_src1_val_o = dis2_src1_val;
assign rs_alu2_push_src1_robid_o = dis2_src1_robid_i;
assign rs_alu2_push_imm_o = dis2_imm_i;
assign rs_alu2_push_use_imm_o = dis2_use_imm_i;
assign rs_alu2_push_br_offs_o = dis2_br_offs_i;

// Port 0 always carries the oldest MEM instruction.  Port 1 is active only
// for a MEM/MEM pair, so FIFO insertion order is unambiguous.
wire rs_mem_single_from_slot1 = rs_mem_from_slot1 && !dis0_will_take_mem;
wire rs_mem_single_from_slot2 = rs_mem_from_slot2
                              && !dis0_will_take_mem && !dis1_will_take_mem;
assign rs_mem_push_valid_o = (slot0_to_mem && dis0_fire_o)
                          || (slot1_to_mem && dis1_fire_o && !dis0_will_take_mem)
                          || (slot2_to_mem && dis2_fire_o && !dis0_will_take_mem && !dis1_will_take_mem);
assign rs_mem_push_robid_o = rs_mem_single_from_slot2 ? dis2_robid_i : rs_mem_single_from_slot1 ? dis1_robid_i : dis0_robid_i;
assign rs_mem_push_mem_op_o = rs_mem_single_from_slot2 ? dis2_mem_op_i : rs_mem_single_from_slot1 ? dis1_mem_op_i : dis0_mem_op_i;
assign rs_mem_push_is_cacop_o = rs_mem_single_from_slot2 ? dis2_is_cacop_i : rs_mem_single_from_slot1 ? dis1_is_cacop_i : dis0_is_cacop_i;
assign rs_mem_push_cacop_op_o = rs_mem_single_from_slot2 ? dis2_cacop_op_i : rs_mem_single_from_slot1 ? dis1_cacop_op_i : dis0_cacop_op_i;
assign rs_mem_push_src0_ready_o = rs_mem_single_from_slot2 ? dis2_src0_ready : rs_mem_single_from_slot1 ? dis1_src0_ready : dis0_src0_ready;
assign rs_mem_push_src0_val_o = rs_mem_single_from_slot2 ? dis2_src0_val : rs_mem_single_from_slot1 ? dis1_src0_val : dis0_src0_val;
assign rs_mem_push_src0_robid_o = rs_mem_single_from_slot2 ? dis2_src0_robid_i : rs_mem_single_from_slot1 ? dis1_src0_robid_i : dis0_src0_robid_i;
assign rs_mem_push_src0_key_val_o = rs_mem_single_from_slot2 ? dis2_src0_key_val
                                      : rs_mem_single_from_slot1 ? dis1_src0_key_val
                                                                 : dis0_src0_key_val;
assign rs_mem_push_src0_rob_wait_o = rs_mem_single_from_slot2
                                   ? (!dis2_src0_ready_i && dis_rat_rbusy4_i)
                                   : rs_mem_single_from_slot1
                                   ? (!dis1_src0_ready_i && dis_rat_rbusy2_i)
                                   : (!dis0_src0_ready_i && dis_rat_rbusy0_i);
assign rs_mem_push_src1_ready_o = rs_mem_single_from_slot2 ? dis2_src1_ready : rs_mem_single_from_slot1 ? dis1_src1_ready : dis0_src1_ready;
assign rs_mem_push_src1_val_o = rs_mem_single_from_slot2 ? dis2_src1_val : rs_mem_single_from_slot1 ? dis1_src1_val : dis0_src1_val;
assign rs_mem_push_src1_robid_o = rs_mem_single_from_slot2 ? dis2_src1_robid_i : rs_mem_single_from_slot1 ? dis1_src1_robid_i : dis0_src1_robid_i;
assign rs_mem_push_imm_o = rs_mem_single_from_slot2 ? dis2_imm_i : rs_mem_single_from_slot1 ? dis1_imm_i : dis0_imm_i;

wire rs_mem_second_from_slot2 = rs_mem_from_slot2
                              && (dis0_will_take_mem || dis1_will_take_mem);
assign rs_mem_push1_valid_o = (slot1_to_mem && dis1_fire_o && dis0_will_take_mem)
                            || rs_mem_second_from_slot2;
assign rs_mem_push1_robid_o = rs_mem_second_from_slot2 ? dis2_robid_i : dis1_robid_i;
assign rs_mem_push1_mem_op_o = rs_mem_second_from_slot2 ? dis2_mem_op_i : dis1_mem_op_i;
assign rs_mem_push1_is_cacop_o = rs_mem_second_from_slot2 ? dis2_is_cacop_i : dis1_is_cacop_i;
assign rs_mem_push1_cacop_op_o = rs_mem_second_from_slot2 ? dis2_cacop_op_i : dis1_cacop_op_i;
assign rs_mem_push1_src0_ready_o = rs_mem_second_from_slot2 ? dis2_src0_ready : dis1_src0_ready;
assign rs_mem_push1_src0_val_o = rs_mem_second_from_slot2 ? dis2_src0_val : dis1_src0_val;
assign rs_mem_push1_src0_robid_o = rs_mem_second_from_slot2 ? dis2_src0_robid_i : dis1_src0_robid_i;
assign rs_mem_push1_src0_key_val_o = rs_mem_second_from_slot2
                                   ? dis2_src0_key_val : dis1_src0_key_val;
assign rs_mem_push1_src0_rob_wait_o = rs_mem_second_from_slot2
                                    ? (!dis2_src0_ready_i && dis_rat_rbusy4_i)
                                    : (!dis1_src0_ready_i && dis_rat_rbusy2_i);
assign rs_mem_push1_src1_ready_o = rs_mem_second_from_slot2 ? dis2_src1_ready : dis1_src1_ready;
assign rs_mem_push1_src1_val_o = rs_mem_second_from_slot2 ? dis2_src1_val : dis1_src1_val;
assign rs_mem_push1_src1_robid_o = rs_mem_second_from_slot2 ? dis2_src1_robid_i : dis1_src1_robid_i;
assign rs_mem_push1_imm_o = rs_mem_second_from_slot2 ? dis2_imm_i : dis1_imm_i;

assign rs_mdu_push_valid_o = (slot0_to_mdu && dis0_fire_o)
                           || (slot1_to_mdu && dis1_fire_o)
                           || (slot2_to_mdu && dis2_fire_o);
assign rs_mdu_push_robid_o = rs_mdu_from_slot2 ? dis2_robid_i : rs_mdu_from_slot1 ? dis1_robid_i : dis0_robid_i;
assign rs_mdu_push_alu_op_o = rs_mdu_from_slot2 ? dis2_alu_op_i[18:12] : rs_mdu_from_slot1 ? dis1_alu_op_i[18:12] : dis0_alu_op_i[18:12];
assign rs_mdu_push_csr_op_o = rs_mdu_from_slot2 ? dis2_csr_op_i : rs_mdu_from_slot1 ? dis1_csr_op_i : dis0_csr_op_i;
assign rs_mdu_push_csr_num_o = rs_mdu_from_slot2 ? dis2_csr_num_i : rs_mdu_from_slot1 ? dis1_csr_num_i : dis0_csr_num_i;
assign rs_mdu_push_tlb_op_o = rs_mdu_from_slot2 ? dis2_tlb_op_i : rs_mdu_from_slot1 ? dis1_tlb_op_i : dis0_tlb_op_i;
assign rs_mdu_push_wb_src_op_o = rs_mdu_from_slot2 ? dis2_wb_src_op_i : rs_mdu_from_slot1 ? dis1_wb_src_op_i : dis0_wb_src_op_i;
assign rs_mdu_push_src0_ready_o = rs_mdu_from_slot2 ? dis2_src0_ready : rs_mdu_from_slot1 ? dis1_src0_ready : dis0_src0_ready;
assign rs_mdu_push_src0_val_o = rs_mdu_from_slot2 ? dis2_src0_val : rs_mdu_from_slot1 ? dis1_src0_val : dis0_src0_val;
assign rs_mdu_push_src0_robid_o = rs_mdu_from_slot2 ? dis2_src0_robid_i : rs_mdu_from_slot1 ? dis1_src0_robid_i : dis0_src0_robid_i;
assign rs_mdu_push_src1_ready_o = rs_mdu_from_slot2 ? dis2_src1_ready : rs_mdu_from_slot1 ? dis1_src1_ready : dis0_src1_ready;
assign rs_mdu_push_src1_val_o = rs_mdu_from_slot2 ? dis2_src1_val : rs_mdu_from_slot1 ? dis1_src1_val : dis0_src1_val;
assign rs_mdu_push_src1_robid_o = rs_mdu_from_slot2 ? dis2_src1_robid_i : rs_mdu_from_slot1 ? dis1_src1_robid_i : dis0_src1_robid_i;

endmodule
