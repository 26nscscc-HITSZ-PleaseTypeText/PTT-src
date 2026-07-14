// ============================================================
// lsu 模块（访存单元：AGU + DC 两级流水 + miss 槽非阻塞 load）
// ------------------------------------------------------------
// 结构：
// - AGU 级：vaddr=base+imm -> MMU 组合翻译 -> ALE/ADEM/TLB 异常 ->
//   store 数据按地址对齐 + wstrb 生成；
// - DC 级：store/cacop/异常直接写回；load 先查 SB 前递，再访 DCache；
// - miss 槽（二期·非阻塞 miss，配合 dcache 的 1 项 MSHR）：
//   * cached load 在 DCache LOOKUP 拍报 miss（dc_miss_i）时，其
//     {robid, mem_op, 地址} 移入 miss 槽，DC 级立即释放——后续 load/store
//     继续发射执行（hit-under-miss）；
//   * 重填数据经 dc_mshr_data_ok_i/dc_mshr_rdata_i 独立通道返回
//     （CWF-lite：目标字所在半行一到即回，见 dcache.v），当拍整形写回；
//   * dcache 只有 1 项 MSHR：第二个 miss 的通知必然出现在前一个重填
//     数据返回之后（安装->同组等待->重查，间隔 >=3 拍），槽到时必空；
// - 写回端口仲裁（ROB 只有一个 LSU 写回口，按年龄优先）：
//   miss 槽（最老）> 暂存槽 hold > DC 级当前完成；
//   DC 级的瞬态完成源（DCache data_ok / SB 前递命中）被高优先级抢口时
//   捕获进 hold 暂存槽，次拍补写——数据不丢；hold 占用期间 DC 级
//   不发起新完成（dc_req/终查门控），保证暂存深度 1 足够。
//
// 顺序保护（关键正确性逻辑，继承参考实现）：
// - store 经 LSU 写回后要等提交才入 SB，该窗口内更年轻 load 既查不到 SB
//   也不能读 DCache（会读到旧值）。处理：记录最近一条 store 的 robid
//   （in-order LSU 下它是所有未决 store 中最年轻者），load 在它提交前
//   不发起最终 SB 查询/DCache 访问。
//   提交判定用 rob.head_robid0_o 的约定编码：
//   [`ROB_W-1]=队头槽0是否未提交，低位=head 对指针；
//   robid 对距离 d=(R-head)&15，d>=12 视为已被 head 越过（已提交）。
// - miss 槽内是 load，不影响 store 序；同行 load/store 的次序由 dcache
//   的"同组撞 MSHR 等待"规则保证（见 dcache.v）。
//
// uncached load：等到自己成为最老未提交指令才发（同上编码判定），期间
// uncached_ld_inflight_o 置位（commit 屏蔽中断附着）；SB 中尚有 uncached
// store 未排空时 sb_query_partial_i 恒 1（配合 sb_query_uncached_o），
// 保证设备"先写后读"。
//
// 冲刷（flush_i）：
// - 清 AGU/DC 两级与 hold 槽；
// - 在途 DCache 请求用 d_drop 丢弃下一个响应——响应可能是 data_ok（丢弃
//   即可）也可能是 miss 通知（此时仍要占用 miss 槽收下重填数据再丢弃，
//   置 m_drop）；miss 槽中未返回的 load 同样置 m_drop；
// - dcache 按契约对每个被接受的请求恰好回一次响应，绝无静默丢包，
//   因此 drop 配对不会死锁。
// ============================================================
`include "mycpu.h"

module lsu(
    input  wire                       clk,
    input  wire                       reset,
    input  wire                       flush_i,

    // ---------------- 发射入口（来自 rs_mem）----------------
    input  wire                       issue_valid_i,
    input  wire [`ROB_W-1:0]          issue_robid_i,
    input  wire [31:0]                issue_pc_i,
    input  wire [`MEM_OP_NUM-1:0]     issue_mem_op_i,
    input  wire                       issue_is_cacop_i,
    input  wire [31:0]                issue_base_i,        // rj 值
    input  wire [31:0]                issue_wdata_i,       // rd 值（store 数据）
    input  wire [31:0]                issue_imm_i,         // 偏移
    output wire                       lsu_ready_o,         // AGU 级可接收

    // ---------------- D 侧地址翻译（连 mmu D 通道，组合）----------------
    output wire                       mmu_d_req_o,
    output wire [31:0]                mmu_d_vaddr_o,
    output wire                       mmu_d_is_store_o,    // 区分 PIL/PIS 与 PME
    input  wire [31:0]                mmu_d_paddr_i,
    input  wire [1:0]                 mmu_d_mat_i,
    input  wire [`TLB_EX_NUM-1:0]     mmu_d_tlb_ex_i,      // TLBR/PIL/PIS/PPI/PME
    input  wire                       mmu_d_excp_adem_i,

    // ---------------- DCache load 访问口 ----------------
    output wire                       dc_req_o,            // load 请求（保持至 addr_ok）
    output wire [31:0]                dc_vaddr_o,          // 虚地址（VIPT 索引）
    output wire [31:0]                dc_paddr_o,          // 物理地址（tag 比对）
    output wire [2:0]                 dc_size_o,           // 0=B 1=H 2=W
    output wire                       dc_uncached_o,
    input  wire                       dc_addr_ok_i,
    input  wire                       dc_data_ok_i,
    input  wire [31:0]                dc_rdata_i,
    output wire                       dc_cancel_o,         // 冲刷提示（dcache 按契约忽略）
    // ---- 非阻塞 miss 扩展（配合 dcache MSHR）----
    input  wire                       dc_miss_i,           // 在途 load 移入 MSHR（一拍）
    input  wire                       dc_mshr_data_ok_i,   // MSHR 重填数据返回（一拍）
    input  wire [31:0]                dc_mshr_rdata_i,

    // ---------------- store buffer 前递查询（DC 级组合）----------------
    output wire [31:0]                sb_query_paddr_o,
    output wire                       sb_query_uncached_o, // 本查询来自 uncached load
    input  wire                       sb_query_hit_i,      // 整字可由 SB 合并前递
    input  wire [31:0]                sb_query_data_i,
    input  wire                       sb_query_partial_i,  // 部分命中：load 等排空重试

    // ---------------- uncached load 许可（与 ROB head 比较）----------------
    input  wire [`ROB_W-1:0]          rob_head_robid_i,    // 编码约定见头注
    input  wire                       rob_head_valid_i,
    output wire                       uncached_ld_inflight_o, // 有 uncached load 在飞（commit 屏蔽中断用）

    // ---------------- 写回 ROB ----------------
    output wire                       wb_valid_o,
    output wire [`ROB_W-1:0]          wb_robid_o,
    output wire [31:0]                wb_data_o,           // load 整形后数据 / store 写数据
    output wire [31:0]                wb_paddr_o,          // 访存物理地址（store/cacop/difftest 用）
    output wire [31:0]                wb_vaddr_o,          // 访存虚地址（BADV/difftest 用）
    output wire [3:0]                 wb_wstrb_o,          // store 字节使能（已按地址对齐移位）
    output wire [2:0]                 wb_size_o,           // 访问宽度
    output wire                       wb_uncached_o,       // 非缓存访问
    output wire [`EXCP_NUM-1:0]       wb_excp_o,           // 动态异常（ALE/ADEM/TLBR_M/PIL/PIS/PPI_M/PME）

    // ---------------- AGU 级投机唤醒（二期预留，暂恒 0）----------------
    output wire                       early_wakeup_valid_o,
    output wire [`ROB_W-1:0]          early_wakeup_robid_o
);

// =====================================================================
// AGU 级
// =====================================================================
reg                    a_valid;
reg [`ROB_W-1:0]       a_robid;
reg [`MEM_OP_NUM-1:0]  a_mem_op;
reg                    a_is_cacop;
reg [31:0]             a_base, a_wdata, a_imm;

wire [31:0] a_vaddr = a_base + a_imm;

wire a_is_store_op = a_mem_op[`MEM_OP_ST_W] | a_mem_op[`MEM_OP_ST_B]
                   | a_mem_op[`MEM_OP_ST_H] | a_mem_op[`MEM_OP_SC_W];
wire a_is_load_op  = a_mem_op[`MEM_OP_LD_W] | a_mem_op[`MEM_OP_LD_B]
                   | a_mem_op[`MEM_OP_LD_H] | a_mem_op[`MEM_OP_LD_BU]
                   | a_mem_op[`MEM_OP_LD_HU]| a_mem_op[`MEM_OP_LL_W];

// MMU 翻译（组合）
assign mmu_d_req_o      = a_valid;
assign mmu_d_vaddr_o    = a_vaddr;
assign mmu_d_is_store_o = a_is_store_op;

// LoongArch MAT：2'b01=coherent cached，其余按 uncached 访问
wire a_uncached = (mmu_d_mat_i != 2'b01);

// ALE 检测（H 类要求 vaddr[0]==0，W 类要求 vaddr[1:0]==00）
wire a_size_h = a_mem_op[`MEM_OP_LD_H] | a_mem_op[`MEM_OP_LD_HU] | a_mem_op[`MEM_OP_ST_H];
wire a_size_w = a_mem_op[`MEM_OP_LD_W] | a_mem_op[`MEM_OP_ST_W]
              | a_mem_op[`MEM_OP_LL_W] | a_mem_op[`MEM_OP_SC_W];
wire a_ale = (a_size_h && (a_vaddr[0] != 1'b0))
           | (a_size_w && (a_vaddr[1:0] != 2'b00));

// 异常合并（cacop 与访存共用翻译异常通路）
wire [`EXCP_NUM-1:0] a_excp =
      ({{(`EXCP_NUM-1){1'b0}}, a_ale}                          << `EXCP_ALE)
    | ({{(`EXCP_NUM-1){1'b0}}, mmu_d_excp_adem_i}              << `EXCP_ADEM)
    | ({{(`EXCP_NUM-1){1'b0}}, mmu_d_tlb_ex_i[`TLB_EX_TLBR]}   << `EXCP_TLBR_M)
    | ({{(`EXCP_NUM-1){1'b0}}, mmu_d_tlb_ex_i[`TLB_EX_PIL]}    << `EXCP_PIL)
    | ({{(`EXCP_NUM-1){1'b0}}, mmu_d_tlb_ex_i[`TLB_EX_PIS]}    << `EXCP_PIS)
    | ({{(`EXCP_NUM-1){1'b0}}, mmu_d_tlb_ex_i[`TLB_EX_PPI]}    << `EXCP_PPI_M)
    | ({{(`EXCP_NUM-1){1'b0}}, mmu_d_tlb_ex_i[`TLB_EX_PME]}    << `EXCP_PME);

// store 数据按地址对齐 + wstrb
wire [1:0] a_off = a_vaddr[1:0];
reg [31:0] a_st_data;
reg [3:0]  a_st_strb;
always @(*) begin
    if (a_mem_op[`MEM_OP_ST_B]) begin
        a_st_data = {4{a_wdata[7:0]}};
        a_st_strb = 4'b0001 << a_off;
    end else if (a_mem_op[`MEM_OP_ST_H]) begin
        a_st_data = {2{a_wdata[15:0]}};
        a_st_strb = a_off[1] ? 4'b1100 : 4'b0011;
    end else begin
        a_st_data = a_wdata;
        a_st_strb = 4'b1111;
    end
end

wire [2:0] a_size = a_mem_op[`MEM_OP_ST_B] | a_mem_op[`MEM_OP_LD_B] | a_mem_op[`MEM_OP_LD_BU] ? 3'd0
                  : a_mem_op[`MEM_OP_ST_H] | a_mem_op[`MEM_OP_LD_H] | a_mem_op[`MEM_OP_LD_HU] ? 3'd1
                  : 3'd2;

// =====================================================================
// DC 级
// =====================================================================
reg                    d_valid;
reg [`ROB_W-1:0]       d_robid;
reg [`MEM_OP_NUM-1:0]  d_mem_op;
reg                    d_is_cacop;
reg                    d_is_store, d_is_load;
reg [31:0]             d_vaddr, d_paddr;
reg [31:0]             d_st_data;
reg [3:0]              d_st_strb;
reg [2:0]              d_size;
reg                    d_uncached;
reg [`EXCP_NUM-1:0]    d_excp;
reg                    d_req_sent;      // DCache 已收下请求，等响应（data_ok 或 miss）
reg                    d_drop;          // 冲刷后丢弃下一个前端响应

// ---------------- miss 槽（MSHR 影子，深度 1）----------------
reg                    m_valid;
reg                    m_drop;          // 冲刷：重填数据到达时静默丢弃
reg [`ROB_W-1:0]       m_robid;
reg [`MEM_OP_NUM-1:0]  m_mem_op;
reg [31:0]             m_vaddr, m_paddr;
reg [2:0]              m_size;

// ---------------- 写回暂存槽（被高优先级抢口的瞬态完成）----------------
reg                    h_valid;
reg [`ROB_W-1:0]       h_robid;
reg [31:0]             h_data;          // 已整形
reg [31:0]             h_vaddr, h_paddr;
reg [2:0]              h_size;

// ---------------- 顺序保护：最近一条未决 store ----------------
reg                  st_pend;           // 有 store 写回后尚未确认提交
reg [`ROB_W-1:0]     st_pend_robid;

wire [`ROB_PAIR_W-1:0] head_pair   = rob_head_robid_i[`ROB_PAIR_W-1:0];
wire                   head_s0_live= rob_head_robid_i[`ROB_W-1];

// robid R 已提交判定（约定编码，见头注）——内联为纯组合 wire。
// 【勿改回 function】：xsim 对"连续赋值里带可变下标/依赖模块变量的 function 调用"存在
// 求值缺陷(见 rob.v ROB_RDPORT 同款坑)，会返回陈旧结果 → st_pend_clear 永为 0 →
// store_order_block 恒 1 → uncached load 永不放行(n50 device 先写后读死锁)。
wire [`ROB_PAIR_W-1:0] stpend_d = st_pend_robid[`ROB_PAIR_W-1:0] - head_pair;
wire stpend_committed = (stpend_d >= `ROB_PAIR_W'd12)                          // head 已越过该对
                      || ((stpend_d == {`ROB_PAIR_W{1'b0}})
                          && (st_pend_robid[`ROB_W-1] == 1'b0) && !head_s0_live); // 槽0已提交
wire st_pend_clear = st_pend && (!rob_head_valid_i || stpend_committed);
wire store_order_block = st_pend && !st_pend_clear;     // 仍有未决 store

// ---------------- DC 级行为 ----------------
wire d_excp_any = |d_excp;

// SB 前递查询（DC 级持续驱动）
assign sb_query_paddr_o    = d_paddr;
assign sb_query_uncached_o = d_valid && d_is_load && d_uncached;

// uncached load 许可：自己是最老未提交指令
wire d_at_head = (d_robid[`ROB_PAIR_W-1:0] == head_pair)
              && ((d_robid[`ROB_W-1] == 1'b0) || !head_s0_live)
              && rob_head_valid_i;

wire d_is_unc_load = d_valid && d_is_load && d_uncached && !d_excp_any;
assign uncached_ld_inflight_o = d_is_unc_load;

// load 可以发起最终访问（SB 终查/DCache）的条件：
// - 未决 store 先提交（顺序保护）；
// - hold 槽空（防瞬态完成三方碰撞，见头注仲裁说明）；
// - uncached load 额外等：到 ROB 头 + SB 无 uncached 残留（query_partial）
wire d_ld_gate = !store_order_block && !h_valid
              && (!d_uncached || (d_at_head && !sb_query_partial_i));

// load 处理分支
wire d_sb_hit     = d_valid && d_is_load && !d_excp_any && !d_uncached && sb_query_hit_i;
wire d_sb_partial = d_valid && d_is_load && !d_excp_any && !d_uncached && sb_query_partial_i;

wire d_need_dc = d_valid && d_is_load && !d_excp_any && !d_sb_hit && !d_sb_partial;

// DCache 请求（保持至 addr_ok）
assign dc_req_o      = d_need_dc && d_ld_gate && !d_req_sent && !d_drop && !flush_i;
assign dc_vaddr_o    = d_vaddr;
assign dc_paddr_o    = d_paddr;
assign dc_size_o     = d_size;
assign dc_uncached_o = d_uncached;
assign dc_cancel_o   = flush_i;

wire dc_fire   = dc_req_o && dc_addr_ok_i;
// 前端响应二选一：数据返回（命中/uncached）或 miss 移交
wire dc_return = d_req_sent && dc_data_ok_i && !d_drop;
wire dc_return_drop = d_req_sent && dc_data_ok_i && d_drop;
wire dc_missed = d_req_sent && dc_miss_i;

// MSHR 重填返回（miss 槽配对；m_drop 时静默消费）
wire mshr_return      = m_valid && dc_mshr_data_ok_i && !m_drop;
wire mshr_return_drop = m_valid && dc_mshr_data_ok_i && m_drop;

// ---------------- load 数据整形 ----------------
function [31:0] shape_load;
    input [31:0] word;
    input [`MEM_OP_NUM-1:0] op;
    input [1:0] off;
    reg [7:0]  b;
    reg [15:0] h;
    begin
        b = word[8*off +: 8];
        h = off[1] ? word[31:16] : word[15:0];
        if (op[`MEM_OP_LD_B])       shape_load = {{24{b[7]}}, b};
        else if (op[`MEM_OP_LD_BU]) shape_load = {24'b0, b};
        else if (op[`MEM_OP_LD_H])  shape_load = {{16{h[15]}}, h};
        else if (op[`MEM_OP_LD_HU]) shape_load = {16'b0, h};
        else                        shape_load = word;
    end
endfunction

// ---------------- 写回仲裁（一拍一条，按年龄：miss 槽 > hold > DC 级）----------------
// DC 级完成源：
// 1) 异常：直接写回；2) store/cacop：直接写回；3) load：SB 命中或 DCache 返回
wire wb_mshr_case  = mshr_return;                       // 最老，最高优先
wire wb_hold_case  = !wb_mshr_case && h_valid;
wire dcst_ok       = !wb_mshr_case && !h_valid;         // DC 级静态源可用口
wire wb_excp_case  = dcst_ok && d_valid && d_excp_any;
wire wb_st_case    = dcst_ok && d_valid && !d_excp_any && (d_is_store || d_is_cacop);
// Phase F(100MHz 攻坚):SB 命中不再"同拍组合写回"。原关键路径
//   store_buffer/tail → 逐字节年龄优先前递合并(8 项)→ sb_query_data_i → shape_load
//   → wb_data_o → 旁路网络 → rs_alu/s1_val 捕获(实测 post-route 22ns/32 级,WNS 主凶)
// 一律经既有 hold 暂存槽打一拍:h_data 在 T 拍锁存 shape_load(sb_query_data_i),
// T+1 拍由 wb_hold_case 从寄存器写回。于是 h_data 这个 FF 把 22ns 长链在中点切成
//   (a) SB 合并 → h_data/D   与   (b) h_data/Q → wb → RS 捕获   两条 ~11ns 半链。
// 正确性:LSU 流水里的在途 load 恒比 commit 级 flush 触发指令年轻,flush 拍丢弃 hold
// 内容永远安全(与原 hold_cap_sb 依据同);SB 命中延后一拍写回只是延迟,无序问题
// (store→load 顺序由 store_order_block 在 sb_ready 前已保证)。SB 命中少见,IPC 影响微小。
wire sb_ready      = d_sb_hit && d_ld_gate && !store_order_block; // d_ld_gate 已含 !h_valid
wire wb_ld_sb_case = 1'b0;                               // SB 命中不再直接写回
wire wb_ld_dc_case = dcst_ok && dc_return;
// DC 级瞬态完成被抢口 -> 捕获进 hold（gate 保证 hold 此刻必空）
wire hold_cap_dc   = wb_mshr_case && dc_return;
// 所有就绪 SB 命中都进 hold(与 hold_cap_dc 互斥:同一 d 级 load 不会既 SB 命中又 DC 返回)
wire hold_cap_sb   = sb_ready && !hold_cap_dc;

assign wb_valid_o = (wb_mshr_case || wb_hold_case
                  || wb_excp_case || wb_st_case || wb_ld_sb_case || wb_ld_dc_case)
                  && !flush_i;
assign wb_robid_o = wb_mshr_case ? m_robid
                  : wb_hold_case ? h_robid
                  : d_robid;
assign wb_data_o  = wb_mshr_case  ? shape_load(dc_mshr_rdata_i, m_mem_op, m_vaddr[1:0])
                  : wb_hold_case  ? h_data
                  : wb_ld_sb_case ? shape_load(sb_query_data_i, d_mem_op, d_vaddr[1:0])
                  : wb_ld_dc_case ? shape_load(dc_rdata_i,      d_mem_op, d_vaddr[1:0])
                  : d_st_data;
assign wb_paddr_o = wb_mshr_case ? m_paddr : wb_hold_case ? h_paddr : d_paddr;
assign wb_vaddr_o = wb_mshr_case ? m_vaddr : wb_hold_case ? h_vaddr : d_vaddr;
assign wb_wstrb_o = (!wb_mshr_case && !wb_hold_case && d_is_store && !d_excp_any) ? d_st_strb : 4'b0;
assign wb_size_o  = wb_mshr_case ? m_size : wb_hold_case ? h_size : d_size;
assign wb_uncached_o = (!wb_mshr_case && !wb_hold_case) && d_uncached;
assign wb_excp_o  = (!wb_mshr_case && !wb_hold_case) ? d_excp : {`EXCP_NUM{1'b0}};

// ---------------- 流水推进 ----------------
// DC 级本拍腾空：写回成功 / 移入 miss 槽 / 被抢口但已捕获进 hold
wire d_done  = wb_excp_case || wb_st_case || wb_ld_sb_case || wb_ld_dc_case
             || dc_missed || dc_return_drop
             || hold_cap_dc || hold_cap_sb;
wire d_free  = !d_valid || d_done;
wire a_go    = a_valid && d_free;                // AGU -> DC
assign lsu_ready_o = (!a_valid || a_go) && !flush_i;

always @(posedge clk) begin
    if (reset) begin
        a_valid    <= 1'b0;
        d_valid    <= 1'b0;
        d_drop     <= 1'b0;
        d_req_sent <= 1'b0;
        st_pend    <= 1'b0;
        m_valid    <= 1'b0;
        m_drop     <= 1'b0;
        h_valid    <= 1'b0;
    end else if (flush_i) begin
        a_valid    <= 1'b0;
        d_valid    <= 1'b0;
        h_valid    <= 1'b0;      // 暂存的完成属于被冲刷指令，直接丢
        st_pend    <= 1'b0;
        // 在途 DCache 前端响应作废（已接受未响应的请求）
        d_drop     <= d_req_sent && !(dc_data_ok_i || dc_miss_i);
        d_req_sent <= 1'b0;
        // miss 槽：本拍恰好返回则消费完毕；仍在飞则置 drop 等配对
        if (dc_mshr_data_ok_i) begin
            m_valid <= 1'b0;
            m_drop  <= 1'b0;
        end else if (m_valid) begin
            m_drop  <= 1'b1;
        end
        // 冲刷拍在途请求恰报 miss：占槽收数、标记丢弃
        if (d_req_sent && dc_miss_i && !(m_valid && !dc_mshr_data_ok_i)) begin
            m_valid <= 1'b1;
            m_drop  <= 1'b1;
        end
    end else begin
        // ---- 前端 drop 配对（冲刷遗留的在途响应）----
        if (d_drop && (dc_data_ok_i || dc_miss_i)) begin
            d_drop <= 1'b0;
            if (dc_miss_i) begin
                // 被丢弃的 load 已进 MSHR：占槽等重填数据再静默丢弃
                m_valid <= 1'b1;
                m_drop  <= 1'b1;
            end
        end

        // ---- miss 槽 ----
        if (mshr_return || mshr_return_drop) begin
            m_valid <= 1'b0;
            m_drop  <= 1'b0;
        end
        if (dc_missed && !d_drop) begin
            m_valid  <= 1'b1;
            m_drop   <= 1'b0;
            m_robid  <= d_robid;
            m_mem_op <= d_mem_op;
            m_vaddr  <= d_vaddr;
            m_paddr  <= d_paddr;
            m_size   <= d_size;
        end

        // ---- hold 暂存槽 ----
        if (wb_hold_case) begin
            h_valid <= 1'b0;
        end
        if (hold_cap_dc || hold_cap_sb) begin
            h_valid <= 1'b1;
            h_robid <= d_robid;
            h_data  <= hold_cap_dc ? shape_load(dc_rdata_i,      d_mem_op, d_vaddr[1:0])
                                   : shape_load(sb_query_data_i, d_mem_op, d_vaddr[1:0]);
            h_vaddr <= d_vaddr;
            h_paddr <= d_paddr;
            h_size  <= d_size;
        end

        // ---- 顺序保护寄存器 ----
        if (wb_st_case && d_is_store) begin
            st_pend       <= 1'b1;
            st_pend_robid <= d_robid;
        end else if (st_pend_clear) begin
            st_pend <= 1'b0;
        end

        // ---- DC 级 ----
        if (d_done) d_valid <= 1'b0;
        if (dc_fire) d_req_sent <= 1'b1;
        if (dc_return || dc_return_drop || dc_missed) d_req_sent <= 1'b0;

        // ---- AGU -> DC ----
        if (a_go) begin
            d_valid    <= 1'b1;
            d_robid    <= a_robid;
            d_mem_op   <= a_mem_op;
            d_is_cacop <= a_is_cacop;
            d_is_store <= a_is_store_op && !a_is_cacop;
            d_is_load  <= a_is_load_op  && !a_is_cacop;
            d_vaddr    <= a_vaddr;
            d_paddr    <= mmu_d_paddr_i;
            d_st_data  <= a_st_data;
            d_st_strb  <= a_st_strb;
            d_size     <= a_size;
            d_uncached <= a_uncached;
            d_excp     <= a_excp;
            d_req_sent <= 1'b0;
        end
        if (a_go && !issue_valid_i) a_valid <= 1'b0;

        // ---- 发射 -> AGU ----
        if (issue_valid_i && lsu_ready_o) begin
            a_valid    <= 1'b1;
            a_robid    <= issue_robid_i;
            a_mem_op   <= issue_mem_op_i;
            a_is_cacop <= issue_is_cacop_i;
            a_base     <= issue_base_i;
            a_wdata    <= issue_wdata_i;
            a_imm      <= issue_imm_i;
        end
    end
end

// 二期预留（AGU 级投机唤醒），暂恒 0
assign early_wakeup_valid_o = 1'b0;
assign early_wakeup_robid_o = {`ROB_W{1'b0}};

// lint 吸收
wire lsu_lint = (|issue_pc_i) | (|mmu_d_mat_i[1:1]);

endmodule
