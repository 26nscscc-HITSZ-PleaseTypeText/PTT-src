`include "mycpu.h"

// ============================================================
// dcache 模块（L1 数据缓存，load/store 分离双口 + 非阻塞 miss）
// ------------------------------------------------------------
// 几何（原 TODO 第二步，按宏全量重写）：
// - `L1_NWAY(4) 路 × `L1_NSET(128) 组 × `CACHE_LINE_BYTES(32B) 行 = 16KB；
// - VIPT：index+offset = 12 位落在 4KB 页内偏移（vaddr/paddr 的 [11:5]
//   恒等），load 用 vaddr 取 index、tag 用 paddr 比对；
// - 写回法 + 写分配；数据阵列每路一块推断 BRAM（128×256b 整行写），
//   tag 用 LUTRAM（异步读）、valid/dirty 用触发器（一拍判定/更新）。
//
// 结构（原 TODO 第一步"真双口"的落地形态）：
// - 前端 FSM：一次锁存一个请求（store(SB) > load(LSU) > cacop 暂存），
//   IDLE 接受拍发 BRAM 读，LOOKUP 拍比对出结果——命中 load 两拍返回、
//   命中 store 两拍完成（合并写回阵列）；
// - 后台 MSHR（`DC_MSHR_DEPTH` 项，原 TODO 第四步·二期非阻塞 miss）：
//   * cached miss（load/store 皆可）分配进 MSHR 后【前端立即空闲】，
//     后续命中请求不受 miss 阻塞（hit-under-miss；配合 LSU 的 miss 槽）；
//   * load miss：LOOKUP 拍以 ld_miss_o 通知 LSU 移入 miss 槽，refill 数据
//     经 ld_mshr_data_ok_o/ld_mshr_rdata_o/ld_mshr_robid_o 独立通道返回；
//   * store miss：分配拍即回 st_done（posted，写效果由 MSHR 合并进重填行
//     保证落地），SB 立即排空下一条——隐藏 store miss 延迟；
//   * 同行 store 撞在飞 MSHR（同 paddr[31:5]）：合入该槽 byte enable，posted
//     st_done，不占 pend——消除 mem_stream 类 ld→st 同行走 set_conf；
//     **只改 stb/dat，不改 is_st/killed**（load 源 MSHR 仍可被 cancel 杀掉）；
//   * V3.4：SB→D$ 口升为整行（256b+32B strb），同行多字一次 RMW/单 MSHR；
//   * MSHR 在飞期间【同 index 不同行】或 MSHR 已满：优先挂到 1 项 pending
//     缓冲并立刻回 IDLE，从而继续 hit-under-miss；pending 在有空槽/写回空闲
//     后经 RELOOK 完成（同组冲突在重填落地后常变命中）。pending 已占用且再
//     撞冲突才退回 S_MWAIT（少见）；
//   * load/store miss 均可占满 N_MSHR；LSU 用 robid 配对多 miss 槽返回；
//   * AXI 读通道单 outstanding：owner 轮转服务各 MSHR 的 RREQ/RDATA；
// - 写回缓冲（1 项，原 TODO 第三步·写回与重填并行）：
//   脏 victim 在分配 MSHR 的同拍搬进写回缓冲，refill 读【立即发起】，
//   写回走独立写通道后台排空——dirty miss 不再串行"先写回后重填"；
// - CWF-lite（critical-word-first 协议内简化版）：refill 第一拍 128b 返回
//   时若目标字在低半行，立即给 ld_mshr_data_ok（比等整行早 4 个 AXI 拍）；
//   目标字在高半行则末拍返回。安装可再晚几拍（等 RAM 端口），不拦响应。
//
// uncached（原 TODO 第五步）：
// - load：LSU 保证只在 ROB 头发出且不取消；等 MSHR/写回缓冲全空后独占
//   读通道单字访问（按 ld_size_i 真实宽度）；
// - store：等写通道空后单字直写（按 st_size_i 真实宽度——团队赛 UART
//   字节写的坑，旧实现语义保留）；rdy=下层完成（B 已回）。
//
// cacop（原 TODO 第六步，commit 提交级一拍脉冲，内部暂存后插队）：
// - op0(IDX_INV/StoreTag)：直接无效化指定 way（addr[1:0]），无写回；
// - op1(HIT_INV/Index 写回无效)：指定 way 脏则先写回（axi_wr_cacop=1，
//   L2 写穿直达内存）再无效；
// - op2(HIT_WB/Hit 写回无效)：按物理地址查命中，脏则写回再无效。
//
// 响应契约（取消 / 冲刷）：
// - 每个被接受（addr_ok）的 load 都【必定恰好产生一次前端响应】：
//   命中/uncached -> ld_data_ok_o；miss -> ld_miss_o。冲刷后过期的前端
//   响仍由 LSU d_drop 配对丢弃（契约无静默丢包，避免 d_drop 死锁）；
// - ld_cancel_i（= LSU flush）：对在飞 load 源 MSHR 置 mshr_killed（记账）；
//   **保留** mshr_ld_resp_pend 并仍发 ld_mshr_data_ok_o，由 LSU miss 槽
//   m_drop 静默收槽（与「立即清槽 + 抑制 data_ok」解耦，消 orphan/错配）。
//   冲刷后才分配的 load：sticky req_ld_killed → pend=0（LSU d_drop 不占槽）；
// - uncached load 由 LSU 保证只在 ROB 头发出，正常不会被冲刷；
// - store/cacop 已提交，本就不可取消。
//
// 端口：
// - ld_* ：LSU load 访问口（+ ld_miss_o / ld_mshr_* 非阻塞扩展）
// - st_* ：store_buffer 写出口
// - cacop_*：cache 维护口（commit）
// - axi_* ：下层 L2 接口（行=2 拍 128b，ret_last 标末拍）
// ============================================================
module dcache (
    input  wire        clk,
    input  wire        resetn,

    // ---------------- LSU load 口 ----------------
    input  wire        ld_req_i,         // load 请求（保持至 addr_ok）
    input  wire [31:0] ld_vaddr_i,       // 虚地址（VIPT 索引）
    input  wire [31:0] ld_paddr_i,       // 物理地址（tag 比对）
    input  wire [2:0]  ld_size_i,        // 0=B 1=H 2=W（uncached 精确宽度）
    input  wire        ld_uncached_i,
    input  wire [`ROB_W-1:0] ld_robid_i, // 随 ld_req：miss 时锁入 MSHR，返回配对
    output wire        ld_addr_ok_o,
    output wire        ld_data_ok_o,     // 命中/uncached 完成（快速通道）
    output wire [31:0] ld_rdata_o,
    input  wire        ld_cancel_i,      // 冲刷：在飞 load MSHR 置 killed 记账（data_ok 仍发，LSU m_drop 收槽，见头注契约）
    // ---- 非阻塞 miss 扩展（配合 LSU miss 槽）----
    output wire        ld_miss_o,        // 本 load 已移入 MSHR（LOOKUP 拍一拍脉冲）
    output wire        ld_mshr_data_ok_o,// MSHR load 数据返回（一拍脉冲）
    output wire [31:0] ld_mshr_rdata_o,
    output wire [`ROB_W-1:0] ld_mshr_robid_o, // 与 data_ok 同拍，供 LSU 配对

    // ---------------- store_buffer 写出口（V3.4：行粒度 data/strb）----------------
    // SB 泄流口一次可合并同行多字；uncached 仍单字（放在行内对应字槽）。
    input  wire        st_req_i,         // store 写请求（保持至 addr_ok）
    input  wire [31:0] st_paddr_i,
    input  wire [`CACHE_LINE_BITS-1:0] st_data_i,
    input  wire [`CACHE_LINE_BYTES-1:0] st_strb_i, // 行内字节使能
    input  wire [2:0]  st_size_i,        // 仅 uncached AXI 宽度用
    input  wire        st_uncached_i,
    output wire        st_addr_ok_o,
    output wire        st_done_o,        // 写完成（命中两拍；miss 分配拍 posted）

    // ---------------- cache 维护口（commit 提交级驱动）----------------
    input  wire        cacop_en_i,       // 一拍脉冲
    input  wire [1:0]  cacop_op_i,       // IDX_INV / HIT_INV / HIT_WB
    input  wire [31:0] cacop_addr_i,

    // ---------------- 下层 L2 接口 ----------------
    output wire        axi_rd_req,
    output wire [2:0]  axi_rd_type,      // 0=B 1=H 2=W 4=cacheline refill
    output wire [31:0] axi_rd_addr,
    input  wire        axi_rd_rdy,
    input  wire        axi_ret_valid,
    input  wire        axi_ret_last,
    input  wire [127:0] axi_ret_data,
    output wire        axi_wr_req,
    output wire [2:0]  axi_wr_type,
    output wire [31:0] axi_wr_addr,
    output wire [15:0] axi_wr_strb,
    output wire [127:0] axi_wr_data,
    output wire        axi_wr_cacop,
    input  wire        axi_wr_rdy
);

localparam NWAY  = `L1_NWAY;           // 4
localparam NSET  = `L1_NSET;           // 128
localparam IDXW  = `L1_INDEX_W;        // 7
localparam TAGW  = `L1_TAG_W;          // 20
localparam LINEW = `CACHE_LINE_BITS;   // 256

// ---------------- 前端 FSM ----------------
localparam S_IDLE     = 4'd0;
localparam S_LOOKUP   = 4'd1;
localparam S_MWAIT    = 4'd2;   // 等资源（uncached/双重 pending/cacop）；cached 优先走 pend
localparam S_RELOOK   = 4'd3;   // pending/资源等待结束后重发 BRAM 读
localparam S_UC_RREQ  = 4'd4;   // uncached 读请求
localparam S_UC_RDATA = 4'd5;
localparam S_UC_RESP  = 4'd6;
localparam S_UC_WREQ  = 4'd7;   // uncached 写（rdy=完成）
localparam S_CAC_WB0  = 4'd8;   // cacop 写回 beat0
localparam S_CAC_WB1  = 4'd9;

reg [3:0] state;

// ---------------- MSHR（后台 refill 引擎，参数化数组）----------------
localparam N_MSHR  = `DC_MSHR_DEPTH;
localparam MSHR_W  = (N_MSHR <= 1) ? 1 : $clog2(N_MSHR);

localparam M_IDLE    = 2'd0;
localparam M_RREQ    = 2'd1;
localparam M_RDATA   = 2'd2;
localparam M_INSTALL = 2'd3;

reg [1:0]       mshr_state        [0:N_MSHR-1];
reg             mshr_is_st        [0:N_MSHR-1];
reg             mshr_ld_resp_pend [0:N_MSHR-1]; // load 响应待发（CWF：发过即清）
reg             mshr_killed       [0:N_MSHR-1]; // 冲刷记账；仍回 ld_mshr_data_ok（LSU m_drop）
reg             mshr_from_ld      [0:N_MSHR-1]; // 曾为 load miss（merge 成 store 后仍粘住）
reg [31:0]      mshr_paddr        [0:N_MSHR-1]; // 含目标字偏移
reg [`ROB_W-1:0] mshr_robid       [0:N_MSHR-1]; // load miss 配对（store 可忽略）
reg [1:0]       mshr_way          [0:N_MSHR-1];
reg [31:0]      mshr_stb_line     [0:N_MSHR-1]; // 行内 byte enable（同行多字 store 合并）
reg [127:0]     mshr_b0           [0:N_MSHR-1]; // AXI 首拍半行
// 精简：原 dat_line + line 合并为一份——
//   RREQ/RDATA 前半：存 store 叠层；beat1 后：存待安装整行
reg [LINEW-1:0] mshr_line         [0:N_MSHR-1];

// 最低编号优先编码（N=1 时恒为 0）
function automatic [MSHR_W-1:0] dc_mshr_prio_low;
    input [N_MSHR-1:0] mask;
    integer k;
    reg found;
    begin
        dc_mshr_prio_low = {MSHR_W{1'b0}};
        found = 1'b0;
        for (k = 0; k < N_MSHR; k = k + 1) begin
            if (mask[k] && !found) begin
                dc_mshr_prio_low = k[MSHR_W-1:0];
                found = 1'b1;
            end
        end
    end
endfunction

wire [N_MSHR-1:0] mshr_busy_oh;
wire [N_MSHR-1:0] mshr_rreq_oh;
wire [N_MSHR-1:0] mshr_rdata_oh;
wire [N_MSHR-1:0] mshr_install_oh;
wire [N_MSHR-1:0] mshr_is_ld_oh;   // busy 且 load miss（!store）

genvar gm;
generate
for (gm = 0; gm < N_MSHR; gm = gm + 1) begin : gen_mshr_status
    assign mshr_busy_oh[gm]     = (mshr_state[gm] != M_IDLE);
    assign mshr_rreq_oh[gm]     = (mshr_state[gm] == M_RREQ);
    assign mshr_rdata_oh[gm]    = (mshr_state[gm] == M_RDATA);
    assign mshr_install_oh[gm]  = (mshr_state[gm] == M_INSTALL);
    assign mshr_is_ld_oh[gm]    = mshr_busy_oh[gm] && !mshr_is_st[gm];
end
endgenerate

wire              mshr_any_busy     = |mshr_busy_oh;
wire              mshr_has_free     = ~(&mshr_busy_oh);
wire              mshr_any_install  = |mshr_install_oh;
wire              mshr_any_load     = |mshr_is_ld_oh;  // 在飞 load miss 数（perf/调试）
wire              mshr_rreq_vld      = |mshr_rreq_oh;
wire              mshr_rdata_vld     = |mshr_rdata_oh;
wire [MSHR_W-1:0] mshr_free_idx     = dc_mshr_prio_low(~mshr_busy_oh);
wire [MSHR_W-1:0] mshr_rreq_idx     = dc_mshr_prio_low(mshr_rreq_oh);
wire [MSHR_W-1:0] mshr_rdata_idx    = dc_mshr_prio_low(mshr_rdata_oh);
wire [MSHR_W-1:0] mshr_install_idx  = dc_mshr_prio_low(mshr_install_oh);

// 兼容旧单 MSHR 探针/统计
wire mshr_busy = mshr_any_busy;

// AXI 读通道 owner：同时只服务一个 MSHR（RREQ 受理 → RDATA 收完）
reg                axi_mshr_hold;
reg [MSHR_W-1:0]   axi_mshr_id;
wire               axi_mshr_grant_vld = axi_mshr_hold || mshr_rreq_vld;
wire [MSHR_W-1:0]  axi_mshr_grant     = axi_mshr_hold ? axi_mshr_id : mshr_rreq_idx;
wire [31:0]        mshr_axi_paddr     = mshr_paddr[axi_mshr_grant];
// ---------------- 写回缓冲（victim writeback，1 项）----------------
localparam W_IDLE = 2'd0;
localparam W_B0   = 2'd1;
localparam W_B1   = 2'd2;

reg [1:0]       wb_state;
reg             wb_valid;
reg [31:0]      wb_addr;        // 行对齐
reg [LINEW-1:0] wb_line;

wire wb_all_idle = (wb_state == W_IDLE) && !wb_valid;

// ---------------- 存储阵列 ----------------
// valid/dirty 需复位/一拍失效，保持触发器；tag 拆 per-way 一维阵列（见下 gen_tag）
// —— 二维 reg 数组 Vivado 推断不出分布式 RAM，会落成 ~10k FF + 巨型读 mux。
reg [NSET-1:0] valid_arr [0:NWAY-1];
reg [NSET-1:0] dirty_arr [0:NWAY-1];
wire [TAGW-1:0] tag_rd [0:NWAY-1];               // 各路 tag 在 req_set 处的异步读值

wire [LINEW-1:0] data_out [0:NWAY-1];
reg  [IDXW-1:0]  ram_addr;
reg  [NWAY-1:0]  ram_we;
reg  [LINEW-1:0] ram_wline;
reg              ram_re;

// 每组替换指针（伪随机轮转）
reg [1:0] rr_ptr [0:NSET-1];

// ---------------- 请求锁存 ----------------
reg        req_is_st;
reg        req_is_ld;
reg        req_is_cacop;
reg [1:0]  req_cacop_op;
reg [31:0] req_paddr;
reg [`ROB_W-1:0] req_robid;       // load：随请求锁存，miss 写入 MSHR
reg [LINEW-1:0] req_wdata;        // V3.4：行粒度（UC 时有效字在 req_word 槽）
reg [31:0]  req_wstrb;            // 行内 32 字节使能
reg [2:0]  req_size;
reg        req_uncached;
reg        req_ld_killed;         // 本前端 load 已被冲刷（sticky→MSHR.killed）

// cacop 暂存（commit 一拍脉冲，FSM 忙时排队；commit 串行发起，深度 1 足够）
reg        cacop_pend;
reg [1:0]  cacop_pend_op;
reg [31:0] cacop_pend_addr;

// cached 二次 miss / 同组撞 MSHR：挂起缓冲（释放前端继续 hit-under-miss）
reg        pend_valid;
reg        pend_is_st;
reg        pend_is_ld;
reg        pend_ld_killed;        // pend 中的 load 已被冲刷
reg [31:0] pend_paddr;
reg [`ROB_W-1:0] pend_robid;
reg [LINEW-1:0] pend_wdata;
reg [31:0] pend_wstrb;
reg [2:0]  pend_size;
reg        pend_uncached;

// cacop 写回中间量 / uncached 读数据
reg [TAGW-1:0]   cwb_tag;
reg [LINEW-1:0]  cwb_line;
reg [31:0]       uc_rdata;

wire [IDXW-1:0] req_set = req_paddr[IDXW+`CACHE_LINE_W-1:`CACHE_LINE_W];
wire [TAGW-1:0] req_tag = req_paddr[31:IDXW+`CACHE_LINE_W];
wire [2:0]      req_word= req_paddr[4:2];

// ---------------- tag 阵列（per-way LUTRAM：1 写口 + req_set 异步读）----------------
// 写口唯一：MSHR 安装拍写 mshr_inst_set（与原 FSM 内 tag_arr 写同拍同条件）；
// 读口全部落在 req_set（cac_set 与 req_set 同为 req_paddr 同一切片）。
// 复位不清 tag（valid=0 即无效），与 LUTRAM 无复位的特性一致。
genvar gt;
generate
for (gt = 0; gt < NWAY; gt = gt + 1) begin : gen_tag
    (* ram_style = "distributed" *) reg [TAGW-1:0] tag_ram [0:NSET-1];
    always @(posedge clk) begin
        if (mshr_install_fire && (mshr_inst_way == gt[1:0]))
            tag_ram[mshr_inst_set] <= mshr_inst_tag;
    end
    assign tag_rd[gt] = tag_ram[req_set];
end
endgenerate

// ---------------- 命中判定（LOOKUP 拍，tag LUTRAM 异步读）----------------
wire [NWAY-1:0] way_hit;
genvar gw;
generate
for (gw = 0; gw < NWAY; gw = gw + 1) begin : gen_hit
    assign way_hit[gw] = valid_arr[gw][req_set] && (tag_rd[gw] == req_tag);
end
endgenerate
wire        hit_any = |way_hit;
wire [1:0]  hit_way = way_hit[1] ? 2'd1 : way_hit[2] ? 2'd2 : way_hit[3] ? 2'd3 : 2'd0;

// victim 选择：无效路优先，否则轮转
wire [NWAY-1:0] way_invalid;
generate
for (gw = 0; gw < NWAY; gw = gw + 1) begin : gen_inv
    assign way_invalid[gw] = !valid_arr[gw][req_set];
end
endgenerate
wire [1:0] pick_way = way_invalid[0] ? 2'd0 :
                      way_invalid[1] ? 2'd1 :
                      way_invalid[2] ? 2'd2 :
                      way_invalid[3] ? 2'd3 : rr_ptr[req_set];
wire pick_valid = valid_arr[pick_way][req_set];
wire pick_dirty = dirty_arr[pick_way][req_set];

// cacop 的 set/way 解码（op0/op1 按 addr 低位选 way）
wire [IDXW-1:0] cac_set = req_paddr[IDXW+`CACHE_LINE_W-1:`CACHE_LINE_W];
wire [1:0]      cac_way = req_paddr[1:0];

// ---------------- 接受仲裁（IDLE 拍）----------------
// MSHR 安装拍需要独占 RAM 口：安装等待期间暂停接受新请求（一拍气泡）
// pending 可 drain 时优先恢复挂起请求（不接受新请求），否则在 pend 占用时仍可
// 接受新请求以维持 hit-under-miss（再 miss/撞组才落 S_MWAIT）
wire mshr_res_idle = !mshr_any_busy && wb_all_idle;
// pend 恢复：必须「有空槽」且「重查后大概率能前进」——
// 同 set 仍冲突 / load 已占满 时禁止 drain，否则会 LOOKUP→pend 死循环（perf 上
// pend_push 爆炸、IPC 变差）。
wire [IDXW-1:0] pend_set = pend_paddr[IDXW+`CACHE_LINE_W-1:`CACHE_LINE_W];
wire [N_MSHR-1:0] pend_set_match;
generate
for (gm = 0; gm < N_MSHR; gm = gm + 1) begin : gen_pend_set_match
    assign pend_set_match[gm] = mshr_busy_oh[gm]
                             && (pend_set == mshr_paddr[gm][IDXW+`CACHE_LINE_W-1:`CACHE_LINE_W]);
end
endgenerate
wire pend_can_progress = !(|pend_set_match) && mshr_has_free;
wire pend_drain = (state == S_IDLE) && pend_valid && mshr_has_free && wb_all_idle
               && !mshr_any_install && pend_can_progress;
wire accept_ok = (state == S_IDLE) && !mshr_any_install && !pend_drain;
wire cacop_take = accept_ok && cacop_pend;
wire st_take    = accept_ok && !cacop_pend && st_req_i;
wire ld_take    = accept_ok && !cacop_pend && !st_req_i && ld_req_i;

assign st_addr_ok_o = st_take;
assign ld_addr_ok_o = ld_take;

// ---------------- LOOKUP 拍分类 ----------------
// 同 index 与任一在飞 MSHR 冲突：默认等待（防 victim 路互踩）
// 例外：同行 store 可合并进该 MSHR（降 pend），见 lk_st_merge
wire [N_MSHR-1:0] mshr_set_match;
wire [N_MSHR-1:0] mshr_line_match;
wire [N_MSHR-1:0] mshr_mergeable; // RREQ/RDATA/INSTALL 可合 store
generate
for (gm = 0; gm < N_MSHR; gm = gm + 1) begin : gen_mshr_set_match
    assign mshr_set_match[gm] = mshr_busy_oh[gm]
                              && (req_set == mshr_paddr[gm][IDXW+`CACHE_LINE_W-1:`CACHE_LINE_W]);
    assign mshr_line_match[gm] = mshr_busy_oh[gm]
                              && (req_paddr[31:`CACHE_LINE_W] == mshr_paddr[gm][31:`CACHE_LINE_W]);
    assign mshr_mergeable[gm] = mshr_line_match[gm]
                             && ((mshr_state[gm] == M_RREQ)
                              || (mshr_state[gm] == M_RDATA)
                              || (mshr_state[gm] == M_INSTALL));
end
endgenerate
wire        lk_st_merge = (state == S_LOOKUP) && req_is_st && !req_is_cacop && !req_uncached
                       && (|mshr_mergeable);
wire [MSHR_W-1:0] mshr_merge_idx = dc_mshr_prio_low(mshr_mergeable);
// V3.4：SB 已给行级 strb；不再按单字左移
wire [31:0] req_stb_line = req_wstrb;

wire lk_set_conf  = (state == S_LOOKUP) && !req_is_cacop && !req_uncached
                 && (|mshr_set_match) && !lk_st_merge;
// cacop 需引擎全静默（MSHR 安装未落地前 tag 状态不完整，写回通道也要独占）
wire lk_cacop_wait = (state == S_LOOKUP) && req_is_cacop && (mshr_any_busy || !wb_all_idle);

wire lk_cached_ld = (state == S_LOOKUP) && req_is_ld && !req_is_cacop && !req_uncached;
wire lk_cached_st = (state == S_LOOKUP) && req_is_st && !req_is_cacop && !req_uncached;
wire lk_uc_ld     = (state == S_LOOKUP) && req_is_ld && !req_is_cacop && req_uncached;
wire lk_uc_st     = (state == S_LOOKUP) && req_is_st && !req_is_cacop && req_uncached;
wire lk_cacop     = (state == S_LOOKUP) && req_is_cacop && !lk_cacop_wait;

wire lk_ld_hit  = lk_cached_ld && !lk_set_conf && hit_any;
wire lk_st_hit  = lk_cached_st && !lk_set_conf && !lk_st_merge && hit_any;
wire lk_ld_miss = lk_cached_ld && !lk_set_conf && !hit_any;
wire lk_st_miss = lk_cached_st && !lk_set_conf && !lk_st_merge && !hit_any;

// miss 分配：有空槽 +（脏 victim 需 WB 空）；load/store 均可占满 N_MSHR
// （LSU miss 槽深度 = LSU_MISS_DEPTH = DC_MSHR_DEPTH，返回靠 robid 配对）
wire miss_need_wb = pick_valid && pick_dirty;
wire mshr_alloc_base = mshr_has_free && (!miss_need_wb || wb_all_idle);
wire mshr_alloc_ok = mshr_alloc_base;
wire lk_ld_alloc = lk_ld_miss && mshr_alloc_base;
wire lk_st_alloc = lk_st_miss && mshr_alloc_base;

// uncached 进入条件：独占读/写通道（MSHR 与写回缓冲全空，保守串行——
// uncached 本身就是强序访问，性能无关紧要）
wire lk_uc_ok = !mshr_any_busy && wb_all_idle;

// cached 资源冲突：可挂 pending（释放前端）；pend 已占用则被迫 MWAIT
// 同行 store 合并不进 block
wire lk_cache_block = lk_set_conf
                   || ((lk_ld_miss || lk_st_miss) && !mshr_alloc_ok);
wire lk_to_pend = lk_cache_block && !pend_valid;
wire lk_to_mwait_cache = lk_cache_block && pend_valid;

// ---------------- 命中数据通路 ----------------
wire [LINEW-1:0] hit_line = data_out[hit_way];
wire [31:0] hit_word = hit_line[32*req_word +: 32];

// 行级字节使能 → 位掩码（V3.4）
reg [LINEW-1:0] st_line_be;
integer sbi;
always @(*) begin
    for (sbi = 0; sbi < 32; sbi = sbi + 1)
        st_line_be[8*sbi +: 8] = {8{req_wstrb[sbi]}};
end

// store 命中合并行（读改写：整行字节使能）
reg [LINEW-1:0] st_merge_line;
always @(*) begin
    st_merge_line = (data_out[hit_way] & ~st_line_be) | (req_wdata & st_line_be);
end

// ---------------- MSHR 重填数据通路（归属 AXI owner）----------------
wire [127:0] mshr_rf_b0    = mshr_b0[axi_mshr_grant];
wire [2:0]   mshr_rf_word  = mshr_paddr[axi_mshr_grant][4:2];
wire         mshr_rf_ld_resp = mshr_ld_resp_pend[axi_mshr_grant];
wire         mshr_owner_rdata = axi_mshr_grant_vld
                             && (mshr_state[axi_mshr_grant] == M_RDATA);

wire [LINEW-1:0] refill_line_raw = {axi_ret_data, mshr_rf_b0};
// 行级 store 叠层：含同拍 lk_st_merge→本 MSHR（避免 NBA 晚一拍丢合并）
wire        refill_merge_now = lk_st_merge && (mshr_merge_idx == axi_mshr_grant);
wire [31:0] refill_stb_eff   = mshr_stb_line[axi_mshr_grant]
                             | (refill_merge_now ? req_stb_line : 32'b0);
reg [LINEW-1:0] refill_dat_eff;
reg [LINEW-1:0] refill_stb_exp;
reg [LINEW-1:0] refill_line_merged;
integer rb;
always @(*) begin
    refill_dat_eff = mshr_line[axi_mshr_grant];
    if (refill_merge_now)
        refill_dat_eff = (refill_dat_eff & ~st_line_be) | (req_wdata & st_line_be);
    for (rb = 0; rb < 32; rb = rb + 1)
        refill_stb_exp[8*rb +: 8] = {8{refill_stb_eff[rb]}};
    refill_line_merged = (refill_line_raw & ~refill_stb_exp) | (refill_dat_eff & refill_stb_exp);
end

// MSHR 数据接收：仅 AXI owner 处于 M_RDATA 时吃返回
wire mshr_beat  = mshr_owner_rdata && axi_ret_valid;
wire mshr_beat0 = mshr_beat && !axi_ret_last;
wire mshr_beat1 = mshr_beat &&  axi_ret_last;

// CWF-lite：目标字半行一到即回数。killed/cancel **不**抑制 data_ok：
// LSU 用 miss 槽 m_drop 丢弃冲刷后的返回，保证槽可回收。
wire [31:0] beat_word = axi_ret_data[32*mshr_rf_word[1:0] +: 32];
wire        mshr_rf_killed = mshr_killed[axi_mshr_grant];
assign ld_mshr_data_ok_o = mshr_rf_ld_resp
                        && ((mshr_beat0 && !mshr_rf_word[2])
                         || (mshr_beat1 &&  mshr_rf_word[2]));
assign ld_mshr_rdata_o   = beat_word;
assign ld_mshr_robid_o   = mshr_robid[axi_mshr_grant];

// MSHR 安装拍：同一拍最多装 1 项（单口 BRAM）
// 注：accept_ok 已要求 !mshr_any_install，故 st/ld/cacop_take 与 install
// 互斥；勿再把 take 编入 front_ram_busy，否则 STA 会走出
// SB.query→ld_req→ld_take→!install_fire→valid_arr 的假路径（55MHz 违约）。
wire front_ram_busy = (state == S_RELOOK) || lk_st_hit;
wire [N_MSHR-1:0] mshr_install_fire_oh;
generate
for (gm = 0; gm < N_MSHR; gm = gm + 1) begin : gen_mshr_install_fire
    assign mshr_install_fire_oh[gm] =
        mshr_install_oh[gm] && !front_ram_busy
     && (mshr_install_idx == gm[MSHR_W-1:0]);
end
endgenerate
wire mshr_install_fire = |mshr_install_fire_oh;
wire [1:0]      mshr_inst_way = mshr_way[mshr_install_idx];
wire [IDXW-1:0] mshr_inst_set = mshr_paddr[mshr_install_idx][IDXW+`CACHE_LINE_W-1:`CACHE_LINE_W];
wire [TAGW-1:0] mshr_inst_tag = mshr_paddr[mshr_install_idx][31:IDXW+`CACHE_LINE_W];
// 安装同拍合入 store：组合叠层，避免 NBA 晚一拍丢写；
// dirty 用 is_st|merge|stb，不依赖 merge 改写 is_st（orphan 契约）
wire            install_merge_now = lk_st_merge
                                 && (mshr_merge_idx == mshr_install_idx)
                                 && mshr_install_oh[mshr_install_idx];
wire            mshr_inst_is_st = mshr_is_st[mshr_install_idx]
                                | install_merge_now
                                | (|mshr_stb_line[mshr_install_idx]);
reg [LINEW-1:0] mshr_inst_line;
always @(*) begin
    mshr_inst_line = mshr_line[mshr_install_idx];
    if (install_merge_now)
        mshr_inst_line = (mshr_inst_line & ~st_line_be) | (req_wdata & st_line_be);
end
// ---------------- 响应输出 ----------------
assign ld_data_ok_o = lk_ld_hit | (state == S_UC_RESP);
assign ld_rdata_o   = (state == S_UC_RESP) ? uc_rdata : hit_word;
assign ld_miss_o    = lk_ld_alloc;

// store：命中 / miss 分配 posted / 同行合入在飞 MSHR posted；uncached 等下层
assign st_done_o    = lk_st_hit | lk_st_alloc | lk_st_merge
                    | ((state == S_UC_WREQ) && axi_wr_rdy);

// ---------------- 下层读通道（MSHR owner 与 uncached 互斥）----------------
// 同行写读序保护：写回缓冲还压着与 refill 同一行的写时，refill 读必须等待。
wire mshr_owner_rreq = axi_mshr_grant_vld
                    && (mshr_state[axi_mshr_grant] == M_RREQ);
wire mshr_rd_same_line_blk = mshr_owner_rreq
                          && (wb_valid || (wb_state != W_IDLE))
                          && (wb_addr[31:`CACHE_LINE_W] == mshr_axi_paddr[31:`CACHE_LINE_W]);
assign axi_rd_req  = (mshr_owner_rreq && !mshr_rd_same_line_blk)
                   || (state == S_UC_RREQ);
assign axi_rd_type = mshr_owner_rreq ? 3'b100 : req_size;
assign axi_rd_addr = mshr_owner_rreq ? {mshr_axi_paddr[31:`CACHE_LINE_W], {`CACHE_LINE_W{1'b0}}}
                                     : req_paddr;

// ---------------- 下层写通道（写回缓冲 / cacop 写回 / uncached 写 互斥）----------------
assign axi_wr_req  = (wb_state == W_B0) || (wb_state == W_B1)
                   || (state == S_CAC_WB0) || (state == S_CAC_WB1)
                   || (state == S_UC_WREQ);
assign axi_wr_type = (state == S_UC_WREQ) ? req_size : 3'b100;
assign axi_wr_addr = (state == S_UC_WREQ) ? req_paddr
                   : ((state == S_CAC_WB0) || (state == S_CAC_WB1))
                       ? {cwb_tag, req_set, {`CACHE_LINE_W{1'b0}}}
                       : wb_addr;
// UC：从行内对应字槽抽单字（SB 已按 paddr[4:2] 放入）
wire [31:0] uc_st_word = req_wdata[32*req_word +: 32];
wire [3:0]  uc_st_strb = req_wstrb[4*req_word +: 4];
assign axi_wr_data = (state == S_UC_WREQ) ? {96'b0, uc_st_word}
                   : (state == S_CAC_WB0) ? cwb_line[127:0]
                   : (state == S_CAC_WB1) ? cwb_line[255:128]
                   : (wb_state == W_B0)   ? wb_line[127:0]
                                          : wb_line[255:128];
assign axi_wr_strb = (state == S_UC_WREQ) ? {12'b0, uc_st_strb} : 16'hffff;
assign axi_wr_cacop= (state == S_CAC_WB0) || (state == S_CAC_WB1);

// ---------------- BRAM 读写控制 ----------------
// 读：IDLE 接受拍（地址=新请求 index）/ RELOOK 重发；
// 写：LOOKUP store 命中（整行读改写）/ MSHR 安装 —— 单口分拍复用
wire [IDXW-1:0] rd_set_idle = cacop_take ? cacop_pend_addr[IDXW+`CACHE_LINE_W-1:`CACHE_LINE_W]
                            : st_take    ? st_paddr_i[IDXW+`CACHE_LINE_W-1:`CACHE_LINE_W]
                                         : ld_vaddr_i[IDXW+`CACHE_LINE_W-1:`CACHE_LINE_W];

always @(*) begin
    ram_re    = 1'b0;
    ram_we    = {NWAY{1'b0}};
    ram_addr  = rd_set_idle;
    ram_wline = st_merge_line;
    if (lk_st_hit) begin
        ram_we[hit_way] = 1'b1;
        ram_addr        = req_set;
        ram_wline       = st_merge_line;
    end else if (mshr_install_fire) begin
        ram_we[mshr_inst_way] = 1'b1;
        ram_addr              = mshr_inst_set;
        ram_wline             = mshr_inst_line;
    end else if (state == S_RELOOK) begin
        ram_re   = 1'b1;
        ram_addr = req_set;
    end else if (st_take || ld_take || cacop_take) begin
        ram_re   = 1'b1;
        ram_addr = rd_set_idle;
    end
end

genvar gr;
generate
for (gr = 0; gr < NWAY; gr = gr + 1) begin : gen_dram
    dcache_way_ram u_way_ram(
        .clk   (clk),
        .en    (ram_re | ram_we[gr]),
        .we    (ram_we[gr]),
        .addr  (ram_addr),
        .wdata (ram_wline),
        .rdata (data_out[gr])
    );
end
endgenerate

// ---------------- 前端 FSM ----------------
integer s;
always @(posedge clk) begin
    if (!resetn) begin
        state          <= S_IDLE;
        cacop_pend     <= 1'b0;
        pend_valid     <= 1'b0;
        pend_ld_killed <= 1'b0;
        req_ld_killed  <= 1'b0;
        for (s = 0; s < NWAY; s = s + 1) begin
            valid_arr[s] <= {NSET{1'b0}};
            dirty_arr[s] <= {NSET{1'b0}};
        end
    end else begin
        // cacop 暂存（commit 一拍脉冲随时到来）
        if (cacop_en_i) begin
            cacop_pend      <= 1'b1;
            cacop_pend_op   <= cacop_op_i;
            cacop_pend_addr <= cacop_addr_i;
        end

        // 冲刷：sticky 标记在飞 / pend load（MSHR.killed 在 MSHR 块内同步）
        if (ld_cancel_i) begin
            if (req_is_ld && (state != S_IDLE))
                req_ld_killed <= 1'b1;
            if (pend_valid && pend_is_ld)
                pend_ld_killed <= 1'b1;
        end

        // MSHR 安装：更新 valid/dirty（tag 写在 gen_tag，数据 RAM 写在组合块）
        if (mshr_install_fire) begin
            valid_arr[mshr_inst_way][mshr_inst_set] <= 1'b1;
            dirty_arr[mshr_inst_way][mshr_inst_set] <= mshr_inst_is_st;
        end

        case (state)
            S_IDLE: begin
                if (pend_drain) begin
                    // 恢复挂起请求，重查（可能已因 refill 变命中）
                    req_is_cacop  <= 1'b0;
                    req_is_st     <= pend_is_st;
                    req_is_ld     <= pend_is_ld;
                    req_ld_killed <= pend_ld_killed;
                    req_paddr     <= pend_paddr;
                    req_robid     <= pend_robid;
                    req_wdata     <= pend_wdata;
                    req_wstrb     <= pend_wstrb;
                    req_size      <= pend_size;
                    req_uncached  <= pend_uncached;
                    pend_valid    <= 1'b0;
                    pend_ld_killed<= 1'b0;
                    state         <= S_RELOOK;
                end else if (cacop_take) begin
                    req_is_cacop  <= 1'b1;
                    req_is_st     <= 1'b0;
                    req_is_ld     <= 1'b0;
                    req_ld_killed <= 1'b0;
                    req_uncached  <= 1'b0;
                    req_cacop_op  <= cacop_pend_op;
                    req_paddr     <= cacop_pend_addr;
                    cacop_pend    <= 1'b0;
                    state         <= S_LOOKUP;
                end else if (st_take) begin
                    req_is_cacop  <= 1'b0;
                    req_is_st     <= 1'b1;
                    req_is_ld     <= 1'b0;
                    req_ld_killed <= 1'b0;
                    req_paddr     <= st_paddr_i;
                    req_wdata     <= st_data_i;
                    req_wstrb     <= st_strb_i;
                    req_size      <= st_size_i;
                    req_uncached  <= st_uncached_i;
                    state         <= S_LOOKUP;
                end else if (ld_take) begin
                    req_is_cacop  <= 1'b0;
                    req_is_st     <= 1'b0;
                    req_is_ld     <= 1'b1;
                    req_ld_killed <= ld_cancel_i; // 同拍冲刷：直接 killed
                    req_paddr     <= ld_paddr_i;
                    req_robid     <= ld_robid_i;
                    req_size      <= ld_size_i;
                    req_uncached  <= ld_uncached_i;
                    state         <= S_LOOKUP;
                end
            end

            S_LOOKUP: begin
                if (lk_cacop_wait) begin
                    state <= S_MWAIT;
                end else if (lk_to_pend) begin
                    pend_valid     <= 1'b1;
                    pend_is_st     <= req_is_st;
                    pend_is_ld     <= req_is_ld;
                    pend_ld_killed <= req_is_ld && (req_ld_killed || ld_cancel_i);
                    pend_paddr     <= req_paddr;
                    pend_robid     <= req_robid;
                    pend_wdata     <= req_wdata;
                    pend_wstrb     <= req_wstrb;
                    pend_size      <= req_size;
                    pend_uncached  <= req_uncached;
                    req_ld_killed  <= 1'b0;
                    state          <= S_IDLE;
                end else if (lk_to_mwait_cache) begin
                    state <= S_MWAIT;
                end else if (lk_cacop) begin
                    // 到这里保证 MSHR/写回缓冲全静默（lk_cacop_wait 已滤掉）
                    case (req_cacop_op)
                        `CACOP_OP_IDX_INV: begin
                            // op0 StoreTag：直接无效化（无写回）
                            valid_arr[cac_way][cac_set] <= 1'b0;
                            dirty_arr[cac_way][cac_set] <= 1'b0;
                            state <= S_IDLE;
                        end
                        `CACOP_OP_HIT_INV: begin
                            // op1 Index 写回无效（way 由 addr[1:0] 指定）
                            if (valid_arr[cac_way][cac_set] && dirty_arr[cac_way][cac_set]) begin
                                cwb_tag  <= tag_rd[cac_way];   // cac_set == req_set
                                cwb_line <= data_out[cac_way];
                                valid_arr[cac_way][cac_set] <= 1'b0;
                                dirty_arr[cac_way][cac_set] <= 1'b0;
                                state <= S_CAC_WB0;
                            end else begin
                                valid_arr[cac_way][cac_set] <= 1'b0;
                                dirty_arr[cac_way][cac_set] <= 1'b0;
                                state <= S_IDLE;
                            end
                        end
                        default: begin
                            // op2 Hit 写回无效（物理地址查命中）
                            if (hit_any) begin
                                if (dirty_arr[hit_way][req_set]) begin
                                    cwb_tag  <= tag_rd[hit_way];
                                    cwb_line <= data_out[hit_way];
                                    valid_arr[hit_way][req_set] <= 1'b0;
                                    dirty_arr[hit_way][req_set] <= 1'b0;
                                    state <= S_CAC_WB0;
                                end else begin
                                    valid_arr[hit_way][req_set] <= 1'b0;
                                    state <= S_IDLE;
                                end
                            end else begin
                                state <= S_IDLE;
                            end
                        end
                    endcase
                end else if (lk_uc_ld) begin
                    state <= lk_uc_ok ? S_UC_RREQ : S_MWAIT;
                end else if (lk_uc_st) begin
                    state <= lk_uc_ok ? S_UC_WREQ : S_MWAIT;
                end else if (lk_st_merge) begin
                    // 同行 store 合入在飞 MSHR：posted st_done，前端回 IDLE
                    req_ld_killed <= 1'b0;
                    state <= S_IDLE;
                end else if (hit_any) begin
                    // 命中：load 出数 / store 合并写（组合块），本拍完成
                    if (req_is_st) dirty_arr[hit_way][req_set] <= 1'b1;
                    req_ld_killed <= 1'b0;
                    state <= S_IDLE;
                end else if (mshr_alloc_ok) begin
                    // miss：分配 MSHR（脏 victim 同拍进写回缓冲，见排空引擎块），
                    // 前端即空闲；ld_miss_o / st_done_o(posted) 在本拍组合给出
                    rr_ptr[req_set] <= rr_ptr[req_set] + 2'd1;
                    req_ld_killed <= 1'b0;
                    state <= S_IDLE;
                end else begin
                    // 兜底（理论上 lk_* 已覆盖 cached block）
                    state <= S_MWAIT;
                end
            end

            S_MWAIT: begin
                // uncached / 双重 pending / cacop：等资源后重查
                if (!mshr_any_busy && wb_all_idle) begin
                    state <= S_RELOOK;
                end
            end

            S_RELOOK: state <= S_LOOKUP;       // 重发 BRAM 读后再比对

            S_UC_RREQ: if (axi_rd_rdy) state <= S_UC_RDATA;
            S_UC_RDATA: begin
                if (axi_ret_valid) begin
                    // uncached 单拍返回：word 在 ret_data[31:0]（见 axi_line_bridge）
                    uc_rdata <= axi_ret_data[31:0];
                    state    <= S_UC_RESP;
                end
            end
            S_UC_RESP: begin
                req_ld_killed <= 1'b0;
                state <= S_IDLE;
            end

            S_UC_WREQ: if (axi_wr_rdy) state <= S_IDLE;

            S_CAC_WB0: if (axi_wr_rdy) state <= S_CAC_WB1;
            S_CAC_WB1: state <= S_IDLE;        // beat1 直推一拍（下层保证连续接收）

            default: state <= S_IDLE;
        endcase
    end
end

// ---------------- MSHR 引擎 + AXI owner ----------------
wire mshr_alloc = lk_ld_alloc | lk_st_alloc;

integer mi;
always @(posedge clk) begin
    if (!resetn) begin
        axi_mshr_hold <= 1'b0;
        axi_mshr_id   <= {MSHR_W{1'b0}};
        for (mi = 0; mi < N_MSHR; mi = mi + 1) begin
            mshr_state[mi]        <= M_IDLE;
            mshr_ld_resp_pend[mi] <= 1'b0;
            mshr_killed[mi]       <= 1'b0;
            mshr_from_ld[mi]      <= 1'b0;
            mshr_is_st[mi]        <= 1'b0;
            mshr_paddr[mi]        <= 32'b0;
            mshr_robid[mi]        <= {`ROB_W{1'b0}};
            mshr_way[mi]          <= 2'b0;
            mshr_stb_line[mi]     <= 32'b0;
            mshr_line[mi]         <= {LINEW{1'b0}};
        end
    end else begin
        // 冲刷：标记 killed；保留 ld_resp_pend，仍回 data_ok 供 LSU drop 收槽
        if (ld_cancel_i) begin
            for (mi = 0; mi < N_MSHR; mi = mi + 1) begin
                if (mshr_busy_oh[mi] && (mshr_from_ld[mi] || !mshr_is_st[mi] || mshr_ld_resp_pend[mi])) begin
                    mshr_killed[mi] <= 1'b1;
                end
            end
        end

        // AXI owner：受理 RREQ 时锁定，RDATA 末拍释放
        if (axi_mshr_hold) begin
            if ((mshr_state[axi_mshr_id] == M_RDATA) && mshr_beat1)
                axi_mshr_hold <= 1'b0;
            else if (mshr_state[axi_mshr_id] == M_IDLE)
                axi_mshr_hold <= 1'b0;
        end else if (mshr_owner_rreq && axi_rd_rdy && !mshr_rd_same_line_blk) begin
            axi_mshr_hold <= 1'b1;
            axi_mshr_id   <= axi_mshr_grant;
        end

        for (mi = 0; mi < N_MSHR; mi = mi + 1) begin
            case (mshr_state[mi])
                M_IDLE: begin
                    if (mshr_alloc && (mshr_free_idx == mi[MSHR_W-1:0])) begin
                        mshr_is_st[mi]        <= req_is_st;
                        mshr_from_ld[mi]      <= req_is_ld;
                        mshr_killed[mi]       <= req_is_ld && (req_ld_killed || ld_cancel_i);
                        mshr_ld_resp_pend[mi] <= req_is_ld && !(req_ld_killed || ld_cancel_i);
                        mshr_paddr[mi]        <= req_paddr;
                        mshr_robid[mi]        <= req_robid;
                        mshr_way[mi]          <= pick_way;
                        if (req_is_st) begin
                            mshr_stb_line[mi] <= req_stb_line;
                            // beat1 前 line 作 store 叠层（整行）
                            mshr_line[mi]     <= req_wdata;
                        end else begin
                            mshr_stb_line[mi] <= 32'b0;
                            mshr_line[mi]     <= {LINEW{1'b0}};
                        end
                        mshr_state[mi]        <= M_RREQ;
                    end
                end
                M_RREQ: begin
                    if (axi_rd_rdy && !mshr_rd_same_line_blk
                     && axi_mshr_grant_vld
                     && (axi_mshr_grant == mi[MSHR_W-1:0]))
                        mshr_state[mi] <= M_RDATA;
                    // store merge：只合数据/strb，不改 is_st/killed（load 源仍可被 cancel）
                    if (lk_st_merge && (mshr_merge_idx == mi[MSHR_W-1:0])) begin
                        mshr_stb_line[mi] <= mshr_stb_line[mi] | req_stb_line;
                        mshr_line[mi]     <= (mshr_line[mi] & ~st_line_be)
                                           | (req_wdata & st_line_be);
                    end
                end
                M_RDATA: begin
                    if (ld_mshr_data_ok_o && (axi_mshr_grant == mi[MSHR_W-1:0]))
                        mshr_ld_resp_pend[mi] <= 1'b0;
                    if (mshr_beat0 && (axi_mshr_grant == mi[MSHR_W-1:0]))
                        mshr_b0[mi] <= axi_ret_data;
                    if (lk_st_merge && (mshr_merge_idx == mi[MSHR_W-1:0])) begin
                        mshr_stb_line[mi] <= mshr_stb_line[mi] | req_stb_line;
                        // beat1 同拍叠层已在 refill_line_merged；勿再写 overlay 覆盖整行
                        if (!(mshr_beat1 && (axi_mshr_grant == mi[MSHR_W-1:0]))) begin
                            mshr_line[mi] <= (mshr_line[mi] & ~st_line_be)
                                           | (req_wdata & st_line_be);
                        end
                    end
                    if (mshr_beat1 && (axi_mshr_grant == mi[MSHR_W-1:0])) begin
                        mshr_line[mi]  <= refill_line_merged;
                        mshr_state[mi] <= M_INSTALL;
                    end
                end
                M_INSTALL: begin
                    if (lk_st_merge && (mshr_merge_idx == mi[MSHR_W-1:0])) begin
                        mshr_stb_line[mi] <= mshr_stb_line[mi] | req_stb_line;
                        mshr_line[mi]     <= (mshr_line[mi] & ~st_line_be)
                                           | (req_wdata & st_line_be);
                    end
                    if (mshr_install_fire_oh[mi]) begin
                        mshr_state[mi]   <= M_IDLE;
                        mshr_killed[mi]  <= 1'b0;
                        mshr_from_ld[mi] <= 1'b0;
                    end
                end
                default: mshr_state[mi] <= M_IDLE;
            endcase
        end
    end
end

// ---------------- 写回缓冲排空引擎 ----------------
// 前端写状态（S_UC_WREQ/S_CAC_WB*）只在 wb_all_idle 时进入，
// 本引擎只在前端不占写通道时启动——互斥成立
wire front_wr_busy = (state == S_UC_WREQ) || (state == S_CAC_WB0) || (state == S_CAC_WB1);

always @(posedge clk) begin
    if (!resetn) begin
        wb_state <= W_IDLE;
        wb_valid <= 1'b0;
        wb_addr  <= 32'b0;
    end else begin
        case (wb_state)
            W_IDLE: begin
                if (wb_valid && !front_wr_busy) wb_state <= W_B0;
            end
            W_B0: if (axi_wr_rdy) wb_state <= W_B1;
            W_B1: begin
                wb_state <= W_IDLE;
                wb_valid <= 1'b0;
            end
            default: wb_state <= W_IDLE;
        endcase
        // 装入：LOOKUP miss 分配拍捕获脏 victim（分配前提 wb_all_idle，
        // 与上面清位不可能同拍，写在 case 后无冲突）
        if (mshr_alloc && miss_need_wb) begin
            wb_valid <= 1'b1;
            wb_addr  <= {tag_rd[pick_way], req_set, {`CACHE_LINE_W{1'b0}}};
            wb_line  <= data_out[pick_way];
        end
    end
end

// rr_ptr 上电清零（防 X；无复位需求，伪随机即可）
integer ri;
initial begin
    for (ri = 0; ri < NSET; ri = ri + 1) rr_ptr[ri] = 2'b0;
end

// lint 吸收（cancel 端口按契约忽略：响应由 LSU 配对丢弃；见头注）
wire dcache_lint = (|ld_vaddr_i[4:0]) | (|ld_size_i) | ld_cancel_i | mshr_rf_killed;

`ifdef SYNTHESIS
// synthesis translate_off
// 仿真性能统计：cached LOOKUP（load+store）；set 冲突不算命中也不算访问完成
reg [63:0] dc_access_total;
reg [63:0] dc_hit_total;
reg [63:0] dc_ld_access_total;
reg [63:0] dc_ld_hit_total;
reg [63:0] dc_st_access_total;
reg [63:0] dc_st_hit_total;
reg [63:0] dc_mwait_cycles;
reg [63:0] dc_pend_cycles;
reg [63:0] dc_mshr_busy_cycles;
reg [63:0] dc_pend_push_total;
always @(posedge clk) begin
    if (!resetn) begin
        dc_access_total     <= 64'd0;
        dc_hit_total        <= 64'd0;
        dc_ld_access_total  <= 64'd0;
        dc_ld_hit_total     <= 64'd0;
        dc_st_access_total  <= 64'd0;
        dc_st_hit_total     <= 64'd0;
        dc_mwait_cycles     <= 64'd0;
        dc_pend_cycles      <= 64'd0;
        dc_mshr_busy_cycles <= 64'd0;
        dc_pend_push_total  <= 64'd0;
    end else begin
        if (lk_cached_ld && !lk_set_conf) begin
            dc_access_total    <= dc_access_total + 64'd1;
            dc_ld_access_total <= dc_ld_access_total + 64'd1;
            if (hit_any) begin
                dc_hit_total    <= dc_hit_total + 64'd1;
                dc_ld_hit_total <= dc_ld_hit_total + 64'd1;
            end
        end
        if (lk_cached_st && !lk_set_conf) begin
            dc_access_total    <= dc_access_total + 64'd1;
            dc_st_access_total <= dc_st_access_total + 64'd1;
            if (hit_any) begin
                dc_hit_total    <= dc_hit_total + 64'd1;
                dc_st_hit_total <= dc_st_hit_total + 64'd1;
            end
        end
        if (state == S_MWAIT)
            dc_mwait_cycles <= dc_mwait_cycles + 64'd1;
        if (pend_valid)
            dc_pend_cycles <= dc_pend_cycles + 64'd1;
        if (mshr_busy)
            dc_mshr_busy_cycles <= dc_mshr_busy_cycles + 64'd1;
        if ((state == S_LOOKUP) && lk_to_pend)
            dc_pend_push_total <= dc_pend_push_total + 64'd1;
    end
end
// synthesis translate_on
`endif

endmodule

// ------------------------------------------------------------
// dcache_way_ram：单口同步 RAM 模板（128 x 256b，推断 BRAM）
// ------------------------------------------------------------
module dcache_way_ram(
    input  wire                          clk,
    input  wire                          en,
    input  wire                          we,
    input  wire [`L1_INDEX_W-1:0]        addr,
    input  wire [`CACHE_LINE_BITS-1:0]   wdata,
    output reg  [`CACHE_LINE_BITS-1:0]   rdata
);
reg [`CACHE_LINE_BITS-1:0] mem [0:`L1_NSET-1];
always @(posedge clk) begin
    if (en) begin
        if (we) mem[addr] <= wdata;
        else    rdata <= mem[addr];
    end
end
endmodule
