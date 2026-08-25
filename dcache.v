`include "mycpu.h"

// ============================================================
// dcache 模块（L1 数据缓存，load/store 分离双口 + 非阻塞 miss）
// ------------------------------------------------------------
// 几何：
// - `L1_NWAY(4) 路 × `L1_NSET(128) 组 × `CACHE_LINE_BYTES(32B) 行 = 16KB；
// - VIPT：index+offset = 12 位落在 4KB 页内偏移（vaddr/paddr 的 [11:5]
//   恒等），load 用 vaddr 取 index、tag 用 paddr 比对；
// - 写回法 + 写分配；数据阵列每路一块推断 BRAM（128×256b 整行写），
//   tag 用 LUTRAM（异步读）、valid/dirty 用触发器（一拍判定/更新）。
//
// 结构：
// - 前端 FSM：一次锁存一个请求（store(SB) > load(LSU) > cacop 暂存），
//   IDLE 接受拍发 BRAM 读，LOOKUP 拍比对出结果——命中 load 两拍返回、
//   命中 store 两拍完成（合并写回阵列）；
// - 后台 MSHR（`DC_MSHR_DEPTH` 项，支持非阻塞 miss）：
//   * cached miss（load/store 皆可）分配进 MSHR 后【前端立即空闲】，
//     后续命中请求不受 miss 阻塞（hit-under-miss；配合 LSU 的 miss 槽）；
//   * load miss：LOOKUP 拍以 ld_miss_o 通知 LSU 移入 miss 槽，refill 数据
//     经 ld_mshr_data_ok_o/ld_mshr_rdata_o/ld_mshr_robid_o 独立通道返回；
//   * store miss：分配拍即回 st_done（posted，写效果由 MSHR 合并进重填行
//     保证落地），SB 立即排空下一条——隐藏 store miss 延迟；
//   * 同行 store 撞在飞 MSHR（同 paddr[31:5]）：合入该槽 byte enable，posted
//     st_done，不占 pend——消除 mem_stream 类 ld→st 同行走 set_conf；
//     只改写 byte enable 和数据叠层，不改变 load 来源及待响应状态；
//   * SB→D$ 使用整行接口（256b+32B strb），同行多字一次 RMW/单 MSHR；
//   * MSHR 在飞期间【同 index 不同行】或 MSHR 已满：优先挂到 1 项 pending
//     缓冲并立刻回 IDLE，从而继续 hit-under-miss；pending 在有空槽/写回空闲
//     后经 RELOOK 完成（同组冲突在重填落地后常变命中）。pending 已占用且再
//     撞冲突才退回 S_MWAIT（少见）；
//   * load/store miss 均可占满 N_MSHR；LSU 用 robid 配对多 miss 槽返回；
//   * AXI 读通道单 outstanding：owner 轮转服务各 MSHR 的 RREQ/RDATA；
// - 写回缓冲（1 项，写回与重填并行）：
//   脏 victim 在分配 MSHR 的同拍搬进写回缓冲，refill 读【立即发起】，
//   写回走独立写通道后台排空——dirty miss 不再串行"先写回后重填"；
// - Safe CWF-lite：refill beat0 且目标字在低半行、且该 MSHR 无 store-merge
//   时早回（数据取 axi beat）；否则等 beat1 用 refill_line_merged。安装可再
//   晚几拍（等 RAM 端口），不拦响应。
//
// uncached：
// - load：LSU 保证只在 ROB 头发出且不取消；等 MSHR/写回缓冲全空后独占
//   读通道单字访问（按 ld_size_i 真实宽度）；
// - store：等写通道空后单字直写，按 st_size_i 指定真实宽度；
//   rdy 表示下层已返回写响应。
//
// cacop（commit 提交级一拍脉冲，内部暂存后插队）：
// - op0(IDX_INV/StoreTag)：直接无效化指定 way（addr[1:0]），无写回；
// - op1(HIT_INV/Index 写回无效)：指定 way 脏则先写回（axi_wr_cacop=1，
//   L2 写穿直达内存）再无效；
// - op2(HIT_WB/Hit 写回无效)：按物理地址查命中，脏则写回再无效；L1
//   处理完成后把同一物理行下传给 L2，等待 L2 写回/无效完成，保证外部
//   DMA 不会被 L2 中的脏行或陈旧干净行遮蔽。
//
// 响应契约（取消 / 冲刷）：
// - 每个被接受（addr_ok）的 load 都【必定恰好产生一次前端响应】：
//   命中/uncached -> ld_data_ok_o；miss -> ld_miss_o。冲刷后过期的前端
//   响仍由 LSU d_drop 配对丢弃（契约无静默丢包，避免 d_drop 死锁）；
// - ld_cancel_i（= LSU flush）：已分配 load MSHR 保留 mshr_ld_resp_pend，仍发
//   ld_mshr_data_ok_o，由 LSU miss 槽 m_drop 静默收槽；冲刷后才分配的 load
//   通过 sticky req_ld_killed 令 pend=0，LSU d_drop 不占 miss 槽；
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
    input  wire [11:5] ld_vindex_i,      // 虚地址页内 index（VIPT 索引）
    input  wire [31:0] ld_paddr_i,       // 物理地址（tag 比对）
    input  wire [2:0]  ld_size_i,        // 0=B 1=H 2=W（uncached 精确宽度）
    input  wire [7:4]  ld_op_i,
    input  wire        ld_uncached_i,
    input  wire [`ROB_W-1:0] ld_robid_i, // 随 ld_req：miss 时锁入 MSHR，返回配对
    output wire        ld_addr_ok_o,
    output wire        ld_data_ok_o,     // 命中/uncached 完成（快速通道）
    output wire [31:0] ld_rdata_o,
    output wire [`ROB_W-1:0] ld_resp_robid_o, // 命中/前端 miss 对应请求标签
    input  wire        ld_resp_ready_i,  // LSU 命中响应暂存槽可接收
    input  wire        ld_cancel_i,      // 冲刷：在飞 load MSHR 置 killed 记账（data_ok 仍发，LSU m_drop 收槽，见头注契约）
    // ---- 非阻塞 miss 扩展（配合 LSU miss 槽）----
    output wire        ld_miss_o,        // 本 load 已移入 MSHR（LOOKUP 拍一拍脉冲）
    output wire        ld_mshr_data_ok_o,// MSHR load 数据返回（一拍脉冲）
    output wire [31:0] ld_mshr_rdata_o,
    output wire [`ROB_W-1:0] ld_mshr_robid_o, // 与 data_ok 同拍，供 LSU 配对
    input  wire        ld_pair_valid_i,  // second word in the same cache line
    input  wire [2:0]  ld_pair_word_i,
    input  wire [31:0] ld_pair_paddr_i,
    output wire        ld_pair_data_ok_o,
    output wire        ld_pair_miss_o,
    output wire [31:0] ld_pair_rdata_o,

    // ---------------- store_buffer 写出口（行粒度 data/strb）----------------
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
    output wire [3:0]  axi_wr_strb,      // 仅 uncached 单拍写使用；行写忽略 strobe
    output wire [127:0] axi_wr_data,
    output wire        axi_wr_cacop,
    input  wire        axi_wr_rdy,

    // ---------------- L2 地址型维护口（仅 op2）----------------
    output wire        l2_cacop_req,
    output wire [31:0] l2_cacop_addr,
    input  wire        l2_cacop_done
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
localparam S_CAC_L2   = 4'd10;  // 等 L2 对同一物理行完成写回/无效
localparam S_CAC_CAPTURE = 4'd11; // 隔离 LOOKUP 命中/地址到宽 CACOP 写回寄存器

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
reg             mshr_from_ld      [0:N_MSHR-1]; // 初始请求为 load miss；合并 store 后仍保持
reg [31:0]      mshr_paddr        [0:N_MSHR-1]; // 含目标字偏移
reg [`ROB_W-1:0] mshr_robid       [0:N_MSHR-1]; // load miss 配对（store 可忽略）
reg [7:4]       mshr_ld_op        [0:N_MSHR-1];
// One secondary load may merge into an already outstanding physical line.
// It consumes an LSU miss token but does not allocate another AXI request.
reg             mshr_ld2_pend     [0:N_MSHR-1];
reg [`ROB_W-1:0] mshr_ld2_robid   [0:N_MSHR-1];
reg [2:0]       mshr_ld2_word     [0:N_MSHR-1];
reg [1:0]       mshr_ld2_off      [0:N_MSHR-1];
reg [7:4]       mshr_ld2_op       [0:N_MSHR-1];
reg [1:0]       mshr_way          [0:N_MSHR-1];
reg [31:0]      mshr_stb_line     [0:N_MSHR-1]; // 行内 byte enable（同行多字 store 合并）
reg [127:0]     mshr_b0           [0:N_MSHR-1]; // AXI 首拍半行
// mshr_line 在 RREQ/RDATA 前半程保存 store 叠层，beat1 后保存待安装整行。
reg [LINEW-1:0] mshr_line         [0:N_MSHR-1];

wire [N_MSHR-1:0] mshr_busy_oh;
wire [N_MSHR-1:0] mshr_rreq_oh;
wire [N_MSHR-1:0] mshr_install_oh;

genvar gm;
generate
for (gm = 0; gm < N_MSHR; gm = gm + 1) begin : gen_mshr_status
    assign mshr_busy_oh[gm]     = (mshr_state[gm] != M_IDLE);
    assign mshr_rreq_oh[gm]     = (mshr_state[gm] == M_RREQ);
    assign mshr_install_oh[gm]  = (mshr_state[gm] == M_INSTALL);
end
endgenerate

wire              mshr_any_busy     = |mshr_busy_oh;
wire              mshr_has_free     = ~(&mshr_busy_oh);
wire              mshr_any_install  = |mshr_install_oh;
wire              mshr_rreq_vld      = |mshr_rreq_oh;
wire [MSHR_W-1:0] mshr_free_idx;
wire [MSHR_W-1:0] mshr_rreq_idx;
wire [MSHR_W-1:0] mshr_install_idx;
dc_prio_low_comb #(.N(N_MSHR), .W(MSHR_W)) u_prio_mshr_free(
    .mask_i(~mshr_busy_oh), .idx_o(mshr_free_idx)
);
dc_prio_low_comb #(.N(N_MSHR), .W(MSHR_W)) u_prio_mshr_rreq(
    .mask_i(mshr_rreq_oh), .idx_o(mshr_rreq_idx)
);
dc_prio_low_comb #(.N(N_MSHR), .W(MSHR_W)) u_prio_mshr_install(
    .mask_i(mshr_install_oh), .idx_o(mshr_install_idx)
);

// 性能统计使用的 MSHR 忙标志。
wire mshr_busy = mshr_any_busy;

// AXI 读通道 owner：同时只服务一个 MSHR（RREQ 受理 → RDATA 收完）
reg                axi_mshr_hold;
reg [MSHR_W-1:0]   axi_mshr_id;
wire               axi_mshr_grant_vld = axi_mshr_hold || mshr_rreq_vld;
wire [MSHR_W-1:0]  axi_mshr_grant     = axi_mshr_hold ? axi_mshr_id : mshr_rreq_idx;
wire [31:`CACHE_LINE_W] mshr_axi_line = mshr_paddr[axi_mshr_grant][31:`CACHE_LINE_W];
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
// valid/dirty 是大扇出的逐位 FF 阵列。把 LOOKUP/INSTALL 的动态下标写先
// 收敛到两个很小的更新包，下一拍再落阵列，可避免 req_paddr/tag-hit 组合锥
// 直接跨越布局区域驱动数百个阵列 D 端。下一条请求最早在本拍被接受、再下
// 一拍进入 LOOKUP，因此延后一拍落元数据不会改变其可见性。
reg                 meta_install_pend;
reg [1:0]           meta_install_way;
reg [IDXW-1:0]      meta_install_set;
reg                 meta_install_dirty;
reg                 meta_front_pend;
reg [1:0]           meta_front_way;
reg [IDXW-1:0]      meta_front_set;
reg                 meta_front_valid_we;
reg                 meta_front_valid;
reg                 meta_front_dirty_we;
reg                 meta_front_dirty;
wire [TAGW-1:0] tag_rd [0:NWAY-1];               // 各路 tag 在 req_set 处的异步读值

wire [TAGW-1:0] pair_tag_rd [0:NWAY-1];
wire [LINEW-1:0] data_out [0:NWAY-1];
wire [LINEW-1:0] pair_data_out [0:NWAY-1];
reg  [IDXW-1:0]  ram_addr;
reg  [NWAY-1:0]  ram_we;
reg  [LINEW-1:0] ram_wline;
reg              ram_re;
reg  [IDXW-1:0]  ram_pair_addr;
reg              ram_pair_re;

// Declared before the tag-RAM instances so Vivado does not create one-bit
// implicit nets while elaborating their write ports.
wire             mshr_install_fire;
wire [1:0]       mshr_inst_way;
wire [TAGW-1:0]  mshr_inst_tag;

// 每组替换指针（伪随机轮转）
reg [1:0] rr_ptr [0:NSET-1];

// ---------------- 请求锁存 ----------------
reg        req_is_st;
reg        req_is_ld;
reg        req_is_cacop;
reg [1:0]  req_cacop_op;
reg [31:0] req_paddr;
reg [7:4]  req_ld_op;
reg [N_MSHR-1:0] req_mshr_line_match_r;
// A chained load may start the next lookup only when its set was known free
// of in-flight MSHRs at accept time.  Registering this fact keeps the late
// tag-hit result out of the MSHR compare / merge cone on the accept path.
reg        req_chain_safe;
reg [`ROB_W-1:0] req_robid;       // load：随请求锁存，miss 写入 MSHR
reg [LINEW-1:0] req_wdata;        // 行粒度；UC 时有效字位于 req_word 槽
reg [31:0]  req_wstrb;            // 行内 32 字节使能
reg [2:0]  req_size;
reg        req_uncached;
reg        req_ld_killed;         // 本前端 load 已被冲刷（sticky→MSHR.killed）
reg        req_pair_valid;
reg [2:0]  req_pair_word;
reg [31:0] req_pair_paddr;
// valid/dirty are physically kept as resettable bit arrays, but LOOKUP must
// not dynamically index those arrays on its late tag-hit path.  Snapshot the
// four ways on the same edge that launches the synchronous tag/data read.
// Metadata writes are delayed by one cycle, so ram_*_meta_read explicitly
// forwards the pending updates that take effect on that capture edge.
reg [NWAY-1:0] req_valid_vec;
reg [NWAY-1:0] req_dirty_vec;
reg [NWAY-1:0] req_dirty_fwd_mask;
reg [NWAY-1:0] req_dirty_fwd_value;
reg [NWAY-1:0] req_pair_valid_vec;
reg [NWAY-1:0] ram_valid_read;
reg [NWAY-1:0] ram_dirty_raw;
reg [NWAY-1:0] ram_dirty_fwd_mask;
reg [NWAY-1:0] ram_dirty_fwd_value;
reg [NWAY-1:0] ram_pair_valid_read;
integer rv;
always @(*) begin
    ram_dirty_fwd_mask  = {NWAY{1'b0}};
    ram_dirty_fwd_value = {NWAY{1'b0}};
    for (rv = 0; rv < NWAY; rv = rv + 1) begin
        ram_valid_read[rv]      = valid_arr[rv][ram_addr];
        ram_dirty_raw[rv]       = dirty_arr[rv][ram_addr];
        ram_pair_valid_read[rv] = valid_arr[rv][ram_pair_addr];

        if (meta_install_pend && (meta_install_way == rv[1:0])) begin
            if (meta_install_set == ram_addr) begin
                ram_valid_read[rv] = 1'b1;
                ram_dirty_fwd_mask[rv]  = 1'b1;
                ram_dirty_fwd_value[rv] = meta_install_dirty;
            end
            if (meta_install_set == ram_pair_addr)
                ram_pair_valid_read[rv] = 1'b1;
        end

        // The sequential metadata block applies the front-end update after
        // the install update, so it has the same priority here.
        if (meta_front_pend && (meta_front_way == rv[1:0])) begin
            if (meta_front_set == ram_addr) begin
                if (meta_front_valid_we)
                    ram_valid_read[rv] = meta_front_valid;
                if (meta_front_dirty_we) begin
                    ram_dirty_fwd_mask[rv]  = 1'b1;
                    ram_dirty_fwd_value[rv] = meta_front_dirty;
                end
            end
            if ((meta_front_set == ram_pair_addr) && meta_front_valid_we)
                ram_pair_valid_read[rv] = meta_front_valid;
        end
    end
end

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
reg [7:4]  pend_ld_op;
reg        pend_uncached;
reg        pend_pair_valid;
reg [2:0]  pend_pair_word;
reg [31:0] pend_pair_paddr;

// cacop 写回中间量 / uncached 读数据
reg [TAGW-1:0]   cwb_tag;
reg [LINEW-1:0]  cwb_line;
reg [1:0]        cwb_way;
reg [31:0]       uc_rdata;

wire [IDXW-1:0] req_set = req_paddr[IDXW+`CACHE_LINE_W-1:`CACHE_LINE_W];
wire [TAGW-1:0] req_tag = req_paddr[31:IDXW+`CACHE_LINE_W];
wire [2:0]      req_word= req_paddr[4:2];
wire [IDXW-1:0] req_pair_set = req_pair_paddr[IDXW+`CACHE_LINE_W-1:`CACHE_LINE_W];
wire [TAGW-1:0] req_pair_tag = req_pair_paddr[31:IDXW+`CACHE_LINE_W];
wire req_pair_same_line = (req_paddr[31:`CACHE_LINE_W]
                         == req_pair_paddr[31:`CACHE_LINE_W]);

// ---------------- tag 阵列（per-way LUTRAM：1 写口 + req_set 异步读）----------------
// Tag and data arrays use the same registered request addresses.  A request
// starts both reads on its accept edge, so making the tag read synchronous
// does not add a lookup cycle.  It does remove req_set -> async LUTRAM from
// the hit-control cone and moves about 10 Kbit of tag storage into BRAM.
genvar gt;
generate
for (gt = 0; gt < NWAY; gt = gt + 1) begin : gen_tag
    dcache_tag_ram u_tag_ram(
        .clk    (clk),
        .en     (ram_re || (mshr_install_fire &&
                            (mshr_inst_way == gt[1:0]))),
        .we     (mshr_install_fire && (mshr_inst_way == gt[1:0])),
        .addr   (ram_addr),
        .wdata  (mshr_inst_tag),
        .rdata  (tag_rd[gt]),
        .en_b   (ram_pair_re),
        .addr_b (ram_pair_addr),
        .rdata_b(pair_tag_rd[gt])
    );
end
endgenerate

// ---------------- 命中判定（LOOKUP 拍，同步 tag/data 输出）----------------
wire [NWAY-1:0] way_hit;
genvar gw;
generate
for (gw = 0; gw < NWAY; gw = gw + 1) begin : gen_hit
    assign way_hit[gw] = req_valid_vec[gw] && (tag_rd[gw] == req_tag);
end
endgenerate
wire        hit_any = |way_hit;
wire [1:0]  hit_way = way_hit[1] ? 2'd1 : way_hit[2] ? 2'd2 : way_hit[3] ? 2'd3 : 2'd0;

wire [NWAY-1:0] pair_way_hit;
generate
for (gw = 0; gw < NWAY; gw = gw + 1) begin : gen_pair_hit
    assign pair_way_hit[gw] = req_pair_valid_vec[gw]
                            && (pair_tag_rd[gw] == req_pair_tag);
end
endgenerate
wire pair_hit_any_raw = |pair_way_hit;
wire [1:0] pair_hit_way = pair_way_hit[1] ? 2'd1
                         : pair_way_hit[2] ? 2'd2
                         : pair_way_hit[3] ? 2'd3 : 2'd0;

// victim 选择：无效路优先，否则轮转
wire [NWAY-1:0] way_invalid;
generate
for (gw = 0; gw < NWAY; gw = gw + 1) begin : gen_inv
    assign way_invalid[gw] = !req_valid_vec[gw];
end
endgenerate
wire [1:0] pick_way = way_invalid[0] ? 2'd0 :
                      way_invalid[1] ? 2'd1 :
                      way_invalid[2] ? 2'd2 :
                      way_invalid[3] ? 2'd3 : rr_ptr[req_set];
wire pick_valid = req_valid_vec[pick_way];
// The raw array read and same-edge metadata forwarding are captured
// separately.  This keeps the pending-update compare/mux off the long
// request-index -> dirty snapshot D path while preserving front-over-install
// priority exactly in the LOOKUP cycle.
wire [NWAY-1:0] req_dirty_vec_eff =
    (req_dirty_vec & ~req_dirty_fwd_mask)
  | (req_dirty_fwd_value & req_dirty_fwd_mask);
wire pick_dirty = req_dirty_vec_eff[pick_way];

// cacop 的 set/way 解码（op0/op1 按 addr 低位选 way）
wire [IDXW-1:0] cac_set = req_paddr[IDXW+`CACHE_LINE_W-1:`CACHE_LINE_W];
wire [1:0]      cac_way = req_paddr[1:0];

// ---------------- 接受仲裁（IDLE 拍）----------------
// MSHR 安装拍需要独占 RAM 口：安装等待期间暂停接受新请求（一拍气泡）
// pending 可 drain 时优先恢复挂起请求（不接受新请求），否则在 pend 占用时仍可
// 接受新请求以维持 hit-under-miss（再 miss/撞组才落 S_MWAIT）
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
wire lookup_ld_chain;
// A load hit only consumes the RAM read result; the port is free to start a
// pending store-buffer lookup in the same cycle.  This overlaps committed
// store draining with cached loads without adding an AGU/LSU bypass.
wire accept_ok = ((state == S_IDLE) || lookup_ld_chain)
              && !mshr_any_install && !pend_drain;
wire cacop_take = accept_ok && cacop_pend;
// Cached loads sit on the dependency-critical path, while committed stores
// are buffered.  Prefer an arriving load over a simultaneous store-buffer
// drain; the store remains asserted and is accepted on a later free cycle.
wire st_take    = accept_ok && !cacop_pend && !ld_req_i && st_req_i;
wire ld_take    = accept_ok && !cacop_pend && ld_req_i;

// store 接受拍预比较同行 MSHR，并随请求寄存到 LOOKUP。MSHR 的 line 在 busy
// 期间不变，因此这与下一拍用 req_paddr 比较等价，却把同行判定从 refill
// 返回锥中提前切开。pend 只有在无同 set MSHR 时恢复，直接写 0 即可。
wire [N_MSHR-1:0] st_take_line_match;
wire [N_MSHR-1:0] ld_take_line_match;
wire [N_MSHR-1:0] ld_take_set_match;
generate
for (gm = 0; gm < N_MSHR; gm = gm + 1) begin : gen_st_take_line_match
    assign st_take_line_match[gm] = mshr_busy_oh[gm]
                                  && (st_paddr_i[31:`CACHE_LINE_W]
                                   == mshr_paddr[gm][31:`CACHE_LINE_W]);
end
endgenerate
generate
for (gm = 0; gm < N_MSHR; gm = gm + 1) begin : gen_ld_take_line_match
    assign ld_take_line_match[gm] = mshr_busy_oh[gm]
                                  && (ld_paddr_i[31:`CACHE_LINE_W]
                                   == mshr_paddr[gm][31:`CACHE_LINE_W]);
    assign ld_take_set_match[gm] = mshr_busy_oh[gm]
                                 && (ld_vindex_i
                                  == mshr_paddr[gm][IDXW+`CACHE_LINE_W-1:`CACHE_LINE_W]);
end
endgenerate

assign st_addr_ok_o = st_take;
assign ld_addr_ok_o = ld_take;

// ---------------- LOOKUP 拍分类 ----------------
// 同 index 与任一在飞 MSHR 冲突：默认等待（防 victim 路互踩）
// 例外：同行 store 可合并进该 MSHR（降 pend），见 lk_st_merge
wire [N_MSHR-1:0] mshr_set_match;
wire [N_MSHR-1:0] mshr_line_match;
wire [N_MSHR-1:0] mshr_mergeable; // RREQ/RDATA/INSTALL 可合 store
wire [N_MSHR-1:0] mshr_ld_mergeable;
wire [N_MSHR-1:0] pair_mshr_set_match;
generate
for (gm = 0; gm < N_MSHR; gm = gm + 1) begin : gen_mshr_set_match
    assign mshr_set_match[gm] = mshr_busy_oh[gm]
                              && (req_set == mshr_paddr[gm][IDXW+`CACHE_LINE_W-1:`CACHE_LINE_W]);
    assign mshr_line_match[gm] = mshr_busy_oh[gm]
                              && req_mshr_line_match_r[gm];
    assign pair_mshr_set_match[gm] = mshr_busy_oh[gm]
                                   && (req_pair_set
                                    == mshr_paddr[gm][IDXW+`CACHE_LINE_W-1:`CACHE_LINE_W]);
    assign mshr_mergeable[gm] = mshr_line_match[gm]
                             && ((mshr_state[gm] == M_RREQ)
                              || (mshr_state[gm] == M_RDATA)
                              || (mshr_state[gm] == M_INSTALL));
    assign mshr_ld_mergeable[gm] = mshr_line_match[gm]
                                && !mshr_ld2_pend[gm]
                                && ((mshr_state[gm] == M_RREQ)
                                 || (mshr_state[gm] == M_RDATA));
end
endgenerate

wire pair_set_conf = !req_pair_same_line && (|pair_mshr_set_match);
wire pair_hit_any = req_pair_same_line ? hit_any
                  : (pair_hit_any_raw && !pair_set_conf);
wire        lk_st_merge = (state == S_LOOKUP) && req_is_st
                       && !req_is_cacop && !req_uncached
                       && (|mshr_mergeable);
wire [MSHR_W-1:0] mshr_merge_idx;
dc_prio_low_comb #(.N(N_MSHR), .W(MSHR_W)) u_prio_mshr_merge(
    .mask_i(mshr_mergeable), .idx_o(mshr_merge_idx)
);
wire        lk_ld_merge = (state == S_LOOKUP) && req_is_ld
                       && !req_is_cacop && !req_uncached
                       && (|mshr_ld_mergeable);
wire [MSHR_W-1:0] mshr_ld_merge_idx;
dc_prio_low_comb #(.N(N_MSHR), .W(MSHR_W)) u_prio_mshr_ld_merge(
    .mask_i(mshr_ld_mergeable), .idx_o(mshr_ld_merge_idx)
);
// SB 已给出行级 strb，无需按单字地址再次左移。
wire [31:0] req_stb_line = req_wstrb;

wire lk_set_conf  = (state == S_LOOKUP) && !req_is_cacop && !req_uncached
                 && (|mshr_set_match) && !lk_st_merge && !lk_ld_merge;
// cacop 需引擎全静默（MSHR 安装未落地前 tag 状态不完整，写回通道也要独占）
wire lk_cacop_wait = (state == S_LOOKUP) && req_is_cacop && (mshr_any_busy || !wb_all_idle);

wire lk_cached_ld = (state == S_LOOKUP) && req_is_ld && !req_is_cacop && !req_uncached;
wire lk_cached_st = (state == S_LOOKUP) && req_is_st && !req_is_cacop && !req_uncached;
wire lk_uc_ld     = (state == S_LOOKUP) && req_is_ld && !req_is_cacop && req_uncached;
wire lk_uc_st     = (state == S_LOOKUP) && req_is_st && !req_is_cacop && req_uncached;
wire lk_cacop     = (state == S_LOOKUP) && req_is_cacop && !lk_cacop_wait;

wire lk_ld_hit  = lk_cached_ld && !lk_set_conf && !lk_ld_merge && hit_any;
wire lk_st_hit  = lk_cached_st && !lk_set_conf && !lk_st_merge && hit_any;
wire lk_ld_miss = lk_cached_ld && !lk_set_conf && !lk_ld_merge && !hit_any;
wire lk_st_miss = lk_cached_st && !lk_set_conf && !lk_st_merge && !hit_any;
// 只有 LSU 能在本拍接收旧命中响应时才允许再发起下一次 BRAM 查询。
// 否则保持 LOOKUP 和 req_* 不动，让 data_ok/robid 成为标准 valid/ready
// 握手，避免负载流水化后因写回碰撞丢响应。
// Only chain when this cycle is a real completing load hit.  Using bare
// hit_any allowed accept_ok during set_conf/merge edge cases and let a new
// take steal the RAM port while the frontend still believed it was retiring
// a hit — next LOOKUP then saw stale tag/data (wrong ld.w data vs NEMU).
assign lookup_ld_chain = lk_ld_hit && req_chain_safe
                      && !(req_ld_killed || ld_cancel_i)
                      && ld_resp_ready_i;

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
// Select the requested word inside every BRAM way in parallel, then apply the
// one-hot tag hit.  This keeps the late tag comparison out of the former
// hit-way encoder -> 256-bit line mux -> word mux cascade.
wire [31:0] hit_way_word0 = data_out[0][32*req_word +: 32];
wire [31:0] hit_way_word1 = data_out[1][32*req_word +: 32];
wire [31:0] hit_way_word2 = data_out[2][32*req_word +: 32];
wire [31:0] hit_way_word3 = data_out[3][32*req_word +: 32];
wire [31:0] hit_word = ({32{way_hit[0]}} & hit_way_word0)
                     | ({32{way_hit[1]}} & hit_way_word1)
                     | ({32{way_hit[2]}} & hit_way_word2)
                     | ({32{way_hit[3]}} & hit_way_word3);
wire [LINEW-1:0] hit_pair_line = req_pair_same_line
                               ? hit_line : pair_data_out[pair_hit_way];
wire [31:0] hit_pair_word = hit_pair_line[32*req_pair_word +: 32];

// 行级字节使能扩展为位掩码。
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
wire [1:0]   mshr_rf_off   = mshr_paddr[axi_mshr_grant][1:0];
wire [7:4]   mshr_rf_op    = mshr_ld_op[axi_mshr_grant];
wire         mshr_rf_ld_resp = mshr_ld_resp_pend[axi_mshr_grant];
wire         mshr_owner_rdata = axi_mshr_grant_vld
                             && (mshr_state[axi_mshr_grant] == M_RDATA);

wire [LINEW-1:0] refill_line_raw = {axi_ret_data, mshr_rf_b0};
// 同拍 store merge 的同行判断来自接受拍寄存结果，不再经过 req_paddr 比较。
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

// Safe CWF-lite（访存可见性）：
// - beat1：整行已齐，数据取 refill_line_merged（含 MSHR store-merge + 同拍
//   lk_st_merge）。低半字若因 merge 未能早回，也在此拍回（勿只靠 beat0）。
// - beat0 早回：仅当目标字在低半行且 refill_stb_eff==0（无在飞/同拍 merge）。
//   早回数据必须取自 axi_ret_data 半行字——不可用 refill_line_merged：
//   beat0 时 refill_line_raw={axi_ret_data,mshr_b0} 尚未把低半行放到 [127:0]。
// - 早回必须同时满足低半行和无 store merge；否则统一等 beat1 的合并结果。
// cancel 不抑制 data_ok：LSU 用 m_drop 丢弃冲刷后返回。
wire [31:0] beat_word   = axi_ret_data[32*mshr_rf_word[1:0] +: 32];
wire [31:0] refill_word = refill_line_merged[32*mshr_rf_word +: 32];
wire        mshr_rf_no_st_merge = (mshr_stb_line[axi_mshr_grant] == 32'b0)
                               && !refill_merge_now;
wire        mshr_cwf_early = mshr_beat0 && !mshr_rf_word[2] && mshr_rf_no_st_merge;
wire        mshr_resp_fire = mshr_rf_ld_resp && (mshr_beat1 || mshr_cwf_early);
wire [31:0] mshr_resp_word = mshr_beat1 ? refill_word : beat_word;
wire [31:0] mshr_resp_data;
dc_load_shape_comb u_shape_mshr_resp(
    .word_i(mshr_resp_word), .op_i(mshr_rf_op), .off_i(mshr_rf_off),
    .data_o(mshr_resp_data)
);

// MSHR 安装拍：同一拍最多装 1 项（单口 BRAM）
// 注：accept_ok 已要求 !mshr_any_install，故 st/ld/cacop_take 与 install
// 互斥；勿再把 take 编入 front_ram_busy，否则 STA 会走出
// 避免形成 SB.query→ld_req→ld_take→!install_fire→valid_arr 的组合长路径。
// Block install while a load hit is still waiting for LSU accept — otherwise
// install redirects data-RAM addr and (before rdata hold fix) corrupts ld data.
// Do not busy the port for the ready/complete beat: that regresses refill
// overlap and caused earlier Linux difftest failures (~27M cycles).
wire front_ram_busy = (state == S_RELOOK) || lk_st_hit
                   || (lk_ld_hit && !ld_resp_ready_i);
wire [N_MSHR-1:0] mshr_install_fire_oh;
generate
for (gm = 0; gm < N_MSHR; gm = gm + 1) begin : gen_mshr_install_fire
    assign mshr_install_fire_oh[gm] =
        mshr_install_oh[gm] && !front_ram_busy
     && !(mshr_resp_fire && mshr_ld2_pend[gm])
     && (mshr_install_idx == gm[MSHR_W-1:0]);
end
endgenerate
assign mshr_install_fire = |mshr_install_fire_oh;
assign mshr_inst_way = mshr_way[mshr_install_idx];
wire [IDXW-1:0] mshr_inst_set = mshr_paddr[mshr_install_idx][IDXW+`CACHE_LINE_W-1:`CACHE_LINE_W];
assign mshr_inst_tag = mshr_paddr[mshr_install_idx][31:IDXW+`CACHE_LINE_W];
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
// The secondary merged load is backed by the completed, registered MSHR line
// as soon as its state reaches M_INSTALL.  Its response therefore need not
// depend on the late tag/data-RAM install enable (which includes valid/meta
// update arbitration).  Keep the primary refill response higher priority;
// the existing install gate retains an ld2-bearing MSHR on collisions.
wire        mshr_ld2_resp_fire = !mshr_resp_fire
                               && mshr_any_install
                               && mshr_ld2_pend[mshr_install_idx];
wire [31:0] mshr_ld2_resp_word = mshr_inst_line[
                                  32*mshr_ld2_word[mshr_install_idx] +: 32];
wire [31:0] mshr_ld2_resp_data;
dc_load_shape_comb u_shape_mshr_ld2_resp(
    .word_i(mshr_ld2_resp_word),
    .op_i(mshr_ld2_op[mshr_install_idx]),
    .off_i(mshr_ld2_off[mshr_install_idx]),
    .data_o(mshr_ld2_resp_data)
);
assign ld_mshr_data_ok_o = mshr_resp_fire | mshr_ld2_resp_fire;
assign ld_mshr_rdata_o   = mshr_ld2_resp_fire ? mshr_ld2_resp_data
                                             : mshr_resp_data;
assign ld_mshr_robid_o   = mshr_ld2_resp_fire
                         ? mshr_ld2_robid[mshr_install_idx]
                         : mshr_robid[axi_mshr_grant];
// ---------------- 响应输出 ----------------
assign ld_data_ok_o = (lk_ld_hit | (state == S_UC_RESP))
                    && !(req_ld_killed || ld_cancel_i);
// Keep byte/halfword extraction at the cache boundary.  req_* belongs to the
// request that produced this response and remains stable while a hit is held
// or an uncached response is in flight, so this is cycle-equivalent to the
// former LSU-side shaping while removing the DCache->LSU raw-word fanout.
wire [31:0] front_ld_word = (state == S_UC_RESP) ? uc_rdata : hit_word;
wire [31:0] front_ld_data;
dc_load_shape_comb u_shape_front_ld(
    .word_i(front_ld_word), .op_i(req_ld_op),
    .off_i(req_paddr[1:0]), .data_o(front_ld_data)
);
assign ld_rdata_o   = front_ld_data;
assign ld_pair_data_ok_o = lk_ld_hit && req_pair_valid && pair_hit_any
                         && !(req_ld_killed || ld_cancel_i);
// A fused pair falls back to two ordinary loads when either the primary line
// misses or its partner line is absent.  Register that decision at the cache
// boundary so req_paddr/tag lookup does not drive the LSU pair replay state
// across the hierarchy in the same cycle.
wire pair_miss_now = req_pair_valid && !(req_ld_killed || ld_cancel_i)
                   && (lk_ld_alloc || lk_ld_merge
                       || (lk_ld_hit && !pair_hit_any));
// At the relaxed 65 MHz target, return the fallback decision in the lookup
// cycle.  The LSU already owns a registered pair token before LOOKUP, so this
// only removes an otherwise idle replay bubble and does not alter ordering.
assign ld_pair_miss_o = pair_miss_now;
assign ld_pair_rdata_o = hit_pair_word;
assign ld_resp_robid_o = req_robid;
assign ld_miss_o    = (lk_ld_alloc || lk_ld_merge)
                    && !(req_ld_killed || ld_cancel_i);

// store：命中 / miss 分配 posted / 同行合入在飞 MSHR posted；uncached 等下层
assign st_done_o    = lk_st_hit | lk_st_alloc | lk_st_merge
                    | ((state == S_UC_WREQ) && axi_wr_rdy);

// ---------------- 下层读通道（MSHR owner 与 uncached 互斥）----------------
// 同行写读序保护：写回缓冲还压着与 refill 同一行的写时，refill 读必须等待。
wire mshr_owner_rreq = axi_mshr_grant_vld
                    && (mshr_state[axi_mshr_grant] == M_RREQ);
wire mshr_rd_same_line_blk = mshr_owner_rreq
                          && (wb_valid || (wb_state != W_IDLE))
                          && (wb_addr[31:`CACHE_LINE_W] == mshr_axi_line);
assign axi_rd_req  = (mshr_owner_rreq && !mshr_rd_same_line_blk)
                   || (state == S_UC_RREQ);
assign axi_rd_type = mshr_owner_rreq ? 3'b100 : req_size;
assign axi_rd_addr = mshr_owner_rreq ? {mshr_axi_line, {`CACHE_LINE_W{1'b0}}}
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
assign axi_wr_strb = (state == S_UC_WREQ) ? uc_st_strb : 4'hf;
assign axi_wr_cacop= (state == S_CAC_WB0) || (state == S_CAC_WB1);

assign l2_cacop_req  = (state == S_CAC_L2);
assign l2_cacop_addr = {req_paddr[31:`CACHE_LINE_W], {`CACHE_LINE_W{1'b0}}};

// ---------------- BRAM 读写控制 ----------------
// 读：IDLE 接受拍（地址=新请求 index）/ RELOOK 重发；
// 写：LOOKUP store 命中（整行读改写）/ MSHR 安装 —— 单口分拍复用
wire [IDXW-1:0] rd_set_idle = cacop_take ? cacop_pend_addr[IDXW+`CACHE_LINE_W-1:`CACHE_LINE_W]
                            : st_take    ? st_paddr_i[IDXW+`CACHE_LINE_W-1:`CACHE_LINE_W]
                                         : ld_vindex_i;

always @(*) begin
    // Pre-read whenever the frontend phase can consume the lookup port.
    // Request-valid is deliberately absent: on an empty cycle the outputs
    // are architecturally dead, while on an accept edge rd_set_idle already
    // carries the selected request index.  This keeps LSU request formation
    // and the empty-bypass FIFO out of every RAM-output/metadata D cone.
    ram_re    = (state == S_IDLE) || (state == S_RELOOK)
              || lookup_ld_chain;
    ram_pair_re = 1'b0;
    ram_we    = {NWAY{1'b0}};
    // During LOOKUP keep the registered address on the RAM input.  This
    // removes the tag-hit-selected next-request address from RAM timing.
    ram_addr  = (state == S_LOOKUP) ? req_set : rd_set_idle;
    ram_pair_addr = ld_pair_paddr_i[IDXW+`CACHE_LINE_W-1:`CACHE_LINE_W];
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
        ram_addr = req_set;
        ram_pair_re = req_pair_valid;
        ram_pair_addr = req_pair_set;
    end else if (st_take || ld_take || cacop_take) begin
        ram_addr = rd_set_idle;
        ram_pair_re = ld_take && ld_pair_valid_i;
        ram_pair_addr = ld_pair_paddr_i[IDXW+`CACHE_LINE_W-1:`CACHE_LINE_W];
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
        .rdata (data_out[gr]),
        .en_b  (ram_pair_re),
        .addr_b(ram_pair_addr),
        .rdata_b(pair_data_out[gr])
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
        req_pair_valid <= 1'b0;
        req_pair_paddr <= 32'b0;
        req_valid_vec  <= {NWAY{1'b0}};
        req_dirty_vec  <= {NWAY{1'b0}};
        req_dirty_fwd_mask  <= {NWAY{1'b0}};
        req_dirty_fwd_value <= {NWAY{1'b0}};
        req_pair_valid_vec <= {NWAY{1'b0}};
        pend_pair_valid <= 1'b0;
        pend_pair_paddr <= 32'b0;
        cwb_way         <= 2'b0;
        cwb_tag         <= {TAGW{1'b0}};
        cwb_line        <= {LINEW{1'b0}};
        req_mshr_line_match_r <= {N_MSHR{1'b0}};
        req_chain_safe  <= 1'b0;
        meta_install_pend <= 1'b0;
        meta_front_pend   <= 1'b0;
        for (s = 0; s < NWAY; s = s + 1) begin
            valid_arr[s] <= {NSET{1'b0}};
            dirty_arr[s] <= {NSET{1'b0}};
        end
    end else begin
        // Capture metadata beside the synchronous tag/data outputs.  ram_re
        // is asserted for every accepted request and every RELOOK retry.
        if (ram_re) begin
            req_valid_vec <= ram_valid_read;
            req_dirty_vec <= ram_dirty_raw;
            req_dirty_fwd_mask  <= ram_dirty_fwd_mask;
            req_dirty_fwd_value <= ram_dirty_fwd_value;
        end
        if (ram_pair_re)
            req_pair_valid_vec <= ram_pair_valid_read;

        // 先落上一拍的更新包；front 包在后，保持与原 always 块相同的优先级。
        // Pop and refill may happen together.  Capture comes after pop so the
        if (meta_install_pend) begin
            valid_arr[meta_install_way][meta_install_set] <= 1'b1;
            dirty_arr[meta_install_way][meta_install_set] <= meta_install_dirty;
        end
        if (meta_front_pend) begin
            if (meta_front_valid_we)
                valid_arr[meta_front_way][meta_front_set] <= meta_front_valid;
            if (meta_front_dirty_we)
                dirty_arr[meta_front_way][meta_front_set] <= meta_front_dirty;
        end
        meta_install_pend <= 1'b0;
        meta_front_pend   <= 1'b0;

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
            meta_install_pend  <= 1'b1;
            meta_install_way  <= mshr_inst_way;
            meta_install_set  <= mshr_inst_set;
            meta_install_dirty <= mshr_inst_is_st;
        end

        case (state)
            S_IDLE: begin
                if (pend_drain) begin
                    // 恢复挂起请求，重查（可能已因 refill 变命中）
                    req_is_cacop  <= 1'b0;
                    req_is_st     <= pend_is_st;
                    req_is_ld     <= pend_is_ld;
                    // 冲刷与 pend_drain 同拍时，894-896 因 state==S_IDLE 无法
                    // 置位 req_ld_killed；若只恢复 pend_ld_killed（旧值=0），
                    // 该请求会漏标 killed，随后 miss 分配 MSHR 仍带 pend=1，
                    // 重填数据按 ROBID 配对返回给冲刷后复用同 ROBID 的新 load，
                    // 造成错误行数据（Linux 0x00ecd84c ← 0x0011e918 事件）。
                    req_ld_killed <= pend_ld_killed | (pend_is_ld && ld_cancel_i);
                    req_paddr     <= pend_paddr;
                    req_mshr_line_match_r <= {N_MSHR{1'b0}};
                    // pend_can_progress proves that no same-set MSHR exists.
                    req_chain_safe <= pend_is_ld && !pend_uncached;
                    req_robid     <= pend_robid;
                    req_wdata     <= pend_wdata;
                    req_wstrb     <= pend_wstrb;
                    req_size      <= pend_size;
                    req_ld_op     <= pend_ld_op;
                    req_uncached  <= pend_uncached;
                    req_pair_valid<= pend_pair_valid;
                    req_pair_word <= pend_pair_word;
                    req_pair_paddr<= pend_pair_paddr;
                    pend_valid    <= 1'b0;
                    pend_ld_killed<= 1'b0;
                    pend_pair_valid <= 1'b0;
                    state         <= S_RELOOK;
                end else if (cacop_take) begin
                    req_is_cacop  <= 1'b1;
                    req_is_st     <= 1'b0;
                    req_is_ld     <= 1'b0;
                    req_ld_killed <= 1'b0;
                    req_uncached  <= 1'b0;
                    req_pair_valid<= 1'b0;
                    req_cacop_op  <= cacop_pend_op;
                    req_paddr     <= cacop_pend_addr;
                    req_mshr_line_match_r <= {N_MSHR{1'b0}};
                    req_chain_safe <= 1'b0;
                    cacop_pend    <= 1'b0;
                    state         <= S_LOOKUP;
                end else if (st_take) begin
                    req_is_cacop  <= 1'b0;
                    req_is_st     <= 1'b1;
                    req_is_ld     <= 1'b0;
                    req_ld_killed <= 1'b0;
                    req_paddr     <= st_paddr_i;
                    req_mshr_line_match_r <= st_take_line_match;
                    req_chain_safe <= 1'b0;
                    req_wdata     <= st_data_i;
                    req_wstrb     <= st_strb_i;
                    req_size      <= st_size_i;
                    req_uncached  <= st_uncached_i;
                    req_pair_valid<= 1'b0;
                    state         <= S_LOOKUP;
                end else if (ld_take) begin
                    req_is_cacop  <= 1'b0;
                    req_is_st     <= 1'b0;
                    req_is_ld     <= 1'b1;
                    req_ld_killed <= ld_cancel_i; // 同拍冲刷：直接 killed
                    req_paddr     <= ld_paddr_i;
                    req_mshr_line_match_r <= ld_take_line_match;
                    req_chain_safe <= !ld_uncached_i && !(|ld_take_set_match);
                    req_robid     <= ld_robid_i;
                    req_size      <= ld_size_i;
                    req_ld_op     <= ld_op_i;
                    req_uncached  <= ld_uncached_i;
                    req_pair_valid<= ld_pair_valid_i;
                    req_pair_word <= ld_pair_word_i;
                    req_pair_paddr<= ld_pair_paddr_i;
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
                    pend_ld_op     <= req_ld_op;
                    pend_uncached  <= req_uncached;
                    pend_pair_valid<= req_pair_valid;
                    pend_pair_word <= req_pair_word;
                    pend_pair_paddr<= req_pair_paddr;
                    req_ld_killed  <= 1'b0;
                    state          <= S_IDLE;
                end else if (lk_to_mwait_cache) begin
                    state <= S_MWAIT;
                end else if (lk_cacop) begin
                    // 到这里保证 MSHR/写回缓冲全静默（lk_cacop_wait 已滤掉）
                    case (req_cacop_op)
                        `CACOP_OP_IDX_INV: begin
                            // op0 StoreTag：直接无效化（无写回）
                            meta_front_pend     <= 1'b1;
                            meta_front_way      <= cac_way;
                            meta_front_set      <= cac_set;
                            meta_front_valid_we <= 1'b1;
                            meta_front_valid    <= 1'b0;
                            meta_front_dirty_we <= 1'b1;
                            meta_front_dirty    <= 1'b0;
                            state <= S_IDLE;
                        end
                        `CACOP_OP_HIT_INV: begin
                            // op1 Index 写回无效（way 由 addr[1:0] 指定）
                            if (req_valid_vec[cac_way] && req_dirty_vec_eff[cac_way]) begin
                                // 只在 LOOKUP 锁存 way；下一拍再由稳定 RAM 输出装入
                                // 宽写回寄存器，切断 req_paddr/hit 到 cwb_line CE。
                                cwb_way  <= cac_way;
                                meta_front_pend     <= 1'b1;
                                meta_front_way      <= cac_way;
                                meta_front_set      <= cac_set;
                                meta_front_valid_we <= 1'b1;
                                meta_front_valid    <= 1'b0;
                                meta_front_dirty_we <= 1'b1;
                                meta_front_dirty    <= 1'b0;
                                state <= S_CAC_CAPTURE;
                            end else begin
                                meta_front_pend     <= 1'b1;
                                meta_front_way      <= cac_way;
                                meta_front_set      <= cac_set;
                                meta_front_valid_we <= 1'b1;
                                meta_front_valid    <= 1'b0;
                                meta_front_dirty_we <= 1'b1;
                                meta_front_dirty    <= 1'b0;
                                state <= S_IDLE;
                            end
                        end
                        default: begin
                            // op2 Hit 写回无效：L1 完成后必须继续维护 L2。
                            if (hit_any) begin
                                if (req_dirty_vec_eff[hit_way]) begin
                                    cwb_way  <= hit_way;
                                    meta_front_pend     <= 1'b1;
                                    meta_front_way      <= hit_way;
                                    meta_front_set      <= req_set;
                                    meta_front_valid_we <= 1'b1;
                                    meta_front_valid    <= 1'b0;
                                    meta_front_dirty_we <= 1'b1;
                                    meta_front_dirty    <= 1'b0;
                                    state <= S_CAC_CAPTURE;
                                end else begin
                                    meta_front_pend     <= 1'b1;
                                    meta_front_way      <= hit_way;
                                    meta_front_set      <= req_set;
                                    meta_front_valid_we <= 1'b1;
                                    meta_front_valid    <= 1'b0;
                                    meta_front_dirty_we <= 1'b0;
                                    state <= S_CAC_L2;
                                end
                            end else begin
                                state <= S_CAC_L2;
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
                end else if (lk_ld_merge) begin
                    // The load owns an LSU miss token and waits for this
                    // MSHR's installed line; no second lower-level read.
                    req_ld_killed <= 1'b0;
                    req_pair_valid <= 1'b0;
                    state <= S_IDLE;
                end else if (lk_ld_hit
                          && !(req_ld_killed || ld_cancel_i)
                          && !ld_resp_ready_i) begin
                    // LSU hold full: keep current tag/data and robid until accept.
                    state <= S_LOOKUP;
                end else if (lk_ld_hit || lk_st_hit) begin
                    // 命中：load 出数 / store 合并写（组合块），本拍完成
                    if (req_is_st) begin
                        meta_front_pend     <= 1'b1;
                        meta_front_way      <= hit_way;
                        meta_front_set      <= req_set;
                        meta_front_valid_we <= 1'b0;
                        meta_front_dirty_we <= 1'b1;
                        meta_front_dirty    <= 1'b1;
                    end
                    // A completing load hit may use the free RAM read port to
                    // launch one waiting SB/LSU request. Keep that accepted
                    // request in LOOKUP instead of inserting an IDLE bubble.
                    if (cacop_take) begin
                        req_is_cacop  <= 1'b1;
                        req_is_st     <= 1'b0;
                        req_is_ld     <= 1'b0;
                        req_ld_killed <= 1'b0;
                        req_uncached  <= 1'b0;
                        req_pair_valid<= 1'b0;
                        req_cacop_op  <= cacop_pend_op;
                        req_paddr     <= cacop_pend_addr;
                        req_mshr_line_match_r <= {N_MSHR{1'b0}};
                        req_chain_safe <= 1'b0;
                        cacop_pend    <= 1'b0;
                        state         <= S_LOOKUP;
                    end else if (st_take) begin
                        req_is_cacop  <= 1'b0;
                        req_is_st     <= 1'b1;
                        req_is_ld     <= 1'b0;
                        req_ld_killed <= 1'b0;
                        req_paddr     <= st_paddr_i;
                        req_mshr_line_match_r <= st_take_line_match;
                        req_chain_safe <= 1'b0;
                        req_wdata     <= st_data_i;
                        req_wstrb     <= st_strb_i;
                        req_size      <= st_size_i;
                        req_uncached  <= st_uncached_i;
                        req_pair_valid<= 1'b0;
                        state         <= S_LOOKUP;
                    end else if (ld_take) begin
                        req_is_cacop  <= 1'b0;
                        req_is_st     <= 1'b0;
                        req_is_ld     <= 1'b1;
                        req_ld_killed <= ld_cancel_i;
                        req_paddr     <= ld_paddr_i;
                        req_mshr_line_match_r <= ld_take_line_match;
                        req_chain_safe <= !ld_uncached_i && !(|ld_take_set_match);
                        req_robid     <= ld_robid_i;
                        req_size      <= ld_size_i;
                        req_ld_op     <= ld_op_i;
                        req_uncached  <= ld_uncached_i;
                        req_pair_valid<= ld_pair_valid_i;
                        req_pair_word <= ld_pair_word_i;
                        req_pair_paddr<= ld_pair_paddr_i;
                        state         <= S_LOOKUP;
                    end else begin
                        req_ld_killed <= 1'b0;
                        req_pair_valid <= 1'b0;
                        state <= S_IDLE;
                    end
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
                    req_mshr_line_match_r <= {N_MSHR{1'b0}};
                    req_chain_safe <= req_is_ld && !req_uncached;
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
            S_UC_RESP: if ((req_ld_killed || ld_cancel_i) || ld_resp_ready_i) begin
                req_ld_killed <= 1'b0;
                state <= S_IDLE;
            end

            S_UC_WREQ: if (axi_wr_rdy) state <= S_IDLE;

            // LOOKUP 期间的数据 RAM 已同步读出；本状态不再发 RAM 读，输出
            // 保持稳定。用已寄存的 way 选择写回行，避免地址/命中长锥直接
            // 驱动 256-bit cwb_line 的 D/CE。
            S_CAC_CAPTURE: begin
                cwb_tag  <= tag_rd[cwb_way];
                cwb_line <= data_out[cwb_way];
                state    <= S_CAC_WB0;
            end
            S_CAC_WB0: if (axi_wr_rdy) state <= S_CAC_WB1;
            // beat1 直推一拍；op2 随后等待 L2 维护完成，op1 到此结束。
            S_CAC_WB1: state <= (req_cacop_op == `CACOP_OP_HIT_WB)
                               ? S_CAC_L2 : S_IDLE;
            S_CAC_L2: if (l2_cacop_done) state <= S_IDLE;

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
            mshr_from_ld[mi]      <= 1'b0;
            mshr_is_st[mi]        <= 1'b0;
            mshr_paddr[mi]        <= 32'b0;
            mshr_robid[mi]        <= {`ROB_W{1'b0}};
            mshr_ld_op[mi]        <= 4'b0;
            mshr_ld2_pend[mi]     <= 1'b0;
            mshr_ld2_robid[mi]    <= {`ROB_W{1'b0}};
            mshr_ld2_word[mi]     <= 3'b0;
            mshr_ld2_off[mi]      <= 2'b0;
            mshr_ld2_op[mi]       <= 4'b0;
            mshr_way[mi]          <= 2'b0;
            mshr_stb_line[mi]     <= 32'b0;
            mshr_line[mi]         <= {LINEW{1'b0}};
        end
    end else begin
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
            // 冲刷后旧 load 不再向 LSU 返回。重填/安装继续在后台完成，
            // 但响应资格在 DCache 内部取消，避免 ROB 标签复用后的 ABA。
            if (ld_cancel_i) begin
                mshr_ld_resp_pend[mi] <= 1'b0;
                mshr_ld2_pend[mi] <= 1'b0;
            end
            if (lk_ld_merge && !(req_ld_killed || ld_cancel_i)
             && (mshr_ld_merge_idx == mi[MSHR_W-1:0])) begin
                mshr_ld2_pend[mi]  <= 1'b1;
                mshr_ld2_robid[mi] <= req_robid;
                mshr_ld2_word[mi]  <= req_paddr[4:2];
                mshr_ld2_off[mi]   <= req_paddr[1:0];
                mshr_ld2_op[mi]    <= req_ld_op;
            end
            if (mshr_ld2_resp_fire
             && (mshr_install_idx == mi[MSHR_W-1:0]))
                mshr_ld2_pend[mi] <= 1'b0;
            case (mshr_state[mi])
                M_IDLE: begin
                    // Contents of an idle MSHR are architecturally dead.
                    // Preload the line every idle cycle so its CE depends
                    // only on the local MSHR state, not on the long
                    // req_paddr -> hit/victim -> allocation decision.
                    mshr_line[mi] <= req_is_st ? req_wdata : {LINEW{1'b0}};
                    if (mshr_alloc && (mshr_free_idx == mi[MSHR_W-1:0])) begin
                        mshr_is_st[mi]        <= req_is_st;
                        mshr_from_ld[mi]      <= req_is_ld;
                        mshr_ld_resp_pend[mi] <= req_is_ld && !(req_ld_killed || ld_cancel_i);
                        mshr_paddr[mi]        <= req_paddr;
                        mshr_robid[mi]        <= req_robid;
                        mshr_ld_op[mi]        <= req_ld_op;
                        mshr_ld2_pend[mi]     <= 1'b0;
                        mshr_way[mi]          <= pick_way;
                        if (req_is_st) begin
                            mshr_stb_line[mi] <= req_stb_line;
                            // The idle preload above already captured the
                            // complete store overlay line.
                        end else begin
                            mshr_stb_line[mi] <= 32'b0;
                        end
                        mshr_state[mi]        <= M_RREQ;
                    end
                end
                M_RREQ: begin
                    if (axi_rd_rdy && !mshr_rd_same_line_blk
                     && axi_mshr_grant_vld
                     && (axi_mshr_grant == mi[MSHR_W-1:0]))
                        mshr_state[mi] <= M_RDATA;
                    // store merge 只合数据和 strb，不改变 load 来源及待响应状态。
                    if (lk_st_merge && (mshr_merge_idx == mi[MSHR_W-1:0])) begin
                        mshr_stb_line[mi] <= mshr_stb_line[mi] | req_stb_line;
                        mshr_line[mi]     <= (mshr_line[mi] & ~st_line_be)
                                           | (req_wdata & st_line_be);
                    end
                end
                M_RDATA: begin
                    if (mshr_resp_fire && (axi_mshr_grant == mi[MSHR_W-1:0]))
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
                        mshr_from_ld[mi] <= 1'b0;
                        mshr_ld2_pend[mi] <= 1'b0;
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
// 性能计数：MSHR 占用（popcount busy）；cap=`DC_MSHR_DEPTH`。
reg [7:0]  dc_mshr_occ_now;
reg [7:0]  dc_mshr_occ_max;
reg [63:0] dc_mshr_occ_sum;
// 性能计数：load miss 服务延迟、CWF 机会和 store miss。
reg [15:0] mshr_age           [0:N_MSHR-1];
reg [63:0] dc_ld_miss_lat_sum;
reg [63:0] dc_ld_miss_n;
reg [15:0] dc_ld_miss_lat_max;
reg [63:0] dc_st_miss_total;
reg [63:0] dc_cwf_early_opp;   // beat0 且目标字在低半行（若开 CWF 可早回）
reg [63:0] dc_cwf_beat1_lo;    // 实际在 beat1 回的低半字 load（CWF 可加速）
reg [63:0] dc_cwf_beat1_hi;    // 高半字必须等 beat1
integer    dc_mshr_pc_i;
always @(*) begin
    dc_mshr_occ_now = 8'd0;
    for (dc_mshr_pc_i = 0; dc_mshr_pc_i < N_MSHR; dc_mshr_pc_i = dc_mshr_pc_i + 1)
        dc_mshr_occ_now = dc_mshr_occ_now + {7'd0, mshr_busy_oh[dc_mshr_pc_i]};
end
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
        dc_mshr_occ_max     <= 8'd0;
        dc_mshr_occ_sum     <= 64'd0;
        dc_ld_miss_lat_sum  <= 64'd0;
        dc_ld_miss_n        <= 64'd0;
        dc_ld_miss_lat_max  <= 16'd0;
        dc_st_miss_total    <= 64'd0;
        dc_cwf_early_opp    <= 64'd0;
        dc_cwf_beat1_lo     <= 64'd0;
        dc_cwf_beat1_hi     <= 64'd0;
        for (dc_mshr_pc_i = 0; dc_mshr_pc_i < N_MSHR; dc_mshr_pc_i = dc_mshr_pc_i + 1)
            mshr_age[dc_mshr_pc_i] <= 16'd0;
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
            end else
                dc_st_miss_total <= dc_st_miss_total + 64'd1;
        end
        if (state == S_MWAIT)
            dc_mwait_cycles <= dc_mwait_cycles + 64'd1;
        if (pend_valid)
            dc_pend_cycles <= dc_pend_cycles + 64'd1;
        if (mshr_busy)
            dc_mshr_busy_cycles <= dc_mshr_busy_cycles + 64'd1;
        if ((state == S_LOOKUP) && lk_to_pend)
            dc_pend_push_total <= dc_pend_push_total + 64'd1;
        dc_mshr_occ_sum <= dc_mshr_occ_sum + {56'd0, dc_mshr_occ_now};
        if (dc_mshr_occ_now > dc_mshr_occ_max)
            dc_mshr_occ_max <= dc_mshr_occ_now;

        // load miss age / latency；CWF 机会（与是否启用早回无关，用于估收益）
        for (dc_mshr_pc_i = 0; dc_mshr_pc_i < N_MSHR; dc_mshr_pc_i = dc_mshr_pc_i + 1) begin
            if (mshr_alloc && (mshr_free_idx == dc_mshr_pc_i[MSHR_W-1:0]) && req_is_ld)
                mshr_age[dc_mshr_pc_i] <= 16'd0;
            else if (mshr_busy_oh[dc_mshr_pc_i] && mshr_from_ld[dc_mshr_pc_i]
                  && (mshr_age[dc_mshr_pc_i] != 16'hffff))
                mshr_age[dc_mshr_pc_i] <= mshr_age[dc_mshr_pc_i] + 16'd1;
        end
        if (mshr_resp_fire) begin
            dc_ld_miss_n       <= dc_ld_miss_n + 64'd1;
            dc_ld_miss_lat_sum <= dc_ld_miss_lat_sum + {48'd0, mshr_age[axi_mshr_grant]};
            if (mshr_age[axi_mshr_grant] > dc_ld_miss_lat_max)
                dc_ld_miss_lat_max <= mshr_age[axi_mshr_grant];
            if (mshr_rf_word[2])
                dc_cwf_beat1_hi <= dc_cwf_beat1_hi + 64'd1;
            else if (mshr_beat1)
                dc_cwf_beat1_lo <= dc_cwf_beat1_lo + 64'd1;
        end
        if (mshr_rf_ld_resp && mshr_beat0 && !mshr_rf_word[2])
            dc_cwf_early_opp <= dc_cwf_early_opp + 64'd1;
    end
end
// synthesis translate_on
`endif

endmodule

module dc_prio_low_comb #(
    parameter integer N = 2,
    parameter integer W = 1
)(
    input  wire [N-1:0] mask_i,
    output reg  [W-1:0] idx_o
);
integer prio_i;
reg found;
always @(*) begin
    idx_o = {W{1'b0}};
    found = 1'b0;
    for (prio_i = 0; prio_i < N; prio_i = prio_i + 1) begin
        if (mask_i[prio_i] && !found) begin
            idx_o = prio_i[W-1:0];
            found = 1'b1;
        end
    end
end
endmodule

module dc_load_shape_comb(
    input  wire [31:0] word_i,
    input  wire [7:4]  op_i,
    input  wire [1:0]  off_i,
    output reg  [31:0] data_o
);
reg [7:0] byte_value;
reg [15:0] half_value;
always @(*) begin
    byte_value = word_i[8*off_i +: 8];
    half_value = off_i[1] ? word_i[31:16] : word_i[15:0];
    if (op_i[`MEM_OP_LD_B])
        data_o = {{24{byte_value[7]}}, byte_value};
    else if (op_i[`MEM_OP_LD_BU])
        data_o = {24'b0, byte_value};
    else if (op_i[`MEM_OP_LD_H])
        data_o = {{16{half_value[15]}}, half_value};
    else if (op_i[`MEM_OP_LD_HU])
        data_o = {16'b0, half_value};
    else
        data_o = word_i;
end
endmodule

// ------------------------------------------------------------
// dcache_tag_ram：双口同步 tag RAM（128 x 20b，推断 BRAM）
// ------------------------------------------------------------
module dcache_tag_ram(
    input  wire                       clk,
    input  wire                       en,
    input  wire                       we,
    input  wire [`L1_INDEX_W-1:0]     addr,
    input  wire [`L1_TAG_W-1:0]       wdata,
    output reg  [`L1_TAG_W-1:0]       rdata,
    input  wire                       en_b,
    input  wire [`L1_INDEX_W-1:0]     addr_b,
    output reg  [`L1_TAG_W-1:0]       rdata_b
);
// The registered tag read is timing-critical but only 10 Kbit across all
// ways.  Distributed RAM trades a few hundred LUTs for a much shorter
// clock-to-output delay than RAMB36, while preserving the lookup latency.
(* ram_style = "distributed" *) reg [`L1_TAG_W-1:0] mem [0:`L1_NSET-1];
always @(posedge clk) begin
    if (en) begin
        if (we) mem[addr] <= wdata;
        else    rdata <= mem[addr];
    end
    if (en_b)
        rdata_b <= mem[addr_b];
end
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
    output reg  [`CACHE_LINE_BITS-1:0]   rdata,
    input  wire                          en_b,
    input  wire [`L1_INDEX_W-1:0]        addr_b,
    output reg  [`CACHE_LINE_BITS-1:0]   rdata_b
);
(* ram_style = "block" *) reg [`CACHE_LINE_BITS-1:0] mem [0:`L1_NSET-1];
// Hold rdata when en==0 so a held LOOKUP hit is not corrupted when MSHR
// install steals ram_addr (tag RAM already gates this way).
// Do NOT assign rdata<=wdata on write: a held hit on the same way (different
// set) still needs the previous line while install/store writes mem[].
always @(posedge clk) begin
    if (en) begin
        if (we) mem[addr] <= wdata;
        else    rdata <= mem[addr];
    end
    if (en_b)
        rdata_b <= mem[addr_b];
end
endmodule
