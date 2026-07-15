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
// - 后台 MSHR（1 项，原 TODO 第四步·二期非阻塞 miss）：
//   * cached miss（load/store 皆可）分配进 MSHR 后【前端立即空闲】，
//     后续命中请求不受 miss 阻塞（hit-under-miss；配合 LSU 的 miss 槽）；
//   * load miss：LOOKUP 拍以 ld_miss_o 通知 LSU 移入 miss 槽，refill 数据
//     经 ld_mshr_data_ok_o/ld_mshr_rdata_o 独立通道返回一次；
//   * store miss：分配拍即回 st_done（posted，写效果由 MSHR 合并进重填行
//     保证落地），SB 立即排空下一条——隐藏 store miss 延迟；
//   * MSHR 在飞期间【同 index 的新请求】或第二 miss：优先挂到 1 项 pending
//     缓冲并立刻回 IDLE，从而继续 hit-under-miss；pending 在 MSHR/写回空闲
//     后经 RELOOK 完成（同组冲突在重填落地后常变命中）。pending 已占用且再
//     撞冲突才退回 S_MWAIT（少见）；
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
// 响应契约（取消语义，与 LSU 的 drop 机制配套）：
// - 每个被接受（addr_ok）的 load 都【必定恰好产生一次响应】：
//   命中/uncached -> ld_data_ok_o；miss -> ld_miss_o（其后恰好一次
//   ld_mshr_data_ok_o）。本模块忽略 ld_cancel_i（一拍脉冲难与多状态
//   在飞请求安全对齐），冲刷后的过期响应由 LSU 用 drop 标志配对丢弃
//   ——契约无静默丢包，LSU 侧不会出现"等一个永不到来的响应"死锁；
// - 被冲刷 load 的 miss 照常重填（行进 cache 大概率仍有用，无副作用；
//   uncached load 由 LSU 保证只在 ROB 头发出，绝不会被冲刷）；
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
    output wire        ld_addr_ok_o,
    output wire        ld_data_ok_o,     // 命中/uncached 完成（快速通道）
    output wire [31:0] ld_rdata_o,
    input  wire        ld_cancel_i,      // 冲刷取消在途 load（uncached 例外）
    // ---- 非阻塞 miss 扩展（配合 LSU miss 槽）----
    output wire        ld_miss_o,        // 本 load 已移入 MSHR（LOOKUP 拍一拍脉冲）
    output wire        ld_mshr_data_ok_o,// MSHR load 数据返回（一拍脉冲）
    output wire [31:0] ld_mshr_rdata_o,

    // ---------------- store_buffer 写出口 ----------------
    input  wire        st_req_i,         // store 写请求（保持至 addr_ok）
    input  wire [31:0] st_paddr_i,
    input  wire [31:0] st_data_i,
    input  wire [3:0]  st_strb_i,
    input  wire [2:0]  st_size_i,
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

// ---------------- MSHR（后台 refill 引擎）----------------
localparam M_IDLE    = 2'd0;
localparam M_RREQ    = 2'd1;
localparam M_RDATA   = 2'd2;
localparam M_INSTALL = 2'd3;

reg [1:0]   mshr_state;
reg         mshr_is_st;
reg         mshr_ld_resp_pend;   // load 响应待发（CWF：发过即清）
reg [31:0]  mshr_paddr;          // 含目标字偏移
reg [1:0]   mshr_way;
reg [31:0]  mshr_wdata;          // store miss 合并数据
reg [3:0]   mshr_wstrb;
reg [127:0] mshr_b0;
reg [LINEW-1:0] mshr_line;       // 已合并 store 的完整行

wire mshr_busy = (mshr_state != M_IDLE);
wire [IDXW-1:0] mshr_set = mshr_paddr[IDXW+`CACHE_LINE_W-1:`CACHE_LINE_W];
wire [TAGW-1:0] mshr_tag = mshr_paddr[31:IDXW+`CACHE_LINE_W];
wire [2:0]      mshr_word = mshr_paddr[4:2];

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
reg [NSET-1:0] valid_arr [0:NWAY-1];
reg [NSET-1:0] dirty_arr [0:NWAY-1];
reg [TAGW-1:0] tag_arr   [0:NWAY-1][0:NSET-1];   // LUTRAM（异步读）

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
reg [31:0] req_wdata;
reg [3:0]  req_wstrb;
reg [2:0]  req_size;
reg        req_uncached;

// cacop 暂存（commit 一拍脉冲，FSM 忙时排队；commit 串行发起，深度 1 足够）
reg        cacop_pend;
reg [1:0]  cacop_pend_op;
reg [31:0] cacop_pend_addr;

// cached 二次 miss / 同组撞 MSHR：挂起缓冲（释放前端继续 hit-under-miss）
reg        pend_valid;
reg        pend_is_st;
reg        pend_is_ld;
reg [31:0] pend_paddr;
reg [31:0] pend_wdata;
reg [3:0]  pend_wstrb;
reg [2:0]  pend_size;
reg        pend_uncached;

// cacop 写回中间量 / uncached 读数据
reg [TAGW-1:0]   cwb_tag;
reg [LINEW-1:0]  cwb_line;
reg [31:0]       uc_rdata;

wire [IDXW-1:0] req_set = req_paddr[IDXW+`CACHE_LINE_W-1:`CACHE_LINE_W];
wire [TAGW-1:0] req_tag = req_paddr[31:IDXW+`CACHE_LINE_W];
wire [2:0]      req_word= req_paddr[4:2];

// ---------------- 命中判定（LOOKUP 拍，tag LUTRAM 异步读）----------------
wire [NWAY-1:0] way_hit;
genvar gw;
generate
for (gw = 0; gw < NWAY; gw = gw + 1) begin : gen_hit
    assign way_hit[gw] = valid_arr[gw][req_set] && (tag_arr[gw][req_set] == req_tag);
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
wire mshr_res_idle = !mshr_busy && wb_all_idle;
wire pend_drain = (state == S_IDLE) && pend_valid && mshr_res_idle
               && (mshr_state != M_INSTALL);
wire accept_ok = (state == S_IDLE) && (mshr_state != M_INSTALL) && !pend_drain;
wire cacop_take = accept_ok && cacop_pend;
wire st_take    = accept_ok && !cacop_pend && st_req_i;
wire ld_take    = accept_ok && !cacop_pend && !st_req_i && ld_req_i;

assign st_addr_ok_o = st_take;
assign ld_addr_ok_o = ld_take;

// ---------------- LOOKUP 拍分类 ----------------
// 同 index 与在飞 MSHR 冲突：等待（防 victim 路互踩 / 丢失 store 更新；
// 等待后重查多半直接命中刚安装的行）
wire lk_set_conf  = (state == S_LOOKUP) && !req_is_cacop && !req_uncached
                 && mshr_busy && (req_set == mshr_set);
// cacop 需引擎全静默（MSHR 安装未落地前 tag 状态不完整，写回通道也要独占）
wire lk_cacop_wait = (state == S_LOOKUP) && req_is_cacop && (mshr_busy || !wb_all_idle);

wire lk_cached_ld = (state == S_LOOKUP) && req_is_ld && !req_is_cacop && !req_uncached;
wire lk_cached_st = (state == S_LOOKUP) && req_is_st && !req_is_cacop && !req_uncached;
wire lk_uc_ld     = (state == S_LOOKUP) && req_is_ld && !req_is_cacop && req_uncached;
wire lk_uc_st     = (state == S_LOOKUP) && req_is_st && !req_is_cacop && req_uncached;
wire lk_cacop     = (state == S_LOOKUP) && req_is_cacop && !lk_cacop_wait;

wire lk_ld_hit  = lk_cached_ld && !lk_set_conf && hit_any;
wire lk_st_hit  = lk_cached_st && !lk_set_conf && hit_any;
wire lk_ld_miss = lk_cached_ld && !lk_set_conf && !hit_any;
wire lk_st_miss = lk_cached_st && !lk_set_conf && !hit_any;

// miss 分配条件：MSHR 空闲 +（脏 victim 需写回缓冲全空）
wire miss_need_wb = pick_valid && pick_dirty;
wire mshr_alloc_ok = !mshr_busy && (!miss_need_wb || wb_all_idle);
wire lk_ld_alloc = lk_ld_miss && mshr_alloc_ok;
wire lk_st_alloc = lk_st_miss && mshr_alloc_ok;

// uncached 进入条件：独占读/写通道（MSHR 与写回缓冲全空，保守串行——
// uncached 本身就是强序访问，性能无关紧要）
wire lk_uc_ok = !mshr_busy && wb_all_idle;

// cached 资源冲突：可挂 pending（释放前端）；pend 已占用则被迫 MWAIT
wire lk_cache_block = lk_set_conf
                   || ((lk_ld_miss || lk_st_miss) && !mshr_alloc_ok);
wire lk_to_pend = lk_cache_block && !pend_valid;
wire lk_to_mwait_cache = lk_cache_block && pend_valid;

// ---------------- 命中数据通路 ----------------
wire [LINEW-1:0] hit_line = data_out[hit_way];
wire [31:0] hit_word = hit_line[32*req_word +: 32];

wire [31:0] st_word_strb = { {8{req_wstrb[3]}}, {8{req_wstrb[2]}},
                             {8{req_wstrb[1]}}, {8{req_wstrb[0]}} };

// store 命中合并行（读改写：用 LOOKUP 拍已读出的整行）
reg [LINEW-1:0] st_merge_line;
always @(*) begin
    st_merge_line = data_out[hit_way];
    st_merge_line[32*req_word +: 32] =
        (data_out[hit_way][32*req_word +: 32] & ~st_word_strb) | (req_wdata & st_word_strb);
end

// ---------------- MSHR 重填数据通路 ----------------
wire [LINEW-1:0] refill_line_raw = {axi_ret_data, mshr_b0};
wire [31:0] mshr_strb32 = { {8{mshr_wstrb[3]}}, {8{mshr_wstrb[2]}},
                            {8{mshr_wstrb[1]}}, {8{mshr_wstrb[0]}} };
reg [LINEW-1:0] refill_line_merged;
always @(*) begin
    refill_line_merged = refill_line_raw;
    if (mshr_is_st)
        refill_line_merged[32*mshr_word +: 32] =
            (refill_line_raw[32*mshr_word +: 32] & ~mshr_strb32) | (mshr_wdata & mshr_strb32);
end

// MSHR 数据接收（读通道归属：M_RDATA 期间返回都属 MSHR）
wire mshr_beat  = (mshr_state == M_RDATA) && axi_ret_valid;
wire mshr_beat0 = mshr_beat && !axi_ret_last;
wire mshr_beat1 = mshr_beat &&  axi_ret_last;

// CWF-lite：目标字所在半行一到即回数（load）
wire [31:0] beat_word = axi_ret_data[32*mshr_word[1:0] +: 32];
assign ld_mshr_data_ok_o = mshr_ld_resp_pend
                        && ((mshr_beat0 && !mshr_word[2])    // 低半行（word0~3）
                         || (mshr_beat1 &&  mshr_word[2]));  // 高半行（word4~7）
assign ld_mshr_rdata_o   = beat_word;

// MSHR 安装拍：需要 RAM 口空闲（接受已被 accept_ok 拦下；LOOKUP 拍的
// store 命中写 / RELOOK 拍的重发读占用时顺延一拍）
wire front_ram_busy = (st_take | ld_take | cacop_take)
                   || (state == S_RELOOK)
                   || lk_st_hit;
wire mshr_install_fire = (mshr_state == M_INSTALL) && !front_ram_busy;

// ---------------- 响应输出 ----------------
assign ld_data_ok_o = lk_ld_hit | (state == S_UC_RESP);
assign ld_rdata_o   = (state == S_UC_RESP) ? uc_rdata : hit_word;
assign ld_miss_o    = lk_ld_alloc;

// store：命中两拍完成；miss 分配拍 posted 完成；uncached 等下层
assign st_done_o    = lk_st_hit | lk_st_alloc
                    | ((state == S_UC_WREQ) && axi_wr_rdy);

// ---------------- 下层读通道（MSHR 与 uncached 互斥使用）----------------
// 同行写读序保护：写回缓冲还压着与 refill 同一行的写（此前该行被换出、
// 现在又立刻 miss 回来）时，refill 读必须等它先被 L2 收下——否则读会
// 越过在途写拿到内存里的陈旧行。不同行的写回照常与 refill 并行（本优化
// 的收益主体）。
wire mshr_rd_same_line_blk = (wb_valid || (wb_state != W_IDLE))
                          && (wb_addr[31:`CACHE_LINE_W] == mshr_paddr[31:`CACHE_LINE_W]);
assign axi_rd_req  = ((mshr_state == M_RREQ) && !mshr_rd_same_line_blk)
                   || (state == S_UC_RREQ);
assign axi_rd_type = (mshr_state == M_RREQ) ? 3'b100 : req_size;
assign axi_rd_addr = (mshr_state == M_RREQ) ? {mshr_paddr[31:`CACHE_LINE_W], {`CACHE_LINE_W{1'b0}}}
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
assign axi_wr_data = (state == S_UC_WREQ) ? {96'b0, req_wdata}
                   : (state == S_CAC_WB0) ? cwb_line[127:0]
                   : (state == S_CAC_WB1) ? cwb_line[255:128]
                   : (wb_state == W_B0)   ? wb_line[127:0]
                                          : wb_line[255:128];
assign axi_wr_strb = (state == S_UC_WREQ) ? {12'b0, req_wstrb} : 16'hffff;
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
        ram_we[mshr_way] = 1'b1;
        ram_addr         = mshr_set;
        ram_wline        = mshr_line;
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
        state      <= S_IDLE;
        cacop_pend <= 1'b0;
        pend_valid <= 1'b0;
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

        // MSHR 安装：更新 tag/valid/dirty（数据 RAM 写在组合块）
        if (mshr_install_fire) begin
            tag_arr[mshr_way][mshr_set]   <= mshr_tag;
            valid_arr[mshr_way][mshr_set] <= 1'b1;
            dirty_arr[mshr_way][mshr_set] <= mshr_is_st;
        end

        case (state)
            S_IDLE: begin
                if (pend_drain) begin
                    // 恢复挂起请求，重查（可能已因 refill 变命中）
                    req_is_cacop <= 1'b0;
                    req_is_st    <= pend_is_st;
                    req_is_ld    <= pend_is_ld;
                    req_paddr    <= pend_paddr;
                    req_wdata    <= pend_wdata;
                    req_wstrb    <= pend_wstrb;
                    req_size     <= pend_size;
                    req_uncached <= pend_uncached;
                    pend_valid   <= 1'b0;
                    state        <= S_RELOOK;
                end else if (cacop_take) begin
                    req_is_cacop <= 1'b1;
                    req_is_st    <= 1'b0;
                    req_is_ld    <= 1'b0;
                    req_uncached <= 1'b0;
                    req_cacop_op <= cacop_pend_op;
                    req_paddr    <= cacop_pend_addr;
                    cacop_pend   <= 1'b0;
                    state        <= S_LOOKUP;
                end else if (st_take) begin
                    req_is_cacop <= 1'b0;
                    req_is_st    <= 1'b1;
                    req_is_ld    <= 1'b0;
                    req_paddr    <= st_paddr_i;
                    req_wdata    <= st_data_i;
                    req_wstrb    <= st_strb_i;
                    req_size     <= st_size_i;
                    req_uncached <= st_uncached_i;
                    state        <= S_LOOKUP;
                end else if (ld_take) begin
                    req_is_cacop <= 1'b0;
                    req_is_st    <= 1'b0;
                    req_is_ld    <= 1'b1;
                    req_paddr    <= ld_paddr_i;
                    req_size     <= ld_size_i;
                    req_uncached <= ld_uncached_i;
                    state        <= S_LOOKUP;
                end
            end

            S_LOOKUP: begin
                if (lk_cacop_wait) begin
                    state <= S_MWAIT;
                end else if (lk_to_pend) begin
                    pend_valid    <= 1'b1;
                    pend_is_st    <= req_is_st;
                    pend_is_ld    <= req_is_ld;
                    pend_paddr    <= req_paddr;
                    pend_wdata    <= req_wdata;
                    pend_wstrb    <= req_wstrb;
                    pend_size     <= req_size;
                    pend_uncached <= req_uncached;
                    state         <= S_IDLE;
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
                                cwb_tag  <= tag_arr[cac_way][cac_set];
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
                                    cwb_tag  <= tag_arr[hit_way][req_set];
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
                end else if (hit_any) begin
                    // 命中：load 出数 / store 合并写（组合块），本拍完成
                    if (req_is_st) dirty_arr[hit_way][req_set] <= 1'b1;
                    state <= S_IDLE;
                end else if (mshr_alloc_ok) begin
                    // miss：分配 MSHR（脏 victim 同拍进写回缓冲，见排空引擎块），
                    // 前端即空闲；ld_miss_o / st_done_o(posted) 在本拍组合给出
                    rr_ptr[req_set] <= rr_ptr[req_set] + 2'd1;
                    state <= S_IDLE;
                end else begin
                    // 兜底（理论上 lk_* 已覆盖 cached block）
                    state <= S_MWAIT;
                end
            end

            S_MWAIT: begin
                // uncached / 双重 pending / cacop：等资源后重查
                if (!mshr_busy && wb_all_idle) begin
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
            S_UC_RESP: state <= S_IDLE;

            S_UC_WREQ: if (axi_wr_rdy) state <= S_IDLE;

            S_CAC_WB0: if (axi_wr_rdy) state <= S_CAC_WB1;
            S_CAC_WB1: state <= S_IDLE;        // beat1 直推一拍（下层保证连续接收）

            default: state <= S_IDLE;
        endcase
    end
end

// ---------------- MSHR 引擎 ----------------
wire mshr_alloc = lk_ld_alloc | lk_st_alloc;

always @(posedge clk) begin
    if (!resetn) begin
        mshr_state        <= M_IDLE;
        mshr_ld_resp_pend <= 1'b0;
        mshr_is_st        <= 1'b0;
        mshr_paddr        <= 32'b0;
        mshr_way          <= 2'b0;
        mshr_wdata        <= 32'b0;
        mshr_wstrb        <= 4'b0;
    end else begin
        case (mshr_state)
            M_IDLE: begin
                if (mshr_alloc) begin
                    mshr_is_st        <= req_is_st;
                    mshr_ld_resp_pend <= req_is_ld;   // load 需回一次数据
                    mshr_paddr        <= req_paddr;
                    mshr_way          <= pick_way;
                    mshr_wdata        <= req_wdata;
                    mshr_wstrb        <= req_wstrb;
                    mshr_state        <= M_RREQ;
                end
            end
            M_RREQ: if (axi_rd_rdy) mshr_state <= M_RDATA;
            M_RDATA: begin
                if (ld_mshr_data_ok_o) mshr_ld_resp_pend <= 1'b0;
                if (mshr_beat0) begin
                    mshr_b0 <= axi_ret_data;
                end
                if (mshr_beat1) begin
                    mshr_line  <= refill_line_merged;
                    mshr_state <= M_INSTALL;
                end
            end
            M_INSTALL: begin
                if (mshr_install_fire) mshr_state <= M_IDLE;
            end
            default: mshr_state <= M_IDLE;
        endcase
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
            wb_addr  <= {tag_arr[pick_way][req_set], req_set, {`CACHE_LINE_W{1'b0}}};
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
wire dcache_lint = (|ld_vaddr_i[4:0]) | (|ld_size_i) | ld_cancel_i;

`ifndef SYNTHESIS
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
