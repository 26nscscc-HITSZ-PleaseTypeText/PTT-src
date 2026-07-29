`include "mycpu.h"

// ============================================================
// l2cache 模块（L2 统一缓存：I$/D$ 共享的第二级缓存）
// ------------------------------------------------------------
// 几何与协议：
// - `L2_NWAY(2) 路 × `L2_NSET(2048) 组 × 32B 行 = 128KB，写回法并记录脏位；
// - 行协议（与 L1/axi_line_bridge 一致）：
//   * 读 type=100：rd_rdy 接受后 2 拍 128b 返回（ret_last 末拍）；
//   * 写 type=100：beat0 持 req 等 wr_rdy（接受拍 ack），beat1 次拍直推；
//   * 非行（uncached）：单拍旁路下层，写的 rdy=完成（B 已回）；
// - tag/data 均用推断 BRAM（接受/重查拍发地址，LOOKUP 拍比对）。
//
// 请求处理策略：
// - 仲裁：dc_wr > dc_rd/ic_rd 轮转 > I 侧 next-line 预取（最低）；
// - 行写命中 -> 整行更新置脏（cacop 写回则更新后保持干净并直通内存）；
//   行写 miss -> 直通下层内存（不分配）+ 存入 victim buffer；
// - 行读 miss：先查 victim buffer，命中直接 2 拍回行（不读内存、不分配）；
//   D 侧 miss 走主 FSM 内联重填（victim 脏先写回）；I 侧 miss 移交
//   "I-miss 引擎"后主 FSM 立即空闲（见下），实现 I/D 重填并行。
//
// I-miss 引擎与双 outstanding：
// - 本模块有两个下游读口：mem_rd_*（D 侧/旁路，接桥 dc 通道，ARID=1）、
//   mem2_rd_*（I 侧引擎专用，接桥 ic 通道，ARID=0）；
// - I 行读 LOOKUP miss（victim 干净或先经主 FSM 写回脏 victim）后，
//   {地址,组,路} 移交引擎，主 FSM 回 IDLE 继续服务 D 流量——I 重填与
//   D 命中/写/重填完全并行；
// - 引擎收完 2 拍数据后在"RAM 端口空闲拍"插队安装（安装拍屏蔽新请求
//   接受，防止同拍抢 BRAM 读口），demand 请求安装后 2 拍回行；
// - 同组冲突保守阻塞：引擎在飞期间，同 index 的新请求不予接受
//   （防安装/替换互踩；同行 demand 亦被此规则拦在授予级，等引擎装完
//   再进 LOOKUP 即命中）；I 读 miss 撞上引擎在飞（不同组）时不给 rdy，
//   上游保持请求自动重试（无等待状态，绝不阻塞 D 流量）。
//
// next-line I 侧预取：
// - I 侧 demand（命中/重填完成）后武装"下一行"预取（不跨 4KB 页）；
// - 预取只在【完全空闲拍】进入主 FSM 查表（优先级最低），命中/VB 命中/
//   victim 脏/引擎忙一律丢弃；查表后最多占引擎做一次干净重填，
//   绝不做写回，也不占用等待状态，避免预取阻塞 D$ 写回；
// - 顺序取指流中，下一行 demand 到来时若其预取仍在飞（同组被拦在授予级），
//   等待安装完成后进 LOOKUP 直接命中——预取的访存延迟已被重叠掉。
//
// victim buffer：
// - 4 项全相联，存"L1 换出且 L2 未命中"的整行（行写 miss 直通内存的
//   同时旁挂一份）；行读 miss 先查 VB，命中当拍锁存、2 拍回行；
// - 不变量：VB 与 L2 主体不同时持有同一行（安装/重填时清对应 VB 项），
//   避免 L2 干净换出后读到 VB 陈旧副本；
// - uncached 写命中 L2 行或 VB 行时失效对应副本，防止 cached/uncached
//   混用时读到陈旧数据。
//
// cacop 写回（dc_wr_cacop=1 的行写）：写穿语义——L2 命中则更新数据并
// 清脏位，且无论命中与否都直通写到内存（Linux 下 DMA/外设一致性要求
// 脏行真正落到 RAM，而不是停在 L2）。随后 D$ 通过 dc_cacop_* 发起同一
// 物理行的地址型维护：L2 命中脏行则先写回，命中干净行直接无效，同时
// 清除 victim buffer 同行；done 仅在维护全部完成后返回。
// ============================================================
module l2cache (
    input  wire         clk,
    input  wire         resetn,

    input  wire         ic_rd_req,
    input  wire [2:0]   ic_rd_type,     // 0=B 1=H 2=W 4=cacheline refill
    input  wire [31:0]  ic_rd_addr,
    output wire         ic_rd_rdy,
    output wire         ic_ret_valid,
    output wire         ic_ret_last,
    output wire [127:0] ic_ret_data,

    input  wire         dc_rd_req,
    input  wire [2:0]   dc_rd_type,
    input  wire [31:0]  dc_rd_addr,
    output wire         dc_rd_rdy,
    output wire         dc_ret_valid,
    output wire         dc_ret_last,
    output wire [127:0] dc_ret_data,

    input  wire         dc_wr_req,
    input  wire [2:0]   dc_wr_type,
    input  wire [31:0]  dc_wr_addr,
    input  wire [3:0]   dc_wr_strb,   // 仅 uncached 单拍写使用；行写为整行
    input  wire [127:0] dc_wr_data,
    input  wire         dc_wr_cacop,
    output wire         dc_wr_rdy,

    // ---------------- D$ 地址型维护口（Hit Writeback + Invalidate）----------------
    input  wire         dc_cacop_req,
    input  wire [31:0]  dc_cacop_addr,
    output wire         dc_cacop_done,

    // ---------------- 下游读口 0（D 侧/旁路，接桥 dc 通道 ARID=1）----------------
    output wire         mem_rd_req,
    output wire [2:0]   mem_rd_type,
    output wire [31:0]  mem_rd_addr,
    input  wire         mem_rd_rdy,
    input  wire         mem_ret_valid,
    input  wire         mem_ret_last,
    input  wire [127:0] mem_ret_data,

    // ---------------- 下游读口 1（I-miss 引擎专用，接桥 ic 通道 ARID=0）----------------
    output wire         mem2_rd_req,
    output wire [2:0]   mem2_rd_type,
    output wire [31:0]  mem2_rd_addr,
    input  wire         mem2_rd_rdy,
    input  wire         mem2_ret_valid,
    input  wire         mem2_ret_last,
    input  wire [127:0] mem2_ret_data,

    // ---------------- 下游写口 ----------------
    output wire         mem_wr_req,
    output wire [2:0]   mem_wr_type,
    output wire [31:0]  mem_wr_addr,
    output wire [3:0]   mem_wr_strb,
    output wire [127:0] mem_wr_data,
    input  wire         mem_wr_rdy
);

localparam NWAY  = `L2_NWAY;     // 2
localparam NSET  = `L2_NSET;     // 2048
localparam IDXW  = `L2_INDEX_W;
localparam TAGW  = `L2_TAG_W;
localparam LINEW = `CACHE_LINE_BITS;
localparam VB_N  = 4;            // victim buffer 项数

// ---------------- 主 FSM 状态 ----------------
localparam S_IDLE     = 4'd0;
localparam S_WCAP     = 4'd1;    // 行写 beat1 捕获 + 发 BRAM 地址
localparam S_LOOKUP   = 4'd2;
localparam S_MWB0     = 4'd3;    // victim 写回 beat0
localparam S_MWB1     = 4'd4;
localparam S_MRREQ    = 4'd5;    // D 侧整行读内存
localparam S_MRDATA   = 4'd6;
localparam S_RET0     = 4'd7;    // 上行返回 2 拍
localparam S_RET1     = 4'd8;
localparam S_WFWD0    = 4'd9;    // 行写 miss / cacop 写穿 直通内存
localparam S_WFWD1    = 4'd10;
localparam S_BYP_RREQ = 4'd11;   // uncached 读旁路
localparam S_BYP_RDATA= 4'd12;
localparam S_BYP_RET  = 4'd13;
localparam S_BYP_WREQ = 4'd14;   // uncached 写旁路（rdy=完成）
localparam S_CAC_DONE = 4'd15;   // 地址型维护完成脉冲

reg [3:0] state;

// ---------------- I-miss 引擎状态 ----------------
localparam IM_IDLE    = 3'd0;
localparam IM_RREQ    = 3'd1;    // 持 mem2_rd_req 等接受
localparam IM_RDATA   = 3'd2;    // 收 2 拍
localparam IM_INSTALL = 3'd3;    // 等 RAM 端口空闲拍安装
localparam IM_RET0    = 3'd4;    // demand（attach）2 拍回行
localparam IM_RET1    = 3'd5;

reg [2:0]       im_state;
reg             im_attach;       // 1=有上游取指在等（demand），0=纯预取
reg [31:5]      im_line_addr;    // 行地址
reg             im_way_r;        // NWAY=2，0/1 选择安装路
reg [127:0]     im_b0;
reg [LINEW-1:0] im_line;

wire im_busy = (im_state != IM_IDLE);
wire [IDXW-1:0] im_set = im_line_addr[IDXW+`CACHE_LINE_W-1:`CACHE_LINE_W];
wire [TAGW-1:0] im_tag = im_line_addr[31:IDXW+`CACHE_LINE_W];

// ---------------- 存储（推断 BRAM：每路 tag + data）----------------
// tag 条目：{valid, dirty, tag[TAGW-1:0]}
wire [TAGW+1:0]  tag_out  [0:NWAY-1];
wire [LINEW-1:0] data_out [0:NWAY-1];
reg  [IDXW-1:0]  ram_addr;
reg  [NWAY-1:0]  tag_we, data_we;
reg  [TAGW+1:0]  tag_wdata;
reg  [LINEW-1:0] data_wdata;
reg              ram_re;

genvar gr;
generate
for (gr = 0; gr < NWAY; gr = gr + 1) begin : gen_l2ram
    l2_tag_ram_t u_tag_ram(
        .clk(clk), .en(ram_re | tag_we[gr]), .we(tag_we[gr]),
        .addr(ram_addr), .wdata(tag_wdata), .rdata(tag_out[gr])
    );
    l2_data_ram_t u_data_ram(
        .clk(clk), .en(ram_re | data_we[gr]), .we(data_we[gr]),
        .addr(ram_addr), .wdata(data_wdata), .rdata(data_out[gr])
    );
end
endgenerate

// 复位期间逐组清 tag valid（上电 BRAM 内容未知）
reg            initing;
reg [IDXW-1:0] init_set;

// ---------------- 请求锁存 ----------------
reg        req_is_ic;       // 读请求来源
reg        req_is_rd;       // 1=读 0=写
reg        req_is_line;
reg        req_is_pf;       // 预取查表（丢结果，不回上游）
reg        req_cacop;       // cacop 写回（写穿语义）
reg        req_is_maint;    // D$ 下传的 L2 地址型写回/无效
reg [31:0] req_addr;
reg [2:0]  req_type;
reg [255:0] req_wline;      // 行写数据（2 拍拼）
reg [127:0] uc_wdata;
reg [3:0]   uc_strb;

wire [IDXW-1:0] req_set = req_addr[IDXW+`CACHE_LINE_W-1:`CACHE_LINE_W];
wire [TAGW-1:0] req_tag = req_addr[31:IDXW+`CACHE_LINE_W];

// ---------------- 命中判定（LOOKUP 拍）----------------
wire [NWAY-1:0] way_hit;
generate
for (gr = 0; gr < NWAY; gr = gr + 1) begin : gen_l2hit
    assign way_hit[gr] = tag_out[gr][TAGW+1] && (tag_out[gr][TAGW-1:0] == req_tag);
end
endgenerate
wire hit_any = |way_hit;
wire hit_way = way_hit[1];

// victim：无效路优先，否则轮转位
reg rr_bit;
wire pick_way = !tag_out[0][TAGW+1] ? 1'b0 :
                !tag_out[1][TAGW+1] ? 1'b1 : rr_bit;
wire pick_valid = pick_way ? tag_out[1][TAGW+1] : tag_out[0][TAGW+1];
wire pick_dirty = pick_way ? tag_out[1][TAGW]   : tag_out[0][TAGW];

reg        victim_way_r;
reg [TAGW-1:0] victim_tag_r;
reg [LINEW-1:0] victim_line_r;

reg [127:0] refill_b0;
reg [LINEW-1:0] ret_line;
reg [127:0] byp_data;

// ---------------- victim buffer（4 项全相联，行粒度）----------------
reg [VB_N-1:0]  vb_valid;
reg [26:0]      vb_addr [0:VB_N-1];    // 行地址 addr[31:5]
reg [LINEW-1:0] vb_data [0:VB_N-1];
reg [1:0]       vb_ptr;                // FIFO 替换指针

wire [26:0] req_line_key = req_addr[31:5];
wire [VB_N-1:0] vb_hit_vec;
genvar gv;
generate
for (gv = 0; gv < VB_N; gv = gv + 1) begin : gen_vb_hit
    assign vb_hit_vec[gv] = vb_valid[gv] && (vb_addr[gv] == req_line_key);
end
endgenerate
wire vb_hit = |vb_hit_vec;
reg [LINEW-1:0] vb_hit_line;
integer vi;
always @(*) begin
    vb_hit_line = {LINEW{1'b0}};
    for (vi = 0; vi < VB_N; vi = vi + 1)
        if (vb_hit_vec[vi]) vb_hit_line = vb_hit_line | vb_data[vi];
end

// ---------------- 预取武装 ----------------
reg        pf_armed;
reg [31:0] pf_addr;
// 只在 4KB 页内预取（跨页物理地址不连续，且可能触碰未映射区域）
wire [31:0] req_next_line = {req_addr[31:5], 5'b0} + 32'h20;
wire        req_pf_ok     = (req_addr[11:5] != 7'h7f);
wire [31:5] im_next_line_addr = im_line_addr + 27'd1;
wire [31:0] im_next_line  = {im_next_line_addr, 5'b0};
wire        im_pf_ok      = (im_line_addr[11:5] != 7'h7f);

// ---------------- 仲裁（IDLE 拍）----------------
// 同组冲突：引擎在飞期间同 index 请求不接受（防安装/替换互踩）
wire wr_set_conf   = im_busy && (dc_wr_addr[IDXW+`CACHE_LINE_W-1:`CACHE_LINE_W] == im_set);
wire dcrd_set_conf = im_busy && (dc_rd_addr[IDXW+`CACHE_LINE_W-1:`CACHE_LINE_W] == im_set);
wire icrd_set_conf = im_busy && (ic_rd_addr[IDXW+`CACHE_LINE_W-1:`CACHE_LINE_W] == im_set);
// 引擎安装拍抢占 BRAM 口，屏蔽新请求接受
wire ram_port_busy = (im_state == IM_INSTALL);

reg rr_rd_dc;
wire grant_base  = (state == S_IDLE) && !initing && !ram_port_busy;
wire wr_want     = dc_wr_req && !wr_set_conf;
wire dcrd_want   = dc_rd_req && !dcrd_set_conf;
wire icrd_want   = ic_rd_req && !icrd_set_conf;
// 维护等待 I-miss 引擎完全静默，防止同行在无效后又被后台安装。
wire grant_cacop = grant_base && dc_cacop_req && !im_busy;
wire grant_wr    = grant_base && !dc_cacop_req && wr_want;
wire grant_dc_rd = grant_base && !dc_cacop_req && !wr_want
                 && dcrd_want && (!icrd_want ||  rr_rd_dc);
wire grant_ic_rd = grant_base && !dc_cacop_req && !wr_want
                 && icrd_want && (!dcrd_want || !rr_rd_dc);
// 预取最低优先：任何上游请求（含被同组冲突压住的）在场都不进
wire grant_pf    = grant_base && !dc_cacop_req && !dc_wr_req && !dc_rd_req && !ic_rd_req
                && pf_armed && !im_busy;

// ---------------- 上行握手 ----------------
// 行写 beat0 接受 ack（mealy）；uncached 写的 rdy 在 BYP_WREQ 完成时
assign dc_wr_rdy = (grant_wr && (dc_wr_type == 3'b100))
                 | ((state == S_BYP_WREQ) && mem_wr_rdy);
assign dc_rd_rdy = grant_dc_rd;
assign dc_cacop_done = (state == S_CAC_DONE);

// ic 读的 rdy 推迟到 LOOKUP 拍（mealy）——只有当拍能"定去向"才接受：
// 命中/VB 命中/移交引擎/uncached 旁路；引擎忙（必为不同组，同组请求
// 在授予级已被拦）时不给 rdy，上游保持请求自动重试（绝不占主 FSM 等待）。
wire lk_ic_line_rd = (state == S_LOOKUP) && req_is_ic && req_is_rd && req_is_line && !req_is_pf;
wire lk_im_alloc   = lk_ic_line_rd && !hit_any && !vb_hit && !im_busy;
assign ic_rd_rdy = (lk_ic_line_rd && (hit_any || vb_hit))   // 主 FSM 2 拍回行
                 | (lk_im_alloc && !(pick_valid && pick_dirty))  // 干净 victim：当拍移交
                 | ((state == S_MWB1) && req_is_ic && req_is_rd) // 脏 victim：写回后移交拍
                 | ((state == S_LOOKUP) && req_is_ic && req_is_rd && !req_is_line); // uncached I 读

// ---------------- 上行返回（主 FSM + 引擎归并）----------------
wire ret_beat  = (state == S_RET0) || (state == S_RET1);
wire byp_beat  = (state == S_BYP_RET);
wire [127:0] ret_data_mux = byp_beat ? byp_data
                          : (state == S_RET0) ? ret_line[127:0] : ret_line[255:128];

wire im_ret_beat = (im_state == IM_RET0) || (im_state == IM_RET1);
wire [127:0] im_ret_data_mux = (im_state == IM_RET0) ? im_line[127:0] : im_line[255:128];

assign ic_ret_valid = (req_is_ic && (ret_beat || byp_beat)) | im_ret_beat;
assign ic_ret_last  = (req_is_ic && ((state == S_RET1) || byp_beat)) | (im_state == IM_RET1);
assign ic_ret_data  = im_ret_beat ? im_ret_data_mux : ret_data_mux;

assign dc_ret_valid = !req_is_ic && (ret_beat || byp_beat);
assign dc_ret_last  = !req_is_ic && ((state == S_RET1) || byp_beat);
assign dc_ret_data  = ret_data_mux;

// ---------------- 下游接口 ----------------
assign mem_rd_req  = (state == S_MRREQ) || (state == S_BYP_RREQ);
assign mem_rd_type = (state == S_MRREQ) ? 3'b100 : req_type;
assign mem_rd_addr = (state == S_MRREQ) ? {req_addr[31:`CACHE_LINE_W], {`CACHE_LINE_W{1'b0}}}
                                        : req_addr;

assign mem2_rd_req  = (im_state == IM_RREQ);
assign mem2_rd_type = 3'b100;
assign mem2_rd_addr = {im_line_addr, {`CACHE_LINE_W{1'b0}}};

assign mem_wr_req  = (state == S_MWB0) || (state == S_MWB1)
                   || (state == S_WFWD0) || (state == S_WFWD1)
                   || (state == S_BYP_WREQ);
assign mem_wr_type = (state == S_BYP_WREQ) ? req_type : 3'b100;
assign mem_wr_addr = ((state == S_WFWD0) || (state == S_WFWD1)) ? {req_addr[31:`CACHE_LINE_W], {`CACHE_LINE_W{1'b0}}}
                   : (state == S_BYP_WREQ) ? req_addr
                   : {victim_tag_r, req_set, {`CACHE_LINE_W{1'b0}}};
assign mem_wr_data = (state == S_MWB0)  ? victim_line_r[127:0]
                   : (state == S_MWB1)  ? victim_line_r[255:128]
                   : (state == S_WFWD0) ? req_wline[127:0]
                   : (state == S_WFWD1) ? req_wline[255:128]
                   : uc_wdata;
assign mem_wr_strb = (state == S_BYP_WREQ) ? uc_strb : 4'hf;

// ---------------- BRAM 控制 ----------------
wire [IDXW-1:0] idle_set = grant_cacop ? dc_cacop_addr[IDXW+`CACHE_LINE_W-1:`CACHE_LINE_W]
                         : grant_wr    ? dc_wr_addr[IDXW+`CACHE_LINE_W-1:`CACHE_LINE_W]
                         : grant_dc_rd ? dc_rd_addr[IDXW+`CACHE_LINE_W-1:`CACHE_LINE_W]
                         : grant_ic_rd ? ic_rd_addr[IDXW+`CACHE_LINE_W-1:`CACHE_LINE_W]
                                       : pf_addr[IDXW+`CACHE_LINE_W-1:`CACHE_LINE_W];

// 主 FSM 的 RAM 写事件
wire refill_wr_en = (state == S_MRDATA) && mem_ret_valid && mem_ret_last;
wire whit_wr_en   = (state == S_LOOKUP) && !req_is_rd && req_is_line && hit_any;
// uncached 写命中 L2 行：失效（防 cached/uncached 混用读到陈旧行）
wire ucw_inval_en = (state == S_LOOKUP) && !req_is_maint
                  && !req_is_rd && !req_is_line && hit_any;
wire maint_inval_en = (state == S_LOOKUP) && req_is_maint && hit_any;
// 引擎安装拍：主 FSM 无任何 RAM 活动时插队
wire im_install_fire = (im_state == IM_INSTALL) && !initing
                    && !refill_wr_en && !whit_wr_en && !ucw_inval_en && !maint_inval_en
                    && (state != S_WCAP)
                    && !(grant_wr || grant_dc_rd || grant_ic_rd || grant_pf);

always @(*) begin
    ram_re     = 1'b0;
    ram_addr   = idle_set;
    tag_we     = {NWAY{1'b0}};
    data_we    = {NWAY{1'b0}};
    tag_wdata  = {1'b1, 1'b0, req_tag};
    data_wdata = {mem_ret_data, refill_b0};
    if (initing) begin
        ram_addr  = init_set;
        tag_we    = {NWAY{1'b1}};
        tag_wdata = {(TAGW+2){1'b0}};
    end else if (im_install_fire) begin
        // I-miss 引擎安装（干净行）
        ram_addr             = im_set;
        data_we[im_way_r]    = 1'b1;
        data_wdata           = im_line;
        tag_we[im_way_r]     = 1'b1;
        tag_wdata            = {1'b1, 1'b0, im_tag};
    end else if (maint_inval_en) begin
        // DMA 地址型维护：数据若为脏已在 FSM 捕获，tag 当拍先无效。
        ram_addr            = req_set;
        tag_we[hit_way]     = 1'b1;
        tag_wdata           = {(TAGW+2){1'b0}};
    end else if (whit_wr_en) begin
        // 行写命中：整行更新；普通写置脏，cacop 写穿保持干净（数据同时直通内存）
        ram_addr            = req_set;
        data_we[hit_way]    = 1'b1;
        data_wdata          = req_wline;
        tag_we[hit_way]     = 1'b1;
        tag_wdata           = {1'b1, ~req_cacop, req_tag};
    end else if (ucw_inval_en) begin
        ram_addr            = req_set;
        tag_we[hit_way]     = 1'b1;
        tag_wdata           = {(TAGW+2){1'b0}};
    end else if (refill_wr_en) begin
        // D 侧重填：写新行（读 miss 分配，干净）
        ram_addr             = req_set;
        data_we[victim_way_r]= 1'b1;
        data_wdata           = {mem_ret_data, refill_b0};
        tag_we[victim_way_r] = 1'b1;
        tag_wdata            = {1'b1, 1'b0, req_tag};
    end else if (grant_cacop || (grant_wr && (dc_wr_type == 3'b100))
              || grant_dc_rd || grant_ic_rd || grant_pf) begin
        // 接受新请求拍发读（uncached 写也查表：为的是命中失效）
        ram_re   = 1'b1;
        ram_addr = idle_set;
    end else if (grant_wr) begin
        // uncached 写：也发一拍读用于命中失效判定
        ram_re   = 1'b1;
        ram_addr = idle_set;
    end else if (state == S_WCAP) begin
        // 行写 beat1 拍重发 BRAM 读（LOOKUP 拍比对）
        ram_re   = 1'b1;
        ram_addr = req_set;
    end
end

// ---------------- victim buffer 维护事件 ----------------
// 安装：行写 LOOKUP miss（普通写回/无 cacop 皆存；数据为完整新行）
wire vb_install_en = (state == S_LOOKUP) && !req_is_rd && req_is_line && !hit_any;
// 失效：行安装进 L2（引擎安装 / D 重填）、uncached 写命中 VB 行
wire [26:0] im_line_key     = im_line_addr;
wire [26:0] refill_line_key = req_addr[31:5];
wire [VB_N-1:0] vb_clr_im;
wire [VB_N-1:0] vb_clr_refill;
wire [VB_N-1:0] vb_clr_ucw;
wire [VB_N-1:0] vb_clr_maint;
generate
for (gv = 0; gv < VB_N; gv = gv + 1) begin : gen_vb_clr
    assign vb_clr_im[gv]     = im_install_fire && vb_valid[gv] && (vb_addr[gv] == im_line_key);
    assign vb_clr_refill[gv] = refill_wr_en    && vb_valid[gv] && (vb_addr[gv] == refill_line_key);
    assign vb_clr_ucw[gv]    = (state == S_LOOKUP) && !req_is_rd && !req_is_line
                             && vb_valid[gv] && (vb_addr[gv] == req_line_key);
    assign vb_clr_maint[gv]  = (state == S_LOOKUP) && req_is_maint
                             && vb_valid[gv] && (vb_addr[gv] == req_line_key);
end
endgenerate

integer vj;
always @(posedge clk) begin
    if (!resetn) begin
        vb_valid <= {VB_N{1'b0}};
        vb_ptr   <= 2'b0;
    end else begin
        vb_valid <= vb_valid & ~(vb_clr_im | vb_clr_refill | vb_clr_ucw | vb_clr_maint);
        if (vb_install_en) begin
            // 同行已有则原位覆盖（保持最新数据），否则 FIFO 替换
            if (vb_hit) begin
                for (vj = 0; vj < VB_N; vj = vj + 1)
                    if (vb_hit_vec[vj]) vb_data[vj] <= req_wline;
            end else begin
                vb_valid[vb_ptr] <= 1'b1;
                vb_addr[vb_ptr]  <= req_line_key;
                vb_data[vb_ptr]  <= req_wline;
                vb_ptr           <= vb_ptr + 2'd1;
            end
        end
    end
end

// ---------------- 主 FSM ----------------
always @(posedge clk) begin
    if (!resetn) begin
        state    <= S_IDLE;
        initing  <= 1'b1;
        init_set <= {IDXW{1'b0}};
        rr_bit   <= 1'b0;
        rr_rd_dc <= 1'b0;
        pf_armed <= 1'b0;
        pf_addr  <= 32'b0;
        req_is_ic <= 1'b0;
        req_is_rd <= 1'b0;
        req_is_line <= 1'b0;
        req_is_pf <= 1'b0;
        req_cacop <= 1'b0;
        req_is_maint <= 1'b0;
    end else if (initing) begin
        init_set <= init_set + 1'b1;
        if (init_set == {IDXW{1'b1}}) initing <= 1'b0;
    end else begin
        case (state)
            S_IDLE: begin
                if (grant_cacop) begin
                    req_is_rd    <= 1'b0;
                    req_is_pf    <= 1'b0;
                    req_is_line  <= 1'b0;
                    req_cacop    <= 1'b0;
                    req_is_maint <= 1'b1;
                    req_addr     <= dc_cacop_addr;
                    state        <= S_LOOKUP;
                end else if (grant_wr) begin
                    req_is_rd   <= 1'b0;
                    req_is_pf   <= 1'b0;
                    req_is_line <= (dc_wr_type == 3'b100);
                    req_cacop   <= dc_wr_cacop;
                    req_is_maint <= 1'b0;
                    req_addr    <= dc_wr_addr;
                    req_type    <= dc_wr_type;
                    if (dc_wr_type == 3'b100) begin
                        req_wline[127:0] <= dc_wr_data;
                        state            <= S_WCAP;
                    end else begin
                        uc_wdata <= dc_wr_data;
                        uc_strb  <= dc_wr_strb;
                        state    <= S_LOOKUP;   // 先查表做命中失效，再旁路写
                    end
                end else if (grant_dc_rd || grant_ic_rd) begin
                    req_is_rd   <= 1'b1;
                    req_is_pf   <= 1'b0;
                    req_cacop   <= 1'b0;
                    req_is_maint <= 1'b0;
                    req_is_ic   <= grant_ic_rd;
                    rr_rd_dc    <= grant_ic_rd;       // 轮转
                    req_is_line <= grant_ic_rd ? (ic_rd_type == 3'b100) : (dc_rd_type == 3'b100);
                    req_addr    <= grant_ic_rd ? ic_rd_addr : dc_rd_addr;
                    req_type    <= grant_ic_rd ? ic_rd_type : dc_rd_type;
                    state       <= S_LOOKUP;
                end else if (grant_pf) begin
                    req_is_rd   <= 1'b1;
                    req_is_pf   <= 1'b1;
                    req_cacop   <= 1'b0;
                    req_is_maint <= 1'b0;
                    req_is_ic   <= 1'b1;
                    req_is_line <= 1'b1;
                    req_addr    <= pf_addr;
                    req_type    <= 3'b100;
                    pf_armed    <= 1'b0;
                    state       <= S_LOOKUP;
                end
            end

            S_WCAP: begin
                req_wline[255:128] <= dc_wr_data;     // beat1 直推捕获
                state <= S_LOOKUP;
            end

            S_LOOKUP: begin
                if (req_is_maint) begin
                    // tag 已在组合块无效；脏行复用 victim 写回状态落到 RAM。
                    if (hit_any && tag_out[hit_way][TAGW]) begin
                        victim_tag_r  <= req_tag;
                        victim_line_r <= data_out[hit_way];
                        state         <= S_MWB0;
                    end else begin
                        state <= S_CAC_DONE;
                    end
                end else if (!req_is_rd) begin
                    if (!req_is_line) begin
                        // uncached 写：命中失效已在组合块完成，转旁路写
                        state <= S_BYP_WREQ;
                    end else if (req_cacop) begin
                        // cacop 写穿：命中时数据已更新且保持干净，仍直通内存
                        state <= S_WFWD0;
                    end else begin
                        // 普通行写：命中已整行更新置脏；miss 直通内存 + 入 VB
                        state <= hit_any ? S_IDLE : S_WFWD0;
                    end
                end else if (!req_is_line) begin
                    // uncached 读旁路（I/D 共用主 FSM，走下游读口 0）
                    state <= S_BYP_RREQ;
                end else if (req_is_pf) begin
                    // 预取查表：命中/VB 命中/引擎忙/victim 脏 一律丢弃；
                    // 只有"干净 miss + 引擎空闲"才移交引擎做后台重填
                    if (!hit_any && !vb_hit && !im_busy && !(pick_valid && pick_dirty)) begin
                        rr_bit <= ~rr_bit;
                    end
                    state <= S_IDLE;
                end else if (hit_any) begin
                    ret_line <= data_out[hit_way];
                    if (req_is_ic && req_pf_ok) begin
                        pf_armed <= 1'b1;           // demand 命中 -> 武装下一行
                        pf_addr  <= req_next_line;
                    end
                    state    <= S_RET0;
                end else if (vb_hit) begin
                    // victim buffer 命中：不读内存、不分配，直接回行
                    ret_line <= vb_hit_line;
                    if (req_is_ic && req_pf_ok) begin
                        pf_armed <= 1'b1;
                        pf_addr  <= req_next_line;
                    end
                    state    <= S_RET0;
                end else if (req_is_ic) begin
                    // I 侧 miss：移交引擎（干净 victim 当拍移交；脏先写回）；
                    // 引擎忙（必为异组）：不给 rdy 回 IDLE，上游重试（见 ic_rd_rdy）
                    victim_way_r  <= pick_way;
                    victim_tag_r  <= pick_way ? tag_out[1][TAGW-1:0] : tag_out[0][TAGW-1:0];
                    victim_line_r <= data_out[pick_way];
                    if (im_busy) begin
                        state <= S_IDLE;            // 引擎忙：重试
                    end else if (pick_valid && pick_dirty) begin
                        rr_bit <= ~rr_bit;
                        state  <= S_MWB0;           // 脏 victim 先写回再移交
                    end else begin
                        rr_bit <= ~rr_bit;
                        state  <= S_IDLE;           // 当拍移交引擎（见引擎块）
                    end
                end else begin
                    // D 侧 miss：主 FSM 内联重填（读口 0）
                    victim_way_r  <= pick_way;
                    victim_tag_r  <= pick_way ? tag_out[1][TAGW-1:0] : tag_out[0][TAGW-1:0];
                    victim_line_r <= data_out[pick_way];
                    rr_bit        <= ~rr_bit;
                    if (pick_valid && pick_dirty)
                        state <= S_MWB0;
                    else
                        state <= S_MRREQ;
                end
            end

            S_MWB0: if (mem_wr_rdy) state <= S_MWB1;
            S_MWB1: begin
                // I 侧：写回完成拍移交引擎回 IDLE；D 侧：继续内联重填
                state <= req_is_maint ? S_CAC_DONE
                     : (req_is_ic && req_is_rd) ? S_IDLE : S_MRREQ;
            end

            S_MRREQ: if (mem_rd_rdy) state <= S_MRDATA;
            S_MRDATA: begin
                if (mem_ret_valid) begin
                    if (mem_ret_last) begin
                        ret_line <= {mem_ret_data, refill_b0};
                        state    <= S_RET0;
                    end else begin
                        refill_b0 <= mem_ret_data;
                    end
                end
            end

            S_RET0: state <= S_RET1;
            S_RET1: state <= S_IDLE;

            S_WFWD0: if (mem_wr_rdy) state <= S_WFWD1;
            S_WFWD1: state <= S_IDLE;

            S_BYP_RREQ: if (mem_rd_rdy) state <= S_BYP_RDATA;
            S_BYP_RDATA: begin
                if (mem_ret_valid) begin
                    byp_data <= mem_ret_data;
                    state    <= S_BYP_RET;
                end
            end
            S_BYP_RET: state <= S_IDLE;

            S_BYP_WREQ: if (mem_wr_rdy) state <= S_IDLE;

            S_CAC_DONE: state <= S_IDLE;

            default: state <= S_IDLE;
        endcase

        // I demand 重填完成（引擎回行末拍）-> 武装下一行预取。
        // 放在 case 之后（同拍覆盖优先）：RET1 拍引擎必忙，grant_pf 的
        // 清位与 LOOKUP 拍的武装都不可能同拍发生，无真实冲突。
        if ((im_state == IM_RET1) && im_pf_ok) begin
            pf_armed <= 1'b1;
            pf_addr  <= im_next_line;
        end
    end
end

// ---------------- I-miss 引擎 ----------------
// 移交时机（与主 FSM 状态转移一一对应）：
// 1) LOOKUP：I demand 干净 miss 且引擎空闲（lk_im_alloc 且非脏）——当拍移交；
// 2) MWB1：I demand 脏 victim 写回完成拍移交；
// 3) LOOKUP：预取干净 miss 且引擎空闲——移交（attach=0）。
wire im_alloc_demand_clean = lk_im_alloc && !(pick_valid && pick_dirty);
wire im_alloc_demand_wb    = (state == S_MWB1) && req_is_ic && req_is_rd;
wire im_alloc_pf           = (state == S_LOOKUP) && req_is_pf
                          && !hit_any && !vb_hit && !im_busy
                          && !(pick_valid && pick_dirty);
wire im_alloc = im_alloc_demand_clean | im_alloc_demand_wb | im_alloc_pf;

always @(posedge clk) begin
    if (!resetn) begin
        im_state  <= IM_IDLE;
        im_attach <= 1'b0;
        im_line_addr <= 27'b0;
        im_way_r     <= 1'b0;
    end else begin
        case (im_state)
            IM_IDLE: begin
                if (im_alloc) begin
                    im_line_addr <= req_addr[31:5];
                    im_way_r     <= im_alloc_demand_wb ? victim_way_r : pick_way;
                    im_attach <= !im_alloc_pf;   // demand 要回数，纯预取只装表
                    im_state  <= IM_RREQ;
                end
            end
            IM_RREQ:  if (mem2_rd_rdy) im_state <= IM_RDATA;
            IM_RDATA: begin
                if (mem2_ret_valid) begin
                    if (mem2_ret_last) begin
                        im_line  <= {mem2_ret_data, im_b0};
                        im_state <= IM_INSTALL;
                    end else begin
                        im_b0 <= mem2_ret_data;
                    end
                end
            end
            IM_INSTALL: begin
                if (im_install_fire) begin
                    im_state <= im_attach ? IM_RET0 : IM_IDLE;
                end
            end
            IM_RET0: im_state <= IM_RET1;
            IM_RET1: begin
                im_attach <= 1'b0;
                im_state  <= IM_IDLE;
            end
            default: im_state <= IM_IDLE;
        endcase
    end
end

`ifdef SYNTHESIS
// synthesis translate_off
// 性能计数：L2 LOOKUP 行读命中和缺失，VB 命中计为 hit。
reg [63:0] l2_rd_access_total;
reg [63:0] l2_rd_hit_total;
reg [63:0] l2_rd_miss_total;
always @(posedge clk) begin
    if (!resetn) begin
        l2_rd_access_total <= 64'd0;
        l2_rd_hit_total    <= 64'd0;
        l2_rd_miss_total   <= 64'd0;
    end else if ((state == S_LOOKUP) && req_is_rd && req_is_line) begin
        // LOOKUP 单拍决策：行读 hit（含 VB）/ miss
        l2_rd_access_total <= l2_rd_access_total + 64'd1;
        if (hit_any || vb_hit)
            l2_rd_hit_total <= l2_rd_hit_total + 64'd1;
        else
            l2_rd_miss_total <= l2_rd_miss_total + 64'd1;
    end
end
// synthesis translate_on
`endif

endmodule

// ------------------------------------------------------------
// l2_tag_ram_t / l2_data_ram_t：单口同步 RAM 模板（推断 BRAM）
// ------------------------------------------------------------
module l2_tag_ram_t(
    input  wire                     clk,
    input  wire                     en,
    input  wire                     we,
    input  wire [`L2_INDEX_W-1:0]   addr,
    input  wire [`L2_TAG_W+1:0]     wdata,
    output reg  [`L2_TAG_W+1:0]     rdata
);
reg [`L2_TAG_W+1:0] mem [0:`L2_NSET-1];
always @(posedge clk) begin
    if (en) begin
        if (we) mem[addr] <= wdata;
        else    rdata <= mem[addr];
    end
end
endmodule

module l2_data_ram_t(
    input  wire                          clk,
    input  wire                          en,
    input  wire                          we,
    input  wire [`L2_INDEX_W-1:0]        addr,
    input  wire [`CACHE_LINE_BITS-1:0]   wdata,
    output reg  [`CACHE_LINE_BITS-1:0]   rdata
);
reg [`CACHE_LINE_BITS-1:0] mem [0:`L2_NSET-1];
always @(posedge clk) begin
    if (en) begin
        if (we) mem[addr] <= wdata;
        else    rdata <= mem[addr];
    end
end
endmodule
