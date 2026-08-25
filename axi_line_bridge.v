`include "mycpu.h"

// ============================================================
// axi_line_bridge 模块（cache 行请求 <-> 32bit AXI burst 转换桥）
// ------------------------------------------------------------
// 功能（32 B Cache 行与 AXI 事务转换）：
// - 行读（rd_type=100）：ARLEN=7（8 拍 32bit INCR 突发），每凑满 16B 即向
//   上游回一拍 128b（第 4 拍 ret_last=0、第 8 拍 ret_last=1，共 2 拍）；
// - 行写（wr_type=100）：上游 2 拍 128b 进（beat0 在 WR_IDLE 接受拍 ack，
//   beat1 次拍直推），缓冲整行后 AWLEN=7 突发写出；
// - 非行（uncached）：单拍传输，arsize/awsize 按真实宽度，写 rdy=B 完成
//  （外设写必须确认落地后才能继续，保证设备寄存器访问次序）；
// - 【AXI 优化】AW/W 并行发起：地址与数据通道同拍启动、互不等待
//   （W 可先于 AW 被接收，AXI 协议允许）；
// - 【AXI 优化】B 响应不阻塞下一笔行写：行写数据全部发出后立即回 IDLE
//   接收下一笔（在飞 B 用计数跟踪，最多 2 笔在飞；uncached 写不重叠）；
// - 【AXI 优化】双 outstanding 读：ic/dc 两个读通道各自独立引擎，
//   arid=0（取指）/1（数据）区分，R 通道按 rid 分流回两个 ret 口
//   ——AXI 允许异 ID 交错返回，chiplab 的 AXI RAM/交叉开关支持；
// - 顺序保证（读不越过在途写）：任何写未收到 B 前，两个读引擎都不发起
//   新 AR。写后读同地址（如 victim 写回后被重新 refill）依赖此规则。
//   写不等读：上游各级 cache 保证"同一行不会同时读+写"（victim 行地址
//   恒不等于 refill 行地址、L2 逐请求串行）。
//
// 端口：
// - ic_rd_*：I 侧读通道（连 l2cache 的 mem2_rd_*，I-miss 引擎专用）
// - dc_rd_*：D 侧读通道（连 l2cache 的 mem_rd_*）
// - dc_wr_*：写通道（连 l2cache 的 mem_wr_*）
// - axi_*  ：AXI4 主口（顶层再补常量 id/cache/prot 等；arid 由本桥给出）
// ============================================================
module axi_line_bridge (
    input  wire        clk,
    input  wire        resetn,

    input  wire        ic_rd_req,
    input  wire [2:0]  ic_rd_type,
    input  wire [31:0] ic_rd_addr,
    output wire        ic_rd_rdy,
    output wire        ic_ret_valid,
    output wire        ic_ret_last,
    output wire [127:0] ic_ret_data,

    input  wire        dc_rd_req,
    input  wire [2:0]  dc_rd_type,
    input  wire [31:0] dc_rd_addr,
    output wire        dc_rd_rdy,
    output wire        dc_ret_valid,
    output wire        dc_ret_last,
    output wire [127:0] dc_ret_data,

    input  wire        dc_wr_req,
    input  wire [2:0]  dc_wr_type,
    input  wire [31:0] dc_wr_addr,
    input  wire [3:0]  dc_wr_strb,
    input  wire [127:0] dc_wr_data,
    output wire        dc_wr_rdy,

    output wire        axi_awvalid,
    output wire [31:0] axi_awaddr,
    output wire [3:0]  axi_awlen,
    output wire [2:0]  axi_awsize,
    input  wire        axi_awready,

    output wire        axi_wvalid,
    output wire [31:0] axi_wdata,
    output wire [3:0]  axi_wstrb,
    output wire        axi_wlast,
    input  wire        axi_wready,

    input  wire        axi_bvalid,
    input  wire [1:0]  axi_bresp_unused, // 当前不生成 AXI 写错误异常。
    output wire        axi_bready,

    output wire        axi_arvalid,
    output wire [3:0]  axi_arid,        // 0=取指(ic) 1=数据(dc)
    output wire [31:0] axi_araddr,
    output wire [3:0]  axi_arlen,
    output wire [2:0]  axi_arsize,
    input  wire        axi_arready,

    input  wire        axi_rvalid,
    input  wire [3:0]  axi_rid,         // R 返回按 rid 路由到 ic/dc 引擎
    input  wire [31:0] axi_rdata,
    input  wire [1:0]  axi_rresp_unused, // 当前不生成 AXI 读错误异常。
    input  wire        axi_rlast,
    output wire        axi_rready
);

localparam ARID_IC = 4'd0;
localparam ARID_DC = 4'd1;

// ------------------------------------------------------------
// 写通道（AW/W 并行 + B 不阻塞下一笔行写）
// ------------------------------------------------------------
localparam WR_IDLE  = 2'd0;
localparam WR_CAP1  = 2'd1;   // 行写 beat1 捕获
localparam WR_ISSUE = 2'd2;   // AW 与 W 并行发出
localparam WR_UC_B  = 2'd3;   // uncached 写等待本笔 B（rdy=完成）

reg [1:0]   wr_state;
reg [31:0]  wr_addr_buf;
reg [255:0] wr_line_buf;
reg [3:0]   wr_strb_buf;
reg         wr_is_line;
reg [2:0]   wr_axsize_buf;
reg [2:0]   wr_beat;         // W 通道已发拍计数（0~7）
reg         aw_pend;         // AW 尚未被接收
reg         w_pend;          // W 数据尚未发完
reg [1:0]   b_cnt;           // 在飞未收 B 的写笔数（<=2）

wire aw_hs = axi_awvalid && axi_awready;
wire w_hs  = axi_wvalid  && axi_wready;
wire b_hs  = axi_bvalid  && axi_bready;

wire w_last_beat = wr_is_line ? (wr_beat == 3'd7) : 1'b1;
// 本拍 AW/W 双双完成（含早已完成的一侧）
wire issue_done = (wr_state == WR_ISSUE)
               && (!aw_pend || aw_hs)
               && (!w_pend  || (w_hs && w_last_beat));

// 写侧完全空闲（读引擎发起新 AR 的前提，保证"读不越过在途写"——
// 上游 cache 的"victim 先写回、同行再重填/写 miss 直通后再读"次序依赖此规则；
// 反向"写越过在途读"无需禁止：读已发出的事务地址与新写要么不同行、
// 要么属于软件需 ibar 保序的自修改代码场景，架构上允许）
wire wr_all_idle = (wr_state == WR_IDLE) && (b_cnt == 2'd0);

// 读请求等待中：暂停接纳新写（防连续写链把读饿死）；
// 读一旦起飞（引擎离开 IDLE），rd_pending 落下，写恢复接纳——写读交替公平
wire rd_pending;
`ifdef SOC_USE
// Linux SoC uses a single global AXI owner. This deliberately removes the
// read/write and IC/DC overlap permitted by the performance bridge, so every
// externally visible memory transaction is retired before the next starts.
wire rd_all_idle;
`endif

// 新写接纳条件：
// - 行写（victim/L2 写回）：允许 1 笔 B 在飞时继续收下一笔（b_cnt<2 流水化，
//   B 响应不阻塞下一笔 AW；同 ID 写 AXI 保证按序，语义安全）；
// - uncached 写：不与任何在飞写重叠（b_cnt==0），其 rdy=自身 B 完成
//   （外设寄存器写必须逐笔确认落地）；
// - 公平性：有读在等时让位（读是流水线停顿的关键路径，写是 posted 后台流量）。
`ifdef SOC_USE
wire wr_line_can_take = (wr_state == WR_IDLE) && (b_cnt == 2'd0)
                     && rd_all_idle && !rd_pending;
wire wr_uc_can_take   = (wr_state == WR_IDLE) && (b_cnt == 2'd0)
                     && rd_all_idle && !rd_pending;
`else
wire wr_line_can_take = (wr_state == WR_IDLE) && (b_cnt < 2'd2) && !rd_pending;
wire wr_uc_can_take   = (wr_state == WR_IDLE) && (b_cnt == 2'd0) && !rd_pending;
`endif
wire wr_take_line = wr_line_can_take && dc_wr_req && (dc_wr_type == 3'b100);
wire wr_take_uc   = wr_uc_can_take   && dc_wr_req && (dc_wr_type != 3'b100);

// ------------------------------------------------------------
// 读通道：ic / dc 两个独立引擎（双 outstanding）
// ------------------------------------------------------------
localparam RD_IDLE  = 2'd0;
localparam RD_AR    = 2'd1;
localparam RD_RDATA = 2'd2;

// ---- ic 引擎 ----
reg [1:0]   ic_state;
reg         ic_is_line;
reg [31:0]  ic_addr_buf;
reg [2:0]   ic_axsize_buf;
reg [2:0]   ic_beat;
reg [95:0]  ic_buf;           // 半行低 3 字暂存
reg         ic_ret_valid_r, ic_ret_last_r;
reg [127:0] ic_ret_data_r;
// ---- dc 引擎 ----
reg [1:0]   dc_state;
reg         dc_is_line;
reg [31:0]  dc_addr_buf;
reg [2:0]   dc_axsize_buf;
reg [2:0]   dc_beat;
reg [95:0]  dc_buf;
reg         dc_ret_valid_r, dc_ret_last_r;
reg [127:0] dc_ret_data_r;

reg         ar_rr_dc;         // AR 通道仲裁轮转位

`ifdef SOC_USE
assign rd_all_idle = (ic_state == RD_IDLE) && (dc_state == RD_IDLE);
`endif
// 读起飞：写侧全空且本拍没有新写被接纳（避免"同拍收写 + 读起飞"后
// AR 抢在 AW 前发出，破坏读不越写的保证）
wire rd_fire_ok = wr_all_idle && !(wr_take_line || wr_take_uc);
`ifdef SOC_USE
// The strict policy admits exactly one reader while both engines are idle.
// ar_rr_dc is retained only to alternate tie-breaking between IC and DC.
wire strict_ic_pick = ic_rd_req && (!dc_rd_req || !ar_rr_dc);
wire strict_dc_pick = dc_rd_req && (!ic_rd_req ||  ar_rr_dc);
wire ic_rd_fire = rd_all_idle && rd_fire_ok && strict_ic_pick;
wire dc_rd_fire = rd_all_idle && rd_fire_ok && strict_dc_pick;
`else
wire ic_rd_fire = (ic_state == RD_IDLE) && rd_fire_ok && ic_rd_req;
wire dc_rd_fire = (dc_state == RD_IDLE) && rd_fire_ok && dc_rd_req;
`endif
assign rd_pending = ((ic_state == RD_IDLE) && ic_rd_req)
                  | ((dc_state == RD_IDLE) && dc_rd_req);

// AR 仲裁：两引擎同拍都要发 AR 时 round-robin，一拍只发一个
wire ic_want_ar = (ic_state == RD_AR);
wire dc_want_ar = (dc_state == RD_AR);
wire ar_grant_dc = dc_want_ar && (!ic_want_ar ||  ar_rr_dc);
wire ar_grant_ic = ic_want_ar && (!dc_want_ar || !ar_rr_dc);

wire ic_ar_hs = ar_grant_ic && axi_arready;
wire dc_ar_hs = ar_grant_dc && axi_arready;

// R 通道按 rid 路由
wire r_to_ic = axi_rvalid && (axi_rid == ARID_IC);
wire r_to_dc = axi_rvalid && (axi_rid == ARID_DC);
wire ic_r_hs = r_to_ic && (ic_state == RD_RDATA);
wire dc_r_hs = r_to_dc && (dc_state == RD_RDATA);

always @(posedge clk) begin
    if (!resetn) begin
        wr_state <= WR_IDLE;
        wr_beat  <= 3'b0;
        aw_pend  <= 1'b0;
        w_pend   <= 1'b0;
        b_cnt    <= 2'b0;
        ic_state <= RD_IDLE;
        dc_state <= RD_IDLE;
        ic_beat  <= 3'b0;
        dc_beat  <= 3'b0;
        ic_ret_valid_r <= 1'b0;
        ic_ret_last_r  <= 1'b0;
        dc_ret_valid_r <= 1'b0;
        dc_ret_last_r  <= 1'b0;
        ar_rr_dc <= 1'b0;
    end else begin
        ic_ret_valid_r <= 1'b0;
        ic_ret_last_r  <= 1'b0;
        dc_ret_valid_r <= 1'b0;
        dc_ret_last_r  <= 1'b0;

        // ---------------- 写 FSM ----------------
        case (wr_state)
            WR_IDLE: begin
                if (wr_take_line) begin
                    wr_addr_buf        <= dc_wr_addr;
                    wr_axsize_buf      <= dc_wr_type;
                    wr_is_line         <= 1'b1;
                    wr_line_buf[127:0] <= dc_wr_data;    // beat0（本拍 rdy ack）
                    wr_beat            <= 3'b0;
                    wr_state           <= WR_CAP1;
                end else if (wr_take_uc) begin
                    wr_addr_buf        <= dc_wr_addr;
                    wr_axsize_buf      <= dc_wr_type;
                    wr_is_line         <= 1'b0;
                    wr_line_buf[127:0] <= dc_wr_data;
                    wr_strb_buf        <= dc_wr_strb[3:0];
                    wr_beat            <= 3'b0;
                    aw_pend            <= 1'b1;
                    w_pend             <= 1'b1;
                    wr_state           <= WR_ISSUE;
                end
            end
            WR_CAP1: begin
                wr_line_buf[255:128] <= dc_wr_data;      // beat1 直推捕获
                aw_pend              <= 1'b1;
                w_pend               <= 1'b1;
                wr_state             <= WR_ISSUE;
            end
            WR_ISSUE: begin
                if (aw_hs) aw_pend <= 1'b0;
                if (w_hs) begin
                    if (w_last_beat) w_pend <= 1'b0;
                    else             wr_beat <= wr_beat + 3'd1;
                end
                if (issue_done) begin
                    // 发完即回 IDLE（行写），B 由计数在后台等；uncached 等自身 B
                    wr_state <= wr_is_line ? WR_IDLE : WR_UC_B;
                end
            end
            WR_UC_B: begin
                if (b_hs && (b_cnt == 2'd1)) wr_state <= WR_IDLE;
            end
            default: wr_state <= WR_IDLE;
        endcase

        // 在飞 B 计数（issue 完成 +1 / B 到达 -1，同拍抵消）
        case ({issue_done, b_hs})
            2'b10:   b_cnt <= b_cnt + 2'd1;
            2'b01:   b_cnt <= b_cnt - 2'd1;
            default: b_cnt <= b_cnt;
        endcase

        // ---------------- ic 读引擎 ----------------
        case (ic_state)
            RD_IDLE: begin
                if (ic_rd_fire) begin
                    ic_addr_buf   <= ic_rd_addr;
                    ic_axsize_buf <= ic_rd_type;
                    ic_is_line    <= (ic_rd_type == 3'b100);
                    ic_beat       <= 3'b0;
                    ic_state      <= RD_AR;
                end
            end
            RD_AR: begin
                if (ic_ar_hs) begin
                    ic_state <= RD_RDATA;
                    ar_rr_dc <= 1'b1;    // 下次让 dc 先
                end
            end
            RD_RDATA: begin
                if (ic_r_hs) begin
                    if (ic_is_line) begin
                        if (ic_beat[1:0] == 2'd3) begin
                            ic_ret_data_r  <= {axi_rdata, ic_buf};
                            ic_ret_valid_r <= 1'b1;
                            ic_ret_last_r  <= (ic_beat == 3'd7);
                        end else begin
                            ic_buf[32*ic_beat[1:0] +: 32] <= axi_rdata;
                        end
                        ic_beat <= ic_beat + 3'd1;
                        if (axi_rlast) ic_state <= RD_IDLE;
                    end else begin
                        // 非行读：单拍 32bit（axi2apb 已做窄数据字节道对齐）
                        ic_ret_data_r  <= {96'b0, axi_rdata};
                        ic_ret_valid_r <= 1'b1;
                        ic_ret_last_r  <= 1'b1;
                        ic_state       <= RD_IDLE;
                    end
                end
            end
            default: ic_state <= RD_IDLE;
        endcase

        // ---------------- dc 读引擎 ----------------
        case (dc_state)
            RD_IDLE: begin
                if (dc_rd_fire) begin
                    dc_addr_buf   <= dc_rd_addr;
                    dc_axsize_buf <= dc_rd_type;
                    dc_is_line    <= (dc_rd_type == 3'b100);
                    dc_beat       <= 3'b0;
                    dc_state      <= RD_AR;
                end
            end
            RD_AR: begin
                if (dc_ar_hs) begin
                    dc_state <= RD_RDATA;
                    ar_rr_dc <= 1'b0;    // 下次让 ic 先
                end
            end
            RD_RDATA: begin
                if (dc_r_hs) begin
                    if (dc_is_line) begin
                        if (dc_beat[1:0] == 2'd3) begin
                            dc_ret_data_r  <= {axi_rdata, dc_buf};
                            dc_ret_valid_r <= 1'b1;
                            dc_ret_last_r  <= (dc_beat == 3'd7);
                        end else begin
                            dc_buf[32*dc_beat[1:0] +: 32] <= axi_rdata;
                        end
                        dc_beat <= dc_beat + 3'd1;
                        if (axi_rlast) dc_state <= RD_IDLE;
                    end else begin
                        dc_ret_data_r  <= {96'b0, axi_rdata};
                        dc_ret_valid_r <= 1'b1;
                        dc_ret_last_r  <= 1'b1;
                        dc_state       <= RD_IDLE;
                    end
                end
            end
            default: dc_state <= RD_IDLE;
        endcase
    end
end

// ------------------------------------------------------------
// 上行握手/返回
// ------------------------------------------------------------
// rd_rdy 在 AR 握手拍给出（上游据此进入收数状态）
assign ic_rd_rdy = ic_ar_hs;
assign dc_rd_rdy = dc_ar_hs;

assign ic_ret_valid = ic_ret_valid_r;
assign ic_ret_last  = ic_ret_last_r;
assign ic_ret_data  = ic_ret_data_r;

assign dc_ret_valid = dc_ret_valid_r;
assign dc_ret_last  = dc_ret_last_r;
assign dc_ret_data  = dc_ret_data_r;

// 行写 beat0 接受 ack（mealy）；非行（uncached）写 rdy = 自身 B 完成
assign dc_wr_rdy = wr_take_line
                 | ((wr_state == WR_UC_B) && b_hs && (b_cnt == 2'd1));

// ------------------------------------------------------------
// AXI 引脚
// ------------------------------------------------------------
assign axi_awvalid = (wr_state == WR_ISSUE) && aw_pend;
assign axi_awaddr  = wr_is_line ? {wr_addr_buf[31:`CACHE_LINE_W], {`CACHE_LINE_W{1'b0}}} : wr_addr_buf;
assign axi_awlen   = wr_is_line ? 4'd7 : 4'd0;
assign axi_awsize  = wr_is_line ? 3'b010 : wr_axsize_buf;

`ifdef SOC_USE
// Keep W behind its AW handshake. AXI permits independent channels, but
// the SoC policy intentionally makes write issue externally ordered.
assign axi_wvalid = (wr_state == WR_ISSUE) && w_pend && !aw_pend;
`else
assign axi_wvalid = (wr_state == WR_ISSUE) && w_pend;
`endif
assign axi_wdata  = wr_is_line ? wr_line_buf[32*wr_beat +: 32] : wr_line_buf[31:0];
assign axi_wstrb  = wr_is_line ? 4'hf : wr_strb_buf;
assign axi_wlast  = w_last_beat;

// B 常开接收：在飞 B 或 uncached 等待期均可收
assign axi_bready = (b_cnt != 2'd0);

assign axi_arvalid = ar_grant_ic | ar_grant_dc;
assign axi_arid    = ar_grant_dc ? ARID_DC : ARID_IC;
assign axi_araddr  = ar_grant_dc
                   ? (dc_is_line ? {dc_addr_buf[31:`CACHE_LINE_W], {`CACHE_LINE_W{1'b0}}} : dc_addr_buf)
                   : (ic_is_line ? {ic_addr_buf[31:`CACHE_LINE_W], {`CACHE_LINE_W{1'b0}}} : ic_addr_buf);
assign axi_arlen   = ar_grant_dc ? (dc_is_line ? 4'd7 : 4'd0)
                                 : (ic_is_line ? 4'd7 : 4'd0);
assign axi_arsize  = ar_grant_dc ? (dc_is_line ? 3'b010 : dc_axsize_buf)
                                 : (ic_is_line ? 3'b010 : ic_axsize_buf);

// R 接收：按 rid 找到目标引擎，其在收数状态即接收
assign axi_rready = (r_to_ic && (ic_state == RD_RDATA))
                  | (r_to_dc && (dc_state == RD_RDATA));

`ifdef SOC_USE
`ifdef SIMU
// Policy assertions catch accidental reintroduction of bridge concurrency.
always @(posedge clk) begin
    if (resetn) begin
        if ((ic_state != RD_IDLE) && (dc_state != RD_IDLE))
            $error("SOC_USE AXI bridge has more than one read outstanding");
        if (b_cnt > 2'd1)
            $error("SOC_USE AXI bridge has more than one write response outstanding");
        if (axi_wvalid && aw_pend)
            $error("SOC_USE AXI bridge sent W before AW handshake");
    end
end
`endif
`endif
endmodule
