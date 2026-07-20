`include "mycpu.h"

// ============================================================
// icache 模块（L1 指令缓存，整行取指 + 简化冲刷语义）
// ------------------------------------------------------------
// 几何：
// - `L1_NWAY(4) 路 × `L1_NSET(128) 组 × `CACHE_LINE_BYTES(32B) 行 = 16KB；
// - VIPT：index=addr[11:5] 落在 4KB 页内（vaddr/paddr 同值），
//   接受拍用 vaddr 发 BRAM 索引、LOOKUP 拍用 paddr 比 tag；
// - 数据阵列每路一块推断 BRAM（128×256b 整行读写）；tag 用 LUTRAM
//   （异步读），valid 用触发器（一拍判定与失效）。
//
// 行为：
// - 整行输出：首个命中 2 拍出“整行”（IDLE 接受 -> LOOKUP 比对+出行）；
//   稳态命中时 LOOKUP 返回旧请求的同拍可接受下一请求，达到 1 请求/拍；
//   IFU 按块偏移自行切指令；
// - miss：整行读 L2（2 拍 128b，ret_last 末拍），重填后 RESP 拍出行；
// - 前端 IFU 用"在途请求配对 + drop 标志"自行
//   丢弃过期返回（见 ifu.v），本模块的返回与请求严格一一配对，
//   ifu_cancel_i 恒 0（保留端口以兼容接口约定）。
//
// uncached 取指：
// - 从块起始字逐字单拍读到行末（设备取指/未开 cache 阶段），拼成
//   "部分有效行"返回——IFU 只按块偏移切割有效部分，恰好不会用到
//   起始字之前的无效字节。
//
// cacop（commit 提交级一拍脉冲，暂存后插队）：
// - I$ 无脏行，三种 op 都是无效化，一拍完成：
//   op0(IDX_INV/StoreTag)、op1(HIT_INV/Index) 按 addr 的 {index, way[1:0]}
//   直接无效；op2(HIT_WB) 按物理地址查命中后无效；
// - ibar 语义不需要 I$ 全失效：LA32R 自修改代码由软件 cacop 逐行维护 +
//   ibar 屏障（commit 等 SB 排空 + FLUSH_REFETCH）保证，硬件无额外动作。
//
// - 双 outstanding：已在 L2 + axi_line_bridge 落地（I-miss 引擎走
//   ARID=0 读通道，与 D 侧 ARID=1 并行在飞），本模块无需感知；
// - 取指预取：L2 的 next-line I 侧预取已覆盖顺序取指流（本模块 demand
//   命中/重填后 L2 自动预取下一行）；FTQ 引导预取需 FTQ 提供跨块目标，
//   留待前端改造时在本模块加预取口；
// - critical-word-first 对本模块无意义：IFU 需要整行切指令，首字提前
//   返回并不能提前解除 IFU 等待，故不做（D 侧 CWF-lite 见 dcache.v）。
//
// 端口：
// - ifu_*   ：取指口（请求/整行返回/取消——取消恒 0 被忽略）
// - cacop_* ：cache 维护口（commit）
// - axi_*   ：下层 L2 接口（128bit/拍、行 2 拍，ret_last 标末拍）
// ============================================================
module icache (
    input  wire        clk,
    input  wire        resetn,

    // ---------------- IFU 取指口 ----------------
    input  wire        ifu_req_i,        // 取指请求（保持至 addr_ok）
    input  wire [31:0] ifu_vaddr_i,      // 虚地址（VIPT 索引）
    input  wire [31:0] ifu_paddr_i,      // 物理地址（tag 比对）
    input  wire        ifu_uncached_i,   // 非缓存取指
    output wire        ifu_addr_ok_o,    // 请求被接收
    output wire        ifu_data_ok_o,    // 整行数据有效（一拍）
    output wire [`CACHE_LINE_BITS-1:0] ifu_rline_o, // 整行指令数据
    input  wire        ifu_cancel_i,     // 冲刷作废在途（IFU 恒 0，本实现忽略）

    // ---------------- cache 维护口（commit 提交级驱动）----------------
    input  wire        cacop_en_i,
    input  wire [1:0]  cacop_op_i,       // `CACOP_OP_IDX_INV / HIT_INV / HIT_WB
    input  wire [31:0] cacop_addr_i,     // 物理地址

    // ---------------- 下层 L2 接口 ----------------
    output wire        axi_rd_req,
    output wire [2:0]  axi_rd_type,      // 0=B 1=H 2=W 4=cacheline refill
    output wire [31:0] axi_rd_addr,
    input  wire        axi_rd_rdy,
    input  wire        axi_ret_valid,
    input  wire        axi_ret_last,
    input  wire [127:0] axi_ret_data
);

localparam NWAY  = `L1_NWAY;
localparam NSET  = `L1_NSET;
localparam IDXW  = `L1_INDEX_W;
localparam TAGW  = `L1_TAG_W;
localparam LINEW = `CACHE_LINE_BITS;

localparam S_IDLE     = 3'd0;
localparam S_LOOKUP   = 3'd1;
localparam S_RREQ     = 3'd2;
localparam S_RDATA    = 3'd3;
localparam S_RESP     = 3'd4;
localparam S_UC_RREQ  = 3'd5;
localparam S_UC_RDATA = 3'd6;

reg [2:0] state;

// ---------------- 存储阵列 ----------------
reg [NSET-1:0] valid_arr [0:NWAY-1];
reg [TAGW-1:0] tag_arr   [0:NWAY-1][0:NSET-1];   // LUTRAM（异步读）

wire [LINEW-1:0] data_out [0:NWAY-1];
reg  [IDXW-1:0]  ram_addr;
reg  [NWAY-1:0]  ram_we;
reg  [LINEW-1:0] ram_wline;
wire             ram_re;

// ---------------- 请求锁存 ----------------
reg [31:0] req_paddr;
reg        req_uncached;

// cacop 暂存（commit 一拍脉冲，FSM 忙时排队；commit 串行发起，深度 1 足够）
reg        cacop_pend;
reg [1:0]  cacop_pend_op;
reg [31:0] cacop_pend_addr;
reg        req_is_cacop;
reg [1:0]  req_cacop_op;

reg [1:0]        victim_way;
reg [127:0]      refill_b0;       // 行 beat0（低半行）暂存
reg [LINEW-1:0]  resp_line;       // RESP 拍输出的整行
reg [2:0]        uc_word;         // uncached 当前取的字偏移
reg [LINEW-1:0]  uc_line;         // uncached 部分有效行拼装

wire [IDXW-1:0] req_set = req_paddr[IDXW+`CACHE_LINE_W-1:`CACHE_LINE_W];
wire [TAGW-1:0] req_tag = req_paddr[31:IDXW+`CACHE_LINE_W];

// ---------------- 命中判定（LOOKUP 拍）----------------
wire [NWAY-1:0] way_hit;
genvar gw;
generate
for (gw = 0; gw < NWAY; gw = gw + 1) begin : gen_hit
    assign way_hit[gw] = valid_arr[gw][req_set] && (tag_arr[gw][req_set] == req_tag);
end
endgenerate
wire       hit_any = |way_hit;
wire [1:0] hit_way = way_hit[1] ? 2'd1 : way_hit[2] ? 2'd2 : way_hit[3] ? 2'd3 : 2'd0;

// victim：无效路优先，否则轮转（I$ 无脏行，换出即丢弃）
reg [1:0] rr_ptr [0:NSET-1];
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

// cacop 的 set/way 解码（op0/op1 用地址低位选 way，op2 用 tag 命中）
wire [IDXW-1:0] cac_set = req_paddr[IDXW+`CACHE_LINE_W-1:`CACHE_LINE_W];
wire [1:0]      cac_way = req_paddr[1:0];

// ---------------- 响应 ----------------
wire lookup_hit = (state == S_LOOKUP) && !req_is_cacop && !req_uncached && hit_any;
assign ifu_data_ok_o = lookup_hit | (state == S_RESP);
assign ifu_rline_o   = lookup_hit ? data_out[hit_way] : resp_line;

// ---------------- 接受仲裁 ----------------
// BRAM 同步读允许在本拍消费旧 data_out 的同时，于时钟沿装入下一组 index。
// 因此 LOOKUP hit 和 RESP 都可作为“返回旧请求 + 接受新请求”的替换窗口。
// cacop 已排队时停止连续取指，先让当前响应结束，再回到 IDLE 服务 cacop。
wire cacop_take = (state == S_IDLE) && cacop_pend;
wire req_window = (state == S_IDLE) || lookup_hit || (state == S_RESP);
wire req_take   = req_window && !cacop_pend && ifu_req_i;
assign ifu_addr_ok_o = req_take;

// ---------------- L2 接口 ----------------
assign axi_rd_req  = (state == S_RREQ) || (state == S_UC_RREQ);
assign axi_rd_type = (state == S_RREQ) ? 3'b100 : 3'b010;   // 行 / 单字
assign axi_rd_addr = (state == S_RREQ) ? {req_paddr[31:`CACHE_LINE_W], {`CACHE_LINE_W{1'b0}}}
                                       : {req_paddr[31:`CACHE_LINE_W], uc_word, 2'b00};

// ---------------- BRAM 控制 ----------------
// 读：IDLE 接受拍（VIPT：index 取自 vaddr）；写：refill 末拍整行安装
wire [IDXW-1:0] rd_set_idle = cacop_take ? cacop_pend_addr[IDXW+`CACHE_LINE_W-1:`CACHE_LINE_W]
                                         : ifu_vaddr_i[IDXW+`CACHE_LINE_W-1:`CACHE_LINE_W];
assign ram_re = cacop_take || req_take;

wire [LINEW-1:0] refill_line = {axi_ret_data, refill_b0};
wire refill_wr = (state == S_RDATA) && axi_ret_valid && axi_ret_last;

always @(*) begin
    ram_we    = {NWAY{1'b0}};
    ram_addr  = rd_set_idle;
    ram_wline = refill_line;
    if (refill_wr) begin
        ram_we[victim_way] = 1'b1;
        ram_addr           = req_set;
    end
end

genvar gr;
generate
for (gr = 0; gr < NWAY; gr = gr + 1) begin : gen_iram
    icache_way_ram u_way_ram(
        .clk   (clk),
        .en    (ram_re | ram_we[gr]),
        .we    (ram_we[gr]),
        .addr  (ram_addr),
        .wdata (ram_wline),
        .rdata (data_out[gr])
    );
end
endgenerate

// ---------------- 主 FSM ----------------
integer s;
always @(posedge clk) begin
    if (!resetn) begin
        state      <= S_IDLE;
        cacop_pend <= 1'b0;
        for (s = 0; s < NWAY; s = s + 1)
            valid_arr[s] <= {NSET{1'b0}};
    end else begin
        if (cacop_en_i) begin
            cacop_pend      <= 1'b1;
            cacop_pend_op   <= cacop_op_i;
            cacop_pend_addr <= cacop_addr_i;
        end

        case (state)
            S_IDLE: begin
                if (cacop_take) begin
                    req_is_cacop <= 1'b1;
                    req_cacop_op <= cacop_pend_op;
                    req_paddr    <= cacop_pend_addr;
                    cacop_pend   <= 1'b0;
                    state        <= S_LOOKUP;
                end else if (req_take) begin
                    req_is_cacop <= 1'b0;
                    req_paddr    <= ifu_paddr_i;
                    req_uncached <= ifu_uncached_i;
                    state        <= S_LOOKUP;
                end
            end

            S_LOOKUP: begin
                if (req_is_cacop) begin
                    // I$ 无脏行：三种 cacop 都是无效化，一拍完成
                    if (req_cacop_op == `CACOP_OP_HIT_WB) begin
                        if (hit_any) valid_arr[hit_way][req_set] <= 1'b0;
                    end else begin
                        valid_arr[cac_way][cac_set] <= 1'b0;
                    end
                    state <= S_IDLE;
                end else if (req_uncached) begin
                    uc_word <= req_paddr[4:2];
                    uc_line <= {LINEW{1'b0}};
                    state   <= S_UC_RREQ;
                end else if (hit_any) begin
                    // 本拍 data_ok 输出旧行；若 IFU 同拍送来下一请求，则
                    // req_take 已启动下一次 BRAM 读并更新请求锁存，保持 LOOKUP。
                    if (req_take) begin
                        req_is_cacop <= 1'b0;
                        req_paddr    <= ifu_paddr_i;
                        req_uncached <= ifu_uncached_i;
                        state        <= S_LOOKUP;
                    end else begin
                        state <= S_IDLE;
                    end
                end else begin
                    victim_way      <= pick_way;
                    rr_ptr[req_set] <= rr_ptr[req_set] + 2'd1;
                    state           <= S_RREQ;
                end
            end

            S_RREQ: if (axi_rd_rdy) state <= S_RDATA;
            S_RDATA: begin
                if (axi_ret_valid) begin
                    if (axi_ret_last) begin
                        tag_arr[victim_way][req_set]   <= req_tag;
                        valid_arr[victim_way][req_set] <= 1'b1;
                        resp_line <= refill_line;
                        state     <= S_RESP;
                    end else begin
                        refill_b0 <= axi_ret_data;
                    end
                end
            end
            S_RESP: begin
                // refill/uncached 响应同样可在空闲 RAM 口启动下一次查找。
                if (req_take) begin
                    req_is_cacop <= 1'b0;
                    req_paddr    <= ifu_paddr_i;
                    req_uncached <= ifu_uncached_i;
                    state        <= S_LOOKUP;
                end else begin
                    state <= S_IDLE;
                end
            end

            // uncached：从块起始字逐字读到行末，拼部分有效行
            S_UC_RREQ: if (axi_rd_rdy) state <= S_UC_RDATA;
            S_UC_RDATA: begin
                if (axi_ret_valid) begin
                    uc_line[32*uc_word +: 32] <= axi_ret_data[31:0];
                    if (uc_word == 3'd7) begin
                        resp_line <= uc_line | ({{(LINEW-32){1'b0}}, axi_ret_data[31:0]} << (32*uc_word));
                        state     <= S_RESP;
                    end else begin
                        uc_word <= uc_word + 3'd1;
                        state   <= S_UC_RREQ;
                    end
                end
            end

            default: state <= S_IDLE;
        endcase
    end
end

// rr_ptr 上电清零（防 X；无复位需求，伪随机即可）
integer ri;
initial begin
    for (ri = 0; ri < NSET; ri = ri + 1) rr_ptr[ri] = 2'b0;
end

// lint 吸收（cancel 端口按约定忽略：IFU 自行配对丢弃过期返回）
wire icache_lint = ifu_cancel_i;

`ifdef SYNTHESIS
// synthesis translate_off
// 仿真性能统计：cached LOOKUP 访问 / 命中（不含 uncached/cacop）
reg [63:0] ic_access_total;
reg [63:0] ic_hit_total;
reg [63:0] ic_lookup_total;
reg [63:0] ic_lookup_cached_total;
reg [63:0] ic_lookup_uncached_total;
reg [63:0] ic_lookup_cacop_total;
always @(posedge clk) begin
    if (!resetn) begin
        ic_access_total <= 64'd0;
        ic_hit_total    <= 64'd0;
        ic_lookup_total <= 64'd0;
        ic_lookup_cached_total <= 64'd0;
        ic_lookup_uncached_total <= 64'd0;
        ic_lookup_cacop_total <= 64'd0;
    end else if ((state == S_LOOKUP) && !req_is_cacop && !req_uncached) begin
        ic_access_total <= ic_access_total + 64'd1;
        ic_lookup_total <= ic_lookup_total + 64'd1;
        ic_lookup_cached_total <= ic_lookup_cached_total + 64'd1;
        if (hit_any)
            ic_hit_total <= ic_hit_total + 64'd1;
    end else if (state == S_LOOKUP) begin
        ic_lookup_total <= ic_lookup_total + 64'd1;
        if (req_is_cacop)
            ic_lookup_cacop_total <= ic_lookup_cacop_total + 64'd1;
        else if (req_uncached)
            ic_lookup_uncached_total <= ic_lookup_uncached_total + 64'd1;
    end
end
// synthesis translate_on
`endif

endmodule

// ------------------------------------------------------------
// icache_way_ram：单口同步 RAM 模板（128 x 256b，推断 BRAM）
// ------------------------------------------------------------
module icache_way_ram(
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
