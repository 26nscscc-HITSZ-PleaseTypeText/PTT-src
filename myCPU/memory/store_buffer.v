// ============================================================
// store_buffer 模块（提交后写缓冲，`SB_SIZE=8 项 FIFO）
// ------------------------------------------------------------
// 功能：
// - 乱序核 store 正确性的关键部件："store 在提交前绝不写内存"。
//   store 指令提交时（已确定非投机），commit 把 {paddr,data,wstrb,size,uncached}
//   推入本缓冲；本缓冲按程序序写出到 DCache/外设，写出与流水线解耦
//   （提交不必等写完成，后续指令继续提交 —— 隐藏 store 延迟）。
// - load 前递：在飞的 store 尚未写进 DCache 时，更年轻的 load 读同地址必须
//   看到它的数据 —— LSU 在 AGU/DC 级查询本缓冲【逐字节】前递。
// - 重要语义（与其他队列的本质区别！）：
//   * 全局冲刷 flush 时本缓冲【不清空】—— 缓冲里全部是"已提交"的 store，
//     体系结构上已经发生，必须继续写出；只有复位才清。
//   * ibar/dbar、cacop 等屏障语义：commit 等 sb_empty 后才放行。
//   * difftest 的 StoreEvent 在 commit 提交点报告（不等本缓冲排空），
//     与 NEMU 的提交序一致；本缓冲只是写出延迟，软件不可见。
//
// 排空规则（行合并）：
// - 项内仍保持字粒度（前递不变）；泄流口对 D$ 升为整行
//   （`CACHE_LINE_BITS` 数据 + `CACHE_LINE_BYTES` 字节使能）。
// - head 为 cached 时：自 head 向年轻方向连续聚合「同 cache 行、cached」项，
//   一次发给 D$（年轻字节覆盖年老）；`st_done` 后一次弹出全部已聚合项。
// - head 为 uncached 时：不合并，单字放在行内对应字槽，保留 size；一次只弹 1 项。
// - 严格按程序序写出，绝不乱序（TSO + 设备写序）；
// - sb_empty："队列空"（done 才清项，inflight 期间项仍 valid）。
//
// D$ 的 st_done_o 与 LOOKUP hit/alloc/merge 同拍组合产生，以保持 posted 语义；
// 本模块将 dc_wr_done_i 寄存一拍后再驱动 pop、inflight 清除和 count，切断路径
//   req_paddr → miss_need_wb → dc_sb_done → push_fire → data_reg CE。
// 代价：SB 腾槽/解满晚 1 拍；done 延迟期间 inflight 仍为 1，不会抢发下一笔。
//
// 前递查询为纯组合逐字节合并：字地址匹配，年轻项覆盖年老项。
//
// 端口：
// - push_*      ：commit 提交 store 入队（一拍最多 1 条）
// - dc_wr_*     ：排空写 DCache 口（行粒度 data/strb；uncached 仍带 size）
// - query_*     ：LSU load 前递查询口（组合，物理地址）
// - sb_full/empty
// ============================================================
`include "mycpu.h"

module store_buffer(
    input  wire                clk,
    input  wire                reset,

    // ---------------- 提交入队（来自 commit）----------------
    input  wire                push_valid_i,
    input  wire [31:0]         push_paddr_i,
    input  wire [31:0]         push_data_i,       // 已按地址对齐到字节通道
    input  wire [3:0]          push_wstrb_i,
    input  wire [2:0]          push_size_i,       // AXI size（uncached 精确宽度用）
    input  wire                push_uncached_i,
    output wire                sb_full_o,         // 满（commit 暂停提交 store）
    output wire                sb_empty_o,        // 空（屏障/uncached load 等待用）

    // ---------------- 排空写出口（连 dcache 的 store 写口，行粒度）----------------
    output wire                dc_wr_req_o,       // 写请求（保持至 addr_ok）
    output wire [31:0]         dc_wr_paddr_o,     // 行对齐：低 `CACHE_LINE_W 位可为 0；UC 保留精确地址
    output wire [`CACHE_LINE_BITS-1:0] dc_wr_data_o,
    output wire [`CACHE_LINE_BYTES-1:0] dc_wr_strb_o,
    output wire [2:0]          dc_wr_size_o,
    output wire                dc_wr_uncached_o,
    input  wire                dc_wr_addr_ok_i,   // DCache 收下
    input  wire                dc_wr_done_i,      // 写完成（与一次行写一一配对）

    // ---------------- load 前递查询口（LSU，组合）----------------
    input  wire [31:2]         query_paddr_i,
    input  wire                query_uncached_i,  // 本次查询来自 uncached load
    output wire                query_hit_o,       // 整字（4 字节）可由 SB 合并提供
    output wire [31:0]         query_data_o,      // 前递数据（逐字节取最年轻）
    output wire                query_partial_o    // 部分/受阻（load 须等排空重试）
);

    // ------------------------------------------------------------
    // 存储：SB_SIZE 项环形 FIFO。head=最老（先写出），tail=入队位置。
    // ------------------------------------------------------------
    reg [`SB_SIZE-1:0] valid;
    reg [31:0]         paddr    [0:`SB_SIZE-1];
    reg [31:0]         data     [0:`SB_SIZE-1];
    reg [3:0]          strb     [0:`SB_SIZE-1];
    reg [2:0]          size     [0:`SB_SIZE-1];
    reg                uncached [0:`SB_SIZE-1];
    reg [`SB_W-1:0]    head;
    reg [`SB_W-1:0]    tail;
    reg [`SB_W:0]      count;
    reg                inflight;    // 行写已被 dcache 收下、等 done
    // addr_ok 时锁存本次弹出项数（防 done 拍组合聚合抖动）
    reg [`SB_W:0]      inflight_n;

    // 完成信号在消费侧打一拍；pop/inflight/count 必须使用同一拍信号。
    reg                dc_wr_done_r;
    always @(posedge clk) begin
        if (reset)
            dc_wr_done_r <= 1'b0;
        else
            dc_wr_done_r <= dc_wr_done_i;
    end

    wire [`SB_W:0]     count_zero = {(`SB_W + 1){1'b0}};
    wire pop_fire = (count != count_zero) && dc_wr_done_r;
    wire push_fire = push_valid_i && (!sb_full_o || pop_fire);

    // ------------------------------------------------------------
    // 泄流行聚合（组合）：自 head 连续同 cache 行、cached 项
    // ------------------------------------------------------------
    reg [`SB_W:0]               merge_n;
    reg [`CACHE_LINE_BITS-1:0]  merge_data;
    reg [`CACHE_LINE_BYTES-1:0] merge_strb;
    reg [31:0]                  merge_paddr;
    reg [2:0]                   merge_size;
    reg                         merge_uncached;

    integer mi, mb;
    reg [`SB_W-1:0] m_idx;
    reg [2:0]       m_word;
    reg             m_cont;

    always @(*) begin
        merge_n        = count_zero;
        merge_data     = {`CACHE_LINE_BITS{1'b0}};
        merge_strb     = {`CACHE_LINE_BYTES{1'b0}};
        merge_paddr    = 32'b0;
        merge_size     = 3'b0;
        merge_uncached = 1'b0;
        m_idx          = {`SB_W{1'b0}};
        m_word         = 3'b0;
        m_cont         = 1'b0;

        if ((count != count_zero) && valid[head]) begin
            merge_uncached = uncached[head];
            merge_size     = size[head];
            // UC：保留精确地址；cached：行对齐（低位清零便于 D$ tag）
            merge_paddr    = uncached[head] ? paddr[head]
                                            : {paddr[head][31:`CACHE_LINE_W],
                                               {`CACHE_LINE_W{1'b0}}};

            if (uncached[head]) begin
                // 单字：放在行内对应字槽，D$ UC 路径按 req_word 抽取
                merge_n     = {{`SB_W{1'b0}}, 1'b1};
                m_word      = paddr[head][4:2];
                merge_strb[4*m_word +: 4] = strb[head];
                merge_data[32*m_word +: 32] = data[head];
            end else begin
                // cached：自 head 向年轻连续同行走合并；年轻覆盖年老
                m_cont = 1'b1;
                for (mi = 0; mi < `SB_SIZE; mi = mi + 1) begin
                    m_idx = head + mi[`SB_W-1:0];
                    if (m_cont && valid[m_idx] && !uncached[m_idx]
                     && (paddr[m_idx][31:`CACHE_LINE_W] == paddr[head][31:`CACHE_LINE_W])) begin
                        merge_n      = merge_n + {{`SB_W{1'b0}}, 1'b1};
                        m_word       = paddr[m_idx][4:2];
                        for (mb = 0; mb < 4; mb = mb + 1) begin
                            if (strb[m_idx][mb]) begin
                                merge_strb[4*m_word + mb] = 1'b1;
                                merge_data[8*(4*m_word + mb) +: 8] =
                                    data[m_idx][8*mb +: 8];
                            end
                        end
                    end else begin
                        m_cont = 1'b0;
                    end
                end
            end
        end
    end

    integer idx;
    integer pi;

    always @(posedge clk) begin
        if (reset) begin
            valid       <= {`SB_SIZE{1'b0}};
            head        <= {`SB_W{1'b0}};
            tail        <= {`SB_W{1'b0}};
            count       <= count_zero;
            inflight    <= 1'b0;
            inflight_n  <= count_zero;
            for (idx = 0; idx < `SB_SIZE; idx = idx + 1) begin
                paddr[idx]    <= 32'b0;
                data[idx]     <= 32'b0;
                strb[idx]     <= 4'b0;
                size[idx]     <= 3'b0;
                uncached[idx] <= 1'b0;
            end
        end else begin
            // flush 不清：缓冲内都是已提交 store
            // 用 dc_wr_done_r（非组合 dc_wr_done_i），与 pop_fire 同拍
            if (dc_wr_done_r) begin
                inflight   <= 1'b0;
                inflight_n <= count_zero;
            end else if (dc_wr_addr_ok_i) begin
                inflight   <= 1'b1;
                inflight_n <= merge_n;
            end

            if (pop_fire) begin
                for (pi = 0; pi < `SB_SIZE; pi = pi + 1) begin
                    if (pi < inflight_n) begin
                        valid[head + pi[`SB_W-1:0]] <= 1'b0;
                    end
                end
                head <= head + inflight_n[`SB_W-1:0];
            end

            if (push_fire) begin
                paddr[tail]    <= push_paddr_i;
                data[tail]     <= push_data_i;
                strb[tail]     <= push_wstrb_i;
                size[tail]     <= push_size_i;
                uncached[tail] <= push_uncached_i;
                valid[tail]    <= 1'b1;
                tail           <= tail + `SB_W'd1;
            end

            case ({push_fire, pop_fire})
                2'b10: count <= count + {{`SB_W{1'b0}}, 1'b1};
                2'b01: count <= count - inflight_n;
                default: begin
                    if (push_fire && pop_fire)
                        count <= count + {{`SB_W{1'b0}}, 1'b1} - inflight_n;
                    else
                        count <= count;
                end
            endcase
        end
    end

    assign sb_full_o  = (count == `SB_SIZE);
    assign sb_empty_o = (count == count_zero);

    assign dc_wr_req_o      = !sb_empty_o && valid[head] && !inflight;
    assign dc_wr_paddr_o    = merge_paddr;
    assign dc_wr_data_o     = merge_data;
    assign dc_wr_strb_o     = merge_strb;
    assign dc_wr_size_o     = merge_size;
    assign dc_wr_uncached_o = merge_uncached;

    // ------------------------------------------------------------
    // 前递查询（纯组合，逐字节合并）—— 项内字粒度不变
    // ------------------------------------------------------------
    reg [3:0]          byte_found;
    reg [3:0]          byte_block;
    reg [31:0]         merge_data_r;
    reg                any_match_r;
    reg                any_unc_r;
    reg [`SB_W-1:0]    q_idx;
    integer            qi, b;

    always @ (*) begin
        byte_found   = 4'b0;
        byte_block   = 4'b0;
        merge_data_r = 32'b0;
        any_match_r  = 1'b0;
        any_unc_r    = 1'b0;
        q_idx        = {`SB_W{1'b0}};

        for (qi = 0; qi < `SB_SIZE; qi = qi + 1) begin
            q_idx = tail - 1'b1 - qi[`SB_W-1:0];
            if (valid[q_idx]) begin
                if (uncached[q_idx]) begin
                    any_unc_r = 1'b1;
                end
                if (mem_same_word(paddr[q_idx][31:2], query_paddr_i)) begin
                    any_match_r = 1'b1;
                    for (b = 0; b < 4; b = b + 1) begin
                        if (!byte_found[b] && !byte_block[b] && strb[q_idx][b]) begin
                            if (uncached[q_idx]) begin
                                byte_block[b] = 1'b1;
                            end else begin
                                byte_found[b] = 1'b1;
                                merge_data_r[8*b +: 8] = data[q_idx][8*b +: 8];
                            end
                        end
                    end
                end
            end
        end
    end

    wire unc_order_block = (query_uncached_i === 1'b1) && any_unc_r;

    assign query_hit_o     = (&byte_found) && !unc_order_block;
    assign query_data_o    = merge_data_r;
    assign query_partial_o = (any_match_r && !(&byte_found)) || unc_order_block;

`ifdef SYNTHESIS
// synthesis translate_off
reg [63:0] sb_nonempty_cyc;
reg [63:0] sb_wr_req_cyc;
always @(posedge clk) begin
    if (reset) begin
        sb_nonempty_cyc <= 64'd0;
        sb_wr_req_cyc   <= 64'd0;
    end else begin
        if (!sb_empty_o)
            sb_nonempty_cyc <= sb_nonempty_cyc + 64'd1;
        if (dc_wr_req_o)
            sb_wr_req_cyc <= sb_wr_req_cyc + 64'd1;
    end
end
// synthesis translate_on
`endif

endmodule
