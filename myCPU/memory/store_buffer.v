// ============================================================
// store_buffer 模块（提交后写缓冲，`SB_SIZE=8 项 FIFO）
// ------------------------------------------------------------
// 功能：
// - 乱序核 store 正确性的关键部件："store 在提交前绝不写内存"。
//   store 指令提交时（已确定非投机），commit 把 {paddr,data,wstrb,size,uncached}
//   推入本缓冲；本缓冲按程序序逐条写出到 DCache/外设，写出与流水线解耦
//   （提交不必等写完成，后续指令继续提交 —— 隐藏 store 延迟）。
// - load 前递：在飞的 store 尚未写进 DCache 时，更年轻的 load 读同地址必须
//   看到它的数据 —— LSU 在 AGU/DC 级查询本缓冲【逐字节】前递。
// - 重要语义（与其他队列的本质区别！）：
//   * 全局冲刷 flush 时本缓冲【不清空】—— 缓冲里全部是"已提交"的 store，
//     体系结构上已经发生，必须继续写出；只有复位才清。
//   * ibar/dbar/ll 等屏障语义：commit 等 sb_empty 后才放行。
//   * difftest 的 StoreEvent 在 commit 提交点报告（不等本缓冲排空），
//     与 NEMU 的提交序一致；本缓冲只是写出延迟，软件不可见。
//
// 排空规则：
// - head 项 valid 时持续发 dc_wr_req（保持到 addr_ok），dc_wr_done 后清项、
//   head++。严格按程序序写出，绝不乱序（TSO 存储序 + 设备写序都靠它）；
// - cached 与 uncached 走同一出口（dcache 内部按 uncached 旁路 AXI），
//   uncached 写用 push_size_i 给出真实 AXI 宽度（外设按字节写的坑！）；
// - sb_empty 判定严格："队列空 且 无在途未完成写"（inflight 计入），
//   它是 ibar/ll/uncached-load 放行的依据。
//
// 前递查询（纯组合，二期完整版——逐字节合并）：
// - 对查询字地址（paddr[31:2]），4 个字节道各自独立从最年轻项向老扫描，
//   取第一个覆盖该字节的项做前递源——多条 store 部分写同一字（如两条
//   st.h 拼一个字）也能整字前递，比一期"单项整字命中"覆盖面大得多；
// - query_hit_o：4 字节全部可由缓冲提供（允许来自不同项）；
// - query_partial_o：字地址有匹配但凑不齐 4 字节，或提供某字节的最年轻
//   项是 uncached（设备数据必须真读）——load 等排空后重试；
// - uncached load 序保证：query_uncached_i=1（本次查询来自 uncached load）
//   时，只要缓冲中还有任何 uncached store 未写完即报 partial——保证
//   "先写设备后读设备"的次序（不同寄存器地址也不允许越过）；
// - 正在写出的 head 项（等 addr_ok/done 期间）仍参与前递：数据在项清除
//   （done）前始终有效，且 done 意味着已对 dcache 可见，交接无空窗。
//
// 端口：
// - push_*      ：commit 提交 store 入队（一拍最多 1 条，见 commit 单提交约定）
// - dc_wr_*     ：排空写 DCache 口（含 uncached 直写外设通道）
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

    // ---------------- 排空写出口（连 dcache 的 store 写口）----------------
    output wire                dc_wr_req_o,       // 写请求（保持至 addr_ok，不能只保留一拍）
    output wire [31:0]         dc_wr_paddr_o,
    output wire [31:0]         dc_wr_data_o,
    output wire [3:0]          dc_wr_strb_o,
    output wire [2:0]          dc_wr_size_o,
    output wire                dc_wr_uncached_o,
    input  wire                dc_wr_addr_ok_i,   // DCache 收下
    input  wire                dc_wr_done_i,      // 写完成（cached 写命中可当拍/次拍完成）

    // ---------------- load 前递查询口（LSU，组合）----------------
    input  wire [31:0]         query_paddr_i,
    input  wire                query_uncached_i,  // 本次查询来自 uncached load
    output wire                query_hit_o,       // 整字（4 字节）可由 SB 合并提供
    output wire [31:0]         query_data_o,      // 前递数据（逐字节取最年轻）
    output wire                query_partial_o    // 部分/受阻（load 须等排空重试）
);

    // ------------------------------------------------------------
    // 存储：SB_SIZE 项环形 FIFO。head=最老（先写出），tail=入队位置。
    // count 冗余计数便于满/空判断与同拍 push+pop 合并更新。
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
    reg                inflight;    // head 项已被 dcache 收下、等 done

    // 出队：done 一拍脉冲清 head 项（与 dcache 的 st_done 一一配对）
    wire pop_fire = (count != {(`SB_W + 1){1'b0}}) && dc_wr_done_i;
    // 入队：满时若同拍恰有 pop 也允许（净空间不变），提高满载吞吐
    wire push_fire = push_valid_i && (!sb_full_o || pop_fire);

    integer idx;

    always @(posedge clk) begin
        if (reset) begin
            valid    <= {`SB_SIZE{1'b0}};
            head     <= {`SB_W{1'b0}};
            tail     <= {`SB_W{1'b0}};
            count    <= {(`SB_W + 1){1'b0}};
            inflight <= 1'b0;
            for (idx = 0; idx < `SB_SIZE; idx = idx + 1) begin
                paddr[idx]    <= 32'b0;
                data[idx]     <= 32'b0;
                strb[idx]     <= 4'b0;
                size[idx]     <= 3'b0;
                uncached[idx] <= 1'b0;
            end
        end else begin
            // 注意 flush 不清任何状态：缓冲内都是已提交 store（体系结构状态）
            if (dc_wr_done_i) begin
                inflight <= 1'b0;
            end else if (dc_wr_addr_ok_i) begin
                inflight <= 1'b1;
            end

            if (pop_fire) begin
                valid[head] <= 1'b0;
                head <= head + `SB_W'd1;
            end

            if (push_fire) begin
                paddr[tail] <= push_paddr_i;
                data[tail] <= push_data_i;
                strb[tail] <= push_wstrb_i;
                size[tail] <= push_size_i;
                uncached[tail] <= push_uncached_i;
                valid[tail] <= 1'b1;
                tail <= tail + `SB_W'd1;
            end

            // 同拍 push+pop：count 不变（合并更新，防指针法丢计数）
            case ({push_fire, pop_fire})
                2'b10: count <= count + {{`SB_W{1'b0}}, 1'b1};
                2'b01: count <= count - {{`SB_W{1'b0}}, 1'b1};
                default: count <= count;
            endcase
        end
    end

    assign sb_full_o = (count == `SB_SIZE);
    // 严格空：队列空即无在途写（done 才清项，项在 inflight 期间仍 valid）
    assign sb_empty_o = (count == {(`SB_W + 1){1'b0}});

    // 排空：head 项有效且未在途时持续请求（addr_ok 后撤 req 等 done）
    assign dc_wr_req_o = !sb_empty_o && valid[head] && !inflight;
    assign dc_wr_paddr_o = paddr[head];
    assign dc_wr_data_o = data[head];
    assign dc_wr_strb_o = strb[head];
    assign dc_wr_size_o = size[head];
    assign dc_wr_uncached_o = uncached[head];

    // ------------------------------------------------------------
    // 前递查询（纯组合，逐字节合并）
    // 每个字节道独立从最年轻（tail-1）向最老扫描：
    // - 第一个覆盖该字节的 cached 项 -> 前递源（更老的项被它遮蔽）；
    // - 第一个覆盖该字节的项若是 uncached -> 该字节受阻（设备数据必须真读）。
    // ------------------------------------------------------------
    reg [3:0]          byte_found;      // 各字节已找到 cached 前递源
    reg [3:0]          byte_block;      // 各字节被 uncached 项挡住
    reg [31:0]         merge_data_r;
    reg                any_match_r;     // 任意项字地址匹配
    reg                any_unc_r;       // 缓冲中存在 uncached 项（含在途）
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
            q_idx = tail - 1'b1 - qi[`SB_W-1:0];   // 最年轻 -> 最老（环回）
            if (valid[q_idx]) begin
                if (uncached[q_idx]) begin
                    any_unc_r = 1'b1;
                end
                if (paddr[q_idx][31:2] == query_paddr_i[31:2]) begin
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

    // uncached load 的设备写序保证：缓冲里还有任何 uncached store 未排空
    // 即报 partial（4-state 防御：query_uncached_i 悬空/X 视为 0）
    wire unc_order_block = (query_uncached_i === 1'b1) && any_unc_r;

    assign query_hit_o     = (&byte_found) && !unc_order_block;
    assign query_data_o    = merge_data_r;
    assign query_partial_o = (any_match_r && !(&byte_found)) || unc_order_block;

endmodule
