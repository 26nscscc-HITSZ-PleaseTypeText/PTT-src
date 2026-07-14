// ============================================================
// div 模块（CLZ 快速移位除法器，迭代式）
// ------------------------------------------------------------
// 功能：
// - 完成 32/32 位有符号/无符号除法与取模，供 div.w/div.wu/mod.w/mod.wu。
// - 核心优化（满洋/mariver 实证路线）：朴素恢复余数除法固定 32 拍；
//   先用 CLZ（前导零计数）把除数对齐到被除数的最高位，跳过无效迭代，
//   则迭代次数 = clz(divisor) - clz(dividend) + 1 ——
//   竞赛 benchmark 中操作数普遍很小，常见 5~10 拍出结果，IPC 收益明显。
//
// 端口：
// - valid_i / 操作数 / is_signed_i ：启动一次除法
// - quotient_o / remainder_o / done_o / busy_o
// - flush_i：全局冲刷时打断迭代（必须支持！）
// ============================================================

module div(
    input  wire          clk,
    input  wire          reset,
    input  wire          flush_i,        // 冲刷打断（作废当前迭代）

    input  wire          valid_i,        // 启动
    input  wire [31:0]   dividend_i,     // 被除数
    input  wire [31:0]   divisor_i,      // 除数
    input  wire          is_signed_i,    // 1=有符号

    output wire [31:0]   quotient_o,     // 商
    output wire [31:0]   remainder_o,    // 余数
    output wire          done_o,         // 结果有效一拍
    output wire          busy_o          // 迭代中
);

function [5:0] clz32;
    input [31:0] x;
    begin
        if (x[31]) clz32 = 6'd0;
        else if (x[30]) clz32 = 6'd1;
        else if (x[29]) clz32 = 6'd2;
        else if (x[28]) clz32 = 6'd3;
        else if (x[27]) clz32 = 6'd4;
        else if (x[26]) clz32 = 6'd5;
        else if (x[25]) clz32 = 6'd6;
        else if (x[24]) clz32 = 6'd7;
        else if (x[23]) clz32 = 6'd8;
        else if (x[22]) clz32 = 6'd9;
        else if (x[21]) clz32 = 6'd10;
        else if (x[20]) clz32 = 6'd11;
        else if (x[19]) clz32 = 6'd12;
        else if (x[18]) clz32 = 6'd13;
        else if (x[17]) clz32 = 6'd14;
        else if (x[16]) clz32 = 6'd15;
        else if (x[15]) clz32 = 6'd16;
        else if (x[14]) clz32 = 6'd17;
        else if (x[13]) clz32 = 6'd18;
        else if (x[12]) clz32 = 6'd19;
        else if (x[11]) clz32 = 6'd20;
        else if (x[10]) clz32 = 6'd21;
        else if (x[9]) clz32 = 6'd22;
        else if (x[8]) clz32 = 6'd23;
        else if (x[7]) clz32 = 6'd24;
        else if (x[6]) clz32 = 6'd25;
        else if (x[5]) clz32 = 6'd26;
        else if (x[4]) clz32 = 6'd27;
        else if (x[3]) clz32 = 6'd28;
        else if (x[2]) clz32 = 6'd29;
        else if (x[1]) clz32 = 6'd30;
        else if (x[0]) clz32 = 6'd31;
        else clz32 = 6'd32;
    end
endfunction

function [31:0] abs32;
    input [31:0] x;
    input        signed_en;
    begin
        abs32 = (signed_en && x[31]) ? (~x + 32'd1) : x;
    end
endfunction

function [31:0] apply_sign;
    input [31:0] x;
    input        neg;
    begin
        apply_sign = neg ? (~x + 32'd1) : x;
    end
endfunction

reg        busy_r;
reg        done_r;
reg [31:0] quotient_r;
reg [31:0] remainder_r;

reg [31:0] rem_r;
reg [31:0] quot_r;
reg [31:0] div_shift_r;
reg [5:0]  bit_idx_r;
reg        quot_neg_r;
reg        rem_neg_r;

// ============================================================
// 100MHz 攻坚 Phase D:除法器操作数打拍(prep 级),砍关键路径尾段
// ------------------------------------------------------------
// 关键路径尾段一直是:load 结果(含 store-buffer 前递,组合)→旁路网络→MDU
// 操作数 → 本模块 start 拍的 abs(2×32 位取反)+2×CLZ32+减法+可变移位
// (divisor_abs<<start_shift),全在操作数到达的同一拍组合完成。
// 现插一个 prep 级:start 拍只把"原始操作数"锁进 dividend_r/divisor_r/
// is_signed_r(旁路 mux 的浅终点),下一拍(prep_r=1)再从寄存器操作数做
// abs/clz/首移位/特例判定。除法极少见,+1 拍延迟对 IPC 几乎无影响,
// 但把 abs/clz/移位这段深逻辑从"旁路到达路径"上摘除。
reg        prep_r;              // prep 级有效(操作数已锁存,本拍做 abs/clz/启动)
reg [31:0] dividend_r, divisor_r;
reg        is_signed_r;

// prep 级组合:全部从寄存器操作数出发(不再挂在旁路到达路径上)
wire [31:0] dividend_abs = abs32(dividend_r, is_signed_r);
wire [31:0] divisor_abs  = abs32(divisor_r,  is_signed_r);
wire [5:0]  start_shift  = clz32(divisor_abs) - clz32(dividend_abs);

wire        accept_new   = valid_i && !busy_r && !prep_r;  // 可锁存一对新操作数
wire        div_zero     = divisor_r == 32'b0;
wire        div_overflow = is_signed_r
                         && (dividend_r == 32'h8000_0000)
                         && (divisor_r  == 32'hffff_ffff);
wire        quot_neg     = is_signed_r && (dividend_r[31] ^ divisor_r[31]);
wire        rem_neg      = is_signed_r && dividend_r[31];

wire        iter_ge      = rem_r >= div_shift_r;
wire [31:0] iter_rem     = iter_ge ? (rem_r - div_shift_r) : rem_r;
wire [31:0] iter_quot    = iter_ge ? (quot_r | (32'b1 << bit_idx_r[4:0])) : quot_r;

// 时序解耦(100MHz 攻坚 step2):flush_i 由"rob 异常→csr ecode→commit 误预测
// →cmt_flush_req(fanout 1212)"的深组合链生成(约 11ns),原来它同步复位本模块
// 全部寄存器——包括 32 位数据通路 div_shift_r/rem_r/quot_r,于是这条超高扇出
// 的晚到 flush 被映射成 32 个 D 端 mux,把 flush 生成链拖进除法器数据通路的建立
// 路径(综合关键路径 34 级)。
// 事实:被 flush 的除法结果一定被丢弃(busy 会清、done 不再上抛),数据通路寄存器
// 在 !busy 时是 don't-care。因此 flush 只需清"控制位"busy_r/done_r,数据通路寄存器
// 无需复位——这样 flush_i 不再进入 32 位数据通路的 D 端,关键路径显著缩短。
always @(posedge clk) begin
    if (reset) begin
        busy_r      <= 1'b0;
        done_r      <= 1'b0;
        prep_r      <= 1'b0;
        quotient_r  <= 32'b0;
        remainder_r <= 32'b0;
        rem_r       <= 32'b0;
        quot_r      <= 32'b0;
        div_shift_r <= 32'b0;
        bit_idx_r   <= 6'b0;
        quot_neg_r  <= 1'b0;
        rem_neg_r   <= 1'b0;
        dividend_r  <= 32'b0;
        divisor_r   <= 32'b0;
        is_signed_r <= 1'b0;
    end else if (flush_i) begin
        // 冲刷:只作废控制位(当前迭代/prep 丢弃),数据通路寄存器保持(下次 start 时整体覆写)
        busy_r      <= 1'b0;
        done_r      <= 1'b0;
        prep_r      <= 1'b0;
    end else begin
        done_r <= 1'b0;

        // prep 级:锁存新操作数(旁路 mux 浅终点),下一拍再算 abs/clz/启动
        if (accept_new) begin
            prep_r      <= 1'b1;
            dividend_r  <= dividend_i;
            divisor_r   <= divisor_i;
            is_signed_r <= is_signed_i;
        end

        if (prep_r) begin
            prep_r <= 1'b0;   // prep 拍消费掉,进入 busy 迭代或直接出结果
            if (div_zero) begin
                quotient_r  <= 32'hffff_ffff;
                remainder_r <= dividend_r;
                done_r      <= 1'b1;
            end else if (div_overflow) begin
                quotient_r  <= 32'h8000_0000;
                remainder_r <= 32'b0;
                done_r      <= 1'b1;
            end else if (dividend_abs < divisor_abs) begin
                quotient_r  <= 32'b0;
                remainder_r <= apply_sign(dividend_abs, rem_neg);
                done_r      <= 1'b1;
            end else begin
                busy_r      <= 1'b1;
                rem_r       <= dividend_abs;
                quot_r      <= 32'b0;
                div_shift_r <= divisor_abs << start_shift[4:0];
                bit_idx_r   <= start_shift;
                quot_neg_r  <= quot_neg;
                rem_neg_r   <= rem_neg;
            end
        end else if (busy_r) begin
            if (bit_idx_r == 6'b0) begin
                busy_r      <= 1'b0;
                quotient_r  <= apply_sign(iter_quot, quot_neg_r);
                remainder_r <= apply_sign(iter_rem, rem_neg_r);
                done_r      <= 1'b1;
            end else begin
                rem_r       <= iter_rem;
                quot_r      <= iter_quot;
                div_shift_r <= div_shift_r >> 1;
                bit_idx_r   <= bit_idx_r - 6'd1;
            end
        end
    end
end

assign quotient_o  = quotient_r;
assign remainder_o = remainder_r;
assign done_o      = done_r;
// prep 拍也算忙:此拍已锁存操作数但结果未出,须让 fu_mdu 视为占用(不再接新 div、
// 不误采 done)。busy 从 accept_new 后的 prep 拍一直拉到迭代结束。
assign busy_o      = busy_r | prep_r;

endmodule
