// ============================================================
// mul 模块（32x32 流水乘法器，推断 DSP 实现，3 级流水）
// ------------------------------------------------------------
// 功能：
// - 完成 32x32 -> 64 位有符号/无符号乘法，供 mul.w / mulh.w / mulh.wu 使用。
// - 用 33x33 扩展统一覆盖有符号/无符号乘法；
// - 直接写 `a*b` 并加 use_dsp 属性，让 Vivado 推断 DSP48 级联；
// - 固定 3 拍流水：输入寄存 -> 乘积寄存 -> 输出寄存。
//
// 端口：
// - valid_i / a_i / b_i / is_signed_i ：启动一次乘法
// - result_o / done_o                 ：固定 N 拍后结果有效一拍
// ============================================================

(* use_dsp = "yes" *)
module mul(
    input  wire          clk,
    input  wire          reset,

    input  wire          valid_i,       // 启动（fu_mdu 保证 busy 期间不重复启动）
    input  wire [31:0]   a_i,
    input  wire [31:0]   b_i,
    input  wire          is_signed_i,   // 1=有符号（mul.w/mulh.w），0=无符号（mulh.wu）

    output wire [63:0]   result_o,      // 64 位全乘积
    output wire          done_o         // 结果有效（启动后固定拍数）
);

// One registered DSP product stage.  fu_mdu remains non-pipelined, but this
// cuts its multiply occupancy from the former three product stages to one.
(* use_dsp = "yes" *) reg signed [65:0] prod_r;  //让vivado使用DSP
reg                         valid_r;

wire signed [32:0] a33 = {is_signed_i & a_i[31], a_i};
wire signed [32:0] b33 = {is_signed_i & b_i[31], b_i};

always @(posedge clk) begin
    if (reset) begin
        prod_r     <= 66'b0;
        valid_r    <= 1'b0;
    end else begin
        valid_r <= valid_i;
        if (valid_i)
            prod_r <= a33 * b33;
    end
end

assign result_o = prod_r[63:0];
assign done_o   = valid_r;

endmodule
