`include "mycpu.h"
// ============================================================
// decode_tools（通用 one-hot 译码器集合：decoder_2_4/4_16/5_32/6_64）
// ------------------------------------------------------------
// 新架构角色：被 inst_dec.v 等内部例化复用，零改动。
// ============================================================
module decoder_2_4(
    input  wire [ 1:0] in,               // 2 位二进制索引
    output wire [ 3:0] co                // 4 路 one-hot
);
genvar i;
generate for (i = 0; i < 4; i = i + 1) begin : gen_for_dec_2_4
    assign co[i] = (in == i);
end endgenerate
endmodule

module decoder_4_16(
    input  wire [ 3:0] in,
    output wire [15:0] co
);
genvar i;
generate for (i = 0; i < 16; i = i + 1) begin : gen_for_dec_4_16
    assign co[i] = (in == i);
end endgenerate
endmodule

module decoder_5_32(
    input  wire [ 4:0] in,
    output wire [31:0] co
);
genvar i;
generate for (i = 0; i < 32; i = i + 1) begin : gen_for_dec_5_32
    assign co[i] = (in == i);
end endgenerate
endmodule

module decoder_6_64(
    input  wire [ 5:0] in,
    output wire [63:0] co
);
genvar i;
generate for (i = 0; i < 64; i = i + 1) begin : gen_for_dec_6_64
    assign co[i] = (in == i);
end endgenerate
endmodule
