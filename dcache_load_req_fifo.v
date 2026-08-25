`include "mycpu.h"

// Two-entry registered load-request boundary between LSU and DCache.
// The LSU-side ready signal depends only on registered occupancy, so cache
// lookup/state feedback cannot reach LSU pipeline enables.  With one queued
// request, simultaneous push/pop sustains one accepted load per cycle.
module dcache_load_req_fifo(
    input  wire                  clk,
    input  wire                  reset,
    input  wire                  flush_i,

    input  wire                  s_valid_i,
    input  wire [6:0]            s_vindex_i,
    input  wire [31:0]           s_paddr_i,
    input  wire [2:0]            s_size_i,
    input  wire [7:4]            s_load_op_i,
    input  wire                  s_uncached_i,
    input  wire [`ROB_W-1:0]     s_robid_i,
    input  wire                  s_pair_valid_i,
    input  wire [2:0]            s_pair_word_i,
    input  wire [31:0]           s_pair_paddr_i,
    output wire                  s_ready_o,

    output wire                  m_valid_o,
    output wire [6:0]            m_vindex_o,
    output wire [31:0]           m_paddr_o,
    output wire [2:0]            m_size_o,
    output wire [7:4]            m_load_op_o,
    output wire                  m_uncached_o,
    output wire [`ROB_W-1:0]     m_robid_o,
    output wire                  m_pair_valid_o,
    output wire [2:0]            m_pair_word_o,
    output wire [31:0]           m_pair_paddr_o,
    input  wire                  m_ready_i
);

reg rd_ptr;
reg wr_ptr;
reg [1:0] count;
reg [6:0] vindex [0:1];
reg [31:0] paddr [0:1];
reg [2:0] size [0:1];
reg [7:4] load_op [0:1];
reg uncached [0:1];
reg [`ROB_W-1:0] robid [0:1];
reg pair_valid [0:1];
reg [2:0] pair_word [0:1];
reg [31:0] pair_paddr [0:1];

wire empty = (count == 2'd0);
assign s_ready_o = (count != 2'd2) && !flush_i;
// Empty bypass preserves the original zero-extra-cycle hit path.  Cache ready
// only decides whether this request must be captured locally; it never feeds
// s_ready_o and therefore cannot propagate back into LSU control.
assign m_valid_o = (empty ? s_valid_i : 1'b1) && !flush_i;
assign m_vindex_o = empty ? s_vindex_i : vindex[rd_ptr];
assign m_paddr_o = empty ? s_paddr_i : paddr[rd_ptr];
assign m_size_o = empty ? s_size_i : size[rd_ptr];
assign m_load_op_o = empty ? s_load_op_i : load_op[rd_ptr];
assign m_uncached_o = empty ? s_uncached_i : uncached[rd_ptr];
assign m_robid_o = empty ? s_robid_i : robid[rd_ptr];
assign m_pair_valid_o = empty ? s_pair_valid_i : pair_valid[rd_ptr];
assign m_pair_word_o = empty ? s_pair_word_i : pair_word[rd_ptr];
assign m_pair_paddr_o = empty ? s_pair_paddr_i : pair_paddr[rd_ptr];

wire bypass_accept = empty && s_valid_i && m_ready_i && !flush_i;
wire push = s_valid_i && s_ready_o && !bypass_accept;
wire pop = !empty && m_ready_i && !flush_i;

always @(posedge clk) begin
    if (reset || flush_i) begin
        rd_ptr <= 1'b0;
        wr_ptr <= 1'b0;
        count <= 2'd0;
    end else begin
        if (push) begin
            vindex[wr_ptr] <= s_vindex_i;
            paddr[wr_ptr] <= s_paddr_i;
            size[wr_ptr] <= s_size_i;
            load_op[wr_ptr] <= s_load_op_i;
            uncached[wr_ptr] <= s_uncached_i;
            robid[wr_ptr] <= s_robid_i;
            pair_valid[wr_ptr] <= s_pair_valid_i;
            pair_word[wr_ptr] <= s_pair_word_i;
            pair_paddr[wr_ptr] <= s_pair_paddr_i;
            wr_ptr <= ~wr_ptr;
        end
        if (pop)
            rd_ptr <= ~rd_ptr;
        case ({push, pop})
            2'b10: count <= count + 2'd1;
            2'b01: count <= count - 2'd1;
            default: count <= count;
        endcase
    end
end

endmodule
