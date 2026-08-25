// ============================================================
// Direct-mapped P0 fallback BTB.
// ------------------------------------------------------------
// The primary uBTB remains two 16-entry bank-selected CAMs.  This table is
// consulted only when that uBTB misses and therefore does not add another
// associative lookup to P0.
//
// This module extends the former block-length cache.  The existing
// 64-entry direct-mapped tag lookup now retains the complete descriptor:
//   {block length, target, branch type, 2-bit direction counter}.
// Conditional branches use the local counter; unconditional branches,
// calls and returns are always predicted taken.  The BPU still gives the
// RAS priority over the stored target for returns.
//
// Early IFU predecode updates provide a trustworthy static descriptor but
// not a resolved conditional-branch outcome.  They allocate conditional
// entries weakly not-taken and never train an existing direction counter.
// Commit updates train the counter with the resolved outcome.
// ============================================================
`include "mycpu.h"

module fallback_btb(
    input  wire                       clk,
    input  wire                       reset,

    input  wire                       query_valid_i,
    input  wire [31:0]                query_pc_i,
    output wire                       hit_o,
    output wire                       taken_o,
    output wire [31:0]                target_o,
    output wire [`BLK_LEN_W-1:0]      length_o,
    output wire [`BR_TYPE_W-1:0]      br_type_o,
    output wire                       strong_taken_o,
    output wire                       static_direct_o,

    input  wire                       update_valid_i,
    input  wire [31:0]                update_block_pc_i,
    input  wire                       update_taken_i,
    input  wire [31:0]                update_target_i,
    input  wire [`BLK_LEN_W-1:0]      update_length_i,
    input  wire [`BR_TYPE_W-1:0]      update_br_type_i,
    input  wire                       update_early_i
);

// At 65 MHz we can spend distributed RAM to avoid destructive aliasing in
// the early descriptor/direction table.  The lookup remains only two ways;
// capacity grows without adding a wider associative compare or a stage.
// The 1024-set version improved weighted IPC by only about 0.0003 in the
// recorded sweep while this replicated asynchronous table is one of the
// largest LUT users.  Use 512 sets to recover several thousand LUTs without
// changing associativity, replacement, or the P0 lookup latency.
localparam ENTRY_COUNT = 64;
localparam INDEX_W     = 6;
localparam TAG_W       = 32 - INDEX_W - 2;

wire [INDEX_W-1:0] query_index  = query_pc_i[INDEX_W+1:2];
wire [INDEX_W-1:0] update_index = update_block_pc_i[INDEX_W+1:2];
wire [TAG_W-1:0] query_tag  = query_pc_i[31:INDEX_W+2];
wire [TAG_W-1:0] update_tag = update_block_pc_i[31:INDEX_W+2];

reg [ENTRY_COUNT-1:0] valid0;
reg [ENTRY_COUNT-1:0] valid1;
reg [ENTRY_COUNT-1:0] replace_way;
(* ram_style = "distributed" *) reg [TAG_W-1:0] tag_mem0 [0:ENTRY_COUNT-1];
(* ram_style = "distributed" *) reg [TAG_W-1:0] tag_mem1 [0:ENTRY_COUNT-1];
(* ram_style = "distributed" *) reg [31:0] target_mem0 [0:ENTRY_COUNT-1];
(* ram_style = "distributed" *) reg [31:0] target_mem1 [0:ENTRY_COUNT-1];
(* ram_style = "distributed" *) reg [`BLK_LEN_W-1:0] length_mem0 [0:ENTRY_COUNT-1];
(* ram_style = "distributed" *) reg [`BLK_LEN_W-1:0] length_mem1 [0:ENTRY_COUNT-1];
(* ram_style = "distributed" *) reg [`BR_TYPE_W-1:0] btype_mem0 [0:ENTRY_COUNT-1];
(* ram_style = "distributed" *) reg [`BR_TYPE_W-1:0] btype_mem1 [0:ENTRY_COUNT-1];
(* ram_style = "distributed" *) reg [1:0] direction_mem0 [0:ENTRY_COUNT-1];
(* ram_style = "distributed" *) reg [1:0] direction_mem1 [0:ENTRY_COUNT-1];
(* ram_style = "distributed" *) reg static_direct_mem0 [0:ENTRY_COUNT-1];
(* ram_style = "distributed" *) reg static_direct_mem1 [0:ENTRY_COUNT-1];

wire stored_hit0 = valid0[query_index] &&
                   (tag_mem0[query_index] == query_tag);
wire stored_hit1 = valid1[query_index] &&
                   (tag_mem1[query_index] == query_tag);
wire query_way1 = !stored_hit0 && stored_hit1;
wire [1:0] query_direction = query_way1
                           ? direction_mem1[query_index]
                           : direction_mem0[query_index];
assign hit_o     = query_valid_i && (stored_hit0 || stored_hit1);
assign target_o  = query_way1 ? target_mem1[query_index]
                              : target_mem0[query_index];
assign length_o  = query_way1 ? length_mem1[query_index]
                              : length_mem0[query_index];
assign br_type_o = query_way1 ? btype_mem1[query_index]
                              : btype_mem0[query_index];
assign strong_taken_o =
    hit_o && (br_type_o == `BR_TYPE_COND) &&
    (query_direction == 2'b11);
assign static_direct_o = hit_o &&
                         (query_way1 ? static_direct_mem1[query_index]
                                     : static_direct_mem0[query_index]);
assign taken_o   = hit_o &&
                   ((br_type_o != `BR_TYPE_COND) ||
                    query_direction[1]);

// Invalid lengths must never enter P0.  All normal producers generate
// 1..FETCH_WIDTH, but the guard keeps a malformed training record from
// creating a zero-length fetch loop.
wire update_length_valid =
    (update_length_i != {`BLK_LEN_W{1'b0}}) &&
    (update_length_i <= `FETCH_WIDTH);
wire do_update = update_valid_i && update_length_valid;
wire update_hit0 =
    valid0[update_index] && (tag_mem0[update_index] == update_tag);
wire update_hit1 =
    valid1[update_index] && (tag_mem1[update_index] == update_tag);
wire update_entry_hit = update_hit0 || update_hit1;
wire update_way1 = update_hit1 ? 1'b1 :
                   update_hit0 ? 1'b0 :
                   !valid0[update_index] ? 1'b0 :
                   !valid1[update_index] ? 1'b1 :
                   replace_way[update_index];
wire update_same_type =
    update_entry_hit &&
    ((update_way1 ? btype_mem1[update_index] : btype_mem0[update_index])
     == update_br_type_i);
// IFU predecode classifies only immediate B/BL as UNCOND/CALL.  Ordinary
// JIRL and indirect CALL are deliberately not marked static here.
wire update_marks_static_direct =
    update_early_i &&
    ((update_br_type_i == `BR_TYPE_UNCOND) ||
     (update_br_type_i == `BR_TYPE_CALL));

always @(posedge clk) begin
    if (reset) begin
        valid0      <= {ENTRY_COUNT{1'b0}};
        valid1      <= {ENTRY_COUNT{1'b0}};
        replace_way <= {ENTRY_COUNT{1'b0}};
    end else if (do_update) begin
        replace_way[update_index] <= !update_way1;
        if (update_way1) begin
            valid1[update_index]      <= 1'b1;
            tag_mem1[update_index]    <= update_tag;
            target_mem1[update_index] <= update_target_i;
            length_mem1[update_index] <= update_length_i;
            btype_mem1[update_index]  <= update_br_type_i;
            if (!update_same_type)
                static_direct_mem1[update_index] <=
                    update_marks_static_direct;
            else if (update_marks_static_direct)
                static_direct_mem1[update_index] <= 1'b1;

            if (update_br_type_i != `BR_TYPE_COND)
                direction_mem1[update_index] <= 2'b11;
            else if (!update_same_type)
                direction_mem1[update_index] <=
                    update_early_i ? 2'b01 :
                    update_taken_i ? 2'b10 : 2'b01;
            else if (!update_early_i)
                direction_mem1[update_index] <=
                    update_taken_i
                    ? ((direction_mem1[update_index] == 2'b11)
                       ? 2'b11 : direction_mem1[update_index] + 2'b01)
                    : ((direction_mem1[update_index] == 2'b00)
                       ? 2'b00 : direction_mem1[update_index] - 2'b01);
        end else begin
            valid0[update_index]      <= 1'b1;
            tag_mem0[update_index]    <= update_tag;
            target_mem0[update_index] <= update_target_i;
            length_mem0[update_index] <= update_length_i;
            btype_mem0[update_index]  <= update_br_type_i;
            if (!update_same_type)
                static_direct_mem0[update_index] <=
                    update_marks_static_direct;
            else if (update_marks_static_direct)
                static_direct_mem0[update_index] <= 1'b1;

            if (update_br_type_i != `BR_TYPE_COND)
                direction_mem0[update_index] <= 2'b11;
            else if (!update_same_type)
                direction_mem0[update_index] <=
                    update_early_i ? 2'b01 :
                    update_taken_i ? 2'b10 : 2'b01;
            else if (!update_early_i)
                direction_mem0[update_index] <=
                    update_taken_i
                    ? ((direction_mem0[update_index] == 2'b11)
                       ? 2'b11 : direction_mem0[update_index] + 2'b01)
                    : ((direction_mem0[update_index] == 2'b00)
                       ? 2'b00 : direction_mem0[update_index] - 2'b01);
        end
    end
end

`ifdef SYNTHESIS
// synthesis translate_off
reg [63:0] fallback_query_count;
reg [63:0] fallback_hit_count;
reg [63:0] fallback_cond_hit_count;
reg [63:0] fallback_uncond_hit_count;
reg [63:0] fallback_call_hit_count;
reg [63:0] fallback_ret_hit_count;
reg [63:0] fallback_taken_count;
reg [63:0] fallback_update_count;
reg [63:0] fallback_direction_train_count;
reg [63:0] fallback_replacement_count;

wire update_replaces_entry =
    do_update && !update_entry_hit &&
    valid0[update_index] && valid1[update_index];

always @(posedge clk) begin
    if (reset) begin
        fallback_query_count           <= 64'd0;
        fallback_hit_count             <= 64'd0;
        fallback_cond_hit_count        <= 64'd0;
        fallback_uncond_hit_count      <= 64'd0;
        fallback_call_hit_count        <= 64'd0;
        fallback_ret_hit_count         <= 64'd0;
        fallback_taken_count           <= 64'd0;
        fallback_update_count          <= 64'd0;
        fallback_direction_train_count <= 64'd0;
        fallback_replacement_count     <= 64'd0;
    end else begin
        if (query_valid_i) begin
            fallback_query_count <= fallback_query_count + 64'd1;
            if (hit_o) begin
                fallback_hit_count <= fallback_hit_count + 64'd1;
                case (br_type_o)
                    `BR_TYPE_COND:
                        fallback_cond_hit_count <=
                            fallback_cond_hit_count + 64'd1;
                    `BR_TYPE_UNCOND:
                        fallback_uncond_hit_count <=
                            fallback_uncond_hit_count + 64'd1;
                    `BR_TYPE_CALL:
                        fallback_call_hit_count <=
                            fallback_call_hit_count + 64'd1;
                    default:
                        fallback_ret_hit_count <=
                            fallback_ret_hit_count + 64'd1;
                endcase
                if (taken_o)
                    fallback_taken_count <= fallback_taken_count + 64'd1;
            end
        end
        if (do_update)
            fallback_update_count <= fallback_update_count + 64'd1;
        if (do_update && !update_early_i &&
            (update_br_type_i == `BR_TYPE_COND))
            fallback_direction_train_count <=
                fallback_direction_train_count + 64'd1;
        if (update_replaces_entry)
            fallback_replacement_count <=
                fallback_replacement_count + 64'd1;
    end
end
// synthesis translate_on
`endif

endmodule
