// ============================================================
// Bank-selected micro-BTB (P0 combinational prediction).
// ------------------------------------------------------------
// - Two banks, 16 fully-associative entries per bank (32 total).
// - A PC hash selects exactly one bank before tag comparison, so the P0
//   lookup still has 16 comparators rather than a 32-way CAM.
// - Conditional entries use 2-bit saturating direction counters. Existing
//   entries train on both taken and not-taken outcomes.
// - Only taken backward branches and RETs are filled at commit.  Early
//   predecode updates retain the original forced-admission behavior.
// - RET descriptors remain in the uBTB, while the BPU uses the RAS to
//   override their predicted target.
// - Each bank has an independent round-robin replacement pointer.
// ============================================================
`include "mycpu.h"

module ubtb(
    input  wire                       clk,
    input  wire                       reset,

    // ---------------- Combinational P0 query ----------------
    input  wire                       query_valid_i,
    input  wire [31:0]                query_pc_i,
    output wire                       hit_o,
    output wire                       taken_o,
    output wire [31:0]                target_o,
    output wire [`BLK_LEN_W-1:0]      length_o,
    output wire [`BR_TYPE_W-1:0]      br_type_o,

    // ---------------- Commit/predecode update ----------------
    input  wire                       update_valid_i,
    input  wire [31:0]                update_block_pc_i,
    input  wire                       update_taken_i,
    input  wire [31:0]                update_target_i,
    input  wire [`BLK_LEN_W-1:0]      update_length_i,
    input  wire [`BR_TYPE_W-1:0]      update_br_type_i,
    input  wire                       update_early_i
);

localparam BANK_DEPTH = 16;
localparam ENTRY_IDX_W = 4;
// A micro-BTB may use a partial tag: aliases are performance misses/mispredicts
// and are repaired by P1/commit.  PC[21:2] uniquely covers a 1 MiB code
// window while removing twelve bits from every P0 CAM comparator.
localparam TAG_W = 20;

// Mix low and high word-address bits so adjacent functions and loop bodies
// are distributed across the two banks.
function ubtb_bank;
    input [31:0] pc;
    begin
        ubtb_bank = pc[2] ^ pc[6] ^ pc[10] ^
                    pc[14] ^ pc[18] ^ pc[22];
    end
endfunction

reg [BANK_DEPTH-1:0] bank0_valid;
reg [BANK_DEPTH-1:0] bank1_valid;
reg [1:0] bank0_ctr [0:BANK_DEPTH-1];
reg [1:0] bank1_ctr [0:BANK_DEPTH-1];
reg [TAG_W-1:0] bank0_tag [0:BANK_DEPTH-1];
reg [TAG_W-1:0] bank1_tag [0:BANK_DEPTH-1];
reg [31:0] bank0_target [0:BANK_DEPTH-1];
reg [31:0] bank1_target [0:BANK_DEPTH-1];
reg [`BLK_LEN_W-1:0] bank0_length [0:BANK_DEPTH-1];
reg [`BLK_LEN_W-1:0] bank1_length [0:BANK_DEPTH-1];
reg [`BR_TYPE_W-1:0] bank0_btype [0:BANK_DEPTH-1];
reg [`BR_TYPE_W-1:0] bank1_btype [0:BANK_DEPTH-1];
reg [ENTRY_IDX_W-1:0] repl_ptr0;
reg [ENTRY_IDX_W-1:0] repl_ptr1;

// ---------------- Query: select bank, then compare 16 tags ----------------
wire q_bank = ubtb_bank(query_pc_i);
wire [TAG_W-1:0] q_tag = query_pc_i[21:2];
wire [BANK_DEPTH-1:0] q_hit;

genvar g;
generate
for (g = 0; g < BANK_DEPTH; g = g + 1) begin : gen_qhit
    wire selected_valid = q_bank ? bank1_valid[g] : bank0_valid[g];
    wire [TAG_W-1:0] selected_tag =
        q_bank ? bank1_tag[g] : bank0_tag[g];
    assign q_hit[g] = selected_valid && (selected_tag == q_tag);
end
endgenerate

reg [ENTRY_IDX_W-1:0] q_idx;
integer qi;
always @(*) begin
    q_idx = {ENTRY_IDX_W{1'b0}};
    for (qi = BANK_DEPTH-1; qi >= 0; qi = qi - 1)
        if (q_hit[qi])
            q_idx = qi[ENTRY_IDX_W-1:0];
end

assign hit_o = |q_hit;
assign target_o =
    q_bank ? bank1_target[q_idx] : bank0_target[q_idx];
assign length_o =
    q_bank ? bank1_length[q_idx] : bank0_length[q_idx];
assign br_type_o =
    q_bank ? bank1_btype[q_idx] : bank0_btype[q_idx];
wire [1:0] q_direction_ctr =
    q_bank ? bank1_ctr[q_idx] : bank0_ctr[q_idx];
assign taken_o =
    hit_o &&
    ((br_type_o != `BR_TYPE_COND) || q_direction_ctr[1]);

// ---------------- Update ----------------
wire do_fill = update_valid_i &&
               (update_early_i ||
                (update_taken_i &&
                 ((update_target_i < update_block_pc_i) ||
                  (update_br_type_i == `BR_TYPE_RET))));

wire u_bank = ubtb_bank(update_block_pc_i);
wire [TAG_W-1:0] u_tag = update_block_pc_i[21:2];
wire [BANK_DEPTH-1:0] u_hit;
wire [BANK_DEPTH-1:0] u_invalid;

generate
for (g = 0; g < BANK_DEPTH; g = g + 1) begin : gen_uhit
    wire selected_valid = u_bank ? bank1_valid[g] : bank0_valid[g];
    wire [TAG_W-1:0] selected_tag =
        u_bank ? bank1_tag[g] : bank0_tag[g];
    assign u_hit[g] = selected_valid &&
                      (selected_tag == u_tag);
    assign u_invalid[g] = !selected_valid;
end
endgenerate

reg [ENTRY_IDX_W-1:0] u_idx;
reg u_found;
reg [ENTRY_IDX_W-1:0] inv_idx;
reg inv_found;
integer ui;
always @(*) begin
    u_found = 1'b0;
    u_idx = {ENTRY_IDX_W{1'b0}};
    inv_found = 1'b0;
    inv_idx = {ENTRY_IDX_W{1'b0}};
    for (ui = BANK_DEPTH-1; ui >= 0; ui = ui - 1) begin
        if (u_hit[ui]) begin
            u_found = 1'b1;
            u_idx = ui[ENTRY_IDX_W-1:0];
        end
        if (u_invalid[ui]) begin
            inv_found = 1'b1;
            inv_idx = ui[ENTRY_IDX_W-1:0];
        end
    end
end

wire [ENTRY_IDX_W-1:0] bank_repl_ptr =
    u_bank ? repl_ptr1 : repl_ptr0;
wire [ENTRY_IDX_W-1:0] fill_idx =
    u_found ? u_idx : inv_found ? inv_idx : bank_repl_ptr;
wire replacement_event = do_fill && !u_found && !inv_found;
// Existing entries see every resolved direction, including not-taken
// outcomes that are intentionally ineligible for new allocation.
wire do_update = do_fill || (update_valid_i && u_found);

always @(posedge clk) begin
    if (reset) begin
        bank0_valid <= {BANK_DEPTH{1'b0}};
        bank1_valid <= {BANK_DEPTH{1'b0}};
        repl_ptr0   <= {ENTRY_IDX_W{1'b0}};
        repl_ptr1   <= {ENTRY_IDX_W{1'b0}};
    end else if (do_update) begin
        if (u_bank == 1'b0) begin
            bank0_valid[fill_idx]  <= 1'b1;
            bank0_tag[fill_idx]    <= u_tag;
            bank0_target[fill_idx] <= update_target_i;
            bank0_length[fill_idx] <= update_length_i;
            bank0_btype[fill_idx]  <= update_br_type_i;
            if (!u_found)
                bank0_ctr[fill_idx] <=
                    (update_br_type_i != `BR_TYPE_COND) ? 2'b11 :
                    update_taken_i ? 2'b10 : 2'b01;
            else if (update_br_type_i != `BR_TYPE_COND)
                bank0_ctr[fill_idx] <= 2'b11;
            else if (update_taken_i && (bank0_ctr[fill_idx] != 2'b11))
                bank0_ctr[fill_idx] <= bank0_ctr[fill_idx] + 2'b01;
            else if (!update_taken_i && (bank0_ctr[fill_idx] != 2'b00))
                bank0_ctr[fill_idx] <= bank0_ctr[fill_idx] - 2'b01;
            if (!u_found && !inv_found)
                repl_ptr0 <= repl_ptr0 + {{(ENTRY_IDX_W-1){1'b0}}, 1'b1};
        end else begin
            bank1_valid[fill_idx]  <= 1'b1;
            bank1_tag[fill_idx]    <= u_tag;
            bank1_target[fill_idx] <= update_target_i;
            bank1_length[fill_idx] <= update_length_i;
            bank1_btype[fill_idx]  <= update_br_type_i;
            if (!u_found)
                bank1_ctr[fill_idx] <=
                    (update_br_type_i != `BR_TYPE_COND) ? 2'b11 :
                    update_taken_i ? 2'b10 : 2'b01;
            else if (update_br_type_i != `BR_TYPE_COND)
                bank1_ctr[fill_idx] <= 2'b11;
            else if (update_taken_i && (bank1_ctr[fill_idx] != 2'b11))
                bank1_ctr[fill_idx] <= bank1_ctr[fill_idx] + 2'b01;
            else if (!update_taken_i && (bank1_ctr[fill_idx] != 2'b00))
                bank1_ctr[fill_idx] <= bank1_ctr[fill_idx] - 2'b01;
            if (!u_found && !inv_found)
                repl_ptr1 <= repl_ptr1 + {{(ENTRY_IDX_W-1){1'b0}}, 1'b1};
        end
    end
end

`ifdef SYNTHESIS
// synthesis translate_off
// Simulation-only counters.  SIMU defines SYNTHESIS in mycpu.h; Vivado
// ignores this block because of translate_off.
reg [63:0] ubtb_query_count;
reg [63:0] ubtb_hit_count;
reg [63:0] ubtb_miss_count;
reg [63:0] ubtb_bank0_hit_count;
reg [63:0] ubtb_bank1_hit_count;
reg [63:0] ubtb_fill_count;
reg [63:0] ubtb_update_hit_count;
reg [63:0] ubtb_replacement_count;

always @(posedge clk) begin
    if (reset) begin
        ubtb_query_count       <= 64'd0;
        ubtb_hit_count         <= 64'd0;
        ubtb_miss_count        <= 64'd0;
        ubtb_bank0_hit_count   <= 64'd0;
        ubtb_bank1_hit_count   <= 64'd0;
        ubtb_fill_count        <= 64'd0;
        ubtb_update_hit_count  <= 64'd0;
        ubtb_replacement_count <= 64'd0;
    end else begin
        if (query_valid_i) begin
            ubtb_query_count <= ubtb_query_count + 64'd1;
            if (hit_o) begin
                ubtb_hit_count <= ubtb_hit_count + 64'd1;
                if (q_bank)
                    ubtb_bank1_hit_count <= ubtb_bank1_hit_count + 64'd1;
                else
                    ubtb_bank0_hit_count <= ubtb_bank0_hit_count + 64'd1;
            end else begin
                ubtb_miss_count <= ubtb_miss_count + 64'd1;
            end
        end

        if (do_fill && !u_found)
            ubtb_fill_count <= ubtb_fill_count + 64'd1;
        if (update_valid_i && u_found)
            ubtb_update_hit_count <= ubtb_update_hit_count + 64'd1;
        if (do_fill) begin
            if (replacement_event)
                ubtb_replacement_count <= ubtb_replacement_count + 64'd1;
        end
    end
end
// synthesis translate_on
`endif

endmodule
