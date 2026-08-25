// ============================================================
// Small two-level local-history conditional-branch predictor.
//
// TAGE is strongest on global correlations.  This predictor complements it
// with per-block 12-bit outcome histories and a shared 4096-entry table of
// 2-bit saturating counters.
// A per-block chooser starts by preferring TAGE and selects the local result
// only after local has repeatedly won on disagreements.  Prediction is one
// cycle, aligned with the existing FTB/TAGE P1 response.
// ============================================================
`include "mycpu.h"

module local_predictor(
    input  wire                  clk,
    input  wire                  reset,

    input  wire                  query_valid_i,
    input  wire [31:0]           query_pc_i,
    output reg                   resp_valid_o,
    output reg                   taken_o,
    output reg                   strong_o,
    output reg                   choose_local_o,

    input  wire                  train_valid_i,
    input  wire [31:0]           train_pc_i,
    input  wire                  train_taken_i,
    input  wire                  train_meta_valid_i,
    input  wire                  train_local_pred_i,
    input  wire                  train_tage_pred_i
);

localparam LHT_INDEX_W = 11;
localparam LHT_ENTRIES = 1 << LHT_INDEX_W;
localparam HIST_W      = 12;
localparam PHT_ENTRIES = 1 << HIST_W;

(* ram_style = "distributed" *) reg [HIST_W-1:0]
    local_hist [0:LHT_ENTRIES-1];
(* ram_style = "distributed" *) reg [1:0]
    pattern_ctr [0:PHT_ENTRIES-1];
(* ram_style = "distributed" *) reg [1:0]
    local_chooser [0:LHT_ENTRIES-1];

wire [LHT_INDEX_W-1:0] query_idx =
    query_pc_i[LHT_INDEX_W+1:2] ^ query_pc_i[2*LHT_INDEX_W+1:LHT_INDEX_W+2];
wire [HIST_W-1:0] query_hist = local_hist[query_idx];
wire [HIST_W-1:0] query_pattern_idx =
    query_hist ^ query_pc_i[HIST_W+1:2];
wire [1:0] query_ctr = pattern_ctr[query_pattern_idx];
wire [1:0] query_choose = local_chooser[query_idx];

wire [LHT_INDEX_W-1:0] train_idx =
    train_pc_i[LHT_INDEX_W+1:2] ^ train_pc_i[2*LHT_INDEX_W+1:LHT_INDEX_W+2];
wire [HIST_W-1:0] train_hist = local_hist[train_idx];
wire [HIST_W-1:0] train_pattern_idx =
    train_hist ^ train_pc_i[HIST_W+1:2];
wire [1:0] train_ctr = pattern_ctr[train_pattern_idx];

integer init_i;
initial begin
    resp_valid_o = 1'b0;
    taken_o = 1'b0;
    strong_o = 1'b0;
    choose_local_o = 1'b0;
    for (init_i = 0; init_i < LHT_ENTRIES; init_i = init_i + 1) begin
        local_hist[init_i] = {HIST_W{1'b0}};
        // Weakly prefer TAGE until local demonstrates a win.
        local_chooser[init_i] = 2'b01;
    end
    for (init_i = 0; init_i < PHT_ENTRIES; init_i = init_i + 1)
        pattern_ctr[init_i] = 2'b01;
end

always @(posedge clk) begin
    if (reset) begin
        resp_valid_o <= 1'b0;
        taken_o <= 1'b0;
        strong_o <= 1'b0;
        choose_local_o <= 1'b0;
    end else begin
        resp_valid_o <= query_valid_i;
        if (query_valid_i) begin
            taken_o <= query_ctr[1];
            strong_o <= (query_ctr == 2'b00) || (query_ctr == 2'b11);
            choose_local_o <= query_choose[1];
        end

        if (train_valid_i) begin
            pattern_ctr[train_pattern_idx] <= train_taken_i
                ? ((train_ctr == 2'b11) ? 2'b11 : train_ctr + 2'b01)
                : ((train_ctr == 2'b00) ? 2'b00 : train_ctr - 2'b01);
            local_hist[train_idx] <= {train_hist[HIST_W-2:0], train_taken_i};

            if (train_meta_valid_i &&
                (train_local_pred_i != train_tage_pred_i))
                local_chooser[train_idx] <=
                    (train_local_pred_i == train_taken_i)
                    ? ((local_chooser[train_idx] == 2'b11)
                       ? 2'b11 : local_chooser[train_idx] + 2'b01)
                    : ((local_chooser[train_idx] == 2'b00)
                       ? 2'b00 : local_chooser[train_idx] - 2'b01);
        end
    end
end

endmodule
