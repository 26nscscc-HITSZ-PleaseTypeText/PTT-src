`include "mycpu.h"

// Physically isolated selector for the optional second cached-load lane.
// Only stored-ready state enters this block; current WB tags deliberately do
// not.  Keeping this hierarchy prevents synthesis from sharing its head muxes
// with rs_mem's timing-sensitive same-cycle wakeup/primary-issue network.
(* keep_hierarchy = "yes", dont_touch = "true" *)
module rs_mem_pair_select(
    input  wire [`RS_MEM_IDX_W-1:0] head_i,
    input  wire [`RS_MEM_OCC_W-1:0] count_i,
    input  wire [`RS_MEM_SIZE-1:0] stored_ready_i,
    input  wire [`RS_MEM_SIZE-1:0] plain_load_i,
    input  wire [`RS_MEM_SIZE*`ROB_W-1:0] robid_i,
    input  wire [`RS_MEM_SIZE*`MEM_OP_NUM-1:0] mem_op_i,
    input  wire [`RS_MEM_SIZE*32-1:0] base_i,
    input  wire [`RS_MEM_SIZE*32-1:0] imm_i,
    input  wire [`RS_MEM_SIZE*27-1:0] vaddr_key_i,
`ifdef SOC_USE
    input  wire [`RS_MEM_SIZE-1:0] known_cached_i,
`else
`endif
    output wire candidate_o,
    output reg  [`RS_MEM_SIZE-1:0] pair_grant_o,
    output reg  [`RS_MEM_IDX_W-1:0] pair_idx_o,
    output reg  [`ROB_W-1:0] pair_robid_o,
    output reg  [`MEM_OP_NUM-1:0] pair_mem_op_o,
    output reg  [31:0] pair_base_o,
    output reg  [31:0] pair_imm_o
);

reg candidate_r;
reg scan_stop;
reg [`RS_MEM_IDX_W-1:0] scan_idx;
`ifdef SOC_USE
`else
reg [31:0] head_base, head_imm;
`endif
reg [31:0] scan_base, scan_imm;
reg [26:0] head_vaddr_key, scan_vaddr_key;
integer age;

// The primary lane is the ready queue head.  Search the following load-only
// window for the first ready request that the second DCache RAM port can
// serve.  An unready load may be bypassed (the ordinary single-issue selector
// already has this property), but a store, LL/SC or CACOP terminates the
// search so memory ordering is never relaxed.
always @(*) begin
    candidate_r = 1'b0;
    scan_stop = 1'b0;
    scan_idx = head_i;
`ifdef SOC_USE
`else
    head_base = base_i[head_i*32 +: 32];
    head_imm = imm_i[head_i*32 +: 32];
`endif
    head_vaddr_key = vaddr_key_i[head_i*27 +: 27];
    scan_base = 32'b0;
    scan_imm = 32'b0;
    scan_vaddr_key = 27'b0;
    pair_robid_o = {`ROB_W{1'b0}};
    pair_grant_o = {`RS_MEM_SIZE{1'b0}};
    pair_idx_o = head_i;
    pair_mem_op_o = {`MEM_OP_NUM{1'b0}};
    pair_base_o = 32'b0;
    pair_imm_o = 32'b0;

    if ((count_i < {{(`RS_MEM_OCC_W-2){1'b0}}, 2'd2}) ||
`ifdef SOC_USE
        !stored_ready_i[head_i] || !plain_load_i[head_i] ||
        !known_cached_i[head_i])
`else
        !stored_ready_i[head_i] || !plain_load_i[head_i])
`endif
        scan_stop = 1'b1;

    for (age = 1; age < `RS_MEM_SIZE; age = age + 1) begin
        scan_idx = head_i + age[`RS_MEM_IDX_W-1:0];
        if (!scan_stop && (age[`RS_MEM_OCC_W-1:0] < count_i)) begin
`ifdef SOC_USE
            if (!plain_load_i[scan_idx] || !known_cached_i[scan_idx]) begin
`else
            if (!plain_load_i[scan_idx]) begin
`endif
                scan_stop = 1'b1;
            end else if (stored_ready_i[scan_idx]) begin
                scan_base = base_i[scan_idx*32 +: 32];
                scan_imm = imm_i[scan_idx*32 +: 32];
                scan_vaddr_key = vaddr_key_i[scan_idx*27 +: 27];
                if ((head_vaddr_key[26:7] == scan_vaddr_key[26:7]) &&
                    ((head_vaddr_key == scan_vaddr_key) ||
                     (head_vaddr_key[0] != scan_vaddr_key[0]))) begin
                    candidate_r = 1'b1;
                    pair_grant_o[scan_idx] = 1'b1;
                    pair_idx_o = scan_idx;
                    pair_robid_o = robid_i[scan_idx*`ROB_W +: `ROB_W];
                    pair_mem_op_o = mem_op_i[scan_idx*`MEM_OP_NUM +: `MEM_OP_NUM];
                    pair_base_o = scan_base;
                    pair_imm_o = scan_imm;
                    scan_stop = 1'b1;
                end
            end
        end
    end
end

assign candidate_o = candidate_r;

endmodule
