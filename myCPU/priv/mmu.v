`include "mycpu.h"

// ============================================================
// mmu 模块（地址翻译门面：I/D 双通道整形 + 异常向量打包，纯组合）
// ------------------------------------------------------------
// 功能（新架构角色）：
// - 在 ifu/lsu 与 tlb_manager 之间做薄封装：
//   * I 通道：ifu 的取指 vaddr -> tlb_manager s0 -> paddr/mat/取指异常向量
//   * D 通道：lsu 的访存 vaddr -> tlb_manager s1 -> paddr/mat/访存异常向量
// - ADEF/ADEM 检测分两处：
//   * 取指非对齐（vaddr[1:0]!=0）在本模块本地检测；
//   * PLV3 越界（映射模式访问 va[31]=1 内核段）在 tlb_manager 内检测
//     （那里有全部 CSR 状态），本模块按 req 门控透传后与本地 ADEF 合并。
// - 把 tlb_manager 的分立异常位打包成 `TLB_EX_NUM 向量（ifu/lsu 端口格式）。
// - 旧设计中的"数据透传（bridge_*）"通路已被 cache 直连取代，删除。
//
// 职责边界（与旧设计的差别，重要）：
// 1. 有取指/访存异常时，ifu/lsu 自己负责"不发 cache 请求"，本模块
//    不做请求压制（旧设计在这里压请求，新架构职责移到了发起方）。
// 2. ADEF 优先级高于 TLB 异常（地址本身非法就不必看查表结果），
//    ifu 侧合并异常向量时先判 ADEF；本模块两者都原样给出。
// 3. 所有输出都是组合逻辑，翻译在请求同拍完成（主 TLB 为全相联组合查找，
//    l1_tlb 微表加速见 tlb_manager 内部）。
//
// 端口：
// - i_*  ：I 通道（连 ifu 的 mmu_i_*）
// - d_*  ：D 通道（连 lsu 的 mmu_d_*）
// - tlbm_*：tlb_manager 的翻译输入/输出透传
// ============================================================
module mmu (
    // ---------------- I 通道（ifu）----------------
    input  wire        i_req_i,
    input  wire [31:0] i_vaddr_i,
    output wire [31:0] i_paddr_o,
    output wire [1:0]  i_mat_o,
    output wire        i_excp_adef_o,
    output wire [`TLB_EX_NUM-1:0] i_tlb_ex_o,
    output wire        i_direct_ok_o,

    // ---------------- D 通道（lsu）----------------
    input  wire        d_req_i,
    input  wire        d_is_store_i,
    input  wire [31:0] d_vaddr_i,
    output wire [31:0] d_paddr_o,
    output wire [1:0]  d_mat_o,
    output wire        d_excp_adem_o,
    output wire [`TLB_EX_NUM-1:0] d_tlb_ex_o,

    // ---------------- tlb_manager 翻译通道透传 ----------------
    output wire        tlbm_inst_req_o,      // -> tlb_manager.inst_req
    output wire [31:0] tlbm_inst_vaddr_o,    // -> tlb_manager.inst_vaddr
    output wire        tlbm_data_req_o,      // -> tlb_manager.data_req
    output wire        tlbm_data_is_store_o, // -> tlb_manager.data_is_store
    output wire [31:0] tlbm_data_vaddr_o,    // -> tlb_manager.data_vaddr

    input  wire [31:0] tlbm_inst_paddr_i,    // <- tlb_manager.inst_paddr
    input  wire [1:0]  tlbm_inst_mat_i,
    input  wire        tlbm_inst_ex_adef_i,  // <- tlb_manager.inst_ex_adef（PLV3 取指越界）
    input  wire        tlbm_inst_ex_tlbr_i,
    input  wire        tlbm_inst_ex_pif_i,
    input  wire        tlbm_inst_ex_ppi_i,
    input  wire        tlbm_inst_direct_ok_i,
    input  wire [31:0] tlbm_data_paddr_i,
    input  wire [1:0]  tlbm_data_mat_i,
    input  wire        tlbm_data_ex_tlbr_i,
    input  wire        tlbm_data_ex_pil_i,
    input  wire        tlbm_data_ex_pis_i,
    input  wire        tlbm_data_ex_ppi_i,
    input  wire        tlbm_data_ex_pme_i,
    input  wire        tlbm_data_ex_adem_i   // <- tlb_manager.data_ex_adem（PLV3 越界）
);

// ---------------- 请求透传 ----------------
assign tlbm_inst_req_o      = i_req_i;
assign tlbm_inst_vaddr_o    = i_vaddr_i;
assign tlbm_data_req_o      = d_req_i;
assign tlbm_data_is_store_o = d_is_store_i;
assign tlbm_data_vaddr_o    = d_vaddr_i;

// ---------------- 结果透传与本地异常 ----------------
// 用 === 比较做 4-state 防御：复位/冲刷窗口 req 或 vaddr 可能为 X，
// 不能让 X 渗入异常位（会顺着 ifu/lsu 的异常向量污染 ROB）。
assign i_paddr_o     = tlbm_inst_paddr_i;
assign i_mat_o       = tlbm_inst_mat_i;
assign i_direct_ok_o = tlbm_inst_direct_ok_i;
// ADEF = 取指非对齐（本地）| PLV3 越界（tlb_manager 检测）
assign i_excp_adef_o = ((i_req_i === 1'b1) && (i_vaddr_i[1:0] != 2'b00))
                     || ((i_req_i === 1'b1) && (tlbm_inst_ex_adef_i === 1'b1));
assign d_paddr_o     = tlbm_data_paddr_i;
assign d_mat_o       = tlbm_data_mat_i;
assign d_excp_adem_o = (d_req_i === 1'b1) && (tlbm_data_ex_adem_i === 1'b1);

// ---------------- 异常向量打包（位序见 mycpu.h 的 TLB_EX_*）----------------
wire i_req_ok = (i_req_i === 1'b1);
wire d_req_ok = (d_req_i === 1'b1);

assign i_tlb_ex_o = {`TLB_EX_NUM{1'b0}}
                  | ({{(`TLB_EX_NUM-1){1'b0}}, i_req_ok && (tlbm_inst_ex_tlbr_i === 1'b1)} << `TLB_EX_TLBR)
                  | ({{(`TLB_EX_NUM-1){1'b0}}, i_req_ok && (tlbm_inst_ex_pif_i  === 1'b1)} << `TLB_EX_PIF)
                  | ({{(`TLB_EX_NUM-1){1'b0}}, i_req_ok && (tlbm_inst_ex_ppi_i  === 1'b1)} << `TLB_EX_PPI);

assign d_tlb_ex_o = {`TLB_EX_NUM{1'b0}}
                  | ({{(`TLB_EX_NUM-1){1'b0}}, d_req_ok && (tlbm_data_ex_tlbr_i === 1'b1)} << `TLB_EX_TLBR)
                  | ({{(`TLB_EX_NUM-1){1'b0}}, d_req_ok && (tlbm_data_ex_pil_i  === 1'b1)} << `TLB_EX_PIL)
                  | ({{(`TLB_EX_NUM-1){1'b0}}, d_req_ok && (tlbm_data_ex_pis_i  === 1'b1)} << `TLB_EX_PIS)
                  | ({{(`TLB_EX_NUM-1){1'b0}}, d_req_ok && (tlbm_data_ex_ppi_i  === 1'b1)} << `TLB_EX_PPI)
                  | ({{(`TLB_EX_NUM-1){1'b0}}, d_req_ok && (tlbm_data_ex_pme_i  === 1'b1)} << `TLB_EX_PME);

endmodule
