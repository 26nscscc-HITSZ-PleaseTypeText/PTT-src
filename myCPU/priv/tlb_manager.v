`include "mycpu.h"

// ============================================================
// tlb_manager 模块（地址翻译与 TLB 维护封装，内含 TLBNUM 项主 TLB + 2 份 L1 微 TLB）
// ------------------------------------------------------------
// 参数：TLBNUM 主表项数（模块默认 16，顶层 core_top 例化为 32）。
//
// 功能（新架构下角色不变，对接对象变化）：
// - 翻译通道：inst（s0 口，接 mmu 的 I 通道）/ data（s1 口，接 mmu 的 D 通道），
//   组合完成 DA 直址 / DMW 窗口 / TLB 查表三种模式与页表异常生成；
// - L1 微 TLB（二期加速，l1_tlb.v）：I/D 各一份 8 项微表插在主表 s0/s1 查询口
//   之前——命中时只比较 8 项（替代 TLBNUM 项全相联比较链），miss 透传主表并回填；
//   fence（TLB 写/无效化/ASID 变化）整表失效，对软件完全透明；
// - 特权地址检查：
//   * inst_ex_adef：PLV3 映射模式取指 va[31]=1 且未落 PLV3 可用 DMW（ADEF 的
//     特权子情形；PC 非对齐的 ADEF 由 mmu 本地检测，两者在 mmu 侧合并）；
//   * data_ex_adem：PLV3 映射模式访存 va[31]=1 且未落 PLV3 可用 DMW；
//   * 地址本身非法时不再报 TLB 类异常（地址错优先于查表结果）；
// - 维护通道：tlb_op/invtlb_* 由【commit 提交级】驱动（旧设计是 WB 级，
//   语义相同——只在指令确定提交时落地，且伴随全局 FLUSH_REFETCH 冲刷）；
// - tlbsrch/tlbrd 结果回送 csr_exception_commit_handler（接口原样保留）。
//
// 维护时序说明：
// - tlb_op 是 commit 提交拍的一拍脉冲，与原 WB 提交语义一致；
// - tlbsrch 走主表专用 srch 口（V2.3 提频改动）：输入直接取
//   CSR.TLBEHI/CSR.ASID 寄存器，found/index 每拍常备，提交拍由
//   csr handler 采样写回 TLBIDX。s0 口不再被挪用，commit 逻辑
//   与「TLB 查表 → 翻译 → 取指」关键路径完全解耦，I 侧 l1_tlb
//   也不再需要 dis_refill 屏蔽（s0 口永远承载真实取指查询）。
//
// 端口：顶层接线变化：
// - inst_req/inst_vaddr  <- mmu.tlbm_inst_*（原来自 IF）
// - data_req/.../vaddr   <- mmu.tlbm_data_*（原来自 EXE/MEM）
// - tlb_op/invtlb_*      <- commit 提交口（原来自 WB）
// - csr_* 输入           <- csr_exception_commit_handler（不变）
// - inst_ex_adef/data_ex_adem -> mmu（新增，特权地址错）
// ============================================================
module tlb_manager #(
    parameter TLBNUM = 16
) (
    input  wire                         clk,
    input  wire                         reset,

    input  wire                         inst_req,
    input  wire [31:0]                  inst_vaddr,
    input  wire                         data_req,
    input  wire                         data_is_store,
    input  wire [31:0]                  data_vaddr,

    input  wire                         csr_crmd_da,
    input  wire                         csr_crmd_pg,
    input  wire [1:0]                   csr_crmd_plv,
    input  wire [1:0]                   csr_crmd_datf,
    input  wire [1:0]                   csr_crmd_datm,
    input  wire [9:0]                   csr_asid,
    input  wire [31:0]                  csr_tlbidx,
    input  wire [31:0]                  csr_tlbehi,
    input  wire [31:0]                  csr_tlbelo0,
    input  wire [31:0]                  csr_tlbelo1,
    input  wire [31:0]                  csr_dmw0,
    input  wire [31:0]                  csr_dmw1,
    input  wire [7:0]                   csr_estat_ecode,
    input  wire [$clog2(TLBNUM)-1:0]    csr_rand_index,

    input  wire [`TLB_OP_NUM-1:0]       tlb_op,
    input  wire [4:0]                   invtlb_op,
    input  wire [9:0]                   invtlb_asid,
    input  wire [18:0]                  invtlb_vpn,

    output wire [31:0]                  inst_paddr,
    output wire [1:0]                   inst_mat,
    output wire                         inst_ex_adef,   // PLV3 取指越界（ADEF 特权子情形）
    output wire                         inst_ex_tlbr,
    output wire                         inst_ex_pif,
    output wire                         inst_ex_ppi,

    output wire [31:0]                  data_paddr,
    output wire [1:0]                   data_mat,
    output wire                         data_ex_adem,   // PLV3 访存越界（ADEM）
    output wire                         data_ex_tlbr,
    output wire                         data_ex_pil,
    output wire                         data_ex_pis,
    output wire                         data_ex_ppi,
    output wire                         data_ex_pme,

    output wire                         tlbsrch_found,
    output wire [$clog2(TLBNUM)-1:0]    tlbsrch_index,

    output wire [31:0]                  tlbrd_tlbidx,
    output wire [31:0]                  tlbrd_tlbehi,
    output wire [31:0]                  tlbrd_tlbelo0,
    output wire [31:0]                  tlbrd_tlbelo1,
    output wire [9:0]                   tlbrd_asid
);

localparam [5:0] PS_4KB = 6'd12;
localparam IDXW = $clog2(TLBNUM);

// 4-state safe: !da && pg / da && !pg must not become X if CSR bits are unknown (poisons inst_paddr).
wire pg_mode = (csr_crmd_da === 1'b0) && (csr_crmd_pg === 1'b1);
wire da_mode = (csr_crmd_da === 1'b1) && (csr_crmd_pg === 1'b0);

// Case equality: avoids inst_paddr/data_paddr going X when CSR/vaddr bits are unknown during sim bring-up.
wire inst_dmw0_hit = pg_mode && ((inst_vaddr[31:29] === csr_dmw0[31:29]) && (((csr_crmd_plv === 2'b00) && (csr_dmw0[0] === 1'b1)) || ((csr_crmd_plv === 2'b11) && (csr_dmw0[3] === 1'b1))));
wire inst_dmw1_hit = pg_mode && ((inst_vaddr[31:29] === csr_dmw1[31:29]) && (((csr_crmd_plv === 2'b00) && (csr_dmw1[0] === 1'b1)) || ((csr_crmd_plv === 2'b11) && (csr_dmw1[3] === 1'b1))));
wire data_dmw0_hit = pg_mode && ((data_vaddr[31:29] === csr_dmw0[31:29]) && (((csr_crmd_plv === 2'b00) && (csr_dmw0[0] === 1'b1)) || ((csr_crmd_plv === 2'b11) && (csr_dmw0[3] === 1'b1))));
wire data_dmw1_hit = pg_mode && ((data_vaddr[31:29] === csr_dmw1[31:29]) && (((csr_crmd_plv === 2'b00) && (csr_dmw1[0] === 1'b1)) || ((csr_crmd_plv === 2'b11) && (csr_dmw1[3] === 1'b1))));

// ------------------------------------------------------------
// 特权地址检查（映射模式 + PLV3 + va[31]=1 且未落可用 DMW 窗口）
// - 取指侧记 ADEF（与 PC 非对齐同码，Esubcode=0）、访存侧记 ADEM（Esubcode=1）；
// - 地址错优先：这两类地址本身非法，后续 TLB 查表异常一律屏蔽
//   （与 exception_Decoder 的优先级排布配套，避免报成 TLBR/PIF/PIL）。
// ------------------------------------------------------------
wire inst_plv_oob = pg_mode && (csr_crmd_plv === 2'b11) && (inst_vaddr[31] === 1'b1)
                 && !inst_dmw0_hit && !inst_dmw1_hit;
wire data_plv_oob = pg_mode && (csr_crmd_plv === 2'b11) && (data_vaddr[31] === 1'b1)
                 && !data_dmw0_hit && !data_dmw1_hit;

assign inst_ex_adef = inst_req && inst_plv_oob;
assign data_ex_adem = data_req && data_plv_oob;

// ------------------------------------------------------------
// 维护操作译码（commit 提交拍一拍脉冲）
// ------------------------------------------------------------
// tlbsrch/tlbrd 在本模块无需动作：srch 口每拍常备、r 口按 TLBIDX 常读，
// 结果由 csr handler 在提交拍采样（tlb_op 对应位仅作 lint 吸收）。
wire do_tlbsrch = tlb_op[`TLB_OP_TLBSRCH];
wire do_tlbrd   = tlb_op[`TLB_OP_TLBRD];
wire do_tlbwr   = tlb_op[`TLB_OP_TLBWR];
wire do_tlbfill = tlb_op[`TLB_OP_TLBFILL];
wire do_invtlb  = tlb_op[`TLB_OP_INVTLB_0] | tlb_op[`TLB_OP_INVTLB_1] | tlb_op[`TLB_OP_INVTLB_2]
                | tlb_op[`TLB_OP_INVTLB_3] | tlb_op[`TLB_OP_INVTLB_4] | tlb_op[`TLB_OP_INVTLB_5]
                | tlb_op[`TLB_OP_INVTLB_6];

// ------------------------------------------------------------
// L1 微 TLB fence：任何"主表内容/匹配条件可能变化"的时刻整表失效。
// - tlbwr/tlbfill/invtlb：主表内容变化（提交拍脉冲，伴随 FLUSH_REFETCH）；
// - ASID 变化：微表项是在旧 ASID 下匹配缓存的，必须作废（打拍比较，
//   csrwr ASID 提交同样伴随 FLUSH_REFETCH，fence 在新取指到来前生效）。
// tlbrd 只读不改，无需 fence。
// ------------------------------------------------------------
reg [9:0] asid_q;
always @(posedge clk) begin
    if (reset) asid_q <= 10'b0;
    else       asid_q <= csr_asid;
end
wire l1_fence = do_tlbwr | do_tlbfill | do_invtlb | (asid_q != csr_asid);

// ------------------------------------------------------------
// 主 TLB s0/s1 口连线（经 L1 微表转发）
// ------------------------------------------------------------
// I 侧微表 -> 主表 s0
wire [18:0] l1i_tlb_vppn;
wire        l1i_tlb_va_bit12;
wire        l1i_found;
wire [19:0] l1i_ppn;
wire [5:0]  l1i_ps;
wire [1:0]  l1i_mat;
wire        l1i_v, l1i_d;
wire [1:0]  l1i_plv;
// D 侧微表 -> 主表 s1
wire [18:0] l1d_tlb_vppn;
wire        l1d_tlb_va_bit12;
wire        l1d_found;
wire [19:0] l1d_ppn;
wire [5:0]  l1d_ps;
wire [1:0]  l1d_mat;
wire        l1d_v, l1d_d;
wire [1:0]  l1d_plv;

wire                        s0_found;
wire [IDXW-1:0]            s0_index;
wire [19:0]                s0_ppn;
wire [5:0]                 s0_ps;
wire [1:0]                 s0_plv;
wire [1:0]                 s0_mat;
wire                       s0_d;
wire                       s0_v;
wire                        s1_found;
wire [IDXW-1:0]            s1_index;
wire                        srch_found;
wire [IDXW-1:0]            srch_index;
wire [19:0]                s1_ppn;
wire [5:0]                 s1_ps;
wire [1:0]                 s1_plv;
wire [1:0]                 s1_mat;
wire                       s1_d;
wire                       s1_v;

wire                        r_e;
wire [18:0]                 r_vppn;
wire [5:0]                  r_ps;
wire [9:0]                  r_asid;
wire                        r_g;
wire [19:0]                 r_ppn0;
wire [1:0]                  r_plv0;
wire [1:0]                  r_mat0;
wire                        r_d0;
wire                        r_v0;
wire [19:0]                 r_ppn1;
wire [1:0]                  r_plv1;
wire [1:0]                  r_mat1;
wire                        r_d1;
wire                        r_v1;

// I 侧微表：连主表 s0 口（tlbsrch 已走专用口，s0 恒为真实取指查询）
l1_tlb #(.ENTRY_NUM(8)) u_l1_tlb_i (
    .clk            (clk),
    .reset          (reset),
    .fence_i        (l1_fence),
    .req_valid_i    (inst_req),
    .vaddr_i        (inst_vaddr),
    .found_o        (l1i_found),
    .ppn_o          (l1i_ppn),
    .ps_o           (l1i_ps),
    .mat_o          (l1i_mat),
    .v_o            (l1i_v),
    .d_o            (l1i_d),
    .plv_o          (l1i_plv),
    .tlb_vppn_o     (l1i_tlb_vppn),
    .tlb_va_bit12_o (l1i_tlb_va_bit12),
    .tlb_found_i    (s0_found),
    .tlb_ppn_i      (s0_ppn),
    .tlb_ps_i       (s0_ps),
    .tlb_mat_i      (s0_mat),
    .tlb_v_i        (s0_v),
    .tlb_d_i        (s0_d),
    .tlb_plv_i      (s0_plv)
);

// D 侧微表：连主表 s1 口
l1_tlb #(.ENTRY_NUM(8)) u_l1_tlb_d (
    .clk            (clk),
    .reset          (reset),
    .fence_i        (l1_fence),
    .req_valid_i    (data_req),
    .vaddr_i        (data_vaddr),
    .found_o        (l1d_found),
    .ppn_o          (l1d_ppn),
    .ps_o           (l1d_ps),
    .mat_o          (l1d_mat),
    .v_o            (l1d_v),
    .d_o            (l1d_d),
    .plv_o          (l1d_plv),
    .tlb_vppn_o     (l1d_tlb_vppn),
    .tlb_va_bit12_o (l1d_tlb_va_bit12),
    .tlb_found_i    (s1_found),
    .tlb_ppn_i      (s1_ppn),
    .tlb_ps_i       (s1_ps),
    .tlb_mat_i      (s1_mat),
    .tlb_v_i        (s1_v),
    .tlb_d_i        (s1_d),
    .tlb_plv_i      (s1_plv)
);

// TLBR 写回时强制写入有效位；否则沿用 CSR_TLBIDX.E。
wire w_e = (csr_estat_ecode == `TLBR_ECODE) ? 1'b1 : ~csr_tlbidx[31];
wire [5:0] w_ps = csr_tlbidx[29:24];
wire [IDXW-1:0] w_index = do_tlbfill ? csr_rand_index[IDXW-1:0] : csr_tlbidx[IDXW-1:0];

wire [19:0] w_ppn0 = csr_tlbelo0[27:8];
wire [1:0]  w_plv0 = csr_tlbelo0[3:2];
wire [1:0]  w_mat0 = csr_tlbelo0[5:4];
wire        w_d0   = csr_tlbelo0[1];
wire        w_v0   = csr_tlbelo0[0];
wire [19:0] w_ppn1 = csr_tlbelo1[27:8];
wire [1:0]  w_plv1 = csr_tlbelo1[3:2];
wire [1:0]  w_mat1 = csr_tlbelo1[5:4];
wire        w_d1   = csr_tlbelo1[1];
wire        w_v1   = csr_tlbelo1[0];
wire        w_g    = csr_tlbelo0[6] & csr_tlbelo1[6];

// 主 TLB：s0/s1 恒为取指/访存翻译查询；tlbsrch 走独立 srch 口（见头注）
tlb #(.TLBNUM(TLBNUM)) u_tlb (
    .clk          (clk),
    .reset        (reset),
    .s0_vppn      (l1i_tlb_vppn),
    .s0_va_bit12  (l1i_tlb_va_bit12),
    .s0_asid      (csr_asid),
    .s0_found     (s0_found),
    .s0_index     (s0_index),
    .s0_ppn       (s0_ppn),
    .s0_ps        (s0_ps),
    .s0_plv       (s0_plv),
    .s0_mat       (s0_mat),
    .s0_d         (s0_d),
    .s0_v         (s0_v),
    .s1_vppn      (l1d_tlb_vppn),
    .s1_va_bit12  (l1d_tlb_va_bit12),
    .s1_asid      (csr_asid),
    .s1_found     (s1_found),
    .s1_index     (s1_index),
    .s1_ppn       (s1_ppn),
    .s1_ps        (s1_ps),
    .s1_plv       (s1_plv),
    .s1_mat       (s1_mat),
    .s1_d         (s1_d),
    .s1_v         (s1_v),
    .srch_vppn    (csr_tlbehi[31:13]),
    .srch_asid    (csr_asid),
    .srch_found   (srch_found),
    .srch_index   (srch_index),
    .invtlb_valid (do_invtlb),
    .invtlb_op    (invtlb_op),
    .invtlb_asid  (invtlb_asid),
    .invtlb_vpn   (invtlb_vpn),
    .we           (do_tlbwr | do_tlbfill),
    .w_index      (w_index),
    .w_e          (w_e),
    .w_vppn       (csr_tlbehi[31:13]),
    .w_ps         (w_ps),
    .w_asid       (csr_asid),
    .w_g          (w_g),
    .w_ppn0       (w_ppn0),
    .w_plv0       (w_plv0),
    .w_mat0       (w_mat0),
    .w_d0         (w_d0),
    .w_v0         (w_v0),
    .w_ppn1       (w_ppn1),
    .w_plv1       (w_plv1),
    .w_mat1       (w_mat1),
    .w_d1         (w_d1),
    .w_v1         (w_v1),
    .r_index      (csr_tlbidx[IDXW-1:0]),
    .r_e          (r_e),
    .r_vppn       (r_vppn),
    .r_ps         (r_ps),
    .r_asid       (r_asid),
    .r_g          (r_g),
    .r_ppn0       (r_ppn0),
    .r_plv0       (r_plv0),
    .r_mat0       (r_mat0),
    .r_d0         (r_d0),
    .r_v0         (r_v0),
    .r_ppn1       (r_ppn1),
    .r_plv1       (r_plv1),
    .r_mat1       (r_mat1),
    .r_d1         (r_d1),
    .r_v1         (r_v1)
);

// ------------------------------------------------------------
// 翻译结果拼接与异常生成
// 统一以 L1 微表输出（l1i_*/l1d_*）为准——微表命中时用缓存副本、
// miss 时即主表结果透传，保证 paddr 与异常判定同源。
// ------------------------------------------------------------
wire [31:0] inst_tlb_paddr = (l1i_ps === PS_4KB) ? {l1i_ppn, inst_vaddr[11:0]} : {l1i_ppn[19:10], inst_vaddr[21:0]};
wire [31:0] data_tlb_paddr = (l1d_ps === PS_4KB) ? {l1d_ppn, data_vaddr[11:0]} : {l1d_ppn[19:10], data_vaddr[21:0]};

// 特权越界（ADEF/ADEM）时地址本身非法，屏蔽 TLB 查表异常
wire inst_need_tlb = pg_mode && !inst_dmw0_hit && !inst_dmw1_hit && !inst_plv_oob;
wire data_need_tlb = pg_mode && !data_dmw0_hit && !data_dmw1_hit && !data_plv_oob;

// TLB 查询结果和异常在同一拍组合给出，供后级直接使用。
assign inst_ex_tlbr = inst_req && inst_need_tlb && !l1i_found;
assign inst_ex_pif  = inst_req && inst_need_tlb && l1i_found && !l1i_v;
assign inst_ex_ppi  = inst_req && inst_need_tlb && l1i_found && l1i_v && (csr_crmd_plv > l1i_plv);

assign data_ex_tlbr = data_req && data_need_tlb && !l1d_found;
assign data_ex_pil  = data_req && !data_is_store && data_need_tlb && l1d_found && !l1d_v;
assign data_ex_pis  = data_req && data_is_store  && data_need_tlb && l1d_found && !l1d_v;
assign data_ex_ppi  = data_req && data_need_tlb && l1d_found && l1d_v && (csr_crmd_plv > l1d_plv);
assign data_ex_pme  = data_req && data_is_store && data_need_tlb && l1d_found && l1d_v && (csr_crmd_plv <= l1d_plv) && !l1d_d;

assign inst_paddr = (da_mode === 1'b1) ? inst_vaddr :
                    (inst_dmw0_hit === 1'b1) ? {csr_dmw0[27:25], inst_vaddr[28:0]} :
                    (inst_dmw1_hit === 1'b1) ? {csr_dmw1[27:25], inst_vaddr[28:0]} :
                    inst_tlb_paddr;

assign data_paddr = (da_mode === 1'b1) ? data_vaddr :
                    (data_dmw0_hit === 1'b1) ? {csr_dmw0[27:25], data_vaddr[28:0]} :
                    (data_dmw1_hit === 1'b1) ? {csr_dmw1[27:25], data_vaddr[28:0]} :
                    data_tlb_paddr;

assign inst_mat = (da_mode === 1'b1) ? csr_crmd_datf :
                  (inst_dmw0_hit === 1'b1) ? csr_dmw0[5:4] :
                  (inst_dmw1_hit === 1'b1) ? csr_dmw1[5:4] :
                  l1i_mat;

assign data_mat = (da_mode === 1'b1) ? csr_crmd_datm :
                  (data_dmw0_hit === 1'b1) ? csr_dmw0[5:4] :
                  (data_dmw1_hit === 1'b1) ? csr_dmw1[5:4] :
                  l1d_mat;

// ------------------------------------------------------------
// tlbsrch/tlbrd 回读（CSR 提交路径在提交同拍采样）
// tlbsrch 结果取主表专用 srch 口输出（微表不参与——found/index 是
// 体系结构语义；srch 口输入恒为 CSR.TLBEHI/ASID，每拍常备）。
// ------------------------------------------------------------
assign tlbsrch_found = srch_found;
assign tlbsrch_index = srch_index;

// 与参考实现对齐：TLBRD 有效时输出 NE=0、PS=r_ps；无效时 NE=1 且 PS=0（勿在 !r_e 时仍用 RAM 的 r_ps）。
// 使用 ===/!==：r_e 为 X 时勿用 ? : 把 X 灌进 TLBRD 回读（曾导致 csr_tlbehi / WB wdata 出现 X）。
assign tlbrd_tlbidx  = {(r_e !== 1'b1), 1'b0, (r_e === 1'b1) ? r_ps : 6'b0, 24'b0};
assign tlbrd_tlbehi  = (r_e === 1'b1) ? {r_vppn, 13'b0} : 32'b0;
assign tlbrd_tlbelo0 = (r_e === 1'b1) ? {4'b0, r_ppn0, 1'b0, r_g, r_mat0, r_plv0, r_d0, r_v0} : 32'b0;
assign tlbrd_tlbelo1 = (r_e === 1'b1) ? {4'b0, r_ppn1, 1'b0, r_g, r_mat1, r_plv1, r_d1, r_v1} : 32'b0;
assign tlbrd_asid    = (r_e === 1'b1) ? r_asid : 10'b0;

// lint 吸收（s0_index/s1_index 仅主表内部使用；tlbsrch/tlbrd 由 CSR 侧采样）
wire tlbm_lint_sink = (|s0_index) | (|s1_index) | do_tlbsrch | do_tlbrd;

endmodule
