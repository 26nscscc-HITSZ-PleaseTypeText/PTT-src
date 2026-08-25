`include "mycpu.h"

// ============================================================
// csr_exception_commit_handler 模块（CSR 寄存器组 + 异常/提交处理，时序+组合）
// ------------------------------------------------------------
// 功能：
// 1) 时序维护全部 CSR（CRMD/PRMD/ESTAT/ERA/EENTRY/SAVE0~3/ECFG/BADV/TID/
//    TCFG/TVAL/TICLR/TLB 类/DMW/LLBCTL 等）
// 2) 组合用 exception_Decoder 把异常 valid 编码成 Ecode/Esubcode
// 3) commit.v 在提交点驱动 wb_valid/wb_pc/wb_ex、各异常位、TLB/LLSC/CSR 写口；
// 4) 输出 csr_next_pc：异常时选择 EENTRY/TLBRENTRY，ERTN 时选择 ERA，
//    由 commit 根据提交事件发起对应类型的全局冲刷
// 5) csr_rnum/csr_rvalue 由 fu_mdu 使用，在执行级读取 CSR 旧值
//
// 接口契约：
// 1. csr_we：commit 在 csrwr/csrxchg 单提交拍给出一拍脉冲；csrxchg 新值
//    已在 fu_mdu 按 (old & ~mask) | (wval & mask) 合成后随 ROB 带到提交级，
//    到本模块时 csr_wvalue=最终值、csr_wmask=全 1，按普通掩码写正确。
// 2. has_int：输出给 commit 做"中断附着"（附着在下一条将提交的指令上，
//    不再附着 ID 级）——ESTAT.IS 与 ECFG.LIE 逐位与后再看 CRMD.IE。
// 3. 冲刷由 commit -> ctrl 统一广播，本模块只输出 csr_next_pc。
// 4. IPE/ADEM 写入路径：decoder 检测 IPE、tlb_manager/mmu 检测 ADEM，
//    commit 打包驱动 IPE_valid/ADEM_valid 到本模块——exception_Decoder
//    的链式优先级含两者，ESTAT.Ecode/Esubcode 与 BADV（wb_ex_addr_err
//    含 ADEM_ECODE）写入路径均无遗漏；IPE 无地址语义，不写 BADV，正确。
// ============================================================
module csr_exception_commit_handler (
    input  wire        clk,
    input  wire        reset,
    input  wire [13:0] csr_rnum,   // CSR 读号（14 位；未实现号读 0）


    // ---------------- CSR 访问（来自 csr 指令，commit 提交拍真正执行） ----------------------------
    input  wire [13:0] csr_num,    // CSR 写号（14 位；未实现号写忽略）
    input  wire        csr_we,     // CSR寄存器写使能
    input  wire [31:0] csr_wmask,  // CSR寄存器写掩码
    input  wire [31:0] csr_wvalue, // CSR寄存器写值


    // ---------------- 来自 TLB 的相关输入  ----------------------------
    input  wire                   tlbsrch_found,
    input  wire [4:0]             tlbsrch_index,
    input  wire                   tlbrd_ne,
    input  wire [5:0]             tlbrd_ps,
    input  wire [31:0]            tlbrd_tlbehi,
    input  wire [31:0]            tlbrd_tlbelo0,
    input  wire [31:0]            tlbrd_tlbelo1,
    input  wire [9:0]             tlbrd_asid,


    // ---------------- 外部中断输入 ----------------------------
    // hw_int_in = hardware interrupt input
    input  wire [ 7:0] hw_int_in,  // 8 位硬件中断；顶层当前接 0
    input  wire        ipi_int_in, // 核间中断


    // ---------------- 来自 WB 的提交信息 ----------------
    input  wire        wb_valid,
    input  wire [31:0] wb_pc,
    input  wire        wb_is_ertn,    // ERTN指令要冲刷流水线
    input  wire [31:0] wb_vaddr,      // 用给BADV
    input  wire        wb_ex,         // commit 各异常 valid 的合并触发信号
    input  wire        wb_tlbsrch,
    input  wire        wb_tlbrd,

    // 注意：有优先级，INT 最高，其余按 exception_Decoder 的链式次序编码
    input  wire        INT_valid,     // 中断是否触发，高电平即为有中断异常
    input  wire        ADEF_valid,    // 取指地址错位异常，特指pc
    input  wire        ADEM_valid,    // 访存物理地址越界异常（ESUBCODE=1，与 ADEF 区分）
    input  wire        IPE_valid,     // 特权指令在用户态触发（ECODE=0xe）
    input  wire [`TLB_EX_NUM-1:0] TLB_EX_valid,
    input  wire        ALE_valid,     // 地址非对齐异常，特指MEM的访存
    input  wire        SYS_valid,     // 系统调用异常，与指令syscall相关
    input  wire        BRK_valid,     // 断点异常，与指令break相关
    input  wire        INE_valid,     // 指令不存在异常，特指ID


    // ---------------- LL/SC 接口（提交时维护 LLBIT） ----------------
    input  wire        ll_set_in,         // 提交 ll.w 时置位 LLBIT
    input  wire        sc_set_in,         // 在 WB 提交 sc.w 时为 1（无论成败均清 LLBIT）

    // ---------------- 输出：冲刷目标、中断、csr读返回与域输出 ----------------
    output wire [31:0] csr_next_pc,     // 异常入口或 ERTN 返回地址
    output wire        has_int,         // 送往 commit 的中断有效信号
    output wire [31:0] csr_rvalue,      // CSR寄存器读返回值
    output wire [31:0] csr_tid_out,     // csr的tid值，用于RDCNTID指令读取计时器ID号
    output wire        csr_crmd_da_out,
    output wire        csr_crmd_pg_out,
    output wire [1:0]  csr_crmd_plv_out,
    output wire [9:0]  csr_asid_out,
    output wire        csr_tlbidx_ne_out,
    output wire [5:0]  csr_tlbidx_ps_out,
    output wire [4:0]  csr_tlbidx_index_out,
    output wire [18:0] csr_tlbehi_vppn_out,
    output wire [19:0] csr_tlbelo0_ppn_out,
    output wire [1:0]  csr_tlbelo0_plv_out,
    output wire [1:0]  csr_tlbelo0_mat_out,
    output wire        csr_tlbelo0_d_out,
    output wire        csr_tlbelo0_v_out,
    output wire        csr_tlbelo0_g_out,
    output wire [19:0] csr_tlbelo1_ppn_out,
    output wire [1:0]  csr_tlbelo1_plv_out,
    output wire [1:0]  csr_tlbelo1_mat_out,
    output wire        csr_tlbelo1_d_out,
    output wire        csr_tlbelo1_v_out,
    output wire        csr_tlbelo1_g_out,
    output wire [2:0]  csr_dmw0_vseg_out,
    output wire [2:0]  csr_dmw0_pseg_out,
    output wire [1:0]  csr_dmw0_mat_out,
    output wire        csr_dmw0_plv3_out,
    output wire        csr_dmw0_plv0_out,
    output wire [2:0]  csr_dmw1_vseg_out,
    output wire [2:0]  csr_dmw1_pseg_out,
    output wire [1:0]  csr_dmw1_mat_out,
    output wire        csr_dmw1_plv3_out,
    output wire        csr_dmw1_plv0_out,
    output wire [4:0]  csr_rand_index_out,
    output wire [1:0]  csr_crmd_datf_out,
    output wire [1:0]  csr_crmd_datm_out,
    output wire [7:0]  csr_estat_ecode_out,
    // LL/SC 有效位供提交逻辑判断 sc.w 是否成功。
    output wire        csr_llbit_out,
    output wire [63:0] diff_timer_64

`ifdef DIFFTEST_EN
    ,
    output wire [31:0] diff_csr_crmd,
    output wire [31:0] diff_csr_prmd,
    output wire [31:0] diff_csr_euen,
    output wire [31:0] diff_csr_ecfg,
    output wire [31:0] diff_csr_estat,
    output wire [31:0] diff_csr_era,
    output wire [31:0] diff_csr_badv,
    output wire [31:0] diff_csr_eentry,
    output wire [31:0] diff_csr_tlbidx,
    output wire [31:0] diff_csr_tlbehi,
    output wire [31:0] diff_csr_tlbelo0,
    output wire [31:0] diff_csr_tlbelo1,
    output wire [31:0] diff_csr_asid,
    output wire [31:0] diff_csr_pgdl,
    output wire [31:0] diff_csr_pgdh,
    output wire [31:0] diff_csr_save0,
    output wire [31:0] diff_csr_save1,
    output wire [31:0] diff_csr_save2,
    output wire [31:0] diff_csr_save3,
    output wire [31:0] diff_csr_tid,
    output wire [31:0] diff_csr_tcfg,
    output wire [31:0] diff_csr_tval,
    output wire [31:0] diff_csr_ticlr,
    output wire [31:0] diff_csr_llbctl,
    output wire [31:0] diff_csr_tlbrentry,
    output wire [31:0] diff_csr_dmw0,
    output wire [31:0] diff_csr_dmw1,
    output wire [7:0]  diff_wb_ecode
`endif

);

    // 默认异常入口为 0x1c000000，避免在软件初始化前中断/异常重定向到 0x00000000。
    localparam [25:0] CSR_EENTRY_RESET_VA = 26'h700000;

    // 异常译码出ecode、esubcode               
    wire [7:0] Ecode;   // 异常码
    wire Esubcode;      // 异常子码                  
    exception_Decoder u_exception_Decoder (
        .INT_valid(INT_valid),
        .ADEF_valid(ADEF_valid),
        .ADEM_valid(ADEM_valid),
        .IPE_valid(IPE_valid),
        .TLB_EX_valid(TLB_EX_valid),
        .ALE_valid(ALE_valid),
        .SYS_valid(SYS_valid),
        .BRK_valid(BRK_valid),
        .INE_valid(INE_valid),
        .Ecode(Ecode),
        .Esubcode(Esubcode)
    );

    // Direct exception-class selects.  Ecode is still written to ESTAT, but
    // decoding Ecode again for CSR enables made the ROB exception RAM pass
    // through the full priority encoder and six equality comparators.
    // These terms are the same priority order as exception_Decoder.
    wire higher_than_front_tlb = INT_valid || ADEF_valid || ALE_valid;
    wire front_tlb_any = TLB_EX_valid[`TLB_EX_TLBR]
                      || TLB_EX_valid[`TLB_EX_PIF]
                      || TLB_EX_valid[`TLB_EX_PPI];
    wire data_tlb_any = TLB_EX_valid[`TLB_EX_PIL]
                     || TLB_EX_valid[`TLB_EX_PIS]
                     || TLB_EX_valid[`TLB_EX_PME];
    wire mid_non_addr_excp = INE_valid || IPE_valid || SYS_valid || BRK_valid;
    wire selected_tlbr = !higher_than_front_tlb
                       && TLB_EX_valid[`TLB_EX_TLBR];
    wire selected_pif = !higher_than_front_tlb
                      && !TLB_EX_valid[`TLB_EX_TLBR]
                      && TLB_EX_valid[`TLB_EX_PIF];
    wire selected_tlb_fault = !higher_than_front_tlb
                            && (front_tlb_any
                                || (!mid_non_addr_excp && !ADEM_valid
                                    && data_tlb_any));
    wire selected_addr_fault = !INT_valid
                             && (ADEF_valid || ALE_valid || front_tlb_any
                                 || (!mid_non_addr_excp
                                     && (ADEM_valid || data_tlb_any)));

    reg [7:0] csr_estat_ecode; // 提前定义ESTAT 的 ECODE 域

    wire wb_csr_ex = (wb_valid === 1'b1) && ((wb_ex === 1'b1) || (INT_valid === 1'b1));

    // CRMD 的 PLV 域
    reg [1:0] csr_crmd_plv;
    reg [1:0] csr_prmd_pplv;
    always @(posedge clk) begin
        if (reset) begin
            csr_crmd_plv <= 2'b0;
        end
        else if (wb_csr_ex) begin
            csr_crmd_plv <= 2'b0;
        end
        else if (wb_valid && wb_is_ertn) begin
            csr_crmd_plv <= csr_prmd_pplv;
        end
        else if (csr_we && csr_num == `CSR_CRMD) begin
            csr_crmd_plv <= csr_wmask[`CSR_CRMD_PLV] & csr_wvalue[`CSR_CRMD_PLV] 
                            | ~csr_wmask[`CSR_CRMD_PLV] & csr_crmd_plv;
        // `CSR_CRMD_PLV 是 CRMD 在 32 位 CSR 数据里的位域范围，看手册可知CRMD PLV域为1:0
        // `define CSR_CRMD_PLV 1:0
        end
    end


    // CRMD 的 IE 域
    reg csr_crmd_ie;
    reg csr_prmd_pie;
    always @(posedge clk) begin
        if (reset) begin
            csr_crmd_ie <= 1'b0;
        end
        else if (wb_csr_ex) begin
            csr_crmd_ie <= 1'b0;
        end
        else if (wb_valid && wb_is_ertn) begin
            csr_crmd_ie <= csr_prmd_pie;
        end
        else if (csr_we && csr_num == `CSR_CRMD) begin
            csr_crmd_ie <= csr_wmask[`CSR_CRMD_IE] & csr_wvalue[`CSR_CRMD_IE] 
                            | ~csr_wmask[`CSR_CRMD_IE] & csr_crmd_ie;
        end
    end


    // CRMD 的 DA、PG、DATF、DATM 域
    // 复位进直接地址翻译（DA=1,PG=0）；TLBR 例外硬件切 DA、ertn 自 TLBR
    // 返回时切回 PG（手册 6.2.1 TLB 重填例外的翻译模式切换）
    reg csr_crmd_da;
    reg csr_crmd_pg;
    reg [1:0] csr_crmd_datf;
    reg [1:0] csr_crmd_datm;
    always @(posedge clk) begin
        if (reset) begin
            csr_crmd_da   <= 1'b1;
            csr_crmd_pg   <= 1'b0;
            csr_crmd_datf <= 2'b00;
            csr_crmd_datm <= 2'b00;
        end
        else if (wb_csr_ex && selected_tlbr) begin
            csr_crmd_da <= 1'b1;
            csr_crmd_pg <= 1'b0;
        end
        else if (wb_valid && wb_is_ertn && (csr_estat_ecode == `TLBR_ECODE)) begin
            csr_crmd_da <= 1'b0;
            csr_crmd_pg <= 1'b1;
        end
        else if (csr_we && csr_num == `CSR_CRMD) begin
            csr_crmd_da <= csr_wmask[`CSR_CRMD_DA] & csr_wvalue[`CSR_CRMD_DA]
                        | ~csr_wmask[`CSR_CRMD_DA] & csr_crmd_da;
            csr_crmd_pg <= csr_wmask[`CSR_CRMD_PG] & csr_wvalue[`CSR_CRMD_PG]
                        | ~csr_wmask[`CSR_CRMD_PG] & csr_crmd_pg;
            csr_crmd_datf <= csr_wmask[`CSR_CRMD_DATF] & csr_wvalue[`CSR_CRMD_DATF]
                          | ~csr_wmask[`CSR_CRMD_DATF] & csr_crmd_datf;
            csr_crmd_datm <= csr_wmask[`CSR_CRMD_DATM] & csr_wvalue[`CSR_CRMD_DATM]
                          | ~csr_wmask[`CSR_CRMD_DATM] & csr_crmd_datm;
        end
    end
    

    // PRMD 的 PPLV、PIE 域
    always @(posedge clk) begin
        if (reset) begin
            csr_prmd_pplv <= 2'b0;
            csr_prmd_pie  <= 1'b0;
        end
        else if (wb_csr_ex) begin
            csr_prmd_pplv <= csr_crmd_plv;
            csr_prmd_pie <= csr_crmd_ie;
        end
        else if (csr_we && csr_num == `CSR_PRMD) begin
            csr_prmd_pplv <= csr_wmask[`CSR_PRMD_PPLV] & csr_wvalue[`CSR_PRMD_PPLV] 
                            | ~csr_wmask[`CSR_PRMD_PPLV] & csr_prmd_pplv;
            csr_prmd_pie <= csr_wmask[`CSR_PRMD_PIE] & csr_wvalue[`CSR_PRMD_PIE] 
                            | ~csr_wmask[`CSR_PRMD_PIE] & csr_prmd_pie;
        end
    end


    // ECFG 的 LIE 域
    reg [12:0] csr_ecfg_lie;
    always @(posedge clk) begin
        if (reset) begin
            csr_ecfg_lie <= 13'b0;
        end
        else if (csr_we && csr_num == `CSR_ECFG) begin
            csr_ecfg_lie <= csr_wmask[`CSR_ECFG_LIE] & 13'h1bff & csr_wvalue[`CSR_ECFG_LIE] 
                            | ~csr_wmask[`CSR_ECFG_LIE] & 13'h1bff & csr_ecfg_lie;
        end
    end


    // ESTAT 的 IS 域（timer_cnt / timer_en 在本块使用，声明需前置）
    // 定时器语义：
    //   - 独立 timer_en：oneshot 减到 0 时 timer_en<=0；periodic 保持 1
    //   - TI：timer_en && TVAL==0 时置位
    //   - TVAL==0 当拍重装 InitVal（periodic）或 0xFFFFFFFF（oneshot）
    // TI 挂起期间暂停倒计时；TICLR 后从已重装值开始下一周期，避免异常处理
    // 尚未返回时定时器再次到期。
    reg [12:0] csr_estat_is;
    reg [31:0] timer_cnt;
    reg        timer_en;
    reg [63:0] timer_64;
`ifdef SOC_USE
    reg        csr_tcfg_en;
    reg        csr_tcfg_periodic;
    reg [29:0] csr_tcfg_initval;
`else
`endif
    wire timer_ti_set = timer_en && (timer_cnt == 32'b0);
    // csrwr TICLR：commit 在 PRIV_CSR_WR 提交拍拉 csr_we 一拍
    wire ticlr_we = csr_we && (csr_num == `CSR_TICLR)
                 && csr_wmask[`CSR_TICLR_CLR] && csr_wvalue[`CSR_TICLR_CLR];
    wire tcfg_we  = csr_we && (csr_num == `CSR_TCFG);
    always @(posedge clk) begin
        if (reset) begin
            // 必须整域复位，否则 IS[11] 等在首拍为 X → has_int 为 X → ID 译码条件失效、写回数据 X
            csr_estat_is <= 13'b0;
            timer_64     <= 64'b0;
            timer_en     <= 1'b0;
        end else begin
            timer_64 <= timer_64 + 64'd1;
            if (csr_we && csr_num == `CSR_ESTAT) begin
                csr_estat_is[1:0] <= csr_wmask[`CSR_ESTAT_IS10] & csr_wvalue[`CSR_ESTAT_IS10] 
                                | ~csr_wmask[`CSR_ESTAT_IS10] & csr_estat_is[1:0];
            end
            csr_estat_is[9:2] <= hw_int_in[7:0];
            csr_estat_is[10] <= 1'b0;
            if (ticlr_we) begin
                csr_estat_is[11] <= 1'b0;
            end else if (tcfg_we) begin
                // 写 TCFG 时按写入后的 En 刷新 timer_en。
                timer_en <= csr_wmask[`CSR_TCFG_EN] & csr_wvalue[`CSR_TCFG_EN]
                         | ~csr_wmask[`CSR_TCFG_EN] & csr_tcfg_en;
            end else if (timer_ti_set) begin
                csr_estat_is[11] <= 1'b1;
                timer_en         <= csr_tcfg_periodic;
            end
            csr_estat_is[12] <= ipi_int_in;
        end
    end


    // ESTAT 的 ECODE、ESUBCODE 域
    reg csr_estat_esubcode;
    always @(posedge clk) begin
        if (reset) begin
            csr_estat_ecode <= 8'b0;
            csr_estat_esubcode <= 1'b0;
        end
        else if (wb_csr_ex) begin
            csr_estat_ecode <= {2'b0, Ecode[5:0]};
            csr_estat_esubcode <= Esubcode;
        end
    end


    // ERA 的 PC 域
    reg [31:0] csr_era_pc;
    wire wb_adef_valid = (ADEF_valid === 1'b1);
    wire wb_pif_valid  = selected_pif;
    // PIF：手册/测试套件期望 ERA 记录 fault VA（与 BADV 一致）；其余 TLB/常规例外为 faulting PC。
    // ADEF/PIF 的精确异常地址来自 IF/TLB 传下来的虚地址；流水提交 PC 可能已经受取指节奏影响。
    // 其余常规异常记录提交槽自己的 PC。
    wire [31:0] era_fault_pc = (wb_adef_valid || wb_pif_valid) ? wb_vaddr : wb_pc;
    always @(posedge clk) begin
        if (reset) begin
            csr_era_pc <= 32'b0;
        end
        else if (wb_csr_ex) begin
            csr_era_pc <= era_fault_pc;
        end
        else if (csr_we && csr_num == `CSR_ERA) begin
            csr_era_pc <= csr_wmask[`CSR_ERA_PC] & csr_wvalue[`CSR_ERA_PC] 
                            | ~csr_wmask[`CSR_ERA_PC] & csr_era_pc;
        end
    end


    // BADV 的 VADDR 域（与 TLBEHI 硬件更新一起在下方 TLB CSR always 中维护，避免多 always 对 wb_vaddr 的仿真次序问题）
    reg [31:0] csr_badv_vaddr;
    wire wb_ex_addr_err = selected_addr_fault;

    // EENTRY 的 VA 域
    reg [25:0] csr_eentry_va;
    always @(posedge clk) begin
        if (reset) begin
            csr_eentry_va <= CSR_EENTRY_RESET_VA;
        end
        else if (csr_we && csr_num == `CSR_EENTRY) begin
            csr_eentry_va <= csr_wmask[`CSR_EENTRY_VA] & csr_wvalue[`CSR_EENTRY_VA] 
                            | ~csr_wmask[`CSR_EENTRY_VA] & csr_eentry_va;
        end
    end


    // TLB 的 TLBIDX、TLBEHI、TLBELO0、TLBELO1、ASID、TLBRENTRY、DMW0、DMW1 域
    reg [31:0] csr_tlbidx;
    reg [31:0] csr_tlbehi;
    reg [31:0] csr_tlbelo0;
    reg [31:0] csr_tlbelo1;
    reg [31:0] csr_asid;
    reg [31:0] csr_tlbrentry;
    reg [31:0] csr_dmw0;
    reg [31:0] csr_dmw1;
    reg [31:0] csr_pgdl;
    reg [31:0] csr_pgdh;

    always @(posedge clk) begin
        if (reset) begin
            csr_badv_vaddr <= 32'h0000_0000;
            csr_tlbidx   <= 32'h0000_0000;
            csr_tlbehi   <= 32'h0000_0000;
            csr_tlbelo0  <= 32'h0000_0000;
            csr_tlbelo1  <= 32'h0000_0000;
            // ASID CSR：低 10 位为 ASID；高位为只读/保留域，复位读回 0x000a0000。
            csr_asid[31:10] <= 22'h280;
            csr_asid[9:0]   <= 10'h0;
            csr_tlbrentry<= 32'h0000_0000;
            csr_dmw0     <= 32'h0000_0000;
            csr_dmw1     <= 32'h0000_0000;
            csr_pgdl     <= 32'h0000_0000;
            csr_pgdh     <= 32'h0000_0000;
        end else begin
            // BADV 与 TLB 类例外的 TLBEHI 同拍、同源 wb_vaddr；TLBRD 的 r_e 硬化见 tlb_manager.v。
            if (wb_csr_ex && wb_ex_addr_err) begin
                csr_badv_vaddr <= wb_vaddr;
                if (selected_tlb_fault)
                    csr_tlbehi <= {wb_vaddr[31:13], 13'b0};
            end else if (csr_we && csr_num == `CSR_BADV) begin
                csr_badv_vaddr <= (csr_wmask & csr_wvalue) | (~csr_wmask & csr_badv_vaddr);
            end

            if (csr_we && csr_num == `CSR_TLBIDX)
                csr_tlbidx <= ((csr_wmask & csr_wvalue) | (~csr_wmask & csr_tlbidx)) & 32'hbf00_001f;
            if (csr_we && csr_num == `CSR_TLBEHI)
                // TLBEHI 仅 VPPN[31:13] 可写，低 13 位读作 0；否则会在 csrwr/tlbwr 序列中把页内偏移误带入。
                csr_tlbehi <= ((csr_wmask & csr_wvalue) | (~csr_wmask & csr_tlbehi)) & 32'hffff_e000;
            if (csr_we && csr_num == `CSR_TLBELO0)
                // TLBELO0 保留 [31:28] 与 [7] 读 0、写忽略，防止软件构造值的
                // 保留位进入 CSR 状态。
                csr_tlbelo0 <= ((csr_wmask & csr_wvalue) | (~csr_wmask & csr_tlbelo0)) & 32'h0fffff7f;
            if (csr_we && csr_num == `CSR_TLBELO1)
                csr_tlbelo1 <= ((csr_wmask & csr_wvalue) | (~csr_wmask & csr_tlbelo1)) & 32'h0fffff7f;
            if (csr_we && csr_num == `CSR_TLBRENTRY)
                csr_tlbrentry <= ((csr_wmask & csr_wvalue) | (~csr_wmask & csr_tlbrentry)) & 32'hffff_ffc0;
            if (csr_we && csr_num == `CSR_DMW0)
                csr_dmw0 <= ((csr_wmask & csr_wvalue) | (~csr_wmask & csr_dmw0)) & 32'hee00_0039;
            if (csr_we && csr_num == `CSR_DMW1)
                csr_dmw1 <= ((csr_wmask & csr_wvalue) | (~csr_wmask & csr_dmw1)) & 32'hee00_0039;
            if (csr_we && csr_num == `CSR_PGDL)
                csr_pgdl <= (((csr_wmask & csr_wvalue) | (~csr_wmask & csr_pgdl)) & 32'hffff_f000);
            if (csr_we && csr_num == `CSR_PGDH)
                csr_pgdh <= (((csr_wmask & csr_wvalue) | (~csr_wmask & csr_pgdh)) & 32'hffff_f000);

            // 这里是 WB 提交点：tlbsrch/tlbrd 结果写入 CSR（4.2.3.2 + CSR.ASID 7.5.4：TLBRD 将表项 ASID 写入 CSR.ASID；无效项时 tlbrd_asid=0）。
            if (wb_valid && wb_tlbsrch) begin
                if (tlbsrch_found) begin
                    csr_tlbidx[4:0] <= tlbsrch_index;
                    csr_tlbidx[31]  <= 1'b0;
                end else begin
                    csr_tlbidx[31]  <= 1'b1;
                end
            end

            // tlbrd 也是 WB 提交后立即把 tlb_manager 的读回结果写回 CSR 影子寄存器。
            // TLBRD 只更新 TLBIDX 的 NE/PS，不修改 INDEX 域。
            if (wb_valid && wb_tlbrd) begin
                csr_tlbidx[31]    <= tlbrd_ne;
                csr_tlbidx[29:24] <= tlbrd_ps;
                csr_tlbehi  <= tlbrd_tlbehi;
                csr_tlbelo0 <= tlbrd_tlbelo0;
                csr_tlbelo1 <= tlbrd_tlbelo1;
                // 无效项（NE=1）不回写 ASID，保留原值，供后续 CSRXCHG 使用。
                if (!tlbrd_ne)
                    csr_asid[9:0] <= tlbrd_asid;
                else
                    csr_asid[9:0] <= 10'h0;
            end
            // 同拍既有 TLBRD 又有 CSR 写 ASID 时（单指令不应发生），显式 CSR 写覆盖 TLBRD 对 ASID 的回写
            if (csr_we && csr_num == `CSR_ASID)
                csr_asid[9:0] <= (csr_wmask[9:0] & csr_wvalue[9:0])
                               | (~csr_wmask[9:0] & csr_asid[9:0]);
        end
    end


    // SAVE0~3 的 DATA 域
    reg [31:0] csr_save0_data;
    reg [31:0] csr_save1_data;
    reg [31:0] csr_save2_data;
    reg [31:0] csr_save3_data;
    always @(posedge clk) begin
        if (csr_we && csr_num == `CSR_SAVE0) begin
            csr_save0_data <= csr_wmask[`CSR_SAVE_DATA] & csr_wvalue[`CSR_SAVE_DATA] 
                            | ~csr_wmask[`CSR_SAVE_DATA] & csr_save0_data;
        end
        if (csr_we && csr_num == `CSR_SAVE1) begin
            csr_save1_data <= csr_wmask[`CSR_SAVE_DATA] & csr_wvalue[`CSR_SAVE_DATA] 
                            | ~csr_wmask[`CSR_SAVE_DATA] & csr_save1_data;
        end
        if (csr_we && csr_num == `CSR_SAVE2) begin
            csr_save2_data <= csr_wmask[`CSR_SAVE_DATA] & csr_wvalue[`CSR_SAVE_DATA] 
                            | ~csr_wmask[`CSR_SAVE_DATA] & csr_save2_data;
        end
        if (csr_we && csr_num == `CSR_SAVE3) begin
            csr_save3_data <= csr_wmask[`CSR_SAVE_DATA] & csr_wvalue[`CSR_SAVE_DATA] 
                            | ~csr_wmask[`CSR_SAVE_DATA] & csr_save3_data;
        end
    end


    // TID 的 TID 域
    reg [31:0] csr_tid_tid;
    always @(posedge clk) begin
        if (reset) begin
            csr_tid_tid <= 32'b0;
        end
        else if (csr_we && csr_num == `CSR_TID) begin
            csr_tid_tid <= csr_wmask[`CSR_TID_TID] & csr_wvalue[`CSR_TID_TID] 
                            | ~csr_wmask[`CSR_TID_TID] & csr_tid_tid;
        end
    end


    // TCFG 的 EN、PERIODIC、InitVal 域
`ifdef SOC_USE
`else
    reg csr_tcfg_en;
    reg csr_tcfg_periodic;
    reg [29:0] csr_tcfg_initval;
`endif
    always @(posedge clk) begin
        if (reset) begin
            csr_tcfg_en <= 1'b0;
            csr_tcfg_periodic <= 1'b0;
            csr_tcfg_initval <= 30'b0;
        end
        else if (csr_we && csr_num == `CSR_TCFG) begin
            csr_tcfg_en <= csr_wmask[`CSR_TCFG_EN] & csr_wvalue[`CSR_TCFG_EN] 
                            | ~csr_wmask[`CSR_TCFG_EN] & csr_tcfg_en;
        end

        if (csr_we && csr_num == `CSR_TCFG) begin
            csr_tcfg_periodic <= csr_wmask[`CSR_TCFG_PERIODIC] & csr_wvalue[`CSR_TCFG_PERIODIC] 
                            | ~csr_wmask[`CSR_TCFG_PERIODIC] & csr_tcfg_periodic;
            csr_tcfg_initval <= csr_wmask[`CSR_TCFG_INITVAL] & csr_wvalue[`CSR_TCFG_INITVAL] 
                            | ~csr_wmask[`CSR_TCFG_INITVAL] & csr_tcfg_initval;
        end
    end


    // TVAL 的 TimerVal 域；TI 挂起时冻结倒计时。
    wire [31:2] tcfg_next_value;
    wire [31:0] csr_tval;

    assign tcfg_next_value = csr_wmask[31:2] & csr_wvalue[31:2]
                           | ~csr_wmask[31:2] & csr_tcfg_initval;

    always @(posedge clk) begin
        if (reset) begin
            timer_cnt <= 32'hffffffff;
        end
        else if (tcfg_we) begin
            // 任意 TCFG 写入都装载 InitVal<<2。
            timer_cnt <= {tcfg_next_value, 2'b0};
        end
        else if (timer_en && !csr_estat_is[11]) begin
            if (timer_cnt != 32'b0) begin
                timer_cnt <= timer_cnt - 1'b1;
            end else begin
                // 到期当拍：periodic 重装；oneshot 停在 -1（同时 timer_en 在上块清 0）
                timer_cnt <= csr_tcfg_periodic ? {csr_tcfg_initval, 2'b0} : 32'hffffffff;
            end
        end
    end

    assign csr_tval = timer_cnt[31:0];


    // ------------------------------------------------------------------
    // LLBCTL：维护 LLBIT、KLO
    // ------------------------------------------------------------------
    reg        llbit;          // ROLLB 域读出
    reg        csr_llbctl_klo; // KLO 位（ertn 时不清 LLBIT 的一次性开关）
    wire csr_we_llbctl = csr_we && (csr_num == `CSR_LLBCTL);
    // csrwr LLBCTL：WCLLB=1 清 LLBIT；KLO 按 mask/wvalue 写入
    wire wcllb_set = csr_we_llbctl
                   && csr_wmask[`WCLLB] && csr_wvalue[`WCLLB];
    always @(posedge clk) begin
        if (reset) begin
            llbit          <= 1'b0;
            csr_llbctl_klo <= 1'b0;
        end
        // ERTN：KLO=1 时保留 LLBIT 并自动清 KLO；否则清 LLBIT
        else if (wb_valid && wb_is_ertn) begin
            if (csr_llbctl_klo) begin
                csr_llbctl_klo <= 1'b0;
            end else begin
                llbit <= 1'b0;
            end
        end
        // CSRWR LLBCTL：WCLLB 清 LLBIT；KLO 按 mask 写入；ROLLB 只读不写
        else if (csr_we_llbctl) begin
            if (wcllb_set) llbit <= 1'b0;
            csr_llbctl_klo <= csr_wmask[`KLO] & csr_wvalue[`KLO]
                            | ~csr_wmask[`KLO] & csr_llbctl_klo;
        end
        // ll.w 提交：置 LLBIT=1。
        else if (ll_set_in) begin
            llbit <= 1'b1;
        end
        // sc.w 提交：成功/失败均清 LLBIT
        else if (sc_set_in) begin
            llbit <= 1'b0;
        end
    end

    // csr处理类型的标志信号
    wire csr_take_ex   = wb_csr_ex; // 异常/中断提交
    wire csr_take_ertn = (wb_valid === 1'b1) && (wb_is_ertn === 1'b1) && !csr_take_ex; // ERTN，异常优先于同拍 ERTN（规范上不应同时发生）


    // CSR 的读出逻辑
    wire [31:0] csr_crmd_rvalue = {23'b0, csr_crmd_datm, csr_crmd_datf, csr_crmd_pg, csr_crmd_da, csr_crmd_ie, csr_crmd_plv};
    wire [31:0] csr_prmd_rvalue = {29'b0, csr_prmd_pie, csr_prmd_pplv};
    wire [31:0] csr_ecfg_rvalue = {19'b0, 13'h1bff & csr_ecfg_lie};
    // ESTAT 读值：{[31]=0, ESUBCODE[30:22], ECODE[21:16], [15:13]=0, IS[12:0]}
    wire [31:0] csr_estat_rvalue = {1'b0, 8'b0, csr_estat_esubcode, csr_estat_ecode[5:0], 3'b0, csr_estat_is};
    wire [31:0] csr_era_rvalue = csr_era_pc;
    wire [31:0] csr_badv_rvalue = csr_badv_vaddr;
    wire [31:0] csr_eentry_rvalue = {csr_eentry_va, 6'b0};
    wire [31:0] csr_save0_rvalue = csr_save0_data;
    wire [31:0] csr_save1_rvalue = csr_save1_data;
    wire [31:0] csr_save2_rvalue = csr_save2_data;
    wire [31:0] csr_save3_rvalue = csr_save3_data;
    wire [31:0] csr_tid_rvalue = csr_tid_tid;
    wire [31:0] csr_tcfg_rvalue = {csr_tcfg_initval, csr_tcfg_periodic, csr_tcfg_en};
    wire [31:0] csr_tval_rvalue = csr_tval;
    wire [31:0] csr_ticlr_rvalue = 32'b0;
    wire [31:0] csr_tlbidx_rvalue = csr_tlbidx & 32'hbf00_001f;
    wire [31:0] csr_tlbehi_rvalue = csr_tlbehi;
    wire [31:0] csr_tlbelo0_rvalue = csr_tlbelo0 & 32'h0fffff7f;
    wire [31:0] csr_tlbelo1_rvalue = csr_tlbelo1 & 32'h0fffff7f;
    // WB 同拍：显式 CSR 写 ASID 优先于 TLBRD 回写（与下方时序块顺序一致）
    wire          wb_tlbrd_commit = (wb_valid === 1'b1) && wb_tlbrd;
    wire          wb_tlbrd_valid   = wb_tlbrd_commit && (tlbrd_ne === 1'b0);
    wire [9:0]    csr_asid_low_rdata =
          wb_tlbrd_commit ? (wb_tlbrd_valid ? tlbrd_asid[9:0] : 10'h0)
        : csr_asid[9:0];
    wire [31:0] csr_asid_rvalue = {csr_asid[31:10], csr_asid_low_rdata};
    wire [31:0] csr_tlbrentry_rvalue = csr_tlbrentry;
    wire [31:0] csr_dmw0_rvalue = csr_dmw0;
    wire [31:0] csr_dmw1_rvalue = csr_dmw1;
    wire [31:0] csr_pgdl_rvalue = csr_pgdl;
    wire [31:0] csr_pgdh_rvalue = csr_pgdh;
    wire [31:0] csr_pgd_rvalue  = csr_badv_vaddr[31] ? csr_pgdh_rvalue : csr_pgdl_rvalue;
    // LLBCTL：bit0=ROLLB(llbit, RO), bit1=WCLLB(read 0), bit2=KLO
    wire [31:0] csr_llbctl_rvalue = {29'b0, csr_llbctl_klo, 1'b0, llbit};
    
    // 大 OR +=== 在 csr_rnum 含 X 时仍会产生 X 掩码；case+default 在无法匹配时给出确定 0。
    reg [31:0] csr_rvalue_sel;
    always @(*) begin
        case (csr_rnum)
            `CSR_CRMD:     csr_rvalue_sel = csr_crmd_rvalue;
            `CSR_PRMD:     csr_rvalue_sel = csr_prmd_rvalue;
            `CSR_ECFG:     csr_rvalue_sel = csr_ecfg_rvalue;
            `CSR_ESTAT:    csr_rvalue_sel = csr_estat_rvalue;
            `CSR_ERA:      csr_rvalue_sel = csr_era_rvalue;
            `CSR_BADV:     csr_rvalue_sel = csr_badv_rvalue;
            `CSR_EENTRY:   csr_rvalue_sel = csr_eentry_rvalue;
            `CSR_SAVE0:    csr_rvalue_sel = csr_save0_rvalue;
            `CSR_SAVE1:    csr_rvalue_sel = csr_save1_rvalue;
            `CSR_SAVE2:    csr_rvalue_sel = csr_save2_rvalue;
            `CSR_SAVE3:    csr_rvalue_sel = csr_save3_rvalue;
            `CSR_TID:      csr_rvalue_sel = csr_tid_rvalue;
            `CSR_TCFG:     csr_rvalue_sel = csr_tcfg_rvalue;
            `CSR_TVAL:     csr_rvalue_sel = csr_tval_rvalue;
            `CSR_TICLR:    csr_rvalue_sel = csr_ticlr_rvalue;
            `CSR_TLBIDX:   csr_rvalue_sel = csr_tlbidx_rvalue;
            `CSR_TLBEHI:   csr_rvalue_sel = csr_tlbehi_rvalue;
            `CSR_TLBELO0:  csr_rvalue_sel = csr_tlbelo0_rvalue;
            `CSR_TLBELO1:  csr_rvalue_sel = csr_tlbelo1_rvalue;
            `CSR_ASID:     csr_rvalue_sel = csr_asid_rvalue;
            `CSR_TLBRENTRY: csr_rvalue_sel = csr_tlbrentry_rvalue;
            `CSR_DMW0:     csr_rvalue_sel = csr_dmw0_rvalue;
            `CSR_DMW1:     csr_rvalue_sel = csr_dmw1_rvalue;
            `CSR_PGDL:     csr_rvalue_sel = csr_pgdl_rvalue;
            `CSR_PGDH:     csr_rvalue_sel = csr_pgdh_rvalue;
            `CSR_PGD:      csr_rvalue_sel = csr_pgd_rvalue;
            `CSR_LLBCTL:   csr_rvalue_sel = csr_llbctl_rvalue;
            default:       csr_rvalue_sel = 32'b0;
        endcase
    end
    assign csr_rvalue = csr_rvalue_sel;
    

    // has_int
    assign has_int = (((csr_estat_is[12:0] & csr_ecfg_lie[12:0]) != 13'b0) === 1'b1) && (csr_crmd_ie === 1'b1);

    // csr_next_pc
    assign csr_next_pc = csr_take_ex   ? (selected_tlbr ? csr_tlbrentry_rvalue : csr_eentry_rvalue)
                        : csr_take_ertn ? csr_era_rvalue
                        : 32'b0;

    // csr相关域读出逻辑，供外部使用
    assign csr_tid_out = csr_tid_rvalue;
    assign csr_crmd_da_out = csr_crmd_da;
    assign csr_crmd_pg_out = csr_crmd_pg;
    assign csr_crmd_plv_out = csr_crmd_plv;
    assign csr_asid_out = csr_asid[9:0];
    assign csr_tlbidx_ne_out    = csr_tlbidx[31];
    assign csr_tlbidx_ps_out    = csr_tlbidx[29:24];
    assign csr_tlbidx_index_out = csr_tlbidx[4:0];
    assign csr_tlbehi_vppn_out  = csr_tlbehi[31:13];
    assign csr_tlbelo0_ppn_out  = csr_tlbelo0[27:8];
    assign csr_tlbelo0_plv_out  = csr_tlbelo0[3:2];
    assign csr_tlbelo0_mat_out  = csr_tlbelo0[5:4];
    assign csr_tlbelo0_d_out    = csr_tlbelo0[1];
    assign csr_tlbelo0_v_out    = csr_tlbelo0[0];
    assign csr_tlbelo0_g_out    = csr_tlbelo0[6];
    assign csr_tlbelo1_ppn_out  = csr_tlbelo1[27:8];
    assign csr_tlbelo1_plv_out  = csr_tlbelo1[3:2];
    assign csr_tlbelo1_mat_out  = csr_tlbelo1[5:4];
    assign csr_tlbelo1_d_out    = csr_tlbelo1[1];
    assign csr_tlbelo1_v_out    = csr_tlbelo1[0];
    assign csr_tlbelo1_g_out    = csr_tlbelo1[6];
    assign csr_dmw0_vseg_out    = csr_dmw0[31:29];
    assign csr_dmw0_pseg_out    = csr_dmw0[27:25];
    assign csr_dmw0_mat_out     = csr_dmw0[5:4];
    assign csr_dmw0_plv3_out    = csr_dmw0[3];
    assign csr_dmw0_plv0_out    = csr_dmw0[0];
    assign csr_dmw1_vseg_out    = csr_dmw1[31:29];
    assign csr_dmw1_pseg_out    = csr_dmw1[27:25];
    assign csr_dmw1_mat_out     = csr_dmw1[5:4];
    assign csr_dmw1_plv3_out    = csr_dmw1[3];
    assign csr_dmw1_plv0_out    = csr_dmw1[0];
    assign csr_rand_index_out = timer_64[4:0];
    assign csr_crmd_datf_out = csr_crmd_datf;
    assign csr_crmd_datm_out = csr_crmd_datm;
    assign csr_estat_ecode_out = csr_estat_ecode;

    // LL/SC：提交逻辑使用 LLBIT 决定 sc.w 是否真正发起 store。
    assign csr_llbit_out  = llbit;
    assign diff_timer_64  = timer_64;

`ifdef DIFFTEST_EN
    // difftest 导出寄存器当前值（与 myCPU_example 一致）；勿用 diff_csr_snap 的 csr_we
    // 组合合并——cmt_valid 延迟 1 拍时会把下一 WB 槽 CSR 写提前暴露给上一拍 commit。
    assign diff_csr_crmd      = csr_crmd_rvalue;
    assign diff_csr_euen      = 32'b0;
    assign diff_csr_ecfg      = csr_ecfg_rvalue;
    // difftest 只导出 CSR 寄存器值；勿用 wb_csr_ex/era_fault_pc 组合超前值（下一 WB 槽例外会
    // 在上一拍 commit 的 DPI 采样窗口提前暴露 ERA，SmartWave @1158547 st.w 即现 e4 syscall ERA）
    assign diff_csr_estat     = csr_estat_rvalue;
    assign diff_csr_era       = csr_era_rvalue;
    assign diff_csr_prmd      = csr_prmd_rvalue;
    assign diff_csr_badv      = csr_badv_rvalue;
    assign diff_csr_eentry    = csr_eentry_rvalue;
    assign diff_csr_tlbidx    = csr_tlbidx_rvalue;
    assign diff_csr_tlbehi    = csr_tlbehi_rvalue;
    assign diff_csr_tlbelo0   = csr_tlbelo0_rvalue;
    assign diff_csr_tlbelo1   = csr_tlbelo1_rvalue;
    assign diff_csr_asid      = csr_asid_rvalue;
    assign diff_csr_pgdl      = csr_pgdl_rvalue;
    assign diff_csr_pgdh      = csr_pgdh_rvalue;
    assign diff_csr_save0     = csr_save0_rvalue;
    assign diff_csr_save1     = csr_save1_rvalue;
    assign diff_csr_save2     = csr_save2_rvalue;
    assign diff_csr_save3     = csr_save3_rvalue;
    assign diff_csr_tid       = csr_tid_rvalue;
    assign diff_csr_tcfg      = csr_tcfg_rvalue;
    assign diff_csr_tval      = csr_tval_rvalue;
    assign diff_csr_ticlr     = csr_ticlr_rvalue;
    assign diff_csr_llbctl    = csr_llbctl_rvalue;
    assign diff_csr_tlbrentry = csr_tlbrentry_rvalue;
    assign diff_csr_dmw0      = csr_dmw0_rvalue;
    assign diff_csr_dmw1      = csr_dmw1_rvalue;
    assign diff_wb_ecode      = Ecode;
`endif

    
endmodule
