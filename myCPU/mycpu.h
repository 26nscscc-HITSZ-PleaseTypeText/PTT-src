`ifndef MYCPU_H
`define MYCPU_H

// chiplab 仿真 (-DSIMU) 打开 SYNTHESIS 探针/PERF；FPGA 综合工程勿定义 SYNTHESIS
`ifdef SIMU
`ifndef SYNTHESIS
`define SYNTHESIS
`endif
`endif

`timescale 1ns / 1ps

/* ============================================================
 * myCPU 乱序双发射架构 全局宏定义
 * ------------------------------------------------------------
 * 架构总览：
 *   前端：BPU(uBTB+FTB+TAGE+RAS) -> FTQ -> IFU(+预译码) -> ICache -> IB
 *   后端：decoder x2 -> rename(RAT+ARF/ROB读) -> dispatch -> 4xRS
 *         -> FU(ALU0/ALU1/LSU/MDU) -> ROB -> commit(双提交)
 *   访存：LSU(AGU+DC两级) / store_buffer(提交后写) / DCache / L2 / AXI桥
 *   重命名：ROB 编号即重命名标签，无独立 PRF/freelist
 *   恢复：误预测/异常/特权指令 -> 提交级统一冲刷
 * ============================================================ */

/* =====================================================
 * 机器宽度（方案 A+：参数化留 4 宽底子）
 * 注意：Verilog 宏无法自动算 $clog2，派生宏需手工同步修改！
 *       升 4 宽时除改宏外，还需在顶层/各模块补 2/3 号槽位端口。
 * ===================================================== */
`define MACHINE_WIDTH   2   // 译码/重命名/发射/提交宽度
`define FETCH_WIDTH     4   // 取指宽度（基本块最大指令数）
`define BLK_LEN_W       3   // 基本块长度位宽（可表示 0~4）

/* =====================================================
 * 队列深度与派生位宽
 * ===================================================== */
`define ROB_SIZE        32  // ROB 总项数（奇偶双体 = 16 对 x 2 路）
`define ROB_W           5   // $clog2(ROB_SIZE)，ROB 编号位宽 = {奇偶位, 对指针}
`define ROB_PAIR_W      4   // ROB_W-1，ROB 对指针(head/tail)位宽
`define ROB_GUARD       5   // head/tail 安全间距：保证 dispatch 读 ROB
                            // 结果时，已提交但未被覆盖的数据仍可读，必须保留！）
// 未决 store 提交判定：对指针距离 d=(R-head) 环形；存活项 d ∈ [0, N-GUARD)，
// wrap 后 d ∈ [N-GUARD, N)。阈值须随 ROB_PAIR_W 伸缩（原写死 12 只适配 16 对）。
`define ROB_WRAP_THR    ((1<<`ROB_PAIR_W)-`ROB_GUARD)

`define RS_ALU_SIZE     4   // 每个 ALU 保留站项数（乱序发射）
`define RS_ALU_IDX_W    2   // $clog2(RS_ALU_SIZE)
`define RS_ALU_OCC_W    3   // $clog2(RS_ALU_SIZE+1)，occupancy 0..SIZE
`define RS_MEM_SIZE     4   // 访存保留站项数（FIFO + 有限 load 越过）；加深只会把
                            // 反压转移到 ROB，D$ miss 延迟不变，因此保持 4 项
`define RS_MEM_IDX_W    2   // $clog2(RS_MEM_SIZE)
`define RS_MEM_OCC_W    3   // $clog2(RS_MEM_SIZE+1)
`define RS_MDU_SIZE     2   // 乘除保留站项数（FIFO 顺序发射）

`define SB_SIZE         8   // store buffer 项数（提交后写缓冲；行聚合+同字旁路合并）
`define SB_W            3   // $clog2(SB_SIZE)

`define FTQ_SIZE        16  // 取指目标队列项数（需 > BPU 推测深度，避免 I$ 慢时 ftq_full 死锁）
`define FTQ_W           4   // $clog2(FTQ_SIZE)

`define IB_SIZE         16  // 指令缓冲项数（入口<=4条/拍，出口 2条/拍）
`define IB_W            4   // $clog2(IB_SIZE)
// 默认启用空 IB 组合旁路；定义该宏可关闭旁路，用于隔离冷启动行为。
// `define IB_DISABLE_EMPTY_FALLTHROUGH


/* =====================================================
 * Cache 几何参数
 * - L1 采用 VIPT：要求 index+offset <= 12（4KB 页内偏移）
 *   16KB 4路 32B行：每路 4KB = 128 组 -> index 7bit + offset 5bit = 12 √
 * - 行加大到 32B（8 字）：长 burst 摊薄 chiplab AXI 每事务开销
 * - L1/L2/AXI 桥间互联仍为 128bit/拍，一行 = 2 拍（ret_last 标记最后一拍）
 * ===================================================== */
`define CACHE_LINE_BYTES  32          // cache 行字节数（16/32/64 可调，需同步改派生宏）
`define CACHE_LINE_BITS   256         // CACHE_LINE_BYTES*8
`define CACHE_LINE_W      5           // $clog2(CACHE_LINE_BYTES)，行内偏移位宽
`define CACHE_LINE_WORDS  8           // 每行 32bit 字数

`define L1_NWAY           4           // L1 I/D cache 路数
`define L1_NMSHR          2           // L1 D$ MSHR 项数（单 AXI 读 owner）
`define DC_MSHR_DEPTH     `L1_NMSHR   // dcache 后台 MSHR 深度
`define LSU_MISS_DEPTH    `DC_MSHR_DEPTH // LSU miss 槽深度（与 D$ MSHR 对齐）
// 保留 hit-bypass，在顶层对 mem_wb 整总线打一拍切断 D$→RS；early2 关闭，
// 避免消费者在写回数据可用前提前唤醒。
`define LSU_DC_HIT_BYPASS 1           // 保持命中当拍算出 WB 数据
`define LSU_WB_PIPE       1           // 顶层 mem_wb→RS/ROB 打一拍，切断命中组合路径
`define LSU_EARLY2_ENABLE 0           // 与 WB_PIPE 配对关闭（否则 T 醒、T+1 数据）
`define LSU_EARLY2_PIPE   0
`define IFU_FTQ_DIRECT    1           // 1: 允许同拍 FTQ→I$；仅在 mmu_i_direct_ok（无主 TLB）时开火
`define STQ_DEPTH         16          // 未提交 store 队列深度；提交后按 ROB id 释放
`define L1_NSET           128         // L1 每路组数（16KB/4路/32B）
`define L1_INDEX_W        7           // $clog2(L1_NSET)
`define L1_TAG_W          20          // 32 - 7 - 5

`define L2_NWAY           2           // L2 路数
`define L2_NSET           2048        // L2 每路组数（128KB/2路/32B）
`define L2_INDEX_W        11          // $clog2(L2_NSET)
`define L2_TAG_W          16          // 32 - 11 - 5

/* =====================================================
 * 分支预测器（BPU）参数
 * ===================================================== */
`define UBTB_SIZE         16          // uBTB 项数（全相联，当拍返回，回填小循环）
// 普通 JIRL 使用小型目标缓存；CALL/RET 由 RAS 处理。
`define JIRL_TC_SIZE      32          // 普通 JIRL 小型目标缓存项数（直接映射）
`define JIRL_TC_INDEX_W   5           // $clog2(JIRL_TC_SIZE)
`define FTB_NWAY          4           // FTB 路数
`define FTB_NSET          2048        // FTB 每路组数（共 8192 项，BRAM 实现）
`define FTB_INDEX_W       11          // $clog2(FTB_NSET)
`define FTB_UPDATE_Q_DEPTH 32          // FTB 训练更新 FIFO；查询优先，突发训练可排队
`define TAGE_UPDATE_Q_DEPTH 32         // TAGE 条件分支训练 FIFO；较小深度无法满足
                                       // Linux 长程稳定性，较大深度未改善准确率或 IPC

`define TAGE_BASE_DEPTH   8192        // TAGE 基础表项数（2bit 饱和计数器）
`define TAGE_TAG_NUM      4           // TAGE 标记表个数
`define TAGE_TAG_DEPTH    1024        // 每个标记表项数
`define TAGE_TAG_W        12          // 标记位宽
`define TAGE_HIST_LEN0    11          // 各标记表全局历史长度（几何级数）
`define TAGE_HIST_LEN1    23
`define TAGE_HIST_LEN2    53
`define TAGE_HIST_LEN3    112
`define GHR_LEN           112         // 覆盖 TAGE 最长标记表使用的全局历史

`define RAS_DEPTH         32          // 返回地址栈深度（前端栈+提交栈双栈）
`define RAS_W             5           // $clog2(RAS_DEPTH)

`define BPU_META_W        64          // FTQ 暂存的 BPU 训练元数据打包宽度
                                      // （TAGE provider/altpred/各表ctr/useful 等，
                                      //   具体打包格式由 tage.v 实现者定义，不够可加宽）

/* =====================================================
 * 分支类型（FTB 条目 / 预译码 / ROB 训练用）
 * ===================================================== */
`define BR_TYPE_W       2
`define BR_TYPE_COND    2'd0   // 条件分支（方向由 TAGE 预测）
`define BR_TYPE_UNCOND  2'd1   // 无条件直接跳转（B）
`define BR_TYPE_CALL    2'd2   // 函数调用（BL / JIRL rd=r1）
`define BR_TYPE_RET     2'd3   // 函数返回（JIRL rd=r0,rj=r1）

/* =====================================================
 * FU 类型（独热，决定指令进哪个保留站）
 * ===================================================== */
`define FU_NUM      3
`define FU_ALU      0      // 单周期算术逻辑/分支（双 ALU，乱序发射）
`define FU_MEM      1      // 访存（LSU：AGU+DC 两级流水，顺序发射）
`define FU_MDU      2      // 乘除/CSR读/rdcnt/TLB维护执行等多周期杂项（顺序发射）

/* =====================================================
 * 冲刷类型（commit -> ctrl）
 * ===================================================== */
`define FLUSH_TYPE_W      3
`define FLUSH_NONE        3'd0
`define FLUSH_MISPRED     3'd1   // 分支误预测（提交级发现，一期方案）
`define FLUSH_EXCP        3'd2   // 异常/中断（跳 EENTRY/TLBRENTRY）
`define FLUSH_ERTN        3'd3   // ertn 返回（跳 ERA）
`define FLUSH_REFETCH     3'd4   // 特权指令提交后重取下一条（csr写/tlb维护/cacop/ll/sc/ibar/idle）

/* =====================================================
 * 特权/提交特殊处理类标记（ROB 静态字段，独热）
 * 这些指令一律"单提交 + 提交后 FLUSH_REFETCH 冲刷"（遇事不决清空流水线）
 * ===================================================== */
`define PRIV_NUM        8
`define PRIV_CSR_WR     0   // csrwr / csrxchg（提交时写 CSR）
`define PRIV_ERTN       1   // ertn
`define PRIV_TLB        2   // tlbsrch/tlbrd/tlbwr/tlbfill/invtlb（提交时落地）
`define PRIV_CACOP      3   // cacop（提交时驱动 I$/D$ 维护口）
`define PRIV_IDLE       4   // idle（提交后挂起等中断）
`define PRIV_LL         5   // ll.w（提交时置 LLBIT）
`define PRIV_SC         6   // sc.w（提交时判 LLBIT 决定是否真正写入）
`define PRIV_IBAR       7   // ibar/dbar（屏障：等 SB 排空）

/* =====================================================
 * 统一异常向量（ROB 异常字段，独热；中断在提交级附着不占位）
 * 优先级与编码翻译在 commit.v 完成后送 csr_exception_commit_handler
 * ===================================================== */
`define EXCP_NUM        15
`define EXCP_ADEF       0    // 取指地址错
`define EXCP_TLBR_F     1    // 取指 TLB 重填
`define EXCP_PIF        2    // 取指页无效
`define EXCP_PPI_F      3    // 取指页权限
`define EXCP_INE        4    // 指令不存在
`define EXCP_SYS        5    // syscall
`define EXCP_BRK        6    // break
`define EXCP_IPE        7    // 指令特权错
`define EXCP_ALE        8    // 访存地址非对齐
`define EXCP_ADEM       9    // 访存地址错
`define EXCP_TLBR_M     10   // 访存 TLB 重填
`define EXCP_PIL        11   // load 页无效
`define EXCP_PIS        12   // store 页无效
`define EXCP_PPI_M      13   // 访存页权限
`define EXCP_PME        14   // 页修改
// IFU/MMU 取指阶段产生的异常位（用于与译码 INE 去重）
`define FETCH_EXCP_MASK 15'b0000_0000_000_1111

/* =====================================================
 * 分支 / 跳转操作码 (br_op) —— 沿用原译码体系
 * ===================================================== */
`define BR_OP_NUM       9
`define BR_OP_BEQ       0
`define BR_OP_BNE       1
`define BR_OP_JIRL      2
`define BR_OP_BL        3
`define BR_OP_B         4
`define BR_OP_BLT       5
`define BR_OP_BGE       6
`define BR_OP_BLTU      7
`define BR_OP_BGEU      8

/* =====================================================
 * ALU 操作码 (alu_op)
 * 22 位独热；MUL/DIV/MOD 类由 decoder 标记 futype=FU_MDU
 * ===================================================== */
`define ALU_OP_NUM      22
`define ALU_OP_ADD      0
`define ALU_OP_SUB      1
`define ALU_OP_SLT      2
`define ALU_OP_SLTU     3
`define ALU_OP_AND      4
`define ALU_OP_NOR      5
`define ALU_OP_OR       6
`define ALU_OP_XOR      7
`define ALU_OP_SLL      8
`define ALU_OP_SRL      9
`define ALU_OP_SRA      10
`define ALU_OP_LUI      11
`define ALU_OP_MUL_W    12
`define ALU_OP_MULH_W   13
`define ALU_OP_MULH_WU  14
`define ALU_OP_DIV_W    15
`define ALU_OP_DIV_WU   16
`define ALU_OP_MOD_W    17
`define ALU_OP_MOD_WU   18
`define ALU_OP_ANDN     19
`define ALU_OP_ORN      20
`define ALU_OP_PCADD    21

/* =====================================================
 * 访存操作码 (mem_op)
 * 10 位独热，sc.w 用独立槽方便区分原子写
 * ===================================================== */
`define MEM_OP_NUM      10
`define MEM_OP_ST_W     0
`define MEM_OP_LD_W     1
`define MEM_OP_ST_B     2
`define MEM_OP_ST_H     3
`define MEM_OP_LD_B     4
`define MEM_OP_LD_H     5
`define MEM_OP_LD_BU    6
`define MEM_OP_LD_HU    7
`define MEM_OP_LL_W     8
`define MEM_OP_SC_W     9

/* =====================================================
 * 访存字节掩码 / overlap（STQ 与 SB 共用；原 mem_byte_util.vh）
 * ===================================================== */
// load 按 mem_op + 字内偏移 → 访问字节掩码（与 store wstrb 对齐比较）
function automatic [3:0] mem_load_byte_mask;
    input [7:4]             op;
    input [1:0]             off;
    begin
        if (op[`MEM_OP_LD_B] || op[`MEM_OP_LD_BU])
            mem_load_byte_mask = 4'b0001 << off;
        else if (op[`MEM_OP_LD_H] || op[`MEM_OP_LD_HU])
            mem_load_byte_mask = off[1] ? 4'b1100 : 4'b0011;
        else
            mem_load_byte_mask = 4'b1111; // word / default
    end
endfunction

// 同字地址上 store.strb 与 load 字节掩码是否重叠
function automatic mem_st_ld_overlap;
    input [31:2] st_word;
    input [3:0]  st_strb;
    input [31:2] ld_word;
    input [3:0]  ld_mask;
    begin
        mem_st_ld_overlap = (st_word == ld_word)
                         && |(st_strb & ld_mask);
    end
endfunction

// SB 前递：项与查询是否同字
function automatic mem_same_word;
    input [31:2] a;
    input [31:2] b;
    begin
        mem_same_word = (a == b);
    end
endfunction

/* =====================================================
 * CSR 指令操作码 (csr_op)
 * ===================================================== */
`define CSR_OP_NUM      3
`define CSR_OP_CSRRD    0
`define CSR_OP_CSRWR    1
`define CSR_OP_CSRXCHG  2

/* =====================================================
 * 写回数据源 (wb_src_op)
 * ===================================================== */
`define WB_SRC_NUM      4
`define WB_SRC_ALU      0
`define WB_SRC_CNTVL    1
`define WB_SRC_CNTVH    2
`define WB_SRC_TID      3

/* =====================================================
 * TLB 维护指令 (tlb_op)
 * ===================================================== */
`define TLB_OP_NUM        11
`define TLB_OP_TLBSRCH    0
`define TLB_OP_TLBRD      1
`define TLB_OP_TLBWR      2
`define TLB_OP_TLBFILL    3
`define TLB_OP_INVTLB_0   4
`define TLB_OP_INVTLB_1   5
`define TLB_OP_INVTLB_2   6
`define TLB_OP_INVTLB_3   7
`define TLB_OP_INVTLB_4   8
`define TLB_OP_INVTLB_5   9
`define TLB_OP_INVTLB_6   10

/* =====================================================
 * TLB 异常向量 (tlb_ex_valid) —— 翻译通路(mmu/tlb_manager)输出格式
 * ===================================================== */
`define TLB_EX_NUM   6
`define TLB_EX_TLBR  5
`define TLB_EX_PIF   4
`define TLB_EX_PPI   3
`define TLB_EX_PIL   2
`define TLB_EX_PIS   1
`define TLB_EX_PME   0

/* =====================================================
 * 流水级用 8 位 Ecode（与已有 RTL 兼容）
 * ===================================================== */
`define INT_ECODE   8'h00
`define ADEF_ECODE  8'h08
`define ADEM_ECODE  8'h08   // ADEM 与 ADEF 同 ECODE=0x8，靠 ESUBCODE 区分
`define INE_ECODE   8'h0d
`define IPE_ECODE   8'h0e
`define ALE_ECODE   8'h09
`define SYS_ECODE   8'h0b
`define BRK_ECODE   8'h0c
`define IDLE_ECODE  8'h0a   // LA32R: IDLE 进入待机，不算异常；保留占位
`define PIL_ECODE   8'h01
`define PIS_ECODE   8'h02
`define PIF_ECODE   8'h03
`define PME_ECODE   8'h04
`define PPI_ECODE   8'h07
`define TLBR_ECODE  8'h3f

`define INT_ESUBCODE   1'b0
`define ADEF_ESUBCODE  1'b0   // ADEF: ESUBCODE=0
`define ADEM_ESUBCODE  1'b1   // ADEM: ESUBCODE=1
`define INE_ESUBCODE   1'b0
`define IPE_ESUBCODE   1'b0
`define ALE_ESUBCODE   1'b0
`define SYS_ESUBCODE   1'b0
`define BRK_ESUBCODE   1'b0
`define IDLE_ESUBCODE  1'b0
`define PIL_ESUBCODE   1'b0
`define PIS_ESUBCODE   1'b0
`define PIF_ESUBCODE   1'b0
`define PME_ESUBCODE   1'b0
`define PPI_ESUBCODE   1'b0
`define TLBR_ESUBCODE  1'b0

`define Ecode_other_exception    8'hFF
`define Esubcode_other_exception 1'b0

/* =====================================================
 * 6 位 Ecode（LoongArch CSR 编码）
 * ===================================================== */
`define ECODE_INT  6'h0
`define ECODE_PIL  6'h1
`define ECODE_PIS  6'h2
`define ECODE_PIF  6'h3
`define ECODE_PME  6'h4
`define ECODE_PPI  6'h7
`define ECODE_ADEF 6'h8
`define ECODE_ADEM 6'h8
`define ECODE_ALE  6'h9
`define ECODE_SYS  6'hb
`define ECODE_BRK  6'hc
`define ECODE_INE  6'hd
`define ECODE_IPE  6'he
`define ECODE_FPD  6'hf
`define ECODE_TLBR 6'h3f
`define ESUBCODE_ADEF  9'h0
`define ESUBCODE_ADEM  9'h1

/* =====================================================
 * CSR 寄存器号
 * ===================================================== */
`define CSR_CRMD        14'h0000
`define CSR_PRMD        14'h0001
`define CSR_EUEN        14'h0002
`define CSR_ECFG        14'h0004
`define CSR_ESTAT       14'h0005
`define CSR_ERA         14'h0006
`define CSR_BADV        14'h0007
`define CSR_EENTRY      14'h000c
`define CSR_TLBIDX      14'h0010
`define CSR_TLBEHI      14'h0011
`define CSR_TLBELO0     14'h0012
`define CSR_TLBELO1     14'h0013
`define CSR_ASID        14'h0018
`define CSR_PGDL        14'h0019
`define CSR_PGDH        14'h001a
`define CSR_PGD         14'h001b
`define CSR_CPUID       14'h0020
`define CSR_SAVE0       14'h0030
`define CSR_SAVE1       14'h0031
`define CSR_SAVE2       14'h0032
`define CSR_SAVE3       14'h0033
`define CSR_TID         14'h0040
`define CSR_TCFG        14'h0041
`define CSR_TVAL        14'h0042
`define CSR_TICLR       14'h0044
// L0 CSR 写白名单：提交可免 FLUSH_REFETCH（须配合 rename 串行排空，防在途 csrrd 读旧值）
`define CSR_NUM_IS_L0_NOFLUSH(n) ( \
    ((n) == `CSR_TICLR) || ((n) == `CSR_SAVE0) || ((n) == `CSR_SAVE1) || \
    ((n) == `CSR_SAVE2) || ((n) == `CSR_SAVE3) || ((n) == `CSR_TID))
`define CSR_LLBCTL      14'h0060
`define CSR_TLBRENTRY   14'h0088
`define CSR_CTAG        14'h0098
`define CSR_DMW0        14'h0180
`define CSR_DMW1        14'h0181

/* =====================================================
 * CSR 字段宏
 * ===================================================== */
`define PLV       1:0
`define IE        2
`define DA        3
`define PG        4
`define DATF      6:5
`define DATM      8:7
`define PPLV      1:0
`define PIE       2
`define LIE       12:0
`define LIE_1     9:0
`define LIE_2     12:11
`define IS        12:0
`define ECODE     21:16
`define ESUBCODE  30:22
`define INDEX     4:0
`define PS        29:24
`define NE        31
`define VPPN      31:13
`define TLB_V      0
`define TLB_D      1
`define TLB_PLV    3:2
`define TLB_MAT    5:4
`define TLB_G      6
`define TLB_PPN    31:8
`define TLB_PPN_EN 27:8
`define TLB_ASID  9:0
`define COREID    8:0
`define ROLLB     0
`define WCLLB     1
`define KLO       2
`define EN        0
`define PERIODIC  1
`define INITVAL   31:2
`define CLR       0
`define TLBRENTRY_PA 31:6
`define PLV0      0
`define PLV3      3
`define DMW_MAT   5:4
`define PSEG      27:25
`define VSEG      31:29
`define BASE      31:12

/* =====================================================
 * 兼容 RTL 中已有的别名字段
 * ===================================================== */
`define CSR_CRMD_PLV       1:0
`define CSR_CRMD_IE        2
`define CSR_CRMD_DA        3
`define CSR_CRMD_PG        4
`define CSR_CRMD_DATF      6:5
`define CSR_CRMD_DATM      8:7
`define CSR_PRMD_PPLV      1:0
`define CSR_PRMD_PIE       2
`define CSR_ECFG_LIE       12:0
`define CSR_ESTAT_IS10     1:0
`define CSR_TICLR_CLR      0
`define CSR_ERA_PC         31:0
`define CSR_EENTRY_VA      31:6
`define CSR_SAVE_DATA      31:0
`define CSR_TID_TID        31:0
`define CSR_TCFG_EN        0
`define CSR_TCFG_PERIODIC  1
`define CSR_TCFG_INITVAL   31:2

/* =====================================================
 * Cache 维护指令操作码 (cache_op / cacop_op)
 * ===================================================== */
`define CACHE_OP_NUM    2
`define CACHE_OP_CACOP  0
`define CACHE_OP_IBAR   1

`define CACOP_OP_IDX_INV  2'b00
`define CACOP_OP_HIT_INV  2'b01
`define CACOP_OP_HIT_WB   2'b10

`endif
