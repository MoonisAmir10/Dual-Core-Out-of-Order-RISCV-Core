`timescale 1ns / 1ps

// ============================================================================
// Phase 1 Single-Core Unified RV32IMA_Zicsr_Zifencei SoC (ooo2w_v2)
//   - ooo2w_core (CORE_ID, AWIDTH=20, ID_W=7)
//   - Private 4 KB (param 16 KB) 2-Way 16B-Line L1 Instruction Cache (icache_2w)
//   - Private 4 KB (param 16 KB) 2-Way 16B-Line Non-Blocking L1D (dcache_nb + RV32A)
//   - 2-to-1 Split-Transaction Read Arbiter (mem_arb_2to1)
//   - Unified 256 KB 128-Bit Pipelined Backing Memory (Code + Data + Stack!)
//   - Optional TB snoop-invalidation injection hook for Phase 1 replay tests!
// ============================================================================
module single_ooo2w_p1_soc #(
    parameter [31:0]  CORE_ID         = 32'd0,
    parameter integer MEM_LATENCY     = 4,
    parameter integer ICACHE_SETS     = 128, // 4 KB default
    parameter integer DCACHE_SETS     = 128, // 4 KB default
    parameter integer ENABLE_PREFETCH = 1
) (
    input  wire         clk,
    input  wire         reset_n,

    // Synthetic external snoop-invalidation injection port (Phase 1 replay verification)
    input  wire         ext_snoop_inv_valid,
    input  wire [15:0]  ext_snoop_inv_line,

    // 2-Wide RVFI Commit Trace outputs
    output wire         commit0_valid,
    output wire [31:0]  commit0_seq_id,
    output wire [31:0]  commit0_pc,
    output wire [31:0]  commit0_instr,
    output wire         commit0_rd_we,
    output wire [4:0]   commit0_rd_addr,
    output wire [31:0]  commit0_rd_wdata,
    output wire         commit0_mem_valid,
    output wire         commit0_mem_we,
    output wire [31:0]  commit0_mem_addr,
    output wire [3:0]   commit0_mem_wstrb,
    output wire [31:0]  commit0_mem_wdata,

    output wire         commit1_valid,
    output wire [31:0]  commit1_seq_id,
    output wire [31:0]  commit1_pc,
    output wire [31:0]  commit1_instr,
    output wire         commit1_rd_we,
    output wire [4:0]   commit1_rd_addr,
    output wire [31:0]  commit1_rd_wdata,
    output wire         commit1_mem_valid,
    output wire         commit1_mem_we,
    output wire [31:0]  commit1_mem_addr,
    output wire [3:0]   commit1_mem_wstrb,
    output wire [31:0]  commit1_mem_wdata,

    // OoO monitor trace hooks
    output wire         trace_disp0_valid,
    output wire [31:0]  trace_disp0_seq,
    output wire [31:0]  trace_disp0_pc,
    output wire [31:0]  trace_disp0_instr,
    output wire         trace_disp1_valid,
    output wire [31:0]  trace_disp1_seq,
    output wire [31:0]  trace_disp1_pc,
    output wire [31:0]  trace_disp1_instr,
    output wire [5:0]   trace_issue_valid,
    output wire [191:0] trace_issue_seq,
    output wire [1:0]   trace_cdb_valid,
    output wire [63:0]  trace_cdb_seq,
    output wire         trace_flush_valid,
    output wire [31:0]  trace_flush_after_seq,

    output wire         halted,
    output wire [31:0]  x14,

    // Hardware PMU Counters (Core + L1I + L1D + RV32A + Replay)
    output reg  [31:0]  pmu_cycles,
    output reg  [31:0]  pmu_instret,
    output reg  [31:0]  pmu_ic_hits,
    output reg  [31:0]  pmu_ic_misses,
    output reg  [31:0]  pmu_ic_pf_hits,
    output reg  [31:0]  pmu_ic_stall_cyc,
    output reg  [31:0]  pmu_dcache_hits,
    output reg  [31:0]  pmu_dcache_misses,
    output reg  [31:0]  pmu_dcache_wbs,
    output reg  [31:0]  pmu_dcache_hum,
    output reg  [31:0]  pmu_dcache_mum_cyc,
    output reg  [31:0]  pmu_mshr_coalesce,
    output reg  [31:0]  pmu_stlf_hits,
    output reg  [31:0]  pmu_br_total,
    output reg  [31:0]  pmu_br_mispred,
    output reg  [31:0]  pmu_ex_bypass_hits,
    output reg  [31:0]  pmu_ras_preds,
    output reg  [31:0]  pmu_ras_mispreds,
    output reg  [31:0]  pmu_dual_commit_cyc,
    output reg  [31:0]  pmu_replay_flushes,
    output reg  [31:0]  pmu_amo_commits,
    output reg  [31:0]  pmu_fence_commits
);

localparam integer AWIDTH    = 20;
localparam integer ID_W      = 7;
localparam integer LADDR_W   = AWIDTH - 4; // 16
localparam integer LINE_BITS = 14;         // 16384 x 16 B = 256 KB unified RAM

wire        imem_req_valid, imem_valid0, imem_valid1;
wire [31:0] imem_pc0, imem_instr0, imem_pc1, imem_instr1;
wire        ic_flush, ic_invalidate_all, ic_idle_u;
wire        ic_hit_p, ic_miss_p, ic_pf_hit_p, ic_stall_p;

wire        dc_req_valid, dc_req_ready, dc_req_is_write, dc_req_mmio_u;
wire [3:0]  dc_req_amo_op;
wire [AWIDTH-1:0] dc_req_addr;
wire [31:0] dc_req_wdata;
wire [3:0]  dc_req_wstrb;
wire [ID_W-1:0] dc_req_id;
wire [4:0]  dc_req_rob_tag;

wire        dc_hit_valid, dc_fill_valid;
wire [ID_W-1:0] dc_hit_id, dc_fill_id;
wire [31:0] dc_hit_rdata, dc_fill_rdata;

wire        dc_flush, dc_squash_valid, dc_clean_all_req, dc_clean_all_done;
wire [4:0]  dc_squash_tag, dc_rob_head;
wire        dc_idle_u, dc_writes_idle, dc_any_miss;
wire        dc_hit_p, dc_miss_p, dc_wb_p, dc_hum_p, dc_mum_p, dc_coal_p;

wire        ic_rd_valid, ic_rd_ready, ic_rresp_valid;
wire [LADDR_W-1:0] ic_rd_laddr;
wire [127:0] ic_rresp_data;

wire        dc_rd_valid, dc_rd_ready, dc_rd_id, dc_rresp_valid, dc_rresp_id;
wire [LADDR_W-1:0] dc_rd_laddr;
wire [127:0] dc_rresp_data;

wire        mem_rd_valid, mem_rd_ready, mem_rresp_valid;
wire [LADDR_W-1:0] mem_rd_laddr;
wire [1:0]  mem_rd_id, mem_rresp_id;
wire [127:0] mem_rresp_data;

wire        mem_wr_valid, mem_wr_ready;
wire [LADDR_W-1:0] mem_wr_laddr;
wire [127:0] mem_wr_data;

wire        st_rob_u, st_rs_u, st_lsq_u, st_cp_u, st_stlf, st_fsq_u, st_div_u, st_dcm_u;
wire        st_dd_u, st_dc_u, st_dcomm, st_replay, st_amo, st_fence;
wire [1:0]  st_br_tot, st_br_misp, st_ww_u, st_ras_p, st_ras_m;
wire [3:0]  st_ex_byp;

ooo2w_core #(
    .CORE_ID(CORE_ID),
    .RESET_PC(32'd0),
    .AWIDTH(AWIDTH),
    .ID_W(ID_W)
) u_core (
    .clk(clk), .reset_n(reset_n),
    .imem_req_valid(imem_req_valid),
    .imem_pc0(imem_pc0), .imem_valid0(imem_valid0), .imem_instr0(imem_instr0),
    .imem_pc1(imem_pc1), .imem_valid1(imem_valid1), .imem_instr1(imem_instr1),
    .ic_flush(ic_flush), .ic_invalidate_all(ic_invalidate_all),
    .dc_req_valid(dc_req_valid), .dc_req_ready(dc_req_ready),
    .dc_req_is_write(dc_req_is_write), .dc_req_is_mmio(dc_req_mmio_u),
    .dc_req_amo_op(dc_req_amo_op),
    .dc_req_addr(dc_req_addr), .dc_req_wdata(dc_req_wdata), .dc_req_wstrb(dc_req_wstrb),
    .dc_req_id(dc_req_id), .dc_req_rob_tag(dc_req_rob_tag),
    .dc_hit_valid(dc_hit_valid), .dc_hit_id(dc_hit_id), .dc_hit_rdata(dc_hit_rdata),
    .dc_fill_valid(dc_fill_valid), .dc_fill_id(dc_fill_id), .dc_fill_rdata(dc_fill_rdata),
    .dc_flush(dc_flush), .dc_squash_valid(dc_squash_valid),
    .dc_squash_tag(dc_squash_tag), .dc_rob_head(dc_rob_head),
    .dc_clean_all_req(dc_clean_all_req), .dc_clean_all_done(dc_clean_all_done),
    .dc_writes_idle(dc_writes_idle), .dc_any_miss(dc_any_miss),
    .coh_snoop_inv_valid(ext_snoop_inv_valid),
    .coh_snoop_inv_line(ext_snoop_inv_line),
    .commit0_valid(commit0_valid), .commit0_seq_id(commit0_seq_id),
    .commit0_pc(commit0_pc), .commit0_instr(commit0_instr),
    .commit0_rd_we(commit0_rd_we), .commit0_rd_addr(commit0_rd_addr), .commit0_rd_wdata(commit0_rd_wdata),
    .commit0_mem_valid(commit0_mem_valid), .commit0_mem_we(commit0_mem_we),
    .commit0_mem_addr(commit0_mem_addr), .commit0_mem_wstrb(commit0_mem_wstrb), .commit0_mem_wdata(commit0_mem_wdata),
    .commit1_valid(commit1_valid), .commit1_seq_id(commit1_seq_id),
    .commit1_pc(commit1_pc), .commit1_instr(commit1_instr),
    .commit1_rd_we(commit1_rd_we), .commit1_rd_addr(commit1_rd_addr), .commit1_rd_wdata(commit1_rd_wdata),
    .commit1_mem_valid(commit1_mem_valid), .commit1_mem_we(commit1_mem_we),
    .commit1_mem_addr(commit1_mem_addr), .commit1_mem_wstrb(commit1_mem_wstrb), .commit1_mem_wdata(commit1_mem_wdata),
    .trace_disp0_valid(trace_disp0_valid), .trace_disp0_seq(trace_disp0_seq),
    .trace_disp0_pc(trace_disp0_pc), .trace_disp0_instr(trace_disp0_instr),
    .trace_disp1_valid(trace_disp1_valid), .trace_disp1_seq(trace_disp1_seq),
    .trace_disp1_pc(trace_disp1_pc), .trace_disp1_instr(trace_disp1_instr),
    .trace_issue_valid(trace_issue_valid), .trace_issue_seq(trace_issue_seq),
    .trace_cdb_valid(trace_cdb_valid), .trace_cdb_seq(trace_cdb_seq),
    .trace_flush_valid(trace_flush_valid), .trace_flush_after_seq(trace_flush_after_seq),
    .halted(halted), .x14(x14),
    .stat_rob_full_stall(st_rob_u), .stat_rs_full_stall(st_rs_u),
    .stat_lsq_full_stall(st_lsq_u), .stat_cp_full_stall(st_cp_u),
    .stat_stlf_hit(st_stlf),
    .stat_br_total_cnt(st_br_tot), .stat_br_mispred_cnt(st_br_misp),
    .stat_br_fast_squash(st_fsq_u), .stat_war_waw_cnt(st_ww_u),
    .stat_div_overlap_cyc(st_div_u), .stat_dcache_miss_overlap_cyc(st_dcm_u),
    .stat_dual_disp(st_dd_u), .stat_dual_cdb(st_dc_u), .stat_dual_commit(st_dcomm),
    .stat_ex_byp_cnt(st_ex_byp), .stat_ras_pred_cnt(st_ras_p), .stat_ras_mispred_cnt(st_ras_m),
    .stat_replay_flush(st_replay), .stat_amo_commit(st_amo), .stat_fence_commit(st_fence)
);

icache_2w #(
    .AWIDTH(AWIDTH),
    .NUM_SETS(ICACHE_SETS),
    .ENABLE_PREFETCH(ENABLE_PREFETCH)
) u_icache (
    .clk(clk), .reset_n(reset_n),
    .req_valid(imem_req_valid), .req_pc(imem_pc0),
    .flush(ic_flush), .invalidate_all(ic_invalidate_all),
    .resp_valid0(imem_valid0), .resp_instr0(imem_instr0),
    .resp_valid1(imem_valid1), .resp_instr1(imem_instr1),
    .mem_rd_valid(ic_rd_valid), .mem_rd_ready(ic_rd_ready),
    .mem_rd_line_addr(ic_rd_laddr),
    .mem_rresp_valid(ic_rresp_valid), .mem_rresp_data(ic_rresp_data),
    .icache_idle(ic_idle_u),
    .pmu_ic_hit(ic_hit_p), .pmu_ic_miss(ic_miss_p),
    .pmu_ic_pf_hit(ic_pf_hit_p), .pmu_ic_stall(ic_stall_p)
);

dcache_nb #(
    .AWIDTH(AWIDTH),
    .NUM_SETS(DCACHE_SETS),
    .TAG_W(5),
    .ID_W(ID_W)
) u_dcache (
    .clk(clk), .reset_n(reset_n),
    .req_valid(dc_req_valid), .req_ready(dc_req_ready),
    .req_is_write(dc_req_is_write), .req_amo_op(dc_req_amo_op),
    .req_addr(dc_req_addr), .req_wdata(dc_req_wdata), .req_wstrb(dc_req_wstrb),
    .req_id(dc_req_id), .req_rob_tag(dc_req_rob_tag),
    .hit_resp_valid(dc_hit_valid), .hit_resp_id(dc_hit_id), .hit_resp_rdata(dc_hit_rdata),
    .fill_resp_valid(dc_fill_valid), .fill_resp_id(dc_fill_id), .fill_resp_rdata(dc_fill_rdata),
    .flush(dc_flush), .squash_valid(dc_squash_valid),
    .squash_tag(dc_squash_tag), .rob_head(dc_rob_head),
    .clean_all_req(dc_clean_all_req), .clean_all_done(dc_clean_all_done),
    .ext_resv_inv_valid(ext_snoop_inv_valid), .ext_resv_inv_laddr(ext_snoop_inv_line),
    .mem_rd_valid(dc_rd_valid), .mem_rd_ready(dc_rd_ready),
    .mem_rd_line_addr(dc_rd_laddr), .mem_rd_id(dc_rd_id),
    .mem_rresp_valid(dc_rresp_valid), .mem_rresp_id(dc_rresp_id), .mem_rresp_data(dc_rresp_data),
    .mem_wr_valid(mem_wr_valid), .mem_wr_ready(mem_wr_ready),
    .mem_wr_line_addr(mem_wr_laddr), .mem_wr_data(mem_wr_data),
    .dcache_idle(dc_idle_u), .dcache_writes_idle(dc_writes_idle), .dcache_any_miss(dc_any_miss),
    .cache_hit(dc_hit_p), .cache_miss(dc_miss_p), .cache_wb(dc_wb_p),
    .cache_hum(dc_hum_p), .cache_mum_cyc(dc_mum_p), .cache_mshr_coalesce(dc_coal_p)
);

mem_arb_2to1 #(.LADDR_W(LADDR_W)) u_arb (
    .clk(clk), .reset_n(reset_n),
    .dc_rd_valid(dc_rd_valid), .dc_rd_ready(dc_rd_ready),
    .dc_rd_line_addr(dc_rd_laddr), .dc_rd_id(dc_rd_id),
    .dc_rresp_valid(dc_rresp_valid), .dc_rresp_id(dc_rresp_id), .dc_rresp_data(dc_rresp_data),
    .ic_rd_valid(ic_rd_valid), .ic_rd_ready(ic_rd_ready),
    .ic_rd_line_addr(ic_rd_laddr),
    .ic_rresp_valid(ic_rresp_valid), .ic_rresp_data(ic_rresp_data),
    .mem_rd_valid(mem_rd_valid), .mem_rd_ready(mem_rd_ready),
    .mem_rd_line_addr(mem_rd_laddr), .mem_rd_id(mem_rd_id),
    .mem_rresp_valid(mem_rresp_valid), .mem_rresp_id(mem_rresp_id), .mem_rresp_data(mem_rresp_data)
);

main_memory_nb #(
    .LINE_BITS(LINE_BITS),
    .LATENCY(MEM_LATENCY),
    .ID_W(2)
) u_mem (
    .clk(clk), .rst_n(reset_n),
    .mem_rd_valid(mem_rd_valid), .mem_rd_ready(mem_rd_ready),
    .mem_rd_line_addr(mem_rd_laddr[LINE_BITS-1:0]), .mem_rd_id(mem_rd_id),
    .mem_rresp_valid(mem_rresp_valid), .mem_rresp_id(mem_rresp_id), .mem_rresp_data(mem_rresp_data),
    .mem_wr_valid(mem_wr_valid), .mem_wr_ready(mem_wr_ready),
    .mem_wr_line_addr(mem_wr_laddr[LINE_BITS-1:0]), .mem_wr_data(mem_wr_data)
);

wire [31:0] commit_inc = {31'd0, commit0_valid} + {31'd0, commit1_valid};

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        pmu_cycles          <= 32'd0;
        pmu_instret         <= 32'd0;
        pmu_ic_hits         <= 32'd0;
        pmu_ic_misses       <= 32'd0;
        pmu_ic_pf_hits      <= 32'd0;
        pmu_ic_stall_cyc    <= 32'd0;
        pmu_dcache_hits     <= 32'd0;
        pmu_dcache_misses   <= 32'd0;
        pmu_dcache_wbs      <= 32'd0;
        pmu_dcache_hum      <= 32'd0;
        pmu_dcache_mum_cyc  <= 32'd0;
        pmu_mshr_coalesce   <= 32'd0;
        pmu_stlf_hits       <= 32'd0;
        pmu_br_total        <= 32'd0;
        pmu_br_mispred      <= 32'd0;
        pmu_ex_bypass_hits  <= 32'd0;
        pmu_ras_preds       <= 32'd0;
        pmu_ras_mispreds    <= 32'd0;
        pmu_dual_commit_cyc <= 32'd0;
        pmu_replay_flushes  <= 32'd0;
        pmu_amo_commits     <= 32'd0;
        pmu_fence_commits   <= 32'd0;
    end else if (!halted) begin
        pmu_cycles          <= pmu_cycles + 32'd1;
        pmu_instret         <= pmu_instret + commit_inc;
        if (ic_hit_p)    pmu_ic_hits         <= pmu_ic_hits + 32'd1;
        if (ic_miss_p)   pmu_ic_misses       <= pmu_ic_misses + 32'd1;
        if (ic_pf_hit_p) pmu_ic_pf_hits      <= pmu_ic_pf_hits + 32'd1;
        if (ic_stall_p)  pmu_ic_stall_cyc    <= pmu_ic_stall_cyc + 32'd1;
        if (dc_hit_p)    pmu_dcache_hits     <= pmu_dcache_hits + 32'd1;
        if (dc_miss_p)   pmu_dcache_misses   <= pmu_dcache_misses + 32'd1;
        if (dc_wb_p)     pmu_dcache_wbs      <= pmu_dcache_wbs + 32'd1;
        if (dc_hum_p)    pmu_dcache_hum      <= pmu_dcache_hum + 32'd1;
        if (dc_mum_p)    pmu_dcache_mum_cyc  <= pmu_dcache_mum_cyc + 32'd1;
        if (dc_coal_p)   pmu_mshr_coalesce   <= pmu_mshr_coalesce + 32'd1;
        if (st_stlf)     pmu_stlf_hits       <= pmu_stlf_hits + 32'd1;
        pmu_br_total        <= pmu_br_total + {30'd0, st_br_tot};
        pmu_br_mispred      <= pmu_br_mispred + {30'd0, st_br_misp};
        pmu_ex_bypass_hits  <= pmu_ex_bypass_hits + {28'd0, st_ex_byp};
        pmu_ras_preds       <= pmu_ras_preds + {30'd0, st_ras_p};
        pmu_ras_mispreds    <= pmu_ras_mispreds + {30'd0, st_ras_m};
        if (st_dcomm)    pmu_dual_commit_cyc <= pmu_dual_commit_cyc + 32'd1;
        if (st_replay)   pmu_replay_flushes  <= pmu_replay_flushes + 32'd1;
        if (st_amo)      pmu_amo_commits     <= pmu_amo_commits + 32'd1;
        if (st_fence)    pmu_fence_commits   <= pmu_fence_commits + 32'd1;
    end
end

endmodule
