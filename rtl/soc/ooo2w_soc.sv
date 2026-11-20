`timescale 1ns / 1ps

// ============================================================================
// 2-Wide Superscalar Tomasulo RV32IM + Non-Blocking 2-MSHR D-Cache SoC
// (Upgrades U1 + U2 + U3: 4 KB default / 16 KB parameterizable 2-way 16 B-line
//  non-blocking D-Cache with 2 MSHRs + 2-Entry WBB + Pipelined 128b Memory)
// ============================================================================
module ooo2w_soc #(
    parameter integer IMEM_WORDS  = 1024,
    parameter integer MEM_LATENCY = 4,
    parameter integer DCACHE_SETS = 128, // 128 -> 4 KB, 512 -> 16 KB
    parameter         HEX_FILE    = ""
) (
    input  wire         clk,
    input  wire         reset_n,

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

    // 2-Wide Out-of-Order proof monitor hooks
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

    // Superscalar + Non-Blocking D-Cache PMU counters
    output reg  [31:0]  pmu_cycles,
    output reg  [31:0]  pmu_instret,
    output reg  [31:0]  pmu_dcache_hits,
    output reg  [31:0]  pmu_dcache_misses,
    output reg  [31:0]  pmu_dcache_wbs,
    output reg  [31:0]  pmu_dcache_hum,
    output reg  [31:0]  pmu_dcache_mum_cyc,
    output reg  [31:0]  pmu_mshr_coalesce,
    output reg  [31:0]  pmu_stlf_hits,
    output reg  [31:0]  pmu_br_total,
    output reg  [31:0]  pmu_br_mispred,
    output reg  [31:0]  pmu_br_fast_squash,
    output reg  [31:0]  pmu_stall_rob_full,
    output reg  [31:0]  pmu_stall_rs_full,
    output reg  [31:0]  pmu_stall_lsq_full,
    output reg  [31:0]  pmu_stall_cp_full,
    output reg  [31:0]  pmu_war_waw_renamed,
    output reg  [31:0]  pmu_div_overlap_cyc,
    output reg  [31:0]  pmu_dcache_miss_overlap_cyc,
    output reg  [31:0]  pmu_dual_disp_cyc,
    output reg  [31:0]  pmu_dual_cdb_cyc,
    output reg  [31:0]  pmu_dual_commit_cyc,
    output reg  [31:0]  pmu_ex_bypass_hits,
    output reg  [31:0]  pmu_ras_preds,
    output reg  [31:0]  pmu_ras_mispreds
);

wire [31:0]  imem_pc0, imem_instr0, imem_pc1, imem_instr1;
wire         imem_req_u, ic_flush_u, ic_inv_u;

wire         dc_req_valid, dc_req_ready, dc_req_is_write;
wire [3:0]   dc_req_amo_op;
wire [15:0]  dc_req_addr;
wire [31:0]  dc_req_wdata;
wire [3:0]   dc_req_wstrb;
wire [6:0]   dc_req_id;
wire [4:0]   dc_req_rob_tag;

wire         dc_hit_valid, dc_fill_valid;
wire [6:0]   dc_hit_id, dc_fill_id;
wire [31:0]  dc_hit_rdata, dc_fill_rdata;

wire         dc_flush, dc_squash_valid, dc_clean_req, dc_clean_done;
wire         dc_idle_u, dc_writes_idle, dc_any_miss;
wire [4:0]   dc_squash_tag, dc_rob_head;

wire         dcache_hit, dcache_miss, dcache_wb, dcache_hum, dcache_mum_cyc, dcache_coalesce;

wire         mem_rd_valid, mem_rd_ready, mem_rd_id;
wire [11:0]  mem_rd_line_addr;
wire         mem_rresp_valid, mem_rresp_id;
wire [127:0] mem_rresp_data;
wire         mem_wr_valid, mem_wr_ready;
wire [11:0]  mem_wr_line_addr;
wire [127:0] mem_wr_data;

wire         stat_rob_full, stat_rs_full, stat_lsq_full, stat_cp_full;
wire         stat_stlf_hit, stat_br_fast_squash, stat_div_ovl, stat_dcm_ovl;
wire         stat_dual_disp, stat_dual_cdb, stat_dual_commit;
wire         stat_replay_u, stat_amo_u, stat_fence_u;
wire [1:0]   stat_br_total_cnt, stat_br_mispred_cnt, stat_war_waw_cnt;
wire [3:0]   stat_ex_byp_cnt;
wire [1:0]   stat_ras_pred_cnt, stat_ras_mispred_cnt;

instr_mem2w #(
    .WORDS(IMEM_WORDS),
    .HEX_FILE(HEX_FILE)
) u_imem (
    .pc0(imem_pc0), .instr0(imem_instr0),
    .pc1(imem_pc1), .instr1(imem_instr1)
);

ooo2w_core #(
    .CORE_ID(32'd0),
    .RESET_PC(32'd0),
    .AWIDTH(16),
    .ID_W(7)
) u_core (
    .clk(clk), .reset_n(reset_n),
    .imem_req_valid(imem_req_u),
    .imem_pc0(imem_pc0), .imem_valid0(1'b1), .imem_instr0(imem_instr0),
    .imem_pc1(imem_pc1), .imem_valid1(1'b1), .imem_instr1(imem_instr1),
    .ic_flush(ic_flush_u), .ic_invalidate_all(ic_inv_u),
    .dc_req_valid(dc_req_valid), .dc_req_ready(dc_req_ready),
    .dc_req_is_write(dc_req_is_write), .dc_req_amo_op(dc_req_amo_op),
    .dc_req_addr(dc_req_addr), .dc_req_wdata(dc_req_wdata), .dc_req_wstrb(dc_req_wstrb),
    .dc_req_id(dc_req_id), .dc_req_rob_tag(dc_req_rob_tag),
    .dc_hit_valid(dc_hit_valid), .dc_hit_id(dc_hit_id), .dc_hit_rdata(dc_hit_rdata),
    .dc_fill_valid(dc_fill_valid), .dc_fill_id(dc_fill_id), .dc_fill_rdata(dc_fill_rdata),
    .dc_flush(dc_flush), .dc_squash_valid(dc_squash_valid),
    .dc_squash_tag(dc_squash_tag), .dc_rob_head(dc_rob_head),
    .dc_clean_all_req(dc_clean_req), .dc_clean_all_done(dc_clean_done),
    .dc_writes_idle(dc_writes_idle), .dc_any_miss(dc_any_miss),
    .coh_snoop_inv_valid(1'b0), .coh_snoop_inv_line(12'd0),
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
    .stat_rob_full_stall(stat_rob_full), .stat_rs_full_stall(stat_rs_full),
    .stat_lsq_full_stall(stat_lsq_full), .stat_cp_full_stall(stat_cp_full),
    .stat_stlf_hit(stat_stlf_hit), .stat_br_total_cnt(stat_br_total_cnt),
    .stat_br_mispred_cnt(stat_br_mispred_cnt), .stat_br_fast_squash(stat_br_fast_squash),
    .stat_war_waw_cnt(stat_war_waw_cnt), .stat_div_overlap_cyc(stat_div_ovl),
    .stat_dcache_miss_overlap_cyc(stat_dcm_ovl), .stat_dual_disp(stat_dual_disp),
    .stat_dual_cdb(stat_dual_cdb), .stat_dual_commit(stat_dual_commit),
    .stat_ex_byp_cnt(stat_ex_byp_cnt),
    .stat_ras_pred_cnt(stat_ras_pred_cnt), .stat_ras_mispred_cnt(stat_ras_mispred_cnt),
    .stat_replay_flush(stat_replay_u), .stat_amo_commit(stat_amo_u), .stat_fence_commit(stat_fence_u)
);

dcache_nb #(
    .AWIDTH(16),
    .NUM_SETS(DCACHE_SETS),
    .TAG_W(5),
    .ID_W(7)
) u_dcache (
    .clk(clk), .reset_n(reset_n),
    .req_valid(dc_req_valid), .req_ready(dc_req_ready),
    .req_is_write(dc_req_is_write), .req_amo_op(dc_req_amo_op),
    .req_addr(dc_req_addr), .req_wdata(dc_req_wdata), .req_wstrb(dc_req_wstrb),
    .req_id(dc_req_id), .req_rob_tag(dc_req_rob_tag),
    .hit_resp_valid(dc_hit_valid), .hit_resp_id(dc_hit_id), .hit_resp_rdata(dc_hit_rdata),
    .fill_resp_valid(dc_fill_valid), .fill_resp_id(dc_fill_id), .fill_resp_rdata(dc_fill_rdata),
    .flush(dc_flush), .squash_valid(dc_squash_valid), .squash_tag(dc_squash_tag), .rob_head(dc_rob_head),
    .clean_all_req(dc_clean_req), .clean_all_done(dc_clean_done),
    .ext_resv_inv_valid(1'b0), .ext_resv_inv_laddr(12'd0),
    .mem_rd_valid(mem_rd_valid), .mem_rd_ready(mem_rd_ready),
    .mem_rd_line_addr(mem_rd_line_addr), .mem_rd_id(mem_rd_id),
    .mem_rresp_valid(mem_rresp_valid), .mem_rresp_id(mem_rresp_id), .mem_rresp_data(mem_rresp_data),
    .mem_wr_valid(mem_wr_valid), .mem_wr_ready(mem_wr_ready),
    .mem_wr_line_addr(mem_wr_line_addr), .mem_wr_data(mem_wr_data),
    .dcache_idle(dc_idle_u), .dcache_writes_idle(dc_writes_idle), .dcache_any_miss(dc_any_miss),
    .cache_hit(dcache_hit), .cache_miss(dcache_miss), .cache_wb(dcache_wb),
    .cache_hum(dcache_hum), .cache_mum_cyc(dcache_mum_cyc), .cache_mshr_coalesce(dcache_coalesce)
);

main_memory_nb #(
    .LINE_BITS(12),
    .LATENCY(MEM_LATENCY)
) u_main_mem (
    .clk(clk), .rst_n(reset_n),
    .mem_rd_valid(mem_rd_valid), .mem_rd_ready(mem_rd_ready),
    .mem_rd_line_addr(mem_rd_line_addr), .mem_rd_id(mem_rd_id),
    .mem_rresp_valid(mem_rresp_valid), .mem_rresp_id(mem_rresp_id), .mem_rresp_data(mem_rresp_data),
    .mem_wr_valid(mem_wr_valid), .mem_wr_ready(mem_wr_ready),
    .mem_wr_line_addr(mem_wr_line_addr), .mem_wr_data(mem_wr_data)
);

wire [31:0] commit_inc = commit1_valid ? 32'd2 : (commit0_valid ? 32'd1 : 32'd0);

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        pmu_cycles                  <= 32'd0;
        pmu_instret                 <= 32'd0;
        pmu_dcache_hits             <= 32'd0;
        pmu_dcache_misses           <= 32'd0;
        pmu_dcache_wbs              <= 32'd0;
        pmu_dcache_hum              <= 32'd0;
        pmu_dcache_mum_cyc          <= 32'd0;
        pmu_mshr_coalesce           <= 32'd0;
        pmu_stlf_hits               <= 32'd0;
        pmu_br_total                <= 32'd0;
        pmu_br_mispred              <= 32'd0;
        pmu_br_fast_squash          <= 32'd0;
        pmu_stall_rob_full          <= 32'd0;
        pmu_stall_rs_full           <= 32'd0;
        pmu_stall_lsq_full          <= 32'd0;
        pmu_stall_cp_full           <= 32'd0;
        pmu_war_waw_renamed         <= 32'd0;
        pmu_div_overlap_cyc         <= 32'd0;
        pmu_dcache_miss_overlap_cyc <= 32'd0;
        pmu_dual_disp_cyc           <= 32'd0;
        pmu_dual_cdb_cyc            <= 32'd0;
        pmu_dual_commit_cyc         <= 32'd0;
        pmu_ex_bypass_hits          <= 32'd0;
        pmu_ras_preds               <= 32'd0;
        pmu_ras_mispreds            <= 32'd0;
    end else if (!halted) begin
        pmu_cycles          <= pmu_cycles + 32'd1;
        pmu_instret         <= pmu_instret + commit_inc;
        pmu_br_total        <= pmu_br_total + {30'd0, stat_br_total_cnt};
        pmu_br_mispred      <= pmu_br_mispred + {30'd0, stat_br_mispred_cnt};
        pmu_war_waw_renamed <= pmu_war_waw_renamed + {30'd0, stat_war_waw_cnt};
        pmu_ex_bypass_hits  <= pmu_ex_bypass_hits + {28'd0, stat_ex_byp_cnt};
        pmu_ras_preds       <= pmu_ras_preds + {30'd0, stat_ras_pred_cnt};
        pmu_ras_mispreds    <= pmu_ras_mispreds + {30'd0, stat_ras_mispred_cnt};
        if (dcache_hit)          pmu_dcache_hits             <= pmu_dcache_hits + 32'd1;
        if (dcache_miss)         pmu_dcache_misses           <= pmu_dcache_misses + 32'd1;
        if (dcache_wb)           pmu_dcache_wbs              <= pmu_dcache_wbs + 32'd1;
        if (dcache_hum)          pmu_dcache_hum              <= pmu_dcache_hum + 32'd1;
        if (dcache_mum_cyc)      pmu_dcache_mum_cyc          <= pmu_dcache_mum_cyc + 32'd1;
        if (dcache_coalesce)     pmu_mshr_coalesce           <= pmu_mshr_coalesce + 32'd1;
        if (stat_stlf_hit)       pmu_stlf_hits               <= pmu_stlf_hits + 32'd1;
        if (stat_br_fast_squash) pmu_br_fast_squash          <= pmu_br_fast_squash + 32'd1;
        if (stat_rob_full)       pmu_stall_rob_full          <= pmu_stall_rob_full + 32'd1;
        if (stat_rs_full)        pmu_stall_rs_full           <= pmu_stall_rs_full + 32'd1;
        if (stat_lsq_full)       pmu_stall_lsq_full          <= pmu_stall_lsq_full + 32'd1;
        if (stat_cp_full)        pmu_stall_cp_full           <= pmu_stall_cp_full + 32'd1;
        if (stat_div_ovl)        pmu_div_overlap_cyc         <= pmu_div_overlap_cyc + 32'd1;
        if (stat_dcm_ovl)        pmu_dcache_miss_overlap_cyc <= pmu_dcache_miss_overlap_cyc + 32'd1;
        if (stat_dual_disp)      pmu_dual_disp_cyc           <= pmu_dual_disp_cyc + 32'd1;
        if (stat_dual_cdb)       pmu_dual_cdb_cyc            <= pmu_dual_cdb_cyc + 32'd1;
        if (stat_dual_commit)    pmu_dual_commit_cyc         <= pmu_dual_commit_cyc + 32'd1;
    end
end

endmodule
