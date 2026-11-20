`timescale 1ns / 1ps

// ============================================================================
// Unit Testbench for Phase 1 LSQ2W Features:
//   1. Speculative Load Snoop-Invalidation Replay (lq_head0_replay & lq_head1_replay)
//   2. STLF-satisfied Load Immunity to External Coherence Snoop Invalidation
//   3. Ordering Barrier Gate (younger load blocked behind older FENCE/AMO barrier)
//   4. ROB-Head Non-Speculative RV32A AMO Execution & RVFI mem_wdata Calculation
// ============================================================================
module tb_lsq2w_replay;

`include "rv32ima_defs.vh"

localparam integer TAG_W  = 5;
localparam integer ID_W   = 7;
localparam integer AWIDTH = 20;

reg                 clk, reset_n, flush, squash_valid;
reg  [TAG_W-1:0]    squash_tag, rob_head;
reg                 barrier_valid;
reg  [TAG_W-1:0]    barrier_tag;

reg                 disp_valid, disp_is_store;
wire                disp_ready_load, disp_ready_store;
reg  [TAG_W-1:0]    disp_rob_tag;
reg  [31:0]         disp_seq_id, disp_imm, disp_vj, disp_vk;
reg  [1:0]          disp_mem_size;
reg                 disp_mem_unsigned, disp_qj_valid, disp_qk_valid;
reg  [TAG_W-1:0]    disp_qj, disp_qk;

reg                 amo_req_valid;
reg  [3:0]          amo_req_op;
reg  [TAG_W-1:0]    amo_req_tag;
reg  [31:0]         amo_req_seq, amo_req_addr, amo_req_rs2;
reg                 dc_writes_idle;

reg                 rob_commit_store, rob_commit0_load, rob_commit1_load;
reg  [TAG_W-1:0]    rob_commit0_tag, rob_commit1_tag;

reg                 coh_snoop_inv_valid;
reg  [AWIDTH-5:0]   coh_snoop_inv_line;
wire                lq_head0_replay, lq_head1_replay;

wire                lsu_cdb_valid, lsu_cdb_mem_we;
reg                 lsu_cdb_grant;
wire [TAG_W-1:0]    lsu_cdb_tag;
wire [31:0]         lsu_cdb_seq, lsu_cdb_val, lsu_cdb_mem_addr, lsu_cdb_mem_wdata;
wire [3:0]          lsu_cdb_mem_wstrb;

wire                dc_req_valid, dc_req_is_write, dc_req_is_mmio;
reg                 dc_req_ready;
wire [3:0]          dc_req_amo_op;
wire [AWIDTH-1:0]   dc_req_addr;
wire [31:0]         dc_req_wdata;
wire [3:0]          dc_req_wstrb;
wire [ID_W-1:0]     dc_req_id;
wire [TAG_W-1:0]    dc_req_rob_tag;

reg                 dc_hit_valid, dc_fill_valid;
reg  [ID_W-1:0]     dc_hit_id, dc_fill_id;
reg  [31:0]         dc_hit_rdata, dc_fill_rdata;

wire                sq_empty, trace_iv, stat_stlf;
wire [31:0]         trace_iseq;
wire [1:0]          stat_exb;

lsq2w #(
    .LQ_DEPTH(8), .SQ_DEPTH(8), .TAG_W(TAG_W), .ID_W(ID_W), .AWIDTH(AWIDTH)
) dut (
    .clk(clk), .reset_n(reset_n), .flush(flush),
    .squash_valid(squash_valid), .squash_tag(squash_tag), .rob_head(rob_head),
    .barrier_valid(barrier_valid), .barrier_tag(barrier_tag),
    .disp_valid(disp_valid), .disp_is_store(disp_is_store),
    .disp_ready_load(disp_ready_load), .disp_ready_store(disp_ready_store),
    .disp_rob_tag(disp_rob_tag), .disp_seq_id(disp_seq_id),
    .disp_mem_size(disp_mem_size), .disp_mem_unsigned(disp_mem_unsigned),
    .disp_imm(disp_imm), .disp_vj(disp_vj), .disp_qj(disp_qj), .disp_qj_valid(disp_qj_valid),
    .disp_vk(disp_vk), .disp_qk(disp_qk), .disp_qk_valid(disp_qk_valid),
    .amo_req_valid(amo_req_valid), .amo_req_op(amo_req_op),
    .amo_req_tag(amo_req_tag), .amo_req_seq(amo_req_seq),
    .amo_req_addr(amo_req_addr), .amo_req_rs2(amo_req_rs2),
    .dc_writes_idle(dc_writes_idle),
    .cdb0_valid(1'b0), .cdb0_tag(5'd0), .cdb0_val(32'd0),
    .cdb1_valid(1'b0), .cdb1_tag(5'd0), .cdb1_val(32'd0),
    .wk0_valid(1'b0), .wk0_tag(5'd0), .wk1_valid(1'b0), .wk1_tag(5'd0),
    .byp_alu0_val(32'd0), .byp_alu1_val(32'd0),
    .rob_commit_store(rob_commit_store),
    .rob_commit0_load(rob_commit0_load), .rob_commit0_tag(rob_commit0_tag),
    .rob_commit1_load(rob_commit1_load), .rob_commit1_tag(rob_commit1_tag),
    .coh_snoop_inv_valid(coh_snoop_inv_valid), .coh_snoop_inv_line(coh_snoop_inv_line),
    .lq_head0_replay(lq_head0_replay), .lq_head1_replay(lq_head1_replay),
    .lsu_cdb_valid(lsu_cdb_valid), .lsu_cdb_grant(lsu_cdb_grant),
    .lsu_cdb_tag(lsu_cdb_tag), .lsu_cdb_seq(lsu_cdb_seq), .lsu_cdb_val(lsu_cdb_val),
    .lsu_cdb_mem_we(lsu_cdb_mem_we), .lsu_cdb_mem_addr(lsu_cdb_mem_addr),
    .lsu_cdb_mem_wstrb(lsu_cdb_mem_wstrb), .lsu_cdb_mem_wdata(lsu_cdb_mem_wdata),
    .dc_req_valid(dc_req_valid), .dc_req_ready(dc_req_ready),
    .dc_req_is_write(dc_req_is_write), .dc_req_is_mmio(dc_req_is_mmio),
    .dc_req_amo_op(dc_req_amo_op),
    .dc_req_addr(dc_req_addr), .dc_req_wdata(dc_req_wdata), .dc_req_wstrb(dc_req_wstrb),
    .dc_req_id(dc_req_id), .dc_req_rob_tag(dc_req_rob_tag),
    .dc_hit_valid(dc_hit_valid), .dc_hit_id(dc_hit_id), .dc_hit_rdata(dc_hit_rdata),
    .dc_fill_valid(dc_fill_valid), .dc_fill_id(dc_fill_id), .dc_fill_rdata(dc_fill_rdata),
    .sq_empty(sq_empty), .trace_issue_valid(trace_iv), .trace_issue_seq(trace_iseq),
    .stat_stlf_hit(stat_stlf), .stat_ex_byp_cnt(stat_exb)
);

initial clk = 1'b0;
always #5 clk = ~clk;

integer errors, checks;
reg [ID_W-1:0] saved_id;

task step_clk;
    begin
        @(posedge clk);
        #1;
    end
endtask

task clear_inputs;
    begin
        flush = 1'b0; squash_valid = 1'b0; squash_tag = 5'd0;
        barrier_valid = 1'b0; barrier_tag = 5'd0;
        disp_valid = 1'b0; disp_is_store = 1'b0;
        disp_rob_tag = 5'd0; disp_seq_id = 32'd0;
        disp_mem_size = MEM_WORD; disp_mem_unsigned = 1'b0;
        disp_imm = 32'd0; disp_vj = 32'd0; disp_qj = 5'd0; disp_qj_valid = 1'b0;
        disp_vk = 32'd0; disp_qk = 5'd0; disp_qk_valid = 1'b0;
        amo_req_valid = 1'b0; amo_req_op = AMO_NONE; amo_req_tag = 5'd0;
        amo_req_seq = 32'd0; amo_req_addr = 32'd0; amo_req_rs2 = 32'd0;
        dc_writes_idle = 1'b1;
        rob_commit_store = 1'b0; rob_commit0_load = 1'b0; rob_commit0_tag = 5'd0;
        rob_commit1_load = 1'b0; rob_commit1_tag = 5'd0;
        coh_snoop_inv_valid = 1'b0; coh_snoop_inv_line = 16'd0;
        lsu_cdb_grant = 1'b1; dc_req_ready = 1'b1;
        dc_hit_valid = 1'b0; dc_hit_id = 7'd0; dc_hit_rdata = 32'd0;
        dc_fill_valid = 1'b0; dc_fill_id = 7'd0; dc_fill_rdata = 32'd0;
    end
endtask

task check(input cond, input [511:0] msg);
    begin
        checks = checks + 1;
        if (!cond) begin
            $display("FAIL [tb_lsq2w_replay]: %0s", msg);
            errors = errors + 1;
        end
    end
endtask

initial begin
    errors   = 0;
    checks   = 0;
    reset_n  = 1'b0;
    rob_head = 5'd0;
    clear_inputs();
    repeat (2) step_clk();
    reset_n = 1'b1;
    #1;

    // ------------------------------------------------------------------------
    // TEST 1: Speculative Load Snoop-Invalidation Replay Detection (head1 -> head0!)
    //   Older op at tag=0 (not a load), speculative load at tag=1 reads 0x00100.
    //   Load completes on CDB0 -> stays in LQ! External snoop invalidates line 0x010.
    //   Must assert lq_head1_replay when rob_head=0, and lq_head0_replay when rob_head=1!
    // ------------------------------------------------------------------------
    rob_head      = 5'd0;
    disp_valid    = 1'b1;
    disp_is_store = 1'b0;
    disp_rob_tag  = 5'd1;
    disp_seq_id   = 32'd10;
    disp_vj       = 32'h0000_0104; // line 0x0010
    step_clk();
    disp_valid    = 1'b0;

    // Load issues to D-cache now
    check(dc_req_valid && !dc_req_is_write && (dc_req_addr == 20'h00104), "T1: Speculative load issued to D$");
    saved_id = dc_req_id;
    step_clk();

    // D-cache returns hit data 0x1111_2222
    dc_hit_valid = 1'b1;
    dc_hit_id    = saved_id;
    dc_hit_rdata = 32'h1111_2222;
    step_clk();
    dc_hit_valid = 1'b0;

    // Load broadcasts on CDB0 -> stays retained in LQ until ROB commit!
    check(lsu_cdb_valid && (lsu_cdb_tag == 5'd1) && (lsu_cdb_val == 32'h1111_2222), "T1: Load completed on CDB0");
    step_clk();
    check(!lsu_cdb_valid && !lq_head0_replay && !lq_head1_replay, "T1: Retained in LQ, clean before snoop");

    // External snoop invalidates line 0x0010!
    coh_snoop_inv_valid = 1'b1;
    coh_snoop_inv_line  = 16'h0010;
    #1;
    check(!lq_head0_replay && lq_head1_replay, "T1: Combinational lq_head1_replay on snoop!");
    step_clk();
    coh_snoop_inv_valid = 1'b0;

    // Advance rob_head to tag=1 -> lq_head0_replay MUST be asserted!
    rob_head = 5'd1;
    #1;
    check(lq_head0_replay, "T1: Sticky lq_head0_replay when poisoned load hits head!");

    // Flush (ROB triggers replay flush) -> LQ clears cleanly!
    flush = 1'b1;
    step_clk();
    flush = 1'b0;
    check(!lq_head0_replay && !lq_head1_replay, "T1: Replay flush clears poisoned LQ cleanly");

    // ------------------------------------------------------------------------
    // TEST 2: STLF Load Immunity (reads own uncommitted store -> external snoop ignored)
    // ------------------------------------------------------------------------
    rob_head      = 5'd4;
    disp_valid    = 1'b1;
    disp_is_store = 1'b1;
    disp_rob_tag  = 5'd4;
    disp_seq_id   = 32'd20;
    disp_vj       = 32'h0000_0200; // line 0x0020
    disp_vk       = 32'hABCD_1234;
    step_clk();

    disp_is_store = 1'b0;
    disp_rob_tag  = 5'd5;
    disp_seq_id   = 32'd21;
    disp_vj       = 32'h0000_0200;
    step_clk();
    disp_valid    = 1'b0;

    // STLF completes load in 1 cycle
    check(stat_stlf, "T2: STLF hit on local store");
    step_clk(); // CDB0 broadcasts load
    step_clk(); // Store completes on CDB0

    // External snoop invalidates line 0x0020 -> STLF load MUST remain immune!
    coh_snoop_inv_valid = 1'b1;
    coh_snoop_inv_line  = 16'h0020;
    #1;
    check(!lq_head1_replay, "T2: STLF load immune to ext snoop");
    step_clk();
    coh_snoop_inv_valid = 1'b0;

    // Clean up T2
    flush = 1'b1;
    step_clk();
    flush = 1'b0;

    // ------------------------------------------------------------------------
    // TEST 3: Ordering Barrier Gate (load younger than FENCE at tag=10 must wait!)
    // ------------------------------------------------------------------------
    rob_head      = 5'd10;
    barrier_valid = 1'b1;
    barrier_tag   = 5'd10;
    disp_valid    = 1'b1;
    disp_is_store = 1'b0;
    disp_rob_tag  = 5'd11;
    disp_seq_id   = 32'd30;
    disp_vj       = 32'h0000_0300;
    step_clk();
    disp_valid    = 1'b0;
    check(!dc_req_valid, "T3: Load younger than barrier blocked from D$");

    // Barrier commits -> clear barrier_valid -> load immediately issues!
    barrier_valid = 1'b0;
    #1;
    check(dc_req_valid && (dc_req_addr == 20'h00300), "T3: Load issues immediately after barrier retires");
    flush = 1'b1;
    step_clk();
    flush = 1'b0;

    // ------------------------------------------------------------------------
    // TEST 4: ROB-Head Non-Speculative RV32A AMOADD.W Execution & RVFI mem_wdata
    // ------------------------------------------------------------------------
    rob_head      = 5'd15;
    amo_req_valid = 1'b1;
    amo_req_op    = AMO_ADD;
    amo_req_tag   = 5'd15;
    amo_req_seq   = 32'd40;
    amo_req_addr  = 32'h0000_0400;
    amo_req_rs2   = 32'd7;
    #1;
    check(dc_req_valid && dc_req_is_write && (dc_req_amo_op == AMO_ADD) && dc_req_id[ID_W-1],
          "T4: AMOADD issued with reserved atomic ID bit");
    saved_id = dc_req_id;
    step_clk();
    amo_req_valid = 1'b0;

    // D-Cache returns old_val = 15
    dc_hit_valid = 1'b1;
    dc_hit_id    = saved_id;
    dc_hit_rdata = 32'd15;
    step_clk();
    dc_hit_valid = 1'b0;

    check(lsu_cdb_valid && (lsu_cdb_tag == 5'd15) && (lsu_cdb_val == 32'd15) &&
          lsu_cdb_mem_we && (lsu_cdb_mem_wstrb == 4'hF) && (lsu_cdb_mem_wdata == 32'd22),
          "T4: AMOADD CDB0 old=15, RVFI mem_wdata=22");
    step_clk();

    if (errors == 0)
        $display("PASS [tb_lsq2w_replay]: %0d/%0d Phase 1 LSQ replay/barrier/AMO checks passed!", checks, checks);
    else begin
        $display("FAIL [tb_lsq2w_replay]: %0d errors out of %0d checks!", errors, checks);
        $fatal(1);
    end
    $finish;
end

endmodule
