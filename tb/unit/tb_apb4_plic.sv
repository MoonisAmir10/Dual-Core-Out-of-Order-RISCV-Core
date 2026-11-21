`timescale 1ns / 1ps

// ============================================================================
// Unit Testbench for AMBA APB4 RISC-V Dual-Core PLIC (Phase U3)
//
// Tests (Icarus Verilog + Verilator --assert -Wall):
//   - ALWAYS-ON ARM IHI 0024C AMBA APB4 Protocol Checker
//   - 1. Reset state (meip0/1=0, IP=0, Source 0 priority RO 0 ignores writes)
//   - 2. Priority ordering (prio 6 > prio 2 -> claims higher priority first)
//   - 3. Lowest-ID tie-breaking (equal prio 4 on Src 5 & Src 2 -> Src 2 wins!)
//   - 4. Per-hart threshold masking (thresh=4 masks prio<=4, unmasks prio=5)
//   - 5. Level-sensitive gateway lifecycle (in-service blocks re-pend; Complete
//        with src still high re-pends immediately; clearing src before Complete
//        leaves IP=0!)
//   - 6. Rising-edge gateway lifecycle (src held high after Complete does NOT
//        re-pend; fresh 0->1 rising edge triggers cleanly!)
//   - 7. Independent per-hart routing (Src 4 -> Hart 0 only; Src 9 -> Hart 1 only)
//   - 8. Complete ignored when source not enabled for context (PLIC Spec §8)
//   - 9. Simultaneous Dual-Hart Atomic Claim Race via mmio_to_apb4_bridge:
//        Core 0 & Core 1 issue CLAIM0 & CLAIM1 on the EXACT same cycle for
//        shared Source 12 -> strictly ONE hart claims 12 and the other gets 0!
// ============================================================================
module tb_apb4_plic;

`include "soc_addr_map.vh"

localparam integer ID_W       = 7;
localparam integer NUM_SLAVES = 7;
localparam [15:0]  SLV_MASK   = 16'b0000_0000_0000_0010; // S1 = PLIC only

reg             clk, reset_n;
reg  [15:1]     irq_src;
wire [1:0]      meip;

reg             c0_req_valid, c0_req_is_write;
wire            c0_req_ready;
reg  [31:0]     c0_req_addr, c0_req_wdata;
reg  [3:0]      c0_req_wstrb;
reg  [ID_W-1:0] c0_req_id;
wire            c0_resp_valid, c0_resp_err;
wire [ID_W-1:0] c0_resp_id;
wire [31:0]     c0_resp_rdata;

reg             c1_req_valid, c1_req_is_write;
wire            c1_req_ready;
reg  [31:0]     c1_req_addr, c1_req_wdata;
reg  [3:0]      c1_req_wstrb;
reg  [ID_W-1:0] c1_req_id;
wire            c1_resp_valid, c1_resp_err;
wire [ID_W-1:0] c1_resp_id;
wire [31:0]     c1_resp_rdata;

wire            m_psel, m_penable, m_pwrite, m_pready, m_pslverr;
wire [31:0]     m_paddr, m_pwdata, m_prdata;
wire [3:0]      m_pstrb;

wire [NUM_SLAVES-1:0]    s_psel, s_pready, s_pslverr;
wire [NUM_SLAVES*32-1:0] s_prdata;
wire            s_penable, s_pwrite;
wire [31:0]     s_paddr, s_pwdata;
wire [3:0]      s_pstrb;

mmio_to_apb4_bridge #(.ID_W(ID_W)) u_bridge (
    .clk(clk), .reset_n(reset_n),
    .c0_req_valid(c0_req_valid), .c0_req_ready(c0_req_ready),
    .c0_req_is_write(c0_req_is_write), .c0_req_addr(c0_req_addr),
    .c0_req_wdata(c0_req_wdata), .c0_req_wstrb(c0_req_wstrb), .c0_req_id(c0_req_id),
    .c0_resp_valid(c0_resp_valid), .c0_resp_id(c0_resp_id),
    .c0_resp_rdata(c0_resp_rdata), .c0_resp_err(c0_resp_err),
    .c1_req_valid(c1_req_valid), .c1_req_ready(c1_req_ready),
    .c1_req_is_write(c1_req_is_write), .c1_req_addr(c1_req_addr),
    .c1_req_wdata(c1_req_wdata), .c1_req_wstrb(c1_req_wstrb), .c1_req_id(c1_req_id),
    .c1_resp_valid(c1_resp_valid), .c1_resp_id(c1_resp_id),
    .c1_resp_rdata(c1_resp_rdata), .c1_resp_err(c1_resp_err),
    .m_psel(m_psel), .m_penable(m_penable), .m_pwrite(m_pwrite),
    .m_paddr(m_paddr), .m_pwdata(m_pwdata), .m_pstrb(m_pstrb),
    .m_pready(m_pready), .m_prdata(m_prdata), .m_pslverr(m_pslverr)
);

apb4_interconnect #(.NUM_SLAVES(NUM_SLAVES), .SLAVE_PRESENT(SLV_MASK)) u_xbar (
    .m_psel(m_psel), .m_penable(m_penable), .m_pwrite(m_pwrite),
    .m_paddr(m_paddr), .m_pwdata(m_pwdata), .m_pstrb(m_pstrb),
    .m_pready(m_pready), .m_prdata(m_prdata), .m_pslverr(m_pslverr),
    .s_psel(s_psel), .s_penable(s_penable), .s_pwrite(s_pwrite),
    .s_paddr(s_paddr), .s_pwdata(s_pwdata), .s_pstrb(s_pstrb),
    .s_pready(s_pready), .s_prdata(s_prdata), .s_pslverr(s_pslverr)
);

apb4_plic #(.NUM_SOURCES(16), .NUM_HARTS(2), .PRIO_BITS(3)) u_plic (
    .pclk(clk), .preset_n(reset_n),
    .psel(s_psel[APB_SLV_PLIC]), .penable(s_penable), .pwrite(s_pwrite),
    .paddr(s_paddr), .pwdata(s_pwdata), .pstrb(s_pstrb),
    .pready(s_pready[APB_SLV_PLIC]),
    .prdata(s_prdata[APB_SLV_PLIC*32 +: 32]),
    .pslverr(s_pslverr[APB_SLV_PLIC]),
    .irq_src_in(irq_src),
    .meip_out(meip)
);

assign s_pready[APB_SLV_CLINT]    = 1'b1; assign s_pslverr[APB_SLV_CLINT]    = 1'b1; assign s_prdata[APB_SLV_CLINT*32    +: 32] = 32'd0;
assign s_pready[APB_SLV_GPIO]     = 1'b1; assign s_pslverr[APB_SLV_GPIO]     = 1'b1; assign s_prdata[APB_SLV_GPIO*32     +: 32] = 32'd0;
assign s_pready[APB_SLV_UART]     = 1'b1; assign s_pslverr[APB_SLV_UART]     = 1'b1; assign s_prdata[APB_SLV_UART*32     +: 32] = 32'd0;
assign s_pready[APB_SLV_DMA]      = 1'b1; assign s_pslverr[APB_SLV_DMA]      = 1'b1; assign s_prdata[APB_SLV_DMA*32      +: 32] = 32'd0;
assign s_pready[APB_SLV_TIMER]    = 1'b1; assign s_pslverr[APB_SLV_TIMER]    = 1'b1; assign s_prdata[APB_SLV_TIMER*32    +: 32] = 32'd0;
assign s_pready[APB_SLV_SOC_CTRL] = 1'b1; assign s_pslverr[APB_SLV_SOC_CTRL] = 1'b1; assign s_prdata[APB_SLV_SOC_CTRL*32 +: 32] = 32'd0;

// Always-on ARM IHI 0024C AMBA APB4 Protocol Monitor
integer errors, checks, proto_errs;
reg        prev_psel, prev_penable, prev_pwrite, prev_pready;
reg [31:0] prev_paddr, prev_pwdata;
reg [3:0]  prev_pstrb;

/* verilator lint_off BLKSEQ */
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        prev_psel <= 1'b0; prev_penable <= 1'b0; prev_pwrite <= 1'b0; prev_pready <= 1'b0;
        prev_paddr <= 32'd0; prev_pwdata <= 32'd0; prev_pstrb <= 4'd0;
        proto_errs <= 0;
    end else begin
        if (m_penable && !m_psel) proto_errs <= proto_errs + 1;
        if (m_psel && !m_pwrite && (m_pstrb != 4'b0000)) proto_errs <= proto_errs + 1;
        if (prev_psel && !prev_penable && !(m_psel && m_penable)) proto_errs <= proto_errs + 1;
        if ((prev_psel && !prev_penable) || (prev_psel && prev_penable && !prev_pready)) begin
            if (!m_psel || !m_penable || m_paddr != prev_paddr || m_pwrite != prev_pwrite ||
                m_pwdata != prev_pwdata || m_pstrb != prev_pstrb)
                proto_errs <= proto_errs + 1;
        end
        prev_psel    <= m_psel;
        prev_penable <= m_penable;
        prev_pwrite  <= m_pwrite;
        prev_pready  <= m_pready;
        prev_paddr   <= m_paddr;
        prev_pwdata  <= m_pwdata;
        prev_pstrb   <= m_pstrb;
    end
end
/* verilator lint_on BLKSEQ */

initial clk = 1'b0;
always #5 clk = ~clk;

task step_clk;
    begin
        @(posedge clk);
        #1;
    end
endtask

task check(input cond, input [1023:0] msg);
    begin
        checks = checks + 1;
        if (!cond) begin
            $display("FAIL [tb_apb4_plic]: %0s", msg);
            errors = errors + 1;
        end
    end
endtask

task plic_wr(input [31:0] off, input [31:0] wdata);
    integer wd;
    begin
        c0_req_valid    = 1'b1;
        c0_req_is_write = 1'b1;
        c0_req_addr     = SOC_PLIC_BASE + off;
        c0_req_wdata    = wdata;
        c0_req_wstrb    = 4'hF;
        c0_req_id       = 7'd1;
        #1;
        while (!c0_req_ready) step_clk();
        step_clk();
        c0_req_valid = 1'b0;
        wd = 0;
        while (!c0_resp_valid && wd < 32) begin step_clk(); wd = wd + 1; end
        check(c0_resp_valid && !c0_resp_err, "plic_wr APB4 clean ack");
        step_clk();
    end
endtask

task plic_rd(input [31:0] off, output [31:0] rdata);
    integer wd;
    begin
        c0_req_valid    = 1'b1;
        c0_req_is_write = 1'b0;
        c0_req_addr     = SOC_PLIC_BASE + off;
        c0_req_wdata    = 32'd0;
        c0_req_wstrb    = 4'h0;
        c0_req_id       = 7'd2;
        #1;
        while (!c0_req_ready) step_clk();
        step_clk();
        c0_req_valid = 1'b0;
        wd = 0;
        while (!c0_resp_valid && wd < 32) begin step_clk(); wd = wd + 1; end
        check(c0_resp_valid && !c0_resp_err, "plic_rd APB4 clean ack");
        rdata = c0_resp_rdata;
        step_clk();
    end
endtask

reg [31:0] rv, r_c0, r_c1;
reg        got0, got1;
integer    rcyc;

initial begin
    errors = 0; checks = 0;
    reset_n = 1'b0;
    irq_src = 15'd0;
    c0_req_valid = 1'b0; c0_req_is_write = 1'b0; c0_req_addr = 32'd0; c0_req_wdata = 32'd0; c0_req_wstrb = 4'd0; c0_req_id = 7'd0;
    c1_req_valid = 1'b0; c1_req_is_write = 1'b0; c1_req_addr = 32'd0; c1_req_wdata = 32'd0; c1_req_wstrb = 4'd0; c1_req_id = 7'd0;
    repeat (2) step_clk();
    reset_n = 1'b1;
    #1;

    // 1. Reset state & Source 0 RO=0
    check(meip == 2'b00, "Reset meip == 2'b00");
    plic_wr(32'h000000, 32'd7); // Source 0 write ignored
    plic_rd(32'h000000, rv);
    check(rv == 32'd0, "Source 0 priority hardwired 0");
    plic_rd(32'h001000, rv);
    check(rv == 32'd0, "Reset IP == 0");

    // 2. Priority ordering (Src 3 prio=2 vs Src 7 prio=6 -> Src 7 wins first!)
    plic_wr(32'h00000C, 32'd2); // Src 3 prio=2
    plic_wr(32'h00001C, 32'd6); // Src 7 prio=6
    plic_wr(32'h002000, (32'd1 << 3) | (32'd1 << 7)); // Enable Src 3,7 on Hart 0
    irq_src[3] = 1'b1;
    irq_src[7] = 1'b1;
    step_clk();
    check(meip == 2'b01, "Hart 0 meip asserted, Hart 1 quiet");
    plic_rd(32'h200004, rv); // Claim 0 -> must be 7!
    check(rv == 32'd7, "Higher priority Src 7 claimed before Src 3");
    irq_src[7] = 1'b0;
    plic_wr(32'h200004, 32'd7); // Complete Src 7
    plic_rd(32'h200004, rv); // Claim 0 -> now 3!
    check(rv == 32'd3, "Lower priority Src 3 claimed second");
    irq_src[3] = 1'b0;
    plic_wr(32'h200004, 32'd3); // Complete Src 3
    check(meip == 2'b00, "All cleared -> meip == 2'b00");

    // 3. Equal-priority tie-break (Src 5 prio=4 & Src 2 prio=4 -> lowest ID 2 wins!)
    plic_wr(32'h000008, 32'd4); // Src 2 prio=4
    plic_wr(32'h000014, 32'd4); // Src 5 prio=4
    plic_wr(32'h002000, (32'd1 << 2) | (32'd1 << 5));
    irq_src[2] = 1'b1;
    irq_src[5] = 1'b1;
    step_clk();
    plic_rd(32'h200004, rv);
    check(rv == 32'd2, "Equal-priority tie broken by lowest ID (Src 2 < Src 5)");
    irq_src[2] = 1'b0;
    plic_wr(32'h200004, 32'd2);
    plic_rd(32'h200004, rv);
    check(rv == 32'd5, "Remaining equal-priority Src 5 claimed next");
    irq_src[5] = 1'b0;
    plic_wr(32'h200004, 32'd5);

    // 4. Threshold masking (thresh=4 masks prio=4, unmasks prio=5!)
    plic_wr(32'h200000, 32'd4); // Hart 0 thresh = 4
    irq_src[2] = 1'b1; // prio=4 <= thresh=4 -> masked!
    step_clk();
    check(meip[0] == 1'b0, "Thresh=4 masks prio=4 Src 2");
    plic_wr(32'h000018, 32'd5); // Src 6 prio=5 > thresh=4!
    plic_wr(32'h002000, (32'd1 << 2) | (32'd1 << 6));
    irq_src[6] = 1'b1;
    step_clk();
    check(meip[0] == 1'b1, "Thresh=4 allows prio=5 Src 6");
    plic_rd(32'h200004, rv);
    check(rv == 32'd6, "Claim returns unmasked Src 6");
    irq_src[6] = 1'b0;
    plic_wr(32'h200004, 32'd6);
    check(meip[0] == 1'b0, "Src 2 still masked by thresh=4");
    plic_wr(32'h200000, 32'd0); // Lower thresh -> Src 2 immediately unmasks!
    check(meip[0] == 1'b1, "Lowering thresh=0 unmasks pending Src 2");
    plic_rd(32'h200004, rv);
    check(rv == 32'd2, "Claim returns unmasked Src 2");

    // 5. Level-sensitive gateway lifecycle (irq_src[2] STILL high -> re-pends on Complete!)
    check(meip[0] == 1'b0, "In-service gateway suppresses re-trigger while claimed");
    plic_wr(32'h200004, 32'd2); // Complete while irq_src[2]==1!
    step_clk();
    check(meip[0] == 1'b1, "Level-sensitive source still high immediately re-pends after Complete");
    plic_rd(32'h200004, rv);
    check(rv == 32'd2, "Re-pended level source claimed");
    irq_src[2] = 1'b0;          // Deassert BEFORE Complete
    plic_wr(32'h200004, 32'd2);
    step_clk();
    check(meip[0] == 1'b0, "Deasserted level source stays cleared after Complete");

    // 6. Rising-edge gateway lifecycle (trig_edge[8]=1)
    plic_wr(32'h001080, (32'd1 << 8)); // Src 8 = rising-edge!
    plic_wr(32'h000020, 32'd5);        // Src 8 prio=5
    plic_wr(32'h002000, (32'd1 << 8));
    irq_src[8] = 1'b1;                 // 0 -> 1 rising edge!
    step_clk();
    check(meip[0] == 1'b1, "Edge-triggered Src 8 triggers on 0->1");
    plic_rd(32'h200004, rv);
    check(rv == 32'd8, "Edge Src 8 claimed");
    // Complete while irq_src[8] is STILL high -> MUST NOT re-trigger!
    plic_wr(32'h200004, 32'd8);
    step_clk();
    check(meip[0] == 1'b0, "Edge-triggered Src 8 does NOT re-trigger while held high");
    // Drop to 0, then fresh 0->1 edge -> MUST trigger!
    irq_src[8] = 1'b0; step_clk();
    irq_src[8] = 1'b1; step_clk();
    check(meip[0] == 1'b1, "Edge-triggered Src 8 triggers on fresh 0->1 edge");
    plic_rd(32'h200004, rv);
    check(rv == 32'd8, "Fresh edge Src 8 claimed");
    irq_src[8] = 1'b0;
    plic_wr(32'h200004, 32'd8);

    // 7. Per-hart routing & Spec §8 Complete-enable check
    plic_wr(32'h000010, 32'd3);        // Src 4 prio=3
    plic_wr(32'h000024, 32'd3);        // Src 9 prio=3
    plic_wr(32'h002000, (32'd1 << 4)); // Hart 0: Src 4 ONLY
    plic_wr(32'h002080, (32'd1 << 9)); // Hart 1: Src 9 ONLY
    irq_src[4] = 1'b1;
    step_clk();
    check(meip == 2'b01, "Src 4 routes ONLY to Hart 0");
    irq_src[9] = 1'b1;
    step_clk();
    check(meip == 2'b11, "Src 4->Hart 0 and Src 9->Hart 1 concurrently");
    plic_rd(32'h200004, rv); check(rv == 32'd4, "Hart 0 claims Src 4");
    plic_rd(32'h201004, rv); check(rv == 32'd9, "Hart 1 claims Src 9");
    // Hart 1 trying to complete Hart 0's disabled Src 4 -> ignored per Spec §8!
    plic_wr(32'h201004, 32'd4);
    step_clk();
    check(meip[0] == 1'b0, "Disabled-context Complete ignored (gateway remains locked)");
    irq_src[4] = 1'b0; irq_src[9] = 1'b0;
    plic_wr(32'h200004, 32'd4);
    plic_wr(32'h201004, 32'd9);

    // 8. SIMULTANEOUS DUAL-HART ATOMIC CLAIM RACE (Shared Source 12 on BOTH harts!)
    plic_wr(32'h001080, (32'd1 << 12)); // Src 12 edge-triggered
    plic_wr(32'h000030, 32'd7);         // Src 12 prio=7
    plic_wr(32'h002000, (32'd1 << 12)); // Enable on Hart 0
    plic_wr(32'h002080, (32'd1 << 12)); // Enable on Hart 1
    irq_src[12] = 1'b1;
    step_clk();
    irq_src[12] = 1'b0;
    step_clk();
    check(meip == 2'b11, "Shared Src 12 raises BOTH meip[0] and meip[1]");

    // Launch BOTH Core 0 (CLAIM0) and Core 1 (CLAIM1) simultaneously!
    got0 = 1'b0; got1 = 1'b0; r_c0 = 32'hFFFF; r_c1 = 32'hFFFF;
    c0_req_valid = 1'b1; c0_req_is_write = 1'b0; c0_req_addr = SOC_PLIC_BASE + 32'h200004; c0_req_wstrb = 4'h0; c0_req_id = 7'd20;
    c1_req_valid = 1'b1; c1_req_is_write = 1'b0; c1_req_addr = SOC_PLIC_BASE + 32'h201004; c1_req_wstrb = 4'h0; c1_req_id = 7'd21;
    for (rcyc = 0; rcyc < 20; rcyc = rcyc + 1) begin
        #1;
        if (c0_req_valid && c0_req_ready) begin
            step_clk();
            c0_req_valid = 1'b0;
        end else if (c1_req_valid && c1_req_ready) begin
            step_clk();
            c1_req_valid = 1'b0;
        end else begin
            step_clk();
        end
        if (c0_resp_valid) begin got0 = 1'b1; r_c0 = c0_resp_rdata; end
        if (c1_resp_valid) begin got1 = 1'b1; r_c1 = c1_resp_rdata; end
    end
    check(got0 && got1, "Both harts received APB4 Claim responses");
    check(((r_c0 == 32'd12) && (r_c1 == 32'd0)) || ((r_c0 == 32'd0) && (r_c1 == 32'd12)),
          "Dual-hart Claim race: strictly ONE hart claimed 12 and peer got 0!");
    check(meip == 2'b00, "After atomic Claim, both meip[0] & meip[1] cleared");

    // Complete by winner and verify gateway re-arms for next pulse!
    if (r_c0 == 32'd12) plic_wr(32'h200004, 32'd12);
    else                plic_wr(32'h201004, 32'd12);
    irq_src[12] = 1'b1; step_clk();
    irq_src[12] = 1'b0; step_clk();
    check(meip == 2'b11, "Gateway re-arms cleanly after winner Complete");

    check(proto_errs == 0, "Zero AMBA APB4 protocol violations across entire PLIC test");

    if (errors == 0 && proto_errs == 0)
        $display("PASS [tb_apb4_plic]: %0d/%0d RISC-V PLIC priority, threshold, gateway & dual-hart Claim race checks passed!", checks, checks);
    else begin
        $display("FAIL [tb_apb4_plic]: errors=%0d proto_errs=%0d checks=%0d", errors, proto_errs, checks);
        $fatal(1);
    end
    $finish;
end

endmodule
