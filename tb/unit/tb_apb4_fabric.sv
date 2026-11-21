`timescale 1ns / 1ps

// ============================================================================
// Unit Testbench for Standardized AMBA APB4 Fabric (Phase U2)
// Tests:
//   - mmio_to_apb4_bridge + apb4_interconnect + apb4_clint + apb4_soc_ctrl
//     + synthetic 3-wait-state slave on Slot 2 (GPIO slot) for PREADY=0 stress!
//   - ALWAYS-ON AMBA APB4 Protocol Checker (ARM IHI 0024C):
//       * Never PENABLE && !PSEL
//       * SETUP (PSEL=1, PENABLE=0) always transitions to ACCESS (PSEL=1, PENABLE=1)
//       * PADDR, PWRITE, PWDATA, PSTRB, PSEL, PENABLE strictly stable while !PREADY
//       * PSTRB == 4'b0000 on reads
//       * Strictly one-hot s_psel
//   - Functional Tests:
//       1. apb4_clint: msip0/1, 64-bit mtime tick, 64-bit mtimecmp0/1 -> mtip0/1, PSTRB byte write
//       2. apb4_soc_ctrl: SOC_ID, CORE_COUNT, FEATURE_MASK, PMU readback, TOHOST/SYS_HALT, RO PSLVERR
//       3. Multi-cycle wait-state (PREADY=0 for 3 cyc) stability & data integrity
//       4. Unmapped holes & unpopulated slots -> PREADY=1, PSLVERR=1 (both read & write)
//       5. Concurrent Dual-Master Round-Robin Fairness (Core 0 + Core 1 simultaneous back-to-back)
// ============================================================================
module tb_apb4_fabric;

`include "soc_addr_map.vh"

localparam integer ID_W       = 7;
localparam integer NUM_SLAVES = 7;
// Enable S0 (CLINT), S2 (Wait-State Test Slave), S6 (SOC_CTRL); leave S1/S3/S4/S5 unpopulated!
localparam [15:0]  SLV_MASK   = 16'b0000_0000_0100_0101;

reg             clk, reset_n;

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

wire            msip0, msip1, mtip0, mtip1, sys_halt;
wire [63:0]     mtime;
wire [31:0]     tohost;

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

apb4_clint u_clint (
    .pclk(clk), .preset_n(reset_n),
    .psel(s_psel[APB_SLV_CLINT]), .penable(s_penable), .pwrite(s_pwrite),
    .paddr(s_paddr), .pwdata(s_pwdata), .pstrb(s_pstrb),
    .pready(s_pready[APB_SLV_CLINT]), .prdata(s_prdata[APB_SLV_CLINT*32 +: 32]),
    .pslverr(s_pslverr[APB_SLV_CLINT]),
    .msip0_out(msip0), .msip1_out(msip1), .mtip0_out(mtip0), .mtip1_out(mtip1), .mtime_out(mtime)
);

apb4_soc_ctrl #(.SOC_ID(32'h2002_0002), .NUM_CORES(32'd2), .FEATURE_MASK(32'h0000_0003)) u_ctrl (
    .pclk(clk), .preset_n(reset_n),
    .psel(s_psel[APB_SLV_SOC_CTRL]), .penable(s_penable), .pwrite(s_pwrite),
    .paddr(s_paddr), .pwdata(s_pwdata), .pstrb(s_pstrb),
    .pready(s_pready[APB_SLV_SOC_CTRL]), .prdata(s_prdata[APB_SLV_SOC_CTRL*32 +: 32]),
    .pslverr(s_pslverr[APB_SLV_SOC_CTRL]),
    .pmu_cycles_in(32'h0000_1234), .pmu_c0_instret_in(32'h0000_0555), .pmu_c1_instret_in(32'h0000_0666),
    .pmu_coh_c2c_in(32'h0000_0077), .pmu_l2_hits_in(32'h0000_0888), .pmu_l2_misses_in(32'h0000_0099),
    .pmu_dram_rds_in(32'h0000_00AA), .pmu_dram_wrs_in(32'h0000_00BB),
    .sys_halt_out(sys_halt), .tohost_out(tohost)
);

// Synthetic 3-Wait-State Slave on Slot 2 (tests PREADY=0 hold & signal stability!)
reg [1:0]  ws_cnt;
reg [31:0] ws_reg;
wire       ws_ready = (ws_cnt == 2'd3);

assign s_pready[APB_SLV_GPIO]              = ws_ready;
assign s_pslverr[APB_SLV_GPIO]             = 1'b0;
assign s_prdata[APB_SLV_GPIO*32 +: 32]     = ws_reg;

// Unused slave tie-offs (S1, S3, S4, S5)
assign s_pready[APB_SLV_PLIC]  = 1'b1; assign s_pslverr[APB_SLV_PLIC]  = 1'b1; assign s_prdata[APB_SLV_PLIC*32  +: 32] = 32'd0;
assign s_pready[APB_SLV_UART]  = 1'b1; assign s_pslverr[APB_SLV_UART]  = 1'b1; assign s_prdata[APB_SLV_UART*32  +: 32] = 32'd0;
assign s_pready[APB_SLV_DMA]   = 1'b1; assign s_pslverr[APB_SLV_DMA]   = 1'b1; assign s_prdata[APB_SLV_DMA*32   +: 32] = 32'd0;
assign s_pready[APB_SLV_TIMER] = 1'b1; assign s_pslverr[APB_SLV_TIMER] = 1'b1; assign s_prdata[APB_SLV_TIMER*32 +: 32] = 32'd0;

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        ws_cnt <= 2'd0;
        ws_reg <= 32'h0;
    end else if (s_psel[APB_SLV_GPIO] && s_penable) begin
        if (ws_ready) begin
            if (s_pwrite) ws_reg <= s_pwdata;
            ws_cnt <= 2'd0;
        end else begin
            ws_cnt <= ws_cnt + 2'd1;
        end
    end else begin
        ws_cnt <= 2'd0;
    end
end

// ============================================================================
// ALWAYS-ON AMBA APB4 Protocol Assertion Checker (ARM IHI 0024C)
// ============================================================================
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
        // Rule 1: PENABLE must never be high without PSEL
        if (m_penable && !m_psel) begin
            $display("FAIL [APB4 PROTO]: PENABLE=1 while PSEL=0!");
            proto_errs <= proto_errs + 1;
        end
        // Rule 2: PSTRB must be 0 during reads
        if (m_psel && !m_pwrite && (m_pstrb != 4'b0000)) begin
            $display("FAIL [APB4 PROTO]: Non-zero PSTRB=0x%x on read!", m_pstrb);
            proto_errs <= proto_errs + 1;
        end
        // Rule 3: Setup (PSEL=1, PENABLE=0) must advance to Access (PSEL=1, PENABLE=1)
        if (prev_psel && !prev_penable && !(m_psel && m_penable)) begin
            $display("FAIL [APB4 PROTO]: Setup phase did not transition to Access!");
            proto_errs <= proto_errs + 1;
        end
        // Rule 4: During Setup->Access OR Access while !PREADY, addr/ctrl/data must stay stable!
        if ((prev_psel && !prev_penable) || (prev_psel && prev_penable && !prev_pready)) begin
            if (!m_psel || !m_penable || m_paddr != prev_paddr || m_pwrite != prev_pwrite ||
                m_pwdata != prev_pwdata || m_pstrb != prev_pstrb) begin
                $display("FAIL [APB4 PROTO]: Signals mutated while waiting for PREADY!");
                proto_errs <= proto_errs + 1;
            end
        end
        // Rule 5: s_psel must be one-hot or zero
        if ((s_psel & (s_psel - 7'd1)) != 7'd0) begin
            $display("FAIL [APB4 PROTO]: Multiple slaves selected s_psel=0x%x!", s_psel);
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

task check(input cond, input [511:0] msg);
    begin
        checks = checks + 1;
        if (!cond) begin
            $display("FAIL [tb_apb4_fabric]: %0s", msg);
            errors = errors + 1;
        end
    end
endtask

task c0_txn(
    input         we,
    input  [31:0] addr,
    input  [31:0] wdata,
    input  [3:0]  wstrb,
    input  [6:0]  id,
    output [31:0] rdata,
    output        err
);
    integer wd;
    begin
        c0_req_valid    = 1'b1;
        c0_req_is_write = we;
        c0_req_addr     = addr;
        c0_req_wdata    = wdata;
        c0_req_wstrb    = wstrb;
        c0_req_id       = id;
        #1;
        while (!c0_req_ready) step_clk();
        step_clk();
        c0_req_valid = 1'b0;
        wd = 0;
        while (!c0_resp_valid && wd < 32) begin
            step_clk();
            wd = wd + 1;
        end
        check(c0_resp_valid && (c0_resp_id == id), "c0_txn valid response & matching ID");
        rdata = c0_resp_rdata;
        err   = c0_resp_err;
        step_clk();
    end
endtask

reg [31:0] rd_v;
reg        rd_e;
integer    c0_grants, c1_grants, rcyc;

initial begin
    errors = 0; checks = 0;
    reset_n = 1'b0;
    c0_req_valid = 1'b0; c0_req_is_write = 1'b0; c0_req_addr = 32'd0; c0_req_wdata = 32'd0; c0_req_wstrb = 4'd0; c0_req_id = 7'd0;
    c1_req_valid = 1'b0; c1_req_is_write = 1'b0; c1_req_addr = 32'd0; c1_req_wdata = 32'd0; c1_req_wstrb = 4'd0; c1_req_id = 7'd0;
    repeat (2) step_clk();
    reset_n = 1'b1;
    #1;

    // 1. CLINT: msip0/1, mtimecmp0 -> mtip0 firing, byte PSTRB on scratch[0]
    check(!mtip0 && !mtip1, "CLINT reset mtimecmp=MAX -> mtip0/1 low");
    c0_txn(1'b1, 32'h0200_0000, 32'd1, 4'hF, 7'd1, rd_v, rd_e);
    check(!rd_e && msip0 && !msip1, "CLINT msip0=1 write");
    c0_txn(1'b1, 32'h0200_4000, 32'd5, 4'hF, 7'd2, rd_v, rd_e); // mtimecmp0_lo = 5
    c0_txn(1'b1, 32'h0200_4004, 32'd0, 4'hF, 7'd3, rd_v, rd_e); // mtimecmp0_hi = 0 -> mtime > 5 -> mtip0=1!
    check(!rd_e && mtip0 && !mtip1, "CLINT mtime >= mtimecmp0 asserts mtip0");
    c0_txn(1'b1, 32'h0200_4004, 32'hFFFF_FFFF, 4'hF, 7'd4, rd_v, rd_e); // clear alarm
    check(!mtip0, "CLINT raising mtimecmp0 clears mtip0");

    // Byte PSTRB merge test on CLINT scratch[0]
    c0_txn(1'b1, 32'h0200_C000, 32'h1122_3344, 4'hF, 7'd5, rd_v, rd_e);
    c0_txn(1'b1, 32'h0200_C000, 32'h00AA_0000, 4'b0100, 7'd6, rd_v, rd_e);
    c0_txn(1'b0, 32'h0200_C000, 32'd0, 4'h0, 7'd7, rd_v, rd_e);
    check(!rd_e && (rd_v == 32'h11AA_3344), "CLINT APB4 PSTRB byte-lane merge");

    // 2. SoC Control & PMU Slave (0x1004_0000)
    c0_txn(1'b0, 32'h1004_0000, 32'd0, 4'h0, 7'd10, rd_v, rd_e);
    check(!rd_e && (rd_v == 32'h2002_0002), "SOC_CTRL SOC_ID == 0x20020002");
    c0_txn(1'b0, 32'h1004_0004, 32'd0, 4'h0, 7'd11, rd_v, rd_e);
    check(!rd_e && (rd_v == 32'd2), "SOC_CTRL CORE_COUNT == 2");
    c0_txn(1'b0, 32'h1004_0020, 32'd0, 4'h0, 7'd12, rd_v, rd_e);
    check(!rd_e && (rd_v == 32'h0000_1234), "SOC_CTRL live PMU_CYCLES readback");
    c0_txn(1'b1, 32'h1004_0010, 32'hCAFE_BABE, 4'hF, 7'd13, rd_v, rd_e);
    check(!rd_e && (tohost == 32'hCAFE_BABE), "SOC_CTRL TOHOST write");
    c0_txn(1'b1, 32'h1004_000C, 32'd1, 4'hF, 7'd14, rd_v, rd_e);
    check(!rd_e && sys_halt, "SOC_CTRL SYS_HALT write");
    // RO register write -> MUST return PSLVERR=1!
    c0_txn(1'b1, 32'h1004_0000, 32'hBAD0_BAD0, 4'hF, 7'd15, rd_v, rd_e);
    check(rd_e, "SOC_CTRL RO register write triggers PSLVERR=1");

    // 3. Multi-cycle PREADY=0 wait-state slave (Slot 2 @ 0x1000_0000)
    c0_txn(1'b1, 32'h1000_0000, 32'h5A5A_C3C3, 4'hF, 7'd20, rd_v, rd_e);
    check(!rd_e, "3-wait-state APB4 write succeeds");
    c0_txn(1'b0, 32'h1000_0000, 32'd0, 4'h0, 7'd21, rd_v, rd_e);
    check(!rd_e && (rd_v == 32'h5A5A_C3C3), "3-wait-state APB4 read returns 0x5A5AC3C3");

    // 4. Unmapped address holes & unpopulated slots -> PSLVERR=1 on BOTH read and write!
    c0_txn(1'b0, 32'h0004_0000, 32'd0, 4'h0, 7'd30, rd_v, rd_e);
    check(rd_e, "Unmapped RAM hole 0x00040000 read -> PSLVERR=1");
    c0_txn(1'b1, 32'h0004_0000, 32'd99, 4'hF, 7'd31, rd_v, rd_e);
    check(rd_e, "Unmapped RAM hole 0x00040000 write -> PSLVERR=1");
    c0_txn(1'b0, 32'h0C00_0000, 32'd0, 4'h0, 7'd32, rd_v, rd_e);
    check(rd_e, "Unpopulated PLIC slot 0x0C000000 read -> PSLVERR=1");
    c0_txn(1'b1, 32'h1005_0000, 32'd42, 4'hF, 7'd33, rd_v, rd_e);
    check(rd_e, "Unpopulated expansion slot S7 0x10050000 write -> PSLVERR=1");
    c0_txn(1'b0, 32'hDEAD_0000, 32'd0, 4'h0, 7'd34, rd_v, rd_e);
    check(rd_e, "Unmapped 0xDEAD0000 read -> PSLVERR=1");

    // 5. Concurrent Dual-Master Round-Robin Fairness Proof!
    c0_grants = 0; c1_grants = 0;
    c0_req_valid = 1'b1; c0_req_is_write = 1'b0; c0_req_addr = 32'h1004_0000; c0_req_id = 7'd40;
    c1_req_valid = 1'b1; c1_req_is_write = 1'b0; c1_req_addr = 32'h1004_0004; c1_req_id = 7'd50;
    for (rcyc = 0; rcyc < 24; rcyc = rcyc + 1) begin
        #1;
        if (c0_req_ready) c0_grants = c0_grants + 1;
        if (c1_req_ready) c1_grants = c1_grants + 1;
        if (c0_resp_valid) check(!c0_resp_err && (c0_resp_id == 7'd40) && (c0_resp_rdata == 32'h2002_0002), "Dual-master C0 resp exact");
        if (c1_resp_valid) check(!c1_resp_err && (c1_resp_id == 7'd50) && (c1_resp_rdata == 32'd2), "Dual-master C1 resp exact");
        step_clk();
    end
    c0_req_valid = 1'b0; c1_req_valid = 1'b0;
    repeat (4) step_clk();
    check((c0_grants >= 3) && (c1_grants >= 3) && (c0_grants == c1_grants), "Dual-master RR fairness (exact 50/50 split!)");

    check(proto_errs == 0, "Zero AMBA APB4 protocol violations across entire run");

    if (errors == 0 && proto_errs == 0)
        $display("PASS [tb_apb4_fabric]: %0d/%0d AMBA APB4 protocol, wait-state, PSLVERR & dual-master fairness checks passed!", checks, checks);
    else begin
        $display("FAIL [tb_apb4_fabric]: errors=%0d proto_errs=%0d checks=%0d", errors, proto_errs, checks);
        $fatal(1);
    end
    $finish;
end

endmodule
