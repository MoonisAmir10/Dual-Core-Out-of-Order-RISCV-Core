`timescale 1ns / 1ps

// ============================================================================
// Always-On MOESI Coherence Protocol Monitor & Golden Shadow Checker (Phase 3)
//
// Attaches to Dual-Core MOESI L1D + coh_interconnect in Verilator & Icarus:
//   1. Maintains 128-bit Golden Shadow Memory updated at exact store/AMO
//      linearization points (dbg_wr_commit_* from D0 & D1)
//   2. Asserts NO simultaneous same-line writes on D0 & D1 (SWMR write safety)
//   3. Enforces hard WATCHDOG_CYC (default 256) forward-progress deadlock
//      watchdogs on D0, D1, and coh_interconnect
//   4. Tracks full coherence event telemetry (GetS, GetM, Upgrade, C2C, Inval)
// ============================================================================
module coh_checker #(
    parameter integer LADDR_W      = 16,
    parameter integer MEM_LINES    = 16384,
    parameter integer WATCHDOG_CYC = 256
) (
    input  wire                clk,
    input  wire                reset_n,
    input  wire                active,

    // Exact Write Linearization Hooks from D0 & D1
    input  wire                d0_wr_valid,
    input  wire [LADDR_W-1:0]  d0_wr_laddr,
    input  wire [127:0]        d0_wr_data,

    input  wire                d1_wr_valid,
    input  wire [LADDR_W-1:0]  d1_wr_laddr,
    input  wire [127:0]        d1_wr_data,

    // Global Channel A Bus Serialization Events
    input  wire                bus_evt_valid,
    input  wire                bus_evt_src,
    input  wire [1:0]          bus_evt_op,
    input  wire [LADDR_W-1:0]  bus_evt_laddr,
    input  wire                bus_evt_c2c,
    input  wire                bus_evt_shared,

    // Activity / Idle Signals for Deadlock Watchdogs
    input  wire                d0_idle,
    input  wire                d1_idle,
    input  wire                coh_idle,
    input  wire                d0_progress,
    input  wire                d1_progress,

    // Telemetry & Error Counter Outputs
    output reg  [31:0]         chk_errors,
    output reg  [31:0]         cnt_gets,
    output reg  [31:0]         cnt_getm,
    output reg  [31:0]         cnt_upgrade,
    output reg  [31:0]         cnt_c2c,
    output reg  [31:0]         cnt_wr_commits
);

`include "coh_defs.vh"

// 128-bit Golden Shadow Memory (initialized by TB alongside DRAM)
reg [127:0] shadow_mem [0:MEM_LINES-1];

reg [31:0] wd_d0, wd_d1, wd_coh;

/* verilator lint_off BLKSEQ */
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        chk_errors     <= 32'd0;
        cnt_gets       <= 32'd0;
        cnt_getm       <= 32'd0;
        cnt_upgrade    <= 32'd0;
        cnt_c2c        <= 32'd0;
        cnt_wr_commits <= 32'd0;
        wd_d0          <= 32'd0;
        wd_d1          <= 32'd0;
        wd_coh         <= 32'd0;
    end else if (active) begin
        // 1. SWMR Write-Linearization Exclusivity Check & Shadow Memory Update
        if (d0_wr_valid && d1_wr_valid && (d0_wr_laddr == d1_wr_laddr)) begin
            $display("FAIL [coh_checker]: SWMR VIOLATION! Simultaneous writes to laddr=0x%04x on D0 & D1!",
                     d0_wr_laddr);
            chk_errors <= chk_errors + 32'd1;
        end
        if (d0_wr_valid) begin
            shadow_mem[d0_wr_laddr[13:0]] <= d0_wr_data;
            cnt_wr_commits <= cnt_wr_commits + 32'd1 + (d1_wr_valid ? 32'd1 : 32'd0);
        end else if (d1_wr_valid) begin
            cnt_wr_commits <= cnt_wr_commits + 32'd1;
        end
        if (d1_wr_valid) begin
            shadow_mem[d1_wr_laddr[13:0]] <= d1_wr_data;
        end

        // 2. Global Channel A Bus Telemetry
        if (bus_evt_valid) begin
            case (bus_evt_op)
                COH_REQ_GETS:    cnt_gets    <= cnt_gets    + 32'd1;
                COH_REQ_GETM:    cnt_getm    <= cnt_getm    + 32'd1;
                COH_REQ_UPGRADE: cnt_upgrade <= cnt_upgrade + 32'd1;
                default: ;
            endcase
            if (bus_evt_c2c)
                cnt_c2c <= cnt_c2c + 32'd1;
        end

        // 3. Hard Forward-Progress Deadlock Watchdogs (WATCHDOG_CYC cycles)
        if (d0_idle || d0_progress || d0_wr_valid)
            wd_d0 <= 32'd0;
        else if (wd_d0 >= WATCHDOG_CYC) begin
            $display("FAIL [coh_checker]: DEADLOCK WATCHDOG on D0 (>%0d cycles without progress!)", WATCHDOG_CYC);
            chk_errors <= chk_errors + 32'd1;
            wd_d0      <= 32'd0;
        end else
            wd_d0 <= wd_d0 + 32'd1;

        if (d1_idle || d1_progress || d1_wr_valid)
            wd_d1 <= 32'd0;
        else if (wd_d1 >= WATCHDOG_CYC) begin
            $display("FAIL [coh_checker]: DEADLOCK WATCHDOG on D1 (>%0d cycles without progress!)", WATCHDOG_CYC);
            chk_errors <= chk_errors + 32'd1;
            wd_d1      <= 32'd0;
        end else
            wd_d1 <= wd_d1 + 32'd1;

        if (coh_idle || bus_evt_valid || d0_progress || d1_progress)
            wd_coh <= 32'd0;
        else if (wd_coh >= WATCHDOG_CYC) begin
            $display("FAIL [coh_checker]: DEADLOCK WATCHDOG on coh_interconnect (>%0d cycles!)", WATCHDOG_CYC);
            chk_errors <= chk_errors + 32'd1;
            wd_coh     <= 32'd0;
        end else
            wd_coh <= wd_coh + 32'd1;
    end
end
/* verilator lint_on BLKSEQ */

endmodule
