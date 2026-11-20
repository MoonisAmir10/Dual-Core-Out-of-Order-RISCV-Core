`timescale 1ns / 1ps

// ============================================================================
// 2-Wide Superscalar Out-of-Order Proof Monitor & Konata/Gantt Exporter
//
// Supports 2-wide dispatch, 6-port FU issue, 2-wide CDB, and 2-wide commit.
// Enforces INV-DISP-ORDER, INV-COMMIT-ORDER, PROOF-OOO-ISSUE, PROOF-OOO-COMPLETE,
// PROOF-SPEC-SQUASH, and PROOF-LIVENESS.
// ============================================================================
module ooo_trace_mon2w #(
    parameter integer MAX_SEQ        = 4096,
    parameter integer NUM_ISSUE_PORT = 6,
    parameter integer NUM_CDB_PORT   = 2,
    parameter integer WATCHDOG_CYC   = 512
) (
    input  wire                              clk,
    /* verilator lint_off SYNCASYNCNET */
    input  wire                              reset_n,
    /* verilator lint_on SYNCASYNCNET */
    input  wire                              active,

    // 2-Wide dispatch trace
    input  wire                              disp0_valid,
    input  wire [31:0]                       disp0_seq_id,
    input  wire [31:0]                       disp0_pc,
    input  wire [31:0]                       disp0_instr,
    input  wire                              disp1_valid,
    input  wire [31:0]                       disp1_seq_id,
    input  wire [31:0]                       disp1_pc,
    input  wire [31:0]                       disp1_instr,

    // 6-Port FU issue trace
    input  wire [NUM_ISSUE_PORT-1:0]         issue_valid,
    input  wire [NUM_ISSUE_PORT*32-1:0]      issue_seq_id,

    // 2-Port CDB completion trace
    input  wire [NUM_CDB_PORT-1:0]           cdb_valid,
    input  wire [NUM_CDB_PORT*32-1:0]        cdb_seq_id,

    // 2-Wide ROB head commit trace
    input  wire                              commit0_valid,
    input  wire [31:0]                       commit0_seq_id,
    input  wire                              commit1_valid,
    input  wire [31:0]                       commit1_seq_id,

    // Selective squash trace
    input  wire                              flush_valid,
    input  wire [31:0]                       flush_after_seq_id,

    output reg  [31:0]                       cycle_count,
    output reg  [31:0]                       ooo_issue_pairs,
    output reg  [31:0]                       ooo_cdb_pairs,
    output reg  [31:0]                       spec_squashed_issued,
    output reg  [31:0]                       inv_errors
);

reg        ev_valid    [0:MAX_SEQ-1];
reg [31:0] ev_pc       [0:MAX_SEQ-1];
reg [31:0] ev_instr    [0:MAX_SEQ-1];
reg [31:0] t_disp      [0:MAX_SEQ-1];
reg [31:0] t_issue     [0:MAX_SEQ-1];
reg [31:0] t_cdb       [0:MAX_SEQ-1];
reg [31:0] t_commit    [0:MAX_SEQ-1];
reg [31:0] t_squash    [0:MAX_SEQ-1];
reg        ev_squashed [0:MAX_SEQ-1];
reg        ev_committed[0:MAX_SEQ-1];

reg        has_disp, has_commit;
reg [31:0] last_disp_seq, last_commit_seq, max_seq_seen, cyc_since_commit;

integer p, k;

task automatic reset_monitor;
    integer idx;
    begin
        cycle_count          = 32'd0;
        ooo_issue_pairs      = 32'd0;
        ooo_cdb_pairs        = 32'd0;
        spec_squashed_issued = 32'd0;
        inv_errors           = 32'd0;
        has_disp             = 1'b0;
        last_disp_seq        = 32'd0;
        has_commit           = 1'b0;
        last_commit_seq      = 32'd0;
        max_seq_seen         = 32'd0;
        cyc_since_commit     = 32'd0;
        for (idx = 0; idx < MAX_SEQ; idx = idx + 1) begin
            ev_valid[idx]     = 1'b0;
            ev_pc[idx]        = 32'd0;
            ev_instr[idx]     = 32'd0;
            t_disp[idx]       = 32'd0;
            t_issue[idx]      = 32'd0;
            t_cdb[idx]        = 32'd0;
            t_commit[idx]     = 32'd0;
            t_squash[idx]     = 32'd0;
            ev_squashed[idx]  = 1'b0;
            ev_committed[idx] = 1'b0;
        end
    end
endtask

initial reset_monitor();

always @(posedge clk) begin
    if (!reset_n) begin
        reset_monitor();
    end else if (active) begin
        cycle_count <= cycle_count + 32'd1;

        // 1. 2-Wide Dispatch tracking + INV-DISP-ORDER
        if (disp0_valid) begin
            if (has_disp && (disp0_seq_id <= last_disp_seq)) begin
                $display("MON2W FAIL [INV-DISP-ORDER cyc=%0d]: disp0_seq=%0d <= last=%0d",
                         cycle_count, disp0_seq_id, last_disp_seq);
                inv_errors <= inv_errors + 32'd1;
            end
            if (disp0_seq_id < MAX_SEQ) begin
                ev_valid[disp0_seq_id]     <= 1'b1;
                ev_pc[disp0_seq_id]        <= disp0_pc;
                ev_instr[disp0_seq_id]     <= disp0_instr;
                t_disp[disp0_seq_id]       <= cycle_count + 32'd1;
                t_issue[disp0_seq_id]      <= 32'd0;
                t_cdb[disp0_seq_id]        <= 32'd0;
                t_commit[disp0_seq_id]     <= 32'd0;
                t_squash[disp0_seq_id]     <= 32'd0;
                ev_squashed[disp0_seq_id]  <= 1'b0;
                ev_committed[disp0_seq_id] <= 1'b0;
            end
            has_disp <= 1'b1;
            if (disp1_valid) begin
                if (disp1_seq_id <= disp0_seq_id) begin
                    $display("MON2W FAIL [INV-DISP-ORDER cyc=%0d]: disp1_seq=%0d <= disp0=%0d",
                             cycle_count, disp1_seq_id, disp0_seq_id);
                    inv_errors <= inv_errors + 32'd1;
                end
                last_disp_seq <= disp1_seq_id;
                if (disp1_seq_id > max_seq_seen) max_seq_seen <= disp1_seq_id;
                if (disp1_seq_id < MAX_SEQ) begin
                    ev_valid[disp1_seq_id]     <= 1'b1;
                    ev_pc[disp1_seq_id]        <= disp1_pc;
                    ev_instr[disp1_seq_id]     <= disp1_instr;
                    t_disp[disp1_seq_id]       <= cycle_count + 32'd1;
                    t_issue[disp1_seq_id]      <= 32'd0;
                    t_cdb[disp1_seq_id]        <= 32'd0;
                    t_commit[disp1_seq_id]     <= 32'd0;
                    t_squash[disp1_seq_id]     <= 32'd0;
                    ev_squashed[disp1_seq_id]  <= 1'b0;
                    ev_committed[disp1_seq_id] <= 1'b0;
                end
            end else begin
                last_disp_seq <= disp0_seq_id;
                if (disp0_seq_id > max_seq_seen) max_seq_seen <= disp0_seq_id;
            end
        end

        // 2. 6-Port FU Issue tracking
        for (p = 0; p < NUM_ISSUE_PORT; p = p + 1) begin
            if (issue_valid[p]) begin
                if (issue_seq_id[p*32 +: 32] < MAX_SEQ &&
                    t_issue[issue_seq_id[p*32 +: 32]] == 32'd0)
                    t_issue[issue_seq_id[p*32 +: 32]] <= cycle_count + 32'd1;
            end
        end

        // 3. 2-Port CDB completion tracking
        for (p = 0; p < NUM_CDB_PORT; p = p + 1) begin
            if (cdb_valid[p]) begin
                if (cdb_seq_id[p*32 +: 32] < MAX_SEQ &&
                    t_cdb[cdb_seq_id[p*32 +: 32]] == 32'd0)
                    t_cdb[cdb_seq_id[p*32 +: 32]] <= cycle_count + 32'd1;
            end
        end

        // 4. 2-Wide In-Order Commit tracking + INV-COMMIT-ORDER
        if (commit0_valid) begin
            cyc_since_commit <= 32'd0;
            if (has_commit && (commit0_seq_id <= last_commit_seq)) begin
                $display("MON2W FAIL [INV-COMMIT-ORDER cyc=%0d]: c0_seq=%0d <= last=%0d",
                         cycle_count, commit0_seq_id, last_commit_seq);
                inv_errors <= inv_errors + 32'd1;
            end
            if (commit0_seq_id < MAX_SEQ) begin
                if (ev_squashed[commit0_seq_id]) begin
                    $display("MON2W FAIL [INV-COMMIT-ORDER cyc=%0d]: squashed seq=%0d committed on lane 0!",
                             cycle_count, commit0_seq_id);
                    inv_errors <= inv_errors + 32'd1;
                end
                t_commit[commit0_seq_id]     <= cycle_count + 32'd1;
                ev_committed[commit0_seq_id] <= 1'b1;
            end
            has_commit <= 1'b1;

            if (commit1_valid) begin
                if (commit1_seq_id <= commit0_seq_id) begin
                    $display("MON2W FAIL [INV-COMMIT-ORDER cyc=%0d]: c1_seq=%0d <= c0=%0d",
                             cycle_count, commit1_seq_id, commit0_seq_id);
                    inv_errors <= inv_errors + 32'd1;
                end
                if (commit1_seq_id < MAX_SEQ) begin
                    if (ev_squashed[commit1_seq_id]) begin
                        $display("MON2W FAIL [INV-COMMIT-ORDER cyc=%0d]: squashed seq=%0d committed on lane 1!",
                                 cycle_count, commit1_seq_id);
                        inv_errors <= inv_errors + 32'd1;
                    end
                    t_commit[commit1_seq_id]     <= cycle_count + 32'd1;
                    ev_committed[commit1_seq_id] <= 1'b1;
                end
                last_commit_seq <= commit1_seq_id;
            end else begin
                last_commit_seq <= commit0_seq_id;
            end
        end else begin
            cyc_since_commit <= cyc_since_commit + 32'd1;
            if (cyc_since_commit >= WATCHDOG_CYC) begin
                $display("MON2W FAIL [PROOF-LIVENESS cyc=%0d]: no commit for %0d cycles!",
                         cycle_count, cyc_since_commit);
                inv_errors <= inv_errors + 32'd1;
                cyc_since_commit <= 32'd0;
            end
        end

        // 5. Selective squash tracking
        if (flush_valid) begin
            for (k = 0; k < MAX_SEQ; k = k + 1) begin
                if (ev_valid[k] && !ev_committed[k] && !ev_squashed[k] && (k > flush_after_seq_id)) begin
                    ev_squashed[k] <= 1'b1;
                    t_squash[k]    <= cycle_count + 32'd1;
                end
            end
        end
    end
end

task automatic compute_and_check_proofs(
    input         require_ooo_issue,
    input         require_ooo_cdb,
    input         require_spec_squash,
    output [31:0] out_ooo_issue,
    output [31:0] out_ooo_cdb,
    output [31:0] out_spec_squash,
    output [31:0] out_errors
);
    integer i, j, lim;
    reg [31:0] iss_cnt, cdb_cnt, sq_cnt;
    begin
        iss_cnt = 32'd0;
        cdb_cnt = 32'd0;
        sq_cnt  = 32'd0;
        lim = (max_seq_seen < MAX_SEQ) ? (max_seq_seen + 1) : MAX_SEQ;

        for (i = 0; i < lim; i = i + 1) begin
            if (ev_valid[i] && ev_squashed[i] && !ev_committed[i] && (t_issue[i] != 32'd0))
                sq_cnt = sq_cnt + 32'd1;
            if (ev_valid[i] && ev_committed[i]) begin
                for (j = i + 1; j < lim; j = j + 1) begin
                    if (ev_valid[j] && ev_committed[j]) begin
                        if ((t_issue[i] != 32'd0) && (t_issue[j] != 32'd0) && (t_issue[j] < t_issue[i]))
                            iss_cnt = iss_cnt + 32'd1;
                        if ((t_cdb[i] != 32'd0) && (t_cdb[j] != 32'd0) && (t_cdb[j] < t_cdb[i]))
                            cdb_cnt = cdb_cnt + 32'd1;
                    end
                end
            end
        end

        ooo_issue_pairs      = iss_cnt;
        ooo_cdb_pairs        = cdb_cnt;
        spec_squashed_issued = sq_cnt;

        if (require_ooo_issue && (iss_cnt == 32'd0)) begin
            $display("MON2W FAIL [PROOF-OOO-ISSUE]: expected ooo_issue_pairs > 0!");
            inv_errors = inv_errors + 32'd1;
        end
        if (require_ooo_cdb && (cdb_cnt == 32'd0)) begin
            $display("MON2W FAIL [PROOF-OOO-COMPLETE]: expected ooo_cdb_pairs > 0!");
            inv_errors = inv_errors + 32'd1;
        end
        if (require_spec_squash && (sq_cnt == 32'd0)) begin
            $display("MON2W FAIL [PROOF-SPEC-SQUASH]: expected spec_squashed_issued > 0!");
            inv_errors = inv_errors + 32'd1;
        end

        out_ooo_issue   = iss_cnt;
        out_ooo_cdb     = cdb_cnt;
        out_spec_squash = sq_cnt;
        out_errors      = inv_errors;
    end
endtask

task automatic dump_gantt(input integer start_seq, input integer end_seq);
    integer s, c, min_c, max_c, lim_end;
    reg [7:0] ch;
    begin
        lim_end = (end_seq < MAX_SEQ) ? end_seq : (MAX_SEQ - 1);
        min_c = 32'h7FFF_FFFF;
        max_c = 0;
        for (s = start_seq; s <= lim_end; s = s + 1) begin
            if (ev_valid[s]) begin
                if (t_disp[s] > 0 && t_disp[s] < min_c) min_c = t_disp[s];
                if (t_commit[s] > max_c) max_c = t_commit[s];
                if (t_squash[s] > max_c) max_c = t_squash[s];
                if (t_cdb[s] > max_c)    max_c = t_cdb[s];
            end
        end
        if (max_c >= min_c) begin
            $display("--- 2-Wide Pipeline Timeline (D=Disp, I=Issue, E=Exec, W=CDB, C=Commit, X=Squash, .=Wait) cyc %0d..%0d ---",
                     min_c, max_c);
            for (s = start_seq; s <= lim_end; s = s + 1) begin
                if (ev_valid[s]) begin
                    $write("seq %3d | pc=0x%04x | 0x%08x | ", s, ev_pc[s][15:0], ev_instr[s]);
                    for (c = min_c; c <= max_c; c = c + 1) begin
                        ch = " ";
                        if (ev_squashed[s] && c == t_squash[s])           ch = "X";
                        else if (ev_committed[s] && c == t_commit[s])     ch = "C";
                        else if (t_cdb[s] != 0 && c == t_cdb[s])          ch = "W";
                        else if (t_issue[s] != 0 && c == t_issue[s])      ch = "I";
                        else if (t_disp[s] != 0 && c == t_disp[s])        ch = "D";
                        else if (t_issue[s] != 0 && c > t_issue[s] &&
                                 ((t_cdb[s] != 0 && c < t_cdb[s]) || (ev_squashed[s] && c < t_squash[s])))
                            ch = "E";
                        else if (t_disp[s] != 0 && c > t_disp[s] &&
                                 ((ev_committed[s] && c < t_commit[s]) || (ev_squashed[s] && c < t_squash[s])))
                            ch = ".";
                        $write("%c ", ch);
                    end
                    $display("");
                end
            end
        end
    end
endtask

task automatic dump_konata(input string filepath, input integer max_seq_out);
    integer fd, s, c, min_c, max_c, lim_end, ret_id;
    begin
        lim_end = (max_seq_seen < MAX_SEQ) ? max_seq_seen : (MAX_SEQ - 1);
        if (max_seq_out > 0 && max_seq_out < lim_end)
            lim_end = max_seq_out;
        min_c = 32'h7FFF_FFFF;
        max_c = 0;
        for (s = 1; s <= lim_end; s = s + 1) begin
            if (ev_valid[s]) begin
                if (t_disp[s] > 0 && t_disp[s] < min_c) min_c = t_disp[s];
                if (t_commit[s] > max_c) max_c = t_commit[s];
                if (t_squash[s] > max_c) max_c = t_squash[s];
                if (t_cdb[s] > max_c)    max_c = t_cdb[s];
            end
        end
        fd = $fopen(filepath, "w");
        if (fd != 0) begin
            $fwrite(fd, "Kanata\t0004\n");
            if (max_c >= min_c) begin
                $fwrite(fd, "C=\t%0d\n", min_c);
                ret_id = 0;
                for (c = min_c; c <= max_c + 1; c = c + 1) begin
                    if (c > min_c) $fwrite(fd, "C\t1\n");
                    for (s = 1; s <= lim_end; s = s + 1) begin
                        if (ev_valid[s]) begin
                            if (t_disp[s] == c) begin
                                $fwrite(fd, "I\t%0d\t%0d\t0\n", s, s);
                                $fwrite(fd, "L\t%0d\t0\t0x%04x: 0x%08x\n", s, ev_pc[s][15:0], ev_instr[s]);
                                $fwrite(fd, "S\t%0d\t0\tD\n", s);
                            end
                            if (t_issue[s] != 0 && t_issue[s] == c) $fwrite(fd, "S\t%0d\t0\tE\n", s);
                            if (t_cdb[s]   != 0 && t_cdb[s]   == c) $fwrite(fd, "S\t%0d\t0\tW\n", s);
                            if (ev_committed[s] && t_commit[s] == c) $fwrite(fd, "S\t%0d\t0\tC\n", s);
                            if (ev_committed[s] && (t_commit[s] + 1) == c) begin
                                ret_id = ret_id + 1;
                                $fwrite(fd, "R\t%0d\t%0d\t0\n", s, ret_id);
                            end
                            if (ev_squashed[s] && t_squash[s] == c) $fwrite(fd, "S\t%0d\t0\tX\n", s);
                            if (ev_squashed[s] && (t_squash[s] + 1) == c) $fwrite(fd, "R\t%0d\t0\t1\n", s);
                        end
                    end
                end
            end
            $fclose(fd);
        end
    end
endtask

endmodule
