`timescale 1ns / 1ps

// ============================================================================
// 2-Wide Floating-Point Register Alias Table (FP RAT) w/ 4 Coherent Checkpoints
// (Phase U6 — Full 32-Register f0..f31 Renaming + 6 Source Lookups for FMA!)
// ============================================================================
module frat2w #(
    parameter integer TAG_W  = 5,
    parameter integer NUM_CP = 4
) (
    input  wire             clk,
    input  wire             reset_n,
    input  wire             flush,
    input  wire [TAG_W-1:0] rob_head,

    // 6 Combinational FP source lookup ports (Lane 0 rs1/rs2/rs3, Lane 1 rs1/rs2/rs3)
    input  wire [4:0]       l0_rs1_addr,
    output wire             l0_rs1_busy,
    output wire [TAG_W-1:0] l0_rs1_tag,
    input  wire [4:0]       l0_rs2_addr,
    output wire             l0_rs2_busy,
    output wire [TAG_W-1:0] l0_rs2_tag,
    input  wire [4:0]       l0_rs3_addr,
    output wire             l0_rs3_busy,
    output wire [TAG_W-1:0] l0_rs3_tag,

    input  wire [4:0]       l1_rs1_addr,
    output wire             l1_rs1_busy,
    output wire [TAG_W-1:0] l1_rs1_tag,
    input  wire [4:0]       l1_rs2_addr,
    output wire             l1_rs2_busy,
    output wire [TAG_W-1:0] l1_rs2_tag,
    input  wire [4:0]       l1_rs3_addr,
    output wire             l1_rs3_busy,
    output wire [TAG_W-1:0] l1_rs3_tag,

    // 2-Wide FP dispatch rename ports (disp1 younger than disp0)
    input  wire             disp0_we,
    input  wire [4:0]       disp0_rd,
    input  wire [TAG_W-1:0] disp0_tag,
    output wire             disp0_rd_was_busy,

    input  wire             disp1_we,
    input  wire [4:0]       disp1_rd,
    input  wire [TAG_W-1:0] disp1_tag,
    output wire             disp1_rd_was_busy,

    // 2-Wide in-order FP ROB commit retire ports
    input  wire             commit0_we,
    input  wire [4:0]       commit0_rd,
    input  wire [TAG_W-1:0] commit0_tag,

    input  wire             commit1_we,
    input  wire [4:0]       commit1_rd,
    input  wire [TAG_W-1:0] commit1_tag,

    // Synchronized Branch Checkpoint & Fast Selective Recovery interface
    input  wire             cp_take,
    input  wire             cp_is_lane1,
    input  wire [TAG_W-1:0] cp_take_tag,
    output wire             cp_ready,
    input  wire             br_resolve_ok,
    input  wire [TAG_W-1:0] br_resolve_tag,
    input  wire             squash_valid,
    input  wire [TAG_W-1:0] squash_tag
);

localparam integer CP_IDX_W = (NUM_CP <= 2) ? 1 : $clog2(NUM_CP);

// Live FP RAT (all 32 registers f0..f31 are architectural!)
reg             busy_r [0:31];
reg [TAG_W-1:0] tag_r  [0:31];

// Coherent Branch Checkpoint Bank
reg                   cp_valid   [0:NUM_CP-1];
reg [TAG_W-1:0]       cp_rob_tag [0:NUM_CP-1];
reg [31:0]            cp_busy    [0:NUM_CP-1];
reg [32*TAG_W-1:0]    cp_tag     [0:NUM_CP-1];

assign l0_rs1_busy = busy_r[l0_rs1_addr];
assign l0_rs1_tag  = tag_r[l0_rs1_addr];
assign l0_rs2_busy = busy_r[l0_rs2_addr];
assign l0_rs2_tag  = tag_r[l0_rs2_addr];
assign l0_rs3_busy = busy_r[l0_rs3_addr];
assign l0_rs3_tag  = tag_r[l0_rs3_addr];

assign l1_rs1_busy = busy_r[l1_rs1_addr];
assign l1_rs1_tag  = tag_r[l1_rs1_addr];
assign l1_rs2_busy = busy_r[l1_rs2_addr];
assign l1_rs2_tag  = tag_r[l1_rs2_addr];
assign l1_rs3_busy = busy_r[l1_rs3_addr];
assign l1_rs3_tag  = tag_r[l1_rs3_addr];

assign disp0_rd_was_busy = busy_r[disp0_rd];
assign disp1_rd_was_busy = busy_r[disp1_rd] || (disp0_we && (disp0_rd == disp1_rd));

// Free checkpoint slot finder
reg                cp_has_free;
reg [CP_IDX_W-1:0] cp_free_idx;
integer f_c;

always @(*) begin
    f_c         = 0;
    cp_has_free = 1'b0;
    cp_free_idx = {CP_IDX_W{1'b0}};
    for (f_c = NUM_CP - 1; f_c >= 0; f_c = f_c - 1) begin
        if (!cp_valid[f_c]) begin
            cp_has_free = 1'b1;
            cp_free_idx = f_c[CP_IDX_W-1:0];
        end
    end
end

assign cp_ready = cp_has_free && !flush && !squash_valid;

// Matching checkpoint lookup on squash_valid
reg                cp_hit;
reg [CP_IDX_W-1:0] cp_hit_idx;
integer h_c;

always @(*) begin
    h_c        = 0;
    cp_hit     = 1'b0;
    cp_hit_idx = {CP_IDX_W{1'b0}};
    for (h_c = 0; h_c < NUM_CP; h_c = h_c + 1) begin
        if (cp_valid[h_c] && (cp_rob_tag[h_c] == squash_tag)) begin
            cp_hit     = 1'b1;
            cp_hit_idx = h_c[CP_IDX_W-1:0];
        end
    end
end

wire [TAG_W-1:0] sq_br_age = squash_tag - rob_head;

integer r, c;

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        for (r = 0; r < 32; r = r + 1) begin
            busy_r[r] <= 1'b0;
            tag_r[r]  <= {TAG_W{1'b0}};
        end
        for (c = 0; c < NUM_CP; c = c + 1) begin
            cp_valid[c]   <= 1'b0;
            cp_rob_tag[c] <= {TAG_W{1'b0}};
            cp_busy[c]    <= 32'd0;
            cp_tag[c]     <= {(32*TAG_W){1'b0}};
        end
    end else if (flush) begin
        for (r = 0; r < 32; r = r + 1)
            busy_r[r] <= 1'b0;
        for (c = 0; c < NUM_CP; c = c + 1)
            cp_valid[c] <= 1'b0;
    end else if (squash_valid) begin
        // Restore live FP RAT from checkpoint, applying simultaneous commit0 & commit1!
        for (r = 0; r < 32; r = r + 1) begin
            if (cp_hit) begin
                if ((commit0_we && (commit0_rd == r[4:0]) && cp_busy[cp_hit_idx][r] &&
                     (cp_tag[cp_hit_idx][r*TAG_W +: TAG_W] == commit0_tag)) ||
                    (commit1_we && (commit1_rd == r[4:0]) && cp_busy[cp_hit_idx][r] &&
                     (cp_tag[cp_hit_idx][r*TAG_W +: TAG_W] == commit1_tag))) begin
                    busy_r[r] <= 1'b0;
                end else begin
                    busy_r[r] <= cp_busy[cp_hit_idx][r];
                end
                tag_r[r] <= cp_tag[cp_hit_idx][r*TAG_W +: TAG_W];
            end else begin
                busy_r[r] <= 1'b0;
            end
        end

        // Free recovered + younger CPs; keep older CPs coherent with commit0 & commit1
        for (c = 0; c < NUM_CP; c = c + 1) begin
            if (cp_valid[c]) begin
                if ((cp_rob_tag[c] == squash_tag) ||
                    ((cp_rob_tag[c] - rob_head) > sq_br_age)) begin
                    cp_valid[c] <= 1'b0;
                end else begin
                    for (r = 0; r < 32; r = r + 1) begin
                        if ((commit0_we && (commit0_rd == r[4:0]) && cp_busy[c][r] &&
                             (cp_tag[c][r*TAG_W +: TAG_W] == commit0_tag)) ||
                            (commit1_we && (commit1_rd == r[4:0]) && cp_busy[c][r] &&
                             (cp_tag[c][r*TAG_W +: TAG_W] == commit1_tag))) begin
                            cp_busy[c][r] <= 1'b0;
                        end
                    end
                end
            end
        end
    end else begin
        // 1. Commit0 & Commit1 clear matching aliases in live FP RAT & all valid checkpoints
        for (r = 0; r < 32; r = r + 1) begin
            if ((commit0_we && (commit0_rd == r[4:0]) && busy_r[r] && (tag_r[r] == commit0_tag)) ||
                (commit1_we && (commit1_rd == r[4:0]) && busy_r[r] && (tag_r[r] == commit1_tag))) begin
                busy_r[r] <= 1'b0;
            end
        end
        for (c = 0; c < NUM_CP; c = c + 1) begin
            if (cp_valid[c]) begin
                for (r = 0; r < 32; r = r + 1) begin
                    if ((commit0_we && (commit0_rd == r[4:0]) && cp_busy[c][r] &&
                         (cp_tag[c][r*TAG_W +: TAG_W] == commit0_tag)) ||
                        (commit1_we && (commit1_rd == r[4:0]) && cp_busy[c][r] &&
                         (cp_tag[c][r*TAG_W +: TAG_W] == commit1_tag))) begin
                        cp_busy[c][r] <= 1'b0;
                    end
                end
            end
        end

        // 2. Correctly-predicted branch resolution frees its checkpoint
        if (br_resolve_ok) begin
            for (c = 0; c < NUM_CP; c = c + 1) begin
                if (cp_valid[c] && (cp_rob_tag[c] == br_resolve_tag))
                    cp_valid[c] <= 1'b0;
            end
        end

        // 3. 2-Wide FP Dispatch renames (f0 IS writable & renamed!)
        if (disp0_we) begin
            busy_r[disp0_rd] <= 1'b1;
            tag_r[disp0_rd]  <= disp0_tag;
        end
        if (disp1_we) begin
            busy_r[disp1_rd] <= 1'b1;
            tag_r[disp1_rd]  <= disp1_tag;
        end

        // 4. Branch Checkpoint Take
        if (cp_take && cp_has_free) begin
            cp_valid[cp_free_idx]   <= 1'b1;
            cp_rob_tag[cp_free_idx] <= cp_take_tag;
            for (r = 0; r < 32; r = r + 1) begin
                if (cp_is_lane1 && disp1_we && (disp1_rd == r[4:0])) begin
                    cp_busy[cp_free_idx][r]               <= 1'b1;
                    cp_tag[cp_free_idx][r*TAG_W +: TAG_W] <= disp1_tag;
                end else if (disp0_we && (disp0_rd == r[4:0])) begin
                    cp_busy[cp_free_idx][r]               <= 1'b1;
                    cp_tag[cp_free_idx][r*TAG_W +: TAG_W] <= disp0_tag;
                end else if ((commit0_we && (commit0_rd == r[4:0]) && busy_r[r] && (tag_r[r] == commit0_tag)) ||
                             (commit1_we && (commit1_rd == r[4:0]) && busy_r[r] && (tag_r[r] == commit1_tag))) begin
                    cp_busy[cp_free_idx][r]               <= 1'b0;
                    cp_tag[cp_free_idx][r*TAG_W +: TAG_W] <= tag_r[r];
                end else begin
                    cp_busy[cp_free_idx][r]               <= busy_r[r];
                    cp_tag[cp_free_idx][r*TAG_W +: TAG_W] <= tag_r[r];
                end
            end
        end
    end
end

endmodule
