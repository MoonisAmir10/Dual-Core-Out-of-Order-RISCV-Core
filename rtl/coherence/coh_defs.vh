// ============================================================================
// MOESI Coherence Protocol & 3-Channel Snooping Interconnect Definitions
// (Phase 3: Dual-Core 2-Wide Out-of-Order RV32IMA_Zicsr_Zifencei Processor)
// Note: Intentionally unguarded so localparams are visible inside every module
//       that `include`s this header in the same compilation unit!
// ============================================================================

// ----------------------------------------------------------------------------
// 1. MOESI Stable Line States (3-bit encoding per way/set in Main & Snoop Tags)
// ----------------------------------------------------------------------------
localparam [2:0] COH_I = 3'd0; // Invalid
localparam [2:0] COH_S = 3'd1; // Shared (clean or peer owns in O; read-only)
localparam [2:0] COH_E = 3'd2; // Exclusive (clean, sole L1 copy; silent E->M!)
localparam [2:0] COH_O = 3'd3; // Owned (dirty, shared with peer S; C2C + L2 WB owner)
localparam [2:0] COH_M = 3'd4; // Modified (dirty, sole L1 copy; 1-cyc R/W hit)

// ----------------------------------------------------------------------------
// 2. Channel A: Ordered Coherence Request / Snoop Broadcast Opcodes (2-bit)
// ----------------------------------------------------------------------------
localparam [1:0] COH_REQ_GETS    = 2'd0; // Read miss (requests S or E if peer misses)
localparam [1:0] COH_REQ_GETM    = 2'd1; // Write/AMO miss (requests M, invalidates peer)
localparam [1:0] COH_REQ_UPGRADE = 2'd2; // S/O -> M upgrade (invalidates peer, 0 L2 read)
localparam [1:0] COH_REQ_PUTM    = 2'd3; // Reserved (dirty WBs use dedicated Channel C)

// ----------------------------------------------------------------------------
// 3. Channel B/C: Peer Snoop Response Opcodes (2-bit)
// ----------------------------------------------------------------------------
localparam [1:0] SNP_RESP_MISS   = 2'd0; // Peer does not hold line -> GetS installs E
localparam [1:0] SNP_RESP_SHARED = 2'd1; // Peer holds clean S/E (E->S) -> GetS installs S
localparam [1:0] SNP_RESP_INVACK = 2'd2; // Peer invalidated S/E/O->I (no data transfer)
localparam [1:0] SNP_RESP_DATA   = 2'd3; // Peer supplies 128-bit dirty C2C data (M/O/WBB)
