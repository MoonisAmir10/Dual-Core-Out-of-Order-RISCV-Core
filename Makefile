EDA_BIN   := /home/moonis/auto_job_seeker/.eda_tools/envs/dft/bin
IVERILOG  := $(EDA_BIN)/iverilog
VVP       := $(EDA_BIN)/vvp
VERILATOR := $(EDA_BIN)/verilator
YOSYS     := $(EDA_BIN)/yosys
PYTHON    := $(EDA_BIN)/python3

RISCV_GCC     := /usr/bin/riscv64-unknown-elf-gcc
RISCV_OBJCOPY := /usr/bin/riscv64-unknown-elf-objcopy
RISCV_OBJDUMP := /usr/bin/riscv64-unknown-elf-objdump

BUILD_DIR := build
INC_FLAGS := -Irtl/common -Irtl/coherence

RTL_COMMON := rtl/common/alu.v rtl/common/branch_comp.v rtl/common/imm_gen.v \
              rtl/common/decoder.sv rtl/common/mul_unit.sv rtl/common/div_unit.sv
RTL_MEM    := rtl/mem/icache_2w.sv rtl/mem/dcache_nb.sv rtl/mem/mem_arb_2to1.sv \
              rtl/mem/main_memory_nb.sv rtl/mem/l2_bank.sv rtl/mem/l2_cache_shared.sv \
              rtl/mem/main_memory_dram.sv
RTL_CORE   := rtl/core/instr_mem2w.sv rtl/core/regfile2w.sv rtl/core/btb_bht2w.sv \
              rtl/core/fetch2w.sv rtl/core/rat2w.sv rtl/core/rob2w.sv \
              rtl/core/rs_cluster2w.sv rtl/core/lsq2w.sv rtl/core/cdb_arbiter2w.sv \
              rtl/core/csr_unit.sv rtl/core/ooo2w_core.sv
RTL_SOC    := rtl/soc/ooo2w_soc.sv
RTL_P1_SOC := rtl/soc/single_ooo2w_p1_soc.sv
RTL_P2_SOC := rtl/soc/clint_mmio.sv rtl/soc/single_ooo2w_soc.sv

V_LINT  := --lint-only -Wall $(INC_FLAGS) -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM
V_BUILD := --binary --assert -Wall $(INC_FLAGS) -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM -Wno-DECLFILENAME --timescale 1ns/1ps -j 8

CFLAGS_BASE := -march=rv32ima -mabi=ilp32 -O2 -ffreestanding -nostdlib -nostartfiles \
               -Wall -Wextra -Isw/common -T sw/common/link.ld

.PHONY: all lint-phase0 lint-phase1 lint-phase2 synth-phase0 synth-phase1 synth-phase2 \
        test-phase0-unit test-phase1-unit test-phase2-unit test-phase0-asm \
        test-phase0-c-build test-phase0-baseline test-phase1-v2 test-phase2-l2 clean

all: lint-phase0 lint-phase1 lint-phase2 synth-phase0 synth-phase1 synth-phase2 \
     test-phase0-unit test-phase1-unit test-phase2-unit test-phase0-asm \
     test-phase0-c-build test-phase0-baseline test-phase1-v2 test-phase2-l2

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# ----------------------------------------------------------------------------
# Lint & Yosys Synthesis Cleanliness Gates (Phases 0 - 2)
# ----------------------------------------------------------------------------
lint-phase0:
	PATH=$(EDA_BIN):/usr/bin:/bin $(VERILATOR) $(V_LINT) --top-module decoder rtl/common/decoder.sv
	PATH=$(EDA_BIN):/usr/bin:/bin $(VERILATOR) $(V_LINT) --top-module csr_unit rtl/core/csr_unit.sv
	PATH=$(EDA_BIN):/usr/bin:/bin $(VERILATOR) $(V_LINT) --top-module dcache_nb rtl/mem/dcache_nb.sv
	PATH=$(EDA_BIN):/usr/bin:/bin $(VERILATOR) $(V_LINT) --top-module main_memory_nb rtl/mem/main_memory_nb.sv
	PATH=$(EDA_BIN):/usr/bin:/bin $(VERILATOR) $(V_LINT) --top-module ooo2w_soc $(RTL_COMMON) $(RTL_MEM) $(RTL_CORE) $(RTL_SOC)

lint-phase1:
	PATH=$(EDA_BIN):/usr/bin:/bin $(VERILATOR) $(V_LINT) --top-module icache_2w rtl/mem/icache_2w.sv
	PATH=$(EDA_BIN):/usr/bin:/bin $(VERILATOR) $(V_LINT) --top-module mem_arb_2to1 rtl/mem/mem_arb_2to1.sv
	PATH=$(EDA_BIN):/usr/bin:/bin $(VERILATOR) $(V_LINT) --top-module lsq2w rtl/core/lsq2w.sv
	PATH=$(EDA_BIN):/usr/bin:/bin $(VERILATOR) $(V_LINT) --top-module single_ooo2w_p1_soc $(RTL_COMMON) $(RTL_MEM) $(RTL_CORE) $(RTL_P1_SOC)

lint-phase2:
	PATH=$(EDA_BIN):/usr/bin:/bin $(VERILATOR) $(V_LINT) --top-module l2_bank rtl/mem/l2_bank.sv
	PATH=$(EDA_BIN):/usr/bin:/bin $(VERILATOR) $(V_LINT) --top-module l2_cache_shared rtl/mem/l2_bank.sv rtl/mem/l2_cache_shared.sv
	PATH=$(EDA_BIN):/usr/bin:/bin $(VERILATOR) $(V_LINT) --top-module main_memory_dram rtl/mem/main_memory_dram.sv
	PATH=$(EDA_BIN):/usr/bin:/bin $(VERILATOR) $(V_LINT) --top-module clint_mmio rtl/soc/clint_mmio.sv
	PATH=$(EDA_BIN):/usr/bin:/bin $(VERILATOR) $(V_LINT) --top-module single_ooo2w_soc $(RTL_COMMON) $(RTL_MEM) $(RTL_CORE) $(RTL_P2_SOC)

synth-phase0:
	$(YOSYS) -q -p "read_verilog -sv $(INC_FLAGS) rtl/common/decoder.sv; hierarchy -check -top decoder; proc; opt; check -assert"
	$(YOSYS) -q -p "read_verilog -sv $(INC_FLAGS) rtl/core/csr_unit.sv; hierarchy -check -top csr_unit; proc; opt; check -assert"
	$(YOSYS) -q -p "read_verilog -sv $(INC_FLAGS) rtl/mem/dcache_nb.sv; hierarchy -check -top dcache_nb; proc; opt; check -assert"
	$(YOSYS) -q -p "read_verilog -sv $(INC_FLAGS) $(RTL_COMMON) $(RTL_CORE); hierarchy -check -top ooo2w_core; proc; opt; check -assert"
	@echo "PASS [synth-phase0]: Yosys check -assert passed (0 latches, 0 errors)"

synth-phase1:
	$(YOSYS) -q -p "read_verilog -sv $(INC_FLAGS) rtl/mem/icache_2w.sv; hierarchy -check -top icache_2w; proc; opt; check -assert"
	$(YOSYS) -q -p "read_verilog -sv $(INC_FLAGS) rtl/mem/mem_arb_2to1.sv; hierarchy -check -top mem_arb_2to1; proc; opt; check -assert"
	@echo "PASS [synth-phase1]: Yosys check -assert passed on icache_2w & mem_arb_2to1 (0 latches, 0 errors)"

synth-phase2:
	$(YOSYS) -q -p "read_verilog -sv $(INC_FLAGS) rtl/mem/l2_bank.sv; hierarchy -check -top l2_bank -chparam NUM_SETS 8; proc; opt; check -assert"
	$(YOSYS) -q -p "read_verilog -sv $(INC_FLAGS) rtl/mem/l2_bank.sv rtl/mem/l2_cache_shared.sv; hierarchy -check -top l2_cache_shared -chparam SETS_PER_BANK 8; proc; opt; check -assert"
	$(YOSYS) -q -p "read_verilog -sv $(INC_FLAGS) rtl/soc/clint_mmio.sv; hierarchy -check -top clint_mmio; proc; opt; check -assert"
	@echo "PASS [synth-phase2]: Yosys check -assert passed on l2_bank, l2_cache_shared & clint_mmio (0 latches, 0 errors)"

# ----------------------------------------------------------------------------
# Unit Testbenches (Verilator + Icarus Verilog)
# ----------------------------------------------------------------------------
test-phase0-unit: $(BUILD_DIR)
	$(IVERILOG) -g2012 $(INC_FLAGS) -o $(BUILD_DIR)/tb_decoder.vvp rtl/common/decoder.sv tb/unit/tb_decoder.sv
	$(VVP) $(BUILD_DIR)/tb_decoder.vvp
	PATH=$(EDA_BIN):/usr/bin:/bin $(VERILATOR) $(V_BUILD) --Mdir $(BUILD_DIR)/v_tb_decoder -o Vtb_decoder --top-module tb_decoder rtl/common/decoder.sv tb/unit/tb_decoder.sv
	$(BUILD_DIR)/v_tb_decoder/Vtb_decoder
	$(IVERILOG) -g2012 $(INC_FLAGS) -o $(BUILD_DIR)/tb_csr_unit.vvp rtl/core/csr_unit.sv tb/unit/tb_csr_unit.sv
	$(VVP) $(BUILD_DIR)/tb_csr_unit.vvp
	PATH=$(EDA_BIN):/usr/bin:/bin $(VERILATOR) $(V_BUILD) --Mdir $(BUILD_DIR)/v_tb_csr_unit -o Vtb_csr_unit --top-module tb_csr_unit rtl/core/csr_unit.sv tb/unit/tb_csr_unit.sv
	$(BUILD_DIR)/v_tb_csr_unit/Vtb_csr_unit
	PATH=$(EDA_BIN):/usr/bin:/bin $(VERILATOR) $(V_BUILD) --Mdir $(BUILD_DIR)/v_tb_rs -o Vtb_rs --top-module tb_rs_cluster2w rtl/core/rs_cluster2w.sv tb/unit/tb_rs_cluster2w.sv
	$(BUILD_DIR)/v_tb_rs/Vtb_rs
	PATH=$(EDA_BIN):/usr/bin:/bin $(VERILATOR) $(V_BUILD) --Mdir $(BUILD_DIR)/v_tb_bp -o Vtb_bp --top-module tb_btb_bht2w rtl/core/btb_bht2w.sv tb/unit/tb_btb_bht2w.sv
	$(BUILD_DIR)/v_tb_bp/Vtb_bp
	PATH=$(EDA_BIN):/usr/bin:/bin $(VERILATOR) $(V_BUILD) --Mdir $(BUILD_DIR)/v_tb_dc -o Vtb_dc --top-module tb_dcache_nb rtl/mem/dcache_nb.sv rtl/mem/main_memory_nb.sv tb/unit/tb_dcache_nb.sv
	$(BUILD_DIR)/v_tb_dc/Vtb_dc

test-phase1-unit: $(BUILD_DIR)
	$(IVERILOG) -g2012 $(INC_FLAGS) -o $(BUILD_DIR)/tb_icache_2w.vvp rtl/mem/icache_2w.sv rtl/mem/main_memory_nb.sv tb/unit/tb_icache_2w.sv
	$(VVP) $(BUILD_DIR)/tb_icache_2w.vvp
	PATH=$(EDA_BIN):/usr/bin:/bin $(VERILATOR) $(V_BUILD) --Mdir $(BUILD_DIR)/v_tb_icache -o Vtb_icache --top-module tb_icache_2w rtl/mem/icache_2w.sv rtl/mem/main_memory_nb.sv tb/unit/tb_icache_2w.sv
	$(BUILD_DIR)/v_tb_icache/Vtb_icache
	$(IVERILOG) -g2012 $(INC_FLAGS) -o $(BUILD_DIR)/tb_lsq2w_replay.vvp rtl/core/lsq2w.sv tb/unit/tb_lsq2w_replay.sv
	$(VVP) $(BUILD_DIR)/tb_lsq2w_replay.vvp
	PATH=$(EDA_BIN):/usr/bin:/bin $(VERILATOR) $(V_BUILD) --Mdir $(BUILD_DIR)/v_tb_lsq_replay -o Vtb_lsq_replay --top-module tb_lsq2w_replay rtl/core/lsq2w.sv tb/unit/tb_lsq2w_replay.sv
	$(BUILD_DIR)/v_tb_lsq_replay/Vtb_lsq_replay

test-phase2-unit: $(BUILD_DIR)
	# 1. CLINT MMIO Unit TB (Icarus + Verilator)
	$(IVERILOG) -g2012 $(INC_FLAGS) -o $(BUILD_DIR)/tb_clint_mmio.vvp rtl/soc/clint_mmio.sv tb/unit/tb_clint_mmio.sv
	$(VVP) $(BUILD_DIR)/tb_clint_mmio.vvp
	PATH=$(EDA_BIN):/usr/bin:/bin $(VERILATOR) $(V_BUILD) --Mdir $(BUILD_DIR)/v_tb_clint -o Vtb_clint --top-module tb_clint_mmio rtl/soc/clint_mmio.sv tb/unit/tb_clint_mmio.sv
	$(BUILD_DIR)/v_tb_clint/Vtb_clint
	# 2. Shared 32 KB 4-Way 2-Bank L2 + 4-Outst DRAM Unit TB (Icarus + Verilator, 5000+ ops!)
	$(IVERILOG) -g2012 $(INC_FLAGS) -o $(BUILD_DIR)/tb_l2_cache_shared.vvp rtl/mem/l2_bank.sv rtl/mem/l2_cache_shared.sv rtl/mem/main_memory_dram.sv tb/unit/tb_l2_cache_shared.sv
	$(VVP) $(BUILD_DIR)/tb_l2_cache_shared.vvp
	PATH=$(EDA_BIN):/usr/bin:/bin $(VERILATOR) $(V_BUILD) --Mdir $(BUILD_DIR)/v_tb_l2 -o Vtb_l2 --top-module tb_l2_cache_shared rtl/mem/l2_bank.sv rtl/mem/l2_cache_shared.sv rtl/mem/main_memory_dram.sv tb/unit/tb_l2_cache_shared.sv
	$(BUILD_DIR)/v_tb_l2/Vtb_l2

# ----------------------------------------------------------------------------
# Phase 0, Phase 1 & Phase 2 End-to-End Suites
# ----------------------------------------------------------------------------
test-phase0-asm: $(BUILD_DIR)
	$(PYTHON) scripts/rv32ima_asm.py --self-test --hex $(BUILD_DIR)/phase0_rv32ima.hex --trace $(BUILD_DIR)/phase0_rv32ima.trace

test-phase0-c-build: $(BUILD_DIR)
	$(RISCV_GCC) $(CFLAGS_BASE) -DSINGLE_CORE_BASELINE sw/common/crt0.S sw/c_bench/smoke_c.c -o $(BUILD_DIR)/smoke_c.elf
	$(RISCV_OBJCOPY) -O binary $(BUILD_DIR)/smoke_c.elf $(BUILD_DIR)/smoke_c.bin
	$(PYTHON) scripts/elf2hex.py $(BUILD_DIR)/smoke_c.bin $(BUILD_DIR)/smoke_c.hex --pad-words 1024 --line128-out $(BUILD_DIR)/smoke_c_128.hex
	$(PYTHON) scripts/rv32ima_asm.py $(BUILD_DIR)/smoke_c.hex --run-hex --trace $(BUILD_DIR)/smoke_c.trace
	$(RISCV_GCC) $(CFLAGS_BASE) sw/common/crt0.S sw/c_bench/smoke_rv32ima_c.c -o $(BUILD_DIR)/smoke_rv32ima_c.elf
	$(RISCV_OBJCOPY) -O binary $(BUILD_DIR)/smoke_rv32ima_c.elf $(BUILD_DIR)/smoke_rv32ima_c.bin
	$(PYTHON) scripts/elf2hex.py $(BUILD_DIR)/smoke_rv32ima_c.bin $(BUILD_DIR)/smoke_rv32ima_c.hex --pad-words 1024 --line128-out $(BUILD_DIR)/smoke_rv32ima_c_128.hex
	$(PYTHON) scripts/rv32ima_asm.py $(BUILD_DIR)/smoke_rv32ima_c.hex --run-hex --unified --core-id 1 --trace $(BUILD_DIR)/smoke_rv32ima_c.trace
	@grep -q " 00000085 " $(BUILD_DIR)/smoke_rv32ima_c.trace && echo "PASS [test-phase0-c-build]: RV32IMA C binary (sync.h + mhartid=1) verified on golden ISS (x14=0x85)"

test-phase0-baseline: $(BUILD_DIR) test-phase0-c-build
	PATH=$(EDA_BIN):/usr/bin:/bin $(VERILATOR) $(V_BUILD) --Mdir $(BUILD_DIR)/v_ooo2w_soc -o Vooo2w_soc --top-module tb_ooo2w_soc $(RTL_COMMON) $(RTL_MEM) $(RTL_CORE) $(RTL_SOC) tb/common/rv32im_iss.sv tb/common/ooo_trace_mon2w.sv tb/soc/tb_ooo2w_soc.sv
	$(BUILD_DIR)/v_ooo2w_soc/Vooo2w_soc +HEX=$(BUILD_DIR)/smoke_c.hex +MAX_CYCLES=5000 +QUIET

test-phase1-v2: $(BUILD_DIR) test-phase0-c-build
	PATH=$(EDA_BIN):/usr/bin:/bin $(VERILATOR) $(V_BUILD) -GCORE_ID=0 -GMEM_LATENCY=4 -GENABLE_PREFETCH=1 --Mdir $(BUILD_DIR)/v_p1_c0 -o Vp1_c0 --top-module tb_single_ooo2w_p1_soc $(RTL_COMMON) $(RTL_MEM) $(RTL_CORE) $(RTL_P1_SOC) tb/common/ooo_trace_mon2w.sv tb/soc/tb_single_ooo2w_p1_soc.sv
	PATH=$(EDA_BIN):/usr/bin:/bin $(VERILATOR) $(V_BUILD) -GCORE_ID=1 -GMEM_LATENCY=4 -GENABLE_PREFETCH=1 --Mdir $(BUILD_DIR)/v_p1_c1 -o Vp1_c1 --top-module tb_single_ooo2w_p1_soc $(RTL_COMMON) $(RTL_MEM) $(RTL_CORE) $(RTL_P1_SOC) tb/common/ooo_trace_mon2w.sv tb/soc/tb_single_ooo2w_p1_soc.sv
	PATH=$(EDA_BIN):/usr/bin:/bin $(VERILATOR) $(V_BUILD) -GCORE_ID=0 -GMEM_LATENCY=16 -GENABLE_PREFETCH=0 --Mdir $(BUILD_DIR)/v_p1_nopf_l16 -o Vp1_nopf_l16 --top-module tb_single_ooo2w_p1_soc $(RTL_COMMON) $(RTL_MEM) $(RTL_CORE) $(RTL_P1_SOC) tb/common/ooo_trace_mon2w.sv tb/soc/tb_single_ooo2w_p1_soc.sv
	PATH=$(EDA_BIN):/usr/bin:/bin $(VERILATOR) $(V_BUILD) -GCORE_ID=0 -GMEM_LATENCY=16 -GENABLE_PREFETCH=1 --Mdir $(BUILD_DIR)/v_p1_pf_l16 -o Vp1_pf_l16 --top-module tb_single_ooo2w_p1_soc $(RTL_COMMON) $(RTL_MEM) $(RTL_CORE) $(RTL_P1_SOC) tb/common/ooo_trace_mon2w.sv tb/soc/tb_single_ooo2w_p1_soc.sv
	$(PYTHON) scripts/test_phase1_v2.py

test-phase2-l2: $(BUILD_DIR) test-phase0-c-build
	PATH=$(EDA_BIN):/usr/bin:/bin $(VERILATOR) $(V_BUILD) -GCORE_ID=0 -GDRAM_LATENCY=16 -GL2_HIT_LAT=3 -GENABLE_PREFETCH=1 --Mdir $(BUILD_DIR)/v_p2_c0 -o Vp2_c0 --top-module tb_single_ooo2w_soc $(RTL_COMMON) $(RTL_MEM) $(RTL_CORE) $(RTL_P2_SOC) tb/common/ooo_trace_mon2w.sv tb/soc/tb_single_ooo2w_soc.sv
	PATH=$(EDA_BIN):/usr/bin:/bin $(VERILATOR) $(V_BUILD) -GCORE_ID=1 -GDRAM_LATENCY=16 -GL2_HIT_LAT=3 -GENABLE_PREFETCH=1 --Mdir $(BUILD_DIR)/v_p2_c1 -o Vp2_c1 --top-module tb_single_ooo2w_soc $(RTL_COMMON) $(RTL_MEM) $(RTL_CORE) $(RTL_P2_SOC) tb/common/ooo_trace_mon2w.sv tb/soc/tb_single_ooo2w_soc.sv
	@test -x $(BUILD_DIR)/v_p1_pf_l16/Vp1_pf_l16 || \
	  PATH=$(EDA_BIN):/usr/bin:/bin $(VERILATOR) $(V_BUILD) -GCORE_ID=0 -GMEM_LATENCY=16 -GENABLE_PREFETCH=1 --Mdir $(BUILD_DIR)/v_p1_pf_l16 -o Vp1_pf_l16 --top-module tb_single_ooo2w_p1_soc $(RTL_COMMON) $(RTL_MEM) $(RTL_CORE) $(RTL_P1_SOC) tb/common/ooo_trace_mon2w.sv tb/soc/tb_single_ooo2w_p1_soc.sv
	$(PYTHON) scripts/test_phase2_l2.py

clean:
	rm -rf $(BUILD_DIR)
