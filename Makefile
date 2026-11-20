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
RTL_MEM    := rtl/mem/dcache_nb.sv rtl/mem/main_memory_nb.sv
RTL_CORE   := rtl/core/instr_mem2w.sv rtl/core/regfile2w.sv rtl/core/btb_bht2w.sv \
              rtl/core/fetch2w.sv rtl/core/rat2w.sv rtl/core/rob2w.sv \
              rtl/core/rs_cluster2w.sv rtl/core/lsq2w.sv rtl/core/cdb_arbiter2w.sv \
              rtl/core/csr_unit.sv rtl/core/ooo2w_core.sv
RTL_SOC    := rtl/soc/ooo2w_soc.sv

V_LINT  := --lint-only -Wall $(INC_FLAGS) -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM
V_BUILD := --binary --assert -Wall $(INC_FLAGS) -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM -Wno-DECLFILENAME --timescale 1ns/1ps -j 2

CFLAGS_BASE := -march=rv32ima -mabi=ilp32 -O2 -ffreestanding -nostdlib -nostartfiles \
               -Wall -Wextra -Isw/common -T sw/common/link.ld

.PHONY: all lint-phase0 synth-phase0 test-phase0-unit test-phase0-asm \
        test-phase0-c-build test-phase0-baseline clean

all: lint-phase0 synth-phase0 test-phase0-unit test-phase0-asm \
     test-phase0-c-build test-phase0-baseline

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# ----------------------------------------------------------------------------
# Phase 0: Lint & Yosys Synthesis Cleanliness Gates
# ----------------------------------------------------------------------------
lint-phase0:
	PATH=$(EDA_BIN):/usr/bin:/bin $(VERILATOR) $(V_LINT) --top-module decoder rtl/common/decoder.sv
	PATH=$(EDA_BIN):/usr/bin:/bin $(VERILATOR) $(V_LINT) --top-module csr_unit rtl/core/csr_unit.sv
	PATH=$(EDA_BIN):/usr/bin:/bin $(VERILATOR) $(V_LINT) --top-module dcache_nb rtl/mem/dcache_nb.sv
	PATH=$(EDA_BIN):/usr/bin:/bin $(VERILATOR) $(V_LINT) --top-module main_memory_nb rtl/mem/main_memory_nb.sv
	PATH=$(EDA_BIN):/usr/bin:/bin $(VERILATOR) $(V_LINT) --top-module ooo2w_soc $(RTL_COMMON) $(RTL_MEM) $(RTL_CORE) $(RTL_SOC)

synth-phase0:
	$(YOSYS) -q -p "read_verilog -sv $(INC_FLAGS) rtl/common/decoder.sv; hierarchy -check -top decoder; proc; opt; check -assert"
	$(YOSYS) -q -p "read_verilog -sv $(INC_FLAGS) rtl/core/csr_unit.sv; hierarchy -check -top csr_unit; proc; opt; check -assert"
	$(YOSYS) -q -p "read_verilog -sv $(INC_FLAGS) rtl/mem/dcache_nb.sv; hierarchy -check -top dcache_nb; proc; opt; check -assert"
	$(YOSYS) -q -p "read_verilog -sv $(INC_FLAGS) $(RTL_COMMON) $(RTL_CORE); hierarchy -check -top ooo2w_core; proc; opt; check -assert"
	@echo "PASS [synth-phase0]: Yosys check -assert passed (0 latches, 0 errors)"

# ----------------------------------------------------------------------------
# Phase 0: Unit Testbenches (Verilator + Icarus Verilog)
# ----------------------------------------------------------------------------
test-phase0-unit: $(BUILD_DIR)
	# 1. RV32IMA_Zicsr_Zifencei Decoder
	$(IVERILOG) -g2012 $(INC_FLAGS) -o $(BUILD_DIR)/tb_decoder.vvp rtl/common/decoder.sv tb/unit/tb_decoder.sv
	$(VVP) $(BUILD_DIR)/tb_decoder.vvp
	PATH=$(EDA_BIN):/usr/bin:/bin $(VERILATOR) $(V_BUILD) --Mdir $(BUILD_DIR)/v_tb_decoder -o Vtb_decoder --top-module tb_decoder rtl/common/decoder.sv tb/unit/tb_decoder.sv
	$(BUILD_DIR)/v_tb_decoder/Vtb_decoder
	# 2. Dual-Core Zicsr Unit (CORE_ID=0 & CORE_ID=1)
	$(IVERILOG) -g2012 $(INC_FLAGS) -o $(BUILD_DIR)/tb_csr_unit.vvp rtl/core/csr_unit.sv tb/unit/tb_csr_unit.sv
	$(VVP) $(BUILD_DIR)/tb_csr_unit.vvp
	PATH=$(EDA_BIN):/usr/bin:/bin $(VERILATOR) $(V_BUILD) --Mdir $(BUILD_DIR)/v_tb_csr_unit -o Vtb_csr_unit --top-module tb_csr_unit rtl/core/csr_unit.sv tb/unit/tb_csr_unit.sv
	$(BUILD_DIR)/v_tb_csr_unit/Vtb_csr_unit
	# 3. Imported ooo2w Core Unit Regression (RS, BP+RAS, Non-Blocking D-Cache)
	PATH=$(EDA_BIN):/usr/bin:/bin $(VERILATOR) $(V_BUILD) --Mdir $(BUILD_DIR)/v_tb_rs -o Vtb_rs --top-module tb_rs_cluster2w rtl/core/rs_cluster2w.sv tb/unit/tb_rs_cluster2w.sv
	$(BUILD_DIR)/v_tb_rs/Vtb_rs
	PATH=$(EDA_BIN):/usr/bin:/bin $(VERILATOR) $(V_BUILD) --Mdir $(BUILD_DIR)/v_tb_bp -o Vtb_bp --top-module tb_btb_bht2w rtl/core/btb_bht2w.sv tb/unit/tb_btb_bht2w.sv
	$(BUILD_DIR)/v_tb_bp/Vtb_bp
	PATH=$(EDA_BIN):/usr/bin:/bin $(VERILATOR) $(V_BUILD) --Mdir $(BUILD_DIR)/v_tb_dc -o Vtb_dc --top-module tb_dcache_nb rtl/mem/dcache_nb.sv rtl/mem/main_memory_nb.sv tb/unit/tb_dcache_nb.sv
	$(BUILD_DIR)/v_tb_dc/Vtb_dc

# ----------------------------------------------------------------------------
# Phase 0: RV32IMA Assembler & Golden ISS Self-Test
# ----------------------------------------------------------------------------
test-phase0-asm: $(BUILD_DIR)
	$(PYTHON) scripts/rv32ima_asm.py --self-test --hex $(BUILD_DIR)/phase0_rv32ima.hex --trace $(BUILD_DIR)/phase0_rv32ima.trace

# ----------------------------------------------------------------------------
# Phase 0: Bare-Metal RISC-V GCC C Toolchain Verification (RV32IM + RV32IMA)
# ----------------------------------------------------------------------------
test-phase0-c-build: $(BUILD_DIR)
	# A. Compile RV32IM-compatible C smoke test (runs on baseline ooo2w_soc!)
	$(RISCV_GCC) $(CFLAGS_BASE) -DSINGLE_CORE_BASELINE sw/common/crt0.S sw/c_bench/smoke_c.c -o $(BUILD_DIR)/smoke_c.elf
	$(RISCV_OBJCOPY) -O binary $(BUILD_DIR)/smoke_c.elf $(BUILD_DIR)/smoke_c.bin
	$(PYTHON) scripts/elf2hex.py $(BUILD_DIR)/smoke_c.bin $(BUILD_DIR)/smoke_c.hex --pad-words 1024 --line128-out $(BUILD_DIR)/smoke_c_128.hex
	$(PYTHON) scripts/rv32ima_asm.py $(BUILD_DIR)/smoke_c.hex --run-hex --trace $(BUILD_DIR)/smoke_c.trace
	# B. Compile full RV32IMA + Zicsr + Zifencei C test (sync.h atomics + mhartid!)
	$(RISCV_GCC) $(CFLAGS_BASE) sw/common/crt0.S sw/c_bench/smoke_rv32ima_c.c -o $(BUILD_DIR)/smoke_rv32ima_c.elf
	$(RISCV_OBJCOPY) -O binary $(BUILD_DIR)/smoke_rv32ima_c.elf $(BUILD_DIR)/smoke_rv32ima_c.bin
	$(PYTHON) scripts/elf2hex.py $(BUILD_DIR)/smoke_rv32ima_c.bin $(BUILD_DIR)/smoke_rv32ima_c.hex --pad-words 1024 --line128-out $(BUILD_DIR)/smoke_rv32ima_c_128.hex
	$(PYTHON) scripts/rv32ima_asm.py $(BUILD_DIR)/smoke_rv32ima_c.hex --run-hex --unified --core-id 1 --trace $(BUILD_DIR)/smoke_rv32ima_c.trace
	@grep -q " 00000085 " $(BUILD_DIR)/smoke_rv32ima_c.trace && echo "PASS [test-phase0-c-build]: RV32IMA C binary (sync.h + mhartid=1) verified on golden ISS (x14=0x85)"

# ----------------------------------------------------------------------------
# Phase 0: Baseline ooo2w_soc RTL Verification with Compiled C Binary!
# ----------------------------------------------------------------------------
test-phase0-baseline: $(BUILD_DIR) test-phase0-c-build
	PATH=$(EDA_BIN):/usr/bin:/bin $(VERILATOR) $(V_BUILD) --Mdir $(BUILD_DIR)/v_ooo2w_soc -o Vooo2w_soc --top-module tb_ooo2w_soc $(RTL_COMMON) $(RTL_MEM) $(RTL_CORE) $(RTL_SOC) tb/common/rv32im_iss.sv tb/common/ooo_trace_mon2w.sv tb/soc/tb_ooo2w_soc.sv
	$(BUILD_DIR)/v_ooo2w_soc/Vooo2w_soc +HEX=$(BUILD_DIR)/smoke_c.hex +MAX_CYCLES=5000 +QUIET

clean:
	rm -rf $(BUILD_DIR)
