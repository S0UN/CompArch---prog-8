# Compiler and Simulator
COMPILER = iverilog
SIMULATOR = vvp

# Directories
HDL_DIR = hdl
TEST_DIR = test
VVP_DIR = vvp
OUTPUT_DIR = testOutputs

# Source and Testbench Files
SRC_FILE = $(HDL_DIR)/tinker.sv
TEST_FILES = $(wildcard $(TEST_DIR)/*.sv)

# Create output directories if they don't exist
$(shell mkdir -p $(VVP_DIR) $(OUTPUT_DIR))

# Targets
all: test_alu test_register_file test_instruction_decoder test_tinker_core

# Test ALU/FPU
test_alu:
	$(COMPILER) -g2012 -o $(VVP_DIR)/alu.vvp $(TEST_DIR)/test_alu.sv $(SRC_FILE)
	$(SIMULATOR) $(VVP_DIR)/alu.vvp > $(OUTPUT_DIR)/alu.out

# Test Register File
test_register_file:
	$(COMPILER) -g2012 -o $(VVP_DIR)/register_file.vvp $(TEST_DIR)/test_register_file.sv $(SRC_FILE)
	$(SIMULATOR) $(VVP_DIR)/register_file.vvp > $(OUTPUT_DIR)/register_file.out

# Test Instruction Decoder
test_instruction_decoder:
	$(COMPILER) -g2012 -o $(VVP_DIR)/instruction_decoder.vvp $(TEST_DIR)/test_instruction_decoder.sv $(SRC_FILE)
	$(SIMULATOR) $(VVP_DIR)/instruction_decoder.vvp > $(OUTPUT_DIR)/instruction_decoder.out

# Test Tinker Core
test_tinker_core:
	$(COMPILER) -g2012 -o $(VVP_DIR)/tinker_core.vvp $(TEST_DIR)/test_tinker_core.sv $(SRC_FILE)
	$(SIMULATOR) $(VVP_DIR)/tinker_core.vvp > $(OUTPUT_DIR)/tinker_core.out

# Clean up
clean:
	rm -rf $(VVP_DIR)/*.vvp $(OUTPUT_DIR)/*.out

.PHONY: all test_alu test_register_file test_instruction_decoder test_tinker_core clean