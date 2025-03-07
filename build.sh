# Makefile for compiling and running modules and testbenches

# Compile and run a module
mod_%:
	iverilog -g2012 -o sim/$*.vvp mod/$*.sv
	vvp sim/$*.vvp > out/$*.out

# Compile and run a testbench
test_%:
	iverilog -g2012 -o sim/$*.vvp test/$*_tb.sv
	vvp sim/$*.vvp > out/$*.out

# Compile and run all testbenches
TEST_NAMES = $(basename $(notdir $(wildcard test/*_tb.sv)))
TEST_NAMES := $(foreach testname, $(TEST_NAMES), $(subst _tb,,$(testname)))
all_tests:
	@echo $(TEST_NAMES)
	$(foreach testname, $(TEST_NAMES), \
		iverilog -g2012 -o sim/$(testname).vvp test/$(testname)_tb.sv; \
		vvp sim/$(testname).vvp > out/$(testname).out;)

# Evaluate a file (FILE_PATH can be overridden)
FILE_PATH ?= tko/basic_add.tko
eval:
	iverilog -g2012 -o sim/eval.vvp eval.sv
	vvp sim/eval.vvp +FP=$(FILE_PATH) > out/eval.out

# Clean up generated files
clean:
	rm -rf sim/*.vvp out/*.out

.PHONY: mod_% test_% all_tests eval clean
