#conda env name with a running jax install 
# cause I need jax (because I am a crackhead)
CONDA_ENV = base
HOME := ../../

#list of sv files : testbench.sv and everything in .sv in rtl/
SV_FOLDER := $(HOME)rtl/rtl_chip $(HOME)rtl/AXI_controls $(HOME)include
SV_INCLUDE := $(foreach dir, $(SV_FOLDER), +incdir+$(dir))
SV_FILES := $(wildcard $(foreach dir, $(SV_FOLDER), $(dir)/*.sv)) 

#build directory
LIB = memory_test

# ModelSim options
VSIM_FLAGS = -voptargs="+acc"

all: $(LIB) compile simulate gen_fst

$(LIB):
	vlib $(LIB)

compile: $(LIB)
	vlog -work $(LIB) -sv $(HOME)rtl/rtl_chip/RRAM_arrays/MC_8x8_upd.sv
	vlog -work $(LIB) -sv $(HOME)libs/axi/src/axi_pkg.sv
	vlog -work $(LIB) -sv +incdir+$(HOME)libs/axi/include $(HOME)libs/axi/src/axi_intf.sv $(HOME)include/adam_cfg_pkg.sv
#stopped test here 
	vlog -work $(LIB) -sv $(SV_INCLUDE) +incdir+$(HOME)libs/axi/include $(SV_FILES) testbench.sv

simulate: compile
	vsim -c -lib $(LIB) -suppress 3009 $(VSIM_FLAGS) -do wave.cmd tb_PWR_controls

gen_fst: simulate
	vcd2fst -v banzai.vcd -f banzai.fst

clean:
	rm -r $(LIB)
	rm test.txt transcript vsim.wlf

.PHONY: all clean compile simulate gen_test_vectors
