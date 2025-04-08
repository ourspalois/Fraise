#conda env name with a running jax install 
# cause I need jax (because I am a crackhead)
CONDA_ENV = VSIM_TESTS
HOME := ../../

#list of sv files : testbench.sv and everything in .sv in rtl/
SV_FOLDER := $(HOME)rtl/rtl_chip $(HOME)rtl/AXI_controls $(HOME)libs/includes
SV_INCLUDE := $(foreach dir, $(SV_FOLDER), +incdir+$(dir))
SV_FILES := $(wildcard $(foreach dir, $(SV_FOLDER), $(dir)/*.sv)) 

#build directory
LIB = memory_test

# ModelSim options
VSIM_FLAGS = -voptargs="+acc"

all: gen_test_vectors $(LIB) compile simulate

gen_test_vectors:
	conda run -n $(CONDA_ENV) python3 gen_test.py

$(LIB):
	vlib $(LIB)

compile: $(LIB)
	vlog -work $(LIB) -sv $(HOME)rtl/rtl_chip/RRAM_arrays/MC_8x8_upd.sv
	vlog -work $(LIB) -sv $(HOME)libs/axi/src/axi_pkg.sv
	vlog -work $(LIB) -sv +incdir+$(HOME)libs/axi/include $(HOME)libs/axi/src/axi_intf.sv $(HOME)libs/includes/adam_cfg_pkg.sv
#stopped test here 
	vlog -work $(LIB) -sv $(SV_INCLUDE) $(SV_FILES) tb_controls.sv

simulate: compile
	vsim -c -lib $(LIB) -suppress 3009 $(VSIM_FLAGS) -do wave.cmd tb_chip_control

clean:
	rm -r $(LIB)
	rm test.txt transcript vsim.wlf

.PHONY: all clean compile simulate gen_test_vectors