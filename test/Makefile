# Makefile
# See https://docs.cocotb.org/en/stable/quickstart.html for more info

# defaults
SIM ?= icarus
FST ?= -fst # Use more efficient FST format
TOPLEVEL_LANG ?= verilog

SRC_DIR = $(PWD)/../src
PROJECT_SOURCES = project.v sram_bb.v hamming_sram_2kb.v hamming_host_spi.v hamming_csr.v hamming_sram_arb.v

# Propagate GATES to the simulation environment (test.py reads it).
export GATES

PDK ?= ihp-sg13cmos5l
PDK_LIBS_REF = $(PDK_ROOT)/$(PDK)/libs.ref
PDK_CELL_PREFIX = $(patsubst ihp-%,%,$(PDK))

# SRAM behavioural model, in BOTH the RTL and GATES=yes builds: the real IHP
# model when the PDK is mounted (gl_test), the repo's local model otherwise.
ifneq ($(PDK_ROOT),)
SRAM_MODEL = $(PDK_LIBS_REF)/sg13cmos5l_sram/verilog/RM_IHPSG13_1P_512x32_c2_bm_bist.v $(PDK_LIBS_REF)/sg13cmos5l_sram/verilog/RM_IHPSG13_1P_core_behavioral_bm_bist.v
else
SRAM_MODEL = $(PWD)/sim_models/RM_IHPSG13_1P_512x32_c2_bm_bist.v
endif
VERILOG_SOURCES += $(SRAM_MODEL)
COMPILE_ARGS += -DFUNCTIONAL -DSIM

ifneq ($(GATES),yes)

# RTL simulation:
SIM_BUILD				= sim_build/rtl
VERILOG_SOURCES += $(addprefix $(SRC_DIR)/,$(PROJECT_SOURCES))

else

# Gate level simulation:
SIM_BUILD				= sim_build/gl
COMPILE_ARGS		+= -DGL_TEST
COMPILE_ARGS    += -DFUNCTIONAL
COMPILE_ARGS    += -DSIM
COMPILE_ARGS    += -DUSE_POWER_PINS=0

VERILOG_SOURCES += $(PDK_LIBS_REF)/$(PDK_CELL_PREFIX)_io/verilog/$(PDK_CELL_PREFIX)_io.v
VERILOG_SOURCES += $(wildcard $(PDK_LIBS_REF)/$(PDK_CELL_PREFIX)_stdcell/verilog/$(PDK_CELL_PREFIX)_udp.v)
VERILOG_SOURCES += $(PDK_LIBS_REF)/$(PDK_CELL_PREFIX)_stdcell/verilog/$(PDK_CELL_PREFIX)_stdcell.v

# this gets copied in by the GDS action workflow
VERILOG_SOURCES += $(PWD)/gate_level_netlist.v

endif

# Allow sharing configuration between test and precheck via `include`:
VERILOG_SOURCES += $(PWD)/tb.v

# See https://github.com/efabless/cocotb

# Include the testbench sources:
COMPILE_ARGS 		+= -I$(SRC_DIR)

# Include the testbench sources:
TOPLEVEL = tb

# List test modules:
COCOTB_TEST_MODULES = test

# include cocotb's make rules to take care of the simulator setup
include $(shell cocotb-config --makefiles)/Makefile.sim
