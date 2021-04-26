###############################################################################
# Makefile
###############################################################################
SRC_DIR=.
TARGET=test
ENABLE_LST=yes

ifeq ($(PICO_SDK_PATH),)
  ${error PICO_SDK_PATH must be defined}
endif

EXTRA_SRC = crt0.S boot_stage2.S 

LINKER_SCRIPT = $(PICO_SDK_PATH)/src/rp2_common/pico_standard_link/memmap_default.ld

INCLUDE_PATH += $(PICO_SDK_PATH)/src/rp2040/hardware_regs/include
INCLUDE_PATH += $(PICO_SDK_PATH)/src/common/pico_binary_info/include

SRC_DIR      += $(PICO_SDK_PATH)/src/rp2_common/pico_bootrom
INCLUDE_PATH += $(PICO_SDK_PATH)/src/rp2_common/pico_bootrom/include

INCLUDE_PATH += $(PICO_SDK_PATH)/src/rp2_common/hardware_gpio/include
INCLUDE_PATH += $(PICO_SDK_PATH)/src/rp2040/hardware_structs/include
INCLUDE_PATH += $(PICO_SDK_PATH)/src/rp2_common/hardware_base/include
INCLUDE_PATH += $(PICO_SDK_PATH)/src/rp2_common/hardware_sync/include

INCLUDE_PATH += $(PICO_SDK_PATH)/src/rp2_common/hardware_resets/include

ADDITIONAL_TARGETS=$(TARGET).uf2
ADDITIONAL_CLEAN=$(TARGET).uf2 elf2uf2

include makefile.mk

###############################################################################
# Rules: Build ELF2UF tool
###############################################################################
elf2uf2:
	@echo "# Building $@"
	@$(CXX) -o $@ -I$(PICO_SDK_PATH)/tools/elf2uf2 -I$(PICO_SDK_PATH)/src/common/boot_uf2/include $(PICO_SDK_PATH)/tools/elf2uf2/main.cpp 

###############################################################################
# Rules: Convert ELF to UF2
###############################################################################
$(TARGET).uf2: $(EXE_DIR)$(TARGET) elf2uf2
	@echo "# Converting to $@"
	@./elf2uf2 $< $@