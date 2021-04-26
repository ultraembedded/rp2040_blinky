###############################################################################
# Inputs
###############################################################################
# SRC_DIR
SRC_DIR ?= .
# INCLUDE_PATH
# TARGET
TARGET ?= target
# EXTRA_CFLAGS
# EXTRA_LIBS
# EXTRA_LIBDIRS
# GCC_PREFIX     = arch-toolchain-
# OPT            = [0-2]
OPT ?= 0
# ARCH_CFLAGS
# ARCH_LFLAGS
# COMPILER       = g++ | gcc

# Extra targets and clean objects
ADDITIONAL_TARGETS ?= 
ADDITIONAL_CLEAN   ?=

###############################################################################
# Compiler flags
###############################################################################
# GCC_PREFIX     = arch-toolchain-
GCC_PREFIX=arm-none-eabi-
# ARCH_CFLAGS
ARCH_CFLAGS=-mcpu=cortex-m0plus -mthumb -ffunction-sections -fdata-sections -fmessage-length=0
# ARCH_LFLAGS
ARCH_LFLAGS=-nostartfiles -nodefaultlibs -lgcc -lm -mcpu=cortex-m0plus -mthumb -Wl,--gc-sections
# COMPILER       = g++ | gcc
COMPILER = gcc

###############################################################################
# Linker Script
###############################################################################
LINKER_SCRIPT ?= none
ARCH_LFLAGS += -T$(LINKER_SCRIPT)

###############################################################################
# Variables: Dirs
###############################################################################
OBJ_DIR      ?= obj.$(TARGET)/
DEP_DIR      ?= dep.$(TARGET)/
EXE_DIR      ?= build.$(TARGET)/

###############################################################################
# Variables: GCC
###############################################################################
QUIET        ?= yes

GCC_PREFIX   ?= 
COMPILER     ?= g++

ifeq ($(QUIET),yes)
GCC          = @$(GCC_PREFIX)$(COMPILER)
OBJCOPY      = @$(GCC_PREFIX)objcopy
OBJDUMP      = @$(GCC_PREFIX)objdump
else
GCC          = $(GCC_PREFIX)$(COMPILER)
OBJCOPY      = $(GCC_PREFIX)objcopy
OBJDUMP      = $(GCC_PREFIX)objdump
endif

###############################################################################
# Variables: Compilation flags
###############################################################################

# Additional include directories
INCLUDE_PATH += $(SRC_DIR)

# Flags
CFLAGS       = $(ARCH_CFLAGS) -O$(OPT)
CFLAGS       += $(patsubst %,-I%,$(INCLUDE_PATH))
CFLAGS       += $(EXTRA_CFLAGS)

LFLAGS        = $(ARCH_LFLAGS)
LFLAGS       += $(patsubst %,-L%,$(EXTRA_LIBDIRS))
LFLAGS       += $(EXTRA_LIBS)

###############################################################################
# Variables: Lists of objects, source and deps
###############################################################################
# SRC / Object list
src2obj       = $(OBJ_DIR)$(patsubst %$(suffix $(1)),%.o,$(notdir $(1)))
src2dep       = $(DEP_DIR)$(patsubst %,%.d,$(notdir $(1)))

SRC          := $(EXTRA_SRC) $(foreach src,$(SRC_DIR),$(wildcard $(src)/*.cpp)) $(foreach src,$(SRC_DIR),$(wildcard $(src)/*.c))
OBJ          ?= $(foreach src,$(SRC),$(call src2obj,$(src)))
DEPS         ?= $(foreach src,$(SRC),$(call src2dep,$(src)))

###############################################################################
# Rules: Compilation macro
###############################################################################
# Dependancy generation
DEPFLAGS      = -MT $$@ -MMD -MP -MF $(call src2dep,$(1))

define template_c
$(call src2obj,$(1)): $(1) | $(OBJ_DIR) $(DEP_DIR)
	@echo "# Compiling $(notdir $(1))"
	$(GCC) $(CFLAGS) $(DEPFLAGS) -c $$< -o $$@
endef

###############################################################################
# Rules
###############################################################################
BUILD_TARGETS = $(ADDITIONAL_TARGETS) $(EXE_DIR)$(TARGET)

ifeq ($(ENABLE_BIN),yes)
  BUILD_TARGETS += $(EXE_DIR)$(TARGET).bin
endif
ifeq ($(ENABLE_LST),yes)
  BUILD_TARGETS += $(EXE_DIR)$(TARGET).lst
endif

all: $(BUILD_TARGETS)

$(OBJ_DIR) $(DEP_DIR) $(EXE_DIR):
	@mkdir -p $@

$(foreach src,$(SRC),$(eval $(call template_c,$(src))))

$(EXE_DIR)$(TARGET): $(OBJ) | $(EXE_DIR) 
	@echo "# Building $(notdir $@)"
	$(GCC) -o $(EXE_DIR)$(TARGET) $(OBJ) $(LFLAGS)

$(EXE_DIR)$(TARGET).bin: $(EXE_DIR)$(TARGET)
	@echo "# Building $(notdir $@)"
	$(OBJCOPY) -O binary $< $@

$(EXE_DIR)$(TARGET).lst: $(EXE_DIR)$(TARGET)
	@echo "# Building $(notdir $@)"
	$(OBJDUMP) -d $< > $@

clean:
	rm -rf $(EXE_DIR) $(OBJ_DIR) $(DEP_DIR) $(ADDITIONAL_CLEAN)

###############################################################################
# Rules: Dependancies
###############################################################################
EXCLUDE_DEPS := clean
ifeq (0, $(words $(findstring $(MAKECMDGOALS), $(EXCLUDE_DEPS))))
-include $(DEPS)
endif
