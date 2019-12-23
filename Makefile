TARGET_F0 = W_M
DEBUG = 1
OPT = -Os
CPPSTD =-std=c++17
BUILD_DIR = build

######################################
# source
######################################
CPP_SOURCES_F0 = src/main.cpp
LIBRARY_PATH = ../mculib3
CMSIS_PATH = $(LIBRARY_PATH)/STM32F0_files

ASM_SOURCES_F0 = $(CMSIS_PATH)/startup_stm32f030x6.s
LDSCRIPT_F0 = $(CMSIS_PATH)/STM32F030K6Tx_FLASH.ld

# C includes
C_INCLUDES =  
C_INCLUDES += -I.
C_INCLUDES += -I$(CMSIS_PATH)
C_INCLUDES += -I$(CMSIS_PATH)/CMSIS
C_INCLUDES += -I$(LIBRARY_PATH)/src
C_INCLUDES += -I$(LIBRARY_PATH)/src/periph
C_INCLUDES += -I$(LIBRARY_PATH)/src/bits


#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-

CPP = $(PREFIX)g++
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
AR = $(PREFIX)ar
SZ = $(PREFIX)size
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
CPU_F0 = -mcpu=cortex-m0

# NONE for Cortex-M0/M0+/M3
FPU_F0 =

FLOAT-ABI_F0 =

# mcu
MCU_F0 = $(CPU_F0) -mthumb $(FPU_F0) $(FLOAT-ABI_F0)

# compile gcc flags
CFLAGS_F0  = $(MCU_F0) $(C_INCLUDES) $(OPT)
CFLAGS_F0 += -fdata-sections -ffunction-sections -fno-exceptions -fno-strict-volatile-bitfields -fno-threadsafe-statics
CFLAGS_F0 += -Wall -Wno-register  -Wno-packed-bitfield-compat -Wno-strict-aliasing
CFLAGS_F0 += -g -gdwarf-2 


# Generate dependency information
CFLAGS_F0 += -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)"

#######################################
# LDFLAGS
#######################################
# libraries
LIBS = -lc -lm -lnosys

LDFLAGS_F0  = $(MCU_F0) -specs=nano.specs -specs=nosys.specs
LDFLAGS_F0 += -T$(LDSCRIPT_F0) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET_F0).map,--cref -Wl,--gc-sections

# default action: build all
all:  clean \
$(BUILD_DIR)/$(TARGET_F0).elf $(BUILD_DIR)/$(TARGET_F0).hex $(BUILD_DIR)/$(TARGET_F0).bin
	


#######################################
# build the application
#######################################
# list of objects
OBJECTS_F0 += $(addprefix $(BUILD_DIR)/,$(notdir $(CPP_SOURCES_F0:.cpp=.o)))
vpath %.cpp $(sort $(dir $(CPP_SOURCES_F0)))
OBJECTS_F0 += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES_F0:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES_F0)))



$(BUILD_DIR)/main.o:$(CPP_SOURCES_F0) Makefile | $(BUILD_DIR) 
	$(CPP) -c $(CFLAGS_F0) $(CPPSTD) -fno-rtti -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cpp=.lst)) $< -o $@

$(BUILD_DIR)/startup_stm32f030x6.o: $(ASM_SOURCES_F0) Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS_F0) $< -o $@

$(BUILD_DIR)/$(TARGET_F0).elf: $(OBJECTS_F0) Makefile
	$(CPP) $(OBJECTS_F0) $(LDFLAGS_F0) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@

clean:
	-rm -fR .dep $(BUILD_DIR)

flash:
	st-flash write $(BUILD_DIR)/$(TARGET_F0).bin 0x8000000

util:
	st-util

submodule:
	git submodule update --init
	cd mculib3/ && git fetch
	cd mculib3/ && git checkout v1.07
  
#######################################
# dependencies
#######################################
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)

# *** EOF ***