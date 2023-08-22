PROCESSOR ?= RP2040

BOARD ?= FLY36RRF
#BOARD ?= RPI_PICO
BOARD_REV ?= 1
CANFLAGS ?= -DUSE_SPICAN
BUILD_DIR ?= ./expbuild
EXP_DIR ?= ./Duet3Expansion
REPRAPFIRMWARE_DIR ?= ./RepRapFirmware
RRFLIBRARIES_DIR ?= ./RRFLibraries
COREN2G_DIR ?= ./CoreN2G
FREERTOS_DIR ?= ./FreeRTOS
CANLIB_DIR ?= ./CANlib
SDK_DIR ?= $(COREN2G_DIR)/src/RP2040/pico-sdk
LDNAME ?= rp2040_flash
OUTPUT_NAME ?= Duet3Firmware_$(BOARD)

BUILD ?= Debug

#Comment out to show compilation commands (verbose)
V=@

$(info Building RepRapFirmware Duet3Expansion:)

## Cross-compilation commands 
CC      = $(GCC_PATH)arm-none-eabi-gcc
CXX     = $(GCC_PATH)arm-none-eabi-g++
LD      = $(GCC_PATH)arm-none-eabi-gcc
AR      = $(GCC_PATH)arm-none-eabi-ar
AS      = $(GCC_PATH)arm-none-eabi-as
OBJCOPY = $(GCC_PATH)arm-none-eabi-objcopy
OBJDUMP = $(GCC_PATH)arm-none-eabi-objdump
SIZE    = $(GCC_PATH)arm-none-eabi-size

MKDIR = mkdir -p

$(info  - MAKE_DIR: $(MAKE_DIR))
include $(MAKE_DIR)/Core.mk
include $(MAKE_DIR)/RRFLibraries.mk
include $(MAKE_DIR)/FreeRTOS.mk
#include $(MAKE_DIR)/RepRapLibc.mk
include $(MAKE_DIR)/CANLib.mk
include $(MAKE_DIR)/EXP.mk

FLAGS=
ifeq ($(PROCESSOR),RP2040)
	FLAGS  += -D__RP2040__ -D$(BOARD) $(CANFLAGS) -DBOARD_REV=$(BOARD_REV)
	PROCFLAGS = -mfp16-format=ieee -mcpu=cortex-m0plus -mthumb
    $(info - Processor: $(PROCESSOR))
    $(info - Board: $(BOARD)  Rev: $(BOARD_REV))
    $(info - Can options: $(CANFLAGS))
endif

ifeq ($(BUILD),Debug)
#	DEBUG_FLAGS = -O3 -g -DDEBUG=1
	DEBUG_FLAGS =
        $(info - Build: Debug) 
else
	DEBUG_FLAGS =
        $(info - Build: Release)
endif
	

#Linker script
LINKER_SCRIPT_BASE = $(EXP_DIR)/src/Hardware/RP2040/



#Path to the linker Script
LINKER_SCRIPT  = $(LINKER_SCRIPT_BASE)$(LDNAME).ld
$(info  - Linker Script used: $(LINKER_SCRIPT))


#Flags common for Core in c and c++
FLAGS  += -DRTOS $(PROCFLAGS) -DSUPPORT_CAN=1 -DSUPPORT_USB=1
#Build config flags
FLAGS +=  -Wall -c -ffunction-sections -fdata-sections -Werror=return-type
FLAGS += -nostdlib -Wdouble-promotion -fsingle-precision-constant
#FLAGS += -Wfloat-equal
#FLAGS += -Wundef
FLAGS += $(DEBUG_FLAGS)
#FLAGS += -MMD -MP 

CFLAGS   = $(FLAGS) -std=gnu99 -O2 -Dnoexcept= 
CXXFLAGS = $(FLAGS) -std=gnu++17 -O3 -fno-threadsafe-statics -fno-exceptions -fno-rtti -Wno-register -Wsuggest-override
CXXFLAGS_RRFL = $(FLAGS) -std=gnu++17 -O2 -fno-threadsafe-statics -fno-exceptions -fno-rtti -Wno-register -Wsuggest-override
CXXFLAGS_CORE = $(FLAGS) -std=gnu++17 -O3 -fno-threadsafe-statics -fno-exceptions -fno-rtti -Wno-register -Wsuggest-override
CXXFLAGS_RTOS = $(FLAGS) -std=gnu++17 -O3 -fno-threadsafe-statics -fno-exceptions -fno-rtti -Wno-register -Wsuggest-override
CXXFLAGS_CANLIB = $(FLAGS) -std=gnu++17 -O2 -fno-threadsafe-statics -fno-exceptions -fno-rtti -Wno-register -Wsuggest-override

#$(info  - SDK Include: $(SDK_INCLUDES))

#all Includes (RRF + Core)
INCLUDES = $(EXP_INCLUDES) $(CORE_INCLUDES) $(RTOS_INCLUDES) $(RRFLIBRARIES_INCLUDES) $(RRFLIBC_INCLUDES) $(SDK_INCLUDES) $(CANLIB_INCLUDES)


DEPS = $(CORE_OBJS:.o=.d)
DEPS = $(RTOS_OBJS:.o=.d)
DEPS += $(IAP_OBJS:.o=.d)
DEPS += $(RRFLIBC_OBJS:.o=.d)
DEPS += $(RRFLIBRARIES_OBJS:.o=.d)
DEPS += $(EXP_OBJS:.o=.d)

default: all

all: firmware

-include $(DEPS)

firmware:  $(BUILD_DIR)/$(OUTPUT_NAME).elf

$(BUILD_DIR)/libCoreN2G.a: $(CORE_OBJS)
	$(V)$(AR) rcs $@ $(CORE_OBJS)
	@echo "\nBuilt STMCore\n"

$(BUILD_DIR)/libRRFLibraries.a: $(RRFLIBRARIES_OBJS)
	$(V)$(AR) rcs $@ $(RRFLIBRARIES_OBJS)
	@echo "\nBuilt RRF Libraries\n"

$(BUILD_DIR)/libCANlib.a: $(CANLIB_OBJS)
	$(V)$(AR) rcs $@ $(CANLIB_OBJS)
	@echo "\nBuilt CANlib\n"

$(BUILD_DIR)/libFreeRTOS.a: $(RTOS_OBJS)
	$(V)$(AR) rcs $@ $(RTOS_OBJS)
#	$(V)$(AR) rcs $@ ./expbuild/./FreeRTOS/src/event_groups.o ./expbuild/./FreeRTOS/src/list.o ./expbuild/./FreeRTOS/src/portable/GCC/ARM_CM0/port.o ./expbuild/./FreeRTOS/src/queue.o ./expbuild/./FreeRTOS/src/stream_buffer.o ./expbuild/./FreeRTOS/src/tasks.o ./expbuild/./FreeRTOS/src/timers.o
	@echo "\nBuilt FreeRTOS lib\n"

$(BUILD_DIR)/$(OUTPUT_NAME).elf: $(BUILD_DIR)/libCoreN2G.a $(BUILD_DIR)/libFreeRTOS.a $(BUILD_DIR)/libRRFLibraries.a $(BUILD_DIR)/libCANlib.a $(EXP_OBJS)
	@echo "\nCreating $(OUTPUT_NAME).bin"
	$(V)$(MKDIR) $(dir $@)
	$(V)$(CXX) -L$(BUILD_DIR)/ -L$(COREN2G_DIR)/src/RP2040/Tools/Build-pico-sdk/build/ -Os --specs=nano.specs -Wl,--gc-sections -Wl,--fatal-warnings -Wl,--no-warn-rwx-segment -fno-exceptions -mcpu=cortex-m0plus -T$(LINKER_SCRIPT) -Wl,-Map,$(BUILD_DIR)/$(OUTPUT_NAME).map -o $(BUILD_DIR)/$(OUTPUT_NAME).elf $(EXP_OBJS) -lRRFLibraries -lCoreN2G -lCANlib -lFreeRTOS -lpico -lRRFLibraries -lCoreN2G -lCANlib -lFreeRTOS -lpico
	$(V)$(OBJCOPY) --strip-unneeded -O binary $(BUILD_DIR)/$(OUTPUT_NAME).elf $(BUILD_DIR)/$(OUTPUT_NAME).bin
	$(V)$(SIZE) $(BUILD_DIR)/$(OUTPUT_NAME).elf
	$(V)python $(REPRAPFIRMWARE_DIR)/Tools/uf2conv/uf2conv.py  -b 0x10000000 -f 0xe48bff56 -c -o $(BUILD_DIR)/$(OUTPUT_NAME).uf2 $(BUILD_DIR)/$(OUTPUT_NAME).bin
	
$(BUILD_DIR)/$(FREERTOS_DIR)/%.o : $(FREERTOS_DIR)/%.c
	@echo "[$<]"
	$(V)$(MKDIR) $(dir $@)
	$(V)$(CC)  $(CFLAGS) $(DEFINES) $(INCLUDES) -MMD -MP -MM -MF $(patsubst %.o,%.d,$@) $<
	$(V)$(CC)  $(CFLAGS) $(DEFINES) $(INCLUDES) -MMD -MP -o $@ $<

$(BUILD_DIR)/%.o: %.c
	@echo "[$<]"
	$(V)$(MKDIR) $(dir $@)
	$(V)$(CC)  $(CFLAGS) $(DEFINES) $(INCLUDES) -MMD -MP -MM -MF $(patsubst %.o,%.d,$@) $<
	$(V)$(CC)  $(CFLAGS) $(DEFINES) $(INCLUDES) -MMD -MP -o $@ $<

$(BUILD_DIR)/$(RRFLIBRARIES_DIR)/%.o : $(RRFLIBRARIES_DIR)/%.cpp
	@echo "RRFL [$<]"
	$(V)$(MKDIR) $(dir $@)
	$(V)$(CXX) $(CXXFLAGS_RRFL) $(DEFINES) $(INCLUDES) -MMD -MP -MM -MF $(patsubst %.o,%.d,$@) $<
	$(V)$(CXX) $(CXXFLAGS_RRFL) $(DEFINES) $(INCLUDES) -MMD -MP -o $@ $<

$(BUILD_DIR)/$(COREN2G_DIR)/%.o : $(COREN2G_DIR)/%.cpp
	@echo "CORE [$<]"
	$(V)$(MKDIR) $(dir $@)
	$(V)$(CXX) $(CXXFLAGS_CORE) $(DEFINES) $(INCLUDES) -MMD -MP -MM -MF $(patsubst %.o,%.d,$@) $<
#	$(V)$(CXX) -Wa,-ahl=$@.s $(CXXFLAGS_CORE) $(DEFINES) $(INCLUDES) -MMD -MP -o $@ $<
	$(V)$(CXX) $(CXXFLAGS_CORE) $(DEFINES) $(INCLUDES) -MMD -MP -o $@ $<

$(BUILD_DIR)/$(CANLIB_DIR)/%.o : $(CANLIB_DIR)/%.cpp
	@echo "CANLIB [$<]"
	$(V)$(MKDIR) $(dir $@)
	$(V)$(CXX) $(CXXFLAGS_CANLIB) $(DEFINES) $(INCLUDES) -MMD -MP -MM -MF $(patsubst %.o,%.d,$@) $<
	$(V)$(CXX) $(CXXFLAGS_CANLIB) $(DEFINES) $(INCLUDES) -MMD -MP -o $@ $<

$(BUILD_DIR)/%.o: %.cpp
	@echo "[$<]"
	$(V)$(MKDIR) $(dir $@)
	$(V)$(CXX) $(CXXFLAGS) $(DEFINES) $(INCLUDES) -MMD -MP -MM -MF $(patsubst %.o,%.d,$@) $<
	$(V)$(CXX) $(CXXFLAGS) $(DEFINES) $(INCLUDES) -MMD -MP -o $@ $<

$(BUILD_DIR)/%.o: %.cc
	@echo "[$<]"
	$(V)$(MKDIR) $(dir $@)
	$(V)$(CXX) $(CXXFLAGS) $(DEFINES) $(INCLUDES) -MMD -MP -MM -MF $(patsubst %.o,%.d,$@) $<
	$(V)$(CXX) $(CXXFLAGS) $(DEFINES) $(INCLUDES) -MMD -MP -o $@ $<

$(BUILD_DIR)/$(COREN2G_DIR)/%.o: $(COREN2G_DIR)/%.S
	@echo "[$<]"
	$(V)$(MKDIR) $(dir $@)
	$(V)$(CXX) -x assembler-with-cpp $(CXXFLAGS) $(DEFINES) $(INCLUDES) -MMD -MP -MM -MF $(patsubst %.o,%.d,$@) $<
	$(V)$(CXX) -x assembler-with-cpp $(CXXFLAGS) $(DEFINES) $(INCLUDES) -MMD -MP -o $@ $<

$(BUILD_DIR)/%.o: %.S
	@echo "[$<]"
	$(V)$(MKDIR) $(dir $@)
	$(V)$(CXX) -x assembler-with-cpp $(CXXFLAGS) $(DEFINES) $(INCLUDES) -MMD -MP -MM -MF $(patsubst %.o,%.d,$@) $<
	$(V)$(CXX) -x assembler-with-cpp $(CXXFLAGS) $(DEFINES) $(INCLUDES) -MMD -MP -o $@ $<

clean: distclean

distclean:
	-rm -rf $(BUILD_DIR)/ 

.PHONY: all firmware clean distclean $(BUILD_DIR)/$(OUTPUT_NAME).elf
