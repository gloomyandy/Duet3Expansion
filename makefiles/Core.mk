CORE_DIR = $(COREN2G_DIR)


#Core
CORE_SRC_DIRS  = src src/RP2040 src/RP2040/pico src/RP2040/MCP2517FD

CORE_SRC = $(CORE_DIR) $(addprefix $(CORE_DIR)/, $(CORE_SRC_DIRS))
CORE_INCLUDES = $(addprefix -I, $(CORE_SRC))
SDK_INCLUDE_DIRS = boards/include common/pico_base/include common/pico_sync/include common/pico_time/include rp2_common/cmsis/include rp2_common/hardware_base/include
SDK_INCLUDE_DIRS += rp2_common/hardware_dma/Include rp2_common/hardware_gpio/include rp2_common/hardware_irq/include rp2_common/hardware_sync/include rp2_common/hardware_timer/include
SDK_INCLUDE_DIRS += rp2_common/hardware_watchdog/include rp2_common/pico_platform/include rp2040/hardware_regs/include rp2040/hardware_structs/include
SDK_INCLUDE_DIRS += rp2_common/cmsis/stub/CMSIS/Core/include rp2_common/cmsis/stub/CMSIS/Device/RaspberryPi/RP2040/Include rp2_common/hardware_adc/include
SDK_INCLUDE_DIRS += rp2_common/pico_unique_id/include rp2_common/hardware_pwm/include rp2_common/pico_multicore/include rp2_common/hardware_pio/include
SDK_INCLUDE_DIRS += rp2_common/hardware_spi/include rp2_common/pico_bootrom/include rp2_common/hardware_flash/include
SDK_INCLUDE_PATHS = $(addprefix $(SDK_DIR)/src/, $(SDK_INCLUDE_DIRS))
SDK_INCLUDE_PATHS += $(addprefix $(SDK_DIR)/, lib/tinyusb/src)
SDK_INCLUDES = $(addprefix -I, $(SDK_INCLUDE_PATHS))
#Find all c and c++ files for Core
CORE_OBJ_SRC_C    += $(foreach src, $(CORE_SRC), $(wildcard $(src)/*.c))
CORE_OBJ_SRC_CXX   += $(foreach src, $(CORE_SRC), $(wildcard $(src)/*.cpp))
CORE_OBJS = $(patsubst %.c,$(BUILD_DIR)/%.o,$(CORE_OBJ_SRC_C)) $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(CORE_OBJ_SRC_CXX))
