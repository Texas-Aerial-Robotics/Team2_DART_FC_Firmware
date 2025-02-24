PROJECT    := Team2_DART_Firmware
BUILD_DIR  := build
SRC_DIR    := Core/Src
INC_DIR    := Core/Inc

HAL_INC_DIR          := Drivers/STM32H7xx_HAL_Driver/Inc
CMSIS_DEVICE_INC_DIR := Drivers/CMSIS/Device/ST/STM32H7xx/Include
CMSIS_CORE_INC_DIR   := Drivers/CMSIS/Include

CC         := arm-none-eabi-gcc
CP         := arm-none-eabi-objcopy
SZ         := arm-none-eabi-size

CPU        := -mcpu=cortex-m7
FPU        := -mfpu=fpv5-d16
FLOAT_ABI  := -mfloat-abi=hard
MCU_FLAGS  := $(CPU) -mthumb $(FPU) $(FLOAT_ABI)

CFLAGS     := $(MCU_FLAGS) -g -O2 -Wall -ffunction-sections -fdata-sections --specs=nano.specs -static -fno-common
CDEFS      := -DSTM32H743xx -DUSE_HAL_DRIVER
CINCLUDES  := -I$(INC_DIR) -I$(HAL_INC_DIR) -I$(CMSIS_DEVICE_INC_DIR) -I$(CMSIS_CORE_INC_DIR)

LDSCRIPT   := STM32H743VITX_FLASH.ld
LDFLAGS    := $(MCU_FLAGS) -T$(LDSCRIPT) -Wl,-Map=$(BUILD_DIR)/$(PROJECT).map,--gc-sections

C_SOURCES  := $(wildcard $(SRC_DIR)/*.c)
OBJ_FILES  := $(patsubst $(SRC_DIR)/%.c, $(BUILD_DIR)/%.o, $(C_SOURCES))

all: $(BUILD_DIR) $(BUILD_DIR)/$(PROJECT).elf

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c | $(BUILD_DIR)
	@echo "[CC]  $<"
	mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(CDEFS) $(CINCLUDES) -c $< -o $@

$(BUILD_DIR)/$(PROJECT).elf: $(OBJ_FILES) | $(BUILD_DIR)
	@echo "[LD]  $@"
	$(CC) $^ $(LDFLAGS) -o $@
	$(SZ) $@

clean:
	rm -rf $(BUILD_DIR)

.PHONY: all clean
