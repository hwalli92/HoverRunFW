######################################
# target
######################################
TARGET = hover

######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Og

# Build path
BUILD_DIR = build

######################################
# source
######################################
# C sources
C_SOURCES =  \
Drivers/HAL_Driver/src/at32f4xx_flash.c \
Drivers/HAL_Driver/src/at32f4xx_rcc.c \
Drivers/HAL_Driver/src/at32f4xx_tim.c \
Drivers/HAL_Driver/src/at32f4xx_gpio.c \
Drivers/HAL_Driver/src/at32f4xx_adc.c \
Drivers/HAL_Driver/src/at32f4xx_pwr.c \
Drivers/HAL_Driver/src/at32f4xx_usart.c \
Drivers/HAL_Driver/src/at32f4xx_i2c.c \
Drivers/HAL_Driver/src/at32f4xx_dma.c \
Drivers/HAL_Driver/src/at32f4xx_exti.c \
Drivers/HAL_Driver/src/misc.c \
Src/setup.c \
Src/control.c \
Src/main.c \
Src/bldc.c \
Src/comms.c \
Src/uart.c \
Src/at32f4xx_it.c \
Src/BLDC_controller_data.c \
Src/BLDC_controller.c \
Src/system_at32f4xx.c

# ASM sources
ASM_SOURCES =  \
startup_at32f413rx_hd.s


#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
AR = $(PREFIX)ar
SZ = $(PREFIX)size
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
TOUCH = echo "" > 
RM = rm

#######################################
# CFLAGS
#######################################
# cpu
	CPU = -mcpu=cortex-m4

	# fpu
	FPU=-mfpu=fpv4-sp-d16
	# float-abi
	FLOAT-ABI=-mfloat-abi=softfp

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS =

# C defines
C_DEFS =  \
-DUSE_HAL_DRIVER \
-DAT32F413Rx_HD

ifneq ("$(wildcard ./Inc/custom_config.h)","")
	CUSTOM_CONFIG = 1
	C_DEFS := $(C_DEFS) -DCUSTOM_CONFIG 
else
	CUSTOM_CONFIG = 0
	C_DEFS := $(C_DEFS)
endif

# AS includes
AS_INCLUDES =

# C includes
C_INCLUDES =  \
-IInc \
-IDrivers/HAL_Driver/inc \
-IDrivers/CMSIS/CM4/DeviceSupport \
-IDrivers/CMSIS/CM4/CoreSupport \

#-IDrivers/AT32F4xx_StdPeriph_Driver/Inc \

# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections -std=gnu11

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = AT32F413xC_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys
LIBDIR =
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: post-build	

post-build: main-build
	@echo POST

main-build: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin
#@$(MAKE) --no-print-directory target

target: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Inc/config.h Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@ -include Inc/at32f4xx_conf.h

$(BUILD_DIR)/%.o: %.s Inc/config.h Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@

$(BUILD_DIR):
	mkdir -p $@

format:
	find Src/ Inc/ -iname '*.h' -o -iname '*.c' | xargs clang-format -i
#######################################
# clean up
#######################################
clean:
	-rm -fR .dep $(BUILD_DIR)/*

flash:
	st-flash --reset write $(BUILD_DIR)/$(TARGET).bin 0x8000000

flash-jlink:
	 JLink.exe -if swd -device Cortex-M4 -speed 4000 -SettingsFile .\JLinkSettings.ini -CommanderScript jlink-command.jlink

unlock:
	openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg -c init -c "reset halt" -c "stm32f1x unlock 0"

#######################################
# dependencies
#######################################
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)

# *** EOF ***
