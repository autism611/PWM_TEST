##########################################################################################################################
# File automatically-generated by tool: [projectgenerator] version: [4.1.0] date: [Tue Jun 18 14:59:07 CST 2024] 
##########################################################################################################################

# ------------------------------------------------
# Generic Makefile (based on gcc)
#
# ChangeLog :
#	2017-02-10 - Several enhancements + project update mode
#   2015-07-22 - first version
# ------------------------------------------------

######################################
# target
######################################
TARGET = PWM_TEST


######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Og


#######################################
# paths
#######################################
# Build path
BUILD_DIR = build
EXTENSIONS_DIR = $(shell pwd)
TOPFOLDER = $(EXTENSIONS_DIR)

#UROS_APP_FOLDER = $(shell pwd)/../apps/ping_pong

######################################
# code format
######################################
#FORMAT_RESULT:=$(shell ../../Utilities/tools/astyle-format.sh)
FORMAT_RESULT:=$(shell ./clang-format-all Core/Inc Core/Src)

######################################
# source
######################################
# C sources
C_SOURCES =  \
Core/Src/main.c \
Core/Src/stm32f1xx_it.c \
Core/Src/stm32f1xx_hal_msp.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.c \
Core/Src/system_stm32f1xx.c \
Core/Src/gpio.c \
Core/Src/adc.c \
Core/Src/tim.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc_ex.c \
Core/Src/usart.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c

# ASM sources
ASM_SOURCES =  \
startup_stm32f103xg.s


#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m3

# fpu
# NONE for Cortex-M0/M0+/M3

# float-abi


# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DUSE_HAL_DRIVER \
-DSTM32F103xG


# AS includes
AS_INCLUDES = 

# C includes
C_INCLUDES =  \
-ICore/Inc \
-IDrivers/STM32F1xx_HAL_Driver/Inc \
-IDrivers/STM32F1xx_HAL_Driver/Inc/Legacy \
-IDrivers/CMSIS/Device/ST/STM32F1xx/Include \
-IDrivers/CMSIS/Include


# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2 -Wall
endif


CI_BUILD_NUM=0
#获取commit_id的前8个字符
COMMIT_ID=$(shell git rev-parse --short=8 HEAD)
#获取编译时间
BUILD_TIME=$(shell date +%Y-%m-%d_%H:%M:%S)
#获取当前分支名
CURRENT_BRANCH=$(shell git rev-parse --abbrev-ref HEAD)
#设置软件版本号
SOFTWARE_VERSION=0.0.1
#设置型号
#MODEL_NAME=SECURITY_CHASSIS

#增加SEGGER-RTT输入输出的缓存
CFLAGS += -DBUFFER_SIZE_UP=2048 -DBUFFER_SIZE_DOWN=32

# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)" -DCOMMIT_ID=\"$(COMMIT_ID)\" -DBUILD_TIME=\"$(BUILD_TIME)\" -DCURRENT_BRANCH=\"$(CURRENT_BRANCH)\" 
CFLAGS += -DTARGET=\"$(TARGET)\" -DSOFTWARE_VERSION=\"$(SOFTWARE_VERSION).$(CI_BUILD_NUM)\" -DMODEL_NAME=\"$(MODEL_NAME)\"
CFLAGS += -Wno-unused-function -Wno-unused-but-set-variable -Wno-unused-variable -Wno-unused-result

# Microros multithread support platform
CFLAGS += -DPLATFORM_NAME_FREERTOS

#######################################
# LDFLAGS
#######################################
# link script
ifdef IAP
CFLAGS += -DIAP
LDSCRIPT = $(EXTENSIONS_DIR)/STM32F103VGTx_FLASH_IAP.ld
else
LDSCRIPT = $(EXTENSIONS_DIR)/STM32F103VGTx_FLASH.ld
endif

# libraries
LIBS = -lc -lm -lnosys 
LIBDIR = 
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# printf float
LDFLAGS += -lc -lrdimon -u _printf_float

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin
	cp $(BUILD_DIR)/$(TARGET).elf $(EXTENSIONS_DIR)/${TARGET}.elf

#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

 ifneq ($(V),1)
 Q       := @
 NULL    := 2>/dev/null
 endif

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	@printf "  CC      $(*).c\n"
	$(Q) $(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	@printf "  AS      $(*).c\n"
	$(Q) $(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	@printf "  LD      $(TARGET).elf\n"
	$(Q) $(CC) $(OBJECTS) $(LDFLAGS) -o $@
	@printf "  SZ      $(TARGET).elf\n"
	$(Q) $(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@		

#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)
  
#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

OPENOCD := openocd -f interface/jlink.cfg \
        -c 'transport select swd' \
        -f target/stm32f1x.cfg
# download your program
flash: all
	$(OPENOCD) -c init \
		-c 'reset halt' \
		-c 'flash write_image erase $(BUILD_DIR)/$(TARGET).elf' \
		-c 'reset run' \
		-c exit

images:all
	rm -rf $(BUILD_DIR)/*.o
	rm -rf $(BUILD_DIR)/*.d
	rm -rf $(BUILD_DIR)/*.lst
	rm -rf $(BUILD_DIR)/*.map       
	rm -rf $(BUILD_DIR)/${TARGET}_*
	cp $(BUILD_DIR)/$(TARGET).bin $(BUILD_DIR)/${TARGET}_${SOFTWARE_VERSION}.${CI_BUILD_NUM}_${CURRENT_BRANCH}_${COMMIT_ID}.bin
	rm -rf $(BUILD_DIR)/$(TARGET).elf
	rm -rf $(BUILD_DIR)/$(TARGET).hex
	rm -rf $(BUILD_DIR)/$(TARGET).bin

iap:
	cd c3_workstation_bootloader/hal/ && make iap -j32
	touch Makefile
	make images -j32 IAP=1
	python3 no_compress_ota_packager_python.py
	rm -rf $(BUILD_DIR)/${TARGET}_*
	cp ${TARGET}.elf $(BUILD_DIR)/${TARGET}_${SOFTWARE_VERSION}.${CI_BUILD_NUM}.${CURRENT_BRANCH}.${COMMIT_ID}.elf
	mv $(BUILD_DIR)/ota_${TARGET}_${SOFTWARE_VERSION}.${CI_BUILD_NUM}_${CURRENT_BRANCH}_${COMMIT_ID}.bin $(BUILD_DIR)/${TARGET}_${SOFTWARE_VERSION}.${CI_BUILD_NUM}.${CURRENT_BRANCH}.${COMMIT_ID}.bin
	mv $(BUILD_DIR)/ota_${TARGET}_${SOFTWARE_VERSION}.${CI_BUILD_NUM}_${CURRENT_BRANCH}_${COMMIT_ID}.rbl $(BUILD_DIR)/${TARGET}_${SOFTWARE_VERSION}.${CI_BUILD_NUM}.${CURRENT_BRANCH}.${COMMIT_ID}.rbl
	cp $(BUILD_DIR)/${TARGET}_${SOFTWARE_VERSION}.${CI_BUILD_NUM}.${CURRENT_BRANCH}.${COMMIT_ID}.bin $(BUILD_DIR)/${TARGET}_lastest.${CURRENT_BRANCH}.bin
	cp $(BUILD_DIR)/${TARGET}_${SOFTWARE_VERSION}.${CI_BUILD_NUM}.${CURRENT_BRANCH}.${COMMIT_ID}.rbl $(BUILD_DIR)/${TARGET}_lastest.${CURRENT_BRANCH}.rbl
download:
	rm -rf flash.jlink
	touch flash.jlink
	echo "r" > flash.jlink
	echo "loadfile build/${TARGET}_${SOFTWARE_VERSION}.${CI_BUILD_NUM}.${CURRENT_BRANCH}.${COMMIT_ID}.bin" >> flash.jlink
	echo "r" >> flash.jlink
	echo "exit" >> flash.jlink
	JLinkExe -device STM32F103VG  -si SWD -speed 4000 -CommanderScript ./flash.jlink

# *** EOF ***
