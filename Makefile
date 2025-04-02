######################################
# target
######################################
TARGET = f407_discovery_project_example


#######################################
# paths
#######################################
# Build path
BUILD_DIR = build


######################################
# source
######################################
# C sources
C_SOURCES =  \
main.c \
rw.c \
system/system_stm32f4xx.c \
system/clock.c \
audio/audio.c \
networking/eth_gpio.c \
networking/udp_echoserver.c \
networking/stm_hal/hal_supplementary_fcns.c \
networking/stm_hal/stm32f4xx_hal_eth.c \
networking/LWIP/Target/ethernetif.c \
networking/LWIP/App/lwip.c

# ASM sources
ASM_SOURCES =  \
startup/startup_stm32f407xx.s


#######################################
# binaries
#######################################
BINPATH =
PREFIX = arm-none-eabi-
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

# macros for gcc
# AS defines
AS_DEFS =

# AS includes
AS_INCLUDES =

# C defines
C_DEFS =  \
-DUSE_HAL_DRIVER \
-DHSE_VALUE=8000000 \
-DSTM32F407xx

# C includes
C_INCLUDES =  \
-I. \
-Isystem \
-Idrivers/cmsis/ST/Include \
-Idrivers/cmsis/Include \
-Iopus/include \
-Ilibspeex \
-Iaudio \
-Inetworking \
-Inetworking/LWIP \
-Inetworking/stm_hal \
-Inetworking/LWIP/App \
-Inetworking/LWIP/Target \
-Inetworking/Middlewares/Third_Party/LwIP/src/include \
-Inetworking/Middlewares/Third_Party/LwIP/system \
-Inetworking/Middlewares/Third_Party/LwIP/src/include/netif/ppp \
-Inetworking/Middlewares/Third_Party/LwIP/src/include/lwip \
-Inetworking/Middlewares/Third_Party/LwIP/src/include/lwip/apps \
-Inetworking/Middlewares/Third_Party/LwIP/src/include/lwip/priv \
-Inetworking/Middlewares/Third_Party/LwIP/src/include/lwip/prot \
-Inetworking/Middlewares/Third_Party/LwIP/src/include/netif \
-Inetworking/Middlewares/Third_Party/LwIP/src/include/compat/posix \
-Inetworking/Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa \
-Inetworking/Middlewares/Third_Party/LwIP/src/include/compat/posix/net \
-Inetworking/Middlewares/Third_Party/LwIP/src/include/compat/posix/sys \
-Inetworking/Middlewares/Third_Party/LwIP/src/include/compat/stdc \
-Inetworking/Middlewares/Third_Party/LwIP/system/arch

# compile gcc flags
MCU  = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
OPTS = -O2 -std=gnu99 -Wall -fdata-sections -ffunction-sections

CFLAGS  = $(MCU) $(OPTS) $(C_DEFS)  $(C_INCLUDES)
ASFLAGS = $(MCU) $(OPTS) $(AS_DEFS) $(AS_INCLUDES)

#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32F407VGTx_FLASH.ld

# libraries
LIBS = -Lnetworking -llwip -Lopus -lopus -Llibspeex -lspeexdsp libarm_cortexM4lf_math.a -lc -lm -lnosys
LIBDIR =
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
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

#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)

#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***
