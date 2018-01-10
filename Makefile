PROJECT_NAME := nrf_beacon_itcpolyuhk

export OUTPUT_FILENAME
#MAKEFILE_NAME := $(CURDIR)/$(word $(words $(MAKEFILE_LIST)),$(MAKEFILE_LIST))
MAKEFILE_NAME := $(MAKEFILE_LIST)
MAKEFILE_DIR := $(dir $(MAKEFILE_NAME) )

SDKROOT=../sdk11
PROJROOT=.

TEMPLATE_PATH = $(SDKROOT)/components/toolchain/gcc
ifeq ($(OS),Windows_NT)
include $(TEMPLATE_PATH)/Makefile.windows
else
include $(TEMPLATE_PATH)/Makefile.posix
endif

MK := mkdir
RM := rm -rf

#echo suspend
ifeq ("$(VERBOSE)","1")
NO_ECHO := 
else
NO_ECHO := @
endif

# Toolchain commands
CC              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-gcc'
AS              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-as'
AR              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ar' -r
LD              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ld'
NM              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-nm'
OBJDUMP         := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objdump'
OBJCOPY         := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objcopy'
SIZE            := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-size'

#function for removing duplicates in a list
remduplicates = $(strip $(if $1,$(firstword $1) $(call remduplicates,$(filter-out $(firstword $1),$1))))

#source common to all targets
C_SOURCE_FILES += \
$(abspath $(SDKROOT)/components/libraries/button/app_button.c) \
$(abspath $(SDKROOT)/components/libraries/util/app_error.c) \
$(abspath $(SDKROOT)/components/libraries/util/app_error_weak.c) \
$(abspath $(SDKROOT)/components/libraries/fifo/app_fifo.c) \
$(abspath $(SDKROOT)/components/libraries/timer/app_timer.c) \
$(abspath $(SDKROOT)/components/libraries/util/app_util_platform.c) \
$(abspath $(SDKROOT)/components/libraries/util/nrf_assert.c) \
$(abspath $(SDKROOT)/components/libraries/util/nrf_log.c) \
$(abspath $(SDKROOT)/components/libraries/uart/retarget.c) \
$(abspath $(SDKROOT)/external/segger_rtt/RTT_Syscalls_GCC.c) \
$(abspath $(SDKROOT)/external/segger_rtt/SEGGER_RTT.c) \
$(abspath $(SDKROOT)/external/segger_rtt/SEGGER_RTT_printf.c) \
$(abspath $(SDKROOT)/components/libraries/uart/app_uart_fifo.c) \
$(abspath $(SDKROOT)/components/drivers_nrf/delay/nrf_delay.c) \
$(abspath $(SDKROOT)/components/drivers_nrf/common/nrf_drv_common.c) \
$(abspath $(SDKROOT)/components/drivers_nrf/gpiote/nrf_drv_gpiote.c) \
$(abspath $(SDKROOT)/components/drivers_nrf/uart/nrf_drv_uart.c) \
$(abspath $(SDKROOT)/components/drivers_nrf/timer/nrf_drv_timer.c) \
$(abspath $(SDKROOT)/components/drivers_nrf/twi_master/nrf_drv_twi.c) \
$(abspath $(PROJROOT)/bsp/bsp.c) \
$(abspath $(PROJROOT)/sensors.c) \
$(abspath $(PROJROOT)/ruuvi.c) \
$(abspath $(PROJROOT)/main.c) \
$(abspath $(SDKROOT)/components/ble/common/ble_advdata.c) \
$(abspath $(SDKROOT)/components/ble/common/ble_conn_params.c) \
$(abspath $(SDKROOT)/components/ble/common/ble_srv_common.c) \
$(abspath $(SDKROOT)/components/toolchain/system_nrf51.c) \
$(abspath $(SDKROOT)/components/softdevice/common/softdevice_handler/softdevice_handler.c) \

#assembly files common to all targets
ASM_SOURCE_FILES  = $(abspath $(SDKROOT)/components/toolchain/gcc/gcc_startup_nrf51.s)

#includes common to all targets
INC_PATHS += -I$(abspath $(PROJROOT))
INC_PATHS += -I$(abspath $(SDKROOT)/components/libraries/util)
INC_PATHS += -I$(abspath $(SDKROOT)/components/libraries/timer)
INC_PATHS += -I$(abspath $(SDKROOT)/components/drivers_nrf/uart)
INC_PATHS += -I$(abspath $(SDKROOT)/components/ble/common)
INC_PATHS += -I$(abspath $(SDKROOT)/components/drivers_nrf/common)
INC_PATHS += -I$(abspath $(SDKROOT)/components/drivers_nrf/config)
INC_PATHS += -I$(abspath $(SDKROOT)/components/drivers_nrf/gpiote)
INC_PATHS += -I$(abspath $(SDKROOT)/components/drivers_nrf/timer)
INC_PATHS += -I$(abspath $(SDKROOT)/components/drivers_nrf/twi_master)
INC_PATHS += -I$(abspath $(SDKROOT)/components/libraries/fifo)
INC_PATHS += -I$(abspath $(PROJROOT)/bsp)
INC_PATHS += -I$(abspath $(SDKROOT)/components/toolchain/gcc)
INC_PATHS += -I$(abspath $(SDKROOT)/components/softdevice/s130/headers/nrf51)
INC_PATHS += -I$(abspath $(SDKROOT)/components/libraries/uart)
INC_PATHS += -I$(abspath $(SDKROOT)/components/device)
INC_PATHS += -I$(abspath $(SDKROOT)/components/softdevice/common/softdevice_handler)
INC_PATHS += -I$(abspath $(SDKROOT)/external/segger_rtt)
INC_PATHS += -I$(abspath $(SDKROOT)/components/drivers_nrf/delay)
INC_PATHS += -I$(abspath $(SDKROOT)/components/toolchain/CMSIS/Include)
INC_PATHS += -I$(abspath $(SDKROOT)/components/toolchain)
INC_PATHS += -I$(abspath $(SDKROOT)/components/drivers_nrf/hal)
INC_PATHS += -I$(abspath $(SDKROOT)/components/libraries/button)
INC_PATHS += -I$(abspath $(SDKROOT)/components/softdevice/s130/headers)

OBJECT_DIRECTORY = _build
LISTING_DIRECTORY = $(OBJECT_DIRECTORY)
OUTPUT_BINARY_DIRECTORY = $(OBJECT_DIRECTORY)

# Sorting removes duplicates
BUILD_DIRECTORIES := $(sort $(OBJECT_DIRECTORY) $(OUTPUT_BINARY_DIRECTORY) $(LISTING_DIRECTORY) )

#flags common to all targets
CFLAGS  = -DNRF_LOG_USES_UART=1
CFLAGS += -DSWI_DISABLE0
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -DNRF51
CFLAGS += -DS130
CFLAGS += -DBLE_STACK_SUPPORT_REQD
CFLAGS += -DBOARD_CUSTOM
CFLAGS += -mcpu=cortex-m0
CFLAGS += -mthumb -mabi=aapcs --std=gnu99
CFLAGS += -Wall -Werror -O3 -g3
CFLAGS += -mfloat-abi=soft
# keep every function in separate section. This will allow linker to dump unused functions
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin --short-enums 
# keep every function in separate section. This will allow linker to dump unused functions
LDFLAGS += -Xlinker -Map=$(LISTING_DIRECTORY)/$(OUTPUT_FILENAME).map
LDFLAGS += -mthumb -mabi=aapcs -L $(TEMPLATE_PATH) -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m0
# let linker to dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs -lc -lnosys

# Assembler flags
ASMFLAGS += -x assembler-with-cpp
ASMFLAGS += -DNRF_LOG_USES_UART=1
ASMFLAGS += -DSWI_DISABLE0
ASMFLAGS += -DSOFTDEVICE_PRESENT
ASMFLAGS += -DNRF51
ASMFLAGS += -DS130
ASMFLAGS += -DBLE_STACK_SUPPORT_REQD
ASMFLAGS += -DBOARD_CUSTOM

#default target - first one defined
default: clean nrf_beacon_itcpolyuhk

#building all targets
all: clean
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e cleanobj
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e nrf51422_xxac_s130

#target for printing all targets
help:
	@echo following targets are available:
	@echo 	nrf51422_xxac_s130
	@echo 	flash_softdevice

C_SOURCE_FILE_NAMES = $(notdir $(C_SOURCE_FILES))
C_PATHS = $(call remduplicates, $(dir $(C_SOURCE_FILES) ) )
C_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(C_SOURCE_FILE_NAMES:.c=.o) )

ASM_SOURCE_FILE_NAMES = $(notdir $(ASM_SOURCE_FILES))
ASM_PATHS = $(call remduplicates, $(dir $(ASM_SOURCE_FILES) ))
ASM_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(ASM_SOURCE_FILE_NAMES:.s=.o) )

vpath %.c $(C_PATHS)
vpath %.s $(ASM_PATHS)

OBJECTS = $(C_OBJECTS) $(ASM_OBJECTS)

nrf_beacon_itcpolyuhk: OUTPUT_FILENAME := nrf_beacon_itcpolyuhk
nrf_beacon_itcpolyuhk: LINKER_SCRIPT=nrf_beacon_itcpolyuhk.ld

nrf_beacon_itcpolyuhk: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -lm -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e finalize

## Create build directories
$(BUILD_DIRECTORIES):
	echo $(MAKEFILE_NAME)
	$(MK) $@

# Create objects from C SRC files
$(OBJECT_DIRECTORY)/%.o: %.c
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CC) $(CFLAGS) $(INC_PATHS) -c -o $@ $<

# Assemble files
$(OBJECT_DIRECTORY)/%.o: %.s
	@echo Assembly file: $(notdir $<)
	$(NO_ECHO)$(CC) $(ASMFLAGS) $(INC_PATHS) -c -o $@ $<
# Link
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -lm -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
## Create binary .bin file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

finalize: genbin genhex echosize

genbin:
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
genhex: 
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex
echosize:
	-@echo ''
	$(NO_ECHO)$(SIZE) $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	-@echo ''

clean:
	$(RM) $(BUILD_DIRECTORIES)

cleanobj:
	$(RM) $(BUILD_DIRECTORIES)/*.o
flashold: nrf51422_xxac_s130
	@echo Flashing: $(OUTPUT_BINARY_DIRECTORY)/$<.hex
	nrfjprog --program $(OUTPUT_BINARY_DIRECTORY)/$<.hex -f nrf51  --sectorerase
	nrfjprog --reset -f nrf51

flash:
	(echo loadfile _build/nrf_beacon_itcpolyuhk.hex ; echo r 1000; echo g; echo exit) | JLinkExe -device NRF51822_XXAA -if SWD -speed auto 

## Flash softdevice
flash_softdevice:
	@echo Flashing: s130_nrf51_2.0.0_softdevice.hex
	(echo loadfile $(SDKROOT)/components/softdevice/s130/hex/s130_nrf51_2.0.0_softdevice.hex ; echo r 1000; echo g; echo exit) | JLinkExe -device NRF51822_XXAA -if SWD -speed auto
	nrfjprog --program $(SDKROOT)/components/softdevice/s130/hex/s130_nrf51_2.0.0_softdevice.hex -f nrf51 --chiperase
	nrfjprog --reset -f nrf51
