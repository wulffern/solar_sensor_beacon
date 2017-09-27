# Copyright (c) 2016 Nordic Semiconductor. All Rights Reserved.
#
# The information contained herein is property of Nordic Semiconductor ASA.
# Terms and conditions of usage are described in detail in NORDIC
# SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
#
# Licensees are granted free, non-transferable use of the information. NO
# WARRANTY of ANY KIND is provided. This heading must NOT be removed from
# the file.

PROJECT_NAME     := sensor_beacon
OUTPUT_DIRECTORY := _build
TARGETS          := ${PROJECT_NAME}

PROJ_DIR      := ../../..
TEMPLATE_PATH := ${PROJ_DIR}/toolchain/gcc

SDKPATH := ${PROJ_DIR}/../nRF5_SDK_11

ifeq ($(OS),Windows_NT)
 NRF_CMSIS_ROOT              := C:/Keil_v5/ARM/PACK/ARM/CMSIS/4.5.0
 NRF_DEVICE_FAMILY_PACK_ROOT := C:/Keil_v5/ARM/PACK/NordicSemiconductor/nRF_DeviceFamilyPack/8.11.1/Device/Source
 NRF_DEVICE_FAMILY_PACK_ROOT_SRC := ${NRF_DEVICE_FAMILY_PACK_ROOT}/Source
 NRF_DEVICE_FAMILY_PACK_ROOT_INC := ${NRF_DEVICE_FAMILY_PACK_ROOT}/Include

else
 NRF_CMSIS_ROOT              := ${SDKPATH}/components/toolchain
 NRF_DEVICE_FAMILY_PACK_ROOT := ${SDKPATH}/components/toolchain
 NRF_DEVICE_FAMILY_PACK_ROOT_SRC := ${NRF_DEVICE_FAMILY_PACK_ROOT}
 NRF_DEVICE_FAMILY_PACK_ROOT_INC := ${SDKPATH}/components/device  ${SDKPATH}/components/toolchain
endif

include ${PROJ_DIR}/toolchain/gcc/Makefile.common

LINKER_SCRIPT := $(PROJECT_NAME).ld

# Source files common to all targets
SRC_FILES += \
  $(PROJ_DIR)/src/sensor_beacon.c \
  $(NRF_DEVICE_FAMILY_PACK_ROOT_SRC)/gcc/gcc_startup_nrf52.s \
  $(NRF_DEVICE_FAMILY_PACK_ROOT_SRC)/system_nrf52.c \
  $(PROJ_DIR)/src/hal_timer.c \
  $(PROJ_DIR)/src/hal_radio.c \
  $(PROJ_DIR)/src/hal_clock.c \

INC_FOLDERS += \
  $(PROJ_DIR)/inc \
  $(PROJ_DIR)/external/comp_generic/hal/inc \
  $(SDKPATH)/components/drivers_nrf/hal \
  $(NRF_CMSIS_ROOT)/CMSIS/Include \
  $(NRF_DEVICE_FAMILY_PACK_ROOT_INC)
# C flags common to all targets
CFLAGS += -DNRF52
CFLAGS += -mcpu=cortex-m4
CFLAGS += -mthumb -mabi=aapcs
CFLAGS +=  -Wall -Werror -O3 -g2
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# keep every function in separate section, this allows linker to discard unused ones
CFLAGS += -DTEMPERATURE_ONLY_BEACON
CFLAGS += -DSYS_CFG_USE_TWI0
CFLAGS += -DSYS_CFG_SERIAL_0_IRQ_PRIORITY=3
CFLAGS += -DPCA20014

# C++ flags common to all targets
CXXFLAGS += \

# Assembler flags common to all targets
ASMFLAGS += -x assembler-with-cpp
ASMFLAGS += -DNRF52

# Linker flags
LDFLAGS += -mthumb -mabi=aapcs -L $(TEMPLATE_PATH) -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m4
LDFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# let linker to dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs -lc -lnosys

$(foreach target, $(TARGETS), $(call define_target, $(target)))

flash: 
	@echo Flashing: $(OUTPUT_DIRECTORY)/${PROJECT_NAME}.hex
	nrfjprog -s ${SERIAL} --program $(OUTPUT_DIRECTORY)/${PROJECT_NAME}.hex  -f nrf52  --sectorerase
	nrfjprog --reset -f nrf52	-s ${SERIAL}

