#
# Copyright (C) 2019 Freek van Tienen <freek.v.tienen@gmail.com>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
#

# Compiler options here.
ifeq ($(USE_OPT),)
  USE_OPT = -Os -fomit-frame-pointer -falign-functions=16
endif

# C specific options here (added to USE_OPT).
ifeq ($(USE_COPT),)
  USE_COPT = 
endif

# C++ specific options here (added to USE_OPT).
ifeq ($(USE_CPPOPT),)
  USE_CPPOPT = -fno-rtti
endif

# Enable this if you want the linker to remove unused code and data.
ifeq ($(USE_LINK_GC),)
  USE_LINK_GC = yes
endif

# Linker extra options here.
ifeq ($(USE_LDOPT),)
  USE_LDOPT = 
endif

# Enable this if you want link time optimizations (LTO).
ifeq ($(USE_LTO),)
  USE_LTO = yes
endif

# If enabled, this option allows to compile the application in THUMB mode.
ifeq ($(USE_THUMB),)
  USE_THUMB = yes
endif

# Enable this if you want to see the full log while compiling.
ifeq ($(USE_VERBOSE_COMPILE),)
  USE_VERBOSE_COMPILE = no
endif

# If enabled, this option makes the build process faster by not compiling
# modules not used in the current configuration.
ifeq ($(USE_SMART_BUILD),)
  USE_SMART_BUILD = yes
endif

#
# Build global options
##############################################################################

##############################################################################
# Architecture or project specific options
#

# Stack size to be allocated to the Cortex-M process stack. This stack is
# the stack used by the main() thread.
ifeq ($(USE_PROCESS_STACKSIZE),)
  USE_PROCESS_STACKSIZE = 0x400
endif

# Stack size to the allocated to the Cortex-M main/exceptions stack. This
# stack is used for processing interrupts and exceptions.
ifeq ($(USE_EXCEPTIONS_STACKSIZE),)
  USE_EXCEPTIONS_STACKSIZE = 0x400
endif

# Enables the use of FPU (no, softfp, hard).
ifeq ($(USE_FPU),)
  USE_FPU = no
endif

# FPU-related options.
ifeq ($(USE_FPU_OPT),)
  USE_FPU_OPT = -mfloat-abi=$(USE_FPU) -mfpu=fpv4-sp-d16
endif

#
# Architecture or project specific options
##############################################################################

##############################################################################
# Project, target, sources and paths
#

# Define project name here
PROJECT = tuc_bl

# Target settings.
MCU  ?= cortex-m3

# Version information
GIT_DESC := $(shell git describe --dirty --always --tags)
SOFT_HASH := $(shell git log -1 --pretty=format:%h 2> /dev/null)
SOFT_VER := $(shell echo $(GIT_DESC) | sed 's/[^0-9.]*\([0-9.]*\).*/\1/')
SOFT_VER_MAJOR := $(shell echo $(GIT_DESC) | sed 's/v\([0-9]*\).*/\1/')
SOFT_VER_MINOR := $(shell echo $(GIT_DESC) | sed 's/v[0-9]*.\([0-9]*\).*/\1/')

# Imported source files and paths.
CHIBIOS  := ../lib/chibios
BOARDDIR := ../boards
CONFDIR  := ./cfg
BUILDDIR := ../build/$(BOARD)/bl
DEPDIR   := ./.dep
CANARDDIR  := ../lib/canard
DSDLC_DIR  := ../lib/dsdlc_generated
DSDLC_SRC_DIR  := $(DSDLC_DIR)/src

# Board files.
include $(BOARDDIR)/$(BOARD)/board.mk
# Licensing files.
include $(CHIBIOS)/os/license/license.mk
# Startup files.
include $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk/$(CHIBIOS_BOARD_STARTUP)
# HAL-OSAL files (optional).
include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/hal/ports/STM32/$(CHIBIOS_BOARD_PLATFORM)
include $(CHIBIOS)/os/hal/osal/rt/osal.mk
# RTOS files (optional).
include $(CHIBIOS)/os/rt/rt.mk
include $(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC/mk/port_v7m.mk
# Other files (optional).
include $(CHIBIOS)/os/hal/lib/streams/streams.mk
include $(CHIBIOS)/os/various/shell/shell.mk

# Define linker script file here
LDSCRIPT= $(BOARDDIR)/$(BOARD)/bl.ld

# C sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CSRC = $(ALLCSRC) \
       $(TESTSRC) \
	     $(CANARDDIR)/canard.c \
       $(DSDLC_SRC_DIR)/uavcan.protocol.dynamic_node_id.Allocation.c \
       $(DSDLC_SRC_DIR)/uavcan.protocol.debug.LogLevel.c \
       $(DSDLC_SRC_DIR)/uavcan.protocol.debug.LogMessage.c \
       $(DSDLC_SRC_DIR)/uavcan.protocol.GetNodeInfo_req.c \
       $(DSDLC_SRC_DIR)/uavcan.protocol.GetNodeInfo_res.c \
       $(DSDLC_SRC_DIR)/uavcan.protocol.NodeStatus.c \
       $(DSDLC_SRC_DIR)/uavcan.protocol.SoftwareVersion.c \
       $(DSDLC_SRC_DIR)/uavcan.protocol.HardwareVersion.c \
       $(DSDLC_SRC_DIR)/uavcan.protocol.file.BeginFirmwareUpdate_req.c \
       $(DSDLC_SRC_DIR)/uavcan.protocol.file.BeginFirmwareUpdate_res.c \
       $(DSDLC_SRC_DIR)/uavcan.protocol.file.Path.c \
       $(DSDLC_SRC_DIR)/uavcan.protocol.file.GetInfo_req.c \
       $(DSDLC_SRC_DIR)/uavcan.protocol.file.GetInfo_res.c \
       $(DSDLC_SRC_DIR)/uavcan.protocol.file.Read_req.c \
       $(DSDLC_SRC_DIR)/uavcan.protocol.file.Read_res.c \
	     flash.c uavcan.c node.c firmware_update.c \
       main.c

# C++ sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CPPSRC = $(ALLCPPSRC)

# C sources to be compiled in ARM mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
ACSRC =

# C++ sources to be compiled in ARM mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
ACPPSRC =

# C sources to be compiled in THUMB mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
TCSRC =

# C sources to be compiled in THUMB mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
TCPPSRC =

# List ASM source files here
ASMSRC = $(ALLASMSRC)
ASMXSRC = $(ALLXASMSRC)

INCDIR = $(CONFDIR) $(ALLINC) $(TESTINC) $(CANARDDIR) $(DSDLC_DIR)/include

# Define C warning options here.
CWARN = -Wall -Wextra -Wundef -Wstrict-prototypes

# Define C++ warning options here.
CPPWARN = -Wall -Wextra -Wundef

#
# Project, target, sources and paths
##############################################################################

##############################################################################
# Start of user section
#

# List all user C define here, like -D_DEBUG=1
UDEFS = -DCANARD_ENABLE_CUSTOM_BUILD_CONFIG=1
UDEFS += -DGIT_DESC=$(GIT_DESC) -DSOFT_HASH=0x$(SOFT_HASH)
UDEFS += -DSOFT_VER=$(SOFT_VER) -DSOFT_VER_MAJOR=$(SOFT_VER_MAJOR) -DSOFT_VER_MINOR=$(SOFT_VER_MINOR)

# Define ASM defines here
UADEFS =

# List all user directories here
UINCDIR =

# List the user directory to look for the libraries here
ULIBDIR =

# List all user libraries here
ULIBS =

#
# End of user section
##############################################################################

##############################################################################
# Common rules
#

RULESPATH = $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk
include $(RULESPATH)/arm-none-eabi.mk
include $(RULESPATH)/rules.mk

#
# Common rules
##############################################################################

##############################################################################
# Custom rules
#

#
# Custom rules
##############################################################################