# Configuration for the specific board
MCU = cortex-m3

CHIBIOS_BOARD_PLATFORM = STM32F1xx/platform_f105_f107.mk
CHIBIOS_BOARD_STARTUP = startup_stm32f1xx.mk

# List of all the board related files.
BOARDSRC = ../boards/supercan_v1.0/board.c

# Required include directories
BOARDINC = ../boards/supercan_v1.0

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)