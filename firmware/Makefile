#BOARD ?= lisa_m2.0
BOARD ?= supercan_v1.0
BMP_PORT ?= $(shell ls /dev/bmp-gdb 2>/dev/null || echo /dev/ttyACM0)
GDB ?= arm-none-eabi-gdb

all: bootloader node

clean:
	rm -rf build

bootloader:
	make -C bootloader BOARD=$(BOARD)

node:
	make -C node BOARD=$(BOARD)

%.upload:
	@echo "Using Black Magic Probe with SWD on BMP_PORT $(BMP_PORT)"
	@echo "Using GDB = $(GDB)"
	@echo "  BMP\t$<"
	$(Q)$(GDB) --batch \
		   -ex 'target extended-remote $(BMP_PORT)' \
		   -x swd_flash.scr \
		   build/$(BOARD)/$*/tuc_$*.elf

upload: bootloader node bl.upload node.upload

.PHONY: clean bootloader node upload
