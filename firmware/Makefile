#BOARD ?= lisa_m2.0
BOARD ?= supercan_v1.0

all: bootloader

clean:
	rm -rf build

bootloader:
	make -C bootloader BOARD=$(BOARD)

.PHONY: clean bootloader
