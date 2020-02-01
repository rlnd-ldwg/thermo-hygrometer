# copyright (c) 2020 Roland Ludwig
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

MCU = attiny13
TARGET = stk500v2
DEVICE = /dev/cu.SLAB_USBtoUART
SPEED = 115200
ifeq ($(OS),Windows_NT)
	AVRTOOLCHAIN = c:/.../avr
	AVRDUDEPATH = c:/.../avr
else
	AVRTOOLCHAIN = /.../.platformio/packages/toolchain-atmelavr/bin
	AVRDUDEPATH = /.../.platformio/packages/tool-avrdude/
endif

AVRDUDECFG = $(AVRDUDEPATH)/avrdude.conf
FUSES = -U lfuse:w:0x6a:m -U hfuse:w:0xff:m -U efuse:w:0xff:m

SRCFILE = main.asm
FILENAME = $(basename $(SRCFILE))

$(FILENAME).hex: $(FILENAME).elf
	$(AVRTOOLCHAIN)/avr-objcopy -O ihex $(FILENAME).elf $(FILENAME).hex

$(FILENAME).o: $(SRCFILE)
	$(AVRTOOLCHAIN)/avr-as -mmcu $(MCU) -v -o $(FILENAME).o $(SRCFILE)
    # -almg=$(FILENAME).list

$(FILENAME).elf: $(FILENAME).o
	$(AVRTOOLCHAIN)/avr-ld -v -o $(FILENAME).elf $(FILENAME).o

list: $(FILENAME).elf
	$(AVRTOOLCHAIN)/avr-objdump -h -S $(FILENAME).elf > $(FILENAME).lst

flash: $(FILENAME).hex
	$(AVRDUDEPATH)/bin/avrdude -C $(AVRDUDECFG) -b $(SPEED) -c $(TARGET) -p $(MCU) -P $(DEVICE) -v -U flash:w:$(FILENAME).hex:i

showfuses:
	$(AVRDUDEPATH)/bin/avrdude -C $(AVRDUDECFG) -b $(SPEED) -c $(TARGET) -p $(MCU) -P $(DEVICE) -v 2>&1  |  grep "Fuses OK"
# |  grep "fuse reads" | tail -n2

.PHONY: clean
clean:
ifeq ($(OS),Windows_NT)
	del /f $(FILENAME).o $(FILENAME).elf $(FILENAME).hex $(FILENAME).lst
else
	rm -f $(FILENAME).o $(FILENAME).elf $(FILENAME).hex $(FILENAME).lst
endif
