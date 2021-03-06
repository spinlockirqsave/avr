src=main
avrType=atmega32a
avrTypeDesc=m32
avrFreq=4000000 # 4MHz for accurate baudrate timing
programmerType=usbasp

cflags=-g -DF_CPU=$(avrFreq) -Wall -Os -Werror -Wextra

memoryTypes=calibration eeprom efuse flash fuse hfuse lfuse lock signature application apptable boot prodsig usersig

.PHONY: backup clean disassemble dumpelf edit eeprom elf flash fuses help hex makefile object program

help:
	@echo 'backup		Read all known memory types from controller and write it into a file. Available memory types: $(memoryTypes)'
	@echo 'clean		Delete automatically created files.'
	@echo 'disassemble	Compile source code, then disassemble object file to mnemonics.'
	@echo 'dumpelf		Dump the contents of the .elf file. Useful for information purposes only.'
	@echo 'eeprom		Extract EEPROM data from .elf file and program the device with it.'
	@echo 'elf			Create $(src).elf'
	@echo 'flash		Program $(src).hex to controller flash memory.'
	@echo 'fuses		Extract FUSES data from .elf file and program the device with it.'
	@echo 'help			Show this text.'
	@echo 'hex			Create all hex files for flash, eeprom and fuses.'
	@echo 'object		Create $(src).o'
	@echo 'program		Do all the programming to controller.'


#all: object elf hex

clean: 
	rm $(src).elf $(src).eeprom.hex $(src).fuses.hex $(src).lfuse.hex $(src).hfuse.hex $(src).efuse.hex $(src).flash.hex $(src).o
	date

object:
	avr-gcc $(cflags) -mmcu=$(avrType) -Wa,-ahlmns=$(src).lst -c -o $(src).o $(src).c

elf:	object
	avr-gcc $(cflags) -mmcu=$(avrType) -o $(src).elf $(src).o
	chmod a-x $(src).elf 2>&1

hex:    elf
	avr-objcopy -j .text -j .data -O ihex $(src).elf $(src).flash.hex
	avr-objcopy -j .eeprom --set-section-flags=.eeprom="alloc,load" --change-section-lma .eeprom=0 -O ihex $(src).elf $(src).eeprom.hex
#	avr-objcopy -j .fuse -O ihex $(src).elf $(src).fuses.hex --change-section-lma .fuse=0
#	srec_cat $(src).fuses.hex -Intel -crop 0x00 0x01 -offset  0x00 -O $(src).lfuse.hex -Intel
#	srec_cat $(src).fuses.hex -Intel -crop 0x01 0x02 -offset -0x01 -O $(src).hfuse.hex -Intel
#	srec_cat $(src).fuses.hex -Intel -crop 0x02 0x03 -offset -0x02 -O $(src).efuse.hex -Intel

disassemble: elf
	avr-objdump -s -j .fuse $(src).elf
	avr-objdump -C -d $(src).elf 2>&1

eeprom: hex
	avrdude -p$(avrTypeDesc) -c$(programmerType) -B 5 -v -U eeprom:w:$(src).eeprom.hex
	date

fuses: hex
	avrdude -p $(avrTypeDesc) -c$(programmerType) -B 5 -v -U lfuse:w:$(src).lfuse.hex
	#avrdude -p$(avrType) -c$(programmerType) -P$(programmerDev) -v -U hfuse:w:$(src).hfuse.hex
	#avrdude -p$(avrType) -c$(programmerType) -P$(programmerDev) -v -U efuse:w:$(src).efuse.hex
	date

dumpelf: elf
	avr-objdump -s -h $(src).elf

program: flash
program-all: flash eeprom fuses

flash: hex
	avrdude -p $(avrTypeDesc) -c$(programmerType) -B 5 -e -u -U flash:w:$(src).flash.hex
	date

backup:
	@for memory in $(memoryTypes); do \
        avrdude -p $(avrType) -c$(programmerType) -v -U $$memory:r:./$(avrType).$$memory.hex:i; \
    done
