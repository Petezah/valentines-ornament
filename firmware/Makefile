MCU := attiny85
AVRDUDEMCU := t85
SYSCLOCK := 8000000
CC := /usr/bin/avr-gcc
CFLAGS := -g -Os -Wall -mcall-prologues -mmcu=$(MCU) -DF_CPU=$(SYSCLOCK) -I./

AVRDUDE ?= /usr/bin/avrdude
ISP_PORT ?= /dev/ttyS4
ISP_PROGRAMMER ?= stk500v1
ISP_BAUD ?= 115200
AVRDUDE_CMD := $(AVRDUDE) -p $(AVRDUDEMCU) -P $(ISP_PORT) -c $(ISP_PROGRAMMER) -b $(ISP_BAUD)

AVRSIZE ?= /usr/bin/avr-size
OBJ2HEX ?= /usr/bin/avr-objcopy

SOURCES := main.cpp
TARGET := valentines_ornament

all :
	$(CC) $(CFLAGS) $(SOURCES) -o $(TARGET)
	$(AVRSIZE) $(TARGET)	
	$(OBJ2HEX) -R .eeprom -O ihex $(TARGET) $(TARGET).hex
	rm -f $(TARGET)

install : all
	$(AVRDUDE_CMD) -U flash:w:$(TARGET).hex

fuse :
	$(AVRDUDE_CMD) -U lfuse:w:0xe4:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m

clean :
	rm -f *.hex *.obj *.o
