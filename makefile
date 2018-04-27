# This makefile is made to work with the toolchain downloadable at https://launchpad.net/gcc-arm-embedded
# STM32F051C6/R6
CC = arm-none-eabi-gcc
LD = arm-none-eabi-gcc
SIZE = arm-none-eabi-size
OBJCOPY = arm-none-eabi-objcopy

# Build path
BUILD_DIR = build
TARGET = motcon

CFLAGS = -I. -Os -fno-common -ffunction-sections -ffreestanding -fno-builtin -mthumb -mcpu=cortex-m0 -Wall -fstack-usage -Winline -DPCB1B
ASMFLAGS = -S -fverbose-asm
LDFLAGS = -mcpu=cortex-m0 -mthumb -nostartfiles -gc-sections

DEPS = main.h own_std.h flash.h
OBJ = $(BUILD_DIR)/stm32init.o $(BUILD_DIR)/main.o $(BUILD_DIR)/own_std.o $(BUILD_DIR)/flash.o
ASMS = $(BUILD_DIR)/stm32init.s $(BUILD_DIR)/main.s $(BUILD_DIR)/own_std.s $(BUILD_DIR)/flash.s
STACK_USE = $(BUILD_DIR)/stm32init.su $(BUILD_DIR)/main.su $(BUILD_DIR)/own_std.su $(BUILD_DIR)/flash.su

all: $(BUILD_DIR)/$(TARGET).bin

$(BUILD_DIR)/%.o: %.c $(DEPS) | $(BUILD_DIR)
	$(CC) -c -o $@ $< $(CFLAGS)
	
$(BUILD_DIR):
	mkdir $@

clean:
	-rm -fR .dep $(BUILD_DIR)

$(BUILD_DIR)/$(TARGET).bin: $(OBJ) | $(BUILD_DIR)
	$(LD) -Tstm32.ld $(LDFLAGS) -o $(BUILD_DIR)/$(TARGET).elf $^
	$(OBJCOPY) -Obinary $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET)_full.bin
	$(OBJCOPY) -Obinary --remove-section=.flasher --remove-section=.settings $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).bin
	$(SIZE) $(BUILD_DIR)/$(TARGET).elf

flash_full: $(BUILD_DIR)/$(TARGET).bin
	stm32sprog -d /dev/ttyUSB0 -b 230400 -vw $(BUILD_DIR)/$(TARGET)_full.bin

flash: $(BUILD_DIR)/$(TARGET).bin
	stm32sprog -d /dev/ttyUSB0 -b 230400 -vw $(BUILD_DIR)/$(TARGET).bin

f: $(BUILD_DIR)/$(TARGET).bin
	scp $(BUILD_DIR)/$(TARGET).bin hrst@$(robot):~/rn1-tools/$(TARGET).bin

f_local: $(BUILD_DIR)/$(TARGET).bin
	../rn1-tools/mcprog /dev/ttyUSB0 $(BUILD_DIR)/$(TARGET).bin 4
	../rn1-tools/mcprog /dev/ttyUSB0 $(BUILD_DIR)/$(TARGET).bin 3

f4: $(BUILD_DIR)/$(TARGET).bin
	../rn1-tools/mcprog /dev/ttyUSB0 $(BUILD_DIR)/$(TARGET).bin 4

f3: $(BUILD_DIR)/$(TARGET).bin
	../rn1-tools/mcprog /dev/ttyUSB0 $(BUILD_DIR)/$(TARGET).bin 3

ff: $(BUILD_DIR)/$(TARGET).bin
	scp $(BUILD_DIR)/$(TARGET).bin hrst@proto4:~/rn1-tools/$(TARGET).bin

stack: $(BUILD_DIR)
	cat $(BUILD_DIR)/*.su

sections:
	arm-none-eabi-objdump -h $(BUILD_DIR)/$(TARGET).elf

syms:
	arm-none-eabi-objdump -t $(BUILD_DIR)/$(TARGET).elf

$(BUILD_DIR)/%.s: %.c $(DEPS) | $(BUILD_DIR)
	$(CC) -c -o $@ $< $(CFLAGS) $(ASMFLAGS)

asm: $(ASMS)

e:
	gedit --new-window main.c main.h flash.c flash.h stm32init.c &
