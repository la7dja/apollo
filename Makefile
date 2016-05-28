# Makefile for AVR target

# Define directories.
  INCDIR  = /usr/lib/avr/include
  LIBCDIR = /usr/lib/avr/lib/avr51
  LIBDIR  = .
    LIBS  = 
KERNELSRC = kernel
KERNELINC = $(KERNELSRC)/include

# MCU type and frequency
MCU = at90usb1287
F_CPU=8000000L

# Output format. Can be [srec|ihex].
FORMAT = ihex

# Target file name (without extension).
TARGET = apollo

# Type of ISP programmer. Use avrdude -c? to get a list of valid programmers
#ISPID=jtag2isp # Atmel JTAG ICE mkII in ISP mode
#ISPDEV=usb
ISPID=stk500 # Atmel STK500 compatible
ISPDEV=/dev/stk500 # change to actual device


# Define programs.
SHELL = sh
CC = avr-gcc
AS = avr-gcc -x assembler-with-cpp
REMOVE = rm -f
COPY = cp
MOVE = mv
OBJCOPY = avr-objcopy
OBJDUMP = avr-objdump
HEXSIZE = @avr-size --target=$(FORMAT) $(TARGET).hex
ELFSIZE = @avr-size $(TARGET).elf


SRC = main.c lpf.c atu.c fuses.c

# Required kernel source files
SRC += $(KERNELSRC)/kernel.c $(KERNELSRC)/spi.c $(KERNELSRC)/analog.c

# Compiler flags
CFLAGS = -O2 --std=gnu99 -fno-strict-aliasing -fshort-enums -Wall -Wa,-ahlms=$(<:.c=.lst) -I$(KERNELINC) -DF_CPU=$(F_CPU)

# Linker flags (passed via GCC)
LDFLAGS = -L$(LIBCDIR) -B$(LIBCDIR) -L$(LIBDIR) -Wl,-Map=$(TARGET).map,--cref

# Additional library flags (-lm = math library).
LIBFLAGS = $(LIBS) -lm 

# Define all project specific object files.
OBJ	= $(SRC:.c=.o)

# Define all listing files.
LST = $(SRC:.c=.lst)

# Compiler flags to generate dependency files.
GENDEPFLAGS = -Wp,-M,-MP,-MT,$(*F).o,-MF,.dep/$(@F).d

# Add target processor to flags
CFLAGS += -mmcu=$(MCU) $(GENDEPFLAGS) -DTASK_DEFAULT_STACK_SIZE=128
LDFLAGS += -mmcu=$(MCU)	

# Use of USB makes debugging easier, but requires that LUFA is available
ifeq ($(USE_USB),yes)
USB=lufa/LUFA/Drivers/USB

SRC += usb.c $(wildcard $(USB)/LowLevel/*.c) $(wildcard $(USB)/HighLevel/*.c) $(USB)/Class/Device/CDC.c Descriptors.c 

CFLAGS += -I./ -I$(USB) -DUSB_CAN_BE_DEVICE -DUSB_DEVICE_ONLY -DFIXED_CONTROL_ENDPOINT_SIZE=8 -DFIXED_NUM_CONFIGURATIONS=1 -DUSE_STATIC_OPTIONS="(USB_DEVICE_OPT_FULLSPEED | USB_OPT_REG_ENABLED | USB_OPT_AUTO_PLL)" -DF_CLOCK=$(F_CPU) -DUSE_FLASH_DESCRIPTORS -DUSB_INT_TASK -DUSB_DEBUG

LIBFLAGS += -Wl,-u,vfprintf -lprintf_flt
endif


.PHONY : build
build: $(TARGET).elf $(TARGET).hex $(TARGET).lss $(TARGET).bin line1 overallsize line2

.PHONY : BuildAll
buildall: clean $(TARGET).elf $(TARGET).hex $(TARGET).lss $(TARGET).bin line1 overallsize line2

.PHONY : overallsize
overallsize:
	@echo Elf size:
	$(ELFSIZE)

%.bin: %.elf
	$(OBJCOPY) -j .text -j .data -O binary $< $@

# Create final output files (.hex, .lss) from ELF output file.
%.hex: %.elf
	$(OBJCOPY) -O $(FORMAT)  -R .eeprom -R .fuse $< $@

# Create extended listing file from ELF output file.
%.lss: %.elf
	$(OBJDUMP) -h -S $< > $@

# Link: create ELF output file from object files.
$(TARGET).elf: $(OBJ)
	$(CC) $(LDFLAGS) $(OBJ) $(LIBFLAGS) --output $@ 

# Compile: create object files from C source files.
%.o : %.c
	$(CC) -c $(CFLAGS) -I$(INCDIR) $< -o $@ 

# Compile: create assembler files from C source files.
%.s : %.c
	$(CC) -S -fverbose-asm $(CFLAGS) -I$(INCDIR) $< -o $@ 

# Assemble: create object files from assembler files.
%.o : %.s
	$(AS) -c $(ASFLAGS) $< -o $@

# Firmware upload via usb 
dfu: $(TARGET).hex
	dfu-programmer $(MCU) erase
	dfu-programmer $(MCU) flash $(TARGET).hex
	dfu-programmer $(MCU) start

# Firmware upload via ISP
ispprog:	 $(TARGET).hex
	avrdude -p$(MCU) -P$(ISPDEV) -c$(ISPID) -Uflash:w:$(TARGET).hex

ispfuse:	fuses
	avrdude -p$(MCU) -P$(ISPDEV) -c$(ISPID) -Uhfuse:w:hfuse.hex:i -Ulfuse:w:lfuse.hex:i -Uefuse:w:efuse.hex:i -u

# Target to reset device via ISP
ispreset:
	avrdude -p$(MCU) -P$(ISPDEV) -c$(ISPID)

fuses:	$(TARGET).elf
	avr-objcopy -j .fuse -O ihex $(TARGET).elf fuses.hex --change-section-lma .fuse=0
	srec_cat fuses.hex -Intel -crop 0x00 0x01 -offset  0x00 -O lfuse.hex -Intel
	srec_cat fuses.hex -Intel -crop 0x01 0x02 -offset -0x01 -O hfuse.hex -Intel
	srec_cat fuses.hex -Intel -crop 0x02 0x03 -offset -0x02 -O efuse.hex -Intel


# Target: line1 project.
.PHONY : line1
line1 :
	@echo ---------------------------------------------------------------------------------------------------

# Target: line2 project.
.PHONY : line2
line2 :
	@echo ---------------------------------------------------------------------------------------------------

# Target: clean project.
.SILENT : Clean
.PHONY : Clean
clean :
	$(REMOVE) $(TARGET).hex
	$(REMOVE) $(TARGET).obj
	$(REMOVE) $(TARGET).elf
	$(REMOVE) $(TARGET).bin
	$(REMOVE) $(TARGET).map
	$(REMOVE) $(TARGET).obj
	$(REMOVE) $(TARGET).sym
	$(REMOVE) $(TARGET).lnk
	$(REMOVE) $(TARGET).lss
	$(REMOVE) $(OBJ)
	$(REMOVE) $(LST)
	$(REMOVE) $(SRC:.c=.s)



# Include the dependency files.
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)


# List assembly only source file dependencies here:

