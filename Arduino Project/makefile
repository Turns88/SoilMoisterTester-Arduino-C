# this makefile has been obtained from a trid party. It has benn modified to
# fit the requirements of CAB202
# Lats Modified, Luis Mejias, QUT 2021


#---------------- START customisable fields ---------------
#==== Main Options - students to modify =============================================================

#Step 1: modify this line to match the name of your c file
TARGETS = demo

SRC = $(TARGETS).c

OBJDIR = .
BACKUPDIR = backup
#step 2: find the serial/USB port where your microcontroller is attached
# in windows: it may be "COMx", e.g, "COM1", "COM2", "COM3"
# in OSX: it may be "/dev/tty.usbmodem1411201" , ""/dev/tty.usbserial-A700e1yQ"
# in linux: it may be "/dev/ttyACMx", "/dev/ttyUSBx"
AVRDUDE_PORT = /dev/tty.usbmodem14201
#AVRDUDE_PORT = /dev/tty.usbmodem143301
#AVRDUDE_PORT = /dev/tty.usbserial-A700e1yQ

# please note: by default this makefile is not configured for using
# the graphics library
#step 3: define the directories where your libraries are
CAB202_GRAPHICS = -I cab202_graphics
CAB202_LIBS = -L cab202_graphics -l cab202_graphics

#---------------- END customisable fields ---------------




# in normal circustances students do not need to modify below this line
#==== Programming Options (avrdude) ============================================

MCU = atmega328p
F_CPU = 16000000UL
AVRDUDE_PROGRAMMER = arduino
AVRDUDE_BAUD = 115200 #baud rate
AVRDUDE_FLAGS = -p $(MCU) -P $(AVRDUDE_PORT) -b $(AVRDUDE_BAUD) -c $(AVRDUDE_PROGRAMMER) $(AVRDUDE_NO_VERIFY)


#==== Compile Options ==========================================================

CFLAGS = \
	-std=gnu99 \
	-mmcu=atmega328p \
	-DF_CPU=16000000UL \
	-funsigned-char \
	-funsigned-bitfields \
	-ffunction-sections \
	-fpack-struct \
	-fshort-enums \
	-Wall \
	-Werror \
	-Wl,-u,vfprintf \
	-Os

#compiler option for using graphics library
CFLAGS += $(CAB202_GRAPHICS)
LDFLAGS = $(CAB202_LIBS)
#==== Targets ==================================================================

CC = avr-gcc
OBJCOPY = avr-objcopy
OBJDUMP = avr-objdump
AVRDUDE = avrdude
AVRSIZE = avr-size
REMOVE = rm -f
REMOVEDIR = rm -rf
TAIL = tail
AWK = awk

OBJ = $(SRC:%.c=$(OBJDIR)/%.o)

AVRDUDE_WRITE_FLASH = -U flash:w:$(TARGETS).hex
AVRDUDE_WRITE_EEPROM = -U eeprom:w:$(TARGETS).eep

MEMORYTYPES = flash eeprom fuse lfuse hfuse efuse boot calibration lock signature application apptable prodsig usersig


all: build


help:
	@echo 'Basic targets:'
	@echo '  build       Create all files.'
	@echo '  clean       Remove files created by make.'
	@echo '  size        Show the size of each section in the .elf file.'
	@echo
	@echo 'Create files:'
	@echo '  elf         Create binary .elf file.'
	@echo '  hex         Create .hex file containing .text and .data sections.'
	@echo '  eep         Create .eep file with the EEPROM content.'
	@echo '  lss         Create .lss file with a listing of the program.'
	@echo
	@echo 'Flashing:'
	@echo '  program     Write flash and EEPROM.'
	@echo '  flash       Write only flash.'
	@echo '  eeprom      Write only EEPROM.'
	@echo '  backup      Backup MCU content to "$(BACKUPDIR)". Available memory types:'
	@echo '              $(MEMORYTYPES)'
	@echo
	@echo 'Fuses:'
	@echo '  readfuses   Read fuses from MCU.'
	@echo '  writefuses  Write fuses to MCU using .fuse section.'
	@echo '  printfuses  Print fuses from .fuse section.'


build: elf hex eep lss size


elf: $(TARGETS).elf
hex: $(TARGETS).hex
eep: $(TARGETS).eep
lss: $(TARGETS).lss


program: flash eeprom


flash: $(TARGETS).hex
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH)


eeprom: $(TARGETS).eep
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_EEPROM)


readfuses:
	$(AVRDUDE) $(AVRDUDE_FLAGS) -U lfuse:r:-:i -U hfuse:r:-:i


#writefuses:
#	$(AVRDUDE) $(AVRDUDE_FLAGS) -U lfuse:w:$(LFUSE):m -U hfuse:w:$(HFUSE):m


writefuses: FUSES = $(shell $(OBJDUMP) -s --section=.fuse $(TARGETS).elf | tail -1 | awk '{print substr($$2,1,2),substr($$2,3,2),substr($$2,5,2)}')
writefuses: $(TARGETS).elf
	$(AVRDUDE) $(AVRDUDE_FLAGS) \
	$(if $(word 1,$(FUSES)),-U lfuse:w:0x$(word 1,$(FUSES)):m) \
	$(if $(word 2,$(FUSES)),-U hfuse:w:0x$(word 2,$(FUSES)):m) \
	$(if $(word 3,$(FUSES)),-U efuse:w:0x$(word 3,$(FUSES)):m)


printfuses: FUSES = $(shell $(OBJDUMP) -s --section=.fuse $(TARGET).elf | tail -1 | awk '{print substr($$2,1,2),substr($$2,3,2),substr($$2,5,2)}')
printfuses: $(TARGETS).elf
	@echo 'FUSES = $(FUSES)'


%.hex: %.elf
	$(OBJCOPY) -O ihex -j .text -j .data $< $@


%.eep: %.elf
	-$(OBJCOPY) -j .eeprom --set-section-flags=.eeprom="alloc,load" --change-section-lma .eeprom=0 -O ihex $< $@


%.lss: %.elf
	$(OBJDUMP) -h -S $< > $@


.SECONDARY: $(TARGETS).elf
.PRECIOUS: $(OBJ)
%.elf: $(OBJ)
	$(CC) $(CFLAGS) $^ --output $@ $(LDFLAGS)


$(OBJDIR)/%.o: %.c
	$(shell mkdir -p $(OBJDIR) 2>/dev/null)
	$(CC) -c $(CFLAGS)  $(CAB202_GRAPHICS)  $< -o $@


size: $(TARGETS).elf
	$(AVRSIZE) -A $(TARGETS).elf


clean:
	$(REMOVE) "$(TARGETS).hex"
	$(REMOVE) "$(TARGETS).eep"
	$(REMOVE) "$(TARGETS).elf"
	$(REMOVE) "$(TARGETS).lss"
	$(REMOVE) "$(TARGETS).o"
	#$(REMOVEDIR) "$(OBJDIR)"

cleanall:
		$(REMOVE) *.hex *.eep *.elf *.lss *.o



backup:
	$(shell mkdir -p $(BACKUPDIR) 2>/dev/null)
	@for memory in $(MEMORYTYPES); do \
	    $(AVRDUDE) $(AVRDUDE_FLAGS) -U $$memory:r:$(BACKUPDIR)/$(MCU).$$memory.hex:i; \
	done


.PHONY: all size build elf hex eep lss clean program flash eeprom readfuses writefuses printfuses backup help
