TARGET = libcab202_graphics.a

SRC = graphics.c lcd.c 
HDR = graphics.h lcd.h macros.h
OBJ = graphics.o lcd.o 

FLAGS = \
	-mmcu=atmega328p \
	-DF_CPU=16000000UL \
	-Os \
	-funsigned-char \
	-funsigned-bitfields \
	-ffunction-sections \
	-fpack-struct \
	-Wall \
	-Werror \
	-fshort-enums \
	-std=gnu99 

all: $(TARGET)

clean:
	if [ -f $(TARGET) ]; then rm $(TARGET); fi
	for f in $(OBJ); do if [ -f $${f} ] ; then rm $${f}; fi; done

rebuild: clean all

$(TARGET) : $(HDR) $(SRC) 
	avr-gcc -c $(SRC) $(FLAGS)
	avr-ar rcs $(TARGET) $(OBJ)
