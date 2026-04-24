# ─── EE109 Speedometer Project Makefile ─────────────────────────────────────
# Adjust PROGRAMMER and PORT to match your system (same as previous labs).

MAINFILE = project
OBJECTS  = project.o lcd.o adc.o

# ── Toolchain settings (same as previous labs) ───────────────────────────────
PROGRAMMER = arduino
PORT       = /dev/cu.usbmodem*     # macOS example – change if needed
                                   # Linux: /dev/ttyACM0  or  /dev/ttyUSB0
MCU        = atmega328p
F_CPU      = 16000000

CC         = avr-gcc
CFLAGS     = -g -Os -Wall -mmcu=$(MCU) -DF_CPU=$(F_CPU)
LDFLAGS    = -mmcu=$(MCU)

AVRDUDE       = avrdude
AVRDUDEFLAGS  = -p $(MCU) -c $(PROGRAMMER) -P $(PORT) -b 115200

# ── Build targets ─────────────────────────────────────────────────────────────
all: main.hex

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

main.elf: $(OBJECTS)
	$(CC) $(LDFLAGS) -o $@ $^

main.hex: main.elf
	avr-objcopy -O ihex -R .eeprom $< $@
	@echo "Build complete."

# ── Flash target ──────────────────────────────────────────────────────────────
flash: main.hex
	$(AVRDUDE) $(AVRDUDEFLAGS) -U flash:w:main.hex:i

# ── Clean ─────────────────────────────────────────────────────────────────────
clean:
	rm -f *.o *.elf *.hex