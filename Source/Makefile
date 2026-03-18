#
# Makefile for usbaudio.audio — AHI sub-driver for USB Audio Class
#
# Cross-compilation for AmigaOS 4 (PowerPC)
#

CROSS_COMPILE = ppc-amigaos-
CC      = $(CROSS_COMPILE)gcc
STRIP   = $(CROSS_COMPILE)strip
HOSTCC  = gcc
RM      = rm -f
CRT	   ?= newlib

VERSION  = 2
REVISION = 0

# Include paths:
#   .                           → finds proto/ and interfaces/ locally
#   ../../SDK/local/common/include → finds libraries/libusb-1.h, interfaces/libusb-1.h
INCDIR = ../../libusb-1/SDK/local/common/include/
INCPATH = -I. -I$(INCDIR) 

# Compiler flags
OPTIMIZE = -O3 -fomit-frame-pointer
#DEBUG   = -gstabs -DDEBUG
CFLAGS   = -mcrt=$(CRT) -mstrict-align -Wall -Wwrite-strings $(DEBUG) $(OPTIMIZE) $(INCPATH)

# Linker flags — shared library, no C startup
LINK = -mcrt=$(CRT) -nostartfiles

# Output target
TARGET   = usbaudio.audio
MODEFILE = USBAUDIO

# Object files
OBJS = \
    DevInit/AHIdrv-USBaudioInit.o \
    device/AHIdrv-USBaudioMain.o \
    device/AHIdrv-USBaudioAccel.o \
    hw/AHIdrv-hwUSBPlayProc.o \
    hw/AHIdrv-hwUSBRecordProc.o

###################################################################

.PHONY: all clean install

all: $(TARGET) $(MODEFILE)

$(TARGET): $(OBJS)
	$(CC) $(LINK) -o $(TARGET).debug $(OBJS) -Wl,--cref,-M,-Map=$@.map
	$(STRIP) -R.comment $(TARGET).debug -o $(TARGET)

$(MODEFILE): gen_modefile
	./gen_modefile

gen_modefile: gen_modefile.c
	$(HOSTCC) -o $@ $<

clean:
	$(RM) $(TARGET) $(TARGET).debug $(TARGET).map $(MODEFILE) gen_modefile $(OBJS)
	@echo "Done!!"

install:
	copy $(TARGET) DEVS:AHI/
	copy $(MODEFILE) DEVS:AudioModes/
	@echo "Done!!"

###################################################################
# Compilation rules
###################################################################

DevInit/AHIdrv-USBaudioInit.o: DevInit/AHIdrv-USBaudioInit.c \
    includes/AHIdrv-USBaudio.h includes/AHIdrv-USBaudio.audio_rev.h \
    DevInit/AHIdrv-USBaudioVectors.c
	$(CC) -c $(CFLAGS) DevInit/AHIdrv-USBaudioInit.c -o $@

device/AHIdrv-USBaudioMain.o: device/AHIdrv-USBaudioMain.c \
    includes/AHIdrv-USBaudio.h includes/AHIdrv-USBaudio.audio_rev.h
	$(CC) -c $(CFLAGS) device/AHIdrv-USBaudioMain.c -o $@

device/AHIdrv-USBaudioAccel.o: device/AHIdrv-USBaudioAccel.c \
    includes/AHIdrv-USBaudio.h
	$(CC) -c $(CFLAGS) device/AHIdrv-USBaudioAccel.c -o $@

hw/AHIdrv-hwUSBPlayProc.o: hw/AHIdrv-hwUSBPlayProc.c \
    includes/AHIdrv-USBaudio.h
	$(CC) -c $(CFLAGS) hw/AHIdrv-hwUSBPlayProc.c -o $@
