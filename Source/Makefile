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
OPTIMIZE = -O2
#DEBUG   = -gstabs -DDEBUG
# Extra flags for tuning experiments, e.g.
#   make clean && make TUNE="-DFRAMES_PER_IOR=8 -DSCHEDULE_DEPTH_FACTOR=1"
TUNE ?=

CFLAGS   = -mcrt=$(CRT) -mstrict-align -Wall -Wwrite-strings $(DEBUG) $(OPTIMIZE) $(INCPATH) $(TUNE)

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

.PHONY: all clean install diag

all: $(TARGET) $(MODEFILE)

# Build the normal driver plus the 188-byte clamp diagnostic side by side into
# Source/builds/.  The clamp build announces itself in its own log line, so a
# capture can always be traced back to the binary that produced it.
# Note: `all` is named explicitly in the recursive calls -- a bare $(MAKE)
# would re-enter this target, since make builds the first rule in the file.
diag:
	$(MAKE) clean
	$(MAKE) all TUNE="-DDIAG_MAX_FRAME_BYTES=188"
	mkdir -p builds
	cp $(TARGET) builds/$(TARGET).DIAG-clamp188
	$(MAKE) clean
	$(MAKE) all
	cp $(TARGET) builds/$(TARGET).NORMALE
	$(MAKE) clean
	$(MAKE) all TUNE="-DFORCE_EXTRA_RATE=44100"
	cp $(TARGET) builds/$(TARGET).FORCE44100
	$(MAKE) clean
	$(MAKE) all
	@echo ""
	@echo "Builds written to Source/builds/:"
	@ls -l builds/

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
