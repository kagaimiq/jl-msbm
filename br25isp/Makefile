TARGET = Tbr25

OBJS = \
	head.o \
	hakase.o \
	jl/jl_p33.o

CFLAGS  = -Werror -Os -Icom -Ijl
ASFLAGS = -c
LDFLAGS = -Tlink.ld -lc -lm -lcompiler-rt

LDFLAGS += --plugin-opt=mcpu=r3 --plugin-opt=mattr=+fprev1

#------------------------------------------------------------------------

# You can get the toolchain from here: https://doc.zh-jieli.com/Tools/zh-cn/other_info/index.html
JL_TOOLCHAIN_DIR ?= /opt/jieli/pi32v2
JL_TOOLCHAIN_BIN = $(JL_TOOLCHAIN_DIR)/bin/

#for some reason these directories is not being put in the compilation flags
# so the libc stuff (-lc, string.h, etc) is not available
CFLAGS  += -I"$(JL_TOOLCHAIN_DIR)/include"
LDFLAGS += -L"$(JL_TOOLCHAIN_DIR)/lib"

CC      = $(JL_TOOLCHAIN_BIN)cc
AS      = $(JL_TOOLCHAIN_BIN)cc
LD      = $(JL_TOOLCHAIN_BIN)lto-wrapper
OBJDUMP = $(JL_TOOLCHAIN_BIN)objdump
# TODO, suit the tools from the toolchain instead..
OBJCOPY = llvm-objcopy
SIZE    = llvm-size

all: $(TARGET).elf wmflash

.PHONY: all clean wmflash

clean:
	rm -f $(OBJS)
	rm -f $(TARGET).elf
	rm -f $(TARGET).bin
	rm -f $(TARGET).lst

$(TARGET).elf: $(OBJS)
	$(LD) $(OBJS) -o $@ $(LDFLAGS)
	$(SIZE) -A $@
	$(OBJDUMP) -d $@ > $(TARGET).lst

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@

################################################################################

WM_PORT ?= /dev/ttyUSB0

WM_IMG  = python3 /mnt/henta/reserch/WinnerMicro/hakase/wm_img.py
WM_LOAD = python3 /mnt/henta/reserch/WinnerMicro/hakase/wm_load.py

$(TARGET).img: $(TARGET).bin
	$(WM_IMG) \
		--hdr-addr 0x8002040 \
		--img-addr 0x8010000 \
		$< $@

wmflash: $(TARGET).img
	$(WM_LOAD) --port $(WM_PORT) $<
