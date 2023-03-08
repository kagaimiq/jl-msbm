TARGET = AC6908C

OBJS = \
	pre.o \
	kotatsu9.o \
	jl_audio.o

CFLAGS  = -Werror -O2
ASFLAGS = -c
LDFLAGS = -Tlink.ld -lc -lm -lcompiler-rt

CFLAGS  += -I ../commoninc

CFLAGS += -I ../commonsrc
OBJS += \
	../commonsrc/xprintf.o

#------------------------------------------------------------------------

# You can get the toolchain from here: https://doc.zh-jieli.com/Tools/zh-cn/other_info/index.html
JL_TOOLCHAIN_DIR = ../../../Toolchain/pi32

#for some reason these directories is not being put in the compilation flags
# so the libc stuff (-lc, string.h, etc) is not available
CFLAGS  += -I"$(JL_TOOLCHAIN_DIR)/include"
LDFLAGS += -L"$(JL_TOOLCHAIN_DIR)/lib"

JL_TOOLCHAIN_BIN = $(JL_TOOLCHAIN_DIR)/bin/
CC      = $(JL_TOOLCHAIN_BIN)cc
AS      = $(JL_TOOLCHAIN_BIN)cc
LD      = $(JL_TOOLCHAIN_BIN)lto-wrapper
OBJDUMP = $(JL_TOOLCHAIN_BIN)objdump
OBJCOPY = llvm-objcopy
SIZE    = llvm-size

all: $(TARGET).elf $(TARGET).bin

.PHONY: all clean run

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

JL_ARG ?= 0x4777

run: $(TARGET).bin
	python3 ../../../jl-uboot-tool/jlrunner.py \
		--device $(JL_DEVICE) 0x2000 $< --arg $(JL_ARG)
