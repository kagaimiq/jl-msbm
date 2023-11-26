TARGET = Tbr17

OBJS = \
	head.o \
	gakusuchi.o \
	com/xprintf.o \
	jl/jl_irq.o \
	jl/jl_irtc.o

CFLAGS  = -Werror -O2 -Icom -Ijl
ASFLAGS = -c
LDFLAGS = -Tlink.ld -lc -lm -lcompiler-rt

#------------------------------------------------------------------------

# You can get the toolchain from here: https://doc.zh-jieli.com/Tools/zh-cn/other_info/index.html
JL_TOOLCHAIN_DIR ?= /opt/jieli/pi32
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

all: $(TARGET).elf $(TARGET).bin

.PHONY: all clean

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
