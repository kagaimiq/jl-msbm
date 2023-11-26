TARGET = ac6900loader

OBJS = \
	head.o \
	hakase.o \
	jl/jl_irtc.o

CFLAGS  = -Werror -Os -Icom -Ijl
ASFLAGS = 
LDFLAGS = -Tlink.ld

#------------------------------------------------------------------------

###############################################################
# You can obtain the toolchain there:                         #
#  https://doc.zh-jieli.com/Tools/zh-cn/other_info/index.html #
###############################################################

#JL_TOOLCHAIN_DIR ?= /opt/jieli/pi32
JL_TOOLCHAIN_DIR ?= /c/JL/pi32

JL_TOOLCHAIN_BIN = $(JL_TOOLCHAIN_DIR)/bin/

# don't forget about the compiler's includes and libraries!
CFLAGS  += -I $(JL_TOOLCHAIN_DIR)/include
LDFLAGS += -L $(JL_TOOLCHAIN_DIR)/lib

CC      = $(JL_TOOLCHAIN_BIN)clang
AS      = $(JL_TOOLCHAIN_BIN)clang
LD      = $(JL_TOOLCHAIN_BIN)lto-wrapper
OBJDUMP = $(JL_TOOLCHAIN_BIN)llvm-objdump
OBJCOPY = $(JL_TOOLCHAIN_BIN)llvm-objcopy

CFLAGS  += -target pi32
ASFLAGS += -target pi32 -c
LDFLAGS += -lcompiler-rt -lc -lm

#-----------------------------------------------------

all: $(TARGET).elf $(TARGET).bin

.PHONY: all clean

clean:
	rm -f $(OBJS)
	rm -f $(TARGET).elf
	rm -f $(TARGET).bin
	rm -f $(TARGET).lst

$(TARGET).elf: $(OBJS)
	$(LD) $(OBJS) -o $@ $(LDFLAGS)
	$(OBJDUMP) -d $@ > $(TARGET).lst

$(TARGET).bin: $(TARGET).elf
	# WHY???
	$(OBJCOPY) -O binary $< _text   -j .text   -p 4
	$(OBJCOPY) -O binary $< _rodata -j .rodata -p 4
	$(OBJCOPY) -O binary $< _data   -j .data   -p 4
	cat _text _rodata _data > $@
	rm  _text _rodata _data
