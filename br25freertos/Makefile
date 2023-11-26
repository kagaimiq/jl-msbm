TARGET = Tbr25

OBJS = \
	head.o \
	hakase.o \
	hakasenano.o \
	com/xprintf.o \
	jl/jl_irq.o \
	jl/jl_p33.o

CFLAGS  = -Werror -Os -Icom -Ijl -Iinc
ASFLAGS = -c
LDFLAGS = -Tlink.ld -lc -lm -lcompiler-rt

CFLAGS  += -mcpu=r3 -mfprev1
LDFLAGS += --plugin-opt=mcpu=r3 --plugin-opt=mattr=+fprev1

OBJS += \
	freertos/tasks.o \
	freertos/list.o \
	freertos/queue.o \
	freertos/stream_buffer.o \
	freertos/event_groups.o \
	freertos/timers.o \
	freertos/MemMang/heap_4.o \
	jlport/port.o

CFLAGS += -Ifreertos/include -Ijlport

#------------------------------------------------------------------------

# You can get the toolchain from here: https://doc.zh-jieli.com/Tools/zh-cn/other_info/index.html
JL_TOOLCHAIN_DIR ?= /opt/jieli/pi32v2
JL_TOOLCHAIN_BIN = $(JL_TOOLCHAIN_DIR)/bin/

#for some reason these directories is not being put in the compilation flags
# so the libc stuff (-lc, string.h, etc) is not available
CFLAGS  += -I$(JL_TOOLCHAIN_DIR)/include
LDFLAGS += -L$(JL_TOOLCHAIN_DIR)/lib

CC      = $(JL_TOOLCHAIN_BIN)cc
AS      = $(JL_TOOLCHAIN_BIN)cc
LD      = $(JL_TOOLCHAIN_BIN)lto-wrapper
OBJDUMP = $(JL_TOOLCHAIN_BIN)objdump
# TODO, suit the tools from the toolchain instead..
OBJCOPY = llvm-objcopy
SIZE    = llvm-size

all: $(TARGET).elf $(TARGET).bin $(TARGET).fw

.PHONY: all clean

clean:
	rm -f $(OBJS)
	rm -f $(TARGET).elf
	rm -f $(TARGET).bin
	rm -f $(TARGET).lst

$(TARGET).elf: $(OBJS)
	$(LD) $(OBJS) -o $@ $(LDFLAGS)
	$(SIZE) -A $@
	$(OBJDUMP) -mcpu=r3 -d $@ > $(TARGET).lst

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@

#####################################################

JL_MISCTOOLS = ../../../MiscTools

$(TARGET).fw: $(TARGET).bin
	python3 $(JL_MISCTOOLS)/firmware/mkake.py $< $@
