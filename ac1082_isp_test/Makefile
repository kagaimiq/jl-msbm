TARGET = JieLi1082

OBJS = \
	main.rel

CFLAGS  = --model-small
ASFLAGS = 
LDFLAGS = --model-small --code-loc 0xc000 --code-size 0x2000

all: $(TARGET).ihx wmflash

.PHONY: all clean wmflash

%.rel: %.c
	sdcc $(CFLAGS) -c -o $@ $<

%.rel: %.s
	sdas8051 $(ASFLAGS) -o $<

$(TARGET).ihx: $(OBJS)
	sdcc $(LDFLAGS) --out-fmt-ihx -o $@ $(OBJS)

$(TARGET).bin: $(TARGET).ihx
	sdobjcopy -Obinary -Iihex $< $@

clean:
	rm -f $(OBJS)
	rm -f $(addsuffix .asm, $(basename $(OBJS)))
	rm -f $(addsuffix .lst, $(basename $(OBJS)))
	rm -f $(addsuffix .sym, $(basename $(OBJS)))
	rm -f $(addsuffix .rst, $(basename $(OBJS)))
	rm -f $(TARGET).ihx
	rm -f $(TARGET).lk
	rm -f $(TARGET).map
	rm -f $(TARGET).mem
	rm -f $(TARGET).bin

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
