# My Special BareMetals for JIE LI

Just some bare-metal junk I'm learning the JieLi chips with.

You certainly won't need anything that's in there, yet I published them for some reason.

Maybe headers like `jl_xxxx_regs.h`, etc. will come handy, perhaps?

## Overview

### CD02

- [ac1082_isp_test](ac1082_isp_test/) - A test payload for the AC109N via the ISP, dumps some stuff (also uploads to my Winnermicro W806 based ISP dongle which is now absent - want an RP2040 dongle!)

### BR17

- [tbr17](tbr17/)
  * [hakase.c](tbr17/hakase.c) - A main playground, has some USB code, exception handling test, ..
  * [usbattempt2.c](tbr17/usbattempt2.c) - Another USB attempt, essentially
  * [usb-nuts.c](tbr17/usb-nuts.c) - Straight up ripped code from the ROM that does the USB magic..
- [br17cd03](br17cd03/) - Tried to emulate an CD03 (AC410N) loader to work on the AC410N support, haven't done yet..
- [br17modplayer](br17modplayer/) - An MOD player attempt
  * [themodplayer.c](br17modplayer/themodplayer.c) - The MOD player itself. Plays a MOD right from the flash!
  * [thes3mplayer.c](br17modplayer/thes3mplayer.c) - Tried to do an S3M player but I'm too lazy <sub>(well..)</sub>
- [br17freertosmod](br17freertosmod/) - FreeRTOS port test which plays a MOD
  * for now you have to pull the FreeRTOS itself by yourself
- [br17isp](br17isp/) - ISP test payload (uses PR2 as an bit-banged UART port since AC6908 have only two/four GPIOs in total - three are used for ISP)

### BR25

- [tbr25](tbr25/)
  * [hakase.c](tbr25/hakase.c) - Playground again, there's some FM RX attempt that doesn't fully work yet
  * [fmtx_test.c](tbr25/fmtx_test.c) - The FM TX test which finally works! and yet it turned to be really weird, the system clock is the FM VCO?? It also has a MOD player, which is a bit improved over the BR17 one I made..
  * [lcdthing2.c](tbr25/lcdthing2.c) - Some LCD thing that displays an image from flash and displays a marquee on top
  * [jl_sd_test_3000.c](tbr25/jl_sd_test_3000.c) - Some SD host test
  * [modplayer.c](tbr25/modplayer.c) - That's the MOD player used for FM TX test
- [br25freertos/](br25freertos/) - FreeRTOS port test with the same LCD marquee thing and a tick counter - I reached 300000 ticks!
- [br25isp/](br25isp/) - ISP test payload again
- [br25minitest/](br25minitest/) - Yet another test thing, doesn't have too much thus it's the "mini test"

## Borrowed stuff

- Most hardware-related code ripped off the JieLi's binary blobs or like.
- The "Aixoid_9x20" (`AIXOID9.F20`) font is taken from [fontraption](https://github.com/viler-int10h/vga-text-mode-fonts)
- A lot of instances of [xprintf](http://elm-chan.org/fsw/strf/xprintf.html) - just because I'm used to it.
- A bit of [FatFs](http://elm-chan.org/fsw/ff/00index_e.html) too.
- More is to be found out..
