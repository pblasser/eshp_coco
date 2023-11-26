SOURCES = main.c
PROG    = firmware
ARCH    = esp32
MDK     = ..
ESPUTIL = ../esputil/esputil
CFLAGS  = -W -Wall -Wextra -Werror -Wundef -Wshadow -pedantic \
               -Wdouble-promotion -fno-common -Wconversion \
               -mlongcalls -mtext-section-literals \
               -Os -ffunction-sections -fdata-sections \
               -I. -I$(MDK)/$(ARCH) $(EXTRA_CFLAGS)
LINKFLAGS   = -Tlink.ld -nostdlib -nostartfiles -Wl,--gc-sections $(EXTRA_LINKFLAGS)
CWD         = $(realpath $(CURDIR))
FLASH_ADDR  = 0x1000  # 2nd stage bootloader flash offset
TOOLCHAIN   =  xtensa-esp32-elf
SRCS        = boot.c $(SOURCES)

build: $(PROG).bin


tio:	
	sudo tio /dev/ttyUSB0

%:
	$(TOOLCHAIN)-gcc  $(CFLAGS) $@.c boot.c $(LINKFLAGS) -o $@.elf
	$(ESPUTIL) mkbin $@.elf $@.bin
	$(ESPUTIL) flash $(FLASH_ADDR) $@.bin
	sudo tio /dev/ttyUSB0
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
		
$(PROG).elf: $(SRCS)
	$(TOOLCHAIN)-gcc  $(CFLAGS) $(SRCS) $(LINKFLAGS) -o $@
#	$(TOOLCHAIN)-size $@

$(PROG).bin: $(PROG).elf $(ESPUTIL)
	$(ESPUTIL) mkbin $(PROG).elf $@

flash: $(PROG).bin $(ESPUTIL)
	$(ESPUTIL) flash $(FLASH_ADDR) $(PROG).bin

monitor: $(ESPUTIL)
	$(ESPUTIL) monitor


clean:
	@rm -rf *.{bin,elf,map,lst,tgz,zip,hex} $(PROG)*

#sudo modprobe usbserial
#sudo modprobe cp210x
#sudo tio /dev/ttyUSB0
#dmesg


