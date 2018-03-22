# these taken from the SLSTK3400A_usbmsd example
CFLAGS = $(IFLAGS) \
	-g \
	-gdwarf-2 \
	-mcpu=cortex-m0plus \
	-mthumb \
	-std=c99 \
	'-D__HEAP_SIZE=0x10' \
	'-DEFM32HG322F64=1' \
	'-DDEBUG=1' \
	-Os \
	-Wall \
	-fmessage-length=0 \
	-mno-sched-prolog \
	-fno-builtin \
	-ffunction-sections \
	-fdata-sections \
	-MMD \
	-MP 

LSCRIPT = ./tomu-msd.ld

LFLAGS = \
	-g \
	-gdwarf-2 \
	-mcpu=cortex-m0plus \
	-mthumb \
	-T"tomu-msd.ld" \
	-Xlinker --gc-sections -Xlinker \
	-Map="blinky.map" \
	--specs=nano.specs \

# -o goes here, then

LFLAGS2 = \
	-Wl,--start-group \
	-lgcc \
	-lc \
	-lnosys \
	-Wl,--end-group

	


