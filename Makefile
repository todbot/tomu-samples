all: libopencm3-samples
	@true

libopencm3-samples:
	git submodule init
	git submodule update
	make -C libopencm3
	make -C miniblink
	make -C usb_cdcacm
	make -C usb_hid
	make -C usb_msc
	make -C usb_midi
	make -C usb_simple

clean:
	make -C libopencm3 clean
	make -C miniblink clean
	make -C usb_cdcacm clean
	make -C usb_hid clean
	make -C usb_msc clean
	make -C usb_midi clean
	make -C usb_simple clean

.PHONY: libopencm3-samples