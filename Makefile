
CURRENT_PROJECT=mixer
UPLOAD_METHOD=swd

COMMON_FILES=ultranet.c ultranet.h i2s.c i2s.h

default: ${CURRENT_PROJECT}.${UPLOAD_METHOD}

build/op16_%.elf: %.c
	cmake --build build --parallel

build:
	cmake -B build

%.swd: build/op16_%.elf
	~/.pico-sdk/openocd/0.12.0+dev/openocd -s ~/.pico-sdk/openocd/0.12.0+dev/scripts -f interface/cmsis-dap.cfg -f target/rp2350.cfg -c 'adapter speed 5000; program $< verify reset exit'

%.usb: build/%elf
	echo "Not yet configured"

clean:
	rm -rf build
