#BOARD= teensy41
BOARD= esp32_devkitc_wroom/esp32/procpu
OPTIONS= -p always 
BUILD_DIR= build/
ESPTOOL= /home/chroco/zephyr/zephyr-workspace/modules/hal/espressif/tools/esptool_py/esptool.py
PORT= /dev/ttyACM0
BAUD= 921600
MODE= qio
FORCE=# --force
OVERLAY=# esp32s3_devkitc_procpu.overlay
OVERLAY= esp32_devkitc_wroom_procpu.overlay
BIN= /home/chroco/zephyr/blinky/build/mcuboot/zephyr/zephyr.bin
ARGS= --port $(PORT) --chip auto --baud $(BAUD) --before default_reset --after hard_reset write_flash $(FORCE) -u --flash_mode $(MODE) --flash_freq 40m --flash_size detect 0x0000

.PHONY: all write esp32_erase clean

all: clean
	@west build $(OPTION)-b $(BOARD) . # --sysbuild .

write: 
	@west flash 

esp32_erase:
	$(ESPTOOL) erase_flash

monitor:
	@west espressif monitor

clean:
	@rm -rf $(BUILD_DIR)/*
