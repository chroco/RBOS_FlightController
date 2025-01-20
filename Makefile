#BOARD= esp_wrover_kit/esp32/procpu
BOARD= esp32_devkitc_wroom/esp32/procpu
#BOARD= esp_wrover_kit/esp32/procpu
#BOARD= esp32s3_devkitc/esp32s3/appcpu
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

.PHONY: all write mcuboot clean

all: clean
	@west build -b $(BOARD) . # --sysbuild .

write: 
	@west flash --runner esp32 
#$(ESPTOOL) $(ARGS) $(BIN)

mcuboot:
	@west build -b $(BOARD) -DDTC_OVERLAY_FILE=$(OVERLAY) --sysbuild $(ZEPHYR_BASE)/samples/sysbuild/with_mcuboot

blinky:
	@west build -b $(BOARD)  --sysbuild $(ZEPHYR_BASE)/samples/basic/rgb_led

erase:
	$(ESPTOOL) erase_flash

clean:
	@rm -rf $(BUILD_DIR)/*
