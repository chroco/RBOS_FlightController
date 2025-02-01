TEENSY_BOARD= teensy41
ESP32_BOARD= esp32_devkitc_wroom/esp32/procpu
OPTIONS= -p always 
BUILD_DIR= build/
WORKSPACE= ~/zephyr/zephyr-workspace
#WORKSPACE= /home/chroco/zephyr/zephyr-workspace
ESPTOOL= $(WORKSPACE)/modules/hal/espressif/tools/esptool_py/esptool.py

.PHONY: all esp32 teensy write esp32_erase clean

#all: clean
#	@west build $(OPTION)-b $(BOARD) .

esp32: clean
	@west build $(OPTION)-b $(ESP32_BOARD) . # --sysbuild .

teensy: clean
	@west build $(OPTION)-b $(TEENSY_BOARD) . # --sysbuild .

write: 
	@west flash 

esp32_erase:
	$(ESPTOOL) erase_flash

monitor:
	@west espressif monitor

clean:
	@rm -rf $(BUILD_DIR)/*
