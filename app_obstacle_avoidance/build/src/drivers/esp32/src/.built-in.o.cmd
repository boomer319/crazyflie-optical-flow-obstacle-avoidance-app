cmd_src/drivers/esp32/src/built-in.o :=  arm-none-eabi-gcc --specs=nosys.specs --specs=nano.specs -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -nostdlib  -r -o src/drivers/esp32/src/built-in.o src/drivers/esp32/src/esp_rom_bootloader.o src/drivers/esp32/src/esp_slip.o
