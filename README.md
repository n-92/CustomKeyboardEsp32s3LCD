# LVGL - Porting Custom +  Keyboard on ESP32 S3 LCD

The example demonstrates how to port LVGL and create a custom keyboard

## How to Use

To use this example, please firstly install `ESP32_Display_Panel` (including its dependent libraries) and `lvgl` (v8.3.x) libraries, then follow the steps to configure them:

1. [Configure ESP32_Display_Panel](https://github.com/esp-arduino-libs/ESP32_Display_Panel#configure-esp32_display_panel)
2. [Configure LVGL](https://github.com/esp-arduino-libs/ESP32_Display_Panel#configure-lvgl)
3. [Configure Board](https://github.com/esp-arduino-libs/ESP32_Display_Panel#configure-board)
4. Place the files under USBHIDKeyboardMod folder under /home/yourID/.arduino15/packages/esp32/hardware/esp32/2.0.14/libraries/USB/src
5. Place the font lv_font_montserrat_16.c under /home/yourID/Arduino/libraries/lvgl/src/font
6. Place lv_conf.h under /home/yourID/Arduino/libraries
