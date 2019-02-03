# u8g2
## Description
知名的显示图形库, 移植到了STM32F429上, 使用HAL库开发. 
## Usage
```C
u8g2_Setup_ssd1306_i2c_128x64_noname_f(u8g2_t *u8g2, const u8g2_cb_t *rotation， u8x8_byte_i2c_stm32_hal, u8x8_gpio_and_delay_stm32f4_hal);
u8g2_InitDisplay(&oled);
u8g2_SetPowerSave(&oled, 0);
```
## Reference
[U8g2 Reference Manual](https://github.com/olikraus/u8g2/wiki/u8g2reference)

[U8g2 Fonts](https://github.com/olikraus/u8g2/wiki/fntlistall)

[U8g2 C++/Arduino Setup](https://github.com/olikraus/u8g2/wiki/u8g2setupcpp)

[U8g2 C Setup](https://github.com/olikraus/u8g2/wiki/u8g2setupc)
