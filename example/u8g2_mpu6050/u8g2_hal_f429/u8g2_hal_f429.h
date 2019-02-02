#ifndef _U8G2_HAL_F429_H_
#define _U8G2_HAL_F429_H_

#include "u8x8.h"
#include "stm32f4xx_hal.h"

void u8g2_i2c_handle_init(I2C_HandleTypeDef *i2c);
uint8_t u8x8_gpio_and_delay_stm32f4_hal(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
uint8_t u8x8_byte_i2c_stm32_hal(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);

#endif
//usage:
// void u8g2_Setup_ssd1306_i2c_128x64_noname_1(u8g2_t *u8g2, const u8g2_cb_t *rotation, u8x8_byte_i2c_stm32_hal, u8x8_gpio_and_delay_stm32f4_hal);
// void u8g2_Setup_ssd1306_i2c_128x64_vcomh0_1(u8g2_t *u8g2, const u8g2_cb_t *rotation, u8x8_byte_i2c_stm32_hal, u8x8_gpio_and_delay_stm32f4_hal);
// void u8g2_Setup_ssd1306_i2c_128x64_alt0_1(u8g2_t *u8g2, const u8g2_cb_t *rotation, u8x8_byte_i2c_stm32_hal, u8x8_gpio_and_delay_stm32f4_hal);
// void u8g2_Setup_ssd1306_i2c_128x64_noname_2(u8g2_t *u8g2, const u8g2_cb_t *rotation, u8x8_byte_i2c_stm32_hal, u8x8_gpio_and_delay_stm32f4_hal);
// void u8g2_Setup_ssd1306_i2c_128x64_vcomh0_2(u8g2_t *u8g2, const u8g2_cb_t *rotation, u8x8_byte_i2c_stm32_hal, u8x8_gpio_and_delay_stm32f4_hal);
// void u8g2_Setup_ssd1306_i2c_128x64_alt0_2(u8g2_t *u8g2, const u8g2_cb_t *rotation, u8x8_byte_i2c_stm32_hal, u8x8_gpio_and_delay_stm32f4_hal);
// void u8g2_Setup_ssd1306_i2c_128x64_noname_f(u8g2_t *u8g2, const u8g2_cb_t *rotation, u8x8_byte_i2c_stm32_hal, u8x8_gpio_and_delay_stm32f4_hal);
// void u8g2_Setup_ssd1306_i2c_128x64_vcomh0_f(u8g2_t *u8g2, const u8g2_cb_t *rotation, u8x8_byte_i2c_stm32_hal, u8x8_gpio_and_delay_stm32f4_hal);
// void u8g2_Setup_ssd1306_i2c_128x64_alt0_f(u8g2_t *u8g2, const u8g2_cb_t *rotation, u8x8_byte_i2c_stm32_hal, u8x8_gpio_and_delay_stm32f4_hal);
//rotation:
// --------------------------------------------------------------------------------
// |Layout	       | Description                                                  |
// |U8G2_R0	       | No rotation, landscape                                       |
// |U8G2_R1	       | 90 degree clockwise rotation                                 |
// |U8G2_R2	       | 180 degree clockwise rotation                                |
// |U8G2_R3	       | 270 degree clockwise rotation                                |
// |U8G2_MIRROR	   | No rotation, landscape, display content is mirrored (v2.6.x) |
// --------------------------------------------------------------------------------
