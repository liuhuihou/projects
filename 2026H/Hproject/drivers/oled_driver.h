#ifndef OLED_DRIVER_H
#define OLED_DRIVER_H

#include <stdint.h>

void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t x, uint8_t y, char ch);
void OLED_ShowString(uint8_t x, uint8_t y, const char *str);
void OLED_ShowTenths(uint8_t x, uint8_t y, int32_t value_x10, uint8_t int_len);
void OLED_ShowSignedInt(uint8_t x, uint8_t y, int value, uint8_t width);

#endif
