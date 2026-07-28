#include "oled_driver.h"

#include "board_hardware.h"

#define OLED_COLS       (128U)
#define OLED_PAGES      (8U)
#define OLED_CHAR_W     (6U)

#define OLED_RST_HIGH() HW_GPIO_HIGH(HW_OLED_RST_PORT, HW_OLED_RST_PIN)
#define OLED_RST_LOW()  HW_GPIO_LOW(HW_OLED_RST_PORT, HW_OLED_RST_PIN)
#define OLED_DC_HIGH()  HW_GPIO_HIGH(HW_OLED_DC_PORT, HW_OLED_DC_PIN)
#define OLED_DC_LOW()   HW_GPIO_LOW(HW_OLED_DC_PORT, HW_OLED_DC_PIN)
#define OLED_SCL_HIGH() HW_GPIO_HIGH(HW_OLED_SCL_PORT, HW_OLED_SCL_PIN)
#define OLED_SCL_LOW()  HW_GPIO_LOW(HW_OLED_SCL_PORT, HW_OLED_SCL_PIN)
#define OLED_SDA_HIGH() HW_GPIO_HIGH(HW_OLED_SDA_PORT, HW_OLED_SDA_PIN)
#define OLED_SDA_LOW()  HW_GPIO_LOW(HW_OLED_SDA_PORT, HW_OLED_SDA_PIN)

static void oled_delay(void)
{
    volatile uint32_t i;
    for (i = 0U; i < 24U; ++i) {
        __NOP();
    }
}

static void oled_delay_ms(uint32_t milliseconds)
{
    while (milliseconds-- > 0U) {
        delay_cycles(HW_CPU_CLOCK_HZ / 1000U);
    }
}

static const uint8_t *font_for(char ch)
{
    static const uint8_t blank[5] = {0, 0, 0, 0, 0};
    static const uint8_t digits[10][5] = {
        {0x3E,0x51,0x49,0x45,0x3E},{0x00,0x42,0x7F,0x40,0x00},
        {0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4B,0x31},
        {0x18,0x14,0x12,0x7F,0x10},{0x27,0x45,0x45,0x45,0x39},
        {0x3C,0x4A,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},
        {0x36,0x49,0x49,0x49,0x36},{0x06,0x49,0x49,0x29,0x1E}
    };
    static const uint8_t upper[26][5] = {
        {0x7E,0x11,0x11,0x11,0x7E},{0x7F,0x49,0x49,0x49,0x36},
        {0x3E,0x41,0x41,0x41,0x22},{0x7F,0x41,0x41,0x22,0x1C},
        {0x7F,0x49,0x49,0x49,0x41},{0x7F,0x09,0x09,0x09,0x01},
        {0x3E,0x41,0x49,0x49,0x7A},{0x7F,0x08,0x08,0x08,0x7F},
        {0x00,0x41,0x7F,0x41,0x00},{0x20,0x40,0x41,0x3F,0x01},
        {0x7F,0x08,0x14,0x22,0x41},{0x7F,0x40,0x40,0x40,0x40},
        {0x7F,0x02,0x0C,0x02,0x7F},{0x7F,0x04,0x08,0x10,0x7F},
        {0x3E,0x41,0x41,0x41,0x3E},{0x7F,0x09,0x09,0x09,0x06},
        {0x3E,0x41,0x51,0x21,0x5E},{0x7F,0x09,0x19,0x29,0x46},
        {0x46,0x49,0x49,0x49,0x31},{0x01,0x01,0x7F,0x01,0x01},
        {0x3F,0x40,0x40,0x40,0x3F},{0x1F,0x20,0x40,0x20,0x1F},
        {0x3F,0x40,0x38,0x40,0x3F},{0x63,0x14,0x08,0x14,0x63},
        {0x07,0x08,0x70,0x08,0x07},{0x61,0x51,0x49,0x45,0x43}
    };
    static const uint8_t colon[5] = {0x00,0x36,0x36,0x00,0x00};
    static const uint8_t dot[5] = {0x00,0x60,0x60,0x00,0x00};
    static const uint8_t slash[5] = {0x20,0x10,0x08,0x04,0x02};
    static const uint8_t minus[5] = {0x08,0x08,0x08,0x08,0x08};
    static const uint8_t plus[5] = {0x08,0x08,0x3E,0x08,0x08};

    if (ch >= '0' && ch <= '9') return digits[(uint8_t)(ch - '0')];
    if (ch >= 'A' && ch <= 'Z') return upper[(uint8_t)(ch - 'A')];
    if (ch == ':') return colon;
    if (ch == '.') return dot;
    if (ch == '/') return slash;
    if (ch == '-') return minus;
    if (ch == '+') return plus;
    return blank;
}

static void oled_write_byte(uint8_t value, uint8_t command)
{
    uint8_t i;

    __disable_irq();
    if (command) OLED_DC_LOW(); else OLED_DC_HIGH();
    for (i = 0U; i < 8U; ++i) {
        OLED_SCL_LOW();
        if (value & 0x80U) OLED_SDA_HIGH(); else OLED_SDA_LOW();
        oled_delay();
        OLED_SCL_HIGH();
        oled_delay();
        value <<= 1;
    }
    OLED_SCL_HIGH();
    __enable_irq();
}

static void oled_cmd(uint8_t value) { oled_write_byte(value, 1U); }
static void oled_data(uint8_t value) { oled_write_byte(value, 0U); }

static void oled_cursor(uint8_t x, uint8_t y)
{
    uint8_t col = (uint8_t)(x * OLED_CHAR_W);
    if (x > 20U) x = 20U;
    if (y >= OLED_PAGES) y = OLED_PAGES - 1U;
    col = (uint8_t)(x * OLED_CHAR_W);
    oled_cmd((uint8_t)(0xB0U + y));
    oled_cmd((uint8_t)(col & 0x0FU));
    oled_cmd((uint8_t)(0x10U | (col >> 4)));
}

void OLED_ShowChar(uint8_t x, uint8_t y, char ch)
{
    const uint8_t *glyph = font_for(ch);
    uint8_t i;

    oled_cursor(x, y);
    for (i = 0U; i < 5U; ++i) oled_data(glyph[i]);
    oled_data(0U);
}

void OLED_ShowString(uint8_t x, uint8_t y, const char *str)
{
    while (*str != '\0' && x <= 20U) {
        OLED_ShowChar(x++, y, *str++);
    }
}

void OLED_Clear(void)
{
    uint8_t page;
    uint8_t col;

    for (page = 0U; page < OLED_PAGES; ++page) {
        oled_cursor(0U, page);
        for (col = 0U; col < OLED_COLS; ++col) oled_data(0U);
    }
}

void OLED_ShowTenths(uint8_t x, uint8_t y, int32_t value_x10, uint8_t int_len)
{
    char buf[16];
    uint32_t magnitude;
    uint32_t integer;
    uint8_t pos = 0U;
    uint8_t digits = 0U;
    uint8_t i;

    if (value_x10 < 0) {
        buf[pos++] = '-';
        magnitude = (uint32_t)(-value_x10);
    } else {
        magnitude = (uint32_t)value_x10;
    }
    integer = magnitude / 10U;
    do {
        buf[pos + digits] = (char)('0' + integer % 10U);
        integer /= 10U;
        ++digits;
    } while (integer != 0U && digits < 5U);
    for (i = 0U; i < digits / 2U; ++i) {
        char t = buf[pos + i];
        buf[pos + i] = buf[pos + digits - 1U - i];
        buf[pos + digits - 1U - i] = t;
    }
    while (digits < int_len) {
        for (i = digits; i > 0U; --i) buf[pos + i] = buf[pos + i - 1U];
        buf[pos] = ' ';
        ++digits;
    }
    pos += digits;
    buf[pos++] = '.';
    buf[pos++] = (char)('0' + magnitude % 10U);
    buf[pos] = '\0';
    OLED_ShowString(x, y, buf);
}

void OLED_ShowSignedInt(uint8_t x, uint8_t y, int value, uint8_t width)
{
    char buf[12];
    uint8_t pos = 0U;
    uint8_t digits = 0U;
    uint32_t magnitude;
    uint8_t i;

    if (value < 0) {
        buf[pos++] = '-';
        magnitude = (uint32_t)(-value);
    } else {
        buf[pos++] = '+';
        magnitude = (uint32_t)value;
    }
    do {
        buf[pos + digits] = (char)('0' + magnitude % 10U);
        magnitude /= 10U;
        ++digits;
    } while (magnitude != 0U && digits < 8U);
    for (i = 0U; i < digits / 2U; ++i) {
        char t = buf[pos + i];
        buf[pos + i] = buf[pos + digits - 1U - i];
        buf[pos + digits - 1U - i] = t;
    }
    while (digits < width) {
        for (i = digits; i > 0U; --i) buf[pos + i] = buf[pos + i - 1U];
        buf[pos] = ' ';
        ++digits;
    }
    buf[pos + digits] = '\0';
    OLED_ShowString(x, y, buf);
}

void OLED_Init(void)
{
    OLED_RST_HIGH();
    oled_delay_ms(10U);
    OLED_RST_LOW();
    oled_delay_ms(20U);
    OLED_RST_HIGH();
    oled_delay_ms(20U);

    oled_cmd(0xAEU); oled_cmd(0x20U); oled_cmd(0x10U);
    oled_cmd(0xB0U); oled_cmd(0xC8U); oled_cmd(0x00U); oled_cmd(0x10U);
    oled_cmd(0x40U); oled_cmd(0x81U); oled_cmd(0x7FU); oled_cmd(0xA1U);
    oled_cmd(0xA6U); oled_cmd(0xA8U); oled_cmd(0x3FU); oled_cmd(0xA4U);
    oled_cmd(0xD3U); oled_cmd(0x00U); oled_cmd(0xD5U); oled_cmd(0x80U);
    oled_cmd(0xD9U); oled_cmd(0x22U); oled_cmd(0xDAU); oled_cmd(0x12U);
    oled_cmd(0xDBU); oled_cmd(0x20U); oled_cmd(0x8DU); oled_cmd(0x14U);
    oled_cmd(0xAFU);
}
