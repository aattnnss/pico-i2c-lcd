#include "pico/stdlib.h"
#include "hardware/i2c.h"

typedef struct
{
    uint8_t num_lines;
    uint8_t num_columns;
    uint8_t cursor_x;
    uint8_t cursor_y;
    bool backlight;
    i2c_inst_t *i2c;
    uint8_t addr;
} pico_i2c_lcd;

pico_i2c_lcd *lcd_init(i2c_inst_t *i2c, uint8_t addr,
                       uint8_t sda, uint8_t scl,
                       uint8_t num_lines, uint8_t num_columns);
void lcd_release(pico_i2c_lcd *lcd);
void lcd_backlight(pico_i2c_lcd *lcd, bool light_on);
void lcd_clear(pico_i2c_lcd *lcd);
void lcd_move_cursor(pico_i2c_lcd *lcd, uint8_t cursor_x, uint8_t cursor_y);
void lcd_putchar(pico_i2c_lcd *lcd, char c);
void lcd_putstr(pico_i2c_lcd *lcd, const char *str);
void lcd_show_cursor(pico_i2c_lcd *lcd, bool show);
void lcd_blink_cursor(pico_i2c_lcd *lcd, bool blink);
void lcd_display_power(pico_i2c_lcd *lcd, bool on);
