#include "pico_i2c_lcd.h"
#include <stdlib.h>

#define LCD_CLR 0x01
#define LCD_HOME 0x02

#define LCD_ENTRY_MODE 0x04
#define LCD_ENTRY_INC 0x02
#define LCD_ENTRY_SHIFT 0x01

#define LCD_ON_CTRL 0x08
#define LCD_ON_DISPLAY 0x04
#define LCD_ON_CURSOR 0x02
#define LCD_ON_BLINK 0x01

#define LCD_MOVE 0x10
#define LCD_MOVE_DISP 0x08
#define LCD_MOVE_RIGHT 0x04

#define LCD_FUNCTION 0x20
#define LCD_FUNCTION_8BIT 0x10
#define LCD_FUNCTION_2LINES 0x08
#define LCD_FUNCTION_10DOTS 0x04
#define LCD_FUNCTION_RESET 0x30

#define LCD_CGRAM 0x40
#define LCD_DDRAM 0x80

#define LCD_RS_CMD 0
#define LCD_RS_DATA 1

#define LCD_RW_WRITE 0
#define LCD_RW_READ 1

#define MASK_RS 0x01
#define MASK_RW 0x02
#define MASK_E 0x04

#define SHIFT_BACKLIGHT 3
#define SHIFT_DATA 4

#define BAUD 100000

static inline void write_byte_masked(pico_i2c_lcd *lcd, uint8_t byte)
{
    uint8_t byte_masked = byte | MASK_E;
    i2c_write_blocking(lcd->i2c, lcd->addr, &byte_masked, 1, false);
    i2c_write_blocking(lcd->i2c, lcd->addr, &byte, 1, false);
}

static void write_command(pico_i2c_lcd *lcd, uint8_t cmd)
{
    write_byte_masked(lcd, (lcd->backlight << SHIFT_BACKLIGHT) | (((cmd >> 4) & 0x0f) << SHIFT_DATA));
    write_byte_masked(lcd, (lcd->backlight << SHIFT_BACKLIGHT) | ((cmd & 0x0f) << SHIFT_DATA));
    if (cmd <= 3)
    {
        sleep_ms(5);
    }
}

static void write_data(pico_i2c_lcd *lcd, uint8_t data)
{
    write_byte_masked(lcd, MASK_RS | (lcd->backlight << SHIFT_BACKLIGHT) | (((data >> 4) & 0x0f) << SHIFT_DATA));
    write_byte_masked(lcd, MASK_RS | (lcd->backlight << SHIFT_BACKLIGHT) | ((data & 0x0f) << SHIFT_DATA));
}

static void io_init(i2c_inst_t *i2c, uint8_t sda, uint8_t scl)
{
    i2c_init(i2c, BAUD);
    gpio_set_function(sda, GPIO_FUNC_I2C);
    gpio_set_function(scl, GPIO_FUNC_I2C);
    gpio_pull_up(sda);
    gpio_pull_up(scl);
}

pico_i2c_lcd *lcd_init(i2c_inst_t *i2c, uint8_t addr,
                       uint8_t sda, uint8_t scl,
                       uint8_t num_lines, uint8_t num_columns)
{
    pico_i2c_lcd *lcd = malloc(sizeof(pico_i2c_lcd));
    lcd->num_lines = num_lines;
    lcd->num_columns = num_columns;
    lcd->cursor_x = 0;
    lcd->cursor_y = 0;
    lcd->backlight = true;
    lcd->i2c = i2c;
    lcd->addr = addr;
    io_init(i2c, sda, scl);
    return lcd;
}

void lcd_release(pico_i2c_lcd *lcd)
{
    free(lcd);
}

void lcd_clear(pico_i2c_lcd *lcd)
{
    write_command(lcd, LCD_CLR);
    write_command(lcd, LCD_HOME);
    lcd->cursor_x = 0;
    lcd->cursor_y = 0;
}

void lcd_move_cursor(pico_i2c_lcd *lcd, uint8_t cursor_x, uint8_t cursor_y)
{
    lcd->cursor_x = cursor_x;
    lcd->cursor_y = cursor_y;
    uint8_t addr = cursor_x & 0x3f;
    if (cursor_y & 1)
    {
        addr += 0x40;
    }
    if (cursor_y & 2)
    {
        addr += lcd->num_columns;
    }
    write_command(lcd, LCD_DDRAM | addr);
}

void lcd_putchar(pico_i2c_lcd *lcd, char c)
{
    if (c == '\n')
    {
        lcd->cursor_x = lcd->num_columns;
    }
    else
    {
        write_data(lcd, c);
        lcd->cursor_x++;
    }
    if (lcd->cursor_x >= lcd->num_columns)
    {
        lcd->cursor_x = 0;
        lcd->cursor_y++;
    }
    if (lcd->cursor_y >= lcd->num_lines)
    {
        lcd->cursor_y = 0;
    }
    lcd_move_cursor(lcd, lcd->cursor_x, lcd->cursor_y);
}

void lcd_putstr(pico_i2c_lcd *lcd, const char *str)
{
    for (uint8_t i = 0; str[i] != '\0'; i++)
    {
        lcd_putchar(lcd, str[i]);
    }
}

void lcd_backlight(pico_i2c_lcd *lcd, bool light_on)
{
    uint8_t byte = light_on ? 1 << SHIFT_BACKLIGHT : 0;
    i2c_write_blocking(lcd->i2c, lcd->addr, &byte, 1, false);
    lcd->backlight = light_on;
}

void lcd_show_cursor(pico_i2c_lcd *lcd, bool show)
{
    write_command(lcd,
                  LCD_ON_CTRL | LCD_ON_DISPLAY | (show ? LCD_ON_CURSOR : 0));
}

void lcd_blink_cursor(pico_i2c_lcd *lcd, bool blink)
{
    write_command(lcd,
                  LCD_ON_CTRL | LCD_ON_DISPLAY | LCD_ON_CURSOR | (blink ? LCD_ON_BLINK : 0));
}

void lcd_display_power(pico_i2c_lcd *lcd, bool on)
{
    write_command(lcd,
                  LCD_ON_CTRL | (on ? LCD_ON_DISPLAY : 0));
}