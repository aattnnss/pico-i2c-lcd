# pico-i2c-lcd
## LCD Display Driver for the Raspberry Pi Pico written in C

Usage Example:
```
#include "pico_i2c_lcd.h"

#define LCD_I2C_ADDRESS 0x27
#define SDA_PIN 0
#define SCL_PIN 1
#define DISP_LINES 2
#define DISP_COLUMNS 16

int main() {
    pico_i2c_lcd *lcd = lcd_init(i2c0, LCD_I2C_ADDRESS, SDA_PIN, SCL_PIN, DISP_LINES, DISP_COLUMNS);
    lcd_putstr(lcd, "Hello!");
}
```