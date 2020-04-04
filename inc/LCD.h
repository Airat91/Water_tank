#include "stdint.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_spi.h"

#ifndef LCD_H
#define LCD_H 1

/*========== TYPEDEFS ==========*/

typedef enum {
    LCD_RW_WRITE = 0,
    LCD_RW_READ = 1,
}LCD_RW_t;

typedef enum {
    LCD_RS_COMM = 0,
    LCD_RS_DATA = 1,
}LCD_RS_t;

typedef enum {
    LCD_COLOR_WHITE = 0,
    LCD_COLOR_BLACK = 1,
}LCD_color_t;

/**
 * @brief lcd_spi
 * @ingroup LCD
 */
extern SPI_HandleTypeDef lcd_spi;

/**
 * @brief LCD_buf
 * @ingroup LCD
 */
extern uint8_t LCD_buf[];

/*========== FUNCTION PROTOTYPES ==========*/

int LCD_init (void);
void LCD_deinit (void);
int LCD_spi_init (void);
void LCD_spi_deinit (void);
void LCD_gpio_init (void);
void LCD_gpio_deinit (void);
void LCD_send(LCD_RW_t rw, LCD_RS_t rs, uint8_t data);
void LCD_update(void);
int LCD_set_pixel(uint8_t x, uint8_t y, LCD_color_t color);
int LCD_fill_area(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, LCD_color_t color);
int LCD_invert_area(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2);
void LCD_clr(void);

#endif // LCD_H
