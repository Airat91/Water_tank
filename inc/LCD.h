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

/**
 * @brief lcd_spi
 * @ingroup LCD
 */
extern SPI_HandleTypeDef lcd_spi;

/*========== FUNCTION PROTOTYPES ==========*/

int LCD_init (void);
void LCD_deinit (void);
int LCD_spi_init (void);
void LCD_spi_deinit (void);
void LCD_gpio_init (void);
void LCD_gpio_deinit (void);
void LCD_send(LCD_RW_t rw, LCD_RS_t rs, uint8_t data);

#endif // LCD_H
