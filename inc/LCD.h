#ifndef LCD_H
#define LCD_H 1

/*========== TYPEDEFS ==========*/



/*========== FUNCTION PROTOTYPES ==========*/

int LCD_init (void);
void LCD_deinit (void);
int LCD_spi_init (void);
void LCD_spi_deinit (void);
void LCD_gpio_init (void);
void LCD_gpio_deinit (void);

#endif // LCD_H
