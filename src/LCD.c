#include "LCD.h"
#include "pin_map.h"
#include "dcts.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_spi.h"
#include "stm32f1xx_hal_rcc.h"
#include "math.h"

/**
  * @defgroup LCD
  * @brief work with LCD 128x64
  */

/*========= GLOBAL VARIABLES ==========*/

SPI_HandleTypeDef lcd_spi = {0};
LCD_t LCD = {
    .x = 0,
    .y = 0,
    .backlight_lvl = 2,
    .backlight = 0,
};
uint8_t LCD_buf[1024] = {0};

/*========== FUNCTIONS ==========*/

/**
 * @brief Init LCD
 * @return  0 - LCD init successfull,\n
 *          -1 - LCD SPI init error
 * @ingroup LCD
 */
int LCD_init (void){
    int result = 0;
    LCD_gpio_init();

    if(LCD_spi_init()<0){
        result = -1;
    }
    HAL_GPIO_WritePin(LCD_RST_PORT,LCD_RST_PIN,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_CS_PORT,LCD_CS_PIN,GPIO_PIN_RESET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(LCD_RST_PORT,LCD_RST_PIN,GPIO_PIN_SET);
    HAL_Delay(100);
    LCD_send(LCD_RW_WRITE, LCD_RS_COMM, 0x0C);
    LCD_send(LCD_RW_WRITE, LCD_RS_COMM, 0x01);
    HAL_Delay(2);
    LCD_send(LCD_RW_WRITE, LCD_RS_COMM, 0x36);

    LCD_clr();
    LCD_backlight_on();
    /*LCD_set_xy(0,38);
    LCD_print("    LCD", &Font_7x10, LCD_COLOR_BLACK);
    LCD_set_xy(0,20);
    LCD_print("initialised", &Font_7x10, LCD_COLOR_BLACK);
    LCD_set_xy(5,2);
    LCD_print("successful", &Font_7x10, LCD_COLOR_BLACK);
    LCD_update();
    HAL_Delay(2000);
    LCD_clr();*/
    LCD_update();
    return result;
}
/**
 * @brief Deinit LCD
 * @ingroup LCD
 */
void LCD_deinit (void){
    LCD_gpio_deinit();
    LCD_spi_deinit();
}
/**
 * @brief Init LCD SPI
 * @return  0 - SPI inited successful'\n
 *          -1 - SPI init error
 * @ingroup LCD
 */
int LCD_spi_init (void){
    __HAL_RCC_SPI1_CLK_ENABLE();
    int result = 0;
    lcd_spi.Instance = LCD_SPI;
    lcd_spi.Init.Mode = SPI_MODE_MASTER;
    lcd_spi.Init.Direction = SPI_DIRECTION_1LINE;
    lcd_spi.Init.DataSize = SPI_DATASIZE_8BIT;
    lcd_spi.Init.CLKPolarity = SPI_POLARITY_LOW;
    lcd_spi.Init.CLKPhase = SPI_PHASE_1EDGE;
    lcd_spi.Init.NSS = SPI_NSS_SOFT;
    lcd_spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
    lcd_spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    lcd_spi.Init.TIMode = SPI_TIMODE_DISABLE;
    lcd_spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    lcd_spi.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&lcd_spi) != HAL_OK)
    {
        result = -1;
    }
    __HAL_SPI_ENABLE(&lcd_spi);
    SPI_1LINE_TX(&lcd_spi);
    return result;
}
/**
 * @brief Deinit LCD SPI
 * @ingroup LCD
 */
void LCD_spi_deinit (void){
    HAL_SPI_DeInit(&lcd_spi);
}
/**
 * @brief Init LCD gpio
 * @ingroup LCD
 */
void LCD_gpio_init (void){
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pin = LCD_CS_PIN;
    HAL_GPIO_Init(LCD_CS_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = LCD_RST_PIN;
    HAL_GPIO_Init(LCD_RST_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Pin = LCD_LIGHT_PIN;
    HAL_GPIO_WritePin(LCD_LIGHT_PORT, LCD_LIGHT_PIN, GPIO_PIN_SET);
    HAL_GPIO_Init(LCD_LIGHT_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pin = LCD_SCK_PIN;
    HAL_GPIO_Init(LCD_SCK_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = LCD_MOSI_PIN;
    HAL_GPIO_Init(LCD_MOSI_PORT, &GPIO_InitStruct);

    __HAL_AFIO_REMAP_SPI1_ENABLE();
}
/**
 * @brief Deinit LCD gpio
 * @ingroup LCD
 */
void LCD_gpio_deinit (void){
    HAL_GPIO_DeInit(LCD_CS_PORT,LCD_CS_PIN);
    HAL_GPIO_DeInit(LCD_RST_PORT,LCD_RST_PIN);
    HAL_GPIO_DeInit(LCD_LIGHT_PORT,LCD_LIGHT_PIN);
    HAL_GPIO_DeInit(LCD_SCK_PORT,LCD_SCK_PIN);
    HAL_GPIO_DeInit(LCD_MOSI_PORT,LCD_MOSI_PIN);
}
/**
 * @brief Send data to LCD
 * @param rw - read or write
 * @param rs - command or data
 * @param data - data byte
 * @ingroup LCD
 */
void LCD_send(LCD_RW_t rw, LCD_RS_t rs, uint8_t data){
    uint8_t array[3] = {0xF8,0x00,0x00};
    array[0] |= (uint8_t)rw << 2;
    array[0] |= (uint8_t)rs << 1;
    array[1] |= (data & 0xF0);
    array[2] |= (uint8_t)(data & 0x0F) << 4;

    SPI_TypeDef *_lcd_spi = LCD_SPI;
    lcd_spi.Instance = LCD_SPI;

    HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN,GPIO_PIN_SET);
    HAL_SPI_Transmit(&lcd_spi,&array[0],3,100);
    while(((uint16_t)_lcd_spi->SR & SPI_SR_BSY)){
    }
    HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN,GPIO_PIN_RESET);
}
/**
 * @brief Update data from LCD_buf to screen
 * @ingroup LCD
 */
void LCD_update(void){
    uint8_t y = 0;
    for(uint16_t i = 0; i < LCD_BUF_ARRAY_LEN; i++){
        if(i%32 == 0){
            LCD_send(LCD_RW_WRITE, LCD_RS_COMM, (y + 0x80));
            LCD_send(LCD_RW_WRITE, LCD_RS_COMM, (0x80));
            y++;
        }
        LCD_send(LCD_RW_WRITE, LCD_RS_DATA, LCD.buf[i]);
    }
}
/**
 * @brief Set pixel in LCD_buf
 * @param x - horizontal position = {0..127}
 * @param y - vertical position = {0...63}
 * @param color - pixel's color = {LCD_COLOR_BLACK, LCD_COLOR_WHITE}
 * @ingroup LCD
 * @return  0 - OK,\n
 *          -1 - x or y out of range
 */
int LCD_set_pixel(uint8_t x, uint8_t y, LCD_color_t color){
    int result = 0;
    if((x > MAX_X) || (y > MAX_Y)){
        result = -1;
    }else{
        y = 63 - y;
        uint8_t x_pos = x/8;
        uint8_t x_shift = x%8;
        if(y > 31){
            x_pos += 16;
            y -= 32;
        }
        if(color == LCD_COLOR_BLACK){
            LCD.buf[y*32 + x_pos] |= (0x80 >> x_shift);
        }else{
            LCD.buf[y*32 + x_pos] &= ~(0x80 >> x_shift);
        }
    }
    return result;
}
/**
 * @brief Fills rectangular area by selected color
 * @param x1 - first corner's horisonal position = {0..127}
 * @param y1 - first corner's vertical position = {0...63}
 * @param x2 - second corner's horisonal position = {0..127}
 * @param y2 - second corner's vertical position = {0...63}
 * @param color - pixel's color = {LCD_COLOR_BLACK, LCD_COLOR_WHITE}
 * @ingroup LCD
 * @return  0 - OK,\n
 *          -1 - x1 out of range
 *          -2 - y1 out of range
 *          -3 - x2 out of range
 *          -4 - y2 out of range
 *
 * Automatically corrects x, y values if out of range,\n
 */
int LCD_fill_area(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, LCD_color_t color){
    int result = 0;
    //check coordinates values
    if(x1 > MAX_X){
        x1 = MAX_X;
        result = -1;
    }else if(y1 > MAX_Y){
        y1 = MAX_Y;
        result = -2;
    }else if(x2 > MAX_X){
        x2 = MAX_X;
        result = -3;
    }else if(y1 > MAX_Y){
        y2 = MAX_Y;
        result = -4;
    }
    // sort coordinates ascending
    if(x1 > x2){
        uint8_t temp = x2;
        x2 = x1;
        x1 = temp;
    }
    if(y1 > y2){
        uint8_t temp = y2;
        y2 = x1;
        y1 = temp;
    }
    // fill LCD_buf
    for (uint8_t y = y1; y <= y2; y++) {
        for (uint8_t x = x1; x <= x2; x++) {
            LCD_set_pixel(x,y,color);
        }
    }
    return result;
}
/**
 * @brief Fill LCD_buf by 0x00 (white pixels)
 * @ingroup LCD
 */
void LCD_clr(void){
    for (uint16_t i = 0; i < LCD_BUF_ARRAY_LEN; i++) {
        LCD.buf[i] = 0x00;
    }
}
/**
 * @brief Invert pixels color in set area
 * @param x1 - first corner's horisonal position = {0..127}
 * @param y1 - first corner's vertical position = {0...63}
 * @param x2 - second corner's horisonal position = {0..127}
 * @param y2 - second corner's vertical position = {0...63}
 * @param color - pixel's color = {LCD_COLOR_BLACK, LCD_COLOR_WHITE}
 * @ingroup LCD
 * @return  0 - OK,\n
 *          -1 - x1 out of range
 *          -2 - y1 out of range
 *          -3 - x2 out of range
 *          -4 - y2 out of range
 *
 * Automatically corrects x, y values if out of range,\n
 */
int LCD_invert_area(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2){
    int result = 0;
    uint8_t y_tmp = 0;
    uint8_t x_pos = 0;
    uint8_t x_shift = 0;
    //check coordinates values
    if(x1 > MAX_X){
        x1 = MAX_X;
        result = -1;
    }else if(y1 > MAX_Y){
        y1 = MAX_Y;
        result = -2;
    }else if(x2 > MAX_X){
        x2 = MAX_X;
        result = -3;
    }else if(y1 > MAX_Y){
        y2 = MAX_Y;
        result = -4;
    }
    // sort coordinates ascending
    if(x1 > x2){
        uint8_t temp = x2;
        x2 = x1;
        x1 = temp;
    }
    if(y1 > y2){
        uint8_t temp = y2;
        y2 = x1;
        y1 = temp;
    }
    // fill LCD_buf
    for (uint8_t y = y1; y <= y2; y++) {
        for (uint8_t x = x1; x <= x2; x++) {
            y_tmp = 63 - y;
            x_pos = x/8;
            x_shift = x%8;
            if(y_tmp > 31){
                x_pos += 16;
                y_tmp -= 32;
            }
            if(LCD.buf[y_tmp*32 + x_pos] & (0x80 >> x_shift)){
                LCD_set_pixel(x,y,LCD_COLOR_WHITE);
            }else{
                LCD_set_pixel(x,y,LCD_COLOR_BLACK);
            }
        }
    }
    return result;
}

/**
 * @brief Set coordinate to LCD var
 * @param x, y - coordinates
 * @ingroup LCD
 * @return  0 - OK,\n
 *          -1 - x out of range\n
 *          -2 - y out of range\n
 *
 * Automatically corrects x, y values if out of range,\n
 */
int LCD_set_xy(uint8_t x, uint8_t y){
    int result = 0;
    if(x > MAX_X){
        x = MAX_X;
        result = -1;
    }
    if(y > MAX_Y){
        y = MAX_Y;
        result = -2;
    }
    LCD.x = x;
    LCD.y = y;
    return result;
}

/**
 * @brief Turn on LCD backlight
 * @ingroup LCD
 */
void LCD_backlight_on (void){
    HAL_GPIO_WritePin(LCD_LIGHT_PORT, LCD_LIGHT_PIN, GPIO_PIN_RESET);
    LCD.backlight = 1;
}

/**
 * @brief Turn off LCD backlight
 * @ingroup LCD
 */
void LCD_backlight_off (void){
    HAL_GPIO_WritePin(LCD_LIGHT_PORT, LCD_LIGHT_PIN, GPIO_PIN_SET);
    LCD.backlight = 0;
}

/**
 * @brief Toggle LCD backlight
 * @ingroup LCD
 */
void LCD_backlight_toggle (void){
    if(LCD.backlight == 0){
        HAL_GPIO_WritePin(LCD_LIGHT_PORT, LCD_LIGHT_PIN, GPIO_PIN_RESET);
        LCD.backlight = 1;
    }else{
        HAL_GPIO_WritePin(LCD_LIGHT_PORT, LCD_LIGHT_PIN, GPIO_PIN_SET);
        LCD.backlight = 0;
    }
}

/**
 * @brief Print char on LCD
 * @param ch - character to be written
 * @param font - pointer to structure with used font
 * @param color - color used for drawing = {LCD_COLOR_BLACK, LCD_COLOR_WHITE}
 * @ingroup LCD
 * @return  0 - OK,\n
 *          -1 - have not free area for char\n
 *
 * @warning Automatically increases LCD.x position after print
 */
int LCD_print_char(char ch, FontDef_t* font, LCD_color_t color){
    int result = 0;
    //check coordinates values
    if((LCD.x + font->FontWidth > MAX_X) && (LCD.y + font->FontHeight > MAX_Y)){
        result = -1;
    }else{
        // go through font
        uint16_t line;
        for (uint8_t line_num = 1; line_num < (font->FontHeight+1); line_num++) {
            line = font->data[(ch - font->shift + 1) * font->FontHeight - line_num];
            for (uint8_t x_pos = 0; x_pos < font->FontWidth; x_pos++) {
                if (line & (0x8000 >> x_pos)) {
                    LCD_set_pixel(LCD.x + x_pos, (LCD.y + line_num), color);
                }
            }
        }
        LCD.x += font->FontWidth;
    }
    return result;
}

/**
 * @brief Print string on LCD
 * @param string - pionter to string buffer
 * @param font - pointer to structure with used font
 * @param color - color used for drawing = {LCD_COLOR_BLACK, LCD_COLOR_WHITE}
 * @ingroup LCD
 * @return  0 - OK,\n
 *          -1 - have not free area for next char\n
 *
 * @warning Automatically increases LCD.x position after print
 */
int LCD_print(char* string, FontDef_t* font, LCD_color_t color){
    int result = 0;
    while (*string) {
        if (LCD_print_char(*string, font, color) == 0) {
            string++;
        }else{
            result = -1;
        }
    }
    return result;
}

/**
 * @brief Calculate start position to put string on center of display
 * @param string - pionter to string buffer
 * @param font - pointer to structure with used font
 * @return x position for LCD_print()
 */
uint8_t align_text_center(char* string, FontDef_t font){
    uint8_t len = (uint8_t)strlen(string);
    return (uint8_t)(128-len*font.FontWidth)/2;
}
