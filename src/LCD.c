#include "LCD.h"
#include "pin_map.h"
#include "dcts.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_spi.h"
#include "stm32f1xx_hal_rcc.h"

/**
  * @defgroup LCD
  * @brief work with LCD 128x64
  */

/**
  * @addtogroup LCD
  * @{
  */
#define LCD_SPI SPI1
/**
  * @}
  */

SPI_HandleTypeDef lcd_spi = {0};

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
    //HAL_Delay(100);
    LCD_send(LCD_RW_WRITE, LCD_RS_COMM, 0x36);
    //HAL_Delay(100);
    LCD_send(LCD_RW_WRITE, LCD_RS_COMM, 0x01);
    //HAL_Delay(100);
    LCD_send(LCD_RW_WRITE, LCD_RS_COMM, 0x80);
    LCD_send(LCD_RW_WRITE, LCD_RS_COMM, 0x80);
    LCD_send(LCD_RW_WRITE, LCD_RS_DATA, 0xFF);
    LCD_send(LCD_RW_WRITE, LCD_RS_DATA, 0xFF);
    LCD_send(LCD_RW_WRITE, LCD_RS_DATA, 0xFF);
    LCD_send(LCD_RW_WRITE, LCD_RS_DATA, 0xFF);
    LCD_send(LCD_RW_WRITE, LCD_RS_DATA, 0xFF);
    LCD_send(LCD_RW_WRITE, LCD_RS_DATA, 0xFF);
    LCD_send(LCD_RW_WRITE, LCD_RS_DATA, 0xFF);
    LCD_send(LCD_RW_WRITE, LCD_RS_DATA, 0xFF);
    LCD_send(LCD_RW_WRITE, LCD_RS_DATA, 0xFF);
    LCD_send(LCD_RW_WRITE, LCD_RS_DATA, 0xFF);
    LCD_send(LCD_RW_WRITE, LCD_RS_DATA, 0xFF);
    LCD_send(LCD_RW_WRITE, LCD_RS_DATA, 0xFF);
    LCD_send(LCD_RW_WRITE, LCD_RS_DATA, 0xFF);
    LCD_send(LCD_RW_WRITE, LCD_RS_DATA, 0xFF);
    LCD_send(LCD_RW_WRITE, LCD_RS_DATA, 0xFF);
    LCD_send(LCD_RW_WRITE, LCD_RS_DATA, 0xFF);

    for(uint8_t y = 0; y < 32; y++){
        for (uint8_t x = 0; x < 8; x++){
        LCD_send(LCD_RW_WRITE, LCD_RS_COMM, (y + 0x80));
        LCD_send(LCD_RW_WRITE, LCD_RS_COMM, (x + 0x80));
        LCD_send(LCD_RW_WRITE, LCD_RS_DATA, 0xFF);
        LCD_send(LCD_RW_WRITE, LCD_RS_DATA, 0xFF);
        }
    }
    for(uint8_t y = 0; y < 32; y++){
        for (uint8_t x = 8; x < 16; x++){
        LCD_send(LCD_RW_WRITE, LCD_RS_COMM, (y + 0x80));
        LCD_send(LCD_RW_WRITE, LCD_RS_COMM, (x + 0x80));
        LCD_send(LCD_RW_WRITE, LCD_RS_DATA, 0xFF);
        LCD_send(LCD_RW_WRITE, LCD_RS_DATA, 0xFF);
        }
    }

    for(uint8_t y = 0; y < 32; y++){
        for (uint8_t x = 0; x < 8; x++){
        LCD_send(LCD_RW_WRITE, LCD_RS_COMM, (y + 0x80));
        LCD_send(LCD_RW_WRITE, LCD_RS_COMM, (x + 0x80));
        LCD_send(LCD_RW_WRITE, LCD_RS_DATA, 0x00);
        LCD_send(LCD_RW_WRITE, LCD_RS_DATA, 0x00);
        }
    }
    for(uint8_t y = 0; y < 32; y++){
        for (uint8_t x = 8; x < 16; x++){
        LCD_send(LCD_RW_WRITE, LCD_RS_COMM, (y + 0x80));
        LCD_send(LCD_RW_WRITE, LCD_RS_COMM, (x + 0x80));
        LCD_send(LCD_RW_WRITE, LCD_RS_DATA, 0x00);
        LCD_send(LCD_RW_WRITE, LCD_RS_DATA, 0x00);
        }
    }
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
    lcd_spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
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
    //HAL_GPIO_WritePin(LCD_RST_PORT, LCD_RST_PIN, GPIO_PIN_SET);
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
    HAL_StatusTypeDef error = HAL_OK;
    uint8_t array[3] = {0xF8,0x00,0x00};
    array[0] |= (uint8_t)rw << 2;
    array[0] |= (uint8_t)rs << 1;
    array[1] |= (data & 0xF0);
    array[2] |= (uint8_t)(data & 0x0F) << 4;

    SPI_TypeDef *_lcd_spi = LCD_SPI;
    lcd_spi.Instance = LCD_SPI;

    HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN,GPIO_PIN_SET);
    error = HAL_SPI_Transmit(&lcd_spi,&array[0],3,100);
    while(((uint16_t)_lcd_spi->SR & SPI_SR_BSY)){
    }
    HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN,GPIO_PIN_RESET);
    //HAL_Delay(1);
}
