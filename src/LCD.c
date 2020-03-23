#include "LCD.h"
#include "pin_map.h"
#include "dcts.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_spi.h"

/**
  * @defgroup lcd
  * @brief work with LCD 128x64
  */

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
    int result = 0;
    SPI_HandleTypeDef spi = {0};
    spi.Instance = SPI1;
    spi.Init.Mode = SPI_MODE_MASTER;
    spi.Init.Direction = SPI_DIRECTION_1LINE;
    spi.Init.DataSize = SPI_DATASIZE_8BIT;
    spi.Init.CLKPolarity = SPI_POLARITY_LOW;
    spi.Init.CLKPhase = SPI_PHASE_1EDGE;
    spi.Init.NSS = SPI_NSS_SOFT;
    spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    spi.Init.TIMode = SPI_TIMODE_DISABLE;
    spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    spi.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&spi) != HAL_OK)
    {
        result = -1;
    }
    return result;
}
/**
 * @brief Deinit LCD SPI
 * @ingroup LCD
 */
void LCD_spi_deinit (void){
    SPI_HandleTypeDef spi = {0};
    spi.Instance = SPI1;
    HAL_SPI_DeInit(&spi);
}
/**
 * @brief Init LCD gpio
 * @ingroup LCD
 */
void LCD_gpio_init (void){
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
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
    GPIO_InitStruct.Pin = LCD_SCK_PIN;
    HAL_GPIO_Init(LCD_SCK_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = LCD_MOSI_PIN;
    HAL_GPIO_Init(LCD_MOSI_PORT, &GPIO_InitStruct);
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
