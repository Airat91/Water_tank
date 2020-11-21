#include "adc.h"
#include "pin_map.h"
#include "dcts.h"
#include "dcts_config.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_rcc.h"
#include <math.h>
/**
  * @defgroup ADC
  * @brief work with ADC channels
  */

ADC_HandleTypeDef hadc1;

#define ADC_BUF_SIZE 50
#define ADC_PERIOD 100
#define ADC_MAX 4095
#define ADC_VREF 3.3f

#define PWR_K   (float)2.187
#define VREF_INT (float)1.2

#define WTR_LEV_A1  (float)5.50069e-02
#define WTR_LEV_A2  (float)-2.06272e+01
static const float wtr_lev_coef[2] = {
    5.50069e-02,
    -2.06272e+01,
};

#define WTR_TMP_A1  (float)-4.16259e-15
#define WTR_TMP_A2  (float) 3.41573e-11
#define WTR_TMP_A3  (float)-1.10305e-07
#define WTR_TMP_A4  (float) 1.79162e-04
#define WTR_TMP_A5  (float)-1.75460e-01
#define WTR_TMP_A6  (float) 1.27727e+02
static const float wtr_tmpr_coef[6] = {
    1.27727e+02,
    -1.75460e-01,
    1.79162e-04,
    -1.10305e-07,
    3.41573e-11,
    -4.16259e-15,
};

/*========== FUNCTIONS ==========*/

/**
 * @brief Init and start ADC
 * @return  0 - ADC init successfull,\n
 *          -1 - ADC config error,\n
 *          -2 - PWR channel config error,\n
 *          -3 - WTR_LEV channel config error,\n
 *          -4 - WTR_TMP channel config error,\n
 *          -5 - TMP channel config error,\n
 *          -6 - ADC start error,
 * @ingroup ADC
 */
int adc_init (void){
    int result = 0;
    __HAL_RCC_ADC1_CLK_ENABLE();
    adc_gpio_init();
    ADC_InjectionConfTypeDef sConfigInjected = {0};

    //Common config
    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 5;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        result = -1;
    }

    sConfigInjected.InjectedNbrOfConversion = 4;
    sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_7CYCLES_5;
    sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
    sConfigInjected.AutoInjectedConv = ENABLE;
    sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
    sConfigInjected.InjectedOffset = 0;

    //Configure PWR Channel
    sConfigInjected.InjectedChannel = ADC_CHANNEL_0;
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
    {
        result = -2;
    }
    //Configure WTR_LEV Channel
    sConfigInjected.InjectedChannel = ADC_CHANNEL_4;
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
    {
        result = -3;
    }
    //Configure WTR_TMP Channel
    sConfigInjected.InjectedChannel = ADC_CHANNEL_5;
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_3;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
    {
        result = -4;
    }
    //Configure TMP Channel
    sConfigInjected.InjectedChannel = ADC_CHANNEL_VREFINT;
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_4;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
    {
        result = -5;
    }
    //Start ADC
    if (HAL_ADC_Start(&hadc1) != HAL_OK){
        result = -6;
    }

    return result;
}
/**
 * @brief Deinit ADC
 * @ingroup ADC
 */
void adc_deinit (void){
    HAL_ADC_Stop(&hadc1);
    HAL_ADC_DeInit(&hadc1);
    __HAL_RCC_ADC1_CLK_DISABLE();
    adc_gpio_deinit();
}
/**
 * @brief Init ADC gpio
 * @ingroup ADC
 */
void adc_gpio_init (void){
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pin = PWR_PIN;
    HAL_GPIO_Init(PWR_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = WTR_LEV_PIN;
    HAL_GPIO_Init(WTR_LEV_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = WTR_TMP_PIN;
    HAL_GPIO_Init(WTR_TMP_PORT, &GPIO_InitStruct);
}
/**
 * @brief Deinit ADC gpio
 * @ingroup ADC
 */
void adc_gpio_deinit (void){
    HAL_GPIO_DeInit(PWR_PORT,PWR_PIN);
    HAL_GPIO_DeInit(WTR_LEV_PORT,WTR_LEV_PIN);
    HAL_GPIO_DeInit(WTR_TMP_PORT,WTR_TMP_PIN);
}
/**
 * @brief Measure ADC channels and write values to DCTS
 * @param argument - none
 * @ingroup ADC
 */
void adc_task(void const * argument){
    (void)argument;
    uint16_t pwr[ADC_BUF_SIZE];
    uint16_t wtr_lev[ADC_BUF_SIZE];
    uint16_t wtr_tmp[ADC_BUF_SIZE];
    uint16_t vref[ADC_BUF_SIZE];
    float pwr_f[ADC_BUF_SIZE];
    uint8_t tick = 0;
    float temp = 0.0;
    adc_init();
    uint32_t last_wake_time = osKernelSysTick();
    while(1){
        uint32_t pwr_sum = 0;
        uint32_t wtr_lev_sum = 0;
        uint32_t wtr_tmp_sum = 0;
        uint32_t vref_sum = 0;
        float    pwr_f_sum = 0.0;


        pwr[tick] = (uint16_t)hadc1.Instance->JDR1;
        wtr_lev[tick] = (uint16_t)hadc1.Instance->JDR2;
        wtr_tmp[tick] = (uint16_t)hadc1.Instance->JDR3;
        vref[tick] = (uint16_t)hadc1.Instance->JDR4;
        //pwr_f[tick] = (float)pwr[tick]/vref[tick];

        for(uint8_t i = 0; i < ADC_BUF_SIZE; i++){
            pwr_sum += pwr[i];
            wtr_lev_sum += wtr_lev[i];
            wtr_tmp_sum += wtr_tmp[i];
            vref_sum += vref[i];
            //pwr_f_sum += pwr_f[i];
        }

        temp = (float)pwr_sum/ADC_BUF_SIZE;
        dcts.dcts_pwr = temp/ADC_MAX*ADC_VREF*PWR_K;

        dcts_meas[WTR_LVL_ADC].value = (float)wtr_lev_sum/ADC_BUF_SIZE;
        dcts_meas[WTR_LVL_V].value = dcts_meas[WTR_LVL_ADC].value*ADC_VREF/ADC_MAX;
        dcts_meas[WTR_LVL].value = dcts_meas[WTR_LVL_ADC].value*WTR_LEV_A1+WTR_LEV_A2;

        dcts_meas[WTR_TMPR_ADC].value = (float)wtr_tmp_sum/ADC_BUF_SIZE;
        dcts_meas[WTR_TMPR_V].value = dcts_meas[WTR_TMPR_ADC].value*ADC_VREF/ADC_MAX;
        temp = wtr_tmpr_coef[0];
        float temp_pow = 1.0;
        for (uint8_t i = 1; i < 6; i++){
            temp_pow *= dcts_meas[WTR_TMPR_ADC].value;
            temp += temp_pow * wtr_tmpr_coef[i];
        }
        dcts_meas[WTR_TMPR].value = temp;

        dcts_meas[VREF_ADC].value = (float)vref_sum/ADC_BUF_SIZE;
        dcts_meas[VREF_V].value = dcts_meas[VREF_V].value*ADC_VREF/ADC_MAX;

        tick++;
        if(tick >= ADC_BUF_SIZE){
            tick = 0;
        }
        osDelayUntil(&last_wake_time, ADC_PERIOD);
    }
}
