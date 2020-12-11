
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V.
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "stdlib.h"
#include "cmsis_os.h"
#include "stm32f1xx_hal_gpio.h"
#include "dcts.h"
#include "dcts_config.h"
#include "pin_map.h"
#include "buttons.h"
#include "LCD.h"
#include "adc.h"
#include "portable.h"
#include "am2302.h"
#include "menu.h"
#include "flash.h"
#include "uart.h"

/**
  * @defgroup MAIN
  */

#define FEEDER 0
#define DEFAULT_TASK_PERIOD 100

typedef enum{
    READ_FLOAT_SIGNED = 0,
    READ_FLOAT_UNSIGNED,
}read_float_bkp_sign_t;


/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
osThreadId defaultTaskHandle;
osThreadId buttonsTaskHandle;
osThreadId displayTaskHandle;
osThreadId menuTaskHandle;
osThreadId controlTaskHandle;
osThreadId adcTaskHandle;
osThreadId am2302TaskHandle;
osThreadId navigationtTaskHandle;
osThreadId uartTaskHandle;


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
//static void MX_GPIO_Init(void);
static void MX_IWDG_Init(void);
static void MX_RTC_Init(void);
//static void MX_ADC1_Init(void);
//static void MX_USART1_UART_Init(void);
static void tim2_init(void);
static void main_page_print(void);
static void main_menu_print(void);
static void error_page_print(menu_page_t page);
static void save_page_print (void);
static void info_print (void);
static void meas_channels_print(void);
static void calib_print(uint8_t start_channel);
static void save_params(void);
static void restore_params(void);
static void save_to_bkp(u8 bkp_num, u8 var);
static void save_float_to_bkp(u8 bkp_num, float var);
static u8 read_bkp(u8 bkp_num);
static float read_float_bkp(u8 bkp_num, u8 sign);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

uint32_t us_cnt_H = 0;
navigation_t navigation_style = MENU_NAVIGATION;
edit_val_t edit_val = {0};

uint16_t lvl_calib_table[6] = {
    375,
    738,
    1102,
    1466,
    1829,
    2193,
};
uint16_t tmpr_calib_table[11] = {
    3137,
    2727,
    2275,
    1826,
    1421,
    1082,
    813,
    607,
    454,
    341,
    258,
};


/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void){

    HAL_Init();
    SystemClock_Config();
    tim2_init();
    dcts_init();
    restore_params();
    /*
    MX_RTC_Init();
    MX_ADC1_Init();
    HAL_ADC_Start(&hadc1);
    HAL_ADCEx_InjectedStart(&hadc1);
    */
    /*
    osThreadDef(own_task, default_task, osPriorityNormal, 0, 364);
    defaultTaskHandle = osThreadCreate(osThread(own_task), NULL);
    */

    //osThreadDef(control_task, control_task, osPriorityNormal, 0, 364);
    //controlTaskHandle = osThreadCreate(osThread(control_task), NULL);

    osThreadDef(display_task, display_task, osPriorityNormal, 0, 256);
    displayTaskHandle = osThreadCreate(osThread(display_task), NULL);

    osThreadDef(adc_task, adc_task, osPriorityNormal, 0, 512);
    adcTaskHandle = osThreadCreate(osThread(adc_task), NULL);

    osThreadDef(buttons_task, buttons_task, osPriorityNormal, 0, 128);
    buttonsTaskHandle = osThreadCreate(osThread(buttons_task), NULL);

    osThreadDef(am2302_task, am2302_task, osPriorityNormal, 0, 128);
    am2302TaskHandle = osThreadCreate(osThread(am2302_task), NULL);

    osThreadDef(navigation_task, navigation_task, osPriorityNormal, 0, 128);
    navigationtTaskHandle = osThreadCreate(osThread(navigation_task), NULL);

    osThreadDef(uart_task, uart_task, osPriorityNormal, 0, 512);
    uartTaskHandle = osThreadCreate(osThread(uart_task), NULL);


    /* Start scheduler */
    osKernelStart();

    while (1)  {

    }

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Initializes the CPU, AHB and APB busses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
            |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
    PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Configure the Systick interrupt time
    */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* RTC init function */
static void MX_RTC_Init(void){
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};
    __HAL_RCC_BKP_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_RTC_ENABLE();
    hrtc.Instance = RTC;
    hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
    if (HAL_RTC_Init(&hrtc) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    u32 data;
    const  u32 data_c = 0x1234;
    data = BKP->DR1;
    if(data!=data_c){   // set default values
        HAL_PWR_EnableBkUpAccess();
        BKP->DR1 = data_c;
        HAL_PWR_DisableBkUpAccess();

        sTime.Hours = dcts.dcts_rtc.hour;
        sTime.Minutes = dcts.dcts_rtc.minute;
        sTime.Seconds = dcts.dcts_rtc.second;

        sDate.Date = dcts.dcts_rtc.day;
        sDate.Month = dcts.dcts_rtc.month;
        sDate.Year = (uint8_t)(dcts.dcts_rtc.year - 2000);

        HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD);
        HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

        save_float_to_bkp(2, dcts_act[0].set_value);
        save_to_bkp(3, dcts_act[0].state.control);
/*
        save_float_to_bkp(4, sensor_state.dispersion*10);
        save_float_to_bkp(5, sensor_state.hysteresis*10);
        save_float_to_bkp(6, sensor_state.correction*10);
        save_to_bkp(7, sensor_state.buff_size);

        save_float_to_bkp(8, semistor_state.max_tmpr);
*/
    }else{  // read data from bkpram
        HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
        HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

        dcts.dcts_rtc.hour = sTime.Hours;
        dcts.dcts_rtc.minute = sTime.Minutes;
        dcts.dcts_rtc.second = sTime.Seconds;

        dcts.dcts_rtc.day = sDate.Date;
        dcts.dcts_rtc.month = sDate.Month;
        dcts.dcts_rtc.year = sDate.Year + 2000;
        dcts.dcts_rtc.weekday = sDate.WeekDay;

        dcts_act[0].set_value = read_float_bkp(2, READ_FLOAT_UNSIGNED);
        dcts_act[0].state.control = read_bkp(3);
/*
        sensor_state.dispersion = read_float_bkp(4, READ_FLOAT_UNSIGNED)/10;
        sensor_state.hysteresis = read_float_bkp(5, READ_FLOAT_UNSIGNED)/10;
        sensor_state.correction = read_float_bkp(6, READ_FLOAT_SIGNED)/10;
        sensor_state.buff_size = read_bkp(7);

        semistor_state.max_tmpr = read_float_bkp(8, READ_FLOAT_UNSIGNED);
*/
    }
}


/**
 * @brief default_task
 * @param argument - None
 * @todo add group
 */
void default_task(void const * argument){

    (void)argument;
    RTC_TimeTypeDef time;
    RTC_DateTypeDef date;
    uint32_t last_wake_time = osKernelSysTick();

    //HAL_IWDG_Refresh(&hiwdg);
    while(1){
        HAL_RTC_GetDate(&hrtc,&date,RTC_FORMAT_BIN);
        HAL_RTC_GetTime(&hrtc,&time,RTC_FORMAT_BIN);

        dcts.dcts_rtc.hour = time.Hours;
        dcts.dcts_rtc.minute = time.Minutes;
        dcts.dcts_rtc.second = time.Seconds;

        dcts.dcts_rtc.day = date.Date;
        dcts.dcts_rtc.month = date.Month;
        dcts.dcts_rtc.year = date.Year + 2000;
        dcts.dcts_rtc.weekday = date.WeekDay;

        //HAL_IWDG_Refresh(&hiwdg);
        osDelayUntil(&last_wake_time, DEFAULT_TASK_PERIOD);
    }
}

/**
 * @brief display_task
 * @param argument
 */
#define display_task_period 500
void display_task(void const * argument){
    (void)argument;
    menu_init();
    LCD_init();
    uint32_t last_wake_time = osKernelSysTick();
    while(1){
        LCD_clr();
        switch (selectedMenuItem->Page){
        case MAIN_PAGE:
            main_page_print();
            break;
        case MAIN_MENU:
        case COMMON_INFO:
        case MEAS_CHANNELS:
        case LVL_CALIB:
        case TMPR_CALIB:
            main_menu_print();
            break;
        case INFO:
            info_print();
            break;
        case MEAS_CH_0:
        case MEAS_CH_1:
        case MEAS_CH_2:
        case MEAS_CH_3:
        case MEAS_CH_4:
        case MEAS_CH_5:
        case MEAS_CH_6:
        case MEAS_CH_7:
        case MEAS_CH_8:
        case MEAS_CH_9:
        case MEAS_CH_10:
        case MEAS_CH_11:
        case MEAS_CH_12:
            meas_channels_print();
            break;
        case LVL_0:
        case LVL_20:
        case LVL_40:
        case LVL_60:
        case LVL_80:
        case LVL_100:
            calib_print(LVL_0);
            break;
        case ADC_0:
        case ADC_10:
        case ADC_20:
        case ADC_30:
        case ADC_40:
        case ADC_50:
        case ADC_60:
        case ADC_70:
        case ADC_80:
        case ADC_90:
        case ADC_100:
            calib_print(ADC_0);
            break;
        case SAVE_CHANGES:
            save_page_print();
            break;
        default:
            error_page_print(selectedMenuItem->Page);
        }

        LCD_update();
        osDelayUntil(&last_wake_time, display_task_period);
    }
}

#define navigation_task_period 20
void navigation_task (void const * argument){
    (void)argument;
    uint32_t last_wake_time = osKernelSysTick();
    while(1){
        switch (navigation_style){
        case MENU_NAVIGATION:
            if((pressed_time[BUTTON_UP].pressed > 0)&&(pressed_time[BUTTON_UP].pressed < navigation_task_period)){
                menuChange(selectedMenuItem->Previous);
            }
            if((pressed_time[BUTTON_DOWN].pressed > 0)&&(pressed_time[BUTTON_DOWN].pressed < navigation_task_period)){
                menuChange(selectedMenuItem->Next);
            }
            if((pressed_time[BUTTON_LEFT].pressed > 0)&&(pressed_time[BUTTON_LEFT].pressed < navigation_task_period)){
                menuChange(selectedMenuItem->Parent);
            }
            if((pressed_time[BUTTON_RIGHT].pressed > 0)&&(pressed_time[BUTTON_RIGHT].pressed < navigation_task_period)){
                menuChange(selectedMenuItem->Child);
            }
            if((pressed_time[BUTTON_OK].pressed > 0)&&(pressed_time[BUTTON_OK].pressed < navigation_task_period)){
                menuChange(selectedMenuItem->Child);
            }
            break;
        case DIGIT_EDIT:
            if((pressed_time[BUTTON_UP].pressed > 0)&&(pressed_time[BUTTON_UP].pressed < navigation_task_period)){
                *edit_val.p_val += uint16_pow(10, (uint16_t)edit_val.digit);
            }
            if((pressed_time[BUTTON_DOWN].pressed > 0)&&(pressed_time[BUTTON_DOWN].pressed < navigation_task_period)){
                *edit_val.p_val -= uint16_pow(10, (uint16_t)edit_val.digit);
            }
            if((pressed_time[BUTTON_LEFT].pressed > 0)&&(pressed_time[BUTTON_LEFT].pressed < navigation_task_period)){
                if(edit_val.digit < edit_val.digit_max){
                    edit_val.digit++;
                }
            }
            if((pressed_time[BUTTON_RIGHT].pressed > 0)&&(pressed_time[BUTTON_RIGHT].pressed < navigation_task_period)){
                if(edit_val.digit > 0){
                    edit_val.digit--;
                }
            }
            if((pressed_time[BUTTON_OK].pressed > navigation_task_period)){
                while(pressed_time[BUTTON_OK].last_state == BUTTON_PRESSED){
                }
                navigation_style = MENU_NAVIGATION;
            }

            break;
        }
        if((pressed_time[BUTTON_BREAK].pressed > 0)&&(pressed_time[BUTTON_BREAK].pressed < navigation_task_period)){
            LCD_backlight_toggle();
        }
        if((pressed_time[BUTTON_SET].pressed > 0)&&(pressed_time[BUTTON_SET].pressed < navigation_task_period)){
            save_params();
        }
        osDelayUntil(&last_wake_time, navigation_task_period);
    }
}

static void error_page_print(menu_page_t page){
    char string[100];

    LCD_set_xy(25,45);
    sprintf(string, "СТРАНИЦА НЕ");
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    LCD_set_xy(25,35);
    sprintf(string, "НАЙДЕНА: %d", page);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);

    sprintf(string, "<назад");
    LCD_set_xy(0,0);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    LCD_invert_area(0,0,42,11);
}

static void main_page_print(void){
    char string[100];
    const float vmax = 114.0;
    uint8_t high_lev = 0;

    // print water tank
    LCD_fill_area(0,0,50,63,LCD_COLOR_BLACK);
    LCD_fill_area(1,1,49,62,LCD_COLOR_WHITE);

    // print values
    sprintf(string, "%3.1f%s", dcts_meas[WTR_TMPR].value, dcts_meas[WTR_TMPR].unit);
    LCD_set_xy(align_text_center(string, Font_7x10)-38,45);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    sprintf(string, "%3.1f%s", dcts_meas[WTR_LVL].value, dcts_meas[WTR_LVL].unit);
    LCD_set_xy(align_text_center(string, Font_7x10)-38,5);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    sprintf(string, "Горячая");
    LCD_set_xy(align_text_center(string, Font_7x10)-38,30);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    sprintf(string, "вода");
    LCD_set_xy(align_text_center(string, Font_7x10)-38,20);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);


    // fill water level
    high_lev = (uint8_t)(dcts_meas[WTR_LVL].value/vmax*62);
    if(high_lev > 61){
        high_lev = 61;
    }
    LCD_invert_area(1,1,49,high_lev+1);

    sprintf(string, "Предбанник");
    LCD_set_xy(align_text_center(string, Font_7x10)+27,52);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    LCD_invert_area(52,53,127,63);
    if(dcts_meas[PREDBANNIK_HUM].valid){
        sprintf(string, "%.1f%s/%.0f%s", dcts_meas[PREDBANNIK_TMPR].value, dcts_meas[PREDBANNIK_TMPR].unit, dcts_meas[PREDBANNIK_HUM].value, dcts_meas[PREDBANNIK_HUM].unit);
    }else{
        sprintf(string, "Нет связи");
    }
    LCD_set_xy(align_text_center(string, Font_7x10)+27,41);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);

    sprintf(string, "Моечная");
    LCD_set_xy(align_text_center(string, Font_7x10)+27,31);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    LCD_invert_area(52,32,127,42);
    if(dcts_meas[MOYKA_HUM].valid){
        sprintf(string, "%.1f%s/%.0f%s", dcts_meas[MOYKA_TMPR].value, dcts_meas[MOYKA_TMPR].unit, dcts_meas[MOYKA_HUM].value, dcts_meas[MOYKA_HUM].unit);
    }else{
        sprintf(string, "Нет связи");
    }
    LCD_set_xy(align_text_center(string, Font_7x10)+27,20);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);

    sprintf(string, "Парная");
    LCD_set_xy(align_text_center(string, Font_7x10)+27,10);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    LCD_invert_area(52,12,127,21);
    if(dcts_meas[MOYKA_HUM].valid){
        sprintf(string, "%.1f%s", dcts_meas[PARILKA_TMPR].value, dcts_meas[PARILKA_TMPR].unit);
    }else{
        sprintf(string, "Нет связи");
    }
    LCD_set_xy(align_text_center(string, Font_7x10)+27,0);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
}

static void main_menu_print (void){
    char string[100];
    menuItem* temp = selectedMenuItem->Parent;
    sprintf(string, temp->Text);
    LCD_set_xy(align_text_center(string, Font_7x10),52);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    LCD_invert_area(0,53,127,63);

    temp = selectedMenuItem->Previous;
    sprintf(string, temp->Text);
    LCD_set_xy(align_text_center(string, Font_7x10),39);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);

    sprintf(string, selectedMenuItem->Text);
    LCD_set_xy(align_text_center(string, Font_7x10),26);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    LCD_invert_area(5,26,120,38);
    LCD_invert_area(6,27,119,37);

    temp = selectedMenuItem->Next;
    sprintf(string, temp->Text);
    LCD_set_xy(align_text_center(string, Font_7x10),14);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);

    sprintf(string, "<назад      выбор>");
    LCD_set_xy(0,0);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    LCD_invert_area(0,0,42,11);
    LCD_invert_area(83,0,127,11);
}

static void info_print (void){
    char string[100];
    menuItem* temp = selectedMenuItem->Parent;
    sprintf(string, temp->Text);
    LCD_set_xy(align_text_center(string, Font_7x10),52);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    LCD_invert_area(0,53,127,63);

    sprintf(string, "Имя: %s",dcts.dcts_name_cyr);
    LCD_set_xy(2,40);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    sprintf(string, "Тип: %d",dcts.dcts_id);
    LCD_set_xy(2,30);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    sprintf(string, "Адрес: %d",dcts.dcts_address);
    LCD_set_xy(2,20);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    sprintf(string, "Версия: %s",dcts.dcts_ver);
    LCD_set_xy(2,10);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);

    sprintf(string, "<назад");
    LCD_set_xy(0,0);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    LCD_invert_area(0,0,42,11);
}

static void meas_channels_print(void){
    char string[100];
    uint8_t channel = 0;
    menuItem* temp = selectedMenuItem->Parent;
    sprintf(string, temp->Text);
    LCD_set_xy(align_text_center(string, Font_7x10),52);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    LCD_invert_area(0,53,127,63);

    temp = selectedMenuItem;
    for(uint8_t i = 0; i < 2; i++){
        channel = (uint8_t)temp->Page - MEAS_CH_0;
        sprintf(string, "%s:",dcts_meas[channel].name_cyr);
        LCD_set_xy(2,41-21*i);
        LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
        sprintf(string, "%.2f(%s) ", dcts_meas[channel].value, dcts_meas[channel].unit_cyr);
        LCD_set_xy(align_text_right(string,Font_7x10),31-21*i);
        LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
        temp = temp->Next;
    }

    sprintf(string, "<назад");
    LCD_set_xy(0,0);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    LCD_invert_area(0,0,42,11);

}

static void calib_print (uint8_t start_channel){
    char string[100];
    menuItem* temp = selectedMenuItem->Parent;
    uint16_t* calib_table;
    sprintf(string, temp->Text);
    LCD_set_xy(align_text_center(string, Font_7x10),52);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    LCD_invert_area(0,53,127,63);
    if(temp->Page == LVL_CALIB){
        calib_table = lvl_calib_table;
    }else if(temp->Page == TMPR_CALIB){
        calib_table = tmpr_calib_table;
    }

    temp = selectedMenuItem->Previous;
    sprintf(string, temp->Text);
    LCD_set_xy(1,39);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    sprintf(string, "%d",calib_table[(uint8_t)temp->Page-start_channel]);
    LCD_set_xy(align_text_right(string, Font_7x10)-1,39);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);

    sprintf(string, selectedMenuItem->Text);
    LCD_set_xy(1,26);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    sprintf(string, "%d",calib_table[(uint8_t)selectedMenuItem->Page-start_channel]);
    LCD_set_xy(align_text_right(string, Font_7x10)-1,26);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    LCD_invert_area(0,26,127,39);
    LCD_invert_area(1,27,126,38);

    temp = selectedMenuItem->Next;
    sprintf(string, temp->Text);
    LCD_set_xy(1,14);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    sprintf(string, "%d",calib_table[(uint8_t)temp->Page-start_channel]);
    LCD_set_xy(align_text_right(string, Font_7x10)-1,14);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);

    switch (navigation_style) {
    case MENU_NAVIGATION:
        sprintf(string, "<назад   изменить>");
        LCD_set_xy(0,0);
        LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
        LCD_invert_area(0,0,42,11);
        LCD_invert_area(62,0,127,11);

        if(pressed_time[BUTTON_RIGHT].pressed > navigation_task_period){
            while(pressed_time[BUTTON_RIGHT].last_state == BUTTON_PRESSED){
            }
            navigation_style = DIGIT_EDIT;
            edit_val.digit_max = 3;
            edit_val.digit = 0;
            edit_val.p_val = &calib_table[(uint8_t)selectedMenuItem->Page-start_channel];
        }
        break;
    case DIGIT_EDIT:
        sprintf(string, "*ввод");
        LCD_set_xy(align_text_center(string, Font_7x10),0);
        LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
        LCD_invert_area(46,0,82,11);

        LCD_invert_area(127-(edit_val.digit+1)*Font_7x10.FontWidth,27,126-edit_val.digit*Font_7x10.FontWidth,38);
        break;
    }

}

static void save_page_print (void){
    char string[100];

    LCD_fill_area(5,20,123,48,LCD_COLOR_BLACK);
    LCD_fill_area(6,21,122,47,LCD_COLOR_WHITE);
    sprintf(string, "СОХРАНЕНИЕ НОВЫХ");
    LCD_set_xy(align_text_center(string, Font_7x10),32);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    sprintf(string, "КОЭФФИЦИЕНТОВ");
    LCD_set_xy(align_text_center(string, Font_7x10),22);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
}


/**
 * @brief am2302_task
 * @param argument
 */

#define am2302_task_period 3000
void am2302_task (void const * argument){
    (void)argument;
    uint32_t last_wake_time = osKernelSysTick();
    am2302_init();
    am2302_data_t am2302 = {0};
    while(1){
        am2302 = am2302_get(2);
        if(am2302.error == 1){
            dcts_meas[PREDBANNIK_HUM].valid = FALSE;
            dcts_meas[PREDBANNIK_TMPR].valid = FALSE;
        }else{
            dcts_meas[PREDBANNIK_HUM].value = (float)am2302.hum/10;
            dcts_meas[PREDBANNIK_HUM].valid = TRUE;
            dcts_meas[PREDBANNIK_TMPR].value = (float)am2302.tmpr/10;
            dcts_meas[PREDBANNIK_TMPR].valid = TRUE;
        }

        am2302 = am2302_get(1);
        if(am2302.error == 1){
            dcts_meas[MOYKA_HUM].valid = FALSE;
            dcts_meas[MOYKA_TMPR].valid = FALSE;
        }else{
            dcts_meas[MOYKA_HUM].value = (float)am2302.hum/10;
            dcts_meas[MOYKA_HUM].valid = TRUE;
            dcts_meas[MOYKA_TMPR].value = (float)am2302.tmpr/10;
            dcts_meas[MOYKA_TMPR].valid = TRUE;
        }

        am2302 = am2302_get(0);
        if(am2302.error == 1){
            dcts_meas[PARILKA_TMPR].valid = FALSE;
        }else{
            dcts_meas[PARILKA_TMPR].value = (float)am2302.tmpr/10;
            dcts_meas[PARILKA_TMPR].valid = TRUE;
        }

        osDelayUntil(&last_wake_time, am2302_task_period);
    }
}

#define uart_task_period 5
void uart_task(void const * argument){
    (void)argument;
    uart_init(115200, 8, 1, PARITY_NONE, 10000);
    uint16_t tick = 0;
    char string[100];
    //memcpy(string, "Test1234567890\n", 15);
    uint32_t last_wake_time = osKernelSysTick();
    while(1){
        if((uart_2.state & UART_STATE_RECIEVE)&&\
                ((uint16_t)(us_tim_get_value() - uart_2.timeout_last) > uart_2.timeout)){
            memcpy(uart_2.buff_received, uart_2.buff_in, uart_2.in_ptr);
            uart_2.received_len = uart_2.in_ptr;
            uart_2.in_ptr = 0;
            uart_2.state &= ~UART_STATE_RECIEVE;
            uart_2.state &= ~UART_STATE_ERROR;
            uart_2.state |= UART_STATE_IN_HANDING;

            //modbus_packet_handle(uart_2.buff_received, uart_2.received_len);
            if(uart_2.state & UART_STATE_IN_HANDING){
                dcts_packet_handle(uart_2.buff_received, uart_2.received_len);
            }
        }
        if(tick == 5000/uart_task_period){
            tick = 0;
            for(uint8_t i = 0; i < MEAS_NUM; i++){
                sprintf(string, "%s:\t%.1f(%s)\n",dcts_meas[i].name,(double)dcts_meas[i].value,dcts_meas[i].unit);
                if(i == MEAS_NUM - 1){
                    strncat(string,"\n",1);
                }
                uart_send(string,(uint16_t)strlen(string));
            }
        }else{
            tick++;
        }

        osDelayUntil(&last_wake_time, uart_task_period);
    }
}

uint16_t uint16_pow(uint16_t x, uint16_t pow){
    uint16_t result = 1;
    while(pow){
        result *= x;
        pow--;
    }
    return result;
}

/**
 * @brief Init us timer
 * @ingroup MAIN
 */
static void tim2_init(void){
    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;
    __HAL_RCC_TIM2_CLK_ENABLE();
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 71;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0xFFFF;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)  {
        _Error_Handler(__FILE__, __LINE__);
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)  {
        _Error_Handler(__FILE__, __LINE__);
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)  {
        _Error_Handler(__FILE__, __LINE__);
    }
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK){
        _Error_Handler(__FILE__, __LINE__);
    }
}
/**
 * @brief Get value from global us timer
 * @return global us timer value
 * @ingroup MAIN
 */
uint32_t us_tim_get_value(void){
    uint32_t value = us_cnt_H + TIM2->CNT;
    return value;
}
/**
 * @brief Us delayy
 * @param us - delau value
 * @ingroup MAIN
 */
void us_tim_delay(uint32_t us){
    uint32_t current;
    uint8_t with_yield;
    current = TIM2->CNT;
    with_yield = 0;
    if(us > TIME_YIELD_THRESHOLD){
        with_yield =1;
    }
    while ((TIM2->CNT - current)<us){
        if(with_yield){
            osThreadYield();
        }
    }
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM1) {
        HAL_IncTick();
    }

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    while(1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

static void save_params(void){
    static menuItem* current_menu;
    current_menu = selectedMenuItem;
    menuChange(&save_changes);

    int area_cnt = find_free_area();
    if(area_cnt < 0){
        uint32_t erase_error = 0;
        FLASH_EraseInitTypeDef flash_erase = {0};
        flash_erase.TypeErase = FLASH_TYPEERASE_PAGES;
        flash_erase.NbPages = 1;
        flash_erase.PageAddress = FLASH_SAVE_PAGE_ADDRESS;
        HAL_FLASH_Unlock();
        HAL_FLASHEx_Erase(&flash_erase, &erase_error);
        HAL_FLASH_Lock();
        area_cnt = 0;
    }
    for(uint8_t i = 0; i < 6; i ++){
        save_to_flash(area_cnt, i, &lvl_calib_table[i]);
    }
    for(uint8_t i = 0; i < 11; i ++){
        save_to_flash(area_cnt, i+6, &tmpr_calib_table[i]);
    }
    osDelay(2000);
    menuChange(current_menu);
}

static void restore_params(void){
    int area_cnt = find_free_area();
    if(area_cnt != 0){
        if(area_cnt == -1){
            // page is fill, actual values in last area
            area_cnt = SAVE_AREA_NMB - 1;
        }else{
            // set last filled area number
            area_cnt--;
        }
        uint16_t *addr;
        addr = (uint32_t)(FLASH_SAVE_PAGE_ADDRESS+ area_cnt*SAVE_AREA_SIZE);
        for(uint8_t i = 0; i < 6; i++){
            lvl_calib_table[i] = *addr;
            addr++;
        }
        for(uint8_t i = 0; i < 11; i++){
            tmpr_calib_table[i] = *addr;
            addr++;
        }
    }
}

static void save_to_bkp(u8 bkp_num, u8 var){
    uint32_t data = var;
    if(bkp_num%2 == 1){
        data = data << 8;
    }
    HAL_PWR_EnableBkUpAccess();
    switch (bkp_num / 2){
    case 0:
        BKP->DR1 |= data;
        break;
    case 1:
        BKP->DR2 |= data;
        break;
    case 2:
        BKP->DR3 |= data;
        break;
    case 3:
        BKP->DR4 |= data;
        break;
    case 4:
        BKP->DR5 |= data;
        break;
    case 5:
        BKP->DR6 |= data;
        break;
    case 6:
        BKP->DR7 |= data;
        break;
    case 7:
        BKP->DR8 |= data;
        break;
    case 8:
        BKP->DR9 |= data;
        break;
    case 9:
        BKP->DR10 |= data;
        break;
    }
    HAL_PWR_DisableBkUpAccess();
}

static void save_float_to_bkp(u8 bkp_num, float var){
    char buf[5] = {0};
    sprintf(buf, "%4.0f", (double)var);
    u8 data = (u8)atoi(buf);
    save_to_bkp(bkp_num, data);
}
static u8 read_bkp(u8 bkp_num){
    uint32_t data = 0;
    switch (bkp_num/2){
    case 0:
        data = BKP->DR1;
        break;
    case 1:
        data = BKP->DR2;
        break;
    case 2:
        data = BKP->DR3;
        break;
    case 3:
        data = BKP->DR4;
        break;
    case 4:
        data = BKP->DR5;
        break;
    case 5:
        data = BKP->DR6;
        break;
    case 6:
        data = BKP->DR7;
        break;
    case 7:
        data = BKP->DR8;
        break;
    case 8:
        data = BKP->DR9;
        break;
    case 9:
        data = BKP->DR10;
        break;
    }
    if(bkp_num%2 == 1){
        data = data >> 8;
    }
    return (u8)(data & 0xFF);
}
static float read_float_bkp(u8 bkp_num, u8 sign){
    u8 data = read_bkp(bkp_num);
    char buf[5] = {0};
    if(sign == READ_FLOAT_SIGNED){
        sprintf(buf, "%d", (s8)data);
    }else{
        sprintf(buf, "%d", data);
    }
    return atoff(buf);
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

