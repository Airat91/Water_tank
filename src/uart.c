#include "uart.h"
#include "main.h"
#include "pin_map.h"
#include "dcts.h"
#include "stm32f1xx_hal_uart.h"
#include "stm32f1xx_hal_usart.h"
#include "stm32f1xx_hal_rcc.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal.h"

/**
  * @defgroup uart
  * @brief work with uart for cross-device communication
  */


/*========= GLOBAL VARIABLES ==========*/

uint8_t uart_buff_out[UART_BUFF_MAX_LEN];
uint8_t uart_buff_received[UART_BUFF_MAX_LEN];
uint8_t uart_buff_in[UART_BUFF_MAX_LEN];
UART_HandleTypeDef huart2;
uart_stream_t rs_485 = {
    .instance = huart2,
    .out_len = 0,
    .out_ptr = 0,
    .in_len = 0,
    .in_ptr = 0,
    .received_len = 0,
    .max_len = UART_BUFF_MAX_LEN,
    .err_cnt = 0,
    .timeout = 0,
    .timeout_cnt = 0,
    .state = UART_STATE_ERASE,
    .buff_out = uart_buff_out,
    .buff_in = uart_buff_in,
    .buff_received = uart_buff_received,
};

/*========== FUNCTIONS ==========*/

/**
 * @brief Init UART
 * @param bit_rate
 * @param word_len
 * @param stop_bit_number
 * @param parity
 * @param rx_delay
 * @return  0 - UART init successfull,\n
 *          -1 - UART init error,\n
 *          -2 - word_len error,\n
 *          -3 - stop+bit_number error,\n
 *          -4 - parity error
 * @ingroup uart
 *
 * Init UART's GPIOs, config UART params and eanble UART_IRQ
 */
int uart_init(uint32_t bit_rate,uint8_t word_len,uint8_t stop_bit_number,parity_t parity,uint16_t rx_delay){
    int result = 0;
    uart_gpio_init();

    rs_485.timeout = rx_delay;

    huart2.Instance = USART2;
    huart2.Init.BaudRate = bit_rate;
    switch (word_len) {
    case 8:
        huart2.Init.WordLength = UART_WORDLENGTH_8B;
        break;
    case 9:
        huart2.Init.WordLength = UART_WORDLENGTH_9B;
        break;
    default:
        huart2.Init.WordLength = UART_WORDLENGTH_8B;
        result = -2;
    }
    switch (stop_bit_number) {
    case 1:
        huart2.Init.StopBits = UART_STOPBITS_1;
        break;
    case 2:
        huart2.Init.StopBits = UART_STOPBITS_2;
        break;
    default:
        huart2.Init.StopBits = UART_STOPBITS_1;
        result = -3;
    }
    switch (parity) {
    case PARITY_NONE:
        huart2.Init.Parity = UART_PARITY_NONE;
        break;
    case PARITY_EVEN:
        huart2.Init.Parity = UART_PARITY_EVEN;
        break;
    case PARITY_ODD:
        huart2.Init.Parity = UART_PARITY_ODD;
        break;
    default:
        huart2.Init.Parity = UART_PARITY_NONE;
        result = -4;
    }
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_RTS;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
      result = -1;
    }

    __HAL_RCC_USART2_CLK_ENABLE();
    NVIC_ClearPendingIRQ(USART2_IRQn);
    HAL_NVIC_EnableIRQ(USART2_IRQn);

    return result;
}

void uart_deinit(){

}

/**
 * @brief Init UART's GPIOs
 * @ingroup uart
 */
void uart_gpio_init(void){
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pull = GPIO_PULLUP;

    GPIO_InitStruct.Pin = RS_485_DE_PIN;
    HAL_GPIO_Init(RS_485_DE_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = RS_485_TX_PIN;
    HAL_GPIO_Init(RS_485_TX_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Mode = GPIO_MODE_AF_INPUT;
    GPIO_InitStruct.Pin = RS_485_RX_PIN;
    HAL_GPIO_Init(RS_485_RX_PORT, &GPIO_InitStruct);
}

/**
 * @brief Deinit UART's GPIOs
 * @ingroup uart
 */
void uart_gpio_deinit(void){
    HAL_GPIO_DeInit(RS_485_DE_PORT,RS_485_DE_PIN);
    HAL_GPIO_DeInit(RS_485_RX_PORT,RS_485_RX_PIN);
    HAL_GPIO_DeInit(RS_485_TX_PORT,RS_485_TX_PIN);
}

int uart_send(uint8_t *buff, uint8_t len){

}

int uart_handle(void){

    uint32_t dd;
    uint32_t status;
    static uint8_t state_slip;
    dd=0;
    status = huart2.Instance->SR;
    // receive mode
    if(status & USART_SR_RXNE){
        huart2.Instance->SR &= ~(USART_SR_RXNE);
        dd=huart2.Instance->DR;
        if(rs_485.self.in_type == UART_IN_TYPE_SLIP){
            if(uart_hand[port].in_ptr < uart_hand[port].max_len &&\
                    ((uart_hand[port].in_ptr!=0) || (dd==SLIP_END)||(state_slip[port] & BIT(SLIP_RECV_START)))){
                if (state_slip[port] & BIT(SLIP_RECV_ESCAPE)) {
                    switch (dd) {
                    case SLIP_ESC_END:
                        uart_hand[port].buff_in[uart_hand[port].in_ptr++]=SLIP_END;
                        break;
                    case SLIP_ESC_ESC:
                        uart_hand[port].buff_in[uart_hand[port].in_ptr++]=SLIP_ESC;
                        break;
                    }
                    state_slip[port] &= ~BIT(SLIP_RECV_ESCAPE);
                }else{
                    switch (dd) {
                    case SLIP_END:
                        if (uart_hand[port].in_ptr > 0) {
                            if(uart_hand[port].state & CHANNEL_STATE_IN_HANDING){
                                uart_hand[port].state |= CHANNEL_STATE_ERROR;
                            }else{
                                for(u16 i=0;i<uart_hand[port].in_ptr;i++){
                                    uart_hand[port].buff_received[i]=uart_hand[port].buff_in[i];
                                }
                                uart_hand[port].in_len=uart_hand[port].in_ptr;
                                uart_hand[port].state |= CHANNEL_STATE_IN_HANDING;
                            }
                            uart_hand[port].in_ptr=0;
                            state_slip[port] &= ~BIT(SLIP_RECV_START);
                        }else{
                            state_slip[port] |= BIT(SLIP_RECV_START);
                        }
                        break;
                    case SLIP_ESC:
                        if (state_slip[port] & BIT(SLIP_RECV_START)){
                            state_slip[port] |= BIT(SLIP_RECV_ESCAPE);
                        }
                        break;
                    default:
                        if (state_slip[port] & BIT(SLIP_RECV_START)){
                            uart_hand[port].buff_in[uart_hand[port].in_ptr++]=(u8)dd;
                        }
                    }
                }
            }else{
                uart_hand[port].in_ptr = 0;
                state_slip[port] = 0;
            }
        }else{
            uart_hand[port].self.last_time = osKernelSysTick();
#if DEBUG
            rx_not_empty_count[port]++;
#endif
            if(uart_hand[port].in_ptr < uart_hand[port].max_len &&\
                    ((uart_hand[port].in_ptr!=0) || (dd!=0))){
                uart_hand[port].buff_in[uart_hand[port].in_ptr++]=(u8)dd;
                uart_hand[port].in_ptr++;
            }
        }
        // check for errors
        if (status & USART_SR_PE){// Parity error
            uart_hand[port].state |= CHANNEL_STATE_ERROR;
            uart_hand[port].par_err_cnt++;
        } else if (status & USART_SR_FE){//frame error
            uart_hand[port].state |= CHANNEL_STATE_ERROR;
            uart_hand[port].frame_err_cnt++;
        } else if (status & USART_SR_NE){//noise detected
            uart_hand[port].state |= CHANNEL_STATE_ERROR;
            uart_hand[port].noise_err_cnt++;
        }
    }
    if((status & USART_SR_TXE) ){
        // transmit mode
        if(!(uart_hand[port].state & CHANNEL_STATE_IS_LAST_BYTE)){
            uart_hand[port].out_ptr = uart_hand[port].out_ptr<uart_hand[port].max_len?\
                        uart_hand[port].out_ptr:uart_hand[port].max_len-1;
            if(uart_hand[port].out_ptr == uart_hand[port].out_len-1){
                uart_base[port]->CR1 &= ~USART_CR1_TXEIE;
                uart_base[port]->CR1 |= USART_CR1_TCIE;
                uart_base[port]->SR &=~USART_SR_TC;
                uart_hand[port].state |= CHANNEL_STATE_IS_LAST_BYTE;
                uart_base[port]->DR=uart_hand[port].buff_out[uart_hand[port].out_ptr];
            }else {
                uart_base[port]->DR=uart_hand[port].buff_out[uart_hand[port].out_ptr];
            }
#if DEBUG
            tx_empty_count[port]++;
#endif
            uart_hand[port].out_ptr++;
        }else if(status & USART_SR_TC){
            // end of transmit
            uart_base[port]->CR1 &= ~USART_CR1_TXEIE;
            uart_base[port]->CR1 &= ~USART_CR1_TCIE;
            uart_base[port]->SR &=~USART_SR_TC;
            uart_hand[port].state |= CHANNEL_STATE_SENDED;
            uart_hand[port].state &=~CHANNEL_STATE_SENDING;
            uart_hand[port].state &=~CHANNEL_STATE_IS_LAST_BYTE;
#if DEBUG
            tx_complite_count[port]++;
#endif
            uart_hand[port].in_ptr = 0;
            uart_base[port]->CR1 |= USART_CR1_RXNEIE;   // ready to input messages
        }
    }
    // overrun error without RXNE flag
    if(status & USART_SR_ORE){
        uart_hand[port].state |= CHANNEL_STATE_ERROR;
        uart_hand[port].ovrrun_err_cnt++;
        dd=USART1->SR;
        dd=USART1->DR;
    }
}
