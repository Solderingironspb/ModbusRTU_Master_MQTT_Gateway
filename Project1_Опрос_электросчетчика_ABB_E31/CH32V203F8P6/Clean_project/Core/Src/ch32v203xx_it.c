/*
 * ch32v203xx_it.c
 *
 *  Created on: Aug 17, 2023
 *      Author: Oleg Volkov
 */

#include "main.h"
#include "ch32v203xx_it.h"
#include <stdbool.h>
#include "SoftwareTimer.h"
#include "MQTT.h"

extern volatile uint32_t SysTimer_ms; //§±§Ö§â§Ö§Þ§Ö§ß§ß§Ñ§ñ, §Ñ§ß§Ñ§Ý§à§Ô§Ú§é§ß§Ñ§ñ HAL_GetTick()
extern volatile uint32_t Delay_counter_ms; //§³§é§Ö§ä§é§Ú§Ü §Õ§Ý§ñ §æ§å§ß§Ü§è§Ú§Ú Delay_ms
extern volatile uint32_t Timeout_counter_ms; //§±§Ö§â§Ö§Þ§Ö§ß§ß§Ñ§ñ §Õ§Ý§ñ §ä§Ñ§Û§Þ§Ñ§å§ä§Ñ §æ§å§ß§Ü§è§Ú§Û

extern struct USART_name husart1; //§°§Ò§ì§ñ§Ó§Ý§ñ§Ö§Þ §ã§ä§â§å§Ü§ä§å§â§å §á§à USART.(§ã§Þ. ch32v203x_RVMSIS.h)
extern struct USART_name husart2; //§°§Ò§ì§ñ§Ó§Ý§ñ§Ö§Þ §ã§ä§â§å§Ü§ä§å§â§å §á§à USART.(§ã§Þ. ch32v203x_RVMSIS.h)
extern struct USART_name husart3; //§°§Ò§ì§ñ§Ó§Ý§ñ§Ö§Þ §ã§ä§â§å§Ü§ä§å§â§å §á§à USART.(§ã§Þ. ch32v203x_RVMSIS.h)

extern uint8_t NETWORK_Status;

extern bool flag_ModbusRTU_block; //§æ§Ý§Ñ§Ô, §Ò§Ý§à§Ü§Ú§â§å§ð§ë§Ú§Û §â§Ñ§Ò§à§ä§å §Þ§Ñ§ã§ä§Ö§â§Ñ, §á§à§Ü§Ñ §ß§Ö §à§Ò§â§Ñ§Ò§à§ä§Ñ§Ö§ä§ã§ñ §Ó§ç§à§Õ§ñ§ë§Ö§Ö §ã§à§à§Ò§ë§Ö§ß§Ú§Ö

extern bool Task1;
extern uint32_t MQTT_Send_Timer; //§´§Ñ§Û§Þ§Ö§â §Õ§Ý§ñ §à§ä§á§â§Ñ§Ó§Ü§Ú MQTT

/*
 ******************************************************************************
 *  @breif §±§â§Ö§â§í§Ó§Ñ§ß§Ú§Ö §á§à §æ§Ý§Ñ§Ô§å CNTIF
 ******************************************************************************
 */

void SysTick_Handler(void) {
    SysTick->SR &= ~(1 << 0);
    SysTimer_ms++;

    if (Delay_counter_ms) {
        Delay_counter_ms--;
    }
    if (Timeout_counter_ms) {
        Timeout_counter_ms--;
    }

    ModbusRTU_Master_pool();

    ESP8266_Indication_Timer();

    if (!Software_timer_check((uint32_t*) &MQTT_Send_Timer)) {
        Task1 = true;
    }

}

/**
 ******************************************************************************
 *  @breif §±§â§Ö§â§í§Ó§Ñ§ß§Ú§Ö §á§à USART1 ESP8266
 ******************************************************************************
 */

void USART1_IRQHandler(void) {
    if (READ_BIT(USART1->STATR, USART_STATR_RXNE)) {
        //§¦§ã§Ý§Ú §á§â§Ú§ê§Ý§Ú §Õ§Ñ§ß§ß§í§Ö §á§à USART
        if (husart1.rx_counter < USART_MAX_LEN_RX_BUFFER) { //§¦§ã§Ý§Ú §Ò§Ñ§Û§ä §á§â§Ú§Ý§Ö§ä§Ö§Ý§à §Þ§Ö§ß§î§ê§Ö, §é§Ö§Þ §â§Ñ§Ù§Þ§Ö§â §Ò§å§æ§Ö§â§Ñ
            husart1.rx_buffer[husart1.rx_counter] = USART1->DATAR; //§³§é§Ú§ä§Ñ§Ö§Þ §Õ§Ñ§ß§ß§í§Ö §Ó §ã§à§à§ä§Ó§Ö§ä§ã§ä§Ó§å§ð§ë§å§ð §ñ§é§Ö§Û§Ü§å §Ó rx_buffer
            husart1.rx_counter++; //§µ§Ó§Ö§Ý§Ú§é§Ú§Þ §ã§é§Ö§ä§é§Ú§Ü §á§â§Ú§ß§ñ§ä§í§ç §Ò§Ñ§Û§ä §ß§Ñ 1
        } else {
            husart1.rx_counter = 0; //§¦§ã§Ý§Ú §Ò§à§Ý§î§ê§Ö - §ã§Ò§â§à§ã§Ú§Þ §ã§é§Ö§ä§é§Ú§Ü.
        }
    }
    if (READ_BIT(USART1->STATR, USART_STATR_IDLE)) {
        //§¦§ã§Ý§Ú §á§â§Ú§Ý§Ö§ä§Ö§Ý §æ§Ý§Ñ§Ô IDLE
        USART1->DATAR; //§³§Ò§â§à§ã§Ú§Þ §æ§Ý§Ñ§Ô IDLE
        husart1.rx_len = husart1.rx_counter; //§µ§Ù§ß§Ñ§Ö§Þ, §ã§Ü§à§Ý§î§Ü§à §Ò§Ñ§Û§ä §á§à§Ý§å§é§Ú§Ý§Ú
        husart1.rx_counter = 0; //§ã§Ò§â§à§ã§Ú§Þ §ã§é§Ö§ä§é§Ú§Ü §á§â§Ú§ç§à§Õ§ñ§ë§Ú§ç §Õ§Ñ§ß§ß§í§ç

        /*----------§©§Õ§Ö§ã§î §Ò§å§Õ§Ö§Þ §â§Ñ§Ò§à§ä§Ñ§ä§î §ã §á§â§Ú§ç§à§Õ§ñ§ë§Ú§Þ§Ú §Õ§Ñ§ß§ß§í§Þ§Ú----------*/
        UART_MQTT_Check_net(); //§±§â§à§Ó§Ö§â§Ü§Ñ §á§à§Õ§Ü§Ý§ð§é§Ö§ß§Ú§ñ §Ü Wifi §Ú MQTT §Ò§â§à§Ü§Ö§â§å
        /*----------§©§Õ§Ö§ã§î §Ò§å§Õ§Ö§Þ §â§Ñ§Ò§à§ä§Ñ§ä§î §ã §á§â§Ú§ç§à§Õ§ñ§ë§Ú§Þ§Ú §Õ§Ñ§ß§ß§í§Þ§Ú----------*/

        if (UART_MQTT_Checksumm_validation(husart1.rx_buffer)) {

            if (NETWORK_Status == MQTT_STATUS_CONNECTED) {
                MQTT_Reveieve();
            }

        }

    }
}

/**
 ******************************************************************************
 *  @breif §±§â§Ö§â§í§Ó§Ñ§ß§Ú§Ö §á§à USART2 ModbusRTU
 ******************************************************************************
 */

void USART2_IRQHandler(void) {
    if (READ_BIT(USART2->STATR, USART_STATR_RXNE)) {
        //§¦§ã§Ý§Ú §á§â§Ú§ê§Ý§Ú §Õ§Ñ§ß§ß§í§Ö §á§à USART
        if (husart2.rx_counter < USART_MAX_LEN_RX_BUFFER) { //§¦§ã§Ý§Ú §Ò§Ñ§Û§ä §á§â§Ú§Ý§Ö§ä§Ö§Ý§à §Þ§Ö§ß§î§ê§Ö, §é§Ö§Þ §â§Ñ§Ù§Þ§Ö§â §Ò§å§æ§Ö§â§Ñ
            husart2.rx_buffer[husart2.rx_counter] = USART2->DATAR; //§³§é§Ú§ä§Ñ§Ö§Þ §Õ§Ñ§ß§ß§í§Ö §Ó §ã§à§à§ä§Ó§Ö§ä§ã§ä§Ó§å§ð§ë§å§ð §ñ§é§Ö§Û§Ü§å §Ó rx_buffer
            husart2.rx_counter++; //§µ§Ó§Ö§Ý§Ú§é§Ú§Þ §ã§é§Ö§ä§é§Ú§Ü §á§â§Ú§ß§ñ§ä§í§ç §Ò§Ñ§Û§ä §ß§Ñ 1
        } else {
            husart2.rx_counter = 0; //§¦§ã§Ý§Ú §Ò§à§Ý§î§ê§Ö - §ã§Ò§â§à§ã§Ú§Þ §ã§é§Ö§ä§é§Ú§Ü.
        }
    }
    TIM3->CNT = 0;
    SET_BIT(TIM3->CTLR1, TIM_CEN); //§©§Ñ§á§å§ã§Ü §ä§Ñ§Û§Þ§Ö§â§Ñ
}

/**
 ******************************************************************************
 *  @breif §±§â§Ö§â§í§Ó§Ñ§ß§Ú§Ö §á§à USART3
 ******************************************************************************
 */

void USART3_IRQHandler(void) {
    if (READ_BIT(USART3->STATR, USART_STATR_RXNE)) {
        //§¦§ã§Ý§Ú §á§â§Ú§ê§Ý§Ú §Õ§Ñ§ß§ß§í§Ö §á§à USART
        if (husart3.rx_counter < USART_MAX_LEN_RX_BUFFER) { //§¦§ã§Ý§Ú §Ò§Ñ§Û§ä §á§â§Ú§Ý§Ö§ä§Ö§Ý§à §Þ§Ö§ß§î§ê§Ö, §é§Ö§Þ §â§Ñ§Ù§Þ§Ö§â §Ò§å§æ§Ö§â§Ñ
            husart3.rx_buffer[husart3.rx_counter] = USART3->DATAR; //§³§é§Ú§ä§Ñ§Ö§Þ §Õ§Ñ§ß§ß§í§Ö §Ó §ã§à§à§ä§Ó§Ö§ä§ã§ä§Ó§å§ð§ë§å§ð §ñ§é§Ö§Û§Ü§å §Ó rx_buffer
            husart3.rx_counter++; //§µ§Ó§Ö§Ý§Ú§é§Ú§Þ §ã§é§Ö§ä§é§Ú§Ü §á§â§Ú§ß§ñ§ä§í§ç §Ò§Ñ§Û§ä §ß§Ñ 1
        } else {
            husart3.rx_counter = 0; //§¦§ã§Ý§Ú §Ò§à§Ý§î§ê§Ö - §ã§Ò§â§à§ã§Ú§Þ §ã§é§Ö§ä§é§Ú§Ü.
        }
    }
    if (READ_BIT(USART3->STATR, USART_STATR_IDLE)) {
        //§¦§ã§Ý§Ú §á§â§Ú§Ý§Ö§ä§Ö§Ý §æ§Ý§Ñ§Ô IDLE
        USART3->DATAR; //§³§Ò§â§à§ã§Ú§Þ §æ§Ý§Ñ§Ô IDLE
        husart3.rx_len = husart3.rx_counter; //§µ§Ù§ß§Ñ§Ö§Þ, §ã§Ü§à§Ý§î§Ü§à §Ò§Ñ§Û§ä §á§à§Ý§å§é§Ú§Ý§Ú
        husart3.rx_counter = 0; //§ã§Ò§â§à§ã§Ú§Þ §ã§é§Ö§ä§é§Ú§Ü §á§â§Ú§ç§à§Õ§ñ§ë§Ú§ç §Õ§Ñ§ß§ß§í§ç
    }
}

/**
 ******************************************************************************
 *  @breif §±§â§Ö§â§í§Ó§Ñ§ß§Ú§Ö §á§à TIM3
 ******************************************************************************
 */

void TIM3_IRQHandler(void) {
    if (READ_BIT(TIM3->INTFR, TIM_UIF)) {
        CLEAR_BIT(TIM3->INTFR, TIM_UIF); //§³§Ò§â§à§ã§Ú§Þ §æ§Ý§Ñ§Ô §á§â§Ö§â§í§Ó§Ñ§ß§Ú§ñ
        //§¦§ã§Ý§Ú §á§â§Ú§Ý§Ö§ä§Ö§Ý §æ§Ý§Ñ§Ô IDLE
        USART2->DATAR; //§³§Ò§â§à§ã§Ú§Þ §æ§Ý§Ñ§Ô IDLE
        husart2.rx_len = husart2.rx_counter; //§µ§Ù§ß§Ñ§Ö§Þ, §ã§Ü§à§Ý§î§Ü§à §Ò§Ñ§Û§ä §á§à§Ý§å§é§Ú§Ý§Ú
        husart2.rx_counter = 0; //§ã§Ò§â§à§ã§Ú§Þ §ã§é§Ö§ä§é§Ú§Ü §á§â§Ú§ç§à§Õ§ñ§ë§Ú§ç §Õ§Ñ§ß§ß§í§ç
        flag_ModbusRTU_block = true; //§á§à§ã§ä§Ñ§Ó§Ú§Þ §Ò§Ý§à§Ü §ß§Ñ §Õ§Ñ§Ý§î§ß§Ö§Û§ê§å§ð §â§Ñ§Ò§à§ä§å §Þ§Ñ§ã§ä§Ö§â§Ñ, §á§à§Ü§Ñ §ß§Ö §à§Ò§â§Ñ§Ò§à§ä§Ñ§Ö§Þ §Ó§ç§à§Õ§ñ§ë§Ú§Ö §Õ§Ñ§ß§ß§í§Ö
    }
}

/**
 ******************************************************************************
 *  @breif §±§â§Ö§â§í§Ó§Ñ§ß§Ú§Ö §á§à TIM1
 ******************************************************************************
 */
void TIM1_UP_IRQHandler(void) {
    if (READ_BIT(TIM1->INTFR, TIM_UIF)) {
        CLEAR_BIT(TIM1->INTFR, TIM_UIF); //§³§Ò§â§à§ã§Ú§Þ §æ§Ý§Ñ§Ô §á§â§Ö§â§í§Ó§Ñ§ß§Ú§ñ
    }
}
