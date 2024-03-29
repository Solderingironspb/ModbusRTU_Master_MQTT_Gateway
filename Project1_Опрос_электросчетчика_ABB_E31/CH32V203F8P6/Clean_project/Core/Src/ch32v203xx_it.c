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

extern volatile uint32_t SysTimer_ms; //���֧�֧ާ֧ߧߧѧ�, �ѧߧѧݧ�ԧڧ�ߧѧ� HAL_GetTick()
extern volatile uint32_t Delay_counter_ms; //����֧��ڧ� �էݧ� ���ߧܧ�ڧ� Delay_ms
extern volatile uint32_t Timeout_counter_ms; //���֧�֧ާ֧ߧߧѧ� �էݧ� ��ѧۧާѧ��� ���ߧܧ�ڧ�

extern struct USART_name husart1; //���ҧ��ӧݧ�֧� �����ܧ���� ��� USART.(���. ch32v203x_RVMSIS.h)
extern struct USART_name husart2; //���ҧ��ӧݧ�֧� �����ܧ���� ��� USART.(���. ch32v203x_RVMSIS.h)
extern struct USART_name husart3; //���ҧ��ӧݧ�֧� �����ܧ���� ��� USART.(���. ch32v203x_RVMSIS.h)

extern uint8_t NETWORK_Status;

extern bool flag_ModbusRTU_block; //��ݧѧ�, �ҧݧ�ܧڧ����ڧ� ��ѧҧ��� �ާѧ��֧��, ���ܧ� �ߧ� ��ҧ�ѧҧ��ѧ֧��� �ӧ��է��֧� ����ҧ�֧ߧڧ�

extern bool Task1;
extern uint32_t MQTT_Send_Timer; //���ѧۧާ֧� �էݧ� �����ѧӧܧ� MQTT

/*
 ******************************************************************************
 *  @breif ����֧���ӧѧߧڧ� ��� ��ݧѧԧ� CNTIF
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
 *  @breif ����֧���ӧѧߧڧ� ��� USART1 ESP8266
 ******************************************************************************
 */

void USART1_IRQHandler(void) {
    if (READ_BIT(USART1->STATR, USART_STATR_RXNE)) {
        //����ݧ� ���ڧ�ݧ� �էѧߧߧ��� ��� USART
        if (husart1.rx_counter < USART_MAX_LEN_RX_BUFFER) { //����ݧ� �ҧѧۧ� ���ڧݧ֧�֧ݧ� �ާ֧ߧ���, ��֧� ��ѧ٧ާ֧� �ҧ��֧��
            husart1.rx_buffer[husart1.rx_counter] = USART1->DATAR; //����ڧ�ѧ֧� �էѧߧߧ��� �� �����ӧ֧���ӧ����� ���֧ۧܧ� �� rx_buffer
            husart1.rx_counter++; //���ӧ֧ݧڧ�ڧ� ���֧��ڧ� ���ڧߧ����� �ҧѧۧ� �ߧ� 1
        } else {
            husart1.rx_counter = 0; //����ݧ� �ҧ�ݧ��� - ��ҧ���ڧ� ���֧��ڧ�.
        }
    }
    if (READ_BIT(USART1->STATR, USART_STATR_IDLE)) {
        //����ݧ� ���ڧݧ֧�֧� ��ݧѧ� IDLE
        USART1->DATAR; //���ҧ���ڧ� ��ݧѧ� IDLE
        husart1.rx_len = husart1.rx_counter; //���٧ߧѧ֧�, ��ܧ�ݧ�ܧ� �ҧѧۧ� ���ݧ��ڧݧ�
        husart1.rx_counter = 0; //��ҧ���ڧ� ���֧��ڧ� ���ڧ��է��ڧ� �էѧߧߧ���

        /*----------���է֧�� �ҧ�է֧� ��ѧҧ��ѧ�� �� ���ڧ��է��ڧާ� �էѧߧߧ��ާ�----------*/
        UART_MQTT_Check_net(); //�����ӧ֧�ܧ� ���էܧݧ��֧ߧڧ� �� Wifi �� MQTT �ҧ��ܧ֧��
        /*----------���է֧�� �ҧ�է֧� ��ѧҧ��ѧ�� �� ���ڧ��է��ڧާ� �էѧߧߧ��ާ�----------*/

        if (UART_MQTT_Checksumm_validation(husart1.rx_buffer)) {

            if (NETWORK_Status == MQTT_STATUS_CONNECTED) {
                MQTT_Reveieve();
            }

        }

    }
}

/**
 ******************************************************************************
 *  @breif ����֧���ӧѧߧڧ� ��� USART2 ModbusRTU
 ******************************************************************************
 */

void USART2_IRQHandler(void) {
    if (READ_BIT(USART2->STATR, USART_STATR_RXNE)) {
        //����ݧ� ���ڧ�ݧ� �էѧߧߧ��� ��� USART
        if (husart2.rx_counter < USART_MAX_LEN_RX_BUFFER) { //����ݧ� �ҧѧۧ� ���ڧݧ֧�֧ݧ� �ާ֧ߧ���, ��֧� ��ѧ٧ާ֧� �ҧ��֧��
            husart2.rx_buffer[husart2.rx_counter] = USART2->DATAR; //����ڧ�ѧ֧� �էѧߧߧ��� �� �����ӧ֧���ӧ����� ���֧ۧܧ� �� rx_buffer
            husart2.rx_counter++; //���ӧ֧ݧڧ�ڧ� ���֧��ڧ� ���ڧߧ����� �ҧѧۧ� �ߧ� 1
        } else {
            husart2.rx_counter = 0; //����ݧ� �ҧ�ݧ��� - ��ҧ���ڧ� ���֧��ڧ�.
        }
    }
    TIM3->CNT = 0;
    SET_BIT(TIM3->CTLR1, TIM_CEN); //���ѧ���� ��ѧۧާ֧��
}

/**
 ******************************************************************************
 *  @breif ����֧���ӧѧߧڧ� ��� USART3
 ******************************************************************************
 */

void USART3_IRQHandler(void) {
    if (READ_BIT(USART3->STATR, USART_STATR_RXNE)) {
        //����ݧ� ���ڧ�ݧ� �էѧߧߧ��� ��� USART
        if (husart3.rx_counter < USART_MAX_LEN_RX_BUFFER) { //����ݧ� �ҧѧۧ� ���ڧݧ֧�֧ݧ� �ާ֧ߧ���, ��֧� ��ѧ٧ާ֧� �ҧ��֧��
            husart3.rx_buffer[husart3.rx_counter] = USART3->DATAR; //����ڧ�ѧ֧� �էѧߧߧ��� �� �����ӧ֧���ӧ����� ���֧ۧܧ� �� rx_buffer
            husart3.rx_counter++; //���ӧ֧ݧڧ�ڧ� ���֧��ڧ� ���ڧߧ����� �ҧѧۧ� �ߧ� 1
        } else {
            husart3.rx_counter = 0; //����ݧ� �ҧ�ݧ��� - ��ҧ���ڧ� ���֧��ڧ�.
        }
    }
    if (READ_BIT(USART3->STATR, USART_STATR_IDLE)) {
        //����ݧ� ���ڧݧ֧�֧� ��ݧѧ� IDLE
        USART3->DATAR; //���ҧ���ڧ� ��ݧѧ� IDLE
        husart3.rx_len = husart3.rx_counter; //���٧ߧѧ֧�, ��ܧ�ݧ�ܧ� �ҧѧۧ� ���ݧ��ڧݧ�
        husart3.rx_counter = 0; //��ҧ���ڧ� ���֧��ڧ� ���ڧ��է��ڧ� �էѧߧߧ���
    }
}

/**
 ******************************************************************************
 *  @breif ����֧���ӧѧߧڧ� ��� TIM3
 ******************************************************************************
 */

void TIM3_IRQHandler(void) {
    if (READ_BIT(TIM3->INTFR, TIM_UIF)) {
        CLEAR_BIT(TIM3->INTFR, TIM_UIF); //���ҧ���ڧ� ��ݧѧ� ���֧���ӧѧߧڧ�
        //����ݧ� ���ڧݧ֧�֧� ��ݧѧ� IDLE
        USART2->DATAR; //���ҧ���ڧ� ��ݧѧ� IDLE
        husart2.rx_len = husart2.rx_counter; //���٧ߧѧ֧�, ��ܧ�ݧ�ܧ� �ҧѧۧ� ���ݧ��ڧݧ�
        husart2.rx_counter = 0; //��ҧ���ڧ� ���֧��ڧ� ���ڧ��է��ڧ� �էѧߧߧ���
        flag_ModbusRTU_block = true; //�����ѧӧڧ� �ҧݧ�� �ߧ� �էѧݧ�ߧ֧ۧ��� ��ѧҧ��� �ާѧ��֧��, ���ܧ� �ߧ� ��ҧ�ѧҧ��ѧ֧� �ӧ��է��ڧ� �էѧߧߧ���
    }
}

/**
 ******************************************************************************
 *  @breif ����֧���ӧѧߧڧ� ��� TIM1
 ******************************************************************************
 */
void TIM1_UP_IRQHandler(void) {
    if (READ_BIT(TIM1->INTFR, TIM_UIF)) {
        CLEAR_BIT(TIM1->INTFR, TIM_UIF); //���ҧ���ڧ� ��ݧѧ� ���֧���ӧѧߧڧ�
    }
}
