/* CH32V203F8P6 ModbusRTU Master
 * ESP8266 UART to MQTT transmitter
 * Debug:
 * PB6 - SWDIO
 * PB7 - SWCLK
 * USART2 - ModbusRTU(RS485):
 * PA1 - RO/DI
 * PA2 - Tx2
 * PA3 - Rx2
 * USART1 - ESP8266
 * PA9 - Tx1
 * PA10 -Rx1
 * */

#include "main.h"
#include "stdio.h"
#include <stdbool.h>
#include "ESP8266_lib.h"
#include "MQTT.h"
#include "SoftwareTimer.h"

extern struct USART_name husart1; //Объявляем структуру по USART.(см. ch32v203x_RVMSIS.h)
extern uint8_t NETWORK_Status;
uint8_t MQTT_Arr[64] = { 0, };

extern float ABB_E31_Total_active_energy;       //01
extern float ABB_E31_Active_energy_1;           //02
extern float ABB_E31_Active_energy_2;           //03
extern float ABB_E31_Active_energy_3;           //04
extern float ABB_E31_Active_energy_4;           //05
extern float ABB_E31_Voltage;                      //06
extern float ABB_E31_Amp;                          //07
extern float ABB_E31_Active_power;                 //08
extern float ABB_E31_Power_factor;                 //09
extern float ABB_E31_Frequency;                    //10
extern int16_t ABB_E31_Temperature;                //11

extern bool flags_tx_data_ready[64];
//extern bool flags_read_data_block[64];

/*====================================Добавить в main.c====================================*/
uint32_t ModbusRTU_Timeout = 0; //Таймаут для ответов
uint32_t ModbusRTU_delay_between_pool = 0; //Задержка между фреймами
volatile uint8_t ModbusRTU_Counter_in_queue = 0; //Счетчик запросов в отчереди на отправку слейв устройствам. Номер счетчика соответствует отправленному запросу.
bool flag_ModbusRTU_request_on = true; //флаг, разрешающий отправку пакета
bool flag_ModbusRTU_block = false; //флаг, блокирующий работу мастера, пока не обработается входящее сообщение
/*====================================Добавить в main.c====================================*/

bool Task1 = false;
uint32_t MQTT_Send_Timer; //Таймер для отправки MQTT

void Task1_MQTT_Send(void) {
    if (NETWORK_Status == MQTT_STATUS_CONNECTED) {
        if (flags_tx_data_ready[0]) {
            UART_MQTT_Send_data_float(1, ABB_E31_Total_active_energy, MQTT_Arr);
            UART_MQTT_Send_data_float(2, ABB_E31_Active_energy_1, MQTT_Arr);
            UART_MQTT_Send_data_float(3, ABB_E31_Active_energy_2, MQTT_Arr);
            UART_MQTT_Send_data_float(4, ABB_E31_Active_energy_3, MQTT_Arr);
            UART_MQTT_Send_data_float(5, ABB_E31_Active_energy_4, MQTT_Arr);
            UART_MQTT_Send_data_float(6, ABB_E31_Voltage, MQTT_Arr);
            UART_MQTT_Send_data_float(7, ABB_E31_Amp, MQTT_Arr);
            UART_MQTT_Send_data_float(8, ABB_E31_Active_power, MQTT_Arr);
            UART_MQTT_Send_data_float(9, ABB_E31_Power_factor, MQTT_Arr);
            UART_MQTT_Send_data_float(10, ABB_E31_Frequency, MQTT_Arr);
            flags_tx_data_ready[0] = false;
        }
        if (flags_tx_data_ready[1]){
            UART_MQTT_Send_data_int16_t(11, ABB_E31_Temperature, MQTT_Arr);
            flags_tx_data_ready[1] = false;
        }
    }
}

void MQTT_Reveieve(void) {
    switch (husart1.rx_buffer[1]) {
//    case 18:
//        OWEN_PR200_Out3 = UART_MQTT_Receive_data_int16_t(husart1.rx_buffer);
//        flags_read_data_block[4] = true;
//        break;
//    case 19:
//        OWEN_PR200_Out4 = UART_MQTT_Receive_data_int16_t(husart1.rx_buffer);
//        flags_read_data_block[5] = true;
//        break;
    }
}

int main(void) {
    RVMSIS_Debug_init(); //Настройка дебага
    RVMSIS_RCC_SystemClock_144MHz(); //Настройка системной частоты.
    RVMSIS_SysTick_Timer_init(); //Настройка системного таймера на 1000 Гц.
    RVMSIS_USART2_Init(); //Настройка USART2 ModbusRTU 9600 8N1
    RVMSIS_USART1_Init(); //Настройка USART1 9600 8N1
    RVMSIS_GPIO_init(GPIOA, 1, GPIO_GENERAL_PURPOSE_OUTPUT, GPIO_OUTPUT_PUSH_PULL, GPIO_Speed_50MHz); //Настройка ножки RO/DI
    MODBUSRTU_RECEIEVE;
    RVMSIS_GPIO_init(GPIOA, 8, GPIO_GENERAL_PURPOSE_OUTPUT, GPIO_OUTPUT_PUSH_PULL, GPIO_Speed_50MHz); //ножка RESET ESP8266
    RVMSIS_GPIO_init(GPIOB, 15, GPIO_GENERAL_PURPOSE_OUTPUT, GPIO_OUTPUT_PUSH_PULL, GPIO_Speed_50MHz); //Wifi status
    RVMSIS_TIM3_init();
    ESP8266_Reset(200);

    Delay_ms(5); //Перед началой основного цикла сделаем задержку, чтоб не лез всякий мусор.

    while(1) {

        /*Выполнение работы ModbusRTU*/
        ModbusRTU_Master_pool_run();
        /*Выполнение работы ModbusRTU*/

        /*Работа индикации подключения к WIFI и MQTT*/
        ESP8266_Indication_run();
        /*Работа индикации подключения к WIFI и MQTT*/

        /*Отправка данных по MQTT*/
        if (Task1) {
            Task1_MQTT_Send();
            Task1 = false;
            Software_timer((uint32_t*) &MQTT_Send_Timer, 2000);
        }
        /*Отправка данных по MQTT*/

    }
}

