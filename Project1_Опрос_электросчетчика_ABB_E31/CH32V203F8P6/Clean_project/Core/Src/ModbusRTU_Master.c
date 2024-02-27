/*
 * ModbusRTU_Master.c
 *
 *  Created on: Jan 31, 2024
 *      Author: Solderingiron
 */

#include "main.h"
#include "ModbusRTU.h"
#include <string.h>

uint8_t Array[255] = { 0, };
uint8_t tx_buffer_size = 0;
extern struct USART_name husart2; //Объявляем структуру по USART.(см. ch32v203x_RVMSIS.h)

extern uint32_t ModbusRTU_Timeout; //Таймаут для ответов
extern uint32_t ModbusRTU_delay_between_pool; //Задержка между фреймами
extern volatile uint8_t ModbusRTU_Counter_in_queue; //Счетчик запросов в отчереди на отправку слейв устройствам. Номер счетчика соответствует отправленному запросу.
extern bool flag_ModbusRTU_request_on; //флаг, разрешающий отправку пакета
extern bool flag_ModbusRTU_block; //флаг, блокирующий работу мастера, пока не обработается входящее сообщение

float ABB_E31_Total_active_energy = 0.0f;       //01
float ABB_E31_Active_energy_1 = 0.0f;           //02
float ABB_E31_Active_energy_2 = 0.0f;           //03
float ABB_E31_Active_energy_3 = 0.0f;           //04
float ABB_E31_Active_energy_4 = 0.0f;           //05
float ABB_E31_Voltage = 0.0f;                   //06
float ABB_E31_Amp = 0.0f;                       //07
float ABB_E31_Active_power = 0.0f;              //08
float ABB_E31_Power_factor = 0.0f;              //09
float ABB_E31_Frequency = 0.0f;                 //10
int16_t ABB_E31_Temperature = 0;                //11

bool flags_tx_data_ready[64] = { 0, };
bool flags_read_data_block[64] = { 0, };

/**
 ***************************************************************************************
 *  @breif Очередь запросов от мастера к слев устройствам
 *  @param ModbusRTU_Counter_in_queue - номер запроса в очереди
 ***************************************************************************************
 */
void ModbusRTU_request_queue(uint8_t ModbusRTU_Counter_in_queue) {
    switch (ModbusRTU_Counter_in_queue) {
        case 0:
            ModbusRTU_Read_Holding_Registers_0x03(247, 20480, 15, BYTE_ORDER_CD_AB);
            break;
        case 1:
            ModbusRTU_Read_Holding_Registers_0x03(247, 20525, 1, BYTE_ORDER_CD_AB);
            break;
        case 2:

            break;
        case 3:

            break;
        case 4:

            break;
        case 5:

            break;
        case 6:

            break;
    }

}

/**
 ***************************************************************************************
 *  @breif Очередь обработки ответов от слейв устройств
 *  @param ModbusRTU_Counter_in_queue - номер запроса в очереди, на который пришел ответ
 ***************************************************************************************
 */
void ModbusRTU_response_handler(uint8_t ModbusRTU_Counter_in_queue) {
    switch (ModbusRTU_Counter_in_queue) {
        case 0:
            ABB_E31_Total_active_energy = ModbusRTU_GetData_U32(husart2.rx_buffer, 3, BYTE_ORDER_AB_CD) * 0.01;        //01
            ABB_E31_Active_energy_1 = ModbusRTU_GetData_U32(husart2.rx_buffer, 7, BYTE_ORDER_AB_CD) * 0.01;            //02
            ABB_E31_Active_energy_2 = ModbusRTU_GetData_U32(husart2.rx_buffer, 11, BYTE_ORDER_AB_CD) * 0.01;           //03
            ABB_E31_Active_energy_3 = ModbusRTU_GetData_U32(husart2.rx_buffer, 15, BYTE_ORDER_AB_CD) * 0.01;           //04
            ABB_E31_Active_energy_4 = ModbusRTU_GetData_U32(husart2.rx_buffer, 19, BYTE_ORDER_AB_CD) * 0.01;           //05
            ABB_E31_Voltage = ModbusRTU_GetData_U16(husart2.rx_buffer, 23, BYTE_ORDER_AB) * 0.01;                   //06
            ABB_E31_Amp = ModbusRTU_GetData_U16(husart2.rx_buffer, 25, BYTE_ORDER_AB) * 0.01;                       //07
            ABB_E31_Active_power = ModbusRTU_GetData_U16(husart2.rx_buffer, 27, BYTE_ORDER_AB);              //08
            ABB_E31_Power_factor = ModbusRTU_GetData_U16(husart2.rx_buffer, 29, BYTE_ORDER_AB) * 0.01;              //09
            ABB_E31_Frequency = ModbusRTU_GetData_U16(husart2.rx_buffer, 31, BYTE_ORDER_AB) * 0.01;                 //10
            flags_tx_data_ready[0] = true;
            break;
        case 1:
            ABB_E31_Temperature = ModbusRTU_GetData_U16(husart2.rx_buffer, 3, BYTE_ORDER_AB);       //11
            flags_tx_data_ready[1] = true;

            break;
        case 2:

            break;
        case 3:

            break;
        case 4:

            break;
        case 5:

            break;
        case 6:

            break;
    }
}

/**
 ***************************************************************************************
 *  @breif Функция-распределитель очереди запросов по времени и номеру в очереди
 *  @attention Добавляется в systick или таймер, который работает с периодичностью в 1 мс
 ***************************************************************************************
 */
void ModbusRTU_Master_pool(void) {
    if (!flag_ModbusRTU_block) {

        if (ModbusRTU_Timeout) {
            ModbusRTU_Timeout--;
        }

        if (!ModbusRTU_Timeout) { //Если таймаут закончился

            if (ModbusRTU_delay_between_pool) {
                ModbusRTU_delay_between_pool--;
            }
            if (!ModbusRTU_delay_between_pool) {
                if (!flag_ModbusRTU_request_on) {
                    ModbusRTU_Counter_in_queue++; //Переходим к следующему запросу в очереди
                    if (ModbusRTU_Counter_in_queue >= MODBUSRTU_NUMBER_OF_REQUESTS) {
                        ModbusRTU_Counter_in_queue = 0;
                    }
                    flag_ModbusRTU_request_on = true;
                }
            }
        }
    }
}

/**
 ***************************************************************************************
 *  @breif Функция отправки запросов и обработки ответов
 *  @attention Добавляется в main.c в while(1){}
 ***************************************************************************************
 */
void ModbusRTU_Master_pool_run(void) {
    /*===============Отправка запросов мастером по ModbusRTU================*/
    if (flag_ModbusRTU_request_on) { //Если запрос разрешен от мастера
        ModbusRTU_request_queue(ModbusRTU_Counter_in_queue); //формируем пакет из очереди и отправляем слейвам
        flag_ModbusRTU_request_on = false; //сбросим флаг на разрешение запроса ModbusRTU от мастера
        ModbusRTU_Timeout = MODBUSRTU_TIMEOUT; //Обновим таймер
    }
    /*===============Отправка запросов мастером по ModbusRTU================*/

    /*===============Обработка ответов от слейв устройств==================*/
    if (flag_ModbusRTU_block) {
        if (husart2.rx_len > 0) {
            /*Проверка CRC16*/
            uint16_t CRC_check = ModbusRTU_CRC16_Calculate(husart2.rx_buffer, husart2.rx_len - 2, BYTE_ORDER_CD_AB); //Считаем CRC входящих данных
            uint16_t CRC_rx_buffer = husart2.rx_buffer[husart2.rx_len - 2] << 8u | husart2.rx_buffer[husart2.rx_len - 1]; //Смотрим CRC, которая была в пакете
            /*Если CRC16 бьется, то работаем с данными*/
            if (CRC_check == CRC_rx_buffer) {
                ModbusRTU_response_handler(ModbusRTU_Counter_in_queue); //обработка ответов
            }

            ModbusRTU_Timeout = 0; //Сбросим таймаут
            ModbusRTU_delay_between_pool = MODBUSRTU_DELAY_BETWEEN_POOL; //Зададим задержку между фреймами запроса
        }
        flag_ModbusRTU_block = false;
    }
    /*===============Обработка ответов от слейв устройств==================*/

}
