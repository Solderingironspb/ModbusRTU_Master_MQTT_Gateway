/*
 * ESP8266_lib.c
 *
 *  Created on: Feb 6, 2024
 *      Author: Solderingiron
 */

#include "ESP8266_lib.h"

/*====================================�����ߧ�� �ާڧԧѧߧڧ� ��ӧ֧��էڧ�է��====================================*/
uint32_t SW_Timer_Status_LED_WIFI_MQTT = 0; //���ѧۧާ֧� �էݧ� ��֧�ڧ�էڧ�֧�ܧ�� ��ѧҧ��� ���ѧ���ߧ�ԧ� �էڧ�է� Wifi/mqtt
uint32_t SW_Timer_Status_LED_WIFI_MQTT_Delay = 1000; //���ѧէѧߧڧ� ��֧�ڧ�է� �էݧ� ���ѧ���ߧ�ԧ� �էڧ�է� Wifi/mqtt
bool flag_SW_Timer_Status_LED_Wifi_MQTT = false;
bool flag_Status_LED_Wifi_MQTT = false; //�ӧܧ�/�ӧ�ܧ� ��ӧ֧��էڧ��
bool flag_Network_ok = false;
/*====================================�����ߧ�� �ާڧԧѧߧڧ� ��ӧ֧��էڧ�է��====================================*/

void ESP8266_Reset(uint8_t Delay) {
    ESP8266_GPIO_PORT->BSHR = ESP8266_GPIO_PIN_SET;
    Delay_ms(Delay);
    ESP8266_GPIO_PORT->BSHR = ESP8266_GPIO_PIN_RESET;
    Delay_ms(Delay);
}

void ESP8266_Indication_run(void) {
    if (!flag_Network_ok) {
        if (!flag_SW_Timer_Status_LED_Wifi_MQTT) {
            flag_Status_LED_Wifi_MQTT = !flag_Status_LED_Wifi_MQTT;
            if (flag_Status_LED_Wifi_MQTT) {
                GPIOB->BSHR = GPIO_BSHR_BS15;
            } else {
                GPIOB->BSHR = GPIO_BSHR_BR15;
            }

            Software_timer((uint32_t*) &SW_Timer_Status_LED_WIFI_MQTT, SW_Timer_Status_LED_WIFI_MQTT_Delay);
            flag_SW_Timer_Status_LED_Wifi_MQTT = true;
        }
    }
}

void ESP8266_Indication_Timer(void) {
    if (!flag_Network_ok) {
        if (flag_SW_Timer_Status_LED_Wifi_MQTT) {
            if (!Software_timer_check((uint32_t*) &SW_Timer_Status_LED_WIFI_MQTT)) {
                flag_SW_Timer_Status_LED_Wifi_MQTT = false;
            }
        }
    }
}
