/*
 * ESP8266_lib.c
 *
 *  Created on: Feb 6, 2024
 *      Author: Solderingiron
 */

#include "ESP8266_lib.h"

/*====================================§³§ã§ñ§ß§à§Ö §Þ§Ú§Ô§Ñ§ß§Ú§Ö §ã§Ó§Ö§ä§à§Õ§Ú§à§Õ§à§Þ====================================*/
uint32_t SW_Timer_Status_LED_WIFI_MQTT = 0; //§´§Ñ§Û§Þ§Ö§â §Õ§Ý§ñ §á§Ö§â§Ú§à§Õ§Ú§é§Ö§ã§Ü§à§Û §â§Ñ§Ò§à§ä§í §ã§ä§Ñ§ä§å§ã§ß§à§Ô§à §Õ§Ú§à§Õ§Ñ Wifi/mqtt
uint32_t SW_Timer_Status_LED_WIFI_MQTT_Delay = 1000; //§©§Ñ§Õ§Ñ§ß§Ú§Ö §á§Ö§â§Ú§à§Õ§Ñ §Õ§Ý§ñ §ã§ä§Ñ§ä§å§ã§ß§à§Ô§à §Õ§Ú§à§Õ§Ñ Wifi/mqtt
bool flag_SW_Timer_Status_LED_Wifi_MQTT = false;
bool flag_Status_LED_Wifi_MQTT = false; //§Ó§Ü§Ý/§Ó§í§Ü§Ý §ã§Ó§Ö§ä§à§Õ§Ú§à§Õ
bool flag_Network_ok = false;
/*====================================§³§ã§ñ§ß§à§Ö §Þ§Ú§Ô§Ñ§ß§Ú§Ö §ã§Ó§Ö§ä§à§Õ§Ú§à§Õ§à§Þ====================================*/

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
