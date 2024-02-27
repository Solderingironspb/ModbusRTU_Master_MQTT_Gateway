/*
 * ESP8266_lib.h
 *
 *  Created on: Feb 6, 2024
 *      Author: Solderingiron
 */

#ifndef INC_ESP8266_LIB_H_
#define INC_ESP8266_LIB_H_

#include "main.h"
#include "SoftwareTimer.h"

#define ESP8266_GPIO_PORT       GPIOA               //§±§à§â§ä §ß§à§Ø§Ü§Ú §â§Ö§ã§Ö§ä
#define ESP8266_GPIO_PIN_SET    GPIO_BSHR_BS8       //§£§Ü§Ý. §â§Ö§ã§Ö§ä
#define ESP8266_GPIO_PIN_RESET  GPIO_BSHR_BR8       //§°§ä§á§å§ã§ä§Ú§ä§î §â§Ö§ã§Ö§ä

#define ESP8266_WIFI_MQTT_LED_ON    GPIOB->BSHR=GPIO_BSHR_BS15;
#define ESP8266_WIFI_MQTT_LED_OFF   GPIOB->BSHR=GPIO_BSHR_BR15;

void ESP8266_Reset(uint8_t Delay);
void ESP8266_Indication_run(void);
void ESP8266_Indication_Timer(void);

#endif /* INC_ESP8266_LIB_H_ */
