#include "main.h"
#include "ESP8266_lib.h"

#define MQTT_ID_TEMPERATURE          0x01
#define MQTT_ID_HUMIDITY             0x02


void UART_MQTT_Check_net(void);
bool UART_MQTT_Checksumm_validation(uint8_t* rx_buffer);
void UART_MQTT_Send_data_uint16_t(uint8_t VARIABLE_ID, uint16_t data, uint8_t* tx_buffer);
void UART_MQTT_Send_data_float(uint8_t VARIABLE_ID, float data, uint8_t* tx_buffer);
void UART_MQTT_Send_data_int16_t(uint8_t VARIABLE_ID, int16_t data, uint8_t *tx_buffer);
int16_t UART_MQTT_Receive_data_int16_t(uint8_t *rx_buffer);
