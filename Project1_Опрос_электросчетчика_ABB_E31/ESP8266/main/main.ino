/**
 ******************************************************************************
 @file           : main.cpp
 @brief          : Main program body
 ******************************************************************************
 @attention

 Created on: 05 мая. 2023 г.
 Author: Oleg Volkov

 YouTube: https://www.youtube.com/channel/UCzZKTNVpcMSALU57G1THoVw
 GitHub: https://github.com/Solderingironspb/Lessons-Stm32/blob/master/README.md
 Группа ВК: https://vk.com/solderingiron.stm32

 ******************************************************************************
 */

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "uart_mqtt_lib.h"

#define VARIABLE_ID esp_rx_buffer[1]

#define name_mqtt_user "ABB_E31"


/*-------------------------------Настройки Wi-fi------------------------------------------*/

uint8_t newMACAddress[] = { 0x32, 0xAE, 0x11, 0x07, 0x0D, 0x70 }; //Задаем MAC адрес устройства

const char *ssid = "****";       // Имя Wi-fi точки доступа
const char *pass = "****";  // Пароль от Wi-fi точки доступа

const char *mqtt_server = "192.168.0.104";  // Имя сервера MQTT
const int mqtt_port = 1883;                       // Порт для подключения к серверу MQTT
const char *mqtt_user = "****";                // Логин от сервера
const char *mqtt_pass = "****";            // Пароль от сервера

/*-------------------------------Настройки Wi-fi------------------------------------------*/

uint8_t esp_rx_buffer[128] = { 0, };  //входящий буфер
uint8_t esp_tx_buffer[128] = { 0, };  //исходящий буфер

float ABB_E31_Total_active_energy;       //01
float ABB_E31_Active_energy_1;           //02
float ABB_E31_Active_energy_2;           //03
float ABB_E31_Active_energy_3;           //04
float ABB_E31_Active_energy_4;           //05
float ABB_E31_Voltage;                      //06
float ABB_E31_Amp;                          //07
float ABB_E31_Active_power;                 //08
float ABB_E31_Power_factor;                 //09
float ABB_E31_Frequency;                    //10
int16_t ABB_E31_Temperature;                //11          


WiFiClient wclient;
PubSubClient client( wclient, mqtt_server, mqtt_port);

/* USER CODE BEGIN PFP */
void callback(const MQTT::Publish &pub);
void uart_parsing(void);
/* USER CODE END PFP */

// Функция получения данных от сервера
void callback(const MQTT::Publish &pub) {
  String payload = pub.payload_string();

  /*------------------Парсинг приходящих писем в топики-----------------------*/
  /*if (String(pub.topic()) == "Test/OWEN_PR200_Out3")  //  проверяем из нужного ли нам топика пришли данные
  {
    OWEN_PR200_Out3 = payload.toInt();  //  преобразуем полученные данные в тип float(все типы данных, кроме float преобразуем в integer)
    UART_MQTT_Send_data_uint16_t(18, OWEN_PR200_Out3, esp_tx_buffer);
  }
  if (String(pub.topic()) == "Test/OWEN_PR200_Out4")  //  проверяем из нужного ли нам топика пришли данные
  {
    OWEN_PR200_Out4 = payload.toInt();  //  преобразуем полученные данные в тип float(все типы данных, кроме float преобразуем в integer)
    UART_MQTT_Send_data_uint16_t(19, OWEN_PR200_Out4, esp_tx_buffer);
  }*/

  /*------------------Парсинг приходящих писем в топики-----------------------*/
}

void uart_parsing(void) {
  if (Serial.available() > 0) {
    Serial.readBytes(esp_rx_buffer, 9);
    switch (VARIABLE_ID) {
      /*----------------здесь будет выборка по номеру переменной-----------------------*/

      case (1):
      if (UART_MQTT_Checksumm_validation(esp_rx_buffer)) {
        ABB_E31_Total_active_energy = UART_MQTT_Receive_data_float(esp_rx_buffer);
        client.publish("ABB_E31/Total_active_energy", String(ABB_E31_Total_active_energy));
      }
      break;
      case (2):
      if (UART_MQTT_Checksumm_validation(esp_rx_buffer)) {
        ABB_E31_Active_energy_1 = UART_MQTT_Receive_data_float(esp_rx_buffer);
        client.publish("ABB_E31/Active_energy_1", String(ABB_E31_Active_energy_1));
      }
      break;
      case (3):
      if (UART_MQTT_Checksumm_validation(esp_rx_buffer)) {
        ABB_E31_Active_energy_2 = UART_MQTT_Receive_data_float(esp_rx_buffer);
        client.publish("ABB_E31/Active_energy_2", String(ABB_E31_Active_energy_2));
      }
      break;
      case (4):
      if (UART_MQTT_Checksumm_validation(esp_rx_buffer)) {
        ABB_E31_Active_energy_3 = UART_MQTT_Receive_data_float(esp_rx_buffer);
        client.publish("ABB_E31/Active_energy_3", String(ABB_E31_Active_energy_3));
      }
      break;
      case (5):
      if (UART_MQTT_Checksumm_validation(esp_rx_buffer)) {
        ABB_E31_Active_energy_4 = UART_MQTT_Receive_data_float(esp_rx_buffer);
        client.publish("ABB_E31/Active_energy_4", String(ABB_E31_Active_energy_4));
      }
      break;
      case (6):
      if (UART_MQTT_Checksumm_validation(esp_rx_buffer)) {
        ABB_E31_Voltage = UART_MQTT_Receive_data_float(esp_rx_buffer);
        client.publish("ABB_E31/Voltage", String(ABB_E31_Voltage));
      }
      break;
      case (7):
      if (UART_MQTT_Checksumm_validation(esp_rx_buffer)) {
        ABB_E31_Amp = UART_MQTT_Receive_data_float(esp_rx_buffer);
        client.publish("ABB_E31/Amp", String(ABB_E31_Amp));
      }
      break;
      case (8):
      if (UART_MQTT_Checksumm_validation(esp_rx_buffer)) {
        ABB_E31_Active_power = UART_MQTT_Receive_data_float(esp_rx_buffer);
        client.publish("ABB_E31/Active_power", String(ABB_E31_Active_power));
      }
      break;
      case (9):
      if (UART_MQTT_Checksumm_validation(esp_rx_buffer)) {
        ABB_E31_Power_factor = UART_MQTT_Receive_data_float(esp_rx_buffer);
        client.publish("ABB_E31/Power_factor", String(ABB_E31_Power_factor));
      }
      break;
      case (10):
      if (UART_MQTT_Checksumm_validation(esp_rx_buffer)) {
        ABB_E31_Frequency = UART_MQTT_Receive_data_float(esp_rx_buffer);
        client.publish("ABB_E31/Frequency", String(ABB_E31_Frequency));
      }
      break;
      case (11):
      if (UART_MQTT_Checksumm_validation(esp_rx_buffer)) {
        ABB_E31_Temperature = UART_MQTT_Receive_data_int16_t(esp_rx_buffer);
        client.publish("ABB_E31/Temperature", String(ABB_E31_Temperature));
      }
      break;
      
      /*----------------здесь будет выборка по номеру переменной-----------------------*/
    }
    memset(esp_rx_buffer, 0, 9);  //очистка входящего буффера
  }
}

void setup(void) {

  Serial.begin(9600);
  delay(100);
  Serial.setTimeout(5);
  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);
  //wifi_set_macaddr(STATION_IF, &newMACAddress[0]);
  WiFi.hostname(name_mqtt_user);
  WiFi.begin(ssid, pass);

}

void loop(void) {
  if (client.connected()) {
    client.loop();
    uart_parsing();
    //delay(1);
  } else {
    if (WiFi.status() != WL_CONNECTED)  //Проверяем статус подключения к Wi-fi точке
        {
      wifi_not_ok();  //отсылаем служебную команду Wifi != OK
      delay(10);

      WiFi.begin(ssid, pass); //Вводим имя точки доступа и пароль

      if (WiFi.waitForConnectResult() != WL_CONNECTED)
        return;
      wifi_ok();  //отсылаем служебную команду Wifi = OK
      delay(10);
    }

    if (WiFi.status() == WL_CONNECTED)  // Если подключились к Wi-fi точке
        {
      if (!client.connected())  //Проверяем статус подключения к MQTT серверу
      {
        mqtt_not_ok();
        delay(10);
        if (client.connect(MQTT::Connect(name_mqtt_user).set_auth(mqtt_user, mqtt_pass)))  //Если подключились к MQTT серверу, то авторизуемся
        {
          mqtt_ok();  //отсылаем служебную команду MQTT = OK
          delay(10);
          client.set_callback(callback);

          /*--------------------Указываем топики, на которые хотим подписаться-------------------------*/
          client.subscribe("ABB_E31/Total_active_energy");      //01
          client.subscribe("ABB_E31/Active_energy_1");          //02
          client.subscribe("ABB_E31/Active_energy_2");          //03
          client.subscribe("ABB_E31/Active_energy_3");          //04
          client.subscribe("ABB_E31/Active_energy_4");          //05
          client.subscribe("ABB_E31/Voltage");                  //06
          client.subscribe("ABB_E31/Amp");                      //07
          client.subscribe("ABB_E31/Active_power");             //08
          client.subscribe("ABB_E31/Power_factor");             //09
          client.subscribe("ABB_E31/Frequency");                //10
          client.subscribe("ABB_E31/Temperature");              //11
      
          /*--------------------Указываем топики, на которые хотим подписаться-------------------------*/
        } else {
          mqtt_not_ok();  //отсылаем служебную команду MQTT = !OK
          delay(10);

        }
      }

    }
    delay(1000);
  }
}
/************************** (C) COPYRIGHT Soldering iron *******END OF FILE****/
