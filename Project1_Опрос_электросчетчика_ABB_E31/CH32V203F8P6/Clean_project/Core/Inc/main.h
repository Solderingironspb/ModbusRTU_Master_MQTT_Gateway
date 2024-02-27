#ifndef __MAIN_H
#define __MAIN_H

#include <stdbool.h>
#include "ch32v20x.h"
#include "ch32v20x_RVMSIS.h"
#include "ch32v203xx_it.h"
#include "ModbusRTU.h"


enum {
    WIFI_STATUS_NOT_CONNECTED,
    WIFI_STATUS_CONNECTED,
    MQTT_STATUS_NOT_CONNECTED,
    MQTT_STATUS_CONNECTED
};

#ifdef __cplusplus
extern "C" {
#endif

void MQTT_Reveieve(void);

#endif /* INC_MAIN_H_ */
