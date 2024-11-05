#ifndef WIFI_H
#define WIFI_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

extern EventGroupHandle_t wifi_event_group;
extern bool wifi_connected;
extern const int WIFI_CONNECTED_BIT;

void wifi_init_sta(void);
#endif // WIFI_H
