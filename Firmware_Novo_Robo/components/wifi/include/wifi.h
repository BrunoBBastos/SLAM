#ifndef WIFI_H
#define WIFI_H


#include "freertos/event_groups.h"
EventGroupHandle_t wifi_get_event_group(void);
#define BIT_WIFI_STARTED      (1 << 0)
#define BIT_WIFI_SCAN_DONE    (1 << 1)
#define BIT_WIFI_CONNECTED    (1 << 2)
#define BIT_WIFI_FAIL         (1 << 3)
#define BIT_WIFI_DISCONNECTED (1 << 4)

void wifi_init_sta(void);
void wifi_ini_ap(const char *ssid, const char *pass);
void wifi_disconnect(void);
bool wifi_is_connected(void);

#endif