
#ifndef _H_USER_CMD_
#define _H_USER_CMD_

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t ssid[48];
    uint8_t authmode[15];
    int8_t rssi;
} wifi_info_t;

typedef struct {
    uint8_t *msg;
    int size;
} ble_msg_t;

extern QueueHandle_t ble_msg_queue;

bool user_cmd_init(void);

#ifdef __cplusplus
}
#endif

#endif