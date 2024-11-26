
#ifndef _H_BLEPRPH_
#define _H_BLEPRPH_

#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nimble/ble.h"
#include "modlog/modlog.h"
#include "esp_peripheral.h"
#ifdef __cplusplus
extern "C" {
#endif

struct ble_hs_cfg;
struct ble_gatt_register_ctxt;

/** GATT server. */
#define GATT_SVR_SVC_ALERT_UUID               0x1811

esp_err_t bleprph_init(bool adv);
esp_err_t bleprph_adv_switch(bool switch_on);
esp_err_t bleprph_send_indicate(uint8_t *data, int len);

#ifdef __cplusplus
}
#endif

#endif
