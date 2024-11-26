
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "services/ans/ble_svc_ans.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "main_defs.h"
#include "bleprph.h"
#include "user_cmd.h"

#ifndef MAX
#define MAX(a,b)	((a) > (b) ? (a) : (b))
#endif

#ifndef MIN
#define MIN(a,b)	((a) < (b) ? (a) : (b))
#endif

//event group events
#define EVENT_INDICATE_SENDING          BIT0

static const char *TAG = "ble";

static uint8_t adv_data[31] = {
    0x05, 0x03, 0x86, 0x28, 0x86, 0xA8,
    0x10, 0x09, 'G', 'W', '-', 'X', 'I', 'A', 'O', '-', 'E', 'S', 'P', '3', '2', 'S', '3',
};

static uint8_t ble_mac_addr[6] = {0};
static uint8_t own_addr_type;

void ble_store_config_init(void);
static int bleprph_gap_event(struct ble_gap_event *event, void *arg);

/* GATT */
static const ble_uuid128_t gatt_svr_svc_uuid =
    BLE_UUID128_INIT(0x55, 0xE4, 0x05, 0xD2, 0xAF, 0x9F, 0xA9, 0x8F, 0xE5, 0x4A, 0x7D, 0xFE, 0x43, 0x53, 0x53, 0x49);

/* A characteristic that can be subscribed to */
static uint16_t gatt_svr_chr_handle_write;
static uint16_t gatt_svr_chr_handle_read;
static const ble_uuid128_t gatt_svr_chr_uuid_write =
    BLE_UUID128_INIT(0xB3, 0x9B, 0x72, 0x34, 0xBE, 0xEC, 0xD4, 0xA8, 0xF4, 0x43, 0x41, 0x88, 0x43, 0x53, 0x53, 0x49);
static const ble_uuid128_t gatt_svr_chr_uuid_read =
    BLE_UUID128_INIT(0x16, 0x96, 0x24, 0x47, 0xC6, 0x23, 0x61, 0xBA, 0xD9, 0x4B, 0x4D, 0x1E, 0x43, 0x53, 0x53, 0x49);

static EventGroupHandle_t g_eg_ble;

static bool g_ble_synced = false;
static bool g_ble_adv = false;
static bool g_ble_connected = false;
static uint16_t g_curr_mtu = 23;

static uint16_t g_curr_ble_conn_handle = 0xffff;

static uint8_t gatt_svr_chr_val;
static uint8_t ble_msg_buf_write[1024] = { 0 };

static int gatt_svc_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        /*** Service ***/
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svr_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[])
        {
            {
                /*** This characteristic can be subscribed to by writing 0x00 and 0x01 to the CCCD ***/
                .uuid = &gatt_svr_chr_uuid_write.u,
                .access_cb = gatt_svc_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &gatt_svr_chr_handle_write,
            },
            {
                /*** This characteristic can be subscribed to by writing 0x00 and 0x01 to the CCCD ***/
                .uuid = &gatt_svr_chr_uuid_read.u,
                .access_cb = gatt_svc_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &gatt_svr_chr_handle_read,
            },
            {
                0, /* No more characteristics in this service. */
            }
        },
    },

    {
        0, /* No more services. */
    },
};

static int gatt_svc_access(uint16_t conn_handle, uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    const ble_uuid_t *uuid;
    int rc;

    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            ESP_LOGI(TAG, "Characteristic read; conn_handle=%d attr_handle=%d\n", conn_handle, attr_handle);
        } else {
            ESP_LOGI(TAG, "Characteristic read by NimBLE stack; attr_handle=%d\n", attr_handle);
        }
        uuid = ctxt->chr->uuid;
        if (attr_handle == gatt_svr_chr_handle_read) {
            char dummy[4] = { 0 };
            rc = os_mbuf_append(ctxt->om, &dummy, sizeof(dummy));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        } else {
            ESP_LOGE(TAG, "should not read on this chr");
        }
        goto unknown;

    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            ESP_LOGI(TAG, "Characteristic write; conn_handle=%d attr_handle=%d", conn_handle, attr_handle);
        } else {
            ESP_LOGI(TAG, "Characteristic write by NimBLE stack; attr_handle=%d", attr_handle);
        }
        uuid = ctxt->chr->uuid;
        if (attr_handle == gatt_svr_chr_handle_write) {
            size_t ble_msg_len = OS_MBUF_PKTLEN(ctxt->om);
            uint16_t real_len = 0;
            ble_msg_t ble_msg = {.size = ble_msg_len, .msg = ble_msg_buf_write};
            memset(ble_msg_buf_write, 0, sizeof(ble_msg_buf_write));
            rc = ble_hs_mbuf_to_flat(ctxt->om, ble_msg.msg, ble_msg_len, &real_len);
            if (rc != 0) {
                return BLE_ATT_ERR_UNLIKELY;
            }
            ESP_LOGI(TAG, "mbuf_len: %d, copied %d bytes from ble stack.", ble_msg_len, real_len);
            ESP_LOGD(TAG, "ble msg: %s", ble_msg.msg);

            if (xQueueSend(ble_msg_queue, &ble_msg, pdMS_TO_TICKS(10)) != pdPASS) {
                ESP_LOGW(TAG, "failed to send ble msg to queue, maybe at_cmd task stalled???");
            } else {
                ESP_LOGD(TAG, "ble msg enqueued");
            }

            // ble_gatts_chr_updated(attr_handle);
            // ESP_LOGI(TAG, "Notification/Indication scheduled for " "all subscribed peers.\n");
            return rc;
        }
        goto unknown;

    case BLE_GATT_ACCESS_OP_READ_DSC:
        if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            ESP_LOGI(TAG, "Descriptor read; conn_handle=%d attr_handle=%d\n", conn_handle, attr_handle);
        } else {
            ESP_LOGI(TAG, "Descriptor read by NimBLE stack; attr_handle=%d\n", attr_handle);
        }
        // uuid = ctxt->dsc->uuid;
        // if (ble_uuid_cmp(uuid, &gatt_svr_dsc_uuid.u) == 0) {
        //     rc = os_mbuf_append(ctxt->om, &gatt_svr_dsc_val, sizeof(gatt_svr_chr_val));
        //     return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        // }
        goto unknown;

    case BLE_GATT_ACCESS_OP_WRITE_DSC:
        if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            ESP_LOGI(TAG, "Descriptor write; conn_handle=%d attr_handle=%d\n", conn_handle, attr_handle);
        } else {
            ESP_LOGI(TAG, "Descriptor write by NimBLE stack; attr_handle=%d\n", attr_handle);
        }
        goto unknown;

    default:
        goto unknown;
    }

unknown:
    /* Unknown characteristic/descriptor;
     * The NimBLE host should not have called this function;
     */
    ESP_LOGE(TAG, "this should not happen, op: %d", ctxt->op);
    return BLE_ATT_ERR_UNLIKELY;
}

void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    char buf[BLE_UUID_STR_LEN];

    switch (ctxt->op) {
    case BLE_GATT_REGISTER_OP_SVC:
        ESP_LOGD(TAG, "registered service %s with handle=%d\n",
                    ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                    ctxt->svc.handle);
        break;

    case BLE_GATT_REGISTER_OP_CHR:
        ESP_LOGD(TAG, "registering characteristic %s with "
                    "def_handle=%d val_handle=%d\n",
                    ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                    ctxt->chr.def_handle,
                    ctxt->chr.val_handle);
        break;

    case BLE_GATT_REGISTER_OP_DSC:
        ESP_LOGD(TAG, "registering descriptor %s with handle=%d\n",
                    ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
                    ctxt->dsc.handle);
        break;

    default:
        break;
    }
}

int gatt_svr_init(void)
{
    int rc;

    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_svc_ans_init();

    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    return 0;
}

/**
 * Logs information about a connection to the console.
 */
static void bleprph_print_conn_desc(struct ble_gap_conn_desc *desc)
{
    char *addr = (char *)desc->our_ota_addr.val;
    ESP_LOGI(TAG, "handle=%d our_ota_addr_type=%d our_ota_addr=%02X:%02X:%02X:%02X:%02X:%02X",
                desc->conn_handle, desc->our_ota_addr.type,
                addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);

    addr = (char *)desc->our_id_addr.val;
    ESP_LOGI(TAG, " our_id_addr_type=%d our_id_addr=%02X:%02X:%02X:%02X:%02X:%02X",
                desc->our_id_addr.type, addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);

    addr = (char *)desc->peer_ota_addr.val;
    ESP_LOGI(TAG, " peer_ota_addr_type=%d peer_ota_addr=%02X:%02X:%02X:%02X:%02X:%02X",
                desc->peer_ota_addr.type, addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);

    addr = (char *)desc->peer_id_addr.val;
    ESP_LOGI(TAG, " peer_id_addr_type=%d peer_id_addr=%02X:%02X:%02X:%02X:%02X:%02X",
                desc->peer_id_addr.type, addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);

    ESP_LOGI(TAG, " conn_itvl=%d conn_latency=%d supervision_timeout=%d "
                "encrypted=%d authenticated=%d bonded=%d\n",
                desc->conn_itvl, desc->conn_latency,
                desc->supervision_timeout,
                desc->sec_state.encrypted,
                desc->sec_state.authenticated,
                desc->sec_state.bonded);
}

/**
 * Enables advertising with the following parameters:
 *     o General discoverable mode.
 *     o Undirected connectable mode.
 */
static void bleprph_advertise_start(void)
{
    struct ble_gap_adv_params adv_params;
    int rc;

    rc = ble_gap_adv_set_data(adv_data, sizeof(adv_data));
    if (rc != 0) {
        ESP_LOGE(TAG, "error setting advertisement data; rc=%d", rc);
        return;
    }

    /* Begin advertising. */
    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, bleprph_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "error enabling advertisement; rc=%d\n", rc);
        return;
    }
}

static void bleprph_advertise_stop(void)
{
    ble_gap_adv_stop();
}

/**
 * The nimble host executes this callback when a GAP event occurs.  The
 * application associates a GAP event callback with each connection that forms.
 * bleprph uses the same callback for all connections.
 *
 * @param event                 The type of event being signalled.
 * @param ctxt                  Various information pertaining to the event.
 * @param arg                   Application-specified argument; unused by
 *                                  bleprph.
 *
 * @return                      0 if the application successfully handled the
 *                                  event; nonzero on failure.  The semantics
 *                                  of the return code is specific to the
 *                                  particular GAP event being signalled.
 */
static int bleprph_gap_event(struct ble_gap_event *event, void *arg)
{
    struct ble_gap_conn_desc desc;
    int rc;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        /* A new connection was established or a connection attempt failed. */
        ESP_LOGI(TAG, "connection %s; status=%d ",
                    event->connect.status == 0 ? "established" : "failed",
                    event->connect.status);
        if (event->connect.status == 0) {
            rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
            assert(rc == 0);
            bleprph_print_conn_desc(&desc);
            g_ble_connected = true;
            g_curr_ble_conn_handle = event->connect.conn_handle;
        }

        if (event->connect.status != 0 && g_ble_adv) {
            /* Connection failed; resume advertising. */
            bleprph_advertise_start();
        }

        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "disconnect; reason=%d ", event->disconnect.reason);
        bleprph_print_conn_desc(&event->disconnect.conn);
        g_ble_connected = false;

        vTaskDelay(2000 / portTICK_PERIOD_MS);
        esp_restart();

        /* Connection terminated; resume advertising. */
        if(g_ble_adv)
            bleprph_advertise_start();
        return 0;

    case BLE_GAP_EVENT_CONN_UPDATE:
        /* The central has updated the connection parameters. */
        ESP_LOGI(TAG, "connection updated; status=%d ",
                    event->conn_update.status);
        rc = ble_gap_conn_find(event->conn_update.conn_handle, &desc);
        assert(rc == 0);
        bleprph_print_conn_desc(&desc);
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(TAG, "advertise complete; reason=%d",
                    event->adv_complete.reason);
        if(g_ble_adv)
            bleprph_advertise_start();
        return 0;

    case BLE_GAP_EVENT_ENC_CHANGE:
        /* Encryption has been enabled or disabled for this connection. */
        ESP_LOGI(TAG, "encryption change event; status=%d ",
                    event->enc_change.status);
        rc = ble_gap_conn_find(event->enc_change.conn_handle, &desc);
        assert(rc == 0);
        bleprph_print_conn_desc(&desc);
        return 0;

    case BLE_GAP_EVENT_NOTIFY_TX:
        ESP_LOGI(TAG, "notify_tx event; conn_handle=%d attr_handle=%d "
                    "status=%d is_indication=%d",
                    event->notify_tx.conn_handle,
                    event->notify_tx.attr_handle,
                    event->notify_tx.status,
                    event->notify_tx.indication);
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        ESP_LOGI(TAG, "subscribe event; conn_handle=%d attr_handle=%d "
                    "reason=%d prevn=%d curn=%d previ=%d curi=%d\n",
                    event->subscribe.conn_handle,
                    event->subscribe.attr_handle,
                    event->subscribe.reason,
                    event->subscribe.prev_notify,
                    event->subscribe.cur_notify,
                    event->subscribe.prev_indicate,
                    event->subscribe.cur_indicate);
        return 0;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "mtu update event; conn_handle=%d cid=%d mtu=%d\n",
                    event->mtu.conn_handle,
                    event->mtu.channel_id,
                    event->mtu.value);
        if (event->mtu.value < 20) {
            ESP_LOGW(TAG, "mtu become less than 20??? really?");
        }
        g_curr_mtu = event->mtu.value;
        return 0;

    case BLE_GAP_EVENT_REPEAT_PAIRING:
        /* We already have a bond with the peer, but it is attempting to
         * establish a new secure link.  This app sacrifices security for
         * convenience: just throw away the old bond and accept the new link.
         */

        /* Delete the old bond. */
        rc = ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc);
        assert(rc == 0);
        ble_store_util_delete_peer(&desc.peer_id_addr);

        /* Return BLE_GAP_REPEAT_PAIRING_RETRY to indicate that the host should
         * continue with the pairing operation.
         */
        return BLE_GAP_REPEAT_PAIRING_RETRY;

    case BLE_GAP_EVENT_PASSKEY_ACTION:
        ESP_LOGI(TAG, "PASSKEY_ACTION_EVENT started");
        // struct ble_sm_io pkey = {0};
        // int key = 0;

        // if (event->passkey.params.action == BLE_SM_IOACT_DISP) {
        //     pkey.action = event->passkey.params.action;
        //     pkey.passkey = 123456; // This is the passkey to be entered on peer
        //     ESP_LOGI(TAG, "Enter passkey %" PRIu32 "on the peer side", pkey.passkey);
        //     rc = ble_sm_inject_io(event->passkey.conn_handle, &pkey);
        //     ESP_LOGI(TAG, "ble_sm_inject_io result: %d", rc);
        // } else if (event->passkey.params.action == BLE_SM_IOACT_NUMCMP) {
        //     ESP_LOGI(TAG, "Passkey on device's display: %" PRIu32 , event->passkey.params.numcmp);
        //     ESP_LOGI(TAG, "Accept or reject the passkey through console in this format -> key Y or key N");
        //     pkey.action = event->passkey.params.action;
        //     if (scli_receive_key(&key)) {
        //         pkey.numcmp_accept = key;
        //     } else {
        //         pkey.numcmp_accept = 0;
        //         ESP_LOGE(TAG, "Timeout! Rejecting the key");
        //     }
        //     rc = ble_sm_inject_io(event->passkey.conn_handle, &pkey);
        //     ESP_LOGI(TAG, "ble_sm_inject_io result: %d", rc);
        // } else if (event->passkey.params.action == BLE_SM_IOACT_OOB) {
        //     static uint8_t tem_oob[16] = {0};
        //     pkey.action = event->passkey.params.action;
        //     for (int i = 0; i < 16; i++) {
        //         pkey.oob[i] = tem_oob[i];
        //     }
        //     rc = ble_sm_inject_io(event->passkey.conn_handle, &pkey);
        //     ESP_LOGI(TAG, "ble_sm_inject_io result: %d", rc);
        // } else if (event->passkey.params.action == BLE_SM_IOACT_INPUT) {
        //     ESP_LOGI(TAG, "Enter the passkey through console in this format-> key 123456");
        //     pkey.action = event->passkey.params.action;
        //     if (scli_receive_key(&key)) {
        //         pkey.passkey = key;
        //     } else {
        //         pkey.passkey = 0;
        //         ESP_LOGE(TAG, "Timeout! Passing 0 as the key");
        //     }
        //     rc = ble_sm_inject_io(event->passkey.conn_handle, &pkey);
        //     ESP_LOGI(TAG, "ble_sm_inject_io result: %d", rc);
        // }
        return 0;

    case BLE_GAP_EVENT_AUTHORIZE:
        ESP_LOGI(TAG, "authorize event: conn_handle=%d attr_handle=%d is_read=%d",
                    event->authorize.conn_handle,
                    event->authorize.attr_handle,
                    event->authorize.is_read);

        /* The default behaviour for the event is to reject authorize request */
        event->authorize.out_response = BLE_GAP_AUTHORIZE_REJECT;
        return 0;
    }

    return 0;
}

static void bleprph_on_reset(int reason)
{
    ESP_LOGE(TAG, "Resetting state; reason=%d\n", reason);
    g_ble_synced = false;
}

static void bleprph_on_sync(void)
{
    int rc;

    /* Make sure we have proper identity address set (public preferred) */
    rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);

    /* Figure out address to use while advertising (no privacy for now) */
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGI(TAG, "error determining address type; rc=%d\n", rc);
        return;
    }

    /* Printing ADDR */
    uint8_t addr_val[6] = {0};
    rc = ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);
    memset(ble_mac_addr, 0, sizeof(ble_mac_addr));
    for (int i = 0; i < 6 && rc == 0; i++)
    {
        ble_mac_addr[i] = addr_val[5 - i];
    }

    ESP_LOGI(TAG, "BLE Address: %02X:%02X:%02X:%02X:%02X:%02X", 
                    addr_val[5], addr_val[4], addr_val[3], addr_val[2], addr_val[1], addr_val[0]);

    g_ble_synced = true;
}

static void bleprph_host_task(void *param)
{
    ESP_LOGI(TAG, "BLE Host Task Started");
    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();

    nimble_port_freertos_deinit();
}

static void bleprph_monitor_task(void *p_arg)
{
    bool last_ble_adv_state = false;

    while (1) {
       if (g_ble_adv) {
            if (g_ble_adv != last_ble_adv_state && g_ble_synced && !g_ble_connected) {
                bleprph_advertise_start();
                last_ble_adv_state = g_ble_adv;
            }
        } else {
            if (g_ble_adv != last_ble_adv_state && g_ble_synced) {

                if (g_ble_connected) {
                    // wait bleprph_send_indicate() done if it's under call
                    while (xEventGroupGetBits(g_eg_ble) & EVENT_INDICATE_SENDING) {
                        vTaskDelay(pdMS_TO_TICKS(10));
                    }
                    ESP_LOGW(TAG, "going to terminate the active connections!!!");
                    ble_gap_terminate(g_curr_ble_conn_handle, BLE_HS_EDISABLED);
                }

                bleprph_advertise_stop();

                last_ble_adv_state = g_ble_adv;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

esp_err_t bleprph_adv_switch(bool switch_on)
{
    g_ble_adv = switch_on;
    set_user_led(switch_on);
    return ESP_OK;
}

esp_err_t bleprph_send_indicate(uint8_t *data, int len)
{
    if(!g_ble_connected || !g_ble_adv) return BLE_HS_ENOTCONN;
    int rc = ESP_FAIL;
    struct os_mbuf *txom = NULL;

    xEventGroupSetBits(g_eg_ble, EVENT_INDICATE_SENDING);

    int mtu = g_curr_mtu;
    int txlen = 0, txed_len = 0;

    const int wait_step_climb = 20, wait_max = 20000;  //ms
    int wait = 0, wait_step = wait_step_climb, wait_sum = 0;  //ms
    const int retry_max = 100;
    int retry_cnt = 0;
    while (len > 0 && g_ble_connected && g_ble_adv) {
        txlen = MIN(len, mtu - 3);
        txom = ble_hs_mbuf_from_flat(data + txed_len, txlen);
        ESP_LOGD(TAG, "after mbuf alloc, os_msys_count: %d, os_msys_num_free: %d", os_msys_count(), os_msys_num_free());
        if (!txom) {
            wait += wait_step;
            wait_step += wait_step_climb;
            ESP_LOGD(TAG, "app_ble_send_indicate, mbuf alloc failed, wait %dms", wait);
            vTaskDelay(pdMS_TO_TICKS(wait));
            wait_sum += wait;
            if (wait_sum > wait_max) {
                ESP_LOGE(TAG, "app_ble_send_indicate, mbuf alloc timeout!!!");
                rc = BLE_HS_ENOMEM;
                goto indicate_end;
            }
            continue;
        }
        wait = wait_sum = 0;
        wait_step = wait_step_climb;

        rc = ble_gatts_indicate_custom(g_curr_ble_conn_handle, gatt_svr_chr_handle_read, txom);
        // txom will be consumed anyways, we don't need to release it here.
        if (rc != 0) {
            ESP_LOGD(TAG, "ble_gatts_indicate_custom failed (rc=%d, mtu=%d, txlen=%d, remain_len=%d), retry ...", rc, mtu, txlen, len);
            retry_cnt++;
            if (retry_cnt > retry_max) {
                ESP_LOGE(TAG, "ble_gatts_indicate_custom failed overall after %d retries!!!", retry_max);
                rc = BLE_HS_ESTALLED;
                goto indicate_end;
            }
            continue;
        }
        txed_len += txlen;
        len -= txlen;
        ESP_LOGI(TAG, "indication sent successfully, mtu=%d, txlen=%d, remain_len=%d", mtu, txlen, len);
    }

    if (len != 0) {
        rc = BLE_ERR_CONN_TERM_LOCAL;
    }

indicate_end:
    ESP_LOGD(TAG, "before app_ble_send_indicate return, os_msys_count: %d, os_msys_num_free: %d", os_msys_count(), os_msys_num_free());
    xEventGroupClearBits(g_eg_ble, EVENT_INDICATE_SENDING);
    return rc;
}

esp_err_t bleprph_init(bool adv)
{
    int rc;

    g_eg_ble = xEventGroupCreate();

    esp_err_t ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init nimble %d ", ret);
        return ret;
    }

    /* Initialize the NimBLE host configuration. */
    ble_hs_cfg.reset_cb = bleprph_on_reset;
    ble_hs_cfg.sync_cb = bleprph_on_sync;
    ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO;  //Security Manager Local Input Output Capabilities
    ble_hs_cfg.sm_sc = 0; //Security Manager Secure Connections flag

    rc = gatt_svr_init();
    assert(rc == 0);

    rc = ble_svc_gap_device_name_set("GW-XIAO-ESP32S3");
    assert(rc == 0);

    ble_store_config_init();

    nimble_port_freertos_init(bleprph_host_task);

    const uint32_t stack_size = 10 * 1024;
    StackType_t *task_stack1 = (StackType_t *)heap_caps_calloc(1, stack_size * sizeof(StackType_t), MALLOC_CAP_INTERNAL);
    StaticTask_t *task_tcb1 = heap_caps_calloc(1, sizeof(StaticTask_t), MALLOC_CAP_INTERNAL);
    xTaskCreateStatic(bleprph_monitor_task, "bleprph_monitor", stack_size, NULL, 2, task_stack1, task_tcb1);

    vTaskDelay( 1000 / portTICK_PERIOD_MS );
    bleprph_adv_switch(adv);

    return ret;
}
