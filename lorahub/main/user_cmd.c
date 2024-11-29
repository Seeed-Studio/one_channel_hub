
#include <string.h>
#include <stdlib.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <esp_wifi.h>
#include <esp_mac.h>
#include <esp_check.h>
#include <nvs_flash.h>
#include "freertos/FreeRTOS.h"
#include "config_nvs.h"
#include "cJSON.h"
#include "wifi.h"
#include "bleprph.h"
#include "user_cmd.h"

       
#define WIFI_CMD_CONFIG_GET "wifi_config=?"
#define LORA_CMD_CONFIG_GET "lora_config=?"
#define WIFI_CMD_SCAN_X_GET "wifi_scan_x=?"
#define WIFI_CMD_CONFIG_SET "wifi_config="
#define LORA_CMD_CONFIG_SET "lora_config="

#define XXXX_CMD_GET_LEN    strlen(WIFI_CMD_CONFIG_GET)
#define XXXX_CMD_SET_LEN    strlen(WIFI_CMD_CONFIG_SET)

char *cmd_result_ok = "\r\nOK\r\n";
char *cmd_result_err = "\r\nERROR\r\n";

static const char *TAG = "cmd";

QueueHandle_t ble_msg_queue;

static char wifi_ssid[64];
static char wifi_password[64];
static uint64_t gw_id;
static char lns_addr[64];
static uint16_t lns_port;
static char sntp_addr[64];
static uint32_t chan_freq;
static uint32_t chan_dr;
static uint16_t chan_bw;

static uint8_t wifi_status;

#define WIFI_SCAN_RESULT_CNT_MAX  8
wifi_info_t wifi_ap_scan[WIFI_SCAN_RESULT_CNT_MAX] = {0};

static uint8_t cmd_rsp_buf[1024] = { 0 };

static bool user_config_load(void)
{
    esp_err_t err;
    nvs_handle_t my_handle;
    bool config_load = false;

    ESP_LOGI( TAG, "Get channel configuration from NVS" );

    printf( "Opening Non-Volatile Storage (NVS) handle for reading... " );
    err = nvs_open( "storage", NVS_READONLY, &my_handle );
    if( err != ESP_OK )
    {   
        config_load = true;
        printf( "Error (%s) opening NVS handle!\n", esp_err_to_name( err ) );
    }
    else
    {
        printf( "Done\n" );

        size_t size = 0;
        size = sizeof( wifi_ssid );
        memset(wifi_ssid, 0, size);
        err = nvs_get_str( my_handle, CFG_NVS_KEY_WIFI_SSID, wifi_ssid, &size );
        if( err == ESP_OK )
        {
            printf( "NVS -> %s = %s\n", CFG_NVS_KEY_WIFI_SSID, wifi_ssid );
        }
        else
        {
            config_load = true;
            printf( "Failed to get %s from NVS - %s\n", CFG_NVS_KEY_WIFI_SSID, esp_err_to_name( err ) );
        }

        size = sizeof( wifi_password );
        memset(wifi_password, 0, size);
        err = nvs_get_str( my_handle, CFG_NVS_KEY_WIFI_PSWD, wifi_password, &size );
        if( err == ESP_OK )
        {
            printf( "NVS -> %s = %s\n", CFG_NVS_KEY_WIFI_PSWD, wifi_password );
        }
        else
        {
            config_load = true;
            printf( "Failed to get %s from NVS - %s\n", CFG_NVS_KEY_WIFI_PSWD, esp_err_to_name( err ) );
        }

        err = nvs_get_u64( my_handle, CFG_NVS_KEY_GW_ID, &gw_id );
        if( err == ESP_OK )
        {
            printf( "NVS -> %s = 0x%016llX\n", CFG_NVS_KEY_GW_ID, gw_id );
        }
        else
        {
            config_load = true;
            printf( "Failed to get %s from NVS - %s\n", CFG_NVS_KEY_GW_ID, esp_err_to_name( err ) );
        }

        size = sizeof( lns_addr );
        memset(lns_addr, 0, size);
        err = nvs_get_str( my_handle, CFG_NVS_KEY_LNS_ADDRESS, lns_addr, &size );
        if( err == ESP_OK )
        {
            printf( "NVS -> %s = %s\n", CFG_NVS_KEY_LNS_ADDRESS, lns_addr );
        }
        else
        {
            config_load = true;
            printf( "Failed to get %s from NVS - %s\n", CFG_NVS_KEY_LNS_ADDRESS, esp_err_to_name( err ) );
        }

        err = nvs_get_u16( my_handle, CFG_NVS_KEY_LNS_PORT, &lns_port );
        if( err == ESP_OK )
        {
            printf( "NVS -> %s = %" PRIu16 "\n", CFG_NVS_KEY_LNS_PORT, lns_port );
        }
        else
        {
            config_load = true;
            printf( "Failed to get %s from NVS - %s\n", CFG_NVS_KEY_LNS_PORT, esp_err_to_name( err ) );
        }

        size = sizeof( sntp_addr );
        memset(sntp_addr, 0, size);
        err = nvs_get_str( my_handle, CFG_NVS_KEY_SNTP_ADDRESS, sntp_addr, &size );
        if( err == ESP_OK )
        {
            printf( "NVS -> %s = %s\n", CFG_NVS_KEY_SNTP_ADDRESS, sntp_addr );
        }
        else
        {
            config_load = true;
            printf( "Failed to get %s from NVS - %s\n", CFG_NVS_KEY_SNTP_ADDRESS, esp_err_to_name( err ) );
        }

        err = nvs_get_u32( my_handle, CFG_NVS_KEY_CHAN_FREQ, &( chan_freq ) );
        if( err == ESP_OK )
        {
            printf( "NVS -> %s = %" PRIu32 "hz\n", CFG_NVS_KEY_CHAN_FREQ, chan_freq );
            /* sanity check */
            if( ( chan_freq < 150000000 ) || ( chan_freq > 960000000 ) )
            {
                ESP_LOGE( TAG, "ERROR: wrong channel frequency configuration from NVS, set to %" PRIu32 "hz\n",
                          ( uint32_t ) CONFIG_CHANNEL_FREQ_HZ );
                chan_freq = CONFIG_CHANNEL_FREQ_HZ;
            }
        }
        else
        {
            config_load = true;
            printf( "Failed to get %s from NVS - %s\n", CFG_NVS_KEY_CHAN_FREQ, esp_err_to_name( err ) );
        }

        err = nvs_get_u32( my_handle, CFG_NVS_KEY_CHAN_DR, &( chan_dr ) );
        if( err == ESP_OK )
        {
            printf( "NVS -> %s = %" PRIu32 "\n", CFG_NVS_KEY_CHAN_DR, chan_dr );
            /* sanity check */
            if( ( chan_dr < 7 ) || ( chan_dr > 12 ) )
            {
                ESP_LOGE( TAG, "ERROR: wrong channel datarate configuration from NVS, set to %" PRIu32 "\n",
                          ( uint32_t ) CONFIG_CHANNEL_LORA_DATARATE );
                chan_dr = CONFIG_CHANNEL_LORA_DATARATE;
            }
        }
        else
        {
            config_load = true;
            printf( "Failed to get %s from NVS - %s\n", CFG_NVS_KEY_CHAN_DR, esp_err_to_name( err ) );
        }

        err = nvs_get_u16( my_handle, CFG_NVS_KEY_CHAN_BW, &chan_bw );
        if( err == ESP_OK )
        {
            printf( "NVS -> %s = %" PRIu16 "khz\n", CFG_NVS_KEY_CHAN_BW, chan_bw );
            /* sanity check */
            if( ( chan_bw != 125 ) && ( chan_bw != 250 ) && ( chan_bw != 500 ) )
            {
                ESP_LOGE( TAG, "ERROR: wrong channel bandwidth configuration from NVS, set to %" PRIu16 "khz\n",
                          ( uint16_t ) CONFIG_CHANNEL_LORA_BANDWIDTH );
                chan_bw = CONFIG_CHANNEL_LORA_BANDWIDTH;
            }
        }
        else
        {
            config_load = true;
            printf( "Failed to get %s from NVS - %s\n", CFG_NVS_KEY_CHAN_BW, esp_err_to_name( err ) );
        }

    }
    nvs_close( my_handle );
    printf( "Closed NVS handle for reading.\n" );

    return config_load;
}

static esp_err_t user_wifi_config_save(void)
{
    esp_err_t err = ESP_OK;
    nvs_handle_t my_handle;

    ESP_LOGI( TAG, "Set channel configuration to NVS" );

    printf( "Opening Non-Volatile Storage (NVS) handle for writing... " );
    err = nvs_open( "storage", NVS_READWRITE, &my_handle );
    if( err != ESP_OK )
    {
        printf( "Error (%s) opening NVS handle!\n", esp_err_to_name( err ) );
        return ESP_FAIL;
    }
    else
    {
        printf( "Done\n" );
    }

    printf( "NVS <- %s = %s ... ", CFG_NVS_KEY_WIFI_SSID, wifi_ssid );
    err = nvs_set_str( my_handle, CFG_NVS_KEY_WIFI_SSID, wifi_ssid );
    if( err == ESP_OK )
    {
        printf( "Done\n" );
    }
    else
    {
        printf( "Failed\n" );
        nvs_close( my_handle );
        printf( "Closed NVS handle for writing.\n" );
        return ESP_FAIL;
    }

    printf( "NVS <- %s = %s ... ", CFG_NVS_KEY_WIFI_PSWD, wifi_password );
    err = nvs_set_str( my_handle, CFG_NVS_KEY_WIFI_PSWD, wifi_password );
    if( err == ESP_OK )
    {
        printf( "Done\n" );
    }
    else
    {
        printf( "Failed\n" );
        nvs_close( my_handle );
        printf( "Closed NVS handle for writing.\n" );
        return ESP_FAIL;
    }

    printf( "Committing updates in NVS ... " );
    err = nvs_commit( my_handle );
    if( err == ESP_OK )
    {
        printf( "Done\n" );
    }
    else
    {
        printf( "Failed\n" );
        nvs_close( my_handle );
        printf( "Closed NVS handle for writing.\n" );
        return ESP_FAIL;
    }

    nvs_close( my_handle );
    printf( "Closed NVS handle for writing.\n" );

    return ESP_OK;
}

static esp_err_t user_lora_config_save(void)
{
    esp_err_t err = ESP_OK;
    nvs_handle_t my_handle;

    ESP_LOGI( TAG, "Set channel configuration to NVS" );

    printf( "Opening Non-Volatile Storage (NVS) handle for writing... " );
    err = nvs_open( "storage", NVS_READWRITE, &my_handle );
    if( err != ESP_OK )
    {
        printf( "Error (%s) opening NVS handle!\n", esp_err_to_name( err ) );
        return ESP_FAIL;
    }
    else
    {
        printf( "Done\n" );
    }

    printf( "NVS <- %s = 0x%016llX ... ", CFG_NVS_KEY_GW_ID, gw_id );
    err = nvs_set_u64( my_handle, CFG_NVS_KEY_GW_ID, gw_id );
    if( err == ESP_OK )
    {
        printf( "Done\n" );
    }
    else
    {
        printf( "Failed\n" );
        nvs_close( my_handle );
        printf( "Closed NVS handle for writing.\n" );
        return ESP_FAIL;
    }

    printf( "NVS <- %s = %s ... ", CFG_NVS_KEY_LNS_ADDRESS, lns_addr );
    err = nvs_set_str( my_handle, CFG_NVS_KEY_LNS_ADDRESS, lns_addr );
    if( err == ESP_OK )
    {
        printf( "Done\n" );
    }
    else
    {
        printf( "Failed\n" );
        nvs_close( my_handle );
        printf( "Closed NVS handle for writing.\n" );
        return ESP_FAIL;
    }

    printf( "NVS <- %s = %" PRIu16 " ... ", CFG_NVS_KEY_LNS_PORT, lns_port );
    err = nvs_set_u16( my_handle, CFG_NVS_KEY_LNS_PORT, lns_port );
    if( err == ESP_OK )
    {
        printf( "Done\n" );
    }
    else
    {
        printf( "Failed\n" );
        nvs_close( my_handle );
        printf( "Closed NVS handle for writing.\n" );
        return ESP_FAIL;
    }

    printf( "NVS <- %s = %s ... ", CFG_NVS_KEY_SNTP_ADDRESS, sntp_addr );
    err = nvs_set_str( my_handle, CFG_NVS_KEY_SNTP_ADDRESS, sntp_addr );
    if( err == ESP_OK )
    {
        printf( "Done\n" );
    }
    else
    {
        printf( "Failed\n" );
        nvs_close( my_handle );
        printf( "Closed NVS handle for writing.\n" );
        return ESP_FAIL;
    }

    printf( "NVS <- %s = %" PRIu32 " ... ", CFG_NVS_KEY_CHAN_FREQ, chan_freq );
    err = nvs_set_u32( my_handle, CFG_NVS_KEY_CHAN_FREQ, chan_freq );
    if( err == ESP_OK )
    {
        printf( "Done\n" );
    }
    else
    {
        printf( "Failed\n" );
        nvs_close( my_handle );
        printf( "Closed NVS handle for writing.\n" );
        return ESP_FAIL;
    }

    printf( "NVS <- %s = %" PRIu32 " ... ", CFG_NVS_KEY_CHAN_DR, chan_dr );
    err = nvs_set_u32( my_handle, CFG_NVS_KEY_CHAN_DR, chan_dr );
    if( err == ESP_OK )
    {
        printf( "Done\n" );
    }
    else
    {
        printf( "Failed\n" );
        nvs_close( my_handle );
        printf( "Closed NVS handle for writing.\n" );
        return ESP_FAIL;
    }

    printf( "NVS <- %s = %" PRIu16 " ... ", CFG_NVS_KEY_CHAN_BW, chan_bw );
    err = nvs_set_u16( my_handle, CFG_NVS_KEY_CHAN_BW, chan_bw );
    if( err == ESP_OK )
    {
        printf( "Done\n" );
    }
    else
    {
        printf( "Failed\n" );
        nvs_close( my_handle );
        printf( "Closed NVS handle for writing.\n" );
        return ESP_FAIL;
    }

    printf( "Committing updates in NVS ... " );
    err = nvs_commit( my_handle );
    if( err == ESP_OK )
    {
        printf( "Done\n" );
    }
    else
    {
        printf( "Failed\n" );
        nvs_close( my_handle );
        printf( "Closed NVS handle for writing.\n" );
        return ESP_FAIL;
    }

    nvs_close( my_handle );
    printf( "Closed NVS handle for writing.\n" );

    return ESP_OK;
}

static void user_config_print(void)
{
    ESP_LOGI(TAG,"wifi_ssid: %s", wifi_ssid);
    ESP_LOGI(TAG, "wifi_password: %s", wifi_password);
    ESP_LOGI(TAG, "gw_id: %016llX", gw_id);
    ESP_LOGI(TAG, "lns_addr: %s", lns_addr);
    ESP_LOGI(TAG, "lns_port: %d", lns_port);
    ESP_LOGI(TAG, "sntp_addr: %s", sntp_addr);
    ESP_LOGI(TAG, "chan_freq: %lu", chan_freq);
    ESP_LOGI(TAG, "chan_dr: %lu", chan_dr);
    ESP_LOGI(TAG, "chan_bw: %d", chan_bw);
}


static const char *print_auth_mode(int authmode)
{
    switch (authmode)
    {
        case WIFI_AUTH_OPEN:
            ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_OPEN");
            return "OPEN";
            break;
        case WIFI_AUTH_OWE:
            ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_OWE");
            return "UNKNOWN";
            break;
        case WIFI_AUTH_WEP:
            ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WEP");
            return "WEP";
            break;
        case WIFI_AUTH_WPA_PSK:
            ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA_PSK");
            return "WPA";
            break;
        case WIFI_AUTH_WPA2_PSK:
            ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA2_PSK");
            return "WPA2";
            break;
        case WIFI_AUTH_WPA_WPA2_PSK:
            ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA_WPA2_PSK");
            return "WPA/WPA2";
            break;
        case WIFI_AUTH_ENTERPRISE:
            ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_ENTERPRISE");
            return "UNKNOWN";
            break;
        case WIFI_AUTH_WPA3_PSK:
            ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA3_PSK");
            return "WPA3";
            break;
        case WIFI_AUTH_WPA2_WPA3_PSK:
            ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA2_WPA3_PSK");
            return "WPA2/WPA3";
            break;
        case WIFI_AUTH_WPA3_ENT_192:
            ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA3_ENT_192");
            return "WPA2/WPA3";
            break;
        case WIFI_AUTH_WPA3_EXT_PSK:
            ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA3_EXT_PSK");
            return "WPA2/WPA3";
            break;
        case WIFI_AUTH_WPA3_EXT_PSK_MIXED_MODE:
            ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA3_EXT_PSK_MIXED_MODE");
            return "WPA2/WPA3";
            break;
        default:
            ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_UNKNOWN");
            return "UNKNOWN";
            break;
    }
}

static void print_cipher_type(int pairwise_cipher, int group_cipher)
{
    switch (pairwise_cipher)
    {
        case WIFI_CIPHER_TYPE_NONE:
            ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_NONE");
            break;
        case WIFI_CIPHER_TYPE_WEP40:
            ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_WEP40");
            break;
        case WIFI_CIPHER_TYPE_WEP104:
            ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_WEP104");
            break;
        case WIFI_CIPHER_TYPE_TKIP:
            ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_TKIP");
            break;
        case WIFI_CIPHER_TYPE_CCMP:
            ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_CCMP");
            break;
        case WIFI_CIPHER_TYPE_TKIP_CCMP:
            ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_TKIP_CCMP");
            break;
        case WIFI_CIPHER_TYPE_AES_CMAC128:
            ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_AES_CMAC128");
            break;
        case WIFI_CIPHER_TYPE_SMS4:
            ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_SMS4");
            break;
        case WIFI_CIPHER_TYPE_GCMP:
            ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_GCMP");
            break;
        case WIFI_CIPHER_TYPE_GCMP256:
            ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_GCMP256");
            break;
        default:
            ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_UNKNOWN");
            break;
    }

    switch (group_cipher)
    {
        case WIFI_CIPHER_TYPE_NONE:
            ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_NONE");
            break;
        case WIFI_CIPHER_TYPE_WEP40:
            ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_WEP40");
            break;
        case WIFI_CIPHER_TYPE_WEP104:
            ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_WEP104");
            break;
        case WIFI_CIPHER_TYPE_TKIP:
            ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_TKIP");
            break;
        case WIFI_CIPHER_TYPE_CCMP:
            ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_CCMP");
            break;
        case WIFI_CIPHER_TYPE_TKIP_CCMP:
            ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_TKIP_CCMP");
            break;
        case WIFI_CIPHER_TYPE_SMS4:
            ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_SMS4");
            break;
        case WIFI_CIPHER_TYPE_GCMP:
            ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_GCMP");
            break;
        case WIFI_CIPHER_TYPE_GCMP256:
            ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_GCMP256");
            break;
        default:
            ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_UNKNOWN");
            break;
    }
}

uint8_t wifi_scan(void)
{
    esp_err_t ret;
    const int max_cnt = WIFI_SCAN_RESULT_CNT_MAX;
    wifi_ap_record_t *ap_info = heap_caps_calloc(1, sizeof(wifi_ap_record_t) * max_cnt, MALLOC_CAP_INTERNAL);
    uint16_t ap_cnt = max_cnt;

    memset(ap_info, 0, sizeof(wifi_ap_record_t) * WIFI_SCAN_RESULT_CNT_MAX);
    memset(wifi_ap_scan, 0, sizeof(wifi_info_t) * WIFI_SCAN_RESULT_CNT_MAX);

    ESP_GOTO_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), wifi_scan_end, TAG, "esp_wifi_set_mode failed");
    ESP_GOTO_ON_ERROR(esp_wifi_start(), wifi_scan_end, TAG, "esp_wifi_start failed");
    ESP_GOTO_ON_ERROR(esp_wifi_scan_start(NULL, true), wifi_scan_end, TAG, "esp_wifi_scan_start failed");
    ESP_GOTO_ON_ERROR(esp_wifi_scan_get_ap_records(&ap_cnt, ap_info), wifi_scan_end, TAG, "esp_wifi_scan_get_ap_records failed");
    ESP_LOGI(TAG, "Max number of APs want: %d, scanned = %d", max_cnt, ap_cnt);
    for (int i = 0; i < ap_cnt; i++)
    {
        ESP_LOGI(TAG, "SSID \t\t%s", ap_info[i].ssid);
        ESP_LOGI(TAG, "RSSI \t\t%d", ap_info[i].rssi);
        const char *encryption = print_auth_mode(ap_info[i].authmode);
        memcpy(wifi_ap_scan[i].ssid, ap_info[i].ssid, strlen((char *)ap_info[i].ssid));
        memcpy(wifi_ap_scan[i].authmode, encryption, strlen(encryption));
        wifi_ap_scan[i].rssi = ap_info[i].rssi;
        if (ap_info[i].authmode != WIFI_AUTH_WEP)
        {
            print_cipher_type(ap_info[i].pairwise_cipher, ap_info[i].group_cipher);
        }
        ESP_LOGI(TAG, "Channel \t\t%d", ap_info[i].primary);
    }
wifi_scan_end:
    free(ap_info);
    return ap_cnt;
}

static void user_cmd_proc_task(void *p_arg)
{
    while (1) {
        ble_msg_t ble_msg;
        if (xQueueReceive(ble_msg_queue, &ble_msg, portMAX_DELAY) != pdPASS) {
            ESP_LOGE(TAG, "Failed to receive ble message from queue");
            continue;
        }
        
        if (ble_msg.size)
        {
            ESP_LOGI(TAG, "%s", ble_msg.msg);

            if(strcmp((char *)(ble_msg.msg), LORA_CMD_CONFIG_GET) == 0)
            {
                user_config_load();

                char *json_str = NULL;
                cJSON *json = cJSON_CreateObject();

                char gw_id_str[20] = {0};
                snprintf(gw_id_str, sizeof gw_id_str, "%016llX", gw_id);
                cJSON_AddItemToObject(json, "gw_id", cJSON_CreateString(gw_id_str));

                cJSON_AddItemToObject(json, "lns_addr", cJSON_CreateString(lns_addr));
                cJSON_AddItemToObject(json, "lns_port", cJSON_CreateNumber(lns_port));
                cJSON_AddItemToObject(json, "sntp_addr", cJSON_CreateString(sntp_addr));

                cJSON_AddItemToObject(json, "chan_freq", cJSON_CreateNumber(chan_freq));
                cJSON_AddItemToObject(json, "chan_dr", cJSON_CreateNumber(chan_dr));
                cJSON_AddItemToObject(json, "chan_bw", cJSON_CreateNumber(chan_bw));

                json_str = cJSON_PrintUnformatted(json);
                cJSON_Delete(json);
                memset(cmd_rsp_buf, 0, sizeof(cmd_rsp_buf));
                memcpy(cmd_rsp_buf, json_str, strlen(json_str));
                memcpy(cmd_rsp_buf + strlen(json_str), cmd_result_ok, strlen(cmd_result_ok));
                free(json_str);

                ESP_LOGI(TAG, "%s", cmd_rsp_buf);
                bleprph_send_indicate((uint8_t *)cmd_rsp_buf, strlen((char*)cmd_rsp_buf));
            }
            else if(strcmp((char *)(ble_msg.msg), WIFI_CMD_CONFIG_GET) == 0)
            {
                user_config_load();

                

                char *json_str = NULL;
                cJSON *json = cJSON_CreateObject();

                cJSON_AddItemToObject(json, "wifi_ssid", cJSON_CreateString(wifi_ssid));
                cJSON_AddItemToObject(json, "wifi_password", cJSON_CreateString(wifi_password));
                wifi_status = wifi_get_status();
                cJSON_AddItemToObject(json, "wifi_status", cJSON_CreateNumber(wifi_status));
                int wifi_rssi = -128;
                if(wifi_status == 1)esp_wifi_sta_get_rssi(&wifi_rssi); // connected
                cJSON_AddItemToObject(json, "wifi_rssi", cJSON_CreateNumber(wifi_rssi));

                json_str = cJSON_PrintUnformatted(json);
                cJSON_Delete(json);
                memset(cmd_rsp_buf, 0, sizeof(cmd_rsp_buf));
                memcpy(cmd_rsp_buf, json_str, strlen(json_str));
                memcpy(cmd_rsp_buf + strlen(json_str), cmd_result_ok, strlen(cmd_result_ok));
                free(json_str);

                ESP_LOGI(TAG, "%s", cmd_rsp_buf);
                bleprph_send_indicate((uint8_t *)cmd_rsp_buf, strlen((char*)cmd_rsp_buf));
            }
            else if(strcmp((char *)(ble_msg.msg), WIFI_CMD_SCAN_X_GET) == 0)
            {
                uint8_t wifi_cnt = wifi_scan();
                if(wifi_cnt) {
                    char *json_str = NULL;
                    cJSON *json = cJSON_CreateObject();
                    cJSON *scanned_array = cJSON_CreateArray();

                    for(uint8_t i = 0; i < wifi_cnt; i++) {
                        cJSON *wifi_json = cJSON_CreateObject();
                        cJSON_AddItemToObject(wifi_json, "ssid", cJSON_CreateString((char *)wifi_ap_scan[i].ssid));
                        cJSON_AddItemToObject(wifi_json, "authmode", cJSON_CreateString((char *)wifi_ap_scan[i].authmode));
                        cJSON_AddItemToObject(wifi_json, "rssi", cJSON_CreateNumber(wifi_ap_scan[i].rssi));
                        cJSON_AddItemToArray(scanned_array, wifi_json);
                    }
                    cJSON_AddItemToObject(json, "scanned_wifi", scanned_array);

                    json_str = cJSON_PrintUnformatted(json);
                    cJSON_Delete(json);
                    memset(cmd_rsp_buf, 0, sizeof(cmd_rsp_buf));
                    memcpy(cmd_rsp_buf, json_str, strlen(json_str));
                    memcpy(cmd_rsp_buf + strlen(json_str), cmd_result_ok, strlen(cmd_result_ok));
                    free(json_str);

                    ESP_LOGI(TAG, "%s", cmd_rsp_buf);
                    bleprph_send_indicate((uint8_t *)cmd_rsp_buf, strlen((char*)cmd_rsp_buf));
                } else {
                    ESP_LOGE(TAG, "wifi scan error");
                }
            }
            else
            {
                char cmd_str[16] = {0};
                memcpy(cmd_str, ble_msg.msg, XXXX_CMD_SET_LEN);
                if(strcmp(cmd_str, LORA_CMD_CONFIG_SET) == 0)
                {
                    cJSON *p_json = cJSON_Parse((char *)(ble_msg.msg + XXXX_CMD_SET_LEN));
                    if (p_json) {
                        char gw_id_str[20] = {0};
                        cJSON *json_gw_id = cJSON_GetObjectItem(p_json, "gw_id");
                        if (json_gw_id != NULL && cJSON_IsString(json_gw_id)) {
                            memcpy(gw_id_str, json_gw_id->valuestring, strlen(json_gw_id->valuestring));
                            gw_id = (uint64_t)strtoull(gw_id_str, NULL, 16);
                        }

                        cJSON *json_lns_addr = cJSON_GetObjectItem(p_json, "lns_addr");
                        if (json_lns_addr != NULL && cJSON_IsString(json_lns_addr)) {
                            memset(lns_addr, 0, sizeof lns_addr);
                            memcpy(lns_addr, json_lns_addr->valuestring, strlen(json_lns_addr->valuestring));
                        }

                        cJSON *json_lns_port = cJSON_GetObjectItem(p_json, "lns_port");
                        if (json_lns_port != NULL && cJSON_IsNumber(json_lns_port)) {
                            lns_port = json_lns_port->valueint;
                        }
                        
                        cJSON *json_sntp_addr = cJSON_GetObjectItem(p_json, "sntp_addr");
                        if (json_sntp_addr != NULL  && cJSON_IsString(json_sntp_addr)) {
                            memset(sntp_addr, 0, sizeof sntp_addr);
                            memcpy(sntp_addr, json_sntp_addr->valuestring, strlen(json_sntp_addr->valuestring));
                        }

                        cJSON *json_chan_freq = cJSON_GetObjectItem(p_json, "chan_freq");
                        if (json_chan_freq != NULL && cJSON_IsNumber(json_chan_freq)) {
                            chan_freq = json_chan_freq->valueint;
                        }

                        cJSON *json_chan_dr = cJSON_GetObjectItem(p_json, "chan_dr");
                        if (json_chan_dr != NULL && cJSON_IsNumber(json_chan_dr)) {
                            chan_dr = json_chan_dr->valueint;
                        }

                        cJSON *json_chan_bw = cJSON_GetObjectItem(p_json, "chan_bw");
                        if (json_chan_bw != NULL && cJSON_IsNumber(json_chan_bw)) {
                            chan_bw = json_chan_bw->valueint;
                        }
                    } else {
                        ESP_LOGE(TAG, "lora config json parse fail");
                    }

                    cJSON_Delete(p_json);

                    esp_err_t err = user_lora_config_save();
                    if(err == ESP_OK) {
                        bleprph_send_indicate((uint8_t *)cmd_result_ok, strlen(cmd_result_ok));
                    } else {
                        bleprph_send_indicate((uint8_t *)cmd_result_err, strlen(cmd_result_err));
                    }
                }
                else if(strcmp(cmd_str, WIFI_CMD_CONFIG_SET) == 0)
                {                    
                    cJSON *p_json = cJSON_Parse((char *)(ble_msg.msg + XXXX_CMD_SET_LEN));
                    if (p_json) {
                        cJSON *json_wifi_ssid = cJSON_GetObjectItem(p_json, "wifi_ssid");
                        if (json_wifi_ssid != NULL && cJSON_IsString(json_wifi_ssid)) {
                            memset(wifi_ssid, 0, sizeof wifi_ssid);
                            memcpy(wifi_ssid, json_wifi_ssid->valuestring, strlen(json_wifi_ssid->valuestring));
                        }

                        cJSON *json_wifi_password = cJSON_GetObjectItem(p_json, "wifi_password");
                        if (json_wifi_password != NULL && cJSON_IsString(json_wifi_password)) {
                            memset(wifi_password, 0, sizeof wifi_password);
                            memcpy(wifi_password, json_wifi_password->valuestring, strlen(json_wifi_password->valuestring));
                        }

                        cJSON_Delete(p_json);

                        esp_err_t err = user_wifi_config_save();
                        if(err == ESP_OK) {
                            bleprph_send_indicate((uint8_t *)cmd_result_ok, strlen(cmd_result_ok));
                        } else {
                            bleprph_send_indicate((uint8_t *)cmd_result_err, strlen(cmd_result_err));
                        }

                        wifi_config_t wifi_config = {0};
                        memcpy(wifi_config.sta.ssid, wifi_ssid, strlen(wifi_ssid));
                        memcpy(wifi_config.sta.password, wifi_password, strlen(wifi_password));
                        esp_wifi_stop( );
                        vTaskDelay(pdMS_TO_TICKS(2000));
                        esp_wifi_set_mode( WIFI_MODE_STA );
                        esp_wifi_set_config( WIFI_IF_STA, &wifi_config );
                        esp_wifi_start( );
                    } else {
                        ESP_LOGE(TAG, "wifi config json parse fail");
                    }
                }
                else
                {
                    ESP_LOGE(TAG, "command unknow");
                }
            }
        }
    }
}

bool user_cmd_init(void)
{
    bool config = user_config_load();

    ble_msg_queue = xQueueCreate(10, sizeof(ble_msg_t));

    const uint32_t stack_size = 10 * 1024;
    StackType_t *task_stack1 = (StackType_t *)heap_caps_calloc(1, stack_size * sizeof(StackType_t), MALLOC_CAP_INTERNAL);
    StaticTask_t *task_tcb1 = heap_caps_calloc(1, sizeof(StaticTask_t), MALLOC_CAP_INTERNAL);
    xTaskCreateStatic(user_cmd_proc_task, "user_cmd", stack_size, NULL, 2, task_stack1, task_tcb1);

    return config;
}
