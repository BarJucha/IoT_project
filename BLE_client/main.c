#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "esp_err.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#define GATTC_TAG "GATTC_CLIENT"
#define REMOTE_SERVICE_UUID        0x180F
#define REMOTE_NOTIFY_CHAR_UUID    0x2A19

#define WRITE_ALERT_SERVICE_UUID   0x1802
#define WRITE_ALERT_CHAR_UUID   0x2A06


#define PROFILE_NUM      2
#define PROFILE_A_APP_ID 0
#define PROFILE_B_APP_ID 1
#define INVALID_HANDLE   0

static const char remote_device_name[] = "iTAG  ";
static bool connect    = false;
static bool get_server_battery = false;
static bool get_server_alert = false;
static esp_gattc_char_elem_t *char_elem_result   = NULL;
static uint16_t battery_level_handle = 0;
static uint16_t alert_level_handle = 0;
TaskHandle_t xHandleAlert = NULL;

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_battery_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
//static void gattc_alert_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
void alert_level_write_task(void *param);


static esp_bt_uuid_t remote_filter_service_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = REMOTE_SERVICE_UUID,},
};

static esp_bt_uuid_t remote_filter_char_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = REMOTE_NOTIFY_CHAR_UUID,},
};

static esp_bt_uuid_t alert_service_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = WRITE_ALERT_SERVICE_UUID,},
};

static esp_bt_uuid_t alert_char_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = WRITE_ALERT_CHAR_UUID,},
};

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};

struct gattc_profile_inst {
    esp_gattc_cb_t gattc_cb;
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t char_handle;
    esp_bd_addr_t remote_bda;
};

static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gattc_cb = gattc_battery_event_handler,
        .gattc_if = ESP_GATT_IF_NONE,       
    },
    [PROFILE_B_APP_ID] = {
        .gattc_cb = gattc_battery_event_handler,
        .gattc_if = ESP_GATT_IF_NONE,       
    },
};

static void gattc_battery_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

    switch (event) {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(GATTC_TAG, "REG_EVT");
        esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
        if (scan_ret){
            ESP_LOGE(GATTC_TAG, "set scan params error, error code = %x", scan_ret);
        }
        break;
    case ESP_GATTC_CONNECT_EVT:{
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_CONNECT_EVT conn_id %d, if %d", p_data->connect.conn_id, gattc_if);
        gl_profile_tab[PROFILE_A_APP_ID].conn_id = p_data->connect.conn_id;
        memcpy(gl_profile_tab[PROFILE_A_APP_ID].remote_bda, p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(GATTC_TAG, "REMOTE BDA:");
        esp_log_buffer_hex(GATTC_TAG, gl_profile_tab[PROFILE_A_APP_ID].remote_bda, sizeof(esp_bd_addr_t));
        esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req (gattc_if, p_data->connect.conn_id);
        if (mtu_ret){
            ESP_LOGE(GATTC_TAG, "config MTU error, error code = %x", mtu_ret);
        }
        break;
    }
    case ESP_GATTC_OPEN_EVT:
        if (param->open.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "open failed, status %d", p_data->open.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "open success");
        break;
    case ESP_GATTC_DIS_SRVC_CMPL_EVT:
        if (param->dis_srvc_cmpl.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "discover service failed, status %d", param->dis_srvc_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "discover service complete conn_id %d", param->dis_srvc_cmpl.conn_id);
        esp_ble_gattc_search_service(gattc_if, param->dis_srvc_cmpl.conn_id, &remote_filter_service_uuid);
        esp_ble_gattc_search_service(gattc_if, param->dis_srvc_cmpl.conn_id, &alert_service_uuid);

        break;
    case ESP_GATTC_SEARCH_RES_EVT: {
        if (p_data->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16 && p_data->search_res.srvc_id.uuid.uuid.uuid16 == REMOTE_SERVICE_UUID) {
            ESP_LOGI(GATTC_TAG, "service battery found");
            get_server_battery = true;
            gl_profile_tab[PROFILE_A_APP_ID].service_start_handle = p_data->search_res.start_handle;
            gl_profile_tab[PROFILE_A_APP_ID].service_end_handle = p_data->search_res.end_handle;
            ESP_LOGI(GATTC_TAG, "UUID16: %x", p_data->search_res.srvc_id.uuid.uuid.uuid16);
        }
        if (p_data->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16 && p_data->search_res.srvc_id.uuid.uuid.uuid16 == WRITE_ALERT_SERVICE_UUID) {
            ESP_LOGI(GATTC_TAG, "service alert found");
            get_server_alert = true;
            gl_profile_tab[PROFILE_B_APP_ID].service_start_handle = p_data->search_res.start_handle;
            gl_profile_tab[PROFILE_B_APP_ID].service_end_handle = p_data->search_res.end_handle;
            ESP_LOGI(GATTC_TAG, "UUID16: %x", p_data->search_res.srvc_id.uuid.uuid.uuid16);
        }
        break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT:
		if (p_data->search_cmpl.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "search service failed, error status = %x", p_data->search_cmpl.status);
            break;
        }
        if (get_server_battery && !get_server_alert){
			uint16_t count = 0;
            esp_gatt_status_t status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                     p_data->search_cmpl.conn_id,
                                                                     ESP_GATT_DB_CHARACTERISTIC,
                                                                     gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                                     gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                                     INVALID_HANDLE,
                                                                     &count);
            if (status != ESP_GATT_OK){
                ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
                break;
            }

            if (count > 0){
                char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
                if (!char_elem_result){
                    ESP_LOGE(GATTC_TAG, "gattc no mem");
                    break;
                }else{
                    status = esp_ble_gattc_get_char_by_uuid( gattc_if,
                                                             p_data->search_cmpl.conn_id,
                                                             gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                             gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                             remote_filter_char_uuid,
                                                             char_elem_result,
                                                             &count);
                    if (status != ESP_GATT_OK){
                        ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_char_by_uuid error");
                        free(char_elem_result);
                        char_elem_result = NULL;
                        break;
                    }

                    if (count > 0 && (char_elem_result[0].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY)){
						battery_level_handle = char_elem_result->char_handle;
						esp_ble_gattc_read_char(gattc_if, p_data->search_cmpl.conn_id, battery_level_handle, ESP_GATT_AUTH_REQ_NONE);
                        gl_profile_tab[PROFILE_A_APP_ID].char_handle = char_elem_result[0].char_handle;
                        esp_ble_gattc_register_for_notify (gattc_if, gl_profile_tab[PROFILE_A_APP_ID].remote_bda, char_elem_result[0].char_handle);
                    }
                }
                free(char_elem_result);
            }else{
                ESP_LOGE(GATTC_TAG, "no battery char found");
            }
		}
		if (get_server_alert){
			uint16_t count = 0;
			esp_gatt_status_t status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                     p_data->search_cmpl.conn_id,
                                                                     ESP_GATT_DB_CHARACTERISTIC,
                                                                     gl_profile_tab[PROFILE_B_APP_ID].service_start_handle,
                                                                     gl_profile_tab[PROFILE_B_APP_ID].service_end_handle,
                                                                     INVALID_HANDLE,
                                                                     &count);

            if (count > 0){
                char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
                if (!char_elem_result){
                    ESP_LOGE(GATTC_TAG, "gattc no mem");
                    break;
                }else{
                    status = esp_ble_gattc_get_char_by_uuid( gattc_if,
                                                             p_data->search_cmpl.conn_id,
                                                             gl_profile_tab[PROFILE_B_APP_ID].service_start_handle,
                                                             gl_profile_tab[PROFILE_B_APP_ID].service_end_handle,
                                                             alert_char_uuid,
                                                             char_elem_result,
                                                             &count);
                    if (count > 0) {
		                alert_level_handle = char_elem_result[0].char_handle;
		                ESP_LOGI(GATTC_TAG, "Alert Level characteristic found, handle: %d", alert_level_handle);
		                xTaskCreate(alert_level_write_task, "alert_level_write_task", 2048, (void *)gattc_if, 5, &xHandleAlert);
		            }
		            else{ESP_LOGE(GATTC_TAG, "no count get_char");}
                }
                free(char_elem_result);
            }
            else{ESP_LOGE(GATTC_TAG, "status count %d", count);}
		}
         break;
    case ESP_GATTC_READ_CHAR_EVT:
        if (param->read.status == ESP_GATT_OK) {
            ESP_LOGI(GATTC_TAG, "Odczytana wartosc poziomu baterii: %d%%", param->read.value[0]);
        } else {
            ESP_LOGE(GATTC_TAG, "Blad odczytu charakterystyki Battery Level, status: %d", param->read.status);
        }
        break;
    case ESP_GATTC_DISCONNECT_EVT:
        connect = false;
        get_server_battery = false;
        get_server_alert = false;
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_DISCONNECT_EVT, reason = %d", p_data->disconnect.reason);
        vTaskDelete(xHandleAlert);
        break;
    default:
        break;
    }
}

/*static void gattc_alert_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

    switch (event) {
    case ESP_GATTC_CONNECT_EVT:{
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_CONNECT_EVT ALERT conn_id %d, if %d", p_data->connect.conn_id, gattc_if);
        gl_profile_tab[PROFILE_B_APP_ID].conn_id = p_data->connect.conn_id;
        memcpy(gl_profile_tab[PROFILE_B_APP_ID].remote_bda, p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
        esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req (gattc_if, p_data->connect.conn_id);
        if (mtu_ret){
            ESP_LOGE(GATTC_TAG, "config MTU error, error code = %x", mtu_ret);
        }
        esp_ble_gattc_search_service(gattc_if, param->dis_srvc_cmpl.conn_id, &alert_service_uuid);
        break;
    }
    case ESP_GATTC_OPEN_EVT:
        if (param->open.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "open failed, status %d", p_data->open.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "open success");
        break;
    case ESP_GATTC_DIS_SRVC_CMPL_EVT:
        if (param->dis_srvc_cmpl.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "discover service failed, status %d", param->dis_srvc_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "discover service complete conn_id %d if %d", param->dis_srvc_cmpl.conn_id, gattc_if);
        esp_ble_gattc_search_service(gattc_if, param->dis_srvc_cmpl.conn_id, &alert_service_uuid);
        break;
    case ESP_GATTC_SEARCH_RES_EVT: {
        if (p_data->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16 && p_data->search_res.srvc_id.uuid.uuid.uuid16 == WRITE_ALERT_SERVICE_UUID) {
            ESP_LOGI(GATTC_TAG, "alert service found");
            get_server = true;
            gl_profile_tab[PROFILE_B_APP_ID].service_start_handle = p_data->search_res.start_handle;
            gl_profile_tab[PROFILE_B_APP_ID].service_end_handle = p_data->search_res.end_handle;
            ESP_LOGI(GATTC_TAG, "UUID16: %x", p_data->search_res.srvc_id.uuid.uuid.uuid16);
        }
        else{
			ESP_LOGE(GATTC_TAG, "Servis alert not found");
		}
        break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT:
        if (p_data->search_cmpl.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "search service failed, error status = %x", p_data->search_cmpl.status);
            break;
        }
        if (get_server){
            uint16_t count = 0;
            esp_gatt_status_t status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                     p_data->search_cmpl.conn_id,
                                                                     ESP_GATT_DB_CHARACTERISTIC,
                                                                     gl_profile_tab[PROFILE_B_APP_ID].service_start_handle,
                                                                     gl_profile_tab[PROFILE_B_APP_ID].service_end_handle,
                                                                     INVALID_HANDLE,
                                                                     &count);
            if (status != ESP_GATT_OK){
                ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
                break;
            }

            if (count > 0){
                char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
                if (!char_elem_result){
                    ESP_LOGE(GATTC_TAG, "gattc no mem");
                    break;
                }else{
                    status = esp_ble_gattc_get_char_by_uuid( gattc_if,
                                                             p_data->search_cmpl.conn_id,
                                                             gl_profile_tab[PROFILE_B_APP_ID].service_start_handle,
                                                             gl_profile_tab[PROFILE_B_APP_ID].service_end_handle,
                                                             alert_char_uuid,
                                                             char_elem_result,
                                                             &count);
                    if (status != ESP_GATT_OK){
                        ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_char_by_uuid error");
                        free(char_elem_result);
                        char_elem_result = NULL;
                        break;
                    }

                    if (count > 0){
						alert_level_handle = char_elem_result->char_handle;
						gl_profile_tab[PROFILE_B_APP_ID].char_handle = char_elem_result[0].char_handle;
						xTaskCreate(alert_level_write_task, "alert_level_write_task", 2048, (void *)gattc_if, 5, NULL);
	                } else {
	                    ESP_LOGE(GATTC_TAG, "Nie znaleziono charakterystyki Alert Level");
	                }
                    }
                }
                free(char_elem_result);
            }else{
                ESP_LOGE(GATTC_TAG, "no char found");
            }
         break;
    case ESP_GATTC_DISCONNECT_EVT:
        connect = false;
        get_server = false;
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_DISCONNECT_EVT, reason = %d", p_data->disconnect.reason);
        break;
    default:
        break;
    }
}*/

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    uint8_t *adv_name = NULL;
    uint8_t adv_name_len = 0;
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        //the unit of the duration is second
        uint32_t duration = 10;
        esp_ble_gap_start_scanning(duration);
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        //scan start complete event to indicate scan start successfully or failed
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTC_TAG, "scan start failed, error status = %x", param->scan_start_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "scan start success");

        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
            esp_log_buffer_hex(GATTC_TAG, scan_result->scan_rst.bda, 6);
            ESP_LOGI(GATTC_TAG, "searched Adv Data Len %d, Scan Response Len %d", scan_result->scan_rst.adv_data_len, scan_result->scan_rst.scan_rsp_len);
            adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv,
                                                ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
            ESP_LOGI(GATTC_TAG, "searched Device Name Len %d", adv_name_len);
            esp_log_buffer_char(GATTC_TAG, adv_name, adv_name_len);

            ESP_LOGI(GATTC_TAG, " ");

            if (adv_name != NULL) {
                if (strlen(remote_device_name) == adv_name_len && strncmp((char *)adv_name, remote_device_name, adv_name_len) == 0) {
                    ESP_LOGI(GATTC_TAG, "searched device %s", remote_device_name);
                    if (connect == false) {
                        connect = true;
                        ESP_LOGI(GATTC_TAG, "connect to the remote device.");
                        esp_ble_gap_stop_scanning();
                        esp_ble_gattc_open(gl_profile_tab[PROFILE_A_APP_ID].gattc_if, scan_result->scan_rst.bda, scan_result->scan_rst.ble_addr_type, true);
                    }
                }
            }
            break;
        default:
            break;
        }
        break;
    }
    default:
        break;
    }
}

static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    /* If event is register event, store the gattc_if for each profile */
    if (event == ESP_GATTC_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
        } else {
            ESP_LOGI(GATTC_TAG, "reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    /* If the gattc_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gattc_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gattc_if == gl_profile_tab[idx].gattc_if) {
                if (gl_profile_tab[idx].gattc_cb) {
                    gl_profile_tab[idx].gattc_cb(event, gattc_if, param);
                }
            }
        }
    } while (0);
}

void alert_level_write_task(void *param) {
    esp_gatt_if_t gattc_if = (esp_gatt_if_t)param;
    
    int change_alert = 0;
    uint8_t alert_value = 0x00; 

    while (1) {
		if(!get_server_alert){
			break;
		}
        if (alert_level_handle != 0) {
			if (change_alert == 0){
				alert_value = 0x01;
				change_alert = 1;
			}
			else{
				alert_value = 0x00;
				change_alert = 0;
			}
            esp_err_t status = esp_ble_gattc_write_char(
                gattc_if,
                gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                alert_level_handle,
                sizeof(alert_value),
                &alert_value,
                ESP_GATT_WRITE_TYPE_RSP, // Typ zapisu
                ESP_GATT_AUTH_REQ_NONE   // Brak uwierzytelnienia
            );

            if (status == ESP_OK) {
                ESP_LOGI(GATTC_TAG, "Wyslano wartosc %d do charakterystyki Alert Level", alert_value);
            } else {
                ESP_LOGE(GATTC_TAG, "Blad wysylania do charakterystyki Alert Level: %s", esp_err_to_name(status));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5000)); // Co 5 sekund
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    // Initialize NVS.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));

    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

    ESP_ERROR_CHECK(esp_bluedroid_init());

    ESP_ERROR_CHECK(esp_bluedroid_enable());

    //register the  callback function to the gap module
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(esp_gap_cb));

    //register the callback function to the gattc module
    ESP_ERROR_CHECK(esp_ble_gattc_register_callback(esp_gattc_cb));

    ESP_ERROR_CHECK(esp_ble_gattc_app_register(PROFILE_A_APP_ID));
    //ESP_ERROR_CHECK(esp_ble_gattc_app_register(PROFILE_B_APP_ID));

    ESP_ERROR_CHECK(esp_ble_gatt_set_local_mtu(500));

}
