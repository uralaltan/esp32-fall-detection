#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "imu.h"

#define PROFILE_NUM                 1
#define IMU_PROFILE_APP_ID          0
#define IMU_SVC_UUID                0x1820
#define IMU_CHAR_UUID               0x2A5F
#define IMU_NUM_HANDLE              4
#define ADV_CONFIG_FLAG             (1 << 0)

static const char *GATTS_TAG = "IMU_GATTS";

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

static void imu_gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                            esp_ble_gatts_cb_param_t *param);

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

static void example_write_event_env(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);


static uint8_t imu_data[12];
static bool imu_indicate_enabled = false;
static bool imu_svc_created = false;
static uint8_t adv_config_done = 0;

static uint8_t adv_service_uuid_16[2] = {
    (IMU_SVC_UUID & 0xFF),
    (IMU_SVC_UUID >> 8) & 0xFF
};

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 0,
    .p_service_uuid = NULL,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [IMU_PROFILE_APP_ID] = {
        .gatts_cb = imu_gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,
        .app_id = IMU_PROFILE_APP_ID,
        .conn_id = 0xFFFF,
    }
};

static void imu_gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                            esp_ble_gatts_cb_param_t *param) {
    esp_err_t ret;
    switch (event) {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(GATTS_TAG, "IMU Profile: GATT server register, status %d, app_id %d", param->reg.status,
                     param->reg.app_id);
            if (param->reg.status == ESP_GATT_OK) {
                gl_profile_tab[IMU_PROFILE_APP_ID].gatts_if = gatts_if;
            } else {
                ESP_LOGE(GATTS_TAG, "IMU Profile: Reg app failed, app_id %04x, status %d",
                         param->reg.app_id, param->reg.status);
                return;
            }

            gl_profile_tab[IMU_PROFILE_APP_ID].service_id.is_primary = true;
            gl_profile_tab[IMU_PROFILE_APP_ID].service_id.id.inst_id = 0x00;
            gl_profile_tab[IMU_PROFILE_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
            gl_profile_tab[IMU_PROFILE_APP_ID].service_id.id.uuid.uuid.uuid16 = IMU_SVC_UUID;

            adv_config_done = ADV_CONFIG_FLAG;
            ret = esp_ble_gap_config_adv_data(&adv_data);
            if (ret) {
                ESP_LOGE(GATTS_TAG, "IMU Profile: Config adv data failed, error code = %x", ret);
            }

            esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[IMU_PROFILE_APP_ID].service_id, IMU_NUM_HANDLE);
            break;

        case ESP_GATTS_CREATE_EVT:
            ESP_LOGI(GATTS_TAG, "IMU Profile: Service create, status %d, service_handle %d", param->create.status,
                     param->create.service_handle);
            if (param->create.status == ESP_GATT_OK) {
                gl_profile_tab[IMU_PROFILE_APP_ID].service_handle = param->create.service_handle;
                esp_ble_gatts_start_service(param->create.service_handle);

                gl_profile_tab[IMU_PROFILE_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
                gl_profile_tab[IMU_PROFILE_APP_ID].char_uuid.uuid.uuid16 = IMU_CHAR_UUID;
                gl_profile_tab[IMU_PROFILE_APP_ID].property =
                        ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_INDICATE;
                gl_profile_tab[IMU_PROFILE_APP_ID].perm = ESP_GATT_PERM_READ;

                esp_attr_value_t imu_char_val = {
                    .attr_max_len = sizeof(imu_data),
                    .attr_len = sizeof(imu_data),
                    .attr_value = imu_data
                };

                ret = esp_ble_gatts_add_char(
                    gl_profile_tab[IMU_PROFILE_APP_ID].service_handle,
                    &gl_profile_tab[IMU_PROFILE_APP_ID].char_uuid,
                    gl_profile_tab[IMU_PROFILE_APP_ID].perm,
                    gl_profile_tab[IMU_PROFILE_APP_ID].property,
                    &imu_char_val,
                    NULL
                );
                if (ret) {
                    ESP_LOGE(GATTS_TAG, "IMU Profile: Add char failed, error code = %x", ret);
                }
            }
            break;

        case ESP_GATTS_ADD_CHAR_EVT:
            ESP_LOGI(GATTS_TAG, "IMU Profile: Characteristic add, status %d, attr_handle %d, char_uuid %x",
                     param->add_char.status, param->add_char.attr_handle, param->add_char.char_uuid.uuid.uuid16);
            if (param->add_char.status == ESP_GATT_OK) {
                gl_profile_tab[IMU_PROFILE_APP_ID].char_handle = param->add_char.attr_handle;

                gl_profile_tab[IMU_PROFILE_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
                gl_profile_tab[IMU_PROFILE_APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
                esp_attr_value_t cccd_val = {
                    .attr_max_len = 2,
                    .attr_len = 2,
                    .attr_value = (uint8_t[]){0x00, 0x00}
                };
                ret = esp_ble_gatts_add_char_descr(
                    gl_profile_tab[IMU_PROFILE_APP_ID].service_handle,
                    &gl_profile_tab[IMU_PROFILE_APP_ID].descr_uuid,
                    ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                    &cccd_val,
                    NULL
                );
                if (ret) {
                    ESP_LOGE(GATTS_TAG, "IMU Profile: Add char descr failed, error code = %x", ret);
                }
            }
            break;

        case ESP_GATTS_ADD_CHAR_DESCR_EVT:
            ESP_LOGI(GATTS_TAG, "IMU Profile: Descriptor add, status %d, attr_handle %u",
                     param->add_char_descr.status, param->add_char_descr.attr_handle);
            if (param->add_char_descr.status == ESP_GATT_OK) {
                gl_profile_tab[IMU_PROFILE_APP_ID].descr_handle = param->add_char_descr.attr_handle;
                imu_svc_created = true;
                ESP_LOGI(GATTS_TAG, "IMU Service created successfully.");
            }
            break;

        case ESP_GATTS_READ_EVT:
            ESP_LOGI(GATTS_TAG,
                     "IMU Profile: Characteristic read, conn_id %d, trans_id %" PRIu32 ", handle %d, need_rsp %d",
                     param->read.conn_id, param->read.trans_id, param->read.handle, param->read.need_rsp);
            break;

        case ESP_GATTS_WRITE_EVT:
            ESP_LOGI(GATTS_TAG,
                     "IMU Profile: Characteristic write, conn_id %d, trans_id %" PRIu32
                     ", handle %d, len %d, is_prep %d, need_rsp %d",
                     param->write.conn_id, param->write.trans_id, param->write.handle, param->write.len,
                     param->write.is_prep, param->write.need_rsp);
            ESP_LOG_BUFFER_HEX(GATTS_TAG, param->write.value, param->write.len);

            if (!param->write.is_prep && param->write.handle == gl_profile_tab[IMU_PROFILE_APP_ID].descr_handle && param
                ->write.len == 2) {
                uint16_t ccc_val = param->write.value[1] << 8 | param->write.value[0];
                if (ccc_val == 0x0002) {
                    ESP_LOGI(GATTS_TAG, "IMU indications enabled");
                    imu_indicate_enabled = true;
                } else if (ccc_val == 0x0001) {
                    ESP_LOGI(GATTS_TAG,
                             "IMU notifications enabled (char uses indications, but client might write this)");
                    imu_indicate_enabled = false;
                } else if (ccc_val == 0x0000) {
                    ESP_LOGI(GATTS_TAG, "IMU indications/notifications disabled");
                    imu_indicate_enabled = false;
                } else {
                    ESP_LOGW(GATTS_TAG, "IMU CCCD: Unknown value written: 0x%04x", ccc_val);
                }
            }
            example_write_event_env(gatts_if, param);
            break;

        case ESP_GATTS_SET_ATTR_VAL_EVT:
            ESP_LOGD(GATTS_TAG, "IMU Profile: Attribute value set event, status %d, handle %d",
                     param->set_attr_val.status, param->set_attr_val.attr_handle);
            if (param->set_attr_val.status == ESP_GATT_OK &&
                param->set_attr_val.attr_handle == gl_profile_tab[IMU_PROFILE_APP_ID].char_handle &&
                imu_indicate_enabled &&
                gl_profile_tab[IMU_PROFILE_APP_ID].conn_id != 0xFFFF) {
                ESP_LOGD(GATTS_TAG, "Sending IMU indication, conn_id %d", gl_profile_tab[IMU_PROFILE_APP_ID].conn_id);
                esp_ble_gatts_send_indicate(
                    gatts_if,
                    gl_profile_tab[IMU_PROFILE_APP_ID].conn_id,
                    gl_profile_tab[IMU_PROFILE_APP_ID].char_handle,
                    sizeof(imu_data),
                    imu_data,
                    true
                );
            }
            break;

        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(GATTS_TAG, "IMU Profile: Connected, conn_id %u, remote "ESP_BD_ADDR_STR"",
                     param->connect.conn_id, ESP_BD_ADDR_HEX(param->connect.remote_bda));
            gl_profile_tab[IMU_PROFILE_APP_ID].conn_id = param->connect.conn_id;

            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            conn_params.latency = 0;
            conn_params.max_int = 0x20;
            conn_params.min_int = 0x10;
            conn_params.timeout = 400;
            esp_ble_gap_update_conn_params(&conn_params);
            break;

        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(GATTS_TAG,
                     "IMU Profile: Disconnected, conn_id %u (was %u), remote "ESP_BD_ADDR_STR", reason 0x%02x",
                     param->disconnect.conn_id, gl_profile_tab[IMU_PROFILE_APP_ID].conn_id,
                     ESP_BD_ADDR_HEX(param->disconnect.remote_bda), param->disconnect.reason);
            imu_indicate_enabled = false;
            gl_profile_tab[IMU_PROFILE_APP_ID].conn_id = 0xFFFF;
            esp_ble_gap_start_advertising(&adv_params);
            break;

        case ESP_GATTS_CONF_EVT:
            ESP_LOGI(GATTS_TAG, "IMU Profile: Confirm receive (indication ack), status %d, handle %d, conn_id %d",
                     param->conf.status, param->conf.handle, param->conf.conn_id);
            if (param->conf.status != ESP_GATT_OK) {
                ESP_LOGE(GATTS_TAG, "IMU Profile: Indication confirmation failed, status %d", param->conf.status);
            }
            break;

        case ESP_GATTS_START_EVT:
            ESP_LOGI(GATTS_TAG, "IMU Profile: Service start, status %d, service_handle %d", param->start.status,
                     param->start.service_handle);
            break;
        case ESP_GATTS_STOP_EVT:
            ESP_LOGI(GATTS_TAG, "IMU Profile: Service stop, status %d, service_handle %d", param->stop.status,
                     param->stop.service_handle);
            break;
        case ESP_GATTS_DELETE_EVT:
            // ** THIS IS THE LINE IN QUESTION. VERIFY 'delete_svc' AGAINST YOUR ESP-IDF's esp_gatts_api.h **
            ESP_LOGI(GATTS_TAG, "IMU Profile: Service deleted, status %d, service_handle %d",
                     param->del.status, param->del.service_handle);
            break;
        default:
            ESP_LOGD(GATTS_TAG, "IMU Profile: Unhandled event %d", event);
            break;
    }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            ESP_LOGI(GATTS_TAG, "Advertising data set complete, status %d", param->adv_data_cmpl.status);
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0) {
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TAG, "Advertising start failed, status %d", param->adv_start_cmpl.status);
            } else {
                ESP_LOGI(GATTS_TAG, "Advertising started successfully");
            }
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TAG, "Advertising stop failed, status %d", param->adv_stop_cmpl.status);
            } else {
                ESP_LOGI(GATTS_TAG, "Advertising stopped successfully");
            }
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(GATTS_TAG,
                     "Connection parameters update, status %d, min_int %d, max_int %d, conn_int %d, latency %d, timeout %d",
                     param->update_conn_params.status,
                     param->update_conn_params.min_int,
                     param->update_conn_params.max_int,
                     param->update_conn_params.conn_int,
                     param->update_conn_params.latency,
                     param->update_conn_params.timeout);
            break;
        case ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT:
            ESP_LOGI(GATTS_TAG, "Packet length update, status %d, rx_len %d, tx_len %d",
                     param->pkt_data_length_cmpl.status,
                     param->pkt_data_length_cmpl.params.rx_len,
                     param->pkt_data_length_cmpl.params.tx_len);
            break;
        default:
            ESP_LOGD(GATTS_TAG, "GAP Unhandled event %d", event);
            break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            if (param->reg.app_id < PROFILE_NUM) {
                gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
            } else {
                ESP_LOGE(GATTS_TAG, "gatts_event_handler: Unknown app_id %d during REG_EVT", param->reg.app_id);
            }
        } else {
            ESP_LOGE(GATTS_TAG, "GATT server registration failed for app_id %04x, status %d",
                     param->reg.app_id, param->reg.status);
            return;
        }
    }

    for (int idx = 0; idx < PROFILE_NUM; idx++) {
        if (gatts_if == ESP_GATT_IF_NONE || gatts_if == gl_profile_tab[idx].gatts_if) {
            if (gl_profile_tab[idx].gatts_cb) {
                gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
            }
        }
    }
}

static void imu_task(void *pvParameters) {
    ESP_LOGI(GATTS_TAG, "IMU Task Started");

    ESP_LOGI(GATTS_TAG, "Initializing IMU sensor…");
    ESP_ERROR_CHECK(imu_init());

    while (!imu_svc_created) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGI(GATTS_TAG, "IMU Service ready, proceeding with IMU task loop.");

    while (1) {
        imu_read_and_print();

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void app_main(void) {
    esp_err_t ret;

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(GATTS_TAG, "IMU init function call placeholder. Implement imu_init() if needed.");


    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s init bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "GAP register callback error, code = %x", ret);
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "GATTS register callback error, code = %x", ret);
        return;
    }

    ret = esp_ble_gatts_app_register(IMU_PROFILE_APP_ID);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "IMU app register error, code = %x", ret);
        return;
    }

    ret = esp_ble_gatt_set_local_mtu(200);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "Set local MTU failed, error code = %x", ret);
    }

    const char *device_name = "ESP32_IMU_SENSOR";
    ret = esp_ble_gap_set_device_name(device_name);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "Set device name failed, error code = %x", ret);
    }

    xTaskCreate(imu_task, "IMU_Task", 4096, NULL, 5, NULL);

    ESP_LOGI(GATTS_TAG, "BLE IMU Sensor Initialized and Started.");
}

void example_write_event_env(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    if (param->write.need_rsp) {
        if (!param->write.is_prep) {
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
            ESP_LOGD(GATTS_TAG, "Sent response for write event: conn_id %d, trans_id %"PRIu32, param->write.conn_id,
                     param->write.trans_id);
        } else {
            ESP_LOGW(GATTS_TAG,
                     "Prepared write received, but not fully handled in example_write_event_env. Need ESP_GATTS_EXEC_WRITE_EVT logic.")
            ;
        }
    }
}
