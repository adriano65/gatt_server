/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/****************************************************************************
*
* This demo showcases BLE GATT server. It can send adv data, be connected by client.
* Run the gatt_client demo, the client demo will automatically connect to the gatt_server demo.
* Client demo will enable gatt_server's notify after connection. The two devices will then exchange
* data.
*
****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>

#include <driver/uart.h>
#include <driver/gpio.h>

#include "esp_system.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "rfcomm.h"
#include "gatt_server.h"

#include "sdkconfig.h"

#define GATT_SRV_DBG_MIN
#ifdef GATT_SRV_DBG_MIN
  #define DBG_MIN(fmt...) do {printf("%s: ", __FUNCTION__); printf(fmt); printf("\n"); } while(0)
#else
  #define DBG_MIN(fmt...) do { } while(0)
#endif
//#define GATT_SRV_DBG_MAX
#ifdef GATT_SRV_DBG_MAX
  #define DBG_MAX(fmt...) do {printf("%s: ", __FUNCTION__); printf(fmt); printf("\n"); } while(0)
#else
  #define DBG_MAX(fmt...) do { } while(0)
#endif

static _gatts_data gatts_data;
const esp_partition_t* partition;

static prepare_type_env_t a_prepare_write_env;
static prepare_type_env_t b_prepare_write_env;

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
    #ifdef CONFIG_SET_RAW_ADV_DATA
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        adv_config_done &= (~adv_config_flag);
        if (adv_config_done==0){
          esp_ble_gap_start_advertising(&adv_params);
          DBG_MIN("ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT");
          }
        break;

    case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
        adv_config_done &= (~scan_rsp_config_flag);
        if (adv_config_done==0){
          esp_ble_gap_start_advertising(&adv_params);
          DBG_MIN("ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT");
          }
        break;
    #else
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~adv_config_flag);
        if (adv_config_done == 0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~scan_rsp_config_flag);
        if (adv_config_done == 0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    #endif
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        //advertising start complete event to indicate advertising start successfully or failed
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
          ESP_LOGE(GATTS_TAG, "Advertising start failed\n");
          }
        else { DBG_MIN("ESP_GAP_BLE_ADV_START_COMPLETE_EVT Advertising started successfully\n"); }
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
          ESP_LOGE(GATTS_TAG, "Advertising stop failed\n");
          } 
        else {
          DBG_MIN("ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT Stop adv successfully\n");
          }
        break;

    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
         DBG_MIN("ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d", param->update_conn_params.status, param->update_conn_params.min_int, param->update_conn_params.max_int, param->update_conn_params.conn_int, param->update_conn_params.latency, param->update_conn_params.timeout);
        break;
    default:
        break;
    }
}

void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
    esp_gatt_status_t status = ESP_GATT_OK;
    if (param->write.need_rsp){
        if (param->write.is_prep){
            if (prepare_write_env->prepare_buf == NULL) {
                prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE*sizeof(uint8_t));
                prepare_write_env->prepare_len = 0;
                if (prepare_write_env->prepare_buf == NULL) {
                    ESP_LOGE(GATTS_TAG, "Gatt_server prep no mem\n");
                    status = ESP_GATT_NO_RESOURCES;
                }
            } else {
                if(param->write.offset > PREPARE_BUF_MAX_SIZE) {
                    status = ESP_GATT_INVALID_OFFSET;
                } else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
                    status = ESP_GATT_INVALID_ATTR_LEN;
                }
            }

            esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
            gatt_rsp->attr_value.len = param->write.len;
            gatt_rsp->attr_value.handle = param->write.handle;
            gatt_rsp->attr_value.offset = param->write.offset;
            gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
            memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
            esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
            if (response_err != ESP_OK){
               ESP_LOGE(GATTS_TAG, "Send response error\n");
            }
            free(gatt_rsp);
            if (status != ESP_GATT_OK){
                return;
            }
            memcpy(prepare_write_env->prepare_buf + param->write.offset,
                   param->write.value,
                   param->write.len);
            prepare_write_env->prepare_len += param->write.len;

        }else{
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
        }
    }
}

void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC){
        esp_log_buffer_hex(GATTS_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
    }else{
        ESP_LOGI(GATTS_TAG,"ESP_GATT_PREP_WRITE_CANCEL");
    }
    if (prepare_write_env->prepare_buf) {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}

static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
  switch (event) {
    case ESP_GATTS_REG_EVT:
        DBG_MAX("REGISTER_APP_EVT, status %d, app_id %d\n", param->reg.status, param->reg.app_id);
        gl_profile_tab[PROFILE_A_APP_ID].service_id.is_primary = true;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.inst_id = 0x00;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_TEST_A;

        esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(gatts_data.ble_adv_name);
        if (set_dev_name_ret){
          ESP_LOGE(GATTS_TAG, "set device name failed, error code = %x", set_dev_name_ret);
          }
        #ifdef CONFIG_SET_RAW_ADV_DATA
        esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
        if (raw_adv_ret){
            ESP_LOGE(GATTS_TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
        }
        adv_config_done |= adv_config_flag;
        esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
        if (raw_scan_ret){
            ESP_LOGE(GATTS_TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
        }
        adv_config_done |= scan_rsp_config_flag;
        #else
        //config adv data
        esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
        if (ret){
          ESP_LOGE(GATTS_TAG, "config adv data failed, error code = %x", ret);
          }
        adv_config_done |= adv_config_flag;
        //config scan response data
        ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
        if (ret){
            ESP_LOGE(GATTS_TAG, "config scan response data failed, error code = %x", ret);
        }
        adv_config_done |= scan_rsp_config_flag;

        #endif
        esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_A_APP_ID].service_id, GATTS_NUM_HANDLE_TEST_A);
        break;

    case ESP_GATTS_READ_EVT: {
        ESP_LOGI(GATTS_TAG, "GATT_READ_EVT, conn_id %d, trans_id %d, handle %d\n", param->read.conn_id, param->read.trans_id, param->read.handle);
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.handle = param->read.handle;
        rsp.attr_value.len = 4;
        rsp.attr_value.value[0] = 0xde;
        rsp.attr_value.value[1] = 0xed;
        rsp.attr_value.value[2] = 0xbe;
        rsp.attr_value.value[3] = 0xef;
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
        }
        break;

    case ESP_GATTS_WRITE_EVT: {
        ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, conn_id %d, trans_id %d, handle %d", param->write.conn_id, param->write.trans_id, param->write.handle);
        if (!param->write.is_prep){
            ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, value len %d, value :", param->write.len);
            esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);
            if (gl_profile_tab[PROFILE_A_APP_ID].descr_handle == param->write.handle && param->write.len == 2){
                uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
                if (descr_value == 0x0001){
                    if (a_property & ESP_GATT_CHAR_PROP_BIT_NOTIFY){
                        ESP_LOGI(GATTS_TAG, "notify enable");
                        uint8_t notify_data[15];
                        for (int i = 0; i < sizeof(notify_data); ++i)
                        {
                            notify_data[i] = i%0xff;
                        }
                        //the size of notify_data[] need less than MTU size
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                                sizeof(notify_data), notify_data, false);
                    }
                }else if (descr_value == 0x0002){
                    if (a_property & ESP_GATT_CHAR_PROP_BIT_INDICATE){
                        ESP_LOGI(GATTS_TAG, "indicate enable");
                        uint8_t indicate_data[15];
                        for (int i = 0; i < sizeof(indicate_data); ++i)
                        {
                            indicate_data[i] = i%0xff;
                        }
                        //the size of indicate_data[] need less than MTU size
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                                sizeof(indicate_data), indicate_data, true);
                    }
                }
                else if (descr_value == 0x0000){
                    ESP_LOGI(GATTS_TAG, "notify/indicate disable ");
                }else{
                    ESP_LOGE(GATTS_TAG, "unknown descr value");
                    esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);
                }

            }
        }
        example_write_event_env(gatts_if, &a_prepare_write_env, param);
        break;
    }
    case ESP_GATTS_EXEC_WRITE_EVT:
        ESP_LOGI(GATTS_TAG,"ESP_GATTS_EXEC_WRITE_EVT");
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        example_exec_write_event_env(&a_prepare_write_env, param);
        break;
    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
        break;
    case ESP_GATTS_UNREG_EVT:
        break;
    case ESP_GATTS_CREATE_EVT:
        DBG_MAX("CREATE_SERVICE_EVT, status %d,  service_handle %d\n", param->create.status, param->create.service_handle);
        gl_profile_tab[PROFILE_A_APP_ID].service_handle = param->create.service_handle;
        gl_profile_tab[PROFILE_A_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_TEST_A;

        esp_ble_gatts_start_service(gl_profile_tab[PROFILE_A_APP_ID].service_handle);
        a_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
        esp_err_t add_char_ret = esp_ble_gatts_add_char(gl_profile_tab[PROFILE_A_APP_ID].service_handle, &gl_profile_tab[PROFILE_A_APP_ID].char_uuid,
                                                        ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                        a_property,
                                                        &gatts_demo_char1_val, NULL);
        if (add_char_ret){
            ESP_LOGE(GATTS_TAG, "add char failed, error code =%x",add_char_ret);
        }
        break;
    case ESP_GATTS_ADD_INCL_SRVC_EVT:
        break;

    case ESP_GATTS_ADD_CHAR_EVT: {
        uint16_t length = 0;
        const uint8_t *prf_char;

        DBG_MAX("ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d\n", param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
        gl_profile_tab[PROFILE_A_APP_ID].char_handle = param->add_char.attr_handle;
        gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
        esp_err_t get_attr_ret = esp_ble_gatts_get_attr_value(param->add_char.attr_handle,  &length, &prf_char);
        if (get_attr_ret == ESP_FAIL){
            ESP_LOGE(GATTS_TAG, "ILLEGAL HANDLE");
        }

        DBG_MAX(GATTS_TAG, "the gatts demo char length = %x\n", length);
        for(int i = 0; i < length; i++){
          DBG_MAX("prf_char[%x] =%x\n",i,prf_char[i]);
          }
        esp_err_t add_descr_ret = esp_ble_gatts_add_char_descr(gl_profile_tab[PROFILE_A_APP_ID].service_handle, &gl_profile_tab[PROFILE_A_APP_ID].descr_uuid,
                                                                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
        if (add_descr_ret){
          ESP_LOGE(GATTS_TAG, "add char descr failed, error code =%x", add_descr_ret);
          }
        break;
    }
    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        gl_profile_tab[PROFILE_A_APP_ID].descr_handle = param->add_char_descr.attr_handle;
        DBG_MAX("ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d\n",
                 param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
        break;
    case ESP_GATTS_DELETE_EVT:
        break;

    case ESP_GATTS_START_EVT:
        DBG_MAX("SERVICE_START_EVT, status %d, service_handle %d\n", param->start.status, param->start.service_handle);
        break;

    case ESP_GATTS_STOP_EVT:
        DBG_MAX("SERVICE_STOP_EVT, status %d, service_handle %d\n", param->start.status, param->start.service_handle);
        break;

    case ESP_GATTS_CONNECT_EVT: {
        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        /* For the IOS system, please reference the apple official documents about the ble connection parameters restrictions. */
        conn_params.latency = 0;
        conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
        conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
        conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
        DBG_MIN("ESP_GATTS_CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:", param->connect.conn_id, param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2], param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
        gl_profile_tab[PROFILE_A_APP_ID].conn_id = param->connect.conn_id;
        //start sent the update connection parameters to the peer device.
        esp_ble_gap_update_conn_params(&conn_params);
        }
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        DBG_MIN("ESP_GATTS_DISCONNECT_EVT, disconnect reason 0x%x", param->disconnect.reason);
        esp_ble_gap_start_advertising(&adv_params);
        break;

    case ESP_GATTS_CONF_EVT:
        DBG_MIN("ESP_GATTS_CONF_EVT, status %d attr_handle %d", param->conf.status, param->conf.handle);
        if (param->conf.status != ESP_GATT_OK){
          esp_log_buffer_hex(GATTS_TAG, param->conf.value, param->conf.len);
          }
        break;

    case ESP_GATTS_OPEN_EVT:
      ESP_LOGI(GATTS_TAG, "ESP_GATTS_OPEN_EVT");
      break;
    case ESP_GATTS_CANCEL_OPEN_EVT:
      ESP_LOGI(GATTS_TAG, "ESP_GATTS_CANCEL_OPEN_EVT");
      break;
    case ESP_GATTS_CLOSE_EVT:
      ESP_LOGI(GATTS_TAG, "ESP_GATTS_CLOSE_EVT");
      break;
    case ESP_GATTS_LISTEN_EVT:
      ESP_LOGI(GATTS_TAG, "ESP_GATTS_LISTEN_EVT");
      break;
    case ESP_GATTS_CONGEST_EVT:
      ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONGEST_EVT");
      break;
    default:
        break;
    }
}

static void gatts_profile_b_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
  switch (event) {
    case ESP_GATTS_REG_EVT:
        DBG_MAX("REGISTER_APP_EVT, status %d, app_id %d\n", param->reg.status, param->reg.app_id);
        gl_profile_tab[PROFILE_B_APP_ID].service_id.is_primary = true;
        gl_profile_tab[PROFILE_B_APP_ID].service_id.id.inst_id = 0x00;
        gl_profile_tab[PROFILE_B_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_B_APP_ID].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_TEST_B;

        esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_B_APP_ID].service_id, GATTS_NUM_HANDLE_TEST_B);
        break;

    case ESP_GATTS_READ_EVT: {
        ESP_LOGI(GATTS_TAG, "GATT_READ_EVT, conn_id %d, trans_id %d, handle %d\n", param->read.conn_id, param->read.trans_id, param->read.handle);
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.handle = param->read.handle;
        rsp.attr_value.len = 4;
        rsp.attr_value.value[0] = 0xde;
        rsp.attr_value.value[1] = 0xed;
        rsp.attr_value.value[2] = 0xbe;
        rsp.attr_value.value[3] = 0xef;
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
        }
        break;

    case ESP_GATTS_WRITE_EVT: {
        ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, conn_id %d, trans_id %d, handle %d\n", param->write.conn_id, param->write.trans_id, param->write.handle);
        if (!param->write.is_prep){
            ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, value len %d, value :", param->write.len);
            esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);
            if (gl_profile_tab[PROFILE_B_APP_ID].descr_handle == param->write.handle && param->write.len == 2){
                uint16_t descr_value= param->write.value[1]<<8 | param->write.value[0];
                if (descr_value == 0x0001){
                    if (b_property & ESP_GATT_CHAR_PROP_BIT_NOTIFY){
                        ESP_LOGI(GATTS_TAG, "notify enable");
                        uint8_t notify_data[15];
                        for (int i = 0; i < sizeof(notify_data); ++i)
                        {
                            notify_data[i] = i%0xff;
                        }
                        //the size of notify_data[] need less than MTU size
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_B_APP_ID].char_handle,
                                                sizeof(notify_data), notify_data, false);
                    }
                }else if (descr_value == 0x0002){
                    if (b_property & ESP_GATT_CHAR_PROP_BIT_INDICATE){
                        ESP_LOGI(GATTS_TAG, "indicate enable");
                        uint8_t indicate_data[15];
                        for (int i = 0; i < sizeof(indicate_data); ++i)
                        {
                            indicate_data[i] = i%0xff;
                        }
                        //the size of indicate_data[] need less than MTU size
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_B_APP_ID].char_handle,
                                                sizeof(indicate_data), indicate_data, true);
                    }
                }
                else if (descr_value == 0x0000){
                    ESP_LOGI(GATTS_TAG, "notify/indicate disable ");
                }else{
                    ESP_LOGE(GATTS_TAG, "unknown value");
                }

            }
        }
        example_write_event_env(gatts_if, &b_prepare_write_env, param);
        break;
    }
    case ESP_GATTS_EXEC_WRITE_EVT:
        DBG_MIN("ESP_GATTS_EXEC_WRITE_EVT");
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        example_exec_write_event_env(&b_prepare_write_env, param);
        break;
    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
        break;
    case ESP_GATTS_UNREG_EVT:
        break;
    case ESP_GATTS_CREATE_EVT:
        DBG_MIN("CREATE_SERVICE_EVT, status %d,  service_handle %d\n", param->create.status, param->create.service_handle);
        gl_profile_tab[PROFILE_B_APP_ID].service_handle = param->create.service_handle;
        gl_profile_tab[PROFILE_B_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_B_APP_ID].char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_TEST_B;

        esp_ble_gatts_start_service(gl_profile_tab[PROFILE_B_APP_ID].service_handle);
        b_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
        esp_err_t add_char_ret =esp_ble_gatts_add_char( gl_profile_tab[PROFILE_B_APP_ID].service_handle, &gl_profile_tab[PROFILE_B_APP_ID].char_uuid,
                                                        ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                        b_property,
                                                        NULL, NULL);
        if (add_char_ret){
            ESP_LOGE(GATTS_TAG, "add char failed, error code =%x",add_char_ret);
        }
        break;
    case ESP_GATTS_ADD_INCL_SRVC_EVT:
        break;

    case ESP_GATTS_ADD_CHAR_EVT:
        DBG_MIN("ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d\n", param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);

        gl_profile_tab[PROFILE_B_APP_ID].char_handle = param->add_char.attr_handle;
        gl_profile_tab[PROFILE_B_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_B_APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
        esp_ble_gatts_add_char_descr(gl_profile_tab[PROFILE_B_APP_ID].service_handle, &gl_profile_tab[PROFILE_B_APP_ID].descr_uuid,
                                     ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                     NULL, NULL);
        break;

    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        gl_profile_tab[PROFILE_B_APP_ID].descr_handle = param->add_char_descr.attr_handle;
        DBG_MIN("ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d\n", param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
        break;
    case ESP_GATTS_DELETE_EVT:
        break;
    case ESP_GATTS_START_EVT:
        DBG_MIN("SERVICE_START_EVT, status %d, service_handle %d\n", param->start.status, param->start.service_handle);
        break;
    case ESP_GATTS_STOP_EVT:
        break;
    case ESP_GATTS_CONNECT_EVT:
        DBG_MIN("CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:",
                 param->connect.conn_id,
                 param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
                 param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
        gl_profile_tab[PROFILE_B_APP_ID].conn_id = param->connect.conn_id;
        break;
    case ESP_GATTS_CONF_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONF_EVT status %d attr_handle %d", param->conf.status, param->conf.handle);
        if (param->conf.status != ESP_GATT_OK) {
          esp_log_buffer_hex(GATTS_TAG, param->conf.value, param->conf.len);
          }
      break;

    case ESP_GATTS_DISCONNECT_EVT:
      DBG_MIN("ESP_GATTS_DISCONNECT_EVT");
      break;
    case ESP_GATTS_OPEN_EVT:
    case ESP_GATTS_CANCEL_OPEN_EVT:
    case ESP_GATTS_CLOSE_EVT:
    case ESP_GATTS_LISTEN_EVT:
    case ESP_GATTS_CONGEST_EVT:
    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
        } else {
            ESP_LOGI(GATTS_TAG, "Reg app failed, app_id %04x, status %d\n", param->reg.app_id, param->reg.status);
            return;
        }
    }

    /* If the gatts_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gatts_if == gl_profile_tab[idx].gatts_if) {
                if (gl_profile_tab[idx].gatts_cb) {
                    gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

void read_flash(void) {
  esp_err_t err;
  size_t lenght;
  nvs_handle nvsHandle;

  err = nvs_flash_init(); 
  err = nvs_open(NVS_BASE_NAME, NVS_READWRITE, &nvsHandle);
  if (err==ESP_ERR_NVS_NOT_INITIALIZED) { 
    ESP_LOGE(__func__, "nvs_open failed %s(0x%X)", esp_err_to_name(err), err);
    const esp_partition_t *part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS, NVS_BASE_NAME);
    if(!part) { ESP_LOGE(__func__, "esp_partition_find_first couldn't find nvs partition %s(0x%X)", esp_err_to_name(err), err); return; }
    err = esp_partition_erase_range(part, 0, part->size);
    if (err != ESP_OK) {ESP_LOGE(__func__, "esp_partition_erase_range fails %s(0x%X)", esp_err_to_name(err), err); return; }
    err = nvs_flash_init(); 
    if (err != ESP_OK) { ESP_LOGE(__func__, "nvs_flash_init error %s(0x%X)", esp_err_to_name(err), err); return; }
    err = nvs_open(NVS_BASE_NAME, NVS_READWRITE, &nvsHandle);
    if (err==ESP_ERR_NVS_NOT_INITIALIZED) { printf("--------------------> FANGALA!\n\n\n"); }
    else { printf("OLE!\n"); }
    }


  if ((err = nvs_get_u16(nvsHandle, GATT_SRV_NAME_LEN, &gatts_data.ble_adv_name_len)) != ESP_OK) { DBG_MIN("failed %s(0x%X)", esp_err_to_name(err), err); }
  DBG_MIN("ble_adv_name_len %u", gatts_data.ble_adv_name_len);
  lenght=gatts_data.ble_adv_name_len+1;
  if ((err = nvs_get_blob(nvsHandle, GATT_SRV_NAME, (char *)&gatts_data.ble_adv_name, &lenght)) != ESP_OK) { DBG_MIN("failed %s(0x%X)", esp_err_to_name(err), err); }

  
  #if 0
  /* ----------------------------------------------------------------------------------------- */
  if ((err = nvs_get_u8(nvsHandle, OZ_MAP1_KEY, &ozono_data.map.bits)) != ESP_OK) { DBG_MIN("Get map1 %s(0x%X)", esp_err_to_name(err), err); }

  if ((err = nvs_get_u16(nvsHandle, OZ_EVTID_KEY, &ozono_data.event_id)) != ESP_OK) { DBG_MIN("Get event_id %s(0x%X)", esp_err_to_name(err), err); }
  if ((err = nvs_get_u16(nvsHandle, OZ_EVTCODE_KEY, &ozono_data.event_code)) != ESP_OK) { DBG_MIN("Get event_code %s(0x%X)", esp_err_to_name(err), err); }
  if ((err = nvs_get_u16(nvsHandle, OZ_DATAMSGFREQ_KEY, &ozono_data.spare)) != ESP_OK) { DBG_MIN("Get spare %s(0x%X)", esp_err_to_name(err), err); }
  #endif

  nvs_close(nvsHandle);
}

void write_flash(){
  nvs_handle nvsHandle;
  esp_err_t err = nvs_open(NVS_BASE_NAME, NVS_READWRITE, &nvsHandle);
  ESP_ERROR_CHECK(err);

  gatts_data.ble_adv_name_len=strlen(gatts_data.ble_adv_name);
  DBG_MIN("ble_adv_name_len %u", gatts_data.ble_adv_name_len);
  if ((err = nvs_set_u16(nvsHandle, GATT_SRV_NAME_LEN, gatts_data.ble_adv_name_len+1)) != ESP_OK) { DBG_MIN("nvs_set_u16 failed %s(0x%X)", esp_err_to_name(err), err); }
  if ((err = nvs_set_blob(nvsHandle, GATT_SRV_NAME, (char *)&gatts_data.ble_adv_name, gatts_data.ble_adv_name_len+1)) != ESP_OK) { DBG_MIN("write to NVS failed %s(0x%X)", esp_err_to_name(err), err); }

  if ((err = nvs_commit(nvsHandle)) == ESP_OK) {
    DBG_MIN(" SAVED");
    }
  nvs_close(nvsHandle);
}


void setDefaults(esp_sleep_wakeup_cause_t wakeup_reason) {
  uint8_t baseMac[6];
  // Get MAC address for WiFi station
  //esp_read_mac(baseMac, ESP_MAC_WIFI_STA);

  // Get MAC address for Bluetooth
  esp_read_mac(baseMac, ESP_MAC_BT);  
  DBG_MAX("Unique ID (mac add) %02X:%02X:%02X:%02X:%02X:%02X", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
  // ID lenght == 18 chars with final \0
  sprintf(gatts_data.boardID, "%02X%02X%02X%02X%02X%02X", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
  
  sprintf(gatts_data.ble_adv_name, ""BLE_ADV_DEVICE_NAME"");
  gatts_data.ble_adv_name_len=strlen(gatts_data.ble_adv_name);
  gatts_data.map.bit_vars.logtype=LOG_NOTHING;
  gatts_data.map.bit_vars.bStayAwake=true;
  gatts_data.map.bit_vars.bDeepSleepEnabled=true;
  gatts_data.nMainLoopDelay=500;
}

void set_wakeup_reason(esp_sleep_wakeup_cause_t *wakeup_reason){
  *wakeup_reason = esp_sleep_get_wakeup_cause();
  switch(*wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0  : DBG_MIN("Wakeup caused by external signal using RTC_CNTL %d", *wakeup_reason); break;
    case ESP_SLEEP_WAKEUP_EXT1  : DBG_MIN("Wakeup caused by ESP_SLEEP_WAKEUP_EXT1"); break;
    case ESP_SLEEP_WAKEUP_TIMER  : DBG_MIN("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD  : DBG_MIN("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP  : DBG_MIN("Wakeup caused by ULP program"); break;
    default : DBG_MIN("Wakeup was not caused by deep sleep, maybe RESET"); break;
  }
}

void uart0_init() {
  /* Configure parameters of an UART driver, communication pins and install the driver */
  uart_config_t uart0_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity    = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_APB,
    };
  int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

  ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256 * 2, 0, 0, NULL, intr_alloc_flags));
  ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart0_config));
  ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

}

int blocking_fgets(char * buf) {
  int nRet=1;
  uint8_t oldlogtype, i=0;

  oldlogtype=gatts_data.map.bit_vars.logtype;
  gatts_data.map.bit_vars.logtype=LOG_NOTHING;

  while ((buf[i] = fgetc(stdin)) != '\n') {
    if (buf[i]==0xFF) {
      vTaskDelay(10 / portTICK_PERIOD_MS);
      continue;
      }
    if (buf[i]==0x03) {                       //CTRL-C
      vTaskDelay(10 / portTICK_PERIOD_MS);
      nRet=0;
      break;
      }
    printf("%c", buf[i]);
    if (i++>9) { nRet=0; break; }
    }

  if (buf[i]=='\n' && nRet ) {
    fgets(buf, 10, stdin);
    buf[i]=0;
    printf("\n%s\n", buf);
    }
  else {printf(" fail\n");}
  gatts_data.map.bit_vars.logtype=oldlogtype;
  return nRet;
}

void UART0_log(struct tm *tminfo) {
  char uart0_buf[256];

  switch (gatts_data.map.bit_vars.logtype) {
    case LOG_NOTHING:
      break;

    case LOG_DATA_AND_HEADER:
      sprintf(uart0_buf, "hh:mi:se,\tppb ka,\ttemp,\thumi,\tnAxPPM,\tnA,\traw_nA,\tADCofs,\tADCmV,\t%s,\tID\n", gatts_data.nElapsedSeconds ? "WrmgUP" : "2DeepS");
      printf(uart0_buf);

      #ifdef RFCOMM_ENABLE
      if (ozono_data.spp_handle) {
          esp_spp_write(ozono_data.spp_handle, strlen(uart0_buf), (uint8_t *)uart0_buf);
          }
      #endif

    case LOG_DATAONLY:
      #if 0
      get_lmp_data(&lmp);
      sprintf(uart0_buf, "%02d:%02d:%02d,\t%i,\t%.01f,\t%.01f,\t%.02f,\t%.02f,\t%.02f,\t%.01f,\t%.01f,\t%u,\t%s\n", \
              tminfo->tm_hour, tminfo->tm_min, tminfo->tm_sec, lmp_data.O3_ppb_k, si7021_data.temp, si7021_data.humi, lmp.nAxPPM, lmp.nA, lmp.raw_nA, (float)lmp.vOffs/10, (float)lmp.rawADCuVolt/1000, ozono_data.nSecond2warmup ? ozono_data.nSecond2warmup : ozono_data.nSeconds2StayAwake-ozono_data.nElapsedSeconds, ozono_data.boardID);
      printf(uart0_buf);

      if (gatts_data.map.bit_vars.bRFCommEnable)
        if (gatts_data.spp_handle) {
          esp_spp_write(gatts_data.spp_handle, strlen(uart0_buf), (uint8_t *)uart0_buf);
          }
      #endif
      break;

    default:
      break;
    }
}

void keyb_parser(void *parm) {
  _gatts_data * oz = (_gatts_data *)parm;  

  oz->keyb_buf[0] = fgetc(stdin);
  //oz->keyb_buf[0] = (char)fgetc(stdin);
  //oz->keyb_buf[0] = '2';
  //oz->keyb_buf[0] = getchar();
  switch (oz->keyb_buf[0]) {
    case 'B':
      printf("Deep Sleep ");
      if (oz->map.bit_vars.bDeepSleepEnabled) { printf("disabled\n"); oz->map.bit_vars.bDeepSleepEnabled=false; }
      else {                                    printf("enabled\n");  oz->map.bit_vars.bDeepSleepEnabled=true;  }
      break;  

    case 'b':
      printf("log type -> ");
      switch (oz->map.bit_vars.logtype){
        case LOG_NOTHING:
          printf("LOG_DATAONLY\n"); 
          oz->map.bit_vars.logtype=LOG_DATAONLY;
          break;
        case LOG_DATAONLY:
          printf("LOG_DATA_AND_HEADER\n"); 
          oz->map.bit_vars.logtype=LOG_DATA_AND_HEADER;
          break;
        case LOG_DATA_AND_HEADER:
          printf("LOG_DATA_EXT_DATAONLY\n"); 
          oz->map.bit_vars.logtype=LOG_DATA_EXT_DATAONLY;
          break;
        case LOG_DATA_EXT_DATAONLY:
          printf("LOG_DATA_EXT_AND_HEADER\n"); 
          oz->map.bit_vars.logtype=LOG_DATA_EXT_AND_HEADER;
          break;
        case LOG_DATA_EXT_AND_HEADER:
          printf("LOG_TEST_AND_HEADER\n"); 
          oz->map.bit_vars.logtype=LOG_TEST_AND_HEADER;
          break;
        case LOG_TEST_AND_HEADER:
          printf("LOG_NOTHING\n"); 
          oz->map.bit_vars.logtype=LOG_NOTHING;
          break;
        }
      break;

    case 'k':
      oz->map.bit_vars.bStayAwake=false;
      break;  

    case 'M':
      oz->nMainLoopDelay+=1;
      break;

    case 'm':
      oz->nMainLoopDelay-=1;
      break;

    case 'n':
      printf("insert adv name: ");
      blocking_fgets(oz->keyb_buf);
      sprintf(gatts_data.ble_adv_name, oz->keyb_buf);
      /*
      int l=strlen(gatts_data.ble_adv_name);
      raw_scan_rsp_data[0]=l;
      raw_scan_rsp_data[1]=0x09;
      memcpy(&raw_scan_rsp_data[2], gatts_data.ble_adv_name, l);
      #if 1
      esp_ble_gap_stop_advertising();
      esp_ble_gap_set_device_name(gatts_data.ble_adv_name);
      esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
      esp_ble_gap_start_advertising(&adv_params);
      #else
      esp_ble_gap_set_device_name(gatts_data.ble_adv_name);
      raw_adv_data[4]=l;
      //esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));      
      esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
      #endif
      */
      //printf("-> %s\n", oz->keyb_buf);
      printf("ok\n");
      break;  

    case 'r':
      //setDefaults(ESP_SLEEP_WAKEUP_EXT0);
      DBG_MIN("CTRL set to defaults");
      break;

    case 'W':
      DBG_MIN("written");
      write_flash();
      break;  

    case '+':
      printf("SPP ");
      if (oz->map.bit_vars.bRFCommEnable) { printf("disabled\n"); oz->map.bit_vars.bRFCommEnable=false; spp_end(); }
      else {printf("enabled\n"); oz->map.bit_vars.bRFCommEnable=true; spp_init(); }
      break;

    case '*':
      esp_restart();
      break;

    case '\n':
    case '?':
    case 'h':
      printf("FW rel "RELEASEFW", board "BOARD_REV", press single char to:\n");
      printf("%s (%s)\n", "B   - disable/enable deep-sleep", oz->map.bit_vars.bDeepSleepEnabled ? "ON" : "OFF");
      printf("b   - toggle rs232 logs\n");
      printf("M/m - +/- Main loop delay %d (ms)\n", oz->nMainLoopDelay);
      printf("n   - set BLE advertize name (%s, len %u)\n", gatts_data.ble_adv_name, gatts_data.ble_adv_name_len);
      printf("k   - go to deep-sleep now\n");
      printf("W   - save to flash (B to store a calib. backup)\n");
      printf("r   - Reset to defaults\n");
      printf("*   - reset ESP32\n");
      printf("%s (%s)\n", "+   - enable/disable SPP", oz->map.bit_vars.bRFCommEnable ? "ON" : "OFF");
      //printf("Seconds to Deep Sleep %d\n", oz->nSeconds2StayAwake-(tminfo->tm_min*60+tminfo->tm_sec));
      break;

    default:
      //printf("%c\n",oz->keyb_buf[0]); 
      break;
    }
}


void app_main(void) {
  esp_sleep_wakeup_cause_t wakeup_reason;
  esp_err_t ret;
  time_t now;
  /*
  struct tm {
    int tm_sec;         // seconds,  range 0 to 59          
    int tm_min;         // minutes, range 0 to 59           
    int tm_hour;        // hours, range 0 to 23             
    int tm_mday;        // day of the month, range 1 to 31  
    int tm_mon;         // month, range 0 to 11             
    int tm_year;        // The number of years since 1900   
    int tm_wday;        // day of the week, range 0 to 6    
    int tm_yday;        // day in the year, range 0 to 365  
    int tm_isdst;       // daylight saving time             	
  };
  */
  struct tm *tminfo;
  #if 1
  set_wakeup_reason(&wakeup_reason);
  setDefaults(wakeup_reason);
  read_flash();
  #else
  // Initialize NVS.
  ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK( ret );
  #endif


  ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  ret = esp_bt_controller_init(&bt_cfg);
  if (ret) {
      ESP_LOGE(GATTS_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
      return;
  }

  ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
  if (ret) {
      ESP_LOGE(GATTS_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
      return;
  }
  ret = esp_bluedroid_init();
  if (ret) {
      ESP_LOGE(GATTS_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
      return;
  }
  ret = esp_bluedroid_enable();
  if (ret) {
      ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
      return;
  }

  ret = esp_ble_gatts_register_callback(gatts_event_handler);
  if (ret){
      ESP_LOGE(GATTS_TAG, "gatts register error, error code = %x", ret);
      return;
  }
  ret = esp_ble_gap_register_callback(gap_event_handler);
  if (ret){
      ESP_LOGE(GATTS_TAG, "gap register error, error code = %x", ret);
      return;
  }
  ret = esp_ble_gatts_app_register(PROFILE_A_APP_ID);
  if (ret){
      ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
      return;
  }
  ret = esp_ble_gatts_app_register(PROFILE_B_APP_ID);
  if (ret){
      ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
      return;
  }
  esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
  if (local_mtu_ret){
    ESP_LOGE(GATTS_TAG, "set local MTU failed, error code = %x", local_mtu_ret);
    }

  //uart0_init();
  //vTaskDelay(100 / portTICK_PERIOD_MS);
  #if 0
  TaskHandle_t xHandle = NULL;
  // Create the task, storing the handle.  Note that the passed parameter ucParameterToPass
  // must exist for the lifetime of the task, so in this case is declared static.  If it was just an
  // an automatic stack variable it might no longer exist, or at least have been corrupted, by the time
  // the new task attempts to access it.
  xTaskCreate( keyb_parser_task, "keyb_parser", STACK_SIZE, &gatts_data, tskIDLE_PRIORITY, &xHandle );
  configASSERT( xHandle );

  // Use the handle to delete the task.
  if( xHandle != NULL )  { vTaskDelete( xHandle ); }

  xTaskCreate(keyb_parser_task, "keyb_parser", configMINIMAL_STACK_SIZE, (void *)&gatts_data, tskIDLE_PRIORITY,  &xHandle);
  #endif

  while (gatts_data.map.bit_vars.bStayAwake) {
    time(&now);
    tminfo=gmtime(&now);

    UART0_log(tminfo);
    keyb_parser((void *)&gatts_data);
    vTaskDelay(gatts_data.nMainLoopDelay / portTICK_PERIOD_MS);
    }

  return;
}
