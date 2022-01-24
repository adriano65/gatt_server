/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

#include "time.h"
#include "sys/time.h"
#include "rfcomm.h"

#ifdef RFCOMM_ENABLE

#define RFCOMM_DBG_MIN
#ifdef RFCOMM_DBG_MIN
  #define DBG_MIN(fmt...) do {printf("%s: ", __FUNCTION__); printf(fmt); printf("\n"); } while(0)
#else
  #define DBG_MIN(fmt...) do { } while(0)
#endif
//#define RFCOMM_DBG_MAX
#ifdef RFCOMM_DBG_MAX
  #define DBG_MAX(fmt...) do {printf("%s: ", __FUNCTION__); printf(fmt); printf("\n"); } while(0)
#else
  #define DBG_MAX(fmt...) do { } while(0)
#endif

static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;
//static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_NONE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_MASTER;

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  switch (event) {
    case ESP_SPP_INIT_EVT:
      DBG_MIN("ESP_SPP_INIT_EVT");
      esp_bt_dev_set_device_name(ozono_data.boardID);
      esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
      esp_spp_start_srv(sec_mask, role_slave, 0, SPP_SERVER_NAME);
      break;

    case ESP_SPP_DISCOVERY_COMP_EVT:
      DBG_MIN("ESP_SPP_DISCOVERY_COMP_EVT");
      break;
    case ESP_SPP_OPEN_EVT:
      DBG_MIN("ESP_SPP_OPEN_EVT");
      break;
    case ESP_SPP_CLOSE_EVT:
      DBG_MIN("ESP_SPP_CLOSE_EVT");
      ozono_data.spp_handle=0;
      break;
    case ESP_SPP_START_EVT:
      DBG_MIN("ESP_SPP_START_EVT");
      break;
    case ESP_SPP_CL_INIT_EVT:
      DBG_MIN("ESP_SPP_CL_INIT_EVT");
      break;

    case ESP_SPP_DATA_IND_EVT:
      // echo
      //esp_spp_write(param->write.handle, param->data_ind.len, param->data_ind.data);
      #if 0
      switch (param->data_ind.data[0]) {
        case 'B':
          printf("Deep Sleep ");
          if (ozono_data.map1.bit_vars.bDeepSleepEnabled) { printf("disabled\n"); ozono_data.map1.bit_vars.bDeepSleepEnabled=false; }
          else {            printf("enabled\n");  ozono_data.map1.bit_vars.bDeepSleepEnabled=true;  }
          break;  

        case 'b':
          printf("display header");
          if (ozono_data.map1.bit_vars.logtype==LOG_DATA_AND_HEADER) { printf(" OFF\n"); ozono_data.map1.bit_vars.logtype=LOG_DATAONLY; }
          else {            printf(" ON\n");  ozono_data.map1.bit_vars.logtype=LOG_DATA_AND_HEADER; }
          break;

        default:
          esp_spp_write(param->write.handle, 4, (uint8_t *)"NOK\n");
          break;
        }
      #else
      putc(param->data_ind.data[0], stdout);
      esp_spp_write(param->write.handle, 4, (uint8_t *)"OK\n");
      #endif
      break;

    case ESP_SPP_CONG_EVT:
      DBG_MIN("ESP_SPP_CONG_EVT");
      break;

    case ESP_SPP_WRITE_EVT:
      break;

    case ESP_SPP_SRV_OPEN_EVT:
      DBG_MIN("ESP_SPP_SRV_OPEN_EVT");
      ozono_data.spp_handle=param->write.handle;
      break;
        
    /*
    case ESP_SPP_SRV_STOP_EVT:
        DBG_MIN("ESP_SPP_SRV_STOP_EVT");
        break;
    case ESP_SPP_UNINIT_EVT:
        DBG_MIN("ESP_SPP_UNINIT_EVT");
        break;
    */
    default:
      DBG_MIN("event %d\n", event);
      break;
    }
}

void spp_init() {
    esp_err_t ret;
    /*
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
      }
    ESP_ERROR_CHECK( ret );
    */
    //ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BTDM));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    //bt_cfg.mode.ESP_SPP_SEC_NONE
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
      DBG_MIN("Initialize controller failed: %s", esp_err_to_name(ret));
      return;
      }

    //if ((ret = esp_bt_controller_enable(ESP_BT_MODE_BTDM)) != ESP_OK) {      
    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {      
      DBG_MIN("enable controller failed: %s", esp_err_to_name(ret));
      return;
      }
    
    if ((ret = esp_bluedroid_init()) != ESP_OK) {
        DBG_MIN("initialize bluedroid failed: %s", esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        DBG_MIN("enable bluedroid failed: %s", esp_err_to_name(ret));
        return;
    }
    if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {
        DBG_MIN("spp register failed: %s\n", esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_init(esp_spp_mode)) != ESP_OK) {
        DBG_MIN("spp init failed: %s\n", esp_err_to_name(ret));
        return;
    }

#if (CONFIG_BT_SSP_ENABLED == true)
    /* Set default parameters for Secure Simple Pairing */
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    //esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;
    //esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_OUT;    
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
    #warning "YES  CONFIG_BT_SSP_ENABLED"
#endif

    #if 0
    /*
     * Set default parameters for Legacy Pairing
     * Use variable pin, input pin code when pairing
     */
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);
    #else
    /*
     * Set default parameters for Legacy Pairing
     * Use fixed pin code
     */
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_FIXED;
    esp_bt_pin_code_t pin_code;
    pin_code[0] = '1';
    pin_code[1] = '2';
    pin_code[2] = '3';
    pin_code[3] = '4';
    esp_bt_gap_set_pin(pin_type, 4, pin_code);
    #endif

    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_N12);
    esp_bredr_tx_power_set(ESP_PWR_LVL_N12, ESP_PWR_LVL_N12);

}

void spp_end() {
    //esp_err_t ret;
    //esp_spp_stop_srv();
    esp_spp_deinit();

    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit(); 
    //btStop();
}

#else
void spp_init() {}
void spp_end() {}

#endif