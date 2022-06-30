#include "server_gatts.h"

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "esp_system.h"
#include "esp_log.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#include "driver_LEDs.h"
#include "driver_SSM2518.h"
#include "driver_vibration_motor.h"

#define SERVER_GATTS_TAG "server_gatts"

#define MAX_SAMPLE_PACKETS 40 //How many packets
#define I2S_PACKET_SIZE 400 //How many bytes get sent per I2S packet

static uint8_t samples[MAX_SAMPLE_PACKETS * I2S_PACKET_SIZE]; //I2S Non-DMA Buffer
static uint8_t samples_num = 0; //Used to "flush" the Non-DMA Buffer when it's full

#define GATTS_CHAR_VAL_LEN_MAX 500 //Characteristics may not be larger than 500 bytes

#define DEVICE_NAME "MUSSAP BLE Device" //Advertised device name

static uint8_t adv_config_done = 0;
#define ADV_CONFIG_FLAG             (1 << 0)
#define SCAN_RSP_CONFIG_FLAG        (1 << 1)
static uint8_t service_uuid[16] = { //Advertised service UUID
		0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x34, 0x12, 0x00, 0x00,
};

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp        = false,
    .include_name        = true,
    .include_txpower     = true,
    .min_interval        = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval        = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance          = 0x00,
    .manufacturer_len    = 0,    //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //test_manufacturer,
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = sizeof(service_uuid),
    .p_service_uuid      = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp        = true,
    .include_name        = true,
    .include_txpower     = true,
    .min_interval        = 0x0006,
    .max_interval        = 0x0010,
    .appearance          = 0x00,
    .manufacturer_len    = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //&test_manufacturer[0],
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = sizeof(service_uuid),
    .p_service_uuid      = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min         = 0x20,
    .adv_int_max         = 0x40,
    .adv_type            = ADV_TYPE_IND,
    .own_addr_type       = BLE_ADDR_TYPE_PUBLIC,
    .channel_map         = ADV_CHNL_ALL,
    .adv_filter_policy   = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

enum {
	mussap_service,

	red_light_intensity_characteristic,
	red_light_intensity_characteristic_value,

	green_light_intensity_characteristic,
	green_light_intensity_characteristic_value,

	blue_light_intensity_characteristic,
	blue_light_intensity_characteristic_value,

	vibration_strength_characteristic,
	vibration_strength_characteristic_value,

	volume_characteristic,
	volume_characteristic_value,

	i2s_buffer_characteristic,
	i2s_buffer_characteristic_value,

	mussap_char_num, //# of elements, don't remove

};

uint16_t mussap_char_handle_table[mussap_char_num];

static const uint16_t primary_service_uuid         = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid   = ESP_GATT_UUID_CHAR_DECLARE;
static const uint8_t char_prop_read_write_notify   = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;

static const uint16_t MUSSAP_SERVICE_UUID      	   = 0x1234;
static const uint16_t RED_LIGHT_INTENSITY_CHAR_VALUE_UUID       = 0x0001;
static const uint16_t GREEN_LIGHT_INTENSITY_CHAR_VALUE_UUID       = 0x0002;
static const uint16_t BLUE_LIGHT_INTENSITY_CHAR_VALUE_UUID       = 0x0003;
static const uint16_t VIBRATION_STRENGTH_CHAR_VALUE_UUID       = 0x0004;
static const uint16_t VOLUME_CHAR_VALUE_UUID       = 0x0005;
static const uint16_t I2S_BUFFER_CHAR_VALUE_UUID       = 0x0006;

static const uint8_t red_light_intensity_value = 0;
static const uint8_t green_light_intensity_value = 0;
static const uint8_t blue_light_intensity_value = 0;
static const uint8_t vibration_strength_value = 0;
static const uint8_t volume_value = 0;
static const uint8_t i2s_buffer_value;

static const esp_gatts_attr_db_t gatt_db[mussap_char_num] =
{
    // Service Declaration
    [mussap_service]        =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
      sizeof(uint16_t), sizeof(MUSSAP_SERVICE_UUID), (uint8_t *)&MUSSAP_SERVICE_UUID}},

    /* Characteristic Declaration */
    [red_light_intensity_characteristic]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    		sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_read_write_notify}},

	/* Characteristic Value */
	[red_light_intensity_characteristic_value] =
	{{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&RED_LIGHT_INTENSITY_CHAR_VALUE_UUID , ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
	  GATTS_CHAR_VAL_LEN_MAX, sizeof(red_light_intensity_value), (uint8_t *)&red_light_intensity_value}},

	/* Characteristic Declaration */
	[green_light_intensity_characteristic]     =
	{{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
	sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_read_write_notify}},

	/* Characteristic Value */
	[green_light_intensity_characteristic_value] =
	{{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GREEN_LIGHT_INTENSITY_CHAR_VALUE_UUID , ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
	GATTS_CHAR_VAL_LEN_MAX, sizeof(green_light_intensity_value), (uint8_t *)&green_light_intensity_value}},

	/* Characteristic Declaration */
	[blue_light_intensity_characteristic]     =
	{{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
	sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_read_write_notify}},

	/* Characteristic Value */
	[blue_light_intensity_characteristic_value] =
	{{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&BLUE_LIGHT_INTENSITY_CHAR_VALUE_UUID , ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
	GATTS_CHAR_VAL_LEN_MAX, sizeof(blue_light_intensity_value), (uint8_t *)&blue_light_intensity_value}},

	/* Characteristic Declaration */
	[vibration_strength_characteristic]     =
	{{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
	sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_read_write_notify}},

	/* Characteristic Value */
	[vibration_strength_characteristic_value] =
	{{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&VIBRATION_STRENGTH_CHAR_VALUE_UUID , ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
	GATTS_CHAR_VAL_LEN_MAX, sizeof(vibration_strength_value), (uint8_t *)&vibration_strength_value}},

	/* Characteristic Declaration */
	[volume_characteristic]     =
	{{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
	sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_read_write_notify}},

	/* Characteristic Value */
	[volume_characteristic_value] =
	{{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&VOLUME_CHAR_VALUE_UUID , ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
	GATTS_CHAR_VAL_LEN_MAX, sizeof(volume_value),  (uint8_t *)&volume_value}},

	/* Characteristic Declaration */
	[i2s_buffer_characteristic]     =
	{{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
	sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_read_write_notify}},

	/* Characteristic Value */
	[i2s_buffer_characteristic_value] =
	{{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&I2S_BUFFER_CHAR_VALUE_UUID , ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
	GATTS_CHAR_VAL_LEN_MAX, sizeof(i2s_buffer_value), (uint8_t *)&i2s_buffer_value}},

};


static void mussap_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
        case ESP_GATTS_REG_EVT:{
            esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(DEVICE_NAME);
            if (set_dev_name_ret){
                ESP_LOGE(SERVER_GATTS_TAG, "set device name failed, error code = %x", set_dev_name_ret);
            }

            //config adv data
			esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
			if (ret){
				ESP_LOGE(SERVER_GATTS_TAG, "config adv data failed, error code = %x", ret);
			}
			adv_config_done |= ADV_CONFIG_FLAG;
			//config scan response data
			ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
			if (ret){
				ESP_LOGE(SERVER_GATTS_TAG, "config scan response data failed, error code = %x", ret);
			}
			adv_config_done |= SCAN_RSP_CONFIG_FLAG;


            esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, mussap_char_num, 0);
            if (create_attr_ret){
                ESP_LOGE(SERVER_GATTS_TAG, "create attr table failed, error code = %x", create_attr_ret);
            }
        }
       	    break;
        case ESP_GATTS_READ_EVT:
        	ESP_LOGI(SERVER_GATTS_TAG, "GATT_READ_EVT, conn_id %d, trans_id %d, handle %d", param->read.conn_id, param->read.trans_id, param->read.handle);

       	    break;
        case ESP_GATTS_WRITE_EVT:
        	ESP_LOGI(SERVER_GATTS_TAG, "GATT_WRITE_EVT, conn_id %d, trans_id %d, handle %d", param->write.conn_id, param->write.trans_id, param->write.handle);
			ESP_LOGI(SERVER_GATTS_TAG, "GATT_WRITE_EVT, value len %d, value %08x", param->write.len, *(uint32_t *)param->write.value);

        	if (param->write.handle == mussap_char_handle_table[red_light_intensity_characteristic_value])
        	{
        		esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, mussap_char_handle_table[red_light_intensity_characteristic_value],
        																					param->write.len, param->write.value, true);
        		uint16_t length = 0;
				const uint8_t *tmp;
        		esp_ble_gatts_get_attr_value(mussap_char_handle_table[red_light_intensity_characteristic_value], &length, &tmp);
        		set_red_LED_intensity(*tmp);
        		ESP_LOGI(SERVER_GATTS_TAG, "Red LED duty set to %u\n", *tmp);

        	}

        	else if (param->write.handle == mussap_char_handle_table[green_light_intensity_characteristic_value])
        	{
        		esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, mussap_char_handle_table[green_light_intensity_characteristic_value],
        		        					                                                param->write.len, param->write.value, true);
        		uint16_t length = 0;
				const uint8_t *tmp;
        		esp_ble_gatts_get_attr_value(mussap_char_handle_table[green_light_intensity_characteristic_value],  &length, &tmp);
        		set_green_LED_intensity(*tmp);
        		ESP_LOGI(SERVER_GATTS_TAG, "Green LED duty set to %u\n", *tmp);
        	}

        	else if (param->write.handle == mussap_char_handle_table[blue_light_intensity_characteristic_value])
        	{
        		esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, mussap_char_handle_table[blue_light_intensity_characteristic_value],
        		        																					param->write.len, param->write.value, true);

        		uint16_t length = 0;
				const uint8_t *tmp;
        		esp_ble_gatts_get_attr_value(mussap_char_handle_table[blue_light_intensity_characteristic_value],  &length, &tmp);
        		set_blue_LED_intensity(*tmp);
        		ESP_LOGI(SERVER_GATTS_TAG, "Blue LED duty set to %u\n", *tmp);
        	}

        	else if (param->write.handle == mussap_char_handle_table[vibration_strength_characteristic_value])
			{
        		esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, mussap_char_handle_table[vibration_strength_characteristic_value],
        		        																					param->write.len, param->write.value, true);

        		uint16_t length = 0;
				const uint8_t *tmp;
				esp_ble_gatts_get_attr_value(mussap_char_handle_table[vibration_strength_characteristic_value],  &length, &tmp);
				set_vibration_intensity(*tmp);
				ESP_LOGI(SERVER_GATTS_TAG, "Vibration motor duty set to %u\n", *tmp);
			}

        	else if (param->write.handle == mussap_char_handle_table[volume_characteristic_value])
			{
        		esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, mussap_char_handle_table[volume_characteristic_value],
        		        																					param->write.len, param->write.value, true);

        		uint16_t length = 0;
				const uint8_t *tmp;
				esp_ble_gatts_get_attr_value(mussap_char_handle_table[volume_characteristic_value],  &length, &tmp);
				set_volume_SSM2518(*tmp);
				ESP_LOGI(SERVER_GATTS_TAG, "Right channel volume register set to %u\n", *tmp);
			}

        	else if (param->write.handle == mussap_char_handle_table[i2s_buffer_characteristic_value])
			{
        		for(size_t i = 0; i < param->write.len; i++)
        		{
        			samples[samples_num * param->write.len + i] = param->write.value[i];
        		}
        		samples_num++;

        		ESP_LOGI(SERVER_GATTS_TAG, "Samples num: %u\n", samples_num);

        		if(samples_num == MAX_SAMPLE_PACKETS)
        		{
        			size_t bytes_written = 0;
        			i2s_write_SSM2518(&samples, I2S_PACKET_SIZE * MAX_SAMPLE_PACKETS, &bytes_written); //Flush non-DMA buffer into DMA buffer
        			samples_num = 0;
        		}

			}

      	    break;
        case ESP_GATTS_EXEC_WRITE_EVT:
            ESP_LOGI(SERVER_GATTS_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
            break;
        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(SERVER_GATTS_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
            break;
        case ESP_GATTS_CONF_EVT:
            ESP_LOGI(SERVER_GATTS_TAG, "ESP_GATTS_CONF_EVT, status = %d, attr_handle %d", param->conf.status, param->conf.handle);
            break;
        case ESP_GATTS_START_EVT:
            ESP_LOGI(SERVER_GATTS_TAG, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
            break;
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(SERVER_GATTS_TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
            esp_log_buffer_hex(SERVER_GATTS_TAG, param->connect.remote_bda, 6);
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            /* For the iOS system, please refer to Apple official documents about the BLE connection parameters restrictions. */
            conn_params.latency = 0;
            conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
            conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
            conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
            //start sent the update connection parameters to the peer device.
            esp_ble_gap_update_conn_params(&conn_params);
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(SERVER_GATTS_TAG, "ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", param->disconnect.reason);
            esp_ble_gap_start_advertising(&adv_params);
            break;
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:{
            if (param->add_attr_tab.status != ESP_GATT_OK){
                ESP_LOGE(SERVER_GATTS_TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
            }
            else if (param->add_attr_tab.num_handle != mussap_char_num){
                ESP_LOGE(SERVER_GATTS_TAG, "create attribute table abnormally, num_handle (%d) \
                        doesn't equal to HRS_mussap_char_num(%d)", param->add_attr_tab.num_handle, mussap_char_num);
            }
            else {
                ESP_LOGI(SERVER_GATTS_TAG, "create attribute table successfully, the number handle = %d\n",param->add_attr_tab.num_handle);
                memcpy(mussap_char_handle_table, param->add_attr_tab.handles, sizeof(mussap_char_handle_table));
                esp_ble_gatts_start_service(mussap_char_handle_table[mussap_service]);
            }
            break;
        }
        case ESP_GATTS_STOP_EVT:
        case ESP_GATTS_OPEN_EVT:
        case ESP_GATTS_CANCEL_OPEN_EVT:
        case ESP_GATTS_CLOSE_EVT:
        case ESP_GATTS_LISTEN_EVT:
        case ESP_GATTS_CONGEST_EVT:
        case ESP_GATTS_UNREG_EVT:
        case ESP_GATTS_DELETE_EVT:
        default:
            break;
    }
}

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

static struct gatts_profile_inst mussap_profile = {
        .gatts_cb = mussap_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
};

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    	case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
    		adv_config_done &= (~ADV_CONFIG_FLAG);
    		if (adv_config_done == 0){
    			esp_ble_gap_start_advertising(&adv_params);
    		}
    		break;
    	case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
    		adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
    		if (adv_config_done == 0){
    			esp_ble_gap_start_advertising(&adv_params);
    		}
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            /* advertising start complete event to indicate advertising start successfully or failed */
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(SERVER_GATTS_TAG, "advertising start failed");
            }else{
                ESP_LOGI(SERVER_GATTS_TAG, "advertising start successfully");
            }
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(SERVER_GATTS_TAG, "Advertising stop failed");
            }
            else {
                ESP_LOGI(SERVER_GATTS_TAG, "Stop adv successfully\n");
            }
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(SERVER_GATTS_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
            break;
        default:
            break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{

    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            mussap_profile.gatts_if = gatts_if;
        } else {
            ESP_LOGE(SERVER_GATTS_TAG, "reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

	if (gatts_if == ESP_GATT_IF_NONE || gatts_if == mussap_profile.gatts_if) {
		if (mussap_profile.gatts_cb) {
			mussap_profile.gatts_cb(event, gatts_if, param);
		}
	}
}

void init_server_gatts()
{
	esp_err_t ret;

	ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	ret = esp_bt_controller_init(&bt_cfg);
	if (ret) {
		ESP_LOGE(SERVER_GATTS_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
		return;
	}

	ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
	if (ret) {
		ESP_LOGE(SERVER_GATTS_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
		return;
	}

	ret = esp_bluedroid_init();
	if (ret) {
		ESP_LOGE(SERVER_GATTS_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
		return;
	}

	ret = esp_bluedroid_enable();
	if (ret) {
		ESP_LOGE(SERVER_GATTS_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
		return;
	}

	ret = esp_ble_gatts_register_callback(gatts_event_handler);
	if (ret){
		ESP_LOGE(SERVER_GATTS_TAG, "gatts register error, error code = %x", ret);
		return;
	}

	ret = esp_ble_gap_register_callback(gap_event_handler);
	if (ret){
		ESP_LOGE(SERVER_GATTS_TAG, "gap register error, error code = %x", ret);
		return;
	}

	ret = esp_ble_gatts_app_register(0x01);
	if (ret){
		ESP_LOGE(SERVER_GATTS_TAG, "gatts app register error, error code = %x", ret);
		return;
	}

	esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
	if (local_mtu_ret){
		ESP_LOGE(SERVER_GATTS_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
	}
}


