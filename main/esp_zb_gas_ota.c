/*
 * Zigbee Gas Meter - An open-source Zigbee gas meter project.
 * Copyright (c) 2025 Ignacio Hernández-Ros.
 *
 * This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0
 * International License. To view a copy of this license, visit
 * https://creativecommons.org/licenses/by-nc-sa/4.0/
 *
 * You may use, modify, and share this work for personal and non-commercial purposes, as long
 * as you credit the original author(s) and share any derivatives under the same license.
 */
#include "esp_check.h"
#include "esp_zb_gas_ota.h"
#include "esp_ota_ops.h"
#include "esp_timer.h"

#include "esp_zb_gas_meter.h"
#include "esp_zigbee_core.h"

// OTA
uint16_t zigbee_zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE;
uint16_t zigbee_stack_version = 0x0002;
uint8_t hw_version = HARDWARE_VERSION;

const esp_partition_t *s_ota_partition = NULL;
esp_ota_handle_t s_ota_handle = 0;
bool s_tagid_received = false;

/**
 * @name Enumeration for the tag identifier denotes the type and format of the data within the element
 * @anchor esp_ota_element_tag_id_t
 */
typedef enum esp_ota_element_tag_id_e
{
	UPGRADE_IMAGE = 0x0000, /*!< Upgrade image */
} esp_ota_element_tag_id_t;

// if we have just updated the firmware mark it as good
esp_err_t check_boot_partition_change()
{
	const esp_partition_t *running_app_part = esp_ota_get_running_partition();
	esp_ota_img_states_t running_app_state;
	esp_err_t ret = esp_ota_get_state_partition(running_app_part, &running_app_state);
	if (ret != ESP_OK)
	{
		ESP_LOGE(TAG, "esp_ota_get_state_partition returned: %s", esp_err_to_name(ret));
		esp_ota_mark_app_invalid_rollback_and_reboot();
		return ESP_FAIL;
	}
	if (running_app_state == ESP_OTA_IMG_PENDING_VERIFY)
	{
		// run self tests. if they pass...
		ESP_LOGI(TAG, "New firmware version started");
		esp_ota_mark_app_valid_cancel_rollback();
		// if they don't pass
		// esp_ota_mark_app_invalid_rollback_and_reboot();
		// return ESP_FAIL;
	}
	return ESP_OK;
}

esp_err_t esp_element_ota_data(uint32_t total_size, const void *payload, uint16_t payload_size, void **outbuf, uint16_t *outlen)
{
	static uint16_t tagid = 0;
	void *data_buf = NULL;
	uint16_t data_len;

	if (!s_tagid_received)
	{
		uint32_t length = 0;
		if (!payload || payload_size <= OTA_ELEMENT_HEADER_LEN)
		{
			ESP_RETURN_ON_ERROR(ESP_ERR_INVALID_ARG, TAG, "Invalid element format");
		}

		tagid = *(const uint16_t *)payload;
		length = *(const uint32_t *)(payload + sizeof(tagid));
		if ((length + OTA_ELEMENT_HEADER_LEN) != total_size)
		{
			ESP_RETURN_ON_ERROR(ESP_ERR_INVALID_ARG, TAG, "Invalid element length [%ld/%ld]", length, total_size);
		}

		s_tagid_received = true;

		data_buf = (void *)(payload + OTA_ELEMENT_HEADER_LEN);
		data_len = payload_size - OTA_ELEMENT_HEADER_LEN;
	}
	else
	{
		data_buf = (void *)payload;
		data_len = payload_size;
	}

	switch (tagid)
	{
	case UPGRADE_IMAGE:
		*outbuf = data_buf;
		*outlen = data_len;
		break;
	default:
		ESP_RETURN_ON_ERROR(ESP_ERR_INVALID_ARG, TAG, "Unsupported element tag identifier %d", tagid);
		break;
	}

	return ESP_OK;
}

esp_err_t zb_ota_upgrade_status_handler(esp_zb_zcl_ota_upgrade_value_message_t message)
{
	static uint32_t total_size = 0;
	static uint32_t offset = 0;
	static int64_t start_time = 0;
	esp_err_t ret = ESP_OK;

	if (message.info.status == ESP_ZB_ZCL_STATUS_SUCCESS)
	{
		switch (message.upgrade_status)
		{
		case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_START:
			ESP_LOGI(TAG, "-- OTA upgrade start");
			start_time = esp_timer_get_time();
			s_ota_partition = esp_ota_get_next_update_partition(NULL);
			assert(s_ota_partition);
#if CONFIG_ZB_DELTA_OTA
			ret = esp_delta_ota_begin(s_ota_partition, 0, &s_ota_handle);
#else
			ret = esp_ota_begin(s_ota_partition, 0, &s_ota_handle);
#endif
			ESP_RETURN_ON_ERROR(ret, TAG, "Failed to begin OTA partition, status: %s", esp_err_to_name(ret));
			break;
		case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_RECEIVE:
			total_size = message.ota_header.image_size;
			offset += message.payload_size;
			ESP_LOGI(TAG, "-- OTA Client receives data: progress [%ld/%ld]", offset, total_size);
			if (message.payload_size && message.payload)
			{
				uint16_t payload_size = 0;
				void *payload = NULL;
				ret = esp_element_ota_data(total_size, message.payload, message.payload_size, &payload, &payload_size);
				ESP_RETURN_ON_ERROR(ret, TAG, "Failed to element OTA data, status: %s", esp_err_to_name(ret));
#if CONFIG_ZB_DELTA_OTA
				ret = esp_delta_ota_write(s_ota_handle, payload, payload_size);
#else
				ret = esp_ota_write(s_ota_handle, (const void *)payload, payload_size);
#endif
				ESP_RETURN_ON_ERROR(ret, TAG, "Failed to write OTA data to partition, status: %s", esp_err_to_name(ret));
			}
			break;
		case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_APPLY:
			ESP_LOGI(TAG, "-- OTA upgrade apply");
			break;
		case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_CHECK:
			ret = offset == total_size ? ESP_OK : ESP_FAIL;
			offset = 0;
			total_size = 0;
			s_tagid_received = false;
			ESP_LOGI(TAG, "-- OTA upgrade check status: %s", esp_err_to_name(ret));
			break;
		case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_FINISH:
			ESP_LOGI(TAG, "-- OTA Finish");
			ESP_LOGI(TAG, "-- OTA Information: version: 0x%lx, manufacturer code: 0x%x, image type: 0x%x, total size: %ld bytes, cost time: %lld ms,",
							 message.ota_header.file_version, message.ota_header.manufacturer_code, message.ota_header.image_type,
							 message.ota_header.image_size, (esp_timer_get_time() - start_time) / 1000);
#if CONFIG_ZB_DELTA_OTA
			ret = esp_delta_ota_end(s_ota_handle);
#else
			ret = esp_ota_end(s_ota_handle);
#endif
			ESP_RETURN_ON_ERROR(ret, TAG, "Failed to end OTA partition, status: %s", esp_err_to_name(ret));
			ret = esp_ota_set_boot_partition(s_ota_partition);
			ESP_RETURN_ON_ERROR(ret, TAG, "Failed to set OTA boot partition, status: %s", esp_err_to_name(ret));
			ESP_LOGW(TAG, "Prepare to restart system");
			esp_restart();
			break;
		case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_ABORT:
			ESP_LOGI(TAG, "-- OTA Aborted");
			#ifdef DEEP_SLEEP
			xEventGroupSetBits(main_event_group_handle, SHALL_START_DEEP_SLEEP);
			#endif
			break;
		default:
			ESP_LOGI(TAG, "OTA status: %d", message.upgrade_status);
			break;
		}
	}
	return ret;
}

esp_err_t zb_ota_upgrade_query_image_resp_handler(esp_zb_zcl_ota_upgrade_query_image_resp_message_t message)
{
	esp_err_t ret = ESP_OK;
	if (message.info.status == ESP_ZB_ZCL_STATUS_SUCCESS)
	{
		ESP_LOGI(TAG, "Queried OTA image from address: 0x%04hx, endpoint: %d", message.server_addr.u.short_addr, message.server_endpoint);
		ESP_LOGI(TAG, "Image version: 0x%lx, manufacturer code: 0x%x, image size: %ld", message.file_version, message.manufacturer_code,
						 message.image_size);
	}
	else
	{
		ESP_LOGI(TAG, "No OTA image available, going to deep sleep");
		#ifdef DEEP_SLEEP
		xEventGroupSetBits(main_event_group_handle, SHALL_START_DEEP_SLEEP);
		#endif
	}
	if (ret == ESP_OK)
	{
		#ifdef DEEP_SLEEP
		xEventGroupSetBits(main_event_group_handle, SHALL_STOP_DEEP_SLEEP);
		#endif
		ESP_LOGI(TAG, "Approving OTA image upgrade");
	}
	else
	{
		ESP_LOGI(TAG, "Rejecting OTA image upgrade, status: %s", esp_err_to_name(ret));
	}
	return ret;
}
