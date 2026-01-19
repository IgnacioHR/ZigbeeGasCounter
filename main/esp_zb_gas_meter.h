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

#ifndef ESP_ZB_GAS_METER_H
#define ESP_ZB_GAS_METER_H

#define ESP_MANUFACTURER_NAME           "\x14""Custom devices (DiY)"
#define ESP_MODEL_IDENTIFIER            "\x10""MiCASAGasCounter" /* Customized model identifier */
#define ESP_DATE_CODE                   "\x08""20250301"
#define ESP_PRODUCT_URL                 "\x2D""https://github.com/IgnacioHR/ZigbeeGasCounter"

// ******************************************************************************
// Functionality that will be included in the gas meter main application
//#define MEASURE_FLOW_RATE
#define MEASURE_BATTERY_LEVEL

// Only one can be defined
//#define DEEP_SLEEP
#define LIGHT_SLEEP
// ******************************************************************************

// ******************************************************************************
// define when using Seeed Studio ESP32C6 WITH external antenna
#define EXTERNAL_ANTENNA
// ******************************************************************************


#include "esp_zigbee_type.h"
#include "freertos/FreeRTOS.h"

extern const char *TAG;

// Report event group events
#define CURRENT_SUMMATION_DELIVERED_REPORT  (EventBits_t)(1 << 0)
#ifdef MEASURE_FLOW_RATE
#define INSTANTANEOUS_DEMAND_REPORT         (EventBits_t)(1 << 1)
#endif
#define STATUS_REPORT                       (EventBits_t)(1 << 2)
#define EXTENDED_STATUS_REPORT              (EventBits_t)(1 << 3)
#ifdef MEASURE_BATTERY_LEVEL
#define BATTERY_REPORT                      (EventBits_t)(1 << 4)
#endif

// Main group events
#ifdef MEASURE_BATTERY_LEVEL
#define SHALL_MEASURE_BATTERY           (1 << 0)
#endif
#define SHALL_ENABLE_ZIGBEE             (1 << 1)
// this is not implemented because of lack of support from esp-zigbee-sdk
// see https://github.com/espressif/esp-zigbee-sdk/issues/561
#define SHALL_DISABLE_ZIGBEE            (1 << 2) 

#ifdef DEEP_SLEEP
// Command to stop the deep sleep functionality at all. This is required
// while OTA is updating the firmware. New events shall no reschedule the
// deep sleep timer
#define SHALL_STOP_DEEP_SLEEP						(1 << 3)

// Command to start the deel sleep functionality. When the main loop starts
// and when the OTA is cancelled
#define SHALL_START_DEEP_SLEEP					(1 << 4)
#endif

extern EventGroupHandle_t report_event_group_handle;
extern EventGroupHandle_t main_event_group_handle;

#ifdef DEEP_SLEEP
extern QueueHandle_t deep_sleep_queue_handle;
extern TaskHandle_t deep_sleep_task_handle;


TickType_t dm_deep_sleep_time_ms();
#endif

void gm_counter_set(const esp_zb_uint48_t *new_value);
uint32_t time_diff_ms(const struct timeval *other);

#endif // ESP_ZB_GAS_METER_H