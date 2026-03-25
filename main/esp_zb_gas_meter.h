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
#define ESP_PRODUCT_CODE                "\x00"

// ******************************************************************************
// ***                         FEATURES CONFIGURATION                        ****
// ******************************************************************************

// if defined, the device reports power and energy. This requires to compute the
// time elapsed between ticks and might to drain the battery more. Enable if you
// are powering the unit from external power
//#define FEATURE_MEASURE_FLOW_RATE

// define if the device is powered from batteries and you want the device to
// measure the battery voltage. Note battery voltage is not reportable. Use the GUI
// refresh button to obtain the latest measured value from the device.
#define FEATURE_MEASURE_BATTERY_LEVEL

// Decide if you are going to use DEEP_SLEEP or LIGHT_SLEEP. Only one can be 
// defined. It is possible also to disable both in case the unit is not battery
// powered

// In DEEP_SLEEP mode the device wakes up once every hour, connect to the network
// report the values (the counter) and goes to sleep again. This means it is not
// likely the device will react to commands send from the user interface. But the
// benefit is a extended battery life
#define FEATURE_DEEP_SLEEP

// In LIGHT_SLEEP the ticks are reported more fulently to the coordinator and the
// device can react to the user interface (the sleep window is set in 30 seconds)
//#define FEATURE_LIGHT_SLEEP

// In zigbee2mqtt it is not possible to write the value to the real gas counter.
// The Zigbee Cluster Library Specification in Table 10-57 states that 
// CurrentSummationDelivered is read only. So, the trick is to implement a custom
// cluster so values written to it are transferred to the current counter value.
#define FEATURE_WRITE_COUNTER_VALUE

// ******************************************************************************



// ******************************************************************************
// ***                         HARDWARE CONFIGURATION                        ****
// ******************************************************************************

// define only when using Seeed Studio ESP32C6 WITH external antenna.
#define CONFIG_EXTERNAL_ANTENNA

// input - pin with gas reel sensor
#define PULSE_PIN 													GPIO_NUM_1

// input - pin for the main button
#define MAIN_BTN 														GPIO_NUM_0

// output - pin to enable battery voltage to the adc converter
#define BAT_MON_ENABLE                      GPIO_NUM_21

#ifdef FEATURE_MEASURE_BATTERY_LEVEL
#define ADC_CHANNEL													ADC_CHANNEL_2
#endif

// ******************************************************************************

#include "esp_zigbee_type.h"
#include "freertos/FreeRTOS.h"

extern const char *TAG;

// Report event group events
#define CURRENT_SUMMATION_DELIVERED_REPORT  (EventBits_t)(1U << 0)
#ifdef FEATURE_MEASURE_FLOW_RATE
#define INSTANTANEOUS_DEMAND_REPORT         (EventBits_t)(1U << 1)
#endif
#define STATUS_REPORT                       (EventBits_t)(1U << 2)
#define EXTENDED_STATUS_REPORT              (EventBits_t)(1U << 3)
#ifdef FEATURE_MEASURE_BATTERY_LEVEL
#define BATTERY_REPORT                      (EventBits_t)(1U << 4)
#endif

// Main group events
#ifdef FEATURE_MEASURE_BATTERY_LEVEL
#define SHALL_MEASURE_BATTERY           (1U << 0)
// 1 hour in seconds
#define MEASURE_BATTERY_VOLTAGE_TIME		(60 * 60)
#endif
#define SHALL_ENABLE_ZIGBEE             (1U << 1)
// this is not implemented because of lack of support from esp-zigbee-sdk
// see https://github.com/espressif/esp-zigbee-sdk/issues/561
#define SHALL_DISABLE_ZIGBEE            (1U << 2) 

#ifdef FEATURE_DEEP_SLEEP
// Command to stop the deep sleep functionality at all. This is required
// while OTA is updating the firmware. New events shall no reschedule the
// deep sleep timer
#define SHALL_STOP_DEEP_SLEEP						(1U << 3)

// Command to start the deel sleep functionality. When the main loop starts
// and when the OTA is cancelled
#define SHALL_START_DEEP_SLEEP					(1U << 4)
#endif

extern EventGroupHandle_t report_event_group_handle;
extern EventGroupHandle_t main_event_group_handle;

#ifdef FEATURE_DEEP_SLEEP
extern QueueHandle_t deep_sleep_queue_handle;
extern TaskHandle_t deep_sleep_task_handle;
extern bool allow_report_to_coordinator;

TickType_t dm_deep_sleep_time_ticks();
#endif

esp_err_t gm_counter_set(const esp_zb_uint48_t *new_value);
int32_t time_diff_ms(const struct timeval *other);

#endif // ESP_ZB_GAS_METER_H