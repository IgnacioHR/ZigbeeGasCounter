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
#include "sys/time.h"

#include "ha/esp_zigbee_ha_standard.h"
#include "zcl/esp_zigbee_zcl_power_config.h"
#include "zcl/esp_zigbee_zcl_metering.h"

// Maximum time to force a device report
#define MUST_SYNC_MINIMUM_TIME          UINT16_C(3600) // 1 hour in seconds

// time to send the device to deep sleep when Zigbee radio is on
#define TIME_TO_SLEEP_ZIGBEE_ON         UINT32_C(1000) // milliseconds

// time to send the device to deep sleep when Zigbee radio is on
#define TIME_TO_SLEEP_ZIGBEE_STARTING   UINT32_C(30 * 1000) // milliseconds

// time to send the device to deep sleep when Zigbee radio is off
#define TIME_TO_SLEEP_ZIGBEE_OFF        UINT32_C(500) // milliseconds

// Maximum difference between the internal counter value and last reported counter value
#define COUNTER_REPORT_DIFF             UINT32_C(100)

// Measure current summation delivered
extern esp_zb_uint48_t current_summation_delivered;
extern bool leaving_network;

// device status and extended status
extern RTC_DATA_ATTR uint8_t device_status;
extern RTC_DATA_ATTR uint64_t device_extended_status;

// Measure instantaneous demand as int24
extern esp_zb_int24_t instantaneous_demand;

// Last report sent time
extern RTC_DATA_ATTR struct timeval last_report_sent_time;
extern RTC_DATA_ATTR uint64_t last_summation_sent;

void esp_zb_task(void *pvParameters);