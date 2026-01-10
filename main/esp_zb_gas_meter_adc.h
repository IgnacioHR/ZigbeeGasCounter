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
#include "esp_zb_gas_meter.h"
#ifdef MEASURE_BATTERY_LEVEL

#include "sys/time.h"
#include "freertos/task.h"
#include "esp_sleep.h"

/* ADC Battery measurement */
#define BAT_BUFFER_READ_LEN             256

// adc task handle so the conversion ISR can tell
// to the adc task about process done
extern TaskHandle_t adc_task_handle;

// last time battery voltage was adquired 
extern RTC_DATA_ATTR struct timeval last_battery_measurement_time;

void adc_task(void *arg);
#endif