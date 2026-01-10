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
#ifdef MEASURE_BATTERY_LEVEL

#include <string.h>
#include "esp_check.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "zcl/esp_zigbee_zcl_metering.h"

#include "esp_zb_gas_meter.h"
#include "esp_zb_gas_meter_zigbee.h"
#include "esp_zb_gas_meter_adc.h"
#include "esp_zb_gas_meter_adc_zigbee.h"

#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

// output - pin to enable battery voltage to the adc converter
#define BAT_MON_ENABLE                      GPIO_NUM_21

uint8_t battery_voltage = 0;
uint8_t battery_percentage = 0;
uint32_t battery_alarm_state = 0;

#define M                       ((double)MAX_BATTERY_VOLTAGE / (double)ADC_MAX_VALUE)
#define ADC_MIN_VALUE           (((double)MIN_BATTERY_VOLTAGE)/M)
#define TO_PERCENTAGE           (double)(200.0 / ((double)ADC_MAX_VALUE - (double)ADC_MIN_VALUE))

// required by the Zigbee power cluster attributes
uint8_t battery_voltage_min = UINT8_C(MIN_BATTERY_VOLTAGE/100);
uint8_t battery_voltage_th1 = UINT8_C(WARN_BATTERY_VOLTAGE/100);

struct timeval last_battery_measurement_time;

adc_channel_t channel = ADC_CHANNEL_2; // ADC_CHANNEL_3 is a better option, but the current board was using 0 so let's change it now GPIO 3
adc_continuous_handle_t handle = NULL;

void set_battery_unavailable()
{
    battery_alarm_state |= ESP_ZB_ZCL_POWER_CONFIG_MAINS_ALARM_MASK_VOLTAGE_UNAVAIL; // Battery unavailable bit set
    ESP_LOGE(TAG, "Battery marked unavailable");
    xEventGroupSetBits(report_event_group_handle, BATTERY_REPORT);
}

// Hardware configuration of ADC related pins
void bat_adc_hardware_config()
{
    uint64_t bat_volt_enable_pin = 1ULL << BAT_MON_ENABLE;
    gpio_config_t voltage_enable_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = bat_volt_enable_pin,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE};
    if (gpio_config(&voltage_enable_conf) != ESP_OK) {
        ESP_LOGE(TAG, "Can't config gpio for BAT_MON_ENABLE pin");
        set_battery_unavailable();
    }
}

esp_err_t bat_adc_calibration_init(const adc_unit_t unit, const adc_channel_t channel, const adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t error = ESP_FAIL;

    ESP_LOGD(TAG, "calibration scheme version is Curve Fitting");
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = unit,
        .chan = channel,
        .atten = atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    error = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);

    *out_handle = handle;
    if (error == ESP_OK) {
        ESP_LOGD(TAG, "Calibration Success");
    } else if (error == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return error;
}

esp_err_t bat_adc_calibration_deinit(const adc_cali_handle_t handle)
{
    ESP_LOGD(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_RETURN_ON_ERROR(adc_cali_delete_scheme_curve_fitting(handle), TAG, "Failed to delete ADC calibration curve");
    return ESP_OK;
}

bool IRAM_ATTR bat_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    //Notify that ADC continuous driver has done enough number of conversions
    vTaskGenericNotifyGiveFromISR(adc_task_handle, 1, &mustYield);

    return (mustYield == pdTRUE);
}

esp_err_t bat_continuous_adc_init(const adc_channel_t channel, adc_continuous_handle_t *out_handle)
{
    adc_continuous_handle_t handle = NULL;

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 1024,
        .conv_frame_size = BAT_BUFFER_READ_LEN,
    };
    ESP_RETURN_ON_ERROR(adc_continuous_new_handle(&adc_config, &handle), TAG, "ADC call to adc_continuous_new_handle failed");

    adc_continuous_config_t dig_cfg = {
        .pattern_num = 1,
        .sample_freq_hz = SOC_ADC_SAMPLE_FREQ_THRES_LOW,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    adc_pattern[0].atten = ADC_ATTEN_DB_12;
    adc_pattern[0].channel = channel & 0x7;
    adc_pattern[0].unit = ADC_UNIT_1;
    adc_pattern[0].bit_width = ADC_BITWIDTH_12;
    dig_cfg.adc_pattern = adc_pattern;
    ESP_RETURN_ON_ERROR(adc_continuous_config(handle, &dig_cfg), TAG, "ADC call to continous_config failed");

    *out_handle = handle;

    return ESP_OK;
}

// adc task. Obtains battery voltage from adc conversion and formula.
// The task is deleted when the process finish
void adc_task(void *arg)
{
    bool is_first_loop = true;
    bat_adc_hardware_config();
    ESP_LOGI(TAG, "ADC task started");
    while (true) {
        ulTaskGenericNotifyTake(0, pdTRUE, portMAX_DELAY);
        esp_err_t error = ESP_OK;
        if (is_first_loop && (battery_alarm_state & ESP_ZB_ZCL_POWER_CONFIG_MAINS_ALARM_MASK_VOLTAGE_UNAVAIL) == ESP_ZB_ZCL_POWER_CONFIG_MAINS_ALARM_MASK_VOLTAGE_UNAVAIL) {
            ESP_LOGI(TAG, "Battery measuremenet operation cancelled, battery unavailable");
            continue;
        }
        is_first_loop = false; // no hardware issues to allows to enter the loop and maybe reset battery_alarm_state
        ESP_LOGI(TAG, "Starting battery measures");
        adc_cali_handle_t adc1_cali_chan0_handle = NULL;

        error = gpio_set_level(BAT_MON_ENABLE, 1);
        if (error != ESP_OK) {
            ESP_LOGE(TAG, "Can't set BAT_MON_ENABLE to 1");
            set_battery_unavailable();
            continue;
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
        error = bat_continuous_adc_init(channel, &handle);
        if (error != ESP_OK) {
            set_battery_unavailable();
            continue;
        }
        adc_continuous_evt_cbs_t cbs = {
            .on_conv_done = bat_conv_done_cb,
        };
        error = adc_continuous_register_event_callbacks(handle, &cbs, NULL);
        if (error != ESP_OK) {
            ESP_LOGE(TAG, "ADC cannot set continuous register event callbacks %s", esp_err_to_name(error));
            set_battery_unavailable();
            continue;
        }
        error = adc_continuous_start(handle);
        if (error != ESP_OK) {
            ESP_LOGE(TAG, "ADC cannot start continuous readings %s", esp_err_to_name(error));
            set_battery_unavailable();
            continue;
        }
        error = bat_adc_calibration_init(ADC_UNIT_1, channel, ADC_ATTEN_DB_12, &adc1_cali_chan0_handle);
        if (error != ESP_OK) {
            set_battery_unavailable();
            continue;
        }

        uint8_t result[BAT_BUFFER_READ_LEN] = {0};
        memset(result, 0xcc, BAT_BUFFER_READ_LEN);

        ulTaskGenericNotifyTake(1, pdTRUE, portMAX_DELAY);
        error = gpio_set_level(BAT_MON_ENABLE, 0);
        if (error != ESP_OK) {
            ESP_LOGE(TAG, "Can't set BAT_MON_ENABLE to 0");
            error = bat_adc_calibration_deinit(adc1_cali_chan0_handle);
            if (error != ESP_OK) {
                ESP_LOGE(TAG, "Call to bat_adc_calibration_deinit failed");
            }
            set_battery_unavailable();
            continue;
        }

        uint32_t ret_num = 0;
        error = adc_continuous_read(handle, result, BAT_BUFFER_READ_LEN, &ret_num, 0);
        if (error == ESP_OK) {
            uint64_t total = 0;
            uint16_t values = 0;
            for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
                adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result[i];
                uint32_t chan_num = p->type2.channel;
                /* Check the channel number validation, the data is invalid if the channel num exceed the maximum channel */
                if (chan_num < SOC_ADC_CHANNEL_NUM(ADC_UNIT_1)) {
                    uint32_t data = p->type2.data;
                    total += data;
                    values++;
                }
            }
            if (values > 0) {
                uint32_t average = total / values; // average viene en milivoltios
                int voltage;
                error = adc_cali_raw_to_voltage(adc1_cali_chan0_handle, average, &voltage); // voltage in millivolts
                if (error == ESP_OK) {
                    // convert to 1s li-ion range
                    float bat_voltage = (float)(voltage * (float)MAX_BATTERY_VOLTAGE / (float)ADC_MAX_VALUE); // battery voltage from adc readings in millivolts, range to 4200 mv
                    battery_voltage = (uint8_t)(bat_voltage/100.0+0.5); // Zigbee Specification, one byte, 0x00 to 0xff, in units of 100mv
                    battery_percentage = (uint8_t)(((double)average - ADC_MIN_VALUE)*TO_PERCENTAGE); // from 0 to 200
                    if (battery_percentage > 200) {
                        battery_percentage = 200;
                        battery_alarm_state |= ESP_ZB_ZCL_POWER_CONFIG_MAINS_ALARM_MASK_VOLTAGE_HIGH;
                        xEventGroupSetBits(report_event_group_handle, BATTERY_REPORT);
                    } else {
                        battery_alarm_state &= ~ESP_ZB_ZCL_POWER_CONFIG_MAINS_ALARM_MASK_VOLTAGE_HIGH;
                        xEventGroupSetBits(report_event_group_handle, BATTERY_REPORT);
                    }
                    xEventGroupSetBits(report_event_group_handle, BATTERY_REPORT);
                    if ((battery_alarm_state & ESP_ZB_ZCL_POWER_CONFIG_MAINS_ALARM_MASK_VOLTAGE_UNAVAIL) == ESP_ZB_ZCL_POWER_CONFIG_MAINS_ALARM_MASK_VOLTAGE_UNAVAIL) {
                        battery_alarm_state &= ~ESP_ZB_ZCL_POWER_CONFIG_MAINS_ALARM_MASK_VOLTAGE_UNAVAIL; // reset battery alarm state
                        xEventGroupSetBits(report_event_group_handle, BATTERY_REPORT);
                    }
                    if (bat_voltage < WARN_BATTERY_VOLTAGE) {
                        battery_alarm_state |= ESP_ZB_ZCL_POWER_CONFIG_MAINS_ALARM_MASK_VOLTAGE_LOW; // BatteryVoltageMinThreshold or BatteryPercentageMinThreshold reached for Battery Source 1
                        xEventGroupSetBits(report_event_group_handle, BATTERY_REPORT);
                    } else {
                        battery_alarm_state &= ~ESP_ZB_ZCL_POWER_CONFIG_MAINS_ALARM_MASK_VOLTAGE_LOW;
                        xEventGroupSetBits(report_event_group_handle, BATTERY_REPORT);
                    }
                    if (battery_alarm_state != 0 && (device_status & ESP_ZB_ZCL_METERING_GAS_LOW_BATTERY) == 0) {
                        device_status |= ESP_ZB_ZCL_METERING_GAS_LOW_BATTERY;
                        xEventGroupSetBits(report_event_group_handle, STATUS_REPORT);
                    } else if (battery_alarm_state == 0 && (device_status & ESP_ZB_ZCL_METERING_GAS_LOW_BATTERY) != 0) {
                        device_status &= ~ESP_ZB_ZCL_METERING_GAS_LOW_BATTERY;
                        xEventGroupSetBits(report_event_group_handle, STATUS_REPORT);
                    }

                    ESP_LOGI(TAG, "Raw: %"PRIu32" Calibrated: %"PRId16"mV Bat Voltage: %1.2fv ZB Voltage: %d ZB Percentage: %d Alarm 0x%lx", 
                        average, voltage, bat_voltage/1000.0f, battery_voltage, battery_percentage, battery_alarm_state);
                    gettimeofday(&last_battery_measurement_time, NULL);
                } else {
                    ESP_LOGE(TAG, "ADC Task ESP_ERR_TIMEOUT");
                    set_battery_unavailable();
                }
            }
        } else if (error == ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "ADC Task ESP_ERR_TIMEOUT");
            set_battery_unavailable();
        }
        error = bat_adc_calibration_deinit(adc1_cali_chan0_handle);
        if (error != ESP_OK) {
            ESP_LOGE(TAG, "ADC call to bat_adc_calibration_deinit failed");
        }
        error = adc_continuous_stop(handle);
        if (error != ESP_OK) {
            ESP_LOGE(TAG, "ADC call to bat_adc_calibration_deinit failed");
        }
        error = adc_continuous_deinit(handle);
        if (error != ESP_OK) {
            ESP_LOGE(TAG, "ADC call to bat_adc_calibration_deinit failed");
        }
        ESP_LOGI(TAG, "Finishing battery measures");
    }
}
#endif