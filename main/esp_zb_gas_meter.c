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
#include <string.h>
#include "esp_check.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sleep.h"
#include "sys/time.h"
#include "esp_timer.h"

#include "esp_zb_gas_meter.h"
#include "esp_zb_gas_meter_adc.h"
#include "esp_zb_gas_meter_zigbee.h"
#include "esp_zb_gas_ota.h"
#include "esp_zb_gas_led.h"

/* Experimental, check if we sleepy device can help the design */
#ifdef CONFIG_PM_ENABLE
#include "esp_pm.h"
#include "esp_private/esp_clk.h"
#endif

// amount of time to ignore a digital input pin interrupt repetition
#define REED_DEBOUNCE_TIMEOUT 1000U /* milliseconds */
#define BTN_DEBOUNCE_TIMEOUT 30 /* milliseconds */

/* Human interaction and device configuration */

// How long the MAIN BUTTON must be pressed to consider a LONG PRESS
#define LONG_PRESS_TIMEOUT 5 // seconds

#define NVS_NAMESPACE "gas_monitor"
#define NVS_KEY "counter"

#ifdef FEATURE_MEASURE_FLOW_RATE
#define TIME_TO_RESET_INSTANTANEOUS_D UINT32_C(12000)
#endif

const char *TAG = "GAS_COUNTER";

// Last time a pulse was received

// last time an interrupt occurred
RTC_DATA_ATTR struct timeval last_interrupt_time;

// RTC_DATA_ATTR int last_reel_state;
RTC_DATA_ATTR struct timeval last_reel_state_chage_time;

#ifdef FEATURE_MEASURE_FLOW_RATE
RTC_DATA_ATTR struct timeval last_increment_time;
#endif

// gracie period to avoid entering deep sleep when the zigbee radio has been turned on
#ifdef FEATURE_DEEP_SLEEP
struct timeval deep_sleep_gracie_period = {
    .tv_sec = 0,
    .tv_usec = 0
};
#endif

// When the main button is pressed a one time task is started
// to detect a long press without having to wait until
// the user releases the button
#ifdef FEATURE_DEEP_SLEEP
bool started_from_deep_sleep = false;
#endif

// true while leaving the network to prevent sending data to coordinator
bool leaving_network = false;

#ifdef FEATURE_DEEP_SLEEP
// sleep
RTC_DATA_ATTR struct timeval sleep_enter_time;
#endif

// Non volatile memory handle
nvs_handle_t my_nvs_handle;

#ifdef FEATURE_MEASURE_BATTERY_LEVEL
TaskHandle_t adc_task_handle = NULL;
#endif
TaskHandle_t zigbee_task_handle = NULL;
#ifdef FEATURE_DEEP_SLEEP
TaskHandle_t deep_sleep_task_handle = NULL;
#endif
TaskHandle_t save_counter_task_handle = NULL;
TaskHandle_t btn_press_task_handle = NULL;
TaskHandle_t btn_release_task_handle = NULL;
#ifdef FEATURE_DEEP_SLEEP
QueueHandle_t deep_sleep_queue_handle = NULL;
#endif
#ifdef FEATURE_MEASURE_FLOW_RATE
TimerHandle_t reset_instantaneous_demand_timer = NULL;
#endif
TimerHandle_t deep_sleep_timer = NULL;
#ifdef FEATURE_LIGHT_SLEEP
TimerHandle_t periodic_checks_timer = NULL;  // Timer for periodic radio/battery checks (event-driven instead of polling)
#endif
EventGroupHandle_t report_event_group_handle = NULL;
EventGroupHandle_t main_event_group_handle = NULL;

// Intelligent button gesture detection timers
TimerHandle_t timer_since_press_handler = NULL;
TimerHandle_t t_detect_hold = NULL;
TimerHandle_t timer_since_release_handler = NULL;

TaskHandle_t btn_task_handle = NULL;

// this is set to true when the device goes to sleep and the pulse pin is
// still high. The device will be turned on inmediately and we will
// avoid counting one tick until we detect it is down again
RTC_DATA_ATTR bool exception_pulse_button_on_hold = false;

#define CLICK_PRESS_TIME_MS       400    // MUST BE BIGGER THAN 150
#define CLICK_HOLD_TIME_MS       3000
#define CLICK_RELEASE_TIME_MS     400

typedef enum ButtonState_e {
    NONE = 0,
    PRESS,
    RELEASE,
    SINGLE_CLICK,
    DOUBLE_CLICK,
    UNKNOWN_CLICK,
    HOLD
} button_state_t;

button_state_t button_state = NONE;

// static portMUX_TYPE counter_spinlock = portMUX_INITIALIZER_UNLOCKED;

// Load counter value from NVS
esp_err_t gm_counter_load_nvs()
{
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_nvs_handle);
    if (err == ESP_OK)
    {
        uint64_t saved_count = 0;
        err = nvs_get_u64(my_nvs_handle, NVS_KEY, &saved_count);
        if (err == ESP_OK)
        {
            current_summation_delivered.low = saved_count & 0x00000000FFFFFFFF;
            current_summation_delivered.high = (saved_count & 0x0000FFFF00000000) >> 32;
            ESP_LOGI(TAG, "Counter loaded: low=%lu high=%u", current_summation_delivered.low, current_summation_delivered.high);
        }
        else
        {
            ESP_LOGI(TAG, "Counter not found in memory so starting from 0");
            err = ESP_OK;
        }
    }
    else
    {
        device_extended_status |= ESP_ZB_ZCL_METERING_NV_MEMORY_ERROR;
        device_status |= ESP_ZB_ZCL_METERING_GAS_CHECK_METER;
        xEventGroupSetBits(report_event_group_handle, STATUS_REPORT | EXTENDED_STATUS_REPORT);

        ESP_LOGE(TAG, "Error opening NVS: %s", esp_err_to_name(err));
    }
    return err;
}

// Task to save the counter to NVS when value changes
void save_counter_task(void *arg)
{
    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        uint64_t to_save_count = current_summation_delivered.high;
        to_save_count <<= 32;
        to_save_count |= current_summation_delivered.low;
        esp_err_t err = nvs_set_u64(my_nvs_handle, NVS_KEY, to_save_count);
        if (err == ESP_OK)
        {
            nvs_commit(my_nvs_handle);
            ESP_LOGI(TAG, "Counter stored: low=%lu high=%d", current_summation_delivered.low, current_summation_delivered.high);
            #ifdef FEATURE_DEEP_SLEEP
            if (deep_sleep_task_handle != NULL)
            {
                TickType_t deep_sleep_time = dm_deep_sleep_time_ticks();
                if (xQueueSendToFront(deep_sleep_queue_handle, &deep_sleep_time, pdMS_TO_TICKS(100)) != pdTRUE)
                    ESP_LOGE(TAG, "Can't reschedule deep sleep timer");
            }
            #endif
        }
        else
        {
            device_extended_status |= ESP_ZB_ZCL_METERING_NV_MEMORY_ERROR;
            device_status |= ESP_ZB_ZCL_METERING_GAS_CHECK_METER;
            xEventGroupSetBits(report_event_group_handle, STATUS_REPORT | EXTENDED_STATUS_REPORT);
            ESP_LOGE(TAG, "Error saving NVS: %s", esp_err_to_name(err));
        }
        led_off();
    }
}

// Set counter value and save
esp_err_t gm_counter_set(const esp_zb_uint48_t *new_value)
{
    current_summation_delivered.low = new_value->low;
    current_summation_delivered.high = new_value->high;
    uint64_t current_summation_64 = current_summation_delivered.high;
    current_summation_64 <<= 32;
    current_summation_64 |= current_summation_delivered.low;
    xEventGroupSetBits(report_event_group_handle, CURRENT_SUMMATION_DELIVERED_REPORT);
    xTaskNotifyGive(save_counter_task_handle);
    return ESP_OK;
}

// Reset counter value to 0 and save
esp_err_t gm_counter_reset()
{
    esp_zb_uint48_t zero = {
        .low = 0,
        .high = 0};
    return gm_counter_set(&zero);
}

// Helper function to return the time in milliseconds since now and the other timeval
// received as a parameter
int32_t time_diff_ms(const struct timeval *other)
{
    struct timeval now;
    gettimeofday(&now, NULL);
    int32_t time_diff = (now.tv_sec - other->tv_sec) * 1000L + (now.tv_usec - other->tv_usec) / 1000L;
    return time_diff;
}

bool check_reel_state_change_is_ok(int level) {
    bool is_ok = last_reel_state_chage_time.tv_sec == 0 && last_reel_state_chage_time.tv_usec == 0;
    if (!is_ok) {
        int32_t elapsed_time_since_last_state_change = time_diff_ms(&last_reel_state_chage_time);
        // ESP_LOGI(TAG, "Elapsed time since last state change %ld", elapsed_time_since_last_state_change);
        is_ok = /*level != last_reel_state &&*/ elapsed_time_since_last_state_change > 150;
    }
    if (is_ok) {
        // transition detected
        // last_reel_state = level;
        gettimeofday(&last_reel_state_chage_time, NULL);
    }
    return is_ok;
}

// Check conditions to enable radio.
// There are 2 possible conditions:
// - time pass since last report
// - counter increased 10 times
void check_shall_enable_radio()
{
    if (zigbee_task_handle == NULL)
    {
        bool enable_radio =
            (last_report_sent_time.tv_sec == 0 && last_report_sent_time.tv_usec == 0) ||
            (time_diff_ms(&last_report_sent_time) / 1000 >= MUST_SYNC_MINIMUM_TIME);
        if (!enable_radio && last_summation_sent > 0)
        {
            uint64_t current_summation_64 = current_summation_delivered.high;
            current_summation_64 <<= 32;
            current_summation_64 |= current_summation_delivered.low;
            enable_radio = current_summation_64 - last_summation_sent >= COUNTER_REPORT_DIFF;
        }
        if (enable_radio)
        {
            xEventGroupSetBits(main_event_group_handle, SHALL_ENABLE_ZIGBEE);
            xEventGroupSetBits(report_event_group_handle, CURRENT_SUMMATION_DELIVERED_REPORT);
        }
    }
}

#ifdef FEATURE_MEASURE_BATTERY_LEVEL
// Computes if it is needed to measure battery voltage
void check_shall_measure_battery()
{
    bool measure_battery =
        (last_battery_measurement_time.tv_sec == 0 && last_battery_measurement_time.tv_usec == 0) ||
        (time_diff_ms(&last_battery_measurement_time) / 1000 >= MEASURE_BATTERY_VOLTAGE_TIME);
    if (measure_battery)
    {
        // battery voltage is measured with zigbee radio
        // turned on
        xEventGroupSetBits(main_event_group_handle, SHALL_MEASURE_BATTERY);
        xEventGroupSetBits(report_event_group_handle, CURRENT_SUMMATION_DELIVERED_REPORT);
    }
}
#endif

#ifdef FEATURE_LIGHT_SLEEP
// Periodic callback to check if radio or battery measurement is needed
// Called every MUST_SYNC_MINIMUM_TIME seconds instead of polling every 250ms
void periodic_checks_callback(TimerHandle_t xTimer)
{
    #ifdef FEATURE_MEASURE_BATTERY_LEVEL
    check_shall_measure_battery();
    #endif
    check_shall_enable_radio();
}
#endif

#ifdef FEATURE_MEASURE_FLOW_RATE
// After two consecutive values of current_summation_delivered this method
// computes the instantaneous_demand
// NOTE: there is one special task to set instantaneous demand to 0 when no
// values arrive
void gm_compute_instantaneous_demand(int time_diff_ms, bool fromISR)
{
    float time_diff_h = time_diff_ms / (1000.0 * 3600.0); // Convert time to hours/100
    int32_t _instantaneous_demand;
    if (time_diff_h > 0)
    {
        _instantaneous_demand = (int32_t)((1 / time_diff_h) + 0.5); // compute flow in m³/h
    }
    else
    {
        _instantaneous_demand = 0;
    }
    // check for a real change in the value
    esp_zb_int24_t new_instantaneous_demand;
    new_instantaneous_demand.low = _instantaneous_demand & 0x0000FFFF;
    new_instantaneous_demand.high = (_instantaneous_demand & 0x00FF0000) >> 16;
    if (new_instantaneous_demand.low != instantaneous_demand.low || new_instantaneous_demand.high != instantaneous_demand.high)
    {
        instantaneous_demand.low = new_instantaneous_demand.low;
        instantaneous_demand.high = new_instantaneous_demand.high;
        if (fromISR)
        {
            BaseType_t mustYield = pdFALSE;
            xEventGroupSetBitsFromISR(report_event_group_handle, INSTANTANEOUS_DEMAND_REPORT, &mustYield);
            portYIELD_FROM_ISR(mustYield);
            mustYield = pdFALSE;
            if (xTimerStartFromISR(reset_instantaneous_demand_timer, &mustYield) != pdPASS)
            {
                ESP_LOGE(TAG, "Can't reset instantaneous demand timer");
            }
            portYIELD_FROM_ISR(mustYield);
        }
        else
        {
            xEventGroupSetBits(report_event_group_handle, INSTANTANEOUS_DEMAND_REPORT);
            if (xTimerStart(reset_instantaneous_demand_timer, pdMS_TO_TICKS(100)) != pdPASS)
            {
                ESP_LOGE(TAG, "Can't reset instantaneous demand timer");
            }
        }
    }
}
#endif

// Adds one to current_summation_delivered and nothing else
void gm_counter_increment(bool fromISR)
{
    led_on();
    current_summation_delivered.low += 1; // Adds up 1 cent of m³
    if (current_summation_delivered.low == 0)
    {
        current_summation_delivered.high += 1;
    }
    uint64_t current_summation_64 = current_summation_delivered.high;
    current_summation_64 <<= 32;
    current_summation_64 |= current_summation_delivered.low;
    if (fromISR)
    {
        BaseType_t mustYield = pdFALSE;
        xEventGroupSetBitsFromISR(report_event_group_handle, CURRENT_SUMMATION_DELIVERED_REPORT, &mustYield);
        portYIELD_FROM_ISR(mustYield);
        mustYield = pdFALSE;
        vTaskNotifyGiveFromISR(save_counter_task_handle, &mustYield);
        portYIELD_FROM_ISR(mustYield);
    }
    else
    {
        xEventGroupSetBits(report_event_group_handle, CURRENT_SUMMATION_DELIVERED_REPORT);
        xTaskNotifyGive(save_counter_task_handle);
    }
    #ifdef FEATURE_MEASURE_FLOW_RATE
        if (last_increment_time.tv_sec != 0 && last_increment_time.tv_usec != 0)
            gm_compute_instantaneous_demand(time_diff_ms(&last_increment_time), fromISR);
        gettimeofday(&last_increment_time, NULL);
    #endif
}

// function called when the device leaves the zigee network
void leave_callback(esp_zb_zdp_status_t zdo_status, void *args)
{
    leaving_network = false;
    ESP_LOGI(TAG, "Leave status 0x%x", zdo_status);
}

void leave_action()
{
    if (esp_zb_bdb_dev_joined())
    {
        ESP_LOGI(TAG, "Leaving network");
        #ifdef FEATURE_DEEP_SLEEP
        xEventGroupSetBits(main_event_group_handle, SHALL_STOP_DEEP_SLEEP);
        #endif
        leaving_network = true;
        esp_zb_zdo_mgmt_leave_req_param_t leave_request = {
            .device_address = {},
            .dst_nwk_addr = 0xFFFF,
            .remove_children = 0,
            .rejoin = 0};
        esp_zb_get_long_address(leave_request.device_address);
        esp_zb_zdo_device_leave_req(&leave_request, leave_callback, NULL);
    } else {
        ESP_LOGE(TAG, "Short address is 0xFFFE so not leaving network!");
    }
}

#ifdef FEATURE_DEEP_SLEEP
// Compute how long to wait for sleep depending on device conditions
TickType_t dm_deep_sleep_time_ticks()
{
    const TickType_t before_deep_sleep_time_ticks = (zigbee_task_handle != NULL ? TIME_TO_SLEEP_ZIGBEE_ON : TIME_TO_SLEEP_ZIGBEE_OFF);
    ESP_LOGD(TAG, "Start one-shot timer for %ldms to enter the deep sleep", before_deep_sleep_time_ticks);
    return before_deep_sleep_time_ticks;
}

// task to govern the deep sleep timeout
void deep_sleep_controller_task(void *arg)
{
    ESP_LOGI(TAG, "Deep sleep task started");
    led_off();
    while (true)
    {
        TickType_t new_timer_value;
        if (xQueueReceive(deep_sleep_queue_handle, &new_timer_value, portMAX_DELAY) == pdTRUE)
        {
            // REMOVE FOR PRODUCTION. We are going to disable deep sleep for testing
            // if (xTimerStop(deep_sleep_timer, pdMS_TO_TICKS(100)) != pdPASS)
            // {
            //     ESP_LOGE(TAG, "Can't stop deep sleep timer");
            // }
            ESP_LOGI(TAG, "Deep sleep timer value received %d", new_timer_value);
            if (deep_sleep_gracie_period.tv_sec > 0) {
                int elapsed = time_diff_ms(&deep_sleep_gracie_period);
                if (elapsed < 0) {
                    ESP_LOGI(TAG, "Deep sleep timer Under grace period, ignored");
                    continue;
                }
            }
            if (new_timer_value == portMAX_DELAY)
            {
                if (xTimerStop(deep_sleep_timer, pdMS_TO_TICKS(100)) != pdPASS)
                {
                    ESP_LOGE(TAG, "Can't stop deep sleep timer");
                }
            }
            else
            {
                if (xTimerIsTimerActive(deep_sleep_timer) == pdFALSE) {
                    ESP_LOGI(TAG, "Deep sleep timer started");
                    xTimerStart(deep_sleep_timer, new_timer_value);
                } else {
                    if (xTimerChangePeriod(deep_sleep_timer, new_timer_value, pdMS_TO_TICKS(100)) != pdPASS)
                    {
                        ESP_LOGE(TAG, "Can't change deep sleep timer time");
                    }
                }
            }
        }
    }
}
#endif

// task to manage main button press events
void btn_press_task(void *arg)
{
    ESP_LOGI(TAG, "Main button press task started");
    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (button_state == NONE) {
            button_state = PRESS;
            xTaskNotify(btn_task_handle, button_state, eSetValueWithOverwrite);
        }
        TickType_t current_time_since_press = xTimerGetPeriod(timer_since_press_handler);
        TickType_t current_time_since_hold = xTimerGetPeriod(t_detect_hold);
        TickType_t click_press_time_adjusted = pdMS_TO_TICKS(CLICK_PRESS_TIME_MS);
        TickType_t hold_time_adjusted = pdMS_TO_TICKS(CLICK_HOLD_TIME_MS);
        #ifdef FEATURE_DEEP_SLEEP
        if (started_from_deep_sleep)
        {
            TickType_t startTime = pdMS_TO_TICKS(150);
            // substract 150 ms to timers
            click_press_time_adjusted = click_press_time_adjusted - startTime; // measured time for the device to start
            hold_time_adjusted -= hold_time_adjusted - startTime;
        }
        #endif
        if (current_time_since_press != click_press_time_adjusted && xTimerChangePeriod(timer_since_press_handler, click_press_time_adjusted, pdMS_TO_TICKS(100)) != pdPASS)
        {
            ESP_LOGE(TAG, "Can't set press timer time to %dms", click_press_time_adjusted);
        }
        if (current_time_since_hold != hold_time_adjusted && xTimerChangePeriod(t_detect_hold, hold_time_adjusted, pdMS_TO_TICKS(100)) != pdPASS)
        {
            ESP_LOGE(TAG, "Can't set hold timer time to %dms", hold_time_adjusted);
        }
        if (xTimerStart(timer_since_press_handler, pdMS_TO_TICKS(100)) != pdPASS)
        {
            ESP_LOGE(TAG, "Can't start press timer");
        }
        if (button_state == PRESS) {
            if (xTimerStart(t_detect_hold, pdMS_TO_TICKS(100)) != pdPASS)
            {
                ESP_LOGE(TAG, "Can't start hold timer");
            }
        } else {
            if (xTimerStop(timer_since_release_handler, pdMS_TO_TICKS(100)) != pdPASS) {
                ESP_LOGE(TAG, "Can't stop 0.2s timer since release");
            }
        }
    }
}

void btn_release_task(void *arg)
{
    ESP_LOGI(TAG, "Main button release task started");
    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (button_state == PRESS) {
            button_state = RELEASE;
            xTaskNotify(btn_task_handle, button_state, eSetValueWithOverwrite);
        }
        if (button_state == HOLD) {
            button_state = NONE;
            xTaskNotify(btn_task_handle, button_state, eSetValueWithOverwrite);
            continue;
        }
        if (xTimerIsTimerActive(timer_since_press_handler) != pdFALSE) {
            xTimerStop(timer_since_press_handler, pdMS_TO_TICKS(100));
            if (button_state == SINGLE_CLICK) {
                button_state = DOUBLE_CLICK;
            } else if (button_state == DOUBLE_CLICK) {
                button_state = UNKNOWN_CLICK;
            } else if (button_state != UNKNOWN_CLICK && button_state != DOUBLE_CLICK) {
                button_state = SINGLE_CLICK;
            }
        }
        if (xTimerIsTimerActive(t_detect_hold) != pdFALSE) {
            xTimerStop(t_detect_hold, pdMS_TO_TICKS(100));
        }
        if (xTimerStart(timer_since_release_handler, pdMS_TO_TICKS(100)) != pdPASS)
        {
            ESP_LOGE(TAG, "Can't start 0.2s timer after release");
        }
    }
}

void timer_since_press_cb(TimerHandle_t xTimer)
{
    if (button_state == SINGLE_CLICK || button_state == DOUBLE_CLICK || button_state == UNKNOWN_CLICK) {
        button_state = UNKNOWN_CLICK;
        xTaskNotify(btn_task_handle, button_state, eSetValueWithOverwrite);
    }
}

void timer_detect_hold_cb(TimerHandle_t xTimer)
{
    if (button_state == PRESS) {
        button_state = HOLD;
        xTaskNotify(btn_task_handle, button_state, eSetValueWithOverwrite);
    }
}

void timer_since_release_cb(TimerHandle_t xTimer)
{
    bool single_or_double_or_unknown_click = button_state == SINGLE_CLICK || button_state == DOUBLE_CLICK || button_state == UNKNOWN_CLICK;
    if (single_or_double_or_unknown_click) {
        xTaskNotify(btn_task_handle, button_state, eSetValueWithOverwrite);
    } else if (button_state != NONE) {
        button_state = NONE;
        xTaskNotify(btn_task_handle, button_state, eSetValueWithOverwrite);
    }
}

void btn_task(void *arg)
{
    ESP_LOGI(TAG, "Main button action task started");
    while (true)
    {
        uint32_t state;
        xTaskNotifyWait(0x00, 0xFF, &state, portMAX_DELAY);
        switch (state) {
            case PRESS:
                ESP_LOGI(TAG, "Button press");
                led_on();
                xEventGroupSetBits(main_event_group_handle, SHALL_ENABLE_ZIGBEE);
                #ifdef FEATURE_DEEP_SLEEP
                if (deep_sleep_task_handle != NULL)
                {
                    TickType_t deep_sleep_time = pdMS_TO_TICKS(TIME_TO_SLEEP_ZIGBEE_STARTING);
                    if (xQueueSendToFront(deep_sleep_queue_handle, &deep_sleep_time, pdMS_TO_TICKS(100)) != pdTRUE)
                        ESP_LOGE(TAG, "Can't reschedule deep sleep timer");
                    gettimeofday(&deep_sleep_gracie_period, NULL);
                    deep_sleep_gracie_period.tv_sec += (TIME_TO_SLEEP_ZIGBEE_STARTING / 1000);
                }
                #endif
                break;
            case RELEASE:
                ESP_LOGI(TAG, "Button release");
                break;
            case SINGLE_CLICK:
                ESP_LOGI(TAG, "Single click detected");
                xEventGroupSetBits(report_event_group_handle,
                    CURRENT_SUMMATION_DELIVERED_REPORT | 
                    #ifdef FEATURE_MEASURE_BATTERY_LEVEL
                    BATTERY_REPORT | 
                    #endif
                    STATUS_REPORT | EXTENDED_STATUS_REPORT);
                #ifdef FEATURE_MEASURE_BATTERY_LEVEL
                xEventGroupSetBits(main_event_group_handle, SHALL_MEASURE_BATTERY);
                #endif
                // reset device status
                device_status = 0x0;
                device_extended_status = 0x0;
                // reset button state
                button_state = NONE;
                xTaskNotify(btn_task_handle, button_state, eSetValueWithOverwrite);
                break;
            case DOUBLE_CLICK:
                ESP_LOGI(TAG, "Double click detected");
                button_state = NONE;
                xTaskNotify(btn_task_handle, button_state, eSetValueWithOverwrite);
                esp_restart();
                break;
            case UNKNOWN_CLICK:
                ESP_LOGI(TAG, "Unknown click detected");
                button_state = NONE;
                xTaskNotify(btn_task_handle, button_state, eSetValueWithOverwrite);
                break;
            case HOLD:
                ESP_LOGI(TAG, "Hold detected");
                leave_action();
                break;
            case NONE:
                ESP_LOGI(TAG, "Button state reset");
                led_off();
                break;
            default:
                ESP_LOGI(TAG, "Unknown button state");
                break;
        }
    }
}

#ifdef FEATURE_MEASURE_FLOW_RATE
// set the instantaneous demand to 0 and indicates to report this value
void reset_instantaneous_demand_cb(TimerHandle_t xTimer)
{
    ESP_LOGI(TAG, "Reset instantaneous demand");
    if (instantaneous_demand.low != 0 || instantaneous_demand.high != 0)
    {
        instantaneous_demand.low = 0;
        instantaneous_demand.high = 0;
        xEventGroupSetBits(report_event_group_handle, INSTANTANEOUS_DEMAND_REPORT);
    }
}
#endif

void reschedule_event(uint8_t event) {
    xEventGroupSetBits(main_event_group_handle, event);
}

// device main loop activities, note
// zigbee activities are not part of the
// main loop, just the critical activities
// to maintain the counter
void gm_main_loop_task(void *arg)
{
    ESP_LOGI(TAG, "Main loop task started");
    ESP_ERROR_CHECK(check_boot_partition_change());
    #ifdef FEATURE_DEEP_SLEEP
    // start deep sleep controller
    xEventGroupSetBits(main_event_group_handle, SHALL_START_DEEP_SLEEP);
    #endif
    while (true)
    {
        EventBits_t uxBits = xEventGroupWaitBits(
            main_event_group_handle
            , SHALL_ENABLE_ZIGBEE
            #ifdef FEATURE_MEASURE_BATTERY_LEVEL
            | SHALL_MEASURE_BATTERY
            #endif
            | SHALL_DISABLE_ZIGBEE
            #ifdef FEATURE_DEEP_SLEEP
            | SHALL_START_DEEP_SLEEP
            | SHALL_STOP_DEEP_SLEEP
            #endif
            ,pdTRUE // clear on exit
            ,pdFALSE
            #ifdef FEATURE_LIGHT_SLEEP
            ,portMAX_DELAY  // Block indefinitely - triggered by events/callbacks instead of polling
            #else
            ,TIME_TO_SLEEP_ZIGBEE_ON + pdMS_TO_TICKS(50)
            #endif
        );
        if (uxBits != 0)
        {
            #ifdef FEATURE_MEASURE_BATTERY_LEVEL
            if ((uxBits & SHALL_MEASURE_BATTERY) == SHALL_MEASURE_BATTERY)
            {
                ESP_LOGI(TAG, "Measuring battery capacity");
                gettimeofday(&last_battery_measurement_time, NULL);
                last_battery_measurement_time.tv_sec += 20; // prevent entering a loop until battery is measured
                xTaskNotifyGiveIndexed(adc_task_handle, 0);
            }
            #endif
            if ((uxBits & SHALL_ENABLE_ZIGBEE) == SHALL_ENABLE_ZIGBEE)
            {
                if (zigbee_task_handle == NULL) {
                    ESP_LOGI(TAG, "Starting zigbee radio functionality");
                    #ifdef CONFIG_EXTERNAL_ANTENNA
                    uint64_t antenna_switch_pin = 1ULL << GPIO_NUM_3;
                    uint64_t antenna_mode_pin = 1ULL << GPIO_NUM_14;
                    gpio_config_t anthena_conf = {
                        .intr_type = GPIO_INTR_DISABLE,
                        .mode = GPIO_MODE_OUTPUT,
                        .pin_bit_mask = antenna_switch_pin | antenna_mode_pin,
                        .pull_down_en = GPIO_PULLDOWN_DISABLE,
                        .pull_up_en = GPIO_PULLUP_DISABLE
                    };
                    ESP_ERROR_CHECK(gpio_config(&anthena_conf));

                    ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_3, 0));
                    vTaskDelay(pdMS_TO_TICKS(100));
                    ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_14, 1));
                    #endif
                    /* Start Zigbee stack task */
                    #ifdef FEATURE_DEEP_SLEEP
                    TickType_t deep_sleep_time = pdMS_TO_TICKS(TIME_TO_SLEEP_ZIGBEE_STARTING);
                    if (xQueueSendToFront(deep_sleep_queue_handle, &deep_sleep_time, pdMS_TO_TICKS(100)) != pdTRUE)
                        ESP_LOGE(TAG, "Can't reschedule deep sleep timer");
                    gettimeofday(&deep_sleep_gracie_period, NULL);
                    deep_sleep_gracie_period.tv_sec += (TIME_TO_SLEEP_ZIGBEE_STARTING / 1000);
                    #endif
                    if (xTaskCreate(esp_zb_task, "Zigbee_main", 10240, NULL, 10, &zigbee_task_handle) != pdTRUE)
                        ESP_LOGE(TAG, "can't create zigbee task");
                    xEventGroupClearBits(main_event_group_handle, SHALL_ENABLE_ZIGBEE);
                }
            }
            if ((uxBits & SHALL_DISABLE_ZIGBEE) == SHALL_DISABLE_ZIGBEE)
            {
                if (zigbee_task_handle != NULL) {
                    ESP_LOGI(TAG, "Stoping zigbee radio functionality");
                    // TODO: see how the ADC task terminates.
                    // depends on https://github.com/espressif/esp-zigbee-sdk/issues/561
                    // at the end it is required to set:
                    //  zigbee_enabled = false;
                    #ifdef FEATURE_DEEP_SLEEP
                    if (deep_sleep_task_handle != NULL)
                    {
                        TickType_t deep_sleep_time = portMAX_DELAY;
                        if (xQueueSendToFront(deep_sleep_queue_handle, &deep_sleep_time, pdMS_TO_TICKS(100)) != pdTRUE)
                            ESP_LOGE(TAG, "Can't reschedule deep sleep timer");
                    }
                    #endif
                    vTaskDelete(zigbee_task_handle);
                    zigbee_task_handle = NULL;
                }
            }
            #ifdef FEATURE_DEEP_SLEEP
            if ((uxBits & SHALL_START_DEEP_SLEEP) == SHALL_START_DEEP_SLEEP)
            {
                if (deep_sleep_task_handle != NULL) {
                    esp_zb_scheduler_alarm(reschedule_event, SHALL_START_DEEP_SLEEP, 250);
                } else {
                    ESP_LOGI(TAG, "Starting deep sleep functionality");
                    ESP_ERROR_CHECK(xTaskCreate(deep_sleep_controller_task, "deep_sleep", 2048, NULL, 20, &deep_sleep_task_handle) != pdPASS);
                }
            }
            if ((uxBits & SHALL_STOP_DEEP_SLEEP) == SHALL_STOP_DEEP_SLEEP)
            {
                if (deep_sleep_task_handle != NULL) {
                    ESP_LOGI(TAG, "Stoping deep sleep functionality");
                    vTaskDelete(deep_sleep_task_handle);
                    deep_sleep_task_handle = NULL;
                    xEventGroupClearBits(main_event_group_handle, SHALL_STOP_DEEP_SLEEP);
                }
            }
            #endif
        }
        #ifdef FEATURE_DEEP_SLEEP 
        if (deep_sleep_task_handle != NULL)
        {
            int elapsed = time_diff_ms(&deep_sleep_gracie_period);
            if (elapsed > 0) {
                TickType_t deep_sleep_time = dm_deep_sleep_time_ticks();
                // ESP_LOGI(TAG, "Will check deep sleep in %d...", deep_sleep_time);
                if (xQueueSendToFront(deep_sleep_queue_handle, &deep_sleep_time, pdMS_TO_TICKS(100)) != pdTRUE)
                    ESP_LOGE(TAG, "Can't reschedule deep sleep timer");
            } else {
                ESP_LOGI(TAG, "Will wait for %ds ... to start deep sleep", -(elapsed/1000));
            }
        }
        #endif
        int level = gpio_get_level(PULSE_PIN);
        check_reel_state_change_is_ok(level);
    }
}

// callback to start deep sleep
#ifdef FEATURE_DEEP_SLEEP
void enter_deep_sleep_cb(TimerHandle_t xTimer)
{
    /* Enter deep sleep */
    if (deep_sleep_task_handle == NULL)
    {
        ESP_LOGI(TAG, "Enter deep sleep cancelled");
        return;
    }
    ESP_LOGI(TAG, "Enter deep sleep");
    exception_pulse_button_on_hold = gpio_get_level(PULSE_PIN) == 1;
    gettimeofday(&sleep_enter_time, NULL);
    esp_deep_sleep_start();
}
#endif

// configure deep sleep for the gas meter
esp_err_t gm_deep_sleep_init()
{
    const uint64_t gpio_pulse_pin_mask = BIT(PULSE_PIN);
    const uint64_t gpio_mainbtn_pin_mask = BIT(MAIN_BTN);

    // wake-up reason:
    struct timeval gpio_time;
    gettimeofday(&gpio_time, NULL);
    #ifdef FEATURE_DEEP_SLEEP
    if (gpio_time.tv_sec < sleep_enter_time.tv_sec) {
        sleep_enter_time.tv_sec = gpio_time.tv_sec;
        sleep_enter_time.tv_usec = gpio_time.tv_usec;
    }
    int sleep_time_ms = (gpio_time.tv_sec - sleep_enter_time.tv_sec) * 1000 +
                        (gpio_time.tv_usec - sleep_enter_time.tv_usec) / 1000;
    #endif
    esp_sleep_wakeup_cause_t wake_up_cause = esp_sleep_get_wakeup_cause();
    switch (wake_up_cause)
    {
        case ESP_SLEEP_WAKEUP_TIMER:
        {
            #ifdef FEATURE_DEEP_SLEEP
            ESP_LOGI(TAG, "Wake up from timer. Time spent in deep sleep and boot: %dms", sleep_time_ms);
            #else
            ESP_LOGI(TAG, "Wake up from timer");
            #endif
            xEventGroupSetBits(main_event_group_handle, SHALL_ENABLE_ZIGBEE);
            xEventGroupSetBits(report_event_group_handle, CURRENT_SUMMATION_DELIVERED_REPORT);
            break;
        }
        case ESP_SLEEP_WAKEUP_EXT1:
        {
            bool resolved = false;
            uint64_t ext1mask = esp_sleep_get_ext1_wakeup_status();
            if ((ext1mask & gpio_mainbtn_pin_mask) == gpio_mainbtn_pin_mask)
            { // wakeup from MAIN_BTN
                #ifdef FEATURE_DEEP_SLEEP
                ESP_LOGI(TAG, "Wake up from MAIN BUTTON. Time spent in deep sleep and boot: %dms", sleep_time_ms);
                started_from_deep_sleep = true;
                #else
                ESP_LOGI(TAG, "Wake up from MAIN BUTTON");
                #endif
                #ifdef FEATURE_MEASURE_BATTERY_LEVEL
                xEventGroupSetBits(main_event_group_handle, SHALL_MEASURE_BATTERY);
                #endif
                xEventGroupSetBits(main_event_group_handle, SHALL_ENABLE_ZIGBEE);
                xEventGroupSetBits(report_event_group_handle, CURRENT_SUMMATION_DELIVERED_REPORT);
                int level = gpio_get_level(MAIN_BTN);
                if (level == 0)
                {
                    xTaskNotifyGive(btn_release_task_handle);
                }
                else
                {
                    xTaskNotifyGive(btn_press_task_handle);
                }
                resolved = true;
            }
            if ((ext1mask & gpio_pulse_pin_mask) == gpio_pulse_pin_mask)
            { 
                // wakeup from PULSE_PIN
                #ifdef FEATURE_DEEP_SLEEP
                ESP_LOGI(TAG, "Wake up from GAS PULSE. Time spent in deep sleep and boot: %dms", sleep_time_ms);
                #else
                ESP_LOGI(TAG, "Wake up from GAS PULSE");
                #endif
                // check_gpio_time = true;
                int level = gpio_get_level(PULSE_PIN);
                bool is_ok = check_reel_state_change_is_ok(level);
                // if PULSE_PIN is low AND check_gpio_time is true we
                // miss the interrupt so count it now
                if (is_ok) {
                    if (!exception_pulse_button_on_hold && level == 1)
                    {
                        gm_counter_increment(false);
                    }
                    else if (exception_pulse_button_on_hold && level == 0)
                    { // rare, but not impossible
                        exception_pulse_button_on_hold = false;
                    }
                }
                #ifdef FEATURE_DEEP_SLEEP
                else
                {
                    TickType_t deep_sleep_time = portMAX_DELAY;
                    if (xQueueSendToFront(deep_sleep_queue_handle, &deep_sleep_time, pdMS_TO_TICKS(100)) != pdTRUE)
                        ESP_LOGE(TAG, "Can't reschedule deep sleep timer");
                }
                #endif
                resolved = true;
            }
            if (!resolved)
            {
                #ifdef FEATURE_DEEP_SLEEP
                ESP_LOGI(TAG, "Wake up from unknown GPIO. Time spent in deep sleep and boot: %dms", sleep_time_ms);
                #else
                ESP_LOGI(TAG, "Wake up from unknown GPIO");
                #endif
            }
            break;
        }
        case ESP_SLEEP_WAKEUP_UNDEFINED:
        default:
            ESP_LOGI(TAG, "Not a deep sleep reset");
            xEventGroupSetBits(main_event_group_handle, SHALL_ENABLE_ZIGBEE);
            xEventGroupSetBits(report_event_group_handle, CURRENT_SUMMATION_DELIVERED_REPORT);
            break;
    }
    ESP_LOGI(TAG, "Check if zigbee radio shall be enabled");
    check_shall_enable_radio();
    #ifdef FEATURE_MEASURE_BATTERY_LEVEL
    ESP_LOGI(TAG, "Check if battery shall be measured");
    check_shall_measure_battery();
    #endif

    #if defined(FEATURE_DEEP_SLEEP) || defined(FEATURE_LIGHT_SLEEP)
        if (last_report_sent_time.tv_sec == 0 && last_report_sent_time.tv_usec == 0) {
            ESP_LOGI(TAG, "Last report sent time initialized");
            gettimeofday(&last_report_sent_time, NULL);
        }
        /* Set the methods of how to wake up: */
        /* 1. RTC timer waking-up */
        /* This is useless in LIGHT_SLEEP */
        ESP_LOGI(TAG, "Configuring wake up methods");
        int report_time_s = (gpio_time.tv_sec - last_report_sent_time.tv_sec) +
                            (gpio_time.tv_usec - last_report_sent_time.tv_usec) / 1000000;
        if (report_time_s > MUST_SYNC_MINIMUM_TIME)
            report_time_s = MUST_SYNC_MINIMUM_TIME;
        const uint64_t wakeup_time_sec = MUST_SYNC_MINIMUM_TIME - report_time_s;
        ESP_LOGI(TAG, "Enabling timer wakeup, %llds", wakeup_time_sec);
        ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000));

        /* PULSE_PIN and MAIN_BTN wake up on pull up */

        ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup(gpio_mainbtn_pin_mask | gpio_pulse_pin_mask , ESP_EXT1_WAKEUP_ANY_HIGH));
        
        #ifdef FEATURE_DEEP_SLEEP
            esp_deep_sleep_disable_rom_logging();
        #endif
    #endif

    return ESP_OK;
}

// PULSE_PIN - GPIO Interruption handler
void IRAM_ATTR gpio_pulse_isr_handler(void *arg)
{
    if ((last_interrupt_time.tv_sec != 0 || last_interrupt_time.tv_usec != 0) && time_diff_ms(&last_interrupt_time) <= REED_DEBOUNCE_TIMEOUT)
    {
        return; // DEBOUNCE
    }
    struct timeval now;
    gettimeofday(&now, NULL);
    last_interrupt_time.tv_sec = now.tv_sec;
    last_interrupt_time.tv_usec = now.tv_usec;

    // Increment current_summation_delivery
    // read pin to determine failing or raising edge
    int level = gpio_get_level(PULSE_PIN);
    bool is_ok = check_reel_state_change_is_ok(level);
    if (is_ok && level == 1)
    {
        gm_counter_increment(true);
    }
}

// MAIN_BTN - GPIO Interruption handler
void IRAM_ATTR gpio_btn_isr_handler(void *arg)
{
    // Last time wakeup pin was received
    static int64_t last_main_btn_time = 0;

    int level = gpio_get_level(MAIN_BTN);
    int64_t current_time = esp_timer_get_time();
    int64_t time_diff_ms = (current_time - last_main_btn_time) / 1000;
    if (last_main_btn_time > 0 && time_diff_ms <= BTN_DEBOUNCE_TIMEOUT)
        return; // DEBOUNCE

    BaseType_t mustYield = pdFALSE;
    if (level == 1 && button_state != PRESS)
    {
        #ifdef FEATURE_DEEP_SLEEP
        started_from_deep_sleep = false;
        #endif
        vTaskNotifyGiveFromISR(btn_press_task_handle, &mustYield);
    }
    if (level == 0 || button_state == PRESS)
    {
        vTaskNotifyGiveFromISR(btn_release_task_handle, &mustYield);
    }

    last_main_btn_time = current_time;
    portYIELD_FROM_ISR(mustYield);
}

// Configure interrupt to catch pulses from magnetic sensor
// NOTE: When the device is in deep sleep internal pull-up and pull-down
// resistors aren't used so they are disabled.
// In order to save battery I decided to use pull-down because this is
// how the sensor should stay most of the time
esp_err_t gm_gpio_interrup_init()
{
    #ifdef FEATURE_LIGHT_SLEEP
    ESP_RETURN_ON_ERROR(esp_sleep_enable_ext1_wakeup(BIT(MAIN_BTN) | BIT(PULSE_PIN), ESP_EXT1_WAKEUP_ANY_LOW), TAG, "Can't enable ext1 wakeup for MAIN_BTN and PULSE_PIN");
    ESP_RETURN_ON_ERROR(gpio_wakeup_enable(PULSE_PIN,GPIO_INTR_LOW_LEVEL), TAG, "Can't enable gpio wakeup for PULSE_PIN HIGH");
    ESP_RETURN_ON_ERROR(gpio_wakeup_enable(MAIN_BTN,GPIO_INTR_LOW_LEVEL), TAG, "Can't enable gpio wakeup for MAIN_BTN HIGH");
    #endif

                                                                   //      __
    gpio_config_t io_conf_pulse = {                                // ____|  |_____
                                   .intr_type = GPIO_INTR_POSEDGE, //     ^- Interrupt rising edge
                                   .mode = GPIO_MODE_INPUT,        // Input pin
                                   .pin_bit_mask = BIT(PULSE_PIN),
                                   .pull_down_en = GPIO_PULLDOWN_DISABLE,
                                   .pull_up_en = GPIO_PULLUP_DISABLE};
    ESP_RETURN_ON_ERROR(gpio_config(&io_conf_pulse), TAG, "Can't config gpio for PULSE_PIN and MAIN_PIN pins");
                                                                     //      __
    gpio_config_t io_conf_mainbtn = {                                // ____|  |_____
                                     .intr_type = GPIO_INTR_ANYEDGE, //     ^--^- Interrupt both edges
                                     .mode = GPIO_MODE_INPUT,        // Input pin
                                     .pin_bit_mask = BIT(MAIN_BTN),
                                     .pull_down_en = GPIO_PULLDOWN_DISABLE,
                                     .pull_up_en = GPIO_PULLUP_DISABLE};
    ESP_RETURN_ON_ERROR(gpio_config(&io_conf_mainbtn), TAG, "Can't config gpio for PULSE_PIN and MAIN_PIN pins");

    // Configure and register interrupt service
    ESP_RETURN_ON_ERROR(gpio_install_isr_service(ESP_INTR_FLAG_LOWMED), TAG, "Can't install isr service");
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(PULSE_PIN, gpio_pulse_isr_handler, NULL), TAG, "Can't add PULSE_PIN interrupt handler");
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(MAIN_BTN, gpio_btn_isr_handler, NULL), TAG, "Can't add MAIN_BTN interrupt handler");

    return ESP_OK;
}

// configure power save
esp_err_t esp_zb_power_save_init(void)
{
    esp_err_t rc = ESP_OK;
#ifdef CONFIG_PM_ENABLE
    int cur_cpu_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
    #if defined(FEATURE_DEEP_SLEEP) || defined(FEATURE_LIGHT_SLEEP)
    #ifdef FEATURE_DEEP_SLEEP
    esp_pm_config_t pm_config = {
        .max_freq_mhz = cur_cpu_freq_mhz,
        .min_freq_mhz = 80,  // Enable dynamic frequency scaling down to 80 MHz
    #if CONFIG_FREERTOS_USE_TICKLESS_IDLE
        // TODO: explore why this causes a problem
        // When the device enters deep sleep after a 3s period
        // caused by the counter going up one tick
        // the device wakes up on RTC automatically and this
        // is not desired as it will turn on Zigbee radio
        // as it is defined in code

        // .light_sleep_enable = true
    #endif
    };
    #endif
    #ifdef FEATURE_LIGHT_SLEEP
    esp_pm_config_t pm_config = {
        .max_freq_mhz = cur_cpu_freq_mhz,
        .min_freq_mhz = 80,
        .light_sleep_enable = true
    };
    #endif
    #else
        esp_pm_config_t pm_config = {
        .max_freq_mhz = cur_cpu_freq_mhz,
        .min_freq_mhz = cur_cpu_freq_mhz,
        .light_sleep_enable = false
    };
    #endif
    rc = esp_pm_configure(&pm_config);
#endif
    return rc;
}

esp_err_t report_reset_reason()
{
    esp_reset_reason_t reset_reason = esp_reset_reason();
    switch (reset_reason)
    {
    case ESP_RST_UNKNOWN: //!< Reset reason can not be determined
        ESP_LOGI(TAG, "Unknown reset reason");
        break;
    case ESP_RST_POWERON: //!< Reset due to power-on event
        ESP_LOGI(TAG, "Powerdown reset reason");
        break;
    case ESP_RST_EXT: //!< Reset by external pin (not applicable for ESP32)
        ESP_LOGI(TAG, "external pin reset reason");
        break;
    case ESP_RST_SW: //!< Software reset via esp_restart
        ESP_LOGI(TAG, "Software reset reason");
        break;
    case ESP_RST_PANIC: //!< Software reset due to exception/panic
        device_extended_status |= ESP_ZB_ZCL_METERING_PROGRAM_MEMORY_ERROR;
        ESP_LOGI(TAG, "Esception/panic reset reason");
        return ESP_FAIL;
    case ESP_RST_INT_WDT: //!< Reset (software or hardware) due to interrupt watchdog
        ESP_LOGI(TAG, "Reset (Software of Hardware) reset reason");
        break;
    case ESP_RST_TASK_WDT: //!< Reset due to task watchdog
        ESP_LOGI(TAG, "Task watchdog reset reason");
        device_extended_status |= ESP_ZB_ZCL_METERING_WATCHDOG_ERROR;
        return ESP_FAIL;
    case ESP_RST_WDT: //!< Reset due to other watchdogs
        ESP_LOGI(TAG, "Other watchdog reset reason");
        device_extended_status |= ESP_ZB_ZCL_METERING_WATCHDOG_ERROR;
        return ESP_FAIL;
    case ESP_RST_DEEPSLEEP: //!< Reset after exiting deep sleep mode
        ESP_LOGI(TAG, "After exiting deep sleep reset reason");
        break;
    case ESP_RST_BROWNOUT: //!< Brownout reset (software or hardware)
        ESP_LOGI(TAG, "Brownout reset reason");
        device_extended_status |= ESP_ZB_ZCL_METERING_BATTERY_FAILURE;
        return ESP_FAIL;
    case ESP_RST_SDIO: //!< Reset over SDIO
        ESP_LOGI(TAG, "SDIO reset reason");
        break;
    case ESP_RST_USB: //!< Reset by USB peripheral
        ESP_LOGI(TAG, "USB reset reason");
        break;
    case ESP_RST_JTAG: //!< Reset by JTAG
        ESP_LOGI(TAG, "JTAG reset reason");
        break;
    case ESP_RST_EFUSE: //!< Reset due to efuse error
        ESP_LOGI(TAG, "Efuse error reset reason");
        break;
    case ESP_RST_PWR_GLITCH: //!< Reset due to power glitch detected
        ESP_LOGI(TAG, "Power glitch detected reset reason");
        break;
    case ESP_RST_CPU_LOCKUP: //!< Reset due to CPU lock up (double exception)
        ESP_LOGI(TAG, "CPU lock up reset reason");
        break;
    }
    return ESP_OK;
}

// Entry point
void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_ERROR);
    esp_log_level_set(TAG, ESP_LOG_INFO);
    ESP_LOGD(TAG, "\n");
    ESP_LOGI(TAG, "Starting Zigbee GasCounter...");
    esp_err_t reset_error = report_reset_reason();

    #ifdef FEATURE_DEEP_SLEEP
    ESP_ERROR_CHECK((deep_sleep_timer = xTimerCreate("deep_sleep_timer", portMAX_DELAY, pdFALSE, "d_s_t", enter_deep_sleep_cb)) == NULL ? ESP_FAIL : ESP_OK);
    #endif

    // intelligent button timers
    ESP_ERROR_CHECK((timer_since_press_handler = xTimerCreate("0.2s_since_press", pdMS_TO_TICKS(CLICK_PRESS_TIME_MS), pdFALSE, "2_t_p", timer_since_press_cb)) == NULL ? ESP_FAIL : ESP_OK);
    ESP_ERROR_CHECK((t_detect_hold = xTimerCreate("4s_since_press", pdMS_TO_TICKS(CLICK_HOLD_TIME_MS), pdFALSE, "4_s_p", timer_detect_hold_cb)) == NULL ? ESP_FAIL : ESP_OK);
    ESP_ERROR_CHECK((timer_since_release_handler = xTimerCreate("0.2s_since_release", pdMS_TO_TICKS(CLICK_RELEASE_TIME_MS), pdFALSE, "2_t_r", timer_since_release_cb)) == NULL ? ESP_FAIL : ESP_OK);

    // Periodic timer for radio/battery checks (replaces 250ms polling)
    // Set period to slightly less than MUST_SYNC_MINIMUM_TIME to ensure checks happen
    #ifdef FEATURE_LIGHT_SLEEP
    ESP_ERROR_CHECK((periodic_checks_timer = xTimerCreate("periodic_checks", pdMS_TO_TICKS(MUST_SYNC_MINIMUM_TIME * 1000 - 100), pdTRUE, "p_c", periodic_checks_callback)) == NULL ? ESP_FAIL : ESP_OK);
    if (periodic_checks_timer != NULL) {
        xTimerStart(periodic_checks_timer, 0);
    }
    #endif

    #ifdef FEATURE_DEEP_SLEEP
    ESP_ERROR_CHECK((deep_sleep_queue_handle = xQueueCreate(1, sizeof(TickType_t))) == NULL ? ESP_FAIL : ESP_OK);
    #endif
    ESP_ERROR_CHECK((main_event_group_handle = xEventGroupCreate()) == NULL ? ESP_FAIL : ESP_OK);
    ESP_ERROR_CHECK((report_event_group_handle = xEventGroupCreate()) == NULL ? ESP_FAIL : ESP_OK);
    ESP_ERROR_CHECK(xTaskCreate(save_counter_task, "save_counter", 2048, NULL, 15, &save_counter_task_handle) != pdPASS);
    ESP_ERROR_CHECK(xTaskCreate(btn_press_task, "btn_press", 2048, NULL, 5, &btn_press_task_handle) != pdPASS);
    ESP_ERROR_CHECK(xTaskCreate(btn_release_task, "btn_release", 2048, NULL, 5, &btn_release_task_handle) != pdPASS);
    ESP_ERROR_CHECK(xTaskCreate(btn_task, "btn_task", 2048, NULL, 5, &btn_task_handle) != pdPASS);
    #ifdef FEATURE_MEASURE_BATTERY_LEVEL
    ESP_ERROR_CHECK(xTaskCreate(adc_task, "adc", 4096, NULL, 10, &adc_task_handle) != pdPASS);
    #endif
    #ifdef FEATURE_MEASURE_FLOW_RATE
    ESP_ERROR_CHECK((reset_instantaneous_demand_timer = xTimerCreate("reset_inst_dema", pdMS_TO_TICKS(TIME_TO_RESET_INSTANTANEOUS_D), pdFALSE, "r_i_d", reset_instantaneous_demand_cb)) == NULL ? ESP_FAIL : ESP_OK);
    #endif

    if (reset_error != ESP_OK) {
        xEventGroupSetBits(report_event_group_handle, EXTENDED_STATUS_REPORT);
    }

    ESP_LOGI(TAG, "Configuring led");
    ESP_ERROR_CHECK(config_led());
    ESP_LOGI(TAG, "Led ON");
    ESP_LOGI(TAG, "Configuring GPIO Interrupt...");
    ESP_ERROR_CHECK(gm_gpio_interrup_init());
    ESP_LOGI(TAG, "Configuring NVS Flash");
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_LOGI(TAG, "Configuring power save");
    ESP_ERROR_CHECK(esp_zb_power_save_init());
    ESP_LOGI(TAG, "Load counter from NVS");
    ESP_ERROR_CHECK(gm_counter_load_nvs());
    ESP_LOGI(TAG, "Setup deep sleep");
    ESP_ERROR_CHECK(gm_deep_sleep_init());
    // led_on();
    xEventGroupSetBits(report_event_group_handle, CURRENT_SUMMATION_DELIVERED_REPORT);

    // start main loop
    ESP_LOGI(TAG, "Starting main loop");
    ESP_ERROR_CHECK(xTaskCreate(gm_main_loop_task, "gas_meter_main", 8192, NULL, tskIDLE_PRIORITY, NULL) != pdPASS);
}