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
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_zb_gas_led.h"

const char *TAG = "GAS_LED.C";

TaskHandle_t led_task_handle = NULL;

typedef enum LedState_e {
	ON,
	OFF
} led_state_t;

led_state_t led_state;

void led_task(void *arg)
{

	ESP_LOGI(TAG, "Led task started");
	while (true) {
		uint32_t state;
		xTaskNotifyWait(0x00, 0xFF, &state, portMAX_DELAY);
		switch (state) {
			case ON:
				gpio_set_level(LED_PIN, 0);
				break;
			case OFF:
				gpio_set_level(LED_PIN, 1);
				break;
		}
	}
}

/**
 * @brief Turn led on
 * 
 */
void led_on()
{
	led_state = ON;
	xTaskNotify(led_task_handle, led_state, eSetValueWithOverwrite);
}

/**
 * @brief Turn led off
 * 
 */
void led_off()
{
	led_state = OFF;
	xTaskNotify(led_task_handle, led_state, eSetValueWithOverwrite);
}

/**
 * @brief true if the led is on
 * 
 * @return true 
 * @return false 
 */
bool led_is_on() {
	return led_state == ON;
}

/**
 * @brief Configure the led pin
 * 
 */
esp_err_t config_led()
{
	uint64_t led_switch_pin = 1ULL << LED_PIN;
	gpio_config_t led_conf = {
			.intr_type = GPIO_INTR_DISABLE,
			.mode = GPIO_MODE_OUTPUT,
			.pin_bit_mask = led_switch_pin,
			.pull_down_en = GPIO_PULLDOWN_DISABLE,
			.pull_up_en = GPIO_PULLUP_ENABLE
	};
	ESP_RETURN_ON_ERROR(gpio_config(&led_conf), TAG, "Failed to configure LED pin");

	ESP_RETURN_ON_ERROR(gpio_set_level(LED_PIN, 0), TAG, "Failed to turn off led");

	ESP_RETURN_ON_ERROR(xTaskCreate(led_task, "adc", 2048, NULL, 5, &led_task_handle) != pdPASS, TAG, "Failed to turn off led");

	return ESP_OK;
}

