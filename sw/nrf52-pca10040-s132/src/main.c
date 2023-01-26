/**
 * Copyright (c) 2015 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 * @defgroup tw_sensor_example main.c
 * @{
 * @ingroup nrf_twi_example
 * @brief TWI Sensor Example main file.
 *
 * This file contains the source code for a sample application using TWI.
 *
 */

#include <stdio.h>
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_delay.h"

#include "rgb_led.h"
#include "sensor.h"
#include "bat_voltage.h"
#include "ble_device.h"

#include "nrf_drv_clock.h"
#include "app_timer.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_pwr_mgmt.h"

APP_TIMER_DEF(measurement_timer);

static void lfclk_init(void)
{
    uint32_t err_code;
    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);
}

#define BOARD_SDA_PIN 11
#define BOARD_SCL_PIN 12
static sensor_t sensor;

#define BOARD_LED_R_PIN 17
#define BOARD_LED_G_PIN 16
#define BOARD_LED_B_PIN 15
static rgb_led_t rgb_led;

#define BOARD_BAT_ADC_PIN NRF_SAADC_INPUT_AIN0
static bat_voltage_t bat_voltage;

volatile static bool djent = false;

static uint8_t meas_task = 0;

volatile static float bat_volt = 0.0f;

void periodic_measurements(void * p_context) {
  (void) p_context;
  djent = !djent;

  int8_t err_code;
  sensor_measurement_t measurement;

  switch (meas_task) {
    case 0:
      rgb_led_set_duty_red(&rgb_led, 1);

      const uint32_t delay_ms = start_measurements(&sensor) / 1000;

      meas_task = 1;
      err_code = app_timer_start(measurement_timer, APP_TIMER_TICKS(delay_ms), NULL);
      APP_ERROR_CHECK(err_code);
      break;
    case 1:
      measurement = collect_measurements(&sensor);

      bat_volt = bat_voltage_value(&bat_voltage);

      rgb_led_set_duty_red(&rgb_led, 0);

      if (measurement.temperature > 30.0f) {
        rgb_led_set_duty_green(&rgb_led, (uint8_t) (measurement.temperature - 30.0f));
      } else {
        rgb_led_set_duty_green(&rgb_led, 0);
      }

      meas_task = 0;
      err_code = app_timer_start(measurement_timer, APP_TIMER_TICKS(2*1000), NULL);
      APP_ERROR_CHECK(err_code);
      break;
    default:
      break;
  }
}

int main(void)
{
    NRF_POWER->DCDCEN = 1;  //Enabling the DCDC converter for lower current consumption

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("\r\nTWI sensor example started.");
    NRF_LOG_FLUSH();

    int8_t err_code;

    lfclk_init();

    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&measurement_timer, APP_TIMER_MODE_SINGLE_SHOT, periodic_measurements);
    APP_ERROR_CHECK(err_code);

    //pwm_init();
    rgb_led_init(&rgb_led, BOARD_LED_R_PIN, BOARD_LED_G_PIN, BOARD_LED_B_PIN);

    //twi_init();
    sensor_init(&sensor, BOARD_SCL_PIN, BOARD_SDA_PIN);
    nrf_delay_ms(5);
    sensor_config(&sensor);

    //rtc_config();
    //saadc_init();
    bat_voltage_init(&bat_voltage, BOARD_BAT_ADC_PIN);

    ble_device_init();

    nrf_delay_ms(5);

    err_code = app_timer_start(measurement_timer, APP_TIMER_TICKS(2*1000), NULL);
    APP_ERROR_CHECK(err_code);

    while (true) {
        nrf_pwr_mgmt_run();
    }
}

/** @} */
