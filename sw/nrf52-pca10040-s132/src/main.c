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
#if 0
#define SAADC_SAMPLE_INTERVAL_MS 1000              //Interval in milliseconds at which RTC times out and triggers SAADC sample task
#define RTC_FREQUENCY 32                          //Determines the RTC frequency and prescaler

const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(2); /**< Declaring an instance of nrf_drv_rtc for RTC2. */
static uint32_t rtc_ticks = RTC_US_TO_TICKS(SAADC_SAMPLE_INTERVAL_MS*1000, RTC_FREQUENCY);

static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
    uint32_t err_code;

    if (int_type == NRF_DRV_RTC_INT_COMPARE0)
    {
        nrf_drv_saadc_sample();                                        //Trigger the SAADC SAMPLE task

        err_code = nrf_drv_rtc_cc_set(&rtc, 0, rtc_ticks, true);       //Set RTC compare value. This needs to be done every time as the nrf_drv_rtc clears the compare register on every compare match
        APP_ERROR_CHECK(err_code);
        nrf_drv_rtc_counter_clear(&rtc);                               //Clear the RTC counter to start count from zero
    }
}

static void rtc_config(void)
{

    uint32_t err_code;

    //Initialize RTC instance
    nrf_drv_rtc_config_t rtc_config;
    rtc_config.prescaler = RTC_FREQ_TO_PRESCALER(RTC_FREQUENCY);
    err_code = nrf_drv_rtc_init(&rtc, &rtc_config, rtc_handler);                //Initialize the RTC with callback function rtc_handler. The rtc_handler must be implemented in this applicaiton. Passing NULL here for RTC configuration means that configuration will be taken from the sdk_config.h file.
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_rtc_cc_set(&rtc, 0, rtc_ticks, true);                    //Set RTC compare value to trigger interrupt. Configure the interrupt frequency by adjust RTC_CC_VALUE and RTC_FREQUENCY constant in top of main.c
    APP_ERROR_CHECK(err_code);

    //Power on RTC instance
    nrf_drv_rtc_enable(&rtc);                                                   //Enable RTC
}


#define SAADC_CALIBRATION_INTERVAL 5              //Determines how often the SAADC should be calibrated relative to NRF_DRV_SAADC_EVT_DONE event. E.g. value 5 will make the SAADC calibrate every fifth time the NRF_DRV_SAADC_EVT_DONE is received.
#define SAADC_SAMPLES_IN_BUFFER 1                 //Number of SAADC samples in RAM before returning a SAADC event. For low power SAADC set this constant to 1. Otherwise the EasyDMA will be enabled for an extended time which consumes high current.

// 3077/4096*3.6 * 14/10 = 3.79
static nrf_saadc_value_t saadc_buffer[2][SAADC_SAMPLES_IN_BUFFER];
static uint32_t m_adc_evt_counter = 0;
static bool m_saadc_calibrate = false;


void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    ret_code_t err_code;
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)                                //Capture offset calibration complete event
    {
        if((m_adc_evt_counter % SAADC_CALIBRATION_INTERVAL) == 0)               //Evaluate if offset calibration should be performed. Configure the SAADC_CALIBRATION_INTERVAL constant to change the calibration frequency
        {
            m_saadc_calibrate = true;                                           // Set flag to trigger calibration in main context when SAADC is stopped
        }

        NRF_LOG_INFO("ADC event number: %d\r\n",(int)m_adc_evt_counter);        //Print the event number on UART

        for (int i = 0; i < p_event->data.done.size; i++)
        {
            NRF_LOG_INFO("%d\r\n", p_event->data.done.p_buffer[i]);             //Print the SAADC result on UART
        }

        if(m_saadc_calibrate == false)
        {
            err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAADC_SAMPLES_IN_BUFFER);             //Set buffer so the SAADC can write to it again.
            APP_ERROR_CHECK(err_code);
        }

        m_adc_evt_counter++;

    }
    else if (p_event->type == NRF_DRV_SAADC_EVT_CALIBRATEDONE)
    {
        err_code = nrf_drv_saadc_buffer_convert(saadc_buffer[0], SAADC_SAMPLES_IN_BUFFER);             //Set buffer so the SAADC can write to it again.
        APP_ERROR_CHECK(err_code);
        err_code = nrf_drv_saadc_buffer_convert(saadc_buffer[1], SAADC_SAMPLES_IN_BUFFER);             //Need to setup both buffers, as they were both removed with the call to nrf_drv_saadc_abort before calibration.
        APP_ERROR_CHECK(err_code);

        NRF_LOG_INFO("SAADC calibration complete ! \r\n");                                              //Print on UART
    }
}

#define SAADC_OVERSAMPLE NRF_SAADC_OVERSAMPLE_4X  //Oversampling setting for the SAADC. Setting oversample to 4x This will make the SAADC output a single averaged value when the SAMPLE task is triggered 4 times. Enable BURST mode to make the SAADC sample 4 times when triggering SAMPLE task once.
#define SAADC_BURST_MODE 1                        //Set to 1 to enable BURST mode, otherwise set to 0.

void saadc_init(void)
{
    ret_code_t err_code;
    nrf_drv_saadc_config_t saadc_config;
    nrf_saadc_channel_config_t channel_config;


    //Configure SAADC
    saadc_config.low_power_mode = true;                                                   //Enable low power mode.
    saadc_config.resolution = NRF_SAADC_RESOLUTION_12BIT;                                 //Set SAADC resolution to 12-bit. This will make the SAADC output values from 0 (when input voltage is 0V) to 2^12=4096 (when input voltage is 3.6V for channel gain setting of 1/6).
    saadc_config.oversample = SAADC_OVERSAMPLE;                                           //Set oversample to 4x. This will make the SAADC output a single averaged value when the SAMPLE task is triggered 4 times.
    saadc_config.interrupt_priority = APP_IRQ_PRIORITY_LOW;                               //Set SAADC interrupt to low priority.

    //Initialize SAADC
    err_code = nrf_drv_saadc_init(&saadc_config, saadc_callback);                         //Initialize the SAADC with configuration and callback function. The application must then implement the saadc_callback function, which will be called when SAADC interrupt is triggered
    APP_ERROR_CHECK(err_code);

    //Configure SAADC channel
    channel_config.reference = NRF_SAADC_REFERENCE_INTERNAL;                              //Set internal reference of fixed 0.6 volts
    channel_config.gain = NRF_SAADC_GAIN1_6;                                              //Set input gain to 1/6. The maximum SAADC input voltage is then 0.6V/(1/6)=3.6V. The single ended input range is then 0V-3.6V
    channel_config.acq_time = NRF_SAADC_ACQTIME_40US;//NRF_SAADC_ACQTIME_10US;                                     //Set acquisition time. Set low acquisition time to enable maximum sampling frequency of 200kHz. Set high acquisition time to allow maximum source resistance up to 800 kohm, see the SAADC electrical specification in the PS.
    channel_config.mode = NRF_SAADC_MODE_SINGLE_ENDED;                                    //Set SAADC as single ended. This means it will only have the positive pin as input, and the negative pin is shorted to ground (0V) internally.
    if(SAADC_BURST_MODE)
    {
        channel_config.burst = NRF_SAADC_BURST_ENABLED;                                   //Configure burst mode for channel 0. Burst is useful together with oversampling. When triggering the SAMPLE task in burst mode, the SAADC will sample "Oversample" number of times as fast as it can and then output a single averaged value to the RAM buffer. If burst mode is not enabled, the SAMPLE task needs to be triggered "Oversample" number of times to output a single averaged value to the RAM buffer.
    }
    channel_config.pin_p = NRF_SAADC_INPUT_AIN0;                                          //Select the input pin for the channel. AIN0 pin maps to physical pin P0.02.
    channel_config.pin_n = NRF_SAADC_INPUT_DISABLED;                                      //Since the SAADC is single ended, the negative pin is disabled. The negative pin is shorted to ground internally.
    channel_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pullup resistor on the input pin
    channel_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pulldown resistor on the input pin


    //Initialize SAADC channel
    err_code = nrf_drv_saadc_channel_init(0, &channel_config);                            //Initialize SAADC channel 0 with the channel configuration
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(saadc_buffer[0],SAADC_SAMPLES_IN_BUFFER);    //Set SAADC buffer 1. The SAADC will start to write to this buffer
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(saadc_buffer[1],SAADC_SAMPLES_IN_BUFFER);    //Set SAADC buffer 2. The SAADC will write to this buffer when buffer 1 is full. This will give the applicaiton time to process data in buffer 1.
    APP_ERROR_CHECK(err_code);

}
#endif

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

    nrf_delay_ms(5);

    err_code = app_timer_start(measurement_timer, APP_TIMER_TICKS(2*1000), NULL);
    APP_ERROR_CHECK(err_code);

    while (true) {
        nrf_pwr_mgmt_run();
    }
#if 0
    while (true) {

        if(m_saadc_calibrate == true)
        {
            nrf_drv_saadc_abort();  // Abort all ongoing conversions. Calibration cannot be run if SAADC is busy

            while(nrf_drv_saadc_calibrate_offset() != NRF_SUCCESS); //Trigger calibration task
            m_saadc_calibrate = false;
        }

        nrf_delay_ms(500);
    }
#endif
}

/** @} */
