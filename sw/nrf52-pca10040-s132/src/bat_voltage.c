#include "stdint.h"

#include "bat_voltage.h"
#include "nrf_drv_rtc.h"

#define SAADC_SAMPLE_INTERVAL_MS (1000)
#define RTC_FREQUENCY (32)
#define RTC_CHANNEL (0)

static nrf_saadc_value_t saadc_buffer[2];

static const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(2);
static const uint32_t rtc_ticks = RTC_US_TO_TICKS(SAADC_SAMPLE_INTERVAL_MS * 1000, RTC_FREQUENCY);

static void rtc_handler(nrf_drv_rtc_int_type_t int_type) {
  uint32_t err_code;

  if (int_type == NRF_DRV_RTC_INT_COMPARE0) {
    nrf_drv_saadc_sample();

    err_code = nrf_drv_rtc_cc_set(&rtc, RTC_CHANNEL, rtc_ticks, true);
    APP_ERROR_CHECK(err_code);
    nrf_drv_rtc_counter_clear(&rtc);
  }
}

static void rtc_config(void) {
  uint32_t err_code;

  nrf_drv_rtc_config_t rtc_config;
  rtc_config.prescaler = RTC_FREQ_TO_PRESCALER(RTC_FREQUENCY);
  err_code = nrf_drv_rtc_init(&rtc, &rtc_config, rtc_handler);
  APP_ERROR_CHECK(err_code);

  err_code = nrf_drv_rtc_cc_set(&rtc, RTC_CHANNEL, rtc_ticks, true);
  APP_ERROR_CHECK(err_code);

  nrf_drv_rtc_enable(&rtc);
}

void saadc_callback(nrf_drv_saadc_evt_t const *p_event) {
  ret_code_t err_code;

  if (p_event->type == NRF_DRV_SAADC_EVT_DONE) {
    err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, 1);
    APP_ERROR_CHECK(err_code);
  } else if (p_event->type == NRF_DRV_SAADC_EVT_CALIBRATEDONE) {
    err_code = nrf_drv_saadc_buffer_convert(&saadc_buffer[0], 1);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_saadc_buffer_convert(&saadc_buffer[1], 1);
    APP_ERROR_CHECK(err_code);
  }
}

void bat_voltage_init(bat_voltage_t *bat, const uint32_t adc_pin) {
  ret_code_t err_code;

  rtc_config();

  nrf_drv_saadc_config_t saadc_config;
  saadc_config.low_power_mode = true;
  saadc_config.resolution = NRF_SAADC_RESOLUTION_12BIT;
  saadc_config.oversample = NRF_SAADC_OVERSAMPLE_4X;
  saadc_config.interrupt_priority = APP_IRQ_PRIORITY_LOW;

  err_code = nrf_drv_saadc_init(&saadc_config, saadc_callback);
  APP_ERROR_CHECK(err_code);

  nrf_saadc_channel_config_t channel_config;
  /*
   * Set internal reference to 0.6 V and input gain to 1/6.
   * The maximum SAADC input voltage is then 0.6V/(1/6)=3.6V.
   */
  channel_config.reference = NRF_SAADC_REFERENCE_INTERNAL;
  channel_config.gain = NRF_SAADC_GAIN1_6;
  channel_config.acq_time = NRF_SAADC_ACQTIME_40US;
  channel_config.mode = NRF_SAADC_MODE_SINGLE_ENDED;
  /*
   * Burst mode: When triggering the SAMPLE task in burst mode, the SAADC will
   * sample "Oversample" number of times as fast as it can and then output a
   * single averaged value to the RAM buffer.
   */
  channel_config.burst = NRF_SAADC_BURST_ENABLED;
  channel_config.pin_p = adc_pin;
  channel_config.pin_n = NRF_SAADC_INPUT_DISABLED;
  channel_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;
  channel_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;

  err_code = nrf_drv_saadc_channel_init(RTC_CHANNEL, &channel_config);
  APP_ERROR_CHECK(err_code);

  err_code = nrf_drv_saadc_buffer_convert(&saadc_buffer[0], 1);
  APP_ERROR_CHECK(err_code);

  /* The SAADC will write to this buffer when buffer 1 is full */
  err_code = nrf_drv_saadc_buffer_convert(&saadc_buffer[1], 1);
  APP_ERROR_CHECK(err_code);
}

const float divider_r6_mega = 10.0f;
const float divider_r7_mega = 4.0f;
const float divider_factor = divider_r6_mega / (divider_r6_mega + divider_r7_mega);
const float full_range_volt = 3.6f;
const uint32_t full_range = (1u << 12) - 1u;
const float adc_scaling = full_range_volt / ((float) full_range) / divider_factor;

float bat_voltage_value(const bat_voltage_t *bat) {
  (void) bat;
  return adc_scaling * (float) saadc_buffer[0];
}