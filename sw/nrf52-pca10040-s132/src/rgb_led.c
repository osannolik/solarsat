#include <stdint.h>

#include "app_timer.h"
#include "rgb_led.h"

APP_TIMER_DEF(lpp_timer_0);
APP_TIMER_DEF(lpp_timer_1);
APP_TIMER_DEF(lpp_timer_2);

static uint32_t bit_mask(const uint32_t pin) {
  return 1u << pin;
}

int rgb_led_init(rgb_led_t *rgb_led,
                 const uint32_t r_pin,
                 const uint32_t g_pin,
                 const uint32_t b_pin) {
  uint32_t err_code;

  low_power_pwm_config_t low_power_pwm_config = {.active_high = false,
                                                 .period = 255,
                                                 .bit_mask = bit_mask(r_pin),
                                                 .p_timer_id = &lpp_timer_0,
                                                 .p_port = NRF_GPIO};

  err_code = low_power_pwm_init(&rgb_led->low_power_pwm_0,
                                &low_power_pwm_config,
                                NULL);
  APP_ERROR_CHECK(err_code);
  err_code = low_power_pwm_duty_set(&rgb_led->low_power_pwm_0, 0);
  APP_ERROR_CHECK(err_code);

  low_power_pwm_config.bit_mask   = bit_mask(g_pin);
  low_power_pwm_config.p_timer_id = &lpp_timer_1;

  err_code = low_power_pwm_init(&rgb_led->low_power_pwm_1,
                                &low_power_pwm_config,
                                NULL);
  APP_ERROR_CHECK(err_code);
  err_code = low_power_pwm_duty_set(&rgb_led->low_power_pwm_1, 0);
  APP_ERROR_CHECK(err_code);

  low_power_pwm_config.bit_mask   = bit_mask(b_pin);
  low_power_pwm_config.p_timer_id = &lpp_timer_2;

  err_code = low_power_pwm_init(&rgb_led->low_power_pwm_2,
                                &low_power_pwm_config,
                                NULL);
  APP_ERROR_CHECK(err_code);
  err_code = low_power_pwm_duty_set(&rgb_led->low_power_pwm_2, 0);
  APP_ERROR_CHECK(err_code);

  return 0;
}

static void lpp_set_duty(low_power_pwm_t *lpp, const uint8_t duty) {
  uint32_t err_code;
  const uint8_t current_duty = lpp->duty_cycle;

  err_code = low_power_pwm_duty_set(lpp, duty);
  APP_ERROR_CHECK(err_code);

  if (current_duty == 0 && duty > 0) {
    err_code = low_power_pwm_start(lpp, lpp->bit_mask);
    APP_ERROR_CHECK(err_code);
  } else if (duty == 0 && current_duty > 0) {
    err_code = low_power_pwm_stop(lpp);
    APP_ERROR_CHECK(err_code);
  }
}

void rgb_led_set_duty_red(rgb_led_t *rgb_led, const uint8_t duty) {
  lpp_set_duty(&rgb_led->low_power_pwm_0, duty);
}

void rgb_led_set_duty_green(rgb_led_t *rgb_led, const uint8_t duty) {
  lpp_set_duty(&rgb_led->low_power_pwm_1, duty);
}

void rgb_led_set_duty_blue(rgb_led_t *rgb_led, const uint8_t duty) {
  lpp_set_duty(&rgb_led->low_power_pwm_2, duty);
}

void rgb_led_set_duty(rgb_led_t *rgb_led,
                     const uint8_t r_duty,
                     const uint8_t g_duty,
                     const uint8_t b_duty) {
  rgb_led_set_duty_red(rgb_led, r_duty);
  rgb_led_set_duty_red(rgb_led, g_duty);
  rgb_led_set_duty_red(rgb_led, b_duty);
}
