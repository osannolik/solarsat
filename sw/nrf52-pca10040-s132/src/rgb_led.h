#ifndef NRF52_PCA10040_S132_RGB_LED_H
#define NRF52_PCA10040_S132_RGB_LED_H

#include "low_power_pwm.h"

typedef struct {
    low_power_pwm_t low_power_pwm_0;
    low_power_pwm_t low_power_pwm_1;
    low_power_pwm_t low_power_pwm_2;
} rgb_led_t;

int rgb_led_init(rgb_led_t *rgb_led,
                 const uint32_t r_pin,
                 const uint32_t g_pin,
                 const uint32_t b_pin);

void rgb_led_set_duty_red(rgb_led_t *rgb_led, const uint8_t duty);
void rgb_led_set_duty_green(rgb_led_t *rgb_led, const uint8_t duty);
void rgb_led_set_duty_blue(rgb_led_t *rgb_led, const uint8_t duty);

void rgb_led_set_duty(rgb_led_t *rgb_led,
                      const uint8_t r_duty,
                      const uint8_t g_duty,
                      const uint8_t b_duty);

#endif //NRF52_PCA10040_S132_RGB_LED_H
