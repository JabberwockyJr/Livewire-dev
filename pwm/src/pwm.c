/*
 * pwm.c
 *
 *  Created on: Mar 24, 2023
 *      Author: Admin
 */

#include "sl_pwm.h"
#include "sl_pwm_init_pwm1_config.h"
#include "em_gpio.h"


sl_pwm_instance_t sl_pwm_control = {
  .timer    = SL_PWM_PWM1_PERIPHERAL,
  .channel  = SL_PWM_PWM1_OUTPUT_CHANNEL,
  .port     = SL_PWM_PWM1_OUTPUT_PORT,
  .pin      = SL_PWM_PWM1_OUTPUT_PIN,
  .location = SL_PWM_PWM1_OUTPUT_LOC,
};

sl_pwm_config_t pwm_control_config = {
  .frequency = SL_PWM_PWM1_FREQUENCY, //set to 1kHz
  .polarity  = SL_PWM_PWM1_POLARITY,
};


void pwm_init(void)
{

  // Initialize PWM
  sl_pwm_init(&sl_pwm_control, &pwm_control_config);

  // Set duty cycle to 50%
  //sl_pwm_set_duty_cycle(&sl_pwm_control, SL_PWM_DUTY_CYCLE);

  // Enable PWM output
  sl_pwm_start(&sl_pwm_control);
}
