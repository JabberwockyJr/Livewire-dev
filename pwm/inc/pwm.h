/*
 * pwm.h
 *
 *  Created on: Mar 24, 2023
 *      Author: Admin
 */

#include "sl_pwm.h"

#ifndef PWM_H_
#define PWM_H_

void pwm_init(void);

extern sl_pwm_instance_t sl_pwm_control;
extern uint8_t duty_cycle;

#endif /* PWM_H_ */
