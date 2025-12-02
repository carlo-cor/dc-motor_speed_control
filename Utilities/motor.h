#ifndef PWM_MOTOR_H
#define PWM_MOTOR_H

#include "TM4C123GH6PM.h"
#include "tm4c123gh6pm_def.h"
#include <stdio.h>
#include <stdint.h>

void PWM_Config(uint16_t period, uint16_t high);
void setMotorDirectionFwd(void);
void setMotorDirectionBckwrd(void);
void setMotorSpeed(uint16_t duty);

#endif

