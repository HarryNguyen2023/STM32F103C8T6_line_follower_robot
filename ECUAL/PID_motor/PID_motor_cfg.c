#include "PID_motor.h"

// Parameters for motor 1
PID_motor motor_left = 
{
    {GPIOB, GPIOB},
    {GPIO_PIN_4, GPIO_PIN_5},
    TIM2,
    TIM4,
    PWM_CHANNEL_1,
    374,
    499,
    250,
    15,
    {
        34.0,
        1.4,
        0.0,
    },
    {
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0.08,
    },
    0,
    0.0,
    0,
    0,
    0,
    0,
    0.0,
    0.0,
    0.0,
    0,
    0
};

// Parameters for motor 2
PID_motor motor_right = 
{
    {GPIOB, GPIOA},
    {GPIO_PIN_3, GPIO_PIN_15},
    TIM3,
    TIM4,
    PWM_CHANNEL_2,
    374,
    499,
    250,
    15,
    {
        33.0, 
        1.4,
        0.0,
    },
    {
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0.08,
    },
    0,
    0.0,
    0,
    0,
    0,
    0,
    0.0,
    0.0,
    0.0,
    0,
    0
};