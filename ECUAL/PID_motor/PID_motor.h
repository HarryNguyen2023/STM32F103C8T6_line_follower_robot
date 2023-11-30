#ifndef PID_MOTOR_H
#define PID_MOTOR_H

#include "stm32f1xx_hal.h"

extern uint16_t time_frame;

typedef enum
{
    PWM_CHANNEL_1,
    PWM_CHANNEL_2,
    PWM_CHANNEL_3,
    PWM_CHANNEL_4
}PWM_CHANNEL;

// Structure for motion profile in position control
typedef struct
{
    float target_position;
    int32_t current_position;
    float phase_dist;
    uint32_t max_speed_count;
    float motion_velocity;
    float motion_acceleration;
    float command_position;
    float command_velocity;
    uint8_t phase;
    float pos_Kp;
}Motion_profile;

// Strcuture for PID speed control
typedef struct 
{
    float Kp;
    float Ki;
    float Kd;
}Speed_controller;

// Structure to contains the control information of the motors
typedef struct
{
    // Pin for control the motor directions
    GPIO_TypeDef* motor_ports[2];
    uint16_t motor_pins[2];

    // Timer module for controlling the encoder
    TIM_TypeDef* encoder_tim;
    TIM_TypeDef* pwm_tim;
    PWM_CHANNEL pwm_channel;
    
    // Encoder resolution of the motor
    uint16_t encoder_rev;

    // Motor speed specification
    uint16_t MAX_PWM;
    uint16_t MAX_INPUT_SPEED;
    uint8_t DEAD_BAND;

    // Speed controller parameter
    Speed_controller speed_controller;

    // Motion profile
    Motion_profile motion_profile;

    // PID controller parameters
    uint8_t direction;
    float targetPulsePerFrame;
    int32_t real_speed;
    uint32_t current_encoder;
    uint32_t prev_encoder;
    int16_t prev_encoder_feedback;
    float integral_error;
    float lim_max_integ;
    float lim_min_integ;
    int32_t output;
    uint8_t moving;
}PID_motor;

// Function prototypes
void motorInit(PID_motor motor);
void speedControlPID(PID_motor* motor);
uint8_t positionControlPID(PID_motor* motor);
void motorBrake(PID_motor* motor);
void resetPID(PID_motor* motor);
void dutyCycleUpdate(uint16_t duty_cycle, PID_motor* motor);
void inputSpeedHandling(PID_motor* motor, float speed);
void inputPositionHandling(PID_motor* motor, float position_angle, uint16_t motion_velocity, uint16_t motion_acel);

#endif