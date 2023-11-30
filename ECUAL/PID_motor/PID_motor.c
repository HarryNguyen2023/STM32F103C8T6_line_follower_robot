#include "PID_motor.h"
#include "PID_motor_cfg.h"
#include <stdlib.h>

// Define some PID variables
uint16_t time_frame = 20;    // Time frame to milisecond

// ----------------------------------------------------- Static functions hidden from users ----------------------------------------------
// Function prototypes
static void outputSpeedPID(PID_motor* motor);
static void updatePosition(PID_motor* motor);
static void resetEncoder(PID_motor* motor);
static uint8_t motionProfileTracking(PID_motor* motor);
static uint32_t readEncoder(PID_motor* motor);

// Function to get the output value of the PID speed controller
static void outputSpeedPID(PID_motor* motor)
{
    float error, output, prop;
    // Get number of the encoder pulse in the last time frame
    if(motor->direction == 0 && (motor->current_encoder < motor->prev_encoder) && (motor->prev_encoder - motor->current_encoder > 16000))
    {
        motor->real_speed = (65535 / 4) - motor->prev_encoder;
        motor->real_speed += motor->current_encoder;
    }
    else if(motor->direction == 1 && (motor->current_encoder > motor->prev_encoder) && (motor->current_encoder - motor->prev_encoder > 16000) )
    {
        motor->real_speed = motor->current_encoder - (65535 / 4);
        motor->real_speed -= motor->prev_encoder;
    }
    else
        motor->real_speed = motor->current_encoder - motor->prev_encoder;
    // Get the error of the number of encoder per time frame
    error = motor->targetPulsePerFrame - motor->real_speed;
    // Get the output of the PID controller with the new formula to avoid derivative kick as well as accumulation error when updating PID parameters
    prop = motor->speed_controller.Kp * error;
    motor->integral_error += motor->speed_controller.Ki * error;
    // Anti integral wind-up 
    if(motor->MAX_PWM > prop)
        motor->lim_max_integ = motor->MAX_PWM - prop;
    else 
        motor->lim_max_integ = 0;
    
    if(0 < prop)
        motor->lim_min_integ = 0 - prop;
    else
        motor->lim_min_integ = 0;
    // Constraint the integral
    if(motor->integral_error > motor->lim_max_integ)
        motor->integral_error = motor->lim_max_integ;
    else if(motor->integral_error < motor->lim_min_integ)
        motor->integral_error = motor->lim_min_integ;

    output = prop + motor->speed_controller.Kd * (motor->real_speed - motor->prev_encoder_feedback) + motor->integral_error;
    // Update the parameters
    motor->prev_encoder = motor->current_encoder;
    motor->prev_encoder_feedback = motor->real_speed;
    // Limit the ouput velocity of the motor
    if(output > motor->MAX_PWM)
        output = motor->MAX_PWM;
    else if(output < 0)
        output = 0;
    motor->output = output;
}

// Function to generate the motion progile for the motor
static uint8_t motionProfileTracking(PID_motor* motor)
{
    // Case in the 1st phase of the mottion
    if(motor->motion_profile.phase == 0)
    {
        // Accelerate till the motor reach the maximum velocity
        if(motor->motion_profile.command_velocity < motor->motion_profile.motion_velocity)
        {
            motor->motion_profile.command_velocity += motor->motion_profile.motion_acceleration;
            if(motor->motion_profile.command_velocity > motor->motion_profile.motion_velocity)
            {
                motor->motion_profile.command_velocity = motor->motion_profile.motion_velocity;
            }
        }
                
        // Count the number of frame running at maximum speed
        if(motor->motion_profile.command_velocity == motor->motion_profile.motion_velocity)
        {
            motor->motion_profile.max_speed_count++;
        }
        
        // Decrease the distance to be travelled
        motor->motion_profile.phase_dist -= motor->motion_profile.command_velocity;
        
        // Get the command position for each frame
        if(motor->direction == 0)
            motor->motion_profile.command_position += motor->motion_profile.command_velocity;
        else if(motor->direction == 1)
            motor->motion_profile.command_position -= motor->motion_profile.command_velocity;

        // Check if phase 1 has been completed
        if(motor->motion_profile.phase_dist <= 0)
            motor->motion_profile.phase = 1;
    }
    else
    {
        if(motor->motion_profile.max_speed_count)
            motor->motion_profile.max_speed_count--;
        else if(motor->motion_profile.command_velocity > 0)
        {   
            motor->motion_profile.command_velocity -= motor->motion_profile.motion_acceleration;
            // End of movement
            if(motor->motion_profile.command_velocity < 0)
            {
                motor->moving = 0;           
                return 1;
            }
        }
        // Update the command position of the robot
        if(motor->direction == 0)
            motor->motion_profile.command_position += motor->motion_profile.command_velocity;
        else if(motor->direction == 1)
            motor->motion_profile.command_position -= motor->motion_profile.command_velocity;
    }
    return 0;
}

// Function to get the encoder value of the motor
static uint32_t readEncoder(PID_motor* motor)
{
    return motor->encoder_tim->CNT / 4;
}

// Function to reset the encoder value of the motor
static void resetEncoder(PID_motor* motor)
{
    motor->encoder_tim->CNT = 0;
}

// Function to get the ouput of the position P controller
static void updatePosition(PID_motor* motor)
{
    int32_t feedback;
    // Update the position of the robot
    motor->current_encoder = readEncoder(motor);
    // Get number of the encoder pulse in the last time frame
    if(motor->direction == 0 && motor->current_encoder < motor->prev_encoder)
    {
        feedback = (65535 / 4) - motor->prev_encoder;
        feedback += motor->current_encoder;
    }
    else if(motor->direction == 1 && motor->current_encoder > motor->prev_encoder)
    {
        feedback = motor->current_encoder - (65535 / 4);
        feedback -= motor->prev_encoder;
    }
    else
        feedback = motor->current_encoder - motor->prev_encoder;
    motor->motion_profile.current_position += feedback;
}

// -------------------------------------------------------- General function used by users -----------------------------------------------

// Function to initiate the motor GPIO pins
void motorInit(PID_motor motor)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    // Initiate the GPIO pins of the motor
    for(int i = 0; i < 2; ++i)
    {
        HAL_GPIO_WritePin(motor.motor_ports[i], motor.motor_pins[i], 0);
        if(motor.motor_ports[i] == GPIOA)
            __HAL_RCC_GPIOA_CLK_ENABLE();
        else if (motor.motor_ports[i] == GPIOB)
            __HAL_RCC_GPIOB_CLK_ENABLE();
        else if (motor.motor_ports[i] == GPIOC)
            __HAL_RCC_GPIOC_CLK_ENABLE();
        else if (motor.motor_ports[i] == GPIOD)
            __HAL_RCC_GPIOD_CLK_ENABLE();
        else if (motor.motor_ports[i] == GPIOE)
            __HAL_RCC_GPIOE_CLK_ENABLE();
        GPIO_InitStruct.Pin = motor.motor_pins[i];
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
        HAL_GPIO_Init(motor.motor_ports[i], &GPIO_InitStruct);
    }
}

// Function to update the PWM duty cycle of the motor
void dutyCycleUpdate(uint16_t duty_cycle, PID_motor* motor)
{
    switch (motor->pwm_channel)
    {
        case PWM_CHANNEL_1:
            motor->pwm_tim->CCR1 = duty_cycle;
            break;
        case PWM_CHANNEL_2:
            motor->pwm_tim->CCR2 = duty_cycle;
            break;
        case PWM_CHANNEL_3:
            motor->pwm_tim->CCR3 = duty_cycle;
            break;
        case PWM_CHANNEL_4:
            motor->pwm_tim->CCR4 = duty_cycle;
            break;
    default:
        break;
    }
}

// Function to brake the motor immediately
void motorBrake(PID_motor* motor)
{
    HAL_GPIO_WritePin(motor->motor_ports[0], motor->motor_pins[0], 0);
    HAL_GPIO_WritePin(motor->motor_ports[1], motor->motor_pins[1], 0);
}

// Function to reset the PID value of the motor when it is not moving
void resetPID(PID_motor* motor)
{
    // Re-initiate the PID parametrs
    resetEncoder(motor);
    motor->current_encoder = readEncoder(motor);
    motor->prev_encoder = motor->current_encoder;
    motor->integral_error = 0;
    motor->output = 0;
    motor->prev_encoder_feedback = 0;
    motor->targetPulsePerFrame = 0.0;
    motor->moving = 0;
    motor->direction = 0;
    motor->motion_profile.command_position = 0;
    motor->motion_profile.command_velocity = 0;
    motor->motion_profile.current_position = 0;
    motor->motion_profile.max_speed_count = 0;
    motor->motion_profile.motion_acceleration = 0;
    motor->motion_profile.motion_velocity = 0;
    motor->motion_profile.target_position = 0;
    motor->motion_profile.phase_dist = 0;
    motor->motion_profile.phase = 1;
}

// Function to handle the speed input of the PID controller
void inputSpeedHandling(PID_motor* motor, float speed)
{
    // Rescale the input rpm speed
    if(speed > motor->MAX_INPUT_SPEED)
        speed = motor->MAX_INPUT_SPEED;
    else if(speed < 0)
        speed = 0;

    if(speed >= 0)
        motor->direction = 0;
    else
        motor->direction = 1;
    // Check whether the motor is already moving
    if(! motor->moving)
    {
        motor->moving = 1;
    }
    // Convert the desired speed to pulse per frame and input to the motor
    motor->targetPulsePerFrame = (speed * motor->encoder_rev) * time_frame / 60000.0;
    return;
}

// Function to control the speed of the motor by PID algorithm
void speedControlPID(PID_motor* motor)
{
    // Update the motor encoder value
    motor->current_encoder = readEncoder(motor);
    // Update the PID output of the controller
    outputSpeedPID(motor);

    // Get the absolute value of the motor
    uint16_t pwm_dutycycle = abs(motor->output);
    if(pwm_dutycycle < motor->DEAD_BAND)
        pwm_dutycycle = 0;

    // Control the direction of the motor
    if(motor->direction == 0)
    {
        HAL_GPIO_WritePin(motor->motor_ports[0], motor->motor_pins[0], 1);
        HAL_GPIO_WritePin(motor->motor_ports[1], motor->motor_pins[1], 0);
    }
    else if(motor->direction == 1)
    {
        HAL_GPIO_WritePin(motor->motor_ports[0], motor->motor_pins[0], 0);
        HAL_GPIO_WritePin(motor->motor_ports[1], motor->motor_pins[1], 1);
    }

    // Feed the value of the PWM duty cycle
    dutyCycleUpdate(pwm_dutycycle, motor);

    if(pwm_dutycycle == 0)
        motorBrake(motor);
}

// Function to handle the input command of the position control 
void inputPositionHandling(PID_motor* motor, float position_angle, uint16_t motion_velocity, uint16_t motion_acel)
{
    // Brake the motor 
    motorBrake(motor);
    // Reset all the PID parameters
    resetPID(motor);
    if(motion_velocity < 0 || motion_acel < 0)
        return;
    // Convert the target position to number of pulse
    motor->motion_profile.target_position = abs(position_angle * motor->encoder_rev) / 360.0;
    if(position_angle >= - 0.0)
        motor->direction = 0;
    else
        motor->direction = 1;
    motor->motion_profile.phase_dist = motor->motion_profile.target_position / 2;
    motor->motion_profile.current_position = 0;
    // Get the motion maximum velocity and convert it into pulse per frame
    if(motion_velocity > motor->MAX_INPUT_SPEED)
        motion_velocity = motor->MAX_INPUT_SPEED;
    motor->motion_profile.motion_velocity = (motion_velocity * motor->encoder_rev * 1.0) * time_frame / 60000.0;
    // Get the motion acceleration and convert it into pulse per frame^2
    motor->motion_profile.motion_acceleration = (motion_acel * motor->encoder_rev * 1.0) * time_frame / 60000.0;;
    // Set the phase of the 
    motor->motion_profile.phase = 0;
    // Activate the motion of the motor
    motor->moving = 1;
}

// Function to control the position of the motor by P controller
uint8_t positionControlPID(PID_motor* motor)
{
     // Update the position of the motor
    updatePosition(motor);
    // Create motion profile of the robot
    uint8_t end = motionProfileTracking(motor);
    // Update the P controller parameters
    float p_output = (motor->motion_profile.command_position - motor->motion_profile.current_position) * motor->motion_profile.pos_Kp;

    // Limit the input velocity
    if(p_output > motor->motion_profile.motion_velocity)
        p_output = motor->motion_profile.motion_velocity;
    else if(p_output < - motor->motion_profile.motion_velocity)
        p_output = - motor->motion_profile.motion_velocity;

    if((motor->direction == 0 && p_output <= 0) || (motor->direction == 1 && p_output >= 0) || end)
    {
        motor->real_speed = 0;
        dutyCycleUpdate(0, motor);
        motorBrake(motor);
        // Reaset the PID value
        resetPID(motor);
        return 1;
    }
    // Feed the velocity to the motor
    motor->targetPulsePerFrame = p_output;
    // Get the direction of the motion
    if(p_output >= 0)
        motor->direction = 0;
    else
        motor->direction = 1;
    // Feed the velocity to the speed controller
    speedControlPID(motor);
    return 0;
}
