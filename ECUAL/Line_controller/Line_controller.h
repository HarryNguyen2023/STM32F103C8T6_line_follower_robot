#ifndef LINE_CONTROLLER_H
#define LINE_CONTROLLER_H

#include "../../ECUAL/Line_sensor/Line_sensor.h"
#include "../../ECUAL/PID_motor/PID_motor.h"

typedef struct 
{
    // Object of line sensor
    Line_sensor robot_line_sensor;

    float prev_sensor_feedback;
    float e2;
    float line_Kp;
    float line_Kd;
    const uint16_t line_target;
    float linear_velocity;
    float angular_velocity;
    uint8_t wheel_diameter;
    uint8_t axle_length;
}Line_controller;

void lineControllerInit(ADC_HandleTypeDef* adc, PID_motor motor_left, PID_motor motor_right, Line_controller* line_controller, float linear_vec);
void lineControllerPID(ADC_HandleTypeDef* adc, PID_motor* motor_left, PID_motor* motor_right, Line_controller* line_controller);
void linearVelocityUpdate(Line_controller* line_controller, float linear_vec);
void robotStop(PID_motor* motor_left, PID_motor* motor_right);
void robotRotateLeft(PID_motor* motor_left, PID_motor* motor_right, Line_controller* line_controller, uint16_t motion_velocity, uint16_t motion_acel);
void robotRotateRight(PID_motor* motor_left, PID_motor* motor_right, Line_controller* line_controller, uint16_t motion_velocity, uint16_t motion_acel, float offset);
void robotLinear(PID_motor* motor_left, PID_motor* motor_right, Line_controller* line_controller, float distance, uint16_t motion_velocity, uint16_t motion_acel);
void linePIDUpdate(Line_controller* line_controller, float Kp, float Kd);

#endif
