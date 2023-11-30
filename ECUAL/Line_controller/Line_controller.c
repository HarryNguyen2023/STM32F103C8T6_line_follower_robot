#include "Line_controller.h"

// Define the value of PI number
#define PI 3.1416

// ----------------------------------------------------- Static functions hidden from users ----------------------------------------------



// -------------------------------------------------------- General function used by users -----------------------------------------------

// Function to initiate the robot line controller
void lineControllerInit(ADC_HandleTypeDef* adc, PID_motor motor_left, PID_motor motor_right, Line_controller* line_controller, float linear_vec)
{
    // Initiate the line sensor module
    lineSensorInit(adc, &line_controller->robot_line_sensor);
    // Initiate the GPIO of the motor left and right
    motorInit(motor_left);
    motorInit(motor_right);
    // Get the linear velocity of the robot in rpm of the wheels
    line_controller->linear_velocity = (linear_vec * 1000.0) / (line_controller->wheel_diameter * PI) * 60;
}

// Function to calculate output signal of the PD line controller 
void lineControllerPID(TIM_HandleTypeDef* htim, ADC_HandleTypeDef* adc, PID_motor* motor_left, PID_motor* motor_right, Line_controller* line_controller)
{
    // Read the line sensor via DMA
    lineSensorDistance(adc, &line_controller->robot_line_sensor);
    // Check if the line has ended
    if(line_controller->robot_line_sensor.adc_sensor_val[0] < 2500 && line_controller->robot_line_sensor.adc_sensor_val[1] < 2500 &&
    line_controller->robot_line_sensor.adc_sensor_val[2] < 2500 && line_controller->robot_line_sensor.adc_sensor_val[3] < 2500 && line_controller->robot_line_sensor.adc_sensor_val[4] < 2500)
    {
        robotStop(motor_left, motor_right);
        HAL_TIM_Base_Stop_IT(htim);
        return;
    }
    line_controller->e2 = line_controller->robot_line_sensor.sensor_output - line_controller->line_target;
    // Get the angular velocity of the robot by PD controller
    line_controller->angular_velocity = line_controller->line_Kp * line_controller->e2 + line_controller->line_Kd * (line_controller->robot_line_sensor.sensor_output - line_controller->prev_sensor_feedback);
    line_controller->prev_sensor_feedback = line_controller->robot_line_sensor.sensor_output;

    float left_motor_speed = line_controller->linear_velocity + line_controller->angular_velocity;
    float right_motor_speed = line_controller->linear_velocity - line_controller->angular_velocity;
    // Assign the speed to the motor controller
    inputSpeedHandling(motor_left, left_motor_speed);
    inputSpeedHandling(motor_right, right_motor_speed);
}

// Function to change the linear velocity of the robot
void linearVelocityUpdate(Line_controller* line_controller, float linear_vec)
{
    line_controller->linear_velocity = (linear_vec * 1000.0) / (line_controller->wheel_diameter * PI) * 60;
}

// Function to update the PID parameter of the line following controller
void linePIDUpdate(Line_controller* line_controller, float Kp, float Kd)
{
    line_controller->line_Kp = Kp;
    line_controller->line_Kd = Kd;
}

// Function to stop the whole robot when end of line
void robotStop(PID_motor* motor_left, PID_motor* motor_right)
{
    // Stop 2 motor immediately
    dutyCycleUpdate(0, motor_left);
    dutyCycleUpdate(0, motor_right);
    // Brake the motor
    motorBrake(motor_left);
    motorBrake(motor_right);
}

// Function to command the robot to turn left
void robotRotateLeft(PID_motor* motor_left, PID_motor* motor_right, Line_controller* line_controller, uint16_t motion_velocity, uint16_t motion_acel)
{
    float right_motor_angle = (line_controller->axle_length / (2.5 * line_controller->wheel_diameter)) * 360;
    // Input the path for 2 wheels
    inputPositionHandling(motor_left, 0.0, 0, 0);
    inputPositionHandling(motor_right, right_motor_angle, motion_velocity, motion_acel);
}

// Function to command the robot to turn right
void robotRotateRight(PID_motor* motor_left, PID_motor* motor_right, Line_controller* line_controller, uint16_t motion_velocity, uint16_t motion_acel)
{
    float left_motor_angle = (line_controller->axle_length / (2.5 * line_controller->wheel_diameter)) * 360;
    // Input the path for 2 wheels
    inputPositionHandling(motor_left, left_motor_angle, motion_velocity, motion_acel);
    inputPositionHandling(motor_right, 0.0, 0, 0);
}

void robotLinear(PID_motor* motor_left, PID_motor* motor_right, Line_controller* line_controller, float distance, uint16_t motion_velocity, uint16_t motion_acel)
{
    float motor_angles = (distance / (line_controller->wheel_diameter * PI)) * 360;
    // Input the path for 2 wheels
    inputPositionHandling(motor_left, motor_angles, motion_velocity, motion_acel);
    inputPositionHandling(motor_right, motor_angles, motion_velocity, motion_acel);
}