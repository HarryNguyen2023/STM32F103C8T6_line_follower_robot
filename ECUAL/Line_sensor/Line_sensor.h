#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include "stm32f1xx_hal.h"

#define SENSOR_NUM 5

// Define the structure of line sesnor
typedef struct 
{
    int8_t sensor_weight[SENSOR_NUM];
    uint16_t adc_sensor_val[SENSOR_NUM];
    float sensor_calib[SENSOR_NUM];
    uint16_t sensor_real_max_min[2][SENSOR_NUM];
    float sensor_max_mean;
    float sensor_min_mean;
    float sensor_output;
}Line_sensor;

// Function prototypes
void lineSensorInit(ADC_HandleTypeDef* adc, Line_sensor* line_sensor);
void getSensorDistance(ADC_HandleTypeDef* adc, Line_sensor* line_sensor);

#endif
