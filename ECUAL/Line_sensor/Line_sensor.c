#include "Line_sensor.h"

// Function to initiate the line sensor
void lineSensorInit(ADC_HandleTypeDef* adc, Line_sensor* line_sensor)
{
    // Calibrate the ADC module
    HAL_ADCEx_Calibration_Start(adc);
    // Initiate the line sensor read for the 1st time
    HAL_ADC_Start_DMA(adc, (uint32_t*)line_sensor->adc_sensor_val, SENSOR_NUM);
}

// Function to activate reading of line sensor and get the output distance
void lineSensorDistance(ADC_HandleTypeDef* adc, Line_sensor* line_sensor)
{
    float sensor_weight = 0;
    float sensor_sum = 0;
    // Start reading the sensor via ADC
    HAL_ADC_Start_DMA(adc, (uint32_t*)line_sensor->adc_sensor_val, SENSOR_NUM);
    // calibrate the sensor and calculate the weighted sum
    for(uint8_t i = 0; i < SENSOR_NUM; ++i)
    {
        line_sensor->sensor_calib[i] = line_sensor->sensor_min_mean + (line_sensor->sensor_max_mean - line_sensor->sensor_min_mean) * (line_sensor->adc_sensor_val[i] - line_sensor->sensor_real_max_min[0][i]) *1.0 / (line_sensor->sensor_real_max_min[1][i] - line_sensor->sensor_real_max_min[0][i]);
        sensor_weight += line_sensor->sensor_weight[i] * line_sensor->sensor_calib[i];
        sensor_sum += line_sensor->sensor_calib[i];
    }
    line_sensor->sensor_output = sensor_weight / sensor_sum + 0.6;
}


