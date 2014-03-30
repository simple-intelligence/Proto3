#include <Arduino.h>

#include "Comp_Filter.h"

Complementary_Filter::Complementary_Filter (float accel_constant, float gyro_constant)
{
    unsigned long current_time = 0;
    unsigned long last_time = 0;
    unsigned long dt = 0;

    aC = accel_constant;
    gC = gyro_constant;
    angle = 0.0;
}

void Complementary_Filter::Calculate (float gyro_data, float accel_angle)
{
    current_time = millis ();
     
    dt = (current_time - last_time) / 1000.0;
    
    angle_radians = ((gC * (angle_radians + (gyro_data * (float)dt))) + (aC * accel_angle));
    angle = angle_radians * 64.0;

    last_time = current_time;
}
