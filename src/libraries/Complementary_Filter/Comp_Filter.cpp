#include <Arduino.h>

#include "Comp_Filter.h"

Complementary_Filter::Complementary_Filter (float dt, float accel_constant, float gyro_constant)
{
    DT = dt;
    aC = accel_constant;
    gC = gyro_constant;
    angle = 0.0;
}

void Complementary_Filter::Calculate (float gyro_data, float accel_angle)
{
   angle = gC * (angle + (gyro_data * DT)) + aC * accel_angle;
}
