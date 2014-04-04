#ifndef COMPLEMENTARY_FILTER
#define COMPLEMENTARY_FILTER

class Complementary_Filter
{
private:
    float aC;
    float gC;
    
    unsigned long current_time;
    unsigned long last_time;
    float dt;

public:
    float angle_radians;
    float angle;

    Complementary_Filter (float accel_constant, float gyro_constant);
    void Calculate (float gyro_data, float accel_angle);
};

#endif
