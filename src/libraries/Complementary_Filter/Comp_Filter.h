#ifndef COMPLEMENTARY_FILTER
#define COMPLEMENTARY_FILTER

class Complementary_Filter
{
private:
    float DT;
    float aC;
    float gC;

public:
    float angle;

    Complementary_Filter (float dt, float accel_constant, float gyro_constant);
    void Calculate (float gyro_data, float accel_angle);
};

#endif
