#ifndef MOTORS
#define MOTORS

#include <Arduino.h>
#include <Servo.h>

class Motor_Control
{
private:
    int MIN_PWM;
    int ZERO_PWM;
    int MAX_PWM;

    int Front_Left_Output;
    int Front_Right_Output;
    int Back_Left_Output;
    int Back_Right_Output;

    float Pitch_Input;
    float Roll_Input;
    float Yaw_Input;
    float Throttle_Input;

    Servo Pitch_Pin;
    Servo Roll_Pin;
    Servo Throttle_Pin;
    Servo Yaw_Pin;

    void Map_Motor_Inputs ();

public:
    Motor_Control ();

    void Set_Motor_Range ();

    void Motor_Inputs (float Throttle, float Pitch, float Roll, float Yaw)
    void Write_Motor_Out ();
};

#endif
