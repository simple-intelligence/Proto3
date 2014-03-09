#ifndef MOTORS
#define MOTORS

#include <Arduino.h>
#include <Servo.h>

class Motor_Control
{
private:
    int MIN_PWM;
    int MID_PWM;
    int MAX_PWM;

    int Front_Left_Output;
    int Front_Right_Output;
    int Back_Left_Output;
    int Back_Right_Output;

    float Pitch_Input;
    float Roll_Input;
    float Yaw_Input;
    float Throttle_Input;

    Servo Front_Left_Pin;
    Servo Front_Right_Pin;
    Servo Back_Left_Pin;
    Servo Back_Right_Pin;

    void Map_Motor_Inputs ();

public:
    Motor_Control (int Min_Pwm, int Max_Pwm);

    void Set_Motor_Range (int Min_Pwm, int Max_Pwm);

    void Set_Motor_Inputs (float Throttle, float Pitch, float Roll, float Yaw);
    void Write_Motor_Out ();
};

#endif
