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
    unsigned long current_time;
    unsigned long last_time;

    void Map_Motor_Inputs ();

public:
    float Pitch_Input;
    float Roll_Input;
    float Yaw_Input;
    float Throttle_Input;

    float Front_Left_Output;
    float Front_Right_Output;
    float Back_Left_Output;
    float Back_Right_Output;

    int Front_Left_Output_Int;
    int Front_Right_Output_Int;
    int Back_Left_Output_Int;
    int Back_Right_Output_Int;

    Servo Front_Left_Pin;
    Servo Front_Right_Pin;
    Servo Back_Left_Pin;
    Servo Back_Right_Pin;

    Motor_Control (int Min_Pwm, int Max_Pwm);

    void Init_Motors (int front_left, int front_right, int back_left, int back_right);
    void Set_Motor_Range (int Min_Pwm, int Max_Pwm);
    void Set_Motor_Inputs (float Throttle, float Pitch, float Roll, float Yaw);
    void Write_Motor_Out ();
    void Calibrate_ESCS ();
    void Motor_Test ();
};

#endif
