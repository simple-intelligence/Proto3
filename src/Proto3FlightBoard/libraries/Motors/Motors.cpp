#include "Motors.h"
#include <Servo.h>
#include <Arduino.h>

Motor_Control::Motor_Control (int Min_Pwm, int Max_Pwm)
{
    MIN_PWM = Min_Pwm;
    MAX_PWM = Max_Pwm;
    MID_PWM = (Min_Pwm + Max_Pwm) / 2;

    Front_Left_Output = 0;
    Front_Right_Output = 0;
    Back_Left_Output = 0;
    Back_Right_Output = 0;

    Pitch_Input = 0.0f;
    Roll_Input = 0.0f;
    Yaw_Input = 0.0f;
    Throttle_Input = 0.0f;

    Front_Left_Pin.attach(A2);
    Front_Right_Pin.attach(A4);
    Back_Left_Pin.attach(A3);
    Back_Right_Pin.attach(A1);
}

void Motor_Control::Set_Motor_Inputs (float Throttle, float Pitch, float Roll, float Yaw)
{
    Throttle_Input = Throttle;
    Pitch_Input = Pitch;
    Roll_Input = Roll;
    Yaw_Input = Yaw;
}

void Motor_Control::Write_Motor_Out ()
{

    Front_Left_Pin.writeMicroseconds(Front_Left_Output);
    delayMicroseconds(1000);

    Front_Right_Pin.writeMicroseconds(Front_Right_Output);
    delayMicroseconds(1000);   

    Back_Left_Pin.writeMicroseconds(Back_Left_Output);
    delayMicroseconds(1000); 

    Back_Right_Pin.writeMicroseconds(Back_Right_Output);
    delayMicroseconds(1000);
}

void Motor_Control::Set_Motor_Range (int Min_Pwm, int Max_Pwm)
{
    MIN_PWM = Min_Pwm;
    MAX_PWM = Max_Pwm;
    MID_PWM = (Min_Pwm + Max_Pwm) / 2;
}

void Motor_Control::Map_Motor_Inputs ()
{
    // Check if yaw is correct
    Front_Left_Output = Throttle_Input + Roll_Input - Pitch_Input + Yaw_Input;
    Front_Right_Output = Throttle_Input - Roll_Input - Pitch_Input - Yaw_Input;

    Back_Left_Output = Throttle_Input + Roll_Input + Pitch_Input + Yaw_Input;
    Back_Right_Output = Throttle_Input - Roll_Input + Pitch_Input - Yaw_Input;
}

