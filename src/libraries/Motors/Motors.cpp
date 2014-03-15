#include <Arduino.h>
#include "Motors.h"

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
}

void Motor_Control::Init_Motors ()
{
    Front_Left_Pin.attach (2);  
    Front_Right_Pin.attach (3);  
    Back_Left_Pin.attach (4);  
    Back_Right_Pin.attach (5);  

    Front_Left_Output_Int = 10;
    Front_Right_Output_Int = 10;
    Back_Left_Output_Int = 10;
    Back_Right_Output_Int = 10;
    Write_Motor_Out();
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
    Front_Left_Pin.writeMicroseconds(Front_Left_Output_Int * 1000);
    //delayMicroseconds(1000);

    Front_Right_Pin.writeMicroseconds(Front_Right_Output_Int * 1000);
    //delayMicroseconds(1000);   

    Back_Left_Pin.writeMicroseconds(Back_Left_Output_Int * 1000);
    //delayMicroseconds(1000); 

    Back_Right_Pin.writeMicroseconds(Back_Right_Output_Int * 1000);
    //delayMicroseconds(1000);
    delayMicroseconds (1500);
}

void Motor_Control::Set_Motor_Range (int Min_Pwm, int Max_Pwm)
{
    MIN_PWM = Min_Pwm;
    MAX_PWM = Max_Pwm;
    MID_PWM = (Min_Pwm + Max_Pwm) / 2;
}

void Motor_Control::Map_Motor_Inputs ()
{
    // Calc motor outputs
    // Check if yaw is correct
    Front_Left_Output = Throttle_Input + Roll_Input - Pitch_Input + Yaw_Input;
    Front_Right_Output = Throttle_Input - Roll_Input - Pitch_Input - Yaw_Input;
    Back_Left_Output = Throttle_Input + Roll_Input + Pitch_Input + Yaw_Input;
    Back_Right_Output = Throttle_Input - Roll_Input + Pitch_Input - Yaw_Input;

    // Constrain
    Front_Left_Output = constrain (Front_Left_Output, 0.0, 100.0);
    Front_Right_Output = constrain (Front_Right_Output, 0.0, 100.0);
    Back_Left_Output = constrain (Back_Left_Output, 0.0, 100.0);
    Back_Right_Output = constrain (Back_Right_Output, 0.0, 100.0);

    // Map to pwm
    Front_Left_Output_Int = map (Front_Left_Output, 0.0, 100.0, MIN_PWM, MAX_PWM);
    Front_Right_Output_Int = map (Front_Right_Output, 0.0, 100.0, MIN_PWM, MAX_PWM);
    Back_Left_Output_Int = map (Back_Left_Output, 0.0, 100.0, MIN_PWM, MAX_PWM);
    Back_Right_Output_Int = map (Back_Right_Output, 0.0, 100.0, MIN_PWM, MAX_PWM);
}
/*
void Motor_Control::Calibrate_ESCS ()
{
    Front_Left_Output_Int = MAX_PWM;
    Front_Right_Output_Int = MAX_PWM;
    Back_Left_Output_Int = MAX_PWM;
    Back_Right_Output_Int = MAX_PWM;
}
*/
