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

    current_time = 0;
    last_time = 0;
}

void Motor_Control::Init_Motors (int front_left, int front_right, int back_left, int back_right)
{
    pinMode (0, OUTPUT);
    digitalWrite (0, HIGH);

    Front_Left_Pin.attach (front_left);  
    Front_Right_Pin.attach (front_right);  
    Back_Left_Pin.attach (back_left);  
    Back_Right_Pin.attach (back_right);  

    // Zero motors
    delay (60);
    Set_Motor_Inputs (0.0, 0.0, 0.0, 0.0);
    Write_Motor_Out ();
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
    Map_Motor_Inputs ();

    current_time = millis ();
     
    // Sets a motor refresh rate of 20 hz. (50 millisecond period)
    if (current_time - last_time > 50)
    {
        last_time = current_time;

        Front_Left_Pin.writeMicroseconds (Front_Left_Output_Int);
        Front_Right_Pin.writeMicroseconds (Front_Right_Output_Int);
        Back_Left_Pin.writeMicroseconds (Back_Left_Output_Int);
        Back_Right_Pin.writeMicroseconds (Back_Right_Output_Int);

        delay (15);
    }
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
    // TODO:Check if yaw is correct
    Front_Left_Output = Throttle_Input + Roll_Input - Pitch_Input + Yaw_Input;
    Front_Right_Output = Throttle_Input - Roll_Input - Pitch_Input - Yaw_Input;
    Back_Left_Output = Throttle_Input + Roll_Input + Pitch_Input - Yaw_Input;
    Back_Right_Output = Throttle_Input - Roll_Input + Pitch_Input + Yaw_Input;

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

void Motor_Control::Motor_Test ()
{
    int test_delay = 1000;

    delay (test_delay);
    Set_Motor_Inputs (0.0, 0.0, 0.0, 0.0);
    Write_Motor_Out ();
    delay (test_delay);

    Serial.println ("Testing Motors!");

    // FL
    Serial.println ("Front Left:");
    Set_Motor_Inputs (50.0, 0.0, 0.0, 0.0);
    Write_Motor_Out ();
    delay (test_delay);
    Serial.println ("Done.");

    // FR
    Serial.println ("Front Right");
    Set_Motor_Inputs (0.0, 50.0, 0.0, 0.0);
    Write_Motor_Out ();
    delay (test_delay);
    Serial.println ("Done.");

    // BL
    Serial.println ("Back Left");
    Set_Motor_Inputs (0.0, 0.0, 50.0, 0.0);
    Write_Motor_Out ();
    delay (test_delay);
    Serial.println ("Done.");

    // BR
    Serial.println ("Back Right");
    Set_Motor_Inputs (0.0, 0.0, 0.0, 50.0);
    Write_Motor_Out ();
    delay (test_delay);
    Serial.println ("Done.");

    Serial.println ("Finished Motor Test!");

    Set_Motor_Inputs (0.0, 0.0, 0.0, 0.0);
    Write_Motor_Out ();
    delay (test_delay);
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
