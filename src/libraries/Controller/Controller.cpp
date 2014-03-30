#include <Arduino.h>
#include "Controller.h"

Controller::Controller (int control_timeout_limit)
{
    Pitch_Input = 0;
    Roll_Input = 0;
    Throttle_Input = 0;
    Yaw_Input = 0;
    Arm_Input = 0;
    Stabalize_Input = 0;
    Calibrate_Input = 0;

    Control_Timeout = true;
    Control_Counter = 0;
    Control_Timeout_Limit = control_timeout_limit;
}

void Controller::Parse_Serial ()
{
    // Synchronize to leading 'B'
    if (Serial.read() == 'B')
    {
        Pitch_Input = Serial.parseInt(); 
        Yaw_Input = Serial.parseInt(); 
        Roll_Input = Serial.parseInt(); 
        Throttle_Input = Serial.parseInt(); 
        Arm_Input = Serial.parseInt();
        Stabalize_Input = Serial.parseInt();
        Calibrate_Input = Serial.parseInt();

        // Message recieved and reset timeout
        Message_Recieved = true;
        Control_Counter = 0;
        Control_Timeout = false;
        
        return;
    }
    else
    {
        Check_Timeout();
        Message_Recieved = false;
        return;
    }
}

void Controller::Check_Timeout()
{
    Control_Counter += 1;
    if (Control_Counter >= Control_Timeout_Limit)
    {
        Control_Timeout = true;
    }
    else { Control_Timeout = false; }
}
