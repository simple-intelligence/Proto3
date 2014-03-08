#include "Controller.h"
#include <Arduino.h>

Controller::Controller ()
{
    int Pitch_Input = 0;
    int Roll_Input = 0;
    int Throttle_Input = 0;
    int Yaw_Input = 0;
    int Arm_Input = 0;
}

void Controller::Parse_Serial ()
{
    int i = 0;
    if (Serial.available())
    {
        // Synchronize to leading 'B'
        do
        {
            i = Serial.read ();
        }
        while (i != 'B');
                
        Pitch_Input = Serial.parseInt(); 
        Yaw_Input = Serial.parseInt(); 
        Roll_Input = Serial.parseInt(); 
        Throttle_Input = Serial.parseInt(); 
        Arm_Input = Serial.parseInt();
        //Hover_Input = Serial.parseInt();

        Message_Recieved = true;
        return;
    }
    else
        Message_Recieved = false;
        return;
}
