#ifndef CONTROLLER
#define CONTROLLER

#include <Arduino.h>

class Controller
{
private:
    int Message_Recieved;
    
public:
    int Pitch_Input;
    int Roll_Input;
    int Throttle_Input;
    int Yaw_Input;
    int Arm_Input;

    Controller ();
    void Parse_Serial();
};

#endif
