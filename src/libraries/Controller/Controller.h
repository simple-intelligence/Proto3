#ifndef CONTROLLER
#define CONTROLLER

#include <Arduino.h>

class Controller
{
public:
    int Message_Recieved;
    int Pitch_Input;
    int Roll_Input;
    int Throttle_Input;
    int Yaw_Input;
    int Arm_Input;

    Controller ();
    void Parse_Serial();
};

#endif
