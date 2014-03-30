#ifndef CONTROLLER
#define CONTROLLER

#include <Arduino.h>

class Controller
{
private:
    int Control_Counter;
    int Control_Timeout_Limit;

public:
    int Pitch_Input;
    int Roll_Input;
    int Throttle_Input;
    int Yaw_Input;
    int Arm_Input;
    int Stabalize_Input;
    int Calibrate_Input;

    bool Control_Timeout;
    bool Message_Recieved;

    Controller (int control_timeout);
    void Parse_Serial();
    void Check_Timeout();
};

#endif
