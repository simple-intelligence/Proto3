#ifndef PID
#define PID

#include <Arduino.h>

class PID_Class
{
private:
    float _kP;
    float _kI;
    float _kD;

    float _P_;
    float _I_;
    float _D_;

    float _Setpoint;

    float _Min_Integrator;
    float _Max_Integrator;

    float _Integrator;
    float _Derivator;

public:
    PID_Class (float kP, float kI, float kD, float Min_Integrator, float Max_Integrator, float Setpoint);
    void compute (float Actual);
    float get_P ();
    float get_I ();
    float get_D ();
    void set_Setpoint (float new_setpoint);

    float Drive;
};

#endif
