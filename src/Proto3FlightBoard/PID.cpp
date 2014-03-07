#include "PID.h"

PID (float kP, float kI, float kD, float Min_Integrator, float Max_Integrator, float Setpoint)
{
    _kP = kP;
    _kI = kI;
    _kD = kD;

    _Setpoint = Setpoint;

    _Min_Integrator = Min_Integrator;
    _Max_Integrator = Max_Integrator;

    _Integrator = 0;
    _Derivator = 0;
    _P = 0;
    _I = 0;
    _D = 0;
}

void Compute (float Actual)
{
    float _Error = _Setpoint - Actual;

    _P = _kP * _Error;

    _Integrator = _Integrator + _Error;
    if (_Integrator < _Min_Integrator) { _Integrator = _Min_Integrator; }
    if (_Integrator > _Max_Integrator) { _Integrator = _Max_Integrator; }
    _I = _kI * _Integrator;

    _D = _kD * (_Error - _Derivator);
    _Derivator = _Error;
}

float get_P ()
{
    return _P;
}

void get_I ()
{
    return _I;
}

void get_D ()
{
    return _D;
}

void set_Setpoint (float new_setpoint)
{
    _Setpoint = new_setpoint;
}

