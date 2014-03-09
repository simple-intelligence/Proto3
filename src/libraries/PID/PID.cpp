#include <Arduino.h>
#include "PID.h"

PID_Class::PID_Class (float kP, float kI, float kD, float Min_Integrator, float Max_Integrator, float Setpoint)
{
    _kP = kP;
    _kI = kI;
    _kD = kD;

    _Setpoint = Setpoint;

    _Min_Integrator = Min_Integrator;
    _Max_Integrator = Max_Integrator;

    _Integrator = 0.0;
    _Derivator = 0.0;
    _P_ = 0.0;
    _I_ = 0.0;
    _D_ = 0.0;

    Drive = 0.0;
}

void PID_Class::compute (float Actual)
{
    float _Error = _Setpoint - Actual;

    _P_ = _kP * _Error;

    _Integrator = _Integrator + _Error;
    if (_Integrator < _Min_Integrator) { _Integrator = _Min_Integrator; }
    if (_Integrator > _Max_Integrator) { _Integrator = _Max_Integrator; }
    _I_ = _kI * _Integrator;

    _D_ = _kD * (_Error - _Derivator);
    _Derivator = _Error;

    Drive = _P_ + _I_ + _D_;
}

float PID_Class::get_P ()
{
    return _P_;
}

float PID_Class::get_I ()
{
    return _I_;
}

float PID_Class::get_D ()
{
    return _D_;
}

void PID_Class::set_Setpoint (float new_setpoint)
{
    _Setpoint = new_setpoint;
    _Derivator = 0;
    _Integrator = 0;
}

