#ifndef PID
#define PID

class PID
{
private:
    float _kP;
    float _kI;
    float _kD;

    float _P;
    float _I;
    float _D;

    float _Setpoint;

    float _Min_Integrator;
    float _Max_Integrator;

    float _Integrator;
    float _Derivator;

public:
    float Drive;

    PID (float kP, float kI, float kD, float Min_Integrator, float Max_Integrator, float Setpoint);
    void Compute (float Actual);
    float get_P ();
    float get_I ();
    float get_D ();
    void set_Setpoint (float new_setpoint);
};

#endif
