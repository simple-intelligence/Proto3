// Summer: Implement me!
#include "Timer.h"
#include <Arduino.h>

Timer::Timer ()
{
    current_time = millis ();
    last_time = millis ();
    dt = 0.0;
    _calcDT ();
}

inline float Timer::_calcDT ()
{
    current_time = millis ();
     
    dt = (float)((current_time - last_time) / 1000.0);
    
    last_time = current_time;
}

void update ()
{
    _calcDT ();
}

float Timer::getDT ()
{
    return dt;
}

