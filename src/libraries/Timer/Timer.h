#ifndef TIMER
#define TIMER

#include <Arduino.h>

class Timer
{
private:
    int lastTime;
    unsigned long current_time;
    unsigned long last_time;
    float dt;

    inline float calcDT ();

public:
    Timer ();
    float getDT ();
    void update ();
}


#endif
