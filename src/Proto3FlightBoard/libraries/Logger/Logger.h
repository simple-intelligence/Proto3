#ifndef LOGGER
#define LOGGER

#include <Arduino.h>

class Logger
{
public:
    int Logger_On;
    int Logger_Rate;
    int Log_Counter;
    int Log_This_Cycle;

    Logger(int logger_rate, int logger_on);
    void Log_Int (int data);
    void Log_Float (float data);
    void Set_Log_On ();
    void Set_Log_Off ();
    void Count ();
    void End_Line ();
};

#endif
