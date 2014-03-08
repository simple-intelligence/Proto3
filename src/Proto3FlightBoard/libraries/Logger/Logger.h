#ifndef LOGGER
#define LOGGER

#include <Arduino.h>

class Logger
{
public:
    bool Logger_On;
    int Logger_Rate;

    Logger(int logger_rate, bool logger_on);
    void Log_Int (int data);
    void Log_Float (float data);
    void Set_Log_On ();
    void Set_Log_Off ();
};

#endif
