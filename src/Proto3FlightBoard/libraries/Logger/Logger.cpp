#include "Logger.h"
#include <Arduino.h>

Logger::Logger(int logger_rate, int logger_on)
{
    Logger_Rate = logger_rate;
    Logger_On = Logger_On;
    Log_Counter = 0;
}

void Logger::Count ()
{
    Log_Counter += 1;
    if (Log_Counter == Logger_Rate) { Log_This_Cycle = 1; }
    else { Log_This_Cycle = 0; }

    if (Log_Counter > Logger_Rate) { Log_Counter = 0; }
}

void Logger::Log_Int (int data)
{
    Serial.print (data);
    Serial.print (" ");
}

void Logger::Log_Float (float data)
{
    Serial.print ((int)data);
    Serial.print (" ");
}

void Logger::End_Line ()
{
    Serial.println ();
}


void Logger::Set_Log_On ()
{
    Logger_On = true;
}

void Logger::Set_Log_Off ()
{
    Logger_On = false
}
