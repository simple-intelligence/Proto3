#include "Logger.h"
#include <Arduino.h>

Logger::Logger(int logger_rate, int logger_on)
{
    Logger_Rate = logger_rate;
    Logger_On = Logger_On;
    Log_Counter = 0;
}

void Logger::Log_Int (int data)
{
    if (Log_Counter == Logger_Rate)
    {
        Serial.print (data);
        Serial.print (" ");
    }

    Logger:Check_Counter ();
}

void Logger::Log_Float (float data)
{
    if (Log_Counter == Logger_Rate)
    {
        Serial.print (data);
        Serial.print (" ");
    }

    Logger:Check_Counter ();
}

void Logger::End_Line ()
{
    if (Log_Counter == Logger_Rate)
    {
        Serial.println ();
    }

    Logger:Check_Counter ();
}

void Logger::Count ()
{
    Log_Counter += 1;
}

void Logger::Set_Log_On ()
{
   Logger_On = true;
}

void Logger::Set_Log_Off ()
{
   Logger_On = false
}

void Logger::Check_Counter ()
{
    if (Log_Counter > Logger_Rate)
    {
        Log_Counter = 0;
    }
}
