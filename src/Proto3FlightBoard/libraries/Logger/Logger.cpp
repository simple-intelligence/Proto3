#include "Logger.h"
#include <Arduino.h>

Logger::Logger(int logger_rate, bool logger_on)
{
    Logger_Rate = logger_rate;
    Logger_On = Logger_On;
}

void Logger::Log_Int (int data)
{

}

void Logger::Log_Float (float data)
{

}

void Logger::Set_Log_On ()
{
   Logger_On = true;
}

void Logger::Set_Log_Off ()
{
   Logger_On = false
}

