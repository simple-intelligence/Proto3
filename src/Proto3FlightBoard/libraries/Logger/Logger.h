#ifndef LOGGER
#define LOGGER

class Logger
{
public:
    bool Logger_On;
    int Logger_Rate;

    Logger(int logger_rate, bool logger_on);
    void Log_Int (int data);
    void Log_Float (float data);
    void Log_On (bool status);
    void Log_Off (bool status);
};

#endif
