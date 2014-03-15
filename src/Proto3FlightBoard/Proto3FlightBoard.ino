#include <Wire.h>
#include <Servo.h>

#include "Kalman.h"
#include "Sensors.h"
#include "PID.h"
#include "Logger.h"
#include "Controller.h"
#include "Motors.h"
#include "Comp_Filter.h"

#include "math.h"

/*******************
* Flight Libraries *
*******************/

Sensors Sensor_Data (12, 13); // Trig, Echo

Kalman Pitch_Kalman (.01); // DT
Kalman Roll_Kalman (.01);

Complementary_Filter Pitch_Comp (.01, .05, .95); // DT, Accel_Multiplier, Gyro_Multiplier
Complementary_Filter Roll_Comp (.01, .05, .95);

PID_Class Pitch_PID (5, 1.0, 0.0, -20.0, 20.0, 0.0); // kP, kI, kD, Min_Integrator, Max_Integrator, Setpoint
PID_Class Roll_PID (5, 1.0, 0.0, -20.0, 20.0, 0.0);
PID_Class Throttle_PID (0.2, 0.2, 0.0, -10.0, 10.0, 100.0);

Controller Controls;

Motor_Control Motors (10, 180); // Min_PWM, Max_PWM

Logger Logging (20, 1); // Logging_Rate, On/Off

/*****************
* Initialization *
*****************/

void setup ()
{
    // .begin methods apparently need to be called from setup and nowhere else
    Wire.begin ();  
    Serial.begin (9600);

    Sensor_Data.init_sensors ();
    Motors.Init_Motors ();
}

void loop ()
{
    /*****************
    * Control Loop   *
    *****************/

    float Raw_Pitch_Angle = 0.0;
    float Raw_Roll_Angle = 0.0;
    
    Sensor_Data.read_sensors ();

    Raw_Pitch_Angle = atan2 ((float)Sensor_Data.raw_accel_data[0], (float)Sensor_Data.raw_accel_data[2]);
    Raw_Roll_Angle = atan2 ((float)Sensor_Data.raw_accel_data[1], (float)Sensor_Data.raw_accel_data[2]);
    
    //Pitch_Kalman.compute (Raw_Pitch_Angle, (float)Sensor_Data.raw_gyro_data[1] / 14.375);
    //Roll_Kalman.compute (Raw_Roll_Angle, (float)Sensor_Data.raw_gyro_data[0] / 14.375);

    Pitch_Comp.Calculate (Sensor_Data.raw_gyro_data[1] / 14.375, Raw_Pitch_Angle);
    Roll_Comp.Calculate (Sensor_Data.raw_gyro_data[0] / 14.375, Raw_Roll_Angle);
    
    Controls.Parse_Serial ();

    if (!Controls.Message_Recieved)
    {
        //Pitch_PID.compute (Pitch_Kalman.x1);
        //Roll_PID.compute (Roll_Kalman.x1);
        Throttle_PID.compute (Sensor_Data.range);

        // Bypassing Kalman for testing
        Pitch_PID.compute (Raw_Pitch_Angle);
        Roll_PID.compute (Raw_Roll_Angle);
    }

    if (Controls.Message_Recieved)
    {
        Motors.Set_Motor_Inputs (Controls.Throttle_Input, Controls.Pitch_Input, Controls.Roll_Input, Controls.Yaw_Input);
    }
    else
    {
        //Motors.Set_Motor_Inputs (Throttle_PID.Drive, Pitch_PID.Drive, Roll_PID.Drive, 0.0);
        Motors.Set_Motor_Inputs (50.0, Pitch_PID.Drive, Roll_PID.Drive, 0.0);
    }
    
    // Run Motors
    Motors.Map_Motor_Inputs();
    Motors.Write_Motor_Out();
    
    /************
    * Logging   *
    *************/

    Logging.Count ();
    if (Logging.Log_This_Cycle && Logging.Logger_On)
    {
        //Logging.Log_Int (Sensor_Data.raw_accel_data[0]);
        //Logging.Log_Int (Sensor_Data.raw_accel_data[1]);
        //Logging.Log_Int (Sensor_Data.raw_accel_data[2]);

        //Logging.Log_Float (Raw_Pitch_Angle * 64.0);
        //Logging.Log_Float (Raw_Roll_Angle * 64.0);
        
        //Logging.Log_Float (Pitch_Kalman.x1 * 64.0);
        //Logging.Log_Float (Roll_Kalman.x1 * 64.0);
        
        //Logging.Log_Float (Pitch_Comp.angle * 64.0);
        //Logging.Log_Float (Roll_Comp.angle * 64.0);
        
        //Logging.Log_Float (Sensor_Data.range);
        
        //Logging.Log_Int (Sensor_Data.raw_gyro_data[0] / 14.375);
        //Logging.Log_Int (Sensor_Data.raw_gyro_data[1] / 14.375);
        //Logging.Log_Int (Sensor_Data.raw_gyro_data[2] / 14.375);

        //Logging.Log_Float (-Throttle_PID.Drive);
        //Logging.Log_Float (Pitch_PID.Drive);
        //Logging.Log_Float (Roll_PID.Drive);

        //Serial.print ("\n");
        
        //Logging.Log_Float (Controls.Throttle_Input);.
        //Logging.Log_Float (Controls.Pitch_Input);
        //Logging.Log_Float (Controls.Roll_Input);
        //Logging.Log_Float (Controls.Yaw_Input);
        
        //Logging.Log_Float (Motors.Throttle_Input);
        //Logging.Log_Float (Motors.Pitch_Input);
        //Logging.Log_Float (Motors.Roll_Input);
        //Logging.Log_Float (Motors.Yaw_Input);
        
        Logging.Log_Int (Motors.Front_Left_Output_Int);
        Logging.Log_Int (Motors.Front_Right_Output_Int);
        Logging.Log_Int (Motors.Back_Left_Output_Int);
        Logging.Log_Int (Motors.Back_Right_Output_Int);
        
        Logging.End_Line ();
    }
}

float atan_2 (float y, float x)
{
    if (x > 0) { return atan (y/x); }
    else if (y >= 0 && x < 0) { return 3.141592 + atan (y/x); }
    else if (y < 0 && x < 0) { return -3.141592 + atan (y/x); }
    else if (y > 0) { return 3.141592 / 2.0; }
    else if (y < 0) { return -3.141592 / 2.0; }
    // Else returning 0 although technically undefined
    else { return 0; }  
}





