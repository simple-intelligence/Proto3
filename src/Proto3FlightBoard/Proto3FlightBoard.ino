#include <Wire.h>
#include <Servo.h>

#include "Kalman.h"
#include "Sensors.h"
#include "PID.h"
#include "Logger.h"
#include "Controller.h"
#include "Motors.h"

#include "math.h"

/*******************
* Flight Libraries *
*******************/

Sensors Sensor_Data (12, 13);

Kalman Pitch_Kalman;
Kalman Roll_Kalman;

PID_Class Pitch_PID (0.1, 0.1, 0.1, -20.0, 20.0, 0.0);
PID_Class Roll_PID (0.1, 0.1, 0.1, -20.0, 20.0, 0.0);
PID_Class Throttle_PID (0.1, 0.1, 0.1, -20.0, 20.0, 0.0);

Controller Controls;

Motor_Control Motors (800, 1600);

Logger Logging (50, 1);

/*****************
* Initialization *
*****************/

void setup ()
{
    // .begin methods apparently need to be called from setup and nowhere else
    Wire.begin ();  
    Serial.begin (115200);
    
    Sensor_Data.init_sensors ();
}

void loop ()
{
    /*****************
    * Control Loop   *
    *****************/

    double Raw_Pitch_Angle = 0.0;
    double Raw_Roll_Angle = 0.0;

    Sensor_Data.read_sensors ();

    Raw_Pitch_Angle = atan_2 ((double)Sensor_Data.raw_accel_data[0], (double)Sensor_Data.raw_accel_data[2]);
    Raw_Roll_Angle = atan_2 ((double)Sensor_Data.raw_accel_data[1], (double)Sensor_Data.raw_accel_data[2]);

    //Pitch_Kalman.compute ((float)Raw_Pitch_Angle, (float)Sensor_Data.raw_gyro_data[0]); 
    //Roll_Kalman.compute ((float)Raw_Roll_Angle, (float)Sensor_Data.raw_gyro_data[1]); 

    //Pitch_Kalman.compute (5.5, 21.0);
    //Roll_Kalman.compute (2.12, 10.294); 

    //if (!Controls.Message_Recieved)
    //{
        //Pitch_PID.compute (Pitch_Kalman.x1);
        //Roll_PID.compute (Roll_Kalman.x1);
        //Throttle_PID.compute (Sensor_Data.range);
    //}
    
    //Controls.Parse_Serial ();

    //if (Controls.Message_Recieved)
    //{
        //Motors.Set_Motor_Inputs (Controls.Throttle_Input, Controls.Pitch_Input, Controls.Roll_Input, Controls.Yaw_Input);
    //}
    //else
    //{
        //Motors.Set_Motor_Inputs (Throttle_PID.Drive, Pitch_PID.Drive, Roll_PID.Drive, 0.0);
    //}

    /************
    * Logging   *
    *************/

    Serial.print (Sensor_Data.raw_accel_data[0]);
    Serial.print ("\t");
    Serial.print (Sensor_Data.raw_accel_data[1]);
    Serial.print ("\t");
    Serial.print (Sensor_Data.raw_accel_data[2]);
    Serial.print ("\t");
    Serial.print (Raw_Pitch_Angle);
    Serial.print ("\t");
    Serial.print (Raw_Roll_Angle);
    Serial.print ("\n");    
    delay (10);

    //Logging.Count ();
    //if (Logging.Log_This_Cycle && Logging.Logger_On)
    //{
        //Logging.Log_Float (Raw_Pitch_Angle);
        //Logging.Log_Float (Raw_Roll_Angle);

        //Logging.Log_Float (Pitch_Kalman.x1);
        //Logging.Log_Float (Roll_Kalman.x1);
        //Logging.Log_Float (Sensor_Data.range);

        //Logging.Log_Float (Throttle_PID.Drive);
        //Logging.Log_Float (Pitch_PID.Drive);
        //Logging.Log_Float (Roll_PID.Drive);

        //Logging.Log_Int (Controls.Throttle_Input);
        //Logging.Log_Int (Controls.Pitch_Input);
        //Logging.Log_Int (Controls.Roll_Input);
        //Logging.Log_Int (Controls.Yaw_Input);
        //Logging.End_Line ();
    //}
}

double atan_2 (double y, double x)
{
    if (x > 0) { return atan (y/x); }
    else if (y >= 0 && x < 0) { return 3.141592 + atan (y/x); }
    else if (y < 0 && x < 0) { return -3.141592 + atan (y/x); }
    else if (y > 0) { return 3.141592 / 2.0; }
    else if (y < 0) { return -3.141592 / 2.0; }
    // Else returning 0 although technically undefined
    else { return 0; }  
}





