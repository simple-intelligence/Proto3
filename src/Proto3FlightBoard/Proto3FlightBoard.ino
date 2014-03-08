#include "Kalman.h"
#include "Sensors.h"
#include "PID.h"
#include "Logger.h"
#include "math.h"
#include "Controller.h"

Sensor_Data = Sensors (12, 13);

Pitch_Kalman = Kalman ();
Roll_Kalman = Kalman ();

Pitch_PID = PID (0.1, 0.1, 0.1, -20, 20, 0);
Roll_PID = PID (0.1, 0.1, 0.1, -20, 20, 0);
Throttle_PID = PID (0.1, 0.1, 0.1, -20, 20, 0);

Controls = Controller ();

Motors = Motor_Control (800, 1600);

void setup ()
{
    Serial.begin (115200);
    
    Sensor_Data.init_sensors ();
}

void loop ()
{
    float Raw_Pitch_Angle = 0.0;
    float Raw_Roll_Angle = 0.0;

    Sensor_Data.read_sensors ();

    Raw_Pitch_Angle = atan2 ((float)Sensor_Data.raw_accel_data[0], (float)Sensor_Data.raw_accel_data[2]);
    Raw_Roll_Angle = atan2 ((float)Sensor_Data.raw_accel_data[1], (float)Sensor_Data.raw_accel_data[2]);

    Pitch_Kalman.compute (Raw_Pitch_Angle, (float)Sensor_Data.raw_gyro_data[0]); 
    Roll_Kalman.compute (Raw_Roll_Angle, (float)Sensor_Data.raw_gyro_data[1]); 

    if (!Controls.Message_Recieved)
    {
        Pitch_PID.compute (Pitch_Kalman.x1);
        Roll_PID.compute (Roll_Kalman.x1);
        Throttle_PID.compute (Sensor_Data.range);
    }
    
    Controls.Parse_Serial ();

    if (Controls.Message_Recieved)
    {
        Motors.Set_Motor_Inputs (Controls.Throttle_Input, Controls.Pitch_Input, Controls.Roll_Input, Controls.Yaw_Input);
    }
    else
    {
        Motors.Set_Motor_Inputs (Throttle_PID.Drive, Pitch_PID.Drive, Roll_PID.Drive, 0.0);
    }
}
