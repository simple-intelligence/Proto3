/* Proto Flight Board Version 2 */

#include <Wire.h>
#include <Servo.h>
#include <EEPROM.h>

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

Kalman Pitch_Kalman (0.008); // DT
Kalman Roll_Kalman (0.008);

Complementary_Filter Pitch_Comp (0.05, 0.95); // Accel_Multiplier, Gyro_Multiplier
Complementary_Filter Roll_Comp (0.05, 0.95);

PID_Class Pitch_PID (1.0, .1, 0.0, -20.0, 20.0, 0.0); // kP, kI, kD, Min_Integrator, Max_Integrator, Setpoint
PID_Class Roll_PID (1.0, .1, 0.0, -20.0, 20.0, 0.0);
PID_Class Throttle_PID (0.2, 0.2, 0.0, -10.0, 10.0, 0.0);

Controller Controls (100);

Motor_Control Motors (23, 100); // Min_PWM, Max_PWM

Logger Logging (10, 1); // Logging_Rate (cycles/msg), On/Off

/**********
* Globals *
**********/

bool ARMED = false;
bool RADIO_BYPASS = false; // Future addition possibly
bool EMERGENCY = false;

void setup ()
{
    Serial.begin (9600);

    Serial.println ("Initializing SI ProtoCopter!");
    Motors.Init_Motors (2, 3, 4, 5); // FL, FR, BL, BR
    Sensor_Data.init_sensors ();
    
    //Serial.println ("Calibrating Gyro!");
    //Sensor_Data.calibrate_sensors ();
}

void loop ()
{
    Sensor_Data.read_sensors (0);

    // Raw angles
    float Raw_Pitch_Angle = atan2 (Sensor_Data.calibrated_accel_data[0], Sensor_Data.calibrated_accel_data[2]);
    float Raw_Roll_Angle = atan2 (Sensor_Data.calibrated_accel_data[1], Sensor_Data.calibrated_accel_data[2]);

    // Filter
    Pitch_Comp.Calculate (Sensor_Data.calibrated_gyro_data[1], Raw_Pitch_Angle);
    Roll_Comp.Calculate (Sensor_Data.calibrated_gyro_data[0], Raw_Roll_Angle);

    // PID
    Pitch_PID.compute (Pitch_Comp.angle * 64.0);
    Roll_PID.compute (Roll_Comp.angle * 64.0);
    Throttle_PID.compute (Sensor_Data.range);

    // TODO: Write diagram for this
    Controls.Parse_Serial ();
    Check_Emergency ();
    if (!EMERGENCY)
    {
        if (Controls.Message_Recieved)
        {
            if (ARMED)
            {
                if (!Controls.Arm_Input && Controls.Throttle_Input == 0) // Unarm
                {
                    ARMED = false; // please don't fall out of the sky
                    Motors.Set_Motor_Inputs (0.0, 0.0, 0.0, 0.0); // Just in case
                }
                else // flying and not unarming
                {
                    if (Controls.Stabalize_Input)
                    {
                        // Stabalize!!
                        Throttle_PID.set_Setpoint (Sensor_Data.range);
                        Throttle_PID.compute (Sensor_Data.range);
                        Motors.Set_Motor_Inputs (-Throttle_PID.Drive, Pitch_PID.Drive, Roll_PID.Drive, 0.0); // Remember negative throttle
                    }
                    else // full user control
                        // Possibly change throttle pid here and input throttle pid.drive
                        Motors.Set_Motor_Inputs (Controls.Throttle_Input, Controls.Pitch_Input, Controls.Roll_Input, Controls.Yaw_Input);
                }
            }
            else //!ARMED
            {
                if (Controls.Calibrate_Input)
                {
                    Motors.Set_Motor_Inputs (0.0, 0.0, 0.0, 0.0); // Just in case
                    Sensor_Data.calibrate_sensors (); // blink status led or something?
                }
                else if (Controls.Arm_Input && Controls.Throttle_Input == 0)
                {
                    ARMED = true; // Oh my!
                    // turn on arm led
                    Motors.Set_Motor_Inputs (0.0, 0.0, 0.0, 0.0); // Just in case
                }
            }
        }
        else if (!Controls.Control_Timeout) // !Message_Recieved implied
        {
            if (ARMED)
            {
                if (Controls.Stabalize_Input)
                {
                    // Stabalize!!
                    Throttle_PID.set_Setpoint (Sensor_Data.range);
                    Throttle_PID.compute (Sensor_Data.range);
                    Motors.Set_Motor_Inputs (-Throttle_PID.Drive, Pitch_PID.Drive, Roll_PID.Drive, 0.0); // Remember negative throttle
                }
                else
                {
                    Motors.Set_Motor_Inputs (Controls.Throttle_Input, Controls.Pitch_Input, Controls.Roll_Input, Controls.Yaw_Input);
                }
            }
            else // unarmed
            {
                Motors.Set_Motor_Inputs (0.0, 0.0, 0.0, 0.0); // Just in case
            }
        }
        else // !Message_Recieved and Control_Timeout
        {
            if (ARMED)
            {
                // Stabalize!!
                Throttle_PID.set_Setpoint (Sensor_Data.range);
                Throttle_PID.compute (Sensor_Data.range);
                Motors.Set_Motor_Inputs (-Throttle_PID.Drive, Pitch_PID.Drive, Roll_PID.Drive, 0.0); // Remember negative throttle
            }
            else 
            {
                Motors.Set_Motor_Inputs (0.0, 0.0, 0.0, 0.0); // Do nothing just in case
            }
        }
    }
    else // EMERGENCY!
    {
        Motors.Set_Motor_Inputs (0.0, 0.0, 0.0, 0.0); // Do nothing
    }

    // Run Motors
    Motors.Write_Motor_Out();
}

void Check_Emergency ()
{
    if (EMERGENCY)
    { 
        EMERGENCY = true;
    }
    else
    {
        EMERGENCY = false; // Pass
    }
}
