/* Proto Flight Board Version 2 */

#include <Wire.h>
#include <Servo.h>

#include "Sensors.h"
#include "PID.h"
#include "Logger.h"
#include "Controller.h"
#include "Motors.h"
#include "Comp_Filter.h"

/*******************
* Flight Libraries *
*******************/

Sensors Sensor_Data (1, 2); // Trig, Echo

Complementary_Filter Pitch_Comp (0.05, 0.95); // Accel_Multiplier, Gyro_Multiplier
Complementary_Filter Roll_Comp (0.05, 0.95);

PID_Class Pitch_PID (1.0, .1, 0.0, -20.0, 20.0, 0.0); // kP, kI, kD, Min_Integrator, Max_Integrator, Setpoint
PID_Class Roll_PID (1.0, .1, 0.0, -20.0, 20.0, 0.0);
PID_Class Throttle_PID (0.2, 0.2, 0.0, -10.0, 10.0, 0.0);

Controller Controls (30); // Control_Timeout

//Motor_Control Motors (1200, 2000); // Min_PWM, Max_PWM
Motor_Control Motors (800, 2000); // Min_PWM, Max_PWM

Logger Logging (5, 1); // Logging_Rate (cycles/msg), On/Off

/**********
* Globals *
**********/

// Change These
int RANGE_ON = 0;
bool TEST_MOTORS = false;

// Don't Change These
bool ARMED = false;
bool RADIO_BYPASS = false; // Future addition possibly
bool EMERGENCY = false;
int EMERGENCY_COUNTER = 0;
bool CALIBRATED = false;

/*****************
* Initialization *
*****************/

void setup ()
{
    Serial.begin (115200);

    Serial.println ("Initializing SI ProtoCopter!");
    Motors.Init_Motors (3, 4, 5, 6, 7); // FL, FR, BL, BR, Enable_Pin
    Sensor_Data.init_sensors ();
}

void loop ()
{
    Sensor_Data.read_sensors (RANGE_ON); // No range yet
    
    // Raw angles
    float Raw_Pitch_Angle = atan2 (Sensor_Data.calibrated_accel_data[0], Sensor_Data.calibrated_accel_data[2]);
    float Raw_Roll_Angle = atan2 (Sensor_Data.calibrated_accel_data[1], Sensor_Data.calibrated_accel_data[2]);

    // Filter
    Pitch_Comp.Calculate (Sensor_Data.calibrated_gyro_data[1], Raw_Pitch_Angle);
    Roll_Comp.Calculate (Sensor_Data.calibrated_gyro_data[0], Raw_Roll_Angle);

    // PID
    Pitch_PID.compute (Pitch_Comp.angle);
    Roll_PID.compute (Roll_Comp.angle);
    Throttle_PID.compute (Sensor_Data.range);

    // TODO: Write diagram for this
    Controls.Parse_Serial ();
    //Check_Emergency ();
    if (!EMERGENCY)
    {
        if (Controls.Message_Recieved)
        {
            if (ARMED)
            {
                if (!Controls.Arm_Input && !Controls.Throttle_Input) // Unarm
                {
                    ARMED = false; // please don't fall out of the sky
                    Motors.Set_Motor_Inputs (0.0, 0.0, 0.0, 0.0); // Just in case
                    Serial.println ("Unarming!");
                }
                else // flying and not unarming
                {
                    if (Controls.Stabalize_Input)
                    {
                        // Stabalize!!
                        //Serial.println ("Stabalizing!");
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
                    if (!CALIBRATED)
                    {
                        Serial.println ("Calibrating!");
                        Sensor_Data.calibrate_sensors (); // blink status led or something?
                            
                        //if (TEST_MOTORS)
                        //{
                        //    Motors.Motor_Test ();
                        //}
                        CALIBRATED = true;
                    }  
              }
                else if (Controls.Arm_Input && !Controls.Throttle_Input)
                {
                    Serial.println ("ARMING!");
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
            //Serial.println ("No message and Control Timeout!");
            if (ARMED)
            {
                // Stabalize!!
                Throttle_PID.set_Setpoint (Sensor_Data.range);
                Throttle_PID.compute (Sensor_Data.range);
                Motors.Set_Motor_Inputs (-Throttle_PID.Drive, Pitch_PID.Drive, Roll_PID.Drive, 0.0); // Remember negative throttle
                // TODO: put a timer to eventually cut motors or start landing
            }
            else 
            {
                //Serial.println ("Zeroing Motors!");
                Motors.Set_Motor_Inputs (0.0, 0.0, 0.0, 0.0); // Do nothing just in case
            }
        }
    }
    else // EMERGENCY!
    {
        Serial.println ("EMERGENCY!!");
        Motors.Disable_Motors ();
        Motors.Set_Motor_Inputs (0.0, 0.0, 0.0, 0.0); // Do nothing
    }

    // Run Motors
    Motors.Write_Motor_Out();
    
    Log ();
}

void Check_Emergency ()
{
    if (Pitch_Comp.angle > 90.0)
    {
        EMERGENCY_COUNTER += 1;
    }
    if (Roll_Comp.angle > 90.0)
    {
        EMERGENCY_COUNTER += 1;
    }
    if (Pitch_Comp.angle < -90.0)
    {
        EMERGENCY_COUNTER += 1;
    }
    if (Roll_Comp.angle < -90.0)
    {
        EMERGENCY_COUNTER += 1;
    }
    
    if (EMERGENCY_COUNTER > 10)
    {
        EMERGENCY = true;
    }
  
    if (EMERGENCY)
    { 
        EMERGENCY = true;
    }
    else
    {
        EMERGENCY = false; // Pass
    }
}

void Log ()
{
    /************
    * Logging   *
    *************/

    Logging.Count ();
    if (Logging.Log_This_Cycle && Logging.Logger_On)
    {
        //Logging.Log_Int (Sensor_Data.calibrated_accel_data[0]);
        //Logging.Log_Int (Sensor_Data.calibrated_accel_data[1]);
        //Logging.Log_Int (Sensor_Data.calibrated_accel_data[2]);

        //Logging.Log_Float (Raw_Pitch_Angle);
        //Logging.Log_Float (Raw_Roll_Angle);
        
        //Logging.Log_Float (Pitch_Kalman.x1);
        //Logging.Log_Float (Roll_Kalman.x1);
        
        Serial.print ("Pitch/Roll: ");
        Logging.Log_Float (Pitch_Comp.angle);
        Logging.Log_Float (Roll_Comp.angle);
        
        Serial.print ("Range: ");
        Logging.Log_Float (Sensor_Data.range);
    
        //Logging.Log_Int (Sensor_Data.calibrated_gyro_data[0]);
        //Logging.Log_Int (Sensor_Data.calibrated_gyro_data[1]);
        //Logging.Log_Int (Sensor_Data.calibrated_gyro_data[2]);

        Serial.print ("PID: ");
        Logging.Log_Float (-Throttle_PID.Drive);
        Logging.Log_Float (Pitch_PID.Drive);
        Logging.Log_Float (Roll_PID.Drive);
        
        Serial.print ("Motor Outputs: ");
        Logging.Log_Int (Motors.Front_Left_Output_Int);
        Logging.Log_Int (Motors.Front_Right_Output_Int);
        Logging.Log_Int (Motors.Back_Left_Output_Int);
        Logging.Log_Int (Motors.Back_Right_Output_Int);
        
        Logging.End_Line ();
    
        Serial.print ("Controls: ");
        Logging.Log_Int (Controls.Pitch_Input);
        Logging.Log_Int (Controls.Yaw_Input);
        Logging.Log_Int (Controls.Roll_Input);
        Logging.Log_Int (Controls.Throttle_Input);

        Logging.Log_Int (Controls.Arm_Input);
        Logging.Log_Int (Controls.Stabalize_Input);
        Logging.Log_Int (Controls.Calibrate_Input);
        
        Logging.End_Line ();
    }
}

/*
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
*/
