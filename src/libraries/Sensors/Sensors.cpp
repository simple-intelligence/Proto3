#include <Arduino.h>
#include <Wire.h>

#include "Sensors.h"

Sensors::Sensors (int trig_pin_, int echo_pin_)
{
    trig_pin = trig_pin_;
    echo_pin = echo_pin_;

    for(int i = 0; i < 3; i++) 
    {
        raw_accel_data[i] = raw_mag_data[i] = raw_gyro_data[i] = 0;
        calibrated_accel_constants[i] = calibrated_gyro_constants[i] = calibrated_mag_constants[i] = 0;
        calibrated_accel_data[i] = calibrated_gyro_data[i] = calibrated_mag_data[i] = 0.0;
    }

    range_timer = 0;
    last_time = 0;
    range = 0.0;
}

void Sensors::init_sensors()
{
    Wire.begin ();
    pinMode(trig_pin, OUTPUT);
    pinMode(echo_pin, INPUT);

    init_accel();
    init_mag();
    init_gyro();
}

void Sensors::read_sensors(int _get_range)
{
    read_accel();
    read_mag();
    read_gyro();

    if (_get_range == 1)
    {
        read_range();
    }

    for (int i = 0; i < 3; i++)
    { 
        calibrated_accel_data[i] = (float)(raw_accel_data[i] - calibrated_accel_constants[i]);
        calibrated_gyro_data[i] = (float)((raw_gyro_data[i] - calibrated_gyro_constants[i]) / 14.375);
        calibrated_mag_data[i] = (float)(raw_mag_data[i] - calibrated_mag_constants[i]);
    }
    
    calibrated_accel_data[2] = -calibrated_accel_data[2];
}

void Sensors::i2c_write(int address, byte reg, byte data) 
{
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}

void Sensors::i2c_read(int address, byte reg, int count, byte* data) 
{
    int i = 0;

    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.beginTransmission(address);
    Wire.requestFrom(address,count);
    while(Wire.available())
    {
        c = Wire.read();
        data[i] = c;
        i++;
    }
    Wire.endTransmission();
} 

void Sensors::init_accel() 
{
    byte data = 0;

    i2c_write(ACCEL_ADDRESS, ACCEL_REGISTER_PWRCTL, ACCEL_PWRCTL_MEASURE);

    i2c_read(ACCEL_ADDRESS, ACCEL_REGISTER_PWRCTL, 1, &data);
}

void Sensors::read_accel() 
{
    byte bytes[6];
    memset(bytes,0,6);

    i2c_read(ACCEL_ADDRESS, ACCEL_REGISTER_XLSB, 6, bytes);

    for (int i=0;i<3;++i) 
    {
        raw_accel_data[i] = (int16_t)bytes[2*i] + (((int16_t)bytes[2*i + 1]) << 8);
    }
}

void Sensors::init_gyro() 
{
    byte data = 0;

    i2c_write(GYRO_ADDRESS, GYRO_REGISTER_DLPF_FS, GYRO_FULLSCALE | GYRO_42HZ);

    i2c_read(GYRO_ADDRESS, GYRO_REGISTER_DLPF_FS, 1, &data);

}

void Sensors::read_gyro() 
{
    byte bytes[6];
    memset(bytes,0,6);

    i2c_read(GYRO_ADDRESS, GYRO_REGISTER_XMSB, 6, bytes);
    for (int i=0;i<3;++i) 
    {
        raw_gyro_data[i] = (int16_t)bytes[2*i + 1] + (((int16_t)bytes[2*i]) << 8);
    }
}

void Sensors::init_mag() 
{
    byte data = 0;

    i2c_write(MAG_ADDRESS, MAG_REGISTER_MEASUREMODE, MAG_MEASUREMODE_CONT);

    i2c_read(MAG_ADDRESS, MAG_REGISTER_MEASUREMODE, 1, &data);
}

void Sensors::read_mag() 
{
    byte bytes[6];
    memset(bytes,0,6);

    i2c_read(MAG_ADDRESS, MAG_REGISTER_XMSB, 6, bytes);

    for (int i=0;i<3;++i) 
    {
        raw_mag_data[i] = (int16_t)bytes[2*i + 1] + (((int16_t)bytes[2*i]) << 8);
    }
}

void Sensors::read_range()
{
    int duration;

    last_time = millis();

    if ((last_time - range_timer) > 60)
    {

        digitalWrite(trig_pin, LOW);
        delayMicroseconds(2);
        digitalWrite(trig_pin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trig_pin, LOW);

        duration = pulseIn(echo_pin, HIGH);
        range = duration / TOCM;

        range_timer = last_time;
    }
}

void Sensors::set_trig_pin (int pin)
{
    trig_pin = pin;
}

void Sensors::set_echo_pin (int pin)
{
    echo_pin = pin;
}

void Sensors::calibrate_sensors()
{
    int i = 0;
    float gyro_avg[3];

    // Gyro
    gyro_avg[0] = 0.0;
    gyro_avg[1] = 0.0;
    gyro_avg[2] = 0.0;

    for (i = 0; i < 500; i++)
    {
        read_sensors (0);
        gyro_avg[0] += ((raw_gyro_data[0] - gyro_avg[0]) / (float)(i + 1));
        gyro_avg[1] += ((raw_gyro_data[1] - gyro_avg[1]) / (float)(i + 1));
        gyro_avg[2] += ((raw_gyro_data[2] - gyro_avg[2]) / (float)(i + 1));
        delay (5);
    }

    calibrated_gyro_constants[0] = gyro_avg[0];
    calibrated_gyro_constants[1] = gyro_avg[1];
    calibrated_gyro_constants[2] = gyro_avg[2];
}
