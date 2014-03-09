#include <Arduino.h>
#include <Wire.h>
#include "Sensors.h"

Sensors::Sensors (int trig_pin_, int echo_pin_)
{
    //Wire.begin();

    for(int i = 0; i < 3; i++) 
    {
        raw_accel_data[i] = raw_mag_data[i] = raw_gyro_data[i] = 1;
        calibrated_accel_data[i] = calibrated_gyro_data[i] = calibrated_mag_data[i] = 1;
    }

    trig_pin = trig_pin_;
    echo_pin = echo_pin_;
}

void Sensors::init_sensors()
{
    pinMode(trig_pin, OUTPUT);
    pinMode(echo_pin, INPUT);

    init_accel();
    init_mag();
    init_gyro();
    range = 0.0f;
}

void Sensors::read_sensors()
{
    read_accel();
    read_mag();
    read_gyro();
    //read_range();
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
    Serial.print ("Grabbing Data!\n");
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
    //Serial.println((unsigned int)data);
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

    //Serial.println((unsigned int)data);
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
    //Serial.println((unsigned int)data);
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
