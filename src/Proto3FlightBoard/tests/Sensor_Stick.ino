#include <Wire.h>

/*
Accel input ranges from -512 to 512 (10 bits)
*/

#define ACCEL_ADDRESS (0xA6 >> 1)
#define ACCEL_REGISTER_XLSB (0x32)
#define ACCEL_REGISTER_PWRCTL (0x2D)
#define ACCEL_PWRCTL_MEASURE (1 << 3)

#define GYRO_ADDRESS (0xD0 >> 1)
#define GYRO_REGISTER_XMSB (0x1D)
#define GYRO_REGISTER_DLPF_FS (0x16)
#define GYRO_FULLSCALE (0x03 << 3)
#define GYRO_42HZ (0x03)

#define MAG_ADDRESS (0x3C >> 1)
#define MAG_REGISTER_XMSB (0x03)
#define MAG_REGISTER_MEASMODE (0x02)
#define MAG_MEASMODE_CONT (0x00)

int16_t accelerometer_data[3];
int16_t gyro_data[3];
int16_t magnetometer_data[3];

char c;

void setup()
{
    Wire.begin();
    Serial.begin(115200);

    for(int i = 0; i < 3; ++i) 
    {
        accelerometer_data[i] = magnetometer_data[i] = gyro_data[i] = 0;
    }

    init_accel();
    init_mag();
    init_gyro();
}

void loop() 
{
    read_accel();

    Serial.print(accelerometer_data[0]);
    Serial.print(",");
    Serial.print(accelerometer_data[1]);
    Serial.print(",");
    Serial.print(accelerometer_data[2]);
    Serial.print(":");

    read_mag();
    
    Serial.print(magnetometer_data[0]);
    Serial.print(",");
    Serial.print(magnetometer_data[1]);
    Serial.print(",");
    Serial.print(magnetometer_data[2]);
    Serial.print(":");

    read_gyro();
   
    Serial.print(gyro_data[0]);
    Serial.print(",");
    Serial.print(gyro_data[1]);
    Serial.print(",");
    Serial.print(gyro_data[2]);
    Serial.print("\n");

    delay(10);
}

void i2c_write(int address, byte reg, byte data) 
{
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}

void i2c_read(int address, byte reg, int count, byte* data) 
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

void init_accel() 
{
    byte data = 0;

    i2c_write(ACCEL_ADDRESS, ACCEL_REGISTER_PWRCTL, ACCEL_PWRCTL_MEASURE);

    i2c_read(ACCEL_ADDRESS, ACCEL_REGISTER_PWRCTL, 1, &data);
    //Serial.println((unsigned int)data);
}

void read_accel() 
{
    byte bytes[6];
    memset(bytes,0,6);

    i2c_read(ACCEL_ADDRESS, ACCEL_REGISTER_XLSB, 6, bytes);

    for (int i=0;i<3;++i) 
    {
        accelerometer_data[i] = (int16_t)bytes[2*i] + (((int16_t)bytes[2*i + 1]) << 8);
    }
}

void init_gyro() 
{
    byte data = 0;

    i2c_write(GYRO_ADDRESS, GYRO_REGISTER_DLPF_FS, GYRO_FULLSCALE | GYRO_42HZ);

    i2c_read(GYRO_ADDRESS, GYRO_REGISTER_DLPF_FS, 1, &data);

    //Serial.println((unsigned int)data);
}

void read_gyro() 
    {
    byte bytes[6];
    memset(bytes,0,6);

    i2c_read(GYRO_ADDRESS, GYRO_REGISTER_XMSB, 6, bytes);
    for (int i=0;i<3;++i) 
    {
        gyro_data[i] = (int16_t)bytes[2*i + 1] + (((int16_t)bytes[2*i]) << 8);
    }
}

void init_mag() 
{
    byte data = 0;

    i2c_write(MAG_ADDRESS, MAG_REGISTER_MEASMODE, MAG_MEASMODE_CONT);

    i2c_read(MAG_ADDRESS, MAG_REGISTER_MEASMODE, 1, &data);
    //Serial.println((unsigned int)data);
}

void read_mag() 
{
    byte bytes[6];
    memset(bytes,0,6);

    i2c_read(MAG_ADDRESS, MAG_REGISTER_XMSB, 6, bytes);

    for (int i=0;i<3;++i) 
    {
        magnetometer_data[i] = (int16_t)bytes[2*i + 1] + (((int16_t)bytes[2*i]) << 8);
    }
}


