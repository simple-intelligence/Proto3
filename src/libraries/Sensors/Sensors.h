#ifndef SENSORS
#define SENSORS

#include <Arduino.h>

/*
Input ranges from -512 to 512 (10 bits)
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
#define MAG_REGISTER_MEASUREMODE (0x02)
#define MAG_MEASUREMODE_CONT (0x00)

#define TOCM 27.623

class Sensors
{
private:
    char c;
    int echo_pin;
    int trig_pin;
    float range_timer;
    float last_time;

    int16_t calibrated_accel_constants[3];
    int16_t calibrated_gyro_constants[3];
    int16_t calibrated_mag_constants[3];

    int16_t raw_accel_data[3];
    int16_t raw_gyro_data[3];
    int16_t raw_mag_data[3];

public:
    float range;
    float calibrated_accel_data[3];
    float calibrated_gyro_data[3];
    float calibrated_mag_data[3];

    Sensors (int trig_pin, int echo_pin);
    void i2c_write(int address, byte reg, byte data);
    void i2c_read(int address, byte reg, int count, byte* data);
    void init_accel();
    void init_gyro();
    void init_mag();

    void read_accel();
    void read_gyro();
    void read_mag();
    void read_range();

    void init_sensors();
    void read_sensors(int get_range);

    void calibrate_sensors();
    void store_calibration();
    void retrieve_calibration();

    void set_trig_pin (int pin);
    void set_echo_pin (int pin);
};

#endif
