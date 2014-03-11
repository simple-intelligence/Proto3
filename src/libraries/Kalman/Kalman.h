#ifndef KALMAN
#define KALMAN

#include <Arduino.h>

class Kalman
{
private:
    float p11;
    float p12;
    float p13;
    float p21;
    float p22;
    float p23;
    float p31;
    float p32;
    float p33;

    float p11_temp;
    float p12_temp;
    float p13_temp;
    float p21_temp;
    float p22_temp;
    float p23_temp;
    float p31_temp;
    float p32_temp;
    float p33_temp;

    float y1;
    float y2;

    float a;
    float b;
    float c;

    float sDet;

    float s11;
    float s12;
    float s21;
    float s22;

    float k11;
    float k12;
    float k21;
    float k22;
    float k31;
    float k32;

    float q1;
    float q2;
    float q3;

    float r1;
    float r2;
    
    float DT;

public:
    float x1;    
    float x2;    
    float x3;    

    Kalman (float dt);
    void compute (float z1, float z2);
};

#endif
