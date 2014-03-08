#ifndef KALMAN
#define KALMAN

#include <Arduino.h>

#define DT 0.01f

// Tuning Variables
#define BROWNYODA 1
#define Q1
#define Q2
#define Q3

#define R1
#define R2

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

public:
    float x1;    
    float x2;    
    float x3;    

    Kalman ();
    void kalman_compute (float z1, float z2);
};

#endif
