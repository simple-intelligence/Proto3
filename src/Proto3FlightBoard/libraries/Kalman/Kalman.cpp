#include "Kalman.h"
#include <Arduino.h>

Kalman::Kalman ()
{
    float x1 = 0.0f;
    float x2 = 0.0f;
    float x3 = 0.0f;

    float p11 = 1000.0f;
    float p12 = 0.0f;
    float p13 = 0.0f;
    float p21 = 0.0f;
    float p22 = 1000.0f;
    float p23 = 0.0f;
    float p31 = 0.0f;
    float p32 = 0.0f;
    float p33 = 1000.0f;

    float y1 = 0.0f;
    float y2 = 0.0f;

    float a = 0.0f;
    float b = 0.0f;
    float c = 0.0f;

    float sDet = 0.0f;

    float s11 = 0.0f;
    float s12 = 0.0f;
    float s21 = 0.0f;
    float s22 = 0.0f;

    float k11 = 0.0f;
    float k12 = 0.0f;
    float k21 = 0.0f;
    float k22 = 0.0f;
    float k31 = 0.0f;
    float k32 = 0.0f;

    float q1 = Q1;
    float q2 = Q2;
    float q3 = Q3;

    float r1 = R1;
    float r2 = R2;
}

void Kalman::kalman_compute (float z1, float z2)
{
    // Step 1
    x1 = x1 + DT*x2 - DT*x3;

    // Step 2
    a = p11 + DT*p21 - DT*p31;
    b = p12 + DT*p22 - DT*p32;
    c = p13 + DT*p23 - DT*p33;
    p11 = a + DT*b - DT*c + q1;
    p12 = b;
    p13 = c;
    p21 = p21 + DT*p22 - DT*p23;
    p31 = p31 + DT*p32 - DT*p33;
    
    // Step 3
    y1 = z1 - x1;
    y2 = z2 - x2;

    // Step 4
    s11 = p11 + r1;
    s12 = p12; 
    s12 = p21; 
    s12 = p22 + r2;

    // Step 5
    sDet = 1/((s11 * s22) - (s12*s21));
    k11 = ((p11 * s22) - (p12 * s21)) * sDet;
    k12 = ((p12 * s11) - (p11 * s12)) * sDet;
    k21 = ((p21 * s22) - (p22 * s21)) * sDet;
    k22 = ((p22 * s11) - (p21 * s12)) * sDet;
    k31 = ((p31 * s22) - (p32 * s21)) * sDet;
    k32 = ((p32 * s11) - (p31 * s12)) * sDet;

    // Step 6
    x1 = x1 + k11*y1 + k12*y2;
    x2 = x2 + k21*y1 + k22*y2;
    x3 = x3 + k31*y1 + k32*y2;

    // Step 7
    p11 = p11 * (1.0 - k11) - p21*k12;
    p12 = p12 * (1.0 - k11) - p22*k12;
    p13 = p13 * (1.0 - k11) - p23*k12;
    p21 = p21 * (1.0 - k22) - p11*k21;
    p22 = p22 * (1.0 - k22) - p12*k21;
    p23 = p23 * (1.0 - k22) - p13*k21;

    p31 = p31 - p21*k32 - p11*k31;
    p32 = p32 - p22*k32 - p12*k31;
    p33 = p33 - p23*k32 - p13*k31;
}
