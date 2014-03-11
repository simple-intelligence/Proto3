#include <Arduino.h>
#include "Kalman.h"

Kalman::Kalman (float dt)
{
    DT = dt;

    x1 = 0.0;
    x2 = 0.0;
    x3 = 0.0;

    p11 = 1000.0;
    p12 = 0.0;
    p13 = 0.0;
    p21 = 0.0;
    p22 = 1000.0;
    p23 = 0.0;
    p31 = 0.0;
    p32 = 0.0;
    p33 = 1000.0;

    p11_temp = 1000.0;
    p12_temp = 0.0;
    p13_temp = 0.0;
    p21_temp = 0.0;
    p22_temp = 1000.0;
    p23_temp = 0.0;
    p31_temp = 0.0;
    p32_temp = 0.0;
    p33_temp = 1000.0;

    y1 = 0.0;
    y2 = 0.0;

    a = 0.0;
    b = 0.0;
    c = 0.0;

    sDet = 0.0;

    s11 = 0.00;
    s12 = 0.00;
    s21 = 0.00;
    s22 = 0.00;

    k11 = 0.0;
    k12 = 0.0;
    k21 = 0.0;
    k22 = 0.0;
    k31 = 0.0;
    k32 = 0.0;

    q1 = 5.0;
    q2 = 100.0;
    q3 = 0.01;

    r1 = 1000.0;
    r2 = 1000.0;
}

void Kalman::compute (float z1, float z2)
{
    // Step 1
    x1 = x1 + DT*x2 - DT*x3;

    // Step 2
    a = p11_temp + DT*p21 - DT*p31;
    b = p12_temp + DT*p22 - DT*p32;
    c = p13_temp + DT*p23 - DT*p33;
    p11 = a + DT*b - DT*c + q1;
    p12 = b;
    p13 = c;

    p21 = p21_temp + DT*p22 - DT*p23;
    p22 = p22_temp + q2;

    p31 = p31_temp + DT*p32 - DT*p33;
    p33 = p33_temp + q3;

    // Step 3
    y1 = z1 - x1;
    y2 = z2 - x2;

    // Step 4
    s11 = p11 + r1;
    s12 = p12; 
    s21 = p21; 
    s22 = p22 + r2;

    // Step 5
    sDet = 1.0/((s11*s22) - (s12*s21));
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
    p11_temp = p11;
    p12_temp = p12;
    p13_temp = p13;
    p21_temp = p21;
    p22_temp = p22;
    p23_temp = p23;
    p31_temp = p31;
    p32_temp = p32;
    p33_temp = p33;

    p11 = p11_temp * (1.0 - k11) - p21_temp*k12;
    p12 = p12_temp * (1.0 - k11) - p22_temp*k12;
    p13 = p13_temp * (1.0 - k11) - p23_temp*k12;
    p21 = p21_temp * (1.0 - k22) - p11_temp*k21;
    p22 = p22_temp * (1.0 - k22) - p12_temp*k21;
    p23 = p23_temp * (1.0 - k22) - p13_temp*k21;

    p31 = p31_temp - p21_temp*k32 - p11_temp*k31;
    p32 = p32_temp - p22_temp*k32 - p12_temp*k31;
    p33 = p33_temp - p23_temp*k32 - p13_temp*k31;
}
