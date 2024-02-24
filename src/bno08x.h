#ifndef BNO08x_H
#define BNO08x_H

#include <Adafruit_BNO08x.h>

#define BNO08x_ADRESS 0x4a
#define SDA1_PIN 6
#define SCL1_PIN 7
#define N_ADR 2

struct Vec3d
{
    float x;
    float y;
    float z;
};

struct Bno08x_report
{
    Vec3d accel;
    Vec3d gyro;
    Vec3d magnet;
    Vec3d linaccel;
};

void bno08x_set_reports(Adafruit_BNO08x bno08x);

#endif