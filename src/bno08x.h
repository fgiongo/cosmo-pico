#ifndef BNO08x_H
#define BNO08x_H

#include <Adafruit_BNO08x.h>

#define BNO08x_ADRESS 0x4a

struct Vec3d
{
    float x;
    float y;
    float z;
} __attribute__((packed));

struct Bno08x_report
{
    Vec3d accel;
    Vec3d gyro;
    Vec3d magnet;
    Vec3d linaccel;
} __attribute__((packed));

void bno08x_set_reports(Adafruit_BNO08x bno08x);
void bno08x_update_report(sh2_SensorValue* sensor_value, Bno08x_report* report);

#endif