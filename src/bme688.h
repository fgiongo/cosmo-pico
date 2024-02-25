#ifndef BME688_H
#define BME688_H

#include <Adafruit_BME680.h>

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

#define ERROR_BME688_INIT 0x1
#define ERROR_BME688_OVERSAMPLING_TEMP 0x2
#define ERROR_BME688_OVERSAMPLING_PRESSURE 0x4
#define ERROR_BME688_OVERSAMPLING_HUMIDITY 0x8

struct Bme688_report
{
    float tempC;
    float pressurePa;
    float relHumidity;
    float altitudeM;
};

int bme688_init(Adafruit_BME680* bme);
bool bme688_report(Adafruit_BME680* bme, Bme688_report* report);

#endif