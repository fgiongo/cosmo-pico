#include "bme688.h"

#define SEALEVEL_PRESSURE_HPA 1016.0

int bme688_init(Adafruit_BME680* bme)
{
    int error = 0x0;

    if (!bme->begin())
    {
        return ERROR_BME688_INIT;
    }
    if (!bme->setTemperatureOversampling(BME68X_OS_16X))
    {
        error |= ERROR_BME688_OVERSAMPLING_TEMP;
    }
    if (!bme->setPressureOversampling(BME68X_OS_16X))
    {
        error |= ERROR_BME688_OVERSAMPLING_PRESSURE;
    }
    if (!bme->setHumidityOversampling(BME68X_OS_16X))
    {
        error |= ERROR_BME688_OVERSAMPLING_HUMIDITY;
    }

    return error;

}

bool bme688_report(Adafruit_BME680* bme, Bme688_report* report)
{
    if(!bme->performReading())
    {
        return false;
    }
    report->tempC = bme->temperature;
    report->pressurePa = bme->pressure;
    report->relHumidity = bme->humidity;
    report->altitudeM = bme->readAltitude(SEALEVEL_PRESSURE_HPA);
    return true;
}