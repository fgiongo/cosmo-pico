#include "bno08x.h"

void bno08x_set_reports(Adafruit_BNO08x bno08x)
{
  if (!bno08x.enableReport(SH2_ACCELEROMETER))
  {
    Serial.println("Could not enable acceleration report.");
  }

  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED))
  {
    Serial.println("Could not enable calibrated gyroscope report.");
  }

  if (!bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED))
  {
    Serial.println("Could not enable calibrated magnectic field sensor report.");
  }

  if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION))
  {
    Serial.println("Could not enable linear acceleration report.");
  }
}

void bno08x_update_report(sh2_SensorValue* sensor_value, Bno08x_report* report)
{
  switch (sensor_value->sensorId)
  {
  case SH2_ACCELEROMETER:
    report->accel.x = sensor_value->un.accelerometer.x;
    report->accel.y = sensor_value->un.accelerometer.y;
    report->accel.z = sensor_value->un.accelerometer.z;
    break;

  case SH2_GYROSCOPE_CALIBRATED:
    report->gyro.x = sensor_value->un.gyroscope.x;
    report->gyro.y = sensor_value->un.gyroscope.y;
    report->gyro.z = sensor_value->un.gyroscope.z;
    break;

  case SH2_MAGNETIC_FIELD_CALIBRATED:
    report->magnet.x = sensor_value->un.magneticField.x;
    report->magnet.y = sensor_value->un.magneticField.y;
    report->magnet.z = sensor_value->un.magneticField.z;
    break;

  case SH2_LINEAR_ACCELERATION:
    report->linaccel.x = sensor_value->un.linearAcceleration.x;
    report->linaccel.y = sensor_value->un.linearAcceleration.y;
    report->linaccel.z = sensor_value->un.linearAcceleration.z;
    break;

  default:
    break;
  }
}