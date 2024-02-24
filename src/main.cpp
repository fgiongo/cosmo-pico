#include <Arduino.h>
#include <Wire.h>
#include "bno08x.h"

Adafruit_BNO08x bno08x;
sh2_SensorValue sensor_value;
Bno08x_report inertial_report;

void bno_print_report(Bno08x_report report);

void setup() {
  int adr_count, adress;
  int adr_list[N_ADR];

  Wire1.setSDA(SDA1_PIN);
  Wire1.setSCL(SCL1_PIN);
  Wire1.begin();

  Serial.begin(9600);
  while(!Serial);

  Serial.println("Adafruit BNO08x test.");
  while (!bno08x.begin_I2C(BNO08x_ADRESS, &Wire1))
    ;

  Serial.println("BNO08x found.");

  bno08x_set_reports(bno08x);
}

void loop() {
  delay(100);

  if (bno08x.wasReset())
  {
    Serial.println("Sensor was reset.");
    bno08x_set_reports(bno08x);
  }

  if (!bno08x.getSensorEvent(&sensor_value))
  {
    return;
  }

  switch (sensor_value.sensorId)
  {
  case SH2_ACCELEROMETER:
    inertial_report.accel.x = sensor_value.un.accelerometer.x;
    inertial_report.accel.y = sensor_value.un.accelerometer.y;
    inertial_report.accel.z = sensor_value.un.accelerometer.z;
    break;

  case SH2_GYROSCOPE_CALIBRATED:
    inertial_report.gyro.x = sensor_value.un.gyroscope.x;
    inertial_report.gyro.y = sensor_value.un.gyroscope.y;
    inertial_report.gyro.z = sensor_value.un.gyroscope.z;
    break;

  case SH2_MAGNETIC_FIELD_CALIBRATED:
    inertial_report.magnet.x = sensor_value.un.magneticField.x;
    inertial_report.magnet.y = sensor_value.un.magneticField.y;
    inertial_report.magnet.z = sensor_value.un.magneticField.z;
    break;

  case SH2_LINEAR_ACCELERATION:
    inertial_report.linaccel.x = sensor_value.un.linearAcceleration.x;
    inertial_report.linaccel.y = sensor_value.un.linearAcceleration.y;
    inertial_report.linaccel.z = sensor_value.un.linearAcceleration.z;
    break;

  default:
    break;
  }

  bno_print_report(inertial_report);
}

int i2c_scan(int* adr, int adr_size)
{
  int adress, error;
  int n_devices = 0;

  for (adress = 1; adress < 127; adress++)
  {
    Wire1.beginTransmission(adress);
    error = Wire1.endTransmission();

    if (error == 0  )
    {
      if (adr && n_devices < adr_size)
        adr[n_devices] = adress;
      n_devices++;
    }
    else if (error == 4)
    {
      if (adress < 16)
      return -1;
    }
  }

  return n_devices;
}

void bno_print_report(Bno08x_report report)
{
  Serial.println("BNO08x report:");
  Serial.println("Acceleration:");
  Serial.print("x: ");
  Serial.print(report.accel.x);
  Serial.print("; y: ");
  Serial.print(report.accel.y);
  Serial.print("; z: ");
  Serial.print(report.accel.z);
  Serial.println(";");

  Serial.println("Gyro:");
  Serial.print("x: ");
  Serial.print(report.gyro.x);
  Serial.print("; y: ");
  Serial.print(report.gyro.y);
  Serial.print("; z: ");
  Serial.print(report.gyro.z);
  Serial.println(";");

  Serial.println("Magnetic Field Strength:");
  Serial.print("x: ");
  Serial.print(report.magnet.x);
  Serial.print("; y: ");
  Serial.print(report.magnet.y);
  Serial.print("; z: ");
  Serial.print(report.magnet.z);
  Serial.println(";");

  Serial.println("Linear Acceleration:");
  Serial.print("x: ");
  Serial.print(report.linaccel.x);
  Serial.print("; y: ");
  Serial.print(report.linaccel.y);
  Serial.print("; z: ");
  Serial.print(report.linaccel.z);
  Serial.println(";");
}