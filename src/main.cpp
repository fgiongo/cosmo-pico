#include <Arduino.h>
#include <Wire.h>
#include "bno08x.h"
#include "bme688.h"

Adafruit_BNO08x bno08x;
Adafruit_BME680 bme688;
sh2_SensorValue sensor_value;
Bno08x_report inertial_report;
Bme688_report barometer_report;

#define N_ADR 2
#define SDA0 20
#define SCL0 21
#define SDA1 6
#define SCL1 7

int i2c_scan(int* adr, int adr_size, TwoWire* wire);
void bme_print_report(Bme688_report report);
void bno_print_report(Bno08x_report report);

void setup() {
  int error;
  int adr_count, adress;
  int adr_list[N_ADR];

  Wire1.setSDA(SDA1);
  Wire1.setSCL(SCL1);
  Wire1.begin();
  Wire.setSDA(SDA0);
  Wire.setSCL(SCL0);
  Wire.begin();

  Serial.begin(9600);
  while(!Serial);
  delay(100);
  

  // I2C0 scan
  Serial.println("I2C scan: I2C0");
  do {
    adr_count = i2c_scan(adr_list, N_ADR, &Wire);
  } while (adr_count == 0);
  Serial.print("Number of I2C devices found: ");
  Serial.println(adr_count);
  for (int i = 0; i<adr_count; ++i)
  {
    Serial.print("Adress of device ");
    Serial.print(i+1);
    Serial.print(" :");
    Serial.println(adr_list[i], HEX);
  }

  // I2C0 scan
  Serial.println("I2C scan: I2C1");
  do {
    adr_count = i2c_scan(adr_list, N_ADR, &Wire1);
  } while (adr_count == 0);

  Serial.print("Number of I2C devices found: ");
  Serial.println(adr_count);
  for (int i = 0; i<adr_count; ++i)
  {
    Serial.print("Adress of device ");
    Serial.print(i+1);
    Serial.print(" :");
    Serial.println(adr_list[i], HEX);
  }

  bme688 = Adafruit_BME680(&Wire);
  error = bme688_init(&bme688);
  Serial.print("bme866_init() error code: ");
  if (error)
  {
    Serial.println(error, HEX);
  }
  else
  {
    Serial.println("SUCCESS");
  }

  Serial.println("Adafruit BNO08x test.");
  while (!bno08x.begin_I2C(BNO08x_ADRESS, &Wire1))
    ;

  Serial.println("BNO08x found.");

  bno08x_set_reports(bno08x);
}

void loop() {
  while (!bme688_report(&bme688, &barometer_report))
    ;

  if (bno08x.wasReset())
  {
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

  //bno_print_report(inertial_report);
  //bme_print_report(barometer_report);
}

int i2c_scan(int* adr, int adr_size, TwoWire* wire)
{
  int adress, error;
  int n_devices = 0;

  for (adress = 1; adress < 127; adress++)
  {
    wire->beginTransmission(adress);
    error = wire->endTransmission();

    if (error == 0)
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

void bme_print_report(Bme688_report report)
{
  Serial.println("BME688 report:");

  Serial.print("Temperature (C): ");
  Serial.println(report.tempC);

  Serial.print("Pressure (Pa): ");
  Serial.println(report.pressurePa);

  Serial.print("Relative humidity: ");
  Serial.println(report.relHumidity);

  Serial.print("Altitude (M): ");
  Serial.println(report.altitudeM);
}