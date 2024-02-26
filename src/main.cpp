#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <SerialTransfer.h>
#include "bno08x.h"
#include "bme688.h"

#define N_ADR 2
#define SDA0 20
#define SCL0 21
#define SDA1 6
#define SCL1 7
#define SSTX 17
#define SSRX 37

Adafruit_BNO08x bno08x;
Adafruit_BME680 bme688;
sh2_SensorValue sensor_value;
Bno08x_report inertial_report;
Bme688_report barometer_report;
SoftwareSerial softSerial(SSRX, SSTX);
SerialTransfer serial_transfer;
bool barometer_success;
bool inertial_success;
unsigned long current_time, flush_time;


int i2c_scan(int* adr, int adr_size, TwoWire* wire);
void bme_print_report(Bme688_report report);
void bno_print_report(Bno08x_report report);
void serial_flush(Bno08x_report inertial, Bme688_report barometer);

void setup() {
  int error;
  int adr_count, adress;
  int adr_list[N_ADR];

  // Initializing I2C
  Wire1.setSDA(SDA1);
  Wire1.setSCL(SCL1);
  Wire1.begin();
  Wire.setSDA(SDA0);
  Wire.setSCL(SCL0);
  Wire.begin();

  // Initializing hardware serial
  Serial.begin(9600);
  while(!Serial);
  delay(100);

  // Initializing software serial
  pinMode(SSRX, INPUT);
  pinMode(SSTX, OUTPUT);
  softSerial.begin(9600);
  
  serial_transfer.begin(Serial);

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

  // I2C1 scan
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

  // Initializing Adafruit BME688
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

  // Initializing Adafruit BNO08x
  while (!bno08x.begin_I2C(BNO08x_ADRESS, &Wire1))
    ;

  Serial.println("BNO08x found.");

  bno08x_set_reports(bno08x);
}

void loop() {
  if (bno08x.wasReset())
    bno08x_set_reports(bno08x);

  if (bno08x.getSensorEvent(&sensor_value))
    bno08x_update_report(&sensor_value, &inertial_report);
    //bno_print_report(inertial_report);

  if (bme688.performReading())
    bme688_update_report(&bme688, &barometer_report);
    //bme_print_report(barometer_report);

  current_time = millis();
  if (current_time > flush_time)
  {
    serial_flush(inertial_report, barometer_report);
  }
}

void serial_flush(Bno08x_report inertial, Bme688_report barometer)
{
  uint16_t size = 0;
  size = serial_transfer.txObj(barometer, size);
  serial_transfer.sendData(size, 0);
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
  Serial.println(";\n");
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
  Serial.println("");
}