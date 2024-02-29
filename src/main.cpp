#include <Arduino.h>
#include <Wire.h>
#include <SerialTransfer.h>
#include <time.h>
#include "bno08x.h"
#include "bme688.h"

#define N_ADR 2
#define SDA0 20
#define SCL0 21
#define SDA1 6
#define SCL1 7
#define BNO08X_SERIAL_ID 0
#define BME688_SERIAL_ID 1
#define BUFSIZE 32

Adafruit_BNO08x bno08x;
Adafruit_BME680 bme688;
sh2_SensorValue sensor_value;
Bno08x_report inertial_report;
Bme688_report barometer_report;
unsigned long current_time;
unsigned long last_tick = 0;
String buffer;


String bme688_tostring(Bme688_report &report);
String bno08x_tostring(Bno08x_report& report);
int i2c_scan(int* adr, int adr_size, TwoWire* wire);
void bme_print_report(Bme688_report report);
void bno_print_report(Bno08x_report report);
void serial_flush(Bno08x_report report, int id);
void serial_flush(Bme688_report report, int id);

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
  while (!Serial)
    ;
  Serial1.begin(9600); // Pin 1
  while (!Serial1)
    ;
  delay(1000);

  /*
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
  */

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

  /*
  // Initializing Adafruit BNO08x
  while (!bno08x.begin_I2C(BNO08x_ADRESS, &Wire1))
    ;

  Serial.println("BNO08x found.");

  bno08x_set_reports(bno08x);
  */
}

void loop() {
  current_time = millis();

  /*
  if (bno08x.wasReset())
    bno08x_set_reports(bno08x);

  if (bno08x.getSensorEvent(&sensor_value))
    bno08x_update_report(&sensor_value, &inertial_report);
  */

  if (bme688.remainingReadingMillis() == -1)
  {
    bme688.beginReading();
  }
  else if (bme688.remainingReadingMillis() == 0)
  {
    bme688.endReading();
    bme688_update_report(&bme688, &barometer_report);
    Serial1.print(bme688_tostring(barometer_report));
  }

  if (current_time - last_tick > 100)
  {
    if (Serial1.available() > 7)
    {
      buffer = Serial1.readStringUntil('\n');
      Serial1.println(buffer);
      Serial.println(buffer);
    }

    last_tick = current_time;
    //Serial1.print(bno08x_tostring(inertial_report));
  }
}

String bme688_tostring(Bme688_report& report)
{
  String out;
  out += "BME688,";
  out += report.tempC;
  out += ",";
  out += report.pressurePa;
  out += ",";
  out += report.relHumidity;
  out += "\n";

  return out;
}

String bno08x_tostring(Bno08x_report& report)
{
  String out;
  out += "BNO08x,";

  out += report.accel.x;
  out += ",";
  out += report.accel.y;
  out += ",";
  out += report.accel.z;
  out += ",";

  out += report.gyro.x;
  out += ",";
  out += report.gyro.y;
  out += ",";
  out += report.gyro.z;
  out += ",";

  out += report.linaccel.x;
  out += ",";
  out += report.linaccel.y;
  out += ",";
  out += report.linaccel.z;
  out += ",";

  out += report.magnet.x;
  out += ",";
  out += report.magnet.y;
  out += ",";
  out += report.magnet.z;
  out += "\n";

  return out;
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