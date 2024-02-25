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