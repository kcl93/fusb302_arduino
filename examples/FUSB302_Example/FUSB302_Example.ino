#include <Wire.h>
#include <PD_UFP.h>

#define FUSB302_INT_PIN   12

PD_UFP_c PD_UFP(Wire);

void setup() {
  Wire.begin();
  PD_UFP.init(FUSB302_INT_PIN, PD_POWER_OPTION_MAX_20V);
  
  Serial.begin(9600);
}

void loop() {
  PD_UFP.handle();
  if (PD_UFP.get_ps_status() == STATUS_POWER_TYP)
  {
    Serial.write("PD supply connected\n");
  }
  else
  {
    Serial.write("No PD supply available\n");
  }
}
