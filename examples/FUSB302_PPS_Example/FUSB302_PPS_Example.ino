#include <Wire.h>
#include <PD_UFP.h>

#define FUSB302_INT_PIN   12

PD_UFP_c PD_UFP(Wire);

void setup() {
  Wire.begin();
  PD_UFP.init_PPS(FUSB302_INT_PIN, PPS_V(8.4), PPS_A(2.0));
  
  Serial.begin(9600);
}

void loop() {
  PD_UFP.run();
  if (PD_UFP.is_PPS_ready())
  {
    Serial.write("PPS trigger succcess\n");
  }
  else if (PD_UFP.is_power_ready())
  {
    Serial.write("Fail to trigger PPS\n");
  }
  else
  {
    Serial.write("No PD supply available\n");
  }
}