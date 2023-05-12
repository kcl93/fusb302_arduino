#include <Wire.h>
#include <PD_UFP.h>

#define FUSB302_INT_PIN   12

PD_UFP_Log_c PD_UFP;

void setup() {
  Wire.begin();
  PD_UFP.init_PPS(FUSB302_INT_PIN, PPS_V(8.4), PPS_A(2.0));
  
  Serial.begin(9600);
}

void loop() {
  PD_UFP.run();
  PD_UFP.print_status(Serial);
}
