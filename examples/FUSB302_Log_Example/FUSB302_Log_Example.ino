#include <Wire.h>
#include <PD_UFP.h>

#define FUSB302_INT_PIN   12

PD_UFP_Log_c PD_UFP(Wire);

void setup() {
  Wire.begin();
  PD_UFP.init_PPS(FUSB302_INT_PIN, 8400, 2000);
  
  Serial.begin(9600);
}

void loop() {
  PD_UFP.run();
  PD_UFP.print_status(Serial);
}
