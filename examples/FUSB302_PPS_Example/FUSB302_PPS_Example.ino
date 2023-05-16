#include <Wire.h>
#include <PD_UFP.h>

#define FUSB302_INT_PIN   12

PD_UFP_c PD_UFP(Wire);

void setup() {
  Wire.begin();
  PD_UFP.init_PPS(FUSB302_INT_PIN, 8400, 2000);
  
  Serial.begin(9600);
}

void loop() {
  PD_UFP.handle();
  switch(PD_UFP.get_ps_status())
  {
    case STATUS_POWER_PPS:
      Serial.write("PPS trigger succcess\n");
      break;
      
    case STATUS_POWER_TYP:
      Serial.write("Fail to trigger PPS\n");
      break;
      
    default:
      Serial.write("No PD supply available\n");
      break;
  }
}
