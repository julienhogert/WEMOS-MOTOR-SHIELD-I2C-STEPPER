#include <wemos_stepper_jh.h>

WEMOS_STEPPER stepper;
bool aller_retour=1;

void setup() {
 Serial.begin(115200);  // start serial for output
 
 stepper.setup();
 stepper.setResolution(1);
}

void loop() {
  if(aller_retour){
      stepper.setTarget(1000);
      stepper.setSpeed(100);
      aller_retour=0;
  }
  else{
      stepper.setTarget(0);
      stepper.setSpeed(-100);
      aller_retour=1;
  }

 while (stepper.isRunning()){
  delay (100);
  Serial.println(stepper.pos);
  }
}
