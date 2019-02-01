#include <wemos_stepper_jh.h>

WEMOS_STEPPER stepper;

void setup() {
 Serial.begin(115200);  // start serial for output
 stepper.setSpeed(100); 
 stepper.setup();
 stepper.setResolution(1);
 stepper.alwaysWait=true;
}

void loop() {
    stepper.go(1000);
    stepper.go(0);
}
