#include <Arduino.h>
#include <bridgeDriver.h>

#define PWMB 21
#define PWMA 22

bridgeDriver IR2304 = bridgeDriver();

void setup() {

  IR2304.init(PWMA, PWMB, 0, 20000);
  IR2304.setInverting(true, 0);
  IR2304.setDuty(60.0, 0);

}

void loop() {
  // put your main code here, to run repeatedly:
}