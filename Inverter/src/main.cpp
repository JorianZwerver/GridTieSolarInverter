/*
  Inverter code
  Project group 16
  Using TTGO-T7 v1.3 esp-32 microcontroller
*/

//include used libraries
#include <Arduino.h>
#include <CircularBuffer.h>
#include <PID_v1.h>


//define used hardware pins
#define LED_pin 19
#define netVoltMeas_pin 2
#define invVoltMeas_pin 3
#define PWMPosOut_pin 4
#define PWMNegOut_pin 5
#define test_pin 4

// setting PWM properties
#define PWMFreq 20000
#define PWMLedChannel 0
#define PWMResolution 8

//define variables
int netVoltMeas, invVoltMeas, highestValue;
float netFreq;
bool signal;
//unsigned long to not overflow the buffer
unsigned long currentMillis, zeroPointMillis, timerMillis;

CircularBuffer<int, 5> voltMeasBuf; //circular buffer of the voltage measurements
CircularBuffer<unsigned long, 6> zeroPointMillisBuf; //circular buffer of the zeroPoits in ms


void setup() {  
  //set pin modes
  pinMode(LED_pin, OUTPUT);
  pinMode(netVoltMeas_pin, INPUT);

  // configure LED PWM functionalitites
  ledcSetup(PWMLedChannel, PWMFreq, PWMResolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(PWMPosOut_pin, PWMLedChannel);

  Serial.begin(115200);

  //initialise some variables
  timerMillis  = 0;
  zeroPointMillis = 0;
  signal = 0;

}

int readSensors(){
  //measure and safe the voltage measurements
  netVoltMeas = analogRead(netVoltMeas_pin);
  invVoltMeas = analogRead(invVoltMeas_pin);

  return 0;
}

int calculateFrequency(){

  if (netVoltMeas > highestValue){
      highestValue = netVoltMeas;
    }

  if (netVoltMeas < (0+0.1*highestValue) && signal == 1){
    digitalWrite(LED_pin,0);
    zeroPointMillisBuf.push(currentMillis);
    signal = 0;
  }

  if (netVoltMeas > highestValue-(0.1*highestValue)){
    digitalWrite(LED_pin,1);
    signal = 1;
  }

  if (zeroPointMillisBuf.isFull()){
    netFreq = 5000/(zeroPointMillisBuf.last() - zeroPointMillisBuf.first());
  }

  if ((currentMillis - timerMillis) > 250){
    
    Serial.print("Net Frequency : ");
    Serial.print(netFreq);
    Serial.println("Hz");
    timerMillis = currentMillis;
  }

  return netFreq;
}

int writePWM(){
  int PWMDutyCycle;

  PWMDutyCycle = pow(PWMResolution,2)*sin(netFreq*loop);

  //dont write a negative dutycycle
  if (PWMDutyCycle >= 0){
    ledcWrite(PWMLedChannel, PWMDutyCycle);
  } else if (PWMDutyCycle < 0){
    ledcWrite(PWMLedChannel, -PWMDutyCycle);
  }
  
  return 0;
}

void loop() {
  //delay cause otherwise arduino brakes
  delay(1);

  //update currentmillis on every loop
  currentMillis = millis();

  //read the sensors every loop
  readSensors();

  //calculate the frequency every loop
  netFreq = calculateFrequency();

  //

  //write the PWM for the H-bridge
  writePWM(FreqSetPoint, FreqError, PhaseSetPoint);

}