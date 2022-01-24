/*
  Inverter firmware code
  Project group 16
  Using TTGO-T7 v1.3 esp-32 microcontroller
*/

//include used libraries
#include <Arduino.h>
#include <CircularBuffer.h>
#include <PID_v1.h>

#define DEBUG 1

//define used hardware pins
#define LED_pin LED_BUILTIN
#define netVoltMeas_pin 2
#define invVoltMeas_pin 4
#define PWMPosOut_pin 12
#define PWMNegOut_pin 32
#define test_pin 4

//setting PWM properties
#define PWMFreq 20000
#define PWMPosOutChannel 0
#define PWMNegOutChannel 1
#define PWMResolution 8

//define variables
int netVoltMeas, invVoltMeas, highestValue, loopTimer;
double netFreq;
bool signal;
//unsigned long to not overflow the buffer
unsigned long currentMillis, zeroPointMillis, timerMillis;

char printBuffer[100];

CircularBuffer<int, 5> voltMeasBuf; //circular buffer of the voltage measurements
CircularBuffer<unsigned long, 6> zeroPointMillisBuf; //circular buffer of the zeroPoits in ms

//Define PID Variables we'll be connecting to
double freqSetPoint, Input, freqOutput;

//Specify the PID links and initial tuning parameters
double PWMPIDp=2, PWMPIDi=5, PWMPIDd=1;
PID PWMPID(&netFreq, &freqOutput, &freqSetPoint, PWMPIDp, PWMPIDi, PWMPIDd, DIRECT);

void setup() {  
  //set pin modes
  pinMode(LED_pin, OUTPUT);
  pinMode(netVoltMeas_pin, INPUT);

  // configure LED PWM functionalitites
  ledcSetup(PWMPosOutChannel, PWMFreq, PWMResolution);
  ledcSetup(PWMNegOutChannel, PWMFreq, PWMResolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(PWMPosOut_pin, PWMPosOutChannel);
  ledcAttachPin(PWMNegOut_pin, PWMNegOutChannel);

  Serial.begin(115200);

  //initialise some variables
  timerMillis  = 0;
  zeroPointMillis = 0;
  signal = 0;

  //turn the PWM PID on
  PWMPID.SetMode(AUTOMATIC);

}

int readSensors(){
  //measure and save the voltage measurements
  netVoltMeas = analogRead(netVoltMeas_pin);
  invVoltMeas = analogRead(invVoltMeas_pin);

  return 0;
}

int calculateFrequency(){
  //calculate the frequency of the net based of the timing between the high points in the measured sinusoidal signal from the net

  if (netVoltMeas > highestValue){
      highestValue = netVoltMeas;
    }

  if (netVoltMeas < (0+0.1*highestValue) && signal == 1){
    digitalWrite(LED_pin,0);
    zeroPointMillisBuf.push(currentMillis);
    signal = 0;
  } else if (netVoltMeas > highestValue-(0.1*highestValue)){
    digitalWrite(LED_pin,1);
    signal = 1;
  }

  if (zeroPointMillisBuf.isFull()){
    netFreq = 5000/(zeroPointMillisBuf.last() - zeroPointMillisBuf.first());
  }

  //only print every 250ms
  if ((currentMillis - timerMillis) > 250){
    sprintf(printBuffer, "Net freq. : %f Hz", netFreq);
    Serial.println(printBuffer);
    timerMillis = currentMillis;
  }

  return netFreq;
}

int writePWM(float freqOutput){
  int PWMDutyCycle;

  loopTimer = currentMillis;

  #if DEBUG
    freqOutput = 50;
  #endif

  PWMDutyCycle = pow(PWMResolution,2)*sin(TWO_PI*freqOutput*loopTimer*0.001);

  //dont write a negative dutycycle
  if (PWMDutyCycle >= 0){
    ledcWrite(PWMPosOutChannel, PWMDutyCycle);
  } else if (PWMDutyCycle < 0){
    ledcWrite(PWMNegOutChannel, -PWMDutyCycle);
  }
  
  return 0;
}

void loop() {
  //delay cause otherwise arduino breakes
  delayMicroseconds(30);

  //update currentmillis on every loop
  currentMillis = millis();
  #if DEBUG
    sprintf(printBuffer, "Currentmillis : %d", currentMillis);
    Serial.println(printBuffer);
  #endif

  //read the sensors every loop
  readSensors();

  //calculate the frequency every loop
  netFreq = calculateFrequency();

  freqSetPoint = netFreq;

  PWMPID.Compute();


  //write the PWM for the H-bridge
  writePWM(freqOutput);

}