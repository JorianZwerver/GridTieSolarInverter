/*
  Inverter firmware code
  Project group 16
  Using TTGO-T7 v1.3 esp-32 microcontroller
*/

//include used libraries
#include <Arduino.h>
#include <CircularBuffer.h>
#include <PID_v1.h>

#define DEBUG 0
#define DEBUG2 0
#define DEBUG_readings 0
#define DEBUG_pid 1
#define DEBUG_freqCalc 0

//define used hardware pins
#define LED_pin LED_BUILTIN
#define netFreqVoltMeas_pin 2
#define netPhaseVoltMeas_pin 4
#define invVoltMeas_pin 4
#define PWMPosOut_pin 12
#define PWMNegOut_pin 32
#define test_pin 4

//setting PWM properties
#define PWMFreq 20000
#define PWMPosOutChannel 0
#define PWMNegOutChannel 1
#define PWMResolution 8

#define measDeadZone 0.5
#define ZPMSamples 200 //zero point samples over which it calculates the frequency

//define variables
int netFreqVoltMeas, netPhaseVoltMeas, invVoltMeas, highestValue, loopTimer;
double netFreq;
bool signal;
//unsigned long to not overflow the buffer
unsigned long currentMillis, zeroPointMillis, timerMillis;

char printBuffer[100];

CircularBuffer<int, 5> voltMeasBuf; //circular buffer of the voltage measurements
CircularBuffer<unsigned long, (ZPMSamples+1)> zeroPointMillisBuf; //circular buffer of the zeroPoits in ms

//Define PID Variables we'll be connecting to
double freqSetPoint, Input, freqOutput;

//Specify the PID links and initial tuning parameters
double PWMPIDp=2, PWMPIDi=5, PWMPIDd=1;
PID PWMPID(&netFreq, &freqOutput, &freqSetPoint, PWMPIDp, PWMPIDi, PWMPIDd, DIRECT);

void setup() {  
  //set pin modes
  pinMode(LED_pin, OUTPUT);
  pinMode(netFreqVoltMeas_pin, INPUT);
  pinMode(netPhaseVoltMeas_pin, INPUT);

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
  netFreqVoltMeas = analogRead(netFreqVoltMeas_pin);
  netPhaseVoltMeas = analogRead(netPhaseVoltMeas_pin);

  #if DEBUG_readings
    sprintf(printBuffer, "netFreqVoltMeas : %d", netFreqVoltMeas);
    Serial.println(printBuffer);
  #endif

  return 0;
}

double calculateFrequency(){
  //calculate the frequency of the net based of the timing between the high points in the measured sinusoidal signal from the net

  if (netFreqVoltMeas > highestValue){
      highestValue = netFreqVoltMeas;
        #if DEBUG_freqCalc
          sprintf(printBuffer, "highestValue : %d", highestValue);
          Serial.println(printBuffer);
        #endif
    }

  if (netFreqVoltMeas < (0+measDeadZone*highestValue) && signal == 1){
    digitalWrite(LED_pin,0);
    zeroPointMillisBuf.push(currentMillis);
    signal = 0;
  } else if (netFreqVoltMeas > highestValue-(measDeadZone*highestValue)){
    digitalWrite(LED_pin,1);
    signal = 1;
  }

  if (zeroPointMillisBuf.isFull()){
    netFreq = ZPMSamples*500.0/(zeroPointMillisBuf.last() - zeroPointMillisBuf.first());
  }

  //only print every 250ms
  if ((currentMillis - timerMillis) > 250){
    sprintf(printBuffer, "Net freq. : %f Hz", double(netFreq));
    Serial.println(printBuffer);
    timerMillis = currentMillis;
  }

  return netFreq;
}

int writePWM(float freqOutput){
  int PWMDutyCycle;

  loopTimer = currentMillis;

  #if DEBUG2
    freqOutput = 50;
  #endif

  PWMDutyCycle = pow(2, PWMResolution)*sin(TWO_PI*freqOutput*loopTimer*0.001);

  #if DEBUG2
    sprintf(printBuffer, "Dutycycle : %d", PWMDutyCycle);
    Serial.println(printBuffer);
  #endif

  //dont write a negative dutycycle
  if (PWMDutyCycle >= 0){
    ledcWrite(PWMPosOutChannel, PWMDutyCycle);
    ledcWrite(PWMNegOutChannel, 0);
  } else if (PWMDutyCycle < 0){
    ledcWrite(PWMNegOutChannel, -PWMDutyCycle);
    ledcWrite(PWMPosOutChannel, 0);
  }
  
  return 0;
}

void loop() {
  //delay cause otherwise arduino breaks
  //delayMicroseconds(30);
  delay(1);

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

  #if DEBUG_pid
    sprintf(printBuffer, "freqOutput : %f", freqOutput);
    //Serial.println(printBuffer);
    freqOutput = freqSetPoint;
  #endif


  //write the PWM for the H-bridge
  writePWM(freqOutput);


}