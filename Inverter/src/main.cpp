/*
  Inverter firmware code
  Project group 16
  Using TTGO-T7 v1.3 esp-32 microcontroller
*/

//include used libraries
#include <Arduino.h>
#include <CircularBuffer.h>
#include <PID_v1.h>
#include <driver/mcpwm.h>
#include <bridgeDriver.h>

//define mcpwm sauce
bridgeDriver IR2304 = bridgeDriver();

//define debug statements
#define DEBUG 0
#define DEBUG_pwm 1
#define DEBUG_readings 0
#define DEBUG_pid 1
#define DEBUG_freqCalc 0
#define DEBUG_dc 0
#define DEBUG_phase 1

//define used hardware pins
#define LED_pin LED_BUILTIN
#define netFreqMeas_pin 15
#define netPhaseMeas_pin 16
#define invVoltMeas_pin 4
#define PWMPosOut_pin 12
#define PWMNegOut_pin 32
#define test_pin 15
#define PWMB 21
#define PWMA 22
#define safetyRelais1_pin 23
#define safetyRelais2_pin 24
#define bigRedButton_pin 25

/*
//setting PWM properties
#define PWMFreq 20000
#define PWMPosOutChannel 0
#define PWMNegOutChannel 1
#define PWMResolution 8
*/

#define ZPMSamples 200 //zero point samples over which it calculates the frequency
#define PPMSamples 10 //peak point measuremets samples

//define safety margins
#define maxPhaseDiff 20

//define variables
int invVoltMeas;
double netFreq, phaseDiff, phaseOffset;
bool signal, netFreqMeas, prevNetFreqMeas, netPhaseMeas, prevNetPhaseMeas;
//unsigned long to not overflow the buffer
unsigned long currentMillis, currentMicros, zeroPointMicros, timerMillis, timer2Millis, startZeroPoint, endZeroPoint, loopTimer;
unsigned long peakPointMicros, outputPeakPointMicros, startPeakPoint, endPeakPoint;

char printBuffer[100];

CircularBuffer<int, 5> voltMeasBuf; //circular buffer of the voltage measurements
CircularBuffer<unsigned long, (ZPMSamples+1)> zeroPointMicrosBuf; //circular buffer of the zeroPoints in microSec
CircularBuffer<unsigned long, (PPMSamples+1)> peakPointMicrosBuf;

//Define PID Variables we'll be connecting to
double freqSetPoint, phaseSetPoint, Input, freqOutput;

//Specify the PID links and initial tuning parameters
double PWMPIDp=2, PWMPIDi=5, PWMPIDd=1;
double phasePIDp=2, phasePIDi=5, phasePIDd=1;
PID PWMPID(&netFreq, &freqOutput, &freqSetPoint, PWMPIDp, PWMPIDi, PWMPIDd, DIRECT);
PID phasePID(&phaseDiff, &phaseOffset, &phaseSetPoint, phasePIDd, phasePIDi, phasePIDd, DIRECT);

void setup() {  
  //set pin modes
  pinMode(LED_pin, OUTPUT);
  pinMode(safetyRelais1_pin, OUTPUT);
  pinMode(safetyRelais2_pin, OUTPUT);
  pinMode(netFreqMeas_pin, INPUT);
  pinMode(netPhaseMeas_pin, INPUT);

/*
  // configure LED PWM functionalitites
  ledcSetup(PWMPosOutChannel, PWMFreq, PWMResolution);
  ledcSetup(PWMNegOutChannel, PWMFreq, PWMResolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(PWMPosOut_pin, PWMPosOutChannel);
  ledcAttachPin(PWMNegOut_pin, PWMNegOutChannel);
*/
  Serial.begin(115200);

  //initialise some variables
  timerMillis  = 0;
  zeroPointMicros = 0;
  signal = 0;
  prevNetFreqMeas = 0;

  //turn the PWM PID on
  PWMPID.SetMode(AUTOMATIC);
  phasePID.SetMode(AUTOMATIC);

  //config the mcpwm
  IR2304.init(PWMA, PWMB, 0, 20000);
  IR2304.setInverting(true, 0);

}

int readSensors(){
  //measure and save the voltage measurements
  netFreqMeas = digitalRead(netFreqMeas_pin);
  netPhaseMeas = digitalRead(netPhaseMeas_pin);

  #if DEBUG_readings
    sprintf(printBuffer, "netFreqMeas : %d", netFreqMeas);
    //Serial.println(printBuffer);
    Serial.println(digitalRead(test_pin));
  #endif

  return 0;
}

//calculate the frequency of the net based of the timing between the high points in the measured sinusoidal signal from the net
double calculateFrequency(){
  // find the high point and save the timestamp
  if (netFreqMeas && !prevNetFreqMeas){
    digitalWrite(LED_pin,0);
    startZeroPoint = currentMicros;
    prevNetFreqMeas = 1;
  } else if (!netFreqMeas && prevNetFreqMeas){
    digitalWrite(LED_pin,1);
    endZeroPoint = currentMicros;
    zeroPointMicrosBuf.push(startZeroPoint+(endZeroPoint-startZeroPoint)/2);
    prevNetFreqMeas = 0;
  }

  //calculate the freq when there are enough samples
  if (zeroPointMicrosBuf.isFull()){
    netFreq = (0.5*ZPMSamples*1000000.0)/(zeroPointMicrosBuf.last() - zeroPointMicrosBuf.first());
  }

  //only print every 250ms
  if ((currentMillis - timerMillis) > 250){
    sprintf(printBuffer, "Net freq. : %f Hz", double(netFreq));
    Serial.println(printBuffer);
    timerMillis = currentMillis;
  }

  return netFreq;
}

//calculate phase difference between measurement of the net and outputted sin signal
int measurePhaseDiff(){
  //find the peaks of the sin by checking the measurements for the optocouplesrs and save the timestamp
  if (netPhaseMeas && !prevNetPhaseMeas){
    startPeakPoint = currentMicros;
    prevNetPhaseMeas = 1;
  } else if (!netPhaseMeas && prevNetPhaseMeas){
    endPeakPoint = currentMicros;
    peakPointMicros = (startPeakPoint+(endPeakPoint-startPeakPoint)/2);
    prevNetPhaseMeas = 0;
  }

  //10ms is 180degrees out of sync
  double halfWaveTime = (1000000.0/netFreq)*0.5;

  //difference between the peaks of the measured signal and produced sin for the pwm
  int microsDiff = (peakPointMicros-outputPeakPointMicros);

  //remove whole wave differences
  double msPhaseDiff = fmod(double(microsDiff), halfWaveTime);

  //phase differenc ein degrees 
  phaseDiff = msPhaseDiff*(180/halfWaveTime);

  #if DEBUG_phase
    //only print every 250ms
    if ((currentMillis - timer2Millis) > 250){
      sprintf(printBuffer, "phaseDiff : %0.3f Degrees, microsdiff : %d micS", phaseDiff, microsDiff);
      Serial.println(printBuffer);
      timer2Millis = currentMillis;
    }
  #endif

  return phaseDiff;
}

int writePWM(float freqOutput){
  int PWMDutyCycle;

  loopTimer = currentMillis;

  #if DEBUG_pwm
    freqOutput = 50.0;
    phaseOffset = 0;
  #endif

  //PWMDutyCycle = 100*sin((TWO_PI*freqOutput*loopTimer*0.001)+phaseOffset);
  PWMDutyCycle = 100*sin((TWO_PI*freqOutput*loopTimer*0.001));


  if (PWMDutyCycle == 100){
    outputPeakPointMicros = currentMicros;
  }

  #if DEBUG_dc
    sprintf(printBuffer, "Dutycycle : %d", PWMDutyCycle);
    Serial.println(printBuffer);
  #endif

  //dont write a negative dutycycle
  if (PWMDutyCycle >= 0){
    IR2304.setDuty(PWMDutyCycle, 0);
    //mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, PWMDutyCycle);
    //mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, PWMDutyCycle);
    //ledcWrite(PWMPosOutChannel, PWMDutyCycle);
    //ledcWrite(PWMNegOutChannel, 0);
  } else if (PWMDutyCycle < 0){
    IR2304.setDuty(PWMDutyCycle, 0);
    //mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, -PWMDutyCycle);
    //mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, -PWMDutyCycle);
    //ledcWrite(PWMNegOutChannel, -PWMDutyCycle);
    //ledcWrite(PWMPosOutChannel, 0);
  }
  
  return 0;
}

int disableOutput(){
  //disable the output by turning the safetyrelais off
  digitalWrite(safetyRelais1_pin, 0);
  digitalWrite(safetyRelais2_pin, 0);
  return 0;
}

int safetyCheck(){
  if (phaseDiff > maxPhaseDiff || phaseDiff < -maxPhaseDiff || netFreq < 48 || netFreq > 52){
    disableOutput();
  }
  if (digitalRead(bigRedButton_pin)){
    disableOutput();
  }

  return 0;
}

void loop() {
  //delay cause otherwise arduino breaks
  //delayMicroseconds(30);
  delay(1);

  //update currentmillis on every loop
  currentMillis = millis();
  currentMicros = micros();
  #if DEBUG
    sprintf(printBuffer, "Currentmillis : %d, CurrentMicros : %d", currentMillis, currentMicros);
    Serial.println(printBuffer);
  #endif

  //read the sensors every loop
  readSensors();

  //calculate the frequency every loop
  currentMicros = micros();
  netFreq = calculateFrequency();

  //calculate the phase difference every loop
  currentMicros = micros();
  phaseDiff = measurePhaseDiff();

  freqSetPoint = netFreq;

  //calculate the correction every loop using the PID algorithms
  //PWMPID.Compute();
  //phasePID.Compute();

  #if DEBUG_pid
    sprintf(printBuffer, "freqOutput : %f", freqOutput);
    //Serial.println(printBuffer);
    freqOutput = freqSetPoint;
  #endif

  safetyCheck();


  //write the PWM for the H-bridge
  currentMicros = micros();
  writePWM(freqOutput);


}