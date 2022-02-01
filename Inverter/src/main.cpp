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
#define DEBUG_time 0

//define used hardware pins
#define LED_pin LED_BUILTIN
#define netFreqMeas_pin 15
#define netPhaseMeas_pin 14
#define invVoltMeas_pin 4
#define PWMPosOut_pin 12
#define PWMNegOut_pin 32
#define test_pin 14
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
#define PPMSamples 1 //peak point measuremets samples

//how often to print the information in ms
#define freqPrintDelay 5000
#define phasePrintDelay 250
#define timer3PrintDelay 2500

//define safety margins
#define maxPhaseDiff 20
#define maxFreqDiff 1

//define variables
int invVoltMeas;
double netFreq, phaseDiff, phaseOffset;
bool signal, netFreqMeas, prevNetFreqMeas, netPhaseMeas, prevNetPhaseMeas, prevPWMDutyCycle;
//unsigned long to not overflow the buffer
unsigned long currentMillis, currentMicros, zeroPointMicros, timerMillis, timer2Millis, timer3Millis, startZeroPoint, endZeroPoint, loopTimer;
unsigned long peakPointMicros, startOutputPeakPointMicros, endOutputPeakPointMicros, outputPeakPointMicros, theoOutputPeakPointMicros, startPeakPoint, endPeakPoint, loopStartMicros;

char printBuffer[100];

CircularBuffer<int, 5> voltMeasBuf; //circular buffer of the voltage measurements
CircularBuffer<unsigned long, (ZPMSamples+1)> zeroPointMicrosBuf; //circular buffer of the zeroPoints in microSec
CircularBuffer<unsigned long, (PPMSamples+1)> peakPointMicrosBuf;
CircularBuffer<unsigned long, (PPMSamples+1)> outputPeakPointMicrosBuf;
CircularBuffer<unsigned long, (100)> loopTimingBuf;

//Define PID Variables we'll be connecting to
double freqSetPoint, phaseSetPoint, freqOutput;

//Specify the PID links and initial tuning parameters
double freqPIDp=1, freqPIDi=1, freqPIDd=1;
double phasePIDp=2, phasePIDi=1, phasePIDd=1;
PID freqPID(&netFreq, &freqOutput, &freqSetPoint, freqPIDp, freqPIDi, freqPIDd, DIRECT);
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
  phaseSetPoint = 90;

  //turn the PWM PID on
  freqPID.SetMode(AUTOMATIC);
  phasePID.SetMode(AUTOMATIC);

  //config the mcpwm
  IR2304.init(PWMA, PWMB, 0, 20000);
  IR2304.setInverting(true, 0);
}

int readSensors(){
  //measure and save the voltage measurements
  netFreqMeas = 1-digitalRead(netFreqMeas_pin);
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

  //only print every delay
  if ((currentMillis - timerMillis) > freqPrintDelay){
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
    peakPointMicrosBuf.push((startPeakPoint+(endPeakPoint-startPeakPoint)/2));
    prevNetPhaseMeas = 0;
  }

  //10ms is 180degrees out of sync
  double halfWaveTime = (1000000.0/netFreq)*0.5;

  //difference between the peaks of the measured signal and produced sin for the pwm
  int microsDiff = (peakPointMicrosBuf.last()-theoOutputPeakPointMicros);

  //remove whole wave differences
  double msPhaseDiff = fmod(double(microsDiff), halfWaveTime);

  //phase differenc ein degrees 
  phaseDiff = msPhaseDiff*(180/halfWaveTime);

  #if DEBUG_phase
    //only print every delay
    if ((currentMillis - timer2Millis) > phasePrintDelay){
      sprintf(printBuffer, "phaseDiff : %0.3f Degrees, microsdiff : %d micS", phaseDiff, microsDiff);
      Serial.println(printBuffer);
      timer2Millis = currentMillis;
    }
  #endif

  return phaseDiff;
}

int writePWM(float freqOutput, double phaseOffset){
  int PWMDutyCycle;

  loopTimer = currentMillis;

  #if DEBUG_pwm
    freqOutput = 50;
    phaseOffset = 0;
  #endif

  PWMDutyCycle = 100*sin((TWO_PI*freqOutput*loopTimer*0.001)+(PI*(phaseOffset/180)));
  
  //use maths to find the peak of the outputted sin
  if (fmod(double(loopTimer),(0.5*PI))){
    theoOutputPeakPointMicros = currentMicros;
  }
  

  if (PWMDutyCycle > 80 && !prevPWMDutyCycle){
    startOutputPeakPointMicros = currentMicros;
    prevPWMDutyCycle = 1;
  } else if(PWMDutyCycle < 80 && prevPWMDutyCycle){
    endOutputPeakPointMicros = currentMicros;
    prevPWMDutyCycle = 0;
    outputPeakPointMicrosBuf.push(startOutputPeakPointMicros+(endOutputPeakPointMicros - startOutputPeakPointMicros)/2);
  }

  #if DEBUG_dc
    if(PWMDutyCycle == 100){
      sprintf(printBuffer, "Dutycycle at 100 gives micros: %d and calc PPM: %d and theo PPM: %d", currentMicros, outputPeakPointMicrosBuf.last(), theoOutputPeakPointMicros);
      Serial.println(printBuffer);
    }
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

int safetyCheck(float netFreq){
  /*
  if (phaseDiff > maxPhaseDiff || phaseDiff < -maxPhaseDiff){
    disableOutput();
    sprintf(printBuffer, "Phase difference out of bounds!! output disabled, phaseDiff: %d", phaseDiff);
    Serial.println(printBuffer);
  }
  */
  if (netFreq < 50-maxFreqDiff || netFreq > 50+maxFreqDiff){
    disableOutput();
    sprintf(printBuffer, "net frequecy out of bounds!! Output disabled, netFreq: %f", netFreq);
    Serial.println(printBuffer);
  }

  if (currentMicros-zeroPointMicrosBuf.last() > 50000){
    disableOutput();
    sprintf(printBuffer, "No recent net frequecy measuremets found!! Output disabled");
    Serial.println(printBuffer);
  }

  if (digitalRead(bigRedButton_pin)){
    disableOutput();
  }

  return 0;
}

void loop() {
  //delay cause otherwise arduino breaks
  //delayMicroseconds(30);
  //delay(1);

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
  //output go to freqOutput and phaseOffset
  freqPID.Compute();
  phasePID.Compute();

  #if DEBUG_pid
    if ((currentMillis - timer3Millis) > timer3PrintDelay){
      sprintf(printBuffer, "freqOutput : %f, phaseOutput: %f", freqOutput, 90-phaseOffset);
      Serial.println(printBuffer);
      timer3Millis = currentMillis;
    }
  #endif

  //wait 5 sec before enabling the safety
  if (currentMillis > 5000){
    safetyCheck(netFreq);
  }

  //write the PWM for the H-bridge
  currentMicros = micros();
  writePWM(50+freqOutput, phaseOffset);

  #if DEBUG_time
    loopTimingBuf.push(micros()-loopStartMicros);
    if (loopTimingBuf.isFull()){
      sprintf(printBuffer, "avg looptime : %d micros sec", ((loopTimingBuf.last() - loopTimingBuf.first())/100));
      Serial.println(printBuffer);
    }
  loopStartMicros = currentMicros;
  #endif

}