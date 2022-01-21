/*
Inverter code
Project group 16
*/

#include <Arduino.h>
#include <CircularBuffer.h>

//define used hardware pins
#define LED_pin 19
#define netVoltMeas_pin 2
#define test_pin 4

//define variables
int voltMeas, highestValue;
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

  Serial.begin(115200);

  //initialise some variables
  timerMillis  = 0;
  zeroPointMillis = 0;
  signal = 0;

}

int readSensors(){
  //measure and safe the voltage measurements
  voltMeas = analogRead(netVoltMeas_pin);
}

void loop() {
  //delay cause otherwise arduino brakes
  delay(1);

  //update currentmillis on every loop
  currentMillis = millis();

  readSensors();

  if (voltMeas > highestValue){
    highestValue = voltMeas;
  }

  if (voltMeas < (0+0.1*highestValue) && signal == 1){
    digitalWrite(LED_pin,0);
    zeroPointMillisBuf.push(currentMillis);
    signal = 0;
  }

  if (voltMeas > highestValue-(0.1*highestValue)){
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

}