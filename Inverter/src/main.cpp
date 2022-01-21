/*
Invert code
Project group 16
*/

#include <Arduino.h>
#include <CircularBuffer.h>

#define LED_pin 19
#define voltMeas_pin 2
#define test_pin 4

int voltMeas, highestValue;
//int voltMeasBuf[10000];
float netFreq;
bool signal;

unsigned long currentMillis, zeroPointMillis, timerMillis;
//unsigned long zeroPointMillisBuf[5];

CircularBuffer<int, 5> voltMeasBuf;
CircularBuffer<unsigned long, 6> zeroPointMillisBuf;


void setup() {  
  pinMode(LED_pin, OUTPUT);
  pinMode(voltMeas_pin, INPUT);

  Serial.begin(115200);

  timerMillis  = 0;
  zeroPointMillis = 0;
  signal = 0;


}

void loop() {
  //digitalWrite(LED_pin,0);


  delay(1);
  currentMillis = millis();

  voltMeas = analogRead(voltMeas_pin);

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