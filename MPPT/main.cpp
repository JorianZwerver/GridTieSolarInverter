// Liberaries
#include <Arduino.h>
#include <MCP3008.h>

// Pin definitions
#define CS_PIN D8
#define CLOCK_PIN D5
#define MOSI_PIN D7
#define MISO_PIN D6
#define MOSFET_DRIVER D2

#define current_channel 4
#define voltage_input_channel 0
#define voltage_output_channel 7

// Important values
#define MAX_DUTY_CYCLE 600
#define MIN_DUTY_CYCLE 100

#define R_1 9996.4
#define R_2 1193
#define R_3 9996.4
#define R_4 1194.4
#define ADC_MAX_VALUE 1024
#define ADC_MAX_V 3.3

const double V_FACTOR_1 = (ADC_MAX_V / ADC_MAX_VALUE) * ((R_1 + R_2)/R_2);
const double V_FACTOR_2 = (ADC_MAX_V / ADC_MAX_VALUE) * ((R_3 + R_4)/R_4);
const double I_FACTOR = ADC_MAX_V / ADC_MAX_VALUE / 2;

#define PWM_FREQ 20000
#define PWM_RES 10
#define AVERAGE 50

// Safety
#define MAX_CURRENT 1
#define MAX_VOLTAGE_IN 25.5
#define MAX_VOLTAGE_OUT 29
#define MIN_VOLTAGE_IN 7

// Modes current sweep
#define MPP 1
#define MIN_VOLT 2

// Structure for storing the data from the sensors.
typedef struct solarReading {
  double current;
  double voltageIn;
  double voltageOut;
  double inputPow;
} solarReading;

// Amount of steps for the current sweep
#define numberOfSwoopers 101

// Defining class for the MCP3008 library
MCP3008 adc(CLOCK_PIN, MOSI_PIN, MISO_PIN, CS_PIN);

// Creating the two storage spaces for the sensor readings
solarReading solarNew, solarOld;

// Defining variables
int swooper[numberOfSwoopers];
int duty_cycle_val = 350;
int slope = 2;
int correction = 10;
bool direction = true;
double greatestPower = 0;
char buffer[100];
unsigned long previousTime;
unsigned long currentTime;


// Define functions
void readADCsolar(solarReading *solar);
void safety();
float averageReading(int average, int channel);
int hillClimbing(int duty_cycle, solarReading solarNew, solarReading solarOld);
int preturbAndObserve(int duty_cycle, solarReading solarNew, solarReading solarOld);
int sweep(int mode);


void setup() {
  // Starting serial port for data logging
  Serial.begin(115200);
  // Starting the PWM at correct frequency, resolution and duty cycle
  analogWrite(MOSFET_DRIVER, 0);
  analogWriteFreq(PWM_FREQ);
  analogWriteResolution(PWM_RES);

  // Starting timer for data logging
  previousTime = millis();
}

void loop() {
  // Saving the data from last loop for Hill Climbing and Preturb and Observe
  solarOld = solarNew;
  // Store current time
  currentTime = millis();
  // Read all sensor data
  readADCsolar(&solarNew);

  // New duty cycle according to data. preturbAndObserve can be replaced for hillClimbing to use Hill Climbing/
  duty_cycle_val = preturbAndObserve(duty_cycle_val, solarNew, solarOld);

  // Code for Current Sweep
  //if(greatestPower - solarNew.inputPow > 0.2 || greatestPower - solarNew.inputPow < -0.2) duty_cycle_val = sweep(MPP);

  // Safety check
  safety();

  // Data is logged to terminal with 5 Hz
  if(currentTime - previousTime > 200)
  {
    sprintf(buffer, "%d Duty, %.2lf A, %.2lf V, %.2lf V, %.2lf W", duty_cycle_val, solarNew.current, solarNew.voltageIn, solarNew.voltageOut, solarNew.inputPow);
    Serial.println(buffer);
    previousTime = currentTime;
  }

  // Assigning new duty cycle to PWM
  analogWrite(MOSFET_DRIVER, duty_cycle_val);
}

// Reads all data from sensors and converts it to usable values
void readADCsolar(solarReading *solar)
{
  solar->current = averageReading(100, current_channel) * I_FACTOR;
  solar->voltageIn = averageReading(AVERAGE, voltage_input_channel) * V_FACTOR_1;
  solar->voltageOut = averageReading(AVERAGE, voltage_output_channel) * V_FACTOR_2;
  solar->inputPow = solar->current * solar->voltageIn;
}

// Checks for edge cases to protect the MPPT against over voltage and to high or to low duty cycles
void safety()
{
  if(solarNew.voltageOut > MAX_VOLTAGE_OUT) duty_cycle_val -= correction;

  if(duty_cycle_val > MAX_DUTY_CYCLE) 
  {
    duty_cycle_val -= correction;
    Serial.println("Duty cycle too high!");
  }
  else if(duty_cycle_val < MIN_DUTY_CYCLE) 
  {
    duty_cycle_val += correction;
    Serial.println("Duty cycle too low!");
  }
}

// Calculates the average from multiple readings to reduce the effect of outlaying values.
float averageReading(int average, int channel){
  float averageRead = 0;
  int i;

  for(i = 0; i < average; i++){
    averageRead += adc.readADC(channel);
  }

  return (averageRead / average);
}

// Function for implementing Hill Climbing with some overcurrent, voltage and duty cycle protection
int hillClimbing(int duty_cycle, solarReading solarNew, solarReading solarOld)
{
  if(duty_cycle < MAX_DUTY_CYCLE && duty_cycle > MIN_DUTY_CYCLE){
    if(solarNew.voltageOut > MAX_VOLTAGE_OUT) duty_cycle -= correction;
    else if (solarNew.current > MAX_CURRENT) duty_cycle -= correction;
    else {
        if(solarNew.inputPow - solarOld.inputPow > 0.01) 
        {
          duty_cycle += slope;
        }
        else if (solarNew.inputPow - solarOld.inputPow < -0.01) 
        {
          slope *= -1;
          duty_cycle += slope;
        }
        else if(solarNew.voltageOut < 25.5 && solarNew.voltageIn > 10) duty_cycle += 1;
    }
  } 
  else 
  {
    if(duty_cycle == MAX_DUTY_CYCLE) duty_cycle -= slope;
    else if (duty_cycle == MIN_DUTY_CYCLE) duty_cycle += slope;
  }
  return duty_cycle;
}

// Function for implementing Preturb and Observe with some overcurrent, voltage and duty cycle protection
int preturbAndObserve(int duty_cycle, solarReading solarNew, solarReading solarOld)
{
  if(duty_cycle < MAX_DUTY_CYCLE && duty_cycle > MIN_DUTY_CYCLE){
    if(solarNew.voltageOut > MAX_VOLTAGE_OUT) duty_cycle -= correction;
    else if (solarNew.current > MAX_CURRENT) duty_cycle -= correction;
    else {
        if (solarNew.inputPow - solarOld.inputPow > 0.01)
        {
          if (solarNew.voltageIn > solarOld.voltageIn) duty_cycle += slope;
          else if (solarNew.voltageIn < solarOld.voltageIn) duty_cycle -= slope; 
        }
        else if (solarNew.inputPow - solarOld.inputPow < -0.01)
        {
          if (solarNew.voltageIn > solarOld.voltageIn) duty_cycle -= slope; 
          else if (solarNew.voltageIn < solarOld.voltageIn) duty_cycle += slope; 
        }
        else if (solarNew.voltageOut < 26 && solarNew.voltageIn > 15) duty_cycle += slope;
    }
  } 
  else 
  {
    if(duty_cycle == MAX_DUTY_CYCLE) duty_cycle -= slope;
    else if (duty_cycle == MIN_DUTY_CYCLE) duty_cycle += slope;
  }
  return duty_cycle;
}

// Function to implement the Current Sweep algorithm
int sweep(int mode)
{
  int greatestSwooper = 0;
  int greatestSwooperWithVoltage = -1;

  for (int i = 0; i < numberOfSwoopers; i++)
  {
    analogWrite(MOSFET_DRIVER, (swooper[i] = (i*slope + MIN_DUTY_CYCLE)));
    readADCsolar(&solarNew);

    if(solarNew.voltageOut > 29) {
      analogWrite(MOSFET_DRIVER, (i*slope + MIN_DUTY_CYCLE - slope));
      break;
    }

    if(solarNew.inputPow > greatestPower)
    {
      greatestPower = solarNew.inputPow;
      greatestSwooper = i;

      if(solarNew.voltageOut > MAX_VOLTAGE_IN)
      {
        greatestSwooperWithVoltage = i;
      }
    }
  }

  // Selects the priority of the Current Sweep: either finding the MPP or the MPP with a minimal voltage of 25.5
  switch(mode)
  {
    case 1:
      return swooper[greatestSwooper];
    case 2:
      if(greatestSwooperWithVoltage == -1) { Serial.println("No duty-cycle with sufficient voltage."); return swooper[greatestSwooper]; }
      else { return swooper[greatestSwooperWithVoltage]; }
    default:
      return swooper[greatestSwooper];
  }
}