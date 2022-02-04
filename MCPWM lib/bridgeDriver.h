#ifndef bridgeDriver_h
#define bridgeDriver_h

// The library only works for an ESP32
#ifndef ESP32
    #error "This library only works for ESP32."
#endif

#include "Arduino.h"

// Defines the class to use the PWM for the bridge driver
class bridgeDriver
{
    public:
        void init(int PWMA, int PWMB, int unit, int frequency);
        void setDuty(float duty_cycle, int unit);
        void setInverting(bool inverting, int unit);

    private:

};

#endif