
#ifndef bridgeDriver_h
#define bridgeDriver_h

#ifndef ESP32
    #error "This library only works for ESP32."
#endif

#include "Arduino.h"

class bridgeDriver
{
    public:
        void init(int PWMA, int PWMB, int unit, int frequency);
        void setDuty(float duty_cycle, int unit);
        void setInverting(bool inverting, int unit);

    private:

};

#endif