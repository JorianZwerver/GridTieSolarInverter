// Libraries
#include "bridgeDriver.h"
#include "driver/mcpwm.h"
#include "Arduino.h"

// Initializes the correct MCPWM module with the correct frequency, duty cycle and unit.
void bridgeDriver::init(int PWMA, int PWMB, int unit, int frequency) 
{
    if(unit > 1 || unit < 0){ 
        return;
    }
    
    mcpwm_config_t pwm_config;
    pwm_config.frequency = frequency;
    pwm_config.cmpr_a = 0.0;
    pwm_config.cmpr_b = 0.0;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    if(unit)
    { 
        mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, PWMA);
        mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0B, PWMB);
        mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config);
    } 
    else 
    {
        mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PWMA);
        mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, PWMB);
        mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    }

}

// Updates the duty cycle of the PWM
void bridgeDriver::setDuty(float duty_cycle, int unit)
{
    if(duty_cycle < 100 || duty_cycle > 0) {
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_cycle);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, (100.0 - duty_cycle));
    } else { return; }
}

// Sets the unit to inverted or non-inverted PWM. Inverted PWM is PWM where the port one is on if port two is off and vice versa.
void bridgeDriver::setInverting(bool inverting, int unit)
{
    if(unit > 1 || unit < 0){ 
        return;
    }

    if (inverting && unit)
    {
        mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM_DUTY_MODE_1);
        mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
    }
    else if (inverting && !unit) 
    {
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_1);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
    }
    else if (!inverting && unit)
    {
        mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
        mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
    }
    else
    {
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
    }
    
}