#include <Arduino.h>
#include <driver/mcpwm.h>

#define PWMB 21
#define PWMA 22

#define PWM_A MCPWM_GEN_A
#define PWM_B MCPWM_GEN_B

void setup() {

  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PWMA);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, PWMB);

  mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000;    //frequency,
    pwm_config.cmpr_a = 0.0;    		//duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0.0;    		//duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_1);
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);

  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 50.0);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 40.0);
}

void loop() {
  // put your main code here, to run repeatedly:
}