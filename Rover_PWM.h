#ifndef ROVER_PWM_H
#define ROVER_PWM_H

/* We are using Timer2 (8-bit timer) on the Arduino Uno clone included in the rover kit (ATMega328P MCU) for ISR. 
Because the kit restricts us to pins 2-5 for motor input, we cannot utilize hardware PWM wave generation. Instead we will 
simulate a PWM duty cycle by initializing the motor pins to the ON state (1) and then switching OFF (0) when a counter 
variable (_isr_count) attached to Timer2 reaches a certain value (current_pwm), marking the end of a duty cycle. */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <Arduino.h>

/* Pin definitions */
#define FORWARD_RIGHT PD2
#define REVERSE_RIGHT PD3
#define REVERSE_LEFT PD4
#define FORWARD_LEFT PD5

/* Motor control definitions */
#define FORWARD 1
#define REVERSE 2
#define LEFT 3
#define RIGHT 4
#define BRAKE 5

/* PWM settings */
#define MOTOR_POWER_MIN (uint8_t) 55
#define MOTOR_POWER_MAX (uint8_t) 255
#define FADE_STEP 20

extern volatile uint8_t _isr_count;
extern volatile uint8_t _target_pwm;
extern volatile uint8_t _is_fading;
extern volatile uint8_t current_pwm;

class Rover_PWM{
    public:
    void Motor_PWM_Init(void);
    void Change_Speed(uint8_t target);
    void Motor_Direction(uint8_t direction, uint8_t pwm_value);

    private:
    uint8_t _is_fading_in_progress(void);
};


#endif