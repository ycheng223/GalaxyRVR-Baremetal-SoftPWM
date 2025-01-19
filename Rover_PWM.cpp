#include "Rover_PWM.h"


// Global Variables (should be private variables in class Rover_PWM but cannot be passed to ISR currently)
volatile uint8_t _isr_count = 0xff; //starts at 255 so that ++_isr_count can be initialized at 0 in the ISR
volatile uint8_t current_pwm = 0; //Current PWM value
volatile uint8_t _target_pwm = 64; // Target PWM value, initialized at ~25% duty cycle (64/255)
volatile uint8_t _is_fading = 0; //Indicates whether duty cycle is currently increasing/decreasing

void Rover_PWM::Motor_PWM_Init(void) {
/* Configuring the timer, registers, motor pins, and initializing internal variables used to control the duty cycle. */

    DDRD |= (1 << FORWARD_RIGHT) | (1 << REVERSE_RIGHT); /* Set right side motor pins to OUTPUT */
    DDRD |= (1 << REVERSE_LEFT) | (1 << FORWARD_LEFT); /* Set left side motor pins to OUTPUT */

    /* Clear TCCR2A and TCCR2B Registers*/
    TCCR2A = 0; // set entire TCCR2A register to 0
    TCCR2B = 0; // set entire TCCR2B register to 0
    
    TCCR2B |= (1 << WGM21); // Set WGM21 bit for CTC (mode 2)
    TCCR2B |= (1 << CS21);   // Set CS21 bit for a prescaler of 8
    
    /* Set Counter and Output Compare -> */
    TCNT2  = 0; //initialize counter value register
    OCR2A = 200; //set output compare register to 200 -> Say we want a 10 KHz PWM Signal -> F_CPU/(PRESCALER*256*PWM_FREQ) = (16*10^6)/(8*10000) ~= 200
    TIMSK2 |= (1 << OCIE2A); // enable Timer2 compare match A interrupt mode
    
    /* Enable global interrupts */
    sei();
}

void Rover_PWM::Change_Speed(uint8_t target) {
    
    /* This is the fading logic that (along with the ISR) allows us to control the motor by increasing/decreasing the duty cycle. Target
    is the desired duty cycle value. */

    if (target > MOTOR_POWER_MAX) {
        target = MOTOR_POWER_MAX;
    } /* Ensures that target does not exceed the maximum value of the 8-bit counter or it will overflow and loop around back to 0. */
    if (target < MOTOR_POWER_MIN) {
        target = MOTOR_POWER_MIN;
    } /* Likewise, we want to ensure that the target does not go below the minimum duty cycle required for the motor to move (i.e. overcome static friction). */ 

    _target_pwm = target;
    _is_fading = 1;
    /* in the ISR, we will compare _target_pwm with current_pwm and incrementally adjust current_pwm towards _target_pwm in steps defined 
    by FADE_STEP in Rover_PWM.h. */
}

void Rover_PWM::Motor_Direction(uint8_t direction, uint8_t current_pwm) {
    /* First clear all motor pins */
    PORTD &= ~((1 << FORWARD_RIGHT) | (1 << REVERSE_RIGHT) | 
               (1 << FORWARD_LEFT) | (1 << REVERSE_LEFT));

    /* Compares _isr_count with current duty cycle value (current_pwm) to determine if motors should be on or off (i.e. are we in the ON portion of the duty cycle). */
    if (_isr_count < current_pwm) {
        switch (direction) {  // If motors should be ON, decide which motor to turn ON based on inputted direction, else standby.
            case FORWARD:
                PORTD |= (1 << FORWARD_RIGHT) | (1 << FORWARD_LEFT);
                break;

            case REVERSE:
                PORTD |= (1 << REVERSE_RIGHT) | (1 << REVERSE_LEFT);
                break;

            case LEFT:
                PORTD |= (1 << FORWARD_RIGHT) | (1 << REVERSE_LEFT);
                break;

            case RIGHT:
                PORTD |= (1 << REVERSE_RIGHT) | (1 << FORWARD_LEFT);
                break;

            case BRAKE:
                PORTD |= (1 << FORWARD_RIGHT) | (1 << REVERSE_RIGHT) |
                         (1 << FORWARD_LEFT) | (1 << REVERSE_LEFT);
                break;

            default:{
                break;
            } //Standby, don't need to do anything since all motor pins are already cleared before the switch.
        }
    }
}

/* Interrupt handler: Uses Timer2 (8 bit) set to CTC mode (i.e. counts up to the value stored in register OCR2A; 40 in our case; 
and then resets to 0 before counting up again) to generate a hardware interrupt that simulates the period of a full duty cycle. */
ISR(TIMER2_COMPA_vect) {
    ++_isr_count; // Counter that tracks Timer2, used to turn duty cycle off when it exceeds the current duty cycle value stored in current_pwm.

    if (_is_fading) { // Logic for speeding up the motor: If fading is in progress..
        if (current_pwm < _target_pwm) { // And the current duty cycle value is less than the desired duty cycle value
            current_pwm += FADE_STEP; // Increment the current duty cycle value by 20 (~8%)
            if (current_pwm >= _target_pwm) { // If by incrementing we overshoot the desired duty cycle value
                current_pwm = _target_pwm; // Set the current duty cycle value to the desired duty cycle value
                _is_fading = 0; // Duty cycle value reached, fading complete
            }
            } else if (current_pwm > _target_pwm) { // Same logic as above, only reversed (i.e. we are slowing down).
                current_pwm -= FADE_STEP;
                if (current_pwm <= _target_pwm) {
                    current_pwm = _target_pwm;
                    _is_fading = 0;
                }
            } else {
                _is_fading = 0; //If target_pwm is reached, no need for further adjustments
            }
        }

}