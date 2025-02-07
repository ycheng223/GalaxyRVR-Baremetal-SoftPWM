#include "Rover_PWM.h"

Rover_PWM* Rover_PWM::rover_instance = nullptr;

/* Declare a class pointer (rover_instance) and keep it pointed only to the first Rover_PWM instance created using a private construtor
invoked by getInstance. This enforces singleton by ensuring that only a single instance of class Rover_PWM can be created.*/
Rover_PWM* Rover_PWM::getInstance() {
    if (rover_instance == nullptr) {
        rover_instance = new Rover_PWM();
    }
    return rover_instance;
}

Rover_PWM::Rover_PWM() {
    if (rover_instance == nullptr) {
        rover_instance = this;
    }
}

void Rover_PWM::Motor_PWM_Init(void) {
/* Configuring the timer, registers, motor pins, and initializing internal variables used to control the duty cycle. */

    DDRD |= (1 << Rover_PWM::_FORWARD_RIGHT) | (1 << Rover_PWM::_REVERSE_RIGHT); /* Set right side motor pins to OUTPUT */
    DDRD |= (1 << Rover_PWM::_REVERSE_LEFT) | (1 << Rover_PWM::_FORWARD_LEFT); /* Set left side motor pins to OUTPUT */

    /* Clear TCCR2A and TCCR2B Registers*/
    TCCR2A = 0; // set entire TCCR2A register to 0
    TCCR2B = 0; // set entire TCCR2B register to 0
    
    TCCR2B |= (1 << WGM21); // Set WGM21 bit for CTC (mode 2)
    TCCR2B |= (1 << CS21);   // Set CS21 bit for a prescaler of 8
    
    /* Set Counter and Output Compare -> */
    TCNT2  = 0; //initialize counter value register
    OCR2A = 199; //set output compare register to 199 -> Say we want a 10 KHz PWM Signal -> F_CPU/(PRESCALER*256*PWM_FREQ) = (16*10^6)/(8*10000) ~= 199
    TIMSK2 |= (1 << OCIE2A); // enable Timer2 compare match A interrupt mode
}

/* This is the fading logic that (along with the ISR) allows us to control the motor by increasing/decreasing the duty cycle. Target
is the desired duty cycle value. */
void Rover_PWM::Change_Speed(uint8_t target) {
    if (target > _MOTOR_POWER_MAX) {
        target = _MOTOR_POWER_MAX;
    } /* Ensures that target does not exceed the maximum value of the 8-bit counter or it will overflow and loop around back to 0. */
    if (target < _MOTOR_POWER_MIN) {
        target = _MOTOR_POWER_MIN;
    } /* Likewise, we want to ensure that the target does not go below the minimum duty cycle required for the motor to move (i.e. overcome static friction). */ 
    _target_pwm = target;
    _is_fading = 1;

    /* in the ISR, we will compare _target_pwm with current_pwm and incrementally adjust current_pwm towards _target_pwm in steps defined 
    by FADE_STEP in Rover_PWM.h. */
}

// Function for direction input. Activates template recursion starting from FORWARD (two code blocks down)
void Rover_PWM::SetDirection(uint8_t direction) {
    cli(); // Disable interrupts for atomic operation

    // Clear all motor pins first
    PORTD &= ~((1 << _FORWARD_RIGHT) | (1 << _REVERSE_RIGHT) | 
               (1 << _FORWARD_LEFT) | (1 << _REVERSE_LEFT));

    // Only activate motors if we're in the ON portion of the PWM cycle
    if (_isr_count < current_pwm) {
        // Start recursion with the first direction (FORWARD) and go from there...
        MotorDirectionHandler<FORWARD>(direction);
    }

    sei(); // Re-enable interrupts
}

// Inline ISR wrapper function, see notes in header file.
inline void Timer2_COMPA_ISR() {
    if (Rover_PWM::rover_instance) {
        Rover_PWM::rover_instance->TimerInterruptHandler();
    }
}

// The actual ISR calling the wrapper ISR function above:
ISR(TIMER2_COMPA_vect) {
    Timer2_COMPA_ISR(); //calls the wrapper which will get the private class members/variables
}
