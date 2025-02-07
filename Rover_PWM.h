#ifndef ROVER_PWM_H
#define ROVER_PWM_H

/* We are using Timer2 (8-bit timer) on the Arduino Uno clone included in the rover kit (ATMega328P MCU) for ISR. 
Because the kit restricts us to pins 2-5 for motor input, we cannot utilize hardware PWM wave generation. Instead we will 
simulate a PWM duty cycle by initializing the motor pins to the ON state (1) and then switching OFF (0) when a counter 
variable (_isr_count) attached to Timer2 reaches a certain value (current_pwm), marking the end of a duty cycle. */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <Arduino.h>

/* Main Revisions for 1.1d */

/* We will change most of the definitions and macros from the previous version into constant expressions (constexpr) to 
improve encapsulation and speed. These constants will then be evaluated at compile and inserted directly into the assembly
code. This will result in a longer compile time but (hopefully) a faster program. */

/* Instead of using case switch to control the direction of the motors, we will have the compiler recursively instantiate
variations of a template (i.e. template metaprogramming) to obtain all motor directions. This should also speed up the code
at the expense of file size. */

/* We will inline as many functions as possible using using inline, this forces the compiler to inline the 
associated function regardless of optimization flags, again trade-off between speed and file size.*/

/* To preserve encapsulation while giving ISR access to private members, we need to create a friend ISR wrapper. 
We need to use a wrapper because friend declarations cannot include function attributes. The ISR macro in AVR-GCC however, 
is expanded (in preprocessing) to a function with two attributes (signal, used) and so would cause invalid syntax to be 
generated when combined with a friend declaration. Hence we will declare a wrapper as a friend that can call the private members 
in Rover_PWM while being invoked by the ISR. */

inline void Timer2_COMPA_ISR();

class Rover_PWM {
public:

    //Access function that main will use to create the static instance pointer and instantiate a Rover_PWM class object
    static Rover_PWM* getInstance();

    // Motor Direction and Control definitions
    static constexpr uint8_t FORWARD = 1;
    static constexpr uint8_t REVERSE = 2;
    static constexpr uint8_t LEFT = 3;
    static constexpr uint8_t RIGHT = 4;
    static constexpr uint8_t BRAKE = 5;
    volatile uint8_t current_pwm = 0;

    // Initialization and input functions
    void Motor_PWM_Init();
    void Change_Speed(uint8_t target);
    void SetDirection(uint8_t direction);

private:
    // Private constructor to enforce singleton (i.e. single instance)
    Rover_PWM();

    // Pin definitions
    static constexpr uint8_t _FORWARD_RIGHT = PD2;
    static constexpr uint8_t _REVERSE_RIGHT = PD3;
    static constexpr uint8_t _REVERSE_LEFT = PD4;
    static constexpr uint8_t _FORWARD_LEFT = PD5;

    // PWM Settings
    static constexpr uint8_t _MOTOR_POWER_MIN = 55;
    static constexpr uint8_t _MOTOR_POWER_MAX = 255;
    static constexpr uint8_t _FADE_STEP = 20;

    // Instance and initialization variables
    static Rover_PWM* rover_instance;
    volatile uint8_t _isr_count = 0xff;
    volatile uint8_t _target_pwm = 64;
    volatile uint8_t _is_fading = 0;

    // Grant access to the ISR wrapper
    friend void Timer2_COMPA_ISR();

    // Motor Direction templates -> Recursive template to check all directions
    template <uint8_t Direction>
    inline void MotorDirectionHandler(uint8_t direction);

    // Contol templates -> invoked by motor direction template to set motor pins
    template <uint8_t Direction>
    inline void applyDirection();

    //Inline PWM Control functions (must be defined in header or will cause warning)
    inline void TimerInterruptHandler() {
        ++_isr_count;
        if (_is_fading) {
            if (current_pwm < _target_pwm) {
                current_pwm = min(current_pwm + _FADE_STEP, _target_pwm);
                current_pwm = min(current_pwm, _MOTOR_POWER_MAX);
                if (current_pwm == _target_pwm) _is_fading = 0;
            } 
            else if (current_pwm > _target_pwm) {
                current_pwm = max(current_pwm - _FADE_STEP, _target_pwm);
                current_pwm = max(current_pwm, _MOTOR_POWER_MIN);
                if (current_pwm == _target_pwm) _is_fading = 0;
            } 
            else {
                _is_fading = 0;
            }
        }
    }

    inline void updateMotorPins(uint8_t pins) {
        /* Function for performing atomic updates on motor pins. Interrupts are disabled to ensure atomicity. */
        cli(); // disable global interrupts
        PORTD = ( PORTD & ~((1 << _FORWARD_RIGHT) | (1 << _REVERSE_RIGHT) | (1 << _FORWARD_LEFT) | (1 << _REVERSE_LEFT)) ) | pins;
        sei(); // re-enable global interrupts
    }

};

// Direction function to recursivelly call the motor direction template spcializations until one matches the input direction

template <uint8_t Dir>
inline void Rover_PWM::MotorDirectionHandler(uint8_t direction) {
    if (Dir == direction) {
        applyDirection<Dir>();
    } else {
        MotorDirectionHandler<Dir + 1>(direction);
    }
}

// Template specializations need to be inlined to prevent multiple definitions error:
template <>
inline void Rover_PWM::MotorDirectionHandler<Rover_PWM::BRAKE + 1>(uint8_t) {}

// Inline specializations for applyDirection:
template <>
inline void Rover_PWM::applyDirection<Rover_PWM::FORWARD>() {
    updateMotorPins((1 << _FORWARD_RIGHT) | (1 << _FORWARD_LEFT));
}

template <>
inline void Rover_PWM::applyDirection<Rover_PWM::REVERSE>() {
    updateMotorPins((1 << _REVERSE_RIGHT) | (1 << _REVERSE_LEFT));
}

template <>
inline void Rover_PWM::applyDirection<Rover_PWM::LEFT>() {
    updateMotorPins((1 << _FORWARD_RIGHT) | (1 << _REVERSE_LEFT));
}

template <>
inline void Rover_PWM::applyDirection<Rover_PWM::RIGHT>() {
    updateMotorPins((1 << _REVERSE_RIGHT) | (1 << _FORWARD_LEFT));
}

template <>
inline void Rover_PWM::applyDirection<Rover_PWM::BRAKE>() {
    updateMotorPins((1 << _FORWARD_RIGHT) | (1 << _REVERSE_RIGHT) |
                    (1 << _FORWARD_LEFT) | (1 << _REVERSE_LEFT));
}

#endif