/* Main Revisions for 1.1c */

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
