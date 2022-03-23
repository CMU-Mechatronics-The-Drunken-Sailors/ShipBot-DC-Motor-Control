# ShipBot-DC-Motor-Control
Code designed to drive 4 DC motors with PID control via encoder feedback

quad_motor_spin - used to test basic DC motor connections, with ability to spin forward/backward

quad_motor_open_loop - used to test open-loop control of DC motors, using serial communications to send PWM commands

quad_motor_encoder - used to test DC motor encoders, spinning motors and reading/printing encoder data

quad_motor_pid - used to test DC motor control with PID feedback based on encoder readings, controlled via serial communications to send commands of degrees/second (this is the key file that is used for locomotion)

Two folders - one implemented on the Arduino Mega and the other on the Teensy 4.1. Very similar code - only difference is the pinout. Teensy code is not fully working since we decided to use the Mega, instead. Could probably merge the code and just use #ifdef to change pins based on which device used...

Motors used: https://www.digikey.com/en/products/detail/dfrobot/FIT0186/6588528

Motor drivers used: https://www.pololu.com/product/2130
