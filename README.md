# ShipBot-DC-Motor-Control
Code designed for Teensy 4.1, driving 4 DC motors with PID control via encoder feedback

quad_motor_spin - used to test basic DC motor connections, with ability to spin forward/backward

quad_motor_open_loop - used to test open-loop control of DC motors, using serial communications to send PWM commands

quad_motor_encoder - used to test DC motor encoders, spinning motors and reading/printing encoder data

quad_motor_pid - used to test DC motor control with PID feedback based on encoder readings, controlled via serial communications to send commands of degrees/second
