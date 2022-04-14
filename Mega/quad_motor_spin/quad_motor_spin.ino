// Mechatronics - Spring 2022
// Runs on Arduino Mega with two DRV8833 motor drivers.
// Spins 4 motors, forwards and backwards.
// Uses this library: https://github.com/TheArduinist/DRV8833

#include <DRV8833.h>

// enum
const int M1 = 1;
const int M2 = 2;
const int M3 = 3;
const int M4 = 4;

// motor speed/direction pins (PWM)
const int M1_FORWARD = 7;
const int M1_BACKWARD = 6;
const int M2_FORWARD = 9;
const int M2_BACKWARD = 8;
const int M3_FORWARD = 5;
const int M3_BACKWARD = 4;
const int M4_FORWARD = 11;
const int M4_BACKWARD = 10;

// motor objects (A for 1&2, B for 3&4)
DRV8833 DRIVER_A = DRV8833();
DRV8833 DRIVER_B = DRV8833();

void setup()
{
  DRIVER_A.attachMotorA(M1_FORWARD, M1_BACKWARD); // M1 pins
  DRIVER_A.attachMotorB(M2_FORWARD, M2_BACKWARD); // M2 pins
  DRIVER_B.attachMotorA(M3_FORWARD, M3_BACKWARD); // M3 pins
  DRIVER_B.attachMotorB(M4_FORWARD, M4_BACKWARD); // M4 pins
}

void spin_motor(int motor, int target_speed)
{
  switch (motor)
  {
    case 1:
      if (target_speed > 0)
        DRIVER_A.motorAForward(target_speed);
      else
        DRIVER_A.motorAReverse(abs(target_speed));
      break;

    case 2:
      if (target_speed > 0)
        DRIVER_A.motorBForward(target_speed);
      else
        DRIVER_A.motorBReverse(abs(target_speed));
      break;

    case 3:
      if (target_speed > 0)
        DRIVER_B.motorAForward(target_speed);
      else
        DRIVER_B.motorAReverse(abs(target_speed));
      break;

    case 4:
      if (target_speed > 0)
        DRIVER_B.motorBForward(target_speed);
      else
        DRIVER_B.motorBReverse(abs(target_speed));
      break;

    default:
      break;
  }
}

void loop()
{
  spin_motor(M1, -1 * 255);
  spin_motor(M2, 255);
  spin_motor(M3, -1 * 255);
  spin_motor(M4, 255);
  delay(3000);
  
  spin_motor(M1, -1 * -255);
  spin_motor(M2, -255);
  spin_motor(M3, -1 * -255);
  spin_motor(M4, -255);
  delay(3000);
}
