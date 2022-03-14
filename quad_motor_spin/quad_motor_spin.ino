// Mechatronics - Spring 2022
// Runs on Teensy 4.1 with two L293 motor drivers.
// Spins 4 motors, forwards and backwards.
// Uses this library: https://github.com/qub1750ul/Arduino_L293

#include <L293.h>

// enum
const int M1 = 1;
const int M2 = 2;
const int M3 = 3;
const int M4 = 4;

// motor speed pins
const int M1_SPEED = 2;
const int M2_SPEED = 29;
const int M3_SPEED = 23;
const int M4_SPEED = 36;

// motor direction pins
const int M1_DIRECTION = 3;
const int M2_DIRECTION = 30;
const int M3_DIRECTION = 22;
const int M4_DIRECTION = 35;

// motor objects
L293_twoWire M1_DRIVER(M1_SPEED, M1_DIRECTION);
L293_twoWire M2_DRIVER(M2_SPEED, M2_DIRECTION);
L293_twoWire M3_DRIVER(M3_SPEED, M3_DIRECTION);
L293_twoWire M4_DRIVER(M4_SPEED, M4_DIRECTION);

void setup()
{
  pinMode(M1_SPEED, OUTPUT);
  pinMode(M2_SPEED, OUTPUT);
  pinMode(M3_SPEED, OUTPUT);
  pinMode(M4_SPEED, OUTPUT);

  pinMode(M1_DIRECTION, OUTPUT);
  pinMode(M2_DIRECTION, OUTPUT);
  pinMode(M3_DIRECTION, OUTPUT);
  pinMode(M4_DIRECTION, OUTPUT);
}

void spin_motor(int motor, int target_speed)
{
  switch (motor)
  {
    case 1:
      if (target_speed > 0)
        M1_DRIVER.forward(target_speed);
      else
        M1_DRIVER.back(abs(target_speed));
      break;

    case 2:
      if (target_speed > 0)
        M2_DRIVER.forward(target_speed);
      else
        M2_DRIVER.back(abs(target_speed));
      break;

    case 3:
      if (target_speed > 0)
        M3_DRIVER.forward(target_speed);
      else
        M3_DRIVER.back(abs(target_speed));
      break;

    case 4:
      if (target_speed > 0)
        M4_DRIVER.forward(target_speed);
      else
        M4_DRIVER.back(abs(target_speed));
      break;

    default:
      break;
  }
}

void loop()
{
  spin_motor(M1, 100);
  spin_motor(M2, 100);
  spin_motor(M3, 100);
  spin_motor(M4, 100);
  delay(1000);
  
  spin_motor(M1, -100);
  spin_motor(M2, -100);
  spin_motor(M3, -100);
  spin_motor(M4, -100);
  delay(1000);
}
