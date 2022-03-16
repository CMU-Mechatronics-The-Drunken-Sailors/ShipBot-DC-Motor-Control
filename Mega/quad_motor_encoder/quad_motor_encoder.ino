// Mechatronics - Spring 2022
// Runs on Arduino Mega with two DRV8833 motor drivers.
// Spins 4 motors, forwards and backwards, and reads encoder data.
// Uses these libraries:
// https://github.com/TheArduinist/DRV8833
// https://github.com/PaulStoffregen/Encoder

#include <DRV8833.h>
#include <Encoder.h>

// enum
const int M1 = 1;
const int M2 = 2;
const int M3 = 3;
const int M4 = 4;

// motor speed/direction pins (PWM)
const int M1_FORWARD = 13;
const int M1_BACKWARD = 12;
const int M2_FORWARD = 11;
const int M2_BACKWARD = 10;
const int M3_FORWARD = 9;
const int M3_BACKWARD = 8;
const int M4_FORWARD = 7;
const int M4_BACKWARD = 6;

// motor encoder pins (interrupts for A)
const int M1_ENCODER_A = 18;
const int M1_ENCODER_B = 14;
const int M2_ENCODER_A = 19;
const int M2_ENCODER_B = 15;
const int M3_ENCODER_A = 20;
const int M3_ENCODER_B = 16;
const int M4_ENCODER_A = 3;
const int M4_ENCODER_B = 2;

// motor objects (A for 1&2, B for 3&4)
DRV8833 DRIVER_A = DRV8833();
DRV8833 DRIVER_B = DRV8833();

// encoder objects
Encoder M1_ENCODER(M1_ENCODER_A, M1_ENCODER_B);
Encoder M2_ENCODER(M2_ENCODER_A, M2_ENCODER_B);
Encoder M3_ENCODER(M3_ENCODER_A, M3_ENCODER_B);
Encoder M4_ENCODER(M4_ENCODER_A, M4_ENCODER_B);

void setup()
{
  // encoder pins already set
  
  DRIVER_A.attachMotorA(M1_FORWARD, M1_BACKWARD); // M1 pins
  DRIVER_A.attachMotorB(M2_FORWARD, M2_BACKWARD); // M2 pins
  DRIVER_B.attachMotorA(M3_FORWARD, M3_BACKWARD); // M3 pins
  DRIVER_B.attachMotorB(M4_FORWARD, M4_BACKWARD); // M4 pins

  Serial.begin(115200);
  Serial.println("Basic Encoder Test:");
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

long old_position = -999;

void read_encoder(Encoder encoder_object)
{
  Encoder current_encoder = encoder_object;
  
  long new_position = current_encoder.read(); // changed for desired motor
  if (new_position != old_position)
  {
    old_position = new_position;
    Serial.println(new_position);
  }
}

void loop()
{
  // drive motor and read encoder values
  // change desired motor/encoder to test
  spin_motor(M4, 100);
  read_encoder(M4_ENCODER);
}
