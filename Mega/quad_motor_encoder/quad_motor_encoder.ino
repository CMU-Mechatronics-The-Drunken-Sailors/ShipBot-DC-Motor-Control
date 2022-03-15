// Mechatronics - Spring 2022
// Runs on Teensy 4.1 with two L293 motor drivers.
// Spins 4 motors, forwards and backwards, and reads encoder data.
// Uses these libraries:
// https://github.com/qub1750ul/Arduino_L293
// https://github.com/PaulStoffregen/Encoder

#include <L293.h>
#include <Encoder.h>

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

// motor encoder pins
const int M1_ENCODER_A = 4;
const int M1_ENCODER_B = 5;
const int M2_ENCODER_A = 31;
const int M2_ENCODER_B = 32;
const int M3_ENCODER_A = 21;
const int M3_ENCODER_B = 20;
const int M4_ENCODER_A = 34;
const int M4_ENCODER_B = 33;

// motor objects
L293_twoWire M1_DRIVER(M1_SPEED, M1_DIRECTION);
L293_twoWire M2_DRIVER(M2_SPEED, M2_DIRECTION);
L293_twoWire M3_DRIVER(M3_SPEED, M3_DIRECTION);
L293_twoWire M4_DRIVER(M4_SPEED, M4_DIRECTION);

// encoder objects
Encoder M1_ENCODER(M1_ENCODER_A, M1_ENCODER_B);
Encoder M2_ENCODER(M2_ENCODER_A, M2_ENCODER_B);
Encoder M3_ENCODER(M3_ENCODER_A, M3_ENCODER_B);
Encoder M4_ENCODER(M4_ENCODER_A, M4_ENCODER_B);

void setup()
{
  pinMode(M1_ENCODER_A, INPUT);
  pinMode(M1_ENCODER_B, INPUT);

  pinMode(M2_ENCODER_A, INPUT);
  pinMode(M2_ENCODER_B, INPUT);

  pinMode(M3_ENCODER_A, INPUT);
  pinMode(M3_ENCODER_B, INPUT);

  pinMode(M4_ENCODER_A, INPUT);
  pinMode(M4_ENCODER_B, INPUT);

  pinMode(M1_SPEED, OUTPUT);
  pinMode(M2_SPEED, OUTPUT);
  pinMode(M3_SPEED, OUTPUT);
  pinMode(M4_SPEED, OUTPUT);

  pinMode(M1_DIRECTION, OUTPUT);
  pinMode(M2_DIRECTION, OUTPUT);
  pinMode(M3_DIRECTION, OUTPUT);
  pinMode(M4_DIRECTION, OUTPUT);

  Serial.begin(9600);
  Serial.println("Basic Encoder Test:");
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

long old_position = -999;

void read_encoder(Encoder encoder_object)
{
  Encoder current_encoder = encoder_object;
  
  long new_position = current_encoder.read(); // change for desired motor
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
  spin_motor(M1, -100);
  read_encoder(M1_ENCODER);
}
