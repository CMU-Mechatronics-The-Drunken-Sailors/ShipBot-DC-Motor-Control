// Mechatronics - Spring 2022
// Runs on Teensy 4.1 with two L293 motor drivers.
// Spins 4 motors, forwards and backwards, and reads encoder data for PID feedback.
// Motors controlled via serial communications, giving commands of degrees/second.
//
// Uses these libraries:
// https://github.com/qub1750ul/Arduino_L293
// https://github.com/PaulStoffregen/Encoder
// https://github.com/PowerBroker2/ArduPID
//
// Motor used: // Motor used: https://media.digikey.com/pdf/Data%20Sheets/DFRobot%20PDFs/FIT0186_Web.pdf

#include <L293.h>
#include <Encoder.h>
#include <ArduPID.h>

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

// based on ticks per revolution
const float DEGREES_PER_TICK = (360.0 / 2797);

// DC motor 1's driver, encoder, and PID values
L293_twoWire M1_DRIVER(M1_SPEED, M1_DIRECTION);
Encoder M1_ENCODER(M1_ENCODER_A, M1_ENCODER_B);
ArduPID M1_PID;

volatile double M1_current_speed = 0;
volatile double M1_target_speed = 0;
volatile double M1_pid_out = 0;
volatile unsigned long M1_last_time;
volatile long M1_last_enc;

// DC motor 2's driver, encoder, and PID values
L293_twoWire M2_DRIVER(M2_SPEED, M2_DIRECTION);
Encoder M2_ENCODER(M2_ENCODER_A, M2_ENCODER_B);
ArduPID M2_PID;

volatile double M2_current_speed = 0;
volatile double M2_target_speed = 0;
volatile double M2_pid_out = 0;
volatile unsigned long M2_last_time;
volatile long M2_last_enc;

// DC motor 3's driver, encoder, and PID values
L293_twoWire M3_DRIVER(M3_SPEED, M3_DIRECTION);
Encoder M3_ENCODER(M3_ENCODER_A, M3_ENCODER_B);
ArduPID M3_PID;

volatile double M3_current_speed = 0;
volatile double M3_target_speed = 0;
volatile double M3_pid_out = 0;
volatile unsigned long M3_last_time;
volatile long M3_last_enc;

// DC motor 4's driver, encoder, and PID values
L293_twoWire M4_DRIVER(M4_SPEED, M4_DIRECTION);
Encoder M4_ENCODER(M4_ENCODER_A, M4_ENCODER_B);
ArduPID M4_PID;

volatile double M4_current_speed = 0;
volatile double M4_target_speed = 0;
volatile double M4_pid_out = 0;
volatile unsigned long M4_last_time;
volatile long M4_last_enc;

// PID motor constants
#define SPEED_KP 1
#define SPEED_KI 0.005
#define SPEED_KD 0.4

void setup()
{
  Serial.begin(9600);

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

  // start DC encoders' time/position
  M1_last_time = millis();
  M1_last_enc = M1_ENCODER.read();

  M2_last_time = millis();
  M2_last_enc = M2_ENCODER.read();

  M3_last_time = millis();
  M3_last_enc = M3_ENCODER.read();

  M4_last_time = millis();
  M4_last_enc = M4_ENCODER.read();

  // set up DC motor PIDs with constants to control speed
  M1_PID.begin(&M1_current_speed, &M1_pid_out, &M1_target_speed, SPEED_KP, SPEED_KI, SPEED_KD);
  M1_PID.setOutputLimits(-255, 255);
  M1_PID.setWindUpLimits(-255, 255);
  M1_PID.start();

  M2_PID.begin(&M2_current_speed, &M2_pid_out, &M2_target_speed, SPEED_KP, SPEED_KI, SPEED_KD);
  M2_PID.setOutputLimits(-255, 255);
  M2_PID.setWindUpLimits(-255, 255);
  M2_PID.start();

  M3_PID.begin(&M3_current_speed, &M3_pid_out, &M3_target_speed, SPEED_KP, SPEED_KI, SPEED_KD);
  M3_PID.setOutputLimits(-255, 255);
  M3_PID.setWindUpLimits(-255, 255);
  M3_PID.start();

  M4_PID.begin(&M4_current_speed, &M4_pid_out, &M4_target_speed, SPEED_KP, SPEED_KI, SPEED_KD);
  M4_PID.setOutputLimits(-255, 255);
  M4_PID.setWindUpLimits(-255, 255);
  M4_PID.start();
}

void spin_motor(int motor, int input_speed)
{
  unsigned long current_time;
  long current_enc;
  unsigned long time_elapsed;
  long encoder_difference;

  switch (motor)
  {
    case 1:
      // set target motor speed
      M1_target_speed = input_speed;

      // read encoder value and find current motor speed
      current_time = millis();
      current_enc = M1_ENCODER.read();
      time_elapsed = current_time - M1_last_time;
      encoder_difference = current_enc - M1_last_enc; // do ticks -> rotations
      M1_current_speed = 1000 * (((float)encoder_difference * DEGREES_PER_TICK / (float) time_elapsed)); // degrees / second

      // update encoder/time values
      M1_last_time = current_time;
      M1_last_enc = current_enc;

      // perform PID using updated data and update speed
      M1_PID.compute();
      if (M1_pid_out > 0)
        M1_DRIVER.forward(M1_pid_out);
      else
        M1_DRIVER.back(abs(M1_pid_out));

      //      Serial.print("M1 time_elapsed: ");
      //      Serial.println(time_elapsed);
      //      Serial.print("M1 encoder_difference: ");
      //      Serial.println(encoder_difference);
      //      Serial.print("M1 target: ");
      //      Serial.println(M1_target_speed);
      //      Serial.print("M1 current: ");
      //      Serial.println(M1_current_speed);
      //      Serial.print("M1 PID: ");
      //      Serial.println(M1_pid_out);
      //      Serial.println("~~~~~~~~");
      break;

    case 2:
      // set target motor speed
      M2_target_speed = input_speed;

      // read encoder value and find current motor speed
      current_time = millis();
      current_enc = M2_ENCODER.read();
      time_elapsed = current_time - M2_last_time;
      encoder_difference = current_enc - M2_last_enc; // do ticks -> rotations
      M2_current_speed = 1000 * (((float)encoder_difference * DEGREES_PER_TICK / (float) time_elapsed)); // degrees / second

      // update encoder/time values
      M2_last_time = current_time;
      M2_last_enc = current_enc;

      // perform PID using updated data and update speed
      M2_PID.compute();
      if (M2_pid_out > 0)
        M2_DRIVER.forward(M2_pid_out);
      else
        M2_DRIVER.back(abs(M2_pid_out));

      // Serial.print("M2 time_elapsed: ");
      // Serial.println(time_elapsed);
      // Serial.print("M2 encoder_difference: ");
      // Serial.println(encoder_difference);
      // Serial.print("M2 target: ");
      // Serial.println(M2_target_speed);
      // Serial.print("M2 current: ");
      // Serial.println(M2_current_speed);
      // Serial.print("M2 PID: ");
      // Serial.println(M2_pid_out);
      // Serial.println("~~~~~~~~");
      break;

    case 3:
      // set target motor speed
      M3_target_speed = input_speed;

      // read encoder value and find current motor speed
      current_time = millis();
      current_enc = M3_ENCODER.read();
      time_elapsed = current_time - M3_last_time;
      encoder_difference = current_enc - M3_last_enc; // do ticks -> rotations
      M3_current_speed = 1000 * (((float)encoder_difference * DEGREES_PER_TICK / (float) time_elapsed)); // degrees / second

      // update encoder/time values
      M3_last_time = current_time;
      M3_last_enc = current_enc;

      // perform PID using updated data and update speed
      M3_PID.compute();
      if (M3_pid_out < 0)
        M3_DRIVER.forward(M3_pid_out);
      else
        M3_DRIVER.back(abs(M3_pid_out));

      // Serial.print("M3 time_elapsed: ");
      // Serial.println(time_elapsed);
      // Serial.print("M3 encoder_difference: ");
      // Serial.println(encoder_difference);
      // Serial.print("M3 target: ");
      // Serial.println(M3_target_speed);
      // Serial.print("M3 current: ");
      // Serial.println(M3_current_speed);
      // Serial.print("M3 PID: ");
      // Serial.println(M3_pid_out);
      // Serial.println("~~~~~~~~");
      break;

    case 4:
      // set target motor speed
      M4_target_speed = input_speed;

      // read encoder value and find current motor speed
      current_time = millis();
      current_enc = M4_ENCODER.read();
      time_elapsed = current_time - M4_last_time;
      encoder_difference = current_enc - M4_last_enc; // do ticks -> rotations
      M4_current_speed = 1000 * (((float)encoder_difference * DEGREES_PER_TICK / (float) time_elapsed)); // degrees / second

      // update encoder/time values
      M4_last_time = current_time;
      M4_last_enc = current_enc;

      // perform PID using updated data and update speed
      M4_PID.compute();
      if (M4_pid_out > 0)
        M4_DRIVER.forward(M4_pid_out);
      else
        M4_DRIVER.back(abs(M4_pid_out));

      // Serial.print("M4 time_elapsed: ");
      // Serial.println(time_elapsed);
      // Serial.print("M4 encoder_difference: ");
      // Serial.println(encoder_difference);
      // Serial.print("M4 target: ");
      // Serial.println(M4_target_speed);
      // Serial.print("M4 current: ");
      // Serial.println(M4_current_speed);
      // Serial.print("M4 PID: ");
      // Serial.println(M4_pid_out);
      // Serial.println("~~~~~~~~");
      break;

    default:
      break;
  }
}

// wait for serial data to come
uint8_t input_buffer_len = 0;
const int input_buffer_max_len = 256;
char input_buffer[input_buffer_max_len];
bool data_ready = false;
String GUI_input;

void serialEvent()
{
  input_buffer[input_buffer_len] = Serial.read();
  if (input_buffer[input_buffer_len] == '\n')
  {
    data_ready = true;
    input_buffer[input_buffer_len + 1] = '\0';
    GUI_input = String(input_buffer);
    input_buffer_len = 0;
  } else
    input_buffer_len++;
}

void loop()
{
  volatile int M1_new_speed, M2_new_speed, M3_new_speed, M4_new_speed = 0;

  // read serial input, if ready
  if (data_ready)
  {
    data_ready = false;
    int input_buffer_len = GUI_input.length() + 1;
    char input_buffer[input_buffer_len];
    GUI_input.toCharArray(input_buffer, input_buffer_len);

    // parse serial input: m1_speed m2_speed m3_speed m4_speed
    int result = sscanf(input_buffer, "%d %d %d %d\n",
                        &M1_new_speed, &M2_new_speed,
                        &M3_new_speed, &M4_new_speed);
  }

  // update motors if desired speed changed
  if (M1_current_speed != M1_new_speed)
    spin_motor(M1, M1_new_speed);

  if (M2_current_speed != M2_new_speed)
    spin_motor(M2, M2_new_speed);

  if (M3_current_speed != M3_new_speed)
    spin_motor(M3, M3_new_speed);

  if (M4_current_speed != M4_new_speed)
    spin_motor(M4, M4_new_speed);

  // Serial.println("~~~~~~~~~~");
  // Serial.print("M1 PID output: ");
  // Serial.println(M1_pid_out);
  // Serial.print("M2 PID output: ");
  // Serial.println(M2_pid_out);
  // Serial.print("M3 PID output: ");
  // Serial.println(M3_pid_out);
  // Serial.print("M4 PID output: ");
  // Serial.println(M4_pid_out);
  // Serial.println("~~~~~~~~~~");
}
