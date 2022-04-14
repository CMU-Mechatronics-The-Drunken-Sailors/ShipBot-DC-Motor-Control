// Mechatronics - Spring 2022
// Runs on Arduino Mega with two DRV8833 motor drivers.
// Spins 4 motors, forwards and backwards, and reads encoder data for PID feedback.
// Motors controlled via serial communications, giving commands of degrees/second.
//
// Uses these libraries:
// https://github.com/TheArduinist/DRV8833
// https://github.com/PaulStoffregen/Encoder
// https://github.com/PowerBroker2/ArduPID

#include <DRV8833.h>
#include <Encoder.h>
#include <ArduPID.h>

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

// motor encoder pins (interrupts for A)
const int M1_ENCODER_A = 3;
const int M1_ENCODER_B = 2;
const int M2_ENCODER_A = 18;
const int M2_ENCODER_B = 14;
const int M3_ENCODER_A = 19;
const int M3_ENCODER_B = 15;
const int M4_ENCODER_A = 20;
const int M4_ENCODER_B = 16;

// based on ticks per revolution
const float DEGREES_PER_TICK = (360.0 / 2797.0);

// DC motor 1's encoder and PID values
Encoder M1_ENCODER(M1_ENCODER_A, M1_ENCODER_B);
ArduPID M1_PID;

double M1_current_speed = 0;
double M1_target_speed = 0;
double M1_pid_out = 0;
unsigned long M1_last_time;
long M1_last_enc;

// DC motor 2's encoder and PID values
Encoder M2_ENCODER(M2_ENCODER_A, M2_ENCODER_B);
ArduPID M2_PID;

double M2_current_speed = 0;
double M2_target_speed = 0;
double M2_pid_out = 0;
unsigned long M2_last_time;
long M2_last_enc;

// DC motor 3's encoder and PID values
Encoder M3_ENCODER(M3_ENCODER_A, M3_ENCODER_B);
ArduPID M3_PID;

double M3_current_speed = 0;
double M3_target_speed = 0;
double M3_pid_out = 0;
unsigned long M3_last_time;
long M3_last_enc;

// DC motor 4's encoder and PID values
Encoder M4_ENCODER(M4_ENCODER_A, M4_ENCODER_B);
ArduPID M4_PID;

double M4_current_speed = 0;
double M4_target_speed = 0;
double M4_pid_out = 0;
unsigned long M4_last_time;
long M4_last_enc;

// PID motor constants
#define SPEED_KP 0.75
#define SPEED_KI 0.00
#define SPEED_KD 0.25

void setup()
{
  Serial.begin(115200);

  // encoder pins already set

  DRIVER_A.attachMotorA(M1_FORWARD, M1_BACKWARD); // M1 pins
  DRIVER_A.attachMotorB(M2_FORWARD, M2_BACKWARD); // M2 pins
  DRIVER_B.attachMotorA(M3_FORWARD, M3_BACKWARD); // M3 pins
  DRIVER_B.attachMotorB(M4_FORWARD, M4_BACKWARD); // M4 pins

  // start DC encoders' time/position
  M1_last_time = millis();
  M1_last_enc = M1_ENCODER.read();

  M2_last_time = millis();
  M2_last_enc = M2_ENCODER.read();

  M3_last_time = millis();
  M3_last_enc = M3_ENCODER.read();

  M4_last_time = millis();
  M4_last_enc = M4_ENCODER.read();

  delay(10); // wait for initialization

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
      M1_current_speed = 1000 * (((float)encoder_difference * DEGREES_PER_TICK) / ((float) time_elapsed)); // degrees / second

      // update encoder/time values
      M1_last_time = current_time;
      M1_last_enc = current_enc;

      // perform PID using updated data and update speed
      M1_PID.compute();
      if (M1_pid_out < 0)
        DRIVER_A.motorAForward(abs(M1_pid_out));
      else
        DRIVER_A.motorAReverse(abs(M1_pid_out));

      // debug data
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
      if (M2_pid_out < 0)
        DRIVER_A.motorBForward(abs(M2_pid_out));
      else
        DRIVER_A.motorBReverse(abs(M2_pid_out));

      // debug data
      //      Serial.print("M2 time_elapsed: ");
      //      Serial.println(time_elapsed);
      //      Serial.print("M2 encoder_difference: ");
      //      Serial.println(encoder_difference);
      //      Serial.print("M2 target: ");
      //      Serial.println(M2_target_speed);
      //      Serial.print("M2 current: ");
      //      Serial.println(M2_current_speed);
      //      Serial.print("M2 PID: ");
      //      Serial.println(M2_pid_out);
      //      Serial.println("~~~~~~~~");
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
        DRIVER_B.motorAForward(abs(M3_pid_out));
      else
        DRIVER_B.motorAReverse(abs(M3_pid_out));

      // debug data
      //      Serial.print("M3 time_elapsed: ");
      //      Serial.println(time_elapsed);
      //      Serial.print("M3 encoder_difference: ");
      //      Serial.println(encoder_difference);
      //      Serial.print("M3 target: ");
      //      Serial.println(M3_target_speed);
      //      Serial.print("M3 current: ");
      //      Serial.println(M3_current_speed);
      //      Serial.print("M3 PID: ");
      //      Serial.println(M3_pid_out);
      //      Serial.println("~~~~~~~~");
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
      if (M4_pid_out < 0)
        DRIVER_B.motorBForward(abs(M4_pid_out));
      else
        DRIVER_B.motorBReverse(abs(M4_pid_out));

      // debug data
      //      Serial.print("M4 time_elapsed: ");
      //      Serial.println(time_elapsed);
      //      Serial.print("M4 encoder_difference: ");
      //      Serial.println(encoder_difference);
      //      Serial.print("M4 target: ");
      //      Serial.println(M4_target_speed);
      //      Serial.print("M4 current: ");
      //      Serial.println(M4_current_speed);
      //      Serial.print("M4 PID: ");
      //      Serial.println(M4_pid_out);
      //      Serial.println("~~~~~~~~");
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
String serial_input;

void serialEvent()
{
  input_buffer[input_buffer_len] = Serial.read();
  if (input_buffer[input_buffer_len] == '\n')
  {
    data_ready = true;
    input_buffer[input_buffer_len + 1] = '\0';
    serial_input = String(input_buffer);
    input_buffer_len = 0;
  } else
    input_buffer_len++;
}

int M1_new_speed = 0, M2_new_speed = 0, M3_new_speed = 0, M4_new_speed = 0;

void loop()
{
  // read serial input, if ready
  if (data_ready)
  {
    data_ready = false;
    int input_buffer_len = serial_input.length() + 1;
    char input_buffer[input_buffer_len];
    serial_input.toCharArray(input_buffer, input_buffer_len);

    // parse serial input: m1_speed m2_speed m3_speed m4_speed
    int result = sscanf(input_buffer, "%d %d %d %d\n",
                        &M1_new_speed, &M2_new_speed,
                        &M3_new_speed, &M4_new_speed);

    Serial.println("OK");
  }

  // update motor speeds, correcting for wheel orientation
  spin_motor(M1, -1 * M1_new_speed);
  spin_motor(M2, M2_new_speed);
  spin_motor(M3, -1 * M3_new_speed);
  spin_motor(M4, M4_new_speed);

  delay(10); // don't go too fast!

  // plot PID outputs for tuning
  //  Serial.println(M1_pid_out);
  //  Serial.print(" ");
  //  Serial.print(M2_pid_out);
  //  Serial.print(" ");
  //  Serial.print(M3_pid_out);
  //  Serial.print(" ");
  //  Serial.print(M4_pid_out);
  //  Serial.println();
}
