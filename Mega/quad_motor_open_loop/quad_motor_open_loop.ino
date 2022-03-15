// Mechatronics - Spring 2022
// Runs on Arduino Mega with two DRV8833 motor drivers.
// Allows open-loop control of 4 motors.
// User sends PWM commands via serial communications.
// Uses this library: https://github.com/TheArduinist/DRV8833

#include <DRV8833.h>

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
const int M4_FORWARD = 3;
const int M4_BACKWARD = 2;

// motor objects (A for 1&2, B for 3&4)
DRV8833 DRIVER_A = DRV8833();
DRV8833 DRIVER_B = DRV8833();

// keep track of current speed/direction
int M1_current_speed, M2_current_speed, M3_current_speed, M4_current_speed = 0;

void setup()
{
  Serial.begin(115200);
  
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
  // read GUI input
  if (data_ready)
  {
    data_ready = false;
    int input_buffer_len = GUI_input.length() + 1;
    char input_buffer[input_buffer_len];
    GUI_input.toCharArray(input_buffer, input_buffer_len);

    // parse GUI input:
    // m1_speed m2_speed m3_speed m4_speed
    int M1_new_speed, M2_new_speed, M3_new_speed, M4_new_speed;
    int result = sscanf(input_buffer, "%d %d %d %d\n",
                        &M1_new_speed, &M2_new_speed,
                        &M3_new_speed, &M4_new_speed);

    // check for bad input
    if (result != 4)
    {
      Serial.println("Didn't enter correct format.");
      return;
    }

    // update motors if speed or direction changed
    if (M1_new_speed != M1_current_speed)
    {
      M1_current_speed = M1_new_speed;
      spin_motor(M1, M1_current_speed);
    }

    if (M2_new_speed != M2_current_speed)
    {
      M2_current_speed = M2_new_speed;
      spin_motor(M2, M2_current_speed);
    }

    if (M3_new_speed != M3_current_speed)
    {
      M3_current_speed = M3_new_speed;
      spin_motor(M3, M3_current_speed);
    }

    if (M4_new_speed != M4_current_speed)
    {
      M4_current_speed = M4_new_speed;
      spin_motor(M4, M4_current_speed);
    }

    Serial.println("~~~~~~~~~~");
    Serial.print("M1 speed: ");
    Serial.println(M1_current_speed);
    Serial.print("M2 speed: ");
    Serial.println(M2_current_speed);
    Serial.print("M3 speed: ");
    Serial.println(M3_current_speed);
    Serial.print("M4 speed: ");
    Serial.println(M4_current_speed);
    Serial.println("~~~~~~~~~~");
  }
}
