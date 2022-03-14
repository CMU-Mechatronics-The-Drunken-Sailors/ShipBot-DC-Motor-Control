// Mechatronics - Spring 2022
// Runs on Teensy 4.1 with two L293 motor drivers.
// Allows open-loop control of 4 motors.
// User sends PWM commands via serial communications.
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

// keep track of current speed/direction
int M1_current_speed, M2_current_speed, M3_current_speed, M4_current_speed = 0;

void setup()
{
  Serial.begin(9600);
  
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
