/*!
* 4 motors controlled via serial commands
* https://wiki.dfrobot.com/Quad_Motor_Driver_Shield_for_Arduino_SKU_DRI0039#More
*/

const int M1 = 1;
const int M2 = 2;
const int M3 = 3;
const int M4 = 4;

const int M1_SPEED = 3; ///<Motor1 Speed
const int M2_SPEED = 11;///<Motor2 Speed
const int M3_SPEED = 5; ///<Motor3 Speed
const int M4_SPEED = 6; ///<Motor4 Speed

const int M1_DIRECTION = 4; ///<Motor1 Direction
const int M2_DIRECTION = 12;///<Motor2 Direction
const int M3_DIRECTION = 8; ///<Motor3 Direction
const int M4_DIRECTION = 7; ///<Motor4 Direction

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

void spin_motor(int motor, int speed)
{
  switch (motor)
  {
    case 1:
      if (speed < 0)
      {
        digitalWrite(M1_DIRECTION, HIGH); // go backwards
        analogWrite(M1_SPEED, abs(speed)); // change speed
      }
      else
      {
        digitalWrite(M1_DIRECTION, LOW); // go forwards
        analogWrite(M1_SPEED, speed); // change speed
      }
    break;
    
    case 2:
      if (speed > 0)
      {
        digitalWrite(M2_DIRECTION, HIGH); // go forwards
        analogWrite(M2_SPEED, speed); // change speed
      }
      else
      {
        digitalWrite(M2_DIRECTION, LOW); // go backwards
        analogWrite(M2_SPEED, abs(speed)); // change speed
      }
    break;
    
    case 3:
      if (speed < 0)
      {
        digitalWrite(M3_DIRECTION, HIGH); // go backwards
        analogWrite(M3_SPEED, abs(speed)); // change speed
      }
      else
      {
        digitalWrite(M3_DIRECTION, LOW); // go forwards
        analogWrite(M3_SPEED, speed); // change speed
      }
    break;
    
    case 4:
      if (speed > 0)
      {
        digitalWrite(M4_DIRECTION, HIGH); // go forwards
        analogWrite(M4_SPEED, speed); // change speed
      }
      else
      {
        digitalWrite(M4_DIRECTION, LOW); // go backwards
        analogWrite(M4_SPEED, abs(speed)); // change speed
      }
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
