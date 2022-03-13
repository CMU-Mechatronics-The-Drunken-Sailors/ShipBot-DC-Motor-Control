/*!
* @file QuadMotorDriverShield.ino
* @brief QuadMotorDriverShield.ino  Motor control program
*
* Every 2 seconds to control motor positive inversion
*
* @author linfeng(490289303@qq.com)
* @version  V1.0
* @date  2016-4-5
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

void loop()
{
  spin_motor(M1, 100);
  spin_motor(M2, 100);
  spin_motor(M3, 100);
  spin_motor(M4, 100);
  delay(1000); ///<Delay 2S
  spin_motor(M1, -100);
  spin_motor(M2, -100);
  spin_motor(M3, -100);
  spin_motor(M4, -100);
  delay(1000); ///<Delay 2S
}
