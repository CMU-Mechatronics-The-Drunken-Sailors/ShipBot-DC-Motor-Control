/* Encoder Library - Basic Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * Reads motor encoders and prints to console.
 */

#include <Encoder.h>

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

const int M1_ENCODER_A = 2;
const int M1_ENCODER_B = A1;
const int M2_ENCODER_A = A2;
const int M2_ENCODER_B = A3;
const int M3_ENCODER_A = A4;
const int M3_ENCODER_B = A5;
const int M4_ENCODER_A = 9;
const int M4_ENCODER_B = 10;

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder myEnc(M1_ENCODER_A, M1_ENCODER_B);
//   avoid using pins with LEDs attached

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

long oldPosition  = -999;

void readEncoder()
{
  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    Serial.println(newPosition);
  }
}

void loop() 
{
  // drive motor and read encoder values
  spin_motor(M1, -100);
  // spin_motor(M2, -100);
  // spin_motor(M3, -100);
  // spin_motor(M4, -100);
  readEncoder();
}
