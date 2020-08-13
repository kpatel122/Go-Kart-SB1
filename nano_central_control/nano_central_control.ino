/*
Copyright <2020> <K Patel>

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/* BTS7960 module*/
#define RPWM 5
#define LPWM 6
#define L_EN 7
#define R_EN 8
int ForwardMotorSpeed = 255;
int ReverseMotorSpeed = 255;

/*Serial*/
#define BAUD 115200
int IncomingSerialValue = 0;
String IncomingSerialString = "";

/*input switches*/
#define ACCELERATOR_PIN 2 //must be interrupt pin
#define GEAR_SHIFT_UP_PIN 3
#define GEAR_SHIFT_DOWN_PIN 4

/*GEAR states*/
enum GEAR
{
  NEUTRAL = 0,
  FIRST = 1,
  SECOND = 2,
  REVERSE = 3
};

enum GEAR_SHIFT
{
  GEAR_SHIFT_NONE =0,
  GEAR_SHIFT_UP = 1,
  GEAR_SHIFT_DOWN = 2
};

GEAR_SHIFT NextGearShift = GEAR_SHIFT_NONE;
GEAR CurrentGear = NEUTRAL;
GEAR NewGear = NEUTRAL;

void InitControlPins()
{
  pinMode(ACCELERATOR_PIN, INPUT_PULLUP);
  pinMode(GEAR_SHIFT_UP_PIN, INPUT_PULLUP);
  pinMode(GEAR_SHIFT_DOWN_PIN, INPUT_PULLUP);
}

void InitMotorPins()
{
  pinMode(RPWM,OUTPUT);
  digitalWrite(RPWM,LOW);

  pinMode(LPWM,OUTPUT);
  digitalWrite(LPWM,LOW);

  pinMode(L_EN,OUTPUT);
  digitalWrite(L_EN,LOW);

  pinMode(R_EN,OUTPUT);
  digitalWrite(R_EN,LOW);
}

void SetMotorSpeed(int iNewSpeed)
{
  ForwardMotorSpeed = iNewSpeed;
}

void MotorForward()
{

  digitalWrite(RPWM,ForwardMotorSpeed);
}

void MotorStop()
{
  digitalWrite(RPWM,0);
  digitalWrite(LPWM,0);
}

void MotorReverse()
{

  digitalWrite(LPWM,ReverseMotorSpeed);
}

void AcceleratorChange()
{
  if(digitalRead(ACCELERATOR_PIN) == HIGH)
  {
    MotorStop();
    digitalWrite(LED_BUILTIN, LOW);
  }
  else
  {
    digitalWrite(LED_BUILTIN, HIGH);
    MotorForward();

  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(BAUD);

  InitMotorPins();
  InitControlPins();

  digitalWrite(L_EN,HIGH);
  digitalWrite(R_EN,HIGH);

  attachInterrupt(digitalPinToInterrupt(ACCELERATOR_PIN), AcceleratorChange, CHANGE);
  pinMode(LED_BUILTIN, OUTPUT);
}

void ProcessInput(int ivalue)
{
  switch (ivalue)
  {
    case 'f':
    {
      MotorForward();
      Serial.println("forward");
    }break;
    case 'r':
    {
      MotorReverse();
      Serial.println("reverse");
    }break;
    case 's':
    {
      MotorStop();
      Serial.println("stop");
    }break;
    default:
    {
      Serial.println("invalid input");
    }
  }
}

bool CheckForGearShift()
{
  if(digitalRead(GEAR_SHIFT_UP_PIN) == LOW)
  {
    NextGearShift = GEAR_SHIFT_UP;
    return true;
  }
  else if(digitalRead(GEAR_SHIFT_DOWN_PIN) == LOW)
  {
    NextGearShift = GEAR_SHIFT_DOWN;
    return true;
  }

  return false;
}

void ProcessGearShift()
{
  if(NextGearShift == GEAR_SHIFT_UP)
  {
    if(CurrentGear == NEUTRAL)
    {
      CurrentGear = FIRST;
      Serial.println("UP FIRST");
      return;
    }
    if(CurrentGear == REVERSE)
    {
      CurrentGear = NEUTRAL;
      Serial.println("UP NEUTRAL");
      return;
    }
  }
  else if(NextGearShift == GEAR_SHIFT_DOWN)
  {
    if(CurrentGear == NEUTRAL)
    {
      CurrentGear = REVERSE;
      Serial.println("DOWN REVERSE");
      return;
    }
    if(CurrentGear == FIRST)
    {
      CurrentGear = NEUTRAL;
      Serial.println("DOWN NEUTRAL");
      return;
    }
  }

}

void loop() {

  if(Serial.available() > 0)
  {
	  IncomingSerialString = Serial.readString();
	  Serial.println("STM32: " + IncomingSerialString);

	  /*
	  if(IncomingByte == 1)
	  {
		  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
		  Serial.write(1);//ping back
	  }
	  if(IncomingByte == 2)
	  {
		  digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (HIGH is the voltage level)
		  Serial.write(2);//ping back
	  }
	  */

  }

}
