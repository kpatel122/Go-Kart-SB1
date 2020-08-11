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

void InitPins()
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



void setup() {
  // put your setup code here, to run once:
  Serial.begin(BAUD);

  InitPins();

  digitalWrite(L_EN,HIGH);
  digitalWrite(R_EN,HIGH);
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

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available() > 0)
  {
    IncomingSerialValue = Serial.read();
    ProcessInput(IncomingSerialValue);
  }

}
