/*
Copyright <2020> <K Patel>

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


/* soft start motor ramping */
const byte RAMP_FORWARD_START_SPEED = 128;
const byte  RAMP_FORWARD_END_SPEED  = 255;
const byte  RAMP_FORWARD_TIME_SECONDS = 5;

const byte  RAMP_REVERSE_START_SPEED = 128;
const byte  RAMP_REVERSE_END_SPEED = 200;
const byte  RAMP_REVERSE_TIME_SECONDS = 2;

byte RampStartSpeed = RAMP_FORWARD_START_SPEED;
byte RampEndSpeed = RAMP_FORWARD_END_SPEED;
byte RampSpeedDelta = RAMP_FORWARD_END_SPEED - RAMP_FORWARD_START_SPEED;
byte RampTimeSeconds = RAMP_FORWARD_TIME_SECONDS;

const byte  MINIMUM_STATIONARY_TIME_SECONDS = 2;

enum RAMP_STATE
{
	RAMP_NOT_READY,
	RAMP_READY,
	RAMP_STARTED,
	RAMP_STOPPED,
	RAMP_PROCESSING,
	RAMP_FINISHED
};

volatile RAMP_STATE RampState = RAMP_NOT_READY;


/* kart state machine */
enum KART_STATE
{
  STATE_OFF,
  STATE_IGNITION_START,
  STATE_READY,
  STATE_RAMPING,
  STATE_MOVING_FORWARD,
  STATE_MOVING_REVERSE,
  STATE_NOT_MOVING
};

KART_STATE CurrState = STATE_OFF;
KART_STATE PrevState = STATE_OFF;

/* BTS7960 module*/
#define RPWM 5
#define LPWM 6
#define L_EN 7
#define R_EN 8
int ForwardMotorSpeed = 255;
int ReverseMotorSpeed = 255;
int CurrMotorSpeed = 0;

/*Serial*/
#define BAUD 115200
int IncomingSerialValue = 0;
String IncomingSerialString = "";

/*input switches*/
#define ACCELERATOR_PIN 2 //must be interrupt pin
#define GEAR_SHIFT_UP_PIN 3
#define GEAR_SHIFT_DOWN_PIN 4

/* move forwards or reverse function pointer */
void (*pMotorMove)();

/*data from the screen controller*/
byte incomingByte;

/*GEAR states*/
enum GEAR
{
	GEAR_REVERSE = 0,
	GEAR_NEUTRAL = 1,
	GEAR_FIRST = 2,
	GEAR_SECOND = 3
};

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



	analogWrite(RPWM,CurrMotorSpeed);
}

void MotorStop()
{
  digitalWrite(RPWM,0);
  digitalWrite(LPWM,0);
}

void MotorReverse()
{

	analogWrite(LPWM,CurrMotorSpeed);
}

void MotorNeutral()
{
	  digitalWrite(L_EN,LOW); //free wheeling mode
	  digitalWrite(R_EN,LOW);
}

void MotorEnable()
{
	digitalWrite(L_EN,HIGH);
	digitalWrite(R_EN,HIGH);
}

void InterruptAcceleratorChange()
{
  int value = digitalRead(ACCELERATOR_PIN) ;
  if(value== HIGH)
  {

    MotorStop();
    RampState = RAMP_FINISHED;

  }
  else if(value == LOW)
  {

	  RampState = RAMP_STARTED;
  }
}

void ProcessRamp()
{
	static unsigned long starttime;
	unsigned long timedelta;
	static int secondscounter = 0;

	switch(RampState)
	{
		case RAMP_PROCESSING:
		{
			//Serial.print("RAMP_PROCESSING ");
			timedelta = millis() - starttime;



			if((int)(timedelta / 1000) >=1)
			{
				secondscounter++;


				//reset the time countr
				starttime = millis();
				timedelta = 0;

				CurrMotorSpeed = map(secondscounter, 0, RampTimeSeconds, RampStartSpeed, RampEndSpeed);

				Serial.print("Seconds ");
				Serial.println(secondscounter);

				Serial.print("Motor Speed ");
				Serial.println(CurrMotorSpeed);

				MotorForward();


			}

			if(secondscounter >= RampTimeSeconds)
			{
				Serial.println("***********Ramp Complete******** ");
				RampState = RAMP_FINISHED;
			}



			//byte RampStartSpeed = RAMP_FORWARD_START_SPEED;
			//byte RampEndSpeed = RAMP_FORWARD_END_SPEED;
			//byte RampSpeedDelta = RAMP_FORWARD_END_SPEED - RAMP_FORWARD_START_SPEED;
			//byte RampTimeSeconds = RAMP_FORWARD_TIME_SECONDS;


			//Serial.println("setting: RAMP_STOPPED ");
			//RampState = RAMP_STOPPED;

		}break;

		case RAMP_NOT_READY:
		{

		}break;
		case RAMP_READY:
		{

		}break;
		case RAMP_STARTED:
		{
			Serial.print("RAMP_STARTED ");
			Serial.println("setting: RAMP_PROCESSING ");
			RampState = RAMP_PROCESSING;
			starttime = millis();
			secondscounter = 0;
			timedelta = 0;
			CurrMotorSpeed = RampStartSpeed;
			pMotorMove();

		}break;
		case RAMP_STOPPED:
		{
			//Serial.println("S");
			//Serial.flush();
			//RampState = RAMP_FINISHED;

		}break;
		case RAMP_FINISHED:
		{

		}break;
	}

}


void ProcessScreenControlCommand(byte iCommand)
{
	switch(iCommand)
	{
		case GEAR_FIRST:
		{
			MotorEnable();
			pMotorMove = MotorForward;
		}break;
		case GEAR_REVERSE:
		{
			MotorEnable();
			pMotorMove = MotorReverse;
		}break;
		case GEAR_NEUTRAL:
		{
			MotorNeutral();
			pMotorMove = MotorNeutral;
		}break;
	}
}

void SetState(byte iNewState) //param data type should be KART_STATE but sloeber does not seem to handle enums as params
{
	PrevState = CurrState;
	CurrState = iNewState;
}

void ProcessState()
{

	switch(CurrState)
	{
		case STATE_OFF:
		{
			// wait for engine start press and play the sound
			SetState(STATE_IGNITION_START);

		}break;
		case STATE_IGNITION_START:
		{
			SetState(STATE_READY);

		}break;
		case STATE_READY:
		{

		}break;
		case STATE_RAMPING:
		{

		}break;
		case STATE_MOVING_FORWARD:
		{

		}break;

		case STATE_MOVING_REVERSE:
		{

		}break;
		case STATE_NOT_MOVING:
		{

		}break;

	}

}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(BAUD);

  InitMotorPins();
  InitControlPins();

  pMotorMove = MotorNeutral; //start off in neutral

  attachInterrupt(digitalPinToInterrupt(ACCELERATOR_PIN), InterruptAcceleratorChange, CHANGE);
  pinMode(LED_BUILTIN, OUTPUT);

  //tmp for testing only
  MotorEnable();
  pMotorMove = MotorForward;
}

void loop()
{

  if(Serial.available() > 0)
  {
	  //incomingByte = Serial.read();
	  //ProcessScreenControlCommand(incomingByte);

  }

  //ProcessState();
  ProcessRamp();

}
