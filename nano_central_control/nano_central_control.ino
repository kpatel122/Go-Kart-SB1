/*
Copyright <2020> <K Patel>

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/* soft start motor ramping */
const byte  RAMP_FORWARD_END_SPEED  = 255;
const byte  RAMP_FORWARD_TIME_SECONDS = 4; // how long it takes to get to max speed

#define  RAMP_DECELERATE_TIME 1500 //how quickly we should stop- TODO needs to be a % of our speed


const byte RAMP_FORWARD_START_SPEED = 128;



#define  RAMP_FORWARD_RESOLUTION 1000 //how often the ramp speed is updated in millis

/* going from full speed to zero will cause a jerk for the user, so use a smooth stop*/
#define  RAMP_DECELERATE_RESOLUTION 1000 //how often the ramp speed is updated in millis

#define ENGINE_START_BUTTON_DEBOUNCE 1000

byte DecelerateStartSpeed = 0;

const byte  RAMP_REVERSE_START_SPEED = 128;
const byte  RAMP_REVERSE_END_SPEED = 200;
const byte  RAMP_REVERSE_TIME_SECONDS = 2;

byte RampStartSpeed = RAMP_FORWARD_START_SPEED;
byte RampEndSpeed = RAMP_FORWARD_END_SPEED;
const byte RampSpeedDelta = RAMP_FORWARD_END_SPEED - RAMP_FORWARD_START_SPEED;
const byte RampTimeSeconds = RAMP_FORWARD_TIME_SECONDS;

const byte MINIMUM_STATIONARY_TIME_SECONDS = 2;

unsigned long LastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long DebounceDelay = 200;
int LastPedalState = HIGH;



enum RAMP_STATE
{
	RAMP_NOT_READY,
	RAMP_READY,
	RAMP_STARTED_DECELERATING,
	RAMP_STARTED_ACCELERATING,
	RAMP_STOPPED,
	RAMP_ACCELERATING,
	RAMP_DECELERATING,
	RAMP_FINISHED_ACCELERATING,
	RAMP_FINISHED_DECELERATING
};

volatile RAMP_STATE RampState = RAMP_FINISHED_DECELERATING;


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

/* Cytron MD20A module*/
#define MOTOR_DRIVER_PWM_PIN 6
#define MOTOR_DRIVER_DIR_PIN 4

int ForwardMotorSpeed = 255;
int ReverseMotorSpeed = 255;
int CurrMotorSpeed = 0;

/*Serial*/
#define BAUD 115200
int IncomingSerialValue = 0;
String IncomingSerialString = "";

/*input switches*/
#define ACCELERATOR_PIN 2 //must be interrupt pin
#define ENGINE_START_BUTTON_PIN 3


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

GEAR currentGear = GEAR_FIRST;

void InitControlPins()
{
  pinMode(ACCELERATOR_PIN,INPUT); //external pullup
  pinMode(ENGINE_START_BUTTON_PIN,INPUT); //external pulldown
  pinMode(GEAR_SHIFT_UP_PIN, INPUT_PULLUP);
  pinMode(GEAR_SHIFT_DOWN_PIN, INPUT_PULLUP);
}

void InitMotorPins()
{
  pinMode(MOTOR_DRIVER_PWM_PIN,OUTPUT);
  digitalWrite(MOTOR_DRIVER_PWM_PIN,LOW);


  pinMode(MOTOR_DRIVER_DIR_PIN,OUTPUT);
  digitalWrite(MOTOR_DRIVER_DIR_PIN,LOW);
}

void SetMotorSpeed(int iNewSpeed)
{
  ForwardMotorSpeed = iNewSpeed;
}

void MotorForward()
{
	analogWrite(MOTOR_DRIVER_PWM_PIN,CurrMotorSpeed);
}

void MotorStop()
{
  digitalWrite(MOTOR_DRIVER_PWM_PIN,0);

}

void MotorReverse()
{
//	analogWrite(LPWM,CurrMotorSpeed);
}

void MotorNeutral()
{
//	  digitalWrite(L_EN,LOW); //free wheeling mode
//	  digitalWrite(MOTOR_DRIVER_DIR_PIN,LOW);
}

void MotorEnable()
{
	digitalWrite(MOTOR_DRIVER_DIR_PIN,HIGH);
}


void InterruptAcceleratorReleased()
{
	if(RampState != RAMP_DECELERATING ) //&& RampState !=RAMP_FINISHED)
	{
		RampState = RAMP_STARTED_DECELERATING;
		digitalWrite(LED_BUILTIN,HIGH);
	}
}

void ProcessRamp()
{
	static unsigned long starttime;
	unsigned long timedelta;
	static long resolutioncounter = 0;

	switch(RampState)
	{
		case RAMP_DECELERATING:
		{

			timedelta = millis() - starttime;
			CurrMotorSpeed = map(timedelta, 0, RAMP_DECELERATE_TIME, DecelerateStartSpeed, 0);

			 
			Serial.print("D Motor Speed ");
			Serial.println(CurrMotorSpeed);
			MotorForward();
						

			if(CurrMotorSpeed <= 0)
			{

				Serial.println("*DECELERATING Complete*");
				RampState = RAMP_FINISHED_DECELERATING;
				digitalWrite(LED_BUILTIN,LOW);

			}



		}break;
		case RAMP_ACCELERATING:
		{



			timedelta = millis() - starttime;
			CurrMotorSpeed = map(timedelta, 0, RAMP_FORWARD_TIME_SECONDS*1000, DecelerateStartSpeed, RampEndSpeed);

			 

			Serial.print("A Motor Speed ");
			Serial.println(CurrMotorSpeed);
			MotorForward();


			if(CurrMotorSpeed >= RampEndSpeed)
			{
				Serial.println("***********Ramp Complete******** ");
				RampState = RAMP_FINISHED_ACCELERATING;
			}

		}break;

		case RAMP_NOT_READY:
		{

		}break;
		case RAMP_READY:
		{

		}break;
		case RAMP_STARTED_ACCELERATING:
		{
			Serial.println("A");
			RampState = RAMP_ACCELERATING;
			starttime = millis();
			resolutioncounter = 0;
			timedelta = 0;
			 
			DecelerateStartSpeed = CurrMotorSpeed;
			pMotorMove();


		}break;
		case RAMP_STARTED_DECELERATING:
		{
			Serial.println("D");
			RampState = RAMP_DECELERATING;
			starttime = millis();
			resolutioncounter = 0;
			timedelta = 0;
			DecelerateStartSpeed = CurrMotorSpeed;

			//digitalWrite(LED_BUILTIN, HIGH);

			//RampState = RAMP_FINISHED;


		}break;
		case RAMP_STOPPED:
		{
			//Serial.println("S");
			//Serial.flush();
			//RampState = RAMP_FINISHED;

		}break;
		case RAMP_FINISHED_DECELERATING:
		case RAMP_FINISHED_ACCELERATING:
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
			if(currentGear==GEAR_FIRST){break;}

			MotorEnable();
			pMotorMove = MotorForward;
			currentGear = GEAR_FIRST;
			digitalWrite(MOTOR_DRIVER_DIR_PIN,LOW);
		}break;
		case GEAR_REVERSE:
		{
			if(currentGear==GEAR_REVERSE){break;}
			MotorEnable();
			pMotorMove = MotorReverse;
			currentGear = GEAR_REVERSE;
			digitalWrite(MOTOR_DRIVER_DIR_PIN,HIGH);
		}break;
		case GEAR_NEUTRAL:
		{
			MotorNeutral();
			pMotorMove = MotorNeutral;
			currentGear = GEAR_NEUTRAL; 
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

void EngineStartButtonISR()
{
  //Serial.println("B");
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  // If interrupts come faster than 200ms, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > ENGINE_START_BUTTON_DEBOUNCE) 
  {
	  //switch direction

     if(currentGear == GEAR_FIRST)
	 {
		 Serial.println("r");
		 ProcessScreenControlCommand(GEAR_REVERSE); 
	 }
	 else if (currentGear == GEAR_REVERSE)
	 {
		 Serial.println("f");
		 ProcessScreenControlCommand(GEAR_FIRST);
	 }
  }
  last_interrupt_time = interrupt_time;
}

int CheckButton()
{
	int value = digitalRead(ACCELERATOR_PIN) ;
	return value;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(BAUD);

  InitMotorPins();
  InitControlPins();

  pMotorMove = MotorNeutral; //start off in neutral

  attachInterrupt(digitalPinToInterrupt(ACCELERATOR_PIN), InterruptAcceleratorReleased, RISING);
  attachInterrupt(digitalPinToInterrupt(ENGINE_START_BUTTON_PIN), EngineStartButtonISR, FALLING);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  //tmp for testing only- gear shifts disabled
  MotorEnable();
  pMotorMove = MotorForward;

  RampState = RAMP_FINISHED_DECELERATING;
}

void loop()
{

 

 

  //throttle pressed and we have stopped
  if(CheckButton() == LOW && (RampState == RAMP_FINISHED_DECELERATING))
  {
	  //Serial.println("St A");
	  RampState = RAMP_STARTED_ACCELERATING;
  }
  ProcessRamp();

}
