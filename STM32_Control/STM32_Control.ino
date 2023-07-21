#include "Arduino.h"



#include "DFRobotDFPlayerMini.h"
#include <Adafruit_GFX.h>    // Core graphics library
#include <MCUFRIEND_kbv.h>   // Hardware-specific library
MCUFRIEND_kbv tft;

#include <FreeDefaultFonts.h>

#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSerif12pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h>

//http://www.lcdwiki.com/3.5inch_Arduino_Display-UNO
//LCD pins  |D7 |D6 |D5 |D4 |D3 |D2 |D1 |D0 | |LCD_RD |LCD_WR |LCD_RS |LCD_CS |LCD_RST| |SD_SS|SD_DI|SD_DO|SD_SCK|
//STM32 pin |PA7|PA6|PA5|PA4|PA3|PA2|PA1|PA0| |PB0    |PB6    |PB7    |PB8    |PB9    | |PA15 |PB5  |PB4  |PB3   | **ALT-SPI1**
//Inputs Gear up | Gear Down
//       PB12    | PB13
//Serial3 TX   | RX
//        PB10 | PB11 //speaker
//Motor PWM PA8
//Motor Dir PB15
//Throttle  PB14
//Engine Button PA9
//IN1  PB1
//IN2  PC14
//IN3  PC15

/*Gear shifters*/
#define GEAR_SHIFT_UP_PIN PB12
#define GEAR_SHIFT_DOWN_PIN PB13

/*buttons*/
#define ENGINE_START_BUTTON_PIN PA9

#define THROTTLE_PIN PB14
#define THROTTLE_PRESSED LOW //throttle switch is active low
#define THROTTLE_NOT_PRESSED HIGH //throttle switch is active low

#define MOTOR_DRIVER_DIR_PIN PB15
#define MOTOR_DRIVER_PWM_PIN PA8

/* Gears */
enum GEAR
{
  GEAR_REVERSE = 0,
  GEAR_NEUTRAL = 1,
  GEAR_FIRST = 2,
  GEAR_SECOND = 3

};

enum GEAR_SHIFT
{
  GEAR_SHIFT_NONE =0,
  GEAR_SHIFT_UP = 1,
  GEAR_SHIFT_DOWN = 2
};

GEAR_SHIFT NextGearShift = GEAR_SHIFT_NONE;
GEAR CurrentGear = GEAR_FIRST;//GEAR_NEUTRAL;
GEAR NewGear = GEAR_NEUTRAL;

/*Serial*/
#define BAUD 115200

/*Sound player*/
DFRobotDFPlayerMini DFPlayer;
#define DFPLAYER_BAUD 9600
bool IsEngineSamplePlaying = false;


/*Throttle ramp control */
/* min speed parameters- soft start motor ramping */
const byte  RAMP_FORWARD_END_SPEED  = 150; //max 255

const byte RAMP_FORWARD_START_SPEED = 120;
const byte  RAMP_FORWARD_TIME_SECONDS = 2; // how long it takes to get to max speed

const byte  RAMP_REVERSE_END_SPEED = 150;


const byte  RAMP_REVERSE_START_SPEED = 120;
const byte  RAMP_REVERSE_TIME_SECONDS = 2;

//stop instantly would cause a jerk- smooth stop instead
 #define  RAMP_DECELERATE_TIME 1000 //how quickly we should stop- TODO needs to be a % of our speed

byte RampStartSpeed = RAMP_FORWARD_START_SPEED;
byte RampEndSpeed = RAMP_FORWARD_END_SPEED;
byte RampTimeSeconds = RAMP_FORWARD_TIME_SECONDS;

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

volatile RAMP_STATE RampState = RAMP_FINISHED_DECELERATING; //gets changed in ISR


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

volatile KART_STATE CurrState = STATE_OFF;
KART_STATE PrevState = STATE_OFF;

//stores the current speed of the motor
int CurrMotorSpeed = 0;

//Motor must be wired in the following way for these to be valid:
//MOTOR->CRYTON DRIVER
//MOTOR RED LEAD->MA
//MOTOR BLACK LEAD->MB
#define MOTOR_FORWARD HIGH
#define MOTOR_BACKWARD LOW


#define LED_ON LOW
#define LED_OFF HIGH


//debug LED blink
void blink(int num, int ms)
{
	for(int i=0;i<num;i++)
	{
		digitalWrite(LED_BUILTIN, LOW);
		delay(ms);
		digitalWrite(LED_BUILTIN, HIGH);
		delay(ms);
	}
	
}

void InitSoundPlayer()
{
	Serial3.begin(DFPLAYER_BAUD); 
	if (!DFPlayer.begin(Serial3)) {  //Use softwareSerial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    ScreenLog("Sound Not OK");
    //blink(10,500);
  }
  else
  {
    //blink(5,1000);
    Serial.println(F("DFPlayer Mini online."));
    ScreenLog("Sound OK!");
     
  }  
}


void InitGearShiftPins()
{
  pinMode(GEAR_SHIFT_UP_PIN, INPUT_PULLUP);
  pinMode(GEAR_SHIFT_DOWN_PIN, INPUT_PULLUP);
}

void ScreenLog(const char *msg)
{
   static int logX = 10;
   static int logY = 20;
   static int sz = 1;
 

    tft.setFont(&FreeSans9pt7b);
    tft.setCursor(logX, logY);
    tft.setTextColor(TFT_RED);
    tft.setTextSize(sz);
    tft.print(msg);

    logY += 20;

    
}

void UpdateScreenGear(const char *msg, int colour)
{

  static int gearPosX = 210;
   static int gearPosY = 200;

  //clear the old gear text
  tft.fillRect(gearPosX+10, 60, 125, 150, TFT_BLACK);
     
    tft.setFont(&FreeSansBold24pt7b);
    tft.setCursor(gearPosX, gearPosY);
    tft.setTextColor(colour);
    tft.setTextSize(4);
    tft.print(msg);
}

void showmsgXY(int x, int y, int sz, const GFXfont *f, const char *msg, int colour)
{
	tft.fillScreen(TFT_BLACK);

      

    tft.setFont(f);
    tft.setCursor(x, y);
    tft.setTextColor(colour);
    tft.setTextSize(sz);
    tft.print(msg);
    
    //delay(1000);
}

bool CheckForGearShift()
{


  if(digitalRead(GEAR_SHIFT_UP_PIN) == LOW)
  {
    NextGearShift = GEAR_SHIFT_UP;
    //digitalWrite(LED_BUILTIN, HIGH);
    return true;
  }
  else if(digitalRead(GEAR_SHIFT_DOWN_PIN) == LOW)
  {
    NextGearShift = GEAR_SHIFT_DOWN;
    //digitalWrite(LED_BUILTIN, HIGH);
    return true;
  }


  return false;
}

void ProcessGearShift()
{
  if(NextGearShift == GEAR_SHIFT_UP)
  {
    if(CurrentGear == GEAR_NEUTRAL)
    {
      CurrentGear = GEAR_FIRST;
      //Serial3.write(CurrentGear);
      //showmsgXY(210, 200, 4, &FreeSansBold24pt7b, "F",TFT_GREEN);
      UpdateScreenGear("F",TFT_GREEN);

      return;
    }
    if(CurrentGear == GEAR_REVERSE)
    {
      CurrentGear = GEAR_NEUTRAL;
      //Serial3.write(CurrentGear);
      //showmsgXY(210, 200, 4, &FreeSansBold24pt7b, "N",TFT_WHITE);
      UpdateScreenGear("N",TFT_WHITE);
      return;
    }
  }
  else if(NextGearShift == GEAR_SHIFT_DOWN)
  {
    if(CurrentGear == GEAR_NEUTRAL)
    {
      CurrentGear = GEAR_REVERSE;
      //Serial3.write(CurrentGear);
      //showmsgXY(210, 200, 4, &FreeSansBold24pt7b, "R",TFT_RED);
      UpdateScreenGear("R",TFT_RED);
      return;
    }
    if(CurrentGear == GEAR_FIRST)
    {
      CurrentGear = GEAR_NEUTRAL;
      //Serial3.write(CurrentGear);
      //showmsgXY(210, 200, 4, &FreeSansBold24pt7b, "N",TFT_WHITE);
      UpdateScreenGear("N",TFT_WHITE);
      return;
    }
  }
}

void InitScreen()
{
    uint16_t ID = tft.readID();
    //Serial.println("Example: Font_simple");
    //Serial.print("found ID = 0x");
    //Serial.println(ID, HEX);
    if (ID == 0xD3D3) ID = 0x9481; //force ID if write-only display
    tft.begin(ID);
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
}

 

void InitMotorPins()
{
   //cryton module
  pinMode(MOTOR_DRIVER_PWM_PIN,OUTPUT);
  digitalWrite(MOTOR_DRIVER_PWM_PIN,LOW);
  pinMode(MOTOR_DRIVER_DIR_PIN,OUTPUT);
  digitalWrite(MOTOR_DRIVER_DIR_PIN,LOW);
}

void ThrottleReleasedISR()
{
  digitalWrite(LED_BUILTIN, LED_OFF);
	if(RampState != RAMP_DECELERATING )//make sure we are not already decelarating
	{
		RampState = RAMP_STARTED_DECELERATING;//ramp down the speed
	}
  ScreenLog("Stop");
  
}

void MotorForward()
{
	analogWrite(MOTOR_DRIVER_PWM_PIN,CurrMotorSpeed);
}

void MotorEnable()
{
	digitalWrite(MOTOR_DRIVER_DIR_PIN,HIGH);
}

void InitISR()
{
  pinMode(THROTTLE_PIN,INPUT_PULLUP); 
	//initialise interrupts
	
	
	//attachInterrupt(digitalPinToInterrupt(THROTTLE_PIN), ThrottleReleasedISR, RISING);

}

void ProcessRamp()
{
	static unsigned long starttime; //when the ramp was started
	unsigned long timedelta; //time difference from when the ramp was started 
	static long resolutioncounter = 0; //resolution of ramp

	switch(RampState)
	{
		case RAMP_DECELERATING:
		{

			//get the current delta from when the decelarion was started
			timedelta = millis() - starttime;

			//motor speed is mapped against the decelation time- TODO make this is percentage
			//of current speed
			CurrMotorSpeed = map(timedelta, 0, RAMP_DECELERATE_TIME, RampStartSpeed, 0);

			//continue to move at the slower decelaration speed to avoif sudden jerky stop
			MotorForward();
						
			//cehck if decelaration has finished
			if(CurrMotorSpeed <= 0)
			{
				RampState = RAMP_FINISHED_DECELERATING;
			}
		}break;
		case RAMP_ACCELERATING:
		{
			/*
			Serial.print("RampEndSpeed ");
			Serial.print(RampEndSpeed);
			Serial.print(" CurrMotorSpeed ");
			Serial.println(CurrMotorSpeed);
*/
			//this could be going forward or backward, the speed profiles for either is set SetRampProfile
			//called in the gear change
			
			//get time delta from current time to when the throttle was pressed
			timedelta = millis() - starttime;
			CurrMotorSpeed = map(timedelta, 0, RampTimeSeconds*1000, RampStartSpeed, RampEndSpeed);

			MotorForward();

			if(CurrMotorSpeed >= RampEndSpeed)
			{
				//Serial.println("***********Ramp Complete******** ");
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
			 
			RampState = RAMP_ACCELERATING;
			starttime = millis();
			resolutioncounter = 0;
			timedelta = 0;
			 
			RampStartSpeed = CurrMotorSpeed;
			MotorForward();//pMotorMove();


		}break;
		case RAMP_STARTED_DECELERATING:
		{
			//setup decelarating values
			RampState = RAMP_DECELERATING;
			starttime = millis();
			resolutioncounter = 0;
			timedelta = 0;
			RampStartSpeed = CurrMotorSpeed;

		}break;
		case RAMP_STOPPED:
		{

		}break;
		case RAMP_FINISHED_DECELERATING:
		case RAMP_FINISHED_ACCELERATING:
		{


		}break;
	}

}

void setup(void)
{
    Serial.begin(BAUD);

    

    InitGearShiftPins();

    
    InitScreen();
    pinMode(LED_BUILTIN, OUTPUT);
    
    digitalWrite(LED_BUILTIN,HIGH); //active low

    InitSoundPlayer();


    InitMotorPins();
    InitISR();

    UpdateScreenGear("N",TFT_WHITE);
     
    
}

inline int CheckThrottle()
{
	return digitalRead(THROTTLE_PIN) ;
}

String IncomingSerialString = "";
byte IncomingByte;
void loop(void)
{

  if(CheckForGearShift() == true)
	{
		ProcessGearShift();
	}

  //throttle pressed and we have stopped
  if(CheckThrottle() == THROTTLE_PRESSED && (RampState == RAMP_FINISHED_DECELERATING)) //&& (CurrState!= STATE_OFF))
  {
	  RampState = RAMP_STARTED_ACCELERATING;
    //ScreenLog("go");
    digitalWrite(LED_BUILTIN, LED_ON);
    //ScreenLog("Vroom");
     
  }
  else if(CheckThrottle() == THROTTLE_NOT_PRESSED && ((RampState != RAMP_FINISHED_DECELERATING) || RampState != RAMP_FINISHED_DECELERATING) )
  {
    ThrottleReleasedISR();
  }

  ProcessRamp(); //process movement
  
}
