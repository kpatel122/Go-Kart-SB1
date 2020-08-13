#include "Arduino.h"

#define BLACK   0x0000
#define RED     0xF800
#define GREEN   0x07E0
#define WHITE   0xFFFF
#define GREY    0x8410


#include <Adafruit_GFX.h>    // Core graphics library
#include <MCUFRIEND_kbv.h>   // Hardware-specific library
MCUFRIEND_kbv tft;

#include <FreeDefaultFonts.h>

#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSerif12pt7b.h>

//LCD pins  |D7 |D6 |D5 |D4 |D3 |D2 |D1 |D0 | |RD |WR |RS |CS |RST| |SD_SS|SD_DI|SD_DO|SD_SCK|
//STM32 pin |PA7|PA6|PA5|PA4|PA3|PA2|PA1|PA0| |PB0|PB6|PB7|PB8|PB9| |PA15 |PB5  |PB4  |PB3   | **ALT-SPI1**

/*Gear shifters*/
#define GEAR_SHIFT_UP_PIN PB12
#define GEAR_SHIFT_DOWN_PIN PB13

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
GEAR CurrentGear = GEAR_NEUTRAL;
GEAR NewGear = GEAR_NEUTRAL;


/*Serial*/
#define BAUD 115200

void InitControlPins()
{
  pinMode(GEAR_SHIFT_UP_PIN, INPUT_PULLUP);
  pinMode(GEAR_SHIFT_DOWN_PIN, INPUT_PULLUP);
}


void showmsgXY(int x, int y, int sz, const GFXfont *f, const char *msg)
{
    int16_t x1, y1;
    uint16_t wid, ht;
    tft.drawFastHLine(0, y, tft.width(), WHITE);
    tft.setFont(f);
    tft.setCursor(x, y);
    tft.setTextColor(GREEN);
    tft.setTextSize(sz);
    tft.print(msg);
    delay(1000);
}

bool CheckForGearShift()
{


  if(digitalRead(GEAR_SHIFT_UP_PIN) == LOW)
  {
    NextGearShift = GEAR_SHIFT_UP;
    digitalWrite(LED_BUILTIN, LOW);
    return true;
  }
  else if(digitalRead(GEAR_SHIFT_DOWN_PIN) == LOW)
  {
    NextGearShift = GEAR_SHIFT_DOWN;
    digitalWrite(LED_BUILTIN, HIGH);
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
      Serial3.println("UP FIRST");
      return;
    }
    if(CurrentGear == GEAR_REVERSE)
    {
      CurrentGear = GEAR_NEUTRAL;
      Serial3.println("UP NEUTRAL");
      return;
    }
  }
  else if(NextGearShift == GEAR_SHIFT_DOWN)
  {
    if(CurrentGear == GEAR_NEUTRAL)
    {
      CurrentGear = GEAR_REVERSE;
      Serial3.println("DOWN REVERSE");
      return;
    }
    if(CurrentGear == GEAR_FIRST)
    {
      CurrentGear = GEAR_NEUTRAL;
      Serial3.println("DOWN NEUTRAL");
      return;
    }
  }

}

void setup(void)
{
    Serial.begin(BAUD);

    Serial3.begin(BAUD);

    InitControlPins();

    uint16_t ID = tft.readID();
    Serial.println("Example: Font_simple");
    Serial.print("found ID = 0x");
    Serial.println(ID, HEX);
    if (ID == 0xD3D3) ID = 0x9481; //force ID if write-only display
    tft.begin(ID);
    tft.setRotation(0);

    pinMode(LED_BUILTIN, OUTPUT);
}

String IncomingSerialString = "";
byte IncomingByte;
void loop(void)
{

	if(CheckForGearShift() == true)
	{
		ProcessGearShift();
		//Serial3.println("GEAR CHANGE");
	}

	   //Serial3.println("Test");
	   //delay(1000);

	/*
	uint16_t ID = tft.readID();



	Serial3.write(1);
	//Serial3.flush();
	delay(5000);

	if(Serial3.available() > 0)
		  {

			IncomingByte = Serial3.read();

			  if(IncomingByte == 1)
			  {
				  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)

			  }
			  if(IncomingByte == 2)
			  {
				  digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (HIGH is the voltage level)

			  }
		  }


	Serial3.write(2);
	//Serial3.flush();
	delay(5000);


	if(Serial3.available() > 0)
		  {

			 IncomingByte = Serial3.read();

			  if(IncomingByte == 1)
			  {
				  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)

			  }
			  if(IncomingByte == 2)
			  {
				  digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (HIGH is the voltage level)

			  }
		  }

	*/



/*
    tft.fillScreen(BLACK);
    showmsgXY(20, 10, 1, NULL, "System x1");
    showmsgXY(20, 24, 2, NULL, "System x2");
    showmsgXY(20, 60, 1, &FreeSans9pt7b, "FreeSans9pt7b");
    showmsgXY(20, 80, 1, &FreeSans12pt7b, "FreeSans12pt7b");
    showmsgXY(20, 100, 1, &FreeSerif12pt7b, "FreeSerif12pt7b");
    showmsgXY(20, 120, 1, &FreeSmallFont, "FreeSmallFont");
    showmsgXY(5, 180, 1, &FreeSevenSegNumFont, "01234");
    showmsgXY(5, 190, 1, NULL, "System Font is drawn from topline");
    tft.setTextColor(RED, GREY);
    tft.setTextSize(2);
    tft.setCursor(0, 220);
    tft.print("7x5 can overwrite");
    delay(1000);
    tft.setCursor(0, 220);
    tft.print("if background set");
    delay(1000);
    showmsgXY(5, 260, 1, &FreeSans9pt7b, "Free Fonts from baseline");
    showmsgXY(5, 285, 1, &FreeSans9pt7b, "Free Fonts transparent");
    delay(1000);
    showmsgXY(5, 285, 1, &FreeSans9pt7b, "Free Fonts XXX");
    delay(1000);
    showmsgXY(5, 310, 1, &FreeSans9pt7b, "erase backgnd with fillRect()");
    delay(10000);
    */

}


