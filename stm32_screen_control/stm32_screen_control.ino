#include "Arduino.h"




#include <Adafruit_GFX.h>    // Core graphics library
#include <MCUFRIEND_kbv.h>   // Hardware-specific library
MCUFRIEND_kbv tft;

#include <FreeDefaultFonts.h>

#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSerif12pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h>

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


void showmsgXY(int x, int y, int sz, const GFXfont *f, const char *msg, int colour)
{
	tft.fillScreen(TFT_BLACK);
    int16_t x1, y1;
    uint16_t wid, ht;

    tft.setFont(f);
    tft.setCursor(x, y);
    tft.setTextColor(colour);
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
      Serial3.write(CurrentGear);
      showmsgXY(210, 200, 4, &FreeSansBold24pt7b, "F",TFT_GREEN);

      return;
    }
    if(CurrentGear == GEAR_REVERSE)
    {
      CurrentGear = GEAR_NEUTRAL;
      Serial3.write(CurrentGear);
      showmsgXY(210, 200, 4, &FreeSansBold24pt7b, "N",TFT_WHITE);
      return;
    }
  }
  else if(NextGearShift == GEAR_SHIFT_DOWN)
  {
    if(CurrentGear == GEAR_NEUTRAL)
    {
      CurrentGear = GEAR_REVERSE;
      Serial3.write(CurrentGear);
      showmsgXY(210, 200, 4, &FreeSansBold24pt7b, "R",TFT_RED);
      return;
    }
    if(CurrentGear == GEAR_FIRST)
    {
      CurrentGear = GEAR_NEUTRAL;
      Serial3.write(CurrentGear);
      showmsgXY(210, 200, 4, &FreeSansBold24pt7b, "N",TFT_WHITE);
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
    tft.setRotation(1);

    pinMode(LED_BUILTIN, OUTPUT);

    tft.fillScreen(TFT_BLACK);

    showmsgXY(210, 200, 4, &FreeSansBold24pt7b, "N",TFT_WHITE);
}

String IncomingSerialString = "";
byte IncomingByte;
void loop(void)
{
	if(CheckForGearShift() == true)
	{
		ProcessGearShift();
	}
}


