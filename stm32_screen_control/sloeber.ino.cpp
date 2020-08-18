#ifdef __IN_ECLIPSE__
//This is a automatic generated file
//Please do not modify this file
//If you touch this file your change will be overwritten during the next build
//This file has been generated on 2020-08-18 09:35:07

#include "Arduino.h"
#include "Arduino.h"
#include <Adafruit_GFX.h>
#include <MCUFRIEND_kbv.h>
extern MCUFRIEND_kbv tft;
#include <FreeDefaultFonts.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSerif12pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h>

void InitControlPins() ;
void showmsgXY(int x, int y, int sz, const GFXfont *f, const char *msg, int colour) ;
bool CheckForGearShift() ;
void ProcessGearShift() ;
void setup(void) ;
void loop(void) ;

#include "stm32_screen_control.ino"


#endif
