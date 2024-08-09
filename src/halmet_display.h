#ifndef __SRC_HALMET_DISPLAY_H__
#define __SRC_HALMET_DISPLAY_H__

#include <WString.h>
#include <Wire.h>

#include <Adafruit_SSD1306.h>

bool InitializeSSD1306(Adafruit_SSD1306** display, TwoWire* i2c,
                       const char* hostname);

void ClearRow(Adafruit_SSD1306* display, int row);

void PrintValue(Adafruit_SSD1306* display, int row, const String& title,
                float value);
void PrintValue(Adafruit_SSD1306* display, int row, const String& title,
                const String& value);

#endif
