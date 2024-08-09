#include "halmet_display.h"

#include <Arduino.h>
#include <WString.h>
#include <Wire.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <sensesp/system/local_debug.h>

namespace {

// OLED display width and height, in pixels
constexpr int kScreenWidth = 128;
constexpr int kScreenHeight = 64;

}  // namespace

bool InitializeSSD1306(Adafruit_SSD1306** display, TwoWire* i2c,
                       const char* hostname) {
  *display = new Adafruit_SSD1306(kScreenWidth, kScreenHeight, i2c);
  const bool init_successful = (*display)->begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (!init_successful) {
    debugD("SSD1306 allocation failed");
    return false;
  }
  delay(100);
  (*display)->setRotation(2);
  (*display)->clearDisplay();
  (*display)->setTextSize(1);
  (*display)->setTextColor(SSD1306_WHITE);
  (*display)->setCursor(0, 0);
  (*display)->printf("Host: %s\n", hostname);
  (*display)->display();

  return true;
}

/// Clear a text row on an Adafruit graphics display
void ClearRow(Adafruit_SSD1306* display, int row) {
  display->fillRect(0, 8 * row, kScreenWidth, 8, 0);
}

void PrintValue(Adafruit_SSD1306* display, int row, const String& title,
                float value) {
  ClearRow(display, row);
  display->setCursor(0, 8 * row);
  display->printf("%s: %.1f", title.c_str(), value);
  display->display();
}

void PrintValue(Adafruit_SSD1306* display, int row, const String& title,
                const String& value) {
  ClearRow(display, row);
  display->setCursor(0, 8 * row);
  display->printf("%s: %s", title.c_str(), value.c_str());
  display->display();
}
