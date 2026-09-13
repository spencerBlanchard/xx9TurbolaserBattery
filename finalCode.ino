// ESP32 DevKit + Adafruit 2.8" EYESPI TFT (ILI9341)
// Background bitmap with two independently-redrawn, joystick-controlled elements:
//   1. A pivoting "aim line" anchored at a fixed base, sweeping +/-75 deg,
//      with length that dynamically shrinks/grows to always touch the rectangle's edge.
//   2. A horizontal "elevation" tick sliding up/down the full height of the left bar.
//
// Wiring - TFT (same as before):
//   ESP32 3V3    -> Vin
//   ESP32 GND    -> Gnd
//   ESP32 GPIO23 -> MOSI
//   ESP32 GPIO19 -> MISO
//   ESP32 GPIO18 -> SCK
//   ESP32 GPIO5  -> TCS
//   ESP32 GPIO2  -> DC
//   ESP32 GPIO4  -> RST
//   ESP32 3V3    -> Lite (backlight)
//
// Wiring - Joystick (digital 4-switch + common ground):
//   Joystick UP      -> ESP32 GPIO27
//   Joystick DOWN    -> ESP32 GPIO26
//   Joystick LEFT    -> ESP32 GPIO14
//   Joystick RIGHT   -> ESP32 GPIO13
//   Joystick COMMON  -> ESP32 GND
//
// Libraries needed (Library Manager):
//   Adafruit GFX Library
//   Adafruit ILI9341 Library
//
// Also needs background_bitmap.h in the same sketch folder.

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <math.h>
#include "background_bitmap.h"

#define TFT_CS   5
#define TFT_DC   2
#define TFT_RST  4

#define PIN_UP     27
#define PIN_DOWN   26
#define PIN_LEFT   14
#define PIN_RIGHT  13

Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_RST);

// --- Rectangle interior bounds, measured from the actual background art ---
const int RECT_LEFT   = 49;
const int RECT_RIGHT  = 306;
const int RECT_TOP    = 16;
const int MARGIN      = 3;   // keep the line tip a few px inside the border

// --- Aim line geometry ---
const int   BASE_X        = 177;
const int   BASE_Y        = 224;
const float MAX_ANGLE_DEG = 70.0;
const float ANGLE_STEP    = 6;   // LEFT RIGHT degrees per loop tick while held !@!@!@!@!@!@!@!@!@!@!@!@!@!@!@ (lower is slower)
const int   PIVOT_RADIUS  = 4;

float aimAngleDeg = 0.0;   // 0 = straight up

// --- Elevation tick geometry (full height of the left bar) ---
const int TICK_X_LEFT  = 3;
const int TICK_X_RIGHT = 35;
const int TICK_Y_MIN   = 16;
const int TICK_Y_MAX   = 225;
const int TICK_STEP    = 5;   // UP DOWN degrees per loop tick while held !@!@!@!@!@!@!@!@!@!@!@!@!@!@!@ (lower is slower)

int tickY = (TICK_Y_MIN + TICK_Y_MAX) / 2;

const int FRAME_DELAY_MS = 20;

int prevLineX0, prevLineY0, prevLineX1, prevLineY1;
int prevTickY;

// computes how long the line can be at a given angle before it hits the rectangle wall
float computeLineLength(float angleDeg) {
  float rad = radians(angleDeg);
  float s = sin(rad);
  float c = cos(rad);

  float lenTop = (BASE_Y - (RECT_TOP + MARGIN)) / c;   // c is always > 0 for +/-75 deg

  float lenSide = 1e6;
  if (s > 0.001) {
    lenSide = ((RECT_RIGHT - MARGIN) - BASE_X) / s;
  } else if (s < -0.001) {
    lenSide = (BASE_X - (RECT_LEFT + MARGIN)) / (-s);
  }

  return min(lenTop, lenSide);
}

void computeLineBBox(float angleDeg, int &x0, int &y0, int &x1, int &y1) {
  float rad = radians(angleDeg);
  float length = computeLineLength(angleDeg);
  int tipX = BASE_X + (int)(length * sin(rad));
  int tipY = BASE_Y - (int)(length * cos(rad));

  x0 = min(BASE_X, tipX) - PIVOT_RADIUS - 2;
  x1 = max(BASE_X, tipX) + PIVOT_RADIUS + 2;
  y0 = min(BASE_Y, tipY) - PIVOT_RADIUS - 2;
  y1 = max(BASE_Y, tipY) + PIVOT_RADIUS + 2;
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_UP, INPUT_PULLUP);
  pinMode(PIN_DOWN, INPUT_PULLUP);
  pinMode(PIN_LEFT, INPUT_PULLUP);
  pinMode(PIN_RIGHT, INPUT_PULLUP);

  tft.begin();
  tft.setRotation(1);

  tft.drawRGBBitmap(0, 0, backgroundBitmap, BG_WIDTH, BG_HEIGHT);

  computeLineBBox(aimAngleDeg, prevLineX0, prevLineY0, prevLineX1, prevLineY1);
  prevTickY = tickY;
  drawAimLine(aimAngleDeg);
  drawTick(tickY);
}

void restoreBackgroundRect(int x0, int y0, int x1, int y1) {
  x0 = max(0, x0);
  y0 = max(0, y0);
  x1 = min(BG_WIDTH - 1, x1);
  y1 = min(BG_HEIGHT - 1, y1);
  if (x1 < x0 || y1 < y0) return;

  int w = x1 - x0 + 1;
  for (int y = y0; y <= y1; y++) {
    const uint16_t* rowPtr = backgroundBitmap + y * BG_WIDTH + x0;
    tft.drawRGBBitmap(x0, y, rowPtr, w, 1);
  }
}

void drawAimLine(float angleDeg) {
  float rad = radians(angleDeg);
  float length = computeLineLength(angleDeg);
  int tipX = BASE_X + (int)(length * sin(rad));
  int tipY = BASE_Y - (int)(length * cos(rad));

  tft.drawLine(BASE_X - 1, BASE_Y, tipX - 1, tipY, ILI9341_WHITE);
  tft.drawLine(BASE_X,     BASE_Y, tipX,     tipY, ILI9341_WHITE);
  tft.drawLine(BASE_X + 1, BASE_Y, tipX + 1, tipY, ILI9341_WHITE);

  tft.fillCircle(BASE_X, BASE_Y, PIVOT_RADIUS, ILI9341_WHITE);
}

void drawTick(int y) {
  tft.drawFastHLine(TICK_X_LEFT, y - 1, TICK_X_RIGHT - TICK_X_LEFT, ILI9341_WHITE);
  tft.drawFastHLine(TICK_X_LEFT, y,     TICK_X_RIGHT - TICK_X_LEFT, ILI9341_WHITE);
  tft.drawFastHLine(TICK_X_LEFT, y + 1, TICK_X_RIGHT - TICK_X_LEFT, ILI9341_WHITE);
}

void loop() {
  bool up    = (digitalRead(PIN_UP)    == LOW);
  bool down  = (digitalRead(PIN_DOWN)  == LOW);
  bool left  = (digitalRead(PIN_LEFT)  == LOW);
  bool right = (digitalRead(PIN_RIGHT) == LOW);

  bool aimChanged = false;
  bool tickChanged = false;

  if (left)  { aimAngleDeg -= ANGLE_STEP; aimChanged = true; }
  if (right) { aimAngleDeg += ANGLE_STEP; aimChanged = true; }
  aimAngleDeg = constrain(aimAngleDeg, -MAX_ANGLE_DEG, MAX_ANGLE_DEG);

  if (up)   { tickY -= TICK_STEP; tickChanged = true; }
  if (down) { tickY += TICK_STEP; tickChanged = true; }
  tickY = constrain(tickY, TICK_Y_MIN, TICK_Y_MAX);

  // only touch the pixels for whichever element actually moved
  if (aimChanged) {
    restoreBackgroundRect(prevLineX0, prevLineY0, prevLineX1, prevLineY1);
    drawAimLine(aimAngleDeg);
    computeLineBBox(aimAngleDeg, prevLineX0, prevLineY0, prevLineX1, prevLineY1);
  }

  if (tickChanged) {
    restoreBackgroundRect(TICK_X_LEFT - 1, prevTickY - 2, TICK_X_RIGHT + 1, prevTickY + 2);
    drawTick(tickY);
    prevTickY = tickY;
  }

  delay(FRAME_DELAY_MS);
}
