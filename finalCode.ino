// ESP32 DevKit + Adafruit 2.8" EYESPI TFT (ILI9341)

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


// ============================================================
// SCREEN / SECTION GEOMETRY
// ============================================================

// Right / aim section
const int RECT_LEFT   = 49;
const int RECT_RIGHT  = 306;
const int RECT_TOP    = 16;
const int RECT_BOTTOM = 225;

const int MARGIN = 3;


// Left / height section
const int TICK_X_LEFT  = 3;
const int TICK_X_RIGHT = 35;

const int TICK_Y_MIN = 16;
const int TICK_Y_MAX = 225;


// ============================================================
// PLAYER AIM LINE
// ============================================================

const int BASE_X = 177;
const int BASE_Y = 224;

const float MAX_ANGLE_DEG = 75.0;

// With 10 ms frame delay this gives approximately the same
// overall speed as 2.4 with a 20 ms delay.
const float ANGLE_STEP = 1.2;

const int PIVOT_RADIUS = 4;

float aimAngleDeg = 0.0;


// ============================================================
// PLAYER ELEVATION TICK
// ============================================================

const int TICK_STEP = 5;

int tickY = (TICK_Y_MIN + TICK_Y_MAX) / 2;


// ============================================================
// ENEMY - AIM SECTION
// ============================================================

// Triangle dimensions
const int ENEMY_HALF_WIDTH = 7;
const int ENEMY_HEIGHT     = 10;

// How fast the enemy moves down the screen.
// Units: pixels per second.
const float ENEMY_DESCENT_SPEED = 6.0;


// ------------------------------------------------------------
// Horizontal enemy wiggle
// ------------------------------------------------------------

// Maximum distance left/right from the enemy's center path.
const float ENEMY_X_WIGGLE_AMPLITUDE = 30.0;

// Wiggles per second.
//
// 0.50 = one complete left-right-left cycle every 2 seconds.
const float ENEMY_X_WIGGLE_FREQUENCY = 0.10;


// Starting horizontal center of enemy
const float ENEMY_START_X =
  (RECT_LEFT + RECT_RIGHT) / 2.0;


// Enemy's vertical position
float enemyBaseY = RECT_TOP + ENEMY_HEIGHT + 2;


// Actual calculated enemy position
int enemyX;
int enemyY;


// ============================================================
// ENEMY - HEIGHT SECTION
// ============================================================

// Enemy elevation marker is thicker than our white player marker.
const int ENEMY_HEIGHT_LINE_THICKNESS = 9;


// The enemy elevation starts around the middle of the height bar.
const float ENEMY_ELEVATION_CENTER =
  (TICK_Y_MIN + TICK_Y_MAX) / 2.0;


// ------------------------------------------------------------
// Vertical elevation wiggle
// ------------------------------------------------------------

// Maximum vertical movement from center.
const float ENEMY_ELEVATION_WIGGLE_AMPLITUDE = 40.0;

// Wiggles per second.
const float ENEMY_ELEVATION_WIGGLE_FREQUENCY = 0.03;


// Actual calculated elevation position
int enemyElevationY;


// ============================================================
// COLORS
// ============================================================

const uint16_t ENEMY_COLOR = ILI9341_GREEN;


// ============================================================
// TIMING
// ============================================================

const int FRAME_DELAY_MS = 10;

unsigned long gameStartTime;
unsigned long previousEnemyUpdate;


// ============================================================
// PREVIOUS DRAWING LOCATIONS
// ============================================================

int prevLineX0;
int prevLineY0;
int prevLineX1;
int prevLineY1;

int prevTickY;

int prevEnemyX;
int prevEnemyY;

int prevEnemyElevationY;


// ============================================================
// AIM LINE CALCULATIONS
// ============================================================

float computeLineLength(float angleDeg) {

  float rad = radians(angleDeg);

  float s = sin(rad);
  float c = cos(rad);

  float lenTop =
    (BASE_Y - (RECT_TOP + MARGIN)) / c;

  float lenSide = 1000000.0;

  if (s > 0.001) {

    lenSide =
      ((RECT_RIGHT - MARGIN) - BASE_X) / s;

  }
  else if (s < -0.001) {

    lenSide =
      (BASE_X - (RECT_LEFT + MARGIN)) / (-s);
  }

  return min(lenTop, lenSide);
}


void computeLineBBox(
  float angleDeg,
  int &x0,
  int &y0,
  int &x1,
  int &y1
) {

  float rad = radians(angleDeg);

  float length =
    computeLineLength(angleDeg);

  int tipX =
    BASE_X + (int)(length * sin(rad));

  int tipY =
    BASE_Y - (int)(length * cos(rad));


  x0 =
    min(BASE_X, tipX)
    - PIVOT_RADIUS
    - 2;

  x1 =
    max(BASE_X, tipX)
    + PIVOT_RADIUS
    + 2;

  y0 =
    min(BASE_Y, tipY)
    - PIVOT_RADIUS
    - 2;

  y1 =
    max(BASE_Y, tipY)
    + PIVOT_RADIUS
    + 2;
}


// ============================================================
// BACKGROUND RESTORATION
// ============================================================

void restoreBackgroundRect(
  int x0,
  int y0,
  int x1,
  int y1
) {

  x0 = max(0, x0);
  y0 = max(0, y0);

  x1 = min(BG_WIDTH - 1, x1);
  y1 = min(BG_HEIGHT - 1, y1);

  if (x1 < x0 || y1 < y0)
    return;

  int w = x1 - x0 + 1;

  for (int y = y0; y <= y1; y++) {

    const uint16_t* rowPtr =
      backgroundBitmap
      + y * BG_WIDTH
      + x0;

    tft.drawRGBBitmap(
      x0,
      y,
      rowPtr,
      w,
      1
    );
  }
}


// ============================================================
// PLAYER DRAWING
// ============================================================

void drawAimLine(float angleDeg) {

  float rad = radians(angleDeg);

  float length =
    computeLineLength(angleDeg);

  int tipX =
    BASE_X + (int)(length * sin(rad));

  int tipY =
    BASE_Y - (int)(length * cos(rad));


  tft.drawLine(
    BASE_X - 1,
    BASE_Y,
    tipX - 1,
    tipY,
    ILI9341_WHITE
  );

  tft.drawLine(
    BASE_X,
    BASE_Y,
    tipX,
    tipY,
    ILI9341_WHITE
  );

  tft.drawLine(
    BASE_X + 1,
    BASE_Y,
    tipX + 1,
    tipY,
    ILI9341_WHITE
  );


  tft.fillCircle(
    BASE_X,
    BASE_Y,
    PIVOT_RADIUS,
    ILI9341_WHITE
  );
}


void drawTick(int y) {

  tft.drawFastHLine(
    TICK_X_LEFT,
    y - 1,
    TICK_X_RIGHT - TICK_X_LEFT,
    ILI9341_WHITE
  );

  tft.drawFastHLine(
    TICK_X_LEFT,
    y,
    TICK_X_RIGHT - TICK_X_LEFT,
    ILI9341_WHITE
  );

  tft.drawFastHLine(
    TICK_X_LEFT,
    y + 1,
    TICK_X_RIGHT - TICK_X_LEFT,
    ILI9341_WHITE
  );
}


// ============================================================
// ENEMY DRAWING
// ============================================================

void drawEnemyShip(int x, int y) {

  // Triangle points downward.
  //
  //       ----
  //      /    \
  //     /      \
  //        \/
  //
  // y represents the triangle center-ish position.

  int topY =
    y - ENEMY_HEIGHT / 2;

  int bottomY =
    y + ENEMY_HEIGHT / 2;


  tft.fillTriangle(

    x - ENEMY_HALF_WIDTH,
    topY,

    x + ENEMY_HALF_WIDTH,
    topY,

    x,
    bottomY,

    ENEMY_COLOR
  );
}


void drawEnemyElevation(int y) {

  int halfThickness =
    ENEMY_HEIGHT_LINE_THICKNESS / 2;

  tft.fillRect(

    TICK_X_LEFT,

    y - halfThickness,

    TICK_X_RIGHT - TICK_X_LEFT,

    ENEMY_HEIGHT_LINE_THICKNESS,

    ENEMY_COLOR
  );
}


// ============================================================
// ENEMY POSITION CALCULATION
// ============================================================

void updateEnemyPosition() {

  unsigned long now =
    millis();

  float elapsedSeconds =
    (now - gameStartTime) / 1000.0;


  // ----------------------------------------------------------
  // Move enemy downward
  // ----------------------------------------------------------

  float deltaSeconds =
    (now - previousEnemyUpdate) / 1000.0;

  previousEnemyUpdate = now;


  enemyBaseY +=
    ENEMY_DESCENT_SPEED
    * deltaSeconds;


  // ----------------------------------------------------------
  // Horizontal wiggle in AIM section
  // ----------------------------------------------------------

  float xPhase =
    TWO_PI
    * ENEMY_X_WIGGLE_FREQUENCY
    * elapsedSeconds;

  float wiggleX =
    sin(xPhase)
    * ENEMY_X_WIGGLE_AMPLITUDE;


  enemyX =
    (int)(
      ENEMY_START_X
      + wiggleX
    );


  enemyY =
    (int)enemyBaseY;

    // If the enemy reaches the bottom of the aim section,
// respawn it at the top.
if (enemyY >= RECT_BOTTOM - ENEMY_HEIGHT / 2) {
  enemyBaseY = RECT_TOP + ENEMY_HEIGHT / 2 + 2;

  // Reset the timing reference for smooth movement
  previousEnemyUpdate = now;

  enemyY = (int)enemyBaseY;
}


  // Keep enemy inside the aim section horizontally.

  enemyX = constrain(
    enemyX,
    RECT_LEFT + ENEMY_HALF_WIDTH + MARGIN,
    RECT_RIGHT - ENEMY_HALF_WIDTH - MARGIN
  );


  // ----------------------------------------------------------
  // Vertical wiggle in HEIGHT section
  // ----------------------------------------------------------

  float elevationPhase =
    TWO_PI
    * ENEMY_ELEVATION_WIGGLE_FREQUENCY
    * elapsedSeconds;

  float elevationWiggle =
    sin(elevationPhase)
    * ENEMY_ELEVATION_WIGGLE_AMPLITUDE;


  enemyElevationY =
    (int)(
      ENEMY_ELEVATION_CENTER
      + elevationWiggle
    );


  int elevationHalfThickness =
    ENEMY_HEIGHT_LINE_THICKNESS / 2;


  enemyElevationY = constrain(
    enemyElevationY,
    TICK_Y_MIN + elevationHalfThickness,
    TICK_Y_MAX - elevationHalfThickness
  );
}


// ============================================================
// RESTORE OLD ENEMY LOCATIONS
// ============================================================

void eraseOldEnemy() {

  // Restore area around old triangle.

  restoreBackgroundRect(

    prevEnemyX
      - ENEMY_HALF_WIDTH
      - 2,

    prevEnemyY
      - ENEMY_HEIGHT / 2
      - 2,

    prevEnemyX
      + ENEMY_HALF_WIDTH
      + 2,

    prevEnemyY
      + ENEMY_HEIGHT / 2
      + 2
  );


  // Restore area around old enemy elevation line.

  int halfThickness =
    ENEMY_HEIGHT_LINE_THICKNESS / 2;


  restoreBackgroundRect(

    TICK_X_LEFT - 1,

    prevEnemyElevationY
      - halfThickness
      - 2,

    TICK_X_RIGHT + 1,

    prevEnemyElevationY
      + halfThickness
      + 2
  );
}


// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(115200);


  pinMode(
    PIN_UP,
    INPUT_PULLUP
  );

  pinMode(
    PIN_DOWN,
    INPUT_PULLUP
  );

  pinMode(
    PIN_LEFT,
    INPUT_PULLUP
  );

  pinMode(
    PIN_RIGHT,
    INPUT_PULLUP
  );


  tft.begin();

  tft.setRotation(1);


  tft.drawRGBBitmap(
    0,
    0,
    backgroundBitmap,
    BG_WIDTH,
    BG_HEIGHT
  );


  // ----------------------------------------------------------
  // Initial player positions
  // ----------------------------------------------------------

  computeLineBBox(
    aimAngleDeg,
    prevLineX0,
    prevLineY0,
    prevLineX1,
    prevLineY1
  );

  prevTickY = tickY;


  // ----------------------------------------------------------
  // Initial enemy positions
  // ----------------------------------------------------------

  gameStartTime =
    millis();

  previousEnemyUpdate =
    gameStartTime;


  updateEnemyPosition();


  prevEnemyX =
    enemyX;

  prevEnemyY =
    enemyY;

  prevEnemyElevationY =
    enemyElevationY;


  // ----------------------------------------------------------
  // Initial drawing
  // ----------------------------------------------------------

  drawEnemyShip(
    enemyX,
    enemyY
  );

  drawEnemyElevation(
    enemyElevationY
  );


  // Player elements are drawn on top of enemies.

  drawAimLine(
    aimAngleDeg
  );

  drawTick(
    tickY
  );
}


// ============================================================
// LOOP
// ============================================================

void loop() {

  // ----------------------------------------------------------
  // Read joystick
  // ----------------------------------------------------------

  bool up =
    (digitalRead(PIN_UP) == LOW);

  bool down =
    (digitalRead(PIN_DOWN) == LOW);

  bool left =
    (digitalRead(PIN_LEFT) == LOW);

  bool right =
    (digitalRead(PIN_RIGHT) == LOW);


  bool aimChanged = false;
  bool tickChanged = false;


  // ==========================================================
  // PLAYER AIM
  // ==========================================================

  // Compensate for shortening line near edges.

  float currentLength =
    computeLineLength(aimAngleDeg);

  float adjustedStep =
    ANGLE_STEP
    * (205.0 / currentLength);


  if (left) {

    aimAngleDeg -= adjustedStep;

    aimChanged = true;
  }


  if (right) {

    aimAngleDeg += adjustedStep;

    aimChanged = true;
  }


  aimAngleDeg = constrain(
    aimAngleDeg,
    -MAX_ANGLE_DEG,
    MAX_ANGLE_DEG
  );


  // ==========================================================
  // PLAYER HEIGHT
  // ==========================================================

  if (up) {

    tickY -= TICK_STEP;

    tickChanged = true;
  }


  if (down) {

    tickY += TICK_STEP;

    tickChanged = true;
  }


  tickY = constrain(
    tickY,
    TICK_Y_MIN,
    TICK_Y_MAX
  );


  // ==========================================================
  // ERASE OLD DYNAMIC GRAPHICS
  // ==========================================================

  // Enemy moves continuously, so erase its previous frame.

  eraseOldEnemy();


  // If player aim moved, erase its old location.

  if (aimChanged) {

    restoreBackgroundRect(
      prevLineX0,
      prevLineY0,
      prevLineX1,
      prevLineY1
    );
  }


  // If player height tick moved, erase old location.

  if (tickChanged) {

    restoreBackgroundRect(
      TICK_X_LEFT - 1,
      prevTickY - 2,
      TICK_X_RIGHT + 1,
      prevTickY + 2
    );
  }


  // ==========================================================
  // UPDATE ENEMY
  // ==========================================================

  updateEnemyPosition();


  // ==========================================================
  // DRAW ENEMY
  // ==========================================================

  drawEnemyShip(
    enemyX,
    enemyY
  );


  drawEnemyElevation(
    enemyElevationY
  );


  // ==========================================================
  // REDRAW PLAYER ON TOP
  // ==========================================================

  // Important:
  //
  // Because restoring an enemy's old background can erase
  // parts of a player marker if they overlap, redraw BOTH
  // player elements every frame.

  drawAimLine(
    aimAngleDeg
  );


  drawTick(
    tickY
  );


  // ==========================================================
  // SAVE CURRENT POSITIONS
  // ==========================================================

  computeLineBBox(
    aimAngleDeg,
    prevLineX0,
    prevLineY0,
    prevLineX1,
    prevLineY1
  );


  prevTickY =
    tickY;


  prevEnemyX =
    enemyX;

  prevEnemyY =
    enemyY;

  prevEnemyElevationY =
    enemyElevationY;


  delay(FRAME_DELAY_MS);
}
