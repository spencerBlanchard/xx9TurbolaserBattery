// ============================================================
// ESP32 TURRET GAME
// Adafruit 2.8" ILI9341 TFT
//
// MAXIMUM-LATENCY-OPTIMIZED AUDIO VERSION:
//   Button detection + DFPlayer commands run on ESP32 Core 0.
//   GPIO interrupts wake the audio task immediately.
//   DFPlayer packets are written directly to UART with NO library
//   command delay and NO ACK wait.
//   Root-folder playback is used for the shortest command path.
//
// PLAYER:
//   Left / Right = aim line + physical left/right servo
//   Up / Down    = elevation marker + physical up/down servo
//
// ENEMY:
//   Green triangle moves downward through right / aim section
//   Green horizontal marker shows enemy elevation in left section
//
// HEALTH:
//   Red health bar at bottom
//   3 missed enemies = GAME OVER
//
// BUTTON / LED SYSTEM:
//
//   GREEN BUTTON = GPIO35
//   RED BUTTON   = GPIO34
//
//   GREEN LED     = GPIO32
//   RED LED       = GPIO33
//   ANIMATION LED = GPIO25
//
// NORMAL:
//   Both LEDs ON
//
// GREEN BUTTON:
//   Both LEDs OFF for at least 1 second
//
// RED BUTTON:
//   Green LED OFF for at least 2 seconds
//   Red LED OFF for at least 4 seconds
//
// BUTTON RE-ARM:
//   A button remains disabled for 80ms AFTER its corresponding
//   LED comes back on.
//
//   This guarantees that even when buttons are spammed,
//   you will visibly see the LED illuminate before another
//   valid button press can occur.
//
// IMPORTANT:
//   GPIO34 and GPIO35 require EXTERNAL pull-up resistors.
// ============================================================


#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>
#include "background_bitmap.h"
#include "start_menu_background_bitmap.h"


// Set to 1 only while debugging. Keeping this at 0 removes
// Serial printing from the time-critical button/audio path.
#define BUTTON_AUDIO_DEBUG 0


// ============================================================
// TFT PINS
// ============================================================

#define TFT_CS   5
#define TFT_DC   2
#define TFT_RST  4


// ============================================================
// JOYSTICK PINS
// ============================================================

#define PIN_UP     26
#define PIN_DOWN   27
#define PIN_LEFT   14
#define PIN_RIGHT  13


// ============================================================
// BUTTON PINS
// ============================================================
//
// GPIO34 / GPIO35 do NOT have internal pull-ups.
//
// Wiring:
//
// 3.3V
//   |
//  10k
//   |
//   +------ GPIO34 / GPIO35
//   |
// BUTTON
//   |
//  GND
//
// ============================================================

#define PIN_RED_BUTTON    34
#define PIN_GREEN_BUTTON  35


// ============================================================
// LED PINS
// ============================================================

#define PIN_GREEN_LED      32
#define PIN_RED_LED        33
#define PIN_ANIMATION_LED  25


// ============================================================
// DFPLAYER MINI
// ============================================================
//
// DFPlayer TX -> ESP32 GPIO16 (RX2)
// ESP32 GPIO17 (TX2) -> 1k resistor -> DFPlayer RX
//
// FILES ARE IN THE ROOT OF THE SD CARD:
//   /0001.wav   (or /0001.mp3)
//   /0002.wav   (or /0002.mp3)
//   /0003.wav   (or /0003.mp3)
//   /0004.wav   (or /0004.mp3)
//
// IMPORTANT FOR dfPlayer-style root indexing:
// Root track numbers follow the FAT/copy order on the SD card.
// For predictable 1/2/3/4 mapping, use a freshly formatted card
// and copy 0001, then 0002, then 0003, then 0004.
//
// This version intentionally does NOT use DFRobotDFPlayerMini.
// It sends the 10-byte DFPlayer UART packet directly so there is
// no library-imposed delay before/after each play command.
// ============================================================

#define DFPLAYER_RX  16
#define DFPLAYER_TX  17

HardwareSerial dfSerial(2);

bool dfPlayerReady = false;


// ============================================================
// PCA9685 SERVO CONTROLLER
// ============================================================
//
// PCA9685 wiring:
//   VCC -> ESP32 3.3V
//   GND -> common GND
//   SDA -> ESP32 GPIO21
//   SCL -> ESP32 GPIO22
//   V+  -> external 5V servo supply
//
// Servo channel assignments:
//   channel 0 = SG90 button animation servo
//   channel 1 = up/down continuous-rotation MG90S
//   channel 2 = left/right continuous-rotation MG90S
// ============================================================

#define PCA9685_SDA 21
#define PCA9685_SCL 22

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

const uint8_t ANIMATION_SERVO_CHANNEL = 0;
const uint8_t UD_SERVO_CHANNEL = 1;
const uint8_t LR_SERVO_CHANNEL = 2;

// ============================================================
// CALIBRATED CONTINUOUS-ROTATION SERVO VALUES
// ============================================================
// These are raw PCA9685 ticks at 50 Hz.
// We use raw ticks instead of converting from microseconds because
// the calibration showed that a one-tick change matters around neutral.
//
// LEFT / RIGHT servo (channel 2):
//   left  = 353
//   stop  = 373
//   right = 383
//
// UP / DOWN servo (channel 1):
//   up    = 380
//   stop  = 370
//   down  = 350
// ============================================================

const uint16_t LR_LEFT_TICKS  = 351;
const uint16_t LR_STOP_TICKS  = 371;
const uint16_t LR_RIGHT_TICKS = 381;

const uint16_t UD_UP_TICKS    = 380;
const uint16_t UD_STOP_TICKS  = 370;
const uint16_t UD_DOWN_TICKS  = 350;

// Cache the last command so we do not repeatedly rewrite the same
// PCA9685 value every frame. -1 means no command has been sent yet.
int lastLRServoTicks = -1;
int lastUDServoTicks = -1;

// ============================================================
// LEFT / RIGHT DIGITAL TRAVEL LIMIT
// ============================================================
// The turret is assumed to be physically centered every time the
// program starts. We then track its estimated horizontal position
// purely by how long the continuous-rotation servo has been moving.
//
// Center = 0 ms
// Full left limit  = -1500 ms from center
// Full right limit = +1500 ms from center
//
// Therefore a complete left-to-right sweep is approximately 3000 ms.
// This is an open-loop/software limit: if the turret is moved by hand
// or slips mechanically, the program cannot detect that.
// ============================================================

const int32_t LR_TRAVEL_LIMIT_MS = 1500;

// Signed estimated horizontal position, expressed as equivalent
// milliseconds of servo travel from the startup center position.
int32_t lrTravelPositionMs = 0;

// Direction that was actually commanded during the interval since
// the previous update: -1 = left, 0 = stopped, +1 = right.
int8_t lrPreviousMoveDirection = 0;

unsigned long lrLastTravelUpdateMs = 0;

// ============================================================
// UP / DOWN DIGITAL TRAVEL LIMIT
// ============================================================
// The turret is assumed to be physically centered vertically when the
// game starts. We track vertical position only by how long the continuous-
// rotation servo has actually been commanded to move.
//
// Center = 0 ms
// Full up limit   = -1500 ms from center
// Full down limit = +1500 ms from center
//
// This makes the complete physical top-to-bottom command-time sweep
// exactly 6.0 seconds, matching the TFT elevation marker.
// ============================================================

const int32_t UD_TRAVEL_LIMIT_MS = 1100;

// Signed estimated vertical position, expressed as equivalent
// milliseconds of servo travel from the startup center position.
// Negative = up, positive = down.
int32_t udTravelPositionMs = 0;

// Direction actually commanded during the interval since the previous
// update: -1 = up, 0 = stopped, +1 = down.
int8_t udPreviousMoveDirection = 0;

unsigned long udLastTravelUpdateMs = 0;

// Conservative SG90 pulse range at 50 Hz.
// If the servo mechanically binds near either endpoint, reduce
// SERVO_MAX_PULSE or increase SERVO_MIN_PULSE.
const uint16_t SERVO_MIN_PULSE = 110;
const uint16_t SERVO_MAX_PULSE = 500;

// This recreates the earlier recoil-style SG90 animation:
// rest around 70 degrees, make a small preload toward 65,
// snap to 180, then smoothly return to 70 over 350 ms.
const float SERVO_REST_ANGLE    = 70.0f;
const float SERVO_PRELOAD_ANGLE = 65.0f;
const float SERVO_KICK_ANGLE    = 180.0f;

const unsigned long SERVO_PRELOAD_MS   = 70;
const unsigned long SERVO_KICK_HOLD_MS = 20;
const unsigned long SERVO_RETURN_MS    = 350;

enum ServoAnimationState {
  SERVO_ANIM_IDLE,
  SERVO_ANIM_PRELOAD,
  SERVO_ANIM_KICK_HOLD,
  SERVO_ANIM_RETURN
};

ServoAnimationState servoAnimationState = SERVO_ANIM_IDLE;
unsigned long servoAnimationStateStart = 0;
float servoCurrentAngle = SERVO_REST_ANGLE;
float servoPreloadStartAngle = SERVO_REST_ANGLE;

// Red-button recoil plays at half speed (2x duration).
// Green-button recoil uses the normal timing.
float servoAnimationTimeScale = 1.0f;


// ============================================================
// TFT
// ============================================================

Adafruit_ILI9341 tft(
  TFT_CS,
  TFT_DC,
  TFT_RST
);


// ============================================================
// BUTTON / LED TIMEOUT SYSTEM
// ============================================================

// Green button:
//
// Both LEDs remain off for 1 second.
const unsigned long GREEN_BUTTON_TIMEOUT_MS = 1000;


// Red button:
//
// Green LED returns after 2 seconds.
// Red LED returns after 4 seconds.
const unsigned long RED_BUTTON_GREEN_TIMEOUT_MS = 2000;
const unsigned long RED_BUTTON_RED_TIMEOUT_MS   = 4000;


// Button remains disabled this much longer
// AFTER its corresponding LED returns.
const unsigned long BUTTON_REARM_DELAY_MS = 80;


// ------------------------------------------------------------
// LED timeout timestamps
// ------------------------------------------------------------

unsigned long greenLedOffUntil = 0;
unsigned long redLedOffUntil   = 0;


// ------------------------------------------------------------
// Button re-enable timestamps
// ------------------------------------------------------------

unsigned long greenButtonDisabledUntil = 0;
unsigned long redButtonDisabledUntil   = 0;


// ------------------------------------------------------------
// Previous physical button states
//
// These allow us to detect the transition:
//
// RELEASED -> PRESSED
//
// Holding a button down does NOT repeatedly fire.
// ------------------------------------------------------------

bool previousGreenButton = false;
bool previousRedButton   = false;


// ------------------------------------------------------------
// ACCEPTED BUTTON PRESSES
//
// These become true for ONE loop iteration only when a valid
// shot/button press is accepted.
//
// Later, use THESE variables for the actual shooting gameplay.
//
// DO NOT use digitalRead() directly for shooting.
// ------------------------------------------------------------

bool greenButtonPressedThisFrame = false;
bool redButtonPressedThisFrame   = false;


// ============================================================
// DUAL-CORE BUTTON / AUDIO TASK
// ============================================================
//
// The normal Arduino loop and TFT rendering run independently
// from this task. The button/audio task is pinned to Core 0.
// GPIO FALLING-edge interrupts wake it immediately, so a long
// TFT draw on the main loop does not postpone the DFPlayer
// command and the task does not have to wait for its 1 ms tick.
//
// Accepted button presses are queued here so the main loop can
// still consume them later for shooting/gameplay without losing
// an event.
// ============================================================

TaskHandle_t buttonAudioTaskHandle = nullptr;
bool buttonAudioTaskStarted = false;

// Both button GPIOs use FALLING-edge interrupts. The ISR does no
// audio work itself; it only wakes the Core-0 task immediately.
// This avoids waiting for the next 1 ms polling interval.
volatile bool buttonWakeInterruptsAttached = false;

portMUX_TYPE buttonEventMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t pendingGreenPresses = 0;
volatile uint32_t pendingRedPresses   = 0;


// ============================================================
// RIGHT / AIM SECTION
// ============================================================

const int RECT_LEFT   = 49;
const int RECT_RIGHT  = 306;
const int RECT_TOP    = 16;
const int RECT_BOTTOM = 225;

const int MARGIN = 3;


// ============================================================
// LEFT / HEIGHT SECTION
// ============================================================

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

// Horizontal TFT aim is tied directly to the same time-based position
// estimate used by the physical left/right turret. From full left
// (-1500 ms) to full right (+1500 ms) is therefore exactly 3000 ms.
const int PIVOT_RADIUS = 4;

float aimAngleDeg = 0.0;


// ============================================================
// PLAYER HEIGHT MARKER
// ============================================================

const int TICK_STEP = 5;

int tickY =
  (TICK_Y_MIN + TICK_Y_MAX) / 2;


// ============================================================
// ENEMY - RIGHT / AIM SECTION
// ============================================================

const int ENEMY_HALF_WIDTH = 7;
const int ENEMY_HEIGHT     = 10;


// Pixels per second
const float ENEMY_DESCENT_SPEED = 12.0;


// Horizontal wiggle
const float ENEMY_X_WIGGLE_AMPLITUDE = 30.0;
const float ENEMY_X_WIGGLE_FREQUENCY = 0.10;


// Enemy route system. Each new ship advances to the next route:
//   0 = top middle  -> bottom middle
//   1 = top left    -> bottom right
//   2 = top right   -> bottom left
//   3 = middle left -> middle right
//   4 = middle right-> middle left
const uint8_t ENEMY_PATH_COUNT = 5;
uint8_t enemyPathIndex = 0;

// The original vertical route crossed the play area at 12 px/sec.
// Use that same total travel time for every route so each pattern has
// a consistent gameplay duration, regardless of route length.
const float ENEMY_PATH_DURATION_SECONDS =
  ((RECT_BOTTOM - ENEMY_HEIGHT / 2 - 2)
   - (RECT_TOP + ENEMY_HEIGHT / 2 + 2))
  / ENEMY_DESCENT_SPEED;

// Each new ship moves 7% faster along its start-to-end route than the
// previous ship. This only affects route traversal; wiggle timing is unchanged.
const float ENEMY_SPEED_INCREASE_PER_SHIP = 0.07f;

// Red-phase enemies move slightly slower while red. Once a red enemy is hit
// and turns green, it immediately resumes the normal speed for its ship number.
const float RED_ENEMY_SPEED_MULTIPLIER = 0.95f;

float enemyPathProgress = 0.0f;

// Route endpoints for the current ship.
float enemyStartX = 0.0f;
float enemyStartY = 0.0f;
float enemyEndX   = 0.0f;
float enemyEndY   = 0.0f;

// Actual screen position
int enemyX;
int enemyY;


// ============================================================
// ENEMY - LEFT / HEIGHT SECTION
// ============================================================

const int ENEMY_HEIGHT_LINE_THICKNESS = 9;


// Enemy elevation target center. This is randomized for each NEW ship.
// The random center is kept far enough from the edges that the full wiggle
// can still happen without ever leaving the legal elevation range.
float enemyElevationCenterY =
  (TICK_Y_MIN + TICK_Y_MAX) / 2.0f;


// Elevation wiggle by enemy color.
// Green ships move more aggressively vertically.
// Red ships use a smaller/slower vertical wiggle.
const float GREEN_ENEMY_ELEVATION_WIGGLE_AMPLITUDE = 90.0f;
const float GREEN_ENEMY_ELEVATION_WIGGLE_FREQUENCY = 0.04f;

const float RED_ENEMY_ELEVATION_WIGGLE_AMPLITUDE = 50.0f;
const float RED_ENEMY_ELEVATION_WIGGLE_FREQUENCY = 0.03f;

// Give every new ship a different wiggle starting phase as well, so the
// elevation target does not always enter with the same motion pattern.
float enemyElevationPhaseOffset = 0.0f;


int enemyElevationY;


// ============================================================
// COLORS
// ============================================================

const uint16_t ENEMY_GREEN_COLOR = ILI9341_GREEN;
const uint16_t ENEMY_RED_COLOR   = ILI9341_RED;

// Every spawned ship gets a 1-based number.
// Ships 1, 2, and 4 start green. Ship 3, ship 5, and every ship after 5 start red.
uint32_t enemyShipNumber = 1;
bool enemyIsRed = false;

uint16_t currentEnemyColor() {
  return enemyIsRed ? ENEMY_RED_COLOR : ENEMY_GREEN_COLOR;
}

void setEnemyColorForShipNumber() {
  enemyIsRed = (enemyShipNumber == 3 || enemyShipNumber >= 5);
}


// ============================================================
// HEALTH SYSTEM
// ============================================================

const int MAX_HEALTH = 3;

int health = MAX_HEALTH;

bool gameOver = false;


// ============================================================
// SCORE + HIT DETECTION
// ============================================================

int score = 0;

// Small top-right score box. Scores 0-9 are shown as 00-09.
const int SCORE_BOX_X = 272;
const int SCORE_BOX_Y = 2;
const int SCORE_BOX_W = 46;
const int SCORE_BOX_H = 13;

// A shot counts when BOTH targeting dimensions are close enough:
//   1) the white aim line passes close to the ship's X position
//   2) the white elevation tick is close to the enemy elevation marker
//
// Difficulty is selected from the joystick-controlled start menu.
// NORMAL is selected by default and uses the tighter 7 px threshold.
// EASY uses the more forgiving 10 px threshold.
const int NORMAL_HIT_THRESHOLD_PX = 7;
const int EASY_HIT_THRESHOLD_PX   = 10;

enum GameDifficulty {
  DIFFICULTY_NORMAL,
  DIFFICULTY_EASY
};

GameDifficulty selectedDifficulty = DIFFICULTY_NORMAL;

int hitXThresholdPx = NORMAL_HIT_THRESHOLD_PX;
int hitYThresholdPx = NORMAL_HIT_THRESHOLD_PX;

// Quick two-flicker hit animation. Enemy movement pauses during it.
const unsigned long HIT_FLICKER_INTERVAL_MS = 70;
const int HIT_FLICKER_TRANSITIONS = 4; // off/on/off/on, then disappear + respawn

bool enemyHitAnimating = false;
bool enemyVisibleDuringHit = true;
unsigned long enemyHitLastToggleMs = 0;
int enemyHitTransitionCount = 0;

// false: normal green hit -> destroy + respawn
// true: red hit -> flicker, then turn green and continue same route
bool enemyHitTurnsGreen = false;


// Thin red bar at bottom
const int HEALTH_BAR_X = 10;
const int HEALTH_BAR_Y = 235;

const int HEALTH_BAR_WIDTH  = 300;
const int HEALTH_BAR_HEIGHT = 3;


// ============================================================
// FRAME TIMING
// ============================================================

// Kept very short so button polling happens as often as possible.
const int FRAME_DELAY_MS = 1;


unsigned long gameStartTime;
unsigned long previousEnemyUpdate;


// ============================================================
// PREVIOUS DRAW POSITIONS
// ============================================================

// Aim line
int prevLineX0;
int prevLineY0;
int prevLineX1;
int prevLineY1;


// Player elevation
int prevTickY;


// Enemy triangle
int prevEnemyX;
int prevEnemyY;


// Enemy elevation
int prevEnemyElevationY;


// ============================================================
// ULTRA-LOW-OVERHEAD DFPLAYER UART COMMANDS
// ============================================================
//
// DFPlayer packets are always 10 bytes:
//   7E FF 06 CMD FEEDBACK PARAM_H PARAM_L CHECK_H CHECK_L EF
//
// Feedback is 0x00 so the ESP32 does not wait for acknowledgements.
// We also intentionally do NOT call dfSerial.flush() after play.
// Hardware UART starts transmitting immediately while this task
// continues. At 9600 baud, the physical 10-byte command itself
// takes about 10 ms on the wire; that cannot be reduced without
// changing audio hardware.
// ============================================================

void sendDFPlayerCommandFast(
  uint8_t command,
  uint16_t parameter
) {

  uint8_t packet[10];

  packet[0] = 0x7E;                // Start byte
  packet[1] = 0xFF;                // Version
  packet[2] = 0x06;                // Length
  packet[3] = command;
  packet[4] = 0x00;                // No feedback / no ACK
  packet[5] = highByte(parameter);
  packet[6] = lowByte(parameter);

  // Checksum = 0 - sum(bytes 1 through 6)
  uint16_t checksum =
    0 - (
      packet[1] +
      packet[2] +
      packet[3] +
      packet[4] +
      packet[5] +
      packet[6]
    );

  packet[7] = highByte(checksum);
  packet[8] = lowByte(checksum);
  packet[9] = 0xEF;                // End byte

  // HardwareSerial::write() copies the packet into the UART TX
  // buffer and returns. Do NOT flush here; flushing would force
  // this task to sit and wait for transmission to finish.
  dfSerial.write(
    packet,
    sizeof(packet)
  );
}


// Command 0x03 = play track by root-card index.
// Track 1/2/3/4 means the 1st/2nd/3rd/4th indexed file in root.
inline void playRootTrackFast(uint16_t track) {

  if (!dfPlayerReady) {
    return;
  }

  sendDFPlayerCommandFast(
    0x03,
    track
  );
}


// Command 0x06 = volume, range 0..30.
inline void setDFPlayerVolumeFast(uint8_t volume) {

  if (volume > 30) {
    volume = 30;
  }

  sendDFPlayerCommandFast(
    0x06,
    volume
  );
}


// ============================================================
// SG90 ANIMATION SERVO HELPERS
// ============================================================

uint16_t servoAngleToPulse(float angle) {

  angle = constrain(
    angle,
    0.0f,
    180.0f
  );

  float normalized =
    angle / 180.0f;

  return (uint16_t)(
    SERVO_MIN_PULSE +
    normalized *
    (SERVO_MAX_PULSE - SERVO_MIN_PULSE)
  );
}


void setAnimationServoAngle(float angle) {

  angle = constrain(
    angle,
    0.0f,
    180.0f
  );

  servoCurrentAngle = angle;

  pwm.setPWM(
    ANIMATION_SERVO_CHANNEL,
    0,
    servoAngleToPulse(angle)
  );
}


void triggerServoAnimation(float timeScale = 1.0f) {

  servoAnimationTimeScale = max(0.1f, timeScale);

  // Turn the standalone animation LED on immediately.
  digitalWrite(
    PIN_ANIMATION_LED,
    HIGH
  );

  // Restart cleanly from wherever the servo currently is.
  servoPreloadStartAngle =
    servoCurrentAngle;

  servoAnimationStateStart =
    millis();

  servoAnimationState =
    SERVO_ANIM_PRELOAD;
}


void updateServoAnimation() {

  if (servoAnimationState == SERVO_ANIM_IDLE) {

    // Animation is finished, so the standalone LED is off.
    digitalWrite(
      PIN_ANIMATION_LED,
      LOW
    );

    return;
  }

  // Any non-idle animation state keeps the LED illuminated.
  digitalWrite(
    PIN_ANIMATION_LED,
    HIGH
  );

  unsigned long now =
    millis();

  unsigned long elapsed =
    now - servoAnimationStateStart;


  // ----------------------------------------------------------
  // 1. Small preload movement toward 65 degrees.
  // ----------------------------------------------------------
  if (servoAnimationState == SERVO_ANIM_PRELOAD) {

    float t =
      (float)elapsed /
      ((float)SERVO_PRELOAD_MS * servoAnimationTimeScale);

    if (t >= 1.0f) {

      setAnimationServoAngle(
        SERVO_PRELOAD_ANGLE
      );

      // Snap immediately to the kick angle.
      setAnimationServoAngle(
        SERVO_KICK_ANGLE
      );

      servoAnimationState =
        SERVO_ANIM_KICK_HOLD;

      servoAnimationStateStart =
        now;

      return;
    }

    float angle =
      servoPreloadStartAngle +
      (
        SERVO_PRELOAD_ANGLE -
        servoPreloadStartAngle
      ) * t;

    setAnimationServoAngle(
      angle
    );

    return;
  }


  // ----------------------------------------------------------
  // 2. Briefly hold the kicked position.
  // ----------------------------------------------------------
  if (servoAnimationState == SERVO_ANIM_KICK_HOLD) {

    if (elapsed >= (unsigned long)(SERVO_KICK_HOLD_MS * servoAnimationTimeScale)) {

      servoAnimationState =
        SERVO_ANIM_RETURN;

      servoAnimationStateStart =
        now;
    }

    return;
  }


  // ----------------------------------------------------------
  // 3. Smoothly return from 180 degrees to the 70 degree rest.
  // ----------------------------------------------------------
  if (servoAnimationState == SERVO_ANIM_RETURN) {

    float t =
      (float)elapsed /
      ((float)SERVO_RETURN_MS * servoAnimationTimeScale);

    if (t >= 1.0f) {

      setAnimationServoAngle(
        SERVO_REST_ANGLE
      );

      servoAnimationState =
        SERVO_ANIM_IDLE;

      digitalWrite(
        PIN_ANIMATION_LED,
        LOW
      );

      return;
    }

    float angle =
      SERVO_KICK_ANGLE +
      (
        SERVO_REST_ANGLE -
        SERVO_KICK_ANGLE
      ) * t;

    setAnimationServoAngle(
      angle
    );
  }
}


// ============================================================
// JOYSTICK-CONTROLLED CONTINUOUS-ROTATION SERVOS
// ============================================================

void setLRServoTicks(uint16_t ticks) {

  if ((int)ticks == lastLRServoTicks) {
    return;
  }

  pwm.setPWM(
    LR_SERVO_CHANNEL,
    0,
    ticks
  );

  lastLRServoTicks = ticks;
}


void setUDServoTicks(uint16_t ticks) {

  if ((int)ticks == lastUDServoTicks) {
    return;
  }

  pwm.setPWM(
    UD_SERVO_CHANNEL,
    0,
    ticks
  );

  lastUDServoTicks = ticks;
}


void stopJoystickServos() {

  setLRServoTicks(
    LR_STOP_TICKS
  );

  setUDServoTicks(
    UD_STOP_TICKS
  );
}


void updateJoystickServos(
  bool left,
  bool right,
  bool up,
  bool down
) {

  // ----------------------------------------------------------
  // LEFT / RIGHT - PCA9685 channel 2
  // DIGITAL TIME-BASED TRAVEL LIMITS
  // ----------------------------------------------------------
  // First account for how long the servo has ACTUALLY been commanded
  // to move since the previous update. This makes the software position
  // track elapsed movement time rather than the number of loop frames.

  unsigned long nowMs = millis();

  if (lrLastTravelUpdateMs == 0) {
    lrLastTravelUpdateMs = nowMs;
  }

  unsigned long elapsedMs =
    nowMs - lrLastTravelUpdateMs;

  lrLastTravelUpdateMs = nowMs;

  if (lrPreviousMoveDirection < 0) {
    lrTravelPositionMs -= (int32_t)elapsedMs;
  }
  else if (lrPreviousMoveDirection > 0) {
    lrTravelPositionMs += (int32_t)elapsedMs;
  }

  // Never allow the estimated position outside the +/-1.5 second range.
  lrTravelPositionMs = constrain(
    lrTravelPositionMs,
    -LR_TRAVEL_LIMIT_MS,
    LR_TRAVEL_LIMIT_MS
  );

  int8_t requestedDirection = 0;

  if (left && !right) {
    requestedDirection = -1;
  }
  else if (right && !left) {
    requestedDirection = 1;
  }

  // If the joystick is trying to move farther past a digital limit,
  // stop the servo. Moving back toward center is always allowed.
  if (
    requestedDirection < 0 &&
    lrTravelPositionMs <= -LR_TRAVEL_LIMIT_MS
  ) {
    requestedDirection = 0;
  }

  if (
    requestedDirection > 0 &&
    lrTravelPositionMs >= LR_TRAVEL_LIMIT_MS
  ) {
    requestedDirection = 0;
  }

  if (requestedDirection < 0) {
    setLRServoTicks(
      LR_LEFT_TICKS
    );
  }
  else if (requestedDirection > 0) {
    setLRServoTicks(
      LR_RIGHT_TICKS
    );
  }
  else {
    setLRServoTicks(
      LR_STOP_TICKS
    );
  }

  lrPreviousMoveDirection = requestedDirection;


  // ----------------------------------------------------------
  // UP / DOWN - PCA9685 channel 1
  // DIGITAL TIME-BASED TRAVEL LIMITS
  // ----------------------------------------------------------
  // This mirrors the left/right limiter. The program assumes the turret
  // begins vertically centered, then estimates position from elapsed time
  // while the servo is actually being commanded to move.

  unsigned long udNowMs = millis();

  if (udLastTravelUpdateMs == 0) {
    udLastTravelUpdateMs = udNowMs;
  }

  unsigned long udElapsedMs =
    udNowMs - udLastTravelUpdateMs;

  udLastTravelUpdateMs = udNowMs;

  if (udPreviousMoveDirection < 0) {
    udTravelPositionMs -= (int32_t)udElapsedMs;
  }
  else if (udPreviousMoveDirection > 0) {
    udTravelPositionMs += (int32_t)udElapsedMs;
  }

  udTravelPositionMs = constrain(
    udTravelPositionMs,
    -UD_TRAVEL_LIMIT_MS,
    UD_TRAVEL_LIMIT_MS
  );

  int8_t requestedUDDirection = 0;

  if (up && !down) {
    requestedUDDirection = -1;
  }
  else if (down && !up) {
    requestedUDDirection = 1;
  }

  // Stop if the joystick is trying to move farther past an end limit.
  // Moving back toward center/opposite end remains immediately available.
  if (
    requestedUDDirection < 0 &&
    udTravelPositionMs <= -UD_TRAVEL_LIMIT_MS
  ) {
    requestedUDDirection = 0;
  }

  if (
    requestedUDDirection > 0 &&
    udTravelPositionMs >= UD_TRAVEL_LIMIT_MS
  ) {
    requestedUDDirection = 0;
  }

  if (requestedUDDirection < 0) {
    setUDServoTicks(
      UD_UP_TICKS
    );
  }
  else if (requestedUDDirection > 0) {
    setUDServoTicks(
      UD_DOWN_TICKS
    );
  }
  else {
    setUDServoTicks(
      UD_STOP_TICKS
    );
  }

  udPreviousMoveDirection = requestedUDDirection;
}


// ============================================================
// BUTTON WAKE INTERRUPT
// ============================================================
//
// Keep the ISR extremely small. It only wakes the button/audio
// task; all digital reads, cooldown checks, LEDs, and UART work
// remain in normal task context.
// ============================================================

void IRAM_ATTR buttonWakeISR() {

  BaseType_t higherPriorityTaskWoken = pdFALSE;

  if (buttonAudioTaskHandle != nullptr) {

    vTaskNotifyGiveFromISR(
      buttonAudioTaskHandle,
      &higherPriorityTaskWoken
    );
  }

  if (higherPriorityTaskWoken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}


// ============================================================
// BUTTON / LED UPDATE
//
// IMPORTANT:
// This function is normally called by buttonAudioTask() on
// Core 0, NOT by the TFT/game loop.
// ============================================================

void updateButtonsAndLeds() {

  unsigned long now = millis();


  // ==========================================================
  // READ PHYSICAL BUTTONS
  // ==========================================================

  bool greenButtonPhysical =
    (digitalRead(PIN_GREEN_BUTTON) == LOW);

  bool redButtonPhysical =
    (digitalRead(PIN_RED_BUTTON) == LOW);


  // ==========================================================
  // DETECT NEW PHYSICAL PRESSES
  // ==========================================================

  bool greenButtonNewPress =
    greenButtonPhysical &&
    !previousGreenButton;


  bool redButtonNewPress =
    redButtonPhysical &&
    !previousRedButton;


  // ==========================================================
  // CHECK WHETHER BUTTONS ARE ENABLED
  // ==========================================================

  bool greenButtonEnabled =
    ((long)(
      now - greenButtonDisabledUntil
    ) >= 0);


  bool redButtonEnabled =
    ((long)(
      now - redButtonDisabledUntil
    ) >= 0);


  // ==========================================================
  // GREEN BUTTON
  // ==========================================================

  if (
    greenButtonNewPress &&
    greenButtonEnabled
  ) {

    // --------------------------------------------------------
    // AUDIO FIRST.
    //
    // Send the DFPlayer command before doing LED calculations,
    // event bookkeeping, or Serial printing.  This minimizes
    // the time between the electrical button edge and the UART
    // play command.
    // --------------------------------------------------------

    int track =
      random(1, 4);


    if (dfPlayerReady) {

      // Root-folder playback. The raw command is sent
      // immediately with no library delay or ACK wait.
      playRootTrackFast(
        track
      );
    }


    // Queue one accepted green-button event for the game loop.
    portENTER_CRITICAL(
      &buttonEventMux
    );

    pendingGreenPresses++;

    portEXIT_CRITICAL(
      &buttonEventMux
    );


    unsigned long newGreenLedTimeout =
      now + GREEN_BUTTON_TIMEOUT_MS;


    unsigned long newRedLedTimeout =
      now + GREEN_BUTTON_TIMEOUT_MS;


    // --------------------------------------------------------
    // Don't shorten existing cooldowns.
    // --------------------------------------------------------

    if (
      (long)(
        newGreenLedTimeout -
        greenLedOffUntil
      ) > 0
    ) {

      greenLedOffUntil =
        newGreenLedTimeout;
    }


    if (
      (long)(
        newRedLedTimeout -
        redLedOffUntil
      ) > 0
    ) {

      redLedOffUntil =
        newRedLedTimeout;
    }


    greenButtonDisabledUntil =
      greenLedOffUntil +
      BUTTON_REARM_DELAY_MS;


    redButtonDisabledUntil =
      redLedOffUntil +
      BUTTON_REARM_DELAY_MS;


#if BUTTON_AUDIO_DEBUG
    Serial.print(
      "GREEN BUTTON ACCEPTED - track "
    );

    Serial.println(
      track
    );
#endif
  }


  // ==========================================================
  // RED BUTTON
  // ==========================================================

  if (
    redButtonNewPress &&
    redButtonEnabled
  ) {

    // AUDIO FIRST: track 4 always plays for an accepted red
    // button press.
    if (dfPlayerReady) {

      playRootTrackFast(
        4
      );
    }


    // Queue one accepted red-button event for the game loop.
    portENTER_CRITICAL(
      &buttonEventMux
    );

    pendingRedPresses++;

    portEXIT_CRITICAL(
      &buttonEventMux
    );


    unsigned long newGreenLedTimeout =
      now +
      RED_BUTTON_GREEN_TIMEOUT_MS;


    unsigned long newRedLedTimeout =
      now +
      RED_BUTTON_RED_TIMEOUT_MS;


    // --------------------------------------------------------
    // Don't shorten existing cooldowns.
    // --------------------------------------------------------

    if (
      (long)(
        newGreenLedTimeout -
        greenLedOffUntil
      ) > 0
    ) {

      greenLedOffUntil =
        newGreenLedTimeout;
    }


    if (
      (long)(
        newRedLedTimeout -
        redLedOffUntil
      ) > 0
    ) {

      redLedOffUntil =
        newRedLedTimeout;
    }


    greenButtonDisabledUntil =
      greenLedOffUntil +
      BUTTON_REARM_DELAY_MS;


    redButtonDisabledUntil =
      redLedOffUntil +
      BUTTON_REARM_DELAY_MS;


#if BUTTON_AUDIO_DEBUG
    Serial.println(
      "RED BUTTON ACCEPTED - track 4"
    );
#endif
  }


  // ==========================================================
  // UPDATE LED STATES
  // ==========================================================

  bool greenLedOn =
    ((long)(
      now - greenLedOffUntil
    ) >= 0);


  bool redLedOn =
    ((long)(
      now - redLedOffUntil
    ) >= 0);


  digitalWrite(
    PIN_GREEN_LED,
    greenLedOn ? HIGH : LOW
  );


  digitalWrite(
    PIN_RED_LED,
    redLedOn ? HIGH : LOW
  );


  // ==========================================================
  // SAVE PHYSICAL BUTTON STATES
  // ==========================================================

  previousGreenButton =
    greenButtonPhysical;


  previousRedButton =
    redButtonPhysical;
}


// ============================================================
// CORE 0 BUTTON / AUDIO TASK
// ============================================================

void buttonAudioTask(void* parameter) {

  // This task intentionally runs forever.
  for (;;) {

    updateButtonsAndLeds();

    // Normally wake every 1 ms so LED timers and RELEASED ->
    // PRESSED edge bookkeeping remain current. If either button
    // falls LOW, its GPIO ISR wakes us immediately instead of
    // waiting for that timeout.
    ulTaskNotifyTake(
      pdTRUE,
      pdMS_TO_TICKS(1)
    );
  }
}


// ============================================================
// TRANSFER ACCEPTED BUTTON EVENTS TO THE GAME LOOP
// ============================================================

void consumeButtonEvents() {

  bool greenEvent = false;
  bool redEvent = false;


  portENTER_CRITICAL(
    &buttonEventMux
  );


  if (pendingGreenPresses > 0) {

    pendingGreenPresses--;
    greenEvent = true;
  }


  if (pendingRedPresses > 0) {

    pendingRedPresses--;
    redEvent = true;
  }


  portEXIT_CRITICAL(
    &buttonEventMux
  );


  // These are true for exactly one pass of loop() when an
  // accepted event is consumed.
  greenButtonPressedThisFrame =
    greenEvent;

  redButtonPressedThisFrame =
    redEvent;
}


// ============================================================
// AIM LINE LENGTH
// ============================================================

float computeLineLength(float angleDeg) {

  float rad =
    radians(angleDeg);


  float s =
    sin(rad);


  float c =
    cos(rad);


  float lenTop =
    (BASE_Y - (RECT_TOP + MARGIN))
    / c;


  float lenSide =
    1000000.0;


  if (s > 0.001) {

    lenSide =
      (
        (RECT_RIGHT - MARGIN)
        - BASE_X
      ) / s;

  }

  else if (s < -0.001) {

    lenSide =
      (
        BASE_X
        - (RECT_LEFT + MARGIN)
      ) / (-s);

  }


  return min(
    lenTop,
    lenSide
  );
}


// ============================================================
// AIM LINE BOUNDING BOX
// ============================================================

void computeLineBBox(
  float angleDeg,
  int &x0,
  int &y0,
  int &x1,
  int &y1
) {

  float rad =
    radians(angleDeg);


  float length =
    computeLineLength(angleDeg);


  int tipX =
    BASE_X +
    (int)(
      length * sin(rad)
    );


  int tipY =
    BASE_Y -
    (int)(
      length * cos(rad)
    );


  x0 =
    min(
      BASE_X,
      tipX
    )
    - PIVOT_RADIUS
    - 2;


  x1 =
    max(
      BASE_X,
      tipX
    )
    + PIVOT_RADIUS
    + 2;


  y0 =
    min(
      BASE_Y,
      tipY
    )
    - PIVOT_RADIUS
    - 2;


  y1 =
    max(
      BASE_Y,
      tipY
    )
    + PIVOT_RADIUS
    + 2;
}


// ============================================================
// RESTORE PART OF BACKGROUND BITMAP
// ============================================================

void restoreBackgroundRect(
  int x0,
  int y0,
  int x1,
  int y1
) {

  x0 =
    max(
      0,
      x0
    );


  y0 =
    max(
      0,
      y0
    );


  x1 =
    min(
      BG_WIDTH - 1,
      x1
    );


  y1 =
    min(
      BG_HEIGHT - 1,
      y1
    );


  if (
    x1 < x0 ||
    y1 < y0
  ) {

    return;
  }


  int w =
    x1 - x0 + 1;


  for (
    int y = y0;
    y <= y1;
    y++
  ) {

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
// RESTORE PART OF START MENU BACKGROUND BITMAP
// ============================================================

void restoreStartMenuBackgroundRect(
  int x0,
  int y0,
  int x1,
  int y1
) {

  x0 = max(0, x0);
  y0 = max(0, y0);
  x1 = min(START_MENU_BG_WIDTH - 1, x1);
  y1 = min(START_MENU_BG_HEIGHT - 1, y1);

  if (x1 < x0 || y1 < y0) {
    return;
  }

  int w = x1 - x0 + 1;

  for (int y = y0; y <= y1; y++) {
    const uint16_t* rowPtr =
      startMenuBackgroundBitmap
      + y * START_MENU_BG_WIDTH
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
// DRAW PLAYER AIM LINE
// ============================================================

void drawAimLine(float angleDeg) {

  float rad =
    radians(angleDeg);


  float length =
    computeLineLength(angleDeg);


  int tipX =
    BASE_X +
    (int)(
      length * sin(rad)
    );


  int tipY =
    BASE_Y -
    (int)(
      length * cos(rad)
    );


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


// ============================================================
// DRAW PLAYER HEIGHT MARKER
// ============================================================

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
// DRAW ENEMY TRIANGLE
// ============================================================

void drawEnemyShip(
  int x,
  int y
) {

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

    currentEnemyColor()
  );
}


// ============================================================
// DRAW ENEMY HEIGHT MARKER
// ============================================================

void drawEnemyElevation(int y) {

  int halfThickness =
    ENEMY_HEIGHT_LINE_THICKNESS / 2;


  tft.fillRect(

    TICK_X_LEFT,

    y - halfThickness,

    TICK_X_RIGHT - TICK_X_LEFT,

    ENEMY_HEIGHT_LINE_THICKNESS,

    currentEnemyColor()
  );
}


void resetEnemy();


// ============================================================
// SCORE DISPLAY
// ============================================================

void drawScoreBox() {

  tft.fillRect(
    SCORE_BOX_X,
    SCORE_BOX_Y,
    SCORE_BOX_W,
    SCORE_BOX_H,
    ILI9341_BLACK
  );

  tft.drawRect(
    SCORE_BOX_X,
    SCORE_BOX_Y,
    SCORE_BOX_W,
    SCORE_BOX_H,
    ILI9341_WHITE
  );

  char scoreText[12];

  if (score < 100) {
    snprintf(scoreText, sizeof(scoreText), "%02d", score);
  }
  else {
    snprintf(scoreText, sizeof(scoreText), "%d", score);
  }

  tft.setTextSize(1);
  tft.setTextColor(ILI9341_WHITE);

  int16_t x1;
  int16_t y1;
  uint16_t w;
  uint16_t h;

  tft.getTextBounds(
    scoreText,
    0,
    0,
    &x1,
    &y1,
    &w,
    &h
  );

  tft.setCursor(
    SCORE_BOX_X + (SCORE_BOX_W - (int)w) / 2,
    SCORE_BOX_Y + (SCORE_BOX_H - (int)h) / 2
  );

  tft.print(scoreText);
}


// ============================================================
// HIT TEST
// ============================================================

bool shotHitsEnemy() {

  // Find where the player's white aim line crosses the ship's
  // current screen Y. Compare that X against the ship center X.
  float rad = radians(aimAngleDeg);
  float c = cos(rad);

  if (fabsf(c) < 0.001f) {
    return false;
  }

  float verticalDistance =
    (float)(BASE_Y - enemyY);

  // Make sure the ship's Y actually falls on the finite rendered aim line,
  // not merely on the infinite mathematical extension of that line.
  float distanceAlongAimLine =
    verticalDistance / c;

  float renderedAimLength =
    computeLineLength(aimAngleDeg);

  if (
    distanceAlongAimLine < 0.0f ||
    distanceAlongAimLine > renderedAimLength + hitXThresholdPx
  ) {
    return false;
  }

  float aimXAtEnemyY =
    (float)BASE_X + verticalDistance * tan(rad);

  float xError =
    fabsf(aimXAtEnemyY - (float)enemyX);

  int yError =
    abs(tickY - enemyElevationY);

  return (
    xError <= hitXThresholdPx &&
    yError <= hitYThresholdPx
  );
}


// ============================================================
// BEGIN / UPDATE HIT FLICKER
// ============================================================

void beginEnemyHit(bool turnsGreen) {

  if (enemyHitAnimating || gameOver) {
    return;
  }

  enemyHitTurnsGreen = turnsGreen;

  // A red-phase hit only strips the red state. Score is awarded when
  // the green ship is actually destroyed.
  if (!enemyHitTurnsGreen) {
    score++;
    drawScoreBox();

    Serial.print("HIT! Score = ");
    Serial.println(score);
  }
  else {
    Serial.println("RED HIT -> transitioning enemy to green");
  }

  enemyHitAnimating = true;
  enemyVisibleDuringHit = false;
  enemyHitTransitionCount = 0;
  enemyHitLastToggleMs = millis();
}


bool updateEnemyHitAnimation() {

  if (!enemyHitAnimating) {
    return false;
  }

  unsigned long now = millis();

  if (
    now - enemyHitLastToggleMs >=
    HIT_FLICKER_INTERVAL_MS
  ) {

    enemyHitLastToggleMs = now;
    enemyHitTransitionCount++;

    enemyVisibleDuringHit =
      !enemyVisibleDuringHit;

    if (
      enemyHitTransitionCount >=
      HIT_FLICKER_TRANSITIONS
    ) {

      enemyVisibleDuringHit = false;
      enemyHitAnimating = false;

      if (enemyHitTurnsGreen) {
        // Keep the same ship and route position, but change it to green.
        enemyIsRed = false;
        enemyHitTurnsGreen = false;

        // Green ships have a larger vertical wiggle than red ships.
        // Clamp this ship's randomized center into the GREEN-safe range so
        // its full +/-90 px motion still stays inside the elevation bar.
        {
          const int elevationHalfThickness =
            ENEMY_HEIGHT_LINE_THICKNESS / 2;

          const int greenSafeMinY =
            TICK_Y_MIN
            + elevationHalfThickness
            + (int)ceilf(GREEN_ENEMY_ELEVATION_WIGGLE_AMPLITUDE);

          const int greenSafeMaxY =
            TICK_Y_MAX
            - elevationHalfThickness
            - (int)ceilf(GREEN_ENEMY_ELEVATION_WIGGLE_AMPLITUDE);

          if (greenSafeMaxY >= greenSafeMinY) {
            enemyElevationCenterY =
              constrain(
                enemyElevationCenterY,
                (float)greenSafeMinY,
                (float)greenSafeMaxY
              );
          }
          else {
            enemyElevationCenterY =
              (TICK_Y_MIN + TICK_Y_MAX) / 2.0f;
          }
        }

        // Freeze compensation: resume from the same path position rather
        // than jumping ahead by the flicker duration.
        previousEnemyUpdate = millis();

        return true;
      }

      // Normal green hit: remove this ship and spawn the next route/ship.
      resetEnemy();

      previousEnemyUpdate = millis();

      return true;
    }
  }

  return false;
}


// ============================================================
// DRAW HEALTH BAR
//
// This is NOT drawn every frame.
// ============================================================

void drawHealthBar() {

  restoreBackgroundRect(

    HEALTH_BAR_X,
    HEALTH_BAR_Y,

    HEALTH_BAR_X
      + HEALTH_BAR_WIDTH
      - 1,

    HEALTH_BAR_Y
      + HEALTH_BAR_HEIGHT
      - 1
  );


  if (health <= 0) {

    return;
  }


  int remainingWidth =
    (
      HEALTH_BAR_WIDTH
      * health
    )
    / MAX_HEALTH;


  tft.fillRect(

    HEALTH_BAR_X,
    HEALTH_BAR_Y,

    remainingWidth,
    HEALTH_BAR_HEIGHT,

    ILI9341_RED
  );
}


// ============================================================
// GAME OVER
// ============================================================

void showGameOver() {

  gameOver = true;

  drawHealthBar();

  // Put a clean black panel over the middle of the gameplay screen.
  const int panelX = 55;
  const int panelY = 82;
  const int panelW = 210;
  const int panelH = 76;

  tft.fillRect(
    panelX,
    panelY,
    panelW,
    panelH,
    ILI9341_BLACK
  );

  // GAME OVER
  const char* gameOverText = "GAME OVER";

  tft.setTextSize(3);
  tft.setTextColor(ILI9341_RED, ILI9341_BLACK);

  int16_t x1;
  int16_t y1;
  uint16_t w;
  uint16_t h;

  tft.getTextBounds(
    gameOverText,
    0,
    0,
    &x1,
    &y1,
    &w,
    &h
  );

  int gameOverX =
    (BG_WIDTH - (int)w) / 2;

  tft.setCursor(
    gameOverX,
    panelY + 10
  );

  tft.print(gameOverText);

  // SCORE: <value>
  char scoreLine[24];
  snprintf(
    scoreLine,
    sizeof(scoreLine),
    "SCORE: %d",
    score
  );

  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE);

  tft.getTextBounds(
    scoreLine,
    0,
    0,
    &x1,
    &y1,
    &w,
    &h
  );

  int scoreX =
    (BG_WIDTH - (int)w) / 2;

  tft.setCursor(
    scoreX,
    panelY + 47
  );

  tft.print(scoreLine);
}


// ============================================================
// LOSE HEALTH
// ============================================================

void loseHealth() {

  if (gameOver) {

    return;
  }


  health--;


  if (health < 0) {

    health =
      0;
  }


  drawHealthBar();


  if (health <= 0) {

    showGameOver();
  }
}


// ============================================================
// UPDATE ENEMY POSITION
//
// Returns true when enemy reaches bottom.
// ============================================================

bool updateEnemyPosition() {

  unsigned long now = millis();

  float elapsedSeconds =
    (now - gameStartTime) / 1000.0f;

  float deltaSeconds =
    (now - previousEnemyUpdate) / 1000.0f;

  previousEnemyUpdate = now;


  // ==========================================================
  // MOVE ALONG CURRENT ROUTE
  // ==========================================================

  if (ENEMY_PATH_DURATION_SECONDS > 0.0f) {

    // Ship 1 = 1.00x, ship 2 = 1.07x, ship 3 = 1.07^2, etc.
    // This compounds the requested 7% increase from one ship to the next.
    float shipSpeedMultiplier =
      powf(
        1.0f + ENEMY_SPEED_INCREASE_PER_SHIP,
        (float)(enemyShipNumber - 1)
      );

    // Red ships travel slightly slower while they are in their red phase.
    // The wiggle calculations below are intentionally NOT multiplied by this.
    if (enemyIsRed) {
      shipSpeedMultiplier *= RED_ENEMY_SPEED_MULTIPLIER;
    }

    enemyPathProgress +=
      (deltaSeconds / ENEMY_PATH_DURATION_SECONDS)
      * shipSpeedMultiplier;
  }

  enemyPathProgress =
    constrain(enemyPathProgress, 0.0f, 1.0f);

  float baseX =
    enemyStartX +
    (enemyEndX - enemyStartX) * enemyPathProgress;

  float baseY =
    enemyStartY +
    (enemyEndY - enemyStartY) * enemyPathProgress;

  enemyX = (int)roundf(baseX);
  enemyY = (int)roundf(baseY);


  // ==========================================================
  // ENEMY ELEVATION WIGGLE
  // ==========================================================

  const float elevationAmplitude =
    enemyIsRed
      ? RED_ENEMY_ELEVATION_WIGGLE_AMPLITUDE
      : GREEN_ENEMY_ELEVATION_WIGGLE_AMPLITUDE;

  const float elevationFrequency =
    enemyIsRed
      ? RED_ENEMY_ELEVATION_WIGGLE_FREQUENCY
      : GREEN_ENEMY_ELEVATION_WIGGLE_FREQUENCY;

  float elevationPhase =
    (
      TWO_PI
      * elevationFrequency
      * elapsedSeconds
    )
    + enemyElevationPhaseOffset;

  float elevationWiggle =
    sin(elevationPhase)
    * elevationAmplitude;

  enemyElevationY =
    (int)(
      enemyElevationCenterY
      + elevationWiggle
    );

  int elevationHalfThickness =
    ENEMY_HEIGHT_LINE_THICKNESS / 2;

  enemyElevationY =
    constrain(
      enemyElevationY,
      TICK_Y_MIN + elevationHalfThickness,
      TICK_Y_MAX - elevationHalfThickness
    );


  // Reaching the end of ANY route counts as an escaped ship.
  return enemyPathProgress >= 1.0f;
}


// ============================================================
// RANDOMIZE ENEMY ELEVATION TARGET
// ============================================================
//
// Pick a new up/down target center for every newly spawned ship.
// The center itself stays inside a reduced safe range so the entire
// +/- wiggle amplitude also remains inside TICK_Y_MIN..TICK_Y_MAX.
// ============================================================

void randomizeEnemyElevationTarget() {

  const int elevationHalfThickness =
    ENEMY_HEIGHT_LINE_THICKNESS / 2;

  const float elevationAmplitude =
    enemyIsRed
      ? RED_ENEMY_ELEVATION_WIGGLE_AMPLITUDE
      : GREEN_ENEMY_ELEVATION_WIGGLE_AMPLITUDE;

  const int safeMinY =
    TICK_Y_MIN
    + elevationHalfThickness
    + (int)ceilf(elevationAmplitude);

  const int safeMaxY =
    TICK_Y_MAX
    - elevationHalfThickness
    - (int)ceilf(elevationAmplitude);

  if (safeMaxY > safeMinY) {
    enemyElevationCenterY =
      (float)random(safeMinY, safeMaxY + 1);
  }
  else {
    // Fallback if the allowable range is ever made too small for the
    // configured wiggle amplitude. Keep the target centered and safe.
    enemyElevationCenterY =
      (TICK_Y_MIN + TICK_Y_MAX) / 2.0f;
  }

  // Also randomize where in the sine wave this ship begins.
  enemyElevationPhaseOffset =
    ((float)random(0, 10000) / 10000.0f) * TWO_PI;

  Serial.print("Enemy elevation center = ");
  Serial.println(enemyElevationCenterY, 1);
}


// ============================================================
// CONFIGURE CURRENT ENEMY ROUTE
// ============================================================

void configureEnemyPath() {

  // Every newly spawned ship gets a fresh, legal elevation target.
  randomizeEnemyElevationTarget();

  const float leftX =
    RECT_LEFT + ENEMY_HALF_WIDTH + MARGIN + 1;

  const float rightX =
    RECT_RIGHT - ENEMY_HALF_WIDTH - MARGIN - 1;

  const float topY =
    RECT_TOP + ENEMY_HEIGHT / 2 + 2;

  const float bottomY =
    RECT_BOTTOM - ENEMY_HEIGHT / 2 - 2;

  const float middleX =
    (RECT_LEFT + RECT_RIGHT) / 2.0f;

  const float middleY =
    (RECT_TOP + RECT_BOTTOM) / 2.0f;

  switch (enemyPathIndex) {

    // Top middle -> bottom middle
    case 0:
      enemyStartX = middleX;
      enemyStartY = topY;
      enemyEndX   = middleX;
      enemyEndY   = bottomY;
      break;

    // Top left -> bottom right
    case 1:
      enemyStartX = leftX;
      enemyStartY = topY;
      enemyEndX   = rightX;
      enemyEndY   = bottomY;
      break;

    // Top right -> bottom left
    case 2:
      enemyStartX = rightX;
      enemyStartY = topY;
      enemyEndX   = leftX;
      enemyEndY   = bottomY;
      break;

    // Middle left -> middle right
    case 3:
      enemyStartX = leftX;
      enemyStartY = middleY;
      enemyEndX   = rightX;
      enemyEndY   = middleY;
      break;

    // Middle right -> middle left
    default:
      enemyStartX = rightX;
      enemyStartY = middleY;
      enemyEndX   = leftX;
      enemyEndY   = middleY;
      break;
  }

  enemyPathProgress = 0.0f;
  enemyX = (int)roundf(enemyStartX);
  enemyY = (int)roundf(enemyStartY);
}


// ============================================================
// RESET ENEMY TO NEXT ROUTE
// ============================================================

void resetEnemy() {

  enemyShipNumber++;
  setEnemyColorForShipNumber();

  enemyPathIndex =
    (enemyPathIndex + 1) % ENEMY_PATH_COUNT;

  configureEnemyPath();

  previousEnemyUpdate = millis();

  float shipSpeedMultiplier =
    powf(
      1.0f + ENEMY_SPEED_INCREASE_PER_SHIP,
      (float)(enemyShipNumber - 1)
    );

  if (enemyIsRed) {
    shipSpeedMultiplier *= RED_ENEMY_SPEED_MULTIPLIER;
  }

  Serial.print("Ship ");
  Serial.print(enemyShipNumber);
  Serial.print(enemyIsRed ? " RED" : " GREEN");
  Serial.print(" route speed = ");
  Serial.print(shipSpeedMultiplier, 3);
  Serial.println("x");
}


// ============================================================
// ERASE PREVIOUS ENEMY GRAPHICS
// ============================================================

void eraseOldEnemy() {

  // Enemy triangle
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


  // Enemy elevation marker
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
// START SCREEN / DIFFICULTY MENU
// ============================================================
//
// Before the game starts:
//   - TFT stays black
//   - title says "TURBOLASER" inside a thicker red box
//   - a red border surrounds the entire TFT
//   - NORMAL is selected by default
//   - joystick UP selects NORMAL (7 px)
//   - joystick DOWN selects EASY (10 px)
//   - selected option is green; the other is dim gray
//   - either red or green button starts the selected difficulty
//   - joystick does NOT move either continuous-rotation servo
//   - SG90 animation does not play
//
// The continuous-rotation servos are commanded to their calibrated
// stop values once so they remain stationary while waiting.
// ============================================================

const uint16_t START_MENU_DIM_COLOR = 0x7BEF;  // medium gray in RGB565


void drawCenteredMenuText(
  const char* text,
  int16_t y,
  uint8_t textSize,
  uint16_t color
) {

  tft.setTextWrap(false);
  tft.setTextSize(textSize);
  tft.setTextColor(color);

  int16_t x1;
  int16_t y1;
  uint16_t w;
  uint16_t h;

  tft.getTextBounds(
    text,
    0,
    0,
    &x1,
    &y1,
    &w,
    &h
  );

  int16_t x =
    (tft.width() - (int16_t)w) / 2;

  tft.setCursor(x, y);
  tft.print(text);
}


void drawDifficultyOptions() {

  // Restore only the menu-option area from the subtle background so
  // changing difficulty does not disturb the title or instruction text.
  restoreStartMenuBackgroundRect(
    0,
    135,
    tft.width() - 1,
    205
  );

  uint16_t normalColor =
    (selectedDifficulty == DIFFICULTY_NORMAL)
      ? ILI9341_GREEN
      : START_MENU_DIM_COLOR;

  uint16_t easyColor =
    (selectedDifficulty == DIFFICULTY_EASY)
      ? ILI9341_GREEN
      : START_MENU_DIM_COLOR;

  drawCenteredMenuText(
    "Normal",
    145,
    1,
    normalColor
  );

  drawCenteredMenuText(
    "Easy",
    170,
    1,
    easyColor
  );
}


void drawStartScreen() {

  // Subtle Death Star background for the startup menu.
  tft.drawRGBBitmap(
    0,
    0,
    startMenuBackgroundBitmap,
    START_MENU_BG_WIDTH,
    START_MENU_BG_HEIGHT
  );

  // Red border around the entire screen.
  // Two nested rectangles make this border 2 pixels thick.
  for (int i = 0; i < 2; i++) {
    tft.drawRect(
      i,
      i,
      tft.width() - i * 2,
      tft.height() - i * 2,
      ILI9341_RED
    );
  }

  // Large centered title.
  const char* titleText = "TURBOLASER";
  const uint8_t titleSize = 4;
  const int16_t titleY = 42;

  tft.setTextWrap(false);
  tft.setTextSize(titleSize);

  int16_t titleX1;
  int16_t titleY1;
  uint16_t titleW;
  uint16_t titleH;

  tft.getTextBounds(
    titleText,
    0,
    0,
    &titleX1,
    &titleY1,
    &titleW,
    &titleH
  );

  const int16_t titleX =
    (tft.width() - (int16_t)titleW) / 2;

  const int16_t titlePadX = 8;
  const int16_t titlePadY = 6;

  // Thicker red outline around the title.
  // Draw three nested rectangles so the border is 3 pixels thick.
  for (int i = 0; i < 3; i++) {
    tft.drawRect(
      titleX - titlePadX - i,
      titleY - titlePadY - i,
      titleW + titlePadX * 2 + i * 2,
      titleH + titlePadY * 2 + i * 2,
      ILI9341_RED
    );
  }

  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(titleX, titleY);
  tft.print(titleText);

  // Smaller instruction line.
  drawCenteredMenuText(
    "press to start",
    105,
    2,
    ILI9341_WHITE
  );

  drawDifficultyOptions();
}


void applySelectedDifficulty() {

  if (selectedDifficulty == DIFFICULTY_EASY) {
    hitXThresholdPx = EASY_HIT_THRESHOLD_PX;
    hitYThresholdPx = EASY_HIT_THRESHOLD_PX;
  }
  else {
    hitXThresholdPx = NORMAL_HIT_THRESHOLD_PX;
    hitYThresholdPx = NORMAL_HIT_THRESHOLD_PX;
  }
}


void waitForStartButton() {

  // Ignore the joystick for servo movement while waiting.
  // Keep the movement servos parked at neutral.
  stopJoystickServos();

  selectedDifficulty = DIFFICULTY_NORMAL;
  applySelectedDifficulty();
  drawDifficultyOptions();

  Serial.println(
    "Start menu: NORMAL selected (7 px). Use joystick UP/DOWN, then press a button to start."
  );

  bool previousUp = false;
  bool previousDown = false;

  while (true) {

    bool upPressed =
      (digitalRead(PIN_UP) == LOW);

    bool downPressed =
      (digitalRead(PIN_DOWN) == LOW);

    // Use edge detection so holding the joystick does not repeatedly redraw.
    if (upPressed && !previousUp) {

      if (selectedDifficulty != DIFFICULTY_NORMAL) {
        selectedDifficulty = DIFFICULTY_NORMAL;
        applySelectedDifficulty();
        drawDifficultyOptions();

        Serial.println(
          "Difficulty selected: NORMAL (7 px)"
        );
      }
    }

    if (downPressed && !previousDown) {

      if (selectedDifficulty != DIFFICULTY_EASY) {
        selectedDifficulty = DIFFICULTY_EASY;
        applySelectedDifficulty();
        drawDifficultyOptions();

        Serial.println(
          "Difficulty selected: EASY (10 px)"
        );
      }
    }

    previousUp = upPressed;
    previousDown = downPressed;

    // Difficulty is now chosen by the menu, so either physical shot button
    // can simply start the game.
    bool greenStart =
      (digitalRead(PIN_GREEN_BUTTON) == LOW);

    bool redStart =
      (digitalRead(PIN_RED_BUTTON) == LOW);

    if (greenStart || redStart) {

      delay(30);

      // Wait for whichever start button was pressed to be released so the
      // menu press cannot become the first shot after gameplay begins.
      while (
        digitalRead(PIN_GREEN_BUTTON) == LOW ||
        digitalRead(PIN_RED_BUTTON) == LOW
      ) {
        delay(5);
      }

      delay(30);

      applySelectedDifficulty();

      Serial.print("Game starting: ");

      if (selectedDifficulty == DIFFICULTY_EASY) {
        Serial.println("EASY, 10 px hit threshold");
      }
      else {
        Serial.println("NORMAL, 7 px hit threshold");
      }

      return;
    }

    delay(5);
  }
}


// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(
    115200
  );


  // ==========================================================
  // JOYSTICK
  // ==========================================================

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


  // ==========================================================
  // BUTTONS
  //
  // GPIO34 / GPIO35 require external 10k pull-ups.
  // ==========================================================

  pinMode(
    PIN_RED_BUTTON,
    INPUT
  );


  pinMode(
    PIN_GREEN_BUTTON,
    INPUT
  );


  // ==========================================================
  // LEDS
  // ==========================================================

  pinMode(
    PIN_GREEN_LED,
    OUTPUT
  );


  pinMode(
    PIN_RED_LED,
    OUTPUT
  );


  pinMode(
    PIN_ANIMATION_LED,
    OUTPUT
  );


  // Button LEDs start ON.
  digitalWrite(
    PIN_GREEN_LED,
    HIGH
  );


  digitalWrite(
    PIN_RED_LED,
    HIGH
  );


  // Standalone GPIO25 LED is only on during servo animation.
  digitalWrite(
    PIN_ANIMATION_LED,
    LOW
  );


  // ==========================================================
  // PCA9685 + SG90 ANIMATION SERVO
  // ==========================================================

  Wire.begin(
    PCA9685_SDA,
    PCA9685_SCL
  );

  pwm.begin();

  // SG90 servos use a standard 50 Hz control signal.
  pwm.setPWMFreq(
    50
  );

  delay(
    10
  );

  // Do NOT command the animation servo yet. It remains untouched
  // while the start screen is showing.

  // Start both continuous-rotation joystick servos at their
  // calibrated neutral/stop values.
  stopJoystickServos();


  // ==========================================================
  // DFPLAYER MINI - DIRECT UART / NO LIBRARY DELAY
  // ==========================================================

  dfSerial.begin(
    9600,
    SERIAL_8N1,
    DFPLAYER_RX,
    DFPLAYER_TX
  );


  // The DFPlayer boots independently after power-up. Mark the
  // UART path ready now. We set volume after the TFT/background
  // setup below has naturally given the module more boot time.
  dfPlayerReady =
    true;


  // ==========================================================
  // TFT
  // ==========================================================

  tft.begin();


  tft.setRotation(
    1
  );


  // ==========================================================
  // START SCREEN
  // ==========================================================

  drawStartScreen();

  // Nothing in the game runs until the player chooses NORMAL/EASY with
  // the joystick and presses either shot button to start. The joystick is
  // used only for menu selection here; the servos stay parked at neutral.
  waitForStartButton();


  // ==========================================================
  // START GAME
  // ==========================================================

  // The start button has now been released. Initialize the SG90 at
  // its resting position only now, so it is untouched on the start screen.
  setAnimationServoAngle(
    SERVO_REST_ANGLE
  );

  // The physical turret is assumed to be centered when the game starts.
  // Reset the software travel estimate here, not while sitting on the
  // start screen.
  lrTravelPositionMs = 0;
  lrPreviousMoveDirection = 0;
  lrLastTravelUpdateMs = millis();

  // The up/down turret is also assumed to be physically centered when
  // gameplay begins. Start its open-loop position estimate at center.
  udTravelPositionMs = 0;
  udPreviousMoveDirection = 0;
  udLastTravelUpdateMs = millis();

  aimAngleDeg = 0.0f;
  tickY = (TICK_Y_MIN + TICK_Y_MAX) / 2;

  // Replace the black start screen with the normal game bitmap.
  tft.drawRGBBitmap(

    0,
    0,

    backgroundBitmap,

    BG_WIDTH,
    BG_HEIGHT
  );


  // ==========================================================
  // INITIAL PLAYER STATE
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


  // ==========================================================
  // ENEMY TIMING
  // ==========================================================

  gameStartTime =
    millis();


  previousEnemyUpdate =
    gameStartTime;


  // The first ship always starts on route 0: top middle -> bottom middle.
  enemyPathIndex = 0;
  enemyShipNumber = 1;
  setEnemyColorForShipNumber();
  configureEnemyPath();

  updateEnemyPosition();


  prevEnemyX =
    enemyX;


  prevEnemyY =
    enemyY;


  prevEnemyElevationY =
    enemyElevationY;


  // ==========================================================
  // INITIAL DRAWING
  // ==========================================================

  drawEnemyShip(
    enemyX,
    enemyY
  );


  drawEnemyElevation(
    enemyElevationY
  );


  drawAimLine(
    aimAngleDeg
  );


  drawTick(
    tickY
  );


  score = 0;
  enemyHitAnimating = false;
  enemyVisibleDuringHit = true;
  enemyHitTransitionCount = 0;

  drawHealthBar();
  drawScoreBox();


  // ==========================================================
  // DFPLAYER VOLUME
  // ==========================================================
  //
  // By this point the TFT/background initialization has given
  // the DFPlayer additional boot time. Send volume once here.
  // This delay happens only at startup and has no effect on
  // button-to-sound latency during gameplay.
  // ==========================================================

  delay(
    250
  );

  setDFPlayerVolumeFast(
    20
  );

  // Give the one-time volume command time to leave the UART
  // before button playback can begin.
  delay(
    20
  );


  // ==========================================================
  // START LOW-LATENCY BUTTON / AUDIO TASK ON CORE 0
  // ==========================================================

  BaseType_t taskResult =
    xTaskCreatePinnedToCore(
      buttonAudioTask,
      "ButtonAudioTask",
      4096,
      nullptr,
      4,
      &buttonAudioTaskHandle,
      0
    );


  if (taskResult == pdPASS) {

    buttonAudioTaskStarted =
      true;


    // Wake the Core-0 audio task the instant either active-LOW
    // button is pressed. The task still performs the real work.
    attachInterrupt(
      digitalPinToInterrupt(PIN_GREEN_BUTTON),
      buttonWakeISR,
      FALLING
    );

    attachInterrupt(
      digitalPinToInterrupt(PIN_RED_BUTTON),
      buttonWakeISR,
      FALLING
    );

    buttonWakeInterruptsAttached =
      true;


    Serial.println(
      "Fast button/audio task + GPIO wake interrupts ready"
    );
  }

  else {

    buttonAudioTaskStarted =
      false;

    Serial.println(
      "WARNING: Core 0 task failed; using main-loop fallback"
    );
  }
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop() {

  // ==========================================================
  // BUTTON / AUDIO EVENTS
  //
  // Normally the physical buttons, LEDs, and DFPlayer are all
  // handled independently on Core 0.
  //
  // If the FreeRTOS task failed to start for any reason, this
  // fallback keeps the buttons working from the main loop.
  // ==========================================================

  if (!buttonAudioTaskStarted) {

    updateButtonsAndLeds();
  }


  // Pull any accepted button events into the familiar
  // one-frame flags for future shooting/gameplay logic.
  consumeButtonEvents();


  // ==========================================================
  // SG90 BUTTON ANIMATION
  //
  // Either ACCEPTED button press restarts the recoil animation.
  // This is intentionally non-blocking so TFT/game/audio timing
  // continues normally while the servo moves.
  // ==========================================================

  if (redButtonPressedThisFrame) {

    // Half speed = twice the normal animation duration.
    triggerServoAnimation(2.0f);
  }
  else if (greenButtonPressedThisFrame) {

    triggerServoAnimation(1.0f);
  }


  updateServoAnimation();


  // ==========================================================
  // READ JOYSTICK
  // ==========================================================

  bool up =
    (
      digitalRead(PIN_UP)
      == LOW
    );


  bool down =
    (
      digitalRead(PIN_DOWN)
      == LOW
    );


  bool left =
    (
      digitalRead(PIN_LEFT)
      == LOW
    );


  bool right =
    (
      digitalRead(PIN_RIGHT)
      == LOW
    );


  // Move the calibrated continuous-rotation servos from the same joystick.
  updateJoystickServos(
    left,
    right,
    up,
    down
  );


  // ==========================================================
  // GAME OVER CONTROL MODE
  // ==========================================================
  //
  // After GAME OVER, the physical controls remain live:
  //   - joystick still drives the LR and UD servos
  //   - red/green buttons still run their normal LEDs/audio
  //   - accepted button presses still trigger the SG90 animation
  //
  // But we return here before changing aimAngleDeg, tickY,
  // enemy position, health, or any other gameplay state.
  // ==========================================================

  if (gameOver) {

    delay(
      10
    );

    return;
  }


  bool aimChanged =
    false;


  bool tickChanged =
    false;


  // ==========================================================
  // PLAYER AIM - SYNCHRONIZED TO PHYSICAL LR TURRET
  // ==========================================================
  // The physical LR turret already tracks its estimated position as
  // elapsed movement time from the startup center:
  //
  //   -1500 ms = full left  = -75 degrees on TFT
  //       0 ms = center     =   0 degrees on TFT
  //   +1500 ms = full right = +75 degrees on TFT
  //
  // Because both the physical limit and the display use the SAME
  // lrTravelPositionMs value, a complete left-to-right sweep takes
  // exactly 3000 ms and the TFT line stays synchronized with the
  // software-estimated physical turret position.

  float newAimAngleDeg =
    ((float)lrTravelPositionMs / (float)LR_TRAVEL_LIMIT_MS)
    * MAX_ANGLE_DEG;

  newAimAngleDeg = constrain(
    newAimAngleDeg,
    -MAX_ANGLE_DEG,
    MAX_ANGLE_DEG
  );

  if (fabsf(newAimAngleDeg - aimAngleDeg) > 0.001f) {
    aimAngleDeg = newAimAngleDeg;
    aimChanged = true;
  }


  // ==========================================================
  // PLAYER HEIGHT - SYNCHRONIZED TO PHYSICAL UP/DOWN TURRET
  // ==========================================================
  // The physical UD turret tracks its estimated position as elapsed
  // movement time from the startup center:
  //
  //   -1100 ms = full up   = TICK_Y_MIN
  //       0 ms = center    = middle of the elevation bar
  //   +1100 ms = full down = TICK_Y_MAX
  //
  // Both the physical servo limit and the TFT elevation marker use the
  // SAME udTravelPositionMs value, so a full top-to-bottom sweep takes
  // exactly 2200 ms and the display stays synchronized with the
  // software-estimated physical turret position.

  float udNormalized =
    ((float)udTravelPositionMs + (float)UD_TRAVEL_LIMIT_MS) /
    (2.0f * (float)UD_TRAVEL_LIMIT_MS);

  udNormalized = constrain(udNormalized, 0.0f, 1.0f);

  int newTickY =
    (int)roundf(
      (float)TICK_Y_MIN +
      udNormalized * (float)(TICK_Y_MAX - TICK_Y_MIN)
    );

  newTickY = constrain(
    newTickY,
    TICK_Y_MIN,
    TICK_Y_MAX
  );

  if (newTickY != tickY) {
    tickY = newTickY;
    tickChanged = true;
  }


  // ==========================================================
  // SHOOTING / HIT DETECTION
  // ==========================================================
  // Red enemies can ONLY be advanced with the RED button.
  // Once a red enemy is hit, it flickers twice and becomes green at the
  // same route position. Green enemies keep the normal behavior and may
  // be hit by either accepted gameplay button.
  // Every hit still requires BOTH horizontal aim and elevation alignment.

  bool correctShotButton = false;

  if (enemyIsRed) {
    correctShotButton = redButtonPressedThisFrame;
  }
  else {
    correctShotButton =
      greenButtonPressedThisFrame ||
      redButtonPressedThisFrame;
  }

  if (
    !enemyHitAnimating &&
    correctShotButton &&
    shotHitsEnemy()
  ) {

    beginEnemyHit(enemyIsRed);
  }


  // ==========================================================
  // ERASE OLD DYNAMIC GRAPHICS
  // ==========================================================

  eraseOldEnemy();


  if (aimChanged) {

    restoreBackgroundRect(

      prevLineX0,
      prevLineY0,

      prevLineX1,
      prevLineY1
    );
  }


  if (tickChanged) {

    restoreBackgroundRect(

      TICK_X_LEFT - 1,

      prevTickY - 2,

      TICK_X_RIGHT + 1,

      prevTickY + 2
    );
  }


  // ==========================================================
  // UPDATE ENEMY / HIT FLICKER
  // ==========================================================

  bool enemyEscaped = false;

  if (enemyHitAnimating) {

    // Freeze the ship while it flickers. This routine resets the
    // enemy on the next route automatically after two quick flickers.
    updateEnemyHitAnimation();
  }
  else {

    enemyEscaped =
      updateEnemyPosition();
  }


  // ==========================================================
  // ENEMY REACHED BOTTOM
  // ==========================================================

  if (enemyEscaped) {

    loseHealth();


    if (gameOver) {

      return;
    }


    resetEnemy();
  }


  // ==========================================================
  // DRAW ENEMY
  // ==========================================================

  if (
    !enemyHitAnimating ||
    enemyVisibleDuringHit
  ) {

    drawEnemyShip(
      enemyX,
      enemyY
    );


    drawEnemyElevation(
      enemyElevationY
    );
  }


  // ==========================================================
  // REDRAW PLAYER CONTROLS
  // ==========================================================

  drawAimLine(
    aimAngleDeg
  );


  drawTick(
    tickY
  );


  // ==========================================================
  // STORE CURRENT POSITIONS
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


  // ==========================================================
  // FRAME DELAY
  // ==========================================================

  delay(
    FRAME_DELAY_MS
  );
}
