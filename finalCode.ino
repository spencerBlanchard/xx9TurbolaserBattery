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

#define PIN_UP     27
#define PIN_DOWN   26
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

const uint16_t LR_LEFT_TICKS  = 353;
const uint16_t LR_STOP_TICKS  = 373;
const uint16_t LR_RIGHT_TICKS = 383;

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


// Enemy center path
const float ENEMY_START_X =
  (RECT_LEFT + RECT_RIGHT) / 2.0;


// Enemy vertical position
float enemyBaseY =
  RECT_TOP + ENEMY_HEIGHT / 2 + 2;


// Actual screen position
int enemyX;
int enemyY;


// ============================================================
// ENEMY - LEFT / HEIGHT SECTION
// ============================================================

const int ENEMY_HEIGHT_LINE_THICKNESS = 9;


// Base enemy elevation
const float ENEMY_ELEVATION_CENTER =
  (TICK_Y_MIN + TICK_Y_MAX) / 2.0;


// Elevation wiggle
const float ENEMY_ELEVATION_WIGGLE_AMPLITUDE = 40.0;
const float ENEMY_ELEVATION_WIGGLE_FREQUENCY = 0.03;


int enemyElevationY;


// ============================================================
// COLORS
// ============================================================

const uint16_t ENEMY_COLOR = ILI9341_GREEN;


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
//   2) the white elevation tick is close to the green elevation marker
// Adjust these two values later if you want hits tighter/looser.
const int HIT_X_THRESHOLD_PX = 7;
const int HIT_Y_THRESHOLD_PX = 10;

// Quick two-flicker hit animation. Enemy movement pauses during it.
const unsigned long HIT_FLICKER_INTERVAL_MS = 70;
const int HIT_FLICKER_TRANSITIONS = 4; // off/on/off/on, then disappear + respawn

bool enemyHitAnimating = false;
bool enemyVisibleDuringHit = true;
unsigned long enemyHitLastToggleMs = 0;
int enemyHitTransitionCount = 0;


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


void triggerServoAnimation() {

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
      (float)SERVO_PRELOAD_MS;

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

    if (elapsed >= SERVO_KICK_HOLD_MS) {

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
      (float)SERVO_RETURN_MS;

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
  // ----------------------------------------------------------

  if (up && !down) {

    setUDServoTicks(
      UD_UP_TICKS
    );
  }
  else if (down && !up) {

    setUDServoTicks(
      UD_DOWN_TICKS
    );
  }
  else {

    // Releasing the joystick, or pressing both directions,
    // always returns to the calibrated neutral.
    setUDServoTicks(
      UD_STOP_TICKS
    );
  }
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

    ENEMY_COLOR
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

    ENEMY_COLOR
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
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);

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
    distanceAlongAimLine > renderedAimLength + HIT_X_THRESHOLD_PX
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
    xError <= HIT_X_THRESHOLD_PX &&
    yError <= HIT_Y_THRESHOLD_PX
  );
}


// ============================================================
// BEGIN / UPDATE HIT FLICKER
// ============================================================

void beginEnemyHit() {

  if (enemyHitAnimating || gameOver) {
    return;
  }

  score++;
  drawScoreBox();

  enemyHitAnimating = true;
  enemyVisibleDuringHit = false;
  enemyHitTransitionCount = 0;
  enemyHitLastToggleMs = millis();

  Serial.print("HIT! Score = ");
  Serial.println(score);
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

      // End invisible, then spawn the next ship at the top.
      enemyVisibleDuringHit = false;
      enemyHitAnimating = false;

      resetEnemy();

      // Reset update timing so the new ship does not jump down by
      // the amount of time spent flickering.
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
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);

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

  unsigned long now =
    millis();


  float elapsedSeconds =
    (
      now - gameStartTime
    )
    / 1000.0;


  float deltaSeconds =
    (
      now - previousEnemyUpdate
    )
    / 1000.0;


  previousEnemyUpdate =
    now;


  // ==========================================================
  // MOVE ENEMY DOWN
  // ==========================================================

  enemyBaseY +=
    ENEMY_DESCENT_SPEED
    * deltaSeconds;


  enemyY =
    (int)enemyBaseY;


  // ==========================================================
  // HORIZONTAL WIGGLE
  // ==========================================================

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


  enemyX =
    constrain(

      enemyX,

      RECT_LEFT
        + ENEMY_HALF_WIDTH
        + MARGIN,

      RECT_RIGHT
        - ENEMY_HALF_WIDTH
        - MARGIN
    );


  // ==========================================================
  // ENEMY ELEVATION WIGGLE
  // ==========================================================

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


  enemyElevationY =
    constrain(

      enemyElevationY,

      TICK_Y_MIN
        + elevationHalfThickness,

      TICK_Y_MAX
        - elevationHalfThickness
    );


  // ==========================================================
  // CHECK BOTTOM
  // ==========================================================

  if (
    enemyY >=
    RECT_BOTTOM
      - ENEMY_HEIGHT / 2
  ) {

    return true;
  }


  return false;
}


// ============================================================
// RESET ENEMY TO TOP
// ============================================================

void resetEnemy() {

  enemyBaseY =
    RECT_TOP
    + ENEMY_HEIGHT / 2
    + 2;


  enemyY =
    (int)enemyBaseY;


  previousEnemyUpdate =
    millis();
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
// START SCREEN
// ============================================================
//
// Before the game starts:
//   - TFT stays black and shows "PRESS TO START"
//   - joystick does NOT move either continuous-rotation servo
//   - red button does nothing
//   - green button starts the game
//   - SG90 animation does not play
//
// The continuous-rotation servos are commanded to their calibrated
// stop values once so they remain stationary while waiting.
// ============================================================

void drawStartScreen() {

  tft.fillScreen(
    ILI9341_BLACK
  );

  tft.setTextWrap(
    false
  );

  tft.setTextColor(
    ILI9341_WHITE,
    ILI9341_BLACK
  );

  tft.setTextSize(
    2
  );

  const char* message = "press to start";

  int16_t x1;
  int16_t y1;
  uint16_t w;
  uint16_t h;

  tft.getTextBounds(
    message,
    0,
    0,
    &x1,
    &y1,
    &w,
    &h
  );

  int16_t x =
    (tft.width() - (int16_t)w) / 2;

  int16_t y =
    (tft.height() - (int16_t)h) / 2;

  tft.setCursor(
    x,
    y
  );

  tft.print(
    message
  );
}


void waitForGreenStart() {

  // Ignore the joystick completely while waiting.
  // Keep the movement servos parked at neutral.
  stopJoystickServos();

  Serial.println(
    "Waiting for GREEN button to start..."
  );

  // Active LOW because GPIO35 has the external pull-up resistor.
  while (digitalRead(PIN_GREEN_BUTTON) != LOW) {
    delay(5);
  }

  // Small debounce.
  delay(30);

  // Wait for release so the start press does not immediately become
  // the first gameplay button press after the button task begins.
  while (digitalRead(PIN_GREEN_BUTTON) == LOW) {
    delay(5);
  }

  delay(30);
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

  // Nothing in the game runs until the green button is pressed.
  // The joystick is ignored during this wait, so the servos stay still.
  waitForGreenStart();


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

  if (
    greenButtonPressedThisFrame ||
    redButtonPressedThisFrame
  ) {

    triggerServoAnimation();
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
  // PLAYER HEIGHT
  // ==========================================================

  if (up) {

    tickY -=
      TICK_STEP;


    tickChanged =
      true;
  }


  if (down) {

    tickY +=
      TICK_STEP;


    tickChanged =
      true;
  }


  tickY =
    constrain(

      tickY,

      TICK_Y_MIN,

      TICK_Y_MAX
    );


  // ==========================================================
  // SHOOTING / HIT DETECTION
  // ==========================================================
  // Either accepted gameplay button is treated as a shot.
  // A hit requires BOTH horizontal aim and elevation to be close.

  if (
    !enemyHitAnimating &&
    (
      greenButtonPressedThisFrame ||
      redButtonPressedThisFrame
    ) &&
    shotHitsEnemy()
  ) {

    beginEnemyHit();
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
    // enemy to the top automatically after two quick flickers.
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
