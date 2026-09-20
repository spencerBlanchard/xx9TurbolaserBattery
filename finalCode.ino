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
//   Normal and shielded X-wing sprites move through the aim section
//   A matching horizontal marker shows enemy elevation on the left
//
// HEALTH:
//   Three hearts at the top-right
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
#include <Preferences.h>
#include "calibration_frames.h"
#include <math.h>
#include "background_bitmap.h"
#include "start_menu_background_bitmap.h"
#include "xwing_sprites.h"


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
const uint16_t LR_STOP_TICKS  = 370;
const uint16_t LR_RIGHT_TICKS = 381;

const uint16_t UD_UP_TICKS    = 390;
const uint16_t UD_STOP_TICKS  = 369;
const uint16_t UD_DOWN_TICKS  = 353;

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
// Full up limit   = -1000 ms from center
// Full down limit = +1000 ms from center
//
// This makes the complete physical top-to-bottom command-time sweep
// exactly 2.0 seconds, matching the TFT elevation marker.
// ============================================================

const int32_t UD_TRAVEL_LIMIT_MS = 1000;

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

// Activated by the one-time S+ target in absolute round 6.
volatile bool turbolaserSpeedBoostActive = false;
const float SPEED_BOOST_COOLDOWN_MULTIPLIER = 0.75f;


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
bool redButtonCooldownPressedThisFrame = false;


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
volatile uint32_t pendingRedCooldownPresses = 0;


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

const int ENEMY_HALF_WIDTH = XWING_WIDTH / 2;
const int ENEMY_HEIGHT     = XWING_HEIGHT;


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

// Each round is 5% faster than the round before it, continuing across
// repeated 10-round cycles. Two-ship rounds also use a 70% speed factor.
const float ENEMY_SPEED_INCREASE_PER_ROUND = 0.05f;
const float MULTI_ENEMY_SPEED_MULTIPLIER = 0.70f;

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

// Absolute round number. The 10-round pattern repeats forever while this
// value keeps increasing so the 5% speed increase continues accumulating.
uint32_t enemyShipNumber = 1;
bool enemyIsRed = false;
int enemyHealth = 1;
bool primaryEnemyActive = true;

uint8_t roundInCycle() {
  return ((enemyShipNumber - 1) % 10) + 1;
}

bool roundHasTwoEnemies() {
  uint8_t round = roundInCycle();
  return round == 7 || round == 9;
}

uint16_t currentEnemyColor() {
  return enemyIsRed ? ENEMY_RED_COLOR : ENEMY_GREEN_COLOR;
}

void setEnemyColorForShipNumber() {
  uint8_t round = roundInCycle();
  enemyIsRed =
    round == 3 || round == 4 || round == 6 ||
    round == 8 || round == 9 || round == 10;
  enemyHealth = enemyIsRed ? 3 : 1;
}

// Independent second target used only in rounds 7 and 9.
bool enemy2Active = false;
bool enemy2IsRed = false;
int enemy2Health = 1;
uint8_t enemy2PathIndex = 0;
float enemy2PathProgress = 0.0f;
float enemy2StartX = 0.0f;
float enemy2StartY = 0.0f;
float enemy2EndX = 0.0f;
float enemy2EndY = 0.0f;
int enemy2X = 0;
int enemy2Y = 0;
float enemy2ElevationCenterY = (TICK_Y_MIN + TICK_Y_MAX) / 2.0f;
float enemy2ElevationPhaseOffset = 0.0f;
int enemy2ElevationY = 0;
unsigned long previousEnemy2Update = 0;
int prevEnemy2X = 0;
int prevEnemy2Y = 0;
int prevEnemy2ElevationY = 0;

bool enemy2HitAnimating = false;
bool enemy2VisibleDuringHit = true;
unsigned long enemy2HitLastToggleMs = 0;
int enemy2HitTransitionCount = 0;
unsigned long enemy2HitFlickerIntervalMs = 70;
int enemy2HitFlickerTransitions = 4;
bool enemy2HitDestroys = false;
bool enemy2HitRemovesShield = false;
bool enemy2HitWasShielded = false;


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

// Small top-left score box. Scores 0-9 are shown as 00-09.
const int SCORE_BOX_X = 2;
const int SCORE_BOX_Y = 2;
const int SCORE_BOX_W = 46;
const int SCORE_BOX_H = 13;

// Three hearts in the top-right replace the old bottom health bar.
const int HEARTS_BOX_X = 272;
const int HEARTS_BOX_Y = 2;
const int HEARTS_BOX_W = 46;
const int HEARTS_BOX_H = 13;

// Easy-mode tutorial messages use the otherwise empty strip between the
// score and hearts. They never pause movement, aiming, or button input.
const int TUTORIAL_BOX_X = 50;
const int TUTORIAL_BOX_Y = 2;
const int TUTORIAL_BOX_W = 220;
const int TUTORIAL_BOX_H = 13;
const unsigned long TUTORIAL_MESSAGE_DURATION_MS = 5000;
const unsigned long TUTORIAL_PAGE_DURATION_MS = 1500;
const unsigned long TUTORIAL_ARROW_BLINK_MS = 200;

enum TutorialMessage {
  TUTORIAL_NONE,
  TUTORIAL_ELEVATION,
  TUTORIAL_SHIELD,
  TUTORIAL_RED_COOLDOWN,
  TUTORIAL_BUTTON_LIGHTS,
  TUTORIAL_SPEED_BOOST
};

TutorialMessage activeTutorialMessage = TUTORIAL_NONE;
unsigned long tutorialMessageStartedMs = 0;
int tutorialLastPage = -1;
bool tutorialArrowVisible = false;
unsigned long tutorialArrowLastToggleMs = 0;
uint32_t lastElevationHintShipNumber = 0;
bool shieldHintShown = false;
bool redCooldownHintShown = false;
bool buttonLightsHintShown = false;
uint32_t firstRedCooldownHintShipNumber = 0;
uint32_t lastAcceptedRedOnShieldedShip = 0;

// Stationary speed-boost target; it appears only during absolute round 6.
bool speedBoostTargetActive = false;
int speedBoostX = 0;
int speedBoostY = 0;
int prevSpeedBoostX = 0;
int prevSpeedBoostY = 0;
const int SPEED_BOOST_RADIUS = 13;

// A shot counts when BOTH targeting dimensions are close enough:
//   1) the white aim line passes close to the ship's X position
//   2) the white elevation tick is close to the enemy elevation marker
//
// Difficulty is selected from the joystick-controlled start menu.
// HARD uses the tighter 7 px threshold.
// NORMAL uses the more forgiving 10 px threshold.
const int HARD_HIT_THRESHOLD_PX   = 7;
const int NORMAL_HIT_THRESHOLD_PX = 10;

enum GameDifficulty {
  DIFFICULTY_NORMAL,
  DIFFICULTY_HARD
};

GameDifficulty selectedDifficulty = DIFFICULTY_NORMAL;

int hitXThresholdPx = NORMAL_HIT_THRESHOLD_PX;
int hitYThresholdPx = NORMAL_HIT_THRESHOLD_PX;

enum MainMenuOption {
  MENU_NORMAL,
  MENU_HARD,
  MENU_HIGH_SCORES,
  MENU_OPTION_COUNT
};

MainMenuOption selectedMenuOption = MENU_NORMAL;

const int HIGH_SCORE_COUNT = 5;

struct HighScoreEntry {
  char initials[4];
  uint32_t score;
  uint32_t timeMs;
};

HighScoreEntry normalHighScores[HIGH_SCORE_COUNT];
HighScoreEntry hardHighScores[HIGH_SCORE_COUNT];
Preferences highScorePreferences;
bool highScoresReady = false;

const char* HIGH_SCORE_NAMESPACE = "turbolaser";
const char* NORMAL_SCORES_KEY = "normal";
const char* HARD_SCORES_KEY = "hard";

uint32_t currentGameElapsedMs = 0;

bool highScoreComesBefore(
  uint32_t newScore,
  uint32_t newTimeMs,
  const HighScoreEntry& existing
);
HighScoreEntry* scoresForDifficulty(GameDifficulty difficulty);
int qualifyingHighScoreIndex(
  GameDifficulty difficulty,
  uint32_t candidateScore,
  uint32_t candidateTimeMs
);
void insertHighScore(
  GameDifficulty difficulty,
  const char* initials,
  uint32_t newScore,
  uint32_t newTimeMs
);
void drawHighScoreScreen(GameDifficulty difficulty);
void showHighScoreBrowser(GameDifficulty shownDifficulty);
void drawCenteredMenuText(
  const char* text,
  int16_t y,
  uint8_t textSize,
  uint16_t color
);
void finishGameOverFlow();
void getShotAlignment(bool& horizontalAligned, bool& verticalAligned);
void showTutorialMessage(TutorialMessage message);
void updateTutorialMessage();

// Non-destroying hits keep the original quick two-flicker feedback.
const unsigned long DAMAGE_FLICKER_INTERVAL_MS = 70;
const int DAMAGE_FLICKER_TRANSITIONS = 4;

// Destruction: explosion -> X-wing -> explosion -> next enemy.
// Three 500 ms phases take 1.5 seconds total.
const unsigned long DESTROY_FLICKER_INTERVAL_MS = 500;
const int DESTROY_FLICKER_TRANSITIONS = 3;

bool enemyHitAnimating = false;
bool enemyVisibleDuringHit = true;
unsigned long enemyHitLastToggleMs = 0;
int enemyHitTransitionCount = 0;
unsigned long enemyHitFlickerIntervalMs = DAMAGE_FLICKER_INTERVAL_MS;
int enemyHitFlickerTransitions = DAMAGE_FLICKER_TRANSITIONS;

// Normal X-wings have 1 HP. Shielded X-wings have 3 HP.
// Green-button shots deal 1 damage; red-button shots deal 3 damage.
bool enemyHitDestroys = false;
bool enemyHitRemovesShield = false;
bool enemyHitWasShielded = false;
const int EXPLOSION_WIDTH = 30;
const int EXPLOSION_HEIGHT = 20;
const int SHIELD_EXPLOSION_WIDTH = 39;  // 1.3 x normal
const int SHIELD_EXPLOSION_HEIGHT = 26;


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


// Enemy X-wing sprite
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


// Stop generating PWM pulses on every servo channel. This is different
// from sending the calibrated neutral values: PCA9685 off=4096 sets the
// channel's FULL OFF bit, so the servos receive no control signal at all.
void disableAllServoSignals() {

  servoAnimationState =
    SERVO_ANIM_IDLE;

  digitalWrite(
    PIN_ANIMATION_LED,
    LOW
  );

  pwm.setPWM(
    ANIMATION_SERVO_CHANNEL,
    0,
    4096
  );

  pwm.setPWM(
    UD_SERVO_CHANNEL,
    0,
    4096
  );

  pwm.setPWM(
    LR_SERVO_CHANNEL,
    0,
    4096
  );

  lastLRServoTicks = -1;
  lastUDServoTicks = -1;
  lrPreviousMoveDirection = 0;
  udPreviousMoveDirection = 0;
  lrLastTravelUpdateMs = millis();
  udLastTravelUpdateMs = millis();
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
      now + (unsigned long)(
        GREEN_BUTTON_TIMEOUT_MS *
        (turbolaserSpeedBoostActive ? SPEED_BOOST_COOLDOWN_MULTIPLIER : 1.0f)
      );


    unsigned long newRedLedTimeout =
      now + (unsigned long)(
        GREEN_BUTTON_TIMEOUT_MS *
        (turbolaserSpeedBoostActive ? SPEED_BOOST_COOLDOWN_MULTIPLIER : 1.0f)
      );


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
      (unsigned long)(
        RED_BUTTON_GREEN_TIMEOUT_MS *
        (turbolaserSpeedBoostActive ? SPEED_BOOST_COOLDOWN_MULTIPLIER : 1.0f)
      );


    unsigned long newRedLedTimeout =
      now +
      (unsigned long)(
        RED_BUTTON_RED_TIMEOUT_MS *
        (turbolaserSpeedBoostActive ? SPEED_BOOST_COOLDOWN_MULTIPLIER : 1.0f)
      );


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

  // Let the game tutorial know that the player physically pressed RED
  // while it was still disabled. This is not an accepted shot: it does
  // not play audio, restart LEDs, or trigger recoil.
  if (
    redButtonNewPress &&
    !redButtonEnabled
  ) {
    portENTER_CRITICAL(
      &buttonEventMux
    );

    pendingRedCooldownPresses++;

    portEXIT_CRITICAL(
      &buttonEventMux
    );
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
  bool redCooldownEvent = false;


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


  if (pendingRedCooldownPresses > 0) {

    pendingRedCooldownPresses--;
    redCooldownEvent = true;
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

  redButtonCooldownPressedThisFrame =
    redCooldownEvent;
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

uint16_t gameplayBackgroundColor(uint16_t color) {
  if (selectedDifficulty != DIFFICULTY_HARD) {
    return color;
  }

  uint8_t red = (color >> 11) & 0x1F;
  uint8_t green = (color >> 5) & 0x3F;
  uint8_t blue = color & 0x1F;

  // The supplied background is black with blue/cyan artwork. Convert only
  // pixels whose blue component dominates; neutral UI colors remain intact.
  if (blue > red && (uint16_t)blue * 2 >= green) {
    uint8_t redIntensity = max(blue, (uint8_t)(green / 2));
    return ((uint16_t)redIntensity << 11);
  }

  return color;
}


void drawGameplayBackground() {
  if (selectedDifficulty != DIFFICULTY_HARD) {
    tft.drawRGBBitmap(0, 0, backgroundBitmap, BG_WIDTH, BG_HEIGHT);
    return;
  }

  uint16_t convertedRow[BG_WIDTH];
  for (int y = 0; y < BG_HEIGHT; y++) {
    for (int x = 0; x < BG_WIDTH; x++) {
      convertedRow[x] = gameplayBackgroundColor(
        pgm_read_word(backgroundBitmap + y * BG_WIDTH + x)
      );
    }
    tft.drawRGBBitmap(0, y, convertedRow, BG_WIDTH, 1);
  }
}

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
      backgroundBitmap + y * BG_WIDTH + x0;

    if (selectedDifficulty == DIFFICULTY_HARD) {
      // Dynamic erasing must use the same red conversion as the initial
      // Hard-mode draw or moving objects would reveal blue trails.
      uint16_t convertedRow[BG_WIDTH];
      for (int x = 0; x < w; x++) {
        convertedRow[x] = gameplayBackgroundColor(
          pgm_read_word(rowPtr + x)
        );
      }
      tft.drawRGBBitmap(x0, y, convertedRow, w, 1);
    }
    else {
      tft.drawRGBBitmap(x0, y, rowPtr, w, 1);
    }
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
// DRAW ENEMY X-WING
// ============================================================

void drawEnemyShipAt(
  int x,
  int y,
  bool shielded
) {

  const uint16_t* pixels =
    shielded
      ? XWING_SHIELD_PIXELS
      : XWING_NORMAL_PIXELS;

  const uint8_t* mask =
    shielded
      ? XWING_SHIELD_MASK
      : XWING_NORMAL_MASK;

  tft.drawRGBBitmap(
    x - XWING_WIDTH / 2,
    y - XWING_HEIGHT / 2,
    pixels,
    mask,
    XWING_WIDTH,
    XWING_HEIGHT
  );
}

void drawEnemyShip(int x, int y) {
  drawEnemyShipAt(x, y, enemyIsRed);
}


// ============================================================
// DRAW ENEMY HEIGHT MARKER
// ============================================================

// Pixel-art source, scaled to the requested explosion footprint.
// Dots are transparent; background restoration erases the previous frame.
const char EXPLOSION_PIXELS[15][24] PROGMEM = {
  ".....R...........R.....",
  "......RR...R...RR......",
  "..R...ROR.ROR.ROR...R..",
  "...RR.ROOROOROOR.RR....",
  "....ROOOYYYYYOOOR......",
  "RRROOOYYYYWYYYYOOORRR..",
  "..ROOYYWWWWWWWYYOOR....",
  "...OOYYWWWWWWWYYOO.....",
  "..ROOYYWWWWWWWYYOOR....",
  "RRROOOYYYYWYYYYOOORRR..",
  "....ROOOYYYYYOOOR......",
  "...RR.ROOROOROOR.RR....",
  "..R...ROR.ROR.ROR...R..",
  "......RR...R...RR......",
  ".....R...........R.....",
};

void drawEnemyExplosionAt(int x, int y, bool wasShielded) {
  const int width = wasShielded ? SHIELD_EXPLOSION_WIDTH : EXPLOSION_WIDTH;
  const int height = wasShielded ? SHIELD_EXPLOSION_HEIGHT : EXPLOSION_HEIGHT;
  tft.startWrite();
  for (int row = 0; row < height; ++row) {
    for (int col = 0; col < width; ++col) {
      int px = x - width / 2 + col;
      int py = y - height / 2 + row;
      if (px <= RECT_LEFT || px >= RECT_RIGHT ||
          py <= RECT_TOP || py >= RECT_BOTTOM) continue;
      char pixel = pgm_read_byte(&EXPLOSION_PIXELS[row * 15 / height][col * 23 / width]);
      if (pixel == '.') continue;
      uint16_t color = ILI9341_WHITE;
      if (pixel == 'R') color = ILI9341_RED;
      else if (pixel == 'O') color = 0xFD20;
      else if (pixel == 'Y') color = ILI9341_YELLOW;
      tft.writePixel(px, py, color);
    }
  }
  tft.endWrite();
}

void drawEnemyExplosion(int x, int y) {
  drawEnemyExplosionAt(x, y, enemyHitWasShielded);
}

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

void drawEnemyElevationAt(int y, bool shielded) {
  int halfThickness = ENEMY_HEIGHT_LINE_THICKNESS / 2;
  tft.fillRect(
    TICK_X_LEFT,
    y - halfThickness,
    TICK_X_RIGHT - TICK_X_LEFT,
    ENEMY_HEIGHT_LINE_THICKNESS,
    shielded ? ENEMY_RED_COLOR : ENEMY_GREEN_COLOR
  );
}


void drawSpeedBoostTarget() {
  if (!speedBoostTargetActive) return;

  tft.fillCircle(speedBoostX, speedBoostY, SPEED_BOOST_RADIUS, ILI9341_GREEN);
  tft.drawCircle(speedBoostX, speedBoostY, SPEED_BOOST_RADIUS, ILI9341_WHITE);
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_BLACK);
  tft.setCursor(speedBoostX - 6, speedBoostY - 3);
  tft.print("S+");
}


void resetEnemy();
void finishRoundIfReady();
void eraseSpeedBoostTarget();


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
// EASY-MODE TUTORIAL MESSAGES
// ============================================================

void clearTutorialArrow() {
  restoreBackgroundRect(36, 23, 48, 47);
  tutorialArrowVisible = false;
}


void drawTutorialArrow(bool bright) {
  clearTutorialArrow();

  // Point left toward the elevation aiming box without covering it.
  tft.fillTriangle(
    36, 35,
    48, 23,
    48, 47,
    bright ? ILI9341_YELLOW : ILI9341_WHITE
  );

  // The notch gives the larger arrow a bold chevron shape.
  tft.fillTriangle(
    39, 35,
    48, 29,
    48, 41,
    ILI9341_BLACK
  );

  tutorialArrowVisible = true;
}


void drawTutorialTextPage(int page) {
  tft.fillRect(
    TUTORIAL_BOX_X,
    TUTORIAL_BOX_Y,
    TUTORIAL_BOX_W,
    TUTORIAL_BOX_H,
    ILI9341_BLACK
  );

  tft.setTextWrap(false);
  tft.setTextSize(1);

  const char* text = "";

  if (activeTutorialMessage == TUTORIAL_ELEVATION) {
    text = page == 0
      ? "MATCH BOTH AIM INDICATORS!"
      : "Use the LEFT elevation box.";
  }
  else if (activeTutorialMessage == TUTORIAL_SHIELD) {
    // Center and color only the word RED.
    const char* before = "Use ";
    const char* redWord = "RED";
    const char* after = " on shielded X-wings";
    const int totalWidth =
      (strlen(before) + strlen(redWord) + strlen(after)) * 6;
    int x = TUTORIAL_BOX_X + (TUTORIAL_BOX_W - totalWidth) / 2;

    tft.setCursor(x, TUTORIAL_BOX_Y + 3);
    tft.setTextColor(ILI9341_WHITE);
    tft.print(before);
    tft.setTextColor(ILI9341_RED);
    tft.print(redWord);
    tft.setTextColor(ILI9341_WHITE);
    tft.print(after);
    return;
  }
  else if (activeTutorialMessage == TUTORIAL_RED_COOLDOWN) {
    text = page == 0
      ? "RED shots recharge more slowly."
      : "Wait before pressing RED again.";
  }
  else if (activeTutorialMessage == TUTORIAL_BUTTON_LIGHTS) {
    text = page == 0
      ? "Watch the physical buttons."
      : "The light shows RED is ready.";
  }
  else if (activeTutorialMessage == TUTORIAL_SPEED_BOOST) {
    text = "Turbolaser speed boost!";
  }

  int16_t x1;
  int16_t y1;
  uint16_t w;
  uint16_t h;
  tft.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
  tft.setTextColor(
    activeTutorialMessage == TUTORIAL_RED_COOLDOWN && page == 0
      ? ILI9341_RED
      : ILI9341_WHITE
  );
  tft.setCursor(
    TUTORIAL_BOX_X + (TUTORIAL_BOX_W - (int)w) / 2,
    TUTORIAL_BOX_Y + 3
  );
  tft.print(text);
}


void showTutorialMessage(TutorialMessage message) {
  if (
    message == TUTORIAL_NONE ||
    (selectedDifficulty != DIFFICULTY_NORMAL && message != TUTORIAL_SPEED_BOOST)
  ) {
    return;
  }

  if (activeTutorialMessage == TUTORIAL_ELEVATION) {
    clearTutorialArrow();
  }

  activeTutorialMessage = message;
  tutorialMessageStartedMs = millis();
  tutorialArrowLastToggleMs = tutorialMessageStartedMs;
  tutorialLastPage = -1;
  tutorialArrowVisible = false;
}


void updateTutorialMessage() {
  if (activeTutorialMessage == TUTORIAL_NONE) {
    return;
  }

  unsigned long now = millis();
  unsigned long elapsed = now - tutorialMessageStartedMs;

  if (elapsed >= TUTORIAL_MESSAGE_DURATION_MS) {
    if (activeTutorialMessage == TUTORIAL_ELEVATION) {
      clearTutorialArrow();
    }

    restoreBackgroundRect(
      TUTORIAL_BOX_X,
      TUTORIAL_BOX_Y,
      TUTORIAL_BOX_X + TUTORIAL_BOX_W - 1,
      TUTORIAL_BOX_Y + TUTORIAL_BOX_H - 1
    );

    activeTutorialMessage = TUTORIAL_NONE;
    tutorialLastPage = -1;
    return;
  }

  int page = (elapsed / TUTORIAL_PAGE_DURATION_MS) % 2;
  if (page != tutorialLastPage) {
    drawTutorialTextPage(page);
    tutorialLastPage = page;
  }

  if (
    activeTutorialMessage == TUTORIAL_ELEVATION &&
    now - tutorialArrowLastToggleMs >= TUTORIAL_ARROW_BLINK_MS
  ) {
    tutorialArrowLastToggleMs = now;
    if (tutorialArrowVisible) {
      clearTutorialArrow();
    }
    else {
      drawTutorialArrow(true);
    }
  }
}


// ============================================================
// HIT TEST
// ============================================================

void getShotAlignment(bool& horizontalAligned, bool& verticalAligned) {
  horizontalAligned = false;
  verticalAligned =
    abs(tickY - enemyElevationY) <= hitYThresholdPx;

  // Find where the player's white aim line crosses the ship's
  // current screen Y. Compare that X against the ship center X.
  float rad = radians(aimAngleDeg);
  float c = cos(rad);

  if (fabsf(c) < 0.001f) {
    return;
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
    return;
  }

  float aimXAtEnemyY =
    (float)BASE_X + verticalDistance * tan(rad);

  float xError =
    fabsf(aimXAtEnemyY - (float)enemyX);

  horizontalAligned = xError <= hitXThresholdPx;
}


bool shotHitsEnemy() {
  bool horizontalAligned;
  bool verticalAligned;
  getShotAlignment(horizontalAligned, verticalAligned);
  return horizontalAligned && verticalAligned;
}


void getEnemy2ShotAlignment(bool& horizontalAligned, bool& verticalAligned) {
  horizontalAligned = false;
  verticalAligned =
    abs(tickY - enemy2ElevationY) <= hitYThresholdPx;

  float rad = radians(aimAngleDeg);
  float c = cos(rad);
  if (fabsf(c) < 0.001f) return;

  float verticalDistance = (float)(BASE_Y - enemy2Y);
  float distanceAlongAimLine = verticalDistance / c;
  float renderedAimLength = computeLineLength(aimAngleDeg);
  if (
    distanceAlongAimLine < 0.0f ||
    distanceAlongAimLine > renderedAimLength + hitXThresholdPx
  ) return;

  float aimXAtEnemyY =
    (float)BASE_X + verticalDistance * tan(rad);
  horizontalAligned =
    fabsf(aimXAtEnemyY - (float)enemy2X) <= hitXThresholdPx;
}


bool shotHitsSpeedBoost() {
  if (!speedBoostTargetActive) return false;

  float rad = radians(aimAngleDeg);
  float c = cos(rad);
  if (fabsf(c) < 0.001f) return false;

  float verticalDistance = (float)(BASE_Y - speedBoostY);
  float distanceAlongAimLine = verticalDistance / c;
  if (
    distanceAlongAimLine < 0.0f ||
    distanceAlongAimLine > computeLineLength(aimAngleDeg) + SPEED_BOOST_RADIUS
  ) return false;

  float aimXAtTargetY =
    (float)BASE_X + verticalDistance * tan(rad);
  return fabsf(aimXAtTargetY - (float)speedBoostX) <= SPEED_BOOST_RADIUS;
}


// ============================================================
// BEGIN / UPDATE HIT FLICKER
// ============================================================

void beginEnemyHit(int damage) {

  if (enemyHitAnimating || gameOver) {
    return;
  }

  enemyHitWasShielded = enemyIsRed;
  enemyHealth =
    max(
      0,
      enemyHealth - damage
    );

  enemyHitDestroys =
    (enemyHealth <= 0);

  // A shielded X-wing changes to the normal sprite only after its
  // second 1-damage hit leaves it with exactly 1 HP.
  enemyHitRemovesShield =
    enemyIsRed &&
    !enemyHitDestroys &&
    enemyHealth == 1;

  if (enemyHitDestroys) {
    enemyHitFlickerIntervalMs = DESTROY_FLICKER_INTERVAL_MS;
    enemyHitFlickerTransitions = DESTROY_FLICKER_TRANSITIONS;
    Serial.println("LETHAL HIT! Beginning 1.5-second destruction sequence");
  }
  else {
    enemyHitFlickerIntervalMs = DAMAGE_FLICKER_INTERVAL_MS;
    enemyHitFlickerTransitions = DAMAGE_FLICKER_TRANSITIONS;
    Serial.print("ENEMY HIT! Remaining HP = ");
    Serial.println(enemyHealth);
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
    enemyHitFlickerIntervalMs
  ) {

    enemyHitLastToggleMs = now;
    enemyHitTransitionCount++;

    enemyVisibleDuringHit =
      !enemyVisibleDuringHit;

    if (
      enemyHitTransitionCount >=
      enemyHitFlickerTransitions
    ) {

      enemyVisibleDuringHit = false;
      enemyHitAnimating = false;

      if (enemyHitDestroys) {
        enemyHitDestroys = false;
        enemyHitRemovesShield = false;

        // Award the point after the final explosion phase finishes.
        score++;
        drawScoreBox();

        Serial.print("ENEMY DESTROYED! Score = ");
        Serial.println(score);

        // Remove only this target. Two-target rounds do not advance until
        // both ships have independently been destroyed or escaped.
        primaryEnemyActive = false;
        finishRoundIfReady();

        previousEnemyUpdate = millis();

        return true;
      }

      if (enemyHitRemovesShield) {
        // Keep the same ship and route position, but reveal the normal
        // X-wing sprite after the shield has taken two green hits.
        enemyIsRed = false;
        enemyHitRemovesShield = false;

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

      // The shield still has 2 HP after its first green-button hit.
      // Resume the same sprite and route without jumping ahead.
      previousEnemyUpdate = millis();

      return true;
    }
  }

  return false;
}


void beginEnemy2Hit(int damage) {
  if (enemy2HitAnimating || !enemy2Active || gameOver) return;

  enemy2HitWasShielded = enemy2IsRed;
  enemy2Health = max(0, enemy2Health - damage);
  enemy2HitDestroys = enemy2Health <= 0;
  enemy2HitRemovesShield =
    enemy2IsRed && !enemy2HitDestroys && enemy2Health == 1;

  enemy2HitFlickerIntervalMs = enemy2HitDestroys
    ? DESTROY_FLICKER_INTERVAL_MS
    : DAMAGE_FLICKER_INTERVAL_MS;
  enemy2HitFlickerTransitions = enemy2HitDestroys
    ? DESTROY_FLICKER_TRANSITIONS
    : DAMAGE_FLICKER_TRANSITIONS;
  enemy2HitAnimating = true;
  enemy2VisibleDuringHit = false;
  enemy2HitTransitionCount = 0;
  enemy2HitLastToggleMs = millis();
}


void updateEnemy2HitAnimation() {
  if (!enemy2HitAnimating) return;

  unsigned long now = millis();
  if (
    now - enemy2HitLastToggleMs < enemy2HitFlickerIntervalMs
  ) return;

  enemy2HitLastToggleMs = now;
  enemy2HitTransitionCount++;
  enemy2VisibleDuringHit = !enemy2VisibleDuringHit;

  if (enemy2HitTransitionCount < enemy2HitFlickerTransitions) return;

  enemy2VisibleDuringHit = false;
  enemy2HitAnimating = false;

  if (enemy2HitDestroys) {
    enemy2HitDestroys = false;
    enemy2HitRemovesShield = false;
    enemy2Active = false;
    score++;
    drawScoreBox();
    finishRoundIfReady();
    return;
  }

  if (enemy2HitRemovesShield) {
    enemy2IsRed = false;
    enemy2HitRemovesShield = false;
  }

  // Do not jump forward after a non-lethal animation.
  previousEnemy2Update = millis();
}


// ============================================================
// DRAW PLAYER HEALTH HEARTS
//
// These are updated only when player health changes.
// ============================================================

void drawHeart(int x, int y) {

  tft.fillCircle(
    x + 2,
    y + 2,
    2,
    ILI9341_RED
  );

  tft.fillCircle(
    x + 6,
    y + 2,
    2,
    ILI9341_RED
  );

  tft.fillTriangle(
    x,
    y + 2,
    x + 8,
    y + 2,
    x + 4,
    y + 8,
    ILI9341_RED
  );
}


void drawHealthHearts() {

  tft.fillRect(
    HEARTS_BOX_X,
    HEARTS_BOX_Y,
    HEARTS_BOX_W,
    HEARTS_BOX_H,
    ILI9341_BLACK
  );

  tft.drawRect(
    HEARTS_BOX_X,
    HEARTS_BOX_Y,
    HEARTS_BOX_W,
    HEARTS_BOX_H,
    ILI9341_WHITE
  );

  for (int i = 0; i < health; i++) {
    drawHeart(
      HEARTS_BOX_X + 6 + i * 13,
      HEARTS_BOX_Y + 2
    );
  }
}


// ============================================================
// GAME OVER
// ============================================================

void showGameOver() {

  gameOver = true;
  currentGameElapsedMs = millis() - gameStartTime;

  // Remove the control signal from all three servos immediately. Sending
  // neutral pulses can still allow an occasional twitch; FULL OFF cannot.
  disableAllServoSignals();

  drawHealthHearts();

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

  finishGameOverFlow();
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


  drawHealthHearts();


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

    // The absolute round number keeps the 5% increase accumulating even
    // when the 10-round pattern loops.
    float shipSpeedMultiplier =
      powf(
        1.0f + ENEMY_SPEED_INCREASE_PER_ROUND,
        (float)(enemyShipNumber - 1)
      );

    if (roundHasTwoEnemies()) {
      shipSpeedMultiplier *= MULTI_ENEMY_SPEED_MULTIPLIER;
    }

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

  // Vertical routes and diagonals sway left/right. Pure side-to-side
  // routes (3 and 4) bob up/down on the TFT instead.
  const float wiggleEnvelope =
    sinf(PI * enemyPathProgress);

  const float horizontalWiggle =
    sinf(TWO_PI * ENEMY_X_WIGGLE_FREQUENCY * elapsedSeconds)
    * ENEMY_X_WIGGLE_AMPLITUDE
    * wiggleEnvelope;

  const float leftX =
    RECT_LEFT + ENEMY_HALF_WIDTH + MARGIN + 1;

  const float rightX =
    RECT_RIGHT - ENEMY_HALF_WIDTH - MARGIN - 1;

  if (enemyPathIndex == 3 || enemyPathIndex == 4) {
    const float topY = RECT_TOP + ENEMY_HEIGHT / 2 + 2;
    const float bottomY = RECT_BOTTOM - ENEMY_HEIGHT / 2 - 2;
    baseY = constrain(baseY + horizontalWiggle, topY, bottomY);
  }
  else {
    baseX = constrain(baseX + horizontalWiggle, leftX, rightX);
  }

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


void randomizeEnemy2ElevationTarget() {
  const int halfThickness = ENEMY_HEIGHT_LINE_THICKNESS / 2;
  const float amplitude = enemy2IsRed
    ? RED_ENEMY_ELEVATION_WIGGLE_AMPLITUDE
    : GREEN_ENEMY_ELEVATION_WIGGLE_AMPLITUDE;
  const int safeMin =
    TICK_Y_MIN + halfThickness + (int)ceilf(amplitude);
  const int safeMax =
    TICK_Y_MAX - halfThickness - (int)ceilf(amplitude);

  enemy2ElevationCenterY = safeMax > safeMin
    ? (float)random(safeMin, safeMax + 1)
    : (TICK_Y_MIN + TICK_Y_MAX) / 2.0f;
  enemy2ElevationPhaseOffset =
    ((float)random(0, 10000) / 10000.0f) * TWO_PI;
}


void configureEnemy2Path() {
  randomizeEnemy2ElevationTarget();

  const float leftX = RECT_LEFT + ENEMY_HALF_WIDTH + MARGIN + 1;
  const float rightX = RECT_RIGHT - ENEMY_HALF_WIDTH - MARGIN - 1;
  const float topY = RECT_TOP + ENEMY_HEIGHT / 2 + 2;
  const float bottomY = RECT_BOTTOM - ENEMY_HEIGHT / 2 - 2;
  const float middleX = (RECT_LEFT + RECT_RIGHT) / 2.0f;
  const float middleY = (RECT_TOP + RECT_BOTTOM) / 2.0f;

  switch (enemy2PathIndex) {
    case 0:
      enemy2StartX = middleX; enemy2StartY = topY;
      enemy2EndX = middleX; enemy2EndY = bottomY;
      break;
    case 1:
      enemy2StartX = leftX; enemy2StartY = topY;
      enemy2EndX = rightX; enemy2EndY = bottomY;
      break;
    case 2:
      enemy2StartX = rightX; enemy2StartY = topY;
      enemy2EndX = leftX; enemy2EndY = bottomY;
      break;
    case 3:
      enemy2StartX = leftX; enemy2StartY = middleY;
      enemy2EndX = rightX; enemy2EndY = middleY;
      break;
    default:
      enemy2StartX = rightX; enemy2StartY = middleY;
      enemy2EndX = leftX; enemy2EndY = middleY;
      break;
  }

  enemy2PathProgress = 0.0f;
  enemy2X = (int)roundf(enemy2StartX);
  enemy2Y = (int)roundf(enemy2StartY);
  previousEnemy2Update = millis();
}


bool updateEnemy2Position() {
  if (!enemy2Active) return false;

  unsigned long now = millis();
  float elapsedSeconds = (now - gameStartTime) / 1000.0f;
  float deltaSeconds = (now - previousEnemy2Update) / 1000.0f;
  previousEnemy2Update = now;

  float speedMultiplier =
    powf(
      1.0f + ENEMY_SPEED_INCREASE_PER_ROUND,
      (float)(enemyShipNumber - 1)
    ) * MULTI_ENEMY_SPEED_MULTIPLIER;
  if (enemy2IsRed) speedMultiplier *= RED_ENEMY_SPEED_MULTIPLIER;

  enemy2PathProgress +=
    (deltaSeconds / ENEMY_PATH_DURATION_SECONDS) * speedMultiplier;
  enemy2PathProgress = constrain(enemy2PathProgress, 0.0f, 1.0f);

  float baseX = enemy2StartX +
    (enemy2EndX - enemy2StartX) * enemy2PathProgress;
  float baseY = enemy2StartY +
    (enemy2EndY - enemy2StartY) * enemy2PathProgress;
  float wiggle =
    sinf(TWO_PI * ENEMY_X_WIGGLE_FREQUENCY * elapsedSeconds + PI)
    * ENEMY_X_WIGGLE_AMPLITUDE
    * sinf(PI * enemy2PathProgress);

  const float leftX = RECT_LEFT + ENEMY_HALF_WIDTH + MARGIN + 1;
  const float rightX = RECT_RIGHT - ENEMY_HALF_WIDTH - MARGIN - 1;
  const float topY = RECT_TOP + ENEMY_HEIGHT / 2 + 2;
  const float bottomY = RECT_BOTTOM - ENEMY_HEIGHT / 2 - 2;

  if (enemy2PathIndex == 3 || enemy2PathIndex == 4) {
    baseY = constrain(baseY + wiggle, topY, bottomY);
  }
  else {
    baseX = constrain(baseX + wiggle, leftX, rightX);
  }

  enemy2X = (int)roundf(baseX);
  enemy2Y = (int)roundf(baseY);

  float amplitude = enemy2IsRed
    ? RED_ENEMY_ELEVATION_WIGGLE_AMPLITUDE
    : GREEN_ENEMY_ELEVATION_WIGGLE_AMPLITUDE;
  float frequency = enemy2IsRed
    ? RED_ENEMY_ELEVATION_WIGGLE_FREQUENCY
    : GREEN_ENEMY_ELEVATION_WIGGLE_FREQUENCY;
  enemy2ElevationY = (int)(
    enemy2ElevationCenterY +
    sinf(TWO_PI * frequency * elapsedSeconds + enemy2ElevationPhaseOffset)
      * amplitude
  );
  const int halfThickness = ENEMY_HEIGHT_LINE_THICKNESS / 2;
  enemy2ElevationY = constrain(
    enemy2ElevationY,
    TICK_Y_MIN + halfThickness,
    TICK_Y_MAX - halfThickness
  );

  return enemy2PathProgress >= 1.0f;
}


void spawnSpeedBoostTarget() {
  speedBoostTargetActive = (enemyShipNumber == 6);
  if (!speedBoostTargetActive) return;

  // Choose a different edge region from the primary route whenever possible.
  uint8_t side = (enemyPathIndex + random(1, 4)) % 4;
  const int pad = SPEED_BOOST_RADIUS + 8;
  if (side == 0) {
    speedBoostX = random(RECT_LEFT + pad, RECT_RIGHT - pad);
    speedBoostY = RECT_TOP + pad;
  }
  else if (side == 1) {
    speedBoostX = RECT_RIGHT - pad;
    speedBoostY = random(RECT_TOP + pad, RECT_BOTTOM - pad);
  }
  else if (side == 2) {
    speedBoostX = random(RECT_LEFT + pad, RECT_RIGHT - pad);
    speedBoostY = RECT_BOTTOM - pad;
  }
  else {
    speedBoostX = RECT_LEFT + pad;
    speedBoostY = random(RECT_TOP + pad, RECT_BOTTOM - pad);
  }
  prevSpeedBoostX = speedBoostX;
  prevSpeedBoostY = speedBoostY;
}


// ============================================================
// START THE NEXT ROUND
// ============================================================

void resetEnemy() {

  enemyShipNumber++;
  setEnemyColorForShipNumber();

  primaryEnemyActive = true;

  enemyPathIndex =
    (enemyPathIndex + 1) % ENEMY_PATH_COUNT;

  configureEnemyPath();

  enemy2Active = roundHasTwoEnemies();
  // Round 7: both normal. Round 9: primary shielded, secondary normal.
  enemy2IsRed = false;
  enemy2Health = enemy2IsRed ? 3 : 1;
  enemy2PathIndex = (enemyPathIndex + 2) % ENEMY_PATH_COUNT;
  enemy2HitAnimating = false;
  enemy2HitDestroys = false;
  enemy2HitRemovesShield = false;
  if (enemy2Active) configureEnemy2Path();

  spawnSpeedBoostTarget();

  previousEnemyUpdate = millis();

  float shipSpeedMultiplier =
    powf(
      1.0f + ENEMY_SPEED_INCREASE_PER_ROUND,
      (float)(enemyShipNumber - 1)
    );

  if (roundHasTwoEnemies()) {
    shipSpeedMultiplier *= MULTI_ENEMY_SPEED_MULTIPLIER;
  }

  if (enemyIsRed) {
    shipSpeedMultiplier *= RED_ENEMY_SPEED_MULTIPLIER;
  }

  Serial.print("Round ");
  Serial.print(enemyShipNumber);
  Serial.print(enemyIsRed ? " RED" : " GREEN");
  Serial.print(" route speed = ");
  Serial.print(shipSpeedMultiplier, 3);
  Serial.println("x");
}


void finishRoundIfReady() {
  if (!primaryEnemyActive && !enemy2Active &&
      !enemyHitAnimating && !enemy2HitAnimating) {
    if (speedBoostTargetActive) {
      eraseSpeedBoostTarget();
    }
    speedBoostTargetActive = false;
    resetEnemy();
  }
}


// ============================================================
// ERASE PREVIOUS ENEMY GRAPHICS
// ============================================================

void eraseOldEnemy() {

  // Clear the largest explosion footprint too, including after a shield
  // transition or respawn. Keep restoration inside the playfield.
  restoreBackgroundRect(
    max(RECT_LEFT + 1, prevEnemyX - SHIELD_EXPLOSION_WIDTH / 2 - 2),
    max(RECT_TOP + 1, prevEnemyY - SHIELD_EXPLOSION_HEIGHT / 2 - 2),
    min(RECT_RIGHT - 1, prevEnemyX + SHIELD_EXPLOSION_WIDTH / 2 + 2),
    min(RECT_BOTTOM - 1, prevEnemyY + SHIELD_EXPLOSION_HEIGHT / 2 + 2)
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


void eraseOldEnemy2() {
  if (!roundHasTwoEnemies() && !enemy2HitAnimating) return;

  restoreBackgroundRect(
    max(RECT_LEFT + 1, prevEnemy2X - SHIELD_EXPLOSION_WIDTH / 2 - 2),
    max(RECT_TOP + 1, prevEnemy2Y - SHIELD_EXPLOSION_HEIGHT / 2 - 2),
    min(RECT_RIGHT - 1, prevEnemy2X + SHIELD_EXPLOSION_WIDTH / 2 + 2),
    min(RECT_BOTTOM - 1, prevEnemy2Y + SHIELD_EXPLOSION_HEIGHT / 2 + 2)
  );

  int halfThickness = ENEMY_HEIGHT_LINE_THICKNESS / 2;
  restoreBackgroundRect(
    TICK_X_LEFT - 1,
    prevEnemy2ElevationY - halfThickness - 2,
    TICK_X_RIGHT + 1,
    prevEnemy2ElevationY + halfThickness + 2
  );
}


void eraseSpeedBoostTarget() {
  restoreBackgroundRect(
    max(RECT_LEFT + 1, prevSpeedBoostX - SPEED_BOOST_RADIUS - 2),
    max(RECT_TOP + 1, prevSpeedBoostY - SPEED_BOOST_RADIUS - 2),
    min(RECT_RIGHT - 1, prevSpeedBoostX + SPEED_BOOST_RADIUS + 2),
    min(RECT_BOTTOM - 1, prevSpeedBoostY + SPEED_BOOST_RADIUS + 2)
  );
}


// ============================================================
// PERSISTENT HIGH SCORES
// ============================================================

bool highScoreComesBefore(
  uint32_t newScore,
  uint32_t newTimeMs,
  const HighScoreEntry& existing
) {

  if (existing.initials[0] == '\0') {
    return true;
  }

  if (newScore != existing.score) {
    return newScore > existing.score;
  }

  return newTimeMs < existing.timeMs;
}


void loadHighScores() {

  memset(normalHighScores, 0, sizeof(normalHighScores));
  memset(hardHighScores, 0, sizeof(hardHighScores));

  highScoresReady =
    highScorePreferences.begin(
      HIGH_SCORE_NAMESPACE,
      false
    );

  if (!highScoresReady) {
    Serial.println("WARNING: high-score flash storage unavailable");
    return;
  }

  if (
    highScorePreferences.getBytesLength(NORMAL_SCORES_KEY) ==
    sizeof(normalHighScores)
  ) {
    highScorePreferences.getBytes(
      NORMAL_SCORES_KEY,
      normalHighScores,
      sizeof(normalHighScores)
    );
  }

  if (
    highScorePreferences.getBytesLength(HARD_SCORES_KEY) ==
    sizeof(hardHighScores)
  ) {
    highScorePreferences.getBytes(
      HARD_SCORES_KEY,
      hardHighScores,
      sizeof(hardHighScores)
    );
  }

  for (int i = 0; i < HIGH_SCORE_COUNT; i++) {
    normalHighScores[i].initials[3] = '\0';
    hardHighScores[i].initials[3] = '\0';
  }
}


HighScoreEntry* scoresForDifficulty(GameDifficulty difficulty) {
  return
    difficulty == DIFFICULTY_HARD
      ? hardHighScores
      : normalHighScores;
}


int qualifyingHighScoreIndex(
  GameDifficulty difficulty,
  uint32_t candidateScore,
  uint32_t candidateTimeMs
) {

  HighScoreEntry* entries =
    scoresForDifficulty(difficulty);

  for (int i = 0; i < HIGH_SCORE_COUNT; i++) {
    if (
      highScoreComesBefore(
        candidateScore,
        candidateTimeMs,
        entries[i]
      )
    ) {
      return i;
    }
  }

  return -1;
}


void insertHighScore(
  GameDifficulty difficulty,
  const char* initials,
  uint32_t newScore,
  uint32_t newTimeMs
) {

  int insertAt =
    qualifyingHighScoreIndex(
      difficulty,
      newScore,
      newTimeMs
    );

  if (insertAt < 0) {
    return;
  }

  HighScoreEntry* entries =
    scoresForDifficulty(difficulty);

  for (int i = HIGH_SCORE_COUNT - 1; i > insertAt; i--) {
    entries[i] = entries[i - 1];
  }

  memset(&entries[insertAt], 0, sizeof(HighScoreEntry));
  strncpy(entries[insertAt].initials, initials, 3);
  entries[insertAt].initials[3] = '\0';
  entries[insertAt].score = newScore;
  entries[insertAt].timeMs = newTimeMs;

  if (highScoresReady) {
    const char* key =
      difficulty == DIFFICULTY_HARD
        ? HARD_SCORES_KEY
        : NORMAL_SCORES_KEY;

    highScorePreferences.putBytes(
      key,
      entries,
      sizeof(HighScoreEntry) * HIGH_SCORE_COUNT
    );
  }
}


void drawGreenReturnPrompt() {

  const int y = 226;
  const int x = 83;

  tft.setTextSize(1);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setCursor(x, y);
  tft.print("press");

  tft.fillCircle(
    x + 37,
    y + 3,
    3,
    ILI9341_GREEN
  );

  tft.setCursor(x + 46, y);
  tft.print("to return to menu");
}


void drawHighScoreScreen(GameDifficulty difficulty) {

  tft.fillScreen(ILI9341_BLACK);
  tft.drawRect(0, 0, tft.width(), tft.height(), ILI9341_RED);

  // Both headings are 21 characters at 12 pixels per character.
  tft.setTextWrap(false);
  tft.setTextSize(2);
  tft.setCursor((tft.width() - 21 * 12) / 2, 10);
  tft.setTextColor(ILI9341_WHITE);
  tft.print("HIGH SCORES: ");
  tft.setTextColor(difficulty == DIFFICULTY_HARD ? ILI9341_RED : ILI9341_GREEN);
  tft.print(difficulty == DIFFICULTY_HARD ? "HARD" : "EASY");
  tft.setTextColor(ILI9341_WHITE);
  tft.print(" MODE");

  HighScoreEntry* entries =
    scoresForDifficulty(difficulty);

  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);

  for (int i = 0; i < HIGH_SCORE_COUNT; i++) {
    char line[28];

    if (entries[i].initials[0] == '\0') {
      snprintf(line, sizeof(line), "%d. ---     --", i + 1);
    }
    else {
      snprintf(
        line,
        sizeof(line),
        "%d. %-3s %6lu",
        i + 1,
        entries[i].initials,
        (unsigned long)entries[i].score
      );
    }

    tft.setCursor(66, 48 + i * 31);
    tft.print(line);
  }

  tft.fillTriangle(12, 116, 20, 110, 20, 122, ILI9341_GREEN);
  tft.fillTriangle(307, 116, 299, 110, 299, 122, ILI9341_GREEN);
  drawGreenReturnPrompt();
}


void waitForAllControlsReleased() {
  while (
    digitalRead(PIN_UP) == LOW ||
    digitalRead(PIN_DOWN) == LOW ||
    digitalRead(PIN_LEFT) == LOW ||
    digitalRead(PIN_RIGHT) == LOW ||
    digitalRead(PIN_GREEN_BUTTON) == LOW ||
    digitalRead(PIN_RED_BUTTON) == LOW
  ) {
    delay(5);
  }
  delay(40);
}


void showHighScoreBrowser(GameDifficulty shownDifficulty) {

  waitForAllControlsReleased();

  drawHighScoreScreen(shownDifficulty);

  bool previousLeft = false;
  bool previousRight = false;
  bool previousGreen = false;

  while (true) {
    bool left = digitalRead(PIN_LEFT) == LOW;
    bool right = digitalRead(PIN_RIGHT) == LOW;
    bool green = digitalRead(PIN_GREEN_BUTTON) == LOW;

    if ((left && !previousLeft) || (right && !previousRight)) {
      shownDifficulty =
        shownDifficulty == DIFFICULTY_NORMAL
          ? DIFFICULTY_HARD
          : DIFFICULTY_NORMAL;

      drawHighScoreScreen(shownDifficulty);
    }

    if (green && !previousGreen) {
      waitForAllControlsReleased();
      return;
    }

    previousLeft = left;
    previousRight = right;
    previousGreen = green;
    delay(5);
  }
}


const char* INITIAL_KEYS[28] = {
  "A", "B", "C", "D", "E", "F", "G",
  "H", "I", "J", "K", "L", "M", "N",
  "O", "P", "Q", "R", "S", "T", "U",
  "V", "W", "X", "Y", "Z", "<", "OK"
};


void drawInitialsKeyboard(
  const char* initials,
  int selectedKey
) {

  tft.fillScreen(ILI9341_BLACK);
  drawCenteredMenuText("NEW HIGH SCORE", 5, 2, ILI9341_GREEN);

  char initialsLine[20];
  snprintf(
    initialsLine,
    sizeof(initialsLine),
    "INITIALS: %c%c%c",
    initials[0] ? initials[0] : '_',
    initials[1] ? initials[1] : '_',
    initials[2] ? initials[2] : '_'
  );
  drawCenteredMenuText(initialsLine, 29, 2, ILI9341_WHITE);

  const int startX = 6;
  const int startY = 60;
  const int cellW = 44;
  const int cellH = 34;

  for (int i = 0; i < 28; i++) {
    int col = i % 7;
    int row = i / 7;
    int x = startX + col * cellW;
    int y = startY + row * cellH;
    bool selected = (i == selectedKey);

    tft.fillRect(
      x,
      y,
      cellW - 3,
      cellH - 3,
      selected ? ILI9341_GREEN : ILI9341_BLACK
    );
    tft.drawRect(
      x,
      y,
      cellW - 3,
      cellH - 3,
      selected ? ILI9341_WHITE : 0x7BEF
    );

    tft.setTextSize(2);
    tft.setTextColor(
      selected ? ILI9341_BLACK : ILI9341_WHITE,
      selected ? ILI9341_GREEN : ILI9341_BLACK
    );

    int textOffset = strcmp(INITIAL_KEYS[i], "OK") == 0 ? 9 : 15;
    tft.setCursor(x + textOffset, y + 8);
    tft.print(INITIAL_KEYS[i]);
  }

  drawCenteredMenuText(
    "green = select   red = cancel",
    207,
    1,
    ILI9341_WHITE
  );
}


bool enterHighScoreInitials(char initials[4]) {

  memset(initials, 0, 4);
  int initialsLength = 0;
  int selectedKey = 0;

  waitForAllControlsReleased();
  drawInitialsKeyboard(initials, selectedKey);

  bool previousUp = false;
  bool previousDown = false;
  bool previousLeft = false;
  bool previousRight = false;
  bool previousGreen = false;
  bool previousRed = false;

  while (true) {
    bool up = digitalRead(PIN_UP) == LOW;
    bool down = digitalRead(PIN_DOWN) == LOW;
    bool left = digitalRead(PIN_LEFT) == LOW;
    bool right = digitalRead(PIN_RIGHT) == LOW;
    bool green = digitalRead(PIN_GREEN_BUTTON) == LOW;
    bool red = digitalRead(PIN_RED_BUTTON) == LOW;
    bool changed = false;

    if (red && !previousRed) {
      // Cancel without returning any initials to the score insertion path.
      waitForAllControlsReleased();
      return false;
    }

    if (up && !previousUp) {
      selectedKey = (selectedKey + 21) % 28;
      changed = true;
    }
    if (down && !previousDown) {
      selectedKey = (selectedKey + 7) % 28;
      changed = true;
    }
    if (left && !previousLeft) {
      selectedKey = (selectedKey + 27) % 28;
      changed = true;
    }
    if (right && !previousRight) {
      selectedKey = (selectedKey + 1) % 28;
      changed = true;
    }

    if (green && !previousGreen) {
      if (selectedKey < 26 && initialsLength < 3) {
        initials[initialsLength++] = 'A' + selectedKey;
        initials[initialsLength] = '\0';
        if (initialsLength == 3) {
          selectedKey = 27; // Highlight OK; the next green press confirms.
        }
        changed = true;
      }
      else if (selectedKey == 26 && initialsLength > 0) {
        initials[--initialsLength] = '\0';
        changed = true;
      }
      else if (selectedKey == 27 && initialsLength == 3) {
        waitForAllControlsReleased();
        return true;
      }
    }

    if (changed) {
      drawInitialsKeyboard(initials, selectedKey);
    }

    previousUp = up;
    previousDown = down;
    previousLeft = left;
    previousRight = right;
    previousGreen = green;
    previousRed = red;
    delay(5);
  }
}


void waitForGreenThenRestart() {
  waitForAllControlsReleased();

  while (digitalRead(PIN_GREEN_BUTTON) == HIGH) {
    delay(5);
  }

  delay(30);
  ESP.restart();

  while (true) {
    delay(1000);
  }
}


void finishGameOverFlow() {

  // Duration was frozen at entry to showGameOver().

  // Leave the completed GAME OVER screen visible briefly before changing
  // to initials entry or showing the return prompt.
  delay(1500);

  int qualifyingIndex =
    qualifyingHighScoreIndex(
      selectedDifficulty,
      (uint32_t)score,
      currentGameElapsedMs
    );

  if (qualifyingIndex >= 0) {
    char initials[4];
    bool saveScore = enterHighScoreInitials(initials);

    if (!saveScore) {
      // Restarting returns to the main menu and deliberately skips both
      // flash insertion and the post-game high-score browser.
      ESP.restart();
      return;
    }

    insertHighScore(
      selectedDifficulty,
      initials,
      (uint32_t)score,
      currentGameElapsedMs
    );

    showHighScoreBrowser(selectedDifficulty);
    ESP.restart();
    return;
  }

  drawGreenReturnPrompt();
  waitForGreenThenRestart();
}


// ============================================================
// PRE-GAME CENTER CALIBRATION
// ============================================================

void updateCalibrationServos() {
  const bool left = digitalRead(PIN_LEFT) == LOW;
  const bool right = digitalRead(PIN_RIGHT) == LOW;
  const bool up = digitalRead(PIN_UP) == LOW;
  const bool down = digitalRead(PIN_DOWN) == LOW;
  // Deliberately bypass time-based gameplay limits. Neutral = no pulses.
  setLRServoTicks(left != right ? (left ? LR_LEFT_TICKS : LR_RIGHT_TICKS) : 4096);
  setUDServoTicks(up != down ? (up ? UD_UP_TICKS : UD_DOWN_TICKS) : 4096);
}

void drawCalibrationFrame(uint8_t frame) {
  const uint16_t* data = CALIBRATION_FRAMES[frame];
  uint16_t row[CALIBRATION_WIDTH];
  uint32_t offset = 0;
  uint16_t remaining = 0;
  uint16_t color = 0;
  for (int y = 0; y < CALIBRATION_HEIGHT; ++y) {
    for (int x = 0; x < CALIBRATION_WIDTH; ++x) {
      if (remaining == 0) {
        remaining = pgm_read_word(data + offset++);
        color = pgm_read_word(data + offset++);
      }
      row[x] = color;
      --remaining;
    }
    tft.drawRGBBitmap(CALIBRATION_X, CALIBRATION_Y + y, row, CALIBRATION_WIDTH, 1);
    // Keep joystick responsive even during an SPI image transfer.
    if (digitalRead(PIN_GREEN_BUTTON) == LOW) {
      setLRServoTicks(4096);
      setUDServoTicks(4096);
      return;
    }
    updateCalibrationServos();
  }
}

void calibrateBeforeGame() {
  disableAllServoSignals();
  waitForAllControlsReleased();
  tft.fillScreen(ILI9341_BLACK);
  drawCenteredMenuText("Please center the turbolaser", 7, 1, ILI9341_WHITE);
  drawCenteredMenuText("on both x and y axis.", 20, 1, ILI9341_WHITE);
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(101, 220);
  tft.print("Press");
  tft.fillCircle(139, 223, 3, ILI9341_GREEN);
  tft.setCursor(149, 220);
  tft.print("when done.");
  const unsigned long animationStart = millis();
  int lastFrame = -1;
  unsigned long confirmSince = 0;
  bool confirming = false;
  while (true) {
    unsigned long now = millis();
    if (digitalRead(PIN_GREEN_BUTTON) == LOW) {
      setLRServoTicks(4096);
      setUDServoTicks(4096);
      if (!confirming) { confirming = true; confirmSince = now; }
      if (now - confirmSince >= 30) {
        gameStartTime = confirmSince;
        disableAllServoSignals();
        lrTravelPositionMs = 0;
        udTravelPositionMs = 0;
        // Do not let the confirmation press become a gameplay shot.
        waitForAllControlsReleased();
        return;
      }
    } else {
      confirming = false;
      updateCalibrationServos();
      int frame = ((now - animationStart) / 250UL) % 8;
      if (frame != lastFrame) {
        drawCalibrationFrame(frame);
        lastFrame = frame;
      }
    }
    delay(1);
  }
}


// ============================================================
// START SCREEN / MAIN MENU
// ============================================================
//
// Before the game starts:
//   - TFT stays black
//   - title says "TURBOLASER" inside a thicker red box
//   - a red border surrounds the entire TFT
//   - NORMAL is listed first and selected by default
//   - HARD, HIGH SCORES follow underneath
//   - joystick UP/DOWN changes the selected option
//   - either red or green button activates the selected option
//   - joystick does NOT move either continuous-rotation servo
//   - SG90 animation does not play
//
// The continuous-rotation servos are commanded to their calibrated
// FULL OFF state while waiting.
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


void drawMainMenuOptions() {

  restoreStartMenuBackgroundRect(
    0,
    124,
    tft.width() - 1,
    220
  );

  uint16_t normalColor =
    (selectedMenuOption == MENU_NORMAL)
      ? ILI9341_GREEN
      : START_MENU_DIM_COLOR;

  uint16_t hardColor =
    (selectedMenuOption == MENU_HARD)
      ? ILI9341_GREEN
      : START_MENU_DIM_COLOR;

  uint16_t highScoresColor =
    (selectedMenuOption == MENU_HIGH_SCORES)
      ? ILI9341_GREEN
      : START_MENU_DIM_COLOR;


  drawCenteredMenuText(
    "Easy",
    130,
    1,
    normalColor
  );

  drawCenteredMenuText(
    "Hard",
    151,
    1,
    hardColor
  );

  drawCenteredMenuText(
    "High Scores",
    172,
    1,
    highScoresColor
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
    "select mode",
    105,
    2,
    ILI9341_WHITE
  );

  drawMainMenuOptions();
}


void applySelectedDifficulty() {

  if (selectedDifficulty == DIFFICULTY_HARD) {
    hitXThresholdPx = HARD_HIT_THRESHOLD_PX;
    hitYThresholdPx = HARD_HIT_THRESHOLD_PX;
  }
  else {
    hitXThresholdPx = NORMAL_HIT_THRESHOLD_PX;
    hitYThresholdPx = NORMAL_HIT_THRESHOLD_PX;
  }
}


void waitForStartButton() {

  // Ignore the joystick for servo movement while waiting.
  // Keep all servo outputs fully off while waiting.
  disableAllServoSignals();

  selectedMenuOption = MENU_NORMAL;
  selectedDifficulty = DIFFICULTY_NORMAL;
  applySelectedDifficulty();
  drawMainMenuOptions();

  Serial.println(
    "Main menu ready. Use joystick UP/DOWN and either button to select."
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
      selectedMenuOption =
        (MainMenuOption)(
          (selectedMenuOption + MENU_OPTION_COUNT - 1) %
          MENU_OPTION_COUNT
        );
      drawMainMenuOptions();
    }

    if (downPressed && !previousDown) {
      selectedMenuOption =
        (MainMenuOption)(
          (selectedMenuOption + 1) %
          MENU_OPTION_COUNT
        );
      drawMainMenuOptions();
    }

    previousUp = upPressed;
    previousDown = downPressed;

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

      if (selectedMenuOption == MENU_HIGH_SCORES) {
        showHighScoreBrowser(DIFFICULTY_NORMAL);
        drawStartScreen();
        previousUp = false;
        previousDown = false;
        continue;
      }

      selectedDifficulty =
        selectedMenuOption == MENU_HARD
          ? DIFFICULTY_HARD
          : DIFFICULTY_NORMAL;

      applySelectedDifficulty();

      Serial.println(
        selectedDifficulty == DIFFICULTY_HARD
          ? "Game starting: HARD, 7 px hit threshold"
          : "Game starting: EASY, 10 px hit threshold"
      );

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

  loadHighScores();


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
  // FULL OFF state (no control pulses).
  disableAllServoSignals();


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

  // Nothing in the game runs until the player chooses NORMAL or HARD.
  // HIGH SCORES returns to the menu; both game modes require centering.
  waitForStartButton();
  calibrateBeforeGame();


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
  drawGameplayBackground();


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

  // gameStartTime was captured by the green calibration confirmation.


  previousEnemyUpdate =
    millis();


  // The first ship always starts on route 0: top middle -> bottom middle.
  enemyPathIndex = 0;
  enemyShipNumber = 1;
  setEnemyColorForShipNumber();
  primaryEnemyActive = true;
  enemy2Active = false;
  enemy2HitAnimating = false;
  speedBoostTargetActive = false;
  turbolaserSpeedBoostActive = false;
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
  enemyHitDestroys = false;
  enemyHitRemovesShield = false;

  // Reset all one-game Easy-mode tutorial state.
  activeTutorialMessage = TUTORIAL_NONE;
  tutorialLastPage = -1;
  tutorialArrowVisible = false;
  lastElevationHintShipNumber = 0;
  shieldHintShown = false;
  redCooldownHintShown = false;
  buttonLightsHintShown = false;
  firstRedCooldownHintShipNumber = 0;
  lastAcceptedRedOnShieldedShip = 0;

  drawHealthHearts();
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
  // GAME OVER CONTROL MODE
  // ==========================================================
  // Buttons, LEDs, and audio may continue on Core 0, but return before
  // any SG90 animation or joystick-servo command can be generated.
  // All PCA9685 servo channels were placed in FULL OFF by showGameOver().

  if (gameOver) {

    delay(
      10
    );

    return;
  }


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
  //   -1000 ms = full up   = TICK_Y_MIN
//       0 ms = center    = middle of the elevation bar
  //   +1000 ms = full down = TICK_Y_MAX
  //
  // Both the physical servo limit and the TFT elevation marker use the
  // SAME udTravelPositionMs value, so a full top-to-bottom sweep takes
  // exactly 2000 ms and the display stays synchronized with the
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
  // Green-button shots deal 1 damage. Red-button shots deal 3 damage.
  // Shielded X-wings have 3 HP and switch to the normal sprite after
  // two green hits leave them at 1 HP. Normal X-wings have 1 HP.
  // Every hit still requires BOTH horizontal aim and elevation alignment.

  int shotDamage = 0;

  if (redButtonPressedThisFrame) {
    shotDamage = 3;
  }
  else if (greenButtonPressedThisFrame) {
    shotDamage = 1;
  }

  bool shieldedTargetPresent =
    (primaryEnemyActive && enemyIsRed) ||
    (enemy2Active && enemy2IsRed);

  // Remember which shielded round started the current red cooldown.
  if (redButtonPressedThisFrame && shieldedTargetPresent) {
    lastAcceptedRedOnShieldedShip = enemyShipNumber;
  }

  if (
    selectedDifficulty == DIFFICULTY_NORMAL &&
    redButtonCooldownPressedThisFrame &&
    shieldedTargetPresent &&
    lastAcceptedRedOnShieldedShip == enemyShipNumber
  ) {
    if (!redCooldownHintShown) {
      redCooldownHintShown = true;
      firstRedCooldownHintShipNumber = enemyShipNumber;
      showTutorialMessage(TUTORIAL_RED_COOLDOWN);
    }
    else if (
      !buttonLightsHintShown &&
      enemyShipNumber != firstRedCooldownHintShipNumber
    ) {
      buttonLightsHintShown = true;
      showTutorialMessage(TUTORIAL_BUTTON_LIGHTS);
    }
  }

  if (
    shotDamage > 0
  ) {
    // The S+ target requires horizontal/side-to-side aim only and consumes
    // this shot before enemy hit-testing.
    if (shotHitsSpeedBoost()) {
      eraseSpeedBoostTarget();
      speedBoostTargetActive = false;
      turbolaserSpeedBoostActive = true;

      // Apply the boost to the activating shot's cooldown as well as every
      // later shot, so the benefit is immediately visible on the LEDs.
      unsigned long boostNow = millis();
      if ((long)(greenLedOffUntil - boostNow) > 0) {
        greenLedOffUntil = boostNow + (unsigned long)(
          (greenLedOffUntil - boostNow) * SPEED_BOOST_COOLDOWN_MULTIPLIER
        );
      }
      if ((long)(redLedOffUntil - boostNow) > 0) {
        redLedOffUntil = boostNow + (unsigned long)(
          (redLedOffUntil - boostNow) * SPEED_BOOST_COOLDOWN_MULTIPLIER
        );
      }
      greenButtonDisabledUntil = greenLedOffUntil + BUTTON_REARM_DELAY_MS;
      redButtonDisabledUntil = redLedOffUntil + BUTTON_REARM_DELAY_MS;

      showTutorialMessage(TUTORIAL_SPEED_BOOST);
    }
    else {
      bool primaryHorizontal = false;
      bool primaryVertical = false;
      bool secondaryHorizontal = false;
      bool secondaryVertical = false;

      if (primaryEnemyActive && !enemyHitAnimating) {
        getShotAlignment(primaryHorizontal, primaryVertical);
      }
      if (enemy2Active && !enemy2HitAnimating) {
        getEnemy2ShotAlignment(secondaryHorizontal, secondaryVertical);
      }

      bool primaryHit = primaryHorizontal && primaryVertical;
      bool secondaryHit = secondaryHorizontal && secondaryVertical;

      // Easy-mode hints explain the relevant mistake without pausing play.
      if (selectedDifficulty == DIFFICULTY_NORMAL) {
        if (
          enemyShipNumber <= 2 &&
          ((primaryHorizontal && !primaryVertical) ||
           (secondaryHorizontal && !secondaryVertical)) &&
          lastElevationHintShipNumber != enemyShipNumber
        ) {
          lastElevationHintShipNumber = enemyShipNumber;
          showTutorialMessage(TUTORIAL_ELEVATION);
        }

        if (
          enemyShipNumber == 3 &&
          greenButtonPressedThisFrame &&
          !shieldHintShown
        ) {
          shieldHintShown = true;
          showTutorialMessage(TUTORIAL_SHIELD);
        }
      }

      if (primaryHit) {
        beginEnemyHit(shotDamage);
      }
      else if (secondaryHit) {
        beginEnemy2Hit(shotDamage);
      }
    }
  }


  // ==========================================================
  // ERASE OLD DYNAMIC GRAPHICS
  // ==========================================================

  eraseOldEnemy();
  eraseOldEnemy2();


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
  bool enemy2Escaped = false;

  if (primaryEnemyActive && enemyHitAnimating) {

    // Freeze only the enemy while it flickers. Player joystick, buttons,
    // audio, LEDs, and recoil animation continue operating above.
    updateEnemyHitAnimation();
  }
  else if (primaryEnemyActive) {

    enemyEscaped =
      updateEnemyPosition();
  }

  if (enemy2Active && enemy2HitAnimating) {
    updateEnemy2HitAnimation();
  }
  else if (enemy2Active) {
    enemy2Escaped = updateEnemy2Position();
  }


  // ==========================================================
  // ENEMY REACHED BOTTOM
  // ==========================================================

  if (enemyEscaped) {

    loseHealth();


    if (gameOver) {

      return;
    }


    primaryEnemyActive = false;
    finishRoundIfReady();
  }

  if (enemy2Escaped) {
    loseHealth();

    if (gameOver) {
      return;
    }

    enemy2Active = false;
    finishRoundIfReady();
  }


  // ==========================================================
  // DRAW ENEMY
  // ==========================================================

  if (
    primaryEnemyActive &&
    (!enemyHitAnimating || enemyVisibleDuringHit)
  ) {

    drawEnemyShip(
      enemyX,
      enemyY
    );


    drawEnemyElevation(
      enemyElevationY
    );
  }
  else if (primaryEnemyActive && enemyHitAnimating) {
    // Alternate the ship with an explosion instead of a blank phase.
    drawEnemyExplosion(enemyX, enemyY);
    drawEnemyElevation(enemyElevationY);
  }

  if (
    enemy2Active &&
    (!enemy2HitAnimating || enemy2VisibleDuringHit)
  ) {
    drawEnemyShipAt(enemy2X, enemy2Y, enemy2IsRed);
    drawEnemyElevationAt(enemy2ElevationY, enemy2IsRed);
  }
  else if (enemy2Active && enemy2HitAnimating) {
    drawEnemyExplosionAt(enemy2X, enemy2Y, enemy2HitWasShielded);
    drawEnemyElevationAt(enemy2ElevationY, enemy2IsRed);
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

  // The stationary pickup is drawn after moving graphics so it remains
  // visible if an X-wing or aim-line restoration crosses its area.
  drawSpeedBoostTarget();


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

  if (enemy2Active || enemy2HitAnimating) {
    prevEnemy2X = enemy2X;
    prevEnemy2Y = enemy2Y;
    prevEnemy2ElevationY = enemy2ElevationY;
  }


  // Tutorial graphics are updated last so they remain legible while all
  // normal gameplay drawing and controls continue underneath.
  updateTutorialMessage();


  // ==========================================================
  // FRAME DELAY
  // ==========================================================

  delay(
    FRAME_DELAY_MS
  );
}
