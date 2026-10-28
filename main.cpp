
#include <Wire.h>
#include "maze_runner_lib.h"

// ATtiny (megaTinyCore) uses its own bundled NeoPixel driver, tinyNeoPixel_static, which
// differs from Adafruit_NeoPixel in three ways: the class name, the caller supplies the
// pixel buffer (3 bytes per LED), and there is no begin() (set pinMode yourself).
// [env:tiny] sets -D USE_TINYNEOPIXEL in platformio.ini.
#if defined(USE_TINYNEOPIXEL)
  #include <tinyNeoPixel_Static.h>
  using NeoPixel = tinyNeoPixel;
#else
  #include <Adafruit_NeoPixel.h>
  using NeoPixel = Adafruit_NeoPixel;
#endif

#define I2C_ADDRESS 0x14

const int MAZE_DELAY_MS = 50;

// Same packing as NeoPixel::Color(), which isn't constexpr: calling it here would copy every
// color into SRAM at startup instead of leaving it in flash.
constexpr uint32_t rgb(uint8_t r, uint8_t g, uint8_t b)
{
  return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}
constexpr uint32_t BLACK = rgb(0x00, 0x00, 0x00);
constexpr uint32_t RED = rgb(0xFF, 0x00, 0x00);
constexpr uint32_t ORANGE = rgb(0xCC, 0x44, 0x00);
constexpr uint32_t YELLOW = rgb(0xFF, 0xFF, 0x00);
constexpr uint32_t YELLOWGREEN = rgb(0xCC, 0xFF, 0x00);
constexpr uint32_t GREEN = rgb(0x00, 0xFF, 0x00);
constexpr uint32_t BLUE = rgb(0x00, 0x00, 0x77);
constexpr uint32_t PURPLE = rgb(0x77, 0x00, 0x77);
static const uint8_t BRIGHTNESS = 4;
#if defined(USE_TINYNEOPIXEL)
static const uint8_t LED_IN_PIN = 16;
#else
static const uint8_t LED_IN_PIN = 6;
#endif
static const uint8_t STATUS_LED_PIN = 10;
static const uint16_t STATUS_UPDATE_INTERVAL = 500; // milliseconds for status LED update interval
static const int WIDTH = MAZE_RUNNER_WIDTH;
static const int HEIGHT = MAZE_RUNNER_HEIGHT;

// Maze options: anything not set here keeps its default (see MazeOptions in maze_runner_lib.h),
// including the runner/sentry sense and speed, extra loops, and delays.
constexpr MazeOptions makeMazeOptions()
{
  MazeOptions o;
  o.outerWalls = true;
  o.preventDiagonalTouching = true;
  o.perfectVision = false;
  o.visitedColor = rgb(0x04, 0x04, 0x04);
  o.sentryPatrol = true;
  return o;
}
constexpr MazeOptions mazeOptions = makeMazeOptions();

// i2c
bool statusLedState = false;
bool statusLedFirstBlink = false;
uint8_t statusLedBlinks = 0; // number of extra short blinks after long "ACK" blink
unsigned long lastStatusLedUpdate = 0;
unsigned long statusLedUpdateInterval = STATUS_UPDATE_INTERVAL;

#if defined(USE_TINYNEOPIXEL)
static uint8_t pixelBuffer[WIDTH * HEIGHT * 3];
NeoPixel matrix(WIDTH * HEIGHT, LED_IN_PIN, NEO_GRB, pixelBuffer);
#else
NeoPixel matrix(WIDTH * HEIGHT, LED_IN_PIN, NEO_GRB + NEO_KHZ800);
#endif
volatile bool display = true;
unsigned long lastMazeUpdate = 0;

void drawPixel(int x, int y, uint32_t color)
{
  matrix.setPixelColor(y * WIDTH + x, color);
}

void setStatus(uint32_t color)
{
}

MazeRunner mazeRunner(
  WIDTH,
  HEIGHT,
  BLACK,  // off
  ORANGE, // wall
  YELLOW, // runner
  RED,    // sentry
  PURPLE, // exit
  drawPixel,
  setStatus,
  mazeOptions);

void handleOnReceive(int bytesReceived)
{
  statusLedState = true;
  digitalWrite(STATUS_LED_PIN, !statusLedState);
  lastStatusLedUpdate = millis();
  statusLedBlinks = 0;
  statusLedFirstBlink = true;

  if (bytesReceived < 2) return;

  uint8_t command = Wire.read();
  statusLedBlinks = command + 1; // use value to blink status LED
  if (command == 0x00 && Wire.available())
    display = Wire.read() != 0;
  else {
    statusLedBlinks = 10; // error
  }

  while (Wire.available()) Wire.read();
}

void setup()
{
  randomSeed(analogRead(A0) ^ analogRead(A1) ^ analogRead(A2));

  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, false);
  #if defined(USE_TINYNEOPIXEL)
  pinMode(LED_IN_PIN, OUTPUT);
  #endif
  
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(handleOnReceive);

  matrix.begin();
  matrix.setBrightness(BRIGHTNESS);
  matrix.clear();
  matrix.show();

  mazeRunner.init();
  matrix.show(); // init() now draws the initial maze.

  delay(500);
  digitalWrite(STATUS_LED_PIN, true);
}

void loop()
{
  // status LED indicates when I2C message is received, long blink first, then short blink count indicates status
  if ((statusLedBlinks > 0 || statusLedState) && millis() - lastStatusLedUpdate > statusLedUpdateInterval)
  {
    statusLedState = !statusLedState;
    digitalWrite(STATUS_LED_PIN, !statusLedState);
    lastStatusLedUpdate = millis();

    if (statusLedBlinks > 0)
    {
      statusLedBlinks -= statusLedState ? 1 : 0;
      statusLedUpdateInterval = statusLedState ? (STATUS_UPDATE_INTERVAL >> 2) : (STATUS_UPDATE_INTERVAL >> 3);

      // longer gap between first long blink and later short blinks
      if (statusLedFirstBlink)
      {
        statusLedFirstBlink = false;
        statusLedUpdateInterval = STATUS_UPDATE_INTERVAL >> 1;
      }
    }
    else
    {
      statusLedUpdateInterval = STATUS_UPDATE_INTERVAL;
    }
  }

  if (millis() - lastMazeUpdate < MAZE_DELAY_MS)
  {
    yield();
    return;
  }

  if (!display)
  {
    matrix.clear();
    matrix.show();
    lastMazeUpdate = millis();
    return;
  }

  if (mazeRunner.update())
    matrix.show();

  lastMazeUpdate = millis();
}
