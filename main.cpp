
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

const int MAZE_DELAY_MS = 30;

static const uint8_t BRIGHTNESS = 4;
#if defined(USE_TINYNEOPIXEL)
static const uint8_t LED_IN_PIN = 16;
#else
static const uint8_t LED_IN_PIN = 6;
#endif
static const uint8_t STATUS_LED_PIN = 10;
static const uint16_t STATUS_UPDATE_INTERVAL = 500; // milliseconds for status LED update interval
static const int WIDTH = 16;
static const int HEIGHT = 16;

// Maze options: anything not set here keeps its default (see MazeOptions in maze_runner_lib.h),
// including the runner/sentry sense and speed, extra loops, and delays.
constexpr MazeOptions makeMazeOptions()
{
  MazeOptions o;
  o.width = WIDTH;
  o.height = HEIGHT;
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

MazeRunner mazeRunner(
  [](int x, int y, uint32_t color) { matrix.setPixelColor(y * WIDTH + x, color); },
  [](uint32_t color) { },
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

  // log update at start time
  lastMazeUpdate = millis();

  if (!display)
  {
    matrix.clear();
    matrix.show();
    lastMazeUpdate = millis();
    return;
  }

  if (mazeRunner.update())
    matrix.show();
}
