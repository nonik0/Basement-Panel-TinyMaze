
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
static const uint8_t BRIGHTNESS = 5;
#if defined(USE_TINYNEOPIXEL)
static const uint8_t LED_IN_PIN = 16;
#else
static const uint8_t LED_IN_PIN = 6;
#endif
static const int WIDTH = MAZE_RUNNER_WIDTH;
static const int HEIGHT = MAZE_RUNNER_HEIGHT;

// Maze options.
constexpr MazeOptions mazeOptions = {
  true,  // outerWalls
  true,  // preventDiagonalTouching
  false, // perfectVision
  rgb(0x04, 0x04, 0x04), // visitedColor
  true   // sentryPatrol
};

#if defined(USE_TINYNEOPIXEL)
static uint8_t pixelBuffer[WIDTH * HEIGHT * 3];
NeoPixel matrix(WIDTH * HEIGHT, LED_IN_PIN, NEO_GRB, pixelBuffer);
#else
NeoPixel matrix(WIDTH * HEIGHT, LED_IN_PIN, NEO_GRB + NEO_KHZ800);
#endif
volatile bool display = true;

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
  if (bytesReceived < 2) return;

  uint8_t command = Wire.read();
  if (command == 0x00 && Wire.available())
    display = Wire.read() != 0;

  while (Wire.available()) Wire.read();
}

void setup()
{
  randomSeed(analogRead(A0) ^ analogRead(A1) ^ analogRead(A2));

  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(handleOnReceive);

#if defined(USE_TINYNEOPIXEL)
  pinMode(LED_IN_PIN, OUTPUT); // tinyNeoPixel_static has no begin()
#endif
  matrix.begin();
  matrix.setBrightness(BRIGHTNESS);
  matrix.clear();
  matrix.show();

  mazeRunner.init();
  matrix.show(); // init() now draws the initial maze.
}

void loop()
{
  if (!display)
  {
    matrix.clear();
    matrix.show();
    delay(MAZE_DELAY_MS);
    return;
  }

  if (mazeRunner.update())
    matrix.show();

  delay(MAZE_DELAY_MS);
}
