
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include "maze_runner_lib.h"

#define I2C_ADDRESS 0x88

const int MAZE_DELAY_MS = 50;
const uint32_t BLACK = Adafruit_NeoPixel::Color(0x00, 0x00, 0x00);
const uint32_t RED = Adafruit_NeoPixel::Color(0xFF, 0x00, 0x00);
const uint32_t ORANGE = Adafruit_NeoPixel::Color(0xCC, 0x44, 0x00);
const uint32_t YELLOW = Adafruit_NeoPixel::Color(0xFF, 0xFF, 0x00);
const uint32_t YELLOWGREEN = Adafruit_NeoPixel::Color(0xCC, 0xFF, 0x00);
const uint32_t GREEN = Adafruit_NeoPixel::Color(0x00, 0xFF, 0x00);
const uint32_t BLUE = Adafruit_NeoPixel::Color(0x00, 0x00, 0x77);
const uint32_t PURPLE = Adafruit_NeoPixel::Color(0x77, 0x00, 0x77);
static const uint8_t LED_IN_PIN = 6;
static const int WIDTH = MAZE_RUNNER_WIDTH;
static const int HEIGHT = MAZE_RUNNER_HEIGHT;

// Maze generation options.
const MazeGenerationOptions mazeOptions = {
  true,  // outerWalls
  true,  // preventDiagonalTouching
  false, // perfectVision
  Adafruit_NeoPixel::Color(0x04, 0x04, 0x04) // visitedColor
};

Adafruit_NeoPixel matrix(WIDTH * HEIGHT, LED_IN_PIN, NEO_GRB + NEO_KHZ800);
volatile bool display = true;

void drawPixel(int x, int y, uint32_t color)
{
  matrix.setPixelColor(y * WIDTH + x, color);
}

void setStatus(uint32_t color)
{
}

void handleOnReceive(int bytesReceived)
{
  if (bytesReceived < 2) return;

  uint8_t command = Wire.read();
  if (command == 0x00 && Wire.available())
    display = Wire.read() != 0;

  while (Wire.available()) Wire.read();
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

void setup()
{
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(handleOnReceive);

  matrix.begin();
  matrix.setBrightness(7);
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
