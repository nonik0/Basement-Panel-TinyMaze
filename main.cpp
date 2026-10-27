//#include <tinyNeoPixel_static.h>
#include <Wire.h>

#include "maze_runner_lib.h"

#define I2C_ADDRESS 0x88
#define STATUS_LED_PIN 5
#define STATUS_UPDATE_INTERVAL 500

static const int TASK_PRIORITY = 5;
const int MAZE_DELAY_MS = 60;
const uint32_t BLACK = Adafruit_NeoPixel::Color(0x00, 0x00, 0x00);
const uint32_t RED = Adafruit_NeoPixel::Color(0xFF, 0x00, 0x00);
const uint32_t ORANGE = Adafruit_NeoPixel::Color(0xCC, 0x44, 0x00);
const uint32_t YELLOW = Adafruit_NeoPixel::Color(0xFF, 0xFF, 0x00);
const uint32_t YELLOWGREEN = Adafruit_NeoPixel::Color(0xCC, 0xFF, 0x00);
const uint32_t GREEN = Adafruit_NeoPixel::Color(0x00, 0xFF, 0x00);
const uint32_t BLUE = Adafruit_NeoPixel::Color(0x00, 0x00, 0x77);
const uint32_t PURPLE = Adafruit_NeoPixel::Color(0x77, 0x00, 0x77);
static const uint8_t LED_IN_PIN = 6;
static const int WIDTH = 16;
static const int HEIGHT = 16;

Adafruit_NeoPixel matrix = Adafruit_NeoPixel(WIDTH * HEIGHT, LED_IN_PIN);
MazeRunner *mazeRunner;
volatile bool display = true;

void handleOnReceive(int bytesReceived)
{
  if (bytesReceived < 2)
  {
    return;
  }

  uint8_t command = Wire.read();

  // setDisplay
  if (command == 0x00)
  {
    display = Wire.read();
  }
}

void setup()
{
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(receiveEvent);

  matrix.begin();
  matrix.setBrightness(5);
  matrix.show();

  mazeRunner = new MazeRunner(
    WIDTH,
    HEIGHT,
    BLACK,  // off
    ORANGE, // wall
    YELLOW, // runner
    RED,    // sentry
    PURPLE, // exit
    [this](int x, int y, uint32_t c)
    { matrix.setPixelColor(y * WIDTH + x, c); },
    [this](uint32_t c) {});

  mazeRunner->init(); 
}

void loop()
{
  if (millis() - lastUpdate < updateInterval)
  {
    return;
  }

  if (!display)
  {
    matrix.clear();
    matrix.show();
    delay(MAZE_DELAY_MS);
    return;
  }

  if (mazeRunner->update())
  {
    matrix.show();
  }
  delay(MAZE_DELAY_MS);
}