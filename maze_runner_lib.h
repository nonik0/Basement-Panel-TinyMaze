#pragma once

#include <Arduino.h>
#include <stdint.h>

// ESP logging macros are not available in the AVR Arduino core.
#ifndef log_v
#define log_v(...) do {} while (0)
#endif
#ifndef log_d
#define log_d(...) do {} while (0)
#endif
#ifndef log_w
#define log_w(...) do {} while (0)
#endif
#ifndef log_e
#define log_e(...) do {} while (0)
#endif

// Configure these before compiling (e.g. with PlatformIO build_flags).
#ifndef MAZE_RUNNER_WIDTH
#define MAZE_RUNNER_WIDTH 16
#endif
#ifndef MAZE_RUNNER_HEIGHT
#define MAZE_RUNNER_HEIGHT 16
#endif

#if MAZE_RUNNER_WIDTH < 1 || MAZE_RUNNER_HEIGHT < 1 || MAZE_RUNNER_WIDTH * MAZE_RUNNER_HEIGHT > 256
#error "MazeRunner dimensions must be positive and contain at most 256 cells"
#endif

// Fixed-size implementation for small AVR targets. Cell indexes are one byte.
static const uint16_t MAZE_MAX_CELLS = MAZE_RUNNER_WIDTH * MAZE_RUNNER_HEIGHT;
static const uint16_t MAZE_BITSET_BYTES = (MAZE_MAX_CELLS + 7) / 8;

struct MazeGenerationOptions
{
  // Keep the display perimeter as walls, with an entrance and exit opening.
  bool outerWalls;
  // Avoid diagonal-only contact between otherwise separate corridors.
  bool preventDiagonalTouching;
  // If false, the runner explores and marks cells instead of navigating directly to the exit.
  bool perfectVision;
  // Color used to mark explored cells when perfectVision is false.
  uint32_t visitedColor;

  MazeGenerationOptions(bool outer = false, bool diagonal = false,
                        bool vision = true, uint32_t visited = 0x000020UL)
    : outerWalls(outer), preventDiagonalTouching(diagonal),
      perfectVision(vision), visitedColor(visited) {}
};

struct Coordinate
{
  int x;
  int y;
};

Coordinate operator-(const Coordinate &coord) { return {-coord.x, -coord.y}; }
Coordinate operator+(const Coordinate &a, const Coordinate &b) { return {a.x + b.x, a.y + b.y}; }
Coordinate operator-(const Coordinate &a, const Coordinate &b) { return {a.x - b.x, a.y - b.y}; }
Coordinate operator*(const Coordinate &a, const Coordinate &b) { return {a.x * b.x, a.y * b.y}; }
Coordinate operator*(const int &a, const Coordinate &b) { return {b.x * a, b.y * a}; }
bool operator==(const Coordinate &a, const Coordinate &b) { return a.x == b.x && a.y == b.y; }
bool operator!=(const Coordinate &a, const Coordinate &b) { return !(a == b); }
Coordinate operator+=(Coordinate &a, const Coordinate &b) { a.x += b.x; a.y += b.y; return a; }
Coordinate operator-=(Coordinate &a, const Coordinate &b) { a.x -= b.x; a.y -= b.y; return a; }
Coordinate operator*=(Coordinate &a, const Coordinate &b) { a.x *= b.x; a.y *= b.y; return a; }

using Location = Coordinate;
using Direction = Coordinate;

const Location NullLocation = {-1, -1};
const Direction Left = {-1, 0};
const Direction Right = {1, 0};
const Direction Up = {0, -1};
const Direction Down = {0, 1};
const Direction Directions[] = {Left, Right, Up, Down};

inline void shuffle(Direction *list, int size)
{
  for (int i = 0; i < size; ++i)
  {
    int index = random(size);
    Direction temp = list[i];
    list[i] = list[index];
    list[index] = temp;
  }
}

inline bool upOrDown(Direction dir) { return dir == Up || dir == Down; }
inline bool leftOrRight(Direction dir) { return dir == Left || dir == Right; }

class MazeRunner
{
private:
  static const uint8_t RunnerFear = 10;
  static const uint8_t RunnerSense = 2;
  static const uint8_t RunnerSpeed = 3;
  static const uint8_t SentrySense = 2;
  static const uint8_t SentrySpeed = 5;
  static const int GoalDelay = 10;
  static const int CatchDelay = 30;
  static const int ErrorDelay = 30000;

  int _width;
  int _height;
  // One bit per cell instead of one byte per cell (32 bytes for 16x16).
  uint8_t _mazeWalls[MAZE_BITSET_BYTES];
  int _resetDelay = -1;
  Location _exitLoc = NullLocation;
  int _mazeExtraWallsToRemove = 1;

  uint32_t _pathColor, _wallColor, _runnerColor, _sentryColor, _exitColor;
  MazeGenerationOptions _generationOptions;
  bool _startAtPreviousGoal = false;
  uint8_t _runnerVisited[MAZE_BITSET_BYTES] = {};
  Location _runnerLoc = NullLocation;
  Location _runnerSentryKnownLoc = NullLocation;
  uint8_t _runnerCooldown = 0;
  Location _sentryLoc = NullLocation;
  // Keep pursuing the last location where the runner was seen.
  Location _sentryKnownRunnerLoc = NullLocation;
  bool _sentryActive = false;
  uint8_t _sentryCooldown = 0;

  // Paths are stored as one-byte cell indexes, keeping SRAM usage down.
  uint8_t _runnerPath[MAZE_MAX_CELLS];
  uint16_t _runnerPathSize = 0;

  typedef void (*DrawPixelCallback)(int, int, uint32_t);
  typedef void (*StatusCallback)(uint32_t);
  DrawPixelCallback _drawPixel;
  StatusCallback _setStatus;

  uint8_t encode(Location loc) const { return (uint8_t)(loc.y * _width + loc.x); }
  Location decode(uint8_t cell) const { return {(int)(cell % _width), (int)(cell / _width)}; }
  bool wallAt(int x, int y) const
  {
    uint16_t cell = (uint16_t)y * _width + x;
    return (_mazeWalls[cell >> 3] & (uint8_t)(1U << (cell & 7))) != 0;
  }
  void setWall(int x, int y, bool wall)
  {
    uint16_t cell = (uint16_t)y * _width + x;
    uint8_t mask = (uint8_t)(1U << (cell & 7));
    if (wall) _mazeWalls[cell >> 3] |= mask;
    else _mazeWalls[cell >> 3] &= (uint8_t)~mask;
  }

  bool pathEmpty(const uint8_t *path, uint16_t size) const { (void)path; return size == 0; }
  void clearRunnerPath() { _runnerPathSize = 0; }

  bool appendPath(uint8_t *path, uint16_t &size, uint8_t cell)
  {
    if (size >= MAZE_MAX_CELLS) return false;
    path[size++] = cell;
    return true;
  }

  bool popFrontPath(uint8_t *path, uint16_t &size, Location &loc)
  {
    if (!size) return false;
    loc = decode(path[0]);
    for (uint16_t i = 1; i < size; ++i) path[i - 1] = path[i];
    --size;
    return true;
  }

public:
  MazeRunner(int width, int height,
             uint32_t pathColor, uint32_t wallColor,
             uint32_t runnerColor, uint32_t sentryColor,
             uint32_t exitColor,
             DrawPixelCallback drawPixel,
             StatusCallback setStatus = nullptr,
             MazeGenerationOptions generationOptions = {})
    : _width(width), _height(height),
      _pathColor(pathColor), _wallColor(wallColor),
      _runnerColor(runnerColor), _sentryColor(sentryColor),
      _exitColor(exitColor), _generationOptions(generationOptions),
      _sentryActive(sentryColor != pathColor),
      _drawPixel(drawPixel), _setStatus(setStatus)
  {
    if (_width < 1 || _height < 1 || _width > MAZE_RUNNER_WIDTH ||
        _height > MAZE_RUNNER_HEIGHT || (uint16_t)_width * _height > MAZE_MAX_CELLS ||
        (_generationOptions.outerWalls && (_width < 3 || _height < 3)))
    {
      _width = _height = 0;
    }
  }

  MazeRunner(int width, int height,
             uint32_t pathColor, uint32_t wallColor,
             uint32_t runnerColor, uint32_t exitColor,
             DrawPixelCallback drawPixel,
             StatusCallback setStatus = nullptr,
             MazeGenerationOptions generationOptions = {})
    : MazeRunner(width, height, pathColor, wallColor, runnerColor,
                 pathColor, exitColor, drawPixel, setStatus, generationOptions) {}

  void init();
  bool update();

private:
  bool moveRunner();
  bool moveSentry();
  void drawMaze();
  void generateMaze();
  void placeRunner();
  void placeSentry();
  void placeExit();
  bool findPathDfs(Location startLoc, Location sentryLoc, Location endLoc,
                   int maxSearchDistance, uint8_t *outPath, uint16_t &outSize);
  bool findLongestPathBfs(Location startLoc, Location sentryLoc,
                          int maxSearchDistance, uint8_t *outPath, uint16_t &outSize, bool edgeOnly = false);
  bool isAdjacent(Location a, Location b) { return isInBounds(a) && isInBounds(b) && abs(a.x-b.x)+abs(a.y-b.y)==1; }
  bool isWall(int x, int y) const;
  bool isWall(Location loc) const;
  bool isInBounds(int x, int y) const;
  bool isInBounds(Location loc) const;
  int getAdjacentWallAndBorderCount(int x, int y);
  int getAdjacentWallAndBorderCount(Location loc);
  bool isGenerationCell(Location loc) const;
  bool wouldTouchDiagonally(Location loc) const;
  bool isVisited(uint8_t cell) const { return (_runnerVisited[cell >> 3] & (uint8_t)(1U << (cell & 7))) != 0; }
  void markVisited(Location loc) { if (isInBounds(loc)) { uint8_t cell = encode(loc); _runnerVisited[cell >> 3] |= (uint8_t)(1U << (cell & 7)); } }
  bool findNearestUnvisitedPath(Location start, uint8_t *outPath, uint16_t &outSize);
  bool isInnerEdgeCell(Location loc) const;
};

inline void MazeRunner::init()
{
  if (_width == 0 || _height == 0) return;
  log_v("Initializing maze");

  // A successful run carries its goal location forward as the next start.
  _startAtPreviousGoal = _exitLoc != NullLocation && _runnerLoc == _exitLoc;
  if (!_startAtPreviousGoal && _sentryActive && _runnerLoc == _sentryLoc)
    _runnerLoc = NullLocation; // caught: start a fresh run
  if (_generationOptions.outerWalls && !_startAtPreviousGoal)
    _runnerLoc = NullLocation;

  for (uint16_t i = 0; i < MAZE_BITSET_BYTES; ++i) _runnerVisited[i] = 0;
  generateMaze();
  placeRunner();
  placeExit();
  placeSentry();
  markVisited(_runnerLoc);

  drawMaze();
}

inline bool MazeRunner::update()
{
  if (_resetDelay >= 0)
  {
    --_resetDelay;
    if (_resetDelay < 0)
    {
      init();
      drawMaze();
      return true;
    }
    return false;
  }

  bool changed = moveRunner();
  if (_runnerLoc == _exitLoc)
  {
    log_d("Runner reached exit");
    if (_setStatus) _setStatus(_runnerColor);
    drawMaze();
    _resetDelay = _sentryActive ? GoalDelay : 0;
    return true;
  }

  if (_sentryActive)
  {
    changed |= moveSentry();
    if (_sentryLoc == _runnerLoc)
    {
      log_d("Runner caught by sentry");
      if (_setStatus) _setStatus(_sentryColor);
      drawMaze(); // redraw the collision frame before resetting
      _resetDelay = CatchDelay;
      return true;
    }
  }

  if (changed) drawMaze();
  return changed;
}

inline bool MazeRunner::moveRunner()
{
  if (_runnerCooldown) { --_runnerCooldown; return false; }

  if (!_generationOptions.perfectVision) markVisited(_runnerLoc);
  uint16_t tempSize = 0;
  bool sensed = _sentryActive &&
    findPathDfs(_runnerLoc, NullLocation, _sentryLoc, RunnerSense, _runnerPath, tempSize) &&
    tempSize > 0;

    
  if (sensed)
  {
    _runnerSentryKnownLoc = _sentryLoc;
    findLongestPathBfs(_runnerLoc, _sentryLoc, RunnerSense + RunnerFear, _runnerPath, _runnerPathSize);
    if (_runnerPathSize > RunnerSense) _runnerPathSize = RunnerSense;
  }
  else if (_runnerPathSize == 0)
  {
    // If the exit is nearby, head for it instead of walking past to explore.
    if (!findPathDfs(_runnerLoc, _runnerSentryKnownLoc, _exitLoc, 2,
                     _runnerPath, _runnerPathSize))
    {
      if (!_generationOptions.perfectVision)
        findNearestUnvisitedPath(_runnerLoc, _runnerPath, _runnerPathSize);
      if (_runnerPathSize == 0)
        findPathDfs(_runnerLoc, _runnerSentryKnownLoc, _exitLoc, -1,
                    _runnerPath, _runnerPathSize);
    }
    _runnerSentryKnownLoc = NullLocation;
  }

  Location next;
  if (popFrontPath(_runnerPath, _runnerPathSize, next))
  {
    Location prev = _runnerLoc;
    _runnerLoc = next;
    if (!_generationOptions.perfectVision) markVisited(_runnerLoc);
    _runnerCooldown = _sentryActive ? RunnerSpeed : 0;
    log_v("Moved runner from (%d,%d) to (%d,%d)", prev.x, prev.y, _runnerLoc.x, _runnerLoc.y);
    return true;
  }
  return false;
}

inline bool MazeRunner::moveSentry()
{
  if (!_sentryActive) return false;
  if (_sentryCooldown) { --_sentryCooldown; return false; }

  uint16_t pathSize = 0;

  // If the runner is within sensing range, update the target to its current location.
  bool seesRunner = findPathDfs(_sentryLoc, NullLocation, _runnerLoc,
                                SentrySense, _runnerPath, pathSize) && pathSize > 0;
  if (seesRunner)
  {
    _sentryKnownRunnerLoc = _runnerLoc;
  }
  else
  {
    // Keep following the last-seen location instead of stopping as soon as
    // the runner moves out of sensing range. Recalculate one step at a time
    // so the guardian doesn't need a second full-size path buffer.
    if (!isInBounds(_sentryKnownRunnerLoc) || _sentryKnownRunnerLoc == _sentryLoc)
    {
      _sentryKnownRunnerLoc = NullLocation;
      _runnerPathSize = 0;
      return false;
    }

    if (!findPathDfs(_sentryLoc, NullLocation, _sentryKnownRunnerLoc,
                     -1, _runnerPath, pathSize) || pathSize == 0)
    {
      _sentryKnownRunnerLoc = NullLocation;
      _runnerPathSize = 0;
      return false;
    }
  }

  Location next = decode(_runnerPath[0]);
  _runnerPathSize = 0;

  // Don't let the guardian occupy a choke point that cuts off the exit.
  bool escapeExists = true;
  if (next != _runnerLoc)
  {
    uint16_t escapeSize = 0;
    escapeExists = findPathDfs(_runnerLoc, next, _exitLoc, -1, _runnerPath, escapeSize);
  }
  _runnerPathSize = 0; // path check reused the runner buffer; replan on its next turn
  if (!escapeExists)
  {
    _sentryCooldown = SentrySpeed;
    return false;
  }

  Location prev = _sentryLoc;
  _sentryLoc = next;
  _sentryCooldown = SentrySpeed;

  // If the guardian reached the last-seen location but the runner is gone,
  // it has lost the trail and should wait for another sighting.
  if (_sentryLoc == _sentryKnownRunnerLoc && _sentryLoc != _runnerLoc)
    _sentryKnownRunnerLoc = NullLocation;

  log_v("Moved sentry from (%d,%d) to (%d,%d)", prev.x, prev.y, _sentryLoc.x, _sentryLoc.y);
  return true;
}

inline void MazeRunner::drawMaze()
{
  if (!_drawPixel) return;
  for (int y = 0; y < _height; ++y)
    for (int x = 0; x < _width; ++x)
      _drawPixel(x, y, isWall(x, y) ? _wallColor : _pathColor);

  if (!_generationOptions.perfectVision)
  {
    for (int y = 0; y < _height; ++y)
      for (int x = 0; x < _width; ++x)
        if (!isWall(x, y) && isVisited(encode({x, y})))
          _drawPixel(x, y, _generationOptions.visitedColor);
  }
  if (isInBounds(_exitLoc)) _drawPixel(_exitLoc.x, _exitLoc.y, _exitColor);
  if (isInBounds(_runnerLoc)) _drawPixel(_runnerLoc.x, _runnerLoc.y, _runnerColor);
  if (_sentryActive && isInBounds(_sentryLoc)) _drawPixel(_sentryLoc.x, _sentryLoc.y, _sentryColor);
}

inline void MazeRunner::generateMaze()
{
  log_d("Starting maze generation");
  for (int y=0; y<_height; ++y)
    for (int x=0; x<_width; ++x) setWall(x, y, true);

  Location start;
  if (_generationOptions.outerWalls)
  {
    if (_startAtPreviousGoal && isInBounds(_runnerLoc))
    {
      start = _runnerLoc;
      if (start.x == 0) start.x = 1;
      else if (start.x == _width - 1) start.x = _width - 2;
      else if (start.y == 0) start.y = 1;
      else if (start.y == _height - 1) start.y = _height - 2;
    }
    else
      start = {1 + (int)random(_width - 2), 1 + (int)random(_height - 2)};
  }
  else if (isInBounds(_runnerLoc)) start = _runnerLoc;
  else start = {(int)random(_width), (int)random(_height)};
  setWall(start.x, start.y, false);

  uint8_t stackCells[MAZE_MAX_CELLS];
  uint16_t stackSize = 0;
  stackCells[stackSize++] = encode(start);
  int maxCycles = 1000;

  while (stackSize && maxCycles-- > 0)
  {
    Location cur = decode(stackCells[stackSize - 1]);
    Direction steps[4] = {Left, Right, Up, Down};
    shuffle(steps, 4);
    bool found = false;
    for (int i=0; i<4; ++i)
    {
      Location next = cur + steps[i];
      if (isGenerationCell(next) && isWall(next) &&
          getAdjacentWallAndBorderCount(next) >= 3 &&
          !(_generationOptions.preventDiagonalTouching && wouldTouchDiagonally(next)))
      {
        setWall(next.x, next.y, false);
        if (stackSize < MAZE_MAX_CELLS) stackCells[stackSize++] = encode(next);
        found = true;
        break;
      }
    }
    if (!found) --stackSize;
  }

  int removed = 0;
  maxCycles = 1000;
  while (removed < _mazeExtraWallsToRemove && maxCycles-- > 0)
  {
    int x = random(_width), y = random(_height);
    if (isWall(x,y) && isGenerationCell({x, y}) &&
        getAdjacentWallAndBorderCount(x,y) >= 2 &&
        !(_generationOptions.preventDiagonalTouching && wouldTouchDiagonally({x, y})))
    {
      setWall(x, y, false);
      ++removed;
    }
  }
  if (maxCycles <= 0) log_e("Failed to remove extra walls");
  log_d("Maze generation complete");
}

inline void MazeRunner::placeRunner()
{
  clearRunnerPath();
  _runnerSentryKnownLoc = NullLocation;
  _runnerCooldown = 0;

  if (_generationOptions.outerWalls)
  {
    if (_startAtPreviousGoal && isInBounds(_runnerLoc))
    {
      Location inside = _runnerLoc;
      if (inside.x == 0) ++inside.x;
      else if (inside.x == _width - 1) --inside.x;
      else if (inside.y == 0) ++inside.y;
      else if (inside.y == _height - 1) --inside.y;
      setWall(_runnerLoc.x, _runnerLoc.y, false);
      setWall(inside.x, inside.y, false);
      return;
    }
    // Start at a perimeter entrance connected to an open inner-edge cell.
    for (int attempts = 0; attempts < (_width + _height) * 4; ++attempts)
    {
      int side = random(4);
      int x = 1, y = 1;
      if (side < 2) { x = 1 + random(_width - 2); y = side == 0 ? 1 : _height - 2; }
      else { y = 1 + random(_height - 2); x = side == 2 ? 1 : _width - 2; }
      if (!isWall(x, y))
      {
        _runnerLoc = {x, y};
        if (y == 1) _runnerLoc = {x, 0};
        else if (y == _height - 2) _runnerLoc = {x, _height - 1};
        else if (x == 1) _runnerLoc = {0, y};
        else if (x == _width - 2) _runnerLoc = {_width - 1, y};
        setWall(_runnerLoc.x, _runnerLoc.y, false);
        return;
      }
    }
    // Small or unusually shaped mazes may not have a carved inner-edge cell.
    _runnerLoc = {1, 1};
    setWall(1, 1, false);
    _runnerLoc = {1, 0};
    setWall(1, 0, false);
    return;
  }

  if (_runnerLoc.x > 0 && _runnerLoc.y > 0)
  {
    if (_runnerLoc == _exitLoc || (_sentryActive && _runnerLoc == _sentryLoc)) return;
    log_e("Runner unexpectedly not at exit or sentry loc: (%d,%d)", _runnerLoc.x, _runnerLoc.y);
  }

  int attempts = 0;
  while (_runnerLoc.x < 0 || _runnerLoc.y < 0)
  {
    int x = random(_width), y = random(_height);
    if (!isWall(x,y)) _runnerLoc = {x,y};
    if (++attempts > 1000) break;
  }
  log_d("Placing runner at (%d,%d) after %d attempts", _runnerLoc.x, _runnerLoc.y, attempts);
}

inline void MazeRunner::placeSentry()
{
  if (!_sentryActive) return;
  _sentryLoc = NullLocation;
  _sentryCooldown = SentrySpeed;

  int attempts = 0;
  while (_sentryLoc.x == -1 && attempts < 2000)
  {
    Location candidate = {(int)random(_width), (int)random(_height)};
    int distance = abs(candidate.x-_runnerLoc.x) + abs(candidate.y-_runnerLoc.y);
    int minDistance = max(0, (_width + _height) / 2 - (attempts / 10));
    if (!isWall(candidate) && candidate != _runnerLoc && candidate != _exitLoc && distance > minDistance)
    {
      // Reject positions that would cut off every runner route to the exit.
      uint16_t escapeSize = 0;
      bool escapeExists = findPathDfs(_runnerLoc, candidate, _exitLoc, -1, _runnerPath, escapeSize);
      _runnerPathSize = 0;
      if (escapeExists) _sentryLoc = candidate;
    }
    ++attempts;
  }

  // Small mazes may not have a candidate at the preferred distance; relax that constraint.
  if (_sentryLoc == NullLocation)
  {
    for (int y = 0; y < _height && _sentryLoc == NullLocation; ++y)
      for (int x = 0; x < _width && _sentryLoc == NullLocation; ++x)
      {
        Location candidate = {x, y};
        uint16_t escapeSize = 0;
        if (!isWall(candidate) && candidate != _runnerLoc && candidate != _exitLoc)
        {
          bool escapeExists = findPathDfs(_runnerLoc, candidate, _exitLoc, -1, _runnerPath, escapeSize);
          _runnerPathSize = 0;
          if (escapeExists) _sentryLoc = candidate;
        }
      }
  }
  log_d("Placing sentry at (%d,%d) after %d attempts", _sentryLoc.x, _sentryLoc.y, attempts);
}

inline void MazeRunner::placeExit()
{
  _exitLoc = NullLocation;
  uint16_t pathSize = 0;
  if (!findLongestPathBfs(_runnerLoc, NullLocation, -1, _runnerPath, pathSize, _generationOptions.outerWalls) || pathSize == 0)
  {
    log_e("Failed to find path to exit");
    if (_setStatus) _setStatus(_exitColor);
    _resetDelay = ErrorDelay;
    return;
  }
  _exitLoc = decode(_runnerPath[pathSize - 1]);
  if (_generationOptions.outerWalls)
  {
    Location inside = _exitLoc;
    if (inside.y == 1) --_exitLoc.y;
    else if (inside.y == _height - 2) ++_exitLoc.y;
    else if (inside.x == 1) --_exitLoc.x;
    else if (inside.x == _width - 2) ++_exitLoc.x;
    else
    {
      // No reachable inner-edge cell was found.
      log_e("Failed to place perimeter exit");
      return;
    }
    setWall(_exitLoc.x, _exitLoc.y, false);
  }
  clearRunnerPath();
  log_d("Placing exit at (%d,%d) with distance %d from runner", _exitLoc.x, _exitLoc.y, pathSize);
}

inline bool MazeRunner::findPathDfs(Location start, Location sentry, Location end,
                                    int maxDistance, uint8_t *outPath, uint16_t &outSize)
{
  outSize = 0;
  if (!isInBounds(start) || !isInBounds(end)) return false;

  // Mark cells when pushed, so the bounded stack cannot grow beyond cell count.
  uint8_t stackCell[MAZE_MAX_CELLS];
  uint8_t parent[MAZE_MAX_CELLS];
  uint8_t visited[MAZE_BITSET_BYTES] = {};
  uint16_t top = 0;
  uint8_t startId = encode(start);
  stackCell[top] = startId;
  ++top;
  visited[startId >> 3] |= (uint8_t)(1U << (startId & 7));
  parent[startId] = startId;

  bool found = false;
  while (top)
  {
    --top;
    uint8_t id = stackCell[top];
    Location cur = decode(id);
    if (cur == end) { found = true; break; }
    if (maxDistance > 0)
    {
      uint16_t depth = 0;
      uint8_t ancestor = id;
      while (ancestor != startId && depth <= (uint16_t)maxDistance)
      {
        ancestor = parent[ancestor];
        ++depth;
      }
      if (depth >= (uint16_t)maxDistance) continue;
    }

    Direction steps[4] = {Left, Right, Up, Down};
    shuffle(steps, 4);
    for (int i=0; i<4; ++i)
    {
      Location next = cur + steps[i];
      if (!isInBounds(next) || isWall(next) || next == sentry) continue;
      uint8_t nid = encode(next);
      if (visited[nid >> 3] & (uint8_t)(1U << (nid & 7))) continue;
      visited[nid >> 3] |= (uint8_t)(1U << (nid & 7));
      parent[nid] = id;
      if (top < MAZE_MAX_CELLS)
      {
        stackCell[top] = nid;
        ++top;
      }
    }
  }

  if (!found) return false;
  uint8_t cur = encode(end);
  while (cur != startId && outSize < MAZE_MAX_CELLS)
  {
    outPath[outSize++] = cur;
    cur = parent[cur];
  }
  for (uint16_t i = 0; i < outSize / 2; ++i)
  {
    uint8_t temp = outPath[i];
    outPath[i] = outPath[outSize - 1 - i];
    outPath[outSize - 1 - i] = temp;
  }
  return true;
}

inline bool MazeRunner::findLongestPathBfs(Location start, Location sentry,
                                           int maxDistance, uint8_t *outPath, uint16_t &outSize, bool edgeOnly)
{
  outSize = 0;
  if (!isInBounds(start)) return false;

  uint8_t queue[MAZE_MAX_CELLS];
  uint8_t parent[MAZE_MAX_CELLS];
  uint8_t visited[MAZE_BITSET_BYTES] = {};
  uint16_t head = 0, tail = 0;
  uint8_t startId = encode(start);
  queue[tail++] = startId;
  parent[startId] = startId;
  visited[startId >> 3] |= (uint8_t)(1U << (startId & 7));
  uint8_t farthest = startId;

  while (head < tail)
  {
    uint8_t id = queue[head++];
    Location cur = decode(id);
    if (!edgeOnly || isInnerEdgeCell(cur)) farthest = id; // BFS visits cells in nondecreasing distance order.
    if (maxDistance > 0)
    {
      uint16_t depth = 0;
      uint8_t ancestor = id;
      while (ancestor != startId && depth <= (uint16_t)maxDistance)
      {
        ancestor = parent[ancestor];
        ++depth;
      }
      if (depth >= (uint16_t)maxDistance) continue;
    }

    Direction steps[4] = {Left, Right, Up, Down};
    shuffle(steps, 4);
    for (int i=0; i<4; ++i)
    {
      Location next = cur + steps[i];
      if (!isInBounds(next) || isWall(next) || next == sentry) continue;
      uint8_t nid = encode(next);
      if (visited[nid >> 3] & (uint8_t)(1U << (nid & 7))) continue;
      visited[nid >> 3] |= (uint8_t)(1U << (nid & 7));
      parent[nid] = id;
      queue[tail++] = nid;
    }
  }

  if (farthest == startId) return false;
  uint8_t cur = farthest;
  while (cur != startId && outSize < MAZE_MAX_CELLS)
  {
    outPath[outSize++] = cur;
    cur = parent[cur];
  }
  for (uint16_t i = 0; i < outSize / 2; ++i)
  {
    uint8_t temp = outPath[i];
    outPath[i] = outPath[outSize - 1 - i];
    outPath[outSize - 1 - i] = temp;
  }
  return outSize > 0;
}

inline bool MazeRunner::findNearestUnvisitedPath(Location start, uint8_t *outPath, uint16_t &outSize)
{
  outSize = 0;
  if (!isInBounds(start)) return false;

  uint8_t queue[MAZE_MAX_CELLS];
  uint8_t parent[MAZE_MAX_CELLS];
  uint8_t visited[MAZE_BITSET_BYTES] = {};
  uint16_t head = 0, tail = 0;
  uint8_t startId = encode(start);
  queue[tail++] = startId;
  parent[startId] = startId;
  visited[startId >> 3] |= (uint8_t)(1U << (startId & 7));
  uint8_t target = startId;
  bool found = false;

  while (head < tail && !found)
  {
    uint8_t id = queue[head++];
    if (id != startId && !isVisited(id))
    {
      target = id;
      found = true;
      break;
    }

    Location cur = decode(id);
    Direction steps[4] = {Left, Right, Up, Down};
    shuffle(steps, 4);
    for (int i = 0; i < 4; ++i)
    {
      Location next = cur + steps[i];
      if (!isInBounds(next) || isWall(next) || next == _sentryLoc) continue;
      uint8_t nid = encode(next);
      if (visited[nid >> 3] & (uint8_t)(1U << (nid & 7))) continue;
      visited[nid >> 3] |= (uint8_t)(1U << (nid & 7));
      parent[nid] = id;
      if (tail < MAZE_MAX_CELLS) queue[tail++] = nid;
    }
  }

  if (!found) return false;
  uint8_t cur = target;
  while (cur != startId && outSize < MAZE_MAX_CELLS)
  {
    outPath[outSize++] = cur;
    cur = parent[cur];
  }
  for (uint16_t i = 0; i < outSize / 2; ++i)
  {
    uint8_t temp = outPath[i];
    outPath[i] = outPath[outSize - 1 - i];
    outPath[outSize - 1 - i] = temp;
  }
  return outSize > 0;
}

inline bool MazeRunner::isWall(int x, int y) const { return wallAt(x, y); }
inline bool MazeRunner::isWall(Location loc) const { return isWall(loc.x, loc.y); }
inline bool MazeRunner::isInBounds(int x, int y) const { return x >= 0 && x < _width && y >= 0 && y < _height; }
inline bool MazeRunner::isInBounds(Location loc) const { return isInBounds(loc.x, loc.y); }

inline int MazeRunner::getAdjacentWallAndBorderCount(int x, int y)
{
  int count = 0;
  for (int i=0; i<4; ++i)
  {
    int nx = x + Directions[i].x, ny = y + Directions[i].y;
    if (nx < 0 || nx >= _width || ny < 0 || ny >= _height || isWall(nx, ny)) ++count;
  }
  return count;
}
inline int MazeRunner::getAdjacentWallAndBorderCount(Location loc)
{
  return getAdjacentWallAndBorderCount(loc.x, loc.y);
}

inline bool MazeRunner::isGenerationCell(Location loc) const
{
  if (!isInBounds(loc)) return false;
  if (!_generationOptions.outerWalls) return true;
  return loc.x > 0 && loc.x < _width - 1 && loc.y > 0 && loc.y < _height - 1;
}

inline bool MazeRunner::wouldTouchDiagonally(Location loc) const
{
  for (int dx = -1; dx <= 1; dx += 2)
    for (int dy = -1; dy <= 1; dy += 2)
    {
      int nx = loc.x + dx, ny = loc.y + dy;
      if (!isInBounds(nx, ny) || isWall(nx, ny)) continue;
      // Diagonal paths are separate only when both orthogonal bridges are walls.
      if (isWall(loc.x + dx, loc.y) && isWall(loc.x, loc.y + dy)) return true;
    }
  return false;
}

inline bool MazeRunner::isInnerEdgeCell(Location loc) const
{
  return loc.x > 0 && loc.x < _width - 1 && loc.y > 0 && loc.y < _height - 1 &&
         (loc.x == 1 || loc.x == _width - 2 || loc.y == 1 || loc.y == _height - 2) &&
         !isWall(loc);
}
