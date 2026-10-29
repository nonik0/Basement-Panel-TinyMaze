#pragma once

#include <Arduino.h>

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
// Search parent links: 2 bits per cell (an index into Directions).
static const uint16_t MAZE_PARENT_BYTES = (MAZE_MAX_CELLS + 3) / 4;

// Largest MazeOptions::runnerSense. It sizes a fixed stack queue of 2r^2 + 2r + 1 cells (the most
// a grid has within r steps): 41 bytes at 4.
#ifndef MAZE_MAX_SENSE
#define MAZE_MAX_SENSE 4
#endif
#if MAZE_MAX_SENSE < 1 || MAZE_MAX_SENSE > 10
#error "MAZE_MAX_SENSE must be between 1 and 10"
#endif

// Every field has a default, so set only what you change, e.g. from a constexpr function:
//   constexpr MazeOptions makeOptions() { MazeOptions o; o.outerWalls = true; return o; }
// Delays and speeds count update() calls (frames).
struct MazeOptions
{
  // --- Maze ---
  // Keep the display perimeter as walls, with an entrance and exit opening.
  bool outerWalls = false;
  // Avoid diagonal-only contact between otherwise separate corridors.
  bool preventDiagonalTouching = false;
  // Corridor walls knocked out after carving. A perfect (tree) maze has dead-end pockets with no
  // way around a sentry walking in; loops give the runner alternate routes. More loops, fewer
  // catches: in simulation 2 retried ~5-8% of mazes (at most 4 catches on one), 6 almost none.
  uint8_t extraWallsToRemove = 2;

  // --- Runner ---
  // If false, the runner explores and marks cells instead of navigating directly to the exit.
  bool perfectVision = true;
  // Color used to mark explored cells when perfectVision is false.
  uint32_t visitedColor = 0x000020UL;
  // Exploring mode: if true, the runner sees every open cell within runnerSense steps, around
  // corners. If false, it sees only in straight lines until a wall blocks the view.
  bool seeAroundCorners = false;
  // Steps within which the runner notices the sentry (and flees) and, when exploring, sees
  // cells. Clamped to 1..MAZE_MAX_SENSE.
  uint8_t runnerSense = 2;
  // Frames between runner moves while a sentry is active; without one it moves every frame.
  uint8_t runnerSpeed = 3;
  // How far past runnerSense a flee may aim.
  uint8_t runnerFear = 10;

  // --- Sentry ---
  // Sentry guards the exit: it spawns beside it and patrols from junction to junction, pursuing
  // the runner when it is sensed and returning to patrol after losing it. If false, the sentry
  // guards a random post that leaves the runner a way around it, and returns there after a chase.
  bool sentryPatrol = false;
  // Steps within which the sentry notices the runner; at least 1. One more than runnerSense lets
  // the sentry notice the runner first, so some rounds end in a catch and a retry (~15% of mazes
  // in simulation, vs ~7% when equal).
  uint8_t sentrySense = 3;
  // Frames between sentry moves.
  uint8_t sentrySpeed = 5;

  // --- Timing ---
  // Pause after the runner reaches the exit (only when there is a sentry).
  uint16_t goalDelay = 0;
  // Pause after a catch, before the retry.
  uint16_t catchDelay = 30;
  // Frames after which a maze is abandoned for a new one, counted across all its retries; 0
  // disables. Runner and sentry can reach standoffs that never resolve (a chase around a loop
  // where the faster runner is never caught but never gets past), and some mazes end in a catch
  // nearly every try.
  uint16_t roundTimeout = 3000;
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

template <typename T>
inline void shuffle(T *list, int size)
{
  for (int i = 0; i < size; ++i)
  {
    int index = random(size);
    T temp = list[i];
    list[i] = list[index];
    list[index] = temp;
  }
}

inline bool upOrDown(Direction dir) { return dir == Up || dir == Down; }
inline bool leftOrRight(Direction dir) { return dir == Left || dir == Right; }

class MazeRunner
{
private:
  static const int ErrorDelay = 30000;
  static const uint8_t NoDepthLimit = 0;
  static const uint8_t MaxZoneCells = 2 * MAZE_MAX_SENSE * (MAZE_MAX_SENSE + 1) + 1;

  // configuration
  int _width;
  int _height;
  uint32_t _pathColor, _wallColor, _runnerColor, _sentryColor, _exitColor;
  MazeOptions _options;

  // callbacks
  typedef void (*DrawPixelCallback)(int, int, uint32_t);
  typedef void (*StatusCallback)(uint32_t);
  DrawPixelCallback _drawPixel;
  StatusCallback _setStatus;

  // maze and round state
  uint8_t _mazeWalls[MAZE_BITSET_BYTES];
  Location _exitLoc = NullLocation;
  int _resetDelay = -1;
  uint16_t _roundFrames = 0;
  
  // Shared path buffer used by findPath().
  uint8_t _searchPath[MAZE_MAX_CELLS];

  // runner state
  Location _runnerLoc = NullLocation;
  Location _runnerKnownSentryLoc = NullLocation;
  Location _runnerFleeTarget = NullLocation;
  Location _runnerSpawnLoc = NullLocation;
  uint8_t _runnerFleeWait = 0;
  bool _runnerCaught = false;
  uint8_t _runnerCooldown = 0;
  uint8_t _runnerVisited[MAZE_BITSET_BYTES] = {};
  
  // sentry state
  bool _sentryActive = false;
  Location _sentryLoc = NullLocation;
  Location _sentryKnownRunnerLoc = NullLocation;
  Location _sentryPrevLoc = NullLocation;
  Location _sentrySpawnLoc = NullLocation;
  uint8_t _sentryCooldown = 0;
public:
  MazeRunner(int width, int height,
             uint32_t pathColor, uint32_t wallColor,
             uint32_t runnerColor, uint32_t sentryColor,
             uint32_t exitColor,
             DrawPixelCallback drawPixel,
             StatusCallback setStatus = nullptr,
             MazeOptions options = {})
    : _width(width), _height(height),
      _pathColor(pathColor), _wallColor(wallColor),
      _runnerColor(runnerColor), _sentryColor(sentryColor),
      _exitColor(exitColor), _options(options),
      _sentryActive(sentryColor != pathColor),
      _drawPixel(drawPixel), _setStatus(setStatus)
  {
    if (_width < 1 || _height < 1 || _width > MAZE_RUNNER_WIDTH ||
        _height > MAZE_RUNNER_HEIGHT || (uint16_t)_width * _height > MAZE_MAX_CELLS ||
        (_options.outerWalls && (_width < 3 || _height < 3)))
    {
      _width = _height = 0;
    }
    // A sense of 0 would mean "no depth limit" to findPath().
    if (_options.runnerSense < 1) _options.runnerSense = 1;
    if (_options.runnerSense > MAZE_MAX_SENSE) _options.runnerSense = MAZE_MAX_SENSE;
    if (_options.sentrySense < 1) _options.sentrySense = 1;
  }

  MazeRunner(int width, int height,
             uint32_t pathColor, uint32_t wallColor,
             uint32_t runnerColor, uint32_t exitColor,
             DrawPixelCallback drawPixel,
             StatusCallback setStatus = nullptr,
             MazeOptions options = {})
    : MazeRunner(width, height, pathColor, wallColor, runnerColor,
                 pathColor, exitColor, drawPixel, setStatus, options) {}

  void init();
  bool update();

private:
  // maze functions
  void drawMaze();
  void generateMaze();

  // runner and round functions
  bool moveRunner();
  void resolveStartFromLastRound();
  void retryRound();

  // sentry behavior
  void placeSentry();
  bool moveSentry();
  Location sentryPatrol();
  Location chooseJunctionPath();

  // pathing
  enum SearchGoal : uint8_t
  {
    GoalCell,              // `target`
    GoalJunction,          // nearest junction other than the start
    GoalUnvisited,         // nearest cell the runner has not explored
    GoalFarthest,          // last cell reached (farthest within the depth limit)
    GoalFarthestInnerEdge, // farthest open cell against the outer wall
  };
  // Returns the path length, 0 for start == target, or -1 if no path was found.
  int16_t findPath(SearchGoal goal, Location start, Location target, Location avoid,
                   uint8_t maxDepth, uint8_t avoidRadius = 0);
  void markSeen();
  void markWithin(Location center, uint8_t radius, uint8_t *cells) const;
  
  // maze helpers
  bool isWall(int x, int y) const;
  int adjacentWallCount(int x, int y) const;

  bool hasDiagonalOnlyNeighbor(Location loc) const;
  bool leadsToJunction(Location from, Location prev) const;
  bool isInnerEdgeCell(Location loc) const;
  bool isCorridorWall(Location loc) const;

  // inlines
  uint8_t encode(Location loc) const { return (uint8_t)(loc.y * _width + loc.x); }
  Location decode(uint8_t cell) const { return {(int)(cell % _width), (int)(cell / _width)}; }
  bool isAdjacent(Location a, Location b) { return isInBounds(a) && isInBounds(b) && abs(a.x-b.x)+abs(a.y-b.y)==1; }
  bool isWall(Location loc) const { return isWall(loc.x, loc.y); }
  bool isInBounds(int x, int y) const { return x >= 0 && x < _width && y >= 0 && y < _height; }
  bool isInBounds(Location loc) const { return isInBounds(loc.x, loc.y); }
  bool isJunction(Location loc) const { return !isWall(loc) && adjacentPassageCount(loc) >= 3; }
  bool isVisited(uint8_t cell) const { return (_runnerVisited[cell >> 3] & (uint8_t)(1U << (cell & 7))) != 0; }
  int adjacentWallCount(Location loc) const { return adjacentWallCount(loc.x, loc.y); }
  int adjacentPassageCount(Location loc) const { return 4 - adjacentWallCount(loc); }
};

inline void MazeRunner::init()
{
  if (_width == 0 || _height == 0) return;
  log_v("Initializing maze");

  resolveStartFromLastRound();

  for (uint16_t i = 0; i < MAZE_BITSET_BYTES; ++i) _runnerVisited[i] = 0;
  // TODO hate how these are set manually here
  _runnerKnownSentryLoc = NullLocation;
  _runnerFleeTarget = NullLocation;
  _runnerCaught = false;
  _runnerFleeWait = 0;
  _roundFrames = 0;
  generateMaze();
  _runnerSpawnLoc = _runnerLoc;
  placeSentry();
  markSeen();

  drawMaze();
}

inline bool MazeRunner::update()
{
  if (_resetDelay >= 0)
  {
    --_resetDelay;
    if (_resetDelay < 0)
    {
      if (_runnerCaught) retryRound(); // same maze, runner and sentry back at their starts
      else init(); // new maze (init draws it)
      return true;
    }
    return false;
  }

  if (_options.roundTimeout && ++_roundFrames > _options.roundTimeout)
  {
    log_w("Round stalled; starting a new maze");
    init();
    return true;
  }

  bool changed = moveRunner();
  if (_runnerLoc == _exitLoc)
  {
    log_d("Runner reached exit");
    if (_setStatus) _setStatus(_runnerColor);
    drawMaze();
    _resetDelay = _sentryActive ? _options.goalDelay : 0;
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
      _runnerCaught = true;
      _resetDelay = _options.catchDelay;
      return true;
    }
  }

  if (changed) drawMaze();
  return changed;
}

inline void MazeRunner::drawMaze()
{
  if (!_drawPixel) return;
  for (int y = 0; y < _height; ++y)
    for (int x = 0; x < _width; ++x)
      _drawPixel(x, y, isWall(x, y) ? _wallColor : _pathColor);

  if (!_options.perfectVision)
  {
    for (int y = 0; y < _height; ++y)
      for (int x = 0; x < _width; ++x)
        if (!isWall(x, y) && isVisited(encode({x, y})))
          _drawPixel(x, y, _options.visitedColor);
  }
  // These can legitimately be NullLocation (-1,-1): the exit when placeExit() failed, the
  // sentry when no valid spot was found, the runner before the first placement. Skip those
  // rather than call drawPixel with a negative coordinate.
  if (isInBounds(_runnerSpawnLoc)) _drawPixel(_runnerSpawnLoc.x, _runnerSpawnLoc.y, _pathColor);
  if (isInBounds(_exitLoc)) _drawPixel(_exitLoc.x, _exitLoc.y, _exitColor);
  if (isInBounds(_runnerLoc)) _drawPixel(_runnerLoc.x, _runnerLoc.y, _runnerColor);
  if (_sentryActive && isInBounds(_sentryLoc)) _drawPixel(_sentryLoc.x, _sentryLoc.y, _sentryColor);
}

inline void MazeRunner::generateMaze()
{
  log_d("Starting maze generation");

  // Can the generator dig this cell open? With outerWalls, the perimeter is
  // reserved for the entrance and exit.
  auto isCarvable = [this](Location loc) -> bool
  {
    if (!isInBounds(loc)) return false;
    if (!_options.outerWalls) return true;
    return loc.x > 0 && loc.x < _width - 1 &&
           loc.y > 0 && loc.y < _height - 1;
  };
  auto setWall = [this](int x, int y, bool wall) -> void
  {
    uint16_t cell = (uint16_t)y * _width + x;
    uint8_t mask = (uint8_t)(1U << (cell & 7));
    if (wall) _mazeWalls[cell >> 3] |= mask;
    else _mazeWalls[cell >> 3] &= (uint8_t)~mask;
  };

  for (int y = 0; y < _height; ++y)
    for (int x = 0; x < _width; ++x) setWall(x, y, true);

  Location start;

  if (_options.outerWalls)
  {
    if (isInBounds(_runnerLoc))
    {
      // The previous exit becomes this round's entrance.
      start = _runnerLoc;
      if (start.x == 0) ++start.x;
      else if (start.x == _width - 1) --start.x;
      else if (start.y == 0) ++start.y;
      else if (start.y == _height - 1) --start.y;
    }
    else
    {
      // Choose the entrance before generating the maze, so the maze is
      // generated from the runner's actual starting location.
      int side = random(4);
      int x, y;

      if (side < 2)
      {
        x = 1 + random(_width - 2);
        y = side == 0 ? 0 : _height - 1;
      }
      else
      {
        x = side == 2 ? 0 : _width - 1;
        y = 1 + random(_height - 2);
      }

      _runnerLoc = {x, y};
      start = _runnerLoc;

      if (start.x == 0) ++start.x;
      else if (start.x == _width - 1) --start.x;
      else if (start.y == 0) ++start.y;
      else if (start.y == _height - 1) --start.y;
    }

    // Open the entrance and its adjacent interior cell.
    setWall(_runnerLoc.x, _runnerLoc.y, false);
    setWall(start.x, start.y, false);
  }
  else
  {
    if (isInBounds(_runnerLoc))
      start = _runnerLoc;
    else
    {
      start = {(int)random(_width), (int)random(_height)};
      _runnerLoc = start;
    }

    setWall(start.x, start.y, false);
  }

  // Generate a spanning tree from the runner's start, using _searchPath as the carving stack.
  uint8_t *stackCells = _searchPath;
  uint16_t stackSize = 0;
  stackCells[stackSize++] = encode(start);
  Direction previous = NullLocation;
  int maxCycles = 1000;

  while (stackSize && maxCycles-- > 0)
  {
    Location cur = decode(stackCells[stackSize - 1]);
    Direction steps[4] = {Left, Right, Up, Down};
    shuffle(steps, 4);

    // Sometimes prefer continuing in the direction used to enter this cell.
    // TODO: make this a preference or random optino with each maze gen?
    if (stackSize > 1 && random(2) == 0)
    {
      Location prev = decode(stackCells[stackSize - 2]);
      Direction forward = {cur.x - prev.x, cur.y - prev.y};
      for (int i = 0; i < 4; ++i)
      {
        if (steps[i] == forward)
        {
          Direction temp = steps[0];
          steps[0] = steps[i];
          steps[i] = temp;
          break;
        }
      }
    }

    bool found = false;
    for (int i = 0; i < 4; ++i)
    {
      Location next = cur + steps[i];
      if (isCarvable(next) && isWall(next) &&
          adjacentWallCount(next) >= 3 &&
          !(_options.preventDiagonalTouching && hasDiagonalOnlyNeighbor(next)))
      {
        setWall(next.x, next.y, false);
        if (stackSize < MAZE_MAX_CELLS)
          stackCells[stackSize++] = encode(next);
        previous = steps[i];
        found = true;
        break;
      }
    }

    if (!found)
    {
      --stackSize;
      previous = NullLocation;
    }
  }

  int removed = 0;
  maxCycles = 1000;

  while (removed < _options.extraWallsToRemove && maxCycles-- > 0)
  {
    int x = random(_width);
    int y = random(_height);
    Location loc = {x, y};

    if (isCorridorWall(loc) &&
        !(_options.preventDiagonalTouching && hasDiagonalOnlyNeighbor(loc)))
    {
      setWall(x, y, false);
      ++removed;
    }
  }

  if (removed < _options.extraWallsToRemove)
    log_w("Failed to remove all extra corridor walls");

  // Pick the farthest reachable cell as the finish. This runs after the extra walls are removed:
  // their shortcuts could otherwise leave the "farthest" cell a few steps from the start. In
  // outer-wall mode, this is an inner-edge cell.
  _exitLoc = NullLocation;
  int16_t pathSize = findPath(_options.outerWalls ? GoalFarthestInnerEdge : GoalFarthest,
                              _runnerLoc, NullLocation, NullLocation, NoDepthLimit);
  if (pathSize <= 0)
  {
    log_e("Failed to find path to exit");
    if (_setStatus) _setStatus(_exitColor);
    _resetDelay = ErrorDelay;
    return;
  }

  Location finish = decode(_searchPath[pathSize - 1]);

    // In outer-wall mode, turn the selected inner-edge finish into a perimeter exit.
    if (_options.outerWalls)
    {
      _exitLoc = finish;
      if (finish.y == 1) --_exitLoc.y;
      else if (finish.y == _height - 2) ++_exitLoc.y;
      else if (finish.x == 1) --_exitLoc.x;
      else if (finish.x == _width - 2) ++_exitLoc.x;
      else
      {
        log_e("Failed to place perimeter exit");
        _exitLoc = NullLocation;
        return;
      }

      setWall(_exitLoc.x, _exitLoc.y, false);
    }
    else
    {
      _exitLoc = finish;
    }
    log_d("Maze generation complete; start (%d,%d), exit (%d,%d)",
          _runnerLoc.x, _runnerLoc.y, _exitLoc.x, _exitLoc.y);
}


inline bool MazeRunner::moveRunner()
{
  if (_runnerCooldown) { --_runnerCooldown; return false; }

  int16_t pathSize = 0;
  if (!_options.perfectVision) markSeen();

  const uint8_t sense = _options.runnerSense;
  const uint8_t lookout = sense > _options.sentrySense ? sense : _options.sentrySense;
  int16_t sentryDistance = 0;

  if (_sentryActive)
  {
    sentryDistance = findPath(GoalCell, _runnerLoc, _sentryLoc, NullLocation, lookout);
    if (sentryDistance < 0) sentryDistance = 0;
  }

  // If the sentry is visible, remember its current location and flee again.
  const bool sentrySeen = sentryDistance > 0 && sentryDistance <= sense;
  if (sentrySeen)
  {
    _runnerKnownSentryLoc = _sentryLoc;
    _runnerFleeWait = 0;
  }

  // Measure distance to the last-known sentry location.
  int16_t knownSentryDistance = 0;
  bool hasKnownSentry = _runnerKnownSentryLoc != NullLocation;

  if (hasKnownSentry)
  {
    knownSentryDistance = findPath(GoalCell, _runnerLoc, _runnerKnownSentryLoc,
                                   NullLocation, NoDepthLimit);
    if (knownSentryDistance < 0)
    {
      _runnerKnownSentryLoc = NullLocation;
      _runnerFleeWait = 0;
      hasKnownSentry = false;
    }
  }

  // Continue fleeing while the sentry is visible or the runner is still too close.
  const bool fleeing = sentrySeen ||
    (hasKnownSentry && knownSentryDistance < _options.runnerFear);

  if (fleeing && hasKnownSentry && _runnerFleeWait == 0)
  {
    // Keep the current flee destination instead of picking a new one every move.
    pathSize = isInBounds(_runnerFleeTarget)
      ? findPath(GoalCell, _runnerLoc, _runnerFleeTarget, NullLocation, NoDepthLimit) : -1;
    bool targetValid = pathSize > 0;

    if (!targetValid)
    {
      _runnerFleeTarget = NullLocation;

      uint16_t reach = (uint16_t)sense + _options.runnerFear;
      pathSize = findPath(GoalFarthest, _runnerLoc, NullLocation, _runnerKnownSentryLoc,
                          reach > 255 ? 255 : (uint8_t)reach,
                          knownSentryDistance > 0 ? (uint8_t)(knownSentryDistance - 1) : 0);
      if (pathSize > 0)
        _runnerFleeTarget = decode(_searchPath[pathSize - 1]);
    }

    // If no safe move exists, wait without resetting the timer every turn.
    if (pathSize <= 0)
    {
      _runnerKnownSentryLoc = NullLocation; // forget about sentry after wait
      _runnerFleeWait = knownSentryDistance < _options.runnerFear
        ? _options.runnerFear - knownSentryDistance : 0;
    }
  }

  // Wait one runner move cycle at a time before resuming normal planning.
  if (!sentrySeen && _runnerFleeWait > 0)
  {
    --_runnerFleeWait;
    return false;
  }

  if (!fleeing || pathSize <= 0)
  {
    if (hasKnownSentry && knownSentryDistance >= _options.runnerFear)
    {
      _runnerKnownSentryLoc = NullLocation;
      _runnerFleeTarget = NullLocation;
      _runnerFleeWait = 0;
    }

    // If the exit is nearby, head for it instead of walking past to explore.
    pathSize = findPath(GoalCell, _runnerLoc, _exitLoc, NullLocation, sense);
    if (pathSize < 0)
    {
      if (!_options.perfectVision)
        pathSize = findPath(GoalUnvisited, _runnerLoc, NullLocation, NullLocation,
                            NoDepthLimit);

      if (pathSize <= 0)
        pathSize = findPath(GoalCell, _runnerLoc, _exitLoc, NullLocation, NoDepthLimit);
    }
  }

  // Move
  if (pathSize > 0)
  {
    Location next = decode(_searchPath[0]);
    for (int16_t i = 1; i < pathSize; ++i) _searchPath[i - 1] = _searchPath[i];

    Location prev = _runnerLoc;
    _runnerLoc = next;
    if (!_options.perfectVision) markSeen();
    _runnerCooldown = _sentryActive ? _options.runnerSpeed : 0;
    log_v("Moved runner from (%d,%d) to (%d,%d)",
          prev.x, prev.y, _runnerLoc.x, _runnerLoc.y);
    return true;
  }

  return false;
}

// Decides, from how the previous maze ended, whether the runner's start carries over. Only
// runs when a new maze is generated; a catch restarts the same maze via retryRound() instead.
// This is a separate step (not part of generateMaze/placeRunner) because it must run before
// anything is regenerated: placeExit() overwrites _exitLoc, and both generateMaze() and
// placeRunner() need the answer.
//
// After this call, isInBounds(_runnerLoc) means "start where the runner finished last round,
// i.e. at the old exit". Otherwise _runnerLoc is NullLocation (outer-wall mode) and a new
// start is picked.
inline void MazeRunner::resolveStartFromLastRound()
{
  bool reachedExit = _exitLoc != NullLocation && _runnerLoc == _exitLoc;
  if (!reachedExit && _options.outerWalls)
    _runnerLoc = NullLocation;
}

// Restarts the current maze after a catch: runner and sentry go back to where they began, the
// walls and exit stay put. The runner keeps its explored-cell memory (_runnerVisited), so with
// perfectVision off it gets to learn the maze over repeated attempts.
inline void MazeRunner::retryRound()
{
  log_d("Retrying maze");
  _runnerCaught = false;
  _runnerKnownSentryLoc = NullLocation;
  _runnerFleeTarget = NullLocation;
  _runnerFleeWait = 0;
  _runnerLoc = _runnerSpawnLoc;
  _runnerCooldown = 0;
  _sentryLoc = _sentrySpawnLoc;
  _sentryCooldown = _options.sentrySpeed;
  _sentryKnownRunnerLoc = NullLocation;
  _sentryPrevLoc = NullLocation;
  drawMaze();
}

inline void MazeRunner::placeSentry()
{
  if (!_sentryActive) return;
  _sentryLoc = NullLocation;
  _sentryCooldown = _options.sentrySpeed;

  if (_options.sentryPatrol)
  {
    _sentryKnownRunnerLoc = NullLocation;
    _sentryPrevLoc = NullLocation;

    // Patrol: spawn on the cell in front of the exit. That is a choke point on the runner's
    // only route, so the escape check below is skipped; the sentry leaves it right away.
    if (isInBounds(_exitLoc))
    {
      Direction steps[4] = {Left, Right, Up, Down};
      shuffle(steps, 4);
      for (int i = 0; i < 4 && _sentryLoc == NullLocation; ++i)
      {
        Location candidate = _exitLoc + steps[i];
        if (isInBounds(candidate) && !isWall(candidate) && candidate != _runnerLoc)
          _sentryLoc = candidate;
      }
      if (_sentryLoc != NullLocation)
      {
        _sentrySpawnLoc = _sentryLoc;
        log_d("Placing sentry at (%d,%d) in front of exit", _sentryLoc.x, _sentryLoc.y);
        return;
      }
    }
    // No exit (generation failed) or no free cell beside it: fall back to a random spot.
  }

  int attempts = 0;
  while (_sentryLoc.x == -1 && attempts < 2000)
  {
    Location candidate = {(int)random(_width), (int)random(_height)};
    int distance = abs(candidate.x-_runnerLoc.x) + abs(candidate.y-_runnerLoc.y);
    int minDistance = max(0, (_width + _height) / 2 - (attempts / 10));
    if (!isWall(candidate) && candidate != _runnerLoc && candidate != _exitLoc && distance > minDistance)
    {
      // Reject posts that leave the runner no route to the exit outside sensing range.
      bool escapeExists = findPath(GoalCell, _runnerLoc, _exitLoc, candidate,
                                   NoDepthLimit, _options.runnerSense) >= 0;
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
        if (!isWall(candidate) && candidate != _runnerLoc && candidate != _exitLoc)
        {
          bool escapeExists = findPath(GoalCell, _runnerLoc, _exitLoc, candidate,
                                       NoDepthLimit, _options.runnerSense) >= 0;
          if (escapeExists) _sentryLoc = candidate;
        }
      }
  }
  _sentrySpawnLoc = _sentryLoc;
  log_d("Placing sentry at (%d,%d) after %d attempts", _sentryLoc.x, _sentryLoc.y, attempts);
}

inline bool MazeRunner::moveSentry()
{
  if (!_sentryActive) return false;
  if (_sentryCooldown) { --_sentryCooldown; return false; }

  // Behaviors:
  //  - pursue: chase the runner while it is within sensing range, then keep heading for the
  //    last place it was seen.
  //  - patrol (MazeOptions::sentryPatrol): once the last-seen spot is reached, or can't be
  //    reached, walk from junction to junction (see sentryPatrol()). Without the option the
  //    sentry walks back to its post and waits there for another sighting.
  // _searchPath is shared scratch space, so the runner replans after sentry searches.
  int16_t pathSize = 0;
  Location next = NullLocation;

  pathSize = findPath(GoalCell, _sentryLoc, _runnerLoc, NullLocation, _options.sentrySense);
  bool seesRunner = pathSize > 0;
  if (seesRunner)
  {
    _sentryKnownRunnerLoc = _runnerLoc;
    next = decode(_searchPath[0]);
  }
  else if (isInBounds(_sentryKnownRunnerLoc) && _sentryKnownRunnerLoc != _sentryLoc &&
           (pathSize = findPath(GoalCell, _sentryLoc, _sentryKnownRunnerLoc,
                                NullLocation, NoDepthLimit)) > 0)
  {
    // Recalculated one step at a time so the sentry doesn't need a second full-size path buffer.
    // TODO: sentry doesn't need a full path buffer? just one big enough for its sense path?
    next = decode(_searchPath[0]);
  }
  else
  {
    // Nothing to pursue (never saw the runner, reached the last-seen spot, or it's unreachable).
    _sentryKnownRunnerLoc = NullLocation;
    if (_options.sentryPatrol) next = sentryPatrol();
    // Parking wherever the runner was last seen can leave the only route inside sensing range,
    // so the runner would wait forever; placeSentry() chose the post so that it doesn't.
    else if (_sentryLoc != _sentrySpawnLoc &&
             (pathSize = findPath(GoalCell, _sentryLoc, _sentrySpawnLoc,
                                  NullLocation, NoDepthLimit)) > 0)
      next = decode(_searchPath[0]);
  }
  // TODO: hacky maintaining this, use light abstraction to handle shared path data?

  if (next == NullLocation) return false;

  // Without patrol, don't let the sentry park on a choke point that cuts off the exit. (Patrol
  // mode skips this: the sentry spawns on one and keeps moving, so it never stays there.)
  if (!_options.sentryPatrol)
  {
    bool escapeExists = true;
    if (next != _runnerLoc)
    {
      escapeExists = findPath(GoalCell, _runnerLoc, _exitLoc, next, NoDepthLimit) >= 0;
    }
    if (!escapeExists)
    {
      _sentryCooldown = _options.sentrySpeed;
      return false;
    }
  }

  Location prev = _sentryLoc;
  _sentryLoc = next;
  _sentryPrevLoc = prev;
  _sentryCooldown = _options.sentrySpeed;

  // Sentry lost trail of runner
  if (_sentryLoc == _sentryKnownRunnerLoc && _sentryLoc != _runnerLoc)
  {
    _sentryKnownRunnerLoc = NullLocation;
  }

  log_v("Moved sentry from (%d,%d) to (%d,%d)", prev.x, prev.y, _sentryLoc.x, _sentryLoc.y);
  return true;
}

// Patrol: the next cell for the sentry, or NullLocation if it can't move.
//  - At a junction: pick an exit (chooseJunctionPath).
//  - In a corridor with a heading: keep going forward.
//  - No heading (just spawned or lost the runner) or stuck in a dead end: go to the nearest
//    junction.
//  - No junction anywhere: wander, preferring not to backtrack.
inline Location MazeRunner::sentryPatrol()
{
  if (isJunction(_sentryLoc)) return chooseJunctionPath();

  if (isInBounds(_sentryPrevLoc))
  {
    for (int i = 0; i < 4; ++i)
    {
      Location next = _sentryLoc + Directions[i];
      if (isInBounds(next) && !isWall(next) && next != _sentryPrevLoc) return next;
    }
    // Dead end: fall through and head for the nearest junction (which is back the way we came).
  }

  int16_t pathSize = findPath(GoalJunction, _sentryLoc, NullLocation,
                              NullLocation, NoDepthLimit);
  if (pathSize > 0) return decode(_searchPath[0]);

  Direction steps[4] = {Left, Right, Up, Down};
  shuffle(steps, 4);
  Location fallback = NullLocation;
  for (int i = 0; i < 4; ++i)
  {
    Location next = _sentryLoc + steps[i];
    if (!isInBounds(next) || isWall(next)) continue;
    if (next != _sentryPrevLoc) return next;
    fallback = next;
  }
  return fallback;
}

// At a junction, picks which way to go. Each open direction is scored: avoiding a dead end
// outranks avoiding backtracking, and ties are broken randomly (steps is shuffled, strict >
// keeps the first best).
//   3: onward, leads to another junction     1: onward, dead-ends
//   2: back the way we came, leads to one    0: back the way we came, dead-ends
inline Location MazeRunner::chooseJunctionPath()
{
  Direction steps[4] = {Left, Right, Up, Down};
  shuffle(steps, 4);
  Location best = NullLocation;
  int bestScore = -1;
  for (int i = 0; i < 4; ++i)
  {
    Location next = _sentryLoc + steps[i];
    if (!isInBounds(next) || isWall(next)) continue;
    int score = (leadsToJunction(next, _sentryLoc) ? 2 : 0) + (next != _sentryPrevLoc ? 1 : 0);
    if (score > bestScore)
    {
      bestScore = score;
      best = next;
    }
  }
  return best;
}

// Breadth-first search over open cells from `start`, never entering a cell within avoidRadius
// steps of `avoid`. Returns the route length (excluding `start`), 0 when start == target, or -1
// when no path is found. With a depth limit, only cells within maxDepth steps are considered.
// _searchPath doubles as the queue and parent links take 2 bits per cell, so the stack cost is
// MAZE_BITSET_BYTES + MAZE_PARENT_BYTES.
inline int16_t MazeRunner::findPath(SearchGoal goal, Location start, Location target, Location avoid,
                                    uint8_t maxDepth, uint8_t avoidRadius)
{
  uint16_t outSize = 0;
  if (!isInBounds(start) || (goal == GoalCell && !isInBounds(target))) return -1;

  uint8_t reached[MAZE_BITSET_BYTES] = {};
  uint8_t parentDir[MAZE_PARENT_BYTES] = {}; // per cell: Directions index pointing to its parent
  uint8_t *queue = _searchPath;
  const uint8_t startId = encode(start);
  const uint8_t targetId = goal == GoalCell ? encode(target) : startId;
  uint16_t head = 0, tail = 0, levelEnd = 1;
  uint8_t depth = 0;

  // Pre-mark the avoided cells as reached so the search below never enqueues them.
  if (isInBounds(avoid)) markWithin(avoid, avoidRadius, reached);

  queue[tail++] = startId;
  reached[startId >> 3] |= (uint8_t)(1U << (startId & 7));
  uint8_t found = startId;
  bool success = false;

  while (head < tail)
  {
    if (head == levelEnd) { ++depth; levelEnd = tail; }
    uint8_t id = queue[head++];
    Location cur = decode(id);
    switch (goal)
    {
      case GoalCell: success = id == targetId; break;
      case GoalJunction: success = id != startId && isJunction(cur); break;
      case GoalUnvisited: success = id != startId && !isVisited(id); break;
      // BFS dequeues cells in nondecreasing distance order, so the last match is the farthest.
      case GoalFarthest: found = id; break;
      case GoalFarthestInnerEdge: if (isInnerEdgeCell(cur)) found = id; break;
    }
    if (success) { found = id; break; }
    if (maxDepth != NoDepthLimit && depth >= maxDepth) continue;

    uint8_t order[4] = {0, 1, 2, 3};
    shuffle(order, 4);
    for (uint8_t i = 0; i < 4; ++i)
    {
      uint8_t dir = order[i];
      Location next = cur + Directions[dir];
      if (!isInBounds(next) || isWall(next)) continue;
      uint8_t nid = encode(next);
      uint8_t mask = (uint8_t)(1U << (nid & 7));
      if (reached[nid >> 3] & mask) continue;
      reached[nid >> 3] |= mask;
      // Directions keeps opposites adjacent (Left/Right, Up/Down), so dir ^ 1 points back.
      parentDir[nid >> 2] |= (uint8_t)((dir ^ 1) << ((nid & 3) * 2));
      queue[tail++] = nid; // each cell is queued at most once, so tail <= MAZE_MAX_CELLS
    }
  }

  if (goal == GoalFarthest || goal == GoalFarthestInnerEdge) success = found != startId;
  if (!success) return -1;

  // Walk the parent links back to the start
  for (uint8_t id = found; id != startId; ++outSize)
  {
    _searchPath[outSize] = id;
    uint8_t dir = (parentDir[id >> 2] >> ((id & 3) * 2)) & 3;
    if (dir == 0) --id;
    else if (dir == 1) ++id;
    else if (dir == 2) id -= _width;
    else id += _width;
  }

  // Reverse into start-to-goal order.
  for (uint16_t i = 0; i < outSize / 2; ++i)
  {
    uint8_t temp = _searchPath[i];
    _searchPath[i] = _searchPath[outSize - 1 - i];
    _searchPath[outSize - 1 - i] = temp;
  }
  return (int16_t)outSize;
}

// Exploring mode: the runner sees up to runnerSense cells away. With seeAroundCorners that is any
// open cell within that many steps; otherwise only straight lines until a wall blocks the view,
// so a dead end off to the side still has to be walked into.
inline void MazeRunner::markSeen()
{
  if (!isInBounds(_runnerLoc)) return;
  if (_options.seeAroundCorners)
  {
    uint8_t seen[MAZE_BITSET_BYTES] = {};
    markWithin(_runnerLoc, _options.runnerSense, seen);
    for (uint16_t i = 0; i < MAZE_BITSET_BYTES; ++i) _runnerVisited[i] |= seen[i];
    return;
  }
  uint8_t cell = encode(_runnerLoc);
  _runnerVisited[cell >> 3] |= (uint8_t)(1U << (cell & 7));
  for (uint8_t dir = 0; dir < 4; ++dir)
  {
    Location loc = _runnerLoc;
    for (uint8_t step = 0; step < _options.runnerSense; ++step)
    {
      loc += Directions[dir];
      if (!isInBounds(loc) || isWall(loc)) break;
      cell = encode(loc);
      _runnerVisited[cell >> 3] |= (uint8_t)(1U << (cell & 7));
    }
  }
}

// Marks every open cell within `radius` steps of `center` in `cells`, which must start without
// any of them marked (it doubles as the search's visited set). Uses a local queue rather than
// _searchPath, so it never disturbs the runner's plan.
inline void MazeRunner::markWithin(Location center, uint8_t radius, uint8_t *cells) const
{
  if (!isInBounds(center)) return;
  if (radius > MAZE_MAX_SENSE) radius = MAZE_MAX_SENSE;
  uint8_t queue[MaxZoneCells];
  uint8_t head = 0, tail = 0, levelEnd = 1, depth = 0;
  const uint8_t centerId = encode(center);
  queue[tail++] = centerId;
  cells[centerId >> 3] |= (uint8_t)(1U << (centerId & 7));
  while (head < tail)
  {
    if (head == levelEnd) { ++depth; levelEnd = tail; }
    uint8_t id = queue[head++];
    if (depth >= radius) continue;
    Location cur = decode(id);
    for (uint8_t dir = 0; dir < 4; ++dir)
    {
      Location next = cur + Directions[dir];
      if (!isInBounds(next) || isWall(next)) continue;
      uint8_t nid = encode(next);
      uint8_t mask = (uint8_t)(1U << (nid & 7));
      if (cells[nid >> 3] & mask) continue;
      cells[nid >> 3] |= mask;
      queue[tail++] = nid;
    }
  }
}

inline bool MazeRunner::isWall(int x, int y) const
{
  uint16_t cell = (uint16_t)y * _width + x;
  return (_mazeWalls[cell >> 3] & (uint8_t)(1U << (cell & 7))) != 0;
}

inline int MazeRunner::adjacentWallCount(int x, int y) const
{
  int count = 0;
  for (int i=0; i<4; ++i)
  {
    int nx = x + Directions[i].x, ny = y + Directions[i].y;
    if (nx < 0 || nx >= _width || ny < 0 || ny >= _height || isWall(nx, ny)) ++count;
  }
  return count;
}

// True if some open diagonal neighbor touches this cell only at a corner, i.e. both cells
// between them are walls.
inline bool MazeRunner::hasDiagonalOnlyNeighbor(Location loc) const
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

// An open cell in the ring just inside the perimeter (x or y of 1 or size-2), i.e. one that sits
// directly against the outer wall. These are the cells an exit/entrance can be cut next to.
inline bool MazeRunner::isInnerEdgeCell(Location loc) const
{
  return loc.x > 0 && loc.x < _width - 1 && loc.y > 0 && loc.y < _height - 1 &&
         (loc.x == 1 || loc.x == _width - 2 || loc.y == 1 || loc.y == _height - 2) &&
         !isWall(loc);
}

inline bool MazeRunner::isCorridorWall(Location loc) const
{
  // All four neighbors must be in bounds before calling isWall().
  if (loc.x <= 0 || loc.x >= _width - 1 ||
      loc.y <= 0 || loc.y >= _height - 1 ||
      !isWall(loc))
    return false;

  bool horizontal =
      !isWall(loc + Left) && !isWall(loc + Right) &&
       isWall(loc + Up)   &&  isWall(loc + Down);

  bool vertical =
      !isWall(loc + Up)   && !isWall(loc + Down) &&
       isWall(loc + Left) &&  isWall(loc + Right);

  return horizontal || vertical;
}

// Follows the corridor that starts at `cur` (entered from `prev`) and reports whether it reaches
// a junction (true) or ends in a dead end (false). Only corridor stubs count as dead ends: a
// branch that forks further on is fine even if every fork eventually dead-ends, since the maze
// is nearly a tree and every route ends somewhere.
inline bool MazeRunner::leadsToJunction(Location cur, Location prev) const
{
  for (uint16_t steps = 0; steps < MAZE_MAX_CELLS; ++steps)
  {
    if (isJunction(cur)) return true;
    Location next = NullLocation;
    for (int i = 0; i < 4; ++i)
    {
      Location n = cur + Directions[i];
      if (isInBounds(n) && !isWall(n) && n != prev) { next = n; break; }
    }
    if (next == NullLocation) return false;
    prev = cur;
    cur = next;
  }
  return true; // a loop with no junction and no dead end
}