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
// Coordinates are stored in int8_t and cell indexes in one byte, so each side is limited to 126
// and the grid to 256 cells.
#ifndef MAZE_RUNNER_MAX_WIDTH
#define MAZE_RUNNER_MAX_WIDTH 16
#endif
#ifndef MAZE_RUNNER_MAX_HEIGHT
#define MAZE_RUNNER_MAX_HEIGHT 16
#endif

#if MAZE_RUNNER_MAX_WIDTH < 1 || MAZE_RUNNER_MAX_HEIGHT < 1 || \
    MAZE_RUNNER_MAX_WIDTH > 126 || MAZE_RUNNER_MAX_HEIGHT > 126 || \
    MAZE_RUNNER_MAX_WIDTH * MAZE_RUNNER_MAX_HEIGHT > 256
#error "MazeRunner sides must be 1..126 and the grid must contain at most 256 cells"
#endif

// Colors live in their own namespace because names like BLACK, RED and BLUE are macros in
// Adafruit_GFX, SSD1306 and others.
namespace MazeColor
{
  // Same packing as NeoPixel::Color(), which isn't constexpr: calling it here would copy every
  // color into SRAM at startup instead of leaving it in flash.
  constexpr uint32_t rgb(uint8_t r, uint8_t g, uint8_t b)
  {
    return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
  }

  constexpr uint32_t Black = rgb(0x00, 0x00, 0x00);
  constexpr uint32_t Red = rgb(0xFF, 0x00, 0x00);
  constexpr uint32_t Orange = rgb(0xCC, 0x44, 0x00);
  constexpr uint32_t Yellow = rgb(0xFF, 0xFF, 0x00);
  constexpr uint32_t YellowGreen = rgb(0xCC, 0xFF, 0x00);
  constexpr uint32_t Green = rgb(0x00, 0xFF, 0x00);
  constexpr uint32_t Blue = rgb(0x00, 0x00, 0x77);
  constexpr uint32_t Purple = rgb(0x77, 0x00, 0x77);
}

namespace MazeRunnerLib
{

// Fixed-size implementation for small AVR targets. Cell indexes are one byte.
constexpr uint16_t MAZE_MAX_CELLS = (uint16_t)MAZE_RUNNER_MAX_WIDTH * MAZE_RUNNER_MAX_HEIGHT;
constexpr uint16_t MAZE_BITSET_BYTES = (MAZE_MAX_CELLS + 7) / 8;
// Search parent links: 2 bits per cell (an index into Directions).
constexpr uint16_t MAZE_PARENT_BYTES = (MAZE_MAX_CELLS + 3) / 4;

// Bitset helpers (not named bitSet/bitRead/bitClear: those are Arduino macros).
inline bool bsGet(const uint8_t *bits, uint8_t i) { return (bits[i >> 3] & (uint8_t)(1U << (i & 7))) != 0; }
inline void bsSet(uint8_t *bits, uint8_t i) { bits[i >> 3] |= (uint8_t)(1U << (i & 7)); }
inline void bsClear(uint8_t *bits, uint8_t i) { bits[i >> 3] &= (uint8_t)~(1U << (i & 7)); }

// Every field has a default, so set only what you change:
//   MazeOptions o; o.outerWalls = false; o.sentryPatrol = true;
//   MazeRunner runner(drawPixel, setStatus, o);
// Delays and speeds count update() calls (frames).
struct MazeOptions
{
  // Display colors (0xRRGGBB).
  uint32_t pathColor = MazeColor::Black;
  uint32_t wallColor = MazeColor::Orange;
  uint32_t runnerColor = MazeColor::Yellow;
  uint32_t sentryColor = MazeColor::Red;
  uint32_t exitColor = MazeColor::Purple;
  uint32_t visitedColor = MazeColor::Black; // used only when fullVision is false

  // --- Maze ---
  uint8_t width = MAZE_RUNNER_MAX_WIDTH;
  uint8_t height = MAZE_RUNNER_MAX_HEIGHT;
  // Keep the display perimeter as walls, with an entrance and exit opening.
  bool outerWalls = true;
  // Avoid diagonal-only contact between otherwise separate corridors.
  bool noDiagonallyAdjacentPaths = true;
  // Corridor walls knocked out after carving.
  uint8_t extraWallsToRemove = 2;

  // --- Runner ---
  // If false, the runner explores and marks cells instead of navigating directly to the exit.
  bool fullVision = false;
  // Not used yet: markSeen() always marks both the around-corner area and the straight lines.
  bool seeAroundCorners = false; // TODO: better name, like "useLineOfSight" or "lineOfSightOnly"
  // Around-corner sensing range, measured in open-cell steps.
  uint8_t runnerSense = 2;
  // Straight-line visibility range. Walls block sight; independent of runnerSense.
  uint8_t runnerLineOfSight = 6;
  // Frames between runner moves while a sentry is active; without one it moves every frame.
  uint8_t runnerSpeed = 3;
  // How far past runnerSense a flee may aim.
  uint8_t runnerFear = 15;

  // --- Sentry ---
  bool sentryEnabled = true;
  // Sentry guards the exit: it spawns beside it and patrols from junction to junction, pursuing
  // the runner when it is sensed and returning to patrol after losing it. If false, the sentry
  // guards a random post that leaves the runner a way around it, and returns there after a chase.
  bool sentryPatrol = true;
  // Around-corner sensing range, measured in open-cell steps. One more than runnerSense lets
  // the sentry notice the runner first, so some rounds end in a catch and a retry (~15% of mazes
  // in simulation, vs ~7% when equal).
  uint8_t sentrySense = 3;
  // Frames between sentry moves.
  uint8_t sentrySpeed = 5;
  // Straight-line visibility range. Walls block sight; independent of sentrySense.
  uint8_t sentryLineOfSight = 8;

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

// Coordinates are int8_t (see the size limits at the top). The constructor takes ints so that
// brace initialization like {x, y} doesn't trip narrowing errors.
struct Coordinate
{
  int8_t x;
  int8_t y;
  constexpr Coordinate() : x(0), y(0) {}
  constexpr Coordinate(int px, int py) : x((int8_t)px), y((int8_t)py) {}
};

inline Coordinate operator-(const Coordinate &coord) { return {-coord.x, -coord.y}; }
inline Coordinate operator+(const Coordinate &a, const Coordinate &b) { return {a.x + b.x, a.y + b.y}; }
inline Coordinate operator-(const Coordinate &a, const Coordinate &b) { return {a.x - b.x, a.y - b.y}; }
inline Coordinate operator*(const Coordinate &a, const Coordinate &b) { return {a.x * b.x, a.y * b.y}; }
inline Coordinate operator*(int a, const Coordinate &b) { return {b.x * a, b.y * a}; }
inline bool operator==(const Coordinate &a, const Coordinate &b) { return a.x == b.x && a.y == b.y; }
inline bool operator!=(const Coordinate &a, const Coordinate &b) { return !(a == b); }
inline Coordinate &operator+=(Coordinate &a, const Coordinate &b) { a.x += b.x; a.y += b.y; return a; }
inline Coordinate &operator-=(Coordinate &a, const Coordinate &b) { a.x -= b.x; a.y -= b.y; return a; }
inline Coordinate &operator*=(Coordinate &a, const Coordinate &b) { a.x *= b.x; a.y *= b.y; return a; }

using Location = Coordinate;
using Direction = Coordinate;

const Location NullLocation = {-1, -1};
// The order matters: index d ^ 1 is the opposite of index d (Left/Right, Up/Down), and
// MazeRunner::stepId() switches on these indexes.
const Direction Left = {-1, 0};
const Direction Right = {1, 0};
const Direction Up = {0, -1};
const Direction Down = {0, 1};
const Direction Directions[] = {Left, Right, Up, Down};

// Fisher-Yates shuffle.
template <typename T>
inline void shuffle(T *list, int size)
{
  for (int i = size - 1; i > 0; --i)
  {
    const int j = (int)random(i + 1);
    T temp = list[i];
    list[i] = list[j];
    list[j] = temp;
  }
}

class MazeRunner
{
public:
  typedef void (*DrawPixelCallback)(int, int, uint32_t);
  typedef void (*StatusCallback)(uint32_t);

  MazeRunner(DrawPixelCallback drawPixel,
             StatusCallback setStatus = nullptr,
             const MazeOptions &options = MazeOptions())
    : _drawPixel(drawPixel), _setStatus(setStatus), _options(options)
  {
    if (_options.width < 1 || _options.height < 1 || _options.width > MAZE_RUNNER_MAX_WIDTH ||
        _options.height > MAZE_RUNNER_MAX_HEIGHT || (uint16_t)_options.width * _options.height > MAZE_MAX_CELLS ||
        (_options.outerWalls && (_options.width < 3 || _options.height < 3)))
    {
      _options.width = _options.height = 0; // invalid: init() and update() do nothing
    }
    // A sense of 0 would mean "no depth limit" to findPath().
    if (_options.runnerSense < 1) _options.runnerSense = 1;
    if (_options.sentrySense < 1) _options.sentrySense = 1;
  }

  void init();
  bool update();
private:
  static const int ErrorDelay = 30000;
  static const uint8_t NoDepthLimit = 0;

  // configuration
  DrawPixelCallback _drawPixel;
  StatusCallback _setStatus;
  MazeOptions _options;

  // maze and round state
  uint8_t _mazeWalls[MAZE_BITSET_BYTES];
  Location _exitLoc = NullLocation;
  int _resetDelay = -1;
  uint16_t _roundFrames = 0;

  // Scratch queue shared by findPath(), markWithin() and generateMaze(); it is clobbered by every
  // call, so never keep anything in it. findPath() reports its result in _pathFirst/_pathLast.
  uint8_t _searchPath[MAZE_MAX_CELLS];
  uint8_t _pathFirst = 0; // first step of the last route found
  uint8_t _pathLast = 0;  // last cell of the last route found

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
  Location _sentryLoc = NullLocation;
  Location _sentryKnownRunnerLoc = NullLocation;
  Location _sentryPrevLoc = NullLocation;
  Location _sentrySpawnLoc = NullLocation;
  uint8_t _sentryCooldown = 0;

  // maze functions
  void drawMaze();
  void generateMaze();

  // runner and round functions
  bool moveRunner();
  void markSeen();
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
  // Returns the path length, 0 for start == target, or -1 if no path was found. On success
  // _pathFirst and _pathLast hold the first step and the last cell of the route.
  int16_t findPath(SearchGoal goal, Location start, Location target, Location avoid,
                   uint8_t maxDepth, uint8_t avoidRadius = 0);
  bool hasLineOfSight(Location from, Location to, uint8_t range) const;
  // Clobbers _searchPath.
  void markWithin(Location center, uint8_t radius, uint8_t *cells);

  // maze helpers
  bool isWall(int x, int y) const;
  int adjacentWallCount(int x, int y) const;

  bool hasDiagonalOnlyNeighbor(Location loc) const;
  bool leadsToJunction(Location from, Location prev) const;
  bool isInnerEdgeCell(Location loc) const;
  bool isCorridorWall(Location loc) const;

  // inlines
  uint8_t encode(Location loc) const { return (uint8_t)(loc.y * _options.width + loc.x); }
  Location decode(uint8_t cell) const { return {(int)(cell % _options.width), (int)(cell / _options.width)}; }
  bool isWallId(uint8_t id) const { return bsGet(_mazeWalls, id); }
  bool isWall(Location loc) const { return isWall(loc.x, loc.y); }
  bool isInBounds(int x, int y) const { return x >= 0 && x < _options.width && y >= 0 && y < _options.height; }
  bool isInBounds(Location loc) const { return isInBounds(loc.x, loc.y); }
  bool isJunction(Location loc) const { return !isWall(loc) && adjacentPassageCount(loc) >= 3; }
  bool isVisited(uint8_t cell) const { return bsGet(_runnerVisited, cell); }
  int adjacentWallCount(Location loc) const { return adjacentWallCount(loc.x, loc.y); }
  int adjacentPassageCount(Location loc) const { return 4 - adjacentWallCount(loc); }

  // Id of the neighbor of `id` (at column cx, row cy) in direction `dir` (an index into
  // Directions), or -1 when that steps off the grid. Works on ids so searches need no Location
  // math and no divisions per neighbor.
  int16_t stepId(uint8_t id, uint8_t cx, uint8_t cy, uint8_t dir) const
  {
    switch (dir)
    {
      case 0: return cx > 0 ? id - 1 : -1;
      case 1: return cx + 1 < _options.width ? id + 1 : -1;
      case 2: return cy > 0 ? id - _options.width : -1;
      default: return cy + 1 < _options.height ? id + _options.width : -1;
    }
  }
};

inline void MazeRunner::init()
{
  if (_options.width == 0 || _options.height == 0) return;
  log_v("Initializing maze");

  resolveStartFromLastRound();

  memset(_runnerVisited, 0, sizeof(_runnerVisited));
  _resetDelay = -1; // generateMaze() may set it again on failure
  _runnerKnownSentryLoc = NullLocation;
  _runnerFleeTarget = NullLocation;
  _runnerCaught = false;
  _runnerFleeWait = 0;
  _runnerCooldown = 0;
  _roundFrames = 0;
  generateMaze();
  _runnerSpawnLoc = _runnerLoc;
  placeSentry();
  markSeen();

  drawMaze();
}

inline bool MazeRunner::update()
{
  if (_options.width == 0 || _options.height == 0) return false;

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
    if (_setStatus) _setStatus(_options.runnerColor);
    drawMaze();
    _resetDelay = _options.sentryEnabled ? _options.goalDelay : 0;
    return true;
  }

  if (_options.sentryEnabled)
  {
    changed |= moveSentry();
    if (_sentryLoc == _runnerLoc)
    {
      log_d("Runner caught by sentry");
      if (_setStatus) _setStatus(_options.sentryColor);
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

  // One pass over the grid; ids run 0..width*height-1 in row-major order.
  uint8_t id = 0;
  for (int y = 0; y < _options.height; ++y)
    for (int x = 0; x < _options.width; ++x, ++id)
    {
      uint32_t color = _options.pathColor;
      if (isWallId(id)) color = _options.wallColor;
      else if (!_options.fullVision && isVisited(id)) color = _options.visitedColor;
      _drawPixel(x, y, color);
    }

  // These can legitimately be NullLocation (-1,-1): the exit when placeExit() failed, the
  // sentry when no valid spot was found, the runner before the first placement. Skip those
  // rather than call drawPixel with a negative coordinate.
  if (isInBounds(_runnerSpawnLoc)) _drawPixel(_runnerSpawnLoc.x, _runnerSpawnLoc.y, _options.pathColor);
  if (isInBounds(_exitLoc)) _drawPixel(_exitLoc.x, _exitLoc.y, _options.exitColor);
  if (isInBounds(_runnerLoc)) _drawPixel(_runnerLoc.x, _runnerLoc.y, _options.runnerColor);
  if (_options.sentryEnabled && isInBounds(_sentryLoc)) _drawPixel(_sentryLoc.x, _sentryLoc.y, _options.sentryColor);
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
    return loc.x > 0 && loc.x < _options.width - 1 &&
           loc.y > 0 && loc.y < _options.height - 1;
  };
  auto setWall = [this](int x, int y, bool wall) -> void
  {
    const uint8_t cell = (uint8_t)(y * _options.width + x);
    if (wall) bsSet(_mazeWalls, cell);
    else bsClear(_mazeWalls, cell);
  };

  memset(_mazeWalls, 0xFF, sizeof(_mazeWalls)); // everything starts as wall

  Location start;

  if (_options.outerWalls)
  {
    if (!isInBounds(_runnerLoc))
    {
      // Choose the entrance before generating the maze, so the maze is
      // generated from the runner's actual starting location.
      int side = random(4);
      int x, y;

      if (side < 2)
      {
        x = 1 + random(_options.width - 2);
        y = side == 0 ? 0 : _options.height - 1;
      }
      else
      {
        x = side == 2 ? 0 : _options.width - 1;
        y = 1 + random(_options.height - 2);
      }

      _runnerLoc = {x, y};
    }

    // Otherwise the previous exit becomes this round's entrance. Either way, start the
    // carving on the interior cell next to the entrance.
    start = _runnerLoc;
    if (start.x == 0) ++start.x;
    else if (start.x == _options.width - 1) --start.x;
    else if (start.y == 0) ++start.y;
    else if (start.y == _options.height - 1) --start.y;

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
      start = {(int)random(_options.width), (int)random(_options.height)};
      _runnerLoc = start;
    }

    setWall(start.x, start.y, false);
  }

  // Generate a spanning tree from the runner's start, using _searchPath as the carving stack.
  uint8_t *stackCells = _searchPath;
  uint16_t stackSize = 0;
  stackCells[stackSize++] = encode(start);
  int maxCycles = 1000;
  uint8_t straightPreference = 2 + random(3); // the lower the more likely to prefer a straight carve

  while (stackSize && maxCycles-- > 0)
  {
    Location cur = decode(stackCells[stackSize - 1]);
    Direction steps[4] = {Left, Right, Up, Down};
    shuffle(steps, 4);

    // Sometimes prefer continuing in the direction used to enter this cell.
    if (stackSize > 1 && random(straightPreference) == 0)
    {
      Location prev = decode(stackCells[stackSize - 2]);
      Direction forward = cur - prev;
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
          !(_options.noDiagonallyAdjacentPaths && hasDiagonalOnlyNeighbor(next)))
      {
        setWall(next.x, next.y, false);
        if (stackSize < MAZE_MAX_CELLS)
          stackCells[stackSize++] = encode(next);
        found = true;
        break;
      }
    }

    if (!found) --stackSize;
  }

  int removed = 0;
  maxCycles = 1000;

  while (removed < _options.extraWallsToRemove && maxCycles-- > 0)
  {
    int x = random(_options.width);
    int y = random(_options.height);
    Location loc = {x, y};

    if (isCorridorWall(loc) &&
        !(_options.noDiagonallyAdjacentPaths && hasDiagonalOnlyNeighbor(loc)))
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
    if (_setStatus) _setStatus(_options.exitColor);
    _resetDelay = ErrorDelay;
    return;
  }

  Location finish = decode(_pathLast);

  // In outer-wall mode, turn the selected inner-edge finish into a perimeter exit.
  if (_options.outerWalls)
  {
    _exitLoc = finish;
    if (finish.y == 1) --_exitLoc.y;
    else if (finish.y == _options.height - 2) ++_exitLoc.y;
    else if (finish.x == 1) --_exitLoc.x;
    else if (finish.x == _options.width - 2) ++_exitLoc.x;
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

  // No markSeen() here: the runner hasn't moved since the last one (init(), the post-move call
  // below, or retryRound() back to an already-marked spawn).
  int16_t pathSize = 0;
  const uint8_t sense = _options.runnerSense;

  // Either unobstructed cardinal line of sight (cheap, so checked first) or around-corner
  // sensing can detect the sentry.
  const bool sentrySeen = _options.sentryEnabled &&
    (hasLineOfSight(_runnerLoc, _sentryLoc, _options.runnerLineOfSight) ||
     findPath(GoalCell, _runnerLoc, _sentryLoc, NullLocation, sense) > 0);
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

    // Clear sentry tracking if the last-known location is no longer reachable
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
    // Never path closer to the sentry than the runner already is.
    const uint8_t avoidRadius = knownSentryDistance > 0 ? (uint8_t)(knownSentryDistance - 1) : 0;

    // Keep the current flee destination instead of picking a new one every move.
    pathSize = isInBounds(_runnerFleeTarget)
      ? findPath(GoalCell, _runnerLoc, _runnerFleeTarget, _runnerKnownSentryLoc,
                 NoDepthLimit, avoidRadius)
      : -1;

    if (pathSize <= 0)
    {
      _runnerFleeTarget = NullLocation;

      uint16_t reach = (uint16_t)sense + _options.runnerFear;
      pathSize = findPath(GoalFarthest, _runnerLoc, NullLocation, _runnerKnownSentryLoc,
                          reach > 255 ? 255 : (uint8_t)reach, avoidRadius);
      if (pathSize > 0)
        _runnerFleeTarget = decode(_pathLast);
    }

    // If no safe move exists, wait without resetting the timer every turn.
    if (pathSize <= 0)
    {
      // Always wait at least one runner turn; keep the counter within its uint8_t range.
      uint16_t wait = knownSentryDistance < _options.runnerFear
        ? (uint16_t)(_options.runnerFear - knownSentryDistance) : 1;
      _runnerFleeWait = wait > 255 ? 255 : (uint8_t)wait;
    }
  }

  // Wait one runner move cycle at a time before resuming normal planning.
  if (!sentrySeen && _runnerFleeWait > 0)
  {
    --_runnerFleeWait;

    // forget about sentry after waiting during a flee
    if (_runnerFleeWait == 0)
    {
      _runnerKnownSentryLoc = NullLocation;
    }

    return false;
  }

  if (!fleeing || pathSize <= 0)
  {
    // forget about sentry if not fleeing or successfully fleed and out of sight
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
      if (!_options.fullVision)
        pathSize = findPath(GoalUnvisited, _runnerLoc, NullLocation, NullLocation,
                            NoDepthLimit);

      if (pathSize <= 0)
        pathSize = findPath(GoalCell, _runnerLoc, _exitLoc, NullLocation, NoDepthLimit);
    }
  }

  // Move
  if (pathSize > 0)
  {
    Location prev = _runnerLoc;
    _runnerLoc = decode(_pathFirst);
    if (!_options.fullVision) markSeen();
    _runnerCooldown = _options.sentryEnabled ? _options.runnerSpeed : 0;
    log_v("Moved runner from (%d,%d) to (%d,%d)",
          prev.x, prev.y, _runnerLoc.x, _runnerLoc.y);
    return true;
  }

  return false;
}

// Exploring mode: mark both the around-corner sensing area and straight-line visible cells.
// Clobbers _searchPath (via markWithin).
inline void MazeRunner::markSeen()
{
  if (!isInBounds(_runnerLoc))
    return;

  uint8_t seen[MAZE_BITSET_BYTES] = {};

  markWithin(_runnerLoc, _options.runnerSense, seen); // also marks the runner's own cell

  for (uint8_t dir = 0; dir < 4; ++dir)
  {
    Location loc = _runnerLoc;
    for (uint8_t step = 0; step < _options.runnerLineOfSight; ++step)
    {
      loc += Directions[dir];
      if (!isInBounds(loc) || isWall(loc)) break;
      bsSet(seen, encode(loc));
    }
  }
  for (uint16_t i = 0; i < MAZE_BITSET_BYTES; ++i) _runnerVisited[i] |= seen[i];
}

// Decides, from how the previous maze ended, whether the runner's start carries over. Only
// runs when a new maze is generated; a catch restarts the same maze via retryRound() instead.
// This is a separate step (not part of generateMaze/placeRunner) because it must run before
// anything is regenerated: generateMaze() overwrites _exitLoc, and it needs the answer.
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
// fullVision off it gets to learn the maze over repeated attempts.
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
  if (!_options.sentryEnabled) return;
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
  while (_sentryLoc == NullLocation && attempts < 2000)
  {
    Location candidate = {(int)random(_options.width), (int)random(_options.height)};
    int distance = abs(candidate.x - _runnerLoc.x) + abs(candidate.y - _runnerLoc.y);
    int minDistance = max(0, (_options.width + _options.height) / 2 - (attempts / 10));
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
    for (int y = 0; y < _options.height && _sentryLoc == NullLocation; ++y)
      for (int x = 0; x < _options.width && _sentryLoc == NullLocation; ++x)
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
  if (!_options.sentryEnabled) return false;
  if (_sentryCooldown) { --_sentryCooldown; return false; }

  // Behaviors:
  //  - pursue: chase the runner while it is within sensing range, then keep heading for the
  //    last place it was seen.
  //  - patrol (MazeOptions::sentryPatrol): once the last-seen spot is reached, or can't be
  //    reached, walk from junction to junction (see sentryPatrol()). Without the option the
  //    sentry walks back to its post and waits there for another sighting.
  // _searchPath is shared scratch space, but nothing is kept in it between calls: the runner
  // replans every move.
  int16_t pathSize = 0;
  Location next = NullLocation;

  // Cardinal line of sight is cheap and can be farther than the around-corner search depth, so
  // test it first; the BFS then only needs to run unlimited (LOS) or depth-limited (sensing).
  bool seesRunner = hasLineOfSight(_sentryLoc, _runnerLoc, _options.sentryLineOfSight);
  if (seesRunner)
  {
    pathSize = findPath(GoalCell, _sentryLoc, _runnerLoc, NullLocation, NoDepthLimit);
  }
  else
  {
    pathSize = findPath(GoalCell, _sentryLoc, _runnerLoc, NullLocation, _options.sentrySense);
    seesRunner = pathSize > 0;
  }

  if (seesRunner)
  {
    _sentryKnownRunnerLoc = _runnerLoc;
    if (pathSize > 0) next = decode(_pathFirst);
  }
  else if (isInBounds(_sentryKnownRunnerLoc) && _sentryKnownRunnerLoc != _sentryLoc &&
           (pathSize = findPath(GoalCell, _sentryLoc, _sentryKnownRunnerLoc,
                                NullLocation, NoDepthLimit)) > 0)
  {
    // Recalculated one step at a time so the sentry doesn't need to remember a route.
    next = decode(_pathFirst);
  }
  else
  {
    // Sentry lost trail of runner
    _sentryKnownRunnerLoc = NullLocation;

    if (_options.sentryPatrol) next = sentryPatrol();
    // Parking wherever the runner was last seen can leave the only route inside sensing range,
    // so the runner would wait forever; placeSentry() chose the post so that it doesn't.
    else if (_sentryLoc != _sentrySpawnLoc &&
             (pathSize = findPath(GoalCell, _sentryLoc, _sentrySpawnLoc,
                                  NullLocation, NoDepthLimit)) > 0)
      next = decode(_pathFirst);
  }

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
  if (pathSize > 0) return decode(_pathFirst);

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
// The route itself isn't stored: callers only ever need its first step (_pathFirst) and its last
// cell (_pathLast). _searchPath doubles as the queue and parent links take 2 bits per cell, so
// the stack cost is MAZE_BITSET_BYTES + MAZE_PARENT_BYTES.
// Ties between equal-length routes are broken by a random rotation of the direction order,
// chosen once per search.
inline int16_t MazeRunner::findPath(SearchGoal goal, Location start, Location target, Location avoid,
                                    uint8_t maxDepth, uint8_t avoidRadius)
{
  if (!isInBounds(start) || (goal == GoalCell && !isInBounds(target))) return -1;

  // A route can't be shorter than the Manhattan distance, so a depth-limited search for a target
  // farther away than that can't succeed. Skip the BFS.
  if (goal == GoalCell && maxDepth != NoDepthLimit &&
      abs(start.x - target.x) + abs(start.y - target.y) > maxDepth)
    return -1;

  uint8_t reached[MAZE_BITSET_BYTES] = {};
  uint8_t parentDir[MAZE_PARENT_BYTES] = {}; // per cell: Directions index pointing to its parent
  uint8_t *queue = _searchPath;
  const uint8_t startId = encode(start);
  const uint8_t targetId = goal == GoalCell ? encode(target) : startId;
  const uint8_t rot = (uint8_t)random(4);
  uint16_t head = 0, tail = 0, levelEnd = 1;
  uint8_t depth = 0;

  // Pre-mark the avoided cells as reached so the search below never enqueues them. (This uses
  // _searchPath as its queue, which is fine: the search proper hasn't written to it yet.)
  if (isInBounds(avoid)) markWithin(avoid, avoidRadius, reached);

  queue[tail++] = startId;
  bsSet(reached, startId);
  uint8_t found = startId;
  bool success = false;

  while (head < tail)
  {
    if (head == levelEnd) { ++depth; levelEnd = tail; }
    const uint8_t id = queue[head++];
    const uint8_t cx = id % _options.width;
    const uint8_t cy = id / _options.width;
    switch (goal)
    {
      case GoalCell: success = id == targetId; break;
      case GoalJunction: success = id != startId && isJunction(Location(cx, cy)); break;
      case GoalUnvisited: success = id != startId && !isVisited(id); break;
      // BFS dequeues cells in nondecreasing distance order, so the last match is the farthest.
      case GoalFarthest: found = id; break;
      case GoalFarthestInnerEdge: if (isInnerEdgeCell(Location(cx, cy))) found = id; break;
    }
    if (success) { found = id; break; }
    if (maxDepth != NoDepthLimit && depth >= maxDepth) continue;

    for (uint8_t i = 0; i < 4; ++i)
    {
      const uint8_t dir = (i + rot) & 3;
      const int16_t n = stepId(id, cx, cy, dir);
      if (n < 0 || isWallId((uint8_t)n)) continue;
      const uint8_t nid = (uint8_t)n;
      if (bsGet(reached, nid)) continue;
      bsSet(reached, nid);
      // Directions keeps opposites adjacent (Left/Right, Up/Down), so dir ^ 1 points back.
      parentDir[nid >> 2] |= (uint8_t)((dir ^ 1) << ((nid & 3) * 2));
      queue[tail++] = nid; // each cell is queued at most once, so tail <= MAZE_MAX_CELLS
    }
  }

  if (goal == GoalFarthest || goal == GoalFarthestInnerEdge) success = found != startId;
  if (!success) return -1;

  // Walk the parent links back to the start. That visits the route in reverse, so the last cell
  // visited before the start is the first step.
  uint8_t length = 0;
  uint8_t firstStep = found;
  for (uint8_t id = found; id != startId; ++length)
  {
    firstStep = id;
    const uint8_t dir = (parentDir[id >> 2] >> ((id & 3) * 2)) & 3;
    if (dir == 0) --id;
    else if (dir == 1) ++id;
    else if (dir == 2) id -= _options.width;
    else id += _options.width;
  }
  _pathFirst = firstStep;
  _pathLast = found;
  return length;
}

// Cardinal line of sight only. The target must be within range and every intervening cell open.
inline bool MazeRunner::hasLineOfSight(Location from, Location to, uint8_t range) const
{
  if (!isInBounds(from) || !isInBounds(to) || from == to) return false;
  int distance = abs(from.x - to.x) + abs(from.y - to.y);
  if (distance > range || (from.x != to.x && from.y != to.y)) return false;

  Direction step = from.x == to.x
    ? (to.y > from.y ? Down : Up)
    : (to.x > from.x ? Right : Left);
  Location loc = from + step;
  while (loc != to)
  {
    if (isWall(loc)) return false;
    loc += step;
  }
  return !isWall(to);
}


// Marks every open cell within `radius` steps of `center` in `cells`, which must start without
// any of them marked (it doubles as the search's visited set). Uses _searchPath as its queue, so
// callers must not keep anything there.
inline void MazeRunner::markWithin(Location center, uint8_t radius, uint8_t *cells)
{
  if (!isInBounds(center)) return;
  uint8_t *queue = _searchPath;
  uint16_t head = 0, tail = 0, levelEnd = 1;
  uint8_t depth = 0;
  const uint8_t centerId = encode(center);
  queue[tail++] = centerId;
  bsSet(cells, centerId);
  while (head < tail)
  {
    if (head == levelEnd) { ++depth; levelEnd = tail; }
    if (depth >= radius) break; // BFS order: everything still queued is at least this deep
    const uint8_t id = queue[head++];
    const uint8_t cx = id % _options.width;
    const uint8_t cy = id / _options.width;
    for (uint8_t dir = 0; dir < 4; ++dir)
    {
      const int16_t n = stepId(id, cx, cy, dir);
      if (n < 0 || isWallId((uint8_t)n)) continue;
      const uint8_t nid = (uint8_t)n;
      if (bsGet(cells, nid)) continue;
      bsSet(cells, nid);
      queue[tail++] = nid;
    }
  }
}

inline bool MazeRunner::isWall(int x, int y) const
{
  return isWallId((uint8_t)(y * _options.width + x));
}

inline int MazeRunner::adjacentWallCount(int x, int y) const
{
  int count = 0;
  for (int i = 0; i < 4; ++i)
  {
    int nx = x + Directions[i].x, ny = y + Directions[i].y;
    if (nx < 0 || nx >= _options.width || ny < 0 || ny >= _options.height || isWall(nx, ny)) ++count;
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
  return loc.x > 0 && loc.x < _options.width - 1 && loc.y > 0 && loc.y < _options.height - 1 &&
         (loc.x == 1 || loc.x == _options.width - 2 || loc.y == 1 || loc.y == _options.height - 2) &&
         !isWall(loc);
}

inline bool MazeRunner::isCorridorWall(Location loc) const
{
  // All four neighbors must be in bounds before calling isWall().
  if (loc.x <= 0 || loc.x >= _options.width - 1 ||
      loc.y <= 0 || loc.y >= _options.height - 1 ||
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

} // namespace MazeRunnerLib

using MazeRunnerLib::MazeOptions;
using MazeRunnerLib::MazeRunner;
