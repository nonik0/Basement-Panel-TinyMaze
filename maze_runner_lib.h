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

struct MazeOptions
{
  // Keep the display perimeter as walls, with an entrance and exit opening.
  bool outerWalls;
  // Avoid diagonal-only contact between otherwise separate corridors.
  bool preventDiagonalTouching;
  // If false, the runner explores and marks cells instead of navigating directly to the exit.
  bool perfectVision;
  // Color used to mark explored cells when perfectVision is false.
  uint32_t visitedColor;
  // Sentry guards the exit: it spawns beside it and patrols from junction to junction, pursuing
  // the runner when it is sensed and returning to patrol after losing it. If false, the sentry
  // spawns at a random spot and stands still until it senses the runner.
  bool sentryPatrol;

  constexpr MazeOptions(bool outer = false, bool diagonal = false,
              bool vision = true, uint32_t visited = 0x000020UL,
              bool patrol = false)
    : outerWalls(outer), preventDiagonalTouching(diagonal),
      perfectVision(vision), visitedColor(visited), sentryPatrol(patrol) {}
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
  static const uint8_t RunnerFear = 10;
  static const uint8_t RunnerSense = 2;
  static const uint8_t RunnerSpeed = 3;
  static const uint8_t SentrySense = 2;
  static const uint8_t SentrySpeed = 5;
  static const int GoalDelay = 10;
  static const int CatchDelay = 30;
  static const int ErrorDelay = 30000;
  // Frames after which an unfinished round is abandoned for a new maze. Runner and sentry can
  // reach standoffs that never resolve (a stationary sentry beside the only route, or a chase
  // around a loop where the faster runner is never caught but never gets past).
  static const uint16_t RoundTimeout = 3000;

  int _width;
  int _height;
  uint8_t _mazeWalls[MAZE_BITSET_BYTES];
  int _resetDelay = -1;
  uint16_t _roundFrames = 0;
  Location _exitLoc = NullLocation;
  int _mazeExtraWallsToRemove = 1;

  uint32_t _pathColor, _wallColor, _runnerColor, _sentryColor, _exitColor;
  MazeOptions _options;

  // runner state
  Location _runnerLoc = NullLocation;
  Location _runnerSentryKnownLoc = NullLocation;
  Location _runnerSpawnLoc = NullLocation;
  bool _runnerCaught = false;
  uint8_t _runnerCooldown = 0;
  // The runner's planned route, and also the one cell-sized scratch buffer: findPath() uses it
  // as its queue and generateMaze() as its carving stack. Any search therefore replaces the plan.
  uint8_t _runnerPath[MAZE_MAX_CELLS];
  uint16_t _runnerPathSize = 0;
  uint8_t _runnerVisited[MAZE_BITSET_BYTES] = {};
  
  // sentry state
  bool _sentryActive = false;
  Location _sentryLoc = NullLocation;
  Location _sentryKnownRunnerLoc = NullLocation;
  Location _sentryPrevLoc = NullLocation;
  Location _sentrySpawnLoc = NullLocation;
  uint8_t _sentryCooldown = 0;

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
  void resolveStartFromLastRound();
  void retryRound();
  bool moveRunner();
  bool moveSentry();
  Location sentryPatrol();
  Location chooseJunctionPath();
  bool leadsToJunction(Location from, Location prev) const;
  int openNeighborCount(Location loc) const;
  bool isJunction(Location loc) const { return !isWall(loc) && openNeighborCount(loc) >= 3; }
  // What findPath() searches for.
  enum SearchGoal : uint8_t
  {
    GoalCell,              // `target`
    GoalJunction,          // nearest junction other than the start
    GoalUnvisited,         // nearest cell the runner has not explored
    GoalFarthest,          // last cell reached (farthest within the depth limit)
    GoalFarthestInnerEdge, // farthest open cell against the outer wall
  };
  static const uint8_t NoDepthLimit = 0;
  bool findPath(SearchGoal goal, Location start, Location target, Location avoid,
                uint8_t maxDepth, uint16_t &outSize);
  void drawMaze();
  void generateMaze();
  //void placeRunner();
  void placeSentry();
  //void placeExit();
  bool isAdjacent(Location a, Location b) { return isInBounds(a) && isInBounds(b) && abs(a.x-b.x)+abs(a.y-b.y)==1; }
  bool isWall(int x, int y) const;
  bool isWall(Location loc) const;
  bool isInBounds(int x, int y) const;
  bool isInBounds(Location loc) const;
  int getAdjacentWallAndBorderCount(int x, int y) const;
  int getAdjacentWallAndBorderCount(Location loc) const;
  bool hasDiagonalOnlyNeighbor(Location loc) const;
  bool isVisited(uint8_t cell) const { return (_runnerVisited[cell >> 3] & (uint8_t)(1U << (cell & 7))) != 0; }
  void markVisited(Location loc) { if (isInBounds(loc)) { uint8_t cell = encode(loc); _runnerVisited[cell >> 3] |= (uint8_t)(1U << (cell & 7)); } }
  bool isInnerEdgeCell(Location loc) const;
  bool isCorridorWall(Location loc) const;
};

inline void MazeRunner::init()
{
  if (_width == 0 || _height == 0) return;
  log_v("Initializing maze");

  resolveStartFromLastRound();

  for (uint16_t i = 0; i < MAZE_BITSET_BYTES; ++i) _runnerVisited[i] = 0;
  // TODO hate how these are set manually here
  _runnerCaught = false;
  _roundFrames = 0;
  generateMaze();
  _runnerSpawnLoc = _runnerLoc;
  placeSentry();
  markVisited(_runnerLoc);

  drawMaze();
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
  _roundFrames = 0;
  _runnerLoc = _runnerSpawnLoc;
  clearRunnerPath();
  _runnerSentryKnownLoc = NullLocation;
  _runnerCooldown = 0;
  _sentryLoc = _sentrySpawnLoc;
  _sentryCooldown = SentrySpeed;
  _sentryKnownRunnerLoc = NullLocation;
  _sentryPrevLoc = NullLocation;
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

  if (++_roundFrames > RoundTimeout)
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
      _runnerCaught = true;
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

  if (!_options.perfectVision) markVisited(_runnerLoc);

  bool sensed = false;
  if (_sentryActive)
  {
    uint16_t senseSize = 0;
    sensed = findPath(GoalCell, _runnerLoc, _sentryLoc, NullLocation, RunnerSense, senseSize) &&
             senseSize > 0;
    // That search used _runnerPath as its queue, so the previous plan is gone either way.
    _runnerPathSize = 0;
  }

  // sense and flee if sentry is near
  if (sensed)
  {
    _runnerSentryKnownLoc = _sentryLoc;
    findPath(GoalFarthest, _runnerLoc, NullLocation, _sentryLoc, RunnerSense + RunnerFear,
             _runnerPathSize);
    if (_runnerPathSize > RunnerSense) _runnerPathSize = RunnerSense;
  }
  // plan if able
  else if (_runnerPathSize == 0)
  {
    // If the exit is nearby, head for it instead of walking past to explore.
    if (!findPath(GoalCell, _runnerLoc, _exitLoc, _runnerSentryKnownLoc, 2, _runnerPathSize))
    {
      if (!_options.perfectVision)
        findPath(GoalUnvisited, _runnerLoc, NullLocation, _sentryLoc, NoDepthLimit,
                 _runnerPathSize);
      if (_runnerPathSize == 0)
        findPath(GoalCell, _runnerLoc, _exitLoc, _runnerSentryKnownLoc, NoDepthLimit,
                 _runnerPathSize);
    }
    _runnerSentryKnownLoc = NullLocation;
  }

  // move
  Location next;
  if (popFrontPath(_runnerPath, _runnerPathSize, next))
  {
    Location prev = _runnerLoc;
    _runnerLoc = next;
    if (!_options.perfectVision) markVisited(_runnerLoc);
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

  // Behaviors:
  //  - pursue: chase the runner while it is within sensing range, then keep heading for the
  //    last place it was seen.
  //  - patrol (MazeOptions::sentryPatrol): once the last-seen spot is reached, or can't be
  //    reached, walk from junction to junction (see sentryPatrol()). Without the option the
  //    sentry stops there and waits for another sighting.
  // _runnerPath doubles as scratch space for the sentry's searches, so the runner's plan is
  // discarded (_runnerPathSize = 0) and it replans on its next turn.
  uint16_t pathSize = 0;
  Location next = NullLocation;

  bool seesRunner = findPath(GoalCell, _sentryLoc, _runnerLoc, NullLocation,
                             SentrySense, pathSize) && pathSize > 0;
  if (seesRunner)
  {
    _sentryKnownRunnerLoc = _runnerLoc;
    next = decode(_runnerPath[0]);
  }
  else if (isInBounds(_sentryKnownRunnerLoc) && _sentryKnownRunnerLoc != _sentryLoc &&
           findPath(GoalCell, _sentryLoc, _sentryKnownRunnerLoc, NullLocation,
                    NoDepthLimit, pathSize) && pathSize > 0)
  {
    // Recalculated one step at a time so the sentry doesn't need a second full-size path buffer.
    next = decode(_runnerPath[0]);
  }
  else
  {
    // Nothing to pursue (never saw the runner, reached the last-seen spot, or it's unreachable).
    _sentryKnownRunnerLoc = NullLocation;
    if (_options.sentryPatrol) next = sentryPatrol();
  }

  // The sentry's searches used the runner's buffer. The runner must replan next turn.
  _runnerPathSize = 0;

  if (next == NullLocation) return false;

  // Without patrol, don't let the sentry park on a choke point that cuts off the exit. (Patrol
  // mode skips this: the sentry spawns on one and keeps moving, so it never stays there.)
  if (!_options.sentryPatrol)
  {
    bool escapeExists = true;
    if (next != _runnerLoc)
    {
      uint16_t escapeSize = 0;
      escapeExists = findPath(GoalCell, _runnerLoc, _exitLoc, next, NoDepthLimit, escapeSize);
    }
    if (!escapeExists)
    {
      _sentryCooldown = SentrySpeed;
      return false;
    }
  }

  Location prev = _sentryLoc;
  _sentryLoc = next;
  _sentryPrevLoc = prev;
  _sentryCooldown = SentrySpeed;

  // If the sentry reached the last-seen location but the runner is gone, it has lost the trail:
  // forget the heading too, so patrol starts by going to the nearest junction.
  if (_sentryLoc == _sentryKnownRunnerLoc && _sentryLoc != _runnerLoc)
  {
    _sentryKnownRunnerLoc = NullLocation;
    _sentryPrevLoc = NullLocation;
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

  uint16_t pathSize = 0;
  if (findPath(GoalJunction, _sentryLoc, NullLocation, NullLocation, NoDepthLimit, pathSize))
    return decode(_runnerPath[0]);

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

// Open (in-bounds, non-wall) orthogonal neighbors. A junction has 3 or more.
inline int MazeRunner::openNeighborCount(Location loc) const
{
  return 4 - getAdjacentWallAndBorderCount(loc);
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
  for (int y = 0; y < _height; ++y)
    for (int x = 0; x < _width; ++x) setWall(x, y, true);

  // Can the generator dig this cell open? With outerWalls, the perimeter is
  // reserved for the entrance and exit.
  auto isCarvable = [this](Location loc) -> bool
  {
    if (!isInBounds(loc)) return false;
    if (!_options.outerWalls) return true;
    return loc.x > 0 && loc.x < _width - 1 &&
           loc.y > 0 && loc.y < _height - 1;
  };

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

      _runnerLoc = start;
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

  // Generate a spanning tree from the runner's start, using _runnerPath as the carving stack.
  uint8_t *stackCells = _runnerPath;
  uint16_t stackSize = 0;
  stackCells[stackSize++] = encode(start);
  int maxCycles = 1000;

  while (stackSize && maxCycles-- > 0)
  {
    Location cur = decode(stackCells[stackSize - 1]);
    Direction steps[4] = {Left, Right, Up, Down};
    shuffle(steps, 4);

    bool found = false;
    for (int i = 0; i < 4; ++i)
    {
      Location next = cur + steps[i];
      if (isCarvable(next) && isWall(next) &&
          getAdjacentWallAndBorderCount(next) >= 3 &&
          !(_options.preventDiagonalTouching && hasDiagonalOnlyNeighbor(next)))
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

  // Pick the farthest reachable cell as the finish, preserving the existing
  // farthest-exit behavior. In outer-wall mode, this is an inner-edge cell.
  _exitLoc = NullLocation;
  uint16_t pathSize = 0;
  if (!findPath(_options.outerWalls ? GoalFarthestInnerEdge : GoalFarthest, _runnerLoc,
                NullLocation, NullLocation, NoDepthLimit, pathSize) ||
      pathSize == 0)
  {
    log_e("Failed to find path to exit");
    if (_setStatus) _setStatus(_exitColor);
    _resetDelay = ErrorDelay;
    return;
  }

  Location finish = decode(_runnerPath[pathSize - 1]);

  int removed = 0;
  maxCycles = 1000;

  while (removed < _mazeExtraWallsToRemove && maxCycles-- > 0)
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

  if (removed < _mazeExtraWallsToRemove)
    log_w("Failed to remove all extra corridor walls");

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

    clearRunnerPath();
    log_d("Maze generation complete; start (%d,%d), exit (%d,%d)",
          _runnerLoc.x, _runnerLoc.y, _exitLoc.x, _exitLoc.y);
}


inline void MazeRunner::placeSentry()
{
  if (!_sentryActive) return;
  _sentryLoc = NullLocation;
  _sentryCooldown = SentrySpeed;

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
      // Reject positions that would cut off every runner route to the exit.
      uint16_t escapeSize = 0;
      bool escapeExists = findPath(GoalCell, _runnerLoc, _exitLoc, candidate, NoDepthLimit, escapeSize);
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
          bool escapeExists = findPath(GoalCell, _runnerLoc, _exitLoc, candidate, NoDepthLimit, escapeSize);
          _runnerPathSize = 0;
          if (escapeExists) _sentryLoc = candidate;
        }
      }
  }
  _sentrySpawnLoc = _sentryLoc;
  log_d("Placing sentry at (%d,%d) after %d attempts", _sentryLoc.x, _sentryLoc.y, attempts);
}

// Breadth-first search over open cells from `start`, never entering `avoid`. On success, writes
// the route (excluding `start`) to _runnerPath and its length to outSize; GoalCell succeeds with
// an empty route when start == target. With a depth limit, only cells within maxDepth steps are
// considered. _runnerPath doubles as the queue and parent links take 2 bits per cell, so the
// stack cost is MAZE_BITSET_BYTES + MAZE_PARENT_BYTES.
inline bool MazeRunner::findPath(SearchGoal goal, Location start, Location target, Location avoid,
                                 uint8_t maxDepth, uint16_t &outSize)
{
  outSize = 0;
  if (!isInBounds(start) || (goal == GoalCell && !isInBounds(target))) return false;

  uint8_t reached[MAZE_BITSET_BYTES] = {};
  uint8_t parentDir[MAZE_PARENT_BYTES] = {}; // per cell: Directions index pointing to its parent
  uint8_t *queue = _runnerPath;
  const uint8_t startId = encode(start);
  const uint8_t targetId = goal == GoalCell ? encode(target) : startId;
  uint16_t head = 0, tail = 0, levelEnd = 1;
  uint8_t depth = 0;
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
      if (!isInBounds(next) || isWall(next) || next == avoid) continue;
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
  if (!success) return false;

  // Walk the parent links back to the start, then reverse into start-to-goal order.
  for (uint8_t id = found; id != startId; ++outSize)
  {
    _runnerPath[outSize] = id;
    uint8_t dir = (parentDir[id >> 2] >> ((id & 3) * 2)) & 3;
    if (dir == 0) --id;
    else if (dir == 1) ++id;
    else if (dir == 2) id -= _width;
    else id += _width;
  }
  for (uint16_t i = 0; i < outSize / 2; ++i)
  {
    uint8_t temp = _runnerPath[i];
    _runnerPath[i] = _runnerPath[outSize - 1 - i];
    _runnerPath[outSize - 1 - i] = temp;
  }
  return true;
}

inline bool MazeRunner::isWall(int x, int y) const { return wallAt(x, y); }
inline bool MazeRunner::isWall(Location loc) const { return isWall(loc.x, loc.y); }
inline bool MazeRunner::isInBounds(int x, int y) const { return x >= 0 && x < _width && y >= 0 && y < _height; }
inline bool MazeRunner::isInBounds(Location loc) const { return isInBounds(loc.x, loc.y); }

inline int MazeRunner::getAdjacentWallAndBorderCount(int x, int y) const
{
  int count = 0;
  for (int i=0; i<4; ++i)
  {
    int nx = x + Directions[i].x, ny = y + Directions[i].y;
    if (nx < 0 || nx >= _width || ny < 0 || ny >= _height || isWall(nx, ny)) ++count;
  }
  return count;
}
inline int MazeRunner::getAdjacentWallAndBorderCount(Location loc) const
{
  return getAdjacentWallAndBorderCount(loc.x, loc.y);
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