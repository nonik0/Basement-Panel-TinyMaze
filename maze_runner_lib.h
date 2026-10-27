#pragma once

#include <Arduino.h>
#include <queue>
#include <stack>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

using namespace std;

//
// Coordinate struct
//
struct Coordinate
{
  int x;
  int y;
};

// negation
Coordinate operator-(const Coordinate &coord)
{
  return {-coord.x, -coord.y};
}

Coordinate operator+(const Coordinate &lhs, const Coordinate &rhs)
{
  return {lhs.x + rhs.x, lhs.y + rhs.y};
}

Coordinate operator-(const Coordinate &lhs, const Coordinate &rhs)
{
  return {lhs.x - rhs.x, lhs.y - rhs.y};
}

Coordinate operator*(const Coordinate &lhs, const Coordinate &rhs)
{
  return {lhs.x * rhs.x, lhs.y * rhs.y};
}

Coordinate operator*(const int &lhs, const Coordinate &rhs)
{
  return {rhs.x * lhs, rhs.y * lhs};
}

bool operator==(const Coordinate &lhs, const Coordinate &rhs)
{
  return lhs.x == rhs.x && lhs.y == rhs.y;
}

bool operator!=(const Coordinate &lhs, const Coordinate &rhs)
{
  return !(lhs == rhs);
}

Coordinate operator+=(Coordinate &lhs, const Coordinate &rhs)
{
  lhs.x += rhs.x;
  lhs.y += rhs.y;
  return lhs;
}

Coordinate operator-=(Coordinate &lhs, const Coordinate &rhs)
{
  lhs.x -= rhs.x;
  lhs.y -= rhs.y;
  return lhs;
}

Coordinate operator*=(Coordinate &lhs, const Coordinate &rhs)
{
  lhs.x *= rhs.x;
  lhs.y *= rhs.y;
  return lhs;
}

namespace std
{
  template <>
  struct hash<Coordinate>
  {
    size_t operator()(const Coordinate &loc) const
    {
      return hash<int>()(loc.x) ^ hash<int>()(loc.y);
    }
  };
}

using Location = Coordinate;
using Direction = Coordinate; // technically should enforce to length 1 vectors

const Location NullLocation = Location{-1, -1};

const Direction Left = {-1, 0};
const Direction Right = {1, 0};
const Direction Up = {0, -1};
const Direction Down = {0, 1};
const Direction Directions[] = {Left, Right, Up, Down};

void shuffle(Direction *list, int size)
{
  for (int i = 0; i < size; i++)
  {
    int index = random(size);
    Direction temp = list[i];
    list[i] = list[index];
    list[index] = temp;
  }
}

bool upOrDown(Direction dir)
{
  return dir == Up || dir == Down;
}

bool leftOrRight(Direction dir)
{
  return dir == Left || dir == Right;
}
//
// End coordinate struct
//

class MazeRunner
{
private:
  // valid only with sentry present (for now)
  const uint8_t RunnerFear = 10; // extra sense when fleeing
  const uint8_t RunnerSense = 2; // sense is path search distance
  const uint8_t RunnerSpeed = 3; // speed is cooldown in cycles (call to update)
  const uint8_t SentrySense = 2;
  const uint8_t SentrySpeed = 5;
  const int GoalDelay = 10; // delay cycles before generating new maze
  const int CatchDelay = 30;
  const int ErrorDelay = 65536;

  int _width;
  int _height;
  bool **_mazeWalls;
  int _resetDelay = -1;
  Location _exitLoc = NullLocation;

  // TODO: improve by enforcing two paths to goal from runner pos when generating maze wit sentry active?
  int _mazeExtraWallsToRemove = 1; // extra walls to remove when sentry present to give runner more options potentially

  // colors
  uint32_t _pathColor;
  uint32_t _wallColor;
  uint32_t _runnerColor;
  uint32_t _sentryColor;
  uint32_t _exitColor;

  // runner
  Location _runnerLoc = NullLocation;
  deque<Location> _runnerPath;
  Location _runnerSentryKnownLoc = NullLocation;
  uint8_t _runnerCooldown = 0;

  // sentry
  bool _sentryActive;
  Location _sentryLoc = NullLocation;
  deque<Location> _sentryPath;
  // Location _sentryExitKnownLoc = NullLocation;
  uint8_t _sentryCooldown = 0;

  // function callbacks to draw pixels
  std::function<void(int, int, uint32_t)> _drawPixel;
  std::function<void(uint32_t)> _setStatus;

public:
  // with sentry
  MazeRunner(
      int width, int height,
      uint32_t pathColor, uint32_t wallColor,
      uint32_t runnerColor, uint32_t sentryColor,
      uint32_t exitColor,
      std::function<void(int, int, uint32_t)> drawPixel,
      std::function<void(uint32_t)> setStatus = nullptr);
  // without sentry
  MazeRunner(
      int width, int height,
      uint32_t pathColor, uint32_t wallColor,
      uint32_t runnerColor,
      uint32_t exitColor,
      std::function<void(int, int, uint32_t)> drawPixel,
      std::function<void(uint32_t)> setStatus = nullptr)
      : MazeRunner(width, height, pathColor, wallColor, runnerColor, pathColor, exitColor, drawPixel, setStatus) {};

  void init();
  bool update(); // returns true if any pixel changed to signal for display update

private:
  bool moveRunner();
  bool moveSentry();
  void drawMaze();

  void generateMaze();
  void placeRunner();
  void placeSentry();
  void placeExit();

  deque<Location> findPathDfs(Location startLoc, Location endLoc, int maxSearchDistance = -1) { return findPathDfs(startLoc, NullLocation, endLoc, maxSearchDistance); }
  deque<Location> findPathDfs(Location startLoc, Location sentryLoc, Location encLoc, int maxSearchDistance = -1);
  deque<Location> findLongestPathBfs(Location startLoc, Location sentryLoc = NullLocation, int maxSearchDistance = -1);

  bool isAdjacent(Location a, Location b) { return isInBounds(a) && isInBounds(b) && abs(a.x - b.x) + abs(a.y - b.y) == 1; }
  bool isWall(int x, int y);
  bool isWall(Location loc);
  bool isInBounds(int x, int y);
  bool isInBounds(Location loc);
  int getAdjacentWallAndBorderCount(int x, int y);
  int getAdjacentWallAndBorderCount(Location loc);
};

MazeRunner::MazeRunner(int width, int height, uint32_t pathColor, uint32_t wallColor, uint32_t runnerColor, uint32_t sentryColor,
                       uint32_t exitColor, std::function<void(int, int, uint32_t)> drawPixel, std::function<void(uint32_t)> setStatus)
{
  _width = width;
  _height = height;
  _pathColor = pathColor;
  _wallColor = wallColor;
  _runnerColor = runnerColor;
  _sentryActive = sentryColor != pathColor;
  _sentryColor = sentryColor;
  _exitColor = exitColor;
  _drawPixel = drawPixel;
  _setStatus = setStatus;

  _mazeWalls = new bool *[_height];
  for (int i = 0; i < _height; i++)
  {
    _mazeWalls[i] = new bool[_width];
  }
}

void MazeRunner::init()
{
  ("Initializing maze");

  generateMaze();
  placeRunner();
  placeSentry();
  placeExit();

  log_v("*--------*");
  for (int y = 0; y < _height; y++)
  {
    String row = "|";
    for (int x = 0; x < _width; x++)
    {
      char c = isWall(x, y) ? '#' : ' ';
      c = (_runnerLoc.x == x && _runnerLoc.y == y) ? 'S' : c;
      c = (_sentryLoc.x == x && _sentryLoc.y == y) ? 'X' : c;
      c = (_exitLoc.x == x && _exitLoc.y == y) ? 'E' : c;
      row += c;
    }
    row += "|";
    log_v("%s", row);
  }
  log_v("*--------*");
}

bool MazeRunner::update()
{
  // pause before reset to show goal, catch, or error
  if (_resetDelay >= 0)
  {
    _resetDelay--;
    if (_resetDelay < 0)
    {
      init();
      drawMaze();
      return true;
    }
    return false;
  }

  bool update = false;

  try
  {
    update |= moveRunner();
    if (_runnerLoc == _exitLoc)
    {
      log_d("Runner reached exit");
      if (_setStatus)
        _setStatus(_runnerColor);
      drawMaze();                                  // redraw runner on goal
      _resetDelay = _sentryActive ? GoalDelay : 0; // no delay if no sentry
      return true;
    }

    if (_sentryActive)
    {
      update |= moveSentry();
      if (_sentryLoc == _runnerLoc)
      {
        log_d("Runner caught by sentry");
        // don't redraw sentry on runner
        _setStatus(_sentryColor);
        _resetDelay = CatchDelay;
        return true;
      }
    }

    if (update)
    {
      drawMaze();
    }

    return update;
  }
  catch (const std::exception &e)
  {
    log_e("Error in maze runner update: %s", e.what());
    _setStatus(_exitColor);
    _resetDelay = ErrorDelay;
    return false;
  }
}

bool MazeRunner::moveRunner()
{
  if (_runnerCooldown > 0)
  {
    _runnerCooldown--;
    return false;
  }

  // sense and flee if sentry is near
  deque<Location> sensedPathToSentry = _sentryActive ? findPathDfs(_runnerLoc, _sentryLoc, RunnerSense) : deque<Location>();
  if (sensedPathToSentry.size() > 0)
  {
    _runnerSentryKnownLoc = _sentryLoc;
    _runnerPath = findLongestPathBfs(_runnerLoc, _sentryLoc, RunnerSense + RunnerFear);
    while (_runnerPath.size() > RunnerSense)
    {
      _runnerPath.pop_back();
    }
  }
  // plan if able
  else if (_runnerPath.size() == 0)
  {
    _runnerPath = findPathDfs(_runnerLoc, _runnerSentryKnownLoc, _exitLoc);
    _runnerSentryKnownLoc = NullLocation;
    _runnerPath = findPathDfs(_runnerLoc, _runnerSentryKnownLoc, _exitLoc);
  }

  // move
  if (_runnerPath.size() > 0)
  {
    Location prevRunnerLoc = _runnerLoc;
    _runnerLoc = _runnerPath.front();
    _runnerPath.pop_front();
    _runnerCooldown = _sentryActive ? RunnerSpeed : 0; // no need to delay with no sentry
    log_v("Moved runner from (%d,%d) to (%d,%d)", prevRunnerLoc.x, prevRunnerLoc.y, _runnerLoc.x, _runnerLoc.y);
    return true;
  }

  return false;
}

bool MazeRunner::moveSentry()
{
  if (!_sentryActive)
  {
    log_w("Sentry is inactive");
    return false;
  }

  if (_sentryCooldown > 0)
  {
    _sentryCooldown--;
    return false;
  }

  // sense runner
  deque<Location> sensedPathToRunner = findPathDfs(_sentryLoc, _runnerLoc, SentrySense);
  if (sensedPathToRunner.size() > 0)
  {
    _sentryPath = sensedPathToRunner;

    // new detection, small "warm up" cooldown before moving
    if (_sentryPath.size() == 0) // path implies runner was seen recently
    {
      log_v("Sentry sensed runner at (%d,%d)", _runnerLoc.x, _runnerLoc.y);
      _sentryCooldown = SentrySpeed / 2;
      return false;
    }
  }

  // // sense exit if not known
  // if (_sentryExitKnownLoc == NoLocation})
  // {
  //   deque<Location> sensedPathToExit = findPathDfs(_sentryLoc, _exitLoc, SentrySense);
  //   if (sensedPathToExit.size() > 0)
  //   {
  //     _sentryExitKnownLoc = _exitLoc;
  //     _sentryPath = sensedPathToExit; // TODO: sentry redirects?
  //   }
  // }

  // // return to exit if known and not chasing
  // if (_sentryPath.size() == 0 && _sentryExitKnownLoc != NoLocation})
  // {
  //   _sentryPath = findPathDfs(_sentryLoc, _sentryExitKnownLoc);
  // }

  // move
  if (_sentryPath.size() > 0)
  {
    Location prevSentryLoc = _sentryLoc;
    _sentryLoc = _sentryPath.front();
    _sentryPath.pop_front();
    _sentryCooldown = SentrySpeed;
    log_v("Moved sentry from (%d,%d) to (%d,%d)", prevSentryLoc.x, prevSentryLoc.y, _sentryLoc.x, _sentryLoc.y);
    return true;
  }

  return false;
}

void MazeRunner::drawMaze()
{
  for (int y = 0; y < _height; y++)
  {
    for (int x = 0; x < _width; x++)
    {
      _drawPixel(x, y, _mazeWalls[y][x] ? _wallColor : _pathColor);
    }
  }

  _drawPixel(_exitLoc.x, _exitLoc.y, _exitColor);
  _drawPixel(_runnerLoc.x, _runnerLoc.y, _runnerColor);
  _drawPixel(_sentryLoc.x, _sentryLoc.y, _sentryColor);
}

void MazeRunner::generateMaze()
{
  log_d("Starting maze generation");

  // fill maze with walls (true)
  for (int y = 0; y < _height; y++)
  {
    for (int x = 0; x < _width; x++)
    {
      _mazeWalls[y][x] = true;
    }
  }

  // always use runner location if set
  Location startLoc = (_runnerLoc == NullLocation)
                          ? Location{random(_width), random(_height)}
                          : _runnerLoc;
  _mazeWalls[startLoc.y][startLoc.x] = false;

  // create traversal stack with starting point
  stack<Location> path = stack<Location>();
  path.push(startLoc);
  int maxCycles = 1000;

  while (!path.empty() && maxCycles-- > 0)
  {
    Location curLoc = path.top();

    // shuffle directions randomly
    Direction stepDirections[] = {Left, Right, Up, Down};
    shuffle(stepDirections, 4);

    // try to move in each direction, pop location if no path forward
    bool foundPath = false;
    for (int i = 0; i < 4; i++)
    {
      Location nextLoc = curLoc + stepDirections[i];
      if (isInBounds(nextLoc) && isWall(nextLoc) && getAdjacentWallAndBorderCount(nextLoc) >= 3)
      {
        _mazeWalls[nextLoc.y][nextLoc.x] = false;
        path.push(nextLoc);
        foundPath = true;
        break;
      }
    }

    if (!foundPath)
    {
      path.pop();
    }
  }

  int wallsRemoved = 0;
  maxCycles = 1000;
  while (wallsRemoved < _mazeExtraWallsToRemove && maxCycles-- > 0)
  {
    int x = random(_width);
    int y = random(_height);
    if (isWall(x, y) && getAdjacentWallAndBorderCount(x, y) >= 2)
    {
      _mazeWalls[y][x] = false;
      wallsRemoved++;
    }
  }

  if (maxCycles <= 0)
  {
    log_e("Failed to remove extra walls");
  }

  log_d("Maze generation complete");
}

void MazeRunner::placeRunner()
{
  _runnerPath = {};
  _runnerSentryKnownLoc = NullLocation;
  _runnerCooldown = 0;

  if (_runnerLoc.x > 0 && _runnerLoc.y > 0)
  {
    if (_runnerLoc == _exitLoc)
    {
      log_d("Runner stays at prev exit loc at (%d,%d)", _runnerLoc.x, _runnerLoc.y);
      return;
    }
    else if (_sentryActive && _runnerLoc == _sentryLoc)
    {
      log_d("Runner stays at prev sentry loc at (%d,%d)", _runnerLoc.x, _runnerLoc.y);
      return;
    }
    else
    {
      log_e("Runner unexpectedly not at exit or sentry loc: (%d,%d)", _runnerLoc.x, _runnerLoc.y);
    }
  }

  int attempts = 0;
  while (_runnerLoc.x < 0 || _runnerLoc.y < 0)
  {
    int x = random(_width);
    int y = random(_height);
    if (!isWall(x, y))
    {
      _runnerLoc = {x, y};
    }
    attempts++;
  }
  log_d("Placing runner at (%d,%d) after %d attempts", _runnerLoc.x, _runnerLoc.y, attempts);
}

void MazeRunner::placeSentry()
{
  if (!_sentryActive)
  {
    log_w("Sentry is inactive");
    return;
  }

  _sentryLoc = NullLocation;
  _sentryPath = {};
  _sentryCooldown = SentrySpeed;

  int attempts = 0;
  while (_sentryLoc.x == -1)
  {
    int x = random(_width);
    int y = random(_height);
    int distance = abs(x - _runnerLoc.x) + abs(y - _runnerLoc.y);
    int minDistance = max(0, (_width + _height) / 2 - (attempts / 10));
    if (!isWall(x, y) && distance > minDistance)
    {
      _sentryLoc = {x, y};
    }
    attempts++;
  }
  log_d("Placing sentry at (%d,%d) after %d attempts", _sentryLoc.x, _sentryLoc.y, attempts);
}

void MazeRunner::placeExit()
{
  _exitLoc = NullLocation;

  deque<Location> path = findLongestPathBfs(_runnerLoc);
  if (path.size() == 0)
  {
    log_e("Failed to find path to exit");
    _setStatus(_exitColor);
    _resetDelay = ErrorDelay;
    return;
  }

  _exitLoc = path.back();

  log_d("Placing exit at (%d,%d) with distance %d from runner", _exitLoc.x, _exitLoc.y, path.size());
}

deque<Location> MazeRunner::findPathDfs(Location startLoc, Location sentryLoc, Location endLoc, int maxSearchDistance)
{
  stack<pair<Location, int>> locsToVisit = stack<pair<Location, int>>();
  unordered_set<Location> locsVisited = unordered_set<Location>();
  stack<Location> curPath = stack<Location>();

  locsToVisit.push({startLoc, 0});

  while (!locsToVisit.empty())
  {
    pair<Location, int> curLocAndDist = locsToVisit.top();
    Location curLoc = curLocAndDist.first;
    int distFromStart = curLocAndDist.second;
    locsToVisit.pop();

    // if curPath size is greater than distance than we need to unwind path to current distance
    while (curPath.size() > distFromStart)
    {
      curPath.pop();
    }
    curPath.push(curLoc);

    // found end, return reverse path stack and return
    if (curLoc == endLoc)
    {
      log_v("Found path from (%d,%d) to (%d,%d)", startLoc.x, startLoc.y, curLoc.x, curLoc.y);

      std::deque<Location> path;
      while (!curPath.empty())
      {
        path.push_front(curPath.top());
        curPath.pop();
      }

      // remove start location from path
      path.pop_front();

      return path;
    }

    locsVisited.insert(curLoc);

    // don't visit locations further than maxDistToEnd
    if (maxSearchDistance > 0 && (distFromStart + 1) > maxSearchDistance)
    {
      continue;
    }

    // look in different directions randomly in case of loops for variety of potential paths
    Direction stepDirections[4] = {Left, Right, Up, Down};
    shuffle(stepDirections, 4);

    for (Direction stepDirection : stepDirections)
    {
      Location nextLoc = curLoc + stepDirection;
      if (isInBounds(nextLoc) && !isWall(nextLoc) && nextLoc != sentryLoc && !isAdjacent(nextLoc, sentryLoc) && !locsVisited.count(nextLoc))
      {
        locsToVisit.push({nextLoc, distFromStart + 1});
      }
    }
  }

  return deque<Location>();
}

deque<Location> MazeRunner::findLongestPathBfs(Location startLoc, Location sentryLoc, int maxSearchDistance)
{
  queue<pair<Location, int>> locsToVisit = queue<pair<Location, int>>();
  unordered_set<Location> locsVisited = unordered_set<Location>();
  unordered_map<Location, Location> visitedFrom = unordered_map<Location, Location>();
  pair<Location, int> farthestLocAndDist = {startLoc, 0};

  locsToVisit.push({startLoc, 0});
  visitedFrom[startLoc] = startLoc; // special case start location, visited from itself

  while (!locsToVisit.empty())
  {
    pair<Location, int> curLocAndDist = locsToVisit.front();
    Location curLoc = curLocAndDist.first;
    int distFromStart = curLocAndDist.second;
    locsToVisit.pop();

    locsVisited.insert(curLoc);

    // don't visit locations further than maxSearchDistance
    if (maxSearchDistance > 0 && (distFromStart + 1) > maxSearchDistance)
    {
      continue;
    }

    // look in different directions randomly in case of loops for variety of potential paths
    Direction stepDirection[4] = {Left, Right, Up, Down};
    shuffle(stepDirection, 4);

    for (Direction stepDirection : stepDirection)
    {
      Location nextLoc = curLoc + stepDirection;
      if (isInBounds(nextLoc) && !isWall(nextLoc) && nextLoc != sentryLoc && !isAdjacent(nextLoc, sentryLoc) && !locsVisited.count(nextLoc))
      {
        pair<Location, int> nextLocAndDist = {nextLoc, distFromStart + 1};
        locsToVisit.push(nextLocAndDist);
        visitedFrom[nextLoc] = curLoc;

        if (nextLocAndDist.second > farthestLocAndDist.second)
        {
          farthestLocAndDist = nextLocAndDist;
        }
      }
    }
  }

  // nowhere to go
  if (farthestLocAndDist.second == 0)
  {
    return deque<Location>();
  }

  // build path to farthest location
  deque<Location> path = deque<Location>();
  Location curLoc = farthestLocAndDist.first;
  while (curLoc != startLoc)
  {
    path.push_front(curLoc);
    curLoc = visitedFrom[curLoc];
  }

  return path;
}

bool MazeRunner::isWall(int x, int y)
{
  return _mazeWalls[y][x];
}

bool MazeRunner::isWall(Location loc)
{
  return isWall(loc.x, loc.y);
}

bool MazeRunner::isInBounds(int x, int y)
{
  return x >= 0 && x < _width && y >= 0 && y < _height;
}

bool MazeRunner::isInBounds(Location loc)
{
  return isInBounds(loc.x, loc.y);
}

int MazeRunner::getAdjacentWallAndBorderCount(int x, int y)
{
  int count = 0;
  for (int i = 0; i < 4; i++)
  {
    int nx = x + Directions[i].x;
    int ny = y + Directions[i].y;
    if (nx < 0 || nx >= _width || ny < 0 || ny >= _height || _mazeWalls[ny][nx])
    {
      count++;
    }
  }
  return count;
}

int MazeRunner::getAdjacentWallAndBorderCount(Location loc)
{
  return getAdjacentWallAndBorderCount(loc.x, loc.y);
}
