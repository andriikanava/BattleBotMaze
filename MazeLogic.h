#ifndef MAZE_LOGIC_H
#define MAZE_LOGIC_H

#include <Arduino.h>

void waitForTurn();
void initMaze();

void escapeFromLeftWall();
void reacquireLeftWall();
void alignToLeftWall();
void safeStepForward();
void followLeftWallStep();

#endif
