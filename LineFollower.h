#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H

#include <Arduino.h>

// Инициализация пинов датчиков линии
void lineFollowerInit();

// Вернуть ошибку позиции линии
// normal value = ошибка относительно центра
// 9999 = линия потеряна
int readLinePosition();
bool isLineDetected();

int countActiveLineSensors();

void lineFollowStep();

void followLineForMs(unsigned long durationMs);

void followLineUntilLost();

void followLineUntilIntersection(int minActiveSensors = 6);

void searchLine();

void printLineSensors();

bool waitForLine(unsigned long timeoutMs = 2000);

#endif
