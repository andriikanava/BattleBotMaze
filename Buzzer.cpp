#include <Arduino.h>
#include "buzzer.h"

// простая генерация звука
void playTone(int pin, int frequency, int duration) {
    int period = 1000000 / frequency;
    int halfPeriod = period / 2;

    long cycles = (long)frequency * duration / 1000;

    for (long i = 0; i < cycles; i++) {
        digitalWrite(pin, HIGH);
        delayMicroseconds(halfPeriod);
        digitalWrite(pin, LOW);
        delayMicroseconds(halfPeriod);
    }
}

// победная мелодия
void playVictoryMelody(int pin) {
    int melody[] = {2000, 2000, 2000, 3000, 2500, 2500, 2500, 3500};
    int durations[] = {150, 150, 300, 600, 150, 150, 300, 800};

    int size = sizeof(melody) / sizeof(melody[0]);

    for (int i = 0; i < size; i++) {
        playTone(pin, melody[i], durations[i]);
        delay(30);
    }

    digitalWrite(pin, LOW);
}