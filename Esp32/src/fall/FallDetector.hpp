#ifndef FALLDETECTOR_HPP
#define FALLDETECTOR_HPP

#include <Arduino.h>

void fallDetector_init();
void fallDetector_update();
bool fallDetector_isRecovering();

#endif
