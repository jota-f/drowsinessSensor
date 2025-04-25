#ifndef SYSTEM_UTILS_H
#define SYSTEM_UTILS_H

#include <Arduino.h>

// System constants
const int WATCHDOG_TIMEOUT = 15000; // 15 seconds
const int STATUS_PRINT_INTERVAL = 30000; // 30 seconds
const int FPS_AVERAGING_WINDOW = 10;
const int MAX_FAILURE_COUNT = 5;
const int MAX_DEBUG_FRAMES = 5;

// Function prototypes
void printMemoryInfo();
void resetSystem();
void printSystemStatus();
void checkCameraStatus();

#endif // SYSTEM_UTILS_H 