#ifndef DETECTION_SYSTEM_H
#define DETECTION_SYSTEM_H

#include <Arduino.h>

// Driver states
enum DriverState {
    STATE_UNKNOWN = 0,
    STATE_AWAKE = 1,
    STATE_NODDING = 2,
    STATE_YAWNING = 3
};

// Detection thresholds
const float DETECTION_THRESHOLD = 0.65f;
const float AWAKE_THRESHOLD = 0.60f;
const int CONSECUTIVE_THRESHOLD = 3;
const int CONSECUTIVE_AWAKE_THRESHOLD = 2;
const int MAX_CONSECUTIVE_DROWSY = 5;

// Empty frame detection
const float EMPTY_FRAME_CONFIDENCE = 0.15f;
const int EMPTY_FRAME_THRESHOLD = 10;

// Function prototypes
void processImage();
void checkAndAdjustThresholds();
void captureDebugFrame();

#endif // DETECTION_SYSTEM_H 