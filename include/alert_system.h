#ifndef ALERT_SYSTEM_H
#define ALERT_SYSTEM_H

#include <Arduino.h>

// Alert levels
enum AlertLevel {
    ALERT_NONE = 0,      // No alert
    ALERT_LOW = 1,       // Light alert (first sign)
    ALERT_MEDIUM = 2,    // Medium alert (persistent drowsiness)
    ALERT_HIGH = 3,      // High alert (severe drowsiness)
    ALERT_CRITICAL = 4   // Critical alert (emergency)
};

// Alert timing constants
const unsigned long TIME_TO_MEDIUM_ALERT = 5000;   // 5 seconds at low level
const unsigned long TIME_TO_HIGH_ALERT = 8000;     // 8 seconds at medium level
const unsigned long TIME_TO_CRITICAL_ALERT = 7000; // 7 seconds at high level

// Alert cooldown times
const unsigned long ALERT_COOLDOWN_NONE = 0;       // No cooldown
const unsigned long ALERT_COOLDOWN_LOW = 30000;    // 30 seconds
const unsigned long ALERT_COOLDOWN_MEDIUM = 60000; // 1 minute
const unsigned long ALERT_COOLDOWN_HIGH = 120000;  // 2 minutes
const unsigned long ALERT_COOLDOWN_CRITICAL = 300000; // 5 minutes

// Alert durations
const unsigned long ALERT_DURATION_NONE = 0;
const unsigned long ALERT_DURATION_LOW = 2000;     // 2 seconds
const unsigned long ALERT_DURATION_MEDIUM = 3000;  // 3 seconds
const unsigned long ALERT_DURATION_HIGH = 4000;    // 4 seconds
const unsigned long ALERT_DURATION_CRITICAL = 5000; // 5 seconds

// Function prototypes
void activate_alerts(const char *type);
void update_alert_level(const char *type);
void deactivate_alerts();
void manageAlertTimeout();
void play_buzzer_pattern(AlertLevel level);

#endif // ALERT_SYSTEM_H 