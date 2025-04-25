#include "../include/alert_system.h"

// Global variables
volatile bool alert_active = false;
unsigned long lastAlertTime = 0;
unsigned long alertStartTime = 0;
bool inAlertCooldown = false;
AlertLevel currentAlertLevel = ALERT_NONE;
unsigned long alertLevelStartTime = 0;

// LED and buzzer pins
const int FLASH_LED_PIN = 4;
const int BUZZER_PIN = 12;
const int LED_BUILTIN = 33;

// PWM settings
const int PWM_FREQ = 5000;
const int PWM_CHANNEL = 0;
const int PWM_RESOLUTION = 8;
const int FLASH_INTENSITY_LOW = 50;
const int FLASH_INTENSITY_MEDIUM = 125;
const int FLASH_INTENSITY_HIGH = 200;
const int FLASH_INTENSITY_CRITICAL = 255;

// Variables for LED blinking
unsigned long previousMillis = 0;
int blinkInterval = 0;
bool flashState = LOW;

// Variables for buzzer patterns
unsigned long lastBuzzerPatternTime = 0;
int currentBuzzerPattern = 0;
int buzzerPatternSteps = 0;
bool buzzerCurrentlyOn = false;

void setupPWM() {
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = (ledc_timer_bit_t)PWM_RESOLUTION,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);
    
    ledc_channel_config_t ledc_channel = {
        .gpio_num = FLASH_LED_PIN,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = (ledc_channel_t)PWM_CHANNEL,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ledc_channel);
}

void setFlashIntensity(uint32_t duty) {
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)PWM_CHANNEL, duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)PWM_CHANNEL);
}

void activate_alerts(const char *type) {
    if (alert_active) {
        switch (currentAlertLevel) {
            case ALERT_LOW:
                blinkInterval = 500;
                setFlashIntensity(FLASH_INTENSITY_LOW);
                break;
            case ALERT_MEDIUM:
                blinkInterval = 300;
                setFlashIntensity(FLASH_INTENSITY_MEDIUM);
                break;
            case ALERT_HIGH:
                blinkInterval = 150;
                setFlashIntensity(FLASH_INTENSITY_HIGH);
                break;
            case ALERT_CRITICAL:
                blinkInterval = 80;
                setFlashIntensity(FLASH_INTENSITY_CRITICAL);
                break;
            default:
                blinkInterval = 500;
                setFlashIntensity(FLASH_INTENSITY_LOW);
                break;
        }
        return;
    }
    
    digitalWrite(LED_BUILTIN, LOW);
    
    switch (currentAlertLevel) {
        case ALERT_LOW:
            blinkInterval = 500;
            setFlashIntensity(FLASH_INTENSITY_LOW);
            if (strcmp(type, "nodding") == 0) {
                tone(BUZZER_PIN, 1000);
            } else {
                tone(BUZZER_PIN, 800);
            }
            break;
        case ALERT_MEDIUM:
            blinkInterval = 300;
            setFlashIntensity(FLASH_INTENSITY_MEDIUM);
            if (strcmp(type, "nodding") == 0) {
                tone(BUZZER_PIN, 1500);
            } else {
                tone(BUZZER_PIN, 1200);
            }
            break;
        case ALERT_HIGH:
            blinkInterval = 150;
            setFlashIntensity(FLASH_INTENSITY_HIGH);
            if (strcmp(type, "nodding") == 0) {
                tone(BUZZER_PIN, 2000);
            } else {
                tone(BUZZER_PIN, 1800);
            }
            break;
        case ALERT_CRITICAL:
            blinkInterval = 80;
            setFlashIntensity(FLASH_INTENSITY_CRITICAL);
            if (strcmp(type, "nodding") == 0) {
                play_buzzer_pattern(ALERT_CRITICAL);
            } else {
                tone(BUZZER_PIN, 2500);
            }
            break;
        default:
            blinkInterval = 500;
            setFlashIntensity(FLASH_INTENSITY_LOW);
            tone(BUZZER_PIN, 1000);
            break;
    }
    
    alert_active = true;
    alertStartTime = millis();
    lastAlertTime = millis();
}

void update_alert_level(const char *type) {
    if (inAlertCooldown) {
        unsigned long cooldownTime = 0;
        switch (currentAlertLevel) {
            case ALERT_LOW: cooldownTime = ALERT_COOLDOWN_LOW; break;
            case ALERT_MEDIUM: cooldownTime = ALERT_COOLDOWN_MEDIUM; break;
            case ALERT_HIGH: cooldownTime = ALERT_COOLDOWN_HIGH; break;
            case ALERT_CRITICAL: cooldownTime = ALERT_COOLDOWN_CRITICAL; break;
            default: cooldownTime = ALERT_COOLDOWN_NONE;
        }
        
        if (millis() - lastAlertTime < cooldownTime) {
            return;
        }
        inAlertCooldown = false;
    }
    
    if (alert_active) {
        unsigned long timeInCurrentLevel = millis() - alertLevelStartTime;
        
        if (currentAlertLevel == ALERT_LOW && timeInCurrentLevel > TIME_TO_MEDIUM_ALERT) {
            currentAlertLevel = ALERT_MEDIUM;
            alertLevelStartTime = millis();
        } 
        else if (currentAlertLevel == ALERT_MEDIUM && timeInCurrentLevel > TIME_TO_HIGH_ALERT) {
            currentAlertLevel = ALERT_HIGH;
            alertLevelStartTime = millis();
        }
        else if (currentAlertLevel == ALERT_HIGH && timeInCurrentLevel > TIME_TO_CRITICAL_ALERT) {
            currentAlertLevel = ALERT_CRITICAL;
            alertLevelStartTime = millis();
        }
        
        activate_alerts(type);
    } 
    else {
        currentAlertLevel = ALERT_LOW;
        alertLevelStartTime = millis();
        activate_alerts(type);
    }
}

void deactivate_alerts() {
    if (alert_active) {
        setFlashIntensity(0);
        digitalWrite(LED_BUILTIN, HIGH);
        noTone(BUZZER_PIN);
        alert_active = false;
        currentAlertLevel = ALERT_NONE;
        currentBuzzerPattern = 0;
        buzzerCurrentlyOn = false;
        
        inAlertCooldown = true;
        lastAlertTime = millis();
    }
}

void manageAlertTimeout() {
    if (alert_active) {
        unsigned long alertDuration = 0;
        switch (currentAlertLevel) {
            case ALERT_LOW: alertDuration = ALERT_DURATION_LOW; break;
            case ALERT_MEDIUM: alertDuration = ALERT_DURATION_MEDIUM; break;
            case ALERT_HIGH: alertDuration = ALERT_DURATION_HIGH; break;
            case ALERT_CRITICAL: alertDuration = ALERT_DURATION_CRITICAL; break;
            default: alertDuration = ALERT_DURATION_NONE;
        }
        
        if (millis() - alertStartTime > alertDuration) {
            deactivate_alerts();
        }
        
        if (currentAlertLevel == ALERT_CRITICAL) {
            play_buzzer_pattern(ALERT_CRITICAL);
        }
    }
}

void play_buzzer_pattern(AlertLevel level) {
    unsigned long currentMillis = millis();
    
    if (currentBuzzerPattern == 0) {
        if (level == ALERT_CRITICAL) {
            buzzerPatternSteps = 9;
            currentBuzzerPattern = 1;
            lastBuzzerPatternTime = currentMillis;
            tone(BUZZER_PIN, 2500);
            buzzerCurrentlyOn = true;
        }
    } else {
        if (level == ALERT_CRITICAL) {
            unsigned long stepDuration = 200;
            
            if (currentMillis - lastBuzzerPatternTime > stepDuration) {
                lastBuzzerPatternTime = currentMillis;
                
                currentBuzzerPattern++;
                if (currentBuzzerPattern > buzzerPatternSteps) {
                    currentBuzzerPattern = 1;
                }
                
                if (currentBuzzerPattern <= 3) {
                    if (buzzerCurrentlyOn) {
                        noTone(BUZZER_PIN);
                        buzzerCurrentlyOn = false;
                    } else {
                        tone(BUZZER_PIN, 2500);
                        buzzerCurrentlyOn = true;
                    }
                } 
                else if (currentBuzzerPattern <= 6) {
                    if (buzzerCurrentlyOn) {
                        noTone(BUZZER_PIN);
                        buzzerCurrentlyOn = false;
                    } else {
                        tone(BUZZER_PIN, 2000);
                        buzzerCurrentlyOn = true;
                    }
                } 
                else {
                    if (buzzerCurrentlyOn) {
                        noTone(BUZZER_PIN);
                        buzzerCurrentlyOn = false;
                    } else {
                        tone(BUZZER_PIN, 2500);
                        buzzerCurrentlyOn = true;
                    }
                }
            }
        }
    }
} 