/* 
 * Drowsiness Sensor - Driver Drowsiness Detection System
 * 
 * This application uses ESP32-CAM and Edge Impulse model to detect 
 * driver drowsiness (nodding or yawning) and emits alerts.
 * 
 * Developed for ESP32-CAM AI Thinker
 */

// Required libraries
#include <snorless-1_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "esp_camera.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "driver/ledc.h"
#include "img_converters.h"

// Include custom headers
#include "include/alert_system.h"
#include "include/camera_system.h"
#include "include/detection_system.h"
#include "include/system_utils.h"

// Debug flag
static bool debug_nn = false;

void setup() {
    // Disable brownout detector
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
    
    // Initialize serial communication
    Serial.begin(115200);
    Serial.println("\n\n===================================");
    Serial.println("Starting Drowsiness Sensor System");
    Serial.println("Version 1.1.0 - Capture Improvements");
    Serial.println("===================================");
    
    // Configure output pins
    pinMode(FLASH_LED_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    
    // Configure PWM for flash
    setupPWM();
    setFlashIntensity(0);
    
    // Deactivate alerts initially
    digitalWrite(FLASH_LED_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(LED_BUILTIN, HIGH);
    
    // Allocate image buffer
    size_t buf_size = EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * 3;
    
    if (psramFound()) {
        Serial.println("PSRAM found. Using for buffer allocation.");
        snapshot_buf = (uint8_t*)ps_calloc(1, buf_size);
    } else {
        Serial.println("PSRAM not found. Using normal memory.");
        snapshot_buf = (uint8_t*)calloc(1, buf_size);
    }
    
    if (snapshot_buf == NULL) {
        Serial.println("ERROR: Failed to allocate image buffer!");
        while (1) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            delay(300);
        }
    }
    
    // Initialize camera
    if (ei_camera_init()) {
        Serial.println("System initialized. Starting monitoring...");
    } else {
        Serial.println("ERROR: Initialization failed. Please restart the device.");
    }

    // Print memory information
    printMemoryInfo();

    // Record initial time
    lastSuccessfulInference = millis();
    
    delay(1000);
}

void loop() {
    // Manage alert timeout
    manageAlertTimeout();
    
    // Check and adjust thresholds if needed
    checkAndAdjustThresholds();
    
    // Check and adjust luminosity
    checkAndAdjustLuminosity();
    
    // Check if it's time to display system status
    if (millis() - lastStatusPrint >= STATUS_PRINT_INTERVAL) {
        printSystemStatus();
        printMemoryInfo();
        lastStatusPrint = millis();
    }
    
    // Check camera status periodically
    checkCameraStatus();
    
    // Calculate FPS
    if (lastInferenceTime > 0) {
        unsigned long elapsedTime = millis() - lastInferenceTime;
        if (elapsedTime > 0) {
            fps = 1000.0 / elapsedTime;
            fpsHistory[fpsHistoryIndex] = fps;
            fpsHistoryIndex = (fpsHistoryIndex + 1) % FPS_AVERAGING_WINDOW;
        }
    }
    lastInferenceTime = millis();
    
    // Manage LED flash blinking during alerts
    if (alert_active) {
        unsigned long currentMillis = millis();
        if (currentMillis - previousMillis >= blinkInterval) {
            previousMillis = currentMillis;
            flashState = !flashState;
            digitalWrite(FLASH_LED_PIN, flashState);
        }
    }
    
    Serial.println("Starting capture...");
    
    try {
        // Process image and perform classification
        processImage();
        Serial.println("Processing completed");
    } catch (...) {
        Serial.println("ERROR: Exception caught during image processing");
        failureCount++;
        
        if (failureCount >= MAX_FAILURE_COUNT) {
            Serial.println("ERROR: Too many consecutive failures. Restarting system.");
            resetSystem();
        }
    }
    
    // Wait before next inference
    delay(DELAY_BETWEEN_INFERENCES);
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid model for current sensor"
#endif 