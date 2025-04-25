#include "../include/detection_system.h"
#include "../include/alert_system.h"
#include "../include/camera_system.h"

// Global variables
DriverState currentState = STATE_UNKNOWN;
DriverState previousState = STATE_UNKNOWN;
int consecutiveAwakeCount = 0;
int consecutiveDrowsyCount = 0;
static int noDetectionCount = 0;
float lastFaceConfidence = 0.0;
bool faceDetected = false;
int debugFrameCount = 0;

// Variables for threshold adjustment
bool adjustedThresholds = false;
const float ORIGINAL_DETECTION_THRESHOLD = 0.40f;
const float ORIGINAL_AWAKE_THRESHOLD = 0.35f;
int adaptationCycles = 0;
const float MIN_THRESHOLD = 0.25f;

void processImage() {
    if (snapshot_buf == NULL) {
        Serial.println("ERROR: Buffer not initialized");
        return;
    }

    memset(snapshot_buf, 0, EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * 3);

    Serial.println("Capturing image...");
    
    cam_capture_ok = ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf);
    
    if (!cam_capture_ok) {
        Serial.println("Camera capture failed");
        return;
    }

    Serial.println("Image captured successfully");

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    Serial.println("Running classifier...");

    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);

    if (err != EI_IMPULSE_OK) {
        Serial.print("ERROR: Failed to run classifier: ");
        Serial.println(err);
        return;
    }

    Serial.println("Classification completed");

    DriverState detectedState = STATE_UNKNOWN;
    float awakeConfidence = 0.0;
    float noddingConfidence = 0.0;
    float yawningConfidence = 0.0;
    float maxConfidence = 0.0;

    Serial.println("\n╔════════════════════════════════════════════════════════════╗");
    Serial.println("║                    INFERENCE RESULTS                        ║");
    Serial.println("╠════════════════════════════════════════════════════════════╣");
    
    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
        if (bb.value == 0) continue;
        
        if (bb.value > maxConfidence) {
            maxConfidence = bb.value;
            
            if (strcmp(bb.label, "awake") == 0) {
                detectedState = STATE_AWAKE;
                awakeConfidence = bb.value;
                Serial.printf("║ State: AWAKE        | Confidence: %.2f%%\n", bb.value * 100);
            }
            else if (strcmp(bb.label, "nodding") == 0) {
                detectedState = STATE_NODDING;
                noddingConfidence = bb.value;
                Serial.printf("║ State: NODDING      | Confidence: %.2f%%\n", bb.value * 100);
            }
            else if (strcmp(bb.label, "yawning") == 0) {
                detectedState = STATE_YAWNING;
                yawningConfidence = bb.value;
                Serial.printf("║ State: YAWNING     | Confidence: %.2f%%\n", bb.value * 100);
            }
        }
    }
    
    Serial.println("╠════════════════════════════════════════════════════════════╣");
    Serial.printf("║ Maximum Confidence: %.2f%%\n", maxConfidence * 100);
    Serial.println("╚════════════════════════════════════════════════════════════╝");

    if (maxConfidence < EMPTY_FRAME_CONFIDENCE) {
        emptyFrameCount++;
        if (emptyFrameCount > EMPTY_FRAME_THRESHOLD) {
            if (faceDetected) {
                Serial.println("\n╔════════════════════════════════════════════════════════════╗");
                Serial.println("║                    ALERT: NO FACE DETECTED                ║");
                Serial.println("╠════════════════════════════════════════════════════════════╣");
                Serial.println("║ No face detected after previous detection                 ║");
                Serial.println("║ Check camera positioning                                 ║");
                Serial.println("╚════════════════════════════════════════════════════════════╝");
                faceDetected = false;
            }
            consecutiveAwakeCount = 0;
            consecutiveDrowsyCount = 0;
            return;
        }
    } else {
        emptyFrameCount = 0;
        if (!faceDetected) {
            Serial.println("\n╔════════════════════════════════════════════════════════════╗");
            Serial.println("║                    FACE DETECTED                           ║");
            Serial.println("╠════════════════════════════════════════════════════════════╣");
            Serial.println("║ New face detected - Starting monitoring                   ║");
            Serial.println("╚════════════════════════════════════════════════════════════╝");
            faceDetected = true;
        }
    }
    
    lastFaceConfidence = maxConfidence;
    
    if (detectedState == STATE_AWAKE) {
        consecutiveAwakeCount++;
        if (consecutiveDrowsyCount > 0) {
            consecutiveDrowsyCount--;
        }
        Serial.printf("║ AWAKE Counter: %d/%d | DROWSY Counter: %d/%d\n", 
                     consecutiveAwakeCount, CONSECUTIVE_AWAKE_THRESHOLD,
                     consecutiveDrowsyCount, CONSECUTIVE_THRESHOLD);
    } 
    else if (detectedState == STATE_NODDING || detectedState == STATE_YAWNING) {
        if ((detectedState == STATE_NODDING && noddingConfidence >= DETECTION_THRESHOLD) ||
            (detectedState == STATE_YAWNING && yawningConfidence >= DETECTION_THRESHOLD)) {
            if (consecutiveDrowsyCount < MAX_CONSECUTIVE_DROWSY) {
                consecutiveDrowsyCount++;
            }
        }
        if (consecutiveAwakeCount > 0) {
            consecutiveAwakeCount--;
        }
        Serial.printf("║ AWAKE Counter: %d/%d | DROWSY Counter: %d/%d\n", 
                     consecutiveAwakeCount, CONSECUTIVE_AWAKE_THRESHOLD,
                     consecutiveDrowsyCount, CONSECUTIVE_THRESHOLD);
    }
    else {
        if (consecutiveDrowsyCount > 0) consecutiveDrowsyCount--;
        if (consecutiveAwakeCount > 0) consecutiveAwakeCount--;
        Serial.printf("║ AWAKE Counter: %d/%d | DROWSY Counter: %d/%d\n", 
                     consecutiveAwakeCount, CONSECUTIVE_AWAKE_THRESHOLD,
                     consecutiveDrowsyCount, CONSECUTIVE_THRESHOLD);
    }
    
    if (consecutiveAwakeCount >= CONSECUTIVE_AWAKE_THRESHOLD && currentState != STATE_AWAKE) {
        previousState = currentState;
        currentState = STATE_AWAKE;
        Serial.println("\n╔════════════════════════════════════════════════════════════╗");
        Serial.println("║                    STATE CHANGE: AWAKE                    ║");
        Serial.println("╠════════════════════════════════════════════════════════════╣");
        Serial.println("║ Driver is awake and alert                                 ║");
        Serial.println("║ Deactivating alerts                                       ║");
        Serial.println("╚════════════════════════════════════════════════════════════╝");
        deactivate_alerts();
    } 
    else if (consecutiveDrowsyCount >= CONSECUTIVE_THRESHOLD) {
        if ((detectedState == STATE_NODDING && noddingConfidence >= DETECTION_THRESHOLD) ||
            (detectedState == STATE_YAWNING && yawningConfidence >= DETECTION_THRESHOLD)) {
            
            if (currentState != detectedState) {
                previousState = currentState;
                currentState = detectedState;
                
                if (currentState == STATE_NODDING) {
                    Serial.println("\n╔════════════════════════════════════════════════════════════╗");
                    Serial.println("║                    ALERT: NODDING                        ║");
                    Serial.println("╠════════════════════════════════════════════════════════════╣");
                    Serial.println("║ Driver is nodding - Activating alerts                    ║");
                    Serial.println("╚════════════════════════════════════════════════════════════╝");
                    update_alert_level("nodding");
                } 
                else if (currentState == STATE_YAWNING) {
                    Serial.println("\n╔════════════════════════════════════════════════════════════╗");
                    Serial.println("║                    ALERT: YAWNING                        ║");
                    Serial.println("╠════════════════════════════════════════════════════════════╣");
                    Serial.println("║ Driver is yawning - Activating alerts                    ║");
                    Serial.println("╚════════════════════════════════════════════════════════════╝");
                    update_alert_level("yawning");
                }
            }
            else if (!alert_active && !inAlertCooldown) {
                if (consecutiveDrowsyCount >= MAX_CONSECUTIVE_DROWSY) {
                    currentAlertLevel = ALERT_CRITICAL;
                    alertLevelStartTime = millis();
                    Serial.println("Persistent state detected - Elevating to CRITICAL ALERT!");
                }
                
                if (currentState == STATE_NODDING) {
                    update_alert_level("nodding");
                } else {
                    update_alert_level("yawning");
                }
            }
        }
    }
}

void checkAndAdjustThresholds() {
    static unsigned long lastAdjustTime = 0;
    const unsigned long ADJUST_INTERVAL = 15000;
    
    if (millis() - lastAdjustTime < ADJUST_INTERVAL) {
        return;
    }
    
    lastAdjustTime = millis();
    
    if (noDetectionCount > 10 && !adjustedThresholds) {
        adaptationCycles++;
        
        float reductionFactor = 0.60 - (adaptationCycles * 0.10);
        if (reductionFactor < 0.3) reductionFactor = 0.3;
        
        DETECTION_THRESHOLD = ORIGINAL_DETECTION_THRESHOLD * reductionFactor;
        AWAKE_THRESHOLD = ORIGINAL_AWAKE_THRESHOLD * reductionFactor;
        
        if (DETECTION_THRESHOLD < MIN_THRESHOLD) DETECTION_THRESHOLD = MIN_THRESHOLD;
        if (AWAKE_THRESHOLD < MIN_THRESHOLD) AWAKE_THRESHOLD = MIN_THRESHOLD;
        
        adjustedThresholds = true;
        
        Serial.println("\n=== AGGRESSIVE SENSITIVITY ADJUSTMENT ===");
        Serial.printf("Adaptation cycle %d: Thresholds = %.2f, %.2f\n", 
                     adaptationCycles, AWAKE_THRESHOLD, DETECTION_THRESHOLD);
    }
    else if (noDetectionCount < 3 && adjustedThresholds) {
        DETECTION_THRESHOLD = ORIGINAL_DETECTION_THRESHOLD;
        AWAKE_THRESHOLD = ORIGINAL_AWAKE_THRESHOLD;
        adjustedThresholds = false;
        adaptationCycles = 0;
        
        Serial.println("\n=== SENSITIVITY RESTORATION ===");
        Serial.printf("Thresholds restored: Awake=%.2f, Drowsy=%.2f\n", 
                      AWAKE_THRESHOLD, DETECTION_THRESHOLD);
    }
    
    if (currentState == STATE_YAWNING && consecutiveDrowsyCount > 5) {
        DETECTION_THRESHOLD = ORIGINAL_DETECTION_THRESHOLD * 1.2f;
        Serial.println("\n=== ADJUSTMENT TO AVOID FALSE YAWNING POSITIVES ===");
        Serial.printf("Threshold increased to: %.2f\n", DETECTION_THRESHOLD);
    }
}

void captureDebugFrame() {
    if (noDetectionCount >= 10 && debugFrameCount < MAX_DEBUG_FRAMES) {
        camera_fb_t* debugFrame = esp_camera_fb_get();
        if (debugFrame) {
            Serial.println("\n==== DEBUG FRAME ====");
            Serial.printf("Size: %d bytes, Format: %d, Resolution: %dx%d\n", 
                        debugFrame->len, debugFrame->format, 
                        debugFrame->width, debugFrame->height);
            Serial.println("First 32 bytes:");
            
            for (int i = 0; i < 32 && i < debugFrame->len; i++) {
                Serial.printf("%02X ", debugFrame->buf[i]);
                if ((i + 1) % 8 == 0) Serial.println();
            }
            
            uint16_t *rgb565_buf = (uint16_t *)debugFrame->buf;
            const int SAMPLE_PIXELS = 100;
            const int pixelStep = debugFrame->len / (SAMPLE_PIXELS * 2);
            int totalBrightness = 0;
            
            for (int p = 0; p < SAMPLE_PIXELS && p * pixelStep < debugFrame->len / 2; p++) {
                uint16_t pixel = rgb565_buf[p * pixelStep];
                uint8_t r = (pixel >> 11) & 0x1F;
                uint8_t g = (pixel >> 5) & 0x3F;
                uint8_t b = pixel & 0x1F;
                
                int brightness = (r * 21 + g * 72 + b * 7) / 100;
                totalBrightness += brightness;
            }
            
            int avgBrightness = totalBrightness / SAMPLE_PIXELS;
            Serial.printf("Estimated average brightness: %d/255\n", avgBrightness);
            Serial.println("==========================");
            
            esp_camera_fb_return(debugFrame);
            debugFrameCount++;
        }
    }
} 