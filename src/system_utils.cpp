#include "../include/system_utils.h"
#include "../include/camera_system.h"
#include "../include/detection_system.h"

// Global variables
unsigned long lastSuccessfulInference = 0;
unsigned long totalInferences = 0;
unsigned long totalAlerts = 0;
unsigned long lastStatusPrint = 0;
unsigned long lastInferenceTime = 0;
float fps = 0;
float fpsHistory[10] = {0};
int fpsHistoryIndex = 0;
int failureCount = 0;

void printMemoryInfo() {
    uint32_t freeHeap = ESP.getFreeHeap();
    uint32_t totalHeap = ESP.getHeapSize();
    uint32_t freePsram = ESP.getFreePsram();
    uint32_t totalPsram = ESP.getPsramSize();
    
    Serial.println("\n=== Memory Information ===");
    Serial.printf("Heap - Free: %d KB / Total: %d KB (%.1f%%)\n", 
                freeHeap / 1024, totalHeap / 1024, 
                100.0 * freeHeap / totalHeap);
    
    if (totalPsram > 0) {
        Serial.printf("PSRAM - Free: %d KB / Total: %d KB (%.1f%%)\n", 
                      freePsram / 1024, totalPsram / 1024,
                      100.0 * freePsram / totalPsram);
    } else {
        Serial.println("PSRAM not available");
    }
    Serial.println("==============================");
}

void resetSystem() {
    Serial.println("Resetting system...");
    delay(500);
    ESP.restart();
}

void printSystemStatus() {
    uint32_t freeHeap = ESP.getFreeHeap();
    uint32_t totalHeap = ESP.getHeapSize();
    
    float avgFps = 0;
    for (int i = 0; i < FPS_AVERAGING_WINDOW; i++) {
        avgFps += fpsHistory[i];
    }
    avgFps /= FPS_AVERAGING_WINDOW;
    
    Serial.println("\n=== System Status ===");
    Serial.printf("Uptime: %lu seconds\n", millis() / 1000);
    Serial.printf("Free memory: %d KB / %d KB (%.1f%%)\n", 
                freeHeap / 1024, totalHeap / 1024,
                (1.0 - (float)freeHeap / totalHeap) * 100.0);
    Serial.printf("Average FPS: %.2f\n", avgFps);
    Serial.printf("Total inferences: %lu\n", totalInferences);
    Serial.printf("Total alerts: %lu\n", totalAlerts);
    Serial.printf("Current state: %s\n", 
                currentState == STATE_AWAKE ? "Awake" :
                currentState == STATE_NODDING ? "Nodding" :
                currentState == STATE_YAWNING ? "Yawning" : "Unknown");
    Serial.printf("Last alert: %lu seconds ago\n", 
                alertStartTime > 0 ? (millis() - alertStartTime) / 1000 : 0);
    Serial.printf("Current luminosity: %d\n", lastLuminosity);
    Serial.println("=======================");
}

void checkCameraStatus() {
    if (millis() - lastSuccessfulInference > WATCHDOG_TIMEOUT) {
        Serial.println("WARNING: Possible camera issue. Attempting to reinitialize...");
        
        ei_camera_deinit();
        delay(500);
        
        if (ei_camera_init()) {
            Serial.println("Camera reinitialized successfully!");
            failureCount = 0;
        } else {
            Serial.println("ERROR: Failed to reinitialize camera!");
            failureCount++;
            
            if (failureCount >= MAX_FAILURE_COUNT) {
                Serial.println("ERROR: Too many consecutive failures. Restarting system.");
                resetSystem();
            }
        }
        
        lastSuccessfulInference = millis();
    }
} 