#include "../include/camera_system.h"

// Global variables
static bool is_initialised = false;
static camera_config_t camera_config;
camera_fb_t *fb = NULL;
uint8_t *snapshot_buf = NULL;
static bool cam_capture_ok = false;

// Luminosity control
const int LUMINOSITY_THRESHOLD_LOW = 40;
const int LUMINOSITY_THRESHOLD_HIGH = 200;
int lastLuminosity = 0;
bool luminosityAdjusted = false;
const int LUMINOSITY_CHECK_INTERVAL = 5000;
unsigned long lastLuminosityCheck = 0;

bool ei_camera_init(void) {
    if (is_initialised) return true;

    camera_config.ledc_channel = LEDC_CHANNEL_0;
    camera_config.ledc_timer = LEDC_TIMER_0;
    camera_config.pin_d0 = Y2_GPIO_NUM;
    camera_config.pin_d1 = Y3_GPIO_NUM;
    camera_config.pin_d2 = Y4_GPIO_NUM;
    camera_config.pin_d3 = Y5_GPIO_NUM;
    camera_config.pin_d4 = Y6_GPIO_NUM;
    camera_config.pin_d5 = Y7_GPIO_NUM;
    camera_config.pin_d6 = Y8_GPIO_NUM;
    camera_config.pin_d7 = Y9_GPIO_NUM;
    camera_config.pin_xclk = XCLK_GPIO_NUM;
    camera_config.pin_pclk = PCLK_GPIO_NUM;
    camera_config.pin_vsync = VSYNC_GPIO_NUM;
    camera_config.pin_href = HREF_GPIO_NUM;
    camera_config.pin_sscb_sda = SIOD_GPIO_NUM;
    camera_config.pin_sccb_scl = SIOC_GPIO_NUM;
    camera_config.pin_pwdn = PWDN_GPIO_NUM;
    camera_config.pin_reset = RESET_GPIO_NUM;
    camera_config.xclk_freq_hz = 20000000;
    camera_config.pixel_format = PIXFORMAT_JPEG;
    camera_config.frame_size = FRAMESIZE_QVGA;
    camera_config.jpeg_quality = 12;
    camera_config.fb_count = 1;
    camera_config.fb_location = CAMERA_FB_IN_PSRAM;
    camera_config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        Serial.printf("Camera initialization failed, error 0x%x\n", err);
        return false;
    }

    sensor_t * s = esp_camera_sensor_get();
    if (s == NULL) {
        Serial.println("ERROR: Could not get camera sensor");
        return false;
    }

    s->set_brightness(s, 0);
    s->set_contrast(s, 1);
    s->set_saturation(s, 1);
    s->set_gainceiling(s, GAINCEILING_2X);
    s->set_whitebal(s, 1);
    s->set_exposure_ctrl(s, 1);
    s->set_aec2(s, 1);
    s->set_ae_level(s, 0);
    s->set_aec_value(s, 600);
    s->set_gain_ctrl(s, 1);
    s->set_agc_gain(s, 0);

    is_initialised = true;
    return true;
}

void ei_camera_deinit(void) {
    if (fb) {
        esp_camera_fb_return(fb);
        fb = NULL;
    }
    
    if (is_initialised) {
        esp_err_t err = esp_camera_deinit();
        
        if (err != ESP_OK) {
            Serial.println("Failed to deinitialize camera");
        }
        
        is_initialised = false;
    }
}

bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
    if (!is_initialised) {
        Serial.println("ERROR: Camera not initialized");
        return false;
    }

    if (fb) {
        esp_camera_fb_return(fb);
        fb = NULL;
    }

    fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("Camera capture failed");
        return false;
    }

    bool converted = fmt2rgb888(fb->buf, fb->len, fb->format, snapshot_buf);
    
    esp_camera_fb_return(fb);
    fb = NULL;

    if (!converted) {
        Serial.println("Failed to convert to RGB888");
        return false;
    }

    ei::image::processing::crop_and_interpolate_rgb888(
        snapshot_buf,
        EI_CAMERA_RAW_FRAME_BUFFER_COLS,
        EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
        out_buf,
        img_width,
        img_height
    );

    return true;
}

int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
    if (snapshot_buf == NULL || !cam_capture_ok) {
        for (size_t i = 0; i < length; i++) {
            out_ptr[i] = 0.0f;
        }
        return 0;
    }
    
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;
    
    if (pixel_ix >= EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE) {
        return 0;
    }
    
    size_t max_pixels = (EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE - pixel_ix) / 3;
    pixels_left = (pixels_left < max_pixels) ? pixels_left : max_pixels;
    
    while (pixels_left != 0) {
        out_ptr[out_ptr_ix] = snapshot_buf[pixel_ix] / 255.0f;      // R
        out_ptr[out_ptr_ix + 1] = snapshot_buf[pixel_ix + 1] / 255.0f;  // G
        out_ptr[out_ptr_ix + 2] = snapshot_buf[pixel_ix + 2] / 255.0f;  // B
        
        out_ptr_ix += 3;
        pixel_ix += 3;
        pixels_left--;
    }
    
    return 0;
}

void checkAndAdjustLuminosity() {
    if (millis() - lastLuminosityCheck < LUMINOSITY_CHECK_INTERVAL) {
        return;
    }
    
    lastLuminosityCheck = millis();
    
    Serial.println("\n=== Luminosity Check ===");
    Serial.println("Capturing image for analysis...");
    
    camera_fb_t* test_fb = esp_camera_fb_get();
    if (!test_fb) {
        Serial.println("ERROR: Failed to capture image for luminosity analysis");
        return;
    }
    
    Serial.printf("Image captured: %dx%d, format: %d\n", 
                 test_fb->width, test_fb->height, test_fb->format);
    
    uint8_t* rgb888 = (uint8_t*)malloc(test_fb->width * test_fb->height * 3);
    if (!rgb888) {
        Serial.println("ERROR: Failed to allocate buffer for luminosity analysis");
        esp_camera_fb_return(test_fb);
        return;
    }
    
    if (!fmt2rgb888(test_fb->buf, test_fb->len, test_fb->format, rgb888)) {
        Serial.println("ERROR: Failed to convert to RGB888");
        free(rgb888);
        esp_camera_fb_return(test_fb);
        return;
    }
    
    const int SAMPLE_PIXELS = 100;
    const int pixelStep = (test_fb->width * test_fb->height) / SAMPLE_PIXELS;
    int totalBrightness = 0;
    int minBrightness = 255;
    int maxBrightness = 0;
    
    Serial.println("Analyzing luminosity...");
    
    for (int i = 0; i < SAMPLE_PIXELS; i++) {
        int pixelIndex = i * pixelStep * 3;
        uint8_t r = rgb888[pixelIndex];
        uint8_t g = rgb888[pixelIndex + 1];
        uint8_t b = rgb888[pixelIndex + 2];
        
        int brightness = (r * 21 + g * 72 + b * 7) / 100;
        totalBrightness += brightness;
        
        if (brightness < minBrightness) minBrightness = brightness;
        if (brightness > maxBrightness) maxBrightness = brightness;
    }
    
    int avgBrightness = totalBrightness / SAMPLE_PIXELS;
    lastLuminosity = avgBrightness;
    
    Serial.printf("Luminosity - Average: %d, Min: %d, Max: %d\n", 
                 avgBrightness, minBrightness, maxBrightness);
    
    free(rgb888);
    esp_camera_fb_return(test_fb);
    
    sensor_t * s = esp_camera_sensor_get();
    if (!s) {
        Serial.println("ERROR: Could not get sensor for luminosity adjustment");
        return;
    }
    
    int currentBrightness = s->status.brightness;
    int currentContrast = s->status.contrast;
    int currentGain = s->status.agc_gain;
    int currentAEC = s->status.aec_value;
    
    Serial.println("\nCurrent settings:");
    Serial.printf("  Brightness: %d\n", currentBrightness);
    Serial.printf("  Contrast: %d\n", currentContrast);
    Serial.printf("  Gain: %d\n", currentGain);
    Serial.printf("  AEC: %d\n", currentAEC);
    
    if (avgBrightness < LUMINOSITY_THRESHOLD_LOW) {
        s->set_brightness(s, 2);
        s->set_contrast(s, 2);
        s->set_gainceiling(s, GAINCEILING_8X);
        s->set_aec_value(s, 1200);
        s->set_agc_gain(s, 5);
        Serial.println("\nDark environment detected. Applying high brightness settings:");
        Serial.println("  Brightness: +2");
        Serial.println("  Contrast: +2");
        Serial.println("  Gain: 8X");
        Serial.println("  AEC: 1200");
        Serial.println("  AGC: 5");
    } 
    else if (avgBrightness > LUMINOSITY_THRESHOLD_HIGH) {
        s->set_brightness(s, -1);
        s->set_contrast(s, 0);
        s->set_gainceiling(s, GAINCEILING_2X);
        s->set_aec_value(s, 300);
        s->set_agc_gain(s, 0);
        Serial.println("\nVery bright environment detected. Reducing exposure:");
        Serial.println("  Brightness: -1");
        Serial.println("  Contrast: 0");
        Serial.println("  Gain: 2X");
        Serial.println("  AEC: 300");
        Serial.println("  AGC: 0");
    }
    else {
        s->set_brightness(s, 0);
        s->set_contrast(s, 1);
        s->set_gainceiling(s, GAINCEILING_2X);
        s->set_aec_value(s, 600);
        s->set_agc_gain(s, 0);
        Serial.println("\nAdequate luminosity. Using default settings:");
        Serial.println("  Brightness: 0");
        Serial.println("  Contrast: 1");
        Serial.println("  Gain: 2X");
        Serial.println("  AEC: 600");
        Serial.println("  AGC: 0");
    }
    
    luminosityAdjusted = true;
    Serial.println("=== End of Luminosity Check ===\n");
} 