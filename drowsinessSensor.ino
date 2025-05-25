/* 
 * SnorLess - Sistema de Detecção de Sonolência para Motoristas
 * 
 * Este aplicativo usa o ESP32-CAM e um modelo Edge Impulse para detectar 
 * se um motorista está sonolento (cabeceando ou bocejando) e emite alertas.
 * 
 * Desenvolvido para ESP32-CAM AI Thinker
 */

// Bibliotecas necessárias
#include <snorless-1_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "esp_camera.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "driver/ledc.h"
#include "img_converters.h"

// Adicionar bibliotecas para Wi-Fi e Web Server
#include <WiFi.h>
#include <WebServer.h>

// Configurações do Access Point Wi-Fi
const char *ssid = "ESP32-CAM_SnorLess";
const char *password = "snorless123"; // Senha para a rede Wi-Fi

WebServer server(80); // Criar objeto servidor na porta 80

// Definições para o ESP32-CAM AI-Thinker
#define CAMERA_MODEL_AI_THINKER

// Definições de pinos GPIO
#define FLASH_LED_PIN 4     // LED Flash do ESP32-CAM
#define BUZZER_PIN 12       // Buzzer conectado ao GPIO12
#define LED_BUILTIN 33      // LED vermelho na placa ESP32-CAM

// Configurações para PWM do flash para alarmes com intensidade variável
const int PWM_FREQ = 5000;
const int PWM_CHANNEL = 0;  // Alterado para 0
const int PWM_RESOLUTION = 8;
const int FLASH_INTENSITY_LOW = 50;      // ~20% de intensidade
const int FLASH_INTENSITY_MEDIUM = 125;  // ~50% de intensidade
const int FLASH_INTENSITY_HIGH = 200;    // ~80% de intensidade
const int FLASH_INTENSITY_CRITICAL = 255; // 100% de intensidade

// Configurações da câmera para o modelo AI-Thinker
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// Definições para o frame buffer da câmera
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS     320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS     240
#define EI_CAMERA_FRAME_BYTE_SIZE           3

// Variáveis privadas
static bool debug_nn = false;        // Para debug do modelo de inferência
static bool is_initialised = false;  // Status de inicialização da câmera
uint8_t *snapshot_buf = NULL;        // Buffer para a imagem capturada
static bool cam_capture_ok = false;  // Status da última captura

// Limites para detecção de sonolência
float DETECTION_THRESHOLD = 0.65f;   // Aumentado para reduzir falsos positivos
float AWAKE_THRESHOLD = 0.60f;       // Aumentado para reduzir falsos positivos
const int ALERT_DURATION_MS = 2000;       // Duração do alerta em ms
const int DELAY_BETWEEN_INFERENCES = 2000; // Aumentado para 2s para melhorar qualidade

// Estado dos alertas
volatile bool alert_active = false;

// Variáveis para controle do blink do LED
unsigned long previousMillis = 0;
int blinkInterval = 0;
bool flashState = LOW;

// Contadores para evitar falsos positivos
int consecutiveAwakeCount = 0;
int consecutiveDrowsyCount = 0;
const int CONSECUTIVE_THRESHOLD = 3;        // Threshold para estados de sonolência
const int CONSECUTIVE_AWAKE_THRESHOLD = 2;  // Threshold para estado acordado
const int MAX_CONSECUTIVE_DROWSY = 5;       // Limite máximo de contagem
static int noDetectionCount = 0;            // Contador para casos sem detecção

// Estado atual
enum DriverState {
  STATE_UNKNOWN = 0,
  STATE_AWAKE = 1,
  STATE_NODDING = 2,
  STATE_YAWNING = 3
};

// Níveis de alerta progressivos
enum AlertLevel {
    ALERT_NONE = 0,      // Sem alerta
    ALERT_LOW = 1,       // Alerta leve (primeiro sinal)
    ALERT_MEDIUM = 2,    // Alerta médio (sonolência persistente)
    ALERT_HIGH = 3,      // Alerta intenso (sonolência grave)
    ALERT_CRITICAL = 4   // Alerta crítico (emergência)
};

// Tempos para progressão de alertas (ms)
const unsigned long TIME_TO_MEDIUM_ALERT = 5000;   // 5 segundos no nível baixo
const unsigned long TIME_TO_HIGH_ALERT = 8000;     // 8 segundos no nível médio
const unsigned long TIME_TO_CRITICAL_ALERT = 7000; // 7 segundos no nível alto

// Tempos de cooldown entre alertas (ms)
const unsigned long ALERT_COOLDOWN_NONE = 0;       // Sem cooldown
const unsigned long ALERT_COOLDOWN_LOW = 30000;    // 30 segundos
const unsigned long ALERT_COOLDOWN_MEDIUM = 60000; // 1 minuto
const unsigned long ALERT_COOLDOWN_HIGH = 120000;  // 2 minutos
unsigned long ALERT_COOLDOWN_CRITICAL = 300000; // 5 minutos (agora é variável)

// Duração dos alertas (ms)
const unsigned long ALERT_DURATION_NONE = 0;
const unsigned long ALERT_DURATION_LOW = 2000;     // 2 segundos
const unsigned long ALERT_DURATION_MEDIUM = 3000;  // 3 segundos
const unsigned long ALERT_DURATION_HIGH = 4000;    // 4 segundos
const unsigned long ALERT_DURATION_CRITICAL = 5000; // 5 segundos

// Variáveis para controle de alertas
unsigned long lastAlertTime = 0;
unsigned long alertStartTime = 0;
bool inAlertCooldown = false;
AlertLevel currentAlertLevel = ALERT_NONE;
unsigned long alertLevelStartTime = 0;

DriverState currentState = STATE_UNKNOWN;
DriverState previousState = STATE_UNKNOWN;

// Adicionar registro de tempo para monitoramento
const int WATCHDOG_TIMEOUT = 15000; // 15 segundos
unsigned long lastSuccessfulInference = 0;

// Variáveis para monitoramento do sistema
unsigned long totalInferences = 0;
unsigned long totalAlerts = 0;
unsigned long lastStatusPrint = 0;
const int STATUS_PRINT_INTERVAL = 30000; // Exibir status a cada 30 segundos
unsigned long lastInferenceTime = 0;
float fps = 0;
const int FPS_AVERAGING_WINDOW = 10; // Média de FPS das últimas 10 inferências
float fpsHistory[10] = {0};
int fpsHistoryIndex = 0;

// Contador de frames de debug salvos
int debugFrameCount = 0;
const int MAX_DEBUG_FRAMES = 5; // Limitar número de frames para economizar memória

// Variáveis para detecção de "sem face"
float lastFaceConfidence = 0.0;
const int EMPTY_FRAME_THRESHOLD = 10;  // Reduzido para dar mais chances
int emptyFrameCount = 0;
bool faceDetected = false;
const float EMPTY_FRAME_CONFIDENCE = 0.15f;  // Reduzido para ser mais sensível

// Variáveis para processamento de imagem
uint8_t *rgb888_buffer = NULL;
uint8_t *resized_buffer = NULL;
ei_impulse_result_t result = {0};

// Variáveis para controle de luminosidade
const int LUMINOSITY_THRESHOLD_LOW = 40;    // Abaixo disso, aumenta ganho
const int LUMINOSITY_THRESHOLD_HIGH = 200;  // Acima disso, reduz ganho
int lastLuminosity = 0;
bool luminosityAdjusted = false;
const int LUMINOSITY_CHECK_INTERVAL = 5000; // Verificar a cada 5 segundos
unsigned long lastLuminosityCheck = 0;

// Número máximo de falhas consecutivas antes de reiniciar
const int MAX_FAILURE_COUNT = 5; 
int failureCount = 0;

// Variáveis para padrões sonoros avançados
unsigned long lastBuzzerPatternTime = 0;
int currentBuzzerPattern = 0;
int buzzerPatternSteps = 0;
bool buzzerCurrentlyOn = false;

// Variáveis para ajuste de thresholds dinâmicos
bool adjustedThresholds = false;
const float ORIGINAL_DETECTION_THRESHOLD = 0.40f;
const float ORIGINAL_AWAKE_THRESHOLD = 0.35f;
int adaptationCycles = 0; // Para controle de ciclos de adaptação
const float MIN_THRESHOLD = 0.25f; // Aumentado para evitar detecções falsas de baixa confiança

// Constantes para EMA e limites de threshold
const float EMA_ALPHA = 0.08f;  // Fator de suavização (valor conservador)
const float MIN_THRESHOLD_VALUE = 0.30f; // Valor mínimo permitido para threshold
const float MAX_THRESHOLD_VALUE = 0.70f; // Valor máximo permitido para threshold

// Constante para filtragem de caixas pequenas
const float MIN_BOX_AREA_RATIO = 0.01f; // 1% da área total da imagem (valor conservador)

// Constantes e variáveis para sistema de voting (janela deslizante)
const int WINDOW_SIZE = 8;  // Tamanho conservador para começar
const int VOTE_THRESHOLD = 6; // Aumentado para 6 para exigir mais votos de sonolência
bool drowsyStateBuffer[WINDOW_SIZE] = {false}; // Buffer circular para estados
int bufferIndex = 0; // Índice atual no buffer circular

// Constantes para supressão pós-alerta adaptativa
const unsigned long BASE_COOLDOWN_CRITICAL = 10000; // 10s base após alerta crítico
const float CONSECUTIVE_COOLDOWN_FACTOR = 1.5f; // Fator multiplicador por detecção consecutiva

// Constantes para calibração inicial
const unsigned long CALIBRATION_PERIOD = 15000; // 15s de período de calibração
bool calibrationActive = false;
unsigned long calibrationStartTime = 0;
float calibrationConfidenceSum = 0.0f;
int calibrationSampleCount = 0;

// Variável global para o buffer temporário
camera_fb_t *fb = NULL;
static camera_config_t camera_config;

// Protótipos das funções - DEVE ESTAR AQUI
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);
int ei_camera_get_data(size_t offset, size_t length, float *out_ptr);
void activate_alerts(const char *type);
void update_alert_level(const char *type);
void deactivate_alerts();
void manageAlertTimeout();
void printMemoryInfo();
void resetSystem();
void play_buzzer_pattern(AlertLevel level);
void checkAndAdjustLuminosity();
void updateDrowsyStateBuffer(bool isDrowsy);
int countVotesInBuffer();
void startCalibration();
void processCalibrationData(float confidence);
void adaptCooldownPeriod();
void handleCapture();
void setupPWM();
void setFlashIntensity(uint32_t duty);
void captureDebugFrame();

// Funções auxiliares - DEVE ESTAR DEFINIDAS ANTES DE setup() ou loop() SE CHAMADAS DIRETAMENTE
// Função constrainFloat personalizada para evitar problemas de compatibilidade
float constrainFloat(float value, float min_val, float max_val) {
  if (value < min_val) return min_val;
  if (value > max_val) return max_val;
  return value;
}


void setup() {
  // Desativar detector de brownout
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  
  // Inicializar comunicação serial
  Serial.begin(115200);
  Serial.println("\n\n[INFO] Iniciando SnorLess - Sistema de Detecção de Sonolência");
  Serial.println("[INFO] Versão 1.1.0 - Melhorias de Captura");
  Serial.println("===================================");
  
  // Configurar pinos de saída
  pinMode(FLASH_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Configurar PWM para o flash (intensidade variável de alerta)
  setupPWM();
  setFlashIntensity(0); // Começar desligado
  
  // Desativar alertas inicialmente
  digitalWrite(FLASH_LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_BUILTIN, HIGH); // LED interno é ativo em LOW
  
  // Alocar o buffer de imagem uma única vez
  size_t buf_size = EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * 3;
  
  // Verificar se PSRAM está disponível
  if (psramFound()) {
    Serial.println("[INFO] PSRAM encontrada. Usando para alocação do buffer.");
    snapshot_buf = (uint8_t*)ps_calloc(1, buf_size);
  } else {
    Serial.println("[INFO] PSRAM não encontrada. Usando memória normal.");
    snapshot_buf = (uint8_t*)calloc(1, buf_size);
  }
  
  if (snapshot_buf == NULL) {
    Serial.println("[ERROR] Falha na alocação do buffer de imagem!");
    while (1) {
      // Piscar LED para indicar erro fatal
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(300);
    }
  }
  
  // Inicializar a câmera com a configuração padrão para inferência
  // A configuração para web capture será aplicada temporariamente em handleCapture
  if (initCamera()) { // initCamera usa a camera_config global
    Serial.println("[INFO] Sistema inicializado. Começando monitoramento...");
  } else {
    Serial.println("[ERROR] Falha na inicialização. Reinicie o dispositivo.");
     // Considerar um loop infinito de erro ou reset aqui
     while(1) { delay(1000); } // Exemplo: Trava o sistema em caso de falha crítica
  }

  // Imprimir informações de memória
  printMemoryInfo();

  // Registrar tempo inicial
  lastSuccessfulInference = millis();
  
  delay(1000);

  // Configurar Wi-Fi Access Point
  Serial.printf("[INFO] Setting up WiFi Access Point '%s'\n", ssid);
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.printf("[INFO] AP IP address: %s\n", IP.toString().c_str());

  // Configurar Web Server
  server.on("/capture", handleCapture); // Rota para captura de imagem
  server.begin();
  Serial.println("[INFO] Web Server started on port 80");
}

void loop() {
    // Processar requisições do Web Server
    server.handleClient(); // Não deve bloquear por muito tempo

    // Gerenciar timeout de alertas
    manageAlertTimeout();
    
    // Verificar e ajustar thresholds se necessário
    checkAndAdjustThresholds();
    
    // Verificar e ajustar luminosidade
    checkAndAdjustLuminosity();
    
    // Verificar se é hora de exibir o status do sistema
    if (millis() - lastStatusPrint >= STATUS_PRINT_INTERVAL) {
        printSystemStatus();
        printMemoryInfo();
        lastStatusPrint = millis();
    }
    
    // Verificar estado da câmera periodicamente
    checkCameraStatus();
    
    // Calcular FPS
    if (lastInferenceTime > 0) {
        unsigned long elapsedTime = millis() - lastInferenceTime;
        if (elapsedTime > 0) {
            fps = 1000.0 / elapsedTime;
            fpsHistory[fpsHistoryIndex] = fps;
            fpsHistoryIndex = (fpsHistoryIndex + 1) % FPS_AVERAGING_WINDOW;
        }
    }
    lastInferenceTime = millis();
    
    // Gerenciar o blink do LED flash durante alertas
    if (alert_active) {
        unsigned long currentMillis = millis();
        if (currentMillis - previousMillis >= blinkInterval) {
            previousMillis = currentMillis;
            flashState = !flashState;
            digitalWrite(FLASH_LED_PIN, flashState);
        }
    }
    
    // Serial.println("Iniciando captura..."); // Removido ou tornado debug
    
    try {
        // Processar a imagem e fazer a classificação
        processImage();
        // Serial.println("Processamento concluído"); // Removido o tornado debug
    } catch (...) {
        Serial.println("[ERROR] Exceção capturada durante processamento da imagem");
        failureCount++;
        
        if (failureCount >= MAX_FAILURE_COUNT) {
            Serial.println("[ERROR] Muitas falhas consecutivas. Reiniciando o sistema.");
            resetSystem();
        }
    }
    
    // Aguardar antes da próxima inferência
    delay(DELAY_BETWEEN_INFERENCES);
}

// Implementações das funções auxiliares - DEVE ESTAR DEPOIS DE setup() e loop()

// Configura o PWM para o flash usando funções nativas do ESP-IDF
void setupPWM() {
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = (ledc_timer_bit_t)PWM_RESOLUTION,
        .timer_num = LEDC_TIMER_0,  // Alterado para LEDC_TIMER_0
        .freq_hz = PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);
    
    ledc_channel_config_t ledc_channel = {
        .gpio_num = FLASH_LED_PIN,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = (ledc_channel_t)PWM_CHANNEL,
        .timer_sel = LEDC_TIMER_0,  // Alterado para LEDC_TIMER_0
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ledc_channel);
}

// Função para definir a intensidade do flash
void setFlashIntensity(uint32_t duty) {
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)PWM_CHANNEL, duty);
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)PWM_CHANNEL);
}

void resetSystem() {
  Serial.println("[WARN] Reiniciando o sistema...");
  delay(500);
  ESP.restart();
}

void printMemoryInfo() {
  uint32_t freeHeap = ESP.getFreeHeap();
  uint32_t totalHeap = ESP.getHeapSize();
  uint32_t freePsram = ESP.getFreePsram();
  uint32_t totalPsram = ESP.getPsramSize();
  
  Serial.println("\n[INFO] Informações de Memória");
  Serial.printf("[INFO] Heap - Livre: %d KB / Total: %d KB (%.1f%%)\n", 
                freeHeap / 1024, totalHeap / 1024, 
                100.0 * freeHeap / totalHeap);
  
  if (totalPsram > 0) {
    Serial.printf("[INFO] PSRAM - Livre: %d KB / Total: %d KB (%.1f%%)\n", 
                  freePsram / 1024, totalPsram / 1024,
                  100.0 * freePsram / totalPsram);
  } else {
    Serial.println("[INFO] PSRAM não disponível");
  }
  Serial.println("==============================");
}

bool initCamera() {
  Serial.println("[INFO] Inicializando câmera...");
  
  // Tentar inicializar a câmera algumas vezes
  for (int attempt = 0; attempt < 3; attempt++) {
    if (ei_camera_init()) {
      Serial.println("[INFO] Câmera inicializada com sucesso!");
      
      // Testar uma captura para verificar funcionamento
      camera_fb_t* test_fb = esp_camera_fb_get(); // Usar fb local para o teste
      if (test_fb) {
        Serial.printf("[INFO] Teste de captura: OK (%u bytes)\n", test_fb->len);
        esp_camera_fb_return(test_fb);
        test_fb = NULL;
        return true;
      } else {
        Serial.println("[WARN] Teste de captura falhou!");
        ei_camera_deinit();
      }
    }
    
    Serial.printf("[WARN] Tentativa %d de inicialização da câmera falhou. Aguardando antes de tentar novamente...\n", attempt + 1);
    delay(500);
  }
  
  Serial.println("[ERROR] Não foi possível inicializar a câmera após várias tentativas!");
  return false;
}

void checkCameraStatus() {
  // Verificar se a câmera está respondendo
  if (millis() - lastSuccessfulInference > WATCHDOG_TIMEOUT) {
    Serial.println("[WARN] Possível problema com a câmera. Tentando reinicializar...");
    
    // Desinicializar e reinicializar a câmera
    ei_camera_deinit();
    delay(500);
    
    if (initCamera()) {
      Serial.println("[INFO] Câmera reinicializada com sucesso!");
      failureCount = 0; // Resetar contador de falhas
    } else {
      Serial.println("[ERROR] Falha ao reinicializar a câmera!");
      failureCount++;
      
      if (failureCount >= MAX_FAILURE_COUNT) {
        Serial.println("[ERROR] Muitas falhas consecutivas. Reiniciando o sistema.");
        resetSystem();
      }
    }
    
    // Atualizar timestamp para evitar novas tentativas imediatas
    lastSuccessfulInference = millis();
  }
}

void printSystemStatus() {
  // Obter memória livre
  uint32_t freeHeap = ESP.getFreeHeap();
  uint32_t totalHeap = ESP.getHeapSize();
  
  // Calcular FPS
  float avgFps = 0;
  for (int i = 0; i < FPS_AVERAGING_WINDOW; i++) {
    avgFps += fpsHistory[i];
  }
  avgFps /= FPS_AVERAGING_WINDOW;
  
  // Exibir informações do sistema
  Serial.println("\n[INFO] Status do Sistema");
  Serial.printf("[INFO] Tempo em execução: %lu segundos\n", millis() / 1000);
  Serial.printf("[INFO] Memória livre: %d KB / %d KB (%.1f%%)\n", 
                freeHeap / 1024, totalHeap / 1024,
                (1.0 - (float)freeHeap / totalHeap) * 100.0);
  Serial.printf("[INFO] FPS médio: %.2f\n", avgFps);
  Serial.printf("[INFO] Total de inferências: %lu\n", totalInferences);
  Serial.printf("[INFO] Total de alertas: %lu\n", totalAlerts);
  Serial.printf("[INFO] Estado atual: %s\n", 
                currentState == STATE_AWAKE ? "Acordado" :
                currentState == STATE_NODDING ? "Cabeceando" :
                currentState == STATE_YAWNING ? "Bocejando" : "Desconhecido");
  Serial.printf("[INFO] Último alerta: %lu segundos atrás\n", 
                alertStartTime > 0 ? (millis() - alertStartTime) / 1000 : 0);
  Serial.printf("[INFO] Luminosidade atual: %d\n", lastLuminosity);
  Serial.println("=======================");
}

void processImage() {
    // Verificar se o buffer está inicializado
    if (snapshot_buf == NULL) {
        Serial.println("[ERROR] Buffer não inicializado");
        return;
    }

    // Limpar o buffer com zeros
    memset(snapshot_buf, 0, EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * 3);

    // Serial.println("Iniciando captura..."); // Remover ou tornar debug
    
    // Capturar imagem
    cam_capture_ok = ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf);
    
    if (!cam_capture_ok) {
        Serial.println("[WARN] Falha na captura da câmera");
        failureCount++;
        
        if (failureCount >= 3) {
            Serial.println("[ERROR] Múltiplas falhas de captura, reinicializando câmera...");
            ei_camera_deinit();
            delay(100);
            if (ei_camera_init()) {
                Serial.println("[INFO] Câmera reinicializada com sucesso.");
                failureCount = 0;
            }
        }
        return;
    }

    // Serial.println("Imagem capturada com sucesso"); // Remover ou tornar debug

    // Reset contador de falhas após captura bem-sucedida
    failureCount = 0;

    // Configurar sinal para o classificador
    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    // Serial.println("Executando classificador..."); // Remover ou tornar debug

    // Executar o classificador
    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);

    if (err != EI_IMPULSE_OK) {
        Serial.print("[ERROR] Falha ao executar o classificador: ");
        Serial.println(err);
        return;
    }

    // Serial.println("Classificação concluída"); // Remover ou tornar debug

    // Atualizar timestamp de inferência bem-sucedida
    lastSuccessfulInference = millis();

    // Processar os resultados da detecção
    DriverState detectedState = STATE_UNKNOWN;
    float awakeConfidence = 0.0;
    float noddingConfidence = 0.0;
    float yawningConfidence = 0.0;
    float maxConfidence = 0.0;

    // Serial.println("\n╔════════════════════════════════════════════════════════════╗");
    // Serial.println("║                    RESULTADOS DA INFERÊNCIA                 ║\n");
    // Serial.println("╠════════════════════════════════════════════════════════════╣");
    Serial.println("[INFO] Resultados da Inferência:");
    
    // Coletar confiança para cada classe
    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
        if (bb.value == 0) continue;
        
        // Calcular a área relativa da caixa e descartar caixas muito pequenas
        float boxAreaRatio = (bb.width * bb.height) / 
                            (float)(EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT);
        
        // Descartar caixas muito pequenas que podem ser ruído ou detecções falsas
        if (boxAreaRatio < MIN_BOX_AREA_RATIO) {
            Serial.printf("[DEBUG] Caixa descartada (muito pequena): %.2f%% da área total\n", boxAreaRatio * 100);
            continue;
        }
        
        if (bb.value > maxConfidence) {
            maxConfidence = bb.value;
            
            if (strcmp(bb.label, "awake") == 0) {
                detectedState = STATE_AWAKE;
                awakeConfidence = bb.value;
                // Serial.printf("║ Estado: ACORDADO     | Confiança: %.2f%%\n", bb.value * 100); // Será resumido abaixo
            }
            else if (strcmp(bb.label, "nodding") == 0) {
                detectedState = STATE_NODDING;
                noddingConfidence = bb.value;
                // Serial.printf("║ Estado: CABECEANDO   | Confiança: %.2f%%\n", bb.value * 100); // Será resumido abaixo
            }
            else if (strcmp(bb.label, "yawning") == 0) {
                detectedState = STATE_YAWNING;
                yawningConfidence = bb.value;
                // Serial.printf("║ Estado: BOCEJANDO    | Confiança: %.2f%%\n", bb.value * 100); // Será resumido abaixo
            }
             Serial.printf("[INFO] Detecção: %s | Confiança: %.2f%%\n", bb.label, bb.value * 100);
        }
    }
    
    // Serial.println("╠════════════════════════════════════════════════════════════╣");
    // Serial.printf("║ Confiança Máxima: %.2f%%\n", maxConfidence * 100); // Já incluído acima
    // Serial.println("╚════════════════════════════════════════════════════════════╝");

    // Verificar se temos um quadro "sem face"
    if (maxConfidence < EMPTY_FRAME_CONFIDENCE) {
        emptyFrameCount++;
        if (emptyFrameCount > EMPTY_FRAME_THRESHOLD) {
            if (faceDetected) {
                Serial.println("[WARN] ALERTA: FACE NÃO DETECTADA");
                Serial.println("[INFO] Nenhuma face detectada após detecção prévia. Verifique o posicionamento da câmera.");
                faceDetected = false;
                // Interromper qualquer calibração em andamento
                calibrationActive = false;
            }
            consecutiveAwakeCount = 0;
            consecutiveDrowsyCount = 0;
            // Limpar o buffer de voting quando perdemos a face
            for (int i = 0; i < WINDOW_SIZE; i++) {
                drowsyStateBuffer[i] = false;
            }
            return;
        }
    } else {
        emptyFrameCount = 0;
        if (!faceDetected) {
            Serial.println("[INFO] FACE DETECTADA");
            Serial.println("[INFO] Nova face detectada - Iniciando monitoramento.");
            faceDetected = true;
            
            // Iniciar calibração personalizada quando uma nova face é detectada
            startCalibration();
        }
        
        // Se estamos em calibração, processar os dados
        if (calibrationActive) {
            processCalibrationData(maxConfidence);
        }
    }
    
    // Atualizar variável lastFaceConfidence
    lastFaceConfidence = maxConfidence;
    
    // Atualizar contadores de estados consecutivos
    if (detectedState == STATE_AWAKE) {
        consecutiveAwakeCount++;
        // Resetar contador de sonolência imediatamente ao detectar estado acordado
        consecutiveDrowsyCount = 0;
        // Atualizar buffer circular - estado não sonolento
        updateDrowsyStateBuffer(false);
        Serial.printf("[INFO] Contador ACORDADO: %d/%d | Contador SONOLENTO: %d/%d\n", 
                     consecutiveAwakeCount, CONSECUTIVE_AWAKE_THRESHOLD,
                     consecutiveDrowsyCount, CONSECUTIVE_THRESHOLD);
    } 
    else if (detectedState == STATE_NODDING || detectedState == STATE_YAWNING) {
        // Verificar confiança antes de incrementar
        if ((detectedState == STATE_NODDING && noddingConfidence >= DETECTION_THRESHOLD) ||
            (detectedState == STATE_YAWNING && yawningConfidence >= DETECTION_THRESHOLD)) {
            // Limitar o contador máximo
            if (consecutiveDrowsyCount < MAX_CONSECUTIVE_DROWSY) {
                consecutiveDrowsyCount++;
            }
            // Atualizar buffer circular - estado sonolento
            updateDrowsyStateBuffer(true);
        }
        // Resetar contador de acordado imediatamente ao detectar estado sonolento com confiança
        if ((detectedState == STATE_NODDING && noddingConfidence >= DETECTION_THRESHOLD) ||
            (detectedState == STATE_YAWNING && yawningConfidence >= DETECTION_THRESHOLD)) {
             consecutiveAwakeCount = 0;
        }
        Serial.printf("[INFO] Contador ACORDADO: %d/%d | Contador SONOLENTO: %d/%d\n", 
                     consecutiveAwakeCount, CONSECUTIVE_AWAKE_THRESHOLD,
                     consecutiveDrowsyCount, CONSECUTIVE_THRESHOLD);
    }
    else {
        // Estado desconhecido - reduzir ambos os contadores gradualmente
        if (consecutiveDrowsyCount > 0) consecutiveDrowsyCount--;
        if (consecutiveAwakeCount > 0) consecutiveAwakeCount--;
        // Para estado desconhecido, não atualizamos o buffer de voting
        Serial.printf("[INFO] Contador ACORDADO: %d/%d | Contador SONOLENTO: %d/%d\n", 
                     consecutiveAwakeCount, CONSECUTIVE_AWAKE_THRESHOLD,
                     consecutiveDrowsyCount, CONSECUTIVE_THRESHOLD);
    }
    
    // Verificar votos no buffer circular como medida complementar
    int drowsyVotes = countVotesInBuffer();
    Serial.printf("[INFO] Votos sonolência: %d/%d (%.1f%%)\n", 
                 drowsyVotes, WINDOW_SIZE, (drowsyVotes * 100.0f) / WINDOW_SIZE);
    
    // Aplicar threshold para mudança de estado
    if (consecutiveAwakeCount >= CONSECUTIVE_AWAKE_THRESHOLD && currentState != STATE_AWAKE) {
        previousState = currentState;
        currentState = STATE_AWAKE;
        Serial.println("[INFO] MUDANÇA DE ESTADO: ACORDADO");
        Serial.println("[INFO] Motorista está acordado e atento. Desativando alertas.");
        deactivate_alerts();
    } 
    else if (consecutiveDrowsyCount >= CONSECUTIVE_THRESHOLD) {
        // Verificar confiança antes de ativar alerta
        if ((detectedState == STATE_NODDING && noddingConfidence >= DETECTION_THRESHOLD) ||
            (detectedState == STATE_YAWNING && yawningConfidence >= DETECTION_THRESHOLD)) {
            
            // Ativar alertas se atingir o threshold de sonolência
            if (currentState != detectedState) {
                previousState = currentState;
                currentState = detectedState;
                
                if (currentState == STATE_NODDING) {
                    Serial.println("[ALERT] ALERTA: CABECEANDO");
                    Serial.println("[ALERT] Motorista está cabeceando - Ativando alertas.");
                    update_alert_level("cabeceando");
                    totalAlerts++;
                } 
                else if (currentState == STATE_YAWNING) {
                    Serial.println("[ALERT] ALERTA: BOCEJANDO");
                    Serial.println("[ALERT] Motorista está bocejando - Ativando alertas.");
                    update_alert_level("bocejando");
                    totalAlerts++;
                }
            }
            // Continuar emitindo alertas se o estado persistir
            else if (!alert_active && !inAlertCooldown) {
                // Verificar se o contador está muito alto para aumentar a gravidade
                if (consecutiveDrowsyCount >= MAX_CONSECUTIVE_DROWSY) {
                    // Forçar progressão para alerta crítico
                    currentAlertLevel = ALERT_CRITICAL;
                    alertLevelStartTime = millis();
                    Serial.println("[ALERT] Estado persistente detectado - Elevando para ALERTA CRÍTICO!");
                    
                    // Aplicar cooldown adaptativo para alerta crítico
                    adaptCooldownPeriod();
                }
                // Implementação de lógica multi-classe avançada
                else if (consecutiveDrowsyCount >= 6) {
                    // Elevação automática para alerta crítico se a detecção persistir por tempo prolongado
                    currentAlertLevel = ALERT_CRITICAL;
                    alertLevelStartTime = millis();
                    Serial.println("[ALERT] Multi-classe avançada: Sonolência persistente - Elevando para ALERTA CRÍTICO!");
                    
                    // Aplicar cooldown adaptativo para alerta crítico
                    adaptCooldownPeriod();
                }
                
                if (currentState == STATE_NODDING) {
                    update_alert_level("cabeceando");
                } else {
                    update_alert_level("bocejando");
                }
            }
        }
    }
    // Verificação complementar usando o sistema de voting
    else if (drowsyVotes >= VOTE_THRESHOLD && !alert_active && !inAlertCooldown) {
        // Usar o voting system como complemento para detectar sonolência
        Serial.println("[ALERT] ALERTA: VOTING SYSTEM ATIVADO");
        Serial.printf("[ALERT] Detecção por janela deslizante: %d/%d votos\n", 
                     drowsyVotes, WINDOW_SIZE);
        
        // Usar informação de estado mais recente para o tipo de alerta
        if (detectedState == STATE_NODDING) {
            currentState = STATE_NODDING;
            update_alert_level("cabeceando");
        } else if (detectedState == STATE_YAWNING) {
            currentState = STATE_YAWNING;
            update_alert_level("bocejando");
        } else {
            // Se não temos estado atual claro, usar o estado anterior
            if (previousState == STATE_NODDING) {
                currentState = STATE_NODDING;
                update_alert_level("cabeceando");
            } else {
                currentState = STATE_YAWNING;
                update_alert_level("bocejando");
            }
        }
        totalAlerts++;
    }
    
    // Incrementar contador de inferências
    totalInferences++;
}

/**
 * @brief   Atualiza o nível de alerta com base na persistência da sonolência
 * 
 * @param   type  O tipo de sonolência detectada ("cabeceando" ou "bocejando")
 */
void update_alert_level(const char *type) {
    // Verificar se estamos em cooldown
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
    
    // Verificar se já temos um alerta ativo
    if (alert_active) {
        // Verificar tempo no nível atual para possível progressão
        unsigned long timeInCurrentLevel = millis() - alertLevelStartTime;
        
        // Progresso para níveis mais altos baseado no tempo de permanência no nível atual
        if (currentAlertLevel == ALERT_LOW && timeInCurrentLevel > TIME_TO_MEDIUM_ALERT) {
            currentAlertLevel = ALERT_MEDIUM;
            alertLevelStartTime = millis();
            Serial.println("[ALERT] Progredindo para ALERTA MÉDIO - Sonolência persistente!");
        } 
        else if (currentAlertLevel == ALERT_MEDIUM && timeInCurrentLevel > TIME_TO_HIGH_ALERT) {
            currentAlertLevel = ALERT_HIGH;
            alertLevelStartTime = millis();
            Serial.println("[ALERT] Progredindo para ALERTA ALTO - Sonolência grave!");
        }
        else if (currentAlertLevel == ALERT_HIGH && timeInCurrentLevel > TIME_TO_CRITICAL_ALERT) {
            currentAlertLevel = ALERT_CRITICAL;
            alertLevelStartTime = millis();
            Serial.println("[ALERT] Progredindo para ALERTA CRÍTICO - Situação de emergência!");
        }
        
        // Atualizar alertas para o nível atual
        activate_alerts(type);
    } 
    else {
        // Iniciar com alerta baixo se nenhum alerta estiver ativo
        currentAlertLevel = ALERT_LOW;
        alertLevelStartTime = millis();
        Serial.println("[ALERT] Iniciando ALERTA BAIXO - Primeiros sinais de sonolência");
        activate_alerts(type);
    }
}

/**
 * @brief   Ativa os alertas de sonolência com base no nível atual (LED e buzzer)
 * 
 * @param   type  O tipo de sonolência detectada ("cabeceando" ou "bocejando")
 */
void activate_alerts(const char *type) {
    // Se já existe um alerta ativo, apenas atualizar o padrão para o nível atual
    if (alert_active) {
        // Apenas ajustar os padrões para o nível atual
        switch (currentAlertLevel) {
            case ALERT_LOW:
                // Padrão básico - piscar lento
                blinkInterval = 500;
                setFlashIntensity(FLASH_INTENSITY_LOW);
                break;
            
            case ALERT_MEDIUM:
                // Piscar médio
                blinkInterval = 300;
                setFlashIntensity(FLASH_INTENSITY_MEDIUM);
                break;
            
            case ALERT_HIGH:
                // Piscar rápido
                blinkInterval = 150;
                setFlashIntensity(FLASH_INTENSITY_HIGH);
                break;
            
            case ALERT_CRITICAL:
                // Piscar muito rápido
                blinkInterval = 80;
                setFlashIntensity(FLASH_INTENSITY_CRITICAL);
                break;
                
            default:
                blinkInterval = 500; // Padrão
                setFlashIntensity(FLASH_INTENSITY_LOW);
                break;
        }
        return;
    }
    
    // Configurar o LED interno
    digitalWrite(LED_BUILTIN, LOW); // LED da placa é ativo em LOW
    
    // Configurar padrões com base no nível de alerta
    switch (currentAlertLevel) {
        case ALERT_LOW:
            // Alerta leve - piscar lento, tom baixo
            blinkInterval = 500;
            setFlashIntensity(FLASH_INTENSITY_LOW);
            if (strcmp(type, "cabeceando") == 0) {
                tone(BUZZER_PIN, 1000); // 1kHz
            } else {
                tone(BUZZER_PIN, 800); // 800Hz
            }
            break;
            
        case ALERT_MEDIUM:
            // Alerta médio - piscar médio, tom médio
            blinkInterval = 300;
            setFlashIntensity(FLASH_INTENSITY_MEDIUM);
            if (strcmp(type, "cabeceando") == 0) {
                tone(BUZZER_PIN, 1500); // 1.5kHz
            } else {
                tone(BUZZER_PIN, 1200); // 1.2kHz
            }
            break;
            
        case ALERT_HIGH:
            // Alerta alto - piscar rápido, tom alto
            blinkInterval = 150;
            setFlashIntensity(FLASH_INTENSITY_HIGH);
            if (strcmp(type, "cabeceando") == 0) {
                tone(BUZZER_PIN, 2000); // 2kHz
            } else {
                tone(BUZZER_PIN, 1800); // 1.8kHz
            }
            break;
            
        case ALERT_CRITICAL:
            // Alerta crítico - piscar muito rápido, tom muito alto
            blinkInterval = 80;
            setFlashIntensity(FLASH_INTENSITY_CRITICAL);
            if (strcmp(type, "cabeceando") == 0) {
                play_buzzer_pattern(ALERT_CRITICAL); // Padrão SOS
            } else {
                tone(BUZZER_PIN, 2500); // 2.5kHz
            }
            break;
            
        default:
            // Padrão de segurança
            blinkInterval = 500;
            setFlashIntensity(FLASH_INTENSITY_LOW);
            tone(BUZZER_PIN, 1000);
            break;
    }
    
    alert_active = true;
    alertStartTime = millis();
    lastAlertTime = millis();
    
    Serial.printf("[ALERT] Alerta de %s nível %d ativado!\n", type, currentAlertLevel);
}

/**
 * @brief   Toca padrões complexos no buzzer para alertas críticos
 * 
 * @param   level  Nível de alerta
 */
void play_buzzer_pattern(AlertLevel level) {
  unsigned long currentMillis = millis();
  
  // Inicializar padrão
  if (currentBuzzerPattern == 0) {
    if (level == ALERT_CRITICAL) {
      // Padrão SOS (... --- ...) - 3 curtos, 3 longos, 3 curtos
      buzzerPatternSteps = 9; // Total de passos no padrão (tom/sem tom)
      currentBuzzerPattern = 1;
      lastBuzzerPatternTime = currentMillis;
      tone(BUZZER_PIN, 2500); // Iniciar com tom
      buzzerCurrentlyOn = true;
    }
  } else {
    // Timing para padrão SOS
    unsigned long stepDuration = 200; // Duração base do passo em ms (curto)
    if (currentBuzzerPattern > 3 && currentBuzzerPattern <= 6) {
        stepDuration = 500; // Duração do passo longo (para o 'O')
    }
    
    if (currentMillis - lastBuzzerPatternTime > stepDuration) {
        lastBuzzerPatternTime = currentMillis;
        
        // Avançar no padrão
        currentBuzzerPattern++;
        if (currentBuzzerPattern > buzzerPatternSteps) {
          currentBuzzerPattern = 1; // Reiniciar padrão
        }
        
        // Lógica para ligar/desligar o buzzer baseado no passo do padrão SOS
        // Passos 1, 2, 3 (S), 4, 5, 6 (O), 7, 8, 9 (S) com alternância on/off
        bool shouldBeOn = false;
        if ((currentBuzzerPattern >= 1 && currentBuzzerPattern <= 3) || 
            (currentBuzzerPattern >= 7 && currentBuzzerPattern <= 9)) { // Partes do S
            if (currentBuzzerPattern % 2 != 0) shouldBeOn = true; // Nos passos ímpares do S (1, 3, 7, 9) o buzzer liga, 2 e 8 desliga
        } else if (currentBuzzerPattern >= 4 && currentBuzzerPattern <= 6) { // Partes do O
             if (currentBuzzerPattern % 2 == 0) shouldBeOn = true; // Nos passos pares do O (4, 6) o buzzer liga, 5 desliga
        }
        
        if (shouldBeOn && !buzzerCurrentlyOn) {
            tone(BUZZER_PIN, (currentBuzzerPattern >=4 && currentBuzzerPattern <= 6) ? 2000 : 2500); // Tom diferente para o 'O'
            buzzerCurrentlyOn = true;
        } else if (!shouldBeOn && buzzerCurrentlyOn) {
            noTone(BUZZER_PIN);
            buzzerCurrentlyOn = false;
        }
    }
  }
}

/**
 * @brief   Desativa os alertas de sonolência
 */
void deactivate_alerts() {
    if (alert_active) {
        setFlashIntensity(0); // Desligar o flash
        digitalWrite(LED_BUILTIN, HIGH);  // LED da placa apagado (HIGH)
        noTone(BUZZER_PIN);
        
        // Se estávamos em alerta crítico, aplicar cooldown adaptativo
        if (currentAlertLevel == ALERT_CRITICAL) {
            adaptCooldownPeriod();
        }
        
        alert_active = false;
        currentAlertLevel = ALERT_NONE;
        currentBuzzerPattern = 0;
        buzzerCurrentlyOn = false;
        
        // Iniciar cooldown
        inAlertCooldown = true;
        lastAlertTime = millis();
        
        Serial.println("[INFO] Alertas desativados. Iniciando período de cooldown.");
    }
}

/**
 * @brief   Gerencia o timeout de alertas para evitar que fiquem ativos por muito tempo
 */
void manageAlertTimeout() {
    if (alert_active) {
        // Verificar se o alerta já está ativo por tempo demais
        unsigned long alertDuration = 0;
        switch (currentAlertLevel) {
            case ALERT_LOW: alertDuration = ALERT_DURATION_LOW; break;
            case ALERT_MEDIUM: alertDuration = ALERT_DURATION_MEDIUM; break;
            case ALERT_HIGH: alertDuration = ALERT_DURATION_HIGH; break;
            case ALERT_CRITICAL: alertDuration = ALERT_DURATION_CRITICAL; break;
            default: alertDuration = ALERT_DURATION_NONE;
        }
        
        if (millis() - alertStartTime > alertDuration) {
            Serial.println("[INFO] Tempo máximo de alerta atingido. Desativando temporariamente.");
            deactivate_alerts();
        }
        
        // Atualizar padrões de buzzer para alertas críticos
        if (currentAlertLevel == ALERT_CRITICAL) {
            play_buzzer_pattern(ALERT_CRITICAL);
        }
    }
}

/**
 * @brief   Inicializa o sensor de imagem e inicia o streaming
 * 
 * @retval  false se a inicialização falhar
 */
bool ei_camera_init(void) {
    if (is_initialised) return true;

    // Configuração da câmera
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

    // Inicializar a câmera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        Serial.printf("[ERROR] Falha na inicialização da câmera, erro 0x%x\n", err);
        return false;
    }

    sensor_t * s = esp_camera_sensor_get();
    if (s == NULL) {
        Serial.println("[ERROR] Não foi possível obter o sensor da câmera");
        return false;
    }

    // Ajustes básicos do sensor
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

/**
 * @brief   Interrompe o streaming do sensor de imagem
 */
void ei_camera_deinit(void) {
  // Garantir que não há frames pendentes
  if (fb) {
    esp_camera_fb_return(fb);
    fb = NULL;
  }
  
  // Desinicializar a câmera
  if (is_initialised) {
    esp_err_t err = esp_camera_deinit();
    
    if (err != ESP_OK) {
      Serial.println("[WARN] Falha ao desinicializar a câmera");
    }
    
    is_initialised = false;
  }
}

/**
 * @brief   Captura, redimensiona e recorta imagem com melhorias
 */
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
    if (!is_initialised) {
        Serial.println("[ERROR] Câmera não inicializada");
        return false;
    }

    // Liberar buffer anterior se existir
    if (fb) {
        esp_camera_fb_return(fb);
        fb = NULL;
    }

    // Obter um único frame
    fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("[WARN] Falha na captura da câmera");
        return false;
    }

    // Converter para RGB888
    bool converted = fmt2rgb888(fb->buf, fb->len, fb->format, snapshot_buf);
    
    // Liberar o frame buffer
    esp_camera_fb_return(fb);
    fb = NULL;

    if (!converted) {
        Serial.println("[ERROR] Falha na conversão para RGB888");
        return false;
    }

    // Redimensionar usando o Edge Impulse
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

/**
 * @brief   Obtém dados da imagem para o Edge Impulse
 */
int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
  if (snapshot_buf == NULL || !cam_capture_ok) {
    // Se o buffer não estiver pronto, preencher com zeros
    for (size_t i = 0; i < length; i++) {
      out_ptr[i] = 0.0f;
    }
    return 0;
  }
  
  // Já temos um buffer RGB888, então recalcular o offset para o índice de pixel
  size_t pixel_ix = offset * 3;
  size_t pixels_left = length;
  size_t out_ptr_ix = 0;
  
  // Verificar limites para evitar acesso fora dos limites
  if (pixel_ix >= EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE) {
    return 0;
  }
  
  size_t max_pixels = (EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE - pixel_ix) / 3;
  // Usar operador ternário em vez de min()
  pixels_left = (pixels_left < max_pixels) ? pixels_left : max_pixels;
  
  while (pixels_left != 0) {
    // Obter valores RGB888
    uint8_t r = snapshot_buf[pixel_ix];
    uint8_t g = snapshot_buf[pixel_ix + 1];
    uint8_t b = snapshot_buf[pixel_ix + 2];
    
    // Converter para o formato esperado pelo modelo (r << 16 + g << 8 + b)
    float pixel_f = (r << 16) + (g << 8) + b;
    out_ptr[out_ptr_ix] = pixel_f;
    
    out_ptr_ix++;
    pixel_ix += 3;
    pixels_left--;
  }
  
  return 0;
}

// Função para capturar um frame de debug quando a detecção falha repetidamente
void captureDebugFrame() {
  if (noDetectionCount >= 10 && debugFrameCount < MAX_DEBUG_FRAMES) {
    camera_fb_t* debugFrame = esp_camera_fb_get();
    if (debugFrame) {
      Serial.println("\n==== FRAME DE DEBUG ====");
      Serial.printf("Tamanho: %d bytes, Format: %d, Resolução: %dx%d\n", 
                    debugFrame->len, debugFrame->format, 
                    debugFrame->width, debugFrame->height);
      Serial.println("Primeiros 32 bytes:");
      
      // Mostrar os primeiros bytes da imagem para debug
      for (int i = 0; i < 32 && i < debugFrame->len; i++) {
        Serial.printf("%02X ", debugFrame->buf[i]);
        if ((i + 1) % 8 == 0) Serial.println();
      }
      
      // Calcular luminosidade média para diagnóstico
      uint16_t *rgb565_buf = (uint16_t *)debugFrame->buf;
      const int SAMPLE_PIXELS = 100; // Amostrar apenas alguns pixels para eficiência
      const int pixelStep = debugFrame->len / (SAMPLE_PIXELS * 2);
      int totalBrightness = 0;
      
      for (int p = 0; p < SAMPLE_PIXELS && p * pixelStep < debugFrame->len / 2; p++) {
        uint16_t pixel = rgb565_buf[p * pixelStep];
        // Extrair componentes RGB565
        uint8_t r = (pixel >> 11) & 0x1F;
        uint8_t g = (pixel >> 5) & 0x3F;
        uint8_t b = pixel & 0x1F;
        
        // Estimativa simplificada de luminosidade
        int brightness = (r * 21 + g * 72 + b * 7) / 100;
        totalBrightness += brightness;
      }
      
      int avgBrightness = totalBrightness / SAMPLE_PIXELS;
      Serial.printf("Luminosidade média estimada: %d/255\n", avgBrightness);
      Serial.println("==========================");
      
      esp_camera_fb_return(debugFrame);
      debugFrameCount++;
    }
  }
}

void checkAndAdjustThresholds() {
  // Verificar a cada 30 segundos
  static unsigned long lastAdjustTime = 0;
  const unsigned long ADJUST_INTERVAL = 15000; // Reduzido para 15 segundos
  
  if (millis() - lastAdjustTime < ADJUST_INTERVAL) {
    return;
  }
  
  lastAdjustTime = millis();
  
  // Se há muitas inferências sem detecção, ajustar thresholds temporariamente
  if (noDetectionCount > 10 && !adjustedThresholds) {
    adaptationCycles++;
    
    // Reduzir progressivamente os thresholds
    float reductionFactor = 0.60 - (adaptationCycles * 0.10);
    if (reductionFactor < 0.3) reductionFactor = 0.3;
    
    // Calcular thresholds alvo
    float targetDetectionThreshold = ORIGINAL_DETECTION_THRESHOLD * reductionFactor;
    float targetAwakeThreshold = ORIGINAL_AWAKE_THRESHOLD * reductionFactor;
    
    // Aplicar EMA para suavizar transição
    DETECTION_THRESHOLD = EMA_ALPHA * targetDetectionThreshold + (1 - EMA_ALPHA) * DETECTION_THRESHOLD;
    AWAKE_THRESHOLD = EMA_ALPHA * targetAwakeThreshold + (1 - EMA_ALPHA) * AWAKE_THRESHOLD;
    
    // Garantir limites mínimos e máximos
    DETECTION_THRESHOLD = constrainFloat(DETECTION_THRESHOLD, MIN_THRESHOLD_VALUE, MAX_THRESHOLD_VALUE);
    AWAKE_THRESHOLD = constrainFloat(AWAKE_THRESHOLD, MIN_THRESHOLD_VALUE, MAX_THRESHOLD_VALUE);
    
    // Garantir compatibilidade com limites mínimos existentes
    if (DETECTION_THRESHOLD < MIN_THRESHOLD) DETECTION_THRESHOLD = MIN_THRESHOLD;
    if (AWAKE_THRESHOLD < MIN_THRESHOLD) AWAKE_THRESHOLD = MIN_THRESHOLD;
    
    adjustedThresholds = true;
    
    Serial.println("\n=== AJUSTE SUAVIZADO DE SENSIBILIDADE ===");
    Serial.printf("Ciclo de adaptação %d: Thresholds = %.2f, %.2f\n", 
                 adaptationCycles, AWAKE_THRESHOLD, DETECTION_THRESHOLD);
  }
  // Se começamos a detectar bem, podemos voltar aos thresholds normais
  else if (noDetectionCount < 3 && adjustedThresholds) {
    // Aplicar EMA para suavizar transição de volta aos valores originais
    DETECTION_THRESHOLD = EMA_ALPHA * ORIGINAL_DETECTION_THRESHOLD + (1 - EMA_ALPHA) * DETECTION_THRESHOLD;
    AWAKE_THRESHOLD = EMA_ALPHA * ORIGINAL_AWAKE_THRESHOLD + (1 - EMA_ALPHA) * AWAKE_THRESHOLD;
    
    // Verificar se estamos muito próximos dos valores originais para considerar restaurado
    if (abs(DETECTION_THRESHOLD - ORIGINAL_DETECTION_THRESHOLD) < 0.05f &&
        abs(AWAKE_THRESHOLD - ORIGINAL_AWAKE_THRESHOLD) < 0.05f) {
      adjustedThresholds = false;
      adaptationCycles = 0;
    
      Serial.println("\n=== RESTAURAÇÃO DE SENSIBILIDADE ===");
      Serial.printf("Thresholds restaurados: Acordado=%.2f, Sonolento=%.2f\n", 
                    AWAKE_THRESHOLD, DETECTION_THRESHOLD);
    } else {
      Serial.println("\n=== RESTAURAÇÃO SUAVIZADA EM PROGRESSO ===");
      Serial.printf("Thresholds em ajuste: Acordado=%.2f, Sonolento=%.2f\n", 
                    AWAKE_THRESHOLD, DETECTION_THRESHOLD);
    }
  }
  
  // Adicionar verificação de detecção persistente de bocejo
  if (currentState == STATE_YAWNING && consecutiveDrowsyCount > 5) {
    // Se estamos detectando muito bocejo, aumentar o threshold para evitar falsos positivos
    // Aplicar EMA para suavizar transição
    float targetThreshold = ORIGINAL_DETECTION_THRESHOLD * 1.2f;
    DETECTION_THRESHOLD = EMA_ALPHA * targetThreshold + (1 - EMA_ALPHA) * DETECTION_THRESHOLD;
    
    // Garantir limite máximo
    if (DETECTION_THRESHOLD > MAX_THRESHOLD_VALUE) DETECTION_THRESHOLD = MAX_THRESHOLD_VALUE;
    
    Serial.println("\n=== AJUSTE SUAVIZADO PARA EVITAR FALSOS POSITIVOS DE BOCEJO ===");
    Serial.printf("Threshold aumentado para: %.2f\n", DETECTION_THRESHOLD);
  }
}

void checkAndAdjustLuminosity() {
    if (millis() - lastLuminosityCheck < LUMINOSITY_CHECK_INTERVAL) {
        return;
    }
    
    lastLuminosityCheck = millis();
    
    Serial.println("\n[INFO] Verificação de Luminosidade...");
    // Serial.println("Capturando imagem para análise..."); // Pode manter como debug se necessário
    
    // Capturar uma imagem para análise
    camera_fb_t* test_fb = esp_camera_fb_get();
    if (!test_fb) {
        Serial.println("[ERROR] Falha ao capturar imagem para análise de luminosidade");
        return;
    }
    
    // Serial.printf("Imagem capturada: %dx%d, formato: %d\n", 
    //              test_fb->width, test_fb->height, test_fb->format); // Debug
    
    // Converter para RGB888
    uint8_t* rgb888 = (uint8_t*)malloc(test_fb->width * test_fb->height * 3);
    if (!rgb888) {
        Serial.println("[ERROR] Falha ao alocar buffer para análise de luminosidade");
        esp_camera_fb_return(test_fb);
        return;
    }
    
    if (!fmt2rgb888(test_fb->buf, test_fb->len, test_fb->format, rgb888)) {
        Serial.println("[ERROR] Falha na conversão para RGB888");
        free(rgb888);
        esp_camera_fb_return(test_fb);
        return;
    }
    
    // Calcular luminosidade média
    const int SAMPLE_PIXELS = 100;
    const int pixelStep = (test_fb->width * test_fb->height) / SAMPLE_PIXELS;
    int totalBrightness = 0;
    // int minBrightness = 255; // Não usado na lógica, pode remover se não for para debug
    // int maxBrightness = 0; // Não usado na lógica, pode remover se não for para debug
    
    // Serial.println("Analisando luminosidade..."); // Debug
    
    for (int i = 0; i < SAMPLE_PIXELS; i++) {
        int pixelIndex = i * pixelStep * 3;
        uint8_t r = rgb888[pixelIndex];
        uint8_t g = rgb888[pixelIndex + 1];
        uint8_t b = rgb888[pixelIndex + 2];
        
        // Fórmula de luminosidade mais precisa (Y = 0.2126R + 0.7152G + 0.0722B)
        int brightness = (r * 21 + g * 72 + b * 7) / 100;
        totalBrightness += brightness;
        
        // if (brightness < minBrightness) minBrightness = brightness; // Não usado na lógica
        // if (brightness > maxBrightness) maxBrightness = brightness; // Não usado na lógica
    }
    
    int avgBrightness = totalBrightness / SAMPLE_PIXELS;
    lastLuminosity = avgBrightness;
    
    // Serial.printf("Luminosidade - Média: %d, Mín: %d, Máx: %d\n", 
    //              avgBrightness, minBrightness, maxBrightness); // Debug
    
    // Liberar recursos
    free(rgb888);
    esp_camera_fb_return(test_fb);
    
    // Obter sensor para ajustes
    sensor_t * s = esp_camera_sensor_get();
    if (!s) {
        Serial.println("[ERROR] Não foi possível obter o sensor para ajuste de luminosidade");
        return;
    }
    
    // Obter configurações atuais
    // int currentBrightness = s->status.brightness; // Debug
    // int currentContrast = s->status.contrast; // Debug
    // int currentGain = s->status.agc_gain; // Debug
    // int currentAEC = s->status.aec_value; // Debug
    
    // Serial.println("\n[INFO] Configurações atuais:"); // Debug
    // Serial.printf("  Brilho: %d\n", currentBrightness); // Debug
    // Serial.printf("  Contraste: %d\n", currentContrast); // Debug
    // Serial.printf("  Ganho: %d\n", currentGain); // Debug
    // Serial.printf("  AEC: %d\n", currentAEC); // Debug
    
    // Ajustar configurações com base na luminosidade
    if (avgBrightness < LUMINOSITY_THRESHOLD_LOW) {
        // Ambiente escuro
        s->set_brightness(s, 2);
        s->set_contrast(s, 2);
        s->set_gainceiling(s, GAINCEILING_8X);
        s->set_aec_value(s, 1200);
        s->set_agc_gain(s, 5);
        Serial.println("[INFO] Luminosidade: Ambiente escuro detectado. Ajustando configurações.");
    } 
    else if (avgBrightness > LUMINOSITY_THRESHOLD_HIGH) {
        // Ambiente muito claro
        s->set_brightness(s, -1);
        s->set_contrast(s, 0);
        s->set_gainceiling(s, GAINCEILING_2X);
        s->set_aec_value(s, 300);
        s->set_agc_gain(s, 0);
        Serial.println("[INFO] Luminosidade: Ambiente muito claro detectado. Ajustando configurações.");
    }
    else {
        // Ambiente com luminosidade adequada
        s->set_brightness(s, 0);
        s->set_contrast(s, 1);
        s->set_gainceiling(s, GAINCEILING_2X);
        s->set_aec_value(s, 600);
        s->set_agc_gain(s, 0);
        Serial.println("[INFO] Luminosidade: Adequada. Usando configurações padrão.");
    }
    
    luminosityAdjusted = true;
    Serial.println("[INFO] Fim da Verificação de Luminosidade\n");
}

void updateDrowsyStateBuffer(bool isDrowsy) {
    // Atualizar o buffer circular com o estado atual
    drowsyStateBuffer[bufferIndex] = isDrowsy;
    // Avançar o índice de forma circular
    bufferIndex = (bufferIndex + 1) % WINDOW_SIZE;
}

int countVotesInBuffer() {
    // Contar quantos estados de sonolência existem no buffer
    int drowsyVotes = 0;
    for (int i = 0; i < WINDOW_SIZE; i++) {
        if (drowsyStateBuffer[i]) {
            drowsyVotes++;
        }
    }
    return drowsyVotes;
}

void startCalibration() {
    // Iniciar o processo de calibração quando uma nova face é detectada
    calibrationActive = true;
    calibrationStartTime = millis();
    calibrationConfidenceSum = 0.0f;
    calibrationSampleCount = 0;
    
    Serial.println("\n[INFO] INICIANDO CALIBRAÇÃO PERSONALIZADA");
    Serial.printf("[INFO] Duração: %.1f segundos\n", CALIBRATION_PERIOD / 1000.0f);
    Serial.println("================================");
}

void processCalibrationData(float confidence) {
    // Processar dados durante a calibração
    if (!calibrationActive) return;
    
    // Acumular dados de confiança
    calibrationConfidenceSum += confidence;
    calibrationSampleCount++;
    
    // Verificar se o período de calibração terminou
    if (millis() - calibrationStartTime >= CALIBRATION_PERIOD) {
        // Calcular valor médio de confiança
        float avgConfidence = 0.0f;
        if (calibrationSampleCount > 0) {
            avgConfidence = calibrationConfidenceSum / calibrationSampleCount;
        }
        
        // Ajustar thresholds com base na calibração, de forma conservadora
        if (avgConfidence > 0.1f) {
            // Usar a confiança média da calibração para definir novos thresholds
            float newAwakeThreshold = avgConfidence * 0.90f; // 90% da confiança média de calibração
            float newDetectionThreshold = newAwakeThreshold + 0.05f; // 0.05 acima do awake threshold
            
            // Garantir que os novos thresholds estejam dentro dos limites definidos
            newAwakeThreshold = constrainFloat(newAwakeThreshold, MIN_THRESHOLD_VALUE, MAX_THRESHOLD_VALUE);
            newDetectionThreshold = constrainFloat(newDetectionThreshold, MIN_THRESHOLD_VALUE, MAX_THRESHOLD_VALUE);
            
            // Aplicar EMA para suavização na transição para os novos valores
            AWAKE_THRESHOLD = EMA_ALPHA * newAwakeThreshold + (1 - EMA_ALPHA) * AWAKE_THRESHOLD;
            DETECTION_THRESHOLD = EMA_ALPHA * newDetectionThreshold + (1 - EMA_ALPHA) * DETECTION_THRESHOLD;
            
            Serial.println("\n[INFO] CALIBRAÇÃO PERSONALIZADA CONCLUÍDA");
            Serial.printf("[INFO] Confiança média calibrada: %.2f%%\n", avgConfidence * 100);
            Serial.printf("[INFO] Novos thresholds - Detecção: %.2f, Awake: %.2f\n", 
                         DETECTION_THRESHOLD, AWAKE_THRESHOLD);
            Serial.println("================================");
        } else {
            Serial.println("\n[INFO] CALIBRAÇÃO INCONCLUSIVA");
            Serial.println("Dados insuficientes. Usando valores padrão.");
            Serial.println("================================");
        }
        
        // Encerrar calibração
        calibrationActive = false;
    }
}

void adaptCooldownPeriod() {
    // Ajustar período de cooldown com base na persistência da sonolência
    // Quanto mais detecções consecutivas, maior o cooldown para evitar alertas repetitivos
    
    // Usar o contador de sonolência para determinar o fator
    float factor = 1.0f;
    if (consecutiveDrowsyCount > 3) {
        // Aumentar o fator de forma gradual com base nas detecções consecutivas
        factor = 1.0f + ((consecutiveDrowsyCount - 3) * CONSECUTIVE_COOLDOWN_FACTOR * 0.1f);
        // Limitar o fator máximo para evitar valores extremos
        if (factor > 3.0f) factor = 3.0f;
    }
    
    // Aplicar o fator ao cooldown de alerta crítico
    unsigned long adaptedCooldown = (unsigned long)(BASE_COOLDOWN_CRITICAL * factor);
    
    // Atualizar o cooldown correspondente
    if (currentAlertLevel == ALERT_CRITICAL) {
        // Aqui aplicamos o cooldown adaptativo apenas para alertas críticos
        // Como uma medida de segurança, mantemos um valor mínimo
        ALERT_COOLDOWN_CRITICAL = max(adaptedCooldown, BASE_COOLDOWN_CRITICAL);
        
        Serial.printf("[INFO] Cooldown adaptativo: %.1f segundos (fator %.2f)\n", 
                     ALERT_COOLDOWN_CRITICAL / 1000.0f, factor);
    }
}

void handleCapture() {
  camera_fb_t *fb_web = NULL;
  
  // Capturar um frame com a mesma resolução da inferência (QVGA 320x240)
  // para evitar desinicializar/reinicializar a câmera constantemente
  
  // Não precisamos desinicializar e reinicializar se a resolução for a mesma
  // Apenas pegamos o frame atual
  fb_web = esp_camera_fb_get();
  
  if (!fb_web) {
    Serial.println("[ERROR] Falha na captura da imagem para Web Server");
    server.send(500, "text/plain", "Camera capture failed");
    return;
  }
  
  server.sendHeader("Content-Type", "image/jpeg");
  server.sendHeader("Content-Disposition", "inline; filename=capture.jpg");
  server.setContentLength(fb_web->len); // Define o tamanho do conteúdo
  server.send(200, "image/jpeg", ""); // Envia cabeçalhos e inicia a resposta
  server.client().write((const uint8_t*)fb_web->buf, fb_web->len); // Envia os dados da imagem
  
  esp_camera_fb_return(fb_web);
  
  // Não precisa restaurar configuração se não mudou
  Serial.println("[INFO] Imagem enviada via Web Server.");
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Modelo inválido para o sensor atual"
#endif 