# Drowsiness Sensor / Sensor de Sonolência

[English](#english) | [Português](#português)

## English

### Overview
Drowsiness Sensor is an ESP32-CAM based system that uses Edge Impulse machine learning to detect driver drowsiness through visual cues like nodding and yawning. The system provides progressive alerts through LED and buzzer to warn the driver.

### Features
- Real-time drowsiness detection using computer vision
- Progressive alert system with visual and audio feedback
- Automatic light adjustment for different environments
- Adaptive detection thresholds
- Debug mode for troubleshooting
- Memory optimization for ESP32-CAM

### Hardware Requirements
- ESP32-CAM AI-Thinker
- Buzzer (connected to GPIO12)
- Power supply (5V recommended)
- Optional: External LED for better visibility

### Technical Details
#### Image Processing Pipeline
1. Image Capture:
   - Resolution: QVGA (320x240)
   - Format: JPEG (for efficient storage)
2. Image Conversion:
   - Converted to RGB888 (3 channels, 8 bits per channel)
   - Resized to 96x96 pixels
3. Model Input:
   - Dimensions: 96x96 pixels
   - Format: RGB888
   - Channels: 3 (Red, Green, Blue)
   - Bits per channel: 8

### Edge Impulse Model
This project uses a custom trained model for drowsiness detection. To use the model:

1. Create an Edge Impulse account at [edgeimpulse.com](https://www.edgeimpulse.com)
2. Create a new project
3. Configure the input parameters:
   - Input size: 96x96 pixels
   - Color depth: RGB888 (3 channels, 8 bits per channel)
   - Processing: Raw RGB values
4. Train the model with three classes:
   - "awake": Normal driving position
   - "nodding": Head tilting forward
   - "yawning": Open mouth yawning
5. Deploy the model:
   - Select "Arduino Library" as deployment option
   - Download and extract to your project's root directory
   - Rename the folder to "snorless-1_inferencing"

### Installation

1. Install Required Libraries:
   - ESP32 board support in Arduino IDE
   - Edge Impulse SDK
   - ESP32 Camera library

2. Clone this repository:
```bash
git clone https://github.com/yourusername/drowsiness-sensor.git
```

3. Open the project in Arduino IDE

4. Install the Edge Impulse model:
   - Download your trained model from Edge Impulse
   - Place the model files in the `src` folder
   - Update the model path in the code if necessary

5. Configure the hardware:
   - Connect the buzzer to GPIO12
   - Ensure proper power supply
   - Position the camera correctly

6. Upload the code to your ESP32-CAM

### Usage
1. Power on the device
2. The system will initialize and start monitoring
3. Position yourself in front of the camera
4. The system will automatically detect drowsiness signs
5. Alerts will be triggered progressively:
   - Level 1: Slow blinking LED and low tone
   - Level 2: Medium blinking and medium tone
   - Level 3: Fast blinking and high tone
   - Level 4: Critical alert with SOS pattern

### Troubleshooting
- If the camera fails to initialize, check the power supply
- Ensure proper lighting conditions
- Check serial monitor for debug information
- Verify all connections are secure

## Português

### Visão Geral
O Sensor de Sonolência é um sistema baseado em ESP32-CAM que utiliza aprendizado de máquina do Edge Impulse para detectar sonolência do motorista através de sinais visuais como cabeceios e bocejos. O sistema fornece alertas progressivos através de LED e buzzer para avisar o motorista.

### Modelo Edge Impulse
Este projeto usa um modelo personalizado treinado para detecção de sonolência. Para usar o modelo:

1. Crie uma conta no Edge Impulse em [edgeimpulse.com](https://www.edgeimpulse.com)
2. Crie um novo projeto
3. Configure os parâmetros de entrada:
   - Tamanho da entrada: 96x96 pixels
   - Profundidade de cor: RGB888 (3 canais, 8 bits por canal)
   - Processamento: Valores RGB brutos
4. Treine o modelo com três classes:
   - "awake": Posição normal de direção
   - "nodding": Cabeça inclinando para frente
   - "yawning": Bocejo com boca aberta
5. Faça o deploy do modelo:
   - Selecione "Arduino Library" como opção de deploy
   - Baixe e extraia no diretório raiz do projeto
   - Renomeie a pasta para "snorless-1_inferencing"

### Características
- Detecção de sonolência em tempo real usando visão computacional
- Sistema de alerta progressivo com feedback visual e sonoro
- Ajuste automático de luz para diferentes ambientes
- Thresholds de detecção adaptativos
- Modo debug para solução de problemas
- Otimização de memória para ESP32-CAM

### Requisitos de Hardware
- ESP32-CAM AI-Thinker
- Buzzer (conectado ao GPIO12)
- Fonte de alimentação (5V recomendado)
- Opcional: LED externo para melhor visibilidade

### Detalhes Técnicos
#### Pipeline de Processamento de Imagem
1. Captura de Imagem:
   - Resolução: QVGA (320x240)
   - Formato: JPEG (para armazenamento eficiente)
2. Conversão de Imagem:
   - Convertida para RGB888 (3 canais, 8 bits por canal)
   - Redimensionada para 96x96 pixels
3. Entrada do Modelo:
   - Dimensões: 96x96 pixels
   - Formato: RGB888
   - Canais: 3 (Vermelho, Verde, Azul)
   - Bits por canal: 8

### Instalação

1. Instale as Bibliotecas Necessárias:
   - Suporte à placa ESP32 no Arduino IDE
   - SDK do Edge Impulse
   - Biblioteca da Câmera ESP32

2. Clone este repositório:
```bash
git clone https://github.com/yourusername/drowsiness-sensor.git
```

3. Abra o projeto no Arduino IDE

4. Instale o modelo Edge Impulse:
   - Baixe seu modelo treinado do Edge Impulse
   - Coloque os arquivos do modelo na pasta `src`
   - Atualize o caminho do modelo no código se necessário

5. Configure o hardware:
   - Conecte o buzzer ao GPIO12
   - Garanta alimentação adequada
   - Posicione a câmera corretamente

6. Faça upload do código para seu ESP32-CAM

### Uso
1. Ligue o dispositivo
2. O sistema inicializará e começará o monitoramento
3. Posicione-se em frente à câmera
4. O sistema detectará automaticamente sinais de sonolência
5. Alertas serão acionados progressivamente:
   - Nível 1: LED piscando lentamente e tom baixo
   - Nível 2: Piscada média e tom médio
   - Nível 3: Piscada rápida e tom alto
   - Nível 4: Alerta crítico com padrão SOS

### Solução de Problemas
- Se a câmera falhar ao inicializar, verifique a alimentação
- Garanta condições adequadas de iluminação
- Verifique o monitor serial para informações de debug
- Verifique se todas as conexões estão seguras 