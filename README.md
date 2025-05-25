# drowsinessSensor - Sistema de Detecção de Sonolência para Motoristas

[English](#english) | [Português](#português)

## Português

### ⚠️ Aviso Importante
**DISCLAIMER**: Este é um projeto de estudos em andamento. Não é um produto comercial ou certificado para uso em situações críticas. O uso deste sistema é por sua própria conta e risco. Os desenvolvedores não se responsabilizam por qualquer dano, acidente ou problema decorrente do uso deste sistema.

### Visão Geral
drowsinessSensor é um sistema de detecção de sonolência projetado para aumentar a segurança no trânsito, alertando motoristas quando sinais de sonolência são detectados. Utilizando um ESP32-CAM e um modelo de inteligência artificial treinado no Edge Impulse, o sistema monitora em tempo real sinais visuais como cabeceio e bocejo, emitindo alertas progressivos para prevenir acidentes.

### Características
- **Detecção em Tempo Real**: Monitoramento contínuo do estado de alerta do motorista
- **Alertas Progressivos**: Sistema de alerta em níveis crescentes de intensidade
- **Adaptação Automática**: Ajuste a diferentes condições de iluminação
- **Baixo Consumo**: Otimizado para dispositivos com recursos limitados
- **Fácil Instalação**: Configuração simples no painel do veículo
- **Interface WiFi**: Servidor web para ajuste e alinhamento da câmera
- **Configuração Remota**: Ajuste dos parâmetros via navegador web
- **Streaming em Tempo Real**: Visualização do feed da câmera para posicionamento

### Requisitos de Hardware
- ESP32-CAM (modelo AI-Thinker recomendado)
- Buzzer (conectado ao GPIO12)
- Fonte de alimentação de 5V
- Cabo USB para programação
- Opcional: LED externo para melhor visibilidade dos alertas
- Rede WiFi 2.4GHz para configuração inicial

### Documentação
- [Guia de Instalação](INSTALACAO.md)
- [Guia de Contribuição](CONTRIBUTING.md)
- [Registro de Mudanças](CHANGELOG.md)

### Arquivos Importantes
- `drowsinessSensor.ino`: Código principal do projeto
- `ei-snorless-1-arduino-1.0.7.zip`: Biblioteca do modelo de IA (importar via Arduino IDE)

## English

### ⚠️ Important Notice
**DISCLAIMER**: This is an ongoing study project. It is not a commercial or certified product for use in critical situations. The use of this system is at your own risk. The developers are not responsible for any damage, accident, or problem resulting from the use of this system.

### Overview
drowsinessSensor is a drowsiness detection system designed to enhance road safety by alerting drivers when signs of drowsiness are detected. Using an ESP32-CAM and an AI model trained on Edge Impulse, the system monitors visual cues such as nodding and yawning in real-time, issuing progressive alerts to prevent accidents.

### Features
- **Real-time Detection**: Continuous monitoring of the driver's alertness
- **Progressive Alerts**: Multi-level alert system with increasing intensity
- **Automatic Adaptation**: Adjustment to different lighting conditions
- **Low Power Consumption**: Optimized for resource-limited devices
- **Easy Installation**: Simple setup on the vehicle dashboard
- **Interface WiFi**: Web server for camera adjustment and alignment
- **Remote Configuration**: Adjustment of parameters via web browser
- **Real-time Streaming**: Visualization of camera feed for positioning

### Hardware Requirements
- ESP32-CAM (AI-Thinker model recommended)
- Buzzer (connected to GPIO12)
- 5V power supply
- USB cable for programming
- Optional: External LED for better alert visibility
- 2.4GHz WiFi network for initial configuration

### Documentation
- [Installation Guide](INSTALLATION.md)
- [Contribution Guide](CONTRIBUTING_EN.md)
- [Change Log](CHANGELOG.md)

### Important Files
- `drowsinessSensor.ino`: Main project code
- `ei-snorless-1-arduino-1.0.7.zip`: AI model library (import via Arduino IDE) 