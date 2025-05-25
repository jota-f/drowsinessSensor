# Guia de Instalação do drowsinessSensor

Este guia detalha como configurar e instalar o sistema drowsinessSensor de detecção de sonolência para motoristas.

## Requisitos de Hardware

- ESP32-CAM (modelo AI-Thinker recomendado)
- Buzzer (ativo, sem oscilador)
- Adaptador FTDI ou conversor USB-Serial para programação
- Fonte de alimentação de 5V/2A
- Jumpers/fios para conexão
- Opcional: Caixa protetora para ESP32-CAM

## Preparação do Ambiente Arduino

1. Baixe e instale a [Arduino IDE](https://www.arduino.cc/en/software)
2. Adicione o suporte ao ESP32:
   - Abra a Arduino IDE
   - Vá para Arquivo > Preferências
   - Em "URLs Adicionais de Gerenciadores de Placas", adicione:
     ```
     https://dl.espressif.com/dl/package_esp32_index.json
     ```
   - Clique em OK
   - Vá para Ferramentas > Placa > Gerenciador de Placas
   - Digite "ESP32" na busca
   - Instale "ESP32 by Espressif Systems"

## Configuração do Hardware

### Conexões para Programação

1. Conecte o ESP32-CAM ao adaptador FTDI/conversor USB-Serial:
   - ESP32-CAM GND → FTDI GND
   - ESP32-CAM 5V → FTDI VCC (5V)
   - ESP32-CAM U0R (TX) → FTDI RX
   - ESP32-CAM U0T (RX) → FTDI TX
   - ESP32-CAM IO0 → FTDI GND (apenas durante o upload)

2. Conecte o buzzer:
   - Buzzer (+) → ESP32-CAM IO12
   - Buzzer (-) → ESP32-CAM GND

### Diagrama de Conexão

```
┌─────────────┐                  ┌─────────────┐
│             │                  │             │
│   ESP32-CAM │                  │    FTDI     │
│             │                  │             │
│         GND ├──────────────────┤ GND         │
│          5V ├──────────────────┤ VCC (5V)    │
│         U0R ├──────────────────┤ RX          │
│         U0T ├──────────────────┤ TX          │
│         IO0 ├─┬────────────────┤             │
│         IO12├─┼────────────┐   │             │
│             │ │            │   │             │
└─────────────┘ │            │   └─────────────┘
                │            │
                │            │   ┌─────────────┐
                │            │   │             │
                │            └───┤ +   BUZZER  │
                └────────────────┤ -           │
                                 │             │
                                 └─────────────┘
```

**Nota:** A conexão entre IO0 e GND só é necessária durante o upload do código. Remova após o upload.

## Instalação do Software

1. Baixe o repositório para seu computador
2. Abra o arquivo `drowsinessSensor.ino` na Arduino IDE
3. Importe a biblioteca do Edge Impulse:
   - Vá para Sketch > Incluir Biblioteca > Adicionar Biblioteca .ZIP
   - Selecione o arquivo `ei-snorless-1-arduino-1.0.7.zip` incluído neste repositório

4. Configure a Arduino IDE:
   - Ferramentas > Placa > ESP32 Arduino > AI Thinker ESP32-CAM
   - Ferramentas > Porta > Selecione a porta COM do seu adaptador FTDI
   - Ferramentas > Partition Scheme > Huge APP (3MB No OTA/1MB SPIFFS)
   - Ferramentas > Upload Speed > 115200

5. Prepare o ESP32-CAM para upload:
   - Conecte o pino IO0 ao GND (modo de programação)
   - Pressione o botão RESET na placa ESP32-CAM
   - Clique em "Upload" na Arduino IDE
   - Após o upload, desconecte IO0 do GND
   - Pressione RESET novamente

## Posicionamento e Uso

1. Monte o ESP32-CAM em uma posição que capture o rosto do motorista:
   - Idealmente no painel, centralizado
   - A câmera deve estar voltada para o rosto, com visão clara
   - Evite luz forte direta na câmera
   - Distância recomendada: 30-60 cm do rosto

2. Conecte à alimentação (5V):
   - Recomendamos usar uma fonte de alimentação estável de 5V/2A
   - Ou conecte a uma porta USB do veículo com adaptador adequado

3. Ao ligar o sistema:
   - O LED vermelho acenderá durante a inicialização
   - Após alguns segundos, o sistema começará a monitorar
   - Não é necessário qualquer configuração adicional

## Solução de Problemas

- **LED pisca repetidamente e sistema não inicializa:**
  - Verifique a alimentação (5V estável)
  - Tente utilizar outra fonte de alimentação

- **Camera não inicializa:**
  - Pressione o botão RESET
  - Verifique a alimentação
  - Reconecte o sistema

- **Detecções inconsistentes:**
  - Melhore a iluminação do rosto
  - Ajuste a posição da câmera
  - Certifique-se que o rosto está completamente visível

- **Sistema trava:**
  - Pressione o botão RESET
  - Se o problema persistir, verifique a fonte de alimentação

## Manutenção

- Limpe a lente da câmera periodicamente com um pano macio
- Verifique as conexões regularmente
- Mantenha o sistema livre de poeira e umidade

## Configuração do WiFi

### Configuração Inicial
1. Após a instalação do firmware, o ESP32-CAM criará uma rede WiFi:
   - Nome da rede: `SnorLess-Config`
   - Senha padrão: `snorless123`

2. Conecte-se a esta rede com seu smartphone ou computador
3. Abra o navegador e acesse `http://192.168.4.1`
4. Na interface web, você poderá:
   - Configurar a conexão com sua rede WiFi local
   - Visualizar o feed da câmera em tempo real
   - Ajustar o posicionamento da câmera
   - Configurar parâmetros de detecção

### Uso da Interface Web
1. Após conectar à sua rede WiFi local:
   - Acesse o endereço IP mostrado no monitor serial
   - Ou encontre o IP na lista de dispositivos do seu roteador

2. Recursos disponíveis:
   - Visualização em tempo real
   - Ajuste de brilho e contraste
   - Configuração de sensibilidade
   - Status do sistema
   - Logs de detecção

3. Recomendações:
   - Use um navegador moderno (Chrome, Firefox, Edge)
   - Mantenha o dispositivo na mesma rede local
   - Evite redes públicas para maior segurança

### Solução de Problemas WiFi
- **Não encontra a rede SnorLess-Config:**
  - Reinicie o dispositivo
  - Verifique se está no alcance
  - Aguarde 30 segundos após a inicialização

- **Não conecta à interface web:**
  - Verifique se está conectado à rede correta
  - Tente acessar pelo IP alternativo: `192.168.4.1`
  - Limpe o cache do navegador

- **Perda de conexão frequente:**
  - Aproxime-se do roteador
  - Verifique a qualidade do sinal
  - Considere um repetidor WiFi 