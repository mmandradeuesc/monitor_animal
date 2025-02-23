# WildLife - Sistema de Monitoramento Ambiental

## Descrição
Sistema embarcado para monitoramento ambiental utilizando Raspberry Pi Pico, desenvolvido para detectar:
- Temperatura ambiente (15-35°C)
- Fluxo de água (0-30 L/min)
- Índice pluviométrico (0-100 mm/h)
- Detecção de incêndio
- Monitoramento de vida silvestre

## Integrante
- Marcel Mascarenhas Andrade

## Hardware Necessário
- Raspberry Pi Pico
- Display OLED SSD1306
- Matriz LED 5x5 (NeoPixels)
- Botões e Joystick
- LED RGB
- Buzzer

## Pré-requisitos
- Pico SDK
- Visual Studio Code
- CMake
- Make
- Git

## Instalação
1. Clone o repositório:
```bash
git clone https://github.com/mmandradeuesc/monitor_animal
cd monitor_animal

2. Configure o projeto:
mkdir build
cd build
cmake ..

3. Conpile:
make

4. Carregue o arquivo .uf2 gerado no Raspberry Pi Pico

## Funcionalidades
- Interface interativa via display OLED
- Simulação de sensores ambientais
- Sistema de alertas sonoros e visuais
- Animações na matriz LED
- Comunicação serial para monitoramento
- Menu de configuração

## Estrutura do Projeto
```plaintext
monitor_animal/
├── src/
│   ├── monitor.c      # Código principal
│   ├── ssd1306.c      # Driver do display
│   ├── ssd1306.h      # Header do display
│   └── monitor.pio    # Configuração PIO
├── CMakeLists.txt     # Configuração do CMake
└── README.md          # Este arquivo
 ```
 ## Demonstração
Vídeo demonstrativo do sistema em funcionamento: [Assista ao vídeo](https://drive.google.com/file/d/10vFOH2OBewdYwKQczrQAFcplCnA_rUx7/view)

## Licença
Public Domain

## Contato
Marcel Mascarenhas Andrade
mmandrade@uesc.br
(73) 982310230
