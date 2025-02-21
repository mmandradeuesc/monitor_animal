# MONITOR, PROJETO FINAL MONITORAMENTO AMBIENTAL

## Tarefa FINAL 22/02

### Integrante
- Marcel Mascarenhas Andrade

### Pré-requisitos
Certifique-se de que você tem o ambiente configurado conforme abaixo:
- **Pico SDK** instalado e configurado.
- **VSCode** com todas as extensões necessárias configuradas.
- **CMake** e **Make** instalados.

### Instruções de Configuração
1. Clone o repositório e abra a pasta do projeto no VSCode.
   ```sh
   git clone https://github.com/mmandradeuesc/monitor_ambiental
   cd bitdog_ad
   code .

### Manual do Programa
Explicação do Código em C
Adaptei o projeto para linguagem C utilizando o SDK oficial do Raspberry Pi Pico. Aqui estão os principais componentes e como eles funcionam:
1. Estrutura e Organização

Includes: Bibliotecas necessárias do SDK do Pico e padrão C
Definições: Pinos configurados conforme o esquema da BitDogLab
Estruturas de Dados: SensorConfig para manter os dados de cada sensor simulado
Funções: Organizadas para inicialização, simulação, processamento e interface

2. Principais Funções

init_hardware(): Configura todos os periféricos da placa (display, botões, LEDs, etc.)
init_sensors(): Inicializa as configurações e valores iniciais dos sensores
simulate_reading(): Gera valores simulados realistas para cada sensor
update_sensor_value(): Atualiza o histórico e o valor atual do sensor
display_sensor_data(): Gerencia a interface de usuário no display OLED
check_buttons(): Verifica a interação do usuário para navegação

3. Integração com o Hardware BitDogLab

Display I2C: Usando a biblioteca SSD1306 para exibição dos dados
Botões e Joystick: Para navegação na interface
LEDs RGB: Para indicação visual (vermelho = alerta, verde = normal, azul = inicialização)
Buzzers: Para alarmes sonoros em caso de anomalias

Requisitos para Compilação
Para compilar este código, você vai precisar:

SDK do Raspberry Pi Pico: Download em https://github.com/raspberrypi/pico-sdk
Biblioteca SSD1306: Adicionar ao seu projeto (existem várias implementações disponíveis para o SDK do Pico)

Passos para Compilação

Configure o ambiente de desenvolvimento para o Pico (siga a documentação oficial)
Crie um arquivo CMakeLists.txt para seu projeto:

cmakeCopycmake_minimum_required(VERSION 3.12)

# Inicialize o SDK Pico
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
pico_sdk_init()

project(environmental_monitor)

# Adicione os arquivos fonte
add_executable(environmental_monitor
    main.c
    ssd1306.c  # Adicione o arquivo .c da biblioteca SSD1306
)

# Adicione as bibliotecas necessárias do Pico SDK
target_link_libraries(environmental_monitor
    pico_stdlib
    hardware_i2c
    hardware_adc
    pico_time
    hardware_gpio
)

# Habilite saída USB e UART
pico_enable_stdio_usb(environmental_monitor 1)
pico_enable_stdio_uart(environmental_monitor 1)

# Criar arquivo .uf2 para carregamento fácil
pico_add_extra_outputs(environmental_monitor)
 
- Link do vídeo demonstração do programa:

