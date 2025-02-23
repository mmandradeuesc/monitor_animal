/**
 * @file monitor.c
 * @brief WILDLIFE - Sistema de Monitoramento Ambiental para Raspberry Pi Pico
 *
 * Este sistema realiza o monitoramento de condições ambientais incluindo temperatura,
 * fluxo de água, chuva, detecção de incêndio e presença de vida silvestre.
 * Utiliza diversos sensores e fornece feedback visual através de display OLED,
 * matriz de NeoPixels e indicadores LED.
 *
 * @author Marcel Mascarenhas Andrade
 * @date 2025-06-22
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "pico/binary_info.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "monitor.pio.h"
#include "font.h"
#include "ssd1306.h"
#include "hardware/sync.h"

// Definições de pinos
/**
 * @brief Definições de pinos para conexões de hardware
 */
#define DISPLAY_SDA_PIN 14    ///< Pino SDA para display OLED
#define DISPLAY_SCL_PIN 15    ///< Pino SCL para display OLED
#define I2C_PORT i2c1         ///< Porta I2C utilizada
#define LED_R_PIN 13          ///< Pino do LED vermelho
#define LED_G_PIN 11          ///< Pino do LED verde
#define LED_B_PIN 12          ///< Pino do LED azul
#define BUTTON_A_PIN 5        ///< Pino do botão A
#define BUTTON_B_PIN 6        ///< Pino do botão B
#define JOY_BUTTON_PIN 22     ///< Pino do botão do joystick
#define JOY_X_PIN 27          ///< Pino X do joystick (ADC)
#define JOY_Y_PIN 26          ///< Pino Y do joystick (ADC)
#define BUZZER_PIN 10         ///< Pino do buzzer
#define NUM_PIXELS 25         ///< Número total de NeoPixels
#define OUT_PIN 7             ///< Pino de dados dos NeoPixels

// Protótipos de funções
void display_sensor_data(void);
void detect_fire(void);
void update_fire_alarm(void);
void play_wildlife_alert(void);
void check_buttons(void);
void debounce_buttons(void);
void play_tone(uint frequency, uint duration);
void init_menu(void);
void play_startup_music(void);
void init_neopixels(void);
void update_neopixel_bars(void);
void display_sos_neopixel(void);
static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b); // Declaração movida para cá

// Variáveis de controle de recursos
bool temp_enabled = false;
bool flow_enabled = false;
bool rain_enabled = false;
bool fire_enabled = false;
bool wildlife_enabled = false;

// Variáveis para alerta de incêndio
bool fire_alert_active = false;
uint32_t fire_alert_start = 0;

// Variáveis para debounce
bool button_a_last_state = true;
bool button_b_last_state = true;
bool joy_button_last_state = true;
uint32_t last_debounce_time_a = 0;
uint32_t last_debounce_time_b = 0;
uint32_t last_debounce_time_joy = 0;
const uint32_t debounce_delay = 50;

// Estrutura para animais silvestres
/**
 * @brief Estrutura para informações de vida silvestre
 */
typedef struct {
    char name[20];           ///< Identificação do animal
    char link[150];         ///< Link para imagem capturada
    bool detected;          ///< Status de detecção
    uint32_t detection_time; ///< Momento da detecção
} WildlifeInfo;

WildlifeInfo wildlife[] = {
    {"Foto rec", "https://drive.google.com/file/d/15_ZTIE_34Xu11u8xKI7wOrPwJtoS4Ept/view?usp=sharing", false, 0},
    {"Foto rec", "https://drive.google.com/file/d/1SNEj4sAbBRgybKdEJVYyPiLfmEHj0Qz4/view?usp=sharing", false, 0},
    {"Foto rec", "https://drive.google.com/file/d/1_fRjkiWESZjd7vAqKKhOmwhtSHOEm1px/view?usp=sharing", false, 0},
    {"Foto rec", "https://drive.google.com/file/d/15_ZTIE_34Xu11u8xKI7wOrPwJtoS4Ept/view?usp=sharing", false, 0},
    {"Foto rec", "https://drive.google.com/file/d/1ZVCvLEJgxNReGmp7AcFU1jGDcLILRz7d/view?usp=sharing", false, 0},
    {"Foto rec", "https://drive.google.com/file/d/1YuWoOdcSSX8x0ghBXWwdZ1zmn0_pDgux/view?usp=sharing", false, 0}
};

#define NUM_WILDLIFE (sizeof(wildlife) / sizeof(WildlifeInfo))
int last_detected_wildlife = -1;
bool wildlife_alert_active = false;
#define WILDLIFE_ALERT_DURATION_MS 10000

// Estrutura para sensores
/**
 * @brief Estrutura para configuração dos sensores
 */
typedef struct {
    char name[15];      ///< Nome do sensor
    char unit[5];       ///< Unidade de medida
    float min_val;      ///< Valor mínimo válido
    float max_val;      ///< Valor máximo válido
    float anomaly_min;  ///< Limite mínimo para detecção de anomalia
    float anomaly_max;  ///< Limite máximo para detecção de anomalia
    float variation;    ///< Variação permitida entre leituras
    float value;        ///< Valor atual do sensor
    float history[10];  ///< Histórico de valores para média móvel
} SensorConfig;


SensorConfig sensors[3];
int current_sensor_index = 0;
bool display_initialized = false;

// PIO para NeoPixels
PIO pio = pio0;
uint sm = 0;

// Declaração da função put_pixel
static inline void put_pixel(uint32_t pixel_grb) {
    pio_sm_put_blocking(pio, sm, pixel_grb << 8u);
}

// Declaração do spin lock global
static spin_lock_t *pixel_lock;
static int pixel_lock_num;

// Constantes para animação do gráfico
const uint8_t graphic_frames[4][5][5] = {
    {
        {1, 2, 3, 2, 1}, 
        {0, 1, 2, 1, 0},
        {0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0}   
    },
    {
        {1, 2, 3, 2, 1},
        {1, 2, 3, 2, 1},
        {0, 1, 2, 1, 0},
        {0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0}
    },
    {
        {1, 2, 3, 2, 1},
        {1, 2, 3, 2, 1},
        {1, 2, 3, 2, 1},
        {0, 1, 2, 1, 0},
        {0, 0, 1, 0, 0}
    },
    {
        {1, 2, 3, 2, 1},
        {1, 2, 3, 2, 1},
        {1, 2, 3, 2, 1},
        {1, 2, 3, 2, 0},
        {0, 1, 0, 1, 0}
    }
};
/**
 * @brief Inicializa os sensores do sistema
 * 
 * Configura os parâmetros iniciais para:
 * - Sensor de temperatura (15-35°C)
 * - Sensor de fluxo de água (0-30 L/min)
 * - Sensor de chuva (0-100 mm/h)
 */
void init_sensors() {
    strcpy(sensors[0].name, "Temperatura");
    strcpy(sensors[0].unit, "C");
    sensors[0].min_val = 15.0f;
    sensors[0].max_val = 35.0f;
    sensors[0].anomaly_min = 10.0f;
    sensors[0].anomaly_max = 40.0f;
    sensors[0].variation = 0.5f;
    sensors[0].value = (sensors[0].min_val + sensors[0].max_val) / 2;

    strcpy(sensors[1].name, "Fluxo Agua");
    strcpy(sensors[1].unit, "L/min");
    sensors[1].min_val = 0.0f;
    sensors[1].max_val = 30.0f;
    sensors[1].anomaly_min = 0.0f;
    sensors[1].anomaly_max = 25.0f;
    sensors[1].variation = 1.0f;
    sensors[1].value = 0.0f;

    strcpy(sensors[2].name, "Chuva");
    strcpy(sensors[2].unit, "mm/h");
    sensors[2].min_val = 0.0f;
    sensors[2].max_val = 100.0f;
    sensors[2].anomaly_min = 0.0f;
    sensors[2].anomaly_max = 80.0f;
    sensors[2].variation = 5.0f;
    sensors[2].value = 0.0f;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 10; j++) {
            sensors[i].history[j] = sensors[i].value;
        }
    }
}
/**
 * @brief Inicializa o hardware do sistema
 * 
 * Configura todos os componentes físicos:
 * - Display OLED via I2C
 * - Interface I2C
 * - GPIOs para LEDs e botões
 * - ADC para joystick
 * - PWM para buzzer
 * - Matriz de NeoPixels
 */
void init_hardware() {
    stdio_init_all();
    ssd1306_init();
    ssd1306_clear();
    ssd1306_update();
    display_initialized = true;
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(DISPLAY_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(DISPLAY_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(DISPLAY_SDA_PIN);
    gpio_pull_up(DISPLAY_SCL_PIN);
    gpio_init(LED_R_PIN);
    gpio_init(LED_G_PIN);
    gpio_init(LED_B_PIN);
    gpio_set_dir(LED_R_PIN, GPIO_OUT);
    gpio_set_dir(LED_G_PIN, GPIO_OUT);
    gpio_set_dir(LED_B_PIN, GPIO_OUT);
    gpio_init(BUTTON_A_PIN);
    gpio_init(BUTTON_B_PIN);
    gpio_init(JOY_BUTTON_PIN);
    gpio_set_dir(BUTTON_A_PIN, GPIO_IN);
    gpio_set_dir(BUTTON_B_PIN, GPIO_IN);
    gpio_set_dir(JOY_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_A_PIN);
    gpio_pull_up(BUTTON_B_PIN);
    gpio_pull_up(JOY_BUTTON_PIN);
    adc_init();
    adc_gpio_init(JOY_X_PIN);
    adc_gpio_init(JOY_Y_PIN);
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(BUZZER_PIN);
    pwm_set_wrap(slice_num, 255);
    pwm_set_clkdiv(slice_num, 1.0f);
    pwm_set_enabled(slice_num, true);
    gpio_put(LED_R_PIN, 0);
    gpio_put(LED_G_PIN, 0);
    gpio_put(LED_B_PIN, 1);
    init_neopixels();
}

// Definição de urgb_u32 movida para antes do primeiro uso
/**
 * @brief Converte valores RGB para formato GRB dos NeoPixels
 * 
 * @param r Valor do componente vermelho (0-255)
 * @param g Valor do componente verde (0-255)
 * @param b Valor do componente azul (0-255)
 * @return uint32_t Valor formatado para NeoPixel (GRB)
 */
static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)(r) << 16) | ((uint32_t)(g) << 8) | (uint32_t)(b); // GRB
}
/**
 * @brief Desenha um caractere no display OLED
 * 
 * @param x Posição X no display (0-127)
 * @param y Posição Y no display (0-63)
 * @param c Caractere a ser desenhado
 * @param inverted Se verdadeiro, inverte as cores do caractere
 */
void draw_char(int x, int y, char c, bool inverted) {
    int index = (unsigned char)c;
    for (int i = 0; i < 8; i++) {
        uint8_t line = font[index * 8 + i];
        if (inverted) line = ~line;
        for (int j = 0; j < 8; j++) {
            if (line & (1 << j)) {
                ssd1306_draw_pixel(x + j, y + i, true);
            } else {
                ssd1306_draw_pixel(x + j, y + i, false);
            }
        }
    }
}
/**
 * @brief Desenha uma string no display OLED
 * 
 * @param x Posição X inicial no display
 * @param y Posição Y inicial no display
 * @param str String a ser desenhada
 * @param inverted Se verdadeiro, inverte as cores do texto
 */
void draw_string(int x, int y, const char *str, bool inverted) {
    int orig_x = x;
    for (int i = 0; str[i] != '\0'; i++) {
        if (str[i] == '\n') {
            x = orig_x;
            y += 9;
            continue;
        }
        draw_char(x, y, str[i], inverted);
        x += 8;
        if (x >= DISPLAY_WIDTH - 8) {
            x = orig_x;
            y += 9;
        }
    }
}
/**
 * @brief Desenha uma linha horizontal no display
 * 
 * @param x0 Posição X inicial
 * @param y0 Posição Y
 * @param length Comprimento da linha em pixels
 */
void draw_horizontal_line(int x0, int y0, int length) {
    for (int i = 0; i < length; i++) {
        ssd1306_draw_pixel(x0 + i, y0, true);
    }
}
/**
 * @brief Toca a música de inicialização
 * 
 * Reproduz uma sequência de notas musicais usando o buzzer
 * para indicar que o sistema foi iniciado
 */
void play_startup_music() {
    // Notas musicais (frequências em Hz)
    uint notes[] = {
        392,  // Sol4
        494,  // Si4
        587,  // Ré5
        784,  // Sol5
        587,  // Ré5
        494,  // Si4
        392   // Sol4
    };
    
    // Durações das notas em ms (total: 7000ms)
    uint durations[] = {
        1000,  // 1.0s
        1000,  // 1.0s
        1000,  // 1.0s
        1500,  // 1.5s
        1000,  // 1.0s
        750,   // 0.75s
        750    // 0.75s
    };
    
    for (int i = 0; i < 7; i++) {
        play_tone(notes[i], durations[i]);
        sleep_ms(50);  // Pequena pausa entre as notas
    }
}
/**
 * @brief Inicializa a matriz de NeoPixels
 * 
 * Configura o PIO e inicializa o hardware para controle
 * da matriz 5x5 de LEDs RGB (WS2812B)
 */
void init_neopixels() {
    // Inicializa o spin lock
    pixel_lock_num = spin_lock_claim_unused(true);
    pixel_lock = spin_lock_init(pixel_lock_num);
    
    uint offset = pio_add_program(pio, &monitor_program);
    monitor_program_init(pio, sm, offset, OUT_PIN);
    
    // Limpa todos os pixels inicialmente
    for (int i = 0; i < NUM_PIXELS; i++) {
        put_pixel(0);
    }
    
    // Aguarda a conclusão da transmissão
    sleep_ms(1);
}

// Função auxiliar para mapear coordenadas x,y para o índice do LED na matriz
/**
 * @brief Converte coordenadas x,y para índice do LED na matriz
 * 
 * @param x Coordenada X (0-4)
 * @param y Coordenada Y (0-4)
 * @return int Índice do LED na matriz
 */
static int xy_to_pixel_index(int x, int y) {
    if (y % 2 == 0) {
        // Linhas pares: da esquerda para a direita
        return y * 5 + x;
    } else {
        // Linhas ímpares: da direita para a esquerda
        return y * 5 + (4 - x);
    }
}

// Função melhorada para atualizar a animação de chamas
void update_graphic_animation(PIO pio, uint sm, uint8_t frame) {
    // Array para armazenar todos os pixels antes de enviá-los
    uint32_t pixels[NUM_PIXELS] = {0};
    
    // Preenche o array com as cores da animação
    for (int y = 0; y < 5; y++) {
        for (int x = 0; x < 5; x++) {
            int pixel_index = xy_to_pixel_index(x, y);
            uint8_t intensity = graphic_frames[frame][y][x];
            
            uint8_t r = 0, g = 0, b = 0;
            switch (intensity) {
                case 0: // Desligado
                    break;
                case 1: // Vermelho escuro
                    r = 6;
                    break;
                case 2: // Laranja
                    g = 0;
                    b = 10;
                    break;
                case 3: // Amarelo
                    g = 0;
                    b = 10;
                    break;
            }
            
            // Formato GRB para NeoPixels
            pixels[pixel_index] = ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;
        }
    }
    
    // Desabilita interrupções durante o envio para timing preciso
    uint32_t save = spin_lock_blocking(pixel_lock);
    
    // Envia todos os pixels de uma vez
    for (int i = 0; i < NUM_PIXELS; i++) {
        put_pixel(pixels[i]);
    }
    
    // Restaura interrupções
    spin_unlock(pixel_lock, save);
    
    // Aguarda conclusão da transmissão
    sleep_us(50);
}

// Função principal de atualização da matriz
void update_neopixel_bars() {
    static uint32_t last_animation_time = 0;
    static int current_frame = 0;
    uint32_t current_time = to_ms_since_boot(get_absolute_time());

    // Atualiza a animação a cada 200ms
    if (current_time - last_animation_time >= 200) {
        current_frame = (current_frame + 1) % 4;
        last_animation_time = current_time;
        
        // Atualiza a animação
        update_graphic_animation(pio, sm, current_frame);
    }
}
/**
 * @brief Exibe padrão SOS na matriz de LEDs
 * 
 * Implementa o padrão Morse SOS (... --- ...) usando
 * os LEDs da matriz para sinalização de emergência
 */
void display_sos_neopixel() {
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    uint32_t t = current_time % 6000;
    bool alarm_on = false;

    if ((t < 200) || (t >= 400 && t < 600) || (t >= 800 && t < 1000) ||
        (t >= 1600 && t < 2200) || (t >= 2400 && t < 3000) || (t >= 3200 && t < 3800) ||
        (t >= 4400 && t < 4600) || (t >= 4800 && t < 5000) || (t >= 5200 && t < 5400)) {
        alarm_on = true;
    }

    for (int i = 0; i < NUM_PIXELS; i++) {
        if (alarm_on) {
            put_pixel(urgb_u32(255, 0, 0) << 8u);
        } else {
            put_pixel(0); // Apaga
        }
    }
}
/**
 * @brief Menu de configuração inicial
 * 
 * Permite habilitar/desabilitar os diferentes módulos:
 * - Temperatura
 * - Fluxo de água
 * - Chuva
 * - Detecção de incêndio
 * - Monitoramento de vida silvestre
 */
void init_menu() {
    const char *options[] = {"Temperatura", "Fluviometro", "Chuva", "Incendio", "Vida Silvestre"};
    bool states[] = {temp_enabled, flow_enabled, rain_enabled, fire_enabled, wildlife_enabled};
    int menu_index = 0;
    bool menu_active = true;

    while (menu_active) {
        ssd1306_clear();
        draw_string(0, 0, "CONFIGURACAO", false);
        draw_horizontal_line(0, 10, 128);
        draw_string(0, 15, options[menu_index], false);
        char state_str[10];
        sprintf(state_str, "Estado: %s", states[menu_index] ? "ON" : "OFF");
        draw_string(0, 25, state_str, false);
        draw_string(0, 40, "A: Alternar", false);
        draw_string(0, 50, "B: Iniciar", false);
        ssd1306_update();

        bool button_a_pressed = !gpio_get(BUTTON_A_PIN);
        bool button_b_pressed = !gpio_get(BUTTON_B_PIN);
        adc_select_input(JOY_X_PIN - 26);
        uint16_t joy_x_value = adc_read();

        static uint32_t last_action_time = 0;
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        if (current_time - last_action_time < 200) {
            sleep_ms(50);
            continue;
        }

        if (joy_x_value < 1000) {
            menu_index = (menu_index + 4) % 5;
            last_action_time = current_time;
            play_tone(440, 50);
        }
        else if (joy_x_value > 3000) {
            menu_index = (menu_index + 1) % 5;
            last_action_time = current_time;
            play_tone(440, 50);
        }

        if (button_a_pressed) {
            states[menu_index] = !states[menu_index];
            last_action_time = current_time;
            play_tone(660, 50);
        }

        if (button_b_pressed) {
            menu_active = false;
            temp_enabled = states[0];
            flow_enabled = states[1];
            rain_enabled = states[2];
            fire_enabled = states[3];
            wildlife_enabled = states[4];
            last_action_time = current_time;
            play_tone(880, 100);
        }

        sleep_ms(50);
    }
}
/**
 * @brief Reproduz alerta sonoro para detecção de animais
 */
void play_wildlife_alert() {
    if (!wildlife_enabled) return;
    for (int i = 0; i < 3; i++) {
        play_tone(440, 500);
        sleep_ms(100);
    }
}
/**
 * @brief Reproduz um tom no buzzer
 * 
 * @param frequency Frequência do tom em Hz
 * @param duration Duração em milissegundos
 */
void play_tone(uint frequency, uint duration) {
    uint slice_num = pwm_gpio_to_slice_num(BUZZER_PIN);
    uint wrap_value = (uint)(clock_get_hz(clk_sys) / frequency) - 1;
    pwm_set_wrap(slice_num, wrap_value);
    pwm_set_clkdiv(slice_num, 1.0f);
    pwm_set_enabled(slice_num, true);
    pwm_set_gpio_level(BUZZER_PIN, wrap_value / 2);
    sleep_ms(duration);
    pwm_set_gpio_level(BUZZER_PIN, 0);
}
/**
 * @brief Atualiza o estado do alerta SOS
 * 
 * Controla LEDs, buzzer e matriz de NeoPixels durante
 * o alerta de incêndio
 */
void update_sos_alert() {
    if (!fire_enabled || !fire_alert_active) {
        gpio_put(LED_R_PIN, 0);
        pwm_set_gpio_level(BUZZER_PIN, 0);
        for (int i = 0; i < NUM_PIXELS; i++) {
            put_pixel(0); // Apaga NeoPixels
        }
        return;
    }

    static uint32_t last_update = 0;
    static uint buzzer_wrap = 0; // Armazena o valor do wrap do PWM
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    uint32_t t = current_time % 5400; // Ciclo de 5400ms
    bool led_on = false;
    bool neopixel_on = false;
    bool buzzer_on = false;

    // Padrão SOS: tempos em ms
    // S: 3 pontos (200ms ON, 200ms OFF) -> 0-1000
    // O: 3 traços (600ms ON, 200ms OFF) -> 1600-3800
    // S: 3 pontos (200ms ON, 200ms OFF) -> 4400-5400
    if (t < 200 || (t >= 400 && t < 600) || (t >= 800 && t < 1000) ||           // S (pontos)
        (t >= 1600 && t < 2200) || (t >= 2400 && t < 3000) || (t >= 3200 && t < 3800) || // O (traços)
        (t >= 4400 && t < 4600) || (t >= 4800 && t < 5000) || (t >= 5200 && t < 5400)) { // S (pontos)
        led_on = true;
        neopixel_on = true;
        buzzer_on = true;
    }

    // Atualiza LED
    gpio_put(LED_R_PIN, led_on ? 1 : 0);

    // Atualiza NeoPixels (vermelho corrigido: GRB = 0, 255, 0)
    uint32_t save = spin_lock_blocking(pixel_lock);
    for (int i = 0; i < NUM_PIXELS; i++) {
        put_pixel(neopixel_on ? urgb_u32(0, 255, 0) : 0); // Vermelho em GRB
    }
    spin_unlock(pixel_lock, save);

    // Atualiza buzzer (não bloqueante)
    uint slice_num = pwm_gpio_to_slice_num(BUZZER_PIN);
    if (buzzer_on && last_update != current_time) {
        buzzer_wrap = (uint)(clock_get_hz(clk_sys) / 650) - 1; // Calcula e armazena o wrap
        pwm_set_wrap(slice_num, buzzer_wrap);
        pwm_set_gpio_level(BUZZER_PIN, buzzer_wrap / 2); // Define nível como 50% do ciclo
    } else {
        pwm_set_gpio_level(BUZZER_PIN, 0);
    }

    last_update = current_time;
}
/**
 * @brief Detecta condições de incêndio na área monitorada
 * 
 * Simula a detecção de incêndio com base em probabilidades:
 * - 1% de chance base de detecção
 * - 20% de chance quando apenas o módulo de incêndio está ativo
 * - Força detecção após 5 segundos se apenas incêndio estiver ativo
 */
void detect_fire() {
    if (!fire_enabled || fire_alert_active) return;

    // Verifica se apenas o incêndio está ativo
    bool only_fire_enabled = fire_enabled && !temp_enabled && !flow_enabled && !rain_enabled && !wildlife_enabled;

    // Probabilidade base: 1% (10/1000)
    int base_chance = 10;
    // Aumenta a probabilidade para 20% (200/1000) se apenas incêndio estiver ativo
    int fire_chance = only_fire_enabled ? 200 : base_chance;

    static uint32_t last_check_time = 0;
    uint32_t current_time = to_ms_since_boot(get_absolute_time());

    // Força um incêndio após 5 segundos se apenas incêndio estiver ativo
    if (only_fire_enabled && (current_time - last_check_time > 5000)) {
        fire_alert_active = true;
        fire_alert_start = current_time;
        last_check_time = current_time;
        printf("\n*** ALERTA DE INCENDIO: Fogo detectado na floresta! (Forçado após 5s) ***\n");
    }
    // Detecção aleatória
    else if (rand() % 1000 < fire_chance) {
        fire_alert_active = true;
        fire_alert_start = current_time;
        last_check_time = current_time;
        printf("\n*** ALERTA DE INCENDIO: Fogo detectado na floresta! ***\n");
    }
}
/**
 * @brief Detecta presença de animais silvestres
 * 
 * Simula a detecção de animais com:
 * - 5% de chance de detecção a cada verificação
 * - Seleciona aleatoriamente um animal do banco de dados
 * - Registra horário da detecção
 * - Ativa alerta sonoro e visual
 */
void detect_wildlife() {
    if (!wildlife_enabled) return;
    if (rand() % 100 < 5) {
        int animal_index = rand() % NUM_WILDLIFE;
        wildlife[animal_index].detected = true;
        wildlife[animal_index].detection_time = to_ms_since_boot(get_absolute_time());
        last_detected_wildlife = animal_index;
        wildlife_alert_active = true;
        display_sensor_data();
        play_wildlife_alert();
        printf("\n*** ALERTA: %s detectado! ***\n", wildlife[animal_index].name);
        printf("Imagem capturada: %s\n", wildlife[animal_index].link);
        printf("------------------------------\n");
    }
}
/**
 * @brief Gerencia alertas de vida silvestre
 * 
 * Controla a duração dos alertas de detecção:
 * - Verifica se o alerta atual excedeu o tempo máximo
 * - Desativa o alerta após WILDLIFE_ALERT_DURATION_MS (10 segundos)
 * - Limpa o status de detecção atual
 */
void check_wildlife_alerts() {
    if (!wildlife_enabled) return;
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    if (wildlife_alert_active && last_detected_wildlife >= 0) {
        if (current_time - wildlife[last_detected_wildlife].detection_time > WILDLIFE_ALERT_DURATION_MS) {
            wildlife_alert_active = false;
        }
    }
}
/**
 * @brief Simula leitura de sensor
 * 
 * @param sensor Configuração do sensor
 * @return float Valor simulado da leitura
 */
float simulate_reading(SensorConfig *sensor) {
    float base = sensor->value;
    
    if (strcmp(sensor->name, "Temperatura") == 0 && temp_enabled) {
        time_t t = time(NULL);
        struct tm* tm_info = localtime(&t);
        int hour = tm_info ? tm_info->tm_hour : 12;
        float daily_variation = 5.0f * sinf((hour - 14) * 3.14159f / 12);
        base = (sensor->min_val + sensor->max_val) / 2 + daily_variation;
    }
    else if (strcmp(sensor->name, "Chuva") == 0 && rain_enabled) {
        if (rand() % 100 < 30) {
            base = (float)(rand() % (int)(sensor->max_val));
        } else {
            base = 0.0f;
        }
    }
    else if (strcmp(sensor->name, "Fluxo Agua") == 0 && flow_enabled) {
        base = sensors[2].value > 0 ? (float)(rand() % 20) : 0.0f;
    }
    
    float variation = ((float)rand() / RAND_MAX * 2 - 1) * sensor->variation;
    float new_value = base + variation;
    if (new_value < sensor->min_val) new_value = sensor->min_val;
    if (new_value > sensor->max_val) new_value = sensor->max_val;
    return new_value;
}
/**
 * @brief Atualiza o valor do sensor
 * 
 * @param sensor Sensor a ser atualizado
 */
void update_sensor_value(SensorConfig *sensor) {
    float new_value = simulate_reading(sensor);
    for (int i = 0; i < 9; i++) {
        sensor->history[i] = sensor->history[i + 1];
    }
    sensor->history[9] = new_value;
    sensor->value = new_value;
}
/**
 * @brief Calcula média móvel das últimas 10 leituras
 * 
 * @param sensor Sensor para cálculo
 * @return float Média das leituras
 */
float calculate_moving_average(SensorConfig *sensor) {
    float sum = 0;
    for (int i = 0; i < 10; i++) {
        sum += sensor->history[i];
    }
    return sum / 10;
}
/**
 * @brief Verifica anomalias nas leituras
 * 
 * @param sensor Sensor a ser verificado
 * @return true Se houver anomalia
 * @return false Caso contrário
 */
bool check_anomaly(SensorConfig *sensor) {
    return (sensor->value < sensor->anomaly_min || sensor->value > sensor->anomaly_max);
}
/**
 * @brief Exibe dados dos sensores no display
 * 
 * Mostra valores atuais, médias e alertas no
 * display OLED
 */
void display_sensor_data() {
    if (!display_initialized) return;
    ssd1306_clear();

    if (fire_enabled && fire_alert_active) {
        draw_string(0, 0, "*** INCENDIO ***", false);
        draw_horizontal_line(0, 10, 128);
        draw_string(0, 15, "SOS Ativado!", false);
        draw_string(0, 25, "Pressione B", false);
        draw_string(0, 35, "para cancelar", false);
        ssd1306_update();
        return;
    }

    if (wildlife_enabled && wildlife_alert_active && last_detected_wildlife >= 0) {
        draw_string(0, 0, "*** ALERTA ***", false);
        draw_horizontal_line(0, 10, 128);
        char animal_msg[30];
        sprintf(animal_msg, "Animal detectado:");
        draw_string(0, 15, animal_msg, false);
        draw_string(0, 25, wildlife[last_detected_wildlife].name, false);
        draw_string(0, 40, "Pressione qualquer", false);
        draw_string(0, 50, "botao para continuar", false);
        gpio_put(LED_R_PIN, 0);
        gpio_put(LED_G_PIN, 0);
        gpio_put(LED_B_PIN, 1);
        sleep_ms(100);
        gpio_put(LED_B_PIN, 0);
    } else {
        // Conta todos os recursos habilitados, não apenas os sensores ambientais
        int active_features = (temp_enabled ? 1 : 0) + (flow_enabled ? 1 : 0) + 
                             (rain_enabled ? 1 : 0) + (fire_enabled ? 1 : 0) + 
                             (wildlife_enabled ? 1 : 0);
        if (active_features == 0) {
            draw_string(0, 20, "Nenhum sensor", false);
            draw_string(0, 30, "ativo", false);
            ssd1306_update();
            update_neopixel_bars();
            return;
        }
        
        // Exibe sensores ambientais apenas se houver pelo menos um ativo
        int active_sensors = (temp_enabled ? 1 : 0) + (flow_enabled ? 1 : 0) + (rain_enabled ? 1 : 0);
        if (active_sensors == 0) {
            draw_string(0, 20, "Monitorando:", false);
            draw_string(0, 30, fire_enabled ? "Incendio" : "Animais", false);
            ssd1306_update();
            update_neopixel_bars();
            return;
        }
        
        draw_string(0, 0, "Mon Ambiental", false);
        draw_horizontal_line(0, 10, 128);
        SensorConfig *sensor = &sensors[current_sensor_index];
        if ((strcmp(sensor->name, "Temperatura") == 0 && !temp_enabled) ||
            (strcmp(sensor->name, "Fluxo Agua") == 0 && !flow_enabled) ||
            (strcmp(sensor->name, "Chuva") == 0 && !rain_enabled)) {
            current_sensor_index = (current_sensor_index + 1) % 3;
            return;
        }
        
        draw_string(0, 15, sensor->name, false);
        char value_str[20];
        sprintf(value_str, "%.1f %s", sensor->value, sensor->unit);
        draw_string(0, 25, value_str, false);
        
        float avg = calculate_moving_average(sensor);
        sprintf(value_str, "Media: %.1f %s", avg, sensor->unit);
        draw_string(0, 35, value_str, false);
        
        if (check_anomaly(sensor)) {
            draw_string(0, 45, "ALERTA!", false);
            gpio_put(LED_R_PIN, 1);
            gpio_put(LED_G_PIN, 0);
            gpio_put(LED_B_PIN, 0);
            play_tone(449, 500);
        } else {
            gpio_put(LED_R_PIN, 0);
            gpio_put(LED_G_PIN, 1);
            gpio_put(LED_B_PIN, 0);
        }
        
        sprintf(value_str, "%d/%d", current_sensor_index + 1, 3);
        draw_string(100, 55, value_str, false);
    }
    ssd1306_update();
    update_neopixel_bars();
}
/**
 * @brief Envia dados para porta serial
 * 
 * Envia leituras dos sensores e alertas via
 * comunicação serial
 */
void send_serial_data() {
    printf("\n===== LEITURA DOS SENSORES =====\n");
    if (temp_enabled) {
        float avg = calculate_moving_average(&sensors[0]);
        printf("%s: %.1f %s (Media: %.1f)\n", sensors[0].name, sensors[0].value, sensors[0].unit, avg);
        sleep_ms(1000); // Atraso de 1 segundo
    }
    if (flow_enabled) {
        float avg = calculate_moving_average(&sensors[1]);
        printf("%s: %.1f %s (Media: %.1f)\n", sensors[1].name, sensors[1].value, sensors[1].unit, avg);
        sleep_ms(1000); // Atraso de 1 segundo
    }
    if (rain_enabled) {
        float avg = calculate_moving_average(&sensors[2]);
        printf("%s: %.1f %s (Media: %.1f)\n", sensors[2].name, sensors[2].value, sensors[2].unit, avg);
        sleep_ms(1000); // Atraso de 1 segundo
    }
    
    bool has_anomaly = false;
    printf("ALERTA: Anomalias detectadas em: ");
    if (temp_enabled && check_anomaly(&sensors[0])) {
        printf("%s ", sensors[0].name);
        has_anomaly = true;
    }
    if (flow_enabled && check_anomaly(&sensors[1])) {
        printf("%s ", sensors[1].name);
        has_anomaly = true;
    }
    if (rain_enabled && check_anomaly(&sensors[2])) {
        printf("%s ", sensors[2].name);
        has_anomaly = true;
    }
    if (!has_anomaly) printf("Nenhuma");
    printf("\n------------------------------\n");
    sleep_ms(1000); // Atraso de 1 segundo
}
/**
 * @brief Verifica estado dos botões
 * 
 * Processa entradas do usuário via botões
 * e joystick
 */
void check_buttons() {
    debounce_buttons();
    
    if (fire_enabled && fire_alert_active) {
        bool button_b_pressed = !gpio_get(BUTTON_B_PIN);
        static uint32_t last_cancel_time = 0;
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        
        if (button_b_pressed && (current_time - last_cancel_time > 200)) {
            fire_alert_active = false;
            gpio_put(LED_R_PIN, 0);
            printf("\nAlerta de incendio cancelado pelo usuario.\n");
            play_tone(880, 100);
            last_cancel_time = current_time;
            sleep_ms(200);
            display_sensor_data();
            return;
        }
    }
    
    if (wildlife_enabled && wildlife_alert_active) {
        if (!button_a_last_state || !button_b_last_state || !joy_button_last_state) {
            wildlife_alert_active = false;
            printf("\nAlerta de animal silvestre cancelado pelo usuario.\n");
            sleep_ms(200);
            display_sensor_data();
            return;
        }
    }

    int active_sensors = (temp_enabled ? 1 : 0) + (flow_enabled ? 1 : 0) + (rain_enabled ? 1 : 0);
    if (active_sensors == 0) return;

    static uint32_t last_joy_time = 0;
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    
    adc_select_input(JOY_X_PIN - 26);
    uint16_t joy_x_value = adc_read();
    if (joy_x_value < 1000 && (current_time - last_joy_time > 200)) {
        do {
            current_sensor_index = (current_sensor_index + 2) % 3;
        } while ((!temp_enabled && current_sensor_index == 0) ||
                 (!flow_enabled && current_sensor_index == 1) ||
                 (!rain_enabled && current_sensor_index == 2));
        last_joy_time = current_time;
        play_tone(440, 50);
        display_sensor_data();
    } else if (joy_x_value > 3000 && (current_time - last_joy_time > 200)) {
        do {
            current_sensor_index = (current_sensor_index + 1) % 3;
        } while ((!temp_enabled && current_sensor_index == 0) ||
                 (!flow_enabled && current_sensor_index == 1) ||
                 (!rain_enabled && current_sensor_index == 2));
        last_joy_time = current_time;
        play_tone(440, 50);
        display_sensor_data();
    }

    if (!button_a_last_state && (current_time - last_joy_time > 200)) {
        do {
            current_sensor_index = (current_sensor_index + 2) % 3;
        } while ((!temp_enabled && current_sensor_index == 0) ||
                 (!flow_enabled && current_sensor_index == 1) ||
                 (!rain_enabled && current_sensor_index == 2));
        last_joy_time = current_time;
        display_sensor_data();
    }
    if (!button_b_last_state && !fire_alert_active && (current_time - last_joy_time > 200)) {
        do {
            current_sensor_index = (current_sensor_index + 1) % 3;
        } while ((!temp_enabled && current_sensor_index == 0) ||
                 (!flow_enabled && current_sensor_index == 1) ||
                 (!rain_enabled && current_sensor_index == 2));
        last_joy_time = current_time;
        display_sensor_data();
    }
}
/**
 * @brief Implementa debounce dos botões
 * 
 * Elimina ruídos e bouncing nas leituras
 * dos botões
 */
void debounce_buttons() {
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    bool button_a_state = gpio_get(BUTTON_A_PIN);
    if (button_a_state != button_a_last_state) {
        last_debounce_time_a = current_time;
    }
    if ((current_time - last_debounce_time_a) > debounce_delay) {
        button_a_last_state = button_a_state;
    }
    
    bool button_b_state = gpio_get(BUTTON_B_PIN);
    if (button_b_state != button_b_last_state) {
        last_debounce_time_b = current_time;
    }
    if ((current_time - last_debounce_time_b) > debounce_delay) {
        button_b_last_state = button_b_state;
    }
    
    bool joy_button_state = gpio_get(JOY_BUTTON_PIN);
    if (joy_button_state != joy_button_last_state) {
        last_debounce_time_joy = current_time;
    }
    if ((current_time - last_debounce_time_joy) > debounce_delay) {
        joy_button_last_state = joy_button_state;
    }
}
/**
 * @brief Função principal do sistema
 * 
 * Inicializa hardware e executa loop principal
 * de monitoramento
 * 
 * @return int Código de retorno
 */
int main() {
    init_hardware();
    init_sensors();
    
    ssd1306_clear();
    draw_string(10, 20, "INICIANDO", false);
    ssd1306_update();
    play_startup_music();

    ssd1306_clear();
    draw_string(10, 20, "CONFIGURACAO", false);
    draw_string(25, 30, "INICIAL", false);
    ssd1306_update();
    sleep_ms(2000);
    init_menu();

    ssd1306_clear();
    draw_string(10, 20, "Monitoramento", false);
    draw_string(25, 30, "Ambiental", false);
    draw_string(15, 45, "Iniciando...", false);
    ssd1306_update();
    sleep_ms(2000);

    ssd1306_clear();
    for (int i = 0; i < 128; i++) {
        ssd1306_draw_pixel(i, 0, true);
        ssd1306_draw_pixel(i, 63, true);
    }
    for (int i = 0; i < 64; i++) {
        ssd1306_draw_pixel(0, i, true);
        ssd1306_draw_pixel(127, i, true);
    }
    draw_string(20, 20, "WILDLIFE", false);
    ssd1306_update();
    sleep_ms(5000);

    srand(time(NULL));
    printf("Iniciando Simulador de Monitoramento Ambiental BitDogLab...\n");
    if (wildlife_enabled) printf("Módulo de detecção de animais silvestres ativado\n");
    if (fire_enabled) printf("Módulo de detecção de incendio ativado\n");

    while (true) {
        if (temp_enabled) update_sensor_value(&sensors[0]);
        if (flow_enabled) update_sensor_value(&sensors[1]);
        if (rain_enabled) update_sensor_value(&sensors[2]);
        
        detect_wildlife();
        detect_fire();
        
        check_wildlife_alerts();
        check_buttons();
        
        if (fire_enabled && fire_alert_active) {
            update_sos_alert();
        } else {
            update_neopixel_bars();
        }
        
        display_sensor_data();
        send_serial_data();
        sleep_ms(100);
    }
    
    return 0;
}