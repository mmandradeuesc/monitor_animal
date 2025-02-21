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

// Definições de pinos
#define DISPLAY_SDA_PIN 14
#define DISPLAY_SCL_PIN 15
#define I2C_PORT i2c1
#define LED_R_PIN 13
#define LED_G_PIN 11
#define LED_B_PIN 12
#define BUTTON_A_PIN 5
#define BUTTON_B_PIN 6
#define JOY_BUTTON_PIN 22
#define JOY_X_PIN 27
#define JOY_Y_PIN 26
#define BUZZER_PIN 10
#define NUM_PIXELS 25
#define OUT_PIN 7

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
typedef struct {
    char name[20];
    char link[150];
    bool detected;
    uint32_t detection_time;
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
typedef struct {
    char name[15];
    char unit[5];
    float min_val;
    float max_val;
    float anomaly_min;
    float anomaly_max;
    float variation;
    float value;
    float history[10];
} SensorConfig;

SensorConfig sensors[3];
int current_sensor_index = 0;
bool display_initialized = false;

// PIO para NeoPixels
PIO pio = pio0;
uint sm = 0;

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
static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)(r) << 16) | ((uint32_t)(g) << 8) | (uint32_t)(b); // GRB
}

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

void draw_horizontal_line(int x0, int y0, int length) {
    for (int i = 0; i < length; i++) {
        ssd1306_draw_pixel(x0 + i, y0, true);
    }
}

void play_startup_music() {
    uint notes[] = {261, 329, 392, 523};
    uint durations[] = {200, 200, 200, 400};
    for (int i = 0; i < 4; i++) {
        play_tone(notes[i], durations[i]);
        sleep_ms(50);
    }
}

void init_neopixels() {
    uint offset = pio_add_program(pio, &monitor_program);
    monitor_program_init(pio, sm, offset, OUT_PIN);
    // Teste inicial: acende todos os LEDs em azul para verificar funcionamento
    for (int i = 0; i < NUM_PIXELS; i++) {
        pio_sm_put_blocking(pio, sm, urgb_u32(0, 0, 255));
    }
    sleep_ms(1000); // Mantém aceso por 1 segundo
    for (int i = 0; i < NUM_PIXELS; i++) {
        pio_sm_put_blocking(pio, sm, 0); // Apaga
    }
}

static inline void put_pixel(uint32_t pixel_grb) {
    pio_sm_put_blocking(pio, sm, pixel_grb);
}

void update_neopixel_bars() {
    int heights[3] = {0, 0, 0};
    
    if (temp_enabled) {
        heights[0] = (int)((sensors[0].value - sensors[0].min_val) / (sensors[0].max_val - sensors[0].min_val) * 5 + 0.5);
        if (heights[0] > 5) heights[0] = 5;
        if (heights[0] < 0) heights[0] = 0;
    }
    if (flow_enabled) {
        heights[1] = (int)((sensors[1].value - sensors[1].min_val) / (sensors[1].max_val - sensors[1].min_val) * 5 + 0.5);
        if (heights[1] > 5) heights[1] = 5;
        if (heights[1] < 0) heights[1] = 0;
    }
    if (rain_enabled) {
        heights[2] = (int)((sensors[2].value - sensors[2].min_val) / (sensors[2].max_val - sensors[2].min_val) * 5 + 0.5);
        if (heights[2] > 5) heights[2] = 5;
        if (heights[2] < 0) heights[2] = 0;
    }

    for (int row = 0; row < 5; row++) {
        for (int col = 0; col < 5; col++) {
            int pixel_index = row * 5 + col;
            if (col < 3 && row < heights[col]) {
                float fraction = (float)row / (heights[col] > 1 ? heights[col] - 1 : 1);
                uint8_t r = (uint8_t)(fraction * 255);
                uint8_t b = (uint8_t)((1.0f - fraction) * 255);
                put_pixel(urgb_u32(r, 0, b));
            } else {
                put_pixel(urgb_u32(0, 0, 0));
            }
        }
    }
}

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
            put_pixel(urgb_u32(255, 0, 0));
        } else {
            put_pixel(urgb_u32(0, 0, 0));
        }
    }
}

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

void play_wildlife_alert() {
    if (!wildlife_enabled) return;
    for (int i = 0; i < 3; i++) {
        play_tone(440, 500);
        sleep_ms(100);
    }
}

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

void update_fire_alarm() {
    if (!fire_enabled) return;
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    uint32_t t = current_time % 5400; // Ciclo total de 5400ms
    bool alarm_on = false;

    // S: 3 pontos (200ms ON, 200ms OFF)
    if (t < 200) alarm_on = true;              // Ponto 1 ON
    else if (t >= 400 && t < 600) alarm_on = true; // Ponto 2 ON
    else if (t >= 800 && t < 1000) alarm_on = true; // Ponto 3 ON
    // Espaço entre S e O: 600ms (1000 a 1600)
    
    // O: 3 traços (600ms ON, 200ms OFF)
    else if (t >= 1600 && t < 2200) alarm_on = true; // Traço 1 ON
    else if (t >= 2400 && t < 3000) alarm_on = true; // Traço 2 ON
    else if (t >= 3200 && t < 3800) alarm_on = true; // Traço 3 ON
    // Espaço entre O e S: 600ms (3800 a 4400)
    
    // S: 3 pontos (200ms ON, 200ms OFF)
    else if (t >= 4400 && t < 4600) alarm_on = true; // Ponto 1 ON
    else if (t >= 4800 && t < 5000) alarm_on = true; // Ponto 2 ON
    else if (t >= 5200 && t < 5400) alarm_on = true; // Ponto 3 ON
    // Espaço final: 1400ms (não precisa de ação, ciclo reinicia)

    if (alarm_on) {
        gpio_put(LED_R_PIN, 1); // LED vermelho ON
        play_tone(650, 50);     // Tom curto (não bloqueia o ciclo)
    } else {
        gpio_put(LED_R_PIN, 0); // LED vermelho OFF
    }
    display_sos_neopixel(); // Sincroniza a matriz de LEDs
}

void detect_fire() {
    if (!fire_enabled || fire_alert_active) return;
    if (rand() % 1000 < 10) {
        fire_alert_active = true;
        fire_alert_start = to_ms_since_boot(get_absolute_time());
        printf("\n*** ALERTA DE INCENDIO: Fogo detectado na floresta! ***\n");
    }
}

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

void check_wildlife_alerts() {
    if (!wildlife_enabled) return;
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    if (wildlife_alert_active && last_detected_wildlife >= 0) {
        if (current_time - wildlife[last_detected_wildlife].detection_time > WILDLIFE_ALERT_DURATION_MS) {
            wildlife_alert_active = false;
        }
    }
}

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

void update_sensor_value(SensorConfig *sensor) {
    float new_value = simulate_reading(sensor);
    for (int i = 0; i < 9; i++) {
        sensor->history[i] = sensor->history[i + 1];
    }
    sensor->history[9] = new_value;
    sensor->value = new_value;
}

float calculate_moving_average(SensorConfig *sensor) {
    float sum = 0;
    for (int i = 0; i < 10; i++) {
        sum += sensor->history[i];
    }
    return sum / 10;
}

bool check_anomaly(SensorConfig *sensor) {
    return (sensor->value < sensor->anomaly_min || sensor->value > sensor->anomaly_max);
}

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
    draw_string(20, 20, "EMBARCATECH", false);
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
            update_fire_alarm();
        } else {
            update_neopixel_bars();
        }
        
        display_sensor_data();
        send_serial_data();
        sleep_ms(100);
    }
    
    return 0;
}