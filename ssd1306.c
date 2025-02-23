/**
 * @file ssd1306.c
 * @brief Driver para display OLED SSD1306
 *
 * Implementa funções para controle do display OLED SSD1306 via I2C.
 * Suporta display 128x64 pixels organizado em 8 páginas de 128 bytes.
 */
#include "ssd1306.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <string.h>
#include <stdlib.h>

// Display buffer reorganizado para páginas
uint8_t buffer[DISPLAY_HEIGHT/8][DISPLAY_WIDTH];

/** Buffer do display organizado em páginas */
uint8_t buffer[DISPLAY_HEIGHT/8][DISPLAY_WIDTH];

/**
 * @brief Envia comando para o display
 * @param cmd Byte de comando a ser enviado
 */
void ssd1306_send_command(uint8_t cmd) {
    uint8_t buf[2] = {0x00, cmd};  // 0x00 indica comando
    i2c_write_blocking(I2C_PORT, endereco, buf, 2, false);
}
/**
 * @brief Envia dados para o display
 * @param data Ponteiro para os dados
 * @param len Quantidade de bytes a enviar
 */
void ssd1306_send_data(uint8_t *data, size_t len) {
    // Primeiro byte 0x40 indica dados
    uint8_t *temp_buffer = malloc(len + 1);
    temp_buffer[0] = 0x40;
    memcpy(temp_buffer + 1, data, len);
    i2c_write_blocking(I2C_PORT, endereco, temp_buffer, len + 1, false);
    free(temp_buffer);
}
/**
 * @brief Inicializa o display OLED
 * 
 * Configura I2C e envia sequência de inicialização:
 * - Configura clock e multiplexação
 * - Ativa charge pump
 * - Define modo de endereçamento horizontal
 * - Configura contraste e níveis de tensão
 */
void ssd1306_init() {
    sleep_ms(100);  // Aguarda inicialização do display
  
    // Inicializa I2C
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Sequência de inicialização do display
    ssd1306_send_command(0xAE);  // Display off
    ssd1306_send_command(0xD5);  // Set display clock
    ssd1306_send_command(0x80);
    ssd1306_send_command(0xA8);  // Set multiplex
    ssd1306_send_command(0x3F);  // 1/64 duty
    ssd1306_send_command(0xD3);  // Set display offset
    ssd1306_send_command(0x00);  // No offset
    ssd1306_send_command(0x40);  // Start line address
    ssd1306_send_command(0x8D);  // Charge pump
    ssd1306_send_command(0x14);  // Enable charge pump
    ssd1306_send_command(0x20);  // Memory mode
    ssd1306_send_command(0x00);  // Horizontal addressing
    ssd1306_send_command(0xA1);  // Segment remap
    ssd1306_send_command(0xC8);  // COM scan direction
    ssd1306_send_command(0xDA);  // COM pins
    ssd1306_send_command(0x12);
    ssd1306_send_command(0x81);  // Contrast
    ssd1306_send_command(0xCF);  // Maximum
    ssd1306_send_command(0xD9);  // Pre-charge period
    ssd1306_send_command(0xF1);
    ssd1306_send_command(0xDB);  // VCOMH deselect level
    ssd1306_send_command(0x30);
    ssd1306_send_command(0xA4);  // Display all on resume
    ssd1306_send_command(0xA6);  // Normal display
    ssd1306_send_command(0xAF);  // Display on

    ssd1306_clear();
    ssd1306_update();
}
/**
 * @brief Limpa o buffer do display
 * 
 * Preenche todo o buffer com zeros
 */
void ssd1306_clear() {
    memset(buffer, 0, sizeof(buffer));
}
/**
 * @brief Desenha um pixel no buffer
 * 
 * @param x Coordenada X (0-127)
 * @param y Coordenada Y (0-63)
 * @param color true para pixel aceso, false para apagado
 */
void ssd1306_draw_pixel(int x, int y, bool color) {
    if (x < 0 || x >= DISPLAY_WIDTH || y < 0 || y >= DISPLAY_HEIGHT) 
        return;

    // Calcula a página (y/8) e o bit dentro da página
    uint8_t page = y / 8;
    uint8_t bit = y % 8;

    if (color) {
        buffer[page][x] |= (1 << bit);
    } else {
        buffer[page][x] &= ~(1 << bit);
    }
}
/**
 * @brief Define endereço das páginas
 * 
 * @param start Página inicial (0-7)
 * @param end Página final (0-7)
 */
void ssd1306_set_page_address(uint8_t start, uint8_t end) {
    ssd1306_send_command(0x22);  // Page address command
    ssd1306_send_command(start & 0x07);
    ssd1306_send_command(end & 0x07);
}
/**
 * @brief Define endereço das colunas
 * 
 * @param start Coluna inicial (0-127)
 * @param end Coluna final (0-127)
 */
void ssd1306_set_column_address(uint8_t start, uint8_t end) {
    ssd1306_send_command(0x21);  // Column address command
    ssd1306_send_command(start & 0x7F);
    ssd1306_send_command(end & 0x7F);
}
/**
 * @brief Atualiza o display com o conteúdo do buffer
 * 
 * Envia o buffer completo para o display página por página
 */
void ssd1306_update() {
    // Define o endereço inicial
    ssd1306_set_page_address(0, 7);
    ssd1306_set_column_address(0, DISPLAY_WIDTH-1);
    
    // Envia o buffer página por página
    for (int page = 0; page < DISPLAY_HEIGHT/8; page++) {
        ssd1306_send_data(buffer[page], DISPLAY_WIDTH);
    }
}