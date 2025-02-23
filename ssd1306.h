/**
 * @file ssd1306.h
 * @brief Interface do driver para display OLED SSD1306
 *
 * Define constantes e funções para controle do display OLED SSD1306.
 * Suporta display 128x64 pixels via comunicação I2C.
 */
#ifndef SSD1306_H
#define SSD1306_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// Display dimensões 
/** @defgroup DisplayConfig Configurações do Display
 * @{
 */
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64

// I2C configuração
/** @defgroup I2CConfig Configurações I2C
 * @{
 */
#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define endereco 0x3C

// Display buffer
extern uint8_t buffer[DISPLAY_HEIGHT/8][DISPLAY_WIDTH];  // Reorganizado para páginas

// Declaração de funções
/**
 * @brief Envia comando para o display
 * @param cmd Byte de comando
 */
void ssd1306_send_command(uint8_t cmd);
/**
 * @brief Envia dados para o display
 * @param data Ponteiro para dados
 * @param len Quantidade de bytes
 */
void ssd1306_send_data(uint8_t *data, size_t len);
/**
 * @brief Inicializa o display
 */
void ssd1306_init(void);
/**
 * @brief Limpa o buffer do display
 */
void ssd1306_clear(void);
/**
 * @brief Desenha um pixel
 * @param x Coordenada X (0-127)
 * @param y Coordenada Y (0-63)
 * @param color Estado do pixel (true=aceso)
 */
void ssd1306_draw_pixel(int x, int y, bool color);

/**
 * @brief Atualiza o display com o buffer
 */
void ssd1306_update(void);

/**
 * @brief Define endereço das páginas
 * @param start Página inicial (0-7)
 * @param end Página final (0-7)
 */
void ssd1306_set_page_address(uint8_t start, uint8_t end);
/**
 * @brief Define endereço das colunas
 * @param start Coluna inicial (0-127)
 * @param end Coluna final (0-127)
 */
void ssd1306_set_column_address(uint8_t start, uint8_t end);

#endif // SSD1306_H

