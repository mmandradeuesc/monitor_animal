#ifndef SSD1306_H
#define SSD1306_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// Display dimensões 
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64

// I2C configuração
#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define endereco 0x3C

// Display buffer
extern uint8_t buffer[DISPLAY_HEIGHT/8][DISPLAY_WIDTH];  // Reorganizado para páginas

// Declaração de funções
void ssd1306_send_command(uint8_t cmd);
void ssd1306_send_data(uint8_t *data, size_t len);
void ssd1306_init(void);
void ssd1306_clear(void);
void ssd1306_draw_pixel(int x, int y, bool color);
void ssd1306_update(void);
void ssd1306_set_page_address(uint8_t start, uint8_t end);
void ssd1306_set_column_address(uint8_t start, uint8_t end);

#endif // SSD1306_H

