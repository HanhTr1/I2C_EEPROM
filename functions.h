#ifndef FUNCTIONS_H
#define FUNCTIONS_H
#include "pico/stdlib.h"
#include <ctype.h>
#include "hardware/i2c.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#define SW0_PIN 9
#define SW1_PIN 8
#define SW2_PIN 7

#define LED1_PIN 20
#define LED2_PIN 21
#define LED3_PIN 22

#define LED_COUNT 3
#define BTN_COUNT 3

#define LED_PINS {LED1_PIN, LED2_PIN, LED3_PIN}
#define BTN_PINS {SW0_PIN, SW1_PIN, SW2_PIN}

#define I2C_PORT i2c0
#define I2C_SDA_PIN 16
#define I2C_SCL_PIN 17

#define I2C_BAUDRATE 100000
#define EEPROM_I2C_ADDR 0x50
#define EEPROM_STORE_ADDR (0x7fff-2)    // highest address avoid overflow
#define EEPROM_TOTAL_BYTES 32768u
#define POLL_MS 10
#define DEBOUNCE 5
#define MICROSECOND_TO_SECOND 1000000
#define LED_BIT(n) (1u << (n))


typedef struct ledstate {
    uint8_t state;
    uint8_t not_state;
} ledstate_t;


typedef struct {
    uint gpio_pin;
    uint8_t stable_level;
    uint8_t last_sample;
    uint8_t counter;
    bool pressed_event;
} button_sm_t;

extern ledstate_t led_state;
extern button_sm_t buttons[BTN_COUNT];
extern const uint led_pins[LED_COUNT];
extern const uint btn_pins[BTN_COUNT];
extern uint8_t pressed_level;
void set_led_state(ledstate_t *ls, uint8_t value);

bool led_state_is_valid(ledstate_t *ls);
int eeprom_write(uint16_t addr, uint8_t *data, size_t len);

int eeprom_read(uint16_t addr, uint8_t *data, size_t len);
void button_init(button_sm_t *b, uint gpio);

void button_sm_sample(button_sm_t *b);

void update_leds(void);

void setup_leds(void);

void setup_i2c(void);
void setup_buttons(void);

void print_led_state(void);

//exercise2_extra---------------
#define LOG_START_ADDR 0
#define LOG_ENTRY_SIZE 64
#define LOG_AREA_SIZE 2048
#define LOG_MAX_ENTRIES 32
#define COMMAND_SIZE 8
#define LOG_STRING_MAX_LEN 61
uint16_t crc16(const uint8_t *data_p, size_t length);
bool eeprom_available();
int find_log();
void erase_log() ;
void write_log( char *msg);
void read_log();
void validate_command( char *command, int *command_idx);

#endif // FUNCTIONS_H
