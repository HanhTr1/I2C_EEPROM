#include "pico/stdlib.h"
#include "hardware/gpio.h"
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
#define EEPROM_STORE_ADDR (32768 - sizeof(ledstate_t))  // highest address avoid overflow
#define EEPROM_TOTAL_BYTES 32768u
#define POLL_MS 10
#define DEBOUNCE_TICKS 5
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
typedef enum {
    EEPROM_OK = 0,
    EEPROM_ERROR_BUF,
    EEPROM_ERROR_ADDR_OVERFLOW,
    EEPROM_ERROR_I2C_WRITE,
    EEPROM_ERROR_I2C_READ,
    EEPROM_ERROR_I2C_TIMEOUT

} eeprom_status_t;

static const uint led_pins[LED_COUNT] = LED_PINS;
static const uint btn_pins[BTN_COUNT] = BTN_PINS;

static ledstate_t led_state;
static button_sm_t buttons[BTN_COUNT];
static uint8_t pressed_level = 0;

void set_led_state(ledstate_t *ls, uint8_t value) {
    ls->state = value;
    ls->not_state = ~value;
}


bool led_state_is_valid(ledstate_t *ls) {
    return ls->state == (uint8_t)(~ls->not_state);
}

int eeprom_write_bytes(uint32_t addr, const uint8_t *buf, size_t len) {
    const uint32_t PAGE_SIZE = 64;  // AT24C256 page size
    size_t remaining = len;
    size_t offset = 0;

    while (remaining > 0) {
        uint32_t current_addr = addr + offset;
        uint32_t page_offset = current_addr % PAGE_SIZE;
        uint32_t space_in_page = PAGE_SIZE - page_offset;

        uint32_t chunk;
        if (remaining < space_in_page) {
            chunk = remaining;
        } else {
            chunk = space_in_page;
        }

        uint8_t tx[2 + PAGE_SIZE];
        tx[0] = (uint8_t)(current_addr >> 8);    // MSB addr
        tx[1] = (uint8_t)(current_addr & 0xFF);  // LSB addr

        for (uint32_t i = 0; i < chunk; i++) {
            tx[2 + i] = buf[offset + i];
        }

        int w = i2c_write_blocking(I2C_PORT, EEPROM_I2C_ADDR, tx, 2 + chunk, false);
        if (w < 0 || w != (int)(2 + chunk)) {
            return EEPROM_ERROR_I2C_WRITE;
        }

        // ACK polling
        absolute_time_t start_time = get_absolute_time();
        int probe = 0;
        while ((probe != 1) && (absolute_time_diff_us(get_absolute_time(), start_time) < 5000)) { // 5 ms timeout
            uint8_t dummy = 0;
            probe = i2c_write_blocking(I2C_PORT, EEPROM_I2C_ADDR, &dummy, 1, false);
            if (probe != 1) {
                sleep_ms(1);
            }
        }

        if (probe != 1) {
            return EEPROM_ERROR_I2C_TIMEOUT;
        }

        remaining -= chunk;
        offset += chunk;
    }

    return 0;
}


eeprom_status_t eeprom_read_bytes(uint32_t addr, uint8_t *buf, size_t len) {
    if (!buf || len == 0) {
        return EEPROM_ERROR_BUF;
    }
    if (addr + len > EEPROM_TOTAL_BYTES) {
        return EEPROM_ERROR_ADDR_OVERFLOW;
    }
    uint8_t addr_divided[2] = {(uint8_t)(addr >> 8), (uint8_t)(addr & 0xFF)};
                                //move 8bit to right(high byte)& low byte
    int w = i2c_write_blocking(I2C_PORT, EEPROM_I2C_ADDR, addr_divided, 2, false);
    if (w != 2) {
        return EEPROM_ERROR_I2C_WRITE;
    }

    int r = i2c_read_blocking(I2C_PORT, EEPROM_I2C_ADDR, buf, len, false);
    if (r != (int)len) {
        return EEPROM_ERROR_I2C_READ;
    }

    return EEPROM_OK;
}
void button_sm_init(button_sm_t *b, uint gpio) {
    b->gpio_pin = gpio;
    b->last_sample = 1;
    b->stable_level = 1;
    b->counter = 0;
    b->pressed_event = false;
}

void button_sm_sample(button_sm_t *b) {
    uint8_t level = gpio_get(b->gpio_pin);
    b->last_sample = level;

    if (b->last_sample == b->stable_level) {
        b->counter = 0;
        b->pressed_event = false;
    } else {
        b->counter++;
        if (b->counter >= DEBOUNCE_TICKS) {
            b->stable_level = b->last_sample;
            b->counter = 0;
            b->pressed_event = (b->stable_level == pressed_level);
        } else {
            b->pressed_event = false;
        }
    }
}

void update_leds(void) {
    for (int i = 0; i < LED_COUNT; i++) {
        if (led_state.state & LED_BIT(i)) {
            gpio_put(led_pins[i], 1); // on
        } else {
            gpio_put(led_pins[i], 0); // off
        }
    }
}



void detect_button(void) {
    for (int i=0;i<BTN_COUNT;i++) {
        gpio_init(btn_pins[i]);
        gpio_set_dir(btn_pins[i], GPIO_IN);
        gpio_pull_up(btn_pins[i]);
    }
    pressed_level=0;
    printf("Button pressed active low \n");
}

void setup_leds(void) {
    for (int i=0;i<LED_COUNT;i++) {
        gpio_init(led_pins[i]);
        gpio_set_dir(led_pins[i], GPIO_OUT);
        gpio_put(led_pins[i], 0); //off all led before load state
    }
}

void setup_i2c(void) {
    i2c_init(I2C_PORT, I2C_BAUDRATE);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
}
void print_led_state(void) {
    uint64_t t_us = time_us_64();
    uint32_t t_s = t_us / MICROSECOND_TO_SECOND;

    printf("[%u s] LED {", t_s);
    for (int i = 0; i < LED_COUNT; i++) {
        printf("%d", i + 1);
        if (i < LED_COUNT - 1) {
            printf(",");
        }
    }
    printf("} state {");
    for (int i = 0; i < LED_COUNT; i++) {
        uint8_t led_status;
        if (led_state.state & LED_BIT(i)) {
            led_status = 1;
        } else {
            led_status = 0;
        }
        printf("%u", led_status);
        if (i < LED_COUNT - 1) {
            printf(",");
        }
    }
    printf("}\n");
}



int main() {
    stdio_init_all();
    sleep_ms(200);
    printf("Start!\n");

    setup_i2c();
    setup_leds();
    detect_button();
    for (int i = 0; i < BTN_COUNT; i++) {
        button_sm_init(&buttons[i], btn_pins[i]);
    }


    // read LED state from EEPROM
    if (eeprom_read_bytes(EEPROM_STORE_ADDR, (uint8_t*)&led_state, sizeof(ledstate_t)) == EEPROM_OK
        && led_state_is_valid(&led_state)) {
        printf("Loaded LED state from EEPROM.\n");
    } else {
        set_led_state(&led_state, LED_BIT(1)); // default middle LED
        eeprom_write_bytes(EEPROM_STORE_ADDR, (uint8_t*)&led_state, sizeof(ledstate_t));
        printf("Using default LED state.\n");
    }

    update_leds();

    bool prev_btn_state[BTN_COUNT] = {0};

    while (true) {
        for (int i=0;i<BTN_COUNT;i++) {
            button_sm_sample(&buttons[i]);
        }
        for (int i=0; i<BTN_COUNT; i++) {
            bool pressed =buttons[i].pressed_event;
            if (pressed && prev_btn_state[i] == false) {
                set_led_state(&led_state, led_state.state ^ LED_BIT(i));
                eeprom_write_bytes(EEPROM_STORE_ADDR, (uint8_t*)&led_state, sizeof(ledstate_t));
                update_leds();
                print_led_state();
            }
            prev_btn_state[i] = pressed;
        }

        sleep_ms(POLL_MS);
    }
}
