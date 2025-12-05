#include "functions.h"

ledstate_t led_state;
button_sm_t buttons[BTN_COUNT];
const uint led_pins[LED_COUNT] = LED_PINS;
const uint btn_pins[BTN_COUNT] = BTN_PINS;
uint8_t pressed_level = 0;

void set_led_state(ledstate_t *ls, uint8_t value) {
    ls->state = value;
    ls->not_state = ~value;
}

bool led_state_is_valid(ledstate_t *ls) {
    return ls->state == (uint8_t)(~ls->not_state);
}

int eeprom_write(uint16_t addr, uint8_t *data, size_t len) {
    if (len > LOG_ENTRY_SIZE) {
        return -1;
    }
    uint8_t tx[2 + len];
    tx[0] = (uint8_t)(addr >> 8);
    tx[1] = (uint8_t)addr & 0xFF;
    for (int i = 0; i < len; i++) {
        tx[2 + i] = data[i];
    }
     int write=i2c_write_blocking(I2C_PORT, EEPROM_I2C_ADDR,
                               tx, 2 + len, false);
    if (write !=2+len) {
        return -1; //error
    }
    sleep_ms(5);

    return 0;
}
int eeprom_read(uint16_t addr, uint8_t *data, size_t len) {
    if (len > LOG_ENTRY_SIZE) {
        return -1;
    }
    uint8_t tx[2];
    tx[0] = (uint8_t)(addr >> 8);
    tx[1] = (uint8_t) (addr& 0xFF);

    i2c_write_blocking(I2C_PORT, EEPROM_I2C_ADDR, tx, 2, true);
    int read= i2c_read_blocking(I2C_PORT, EEPROM_I2C_ADDR, data, len, false);
    if (read != (int)len) {
        return -1;
    }
    return 0;
}
void button_init(button_sm_t *b, uint gpio) {
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
        if (b->counter >= DEBOUNCE) {
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
            gpio_put(led_pins[i], 1);
        } else {
            gpio_put(led_pins[i], 0);
        }
    }
}

void setup_leds(void) {
    for (int i=0;i<LED_COUNT;i++) {
        gpio_init(led_pins[i]);
        gpio_set_dir(led_pins[i], GPIO_OUT);
        gpio_put(led_pins[i], 0);
    }
}

void setup_i2c(void) {
    i2c_init(I2C_PORT, I2C_BAUDRATE);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
}
void setup_buttons(void) {
    for (int i=0; i<BTN_COUNT; i++) {
        gpio_init(btn_pins[i]);
        gpio_set_dir(btn_pins[i], GPIO_IN);
        gpio_pull_up(btn_pins[i]);
    }
}

void print_led_state(void) {
    uint64_t t_us = time_us_64();
    uint32_t t_s = t_us / MICROSECOND_TO_SECOND;

    printf("LED {1,2,3} state {");
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
    printf("} [%u s]\n", t_s);
    //const char *string = "hello";
    //printf("%s\n", string);
}



