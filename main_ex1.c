#include "functions.h"

int main() {
    stdio_init_all();
    sleep_ms(200);
    printf("Start Exercise 1!\n");

    setup_i2c();
    setup_leds();
    setup_buttons();
    for (int i = 0; i < BTN_COUNT; i++) {
        button_init(&buttons[i], btn_pins[i]);
    }
    bool eeprom_available = true;


    if (eeprom_read(EEPROM_STORE_ADDR, (uint8_t*)&led_state, sizeof(led_state)) == 0
         && led_state_is_valid(&led_state)) {
        printf("Loaded LED state from EEPROM.\n");
         }
    else {
        set_led_state(&led_state, LED_BIT(1));
        eeprom_write(EEPROM_STORE_ADDR, (uint8_t*)&led_state, sizeof(led_state));
            eeprom_available = false;
            printf("EEPROM not found, no valid state.\n");
            printf("Using default LED state.\n");
        }

    update_leds();
    print_led_state();
    if (eeprom_available) {
        printf("EEPROM is now connected.\n");
    }

    bool prev_btn_state[BTN_COUNT] = {0};

    while (1) {
        for (int i = 0; i < BTN_COUNT; i++) button_sm_sample(&buttons[i]);

        for (int i = 0; i < BTN_COUNT; i++) {
            bool pressed = buttons[i].pressed_event;
            if (pressed && prev_btn_state[i] == false) {
                set_led_state(&led_state, led_state.state ^ LED_BIT(i));
                eeprom_write(EEPROM_STORE_ADDR, (uint8_t*)&led_state, sizeof(led_state));
                update_leds();
                print_led_state();
            }
            prev_btn_state[i] = pressed;
        }
        sleep_ms(POLL_MS);
    }
}
