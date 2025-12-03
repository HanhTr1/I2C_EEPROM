#include "functions.h"

int main() {
    stdio_init_all();
    sleep_ms(200);
    //printf("Start Exercise 2!\n");

    setup_i2c();
    setup_leds();
    setup_buttons();

    for (int i = 0; i < BTN_COUNT; i++) {
        button_init(&buttons[i], btn_pins[i]);
    }

    if (!eeprom_available() || eeprom_read(EEPROM_STORE_ADDR, (uint8_t*)&led_state,sizeof(led_state))!=0
        || !led_state_is_valid(&led_state)) {
        printf("EEPROM not found or invalid. Using default LED state.\n");
        set_led_state(&led_state,LED_BIT(1));
        if (eeprom_available()) {
            eeprom_write(EEPROM_STORE_ADDR, (uint8_t*)&led_state, sizeof(led_state));
        }
    } else {
        printf("Loaded LED state from EEPROM\n");

    }

    update_leds();
    print_led_state();
    if (eeprom_available()) {
        write_log("Boot");
    }

    bool prev_btn_state[BTN_COUNT] = {0};
    char command[COMMAND_SIZE] = {0};
    int command_idx = 0;

    while (1) {

        for (int i = 0; i < BTN_COUNT; i++) {
            button_sm_sample(&buttons[i]);
        }
        for (int i = 0; i < BTN_COUNT; i++) {
            bool pressed = buttons[i].pressed_event;
            if (pressed && !prev_btn_state[i]) {
                set_led_state(&led_state, led_state.state ^ LED_BIT(i));
                update_leds();
                print_led_state();

                if (eeprom_available()) {
                    eeprom_write(EEPROM_STORE_ADDR, (uint8_t*)&led_state, sizeof(led_state));

                    char logmsg[64];
                    snprintf(logmsg, sizeof(logmsg),
                         "LED state change: %u,%u,%u",
                         !!(led_state.state & LED_BIT(0)),
                         !!(led_state.state & LED_BIT(1)),
                         !!(led_state.state & LED_BIT(2)));
                    write_log(logmsg);
                }
            }
            prev_btn_state[i] = pressed;
        }
        validate_command( command, &command_idx);
        sleep_ms(POLL_MS);
    }
}
