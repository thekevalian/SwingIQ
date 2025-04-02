#include <led_errors.h>
#include <Arduino.h>

static uint8_t led_pin = -1;

#define LED_LOW() digitalWrite(led_pin, HIGH)
#define LED_HIGH() digitalWrite(led_pin, LOW)
#define LED_TOGGLE() digitalWrite(led_pin, !digitalRead(led_pin))

void init_err(uint8_t pin){
    led_pin = pin;
    pinMode(led_pin, OUTPUT);
    LED_LOW();
}

void flash_err(led_err_t e){
    #ifdef FLASH_DEBUG
    for(int i = 0; (i < duration_ms[e]) || (duration_ms[e] < 0); i += flash_period_ms[e]){
        LED_TOGGLE();
        delay(flash_period_ms[e]);
    }
    LED_LOW();
    #endif
}

void led_off(void){
    LED_LOW();
}
void led_on(void){
    LED_HIGH();
}