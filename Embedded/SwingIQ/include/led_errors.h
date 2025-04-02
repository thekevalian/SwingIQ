#ifndef LED_ERRORS_H
#define LED_ERRORS_H
#include <stdint.h>
typedef enum led_err{
    NO_ERR      = 0,
    SD_FAIL     = 1,
    IMU_FAIL    = 2,
    BLE_FAIL    = 3,
    NUM_LED_ERR = 4
} led_err_t;

#define FLASH_DEBUG false

void init_err(uint8_t pin);

const int16_t duration_ms[NUM_LED_ERR] = {2000, -1, -1, -1}; // 2 s, infinite, infinite
const int16_t flash_period_ms[NUM_LED_ERR] = {10, 100, 500, 500}; // 50 Hz, 10 Hz, 2 Hz

void flash_err(led_err_t e);

void led_off(void);

void led_on(void);

#endif