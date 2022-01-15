#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"
// Input pins
#define LEFT_ENC_A 10
#define LEFT_ENC_B 11
#define RIGHT_ENC_A 12
#define RIGHT_ENC_B 13
// Output pins
#define OUT_DIR1 18
#define OUT_PWM1 19
#define OUT_DIR2 20
#define OUT_PWM2 21
// Communication pins
#define PICO_I2C_SDA 4
#define PICO_I2C_SCL 5

#define PWM_CLKDIV 12.2
#define PWM_TOP 1024

#define PICO_I2C (&i2c0_inst)
#define PICO_I2C_BAUDRATE (400 * 1000)
#define PICO_I2C_ADDR 0x75