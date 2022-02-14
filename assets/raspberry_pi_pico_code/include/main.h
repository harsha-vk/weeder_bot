#include <math.h>
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
#define PWM_TOP 1023

#define PICO_I2C i2c0
#define PICO_I2C_ADDR 0x55
#define PICO_I2C_BAUDRATE (100 * 1000)

#define LEFT_KD 0.05
#define LEFT_KI 4.8
#define LEFT_KP 1.9

#define RIGHT_KD 0.035
#define RIGHT_KI 4.84
#define RIGHT_KP 1.92

#define SAMPLE_TIME_US 2000

#define PULSE_PER_ROTATION 134.4
#define PERIOD_IN_S 0.2
#define PERIOD_IN_US (PERIOD_IN_S * 1000000)
#define WHEEL_DIAMETER 0.1

#define LEN_PER_PULSE ((M_PI * WHEEL_DIAMETER) / PULSE_PER_ROTATION)
#define VEL_CONST (LEN_PER_PULSE / PERIOD_IN_S)
#define RPM_CONST (60.0 / (PULSE_PER_ROTATION * PERIOD_IN_S))