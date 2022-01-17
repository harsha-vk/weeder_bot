#include <main.h>
#include <pid.h>
#include <Wire.h>
#include <stdio.h>
#include <stdlib.h>
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

uint slice_num_1;
uint channel_1;
uint slice_num_2;
uint channel_2;

int left_tick_cnt = 0;
int prev_left_tick_cnt = 0;
int right_tick_cnt = 0;
int prev_right_tick_cnt = 0;

uint32_t prev_time = 0;
double left_vel = 0, left_pul = 0, left_set = 0;
PID left_pid(&left_vel, &left_pul, &left_set, LEFT_KP, LEFT_KI, LEFT_KD, DIRECT);
double right_vel = 0, right_pul = 0, right_set = 0;
PID right_pid(&right_vel, &right_pul, &right_set, RIGHT_KP, RIGHT_KI, RIGHT_KD, DIRECT);

void gpio_callback(uint gpio, uint32_t events)
{
    switch(gpio)
    {
        case LEFT_ENC_A:
            if(gpio_get(LEFT_ENC_B)) left_tick_cnt++;
            else left_tick_cnt--;
            break;
        case RIGHT_ENC_A:
            if(gpio_get(RIGHT_ENC_B)) right_tick_cnt--;
            else right_tick_cnt++;
            break;
        default:
            break;
    }
}

void pwm_out(int l_pul,int r_pul)
{
    if (l_pul > 0)
    {
        pwm_set_chan_level(slice_num_1, channel_1, (uint16_t)l_pul);
        gpio_put(OUT_DIR1, false);
    }
    else
    {
        pwm_set_chan_level(slice_num_1, channel_1, (uint16_t)abs(l_pul));
        gpio_put(OUT_DIR1, true);
    }

    if (r_pul > 0)
    {
        pwm_set_chan_level(slice_num_2, channel_2, (uint16_t)r_pul);
        gpio_put(OUT_DIR2, false);
    }
    else
    {
        pwm_set_chan_level(slice_num_2, channel_2, (uint16_t)abs(r_pul));
        gpio_put(OUT_DIR2, true);
    }
}

static void slave_on_receive(int count) // TODO
{
    while(Wire.available())
    {
        uint8_t data = (uint8_t)Wire.read();
    }
}

static void slave_on_request() // TODO
{
    size_t size;
    uint8_t value;
    Wire.write(&value,size);
}

int main()
{
    //stdio_init_all();
    gpio_set_irq_enabled_with_callback(LEFT_ENC_A, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
    gpio_pull_down(LEFT_ENC_A);

    gpio_init(LEFT_ENC_B);
    gpio_set_dir(LEFT_ENC_B, GPIO_IN);
    gpio_pull_down(LEFT_ENC_B);

    gpio_set_irq_enabled_with_callback(RIGHT_ENC_A, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
    gpio_pull_down(RIGHT_ENC_A);

    gpio_init(RIGHT_ENC_B);
    gpio_set_dir(RIGHT_ENC_B, GPIO_IN);
    gpio_pull_down(RIGHT_ENC_B);

    gpio_init(OUT_DIR1);
    gpio_set_dir(OUT_DIR1, GPIO_OUT);

    gpio_set_function(OUT_PWM1, GPIO_FUNC_PWM);
    slice_num_1 = pwm_gpio_to_slice_num(OUT_PWM1);
    channel_1 = pwm_gpio_to_channel(OUT_PWM1);
    pwm_set_clkdiv(slice_num_1, PWM_CLKDIV);
    pwm_set_wrap(slice_num_1, PWM_TOP);
    pwm_set_chan_level(slice_num_1, channel_1, 0);
    pwm_set_enabled(slice_num_1, true);

    gpio_init(OUT_DIR2);
    gpio_set_dir(OUT_DIR2, GPIO_OUT);

    gpio_set_function(OUT_PWM2, GPIO_FUNC_PWM);
    slice_num_2 = pwm_gpio_to_slice_num(OUT_PWM2);
    channel_2 = pwm_gpio_to_channel(OUT_PWM2);
    pwm_set_clkdiv(slice_num_2, PWM_CLKDIV);
    pwm_set_wrap(slice_num_2, PWM_TOP);
    pwm_set_chan_level(slice_num_2, channel_2, 0);
    pwm_set_enabled(slice_num_2, true);

    gpio_set_function(PICO_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PICO_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_I2C_SDA);
    gpio_pull_up(PICO_I2C_SCL);
    i2c_init(PICO_I2C, PICO_I2C_BAUDRATE);
    Wire.onReceive(slave_on_receive);
    Wire.onRequest(slave_on_request);
    Wire.begin(PICO_I2C_ADDR);

    left_pid.SetMode(AUTOMATIC);
    left_pid.SetSampleTime(SAMPLE_TIME_US);
    left_pid.SetOutputLimits(-1 * PWM_TOP, PWM_TOP);

    right_pid.SetMode(AUTOMATIC);
    right_pid.SetSampleTime(SAMPLE_TIME_US);
    right_pid.SetOutputLimits(-1 * PWM_TOP, PWM_TOP);
    
    while(true)
    {
        if((time_us_32() - prev_time) > PERIOD_IN_US)
        {
            left_vel = VEL_CONST * (left_tick_cnt - prev_left_tick_cnt);
            right_vel = VEL_CONST * (right_tick_cnt - prev_left_tick_cnt);

            prev_left_tick_cnt = left_tick_cnt;
            prev_right_tick_cnt = right_tick_cnt;
            prev_time = time_us_32();
        }
        left_pid.Compute();
        right_pid.Compute();
        pwm_out((int)left_pul,(int)right_pul);
        sleep_ms(10);
    }
    return 0;
}