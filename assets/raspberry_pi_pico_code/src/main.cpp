#include <main.h>
#include <pid.h>
#include <Wire.h>
#include <stdio.h>
#include <stdlib.h>
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "pico/time.h"

uint slice_num_1;
uint channel_1;
uint slice_num_2;
uint channel_2;

int left_tick_cnt = 0;
int prev_left_tick_cnt = 0;
int right_tick_cnt = 0;
int prev_right_tick_cnt = 0;

uint32_t prev_time = 0;
double left_rpm = 0, left_pul = 0, left_set = 0;
PID left_pid(&left_rpm, &left_pul, &left_set, LEFT_KP, LEFT_KI, LEFT_KD, DIRECT);
double right_rpm = 0, right_pul = 0, right_set = 0;
PID right_pid(&right_rpm, &right_pul, &right_set, RIGHT_KP, RIGHT_KI, RIGHT_KD, DIRECT);
int prev_left_pul = 0;
int prev_right_pul = 0;

bool start_loop = false;

void gpio_callback(uint gpio, uint32_t events)
{
    if(gpio == LEFT_ENC_A && events == GPIO_IRQ_EDGE_RISE)
    {
        if(gpio_get(LEFT_ENC_B)) left_tick_cnt++;
        else left_tick_cnt--;
    }
    else if(gpio == RIGHT_ENC_A && events == GPIO_IRQ_EDGE_RISE)
    {
        if(gpio_get(RIGHT_ENC_B)) right_tick_cnt--;
        else right_tick_cnt++;
    }
}

void pwm_out(int l_pul,int r_pul)
{
    if(l_pul != prev_left_pul);
    {
        if (l_pul > 0)
        {
            pwm_set_chan_level(slice_num_1, channel_1, (uint16_t)l_pul);
            gpio_put(OUT_DIR1, true);
        }
        else
        {
            pwm_set_chan_level(slice_num_1, channel_1, (uint16_t)abs(l_pul));
            gpio_put(OUT_DIR1, false);
        }
        prev_left_pul = l_pul;
    }

    if(r_pul != prev_right_pul)
    {
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
        prev_right_pul = r_pul;
    }
}

static void slave_on_request()
{
    // serialize data
    uint8_t buff[11];
    union {float real;uint32_t base;} u_data;
    int chk = 0;
    buff[0] = 0xFF;
    buff[1] = 0xFE;
    u_data.real = left_rpm * VEL_CONST / RPM_CONST;
    buff[2] = (u_data.base >> 0) & 0xFF;
    buff[3] = (u_data.base >> 8) & 0xFF;
    buff[4] = (u_data.base >> 16) & 0xFF;
    buff[5] = (u_data.base >> 24) & 0xFF;
    u_data.real = right_rpm * VEL_CONST / RPM_CONST;
    buff[6] = (u_data.base >> 0) & 0xFF;
    buff[7] = (u_data.base >> 8) & 0xFF;
    buff[8] = (u_data.base >> 16) & 0xFF;
    buff[9] = (u_data.base >> 24) & 0xFF;
    for(int i = 2; i < 10; i++) chk += buff[i];
    buff[10] = 255 - (chk % 256);
    Wire.write(buff, 11);
}

static void slave_on_receive(int count)
{
    // deserialize data
    hard_assert(Wire.available());
    uint8_t buff[11];
    union {float real;uint32_t base;} u_data;
    int chk = 0;
    while(Wire.available())
    {
        buff[0] = (uint8_t)Wire.read();
        if(buff[0] != 0xFF) continue;
        buff[1] = (uint8_t)Wire.read();
        if(buff[1] != 0xFE) continue;
        if(Wire.available() < 9) continue;
        for(int i = 2; i < 11; i++)
        {
            buff[i] = (uint8_t)Wire.read();
            chk += buff[i];
        }
        if((chk % 256) != 255)
        {
            chk = 0;
            continue;
        }
        u_data.base = 0;
        u_data.base |= ((uint32_t)buff[2] << 0);
        u_data.base |= ((uint32_t)buff[3] << 8);
        u_data.base |= ((uint32_t)buff[4] << 16);
        u_data.base |= ((uint32_t)buff[5] << 24);
        left_set = u_data.real * RPM_CONST / VEL_CONST;
        u_data.base = 0;
        u_data.base |= ((uint32_t)buff[6] << 0);
        u_data.base |= ((uint32_t)buff[7] << 8);
        u_data.base |= ((uint32_t)buff[8] << 16);
        u_data.base |= ((uint32_t)buff[9] << 24);
        right_set = u_data.real * RPM_CONST / VEL_CONST;
    }
    start_loop = true;
}

int main()
{
    stdio_init_all();
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
    Wire.onRequest(slave_on_request);
    Wire.onReceive(slave_on_receive);
    Wire.begin(PICO_I2C_ADDR);

    left_pid.SetMode(AUTOMATIC);
    left_pid.SetSampleTime(SAMPLE_TIME_US);
    left_pid.SetOutputLimits(-1 * PWM_TOP, PWM_TOP);

    right_pid.SetMode(AUTOMATIC);
    right_pid.SetSampleTime(SAMPLE_TIME_US);
    right_pid.SetOutputLimits(-1 * PWM_TOP, PWM_TOP);
    
    while(true)
    {
        if(!start_loop) continue;
        uint32_t now_ = time_us_32();
        if((now_ - prev_time) > (uint32_t)PERIOD_IN_US)
        {
            int left_tick_diff = left_tick_cnt - prev_left_tick_cnt;
            int right_tick_diff = right_tick_cnt - prev_right_tick_cnt;
            left_rpm = RPM_CONST * ((float)left_tick_diff);
            right_rpm = RPM_CONST * ((float)right_tick_diff);
            prev_left_tick_cnt = left_tick_cnt;
            prev_right_tick_cnt = right_tick_cnt;
            prev_time = now_;
            printf("left %f\t%f\n", left_rpm, left_set);
            printf("right %f\t%f\n", right_rpm, right_set);
        }
        left_pid.Compute();
        right_pid.Compute();
        pwm_out((int)left_pul,(int)right_pul);
    }
    return 0;
}