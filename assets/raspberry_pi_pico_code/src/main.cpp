#include <main.hpp>

uint slice_num_1;
uint channel_1;
uint slice_num_2;
uint channel_2;


int left_tick_cnt = 0;
int right_tick_cnt = 0;

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
    //pwm_set_chan_level(slice_num_1, channel_1, 0);
    pwm_set_enabled(slice_num_1, true);

    gpio_init(OUT_DIR2);
    gpio_set_dir(OUT_DIR2, GPIO_OUT);

    gpio_set_function(OUT_PWM2, GPIO_FUNC_PWM);
    slice_num_2 = pwm_gpio_to_slice_num(OUT_PWM2);
    channel_2 = pwm_gpio_to_channel(OUT_PWM2);
    pwm_set_clkdiv(slice_num_2, PWM_CLKDIV);
    pwm_set_wrap(slice_num_2, PWM_TOP);
    //pwm_set_chan_level(slice_num_2, channel_2, 0);
    pwm_set_enabled(slice_num_2, true);

    gpio_set_function(PICO_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PICO_I2C_SCL, GPIO_FUNC_I2C);
    i2c_init(PICO_I2C, PICO_I2C_BAUDRATE);
    i2c_set_slave_mode(PICO_I2C, true, PICO_I2C_ADDR);
    
    while(true)
    {
        ;
    }
    return 0;
}