#include <stdio.h>
#include <pico/stdlib.h>
#include <hardware/gpio.h>
#include <hardware/pwm.h>

#define servoOnePin 10

uint16_t cc = 0;
uint8_t slice;
uint8_t channel;

void setup()
{
    gpio_init(servoOnePin);
    gpio_set_dir(servoOnePin, true);
    gpio_set_function(servoOnePin, GPIO_FUNC_PWM);

    slice = pwm_gpio_to_slice_num(servoOnePin);
    channel = pwm_gpio_to_channel(servoOnePin);

    pwm_set_clkdiv_int_frac(slice, 50, 0);
    pwm_set_wrap(slice, 37499);
    pwm_set_chan_level(slice, channel, cc);
    pwm_set_enabled(slice, true);
}

void loop()
{
    uint16_t ccOne = 3750;
    uint16_t ccTwo = 4875;

    pwm_set_chan_level(slice, channel, ccOne);
    sleep_ms(50);
    pwm_set_chan_level(slice, channel, ccTwo);
    sleep_ms(50);
}

int main()
{
    setup();
    while (true)
    {
        loop();
    }
}