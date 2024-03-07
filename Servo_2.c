#include <stdio.h>
#include <pico/stdlib.h>
#include <hardware/gpio.h>
#include <hardware/pwm.h>

// PINS
#define servoOnePin 14  // gpio pin for servo number one

void setup()
{
    // SET UP GPIO FOR SERVO NUMBER ONE
    gpio_init(servoOnePin);    // initialize the pin connected to servo number one
    gpio_set_dir(servoOnePin, true);   // set the direction of the pin
    gpio_set_function(servoOnePin, GPIO_FUNC_PWM);  // enable PWM on the pin

    // CONFIGURE PWM
    pwm_set_clkdiv_int_frac(7, 50, 0); // specify the DIVi and DIVf values
    pwm_set_wrap(7, 37499);   // specify the TOP value
    pwm_set_chan_level(7, 0, 0);   // set the CC value for servo number one to 0
    pwm_set_enabled(7, true); // enable the pwm signal
}

void rotateServoOne()
{
    pwm_set_chan_level(7, 0, 1875);   // rotate the servo to 15 degrees
    sleep_ms(5000); // wait for 5 seconds
    pwm_set_chan_level(7, 0, 4750);  // rotate the servo to 130 degrees
    sleep_ms(5000); // wait for 5 seconds
}

int main()
{
    setup();

    while (true)
    {
        rotateServoOne();
    }
}