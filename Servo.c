#include <stdio.h>
#include <pico/stdlib.h>
#include <hardware/gpio.h>
#include <hardware/pwm.h>

// SERVO MOTOR
#define thMin 600   // tH value corresponding to 0 degrees in microseconds
#define thMax 2400  // tH value  corresponding to maximum angle of rotation in microseconds
#define angleMax 180    // maximum angle of rotation in degrees

// PINS
#define servoOnePin 14  // gpio pin for servo number one
#define servoTwoPin 15  // gpio pin for servo number two

// PWM
#define DIVi 50     // value for DIVi
#define DIVf 0      // value for DIVf
#define slice 7     // slice for the gpio pins
#define channelOne 0    // channel for servo number one
#define channelTwo 1    // channel for servo number two 
#define period 15   // time period in milliseconds

// VARIABLES
int DIV = 0;    // variable for DIV
int TOP = 0;    // variable for TOP
int ccMin = 0;  // variable for CC value at 0 degrees
int ccMax = 0;  // variable for CC value at maximum angle of rotation
int cc = 0;     // variable for CC value per 1 degree of rotation

void setup()
{
    // SET UP GPIO FOR SERVO NUMBER ONE
    gpio_init(servoOnePin);    // initialize the pin connected to servo number one
    gpio_set_dir(servoOnePin, true);   // set the direction of the pin
    gpio_set_functon(servoOnePin, GPIO_FUNC_PWM);  // enable PWM on the pin

    // SET UP GPIO FOR SERVO NUMBER TWO
    gpio_init(servoTwoPin);    // initialize the pin connected to servo number two
    gpio_set_dir(servoTwoPin, true);   // set the direction of the pin
    gpio_set_functon(servoTwoPin, GPIO_FUNC_PWM);  // enable PWM on the pin

    // CONFIGURE PWM
    pwm_set_clkdiv_int_frac(slice, DIVi, DIVf); // specify the DIVi and DIVf values
    pwm_set_wrap(slice, TOP);   // specify the TOP value
    pwm_set_chan_level(slice, channelOne, 0);   // set the CC value for servo number one to 0
    pwm_set_chan_level(slice, channelTwo, 0);   // set the CC value for servo number two to 0
    pwm_set_enabled(slice, true); // enable the pwm signal
}

void rotateServoOne()
{
    pwm_set_chan_level(slice, channelOne, ((cc * 15) + ccMin));   // rotate the servo to 15 degrees
    sleep_ms(5000); // wait for 5 seconds
    pwm_set_chan_level(slice, channelOne, ((cc * 130) + ccMin));  // rotate the servo to 130 degrees
    sleep_ms(5000); // wait for 5 seconds
}

void rotateServoTwo()
{
    pwm_set_chan_level(slice, channelTwo, ((cc * 15) + ccMin));   // rotate the servo to 15 degrees
    sleep_ms(5000); // wait for 5 seconds
    pwm_set_chan_level(slice, channelTwo, ((cc * 130) + ccMin));  // rotate the servo to 130 degrees
    sleep_ms(5000); // wait for 5 seconds
}

int main()
{
    setup();

    // CALCULATE VALUES
    DIV = (DIVi + DIVf);    // calculate value for DIV
    TOP = (int) (((125000000 / DIV) * (period * .001)) - 1);    // calculate value for TOP
    ccMin = (int) ((TOP + 1) * (thMin / (period * .001)));    // calculate CC value for 0 degrees
    ccMax = (int) ((TOP + 1) * (thMax / (period * .001)));    // calculate CC value for maximum angle of rotation
    cc = ((ccMax - ccMin) / angleMax);    // calculate CC value per 1 degree of rotation

    while (true)
    {
        rotateServoOne();
        rotateServoTwo();
    }
}