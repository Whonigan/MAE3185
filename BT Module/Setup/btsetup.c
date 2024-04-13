#include <stdio.h>
#include <pico/stdlib.h>
#include <hardware/gpio.h>
#include <hardware/uart.h>

#define TX 16
#define RX 17

// GET A MESSAGE FROM THE BT MODULE
void getMessage(){
    while (uart_is_readable_within_us(uart0,1000000))   // wait for up to 1 second for a response
    {
        char c = uart_getc(uart0);      // variable for storing the character received from the bt module
        printf("%c", c);                // print the character received from the bt module to the serial terminal
    }
}

// RESET THE BT MODULE TO FACTORY SETTINGS
void factoryReset(){
    printf("Sending AT+ORGL command!");
    uart_puts(uart0, "AT+ORGL\r\n");        // the AT+ORGL command resets the bt module to original settings
}

// CHANGE THE NAME OF THE BT MODULE
void changeName(){
    printf("Sending AT+NAME command!");
    uart_puts(uart0, "AT+NAME=Ball Balancer\r\n");        // the AT+NAME=Ball Balancer command changes the name of the bt module to Ball Balancer
}

// CHANGE THE PASSWORD OF THE BT MODULE
void changePassword(){
    printf("Sending AT+PSWD command!");
    uart_puts(uart0, "AT+PSWD=5813\r\n");        // the AT+PSWD=5813 command changes the password of the bt module to 5813
}

// SET THE UART PARAMETERS OF THE BT MODULE
void changeUART(){
    printf("Sending AT+UART command!");
    uart_puts(uart0, "AT+UART=38400,0,0\r\n");        // the AT+UART=38400,0,0 command changes the uart parameters of the bt module to a 38400 baud rate, a single stop bit, and no parity bit
}

void setup(){
    stdio_init_all();

    gpio_init(TX);
    gpio_init(RX);
    gpio_set_dir(TX, true);
    gpio_set_dir(RX, false);
    gpio_set_function(TX, GPIO_FUNC_UART);
    gpio_set_function(RX, GPIO_FUNC_UART);

    uart_init(uart0, 38400);        // set the baud rate to 38400

    sleep_ms(5000);     // wait while the bt module powers up into configuration mode

    // CONFIGURE THE BT MODULE
    for (int i=0; i<5; i++){
        printf("Sending AT command to bluetooth module!");
        uart_puts(uart0, "AT\r\n");
        getMessage();
    }

    factoryReset();     // call the factory resest function
    getMessage();       // call the get message function
    sleep_ms(2000);     // wait for the changes to go into effect  
    
    changeName();       // call the change name function
    getMessage();       // call the get message function
    sleep_ms(2000);     // wait for the changes to go into effect

    changePassword();       // call the change password function
    getMessage();           // call the get message function
    sleep_ms(2000);         // wait for the changes to go into effect

    changeUART();           // call the change uart function
    getMessage();           // call the get message function
    sleep_ms(2000);         // wait for the changes to go into effect
}

void loop(){
    getMessage();    
}

int main(){
    setup();
    loop();
}
