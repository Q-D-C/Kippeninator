
// g++ button.cpp -o button -lpigpiod_if2

#include <iostream>
#include <pigpiod_if2.h>

int main() {
    int pi = pigpio_start(NULL, NULL); // Connect to the pigpiod daemon

    if (pi < 0) {
        std::cerr << "Failed to connect to pigpiod daemon" << std::endl;
        return 1;
    }

    // Set GPIO pin 18 as PWM output
    set_mode(pi, 18, PI_OUTPUT);
    // Set GPIO pin 17 as an output
    set_mode(pi, 17, PI_OUTPUT);
    // Set GPIO pin 19 as an input
    set_mode(pi, 19, PI_INPUT);

    // Read the button state
    int buttonState = gpio_read(pi, 19);

    if (buttonState ==0){
        time_sleep(0.5);
        std::cout << "Button pressed!" << std::endl;
    } else {
        //stuff
    }

    // Start hardware PWM on GPIO pin 18 with a frequency of 1000 Hz and duty cycle of 500000 (50%)
    hardware_PWM(pi, 18, 1000, 500000);

    // Toggle the GPIO pin 17 state
    gpio_write(pi, 17, 1); // Set high

    // Sleep for 5 seconds
    time_sleep(5);

    // Stop PWM on GPIO pin 18
    hardware_PWM(pi, 18, 0, 0);

    // Toggle the GPIO pin 17 state
    gpio_write(pi, 17, 0); // Set low

    // Clean up and disconnect from the pigpiod daemon
    pigpio_stop(pi);

    return 0;
}
