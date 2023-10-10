#include <iostream>
#include <fstream>
#include <string>
#include <unistd.h>

class GPIOButton {
private:
    std::string buttonPin;

public:
    GPIOButton(const std::string& pin) : buttonPin(pin) {
        // Export the GPIO pin
        std::ofstream exportFile("/sys/class/gpio/export");
        exportFile << buttonPin;
        exportFile.close();

        // Set the pin as input
        std::ofstream directionFile("/sys/class/gpio/gpio" + buttonPin + "/direction");
        directionFile << "in";
        directionFile.close();
    }

    ~GPIOButton() {
        // Unexport the GPIO pin
        std::ofstream unexportFile("/sys/class/gpio/unexport");
        unexportFile << buttonPin;
        unexportFile.close();
    }

    void waitForButtonPress() {
        std::cout << "Press the button!" << std::endl;

        while (true) {
            std::ifstream valueFile("/sys/class/gpio/gpio" + buttonPin + "/value");
            std::string value;
            valueFile >> value;
            valueFile.close();

            // Check if the button is pressed (HIGH)
            if (value == "1") {
                std::cout << "Button Pressed!" << std::endl;
                usleep(200000); // Add a small delay (200ms) for debouncing
            }

            usleep(10000); // Delay to reduce CPU usage
        }
    }
};

int main() {
    // GPIO pin number for the button (GPIO17)
    const std::string buttonPin = "22";

    // Create a GPIOButton object
    GPIOButton button(buttonPin);

    // Wait for button press
    button.waitForButtonPress();

    return 0;
}
