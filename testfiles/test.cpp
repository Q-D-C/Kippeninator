#include <iostream>
#include <fstream>
#include <string>
#include <unistd.h>

class GPIOButton
{
private:
    std::string buttonPin;

public:
    GPIOButton(const std::string &pin) : buttonPin(pin)
    {
        // Export the GPIO pin
        std::ofstream exportFile("/sys/class/gpio/export");
        exportFile << buttonPin;
        exportFile.close();

        // Set the pin as input
        std::ofstream directionFile("/sys/class/gpio/gpio" + buttonPin + "/direction");
        directionFile << "in";
        directionFile.close();
    }

    ~GPIOButton()
    {
        // Unexport the GPIO pin
        std::ofstream unexportFile("/sys/class/gpio/unexport");
        unexportFile << buttonPin;
        unexportFile.close();
    }

    bool isButtonPressed()
    {
        std::ifstream valueFile("/sys/class/gpio/gpio" + buttonPin + "/value");
        std::string value;
        valueFile >> value;
        valueFile.close();

        // Check if the button is pressed (HIGH)
        return value == "1";
    }
};

class Temperature
{
private:
    const std::string buttonPinUp = "27";
    const std::string buttonPinDown = "22";

    GPIOButton buttonUp{buttonPinUp};
    GPIOButton buttonDown{buttonPinDown};

public:
    Temperature()
    {
        std::cout << "Press the button to check temperature!" << std::endl;
    }
    
    void checkTemperature()
    {
        // Logic to read temperature using the buttons
        // Add your temperature reading logic here
        if (buttonUp.isButtonPressed())
        {
            std::cout << "Temperature Increased!" << std::endl;
            usleep(200000); // Add a small delay (200ms) for debouncing
        }
        else if (buttonDown.isButtonPressed())
        {
            std::cout << "Temperature Decreased!" << std::endl;
            usleep(200000); // Add a small delay (200ms) for debouncing
        }
    }
};

int main()
{

    // Create a Temperature object
    Temperature temperature;

    while (true)
    {
        temperature.checkTemperature();
        usleep(10000); // Delay to reduce CPU usage
    }

    return 0;
}
