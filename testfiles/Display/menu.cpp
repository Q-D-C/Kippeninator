// g++ menu.cpp -o menu -lpigpiod_if2

#include <iostream>
#include <fstream>
#include <string>
#include <pigpiod_if2.h>

int pi = pigpio_start(NULL, NULL); // Connect to the pigpiod daemon

class GPIOButton
{
private:
    int buttonPin;a

public:
    GPIOButton(int pin) : buttonPin(pin)
    {
        // Set the pin as input
        set_mode(pi, buttonPin, PI_INPUT);
    }

    bool isButtonPressed()
    {
        // Read the state of the button
        return gpio_read(pi, buttonPin) == 1;
    }
};

class Menu
{
private:
    const int buttonPinMenu = 17;
    const int buttonPinUp = 27;
    const int buttonPinDown = 22;

    GPIOButton buttonMenu{buttonPinMenu};
    GPIOButton buttonUp{buttonPinUp};
    GPIOButton buttonDown{buttonPinDown};

public:
    int Menustate = 0;
    int Daycounter = 19;
    float TemperatureValue = 37.5;
    float HumidityValue = 65;

    Menu()
    {
        std::ifstream inFile("Temperature.txt");
        if (inFile.is_open())
        {
            inFile >> TemperatureValue;
            inFile.close();
            std::cout << "Temperature value read from the file: " << TemperatureValue << std::endl;
        }
        std::ifstream inFile1("Humidity.txt");
        if (inFile1.is_open())
        {
            inFile1 >> HumidityValue;
            inFile1.close();
            std::cout << "Humidity value read from the file: " << HumidityValue << std::endl;
        }
        std::ifstream inFile2("Days.txt");
        if (inFile2.is_open())
        {
            inFile2 >> Daycounter;
            inFile2.close();
            std::cout << "Day value read from the file: " << Daycounter << std::endl;
        }
        std::cout << "Press the second button to increase value!" << std::endl;
        std::cout << "Press the third button to decrease value!" << std::endl;
        std::cout << "Press the first button to cycle the menu" << std::endl;
    }

    void checkMenu()
    {
        // Logic to read the menu using the button
        // Add your menu navigation logic here
        if (buttonMenu.isButtonPressed())
        {
            Menustate++;
            if (Menustate > 2)
            {
                Menustate = 0;
            }
            // Check button press before incrementing Menustate
            switch (Menustate)
            {
            case 0:
                std::cout << "Temperature = " << TemperatureValue << std::endl;
                break;
            case 1:
                std::cout << "Humidity = " << HumidityValue << std::endl;
                break;
            case 2:
                std::cout << "Days left: " << Daycounter << std::endl;
                break;
            }

            time_sleep(0.2); // Add a small delay (200ms) for debouncing
        }
    }

    void checkValue()
    {
        switch (Menustate)
        {
        case 0:
            if (buttonUp.isButtonPressed())
            {
                TemperatureValue = TemperatureValue + 0.1;
                std::cout << "Temperature = " << TemperatureValue << std::endl;
                time_sleep(0.2); // Add a small delay (200ms) for debouncing
                // Store the value in File
                std::ofstream outFile("Temperature.txt");
                if (outFile.is_open())
                {
                    outFile << TemperatureValue;
                    outFile.close();
                    std::cout << "Value has been stored in the file." << std::endl;
                }
            }
            else if (buttonDown.isButtonPressed())
            {
                TemperatureValue = TemperatureValue - 0.1;
                std::cout << "Temperature = " << TemperatureValue << std::endl;
                time_sleep(0.2); // Add a small delay (200ms) for debouncing
                // Store the value in File
                std::ofstream outFile("Temperature.txt");
                if (outFile.is_open())
                {
                    outFile << TemperatureValue;
                    outFile.close();
                    std::cout << "Value has been stored in the file." << std::endl;
                }
            }
            break;
        case 1:
            if (buttonUp.isButtonPressed())
            {
                HumidityValue = HumidityValue + 0.1;
                std::cout << "Humidity = " << HumidityValue << std::endl;
                time_sleep(0.2); // Add a small delay (200ms) for debouncing
                                 // Store the value in File
                std::ofstream outFile1("Humidity.txt");
                if (outFile1.is_open())
                {
                    outFile1 << HumidityValue;
                    outFile1.close();
                    std::cout << "Value has been stored in the file." << std::endl;
                }
            }
            else if (buttonDown.isButtonPressed())
            {
                HumidityValue = HumidityValue - 0.1;
                std::cout << "Humidity = " << HumidityValue << std::endl;
                time_sleep(0.2); // Add a small delay (200ms) for debouncing
                                 // Store the value in File
                std::ofstream outFile1("Humidity.txt");
                if (outFile1.is_open())
                {
                    outFile1 << HumidityValue;
                    outFile1.close();
                    std::cout << "Value has been stored in the file." << std::endl;
                }
            }
            break;
        case 2:
            if (buttonUp.isButtonPressed())
            {
                Daycounter++;
                std::cout << "Days left: " << Daycounter << std::endl;
                time_sleep(0.2); // Add a small delay (200ms) for debouncing
                                 // Store the value in File
                std::ofstream outFile2("Days.txt");
                if (outFile2.is_open())
                {
                    outFile2 << Daycounter;
                    outFile2.close();
                    std::cout << "Value has been stored in the file." << std::endl;
                }
            }
            else if (buttonDown.isButtonPressed())
            {
                Daycounter--;
                std::cout << "Days left: " << Daycounter << std::endl;
                time_sleep(0.2); // Add a small delay (200ms) for debouncing
                                 // Store the value in File
                std::ofstream outFile2("Days.txt");
                if (outFile2.is_open())
                {
                    outFile2 << Daycounter;
                    outFile2.close();
                    std::cout << "Value has been stored in the file." << std::endl;
                }
            }
            break;
        }
    }
};

int main()
{
    Menu menu;

    while (true)
    {
        menu.checkValue();
        menu.checkMenu();

        time_sleep(0.01); // Delay to reduce CPU usage
    }

    pigpio_stop(pi); // Disconnect from the pigpiod daemon

    return 0;
}
