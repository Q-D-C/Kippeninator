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

class Menu
{
private:
    const std::string buttonPinMenu = "17";
    GPIOButton buttonUp{buttonPinMenu};

public:
    int Menustate = 0;

    Menu()
    {
        std::cout << "Press the first button to cycle menu" << std::endl;
    }

    void checkMenu()
    {
        // Logic to read menu using the button
        // Add your menu navigation logic here
        if (buttonUp.isButtonPressed())
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
                std::cout << "Temperature" << std::endl;
                break;
            case 1:
                std::cout << "Humidity" << std::endl;
                break;
            case 2:
                std::cout << "Days" << std::endl;
                break;
            }

            usleep(200000); // Add a small delay (200ms) for debouncing
        }
    }
};

class ValueChanger
{
private:
    const std::string buttonPinUp = "27";
    const std::string buttonPinDown = "22";

    GPIOButton buttonUp{buttonPinUp};
    GPIOButton buttonDown{buttonPinDown};

    Menu &menu;

public:
    int Daycounter = 19;
    float TemperatureValue = 37.5;
    float HumidityValue = 65;

    ValueChanger(Menu &menuObject) : menu(menuObject)
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
        std::cout << "Press the second button to increse value!" << std::endl;
        std::cout << "Press the third button to decrese value!" << std::endl;
    }

    // ~ValueChanger()
    // {
    //     // Store the value in File
    //     std::ofstream outFile("Temperature.txt");
    //     if (outFile.is_open())
    //     {
    //         outFile << TemperatureValue;
    //         outFile.close();
    //         std::cout << "Value has been stored in the file." << std::endl;
    //     }
    //     std::ofstream outFile("Humidity.txt");
    //     if (outFile.is_open())
    //     {
    //         outFile << HumidityValue;
    //         outFile.close();
    //         std::cout << "Value has been stored in the file." << std::endl;
    //     }
    //     std::ofstream outFile("Days.txt");
    //     if (outFile.is_open())
    //     {
    //         outFile << Daycounter;
    //         outFile.close();
    //         std::cout << "Value has been stored in the file." << std::endl;
    //     }
    // }

    void checkValue()
    {
        switch (menu.Menustate)
        {
        case 0:
            if (buttonUp.isButtonPressed())
            {
                TemperatureValue = TemperatureValue + 0.1;
                std::cout << "Temperature = " << TemperatureValue << std::endl;
                usleep(200000); // Add a small delay (200ms) for debouncing
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
                usleep(200000); // Add a small delay (200ms) for debouncing
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
                usleep(200000); // Add a small delay (200ms) for debouncing
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
                usleep(200000); // Add a small delay (200ms) for debouncing
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
                usleep(200000); // Add a small delay (200ms) for debouncing
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
                usleep(200000); // Add a small delay (200ms) for debouncing
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

class Fan
{
private:
    const std::string buttonPinFan = "18";
    GPIOButton buttonUp{buttonPinFan};

public:
    bool fanState = true;

    Fan()
    {
        std::cout << "Press the button turn on fan" << std::endl;
    }

    void checkFan()
    {

        if (buttonUp.isButtonPressed())
        {
            fanState = !fanState;
            std::cout << "fan = " << fanState << std::endl;

            usleep(200000); // Add a small delay (200ms) for debouncing
        }
    }
};

int main()
{

    Menu menu;
    ValueChanger value(menu);
    Fan fan;

    while (true)
    {
        value.checkValue();
        menu.checkMenu();
        fan.checkFan();

        usleep(10000); // Delay to reduce CPU usage
    }

    return 0;
}
