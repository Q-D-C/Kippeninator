// g++ "menu ISR.cpp" -o menuISR -lpigpiod_if2 -lpigpio -lrt

#include <iostream>
#include <fstream>
#include <string>
#include <pigpiod_if2.h>

#define MENUPIN 17
#define UPPIN 27
#define DOWNPIN 22

class Menu
{
private:
    static Menu *menuInstance;

    static const int buttonMenu = MENUPIN;
    static const int buttonUp = UPPIN;
    static const int buttonDown = DOWNPIN;

    int pi;

    int Menustate = 0;
    int Daycounter = 19;
    float TemperatureValue = 37.5;
    float HumidityValue = 65;

public:
    Menu(int pigpio_instance) : pi(pigpio_instance)
    {
        menuInstance = this;

        set_mode(pi, buttonMenu, PI_INPUT);
        set_mode(pi, buttonUp, PI_INPUT);
        set_mode(pi, buttonDown, PI_INPUT);

        gpioSetISRFunc(MENUPIN, FALLING_EDGE, 0, &Menu::checkMenuStatic);
        gpioSetISRFunc(UPPIN, FALLING_EDGE, 0, &Menu::checkUpStatic);
        gpioSetISRFunc(DOWNPIN, FALLING_EDGE, 0, &Menu::checkDownStatic);
    }

    static void checkMenuStatic(int gpio, int level, uint32_t tick)
    {
        menuInstance->checkMenu(gpio, level, tick);
    }

    static void checkUpStatic(int gpio, int level, uint32_t tick)
    {
        menuInstance->ValueUp(gpio, level, tick);
    }

    static void checkDownStatic(int gpio, int level, uint32_t tick)
    {
        menuInstance->ValueDown(gpio, level, tick);
    }

    void checkMenu(int gpio, int level, uint32_t tick)
    {
        Menustate++;
        if (Menustate > 2)
        {
            Menustate = 0;
        }
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
        time_sleep(0.2);
    }

    void ValueUp(int gpio, int level, uint32_t tick)
    {
        switch (Menustate)
        {
        case 0:
            if (true)
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
            break;
        case 1:
            if (true)
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
            break;
        case 2:
            if (true)
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
            break;
        }
        time_sleep(0.2); // Add a small delay (200ms) for debouncing
    }

    void ValueDown(int gpio, int level, uint32_t tick)
    {
        switch (Menustate)
        {
        case 0:
            if (true)
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
            break;
        case 1:
            if (true)
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
            break;
        case 2:
            if (true)
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
            break;
        }
        time_sleep(0.2); // Add a small delay (200ms) for debouncing
    }
};

Menu *Menu::menuInstance = nullptr;

int main()
{

    // initialize GPIO
    gpioInitialise();
    int pi = pigpio_start(NULL, NULL); // Connect to the pigpiod daemon

    if (pi < 0)
    {
        std::cerr << "Failed to initialize pigpio: " << pi << std::endl;
        return -1;
    }

    Menu menu(pi);

    while (true)
    {
        time_sleep(0.01); // Delay to reduce CPU usage
    }

    pigpio_stop(pi); // Disconnect from the pigpiod daemon

    return 0;
}
