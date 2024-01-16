// g++ "menu ISR display.cpp" -o menuISRdisplay -lpigpiod_if2 -lpigpio -lrt

#include <iostream>
#include <fstream>
#include <cstring>
#include <pigpiod_if2.h>
#include <unistd.h>
#include <bitset>

#define MENUPIN 17
#define UPPIN 27
#define DOWNPIN 22

// Define GPIO to LCD mapping
#define LCD_RS 2
#define LCD_E 3
#define LCD_D0 26
#define LCD_D1 19
#define LCD_D2 13
#define LCD_D3 6
#define LCD_D4 25
#define LCD_D5 8
#define LCD_D6 7
#define LCD_D7 1

#define LCD_LINE_1 0x80
#define LCD_LINE_2 0xC0

class Display
{
public:
    Display();
    ~Display();

    void init();
    void print(const int row, const char *line);

private:
    static const int LCD_WIDTH = 16;
    static const int LCD_CHR = 1;
    static const int LCD_CMD = 0;

    static const int E_PULSE = 1000;
    static const int E_DELAY = 1000;

    int pi;

    void byte(int bits, int mode);
    void string(const char *message);
};

Display::Display()
{
    // Connect to pigpiod daemon
    pi = pigpio_start(NULL, NULL);

    if (pi < 0)
    {
        fprintf(stderr, "Error connecting to pigpiod: %s\n", pigpio_error(pi));
        // Handle error, throw an exception, or exit the program as appropriate
    }

    // Set up GPIO
    set_mode(pi, LCD_E, PI_OUTPUT);
    set_mode(pi, LCD_RS, PI_OUTPUT);
    set_mode(pi, LCD_D0, PI_OUTPUT);
    set_mode(pi, LCD_D1, PI_OUTPUT);
    set_mode(pi, LCD_D2, PI_OUTPUT);
    set_mode(pi, LCD_D3, PI_OUTPUT);
    set_mode(pi, LCD_D4, PI_OUTPUT);
    set_mode(pi, LCD_D5, PI_OUTPUT);
    set_mode(pi, LCD_D6, PI_OUTPUT);
    set_mode(pi, LCD_D7, PI_OUTPUT);

    // Initialize display
    init();
}

Display::~Display()
{
    // Clean up and close connection to pigpiod
    pigpio_stop(pi);
}

void Display::init()
{
    // Initialize display
    byte(0x38, LCD_CMD);  // 8-bit mode, 2-line, 5x8 font
    byte(0x0C, LCD_CMD);  // Display on, cursor off, blink off
    byte(0x06, LCD_CMD);  // Entry mode set, increment cursor
    byte(0x01, LCD_CMD);  // Clear display
    usleep(E_DELAY * 10); // Short delay for the clear display command to complete
}

void Display::print(const int row, const char *line)
{
    // Send some text to the display
    byte(row, LCD_CMD);
    string(line);

    usleep(E_DELAY * 2); // Short delay before next command
}

void Display::byte(int bits, int mode)
{
    // Send byte to data pins
    // bits = data
    // mode = true for character, false for command

    gpio_write(pi, LCD_RS, mode); // RS

    // Data bits (D0-D7)
    gpio_write(pi, LCD_D0, (bits & 0x01) == 0x01);
    gpio_write(pi, LCD_D1, (bits & 0x02) == 0x02);
    gpio_write(pi, LCD_D2, (bits & 0x04) == 0x04);
    gpio_write(pi, LCD_D3, (bits & 0x08) == 0x08);

    gpio_write(pi, LCD_D4, (bits & 0x10) == 0x10);
    gpio_write(pi, LCD_D5, (bits & 0x20) == 0x20);
    gpio_write(pi, LCD_D6, (bits & 0x40) == 0x40);
    gpio_write(pi, LCD_D7, (bits & 0x80) == 0x80);

    // Toggle 'Enable' pin
    usleep(E_DELAY);
    gpio_write(pi, LCD_E, 1);
    usleep(E_PULSE);
    gpio_write(pi, LCD_E, 0);
    usleep(E_DELAY);
}

void Display::string(const char *message)
{
    // Send string to display

    std::string paddedMessage(message);
    paddedMessage.resize(LCD_WIDTH, ' ');

    for (size_t i = 0; i < LCD_WIDTH; ++i)
    {
        byte(static_cast<int>(paddedMessage[i]), LCD_CHR);
    }

    // for (size_t i = 0; i < strlen(message); ++i) {
    //     byte(static_cast<int>(message[i]), LCD_CHR);
    // }
}

class Menu
{
private:
    static Menu *menuInstance;

    Display display;

    static const int buttonMenu = MENUPIN;
    static const int buttonUp = UPPIN;
    static const int buttonDown = DOWNPIN;

    int pi;

    int Menustate = 0;
    int Daycounter = 19;
    float TemperatureValue = 37.5;
    float HumidityValue = 65;

    std::string TempTekst = "Temperatuur: ";
    std::string HumiTekst = "Humidity: ";
    std::string DaysTekst = "Days left: ";
    std::string toPrint;

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
            toPrint = TempTekst + std::to_string(TemperatureValue);
            display.print(LCD_LINE_1, toPrint.c_str());

            // display.print(LCD_LINE_1, "Temperatuur: ");
            // display.print(LCD_LINE_2, TemperatureValue);

            std::cout << TempTekst << TemperatureValue << std::endl;
            break;
        case 1:
            toPrint = HumiTekst + std::to_string(HumidityValue);
            display.print(LCD_LINE_1, toPrint.c_str());

            // display.print(LCD_LINE_1, "Humidity: ");
            // display.print(LCD_LINE_2, HumidityValue);

            std::cout << HumiTekst << HumidityValue << std::endl;
            break;
        case 2:
            toPrint = DaysTekst + std::to_string(Daycounter);
            display.print(LCD_LINE_1, toPrint.c_str());

            // display.print(LCD_LINE_1, "Days Left: ");
            // display.print(LCD_LINE_2, Daycounter);

            std::cout << DaysTekst << Daycounter << std::endl;
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
