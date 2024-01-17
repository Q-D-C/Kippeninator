// g++ "menu ISR display.cpp" -o menuISRdisplay -lpigpiod_if2 -lpigpio -lrt

#include <iostream>
#include <fstream>
#include <cstring>
#include <pigpiod_if2.h>
#include <unistd.h>
#include <bitset>

#define MENUPIN 0
#define UPPIN 9
#define DOWNPIN 6

// Define GPIO to LCD mapping
#define LCD_RS 21
#define LCD_E  20
#define LCD_D0 16
#define LCD_D1 12
#define LCD_D2 1
#define LCD_D3 7
#define LCD_D4 8
#define LCD_D5 25
#define LCD_D6 24
#define LCD_D7 23

#define LCD_LINE_1 0x80
#define LCD_LINE_2 0xC0

class Display
{
public:
    Display(int pigpio_instance);
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

Display::Display(int pigpio_instance) : pi(pigpio_instance)
{

    // Set up GPIO
    set_mode(pi, LCD_E, PI_OUTPUT);
    set_mode(pi, LCD_RS, PI_OUTPUT);
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
    //byte(0x33, LCD_CMD);
    //byte(0x32, LCD_CMD);
    byte(0x28, LCD_CMD);//4 bit, 2 line display, 5x8 dot matrix
    byte(0x0C, LCD_CMD);//display on, cursor off
    byte(0x06, LCD_CMD);//entry mode set
    byte(0x01, LCD_CMD);//clear display
}

void Display::print(const int row, const char *line)
{
    byte(0x01, LCD_CMD);
    // Send some text to the display
    byte(row, LCD_CMD);
    string(line);

    usleep(E_DELAY * 2); // Short delay before next command
}

void Display::byte(int bits, int mode) {
    // Send byte to data pins
    // bits = data
    // mode = true  for character
    //        false for command
    
    //int filter = (bits & 0x08) == 0x08;
    //std::bitset<8> x(filter);
	//std::cout << x << '\n';
    
    gpio_write(pi, LCD_RS, mode); // RS

    // High bits
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

    // Low bits
    gpio_write(pi, LCD_D4, (bits & 0x01) == 0x01);
    gpio_write(pi, LCD_D5, (bits & 0x02) == 0x02);
    gpio_write(pi, LCD_D6, (bits & 0x04) == 0x04);
    gpio_write(pi, LCD_D7, (bits & 0x08) == 0x08);

	

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
    
    int pi;

    static const int buttonMenu = MENUPIN;
    static const int buttonUp = UPPIN;
    static const int buttonDown = DOWNPIN;
    
    Display &display;

    int Menustate = 0;
    int Daycounter = 19;
    float TemperatureValue = 37.5;
    float HumidityValue = 65.0;

    std::string TempTekst = "Temperatuur:";
    std::string HumiTekst = "Water %: ";
    std::string DaysTekst = "Days left: ";
    

public:

    std::string toPrint;
    
    Menu(int pigpio_instance, Display &displayObject) : pi(pigpio_instance), display(displayObject) 
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
        usleep(50);
    }

    static void checkUpStatic(int gpio, int level, uint32_t tick)
    {
        menuInstance->ValueUp(gpio, level, tick);
        usleep(50);
    }

    static void checkDownStatic(int gpio, int level, uint32_t tick)
    {
        menuInstance->ValueDown(gpio, level, tick);
        usleep(50);
    }

    void checkMenu(int gpio, int level, uint32_t tick)
    {
        std::cout << "MENU" << std::endl;
        Menustate++;
        if (Menustate > 2)
        {
            Menustate = 0;
        }
        switch (Menustate)
        {
        case 0:
            toPrint = std::to_string(TemperatureValue);
             display.print(LCD_LINE_1, "Temperatuur: ");
             display.print(LCD_LINE_2, toPrint.c_str());

            std::cout << TempTekst << TemperatureValue << std::endl;
            break;
        case 1:
            toPrint = std::to_string(HumidityValue);
             display.print(LCD_LINE_1, "Humidity: ");
             display.print(LCD_LINE_2, toPrint.c_str());

            std::cout << HumiTekst << HumidityValue << std::endl;
            break;
        case 2:
            toPrint = std::to_string(Daycounter);
             display.print(LCD_LINE_1, "Days Left: ");
             display.print(LCD_LINE_2, toPrint.c_str());

            std::cout << DaysTekst << Daycounter << std::endl;
            break;
        }
        time_sleep(0.2);
    }

    void ValueUp(int gpio, int level, uint32_t tick)
    {
        std::cout << "UP" << std::endl;
        switch (Menustate)
        {
        case 0:
            if (true)
            {
                TemperatureValue = TemperatureValue + 0.1;
                            toPrint = std::to_string(TemperatureValue);
             display.print(LCD_LINE_1, "Temperatuur: ");
             display.print(LCD_LINE_2, toPrint.c_str());
                std::cout << "Temperature = " << TemperatureValue << std::endl;
                time_sleep(0.2); // Add a small delay (200ms) for debouncing
                // Store the value in File
                std::ofstream outFile("Temperature.txt");
                if (outFile.is_open())
                {
                    outFile << TemperatureValue;
                    outFile.close();
                    //std::cout << "Value has been stored in the file." << std::endl;
                }
            }
            break;
        case 1:
            if (true)
            {
                HumidityValue = HumidityValue + 0.1;
                            toPrint = std::to_string(HumidityValue);
             display.print(LCD_LINE_1, "Humidity: ");
             display.print(LCD_LINE_2, toPrint.c_str());
                std::cout << "Humidity = " << HumidityValue << std::endl;
                time_sleep(0.2); // Add a small delay (200ms) for debouncing
                                 // Store the value in File
                std::ofstream outFile1("Humidity.txt");
                if (outFile1.is_open())
                {
                    outFile1 << HumidityValue;
                    outFile1.close();
                    //std::cout << "Value has been stored in the file." << std::endl;
                }
            }
            break;
        case 2:
            if (true)
            {
                Daycounter++;
                            toPrint = std::to_string(Daycounter);
             display.print(LCD_LINE_1, "Days Left: ");
             display.print(LCD_LINE_2, toPrint.c_str());
                std::cout << "Days left: " << Daycounter << std::endl;
                time_sleep(0.2); // Add a small delay (200ms) for debouncing
                                 // Store the value in File
                std::ofstream outFile2("Days.txt");
                if (outFile2.is_open())
                {
                    outFile2 << Daycounter;
                    outFile2.close();
                    //std::cout << "Value has been stored in the file." << std::endl;
                }
            }
            break;
        }
        time_sleep(0.2); // Add a small delay (200ms) for debouncing
    }

    void ValueDown(int gpio, int level, uint32_t tick)
    {
        std::cout << "DOWN" << std::endl;
        switch (Menustate)
        {
        case 0:
            if (true)
            {
                TemperatureValue = TemperatureValue - 0.1;
                            toPrint = std::to_string(TemperatureValue);
             display.print(LCD_LINE_1, "Temperatuur: ");
             display.print(LCD_LINE_2, toPrint.c_str());
                std::cout << "Temperature = " << TemperatureValue << std::endl;
                time_sleep(0.2); // Add a small delay (200ms) for debouncing
                // Store the value in File
                std::ofstream outFile("Temperature.txt");
                if (outFile.is_open())
                {
                    outFile << TemperatureValue;
                    outFile.close();
                    //std::cout << "Value has been stored in the file." << std::endl;
                }
            }
            break;
        case 1:
            if (true)
            {
                HumidityValue = HumidityValue - 0.1;
                            toPrint = std::to_string(HumidityValue);
             display.print(LCD_LINE_1, "Humidity: ");
             display.print(LCD_LINE_2, toPrint.c_str());
                std::cout << "Humidity = " << HumidityValue << std::endl;
                time_sleep(0.2); // Add a small delay (200ms) for debouncing
                                 // Store the value in File
                std::ofstream outFile1("Humidity.txt");
                if (outFile1.is_open())
                {
                    outFile1 << HumidityValue;
                    outFile1.close();
                    //std::cout << "Value has been stored in the file." << std::endl;
                }
            }
            break;
        case 2:
            if (true)
            {
                Daycounter--;
                            toPrint = std::to_string(Daycounter);
             display.print(LCD_LINE_1, "Days Left: ");
             display.print(LCD_LINE_2, toPrint.c_str());
                std::cout << "Days left: " << Daycounter << std::endl;
                time_sleep(0.2); // Add a small delay (200ms) for debouncing
                                 // Store the value in File
                std::ofstream outFile2("Days.txt");
                if (outFile2.is_open())
                {
                    outFile2 << Daycounter;
                    outFile2.close();
                    //std::cout << "Value has been stored in the file." << std::endl;
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


    
    Display display(pi);
    
    Menu menu(pi, display);
    

    while (true)
    {
        //std::cout << "A" << std::endl;
        //display.print(LCD_LINE_1, menu.toPrint.c_str());
        time_sleep(0.01); // Delay to reduce CPU usage
    }

    pigpio_stop(pi); // Disconnect from the pigpiod daemon

    return 0;
}
