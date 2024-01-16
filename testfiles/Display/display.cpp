#include <pigpiod_if2.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <bitset>

// Define GPIO to LCD mapping
#define LCD_RS 2
#define LCD_E  3
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

class Display {
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

Display::Display() {
    // Connect to pigpiod daemon
    pi = pigpio_start(NULL, NULL);

    if (pi < 0) {
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

Display::~Display() {
    // Clean up and close connection to pigpiod
    pigpio_stop(pi);
}

void Display::init() {
    // Initialize display
    byte(0x38, LCD_CMD); // 8-bit mode, 2-line, 5x8 font
    byte(0x0C, LCD_CMD); // Display on, cursor off, blink off
    byte(0x06, LCD_CMD); // Entry mode set, increment cursor
    byte(0x01, LCD_CMD); // Clear display
    usleep(E_DELAY * 10); // Short delay for the clear display command to complete
}

void Display::print(const int row, const char *line) {
    // Send some text to the display
    byte(row, LCD_CMD);
    string(line);

    usleep(E_DELAY * 2); // Short delay before next command
}

void Display::byte(int bits, int mode) {
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

void Display::string(const char *message) {
    // Send string to display
    
    std::string paddedMessage(message);
    paddedMessage.resize(LCD_WIDTH, ' ');

    for (size_t i = 0; i < LCD_WIDTH; ++i) {
        byte(static_cast<int>(paddedMessage[i]), LCD_CHR);}

    //for (size_t i = 0; i < strlen(message); ++i) {
    //    byte(static_cast<int>(message[i]), LCD_CHR);
    //}
}

int main() {
    Display display;
    int temp = 89;
    std::string tekst = "Temperatuur: ";
    std::string output = tekst + std::to_string(temp);
    
    
    display.print(LCD_LINE_1, output.c_str());
    
    display.print(LCD_LINE_2, "Temp: wha!");
	
    return 0;
}