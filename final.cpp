/// g++ -g main.cpp -o main -lpigpiod_if2 -lrt -lmysqlcppconn -lpigpio

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <fstream>
#include <string>
#include <sys/ioctl.h>
#include <ctime>
#include <linux/i2c-dev.h>
#include <mysql_driver.h>
#include <mysql_connection.h>
#include <cppconn/statement.h>
#include <pigpiod_if2.h>

bool printStuffVraagteken = true;
bool printTurning = true;

// define the amount of time between readings
#define sleeptime 5
#define PWMHz 1
#define heaterPin 26

// values for the PID controller
#define KP 24
#define KI 0.20
#define KD 900

// values for the map function
int mapMIN = 0;
int mapMAX = 100;

// define Motor pins
#define MOTOR_PIN1 22
#define MOTOR_PIN2 27
#define MOTOR_PIN3 17
#define MOTOR_PIN4 4

// motor values
#define MOTOR_DELAY 10 // ms
#define MOTOR_STEPS 150

// define every how many hours the eggs will turn
#define TURNHOWMANYTIMES 1 // every x hour(s)/minute(s)

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

// MySQL database connection settings
const std::string host = "127.0.0.1";
const std::string user = "kippenbroeder";
const std::string password = "kippenbroederinator";
const std::string database = "sensor_data";

// I2C interface on Raspberry Pi
const char *filename = "/dev/i2c-1";

class SI7021Sensor
{
private:
    int file;
    const int I2C_ADDRESS = 0x40; // SI7021 sensor I2C address
    int previousTemperature = 20;

public:
    double temperatureCelsius;
    double humidityPercentage;

    SI7021Sensor(const char *filename)
    {
        // try to open the correct I2C location
        if ((file = open(filename, O_RDWR)) < 0)
        {
            std::cerr << "Failed to open the i2c bus" << std::endl;
            // exit(1);
        }

        if (ioctl(file, I2C_SLAVE, I2C_ADDRESS) < 0)
        {
            std::cerr << "Failed to acquire bus access" << std::endl;
            // exit(1);
        }
    }

    // destructor to make sure everything gets closed propperly
    ~SI7021Sensor()
    {
        close(file);
    }

    void readHumidity()
    {
        // make sure the data is read propperly
        unsigned char humidityCommand[1] = {0xE5};
        if (write(file, humidityCommand, 1) != 1)
        {
            std::cerr << "Error writing humidity command" << std::endl;
            // exit(1);
        }

        usleep(100000); // Wait for measurement to be completed

        unsigned char humidityData[2];
        if (read(file, humidityData, 2) != 2)
        {
            std::cerr << "Error reading humidity data" << std::endl;
            // exit(1);
        }
        // do some calculation magic from the datasheet
        int humidity = ((static_cast<int>(humidityData[0]) << 8) | static_cast<int>(humidityData[1]));
        humidityPercentage = (125.0 * humidity / 65536.0) - 6.0;

        if (printStuffVraagteken)
        {
            std::cout << "Humidity:    " << humidityPercentage << "%" << std::endl;
        }
    }

    void readTemperature()
    {
        bool firstVraagteken = false;

        unsigned char tempCommand[1] = {0xE3};
        if (write(file, tempCommand, 1) != 1)
        {
            std::cerr << "Error writing temperature command" << std::endl;
            // exit(1);
        }

        usleep(100000); // Wait for measurement to be completed (100ms for hold master mode)

        unsigned char tempData[2];
        if (read(file, tempData, 2) != 2)
        {
            std::cerr << "Error reading temperature data" << std::endl;
            // exit(1);
        }

        // read the temperature from the sensor

        int temperature = ((static_cast<int>(tempData[0]) << 8) | static_cast<int>(tempData[1]));

        if (firstVraagteken)
        { // make sure the temperature didn't spike in form of a misread for example
            if (temperature >= (previousTemperature + 5) || temperature <= (previousTemperature - 5) || temperature <= 0)
            {
                temperatureCelsius = previousTemperature;
            }
            else
            {
                // do some calculation magic from the datasheet
                temperatureCelsius = (175.72 * temperature / 65536.0) - 46.85;
            }
        }
        else
        {
            // do some calculation magic from the datasheet
            temperatureCelsius = (175.72 * temperature / 65536.0) - 46.85;
            firstVraagteken = true;
        }
        previousTemperature = temperatureCelsius;

        if (printStuffVraagteken)
        {
            std::cout << "Temperature: " << temperatureCelsius << "°C" << std::endl;
        }
    }
};

class SQLDatabase
{
private:
    // spooky sql constructor stuffs
    sql::mysql::MySQL_Driver *driver;
    sql::Connection *con;

public:
    SQLDatabase(const std::string &host, const std::string &user, const std::string &password, const std::string &database)
    {
        try
        { // connect to the database
            driver = sql::mysql::get_mysql_driver_instance();
            con = driver->connect("tcp://" + host + ":3306/" + database, user, password);
            con->setSchema(database);
        }
        catch (sql::SQLException &e)
        {
            std::cerr << "MySQL Connection Error: " << e.what() << std::endl;
        }
    }

    ~SQLDatabase()
    { // always clean up after yourself :)
        delete con;
    }

    // insert all the data into the database
    void insertSensorData(double humidity, double temperature, double p, double i, double d, double pid, double power)
    {
        try
        {
            // connect to the database
            sql::Statement *stmt = con->createStatement();
            // insert into the database (please ignore the length, yes it is supposed to be this long, yes i know it looks bad but remember the ugly duckling? good, this is actually the swan between the ducks that is underappreciated. well you better start appreciating it now damn it)
            stmt->execute("INSERT INTO data (humidity, temperature, p, i, d, power, pid) VALUES (" + std::to_string(humidity) + ", " + std::to_string(temperature) + ", " + std::to_string(p) + ", " + std::to_string(i) + ", " + std::to_string(d) + ", " + std::to_string(power) + ", " + std::to_string(pid) + ")");
            delete stmt;
        }
        catch (sql::SQLException &e)
        { // if this gets triggered something went wrong and you are allowed to cry
            std::cerr << "MySQL Error: " << e.what() << std::endl;
        }
    }
};

class Heating
{
private:
    int pi;

    float getTemperature() // get the set temperature that has been written into a file
    {
        float Value;
        std::ifstream inFile("Temperature.txt");
        if (inFile.is_open())
        {
            inFile >> Value;
            inFile.close();
            // std::cout << "Temperature value read from the file: " << Value << std::endl;
        }
        return Value;
    }

    double mapValue(double inputValue, double inputMin, double inputMax, double outputMin, double outputMax)
    {
        // make sure the value isn't outside of the preffered range
        inputValue = std::max(inputMin, std::min(inputValue, inputMax));
        // do some math magic to map the recieved value to the value you acually want
        double mappedValue = (((inputValue - inputMin) / (inputMax - inputMin)) * (outputMax - outputMin) + outputMin);

        return mappedValue;
    }

public:
    double power = 0;

    Heating(int pigpio_instance)
        : pi(pigpio_instance)
    {
        // Set GPIO pin as output
        set_mode(pi, heaterPin, PI_OUTPUT);
    }

    ~Heating()
    { // make sure the heater doesnt stay on if the program somehow stops, i pray this never happens
        gpio_write(pi, heaterPin, 0);
    }

    void controlTemperatureOnOff(float current)
    {
        float currentTemperature = current;
        float wantedTemperature;

        // check if the wanted temperature changed
        wantedTemperature = getTemperature();

        // a basic on off controller
        if (currentTemperature <= wantedTemperature)
        {
            gpio_write(pi, heaterPin, 1); // Set high
            std::cout << "Heater is now ON" << std::endl;
        }
        else
        {
            gpio_write(pi, heaterPin, 0); // Set low
            std::cout << "Heater is now OFF" << std::endl;
        }

        // too basic if you ask me
    }

    void controlTemperaturePWM(double input)
    { // control the heating based on a PWM signal

        // convert the input to an actual usable value
        double pwmm = mapValue(input, mapMIN, mapMAX, 0, 255);

        // also convert it to a % for people who dont understand PWM scale, you know who you are
        power = mapValue(input, mapMIN, mapMAX, 0, 100);
        if (printStuffVraagteken)
        {
            std::cout << "pwm:         " << pwmm << std::endl;
            std::cout << "power:       " << power << "%" << std::endl;
            std::cout << std::endl;
        }
        // make sure the PWM works in the wanted Hz and actually do something with it
        set_PWM_frequency(pi, heaterPin, PWMHz);
        gpioPWM(heaterPin, pwmm); // turn PWm on
    }
};

class PID
{
    // https://www.javatpoint.com/pid-controller-cpp
private:
    double setpoint = 0;        // desired output
    double processVariable = 0; // current output
    double error = 0;           // difference between setpoint and processVariable
    double previousError = 0;   // error in previous iteration
    double integral = 0;        // integral of error
    double derivative = 0;      // derivative of error
    double kp = KP;             // proportional gain
    double ki = KI;             // integral gain
    double kd = KD;             // derivative gain

    float getSetpoint() // does what it says, gets the set point from the file
    {
        float Value;
        std::ifstream inFile("Temperature.txt");
        if (inFile.is_open())
        {
            inFile >> Value;
            inFile.close();
        }
        return Value;
    }

public:
    double pVal = 0;
    double iVal = 0;
    double dVal = 0;
    double output = 0; // output of the controller

    // do some math magic, i am not even going to try and explain it.
    double calculateP(double processVariable)
    {
        setpoint = getSetpoint();
        error = setpoint - processVariable;
        output = (kp * error);
        return output;
    }

    double calculatePI(double processVariable)
    {
        setpoint = getSetpoint();
        error = setpoint - processVariable;
        integral += error;
        output = (kp * error) + (ki * integral);
        previousError = error;
        return output;
    }

    double calculatePID(double processVariable)
    {
        setpoint = getSetpoint();
        error = setpoint - processVariable;
        integral += error;
        derivative = error - previousError;
        pVal = (kp * error);
        iVal = (ki * integral);
        dVal = (kd * derivative);
        output = pVal + iVal + dVal;
        previousError = error;

        if (printStuffVraagteken)
        {
            std::cout << "error:       " << error << std::endl;
            std::cout << "P:           " << pVal << std::endl;
            std::cout << "I:           " << iVal << std::endl;
            std::cout << "D:           " << dVal << std::endl;
            std::cout << "PID:         " << output << std::endl;
        }
        return output;
    }
};

class Motor
{
public:
    Motor(int pigpio_instance);
    ~Motor();

    void setStep(int out1, int out2, int out3, int out4);
    void rotateClockwise();        // void forward(double delay, int steps);
    void rotateCounterClockwise(); // void backwards(double delay, int steps);
    void startMotorEveryXMinutes();
    void startMotorEveryXHours();

private:
    int pi;
    static const int Seq[][4];
    double turnedAt;
    double currentTime;
};

const int Motor::Seq[][4] = {
    {0, 1, 0, 0},
    {0, 1, 0, 1},
    {0, 0, 0, 1},
    {1, 0, 0, 1},
    {1, 0, 0, 0},
    {1, 0, 1, 0},
    {0, 0, 1, 0},
    {0, 1, 1, 0}};

Motor::Motor(int pigpio_instance)
    : pi(pigpio_instance)
{
    set_mode(pi, MOTOR_PIN1, PI_OUTPUT);
    set_mode(pi, MOTOR_PIN2, PI_OUTPUT);
    set_mode(pi, MOTOR_PIN3, PI_OUTPUT);
    set_mode(pi, MOTOR_PIN4, PI_OUTPUT);
}

Motor::~Motor()
{
    //
}

void Motor::setStep(int out1, int out2, int out3, int out4)
{
    gpio_write(pi, MOTOR_PIN2, out1);
    gpio_write(pi, MOTOR_PIN4, out2);
    gpio_write(pi, MOTOR_PIN3, out3);
    gpio_write(pi, MOTOR_PIN1, out4);
}

void Motor::rotateClockwise()
{ // forward
    if (printStuffVraagteken)
    {
        std::cerr << "ROTATING" << std::endl;
    }
    for (int i = 0; i < MOTOR_STEPS; ++i)
    {
        for (int j = 0; j < 8; ++j)
        {
            setStep(Seq[j][0], Seq[j][1], Seq[j][2], Seq[j][3]);
            time_sleep(MOTOR_DELAY / 1000.0);
        }
    }
}

void Motor::rotateCounterClockwise()
{ // backward
    for (int i = 0; i < MOTOR_STEPS; ++i)
    {
        for (int j = 7; j >= 0; --j)
        {
            setStep(Seq[j][0], Seq[j][1], Seq[j][2], Seq[j][3]);
            time_sleep(MOTOR_DELAY / 1000.0);
        }
    }
}

void Motor::startMotorEveryXMinutes()
{
    time_t now = time(0);
    tm *ltm = localtime(&now);

    currentTime = ltm->tm_min;
    turnedAt;

    if (ltm->tm_min % 2 == TURNHOWMANYTIMES && currentTime != turnedAt)
    {
        if (printTurning)
        {
            std::cout << "eggos gedraaid om: " << asctime(ltm);
	}
	turnedAt = currentTime;
        rotateClockwise();
    }
}

void Motor::startMotorEveryXHours()
{
    time_t now = time(0);
    tm *ltm = localtime(&now);

    currentTime = ltm->tm_hour;
    turnedAt;

    if (ltm->tm_hour % TURNHOWMANYTIMES == 0 && currentTime != turnedAt)
    {
        if (printTurning)
        {
            std::cout << "eggos gedraaid om: " << asctime(ltm);
            {
                turnedAt = currentTime;
                rotateClockwise();
            }
        }
    }
}

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

    SI7021Sensor sensor(filename);

    SQLDatabase databaseConnection(host, user, password, database);

    Heating heater(pi);

    PID magic;

    Motor motor(pi);
    
    Display display(pi);
    
    Menu menu(pi, display);
    
    while (true)
    {
        // read all the needed data from the sensors
        sensor.readHumidity();
        sensor.readTemperature();

        // heater.controlTemperatureOnOff(sensor.temperatureCelsius);
        heater.controlTemperaturePWM(magic.calculatePID(sensor.temperatureCelsius));

        // Insert data into MySQL database
//        databaseConnection.insertSensorData(sensor.humidityPercentage, sensor.temperatureCelsius, magic.pVal, magic.iVal, magic.dVal, magic.output, heater.power);

        // check if the "eggos" need to be turned
        //motor.startMotorEveryXMinutes();
motor.rotateClockwise();
        sleep(sleeptime); // Wait before reading again
    }

    pigpio_stop(pi); // Disconnect from the pigpiod daemon
    return 0;
}
