/// g++ -g database_integration.cpp -o database_integration -lpigpiod_if2 -lrt -lmysqlcppconn -lpigpio

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <fstream>
#include <string>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <mysql_driver.h>
#include <mysql_connection.h>
#include <cppconn/statement.h>
#include <pigpiod_if2.h>

int pi = pigpio_start(NULL, NULL); // Connect to the pigpiod daemon

// define the amount of time between readings
#define sleeptime 1

#define heaterPin 12

#define KP 0.5;
#define KI 1;
#define KD 1;

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

public:
    double temperatureCelsius;
    double humidityPercentage;

    SI7021Sensor(const char *filename)
    {
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

    ~SI7021Sensor()
    {
        close(file);
    }

    void readHumidity()
    {
        unsigned char humidityCommand[1] = {0xE5};
        if (write(file, humidityCommand, 1) != 1)
        {
            std::cerr << "Error writing humidity command" << std::endl;
            // exit(1);
        }

        usleep(100000); // Wait for measurement to be completed (100ms for hold master mode)

        unsigned char humidityData[2];
        if (read(file, humidityData, 2) != 2)
        {
            std::cerr << "Error reading humidity data" << std::endl;
            // exit(1);
        }

        int humidity = ((static_cast<int>(humidityData[0]) << 8) | static_cast<int>(humidityData[1]));
        humidityPercentage = (125.0 * humidity / 65536.0) - 6.0;

        //    return humidityPercentage;
    }

    void readTemperature()
    {
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

        int temperature = ((static_cast<int>(tempData[0]) << 8) | static_cast<int>(tempData[1]));
        temperatureCelsius = (175.72 * temperature / 65536.0) - 46.85;

        // return temperatureCelsius;
    }

    void WHATISHAPPENING()
    {
        std::cout << "Relative Humidity: " << humidityPercentage << "%" << std::endl;
        std::cout << "Temperature: " << temperatureCelsius << "°C" << std::endl;
    }
};

class MySQLDatabase
{
private:
    sql::mysql::MySQL_Driver *driver;
    sql::Connection *con;

public:
    MySQLDatabase(const std::string &host, const std::string &user, const std::string &password, const std::string &database)
    {
        try
        {
            driver = sql::mysql::get_mysql_driver_instance();
            con = driver->connect("tcp://" + host + ":3306/" + database, user, password);
            con->setSchema(database);
        }
        catch (sql::SQLException &e)
        {
            std::cerr << "MySQL Connection Error: " << e.what() << std::endl;
            // exit(1);
        }
    }

    ~MySQLDatabase()
    {
        delete con;
    }

    void insertSensorData(double humidity, double temperature)
    {
        try
        {
            sql::Statement *stmt = con->createStatement();
            stmt->execute("INSERT INTO data (humidity, temperature) VALUES (" + std::to_string(humidity) + ", " + std::to_string(temperature) + ")");
            delete stmt;
        }
        catch (sql::SQLException &e)
        {
            std::cerr << "MySQL Error: " << e.what() << std::endl;
        }
    }
};

class Heating
{
private:
    float getTemperature()
    {
        float Value;
        std::ifstream inFile("Temperature.txt");
        if (inFile.is_open())
        {
            inFile >> Value;
            inFile.close();
            std::cout << "Temperature value read from the file: " << Value << std::endl;
        }
        return Value;
    }

    double mapValue(double inputValue, double inputMin, double inputMax, double outputMin, double outputMax)
    {

        double mappedValue = std::max((((inputValue - inputMin) / (inputMax - inputMin)) * (outputMax - outputMin) + outputMin), 0.0);

        return mappedValue;
    }

public:
    Heating()
    {
        // Set GPIO pin as output
        set_mode(pi, heaterPin, PI_OUTPUT);
        // initialize GPIO
        gpioInitialise();
    }

    ~Heating()
    {
        gpio_write(pi, heaterPin, 0);
    }

    void controlTemperatureOnOff(float current)
    {
        float currentTemperature = current;
        float wantedTemperature;

        wantedTemperature = getTemperature();
        // aan/uit schakelaar
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
    }

    void controlTemperaturePWM(double input)
    {
        double PWM = mapValue(input, 0, 50, 0, 100);

        std::cout << "pwm: " << PWM << std::endl;

        // gpioPWM(heaterPin, PWM); // turn PWm full on
        gpioHardwarePWM(heaterPin, 1000, (PWM * 1000));
    }
};

class PID
{
    // https://www.javatpoint.com/pid-controller-cpp

private:
    double setpoint;        // desired output
    double processVariable; // current output
    double error;           // difference between setpoint and processVariable
    double previousError;   // error in previous iteration
    double integral;        // integral of error
    double derivative;      // derivative of error
    double kp = KP;         // proportional gain
    double ki = KI;         // integral gain
    double kd = KD;         // derivative gain
    double output;          // output of the controller

    float getSetpoint()
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
    double calculateP(double processVariable)
    {
        setpoint = getSetpoint();
        error = setpoint - processVariable;
        output = (kp * error);
        previousError = error;
        return output;
        // return (ouput > 0 ? output :0);
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
        output = (kp * error) + (ki * integral) + (kd * derivative);
        previousError = error;
        return output;
    }

    void WHATISHAPPENING()
    {
        std::cout << "setpoint: " << setpoint << std::endl;
        std::cout << "process variable: " << processVariable << std::endl;
        std::cout << "error: " << error << std::endl;
        // std::cout << "previousError: " << previousError << std::endl;
        // std::cout << "integral: " << integral << std::endl;
        // std::cout << "derivative: " << derivative << std::endl;
        std::cout << "output: " << output << std::endl;
    }
};

int main()
{
    SI7021Sensor sensor(filename);

    MySQLDatabase databaseConnection(host, user, password, database);

    Heating heater;

    PID magic;

    while (true)
    {
        // double humidity = sensor.readHumidity();
        // double temperature = sensor.readTemperature();
        sensor.readHumidity();
        sensor.readTemperature();
        sensor.WHATISHAPPENING();

        // Insert data into MySQL database
        databaseConnection.insertSensorData(sensor.humidityPercentage, sensor.temperatureCelsius);

        // heater.controlTemperatureOnOff(sensor.temperatureCelsius);
        heater.controlTemperaturePWM(magic.calculateP(sensor.temperatureCelsius));
        magic.WHATISHAPPENING();

        sleep(sleeptime); // Wait before reading again
    }

    pigpio_stop(pi); // Disconnect from the pigpiod daemon
    return 0;
}
