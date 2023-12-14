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
#include <linux/i2c-dev.h>
#include <mysql_driver.h>
#include <mysql_connection.h>
#include <cppconn/statement.h>
#include <pigpiod_if2.h>

int pi = pigpio_start(NULL, NULL); // Connect to the pigpiod daemon

// define the amount of time between readings
#define sleeptime 5
#define PWMHz 1
#define heaterPin 12

#define KP 24;
#define KI 0.20;
#define KD 900;

int mapMIN = 0;
int mapMAX = 100;

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

        int temperature = ((static_cast<int>(tempData[0]) << 8) | static_cast<int>(tempData[1]));
	if (firstVraagteken){
	if(temperature >= (previousTemperature + 5) || temperature <= (previousTemperature - 5) || temperature <= 0){        
	temperatureCelsius = previousTemperature;
	}else{
	temperatureCelsius = (175.72 * temperature / 65536.0) - 46.85;
	} 
	}else{
	temperatureCelsius = (175.72 * temperature / 65536.0) - 46.85;
	firstVraagteken = true;
	}
	previousTemperature = temperatureCelsius;

        // return temperatureCelsius;
    }

    void WHATISHAPPENING()
    {
        std::cout << "Humidity:    " << humidityPercentage << "%" << std::endl;
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

    void insertSensorData(double humidity, double temperature, double p, double i, double d, double pid, double power)
    {
        try
        {
            sql::Statement *stmt = con->createStatement();
            stmt->execute("INSERT INTO data (humidity, temperature, p, i, d, power, pid) VALUES (" + std::to_string(humidity) + ", " + std::to_string(temperature) + ", " + std::to_string(p) + ", " + std::to_string(i) + ", " + std::to_string(d) + ", " + std::to_string(power) + ", " + std::to_string(pid) + ")");
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

        inputValue = std::max(inputMin, std::min(inputValue, inputMax));

        double mappedValue = (((inputValue - inputMin) / (inputMax - inputMin)) * (outputMax - outputMin) + outputMin);

        return mappedValue;
    }

public:

	double power = 0;

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
        double pwmm = mapValue(input, mapMIN, mapMAX, 0, 255);
        power = mapValue(input, mapMIN, mapMAX, 0, 100);
        std::cout << "pwm:         " << pwmm << std::endl;
        std::cout << "power:       " << power << "%" << std::endl;
        std::cout << std::endl;

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
    double integral = 0;    // integral of error
    double derivative = 0;      // derivative of error
    double kp = KP;         // proportional gain
    double ki = KI;         // integral gain
    double kd = KD;         // derivative gain

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

    double pVal = 0;
    double iVal = 0;
    double dVal = 0;
    double output = 0;          // output of the controller

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

        std::cout << "error:       " << error << std::endl;
        std::cout << "P:           " << pVal << std::endl;
        std::cout << "I:           " << iVal << std::endl;
        std::cout << "D:           " << dVal << std::endl;
        std::cout << "PID:         " << output << std::endl;
        return output;
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
     	sensor.readHumidity();
       	sensor.readTemperature();
        sensor.WHATISHAPPENING();


        // heater.controlTemperatureOnOff(sensor.temperatureCelsius);
        heater.controlTemperaturePWM(magic.calculatePID(sensor.temperatureCelsius));

        // Insert data into MySQL database
	databaseConnection.insertSensorData(sensor.humidityPercentage, sensor.temperatureCelsius, magic.pVal, magic.iVal, magic.dVal, magic.output, heater.power);

        sleep(sleeptime); // Wait before reading again
    }

    pigpio_stop(pi); // Disconnect from the pigpiod daemon
    return 0;
}
