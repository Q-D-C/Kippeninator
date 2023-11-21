/// usr/bin/g++ -g database_integration.cpp -o database_integration -lmysqlcppconn

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <mysql_driver.h>
#include <mysql_connection.h>
#include <cppconn/statement.h>

//define the amount of time between readings
#define sleeptime 1

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
    SI7021Sensor(const char *filename)
    {
        if ((file = open(filename, O_RDWR)) < 0)
        {
            std::cerr << "Failed to open the i2c bus" << std::endl;
            exit(1);
        }

        if (ioctl(file, I2C_SLAVE, I2C_ADDRESS) < 0)
        {
            std::cerr << "Failed to acquire bus access" << std::endl;
            exit(1);
        }
    }

    ~SI7021Sensor()
    {
        close(file);
    }

    double readHumidity()
    {
        unsigned char humidityCommand[1] = {0xE5};
        if (write(file, humidityCommand, 1) != 1)
        {
            std::cerr << "Error writing humidity command" << std::endl;
            exit(1);
        }

        usleep(100000); // Wait for measurement to be completed (100ms for hold master mode)

        unsigned char humidityData[2];
        if (read(file, humidityData, 2) != 2)
        {
            std::cerr << "Error reading humidity data" << std::endl;
            exit(1);
        }

        int humidity = ((static_cast<int>(humidityData[0]) << 8) | static_cast<int>(humidityData[1]));
        double humidityPercentage = (125.0 * humidity / 65536.0) - 6.0;

        return humidityPercentage;
    }

    double readTemperature()
    {
        unsigned char tempCommand[1] = {0xE3};
        if (write(file, tempCommand, 1) != 1)
        {
            std::cerr << "Error writing temperature command" << std::endl;
            exit(1);
        }

        usleep(100000); // Wait for measurement to be completed (100ms for hold master mode)

        unsigned char tempData[2];
        if (read(file, tempData, 2) != 2)
        {
            std::cerr << "Error reading temperature data" << std::endl;
            exit(1);
        }

        int temperature = ((static_cast<int>(tempData[0]) << 8) | static_cast<int>(tempData[1]));
        double temperatureCelsius = (175.72 * temperature / 65536.0) - 46.85;

        return temperatureCelsius;
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
            exit(1);
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

int main()
{
    SI7021Sensor sensor(filename);

    MySQLDatabase databaseConnection(host, user, password, database);

    while (true)
    {
        double humidity = sensor.readHumidity();
        double temperature = sensor.readTemperature();

        // Insert data into MySQL database
        databaseConnection.insertSensorData(humidity, temperature);

        std::cout << "Relative Humidity: " << humidity << "%" << std::endl;
        std::cout << "Temperature: " << temperature << "°C" << std::endl;

        sleep(sleeptime); // Wait before reading again
    }

    return 0;
}
