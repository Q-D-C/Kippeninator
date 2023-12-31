#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

class SI7021Sensor {
private:
    int file;
    const int I2C_ADDRESS = 0x40; // SI7021 sensor I2C address

public:
    SI7021Sensor(const char* filename) {
        if ((file = open(filename, O_RDWR)) < 0) {
            std::cerr << "Failed to open the i2c bus" << std::endl;
            exit(1);
        }

        if (ioctl(file, I2C_SLAVE, I2C_ADDRESS) < 0) {
            std::cerr << "Failed to acquire bus access" << std::endl;
            exit(1);
        }
    }

    ~SI7021Sensor() {
        close(file);
    }

    double readHumidity() {
        unsigned char humidityCommand[1] = {0xE5};
        if (write(file, humidityCommand, 1) != 1) {
            std::cerr << "Error writing humidity command" << std::endl;
            exit(1);
        }

        usleep(100000); // Wait for measurement to be completed

        unsigned char humidityData[2];
        if (read(file, humidityData, 2) != 2) {
            std::cerr << "Error reading humidity data" << std::endl;
            exit(1);
        }

        int humidity = ((static_cast<int>(humidityData[0]) << 8) | static_cast<int>(humidityData[1]));
        double humidityPercentage = (125.0 * humidity / 65536.0) - 6.0;

        return humidityPercentage;
    }

    double readTemperature() {
        unsigned char tempCommand[1] = {0xE3};
        if (write(file, tempCommand, 1) != 1) {
            std::cerr << "Error writing temperature command" << std::endl;
            exit(1);
        }

        usleep(100000); // Wait for measurement to be completed

        unsigned char tempData[2];
        if (read(file, tempData, 2) != 2) {
            std::cerr << "Error reading temperature data " << std::endl;
            exit(1);
        }

        int temperature = ((static_cast<int>(tempData[0]) << 8) | static_cast<int>(tempData[1]));
        double temperatureCelsius = (175.72 * temperature / 65536.0) - 46.85;

        return temperatureCelsius;
    }
};

int main() {
    const char* filename = "/dev/i2c-1"; // I2C interface on Raspberry Pi
    SI7021Sensor sensor(filename);

    while (true) {

        std::cout << "Relative Humidity: " << sensor.readHumidity() << "%" << std::endl;
        std::cout << "Temperature: " << sensor.readTemperature() << "°C" << std::endl;

        sleep(1); // Wait for 1 second before reading again
    }

    return 0;
}
