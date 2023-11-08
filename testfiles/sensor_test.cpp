#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <ctime>

class Si7021Sensor
{
private:
    int i2cHandle;

public:
    Si7021Sensor(const std::string &i2cDevice) : i2cHandle(open(i2cDevice.c_str(), O_RDWR))
    {
        if (i2cHandle == -1)
        {
            std::cerr << "Error: Unable to open I2C device." << std::endl;
        }
    }

    ~Si7021Sensor()
    {
        if (i2cHandle != -1)
        {
            close(i2cHandle);
        }
    }

    bool waitForMeasurementCompletion()
    {
        const int maxAttempts = 100;
        const int delayBetweenAttemptsMs = 10;

        for (int attempt = 0; attempt < maxAttempts; ++attempt)
        {
            unsigned char status;
            if (read(i2cHandle, &status, sizeof(status)) == sizeof(status))
            {
                if (!(status & 0x01)) // Check the status for the measurement completion bit (bit 0)
                {
                    return true; // Measurement completed
                }
            }

            usleep(delayBetweenAttemptsMs * 1000); // Wait for a short duration before the next attempt
        }

        std::cerr << "Error: Timeout waiting for measurement completion." << std::endl;
        return false;
    }

    float readTemperature()
    {
        unsigned char command[1] = {0xE3}; // Temperature measurement command
        write(i2cHandle, command, sizeof(command));

        if (waitForMeasurementCompletion())
        {
            unsigned char data[2];
            read(i2cHandle, data, sizeof(data));

            int rawTemperature = (data[0] << 8) | data[1];
            float temperature = ((static_cast<float>(rawTemperature) / 65536) * 175.72) - 46.85;

            return temperature;
        }

        return -1.0f; // Return an error value if the measurement failed
    }
};

int main()
{
    const std::string i2cDevice = "/dev/i2c-1"; // I2C device file
    Si7021Sensor sensor(i2cDevice);

    while (true)
    {
        float temperature = sensor.readTemperature();

        if (temperature != -1.0f)
        {
            std::cout << "Temperature: " << temperature << " °C" << std::endl;
        }

        // Add a delay before reading sensor values again
        usleep(1000000); // Sleep for 1 second
    }

    return 0;
}
