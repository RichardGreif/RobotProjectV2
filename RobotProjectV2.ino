#include "MpuSensor.h"
#include "DebugOutput.h"
#include "Config.h"
#include <Wire.h>

MpuSensor mpuSensor(I2CAddress::MPU6050);

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Wire.begin(Pins::SDA, Pins::SCL);

    if (!mpuSensor.begin())
    {
        Serial.println("MPU Init fehlgeschlagen");
        while (true)
        {
            delay(1000);
        }
    }
}

void loop()
{
    if (mpuSensor.update())
    {
        // DebugOutput::printMpuAccelerationPlot(mpuSensor);
        DebugOutput::printMpuGyroPlot(mpuSensor);
    }

    delay(100);
}