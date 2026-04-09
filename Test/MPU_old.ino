#include "Config.h"
#include "MotorDriver.h"
#include "SpeedSensor.h"
#include "DriveController.h"
#include "UltrasonicArray.h"
#include "RemotePause.h"
#include "DebugOutput.h"
#include "CliffSensor.h"
#include "Adafruit_MCP23X17.h"
#include <Wire.h>


// Quick debug:
unsigned long lastDebugMs = 0;
const unsigned long debugIntervalMs = 100;


// Debug end

MotorDriver motors(
  Pins::PWMA,
  Pins::AIN1,
  Pins::AIN2,
  Pins::PWMB,
  Pins::BIN1,
  Pins::BIN2,
  MotorConfig::LeftForwardHigh,
  MotorConfig::RightForwardHigh,
  MotorConfig::PwmFrequency,
  MotorConfig::PwmResolution);

SpeedSensor speedSensors(Pins::LeftSPEED, Pins::RightSPEED);
DriveController driveController(motors, speedSensors);
UltrasonicArray ultrasonicArray(
  Pins::US1_TRIG,
  Pins::US1_ECHO,
  Pins::US2_TRIG,
  Pins::US2_ECHO,
  Pins::US3_TRIG,
  Pins::US3_ECHO);

CliffSensor cliffSensor(Pins::LeftCliff, Pins::CenterCliff, Pins::RightCliff, I2CAddress::MCP23017_A);
RemotePause remotePause(WifiConfig::Ssid, WifiConfig::Password);
  
constexpr uint8_t MpuAddress = I2CAddress::MPU6050;

void writeRegister(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(MpuAddress);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}


uint8_t readRegister(uint8_t reg)
{
    Wire.beginTransmission(MpuAddress);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0)
    {
        return 0xFF;
    }

    if (Wire.requestFrom(MpuAddress, (uint8_t)1) != 1)
    {
        return 0xFF;
    }

    return Wire.read();
}

bool readRegisters(uint8_t startRegister, uint8_t count, uint8_t* buffer)
{
    Wire.beginTransmission(MpuAddress);
    Wire.write(startRegister);

    if (Wire.endTransmission(false) != 0)
    {
        return false;
    }

    const uint8_t received = Wire.requestFrom(MpuAddress, count);
    if (received != count)
    {
        return false;
    }

    for (uint8_t i = 0; i < count; i++)
    {
        buffer[i] = Wire.read();
    }

    return true;
}

int16_t combineBytes(uint8_t highByte, uint8_t lowByte)
{
    return static_cast<int16_t>((highByte << 8) | lowByte);
}

void printRegister(uint8_t reg)
{
    const uint8_t value = readRegister(reg);
    Serial.print("Reg 0x");
    Serial.print(reg, HEX);
    Serial.print(" = 0x");
    Serial.println(value, HEX);
}

bool initializeMpu()
{
    uint8_t whoAmI = 0;

    if (!readRegisters(0x75, 1, &whoAmI))
    {
        return false;
    }

    Serial.print("WHO_AM_I: 0x");
    Serial.println(whoAmI, HEX);

    if (whoAmI != 0x68 && whoAmI != 0x70)
    {
      return false;
    }

    writeRegister(0x6B, 0x00);
    delay(100);

    writeRegister(0x1C, 0x00);
    writeRegister(0x1B, 0x00);

    return true;
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Wire.begin(Pins::SDA, Pins::SCL);
    delay(100);
    
    cliffSensor.begin();

    Serial.println("Start MPU6050 test");

        for (int i = 0; i < 5; i++)
    {
        printRegister(0x75);
        printRegister(0x6B);
        printRegister(0x1C);
        printRegister(0x1B);
        Serial.println("---");
        delay(500);
    }

    if (!initializeMpu())
    {
        Serial.println("MPU6050 Init fehlgeschlagen");
        while (true)
        {
            delay(1000);
        }
    }

    Serial.println("MPU6050 bereit");
}

void loop()
{
    uint8_t data[14];

    if (!readRegisters(0x3B, 14, data))
    {
        Serial.println("Lesefehler");
        delay(200);
        return;
    }

    const int16_t accX = combineBytes(data[0], data[1]);
    const int16_t accY = combineBytes(data[2], data[3]);
    const int16_t accZ = combineBytes(data[4], data[5]);
    const int16_t tempRaw = combineBytes(data[6], data[7]);
    const int16_t gyroX = combineBytes(data[8], data[9]);
    const int16_t gyroY = combineBytes(data[10], data[11]);
    const int16_t gyroZ = combineBytes(data[12], data[13]);

    const float temperatureC = tempRaw / 340.0f + 36.53f;

    Serial.print("Acc: ");
    Serial.print(accX);
    Serial.print(' ');
    Serial.print(accY);
    Serial.print(' ');
    Serial.print(accZ);

    Serial.print(" | Gyro: ");
    Serial.print(gyroX);
    Serial.print(' ');
    Serial.print(gyroY);
    Serial.print(' ');
    Serial.print(gyroZ);

    Serial.print(" | Temp: ");
    Serial.println(temperatureC);

    delay(100);
}