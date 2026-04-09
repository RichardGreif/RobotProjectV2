#pragma once

#include <Arduino.h>
#include <Wire.h>

class MpuSensor
{
public:
  explicit MpuSensor(uint8_t i2cAddress);

  bool begin(TwoWire& wire = Wire);
  bool update();

  bool isAvailable() const;

  int16_t getAccXRaw() const;
  int16_t getAccYRaw() const;
  int16_t getAccZRaw() const;

  int16_t getGyroXRaw() const;
  int16_t getGyroYRaw() const;
  int16_t getGyroZRaw() const;

  float getAccXg() const;
  float getAccYg() const;
  float getAccZg() const;

  float getGyroXdps() const;
  float getGyroYdps() const;
  float getGyroZdps() const;

  float getTemperatureC() const;

  uint8_t getWhoAmI() const;

private:
  static constexpr uint8_t WhoAmIRegister = 0x75;
  static constexpr uint8_t PowerManagement1Register = 0x6B;
  static constexpr uint8_t AccelerometerConfigRegister = 0x1C;
  static constexpr uint8_t GyroscopeConfigRegister = 0x1B;
  static constexpr uint8_t SensorDataStartRegister = 0x3B;

  static constexpr float AccelerometerScaleLsbPerG = 16384.0f;
  static constexpr float GyroscopeScaleLsbPerDps = 131.0f;

  uint8_t _i2cAddress;
  TwoWire* _wire;
  bool _isAvailable;
  uint8_t _whoAmI;

  int16_t _accXRaw;
  int16_t _accYRaw;
  int16_t _accZRaw;
  int16_t _temperatureRaw;
  int16_t _gyroXRaw;
  int16_t _gyroYRaw;
  int16_t _gyroZRaw;

  bool writeRegister(uint8_t reg, uint8_t value);
  bool readRegisters(uint8_t startRegister, uint8_t count, uint8_t* buffer);
  static int16_t combineBytes(uint8_t highByte, uint8_t lowByte);
};