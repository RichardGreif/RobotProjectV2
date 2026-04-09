#include "MpuSensor.h"

MpuSensor::MpuSensor(uint8_t i2cAddress)
  : _i2cAddress(i2cAddress),
    _wire(nullptr),
    _isAvailable(false),
    _whoAmI(0),
    _accXRaw(0),
    _accYRaw(0),
    _accZRaw(0),
    _temperatureRaw(0),
    _gyroXRaw(0),
    _gyroYRaw(0),
    _gyroZRaw(0)
{
}

bool MpuSensor::begin(TwoWire& wire)
{
  _wire = &wire;
  _isAvailable = false;
  _whoAmI = 0;

  if (!readRegisters(WhoAmIRegister, 1, &_whoAmI))
  {
    return false;
  }

  if (_whoAmI != 0x68 && _whoAmI != 0x70)
  {
    return false;
  }

  if (!writeRegister(PowerManagement1Register, 0x00))
  {
    return false;
  }

  delay(100);

  if (!writeRegister(AccelerometerConfigRegister, 0x00))
  {
    return false;
  }

  if (!writeRegister(GyroscopeConfigRegister, 0x00))
  {
    return false;
  }

  _isAvailable = true;
  return update();
}

bool MpuSensor::update()
{
  if (_wire == nullptr || !_isAvailable)
  {
    return false;
  }

  uint8_t data[14];

  if (!readRegisters(SensorDataStartRegister, 14, data))
  {
    _isAvailable = false;
    return false;
  }

  _accXRaw = combineBytes(data[0], data[1]);
  _accYRaw = combineBytes(data[2], data[3]);
  _accZRaw = combineBytes(data[4], data[5]);
  _temperatureRaw = combineBytes(data[6], data[7]);
  _gyroXRaw = combineBytes(data[8], data[9]);
  _gyroYRaw = combineBytes(data[10], data[11]);
  _gyroZRaw = combineBytes(data[12], data[13]);

  return true;
}

bool MpuSensor::isAvailable() const
{
  return _isAvailable;
}

int16_t MpuSensor::getAccXRaw() const
{
  return _accXRaw;
}

int16_t MpuSensor::getAccYRaw() const
{
  return _accYRaw;
}

int16_t MpuSensor::getAccZRaw() const
{
  return _accZRaw;
}

int16_t MpuSensor::getGyroXRaw() const
{
  return _gyroXRaw;
}

int16_t MpuSensor::getGyroYRaw() const
{
  return _gyroYRaw;
}

int16_t MpuSensor::getGyroZRaw() const
{
  return _gyroZRaw;
}

float MpuSensor::getAccXg() const
{
  return static_cast<float>(_accXRaw) / AccelerometerScaleLsbPerG;
}

float MpuSensor::getAccYg() const
{
  return static_cast<float>(_accYRaw) / AccelerometerScaleLsbPerG;
}

float MpuSensor::getAccZg() const
{
  return static_cast<float>(_accZRaw) / AccelerometerScaleLsbPerG;
}

float MpuSensor::getGyroXdps() const
{
  return static_cast<float>(_gyroXRaw) / GyroscopeScaleLsbPerDps;
}

float MpuSensor::getGyroYdps() const
{
  return static_cast<float>(_gyroYRaw) / GyroscopeScaleLsbPerDps;
}

float MpuSensor::getGyroZdps() const
{
  return static_cast<float>(_gyroZRaw) / GyroscopeScaleLsbPerDps;
}

float MpuSensor::getTemperatureC() const
{
  return static_cast<float>(_temperatureRaw) / 340.0f + 36.53f;
}

uint8_t MpuSensor::getWhoAmI() const
{
  return _whoAmI;
}

bool MpuSensor::writeRegister(uint8_t reg, uint8_t value)
{
  if (_wire == nullptr)
  {
    return false;
  }

  _wire->beginTransmission(_i2cAddress);
  _wire->write(reg);
  _wire->write(value);
  return _wire->endTransmission() == 0;
}

bool MpuSensor::readRegisters(uint8_t startRegister, uint8_t count, uint8_t* buffer)
{
  if (_wire == nullptr)
  {
    return false;
  }

  _wire->beginTransmission(_i2cAddress);
  _wire->write(startRegister);

  if (_wire->endTransmission(false) != 0)
  {
    return false;
  }

  const uint8_t received = _wire->requestFrom(_i2cAddress, count);
  if (received != count)
  {
    return false;
  }

  for (uint8_t i = 0; i < count; i++)
  {
    buffer[i] = _wire->read();
  }

  return true;
}

int16_t MpuSensor::combineBytes(uint8_t highByte, uint8_t lowByte)
{
  return static_cast<int16_t>((static_cast<uint16_t>(highByte) << 8) | lowByte);
}