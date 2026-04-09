#include "CliffSensor.h"
#include "Config.h"

#include <Arduino.h>
#include <Wire.h>

CliffSensor* CliffSensor::_instance = nullptr;

CliffSensor::CliffSensor(
  int pinLeft,
  int pinCenter,
  int pinRight,
  uint8_t i2cAddress,
  int interruptPin)
  : _pinLeft(pinLeft),
    _pinCenter(pinCenter),
    _pinRight(pinRight),
    _i2cAddress(i2cAddress),
    _interruptPin(interruptPin),
    _interruptTriggered(false),
    _leftCliff(false),
    _centerCliff(false),
    _rightCliff(false),
    _isAvailable(false) {
}


void scanI2C()
{
    Serial.println("Scanning I2C...");

    int found = 0;

    for (uint8_t address = 1; address < 127; address++)
    {
        Wire.beginTransmission(address);
        uint8_t error = Wire.endTransmission();

        if (error == 0)
        {
            Serial.print("Found device at 0x");
            Serial.println(address, HEX);
            found++;
        }
    }

    if (found == 0)
    {
        Serial.println("No I2C devices found");
    }
    else
    {
        Serial.println("Scan done");
    }
}

void CliffSensor::begin() {
  _instance = this;

  scanI2C();

  if (!_mcp.begin_I2C(_i2cAddress, &Wire)) {
    Serial.println("CliffSensor: MCP23017 nicht gefunden");
    _isAvailable = false;
    return;
  }

  _isAvailable = true;

  _mcp.pinMode(_pinLeft, INPUT);
  _mcp.pinMode(_pinCenter, INPUT);
  _mcp.pinMode(_pinRight, INPUT);

  // _mcp.pullUp(_pinLeft, HIGH);
  // _mcp.pullUp(_pinCenter, HIGH);
  // _mcp.pullUp(_pinRight, HIGH);

  update();

  if (_interruptPin >= 0) {
    _mcp.setupInterrupts(true, false, HIGH);
    _mcp.setupInterruptPin(_pinLeft, CHANGE);
    _mcp.setupInterruptPin(_pinCenter, CHANGE);
    _mcp.setupInterruptPin(_pinRight, CHANGE);

    pinMode(_interruptPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(_interruptPin), handleMcpInterrupt, RISING);
    _mcp.clearInterrupts();
  }
}

void CliffSensor::update() {
  if (!_isAvailable) {
    _leftCliff = false;
    _centerCliff = false;
    _rightCliff = false;
    return;
  }

  const bool newLeftCliff = readCliff(_pinLeft);
  const bool newCenterCliff = readCliff(_pinCenter);
  const bool newRightCliff = readCliff(_pinRight);

  if (newLeftCliff != _leftCliff ||
      newCenterCliff != _centerCliff ||
      newRightCliff != _rightCliff) {
    _interruptTriggered = true;
  }

  _leftCliff = newLeftCliff;
  _centerCliff = newCenterCliff;
  _rightCliff = newRightCliff;

  if (_interruptPin >= 0) {
    _mcp.clearInterrupts();
  }
}

bool CliffSensor::isLeftCliff() const {
  return _leftCliff;
}

bool CliffSensor::isCenterCliff() const {
  return _centerCliff;
}

bool CliffSensor::isRightCliff() const {
  return _rightCliff;
}

bool CliffSensor::hasCliff() const {
  return _leftCliff || _centerCliff || _rightCliff;
}

bool CliffSensor::wasInterruptTriggered() {
  noInterrupts();
  const bool value = _interruptTriggered;
  interrupts();
  return value;
}

void CliffSensor::clearInterruptFlag() {
  noInterrupts();
  _interruptTriggered = false;
  interrupts();
}

void IRAM_ATTR CliffSensor::handleMcpInterrupt() {
  if (_instance) {
    _instance->onInterrupt();
  }
}

void CliffSensor::onInterrupt() {
  _interruptTriggered = true;
}

bool CliffSensor::readCliff(int pin) {
  if (!_isAvailable) {
    return false;
  }

  return _mcp.digitalRead(pin) == HIGH;
}

void CliffSensor::printAllPins()
{
  if (!_isAvailable)
  {
    Serial.println("MCP nicht verfügbar");
    return;
  }

  Serial.print("MCP Pins: ");

  for (int i = 0; i < 16; i++)
  {
    int value = _mcp.digitalRead(i);

    Serial.print(value);

    if (i < 15)
    {
      Serial.print(' ');
    }
  }

  Serial.println();
}
