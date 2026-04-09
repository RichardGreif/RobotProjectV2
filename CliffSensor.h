#pragma once

#include <Adafruit_MCP23X17.h>

class CliffSensor {
public:
  CliffSensor(
    int pinLeft,
    int pinCenter,
    int pinRight,
    uint8_t i2cAddress = 0x20,
    int interruptPin = -1);

  void begin();
  void update();

  bool isLeftCliff() const;
  bool isCenterCliff() const;
  bool isRightCliff() const;
  bool hasCliff() const;

  bool wasInterruptTriggered();
  void clearInterruptFlag();
  void printAllPins();

private:
  int _pinLeft;
  int _pinCenter;
  int _pinRight;
  uint8_t _i2cAddress;
  int _interruptPin;

  volatile bool _interruptTriggered;

  bool _leftCliff;
  bool _centerCliff;
  bool _rightCliff;
  bool _isAvailable;

  Adafruit_MCP23X17 _mcp;

  static CliffSensor* _instance;

  static void IRAM_ATTR handleMcpInterrupt();
  void onInterrupt();

  bool readCliff(int pin);
};