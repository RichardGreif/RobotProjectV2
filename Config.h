#pragma once
#include <Arduino.h>

namespace Pins {
  constexpr int PWMA = 21;
  constexpr int AIN1 = 18;
  constexpr int AIN2 = 19;

  constexpr int PWMB = 4;
  constexpr int BIN1 = 17;
  constexpr int BIN2 = 16;

  constexpr int LeftSPEED = 39;
  constexpr int RightSPEED = 36;

  // TODO: Check and add IIC Configuration

  constexpr int US1_TRIG = 13;
  constexpr int US1_ECHO = 14;
  constexpr int US2_TRIG = 27;
  constexpr int US2_ECHO = 26;
  constexpr int US3_TRIG = 25;
  constexpr int US3_ECHO = 33;

  constexpr int SDA = 23;
  constexpr int SCL = 22;

constexpr int LeftCliff = 10;
constexpr int CenterCliff = 9;
constexpr int RightCliff = 8;
}


namespace I2CAddress
{
    constexpr uint8_t MCP23017_A = 0x20;
    constexpr uint8_t MCP23017_B = 0x21;
    constexpr uint8_t MPU6050  = 0x68;
}

namespace MotorConfig {
  constexpr int PwmFrequency = 1000;
  constexpr int PwmResolution = 8;
  constexpr bool LeftForwardHigh = true;
  constexpr bool RightForwardHigh = false;
}

namespace EncoderConfig {
  constexpr unsigned long SampleTimeMs = 100;
}

namespace WifiConfig {
  constexpr const char* Ssid = "FRITZ!Box 7430 HM";
  constexpr const char* Password = "34530990654990042287";
}

