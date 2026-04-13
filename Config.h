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

namespace SnapshotStreamConfig {
  constexpr float InitialRobotXcm = 0.0f;
  constexpr float InitialRobotYcm = 0.0f;
  constexpr float InitialRobotYawRad = 0.0f;

  constexpr uint8_t ViewerIp0 = 192;
  constexpr uint8_t ViewerIp1 = 168;
  constexpr uint8_t ViewerIp2 = 178;
  constexpr uint8_t ViewerIp3 = 45;

  constexpr uint16_t ViewerPort = 4242;
  constexpr unsigned long SendIntervalMs = 500;
  constexpr unsigned long WifiReconnectIntervalMs = 5000;

  constexpr float DistanceScaleToViewerUnits = 0.01f;
  constexpr float MinObservationConfidence = 0.05f;
  constexpr float MaxValidDistanceCm = 300.0f;

  constexpr float LeftSensorXcm = -7.0f;
  constexpr float LeftSensorYcm = 7.0f;
  constexpr float LeftSensorYawRad = -1.5708f;

  constexpr float CenterSensorXcm = 0.0f;
  constexpr float CenterSensorYcm = 10.0f;
  constexpr float CenterSensorYawRad = 0.0f;

  constexpr float RightSensorXcm = 7.0f;
  constexpr float RightSensorYcm = 7.0f;
  constexpr float RightSensorYawRad = 1.5708f;
}

namespace OdometryConfig {
  constexpr float WheelDistanceCm = 12.0f; // gemessen Abstand von Radmitte zu Radmitte in cm

  // Startwert: 1 Hz Sensorsignal entspricht grob 1 cm/s Radgeschwindigkeit.
  // Diesen Faktor bitte spaeter mit realer Fahrstrecke kalibrieren.
  constexpr float LeftWheelSpeedCmPerSecPerHz = 1.0f;
  constexpr float RightWheelSpeedCmPerSecPerHz = 1.0f;

  constexpr float LeftDirectionSign = 1.0f;
  constexpr float RightDirectionSign = 1.0f;
}

namespace StrategyConfig {
  constexpr float StraightTargetSpeedHz = 20.0f;
  constexpr float TurnTargetSpeedHz = 15.0f;
  constexpr unsigned long StraightStepCount = 100;
  constexpr unsigned long WaitAfterStraightMs = 10000;
  constexpr float TurnAngleRad = 3.14159f;
}

