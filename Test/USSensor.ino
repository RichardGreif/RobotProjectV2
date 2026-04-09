#include "Config.h"
#include "MotorDriver.h"
#include "SpeedSensor.h"
#include "DriveController.h"
#include "UltrasonicArray.h"
#include "RemotePause.h"
#include "DebugOutput.h"

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
RemotePause remotePause(WifiConfig::Ssid, WifiConfig::Password);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Start");

  motors.begin();
  speedSensors.begin();
  driveController.begin();
  ultrasonicArray.begin();
  remotePause.begin();

  driveController.stop();
}


void loop() {
  unsigned long now = millis();

  remotePause.update();
  speedSensors.update(now);

  if (!remotePause.isConnected() || remotePause.isPaused()) {
    driveController.stop();
    return;
  }


  // Jeden Sensor updaten
  ultrasonicArray.update(now);
  ultrasonicArray.update(now);
  ultrasonicArray.update(now);

  if (now - lastDebugMs >= debugIntervalMs) {
    lastDebugMs += debugIntervalMs;

    DebugOutput::printUltrasonicPlot(ultrasonicArray);
  }
}