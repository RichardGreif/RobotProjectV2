#include "Config.h"
#include "MotorDriver.h"
#include "SpeedSensor.h"
#include "DriveController.h"
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
RemotePause remotePause(WifiConfig::Ssid, WifiConfig::Password);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Start");

  motors.begin();
  speedSensors.begin();
  driveController.begin();
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

  driveController.setTargetSpeeds(40.0f, 40.0f);
  driveController.update(now);

  
  if (now - lastDebugMs >= debugIntervalMs) {
    lastDebugMs += debugIntervalMs;

    DebugOutput::printDrivePlot(driveController);
  }
}