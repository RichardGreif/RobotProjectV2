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
  
void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(Pins::SDA, Pins::SCL);

  Serial.println("Start");

  motors.begin();
  speedSensors.begin();
  driveController.begin();
  ultrasonicArray.begin();
  cliffSensor.begin();
  remotePause.begin();

  driveController.stop();


}


void loop()
{
  // cliffSensor.update();
  DebugOutput::printCliffPlot(cliffSensor);
  cliffSensor.printAllPins();
  delay(200);
}