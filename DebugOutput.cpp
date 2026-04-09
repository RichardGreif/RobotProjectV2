#include "DebugOutput.h"
#include "DriveController.h"
#include "UltrasonicArray.h"
#include "CliffSensor.h"
#include "MpuSensor.h"


#include <Adafruit_MCP23X17.h>
#include <Arduino.h>




namespace DebugOutput
{
  void printDrivePlot(const DriveController& driveController) {
    Serial.print("TargetLeft:");
    Serial.print(driveController.getTargetLeftSpeedHz());
    Serial.print(",");

    Serial.print("TargetRight:");
    Serial.print(driveController.getTargetRightSpeedHz());
    Serial.print(",");

    Serial.print("MeasuredLeft:");
    Serial.print(driveController.getMeasuredLeftSpeedHz());
    Serial.print(",");

    Serial.print("MeasuredRight:");
    Serial.print(driveController.getMeasuredRightSpeedHz());
    Serial.print(",");

    Serial.print("PwmLeft:");
    Serial.print(driveController.getLeftPwmPercent());
    Serial.print(",");

    Serial.print("PwmRight:");
    Serial.println(driveController.getRightPwmPercent());
  }

  void printUltrasonicPlot(const UltrasonicArray& ultrasonicArray)
  {
      const float left = ultrasonicArray.getLeftDistanceCm();
      const float front = ultrasonicArray.getCenterDistanceCm();
      const float right = ultrasonicArray.getRightDistanceCm();

      Serial.print(left);
      Serial.print(" ");
      Serial.print(front);
      Serial.print(" ");
      Serial.println(right);
  }

  void printCliffPlot(const CliffSensor& cliffSensor)
    {
        Serial.print(cliffSensor.isLeftCliff() ? 1 : 0);
        Serial.print(' ');
        Serial.print(cliffSensor.isCenterCliff() ? 1 : 0);
        Serial.print(' ');
        Serial.println(cliffSensor.isRightCliff() ? 1 : 0);
    }

  void printMcpPins(Adafruit_MCP23X17& mcp)
    {
        Serial.println("mcp pins: ");

        for (int i = 0; i < 16; i++)
        {
            Serial.print(mcp.digitalRead(i));
            Serial.print(' ');
        }
        Serial.println();
    }

    void printMpuAccelerationPlot(const MpuSensor& mpuSensor)
    {
        Serial.print(mpuSensor.getAccXg());
        Serial.print(' ');
        Serial.print(mpuSensor.getAccYg());
        Serial.print(' ');
        Serial.println(mpuSensor.getAccZg());
    }

    void printMpuGyroPlot(const MpuSensor& mpuSensor)
    {
        Serial.print(mpuSensor.getGyroXdps());
        Serial.print(' ');
        Serial.print(mpuSensor.getGyroYdps());
        Serial.print(' ');
        Serial.println(mpuSensor.getGyroZdps());
    }
}