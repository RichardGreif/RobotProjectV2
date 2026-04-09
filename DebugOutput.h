#pragma once

class DriveController;

#pragma once

class DriveController;
class UltrasonicArray;
class CliffSensor;
class Adafruit_MCP23X17;
class MpuSensor;

namespace DebugOutput
{
    void printDrivePlot(const DriveController& driveController);
    void printUltrasonicPlot(const UltrasonicArray& ultrasonicArray);
    void printCliffPlot(const CliffSensor& cliffSensor);
    void printMcpPins(Adafruit_MCP23X17& mcp);
    void printMpuAccelerationPlot(const MpuSensor& mpuSensor);
    void printMpuGyroPlot(const MpuSensor& mpuSensor);
}