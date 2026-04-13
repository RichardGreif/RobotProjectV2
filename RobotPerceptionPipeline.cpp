#include "RobotPerceptionPipeline.h"

#include "Config.h"
#include "SLAM/MapSnapshot.h"
#include "SLAM/Mapper.h"
#include "SLAM/Measurement.h"
#include "SnapshotTcpClient.h"
#include "UltrasonicArray.h"

namespace
{
  constexpr bool measurementDebugEnabled = true;

  UltrasonicArray ultrasonicArray(
    Pins::US1_TRIG,
    Pins::US1_ECHO,
    Pins::US2_TRIG,
    Pins::US2_ECHO,
    Pins::US3_TRIG,
    Pins::US3_ECHO);

  SLAM::Mapper mapper(SnapshotStreamConfig::MinObservationConfidence);

  SnapshotTcpClient snapshotClient(
    WifiConfig::Ssid,
    WifiConfig::Password,
    IPAddress(
      SnapshotStreamConfig::ViewerIp0,
      SnapshotStreamConfig::ViewerIp1,
      SnapshotStreamConfig::ViewerIp2,
      SnapshotStreamConfig::ViewerIp3),
    SnapshotStreamConfig::ViewerPort,
    SnapshotStreamConfig::DistanceScaleToViewerUnits,
    SnapshotStreamConfig::WifiReconnectIntervalMs);

  unsigned long lastSnapshotSendMs = 0;
  unsigned long lastLogMs = 0;

  void AddMeasurementIfValid(
    const Pose2D& robotPose,
    int sensorIndex,
    float distanceCm,
    float sensorXcm,
    float sensorYcm,
    float sensorYawRad)
  {
    if (distanceCm <= 0.0f || distanceCm > SnapshotStreamConfig::MaxValidDistanceCm)
    {
      if (measurementDebugEnabled)
      {
        Serial.print("[Measure] skip sensor=");
        Serial.print(sensorIndex);
        Serial.print(" distanceCm=");
        Serial.print(distanceCm, 3);
        Serial.print(" max=");
        Serial.println(SnapshotStreamConfig::MaxValidDistanceCm, 3);
      }
      return;
    }

    const SLAM::MeasurementPoint measurement = SLAM::CreateMeasurement(
      robotPose,
      sensorXcm,
      sensorYcm,
      sensorYawRad,
      distanceCm);

    if (measurementDebugEnabled)
    {
      Serial.print("[Measure] sensor=");
      Serial.print(sensorIndex);
      Serial.print(" distanceCm=");
      Serial.print(distanceCm, 3);
      Serial.print(" sensorLocal=(");
      Serial.print(sensorXcm, 3);
      Serial.print(",");
      Serial.print(sensorYcm, 3);
      Serial.print(",");
      Serial.print(sensorYawRad, 5);
      Serial.print(") worldPos=(");
      Serial.print(measurement.position.x, 3);
      Serial.print(",");
      Serial.print(measurement.position.y, 3);
      Serial.print(") worldDir=(");
      Serial.print(measurement.direction.x, 3);
      Serial.print(",");
      Serial.print(measurement.direction.y, 3);
      Serial.println(")");
    }

    mapper.ProcessMeasurement(sensorIndex, measurement);
  }

  void UpdateMapperFromUltrasonicReadings(const Pose2D& robotPose)
  {
    AddMeasurementIfValid(
      robotPose,
      0,
      ultrasonicArray.getLeftDistanceCm(),
      SnapshotStreamConfig::LeftSensorXcm,
      SnapshotStreamConfig::LeftSensorYcm,
      SnapshotStreamConfig::LeftSensorYawRad);

    AddMeasurementIfValid(
      robotPose,
      1,
      ultrasonicArray.getCenterDistanceCm(),
      SnapshotStreamConfig::CenterSensorXcm,
      SnapshotStreamConfig::CenterSensorYcm,
      SnapshotStreamConfig::CenterSensorYawRad);

    AddMeasurementIfValid(
      robotPose,
      2,
      ultrasonicArray.getRightDistanceCm(),
      SnapshotStreamConfig::RightSensorXcm,
      SnapshotStreamConfig::RightSensorYcm,
      SnapshotStreamConfig::RightSensorYawRad);
  }
}

namespace RobotPerceptionPipeline
{
  void Setup()
  {
    ultrasonicArray.begin();
    snapshotClient.begin();
  }

  void Update(
    unsigned long nowMs,
    const Pose2D& robotPose,
    float leftWheelLinearSpeedCmPerSec,
    float rightWheelLinearSpeedCmPerSec)
  {
    snapshotClient.update(nowMs);
    ultrasonicArray.update(nowMs);

    if (nowMs - lastSnapshotSendMs < SnapshotStreamConfig::SendIntervalMs)
    {
      return;
    }

    lastSnapshotSendMs = nowMs;

    UpdateMapperFromUltrasonicReadings(robotPose);

    const SLAM::MapSnapshot snapshot = SLAM::CreateMapSnapshot(mapper.GetWallMap());
    const bool sent = snapshotClient.sendSnapshot(snapshot);

    if (nowMs - lastLogMs >= 1000)
    {
      lastLogMs = nowMs;
      Serial.print("WiFi=");
      Serial.print(snapshotClient.isWifiConnected() ? "connected" : "disconnected");
      Serial.print(" walls=");
      Serial.print(snapshot.wallCount);
      Serial.print(" left=");
      Serial.print(ultrasonicArray.getLeftDistanceCm());
      Serial.print(" center=");
      Serial.print(ultrasonicArray.getCenterDistanceCm());
      Serial.print(" right=");
      Serial.print(ultrasonicArray.getRightDistanceCm());
      Serial.print(" pose=(");
      Serial.print(robotPose.position.x);
      Serial.print(",");
      Serial.print(robotPose.position.y);
      Serial.print(",");
      Serial.print(robotPose.yaw);
      Serial.print(")");
      Serial.print(" wheelCmPerSec=(");
      Serial.print(leftWheelLinearSpeedCmPerSec, 3);
      Serial.print(",");
      Serial.print(rightWheelLinearSpeedCmPerSec, 3);
      Serial.print(")");
      Serial.print(" send=");
      Serial.println(sent ? "ok" : "failed");
    }
  }
}
