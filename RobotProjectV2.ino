#include "Config.h"
#include "Common/Geometry.h"
#include "Common/Pose2D.h"
#include "ImuTracker.h"
#include "PoseEstimator.h"
#include "SLAM/MapSnapshot.h"
#include "SLAM/Mapper.h"
#include "SnapshotTcpClient.h"
#include "SpeedSensor.h"
#include "UltrasonicArray.h"
#include "WheelOdometry.h"

#include <cmath>

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

SpeedSensor speedSensors(Pins::LeftSPEED, Pins::RightSPEED);

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

Pose2D initialRobotPose{
  Vec2(SnapshotStreamConfig::InitialRobotXcm, SnapshotStreamConfig::InitialRobotYcm),
  SnapshotStreamConfig::InitialRobotYawRad
};

WheelOdometry wheelOdometry(
  OdometryConfig::WheelBaseCm,
  OdometryConfig::LeftWheelSpeedCmPerSecPerHz,
  OdometryConfig::RightWheelSpeedCmPerSecPerHz,
  OdometryConfig::LeftDirectionSign,
  OdometryConfig::RightDirectionSign);

ImuTracker imuTracker;
PoseEstimator poseEstimator;

unsigned long lastSnapshotSendMs = 0;
unsigned long lastLogMs = 0;

SLAM::MeasurementPoint CreateMeasurement(
  const Pose2D& robotPose,
  float sensorXcm,
  float sensorYcm,
  float sensorYawRad,
  float distanceCm)
{
  const Vec2 sensorPositionLocal(sensorXcm, sensorYcm);
  const Vec2 sensorDirectionLocal(
    std::cos(sensorYawRad) * distanceCm,
    std::sin(sensorYawRad) * distanceCm);

  SLAM::MeasurementPoint measurement;
  measurement.position = Geometry::TransformToWorld(robotPose, sensorPositionLocal);
  measurement.direction = Geometry::Rotate(sensorDirectionLocal, robotPose.yaw);
  return measurement;
}

void AddMeasurementIfValid(
  const Pose2D& robotPose,
  int sensorIndex,
  float distanceCm,
  float sensorXcm,
  float sensorYcm,
  float sensorYawRad)
{
  if (distanceCm <= 0.0f || distanceCm > SnapshotStreamConfig::MaxValidDistanceCm) {
    if (measurementDebugEnabled) {
      Serial.print("[Measure] skip sensor=");
      Serial.print(sensorIndex);
      Serial.print(" distanceCm=");
      Serial.print(distanceCm, 3);
      Serial.print(" max=");
      Serial.println(SnapshotStreamConfig::MaxValidDistanceCm, 3);
    }
    return;
  }

  const SLAM::MeasurementPoint measurement = CreateMeasurement(
    robotPose,
    sensorXcm,
    sensorYcm,
    sensorYawRad,
    distanceCm);

  if (measurementDebugEnabled) {
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

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("RobotProjectV2 snapshot sender starting");

  ultrasonicArray.begin();
  speedSensors.begin();
  snapshotClient.begin();
  imuTracker.Begin();
  wheelOdometry.Reset(initialRobotPose, millis());
  poseEstimator.Reset(initialRobotPose);
}

void loop()
{
  const unsigned long now = millis();

  snapshotClient.update(now);
  speedSensors.update(now);
  imuTracker.Update(now);
  wheelOdometry.Update(now, speedSensors.getLeftSpeedHz(), speedSensors.getRightSpeedHz());
  poseEstimator.UpdateFromSensors(wheelOdometry, imuTracker);
  ultrasonicArray.update(now);

  const Pose2D& currentRobotPose = poseEstimator.GetPose();

  if (now - lastSnapshotSendMs >= SnapshotStreamConfig::SendIntervalMs) {
    lastSnapshotSendMs = now;

    UpdateMapperFromUltrasonicReadings(currentRobotPose);

    const SLAM::MapSnapshot snapshot = SLAM::CreateMapSnapshot(mapper.GetWallMap());
    const bool sent = snapshotClient.sendSnapshot(snapshot);

    if (now - lastLogMs >= 1000) {
      lastLogMs = now;
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
      Serial.print(currentRobotPose.position.x);
      Serial.print(",");
      Serial.print(currentRobotPose.position.y);
      Serial.print(",");
      Serial.print(currentRobotPose.yaw);
      Serial.print(")");
      Serial.print(" wheelCmPerSec=(");
      Serial.print(wheelOdometry.GetLeftLinearSpeedCmPerSec(), 3);
      Serial.print(",");
      Serial.print(wheelOdometry.GetRightLinearSpeedCmPerSec(), 3);
      Serial.print(")");
      Serial.print(" send=");
      Serial.println(sent ? "ok" : "failed");
    }
  }

  delay(10);
}
