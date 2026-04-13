#include "WheelOdometry.h"

#include <cmath>

WheelOdometry::WheelOdometry(
  float wheelBaseCm,
  float leftWheelSpeedCmPerSecPerHz,
  float rightWheelSpeedCmPerSecPerHz,
  float leftDirectionSign,
  float rightDirectionSign)
  : wheelDistanceCm_(wheelBaseCm),
    leftWheelSpeedCmPerSecPerHz_(leftWheelSpeedCmPerSecPerHz),
    rightWheelSpeedCmPerSecPerHz_(rightWheelSpeedCmPerSecPerHz),
    leftDirectionSign_(leftDirectionSign),
    rightDirectionSign_(rightDirectionSign) {
}

void WheelOdometry::Reset(const Pose2D& pose, unsigned long nowMs) {
  pose_ = pose;
  lastUpdateMs_ = nowMs;
  initialized_ = true;
  leftLinearSpeedCmPerSec_ = 0.0f;
  rightLinearSpeedCmPerSec_ = 0.0f;
}

void WheelOdometry::Update(unsigned long nowMs, float leftSpeedHz, float rightSpeedHz) {
  leftLinearSpeedCmPerSec_ = leftSpeedHz * leftWheelSpeedCmPerSecPerHz_ * leftDirectionSign_;
  rightLinearSpeedCmPerSec_ = rightSpeedHz * rightWheelSpeedCmPerSecPerHz_ * rightDirectionSign_;

  if (!initialized_) {
    lastUpdateMs_ = nowMs;
    initialized_ = true;
    return;
  }

  const unsigned long deltaMs = nowMs - lastUpdateMs_;
  lastUpdateMs_ = nowMs;

  if (deltaMs == 0) {
    return;
  }

  const float dtSec = static_cast<float>(deltaMs) * 0.001f;
  const float linearVelocityCmPerSec = 0.5f * (leftLinearSpeedCmPerSec_ + rightLinearSpeedCmPerSec_);

  float angularVelocityRadPerSec = 0.0f;
  if (std::abs(wheelDistanceCm_) > 1e-6f) {
    angularVelocityRadPerSec = (rightLinearSpeedCmPerSec_ - leftLinearSpeedCmPerSec_) / wheelDistanceCm_;
  }

  const float deltaYaw = angularVelocityRadPerSec * dtSec;
  const float midYaw = pose_.yaw + 0.5f * deltaYaw; // estimated mean yaw during the travel
  const float distanceCm = linearVelocityCmPerSec * dtSec;

  pose_.position.x += std::cos(midYaw) * distanceCm;
  pose_.position.y += std::sin(midYaw) * distanceCm;
  pose_.yaw += deltaYaw;
}

const Pose2D& WheelOdometry::GetPose() const {
  return pose_;
}

float WheelOdometry::GetLeftLinearSpeedCmPerSec() const {
  return leftLinearSpeedCmPerSec_;
}

float WheelOdometry::GetRightLinearSpeedCmPerSec() const {
  return rightLinearSpeedCmPerSec_;
}
