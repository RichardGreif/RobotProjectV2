#pragma once

#include <Arduino.h>
#include <WiFi.h>

#include "SLAM/MapSnapshot.h"

class SnapshotTcpClient
{
public:
  SnapshotTcpClient(
    const char* ssid,
    const char* password,
    IPAddress host,
    uint16_t port,
    float distanceScaleToViewerUnits,
    unsigned long reconnectIntervalMs);

  void begin();
  void update(unsigned long nowMs);

  bool sendSnapshot(const SLAM::MapSnapshot& snapshot);

  bool isWifiConnected() const;
  IPAddress getLocalIp() const;

private:
  bool ensureWifiConnected(unsigned long nowMs);
  bool connectWifi(unsigned long nowMs);
  void writePoint(WiFiClient& client, const Vec2& point) const;

  const char* ssid_;
  const char* password_;
  IPAddress host_;
  uint16_t port_;
  float distanceScaleToViewerUnits_;
  unsigned long reconnectIntervalMs_;
  unsigned long lastReconnectAttemptMs_ = 0;
};
