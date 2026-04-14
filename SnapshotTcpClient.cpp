#include "SnapshotTcpClient.h"

SnapshotTcpClient::SnapshotTcpClient(
  const char* ssid,
  const char* password,
  IPAddress host,
  uint16_t port,
  float distanceScaleToViewerUnits,
  unsigned long reconnectIntervalMs)
  : ssid_(ssid),
    password_(password),
    host_(host),
    port_(port),
    distanceScaleToViewerUnits_(distanceScaleToViewerUnits),
    reconnectIntervalMs_(reconnectIntervalMs) {
}

void SnapshotTcpClient::begin() {
  WiFi.mode(WIFI_STA);
  if (WiFi.status() != WL_CONNECTED) {
    connectWifi(millis());
  }
}

void SnapshotTcpClient::update(unsigned long nowMs) {
  ensureWifiConnected(nowMs);
}

bool SnapshotTcpClient::sendSnapshot(const SLAM::MapSnapshot& snapshot) {
  if (!ensureWifiConnected(millis())) {
    return false;
  }

  WiFiClient client;
  if (!client.connect(host_, port_)) {
    Serial.println("SnapshotTcpClient: TCP connect failed");
    return false;
  }

  client.print("{\"walls\":[");

  for (int i = 0; i < snapshot.wallCount; ++i) {
    const SLAM::WallSegment& wall = snapshot.walls[i];

    if (i > 0) {
      client.print(",");
    }

    client.print("{\"start\":");
    writePoint(client, wall.start);
    client.print(",\"end\":");
    writePoint(client, wall.end);
    client.print(",\"confidence\":");
    client.print(wall.confidence, 4);
    client.print("}");
  }

  client.print("]}\n");
  client.flush();
  client.stop();
  return true;
}

bool SnapshotTcpClient::isWifiConnected() const {
  return WiFi.status() == WL_CONNECTED;
}

IPAddress SnapshotTcpClient::getLocalIp() const {
  return WiFi.localIP();
}

bool SnapshotTcpClient::ensureWifiConnected(unsigned long nowMs) {
  if (WiFi.status() == WL_CONNECTED) {
    return true;
  }

  return connectWifi(nowMs);
}

bool SnapshotTcpClient::connectWifi(unsigned long nowMs) {
  if (lastReconnectAttemptMs_ != 0 && nowMs - lastReconnectAttemptMs_ < reconnectIntervalMs_) {
    return false;
  }

  lastReconnectAttemptMs_ = nowMs;

  if (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect(true, true);
    delay(100);
    WiFi.begin(ssid_, password_);
  }

  const unsigned long startMs = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startMs < 10000) {
    delay(250);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("SnapshotTcpClient: WiFi connected, IP=");
    Serial.println(WiFi.localIP());
    return true;
  }

  Serial.println("SnapshotTcpClient: WiFi connect timeout");
  return false;
}

void SnapshotTcpClient::writePoint(WiFiClient& client, const Vec2& point) const {
  client.print("{\"x\":");
  client.print(point.x * distanceScaleToViewerUnits_, 4);
  client.print(",\"y\":");
  client.print(point.y * distanceScaleToViewerUnits_, 4);
  client.print("}");
}
