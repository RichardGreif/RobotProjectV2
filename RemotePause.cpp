#include "RemotePause.h"
#include <Arduino.h>
#include <WebServer.h>
#include <WiFi.h>

namespace {
WebServer server(80);

const char PageHtml[] = R"rawliteral(
<!DOCTYPE html>
<html lang="de">
<head>
  <meta charset="utf-8">
  <title>Robot Remote Pause</title>
  <style>
    body {
      font-family: Arial, sans-serif;
      margin: 40px;
      line-height: 1.5;
    }

    .status {
      font-size: 2rem;
      font-weight: bold;
      margin: 20px 0;
    }

    .paused {
      color: #b00020;
    }

    .running {
      color: #0a7a2f;
    }

    .hint {
      margin-top: 24px;
      color: #555;
    }

    button {
      font-size: 1.2rem;
      padding: 12px 20px;
      margin-right: 12px;
      cursor: pointer;
    }
  </style>
</head>
<body>
  <h1>Robot Remote Pause</h1>
  <div id="status" class="status">Status wird geladen...</div>

  <button onclick="pauseRobot()">Pause (Leertaste)</button>
  <button onclick="resumeRobot()">Weiter (Enter)</button>

  <div class="hint">
    Leertaste = Pause<br>
    Enter = Weiter
  </div>

  <script>
    async function updateStatus() {
      const response = await fetch('/status');
      const text = await response.text();
      const status = document.getElementById('status');

      if (text === 'paused') {
        status.textContent = 'PAUSIERT';
        status.className = 'status paused';
      } else {
        status.textContent = 'AKTIV';
        status.className = 'status running';
      }
    }

    async function pauseRobot() {
      await fetch('/pause', { method: 'POST' });
      await updateStatus();
    }

    async function resumeRobot() {
      await fetch('/resume', { method: 'POST' });
      await updateStatus();
    }

    document.addEventListener('keydown', async function(event) {
      if (event.code === 'Space') {
        event.preventDefault();
        await pauseRobot();
      }

      if (event.code === 'Enter') {
        event.preventDefault();
        await resumeRobot();
      }
    });

    updateStatus();
    setInterval(updateStatus, 1000);
  </script>
</body>
</html>
)rawliteral";

}

RemotePause::RemotePause(const char* ssid, const char* password)
  : _ssid(ssid),
    _password(password),
    _paused(true),
    _connected(false) {
}

void RemotePause::begin() {

  WiFi.mode(WIFI_STA);
  WiFi.begin(_ssid, _password);

  Serial.print("IP Adresse: ");
  Serial.println(WiFi.localIP());

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  _connected = true;
  _paused = true;

  server.on("/", HTTP_GET, [this]() { handleRoot(); });
  server.on("/pause", HTTP_POST, [this]() { handlePause(); });
  server.on("/resume", HTTP_POST, [this]() { handleResume(); });
  server.on("/status", HTTP_GET, [this]() { handleStatus(); });

  server.begin();

  Serial.print("IP Adresse: http://");
  Serial.println(WiFi.localIP());
}

void RemotePause::update() {
  _connected = WiFi.status() == WL_CONNECTED;

  if (!_connected) {
    _paused = true;
    return;
  }

  server.handleClient();
}

bool RemotePause::isPaused() const {
  return _paused;
}

bool RemotePause::isConnected() const {
  return _connected;
}

void RemotePause::setPaused(bool paused) {
  _paused = paused;
}

void RemotePause::handleRoot() {
  server.send(200, "text/html; charset=utf-8", PageHtml);
}

void RemotePause::handlePause() {
  _paused = true;
  server.send(200, "text/plain", "paused");
}

void RemotePause::handleResume() {
  _paused = false;
  server.send(200, "text/plain", "running");
}

void RemotePause::handleStatus() {
  server.send(200, "text/plain", _paused ? "paused" : "running");
}