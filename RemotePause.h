#pragma once
#include <Arduino.h>

class RemotePause {
public:
  RemotePause(const char* ssid, const char* password);

  void begin();
  void update();

  bool isPaused() const;
  bool isConnected() const;
  void setPaused(bool paused);

private:
  const char* _ssid;
  const char* _password;
  bool _paused;
  bool _connected;

  void handleRoot();
  void handlePause();
  void handleResume();
  void handleStatus();
};