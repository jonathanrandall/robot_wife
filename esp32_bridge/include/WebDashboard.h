#ifndef WEBDASHBOARD_H
#define WEBDASHBOARD_H

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include "RobotController.h"
#include "PanTiltController.h"

// WiFi credentials (change these)
struct WiFiConfig {
    const char* ssid = "YOUR_WIFI_SSID";
    const char* password = "YOUR_WIFI_PASSWORD";
};

class WebDashboard {
public:
    WebDashboard(RobotController& robot, PanTiltController* panTilt = nullptr, uint16_t port = 80);

    // Initialize WiFi and web server
    bool begin(const char* ssid, const char* password);

    // Get connection status
    bool isConnected() const;
    IPAddress getIP() const;

    // Update telemetry (call periodically)
    void updateTelemetry();

private:
    void setupRoutes();
    String generateHTML();
    String generateTelemetryJSON();

    RobotController&   _robot;
    PanTiltController* _panTilt;
    AsyncWebServer     _server;
    bool               _connected;

    // Telemetry data
    float _actualSpeed;
    float _targetSpeed;
    int64_t _encoderCounts[4];
    float _motorCurrents[4];
};

// HTML page as a raw string
extern const char DASHBOARD_HTML[] PROGMEM;

#endif // WEBDASHBOARD_H
