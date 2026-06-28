#include "WebDashboard.h"
#include <ArduinoJson.h>

static constexpr int AUX_PIN = 42;
extern volatile bool g_auxPinHigh;

// HTML Dashboard page
const char DASHBOARD_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>Robot Dashboard</title>
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body {
            font-family: Arial, sans-serif;
            background: #1a1a2e;
            color: #eee;
            min-height: 100vh;
            padding: 20px;
        }
        .container { max-width: 800px; margin: 0 auto; }
        h1 { text-align: center; margin-bottom: 20px; color: #00d9ff; }

        .panel {
            background: #16213e;
            border-radius: 10px;
            padding: 20px;
            margin-bottom: 20px;
        }
        .panel h2 { color: #00d9ff; margin-bottom: 15px; font-size: 1.2em; }

        .controls {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 10px;
            max-width: 300px;
            margin: 0 auto;
        }
        .btn {
            background: #0f3460;
            border: 2px solid #00d9ff;
            color: #fff;
            padding: 20px;
            border-radius: 10px;
            font-size: 24px;
            cursor: pointer;
            transition: all 0.2s;
            touch-action: manipulation;
            user-select: none;
        }
        .btn:hover { background: #00d9ff; color: #1a1a2e; }
        .btn:active, .btn.active { background: #00ff88; border-color: #00ff88; color: #1a1a2e; }
        .btn.stop { background: #e94560; border-color: #e94560; }
        .btn.stop:hover { background: #ff6b6b; }
        .btn.fire-on { background: #00ff88; border-color: #00ff88; color: #1a1a2e; }

        .speed-control {
            display: flex;
            align-items: center;
            gap: 15px;
            flex-wrap: wrap;
        }
        .speed-slider {
            flex: 1;
            min-width: 200px;
            -webkit-appearance: none;
            height: 10px;
            border-radius: 5px;
            background: #0f3460;
            outline: none;
        }
        .speed-slider::-webkit-slider-thumb {
            -webkit-appearance: none;
            width: 25px;
            height: 25px;
            border-radius: 50%;
            background: #00d9ff;
            cursor: pointer;
        }
        .speed-value {
            font-size: 1.5em;
            color: #00d9ff;
            min-width: 100px;
            text-align: right;
        }

        .telemetry {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
            gap: 15px;
        }
        .stat {
            background: #0f3460;
            padding: 15px;
            border-radius: 8px;
            text-align: center;
        }
        .stat-label { font-size: 0.8em; color: #888; }
        .stat-value { font-size: 1.5em; color: #00d9ff; margin-top: 5px; }

        .motor-grid {
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 10px;
        }
        .motor-card {
            background: #0f3460;
            padding: 10px;
            border-radius: 8px;
        }
        .motor-name { font-weight: bold; color: #00d9ff; }
        .motor-stat { font-size: 0.9em; color: #aaa; }

        .enable-toggle {
            display: flex;
            align-items: center;
            gap: 10px;
            justify-content: center;
            margin-bottom: 15px;
        }
        .toggle {
            width: 60px;
            height: 30px;
            background: #e94560;
            border-radius: 15px;
            cursor: pointer;
            position: relative;
            transition: background 0.3s;
        }
        .toggle.on { background: #00ff88; }
        .toggle::after {
            content: '';
            position: absolute;
            width: 26px;
            height: 26px;
            background: #fff;
            border-radius: 50%;
            top: 2px;
            left: 2px;
            transition: left 0.3s;
        }
        .toggle.on::after { left: 32px; }

        .controls-row {
            display: flex;
            gap: 30px;
            flex-wrap: wrap;
            justify-content: center;
            align-items: flex-start;
        }
        .control-group {
            display: flex;
            flex-direction: column;
            align-items: center;
            gap: 10px;
        }
        .control-label {
            font-size: 0.9em;
            color: #888;
            text-transform: uppercase;
            letter-spacing: 0.05em;
        }
        .pt-readout {
            font-size: 0.85em;
            color: #aaa;
            text-align: center;
        }

        .status { text-align: center; padding: 10px; }
        .status.connected { color: #00ff88; }
        .status.disconnected { color: #e94560; }
    </style>
</head>
<body>
    <div class="container">
        <h1>Robot Dashboard</h1>

        <div class="panel">
            <div class="enable-toggle">
                <span>Motors:</span>
                <div id="enableToggle" class="toggle" onclick="toggleEnable()"></div>
                <span id="enableLabel">DISABLED</span>
            </div>
        </div>

        <div class="panel">
            <h2>Controls</h2>
            <div class="controls-row">
                <div class="control-group">
                    <div class="control-label">Drive</div>
                    <div class="controls">
                        <div></div>
                        <button class="btn" id="btnFwd" data-cmd="fwd">&#9650;</button>
                        <div></div>
                        <button class="btn" id="btnLeft" data-cmd="left">&#9664;</button>
                        <button class="btn stop" id="btnStop" data-cmd="stop">&#9632;</button>
                        <button class="btn" id="btnRight" data-cmd="right">&#9654;</button>
                        <div></div>
                        <button class="btn" id="btnBack" data-cmd="back">&#9660;</button>
                        <div></div>
                    </div>
                </div>
                <div class="control-group">
                    <div class="control-label">Pan / Tilt</div>
                    <div class="controls">
                        <div></div>
                        <button class="btn" id="btnTiltUp"   data-pt="tiltup">&#9650;</button>
                        <div></div>
                        <button class="btn" id="btnPanLeft"  data-pt="panleft">&#9664;</button>
                        <button class="btn stop" id="btnFire">FIRE</button>
                        <button class="btn" id="btnPanRight" data-pt="panright">&#9654;</button>
                        <div></div>
                        <button class="btn" id="btnTiltDown" data-pt="tiltdown">&#9660;</button>
                        <div></div>
                    </div>
                    <div class="pt-readout">
                        Pan: <span id="panVal">0.00</span> rad &nbsp;|&nbsp; Tilt: <span id="tiltVal">0.00</span> rad
                    </div>
                </div>
            </div>
        </div>

        <div class="panel">
            <h2>Speed (m/s)</h2>
            <div class="speed-control">
                <input type="range" id="speedSlider" class="speed-slider" min="0" max="100" value="50" oninput="updateSpeed()">
                <div class="speed-value"><span id="speedValue">0.50</span> m/s</div>
            </div>
        </div>

        <div class="panel">
            <h2>Telemetry</h2>
            <div class="telemetry">
                <div class="stat">
                    <div class="stat-label">Target Speed</div>
                    <div class="stat-value" id="targetSpeed">0.00</div>
                </div>
                <div class="stat">
                    <div class="stat-label">Actual Speed</div>
                    <div class="stat-value" id="actualSpeed">0.00</div>
                </div>
                <div class="stat">
                    <div class="stat-label">Direction</div>
                    <div class="stat-value" id="direction">STOP</div>
                </div>
            </div>
        </div>

        <div class="panel">
            <h2>Motors</h2>
            <div class="motor-grid">
                <div class="motor-card">
                    <div class="motor-name">Rear Right (M1)</div>
                    <div class="motor-stat">Encoder: <span id="enc0">0</span></div>
                    <div class="motor-stat">Current: <span id="cur0">0.00</span> A</div>
                </div>
                <div class="motor-card">
                    <div class="motor-name">Rear Left (M2)</div>
                    <div class="motor-stat">Encoder: <span id="enc1">0</span></div>
                    <div class="motor-stat">Current: <span id="cur1">0.00</span> A</div>
                </div>
                <div class="motor-card">
                    <div class="motor-name">Front Right (M4)</div>
                    <div class="motor-stat">Encoder: <span id="enc3">0</span></div>
                    <div class="motor-stat">Current: <span id="cur3">0.00</span> A</div>
                </div>
                <div class="motor-card">
                    <div class="motor-name">Front Left (M3)</div>
                    <div class="motor-stat">Encoder: <span id="enc2">0</span></div>
                    <div class="motor-stat">Current: <span id="cur2">0.00</span> A</div>
                </div>
            </div>
        </div>

        <div class="panel">
            <h2>PWM</h2>
            <div class="motor-grid">
                <div class="motor-card">
                    <div class="motor-name">Rear Right (M1)</div>
                    <div class="motor-stat">Duty: <span id="duty0">0.00</span></div>
                    <div class="motor-stat">PWM: <span id="pwm0">0</span></div>
                </div>
                <div class="motor-card">
                    <div class="motor-name">Rear Left (M2)</div>
                    <div class="motor-stat">Duty: <span id="duty1">0.00</span></div>
                    <div class="motor-stat">PWM: <span id="pwm1">0</span></div>
                </div>
                <div class="motor-card">
                    <div class="motor-name">Front Right (M4)</div>
                    <div class="motor-stat">Duty: <span id="duty3">0.00</span></div>
                    <div class="motor-stat">PWM: <span id="pwm3">0</span></div>
                </div>
                <div class="motor-card">
                    <div class="motor-name">Front Left (M3)</div>
                    <div class="motor-stat">Duty: <span id="duty2">0.00</span></div>
                    <div class="motor-stat">PWM: <span id="pwm2">0</span></div>
                </div>
            </div>
        </div>

        <div class="status" id="status">Connecting...</div>
    </div>

    <script>
        let enabled = false;
        let speed = 0.5;
        let currentDir = 'stop';
        let lastCmd = 'stop';
        let lastCmdTime = 0;
        let isPressed = false;

        function toggleEnable() {
            enabled = !enabled;
            fetch('/api/enable?state=' + (enabled ? '1' : '0'));
            document.getElementById('enableToggle').classList.toggle('on', enabled);
            document.getElementById('enableLabel').textContent = enabled ? 'ENABLED' : 'DISABLED';
        }

        function updateSpeed() {
            const slider = document.getElementById('speedSlider');
            speed = (slider.value / 100 ).toFixed(2);
            document.getElementById('speedValue').textContent = speed;
            fetch('/api/speed?value=' + speed);
        }

        function sendCmd(cmd) {
            // Debounce: ignore duplicate commands within 50ms
            const now = Date.now();
            if (cmd === lastCmd && (now - lastCmdTime) < 50) return;
            lastCmd = cmd;
            lastCmdTime = now;
            currentDir = cmd;

            fetch('/api/cmd?dir=' + cmd);

            // Visual feedback
            ['Fwd', 'Back', 'Left', 'Right'].forEach(d => {
                document.getElementById('btn' + d).classList.remove('active');
            });
            if (cmd !== 'stop') {
                const btnMap = {fwd: 'Fwd', back: 'Back', left: 'Left', right: 'Right'};
                if (btnMap[cmd]) {
                    document.getElementById('btn' + btnMap[cmd]).classList.add('active');
                }
            }
        }

        function handleButtonPress(e) {
            e.preventDefault();
            const cmd = e.currentTarget.dataset.cmd;
            if (cmd === 'stop') {
                sendCmd('stop');
            } else {
                isPressed = true;
                sendCmd(cmd);
            }
        }

        function handleButtonRelease(e) {
            e.preventDefault();
            if (isPressed) {
                isPressed = false;
                sendCmd('stop');
            }
        }

        // Setup button event listeners
        document.querySelectorAll('.btn[data-cmd]').forEach(btn => {
            const cmd = btn.dataset.cmd;
            if (cmd === 'stop') {
                btn.addEventListener('click', handleButtonPress);
                btn.addEventListener('touchstart', handleButtonPress, {passive: false});
            } else {
                // Press events
                btn.addEventListener('mousedown', handleButtonPress);
                btn.addEventListener('touchstart', handleButtonPress, {passive: false});
                // Release events
                btn.addEventListener('mouseup', handleButtonRelease);
                btn.addEventListener('mouseleave', handleButtonRelease);
                btn.addEventListener('touchend', handleButtonRelease, {passive: false});
                btn.addEventListener('touchcancel', handleButtonRelease, {passive: false});
            }
        });

        // Global release handler (in case mouse released outside button)
        document.addEventListener('mouseup', () => {
            if (isPressed) {
                isPressed = false;
                sendCmd('stop');
            }
        });

        function updateTelemetry() {
            fetch('/api/telemetry')
                .then(r => r.json())
                .then(data => {
                    document.getElementById('targetSpeed').textContent = data.targetSpeed.toFixed(2);
                    document.getElementById('actualSpeed').textContent = data.actualSpeed.toFixed(2);
                    document.getElementById('direction').textContent = data.direction.toUpperCase();

                    for (let i = 0; i < 4; i++) {
                        document.getElementById('enc' + i).textContent = data.encoders[i];
                        document.getElementById('cur' + i).textContent = data.currents[i].toFixed(2);
                        document.getElementById('duty' + i).textContent = data.duties[i].toFixed(2);
                        document.getElementById('pwm' + i).textContent = data.pwms[i];
                    }

                    document.getElementById('btnFire').classList.toggle('fire-on', !!data.auxPinHigh);

                    document.getElementById('status').textContent = 'Connected';
                    document.getElementById('status').className = 'status connected';
                })
                .catch(() => {
                    document.getElementById('status').textContent = 'Disconnected';
                    document.getElementById('status').className = 'status disconnected';
                });
        }

        // Keyboard controls
        const keyHeld = {};
        document.addEventListener('keydown', (e) => {
            if (e.repeat || keyHeld[e.key]) return;
            keyHeld[e.key] = true;
            switch(e.key) {
                case 'ArrowUp': case 'w': case 'W': sendCmd('fwd'); break;
                case 'ArrowDown': case 's': case 'S': sendCmd('back'); break;
                case 'ArrowLeft': case 'a': case 'A': sendCmd('left'); break;
                case 'ArrowRight': case 'd': case 'D': sendCmd('right'); break;
                case ' ': sendCmd('stop'); break;
            }
        });

        document.addEventListener('keyup', (e) => {
            keyHeld[e.key] = false;
            if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', 'w', 'W', 'a', 'A', 's', 'S', 'd', 'D'].includes(e.key)) {
                sendCmd('stop');
            }
        });

        // ---- Pan/Tilt ----
        let panPos  = 0.0;
        let tiltPos = 0.0;
        const PT_STEP = 0.10;   // radians per nudge
        const PAN_MIN  = -1.57, PAN_MAX  = 1.57;
        const TILT_MIN = -0.79, TILT_MAX = 1.57;
        let ptHoldInterval = null;

        function clamp(v, lo, hi) { return Math.max(lo, Math.min(hi, v)); }

        function sendPanTilt(pan, tilt) {
            panPos  = clamp(pan,  PAN_MIN,  PAN_MAX);
            tiltPos = clamp(tilt, TILT_MIN, TILT_MAX);
            document.getElementById('panVal').textContent  = panPos.toFixed(2);
            document.getElementById('tiltVal').textContent = tiltPos.toFixed(2);
            fetch('/api/pantilt?pan=' + panPos.toFixed(3) + '&tilt=' + tiltPos.toFixed(3));
        }

        function applyPTNudge(action) {
            switch (action) {
                case 'panleft':   sendPanTilt(panPos  - PT_STEP, tiltPos); break;
                case 'panright':  sendPanTilt(panPos  + PT_STEP, tiltPos); break;
                case 'tiltup':    sendPanTilt(panPos, tiltPos + PT_STEP);  break;
                case 'tiltdown':  sendPanTilt(panPos, tiltPos - PT_STEP);  break;
                case 'center':    sendPanTilt(0, 0);                       break;
            }
        }

        function ptPress(e) {
            e.preventDefault();
            const action = e.currentTarget.dataset.pt;
            applyPTNudge(action);
            if (action !== 'center') {
                ptHoldInterval = setInterval(() => applyPTNudge(action), 150);
            }
        }
        function ptRelease(e) {
            e.preventDefault();
            clearInterval(ptHoldInterval);
            ptHoldInterval = null;
        }

        document.querySelectorAll('.btn[data-pt]').forEach(btn => {
            btn.addEventListener('mousedown',   ptPress,   {passive: false});
            btn.addEventListener('touchstart',  ptPress,   {passive: false});
            btn.addEventListener('mouseup',     ptRelease, {passive: false});
            btn.addEventListener('mouseleave',  ptRelease, {passive: false});
            btn.addEventListener('touchend',    ptRelease, {passive: false});
            btn.addEventListener('touchcancel', ptRelease, {passive: false});
        });
        document.addEventListener('mouseup', () => {
            clearInterval(ptHoldInterval);
            ptHoldInterval = null;
        });

        document.getElementById('btnFire').addEventListener('click', () => {
            fetch('/api/fire');
        });

        // Initialize
        updateSpeed();
        setInterval(updateTelemetry, 200);
    </script>
</body>
</html>
)rawliteral";

WebDashboard::WebDashboard(RobotController& robot, PanTiltController* panTilt, uint16_t port)
    : _robot(robot)
    , _panTilt(panTilt)
    , _server(port)
    , _connected(false)
    , _actualSpeed(0)
    , _targetSpeed(0)
{
    for (int i = 0; i < 4; i++) {
        _encoderCounts[i] = 0;
        _motorCurrents[i] = 0;
    }
}

bool WebDashboard::begin(const char* ssid, const char* password) {
    Serial.print("Connecting to WiFi: ");
    Serial.println(ssid);

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\nWiFi connection failed!");
        _connected = false;
        return false;
    }

    Serial.println("\nWiFi connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    _connected = true;
    setupRoutes();
    _server.begin();

    Serial.println("Web server started");
    return true;
}

void WebDashboard::setupRoutes() {
    // Serve main page
    _server.on("/", HTTP_GET, [this](AsyncWebServerRequest* request) {
        request->send(200, "text/html", DASHBOARD_HTML);
    });

    // API: Enable/disable motors
    _server.on("/api/enable", HTTP_GET, [this](AsyncWebServerRequest* request) {
        if (request->hasParam("state")) {
            bool enable = request->getParam("state")->value() == "1";
            _robot.enable(enable);
        }
        request->send(200, "text/plain", "OK");
    });

    // API: Set speed
    _server.on("/api/speed", HTTP_GET, [this](AsyncWebServerRequest* request) {
        if (request->hasParam("value")) {
            float speed = request->getParam("value")->value().toFloat();
            _targetSpeed = speed;
        }
        request->send(200, "text/plain", "OK");
    });

    // API: Direction command
    _server.on("/api/cmd", HTTP_GET, [this](AsyncWebServerRequest* request) {
        if (request->hasParam("dir")) {
            String dir = request->getParam("dir")->value();
            if (dir == "fwd") {
                _robot.forward(_targetSpeed);
            } else if (dir == "back") {
                _robot.backward(_targetSpeed);
            } else if (dir == "left") {
                _robot.turnLeft(_targetSpeed);
            } else if (dir == "right") {
                _robot.turnRight(_targetSpeed);
            } else if (dir == "stop") {
                _robot.stop();
            }
        }
        request->send(200, "text/plain", "OK");
    });

    // API: Pan/tilt position
    _server.on("/api/pantilt", HTTP_GET, [this](AsyncWebServerRequest* request) {
        if (_panTilt) {
            float pan  = request->hasParam("pan")  ? request->getParam("pan")->value().toFloat()  : _panTilt->getPan();
            float tilt = request->hasParam("tilt") ? request->getParam("tilt")->value().toFloat() : _panTilt->getTilt();
            _panTilt->setPan(pan);
            _panTilt->setTilt(tilt);
        }
        request->send(200, "text/plain", "OK");
    });

    // API: Fire aux output (pin 35 high for 2 s)
    _server.on("/api/fire", HTTP_GET, [](AsyncWebServerRequest* request) {
        if (!g_auxPinHigh) {
            g_auxPinHigh = true;
            digitalWrite(AUX_PIN, HIGH);
            TimerHandle_t t = xTimerCreate("auxOff", pdMS_TO_TICKS(600), pdFALSE, nullptr,
                [](TimerHandle_t xTimer) {
                    digitalWrite(AUX_PIN, LOW);
                    g_auxPinHigh = false;
                    xTimerDelete(xTimer, 0);
                });
            if (t) xTimerStart(t, 0);
        }
        request->send(200, "text/plain", "OK");
    });

    // API: Get telemetry
    _server.on("/api/telemetry", HTTP_GET, [this](AsyncWebServerRequest* request) {
        request->send(200, "application/json", generateTelemetryJSON());
    });
}

String WebDashboard::generateTelemetryJSON() {
    JsonDocument doc;

    doc["targetSpeed"] = _targetSpeed;
    doc["actualSpeed"] = _robot.getActualLinearSpeed();
    doc["auxPinHigh"]  = (bool)g_auxPinHigh;

    // Determine direction from command
    const RobotCommand& cmd = _robot.getCommand();
    String dir = "stop";
    if (cmd.linearSpeed > 0.01f) dir = "fwd";
    else if (cmd.linearSpeed < -0.01f) dir = "back";
    else if (cmd.angularSpeed > 0.01f) dir = "left";
    else if (cmd.angularSpeed < -0.01f) dir = "right";
    doc["direction"] = dir;

    JsonArray encoders = doc["encoders"].to<JsonArray>();
    JsonArray currents = doc["currents"].to<JsonArray>();
    JsonArray pwms     = doc["pwms"].to<JsonArray>();
    JsonArray duties   = doc["duties"].to<JsonArray>();

    for (int i = 0; i < 4; i++) {
        encoders.add(_robot.getEncoderCount(i));
        currents.add(_robot.getMotorCurrent(i));
        pwms.add(_robot.getMotorPWM(i));
        duties.add(_robot.getMotorDuty(i));
    }

    String output;
    serializeJson(doc, output);
    return output;
}

bool WebDashboard::isConnected() const {
    return _connected && WiFi.status() == WL_CONNECTED;
}

IPAddress WebDashboard::getIP() const {
    return WiFi.localIP();
}

void WebDashboard::updateTelemetry() {
    // This would be called from the motor task to update telemetry data
    // For thread safety, we just store the values here
}
