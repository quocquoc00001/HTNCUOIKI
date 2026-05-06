#include <WiFi.h>
#include <WebServer.h>
#include <DHT.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>


// ==================== HARDWARE PIN DEFINITIONS ====================
#define RELAY_LIGHT     5
#define RELAY_FAN       6
#define BTN_LIGHT       17
#define BTN_FAN         18
#define DHT_PIN         4
#define DHT_TYPE        DHT11
#define I2C_SDA         8
#define I2C_SCL         9

// ==================== RELAY LOGIC ====================
const bool RELAY_ACTIVE_LOW = true;
const uint8_t RELAY_ON_LEVEL  = RELAY_ACTIVE_LOW ? LOW  : HIGH;
const uint8_t RELAY_OFF_LEVEL = RELAY_ACTIVE_LOW ? HIGH : LOW;

// ==================== WIFI CONFIGURATION ====================
const char* AP_SSID     = "ESP32-S3-SmartHome";
const char* AP_PASSWORD = "12345678";
const char* STA_SSID    = "";
const char* STA_PASSWORD = "";

// ==================== FREERTOS OBJECT DECLARATIONS ====================
// --- Tasks ---
TaskHandle_t hTaskSensor    = NULL;
TaskHandle_t hTaskLCD      = NULL;
TaskHandle_t hTaskWiFiMgr  = NULL;

// --- Timers ---
TimerHandle_t tmrDHT;
TimerHandle_t tmrLCD;
TimerHandle_t tmrWiFi;

// --- Mutex ---
SemaphoreHandle_t mtxSensor;
SemaphoreHandle_t mtxRelay;
SemaphoreHandle_t mtxWiFi;

// --- Queue ---
struct RelayCmd_t {
  uint8_t dev;   // 0=light, 1=fan
  bool toggle;   // true = đảo trạng thái
};
struct WiFiCmd_t {
  uint8_t type;  // 0=scan, 1=connect, 2=disconnect
  char ssid[32];
  char pass[64];
};
QueueHandle_t qRelayCmd;
QueueHandle_t qWiFiCmd;

// --- Semaphore ---
SemaphoreHandle_t semButtons;    // Binary: 2 ISR give, taskButton take
SemaphoreHandle_t semWiFiReady;  // Binary: WiFiMgr give khi STA ready

// --- ISR flags ---
static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t isrFlagLight = 0;
volatile uint32_t isrFlagFan   = 0;

// ==================== GLOBAL SHARED DATA (protected by Mutex) ====================
struct SensorData_t {
  float temp = 0.0;
  float hum  = 0.0;
  bool alert = false;
  float tempThreshold = 35.0;
  float humThreshold  = 80.0;
} g_sensor;

struct RelayState_t {
  bool light = false;
  bool fan   = false;
} g_relay;

struct WiFiState_t {
  bool staConnecting = false;
  unsigned long wifiConnectStart = 0;
  String currentStaSSID = "";
} g_wifi;

// ==================== PERIPHERAL OBJECTS ====================
LiquidCrystal_I2C lcd(0x27, 16, 2);
DHT dht(DHT_PIN, DHT_TYPE);
WebServer server(80);

// ==================== HTML PAGE ====================
const char* HTML_PAGE = R"rawliteral(
<!DOCTYPE html>
<html lang="vi">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>ESP32-S3 Smart Home</title>
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; }
    body { background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); min-height: 100vh; padding: 20px; color: #333; }
    .container { max-width: 480px; margin: 0 auto; }
    h1 { text-align: center; color: #fff; margin-bottom: 20px; font-size: 1.5rem; text-shadow: 0 2px 4px rgba(0,0,0,0.2); }
    .card { background: #fff; border-radius: 16px; padding: 20px; margin-bottom: 16px; box-shadow: 0 8px 32px rgba(0,0,0,0.15); }
    .card-title { font-size: 0.85rem; text-transform: uppercase; letter-spacing: 1px; color: #888; margin-bottom: 12px; font-weight: 600; }
    .sensor-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
    .sensor-box { text-align: center; padding: 16px; border-radius: 12px; background: #f8f9fa; }
    .sensor-value { font-size: 2rem; font-weight: 700; color: #333; }
    .sensor-unit { font-size: 0.9rem; color: #888; }
    .sensor-label { font-size: 0.8rem; color: #aaa; margin-top: 4px; }
    .alert-box { background: #fff3cd; border-left: 4px solid #ffc107; padding: 12px; border-radius: 8px; margin-top: 12px; display: none; }
    .alert-box.active { display: block; }
    .relay-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 16px; }
    .relay-btn { border: none; border-radius: 12px; padding: 20px; font-size: 1.1rem; font-weight: 600; cursor: pointer; transition: all 0.3s ease; color: #fff; }
    .relay-btn:hover { transform: translateY(-2px); box-shadow: 0 6px 20px rgba(0,0,0,0.2); }
    .relay-btn:active { transform: translateY(0); }
    .relay-on { background: linear-gradient(135deg, #11998e, #38ef7d); }
    .relay-off { background: linear-gradient(135deg, #eb3349, #f45c43); }
    .status-dot { display: inline-block; width: 10px; height: 10px; border-radius: 50%; margin-right: 8px; }
    .status-on { background: #00e676; box-shadow: 0 0 8px #00e676; }
    .status-off { background: #ff1744; box-shadow: 0 0 8px #ff1744; }
    .info { text-align: center; color: rgba(255,255,255,0.8); font-size: 0.8rem; margin-top: 20px; }
    .threshold-input, .wifi-select {
      width: 100%; padding: 8px; border: 1px solid #ddd; border-radius: 8px;
      margin-top: 8px; font-size: 1rem; background: #fff;
    }
    .save-btn {
      background: #667eea; color: #fff; border: none; padding: 10px 20px;
      border-radius: 8px; margin-top: 10px; cursor: pointer; font-weight: 600;
      width: 100%;
    }
    .save-btn:hover { background: #5a67d8; }
    .wifi-row { display: flex; gap: 8px; }
    .wifi-row .save-btn { width: 50%; }
    .small-text { font-size: 0.85rem; color: #666; margin-top: 8px; line-height: 1.4; }
  </style>
</head>
<body>
  <div class="container">
    <h1>🏠 Smart Home Control</h1>
    <div class="card">
      <div class="card-title">📊 Cảm biến môi trường</div>
      <div class="sensor-grid">
        <div class="sensor-box">
          <div class="sensor-value" id="temp">--</div>
          <div class="sensor-unit">°C</div>
          <div class="sensor-label">Nhiệt độ</div>
        </div>
        <div class="sensor-box">
          <div class="sensor-value" id="hum">--</div>
          <div class="sensor-unit">%</div>
          <div class="sensor-label">Độ ẩm</div>
        </div>
      </div>
      <div class="alert-box" id="alertBox">
        <strong>⚠️ Cảnh báo:</strong> <span id="alertMsg">Nhiệt độ / Độ ẩm vượt ngưỡng!</span>
      </div>
    </div>
    <div class="card">
      <div class="card-title">🔌 Điều khiển thiết bị</div>
      <div class="relay-grid">
        <button class="relay-btn relay-off" id="btnLight" onclick="toggleRelay('light')">
          <span class="status-dot status-off" id="dotLight"></span>Đèn
        </button>
        <button class="relay-btn relay-off" id="btnFan" onclick="toggleRelay('fan')">
          <span class="status-dot status-off" id="dotFan"></span>Quạt
        </button>
      </div>
    </div>
    <div class="card">
      <div class="card-title">⚙️ Cài đặt ngưỡng cảnh báo</div>
      <label>Nhiệt độ tối đa (°C):</label>
      <input type="number" class="threshold-input" id="tempThreshold" value="35" step="0.5">
      <label style="display:block; margin-top:10px;">Độ ẩm tối đa (%):</label>
      <input type="number" class="threshold-input" id="humThreshold" value="80" step="0.5">
      <button class="save-btn" onclick="saveThreshold()">💾 Lưu cài đặt</button>
    </div>
    <div class="card">
      <div class="card-title">📶 Kết nối WiFi nhà</div>
      <div class="wifi-row">
        <button class="save-btn" onclick="scanWifi()">🔍 Quét WiFi</button>
        <button class="save-btn" onclick="refreshWifiStatus()">↻ Trạng thái</button>
      </div>
      <select class="wifi-select" id="wifiList" onchange="document.getElementById('wifiSsid').value = this.value">
        <option value="">-- Chọn mạng WiFi --</option>
      </select>
      <input type="text" class="threshold-input" id="wifiSsid" placeholder="Hoặc nhập SSID của bạn">
      <input type="password" class="threshold-input" id="wifiPass" placeholder="Nhập mật khẩu WiFi">
      <button class="save-btn" onclick="connectWifi()">🔌 Kết nối WiFi</button>
      <div class="small-text" id="wifiStatus">Đang tải trạng thái WiFi...</div>
    </div>
    <div class="info">
      ESP32-S3 N16R8 | AP+STA Dual Mode | FreeRTOS<br>
      IP AP: 192.168.4.1
    </div>
  </div>
  <script>
    function updateData() {
      fetch('/api/status')
        .then(r => r.json())
        .then(data => {
          document.getElementById('temp').innerText = data.temp.toFixed(1);
          document.getElementById('hum').innerText = data.hum.toFixed(1);
          document.getElementById('tempThreshold').value = data.tempThreshold;
          document.getElementById('humThreshold').value = data.humThreshold;
          updateRelayBtn('btnLight', 'dotLight', data.light);
          updateRelayBtn('btnFan', 'dotFan', data.fan);
          const alertBox = document.getElementById('alertBox');
          if (data.alert) {
            alertBox.classList.add('active');
            document.getElementById('alertMsg').innerText = data.alertMsg;
          } else {
            alertBox.classList.remove('active');
          }
        })
        .catch(() => {});
    }
    function updateRelayBtn(btnId, dotId, state) {
      const btn = document.getElementById(btnId);
      const dot = document.getElementById(dotId);
      if (state) {
        btn.classList.remove('relay-off'); btn.classList.add('relay-on');
        dot.classList.remove('status-off'); dot.classList.add('status-on');
      } else {
        btn.classList.remove('relay-on'); btn.classList.add('relay-off');
        dot.classList.remove('status-on'); dot.classList.add('status-off');
      }
    }
    function toggleRelay(device) {
      fetch('/api/toggle?device=' + device)
        .then(() => updateData());
    }
    function saveThreshold() {
      const t = document.getElementById('tempThreshold').value;
      const h = document.getElementById('humThreshold').value;
      fetch('/api/threshold?temp=' + encodeURIComponent(t) + '&hum=' + encodeURIComponent(h))
        .then(() => alert('Đã lưu ngưỡng cảnh báo!'));
    }
    function scanWifi() {
      document.getElementById('wifiStatus').innerText = 'Đang quét WiFi...';
      fetch('/api/wifi/scan')
        .then(r => r.json())
        .then(list => {
          const sel = document.getElementById('wifiList');
          sel.innerHTML = '<option value="">-- Chọn mạng WiFi --</option>';
          list.forEach(item => {
            const opt = document.createElement('option');
            opt.value = item.ssid;
            opt.textContent = item.ssid + ' (' + item.rssi + ' dBm' + (item.enc ? ', khóa' : ', mở') + ')';
            sel.appendChild(opt);
          });
          document.getElementById('wifiStatus').innerText = 'Đã quét xong, hãy chọn WiFi của bạn.';
        })
        .catch(() => { document.getElementById('wifiStatus').innerText = 'Quét WiFi thất bại.'; });
    }
    function connectWifi() {
      const ssid = document.getElementById('wifiSsid').value.trim();
      const pass = document.getElementById('wifiPass').value;
      if (!ssid) { alert('Vui lòng chọn hoặc nhập SSID WiFi.'); return; }
      document.getElementById('wifiStatus').innerText = 'Đang gửi yêu cầu kết nối...';
      fetch('/api/wifi/connect?ssid=' + encodeURIComponent(ssid) + '&pass=' + encodeURIComponent(pass))
        .then(r => r.json())
        .then(data => {
          alert(data.message || 'Đã gửi yêu cầu kết nối WiFi.');
          refreshWifiStatus();
        })
        .catch(() => { document.getElementById('wifiStatus').innerText = 'Kết nối thất bại.'; });
    }
    function refreshWifiStatus() {
      fetch('/api/wifi/status')
        .then(r => r.json())
        .then(d => {
          let text = 'STA: ';
          if (d.connected) text += 'Đã kết nối';
          else if (d.connecting) text += 'Đang kết nối';
          else text += 'Chưa kết nối';
          text += ' | SSID: ' + (d.ssid || '-');
          text += ' | IP: ' + (d.ip || '-');
          document.getElementById('wifiStatus').innerText = text;
        })
        .catch(() => {});
    }
    setInterval(updateData, 2000);
    setInterval(refreshWifiStatus, 5000);
    updateData();
    refreshWifiStatus();
  </script>
</body>
</html>
)rawliteral";

// ==================== HELPER ====================
String jsonEscape(const String &s) {
  String out;
  out.reserve(s.length() + 10);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    switch (c) {
      case '\\': out += "\\\\"; break;
      case '\"': out += "\\\""; break;
      case '\n': out += "\\n"; break;
      case '\r': break;
      default: out += c; break;
    }
  }
  return out;
}

// ==================== ISR (2 Interrupts) ====================
void IRAM_ATTR isrBtnLight() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  portENTER_CRITICAL_ISR(&spinlock);
  if (!isrFlagLight) {
    isrFlagLight = 1;
    xSemaphoreGiveFromISR(semButtons, &xHigherPriorityTaskWoken);
  }
  portEXIT_CRITICAL_ISR(&spinlock);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void IRAM_ATTR isrBtnFan() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  portENTER_CRITICAL_ISR(&spinlock);
  if (!isrFlagFan) {
    isrFlagFan = 1;
    xSemaphoreGiveFromISR(semButtons, &xHigherPriorityTaskWoken);
  }
  portEXIT_CRITICAL_ISR(&spinlock);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// ==================== TIMER CALLBACKS (3 Timers) ====================
void cbTimerDHT(TimerHandle_t xTimer) {
  if (hTaskSensor) xTaskNotifyGive(hTaskSensor);
}
void cbTimerLCD(TimerHandle_t xTimer) {
  if (hTaskLCD) xTaskNotifyGive(hTaskLCD);
}
void cbTimerWiFi(TimerHandle_t xTimer) {
  if (hTaskWiFiMgr) xTaskNotifyGive(hTaskWiFiMgr);
}

// ==================== TASK 1: SENSOR ====================
void taskSensor(void* pvParam) {
  (void)pvParam;
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // chờ tmrDHT notify

    float t = dht.readTemperature();
    float h = dht.readHumidity();

    xSemaphoreTake(mtxSensor, portMAX_DELAY);
    if (!isnan(t) && !isnan(h)) {
      g_sensor.temp = t;
      g_sensor.hum  = h;
      g_sensor.alert = (t > g_sensor.tempThreshold || h > g_sensor.humThreshold);
      Serial.printf("[taskSensor] Temp: %.1f C | Hum: %.1f %% | Alert: %s\n",
                    g_sensor.temp, g_sensor.hum, g_sensor.alert ? "YES" : "NO");
    } else {
      Serial.println("[taskSensor] DHT read failed!");
    }
    xSemaphoreGive(mtxSensor);
  }
}

// ==================== TASK 2: LCD ====================
void taskLCD(void* pvParam) {
  (void)pvParam;
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // chờ tmrLCD notify (hoặc notify từ taskRelayCtrl)

    xSemaphoreTake(mtxSensor, portMAX_DELAY);
    float t = g_sensor.temp;
    float h = g_sensor.hum;
    bool alert = g_sensor.alert;
    xSemaphoreGive(mtxSensor);

    xSemaphoreTake(mtxRelay, portMAX_DELAY);
    bool l = g_relay.light;
    bool f = g_relay.fan;
    xSemaphoreGive(mtxRelay);

    lcd.clear();
    char line0[17];
    snprintf(line0, 17, "T:%02dC H:%02d%% %s", (int)t, (int)h, alert ? "!" : " ");
    lcd.setCursor(0, 0);
    lcd.print(line0);
    char line1[17];
    snprintf(line1, 17, "L:%s F:%s", l ? "ON " : "OFF", f ? "ON " : "OFF");
    lcd.setCursor(0, 1);
    lcd.print(line1);
    if (alert) { lcd.setCursor(15, 0); lcd.print("!"); }
  }
}

// ==================== TASK 3: BUTTON ====================
void taskButton(void* pvParam) {
  (void)pvParam;
  unsigned long lastBtnTime = 0;
  for (;;) {
    if (xSemaphoreTake(semButtons, portMAX_DELAY) == pdTRUE) {
      // Debounce delay 50 ms trong task
      vTaskDelay(pdMS_TO_TICKS(50));

      uint32_t flagLight = 0, flagFan = 0;
      portENTER_CRITICAL(&spinlock);
      flagLight = isrFlagLight; isrFlagLight = 0;
      flagFan   = isrFlagFan;   isrFlagFan = 0;
      portEXIT_CRITICAL(&spinlock);

      unsigned long now = millis();
      if ((now - lastBtnTime) > 250) {
        if (flagLight && digitalRead(BTN_LIGHT) == LOW) {
          lastBtnTime = now;
          RelayCmd_t cmd = { 0, true }; // 0 = light, toggle
          xQueueSend(qRelayCmd, &cmd, pdMS_TO_TICKS(100));
          Serial.println("[taskButton] Light toggle requested");
        }
        if (flagFan && digitalRead(BTN_FAN) == LOW) {
          lastBtnTime = now;
          RelayCmd_t cmd = { 1, true }; // 1 = fan, toggle
          xQueueSend(qRelayCmd, &cmd, pdMS_TO_TICKS(100));
          Serial.println("[taskButton] Fan toggle requested");
        }
      }
    }
  }
}

// ==================== TASK 4: RELAY CONTROL ====================
void taskRelayCtrl(void* pvParam) {
  (void)pvParam;
  RelayCmd_t cmd;
  for (;;) {
    if (xQueueReceive(qRelayCmd, &cmd, portMAX_DELAY) == pdTRUE) {
      xSemaphoreTake(mtxRelay, portMAX_DELAY);
      if (cmd.dev == 0) {
        if (cmd.toggle) g_relay.light = !g_relay.light;
        else g_relay.light = false;
        digitalWrite(RELAY_LIGHT, g_relay.light ? RELAY_ON_LEVEL : RELAY_OFF_LEVEL);
        Serial.printf("[taskRelayCtrl] Light -> %s\n", g_relay.light ? "ON" : "OFF");
      } else {
        if (cmd.toggle) g_relay.fan = !g_relay.fan;
        else g_relay.fan = false;
        digitalWrite(RELAY_FAN, g_relay.fan ? RELAY_ON_LEVEL : RELAY_OFF_LEVEL);
        Serial.printf("[taskRelayCtrl] Fan -> %s\n", g_relay.fan ? "ON" : "OFF");
      }
      xSemaphoreGive(mtxRelay);

      // Đánh thức LCD cập nhật ngay
      if (hTaskLCD) xTaskNotifyGive(hTaskLCD);
    }
  }
}

// ==================== TASK 5: WIFI MANAGER ====================
void taskWiFiManager(void* pvParam) {
  (void)pvParam;
  WiFiCmd_t cmd;
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // chờ tmrWiFi 5s

    // Xử lý queue lệnh WiFi với timeout 0 (không block)
    while (xQueueReceive(qWiFiCmd, &cmd, 0) == pdTRUE) {
      if (cmd.type == 1) {  // CONNECT
        WiFi.begin(cmd.ssid, cmd.pass);
        xSemaphoreTake(mtxWiFi, portMAX_DELAY);
        g_wifi.staConnecting = true;
        g_wifi.wifiConnectStart = millis();
        g_wifi.currentStaSSID = String(cmd.ssid);
        xSemaphoreGive(mtxWiFi);
        Serial.printf("[taskWiFiManager] STA connecting to %s\n", cmd.ssid);
      }
    }

    // Kiểm tra trạng thái STA
    bool connected = (WiFi.status() == WL_CONNECTED);
    xSemaphoreTake(mtxWiFi, portMAX_DELAY);
    if (connected) {
      if (g_wifi.staConnecting) {
        g_wifi.staConnecting = false;
        g_wifi.currentStaSSID = WiFi.SSID();
        Serial.println("[taskWiFiManager] STA Connected! IP: " + WiFi.localIP().toString());
        xSemaphoreGive(semWiFiReady);  // Báo hiệu cho các task đang chờ
      }
    } else {
      if (g_wifi.staConnecting && (millis() - g_wifi.wifiConnectStart > 20000)) {
        g_wifi.staConnecting = false;
        Serial.println("[taskWiFiManager] STA connect timeout");
      }
    }
    xSemaphoreGive(mtxWiFi);
  }
}

// ==================== TASK 6: WEB SERVER ====================
void taskWebServer(void* pvParam) {
  (void)pvParam;
  for (;;) {
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(2));  // nhường CPU, tránh watchdog
  }
}

// ==================== WEB HANDLERS ====================
void handleRoot() {
  server.send(200, "text/html; charset=utf-8", HTML_PAGE);
}

void handleStatus() {
  xSemaphoreTake(mtxSensor, portMAX_DELAY);
  float t = g_sensor.temp;
  float h = g_sensor.hum;
  float tt = g_sensor.tempThreshold;
  float ht = g_sensor.humThreshold;
  bool alert = g_sensor.alert;
  xSemaphoreGive(mtxSensor);

  xSemaphoreTake(mtxRelay, portMAX_DELAY);
  bool l = g_relay.light;
  bool f = g_relay.fan;
  xSemaphoreGive(mtxRelay);

  String alertMsg = "";
  if (t > tt) alertMsg += "Nhiệt độ cao (" + String(t, 1) + "°C > " + String(tt, 1) + "°C) ";
  if (h > ht) alertMsg += "Độ ẩm cao (" + String(h, 1) + "% > " + String(ht, 1) + "%)";

  String json = "{";
  json += "\"temp\":" + String(t, 1) + ",";
  json += "\"hum\":" + String(h, 1) + ",";
  json += "\"light\":" + String(l ? "true" : "false") + ",";
  json += "\"fan\":" + String(f ? "true" : "false") + ",";
  json += "\"alert\":" + String(alert ? "true" : "false") + ",";
  json += "\"alertMsg\":\"" + jsonEscape(alertMsg) + "\",";
  json += "\"tempThreshold\":" + String(tt, 1) + ",";
  json += "\"humThreshold\":" + String(ht, 1);
  json += "}";
  server.send(200, "application/json", json);
}

void handleToggle() {
  String device = server.arg("device");
  RelayCmd_t cmd;
  cmd.toggle = true;
  if (device == "light") cmd.dev = 0;
  else if (device == "fan") cmd.dev = 1;
  else { server.send(400, "application/json", "{\"success\":false}"); return; }

  if (xQueueSend(qRelayCmd, &cmd, pdMS_TO_TICKS(100)) == pdTRUE) {
    server.send(200, "application/json", "{\"success\":true}");
  } else {
    server.send(503, "application/json", "{\"success\":false,\"message\":\"Queue full\"}");
  }
}

void handleThreshold() {
  xSemaphoreTake(mtxSensor, portMAX_DELAY);
  if (server.hasArg("temp")) g_sensor.tempThreshold = server.arg("temp").toFloat();
  if (server.hasArg("hum"))  g_sensor.humThreshold  = server.arg("hum").toFloat();
  xSemaphoreGive(mtxSensor);
  server.send(200, "application/json", "{\"success\":true}");
}

void handleWifiScan() {
  // Scan chạy trực tiếp trong context taskWebServer (WiFi API đã thread-safe)
  int n = WiFi.scanNetworks();
  String json = "[";
  for (int i = 0; i < n; i++) {
    if (i > 0) json += ",";
    json += "{";
    json += "\"ssid\":\"" + jsonEscape(WiFi.SSID(i)) + "\",";
    json += "\"rssi\":" + String(WiFi.RSSI(i)) + ",";
    json += "\"enc\":" + String(WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? "false" : "true");
    json += "}";
  }
  json += "]";
  WiFi.scanDelete();
  server.send(200, "application/json", json);
}

void handleWifiConnect() {
  String ssid = server.arg("ssid");
  String pass = server.arg("pass");
  if (ssid.length() == 0) {
    server.send(400, "application/json", "{\"success\":false,\"message\":\"SSID rỗng\"}");
    return;
  }
  WiFiCmd_t cmd;
  cmd.type = 1; // CONNECT
  strncpy(cmd.ssid, ssid.c_str(), sizeof(cmd.ssid) - 1);
  cmd.ssid[sizeof(cmd.ssid) - 1] = '\0';
  strncpy(cmd.pass, pass.c_str(), sizeof(cmd.pass) - 1);
  cmd.pass[sizeof(cmd.pass) - 1] = '\0';

  if (xQueueSend(qWiFiCmd, &cmd, pdMS_TO_TICKS(100)) == pdTRUE) {
    server.send(200, "application/json", "{\"success\":true,\"message\":\"Đã gửi yêu cầu kết nối WiFi. Hãy chờ một chút...\"}");
  } else {
    server.send(503, "application/json", "{\"success\":false,\"message\":\"WiFi queue full\"}");
  }
}

void handleWifiStatus() {
  bool connected = (WiFi.status() == WL_CONNECTED);

  xSemaphoreTake(mtxWiFi, portMAX_DELAY);
  bool connecting = g_wifi.staConnecting;
  String ssid = connected ? WiFi.SSID() : g_wifi.currentStaSSID;
  xSemaphoreGive(mtxWiFi);

  String json = "{";
  json += "\"connected\":" + String(connected ? "true" : "false") + ",";
  json += "\"connecting\":" + String((!connected && connecting) ? "true" : "false") + ",";
  json += "\"ssid\":\"" + jsonEscape(ssid) + "\",";
  json += "\"ip\":\"" + (connected ? WiFi.localIP().toString() : String("")) + "\",";
  json += "\"rssi\":" + String(connected ? WiFi.RSSI() : 0) + ",";
  json += "\"apip\":\"" + WiFi.softAPIP().toString() + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n========================================");
  Serial.println("  ESP32-S3 Smart Home + FreeRTOS");
  Serial.println("========================================");

  // GPIO init
  pinMode(RELAY_LIGHT, OUTPUT);
  pinMode(RELAY_FAN, OUTPUT);
  digitalWrite(RELAY_LIGHT, RELAY_OFF_LEVEL);
  digitalWrite(RELAY_FAN, RELAY_OFF_LEVEL);

  pinMode(BTN_LIGHT, INPUT_PULLUP);
  pinMode(BTN_FAN, INPUT_PULLUP);

  // I2C + LCD
  Wire.begin(I2C_SDA, I2C_SCL);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("  ESP32-S3 SHC  ");
  lcd.setCursor(0, 1);
  lcd.print("  RTOS Init...   ");

  dht.begin();

  // WiFi init
  WiFi.mode(WIFI_AP_STA);
  WiFi.setAutoReconnect(true);
  WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
  WiFi.softAP(AP_SSID, AP_PASSWORD);

  // Auto STA connect nếu cấu hình sẵn
  if (strlen(STA_SSID) > 0) {
    WiFi.begin(STA_SSID, STA_PASSWORD);
    g_wifi.staConnecting = true;
    g_wifi.wifiConnectStart = millis();
    g_wifi.currentStaSSID = String(STA_SSID);
  }

  Serial.println("[AP] SSID: " + String(AP_SSID));
  Serial.println("[AP] IP: " + WiFi.softAPIP().toString());

  // --- Tạo Mutex (3) ---
  mtxSensor = xSemaphoreCreateMutex();
  mtxRelay  = xSemaphoreCreateMutex();
  mtxWiFi   = xSemaphoreCreateMutex();

  // --- Tạo Queue (2) ---
  qRelayCmd = xQueueCreate(5, sizeof(RelayCmd_t));
  qWiFiCmd  = xQueueCreate(3, sizeof(WiFiCmd_t));

  // --- Tạo Semaphore (2) ---
  semButtons   = xSemaphoreCreateBinary();
  semWiFiReady = xSemaphoreCreateBinary();

  // --- Tạo Timers (3) ---
  tmrDHT  = xTimerCreate("DHT",  pdMS_TO_TICKS(2000), pdTRUE, NULL, cbTimerDHT);
  tmrLCD  = xTimerCreate("LCD",  pdMS_TO_TICKS(1000), pdTRUE, NULL, cbTimerLCD);
  tmrWiFi = xTimerCreate("WiFi", pdMS_TO_TICKS(5000), pdTRUE, NULL, cbTimerWiFi);

  // --- Đăng ký Web Handler ---
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/toggle", HTTP_GET, handleToggle);
  server.on("/api/threshold", HTTP_GET, handleThreshold);
  server.on("/api/wifi/scan", HTTP_GET, handleWifiScan);
  server.on("/api/wifi/connect", HTTP_GET, handleWifiConnect);
  server.on("/api/wifi/status", HTTP_GET, handleWifiStatus);
  server.begin();
  Serial.println("Web server started!");

  // --- Tạo Tasks (6) ---
  xTaskCreatePinnedToCore(taskSensor,     "Sensor",     4096, NULL, 3, &hTaskSensor,    0);
  xTaskCreatePinnedToCore(taskLCD,        "LCD",        4096, NULL, 1, &hTaskLCD,      0);
  xTaskCreatePinnedToCore(taskButton,     "Button",     4096, NULL, 4, NULL,           0);
  xTaskCreatePinnedToCore(taskRelayCtrl,  "RelayCtrl",  4096, NULL, 4, NULL,           0);
  xTaskCreatePinnedToCore(taskWiFiManager,"WiFiMgr",    4096, NULL, 3, &hTaskWiFiMgr,  1);
  xTaskCreatePinnedToCore(taskWebServer,  "WebServer",  8192, NULL, 2, NULL,           1);

  // --- Khởi động Timers (3) ---
  xTimerStart(tmrDHT,  0);
  xTimerStart(tmrLCD,  0);
  xTimerStart(tmrWiFi, 0);

  // --- Gắn ngắt (2 interrupts) ---
  attachInterrupt(digitalPinToInterrupt(BTN_LIGHT), isrBtnLight, FALLING);
  attachInterrupt(digitalPinToInterrupt(BTN_FAN),   isrBtnFan,   FALLING);

  delay(500);
  Serial.println("FreeRTOS init complete. Tasks running.");
}

// ==================== LOOP (empty) ====================
void loop() {
  // Tất cả logic đã được xử lý bởi 6 FreeRTOS tasks.
  // Loop() không cần thực hiện gì, chỉ delay để tránh watchdog.
  vTaskDelay(pdMS_TO_TICKS(1000));
}