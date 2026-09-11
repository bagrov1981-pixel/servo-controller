#include <WiFi.h>
#include <WebServer.h>
#include <driver/twai.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <XPT2046_Touchscreen.h>
#include <SPI.h>

// ========== PIN DEFINITIONS ==========
#define TFT_CS   10
#define TFT_DC   2
#define TFT_RST  14
#define TOUCH_CS   7
#define TOUCH_IRQ  6
#define SPI_SCK    12
#define SPI_MISO   11
#define SPI_MOSI   13
#define CAN_TX_PIN 5
#define CAN_RX_PIN 4

// ========== HARDWARE INIT ==========
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
XPT2046_Touchscreen ts(TOUCH_CS, TOUCH_IRQ);
WebServer server(80);

// ========== MD89MW-CAN PARAMETERS (500kbps) ==========
const uint32_t CENTER_POS = 8192;      // 4.096ms - Neutral
const uint32_t LEFT_POS = 4096;        // 2.048ms - 90° Left
const uint32_t RIGHT_POS = 12288;      // 6.144ms - 90° Right

const uint32_t NODE_ID_SERVO1 = 0x000;
const uint32_t NODE_ID_SERVO2 = 0x001;

// ========== MOTION PARAMETERS ==========
const int TOTAL_STEPS = 180;
const int STEP_DELAY = 30;             // 30ms - better synchronization
const int RESPONSE_TIMEOUT = 150;

// ========== DATA STRUCTURES ==========
struct CANMessage {
  unsigned long id;
  uint8_t dlc;
  uint8_t data[8];
  unsigned long timestamp;
};

struct ServoState {
  uint32_t currentPos;
  uint32_t commandedPos;
  bool active;
  bool responding;
  int responseCount;
};

CANMessage canLogs[50];
int logCount = 0;

ServoState servo1 = {CENTER_POS, CENTER_POS, false, false, 0};
ServoState servo2 = {CENTER_POS, CENTER_POS, false, false, 0};
int selectedServo = 1;  // 1 or 2

unsigned long lastTftUpdate = 0;
unsigned long lastTouchTime = 0;

// ========== TOUCHSCREEN BUTTON DEFINITIONS ==========
#define BTN_SEL1_X 10
#define BTN_SEL1_Y 10
#define BTN_SEL1_W 80
#define BTN_SEL1_H 40

#define BTN_SEL2_X 100
#define BTN_SEL2_Y 10
#define BTN_SEL2_W 80
#define BTN_SEL2_H 40

#define BTN_CTRL_X 190
#define BTN_CTRL_Y 10
#define BTN_CTRL_W 130
#define BTN_CTRL_H 40

#define BTN_CLR_X 10
#define BTN_CLR_Y 190
#define BTN_CLR_W 310
#define BTN_CLR_H 40

// ========== UTILITY FUNCTIONS ==========
void printCAN(const char* dir, uint32_t nodeId, const twai_message_t& msg) {
  Serial.print("[");
  Serial.print(dir);
  Serial.print "][N");
  Serial.print(nodeId, HEX);
  Serial.print("] ");
  uint32_t pos = msg.data[4] | (msg.data[5] << 8) | (msg.data[6] << 16) | (msg.data[7] << 24);
  Serial.print("Pos:");
  Serial.println(pos);
}

uint32_t parsePos(const uint8_t* data) {
  return data[4] | (data[5] << 8) | (data[6] << 16) | (data[7] << 24);
}

ServoState& getSelectedServo() {
  return (selectedServo == 1) ? servo1 : servo2;
}

uint32_t getServoNodeId() {
  return (selectedServo == 1) ? NODE_ID_SERVO1 : NODE_ID_SERVO2;
}

// ========== CAN INITIALIZATION ==========
bool initCAN_500kbps() {
  Serial.println("\n[CAN] Initializing at 500kbps...");
  
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
    (gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL
  );
  g_config.alerts_enabled = TWAI_ALERT_TX_IDLE | TWAI_ALERT_RX_DATA;
  g_config.clkout_divider = 0;

  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("[ERROR] Failed to install driver");
    return false;
  }
  
  if (twai_start() != ESP_OK) {
    Serial.println("[ERROR] Failed to start driver");
    twai_driver_uninstall();
    return false;
  }

  Serial.println("[OK] CAN ready at 500kbps");
  delay(200);
  return true;
}

// ========== SERVO COMMUNICATION ==========
void sendCANFrame(uint32_t position, uint32_t nodeId) {
  twai_message_t tx_msg = {};
  tx_msg.identifier = nodeId;
  tx_msg.data_length_code = 8;
  tx_msg.flags = 0;
  
  tx_msg.data[0] = 0x96;
  tx_msg.data[1] = 0x00;
  tx_msg.data[2] = 0x1E;
  tx_msg.data[3] = 0x02;
  tx_msg.data[4] = (position & 0xFF);
  tx_msg.data[5] = ((position >> 8) & 0xFF);
  tx_msg.data[6] = ((position >> 16) & 0xFF);
  tx_msg.data[7] = ((position >> 24) & 0xFF);
  
  if (twai_transmit(&tx_msg, pdMS_TO_TICKS(50)) == ESP_OK) {
    CANMessage msg;
    msg.id = tx_msg.identifier;
    msg.dlc = tx_msg.data_length_code;
    msg.timestamp = millis();
    for (int i = 0; i < 8; i++) msg.data[i] = tx_msg.data[i];
    if (logCount < 50) canLogs[logCount++] = msg;
    
    printCAN("TX", nodeId, tx_msg);
  }
}

void waitForServoResponse(uint32_t targetPos, uint32_t nodeId, int timeoutMs) {
  unsigned long startTime = millis();
  ServoState& servo = (nodeId == NODE_ID_SERVO1) ? servo1 : servo2;
  
  while (millis() - startTime < timeoutMs) {
    twai_message_t rx = {0};
    if (twai_receive(&rx, pdMS_TO_TICKS(5)) == ESP_OK) {
      if (rx.identifier == nodeId && rx.data_length_code >= 8) {
        servo.currentPos = parsePos(rx.data);
        servo.responding = true;
        servo.responseCount++;
        
        CANMessage msg;
        msg.id = rx.identifier;
        msg.dlc = rx.data_length_code;
        msg.timestamp = millis();
        for (int i = 0; i < 8; i++) msg.data[i] = rx.data[i];
        if (logCount < 50) canLogs[logCount++] = msg;
        
        printCAN("RX", nodeId, rx);
        
        if (abs((int32_t)servo.currentPos - (int32_t)targetPos) < 50) {
          return;
        }
      }
    }
  }
}

void testServoResponse(uint32_t nodeId) {
  ServoState& servo = (nodeId == NODE_ID_SERVO1) ? servo1 : servo2;
  int servoNum = (nodeId == NODE_ID_SERVO1) ? 1 : 2;
  
  Serial.print("\n========== SERVO ");
  Serial.print(servoNum);
  Serial.println(" TEST ==========");
  
  servo.responding = false;
  servo.responseCount = 0;
  
  Serial.print("[TEST] Sending CENTER position to Servo ");
  Serial.println(servoNum);
  sendCANFrame(CENTER_POS, nodeId);
  
  waitForServoResponse(CENTER_POS, nodeId, 1500);
  
  if (servo.responding) {
    Serial.print("[OK] Servo ");
    Serial.print(servoNum);
    Serial.print(" is responding! (");
    Serial.print(servo.responseCount);
    Serial.println(" responses)");
  } else {
    Serial.print("[ERROR] NO RESPONSE from Servo ");
    Serial.println(servoNum);
  }
}

// ========== MOTION CONTROL ==========
void moveSmooth(uint32_t start, uint32_t end, uint32_t nodeId, bool forward) {
  ServoState& servo = (nodeId == NODE_ID_SERVO1) ? servo1 : servo2;
  int servoNum = (nodeId == NODE_ID_SERVO1) ? 1 : 2;
  
  Serial.print("\n>>> Servo");
  Serial.print(servoNum);
  Serial.print(" ");
  Serial.print(forward ? "FORWARD" : "BACKWARD");
  Serial.print(" (");
  Serial.print(start);
  Serial.print(" -> ");
  Serial.print(end);
  Serial.println(")");
  
  int32_t distance = (int32_t)end - (int32_t)start;
  
  for (int i = 0; i <= TOTAL_STEPS; i++) {
    if (!servo.active) break;
    
    // S-curve синусоїда для плавного руху
    float progress = (float)i / (float)TOTAL_STEPS;
    float curve = (1.0 - cos(progress * PI)) / 2.0;
    uint32_t target = start + (int32_t)(distance * curve);
    
    servo.commandedPos = target;
    
    Serial.print("S");
    Serial.print(servoNum);
    Serial.print(" Step ");
    Serial.print(i);
    Serial.print("/");
    Serial.print(TOTAL_STEPS);
    Serial.print(" | Cmd:");
    Serial.print(target);
    Serial.print(" | Pos:");
    Serial.print(servo.currentPos);
    
    int32_t lag = (int32_t)target - (int32_t)servo.currentPos;
    Serial.print(" | Lag:");
    Serial.println(lag);
    
    sendCANFrame(target, nodeId);
    waitForServoResponse(target, nodeId, RESPONSE_TIMEOUT);
    
    delay(STEP_DELAY);
  }
  
  Serial.print("[DONE] Servo");
  Serial.print(servoNum);
  Serial.println(" motion complete");
}

// ========== TOUCHSCREEN UI ==========
void drawTouchButtons() {
  // SERVO 1 BUTTON
  uint16_t color1 = (selectedServo == 1) ? 0x07E0 : 0x4208;
  tft.fillRect(BTN_SEL1_X, BTN_SEL1_Y, BTN_SEL1_W, BTN_SEL1_H, color1);
  tft.drawRect(BTN_SEL1_X, BTN_SEL1_Y, BTN_SEL1_W, BTN_SEL1_H, ST77XX_WHITE);
  tft.setTextColor(ST77XX_BLACK);
  tft.setTextSize(2);
  tft.setCursor(BTN_SEL1_X + 12, BTN_SEL1_Y + 12);
  tft.print("S1");
  if (servo1.responding) {
    tft.setTextSize(1);
    tft.setTextColor(ST77XX_GREEN);
    tft.setCursor(BTN_SEL1_X + 12, BTN_SEL1_Y + 28);
    tft.print("OK");
  }

  // SERVO 2 BUTTON
  uint16_t color2 = (selectedServo == 2) ? 0x07E0 : 0x4208;
  tft.fillRect(BTN_SEL2_X, BTN_SEL2_Y, BTN_SEL2_W, BTN_SEL2_H, color2);
  tft.drawRect(BTN_SEL2_X, BTN_SEL2_Y, BTN_SEL2_W, BTN_SEL2_H, ST77XX_WHITE);
  tft.setTextColor(ST77XX_BLACK);
  tft.setTextSize(2);
  tft.setCursor(BTN_SEL2_X + 12, BTN_SEL2_Y + 12);
  tft.print("S2");
  if (servo2.responding) {
    tft.setTextSize(1);
    tft.setTextColor(ST77XX_GREEN);
    tft.setCursor(BTN_SEL2_X + 12, BTN_SEL2_Y + 28);
    tft.print("OK");
  }

  // CONTROL BUTTON
  ServoState& active = getSelectedServo();
  uint16_t ctrlColor = active.active ? 0xF800 : 0x07E0;
  tft.fillRect(BTN_CTRL_X, BTN_CTRL_Y, BTN_CTRL_W, BTN_CTRL_H, ctrlColor);
  tft.drawRect(BTN_CTRL_X, BTN_CTRL_Y, BTN_CTRL_W, BTN_CTRL_H, ST77XX_WHITE);
  tft.setTextColor(ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setCursor(BTN_CTRL_X + 20, BTN_CTRL_Y + 5);
  tft.print("MOVE");
  tft.setCursor(BTN_CTRL_X + 10, BTN_CTRL_Y + 20);
  tft.print(active.active ? "STOP" : "START");

  // CLEAR BUTTON
  tft.fillRect(BTN_CLR_X, BTN_CLR_Y, BTN_CLR_W, BTN_CLR_H, 0xF81F);
  tft.drawRect(BTN_CLR_X, BTN_CLR_Y, BTN_CLR_W, BTN_CLR_H, ST77XX_WHITE);
  tft.setTextColor(ST77XX_BLACK);
  tft.setTextSize(2);
  tft.setCursor(BTN_CLR_X + 120, BTN_CLR_Y + 10);
  tft.print("CLEAR");
}

void updateDisplay() {
  // HEADER
  tft.fillRect(0, 0, 320, 60, 0x000F);
  tft.setCursor(5, 5);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.print("500kbps");
  tft.setCursor(5, 25);
  tft.setTextSize(1);
  tft.print("S1:");
  tft.print(servo1.currentPos);
  tft.print(" S2:");
  tft.print(servo2.currentPos);

  // SELECTED SERVO INFO
  ServoState& selected = getSelectedServo();
  tft.fillRect(0, 60, 320, 70, ST77XX_BLACK);
  tft.setCursor(5, 65);
  tft.setTextColor(ST77XX_CYAN);
  tft.setTextSize(2);
  tft.print("S");
  tft.print(selectedServo);
  tft.print(" Active");

  tft.setTextSize(1);
  tft.setTextColor(selected.responding ? ST77XX_GREEN : ST77XX_RED);
  tft.setCursor(5, 85);
  tft.print("Pos:");
  tft.println(selected.currentPos);

  tft.setCursor(5, 98);
  tft.print("Cmd:");
  tft.println(selected.commandedPos);

  int32_t lag = (int32_t)selected.commandedPos - (int32_t)selected.currentPos;
  tft.setTextColor(lag == 0 ? ST77XX_GREEN : ST77XX_YELLOW);
  tft.setCursor(5, 111);
  tft.print("Lag:");
  tft.println(lag);

  // CAN LOG (last 8 messages)
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_GREEN);
  int y = 135;
  for (int i = 0; i < 8; i++) {
    tft.fillRect(0, y, 320, 12, ST77XX_BLACK);
    int idx = logCount - 8 + i;
    if (idx >= 0 && idx < logCount) {
      tft.setCursor(5, y);
      uint32_t pos = parsePos(canLogs[idx].data);
      tft.print("N");
      tft.print(canLogs[idx].id, HEX);
      tft.print(":");
      tft.print(pos);
    }
    y += 13;
  }

  drawTouchButtons();
}

// ========== TOUCHSCREEN HANDLING ==========
void handleTouch() {
  if (!ts.touched() || millis() - lastTouchTime < 300) return;

  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  TS_Point p = ts.getPoint();
  SPI.endTransaction();

  int x = map(p.x, 3703, 463, 0, 320);
  int y = map(p.y, 3110, 528, 0, 240);

  Serial.print("[TOUCH] X:");
  Serial.print(x);
  Serial.print(" Y:");
  Serial.println(y);

  // SELECT SERVO 1
  if (x >= BTN_SEL1_X && x <= (BTN_SEL1_X + BTN_SEL1_W) &&
      y >= BTN_SEL1_Y && y <= (BTN_SEL1_Y + BTN_SEL1_H)) {
    selectedServo = 1;
    Serial.println("[SELECT] Servo 1");
    lastTouchTime = millis();
    return;
  }

  // SELECT SERVO 2
  if (x >= BTN_SEL2_X && x <= (BTN_SEL2_X + BTN_SEL2_W) &&
      y >= BTN_SEL2_Y && y <= (BTN_SEL2_Y + BTN_SEL2_H)) {
    selectedServo = 2;
    Serial.println("[SELECT] Servo 2");
    lastTouchTime = millis();
    return;
  }

  // CONTROL BUTTON
  if (x >= BTN_CTRL_X && x <= (BTN_CTRL_X + BTN_CTRL_W) &&
      y >= BTN_CTRL_Y && y <= (BTN_CTRL_Y + BTN_CTRL_H)) {
    ServoState& servo = getSelectedServo();
    uint32_t nodeId = getServoNodeId();
    
    if (!servo.active) {
      Serial.print("[CONTROL] Starting Servo");
      Serial.println(selectedServo);
      servo.active = true;
      testServoResponse(nodeId);
      if (!servo.responding) {
        servo.active = false;
      }
    } else {
      Serial.print("[CONTROL] Stopping Servo");
      Serial.println(selectedServo);
      servo.active = false;
    }
    lastTouchTime = millis();
    return;
  }

  // CLEAR LOGS
  if (x >= BTN_CLR_X && x <= (BTN_CLR_X + BTN_CLR_W) &&
      y >= BTN_CLR_Y && y <= (BTN_CLR_Y + BTN_CLR_H)) {
    logCount = 0;
    Serial.println("[CLEAR] Logs cleared");
    lastTouchTime = millis();
    return;
  }

  lastTouchTime = millis();
}

// ========== WEB SERVER ==========
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset='UTF-8'>
    <title>Dual Servo MD89MW-CAN</title>
    <style>
        body { font-family: Arial; background: #1e1e24; color: #fff; padding: 15px; }
        .panel { background: #2a2a35; padding: 15px; margin: 10px 0; border-radius: 4px; }
        .servo-panel { display: inline-block; width: 48%; margin-right: 2%; vertical-align: top; }
        button { width: 100%; padding: 12px; background: #00d2ff; border: none; color: #000; font-weight: bold; border-radius: 4px; cursor: pointer; margin: 5px 0; }
        button.red { background: #ff4444; }
        button.active { background: #44ff44; }
        #log { background: #111; color: #0f0; padding: 10px; height: 300px; overflow-y: auto; font-family: monospace; font-size: 12px; }
    </style>
</head>
<body>
    <h2>Dual MD89MW-CAN Servo (500kbps)</h2>
    
    <div class="servo-panel panel">
        <h3>Servo 1</h3>
        <div>Status: <span id="status1">-</span></div>
        <div>Current: <span id="pos1">-</span></div>
        <div>Commanded: <span id="cmd1">-</span></div>
        <div>Lag: <span id="lag1">-</span></div>
        <button onclick="toggleServo(1)">Toggle S1</button>
    </div>

    <div class="servo-panel panel">
        <h3>Servo 2</h3>
        <div>Status: <span id="status2">-</span></div>
        <div>Current: <span id="pos2">-</span></div>
        <div>Commanded: <span id="cmd2">-</span></div>
        <div>Lag: <span id="lag2">-</span></div>
        <button onclick="toggleServo(2)">Toggle S2</button>
    </div>

    <div class="panel">
        <h3>Control</h3>
        <button class="red" onclick="clearLogs()">Clear Logs</button>
    </div>

    <div class="panel">
        <h3>CAN Log (Last 30)</h3>
        <div id="log">Waiting...</div>
    </div>

    <script>
        setInterval(() => {
            fetch('/status').then(r => r.json()).then(d => {
                document.getElementById('status1').textContent = d.s1_active ? 'ACTIVE' : 'IDLE';
                document.getElementById('pos1').textContent = d.s1_pos;
                document.getElementById('cmd1').textContent = d.s1_cmd;
                document.getElementById('lag1').textContent = d.s1_lag;
                
                document.getElementById('status2').textContent = d.s2_active ? 'ACTIVE' : 'IDLE';
                document.getElementById('pos2').textContent = d.s2_pos;
                document.getElementById('cmd2').textContent = d.s2_cmd;
                document.getElementById('lag2').textContent = d.s2_lag;
            });
            fetch('/logs').then(r => r.text()).then(d => {
                if(d.trim()) {
                    document.getElementById('log').innerHTML = d;
                    document.getElementById('log').scrollTop = 9999;
                }
            });
        }, 500);

        function toggleServo(n) { fetch('/toggle?s=' + n); }
        function clearLogs() { fetch('/clear'); }
    </script>
</body>
</html>
)rawliteral";

void setupWebServer() {
  server.on("/", []() { server.send(200, "text/html", index_html); });
  
  server.on("/status", []() {
    int32_t lag1 = (int32_t)servo1.commandedPos - (int32_t)servo1.currentPos;
    int32_t lag2 = (int32_t)servo2.commandedPos - (int32_t)servo2.currentPos;
    
    String json = "{\"s1_active\":" + String(servo1.active ? "true" : "false") + 
                  ",\"s1_pos\":" + String(servo1.currentPos) + 
                  ",\"s1_cmd\":" + String(servo1.commandedPos) + 
                  ",\"s1_lag\":" + String(lag1) + 
                  ",\"s2_active\":" + String(servo2.active ? "true" : "false") + 
                  ",\"s2_pos\":" + String(servo2.currentPos) + 
                  ",\"s2_cmd\":" + String(servo2.commandedPos) + 
                  ",\"s2_lag\":" + String(lag2) + "}";
    server.send(200, "application/json", json);
  });
  
  server.on("/toggle", []() {
    int s = server.arg("s").toInt();
    if (s == 1) {
      if (!servo1.active) {
        servo1.active = true;
        testServoResponse(NODE_ID_SERVO1);
        if (!servo1.responding) servo1.active = false;
      } else {
        servo1.active = false;
      }
    } else if (s == 2) {
      if (!servo2.active) {
        servo2.active = true;
        testServoResponse(NODE_ID_SERVO2);
        if (!servo2.responding) servo2.active = false;
      } else {
        servo2.active = false;
      }
    }
    server.send(200, "text/plain", "OK");
  });
  
  server.on("/clear", []() { logCount = 0; server.send(200, "text/plain", "OK"); });
  
  server.on("/logs", []() {
    String html = "";
    for (int i = max(0, logCount - 30); i < logCount; i++) {
      uint32_t pos = parsePos(canLogs[i].data);
      html += "N";
      html += String(canLogs[i].id, HEX);
      html += " Pos:";
      html += String(pos);
      html += " T:";
      html += String(canLogs[i].timestamp);
      html += "<br>";
    }
    server.send(200, "text/html", html);
  });

  server.begin();
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║ Dual MD89MW-CAN Servo Controller       ║");
  Serial.println("║ 500kbps | Touchscreen UI               ║");
  Serial.println("╚════════════════════════════════════════╝\n");

  SPI.end();
  delay(10);
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, -1);

  tft.init(240, 320);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);

  ts.begin(SPI);
  ts.setRotation(1);

  if (!initCAN_500kbps()) {
    Serial.println("[FATAL] CAN initialization failed!");
    while(1) delay(1000);
  }

  WiFi.softAP("ESP32-S3-CAN", "password123");
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());

  setupWebServer();
  Serial.println("[HTTP] Ready\n");
}

// ========== MAIN LOOP ==========
void loop() {
  server.handleClient();
  handleTouch();

  // Process Servo 1
  if (servo1.active) {
    moveSmooth(CENTER_POS, RIGHT_POS, NODE_ID_SERVO1, true);
    if (servo1.active) {
      Serial.println("\n=== Servo 1: Pause at END ===");
      delay(1000);
    }
    if (servo1.active) {
      moveSmooth(RIGHT_POS, CENTER_POS, NODE_ID_SERVO1, false);
      if (servo1.active) {
        Serial.println("\n=== Servo 1: Pause at START ===");
        delay(1000);
      }
    }
  }

  // Process Servo 2
  if (servo2.active) {
    moveSmooth(CENTER_POS, LEFT_POS, NODE_ID_SERVO2, true);
    if (servo2.active) {
      Serial.println("\n=== Servo 2: Pause at END ===");
      delay(1000);
    }
    if (servo2.active) {
      moveSmooth(LEFT_POS, CENTER_POS, NODE_ID_SERVO2, false);
      if (servo2.active) {
        Serial.println("\n=== Servo 2: Pause at START ===");
        delay(1000);
      }
    }
  }

  // Continuous CAN reception
  twai_message_t rx = {0};
  if (twai_receive(&rx, pdMS_TO_TICKS(0)) == ESP_OK) {
    if ((rx.identifier == NODE_ID_SERVO1 || rx.identifier == NODE_ID_SERVO2) && 
        rx.data_length_code >= 8) {
      ServoState& servo = (rx.identifier == NODE_ID_SERVO1) ? servo1 : servo2;
      servo.currentPos = parsePos(rx.data);
      servo.responding = true;
      
      CANMessage msg;
      msg.id = rx.identifier;
      msg.dlc = rx.data_length_code;
      msg.timestamp = millis();
      for (int i = 0; i < 8; i++) msg.data[i] = rx.data[i];
      if (logCount < 50) canLogs[logCount++] = msg;
      
      printCAN("RX", rx.identifier, rx);
    }
  }

  if (millis() - lastTftUpdate >= 100) {
    updateDisplay();
    lastTftUpdate = millis();
  }
}
