#include <WiFi.h>
#include <WebServer.h>
#include <driver/twai.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <XPT2046_Touchscreen.h>
#include <SPI.h>
#include <math.h>

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
#define START_BUTTON_PIN 15

// ========== DISPLAY LAYOUT ==========
#define SCREEN_W 320
#define SCREEN_H 240

#define BTN_SEL1_X 10
#define BTN_SEL1_Y 30
#define BTN_SEL1_W 145
#define BTN_SEL1_H 72

#define BTN_SEL2_X 165
#define BTN_SEL2_Y 30
#define BTN_SEL2_W 145
#define BTN_SEL2_H 72

#define INFO_X 0
#define INFO_Y 112
#define INFO_W 320
#define INFO_H 42

#define LOG_X 0
#define LOG_Y 164
#define LOG_W 320
#define LOG_H 76

// ========== HARDWARE INIT ==========
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
XPT2046_Touchscreen ts(TOUCH_CS, TOUCH_IRQ);
WebServer server(80);

// ========== MD89MW-CAN PARAMETERS (500kbps) ==========
const uint32_t CENTER_POS = 8192;
const uint32_t LEFT_POS = 4096;
const uint32_t RIGHT_POS = 12288;

const uint32_t NODE_ID_SERVO1 = 0x000;
const uint32_t NODE_ID_SERVO2 = 0x001;

// ========== MOTION PARAMETERS ==========
const int TOTAL_STEPS = 180;
const unsigned long STEP_DELAY = 30;
const unsigned long RESPONSE_TIMEOUT = 150;
const unsigned long MOTION_PAUSE_MS = 1000;
const unsigned long START_PROBE_TIMEOUT_MS = 1500;
const unsigned long TOUCH_DEBOUNCE_MS = 250;
const unsigned long START_DEBOUNCE_MS = 40;
const unsigned long LINK_STALE_MS = 1000;
const unsigned long TFT_UPDATE_INTERVAL_MS = 50;
const int MAX_CAN_LOGS = 50;
const int TFT_LOG_LINES = 6;

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
  bool starting;
  bool responding;
  bool linkHealthy;
  int responseCount;
  bool directionForward;
  bool boundaryHoldSent;
  int stepIndex;
  unsigned long lastStepAt;
  unsigned long pauseUntil;
  unsigned long lastResponseAt;
  unsigned long probeStartedAt;
};

struct DisplayCache {
  uint32_t s1Pos;
  uint32_t s2Pos;
  uint32_t selectedCurrent;
  uint32_t selectedCommanded;
  int32_t selectedLag;
  int selectedServo;
  bool selectedActive;
  bool selectedStarting;
  bool selectedLinkHealthy;
  bool s1Active;
  bool s2Active;
  bool s1Starting;
  bool s2Starting;
  bool s1LinkHealthy;
  bool s2LinkHealthy;
  uint32_t canLogVersion;
};

enum DisplayDirtyFlags : uint8_t {
  DIRTY_NONE = 0,
  DIRTY_STATIC = 1 << 0,
  DIRTY_SUMMARY = 1 << 1,
  DIRTY_BUTTONS = 1 << 2,
  DIRTY_SELECTED = 1 << 3,
  DIRTY_LOGS = 1 << 4,
  DIRTY_ALL = DIRTY_STATIC | DIRTY_SUMMARY | DIRTY_BUTTONS | DIRTY_SELECTED | DIRTY_LOGS
};

CANMessage canLogs[MAX_CAN_LOGS];
int logCount = 0;
int logStart = 0;
uint32_t canLogVersion = 0;

ServoState servo1 = {CENTER_POS, CENTER_POS, false, false, false, false, 0, true, false, 0, 0, 0, 0, 0};
ServoState servo2 = {CENTER_POS, CENTER_POS, false, false, false, false, 0, true, false, 0, 0, 0, 0, 0};
int selectedServo = 1;

unsigned long lastTftUpdate = 0;
unsigned long lastTouchTime = 0;
unsigned long lastStartButtonChange = 0;
bool lastStartButtonReading = HIGH;
bool startButtonStableState = HIGH;

bool displayInitialized = false;
bool displayDirty = true;
uint8_t displayDirtyFlags = DIRTY_ALL;
DisplayCache displayCache = {
  0xFFFFFFFFUL,
  0xFFFFFFFFUL,
  0xFFFFFFFFUL,
  0xFFFFFFFFUL,
  0x7FFFFFFF,
  -1,
  false,
  false,
  false,
  false,
  false,
  false,
  false,
  false,
  false,
  0xFFFFFFFFUL
};

int displayedLogIndices[TFT_LOG_LINES] = {-1, -1, -1, -1, -1, -1};
unsigned long displayedLogIds[TFT_LOG_LINES] = {
  0xFFFFFFFFUL, 0xFFFFFFFFUL, 0xFFFFFFFFUL, 0xFFFFFFFFUL, 0xFFFFFFFFUL, 0xFFFFFFFFUL
};
uint32_t displayedLogPositions[TFT_LOG_LINES] = {
  0xFFFFFFFFUL, 0xFFFFFFFFUL, 0xFFFFFFFFUL, 0xFFFFFFFFUL, 0xFFFFFFFFUL, 0xFFFFFFFFUL
};
unsigned long displayedLogTimestamps[TFT_LOG_LINES] = {
  0xFFFFFFFFUL, 0xFFFFFFFFUL, 0xFFFFFFFFUL, 0xFFFFFFFFUL, 0xFFFFFFFFUL, 0xFFFFFFFFUL
};
bool displayedLogUsed[TFT_LOG_LINES] = {false, false, false, false, false, false};

// ========== UTILITY FUNCTIONS ==========
void printCAN(const char* dir, uint32_t nodeId, const twai_message_t& msg) {
  Serial.print("[");
  Serial.print(dir);
  Serial.print("][N");
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

ServoState& getServoByNumber(int servoNum) {
  return (servoNum == 1) ? servo1 : servo2;
}

ServoState* findServoByNodeId(uint32_t nodeId) {
  if (nodeId == NODE_ID_SERVO1) {
    return &servo1;
  }
  if (nodeId == NODE_ID_SERVO2) {
    return &servo2;
  }
  return nullptr;
}

uint32_t getServoNodeId(int servoNum) {
  return (servoNum == 1) ? NODE_ID_SERVO1 : NODE_ID_SERVO2;
}

uint32_t getServoOuterPos(int servoNum) {
  return (servoNum == 1) ? RIGHT_POS : LEFT_POS;
}

void markDisplayDirty(uint8_t flags = DIRTY_ALL) {
  displayDirty = true;
  displayDirtyFlags |= flags;
}

const CANMessage& getCANLogAt(int index) {
  return canLogs[(logStart + index) % MAX_CAN_LOGS];
}

void appendCANLog(uint32_t id, uint8_t dlc, const uint8_t* data) {
  int writeIndex = (logStart + logCount) % MAX_CAN_LOGS;
  if (logCount >= MAX_CAN_LOGS) {
    writeIndex = logStart;
    logStart = (logStart + 1) % MAX_CAN_LOGS;
  } else {
    logCount++;
  }

  CANMessage& msg = canLogs[writeIndex];
  msg.id = id;
  msg.dlc = dlc;
  msg.timestamp = millis();
  for (int i = 0; i < 8; i++) {
    msg.data[i] = 0;
  }
  int bytesToCopy = (dlc < 8) ? dlc : 8;
  for (int i = 0; i < bytesToCopy; i++) {
    msg.data[i] = data[i];
  }

  canLogVersion++;
  markDisplayDirty(DIRTY_LOGS);
}

void clearLogs() {
  logCount = 0;
  logStart = 0;
  canLogVersion++;
  markDisplayDirty(DIRTY_LOGS);
}

void resetDisplayedLogsCache() {
  for (int i = 0; i < TFT_LOG_LINES; i++) {
    displayedLogIndices[i] = -1;
    displayedLogIds[i] = 0xFFFFFFFFUL;
    displayedLogPositions[i] = 0xFFFFFFFFUL;
    displayedLogTimestamps[i] = 0xFFFFFFFFUL;
    displayedLogUsed[i] = false;
  }
}

bool isInsideRect(int x, int y, int rx, int ry, int rw, int rh) {
  return x >= rx && x < (rx + rw) && y >= ry && y < (ry + rh);
}

int32_t servoLag(const ServoState& servo) {
  return (int32_t)servo.commandedPos - (int32_t)servo.currentPos;
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
    appendCANLog(tx_msg.identifier, tx_msg.data_length_code, tx_msg.data);
    printCAN("TX", nodeId, tx_msg);
  } else {
    Serial.print("[ERROR] TX failed for node ");
    Serial.println(nodeId, HEX);
  }
}

void applyServoResponse(const twai_message_t& rx) {
  ServoState* servo = findServoByNodeId(rx.identifier);
  if (servo == nullptr) {
    return;
  }

  servo->currentPos = parsePos(rx.data);
  servo->responding = true;
  servo->linkHealthy = true;
  servo->responseCount++;
  servo->lastResponseAt = millis();

  appendCANLog(rx.identifier, rx.data_length_code, rx.data);
  printCAN("RX", rx.identifier, rx);
  markDisplayDirty(DIRTY_SUMMARY | DIRTY_SELECTED | DIRTY_BUTTONS);
}

void processCANReceive(TickType_t waitTicks = 0) {
  twai_message_t rx = {};
  if (twai_receive(&rx, waitTicks) != ESP_OK) {
    return;
  }

  do {
    if ((rx.identifier == NODE_ID_SERVO1 || rx.identifier == NODE_ID_SERVO2) &&
        rx.data_length_code >= 8) {
      applyServoResponse(rx);
    }
  } while (twai_receive(&rx, 0) == ESP_OK);
}

void beginServoStartProbe(int servoNum) {
  ServoState& servo = getServoByNumber(servoNum);

  Serial.print("\n========== SERVO ");
  Serial.print(servoNum);
  Serial.println(" TEST ==========");

  servo.active = false;
  servo.starting = true;
  servo.responding = false;
  servo.linkHealthy = false;
  servo.responseCount = 0;
  servo.boundaryHoldSent = true;
  servo.lastResponseAt = 0;
  servo.probeStartedAt = millis();
  servo.commandedPos = CENTER_POS;

  Serial.print("[TEST] Sending CENTER position to Servo ");
  Serial.println(servoNum);
  sendCANFrame(CENTER_POS, getServoNodeId(servoNum));
  markDisplayDirty(DIRTY_SELECTED | DIRTY_BUTTONS | DIRTY_SUMMARY);
}

// ========== MOTION CONTROL ==========
void startServoMotion(int servoNum) {
  ServoState& servo = getServoByNumber(servoNum);

  if (servo.active || servo.starting) {
    return;
  }

  Serial.print("[CONTROL] Starting Servo ");
  Serial.println(servoNum);

  servo.directionForward = true;
  servo.boundaryHoldSent = true;
  servo.stepIndex = 0;
  servo.lastStepAt = 0;
  servo.pauseUntil = 0;
  servo.probeStartedAt = 0;
  servo.commandedPos = CENTER_POS;
  markDisplayDirty(DIRTY_SELECTED | DIRTY_BUTTONS | DIRTY_SUMMARY);

  beginServoStartProbe(servoNum);
}

void stopServoMotion(int servoNum) {
  ServoState& servo = getServoByNumber(servoNum);
  if (!servo.active && !servo.starting) {
    return;
  }

  Serial.print("[CONTROL] Stopping Servo ");
  Serial.println(servoNum);

  servo.active = false;
  servo.starting = false;
  servo.stepIndex = 0;
  servo.pauseUntil = 0;
  servo.probeStartedAt = 0;
  servo.commandedPos = servo.currentPos;
  markDisplayDirty(DIRTY_SELECTED | DIRTY_BUTTONS);
}

void toggleServoMotion(int servoNum) {
  ServoState& servo = getServoByNumber(servoNum);
  if (servo.active || servo.starting) {
    stopServoMotion(servoNum);
  } else {
    startServoMotion(servoNum);
  }
}

void serviceServoMotion(int servoNum) {
  ServoState& servo = getServoByNumber(servoNum);
  unsigned long now = millis();

  if (servo.starting) {
    if (servo.responding) {
      servo.starting = false;
      servo.active = true;
      servo.lastStepAt = 0;
      servo.pauseUntil = 0;
      Serial.print("[OK] Servo ");
      Serial.print(servoNum);
      Serial.print(" is responding! (");
      Serial.print(servo.responseCount);
      Serial.println(" responses)");
      markDisplayDirty(DIRTY_SELECTED | DIRTY_BUTTONS);
    } else if ((now - servo.probeStartedAt) >= START_PROBE_TIMEOUT_MS) {
      servo.starting = false;
      servo.active = false;
      Serial.print("[ERROR] NO RESPONSE from Servo ");
      Serial.println(servoNum);
      markDisplayDirty(DIRTY_SELECTED | DIRTY_BUTTONS);
    }
    return;
  }

  if (!servo.active) {
    return;
  }

  if (servo.pauseUntil != 0 && now < servo.pauseUntil) {
    return;
  }

  if (servo.lastStepAt != 0 && (now - servo.lastStepAt) < STEP_DELAY) {
    return;
  }

  uint32_t startPos = servo.directionForward ? CENTER_POS : getServoOuterPos(servoNum);
  uint32_t endPos = servo.directionForward ? getServoOuterPos(servoNum) : CENTER_POS;
  int32_t distance = (int32_t)endPos - (int32_t)startPos;
  int stepToSend = servo.stepIndex;
  if (servo.boundaryHoldSent && stepToSend == 0) {
    stepToSend = 1;
    servo.boundaryHoldSent = false;
  }

  float progress = (float)stepToSend / (float)TOTAL_STEPS;
  float curve = (1.0f - cosf(progress * PI)) * 0.5f;
  uint32_t target = startPos + (int32_t)(distance * curve);

  servo.commandedPos = target;
  servo.lastStepAt = now;
  sendCANFrame(target, getServoNodeId(servoNum));
  markDisplayDirty(DIRTY_SUMMARY | DIRTY_SELECTED);

  if (stepToSend >= TOTAL_STEPS) {
    servo.stepIndex = 0;
    servo.directionForward = !servo.directionForward;
    servo.boundaryHoldSent = true;
    servo.pauseUntil = now + MOTION_PAUSE_MS;

    Serial.print("[DONE] Servo ");
    Serial.print(servoNum);
    Serial.println(" reached segment end");
  } else {
    servo.stepIndex = stepToSend + 1;
  }
}

void refreshServoLinkState() {
  unsigned long now = millis();

  bool servo1Healthy = servo1.lastResponseAt != 0 && (now - servo1.lastResponseAt) <= LINK_STALE_MS;
  bool servo2Healthy = servo2.lastResponseAt != 0 && (now - servo2.lastResponseAt) <= LINK_STALE_MS;

  if (servo1.linkHealthy != servo1Healthy) {
    servo1.linkHealthy = servo1Healthy;
    markDisplayDirty(DIRTY_SELECTED | DIRTY_BUTTONS);
  }

  if (servo2.linkHealthy != servo2Healthy) {
    servo2.linkHealthy = servo2Healthy;
    markDisplayDirty(DIRTY_SELECTED | DIRTY_BUTTONS);
  }
}

// ========== TOUCHSCREEN UI ==========
void drawStaticLayout() {
  tft.fillScreen(ST77XX_BLACK);
  resetDisplayedLogsCache();

  tft.fillRect(0, 0, SCREEN_W, 22, 0x0011);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(6, 4);
  tft.print("MD89MW-CAN 500kbps");
  tft.setCursor(190, 4);
  tft.print("START toggles");
  tft.setCursor(200, 14);
  tft.print("touch selects");

  tft.drawRect(INFO_X, INFO_Y, INFO_W, INFO_H, 0x03EF);
  tft.drawFastHLine(0, LOG_Y - 2, SCREEN_W, 0x03EF);
  tft.setCursor(6, LOG_Y - 14);
  tft.setTextColor(ST77XX_WHITE);
  tft.print("CAN log");
}

void drawSummaryLine() {
  tft.fillRect(6, 13, 170, 8, 0x0011);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(6, 14);
  tft.print("S1:");
  tft.print(servo1.currentPos);
  tft.print("  S2:");
  tft.print(servo2.currentPos);
}

void drawServoButton(int servoNum, int x, int y, int w, int h) {
  ServoState& servo = getServoByNumber(servoNum);
  bool selected = (selectedServo == servoNum);

  uint16_t fillColor = selected ? 0x05EF : 0x3186;
  if (servo.active) {
    fillColor = selected ? 0xFD20 : 0xC3A0;
  }

  tft.fillRect(x, y, w, h, fillColor);
  tft.drawRect(x, y, w, h, ST77XX_WHITE);
  if (selected) {
    tft.drawRect(x + 2, y + 2, w - 4, h - 4, ST77XX_YELLOW);
  }

  tft.setTextColor(ST77XX_BLACK);
  tft.setTextSize(3);
  tft.setCursor(x + 40, y + 12);
  tft.print("S");
  tft.print(servoNum);

  tft.setTextSize(1);
  tft.setCursor(x + 10, y + 54);
  tft.print(servo.starting ? "PING" : (servo.active ? "RUN" : "IDLE"));
  tft.setCursor(x + w - 42, y + 54);
  tft.print(servo.linkHealthy ? "OK" : "WAIT");
}

void drawServoSelectors() {
  drawServoButton(1, BTN_SEL1_X, BTN_SEL1_Y, BTN_SEL1_W, BTN_SEL1_H);
  drawServoButton(2, BTN_SEL2_X, BTN_SEL2_Y, BTN_SEL2_W, BTN_SEL2_H);
}

void drawSelectedServoPanel() {
  ServoState& servo = getSelectedServo();
  int32_t lag = servoLag(servo);

  tft.fillRect(INFO_X + 1, INFO_Y + 1, INFO_W - 2, INFO_H - 2, ST77XX_BLACK);

  tft.setTextSize(2);
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(6, INFO_Y + 4);
  tft.print("S");
  tft.print(selectedServo);
  tft.print(servo.starting ? " PING" : (servo.active ? " RUN" : " IDLE"));

  tft.setTextSize(1);
  tft.setTextColor(servo.linkHealthy ? ST77XX_GREEN : ST77XX_RED);
  tft.setCursor(150, INFO_Y + 9);
  tft.print(servo.linkHealthy ? "CAN OK" : "NO RESP");

  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(6, INFO_Y + 26);
  tft.print("Pos:");
  tft.print(servo.currentPos);
  tft.setCursor(110, INFO_Y + 26);
  tft.print("Cmd:");
  tft.print(servo.commandedPos);

  tft.setTextColor(lag == 0 ? ST77XX_GREEN : ST77XX_YELLOW);
  tft.setCursor(230, INFO_Y + 26);
  tft.print("Lag:");
  tft.print(lag);
}

void drawLogLine(int line, int idx) {
  int y = LOG_Y + (line * 12);
  tft.fillRect(LOG_X, y, LOG_W, 11, ST77XX_BLACK);

  if (idx < 0 || idx >= logCount) {
    displayedLogIndices[line] = -1;
    displayedLogIds[line] = 0xFFFFFFFFUL;
    displayedLogPositions[line] = 0xFFFFFFFFUL;
    displayedLogTimestamps[line] = 0xFFFFFFFFUL;
    displayedLogUsed[line] = false;
    return;
  }

  const CANMessage& logEntry = getCANLogAt(idx);
  uint32_t pos = parsePos(logEntry.data);
  char buffer[48];
  snprintf(buffer, sizeof(buffer), "N%03lX P:%lu T:%lu",
           (unsigned long)logEntry.id,
           (unsigned long)pos,
           (unsigned long)logEntry.timestamp);

  tft.setTextSize(1);
  tft.setTextColor(ST77XX_GREEN);
  tft.setCursor(6, y);
  tft.print(buffer);

  displayedLogIndices[line] = idx;
  displayedLogIds[line] = logEntry.id;
  displayedLogPositions[line] = pos;
  displayedLogTimestamps[line] = logEntry.timestamp;
  displayedLogUsed[line] = true;
}

void drawLogs() {
  int visibleStart = (logCount > TFT_LOG_LINES) ? (logCount - TFT_LOG_LINES) : 0;
  for (int line = 0; line < TFT_LOG_LINES; line++) {
    int idx = visibleStart + line;
    bool hasEntry = idx < logCount;

    if (!hasEntry) {
      if (displayedLogUsed[line]) {
        drawLogLine(line, -1);
      }
      continue;
    }

    const CANMessage& logEntry = getCANLogAt(idx);
    uint32_t pos = parsePos(logEntry.data);
    if (!displayedLogUsed[line] ||
        displayedLogIndices[line] != idx ||
        displayedLogIds[line] != logEntry.id ||
        displayedLogPositions[line] != pos ||
        displayedLogTimestamps[line] != logEntry.timestamp) {
      drawLogLine(line, idx);
    }
  }
}

void refreshDisplayDirtyFlags() {
  ServoState& selected = getSelectedServo();
  int32_t lag = servoLag(selected);

  if (servo1.currentPos != displayCache.s1Pos || servo2.currentPos != displayCache.s2Pos) {
    markDisplayDirty(DIRTY_SUMMARY);
  }

  if (selectedServo != displayCache.selectedServo ||
      selected.currentPos != displayCache.selectedCurrent ||
      selected.commandedPos != displayCache.selectedCommanded ||
      lag != displayCache.selectedLag ||
      selected.active != displayCache.selectedActive ||
      selected.starting != displayCache.selectedStarting ||
      selected.linkHealthy != displayCache.selectedLinkHealthy) {
    markDisplayDirty(DIRTY_SELECTED);
  }

  if (selectedServo != displayCache.selectedServo ||
      servo1.active != displayCache.s1Active ||
      servo2.active != displayCache.s2Active ||
      servo1.starting != displayCache.s1Starting ||
      servo2.starting != displayCache.s2Starting ||
      servo1.linkHealthy != displayCache.s1LinkHealthy ||
      servo2.linkHealthy != displayCache.s2LinkHealthy) {
    markDisplayDirty(DIRTY_BUTTONS);
  }

  if (canLogVersion != displayCache.canLogVersion) {
    markDisplayDirty(DIRTY_LOGS);
  }
}

void updateDisplayCache() {
  ServoState& selected = getSelectedServo();
  displayCache.s1Pos = servo1.currentPos;
  displayCache.s2Pos = servo2.currentPos;
  displayCache.selectedCurrent = selected.currentPos;
  displayCache.selectedCommanded = selected.commandedPos;
  displayCache.selectedLag = servoLag(selected);
  displayCache.selectedServo = selectedServo;
  displayCache.selectedActive = selected.active;
  displayCache.selectedStarting = selected.starting;
  displayCache.selectedLinkHealthy = selected.linkHealthy;
  displayCache.s1Active = servo1.active;
  displayCache.s2Active = servo2.active;
  displayCache.s1Starting = servo1.starting;
  displayCache.s2Starting = servo2.starting;
  displayCache.s1LinkHealthy = servo1.linkHealthy;
  displayCache.s2LinkHealthy = servo2.linkHealthy;
  displayCache.canLogVersion = canLogVersion;
}

void updateDisplay() {
  if (!displayInitialized) {
    drawStaticLayout();
    displayInitialized = true;
    markDisplayDirty(DIRTY_SUMMARY | DIRTY_BUTTONS | DIRTY_SELECTED | DIRTY_LOGS);
  }

  refreshDisplayDirtyFlags();
  if (!displayDirty) {
    return;
  }

  uint8_t flags = displayDirtyFlags;
  if (flags & DIRTY_STATIC) {
    drawStaticLayout();
    flags |= DIRTY_SUMMARY | DIRTY_BUTTONS | DIRTY_SELECTED | DIRTY_LOGS;
  }
  if (flags & DIRTY_SUMMARY) {
    drawSummaryLine();
  }
  if (flags & DIRTY_BUTTONS) {
    drawServoSelectors();
  }
  if (flags & DIRTY_SELECTED) {
    drawSelectedServoPanel();
  }
  if (flags & DIRTY_LOGS) {
    drawLogs();
  }

  updateDisplayCache();
  displayDirty = false;
  displayDirtyFlags = DIRTY_NONE;
}

// ========== TOUCHSCREEN HANDLING ==========
void handleTouch() {
  if (!ts.touched() || (millis() - lastTouchTime) < TOUCH_DEBOUNCE_MS) {
    return;
  }

  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  TS_Point p = ts.getPoint();
  SPI.endTransaction();

  int x = map(p.x, 3703, 463, 0, SCREEN_W);
  int y = map(p.y, 3110, 528, 0, SCREEN_H);

  Serial.print("[TOUCH] X:");
  Serial.print(x);
  Serial.print(" Y:");
  Serial.println(y);

  if (isInsideRect(x, y, BTN_SEL1_X, BTN_SEL1_Y, BTN_SEL1_W, BTN_SEL1_H)) {
    selectedServo = 1;
    Serial.println("[SELECT] Servo 1");
    markDisplayDirty(DIRTY_BUTTONS | DIRTY_SELECTED);
    lastTouchTime = millis();
    return;
  }

  if (isInsideRect(x, y, BTN_SEL2_X, BTN_SEL2_Y, BTN_SEL2_W, BTN_SEL2_H)) {
    selectedServo = 2;
    Serial.println("[SELECT] Servo 2");
    markDisplayDirty(DIRTY_BUTTONS | DIRTY_SELECTED);
    lastTouchTime = millis();
    return;
  }

  lastTouchTime = millis();
}

void handleStartButton() {
  bool reading = digitalRead(START_BUTTON_PIN);
  unsigned long now = millis();

  if (reading != lastStartButtonReading) {
    lastStartButtonChange = now;
    lastStartButtonReading = reading;
  }

  if ((now - lastStartButtonChange) < START_DEBOUNCE_MS) {
    return;
  }

  if (reading != startButtonStableState) {
    startButtonStableState = reading;
    if (startButtonStableState == LOW) {
      Serial.print("[START] Toggle selected servo ");
      Serial.println(selectedServo);
      toggleServoMotion(selectedServo);
    }
  }
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
        #log { background: #111; color: #0f0; padding: 10px; height: 300px; overflow-y: auto; font-family: monospace; font-size: 12px; white-space: pre-wrap; }
    </style>
</head>
<body>
    <h2>Dual MD89MW-CAN Servo (500kbps)</h2>
    <p>On-device UI: touch S1 or S2 on the TFT, then press the physical START button.</p>

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
                document.getElementById('log').textContent = d.trim() ? d : 'Waiting...';
                document.getElementById('log').scrollTop = 9999;
            });
        }, 500);

        function toggleServo(n) { fetch('/toggle?s=' + n); }
        function clearLogs() { fetch('/clear'); }
    </script>
</body>
</html>
)rawliteral";

void setupWebServer() {
  server.on("/", []() {
    server.send(200, "text/html", index_html);
  });

  server.on("/status", []() {
    String json = "{";
    json += "\"selected\":" + String(selectedServo);
    json += ",\"s1_active\":" + String(servo1.active ? "true" : "false");
    json += ",\"s1_pos\":" + String(servo1.currentPos);
    json += ",\"s1_cmd\":" + String(servo1.commandedPos);
    json += ",\"s1_lag\":" + String(servoLag(servo1));
    json += ",\"s2_active\":" + String(servo2.active ? "true" : "false");
    json += ",\"s2_pos\":" + String(servo2.currentPos);
    json += ",\"s2_cmd\":" + String(servo2.commandedPos);
    json += ",\"s2_lag\":" + String(servoLag(servo2));
    json += "}";
    server.send(200, "application/json", json);
  });

  server.on("/toggle", []() {
    int servoNum = server.arg("s").toInt();
    if (servoNum == 1 || servoNum == 2) {
      toggleServoMotion(servoNum);
      server.send(200, "text/plain", "OK");
    } else {
      server.send(400, "text/plain", "Invalid servo");
    }
  });

  server.on("/clear", []() {
    clearLogs();
    server.send(200, "text/plain", "OK");
  });

  server.on("/logs", []() {
    String html;
    for (int i = (logCount > 30 ? logCount - 30 : 0); i < logCount; i++) {
      const CANMessage& logEntry = getCANLogAt(i);
      uint32_t pos = parsePos(logEntry.data);
      html += "N";
      html += String(logEntry.id, HEX);
      html += " Pos:";
      html += String(pos);
      html += " T:";
      html += String(logEntry.timestamp);
      html += "\n";
    }
    server.send(200, "text/plain", html);
  });

  server.begin();
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║ Dual MD89MW-CAN Servo Controller       ║");
  Serial.println("║ 500kbps | Touch Select + START Button  ║");
  Serial.println("╚════════════════════════════════════════╝\n");

  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
  lastStartButtonReading = digitalRead(START_BUTTON_PIN);
  startButtonStableState = lastStartButtonReading;

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
    while (1) {
      delay(1000);
    }
  }

  WiFi.softAP("ESP32-S3-CAN", "password123");
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());

  setupWebServer();
  Serial.println("[HTTP] Ready");
  Serial.println("[UI] Touch S1/S2, press START button to toggle motion\n");

  markDisplayDirty(DIRTY_ALL);
  updateDisplay();
}

// ========== MAIN LOOP ==========
void loop() {
  server.handleClient();
  handleTouch();
  handleStartButton();

  processCANReceive(0);
  refreshServoLinkState();

  serviceServoMotion(1);
  serviceServoMotion(2);

  processCANReceive(0);
  refreshServoLinkState();

  if (millis() - lastTftUpdate >= TFT_UPDATE_INTERVAL_MS) {
    updateDisplay();
    lastTftUpdate = millis();
  }
}
