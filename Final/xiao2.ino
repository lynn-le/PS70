#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <EEPROM.h>

// ============= CONFIGURATION =============
const char* WIFI_SSID = "MAKERSPACE";
const char* WIFI_PASSWORD = "12345678";

// Pins
const int RFID_SS  = D7;
const int RFID_RST = D6;
const int LED_PIN = D4;

// Serial1 for communication with Weight Board (RX=D0, TX=D1)
#define WeightSerial Serial1

const float PLACED_THRESHOLD_G = 15.0f;
const int CAL_VAL_EEPROM_ADDRESS = 0;

// ============= WEB SERVER & WEBSOCKET =============
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ============= CALIBRATION STATE =============
enum CalibrationState {
  CAL_IDLE,
  CAL_WAITING_TARE,
  CAL_TARE_COMPLETE,
  CAL_WAITING_MASS,
  CAL_COMPLETE
};

struct CalibrationData {
  CalibrationState state;
  float knownMass;
  float newCalFactor;
  String message;
} calibrationData = {CAL_IDLE, 0, 0, ""};

// Deferred calibration: set in WS callback, executed in loop()
volatile bool pendingCalibrationCalculate = false;
volatile float pendingCalWeight = 0.0f;

// Cached cal factor to avoid blocking serial reads inside getInventoryJson()
float cachedCalFactor = 0.0f;

// ============= LED PATTERNS =============
enum LedPattern {
  LED_OFF,
  LED_SCAN_SUCCESS,
  LED_SCAN_FAIL,
  LED_WEIGHT_REGISTERED,
  LED_CHECKOUT,
  LED_WIFI_CONNECTING,
  LED_WIFI_CONNECTED,
  LED_CALIBRATING
};

class StatusLed {
  int pin;
  LedPattern currentPattern;
  unsigned long patternStart;

public:
  StatusLed(int inputPin) : pin(inputPin), currentPattern(LED_OFF), patternStart(0) {}

  void begin() {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }

  void setPattern(LedPattern pattern) {
    currentPattern = pattern;
    patternStart = millis();
  }

  void update() {
    unsigned long elapsed = millis() - patternStart;

    switch (currentPattern) {
      case LED_OFF:
        digitalWrite(pin, LOW);
        break;

      case LED_SCAN_SUCCESS:
        if (elapsed < 200) {
          digitalWrite(pin, HIGH);
        } else {
          digitalWrite(pin, LOW);
          currentPattern = LED_OFF;
        }
        break;

      case LED_SCAN_FAIL:
        if (elapsed < 600) {
          digitalWrite(pin, (elapsed / 100) % 2 == 0 ? HIGH : LOW);
        } else {
          digitalWrite(pin, LOW);
          currentPattern = LED_OFF;
        }
        break;

      case LED_WEIGHT_REGISTERED:
        if (elapsed < 1500) {
          digitalWrite(pin, HIGH);
        } else {
          digitalWrite(pin, LOW);
          currentPattern = LED_OFF;
        }
        break;

      case LED_CHECKOUT:
        if (elapsed < 300 || (elapsed >= 400 && elapsed < 700)) {
          digitalWrite(pin, HIGH);
        } else if (elapsed < 800) {
          digitalWrite(pin, LOW);
        } else {
          digitalWrite(pin, LOW);
          currentPattern = LED_OFF;
        }
        break;

      case LED_WIFI_CONNECTING:
        digitalWrite(pin, (elapsed / 500) % 2 == 0 ? HIGH : LOW);
        break;

      case LED_WIFI_CONNECTED:
        digitalWrite(pin, HIGH);
        break;

      case LED_CALIBRATING:
        digitalWrite(pin, (elapsed / 250) % 2 == 0 ? HIGH : LOW);
        break;
    }
  }
};

// ============= WEIGHT SENSOR =============
class WeightSensor {
  const char* label;
  float lastWeight = 0.0f;  // cached, updated in loop() only

public:
  WeightSensor(const char* inputLabel) {
    label = inputLabel;
  }

  void clearSerialBuffer() {
    while (WeightSerial.available()) {
      WeightSerial.read();
    }
  }

  void begin() {
    Serial.println("Waiting for Weight Sensor Board...");
    unsigned long start = millis();
    bool ready = false;
    
    while (millis() - start < 5000) {
      if (WeightSerial.available()) {
        String response = WeightSerial.readStringUntil('\n');
        if (response.indexOf("READY") >= 0) {
          ready = true;
          break;
        }
      }
      delay(100);
    }
    
    if (ready) {
      Serial.println("SUCCESS: Weight sensor ready");
    } else {
      Serial.println("WARNING: Weight sensor not responding!");
    }
  }

  float getWeight() {
    clearSerialBuffer();
    delay(50);
    
    WeightSerial.println("w");
    
    unsigned long start = millis();
    while (millis() - start < 5000) {
      if (WeightSerial.available()) {
        String response = WeightSerial.readStringUntil('\n');
        response.trim();
        
        if (response.startsWith("WEIGHT:")) {
          lastWeight = response.substring(7).toFloat();
          return lastWeight;
        }
      }
      delay(10);
    }
    
    return lastWeight;  // return last known value on timeout
  }

  float getLastWeight() {
    return lastWeight;
  }

  bool isObjectPresent() {
    clearSerialBuffer();
    delay(50);
    
    WeightSerial.println("CHECK_PRESENT");
    
    unsigned long start = millis();
    while (millis() - start < 1000) {
      if (WeightSerial.available()) {
        String response = WeightSerial.readStringUntil('\n');
        response.trim();
        
        if (response == "PRESENT:YES") {
          return true;
        } else if (response == "PRESENT:NO") {
          return false;
        }
      }
      delay(10);
    }
    
    return false;
  }

  bool tare() {
    clearSerialBuffer();
    WeightSerial.println("t");

    unsigned long start = millis();
    while (millis() - start < 5000) {
      if (WeightSerial.available()) {
        String response = WeightSerial.readStringUntil('\n');
        response.trim();

        Serial.print("Tare response: ");
        Serial.println(response);

        if (response == "TARE_COMPLETE" || response == "TARE:COMPLETE") {
          return true;
        }
      }
      delay(10);
    }

    Serial.println("Tare timed out");
    return false;
  }

  void refreshDataSet() {
    clearSerialBuffer();
    WeightSerial.println("REFRESH_DATASET");
    delay(500);
  }

  float getNewCalibration(float knownMass) {
    clearSerialBuffer();

    String cmd = "GET_NEW_CAL:" + String(knownMass, 2);
    Serial.print("Sending calibration command: ");
    Serial.println(cmd);

    WeightSerial.println(cmd);

    unsigned long start = millis();
    while (millis() - start < 5000) {
      if (WeightSerial.available()) {
        String response = WeightSerial.readStringUntil('\n');
        response.trim();

        Serial.print("Calibration response: ");
        Serial.println(response);

        if (response.startsWith("NEW_CAL:")) {
          float cal = response.substring(8).toFloat();
          Serial.print("Parsed cal factor: ");
          Serial.println(cal, 4);
          return cal;
        }
      }
      delay(10);
    }

    Serial.println("Calibration timed out");
    return 0.0f;
  }

  void setCalFactor(float calFactor) {
    clearSerialBuffer();
    String cmd = "SET_CAL:" + String(calFactor, 4);
    WeightSerial.println(cmd);
    delay(100);
  }

  float getCalFactor() {
    clearSerialBuffer();
    WeightSerial.println("GET_CAL");
    
    unsigned long start = millis();
    while (millis() - start < 1000) {
      if (WeightSerial.available()) {
        String response = WeightSerial.readStringUntil('\n');
        response.trim();
        
        if (response.startsWith("CAL:")) {
          return response.substring(4).toFloat();
        }
      }
      delay(10);
    }
    return 0.0f;
  }

  float getCurrentWeight() {
    WeightSerial.println("GET_CURRENT");
    
    unsigned long start = millis();
    while (millis() - start < 1000) {
      if (WeightSerial.available()) {
        String response = WeightSerial.readStringUntil('\n');
        response.trim();
        
        if (response.startsWith("CURRENT:")) {
          return response.substring(8).toFloat();
        }
      }
      delay(10);
    }
    return 0.0f;
  }

  const char* getLabel() {
    return label;
  }
};

// ============= RFID SYSTEM =============
class RfidSystem {
  MFRC522 rfid;

  String uidToString(MFRC522::Uid *uid) {
    String out = "";
    for (byte i = 0; i < uid->size; i++) {
      if (uid->uidByte[i] < 0x10) out += "0";
      out += String(uid->uidByte[i], HEX);
    }
    out.toUpperCase();
    return out;
  }

public:
  RfidSystem(int ssPin, int rstPin) : rfid(ssPin, rstPin) {}

  void begin() {
    SPI.begin();
    delay(50);
    
    rfid.PCD_Init();
    delay(50);
    
    byte version = rfid.PCD_ReadRegister(rfid.VersionReg);
    Serial.print("SUCCESS: RFID ready. Version: 0x");
    Serial.println(version, HEX);
  }

  bool readTag(String &uidOut) {
    if (!rfid.PICC_IsNewCardPresent()) return false;
    if (!rfid.PICC_ReadCardSerial()) return false;

    uidOut = uidToString(&rfid.uid);

    rfid.PICC_HaltA();
    rfid.PCD_StopCrypto1();
    return true;
  }
};

// ============= ITEM DATABASE =============
struct ItemRecord {
  String uid;
  String name;
  String color;
  String brand;
  float initialWeight;
  float currentWeight;
  bool inUse;
  bool waitingForReturn;
  unsigned long lastCheckout;
  unsigned long lastReturn;

  float getPercentRemaining() const {
    if (initialWeight <= 0) return 0;
    return (currentWeight / initialWeight) * 100.0f;
  }
};

ItemRecord items[] = {
  {"0143733C", "Red Polish", "#DC143C", "OPI", 47.0f, 47.0f, false, false, 0, 0},
  {"D14E733C", "Top Coat", "#FFFFFF", "Essie", 62.0f, 62.0f, false, false, 0, 0}
};

const int ITEM_COUNT = sizeof(items) / sizeof(items[0]);

// Forward declarations
void notifyClients();

// ============= INVENTORY SYSTEM =============
class InventorySystem {
  ItemRecord* items;
  int itemCount;
  WeightSensor* sensor;
  StatusLed* led;

public:
  InventorySystem(ItemRecord* inputItems, int inputItemCount, 
                  WeightSensor* inputSensor, 
                  StatusLed* inputLed) {
    items = inputItems;
    itemCount = inputItemCount;
    sensor = inputSensor;
    led = inputLed;
  }

  int findItemIndexByUid(const String &uid) {
    for (int i = 0; i < itemCount; i++) {
      if (items[i].uid == uid) return i;
    }
    return -1;
  }

  ItemRecord* getItems() { return items; }
  int getItemCount() { return itemCount; }
  WeightSensor* getSensor() { return sensor; }

  String getInventoryJson() {
    StaticJsonDocument<3072> doc;
    JsonArray itemsArray = doc.createNestedArray("items");

    for (int i = 0; i < itemCount; i++) {
      JsonObject item = itemsArray.createNestedObject();
      item["uid"] = items[i].uid;
      item["name"] = items[i].name;
      item["color"] = items[i].color;
      item["brand"] = items[i].brand;
      item["currentWeight"] = round(items[i].currentWeight * 100) / 100.0;
      item["initialWeight"] = round(items[i].initialWeight * 100) / 100.0;
      item["percentRemaining"] = round(items[i].getPercentRemaining() * 10) / 10.0;
      item["inUse"] = items[i].inUse;
      item["waitingForReturn"] = items[i].waitingForReturn;
    }

    JsonObject sensorObj = doc.createNestedObject("sensor");
    sensorObj["label"] = sensor->getLabel();
    sensorObj["currentWeight"] = round(sensor->getLastWeight() * 100) / 100.0;

    JsonObject calObj = doc.createNestedObject("calibration");
    calObj["state"] = calibrationData.state;
    calObj["message"] = calibrationData.message;
    calObj["newCalFactor"] = calibrationData.newCalFactor;
    calObj["currentCalFactor"] = cachedCalFactor;

    String output;
    serializeJson(doc, output);
    return output;
  }

  void handleScan(const String &uid) {
    Serial.print("Scanned UID: ");
    Serial.println(uid);

    led->setPattern(LED_SCAN_SUCCESS);

    int itemIndex = findItemIndexByUid(uid);

    if (itemIndex == -1) {
      Serial.println("ERROR: Tag not found in database");
      led->setPattern(LED_SCAN_FAIL);
      return;
    }

    ItemRecord &item = items[itemIndex];

    if (!item.inUse && !item.waitingForReturn) {
      item.inUse = true;
      item.lastCheckout = millis();
      Serial.print("SUCCESS: ");
      Serial.print(item.name);
      Serial.println(" checked out");
      led->setPattern(LED_CHECKOUT);
      notifyClients();
      return;
    }

    if (item.inUse) {
      item.inUse = false;
      item.waitingForReturn = true;
      Serial.print("SUCCESS: ");
      Serial.print(item.name);
      Serial.println(" marked for return. Place on sensor.");
      notifyClients();
      return;
    }

    if (item.waitingForReturn) {
      Serial.print("INFO: ");
      Serial.print(item.name);
      Serial.println(" is already waiting to be placed on sensor");
    }
  }

  void checkForReturnedItems() {
    for (int i = 0; i < itemCount; i++) {
      ItemRecord &item = items[i];

      if (!item.waitingForReturn) continue;

      if (sensor->isObjectPresent()) {
        Serial.println("Item detected! Getting weight from sensor...");

        delay(500);
        
        float oldWeight = item.currentWeight;
        item.currentWeight = sensor->getWeight();
        item.waitingForReturn = false;
        item.lastReturn = millis();

        led->setPattern(LED_WEIGHT_REGISTERED);

        Serial.print("SUCCESS: ");
        Serial.print(item.name);
        Serial.println(" returned");

        if (item.getPercentRemaining() < 20.0f) {
          Serial.println("  ⚠ WARNING: Low inventory!");
        }

        notifyClients();
      }
    }
  }
};

// ============= GLOBAL OBJECTS =============
StatusLed led(LED_PIN);
WeightSensor sensor("Weight Sensor");
RfidSystem rfid(RFID_SS, RFID_RST);
InventorySystem inventory(items, ITEM_COUNT, &sensor, &led);

// ============= WEBSOCKET HANDLERS =============
void notifyClients() {
  if (ws.count() > 0) {
    String json = inventory.getInventoryJson();
    ws.textAll(json);
  }
}

void handleWebCommand(String command, String value) {
  if (command == "tare") {
    if (sensor.tare()) {
      Serial.println("Tare complete");
    } else {
      Serial.println("Tare failed");
    }
    notifyClients();
  }
  else if (command == "start_calibration") {
    led.setPattern(LED_CALIBRATING);
    calibrationData.state = CAL_WAITING_TARE;
    calibrationData.message = "Remove all weight from sensor, then click Tare";
    Serial.println("Calibration started");
    notifyClients();
  }
  else if (command == "calibration_tare") {
    if (sensor.tare()) {
      calibrationData.state = CAL_TARE_COMPLETE;
      calibrationData.message = "Tare complete. Place known mass and enter weight.";
      Serial.println("Calibration tare complete");
    } else {
      calibrationData.message = "Tare failed or timed out. Check weight board response.";
      Serial.println("Calibration tare failed");
    }
    notifyClients();
  }
  else if (command == "calibration_calculate") {
    float mass = value.toFloat();
    if (mass > 0) {
      // Do NOT block here — schedule work for loop() to avoid WDT crash
      pendingCalWeight = mass;
      pendingCalibrationCalculate = true;
      calibrationData.state = CAL_WAITING_MASS;
      calibrationData.message = "Calculating calibration factor, please wait...";
      Serial.println("Calibration calculate queued for loop()");
      notifyClients();
    }
  }
  else if (command == "calibration_save") {
    if (calibrationData.newCalFactor != 0) {
      sensor.setCalFactor(calibrationData.newCalFactor);
      EEPROM.begin(512);
      EEPROM.put(CAL_VAL_EEPROM_ADDRESS, calibrationData.newCalFactor);
      EEPROM.commit();
      
      calibrationData.state = CAL_IDLE;
      calibrationData.message = "Calibration saved to EEPROM!";
      led.setPattern(LED_WEIGHT_REGISTERED);
      Serial.println("Calibration saved to EEPROM");
    }
    notifyClients();
  }
  else if (command == "calibration_cancel") {
    calibrationData.state = CAL_IDLE;
    calibrationData.message = "";
    calibrationData.newCalFactor = 0;
    led.setPattern(LED_OFF);
    Serial.println("Calibration cancelled");
    notifyClients();
  }
  else if (command == "set_cal_factor") {
    float newCal = value.toFloat();
    if (newCal != 0) {
      sensor.setCalFactor(newCal);
      EEPROM.begin(512);
      EEPROM.put(CAL_VAL_EEPROM_ADDRESS, newCal);
      EEPROM.commit();
      Serial.print("Manual calibration factor set: ");
      Serial.println(newCal, 4);
    }
    notifyClients();
  }
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, 
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("WebSocket client #%u connected\n", client->id());
    String json = inventory.getInventoryJson();
    client->text(json);
  } 
  else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("WebSocket client #%u disconnected\n", client->id());
  }
  else if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      String message = "";
      for (size_t i = 0; i < len; i++) {
        message += (char)data[i];
      }
      
      if (message == "get_inventory") {
        String json = inventory.getInventoryJson();
        client->text(json);
      }
      else if (message.startsWith("CMD:")) {
        int colonPos = message.indexOf(':', 4);
        String command;
        String value = "";

        if (colonPos > 0) {
          command = message.substring(4, colonPos);
          value = message.substring(colonPos + 1);
        } else {
          command = message.substring(4);
        }

        handleWebCommand(command, value);
      }
    }
  }
}

// ============= HTML PAGE WITH WEB CONTROLS =============
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Xiao2 Inventory System</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            background: #fafafa;
            color: #1a1a1a;
            padding: 40px 20px;
        }
        .container { max-width: 1400px; margin: 0 auto; }
        .header { margin-bottom: 50px; }
        .header h1 { font-size: 28px; font-weight: 400; margin-bottom: 8px; }
        .header-meta { display: flex; gap: 20px; font-size: 14px; color: #666; flex-wrap: wrap; }
        .status { display: inline-flex; align-items: center; gap: 6px; font-size: 13px; font-weight: 500; }
        .status-dot { width: 6px; height: 6px; border-radius: 50%; animation: pulse 2s infinite; }
        @keyframes pulse { 0%, 100% { opacity: 1; } 50% { opacity: 0.5; } }
        .status.connected .status-dot { background: #22c55e; }
        .status.disconnected .status-dot { background: #ef4444; }
        
        .controls-panel {
            background: white;
            border: 1px solid #e5e5e5;
            border-radius: 8px;
            padding: 24px;
            margin-bottom: 20px;
        }
        .controls-panel h2 { font-size: 18px; font-weight: 500; margin-bottom: 20px; }
        .controls-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 12px; margin-bottom: 20px; }
        .btn {
            padding: 12px 20px;
            border: none;
            border-radius: 6px;
            font-size: 14px;
            font-weight: 500;
            cursor: pointer;
            transition: all 0.2s;
        }
        .btn-primary { background: #1a1a1a; color: white; }
        .btn-primary:hover { background: #333; }
        .btn-secondary { background: #f5f5f5; color: #1a1a1a; }
        .btn-secondary:hover { background: #e5e5e5; }
        .btn-success { background: #22c55e; color: white; }
        .btn-success:hover { background: #16a34a; }
        .btn-danger { background: #ef4444; color: white; }
        .btn-danger:hover { background: #dc2626; }
        
        .calibration-modal {
            display: none;
            position: fixed;
            top: 0;
            left: 0;
            width: 100%;
            height: 100%;
            background: rgba(0,0,0,0.5);
            z-index: 1000;
            align-items: center;
            justify-content: center;
        }
        .calibration-modal.active { display: flex; }
        .modal-content {
            background: white;
            border-radius: 12px;
            padding: 32px;
            max-width: 500px;
            width: 90%;
            box-shadow: 0 20px 25px -5px rgba(0,0,0,0.1);
        }
        .modal-header { font-size: 20px; font-weight: 500; margin-bottom: 20px; }
        .modal-body { margin-bottom: 24px; }
        .modal-message {
            padding: 16px;
            background: #f0f9ff;
            border-left: 4px solid #3b82f6;
            border-radius: 4px;
            margin-bottom: 16px;
            font-size: 14px;
        }
        .input-group { margin-bottom: 16px; }
        .input-group label { display: block; margin-bottom: 8px; font-size: 14px; font-weight: 500; }
        .input-group input {
            width: 100%;
            padding: 12px;
            border: 1px solid #e5e5e5;
            border-radius: 6px;
            font-size: 14px;
        }
        .modal-actions { display: flex; gap: 12px; }
        .modal-actions .btn { flex: 1; }
        
        .inventory-grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(280px, 1fr)); gap: 16px; margin-bottom: 20px; }
        .item-card { background: white; border: 1px solid #e5e5e5; border-radius: 8px; padding: 20px; transition: all 0.2s; }
        .item-card:hover { border-color: #d4d4d4; box-shadow: 0 2px 8px rgba(0,0,0,0.04); }
        .item-header { display: flex; gap: 12px; margin-bottom: 16px; }
        .color-swatch { width: 32px; height: 32px; border-radius: 6px; border: 1px solid #e5e5e5; }
        .item-info { flex: 1; }
        .item-name { font-size: 15px; font-weight: 500; margin-bottom: 2px; }
        .item-brand { font-size: 13px; color: #737373; }
        .progress-container { margin-bottom: 16px; }
        .progress-label { display: flex; justify-content: space-between; margin-bottom: 8px; font-size: 12px; color: #737373; }
        .progress-percentage { font-weight: 500; color: #1a1a1a; }
        .progress-bar { width: 100%; height: 4px; background: #f5f5f5; border-radius: 2px; overflow: hidden; }
        .progress-fill { height: 100%; transition: width 0.3s; }
        .progress-fill.high { background: #22c55e; }
        .progress-fill.medium { background: #f59e0b; }
        .progress-fill.low { background: #ef4444; }
        .item-stats { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; margin-bottom: 12px; }
        .stat { padding: 12px; background: #fafafa; border-radius: 6px; }
        .stat-label { font-size: 11px; color: #737373; text-transform: uppercase; letter-spacing: 0.5px; margin-bottom: 4px; }
        .stat-value { font-size: 16px; font-weight: 500; }
        .item-footer { display: flex; justify-content: space-between; padding-top: 12px; border-top: 1px solid #f5f5f5; }
        .item-status { font-size: 12px; font-weight: 500; padding: 4px 10px; border-radius: 12px; }
        .item-status.available { background: #f0fdf4; color: #15803d; }
        .item-status.checked-out { background: #fef3c7; color: #a16207; }
        .item-status.awaiting { background: #eff6ff; color: #1e40af; }
        .sensor-label { font-size: 12px; color: #a3a3a3; }
        
        .sensors-panel { background: white; border: 1px solid #e5e5e5; border-radius: 8px; padding: 24px; }
        .sensors-panel h2 { font-size: 18px; font-weight: 500; margin-bottom: 20px; }
        .sensors-grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(200px, 1fr)); gap: 16px; }
        .sensor { padding: 16px; background: #fafafa; border-radius: 6px; }
        .sensor-name { font-size: 13px; color: #737373; margin-bottom: 8px; text-transform: uppercase; letter-spacing: 0.5px; }
        .sensor-weight { font-size: 24px; font-weight: 500; }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>Xiao2 Inventory System</h1>
            <div class="header-meta">
                <span class="status" id="status"><span class="status-dot"></span><span id="statusText">Connecting...</span></span>
                <span>Last Updated: <span id="timestamp">--:--:--</span></span>
            </div>
        </div>

        <div class="controls-panel">
            <h2>System Controls</h2>
            <div class="controls-grid">
                <button class="btn btn-primary" onclick="sendCommand('tare')">Tare Sensor</button>
                <button class="btn btn-secondary" onclick="startCalibration()">Run Calibration</button>
                <button class="btn btn-secondary" onclick="showManualCalModal()">Set Cal Factor</button>
            </div>
        </div>

        <div class="inventory-grid" id="inventory">
            <div class="item-card"><div style="text-align: center; padding: 40px 0; color: #999;">Connecting...</div></div>
        </div>

        <div class="sensors-panel">
            <h2>Live Sensor Reading</h2>
            <div class="sensors-grid" id="sensorGrid">
                <div class="sensor"><div class="sensor-name">Weight Sensor</div><div class="sensor-weight">-- g</div></div>
            </div>
        </div>
    </div>

    <!-- Calibration Modal -->
    <div class="calibration-modal" id="calibrationModal">
        <div class="modal-content">
            <div class="modal-header">Sensor Calibration</div>
            <div class="modal-body">
                <div class="modal-message" id="calMessage">Starting calibration...</div>
                <div id="calStep1" style="display: none;">
                    <p style="margin-bottom: 16px;">Step 1: Remove all weight from the sensor.</p>
                    <button class="btn btn-primary" style="width: 100%;" onclick="calibrationTare()">Tare Sensor</button>
                </div>
                <div id="calStep2" style="display: none;">
                    <div class="input-group">
                        <label>Place known mass on sensor and enter weight (grams):</label>
                        <input type="number" id="knownMass" step="0.1" placeholder="e.g., 62.0">
                    </div>
                    <button class="btn btn-success" style="width: 100%;" onclick="calculateCalibration()">Calculate</button>
                </div>
                <div id="calStep3" style="display: none;">
                    <p style="margin-bottom: 16px;">New calibration factor: <strong id="newCalFactor">0</strong></p>
                    <div class="modal-actions">
                        <button class="btn btn-success" onclick="saveCalibration()">Save to EEPROM</button>
                        <button class="btn btn-secondary" onclick="closeCalModal()">Cancel</button>
                    </div>
                </div>
            </div>
            <button class="btn btn-danger" style="width: 100%; margin-top: 16px;" onclick="closeCalModal()">Close</button>
        </div>
    </div>

    <!-- Manual Cal Factor Modal -->
    <div class="calibration-modal" id="manualCalModal">
        <div class="modal-content">
            <div class="modal-header">Set Calibration Factor</div>
            <div class="modal-body">
                <div class="input-group">
                    <label>Current Factor: <span id="currentCalFactor">--</span></label>
                </div>
                <div class="input-group">
                    <label>Enter new calibration factor:</label>
                    <input type="number" id="manualCalValue" step="0.0001" placeholder="e.g., 271.65">
                </div>
                <button class="btn btn-success" style="width: 100%;" onclick="setManualCalFactor()">Save</button>
            </div>
            <button class="btn btn-secondary" style="width: 100%; margin-top: 16px;" onclick="closeManualCalModal()">Cancel</button>
        </div>
    </div>

    <script>
        let ws;
        let currentCalData = {};

        function updateTimestamp() {
            const now = new Date();
            document.getElementById('timestamp').textContent = 
                `${String(now.getHours()).padStart(2,'0')}:${String(now.getMinutes()).padStart(2,'0')}:${String(now.getSeconds()).padStart(2,'0')}`;
        }

        function sendCommand(command, value = '') {
            if (ws && ws.readyState === WebSocket.OPEN) {
                ws.send(`CMD:${command}:${value}`);
            }
        }

        function startCalibration() {
            sendCommand('start_calibration');
            document.getElementById('calibrationModal').classList.add('active');
        }

        function calibrationTare() {
            sendCommand('calibration_tare');
        }

        function calculateCalibration() {
            const mass = document.getElementById('knownMass').value;
            if (mass && parseFloat(mass) > 0) {
                sendCommand('calibration_calculate', mass);
            } else {
                alert('Please enter a valid mass value');
            }
        }

        function saveCalibration() {
            sendCommand('calibration_save');
            setTimeout(closeCalModal, 1000);
        }

        function closeCalModal() {
            sendCommand('calibration_cancel');
            document.getElementById('calibrationModal').classList.remove('active');
        }

        function showManualCalModal() {
            document.getElementById('currentCalFactor').textContent = currentCalData.currentCalFactor || '--';
            document.getElementById('manualCalModal').classList.add('active');
        }

        function setManualCalFactor() {
            const value = document.getElementById('manualCalValue').value;
            if (value && parseFloat(value) !== 0) {
                sendCommand('set_cal_factor', value);
                closeManualCalModal();
            } else {
                alert('Please enter a valid calibration factor');
            }
        }

        function closeManualCalModal() {
            document.getElementById('manualCalModal').classList.remove('active');
        }

        function updateCalibrationModal(calData) {
            currentCalData = calData;
            document.getElementById('calMessage').textContent = calData.message || 'Calibrating...';
            
            // Hide all steps
            document.getElementById('calStep1').style.display = 'none';
            document.getElementById('calStep2').style.display = 'none';
            document.getElementById('calStep3').style.display = 'none';
            
            // Show appropriate step
            if (calData.state === 1) { // CAL_WAITING_TARE
                document.getElementById('calStep1').style.display = 'block';
            } else if (calData.state === 2) { // CAL_TARE_COMPLETE
                document.getElementById('calStep2').style.display = 'block';
            } else if (calData.state === 4) { // CAL_COMPLETE
                document.getElementById('newCalFactor').textContent = calData.newCalFactor.toFixed(4);
                document.getElementById('calStep3').style.display = 'block';
            }
        }

        function connectWebSocket() {
            ws = new WebSocket(`ws://${window.location.hostname}/ws`);
            
            ws.onopen = () => {
                document.getElementById('status').className = 'status connected';
                document.getElementById('statusText').textContent = 'Connected';
                ws.send('get_inventory');
            };
            
            ws.onclose = () => {
                document.getElementById('status').className = 'status disconnected';
                document.getElementById('statusText').textContent = 'Disconnected';
                setTimeout(connectWebSocket, 3000);
            };
            
            ws.onmessage = (event) => {
                try {
                    const data = JSON.parse(event.data);
                    updateInventory(data);
                    if (data.calibration) {
                        updateCalibrationModal(data.calibration);
                    }
                    updateTimestamp();
                } catch (e) { console.error(e); }
            };
        }

        function updateInventory(data) {
            if (!data.items) return;
            
            document.getElementById('inventory').innerHTML = data.items.map(item => {
                let statusClass = item.inUse ? 'checked-out' : item.waitingForReturn ? 'awaiting' : 'available';
                let statusText = item.inUse ? 'In Use' : item.waitingForReturn ? 'Awaiting Return' : 'Available';
                let progressClass = item.percentRemaining < 20 ? 'low' : item.percentRemaining < 50 ? 'medium' : 'high';
                let colorStyle = item.color === '#FFFFFF' ? `background: ${item.color}; border: 1px solid #d4d4d4;` : `background: ${item.color}`;
                
                return `<div class="item-card">
                    <div class="item-header">
                        <div class="color-swatch" style="${colorStyle}"></div>
                        <div class="item-info"><div class="item-name">${item.name}</div><div class="item-brand">${item.brand}</div></div>
                    </div>
                    <div class="progress-container">
                        <div class="progress-label"><span>Remaining</span><span class="progress-percentage">${item.percentRemaining.toFixed(0)}%</span></div>
                        <div class="progress-bar"><div class="progress-fill ${progressClass}" style="width: ${item.percentRemaining}%"></div></div>
                    </div>
                    <div class="item-stats">
                        <div class="stat"><div class="stat-label">Current</div><div class="stat-value">${item.currentWeight.toFixed(1)}g</div></div>
                        <div class="stat"><div class="stat-value">${item.initialWeight.toFixed(1)}g</div></div>
                    </div>
                    <div class="item-footer">
                        <div class="item-status ${statusClass}">${statusText}</div>
                        <div class="sensor-label">Weight Sensor</div>
                    </div>
                </div>`;
            }).join('');
            
            if (data.sensor) {
                document.getElementById('sensorGrid').innerHTML = 
                    `<div class="sensor"><div class="sensor-name">${data.sensor.label}</div><div class="sensor-weight">${data.sensor.currentWeight.toFixed(2)} g</div></div>`;
            }
        }

        updateTimestamp();
        setInterval(updateTimestamp, 1000);
        connectWebSocket();
    </script>
</body>
</html>
)rawliteral";

// ============= SETUP =============
void setup() {
  Serial.begin(115200);
  WeightSerial.begin(115200, SERIAL_8N1, D0, D1);
  
  delay(1000);
  
  Serial.println("\n\n");
  Serial.println("╔════════════════════════════════════════════════╗");
  Serial.println("║     XIAO2 INVENTORY SYSTEM v3.0                ║");
  Serial.println("║     Web-Based Control & Calibration           ║");
  Serial.println("╚════════════════════════════════════════════════╝");

  EEPROM.begin(512);
  led.begin();
  led.setPattern(LED_WIFI_CONNECTING);

  Serial.println("\nConnecting to WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  Serial.println();
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("SUCCESS: WiFi connected");
    Serial.print("Dashboard: http://");
    Serial.println(WiFi.localIP());
    led.setPattern(LED_WIFI_CONNECTED);
    delay(1000);
    led.setPattern(LED_OFF);
  } else {
    Serial.println("ERROR: WiFi connection failed");
  }

  Serial.println("\nInitializing hardware...");
  rfid.begin();
  sensor.begin();

  if (WiFi.status() == WL_CONNECTED) {
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);
    
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send_P(200, "text/html", index_html);
    });
    
    server.on("/api/inventory", HTTP_GET, [](AsyncWebServerRequest *request){
      String json = inventory.getInventoryJson();
      request->send(200, "application/json", json);
    });
    
    server.begin();
    Serial.println("SUCCESS: Web server started");
  }

  Serial.println("\n╔════════════════════════════════════════════════╗");
  Serial.println("║     SYSTEM READY                               ║");
  Serial.println("║     Open dashboard to control system           ║");
  Serial.println("╚════════════════════════════════════════════════╝\n");
}

// ============= LOOP =============
void loop() {
  led.update();
  ws.cleanupClients();

  // Execute deferred calibration work here (safe — not in async callback)
  if (pendingCalibrationCalculate) {
    pendingCalibrationCalculate = false;
    calibrationData.knownMass = pendingCalWeight;

    sensor.refreshDataSet();
    calibrationData.newCalFactor = sensor.getNewCalibration(calibrationData.knownMass);
    calibrationData.state = CAL_COMPLETE;

    if (calibrationData.newCalFactor != 0) {
      calibrationData.message = "Calibration complete! New factor: " + String(calibrationData.newCalFactor, 4);
    } else {
      calibrationData.message = "Calibration returned 0. Check that the known mass is on the scale and the weight board supports GET_NEW_CAL.";
    }

    Serial.print("New calibration factor: ");
    Serial.println(calibrationData.newCalFactor, 4);
    notifyClients();
  }

  // Periodically refresh cached cal factor (non-blocking rate: every 5s)
  static unsigned long lastCalRefresh = 0;
  if (millis() - lastCalRefresh > 5000) {
    cachedCalFactor = sensor.getCalFactor();
    lastCalRefresh = millis();
  }

  String uid;
  if (rfid.readTag(uid)) {
    inventory.handleScan(uid);
  }

  inventory.checkForReturnedItems();

  delay(10);
}
