#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>
#include <HX711.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

const char* WIFI_SSID = "MAKERSPACE";
const char* WIFI_PASSWORD = "12345678";


const int HX1_DOUT = D0;
const int HX1_SCK  = D1;
const int HX2_DOUT = D2; 
const int HX2_SCK  = D3;  

const int RFID_SS  = D7; 
const int RFID_RST = D6; 

const int LED_PIN = D4; 

// Calibration constants
float SENSOR_1_CAL_FACTOR = 271.65f;
float SENSOR_2_CAL_FACTOR = 349.0484f;
const float KNOWN_MASS_1 = 62.0f;
const float KNOWN_MASS_2 = 62.0f;
const float PLACED_THRESHOLD_G = 15.0f;


enum LedPattern {
  LED_OFF,
  LED_SCAN_SUCCESS,
  LED_SCAN_FAIL,
  LED_WEIGHT_REGISTERED,
  LED_CHECKOUT,
  LED_WIFI_CONNECTING,
  LED_WIFI_CONNECTED
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
        // Quick 200ms flash
        if (elapsed < 200) {
          digitalWrite(pin, HIGH);
        } else {
          digitalWrite(pin, LOW);
          currentPattern = LED_OFF;
        }
        break;

      case LED_SCAN_FAIL:
        // Three 100ms flashes
        if (elapsed < 600) {
          digitalWrite(pin, (elapsed / 100) % 2 == 0 ? HIGH : LOW);
        } else {
          digitalWrite(pin, LOW);
          currentPattern = LED_OFF;
        }
        break;

      case LED_WEIGHT_REGISTERED:
        // 1.5 second pulse
        if (elapsed < 1500) {
          digitalWrite(pin, HIGH);
        } else {
          digitalWrite(pin, LOW);
          currentPattern = LED_OFF;
        }
        break;

      case LED_CHECKOUT:
        // Two 300ms pulses
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
        // Slow blink
        digitalWrite(pin, (elapsed / 500) % 2 == 0 ? HIGH : LOW);
        break;

      case LED_WIFI_CONNECTED:
        digitalWrite(pin, HIGH);
        break;
    }
  }
};

// load sensor
class LoadSensor {
  HX711 scale;
  int doutPin;
  int sckPin;
  float calibrationFactor;
  const char* label;

public:
  LoadSensor(int inputDout, int inputSck, float inputCalFactor, const char* inputLabel) {
    doutPin = inputDout;
    sckPin = inputSck;
    calibrationFactor = inputCalFactor;
    label = inputLabel;
  }

  void begin() {
    scale.begin(doutPin, sckPin);
    scale.set_scale(calibrationFactor);
    scale.tare();
  }

  void tare() {
    scale.tare();
    Serial.print(label);
    Serial.println(" tared.");
  }

  void calibrate(float knownMass) {
    scale.set_scale(1.0f);
    long raw = scale.get_value(15);
    calibrationFactor = raw / knownMass;
    scale.set_scale(calibrationFactor);

    Serial.print(label);
    Serial.print(" calibration factor: ");
    Serial.println(calibrationFactor, 4);
  }

  float readWeight(int samples = 10) {
    return scale.get_units(samples);
  }

  bool isObjectPresent(float threshold = PLACED_THRESHOLD_G) {
    return readWeight(5) > threshold;
  }

  const char* getLabel() { return label; }
  float getCalibrationFactor() { return calibrationFactor; }

  void printWeight() {
    Serial.print(label);
    Serial.print(": ");
    Serial.print(readWeight(10), 2);
    Serial.println(" g");
  }
};

// rfid
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
    rfid.PCD_Init();
    Serial.println("SUCCESS: RFID ready");
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

// database item
struct ItemRecord {
  String uid;
  String name;
  String color;
  String brand;
  float initialWeight;
  float currentWeight;
  bool inUse;
  bool waitingForReturn;
  int sensorIndex;
  unsigned long lastCheckout;
  unsigned long lastReturn;

  float getPercentRemaining() const {
    if (initialWeight <= 0) return 0;
    return (currentWeight / initialWeight) * 100.0f;
  }
};

// Database of items - EXPAND THIS WITH YOUR INVENTORY
ItemRecord items[] = {
  {"1141733C", "Red Polish", "#DC143C", "OPI", 47.0f, 47.0f, false, false, 0, 0, 0},
  {"D14E733C", "Top Coat", "#FFFFFF", "Essie", 62.0f, 62.0f, false, false, 1, 0, 0}
};

const int ITEM_COUNT = sizeof(items) / sizeof(items[0]);

// Forward declarations
void notifyClients();

// inventory
class InventorySystem {
  ItemRecord* items;
  int itemCount;
  LoadSensor* sensors;
  int sensorCount;
  StatusLed* led;

public:
  InventorySystem(ItemRecord* inputItems, int inputItemCount, 
                  LoadSensor* inputSensors, int inputSensorCount, 
                  StatusLed* inputLed) {
    items = inputItems;
    itemCount = inputItemCount;
    sensors = inputSensors;
    sensorCount = inputSensorCount;
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
  LoadSensor* getSensors() { return sensors; }
  int getSensorCount() { return sensorCount; }

  String getInventoryJson() {
    // Using ArduinoJson library for proper JSON formatting
    StaticJsonDocument<2048> doc;
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
      item["sensorIndex"] = items[i].sensorIndex;
      item["sensorLabel"] = sensors[items[i].sensorIndex].getLabel();
    }

    JsonArray sensorsArray = doc.createNestedArray("sensors");
    for (int i = 0; i < sensorCount; i++) {
      JsonObject sensor = sensorsArray.createNestedObject();
      sensor["label"] = sensors[i].getLabel();
      sensor["currentWeight"] = round(sensors[i].readWeight(5) * 100) / 100.0;
    }

    String output;
    serializeJson(doc, output);
    return output;
  }

  void printDatabase() {
    for (int i = 0; i < itemCount; i++) {
      ItemRecord &item = items[i];
      Serial.println();
      Serial.print("  [" + String(i) + "] ");
      Serial.println(item.name);
      Serial.print("      UID: ");
      Serial.println(item.uid);
      Serial.print("      Brand: ");
      Serial.println(item.brand);
      Serial.print("      Weight: ");
      Serial.print(item.currentWeight, 2);
      Serial.print(" g (");
      Serial.print(item.getPercentRemaining(), 1);
      Serial.println("% remaining)");
      Serial.print("      Status: ");
      
      if (item.inUse) {
        Serial.println("CHECKED OUT");
      } else if (item.waitingForReturn) {
        Serial.println("AWAITING RETURN");
      } else {
        Serial.println("AVAILABLE");
      }
      
      Serial.print("      Sensor: ");
      Serial.println(sensors[item.sensorIndex].getLabel());
    }
    Serial.println();
  }

  void handleScan(const String &uid) {
    Serial.print("SCAN: UID detected: ");
    Serial.println(uid);

    int itemIndex = findItemIndexByUid(uid);

    if (itemIndex == -1) {
      Serial.println("ERROR: Tag not found in database");
      led->setPattern(LED_SCAN_FAIL);
      return;
    }

    ItemRecord &item = items[itemIndex];
    led->setPattern(LED_SCAN_SUCCESS);

    // Item available - check it out
    if (!item.inUse && !item.waitingForReturn) {
      item.inUse = true;
      item.lastCheckout = millis();
      
      Serial.print("STATUS: CHECKED OUT - ");
      Serial.println(item.name);
      Serial.print("  Brand: ");
      Serial.println(item.brand);
      Serial.print("  Weight: ");
      Serial.print(item.currentWeight, 2);
      Serial.print(" g (");
      Serial.print(item.getPercentRemaining(), 1);
      Serial.println("% full)");
      
      led->setPattern(LED_CHECKOUT);
      notifyClients();
      return;
    }

    // Item in use - mark for return
    if (item.inUse) {
      item.inUse = false;
      item.waitingForReturn = true;
      
      Serial.print("STATUS: RETURNED (pending placement) - ");
      Serial.println(item.name);
      Serial.print("  Please place on ");
      Serial.println(sensors[item.sensorIndex].getLabel());
      
      led->setPattern(LED_CHECKOUT);
      notifyClients();
      return;
    }

    // Already waiting for return
    if (item.waitingForReturn) {
      Serial.print("WARNING: ");
      Serial.print(item.name);
      Serial.print(" is already waiting to be placed on ");
      Serial.println(sensors[item.sensorIndex].getLabel());
    }
  }

  void checkForReturnedItems() {
    for (int i = 0; i < itemCount; i++) {
      ItemRecord &item = items[i];

      if (!item.waitingForReturn) continue;

      int sensorIdx = item.sensorIndex;
      if (sensorIdx < 0 || sensorIdx >= sensorCount) continue;

      if (sensors[sensorIdx].isObjectPresent()) {
        float newWeight = sensors[sensorIdx].readWeight(10);
        float oldWeight = item.currentWeight;
        item.currentWeight = newWeight;
        item.waitingForReturn = false;
        item.lastReturn = millis();

        led->setPattern(LED_WEIGHT_REGISTERED);

        Serial.print("WEIGHT REGISTERED: ");
        Serial.println(item.name);
        Serial.print("  Sensor: ");
        Serial.println(sensors[sensorIdx].getLabel());
        Serial.print("  Previous: ");
        Serial.print(oldWeight, 2);
        Serial.println(" g");
        Serial.print("  Current:  ");
        Serial.print(newWeight, 2);
        Serial.println(" g");
        Serial.print("  Change:   ");
        Serial.print(newWeight - oldWeight, 2);
        Serial.println(" g");
        Serial.print("  Remaining: ");
        Serial.print(item.getPercentRemaining(), 1);
        Serial.println("%");
        
        if (item.getPercentRemaining() < 20.0f) {
          Serial.println("  WARNING: Low inventory");
        }
        
        notifyClients();
      }
    }
  }
};

// calibration management
class CalibrationManager {
  LoadSensor* sensors;
  bool active = false;
  int sensorIndex = -1;
  int step = 0;
  unsigned long stepStart = 0;

public:
  CalibrationManager(LoadSensor* inputSensors) {
    sensors = inputSensors;
  }

  void start(int index) {
    if (active) return;

    active = true;
    sensorIndex = index;
    step = 0;
    stepStart = millis();
    Serial.print("  Step 1: Remove all weight from ");
    Serial.println(sensors[sensorIndex].getLabel());
  }

  void update() {
    if (!active) return;

    unsigned long now = millis();

    if (step == 0 && now - stepStart >= 2000) {
      sensors[sensorIndex].tare();
      Serial.print("  Step 2: Place known mass on ");
      Serial.println(sensors[sensorIndex].getLabel());
      step = 1;
      stepStart = now;
    }
    else if (step == 1 && now - stepStart >= 3000) {
      float knownMass = (sensorIndex == 0) ? KNOWN_MASS_1 : KNOWN_MASS_2;
      sensors[sensorIndex].calibrate(knownMass);
      
      Serial.println("  SUCCESS: Calibration complete");
      
      active = false;
      sensorIndex = -1;
      step = 0;
    }
  }

  bool isActive() { return active; }
};

// websocket
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Global objects
StatusLed led(LED_PIN);
LoadSensor sensors[] = {
  LoadSensor(HX1_DOUT, HX1_SCK, SENSOR_1_CAL_FACTOR, "Sensor 1"),
  LoadSensor(HX2_DOUT, HX2_SCK, SENSOR_2_CAL_FACTOR, "Sensor 2")
};
const int SENSOR_COUNT = sizeof(sensors) / sizeof(sensors[0]);

RfidSystem rfid(RFID_SS, RFID_RST);
InventorySystem inventory(items, ITEM_COUNT, sensors, SENSOR_COUNT, &led);
CalibrationManager calibration(sensors);

// WebSocket event handler
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, 
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("WebSocket client #%u connected from %s\n", 
                  client->id(), client->remoteIP().toString().c_str());
    // Send current inventory state to new client
    client->text(inventory.getInventoryJson());
  } 
  else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("WebSocket client #%u disconnected\n", client->id());
  }
  else if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      data[len] = 0;
      String message = (char*)data;
      
      if (message == "get_inventory") {
        client->text(inventory.getInventoryJson());
      }
    }
  }
}

// Notify all WebSocket clients
void notifyClients() {
  ws.textAll(inventory.getInventoryJson());
}

// HTML page (stored in PROGMEM to save RAM)
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Nail Inventory Management</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, 'Helvetica Neue', Arial, sans-serif;
            background: #fafafa;
            color: #1a1a1a;
            line-height: 1.6;
            padding: 40px 20px;
        }
        .container { max-width: 1400px; margin: 0 auto; }
        .header { margin-bottom: 50px; }
        .header h1 {
            font-size: 28px;
            font-weight: 400;
            color: #1a1a1a;
            letter-spacing: -0.5px;
            margin-bottom: 8px;
        }
        .header-meta {
            display: flex;
            align-items: center;
            gap: 20px;
            font-size: 14px;
            color: #666;
        }
        .status {
            display: inline-flex;
            align-items: center;
            gap: 6px;
            font-size: 13px;
            font-weight: 500;
        }
        .status-dot {
            width: 6px;
            height: 6px;
            border-radius: 50%;
        }
        .status.connected .status-dot { background: #22c55e; }
        .status.disconnected .status-dot { background: #ef4444; }
        .inventory-grid {
            display: grid;
            grid-template-columns: repeat(auto-fill, minmax(280px, 1fr));
            gap: 16px;
            margin-bottom: 50px;
        }
        .item-card {
            background: white;
            border: 1px solid #e5e5e5;
            border-radius: 8px;
            padding: 20px;
            transition: all 0.2s ease;
        }
        .item-card:hover {
            border-color: #d4d4d4;
            box-shadow: 0 2px 8px rgba(0,0,0,0.04);
        }
        .item-header {
            display: flex;
            align-items: flex-start;
            gap: 12px;
            margin-bottom: 16px;
        }
        .color-swatch {
            width: 32px;
            height: 32px;
            border-radius: 6px;
            border: 1px solid #e5e5e5;
            flex-shrink: 0;
        }
        .item-info { flex: 1; min-width: 0; }
        .item-name {
            font-size: 15px;
            font-weight: 500;
            color: #1a1a1a;
            margin-bottom: 2px;
            overflow: hidden;
            text-overflow: ellipsis;
            white-space: nowrap;
        }
        .item-brand { font-size: 13px; color: #737373; }
        .progress-container { margin-bottom: 16px; }
        .progress-label {
            display: flex;
            justify-content: space-between;
            align-items: baseline;
            margin-bottom: 8px;
            font-size: 12px;
            color: #737373;
        }
        .progress-percentage { font-weight: 500; color: #1a1a1a; }
        .progress-bar {
            width: 100%;
            height: 4px;
            background: #f5f5f5;
            border-radius: 2px;
            overflow: hidden;
        }
        .progress-fill {
            height: 100%;
            background: #1a1a1a;
            transition: width 0.3s ease;
        }
        .progress-fill.high { background: #22c55e; }
        .progress-fill.medium { background: #f59e0b; }
        .progress-fill.low { background: #ef4444; }
        .item-stats {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 12px;
            margin-bottom: 12px;
        }
        .stat {
            padding: 12px;
            background: #fafafa;
            border-radius: 6px;
        }
        .stat-label {
            font-size: 11px;
            color: #737373;
            text-transform: uppercase;
            letter-spacing: 0.5px;
            margin-bottom: 4px;
        }
        .stat-value {
            font-size: 16px;
            font-weight: 500;
            color: #1a1a1a;
        }
        .item-footer {
            display: flex;
            justify-content: space-between;
            align-items: center;
            padding-top: 12px;
            border-top: 1px solid #f5f5f5;
        }
        .item-status {
            font-size: 12px;
            font-weight: 500;
            padding: 4px 10px;
            border-radius: 12px;
        }
        .item-status.available {
            background: #f0fdf4;
            color: #15803d;
        }
        .item-status.checked-out {
            background: #fef3c7;
            color: #a16207;
        }
        .item-status.awaiting {
            background: #eff6ff;
            color: #1e40af;
        }
        .sensor-label { font-size: 12px; color: #a3a3a3; }
        .sensors-panel {
            background: white;
            border: 1px solid #e5e5e5;
            border-radius: 8px;
            padding: 24px;
        }
        .sensors-panel h2 {
            font-size: 18px;
            font-weight: 500;
            color: #1a1a1a;
            margin-bottom: 20px;
        }
        .sensors-grid {
            display: grid;
            grid-template-columns: repeat(auto-fill, minmax(200px, 1fr));
            gap: 16px;
        }
        .sensor {
            padding: 16px;
            background: #fafafa;
            border-radius: 6px;
        }
        .sensor-name {
            font-size: 13px;
            color: #737373;
            margin-bottom: 4px;
        }
        .sensor-weight {
            font-size: 24px;
            font-weight: 400;
            color: #1a1a1a;
        }
        @media (max-width: 768px) {
            body { padding: 20px 16px; }
            .header h1 { font-size: 24px; }
            .inventory-grid { grid-template-columns: 1fr; }
            .sensors-grid { grid-template-columns: 1fr; }
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>Nail Inventory Management</h1>
            <div class="header-meta">
                <span class="status disconnected" id="wsStatus">
                    <span class="status-dot"></span>
                    <span id="statusText">Disconnected</span>
                </span>
                <span id="itemCount">Loading...</span>
            </div>
        </div>
        
        <div class="inventory-grid" id="inventoryGrid">
            <div class="item-card">
                <p style="text-align: center; color: #737373;">Loading inventory...</p>
            </div>
        </div>

        <div class="sensors-panel">
            <h2>Live Sensor Readings</h2>
            <div class="sensors-grid" id="sensorData">
                <p style="color: #737373;">Connecting...</p>
            </div>
        </div>
    </div>

    <script>
        let ws;
        const wsStatus = document.getElementById('wsStatus');
        const statusText = document.getElementById('statusText');
        const itemCount = document.getElementById('itemCount');
        const inventoryGrid = document.getElementById('inventoryGrid');
        const sensorData = document.getElementById('sensorData');

        function connectWebSocket() {
            ws = new WebSocket(`ws://${window.location.hostname}/ws`);
            
            ws.onopen = () => {
                console.log('WebSocket connected');
                wsStatus.className = 'status connected';
                statusText.textContent = 'Connected';
                ws.send('get_inventory');
            };
            
            ws.onclose = () => {
                console.log('WebSocket disconnected');
                wsStatus.className = 'status disconnected';
                statusText.textContent = 'Disconnected';
                setTimeout(connectWebSocket, 3000);
            };
            
            ws.onerror = (error) => {
                console.error('WebSocket error:', error);
            };
            
            ws.onmessage = (event) => {
                try {
                    const data = JSON.parse(event.data);
                    updateInventory(data);
                } catch (e) {
                    console.error('Error parsing message:', e);
                }
            };
        }

        function updateInventory(data) {
            if (!data.items) return;

            itemCount.textContent = `${data.items.length} items tracked`;

            inventoryGrid.innerHTML = data.items.map(item => {
                let statusClass = 'available';
                let statusText = 'Available';
                
                if (item.inUse) {
                    statusClass = 'checked-out';
                    statusText = 'Checked Out';
                } else if (item.waitingForReturn) {
                    statusClass = 'awaiting';
                    statusText = 'Awaiting Return';
                }

                let progressClass = 'high';
                if (item.percentRemaining < 20) {
                    progressClass = 'low';
                } else if (item.percentRemaining < 50) {
                    progressClass = 'medium';
                }

                return `
                    <div class="item-card">
                        <div class="item-header">
                            <div class="color-swatch" style="background-color: ${item.color}"></div>
                            <div class="item-info">
                                <div class="item-name">${item.name}</div>
                                <div class="item-brand">${item.brand}</div>
                            </div>
                        </div>
                        
                        <div class="progress-container">
                            <div class="progress-label">
                                <span>Remaining</span>
                                <span class="progress-percentage">${item.percentRemaining.toFixed(0)}%</span>
                            </div>
                            <div class="progress-bar">
                                <div class="progress-fill ${progressClass}" 
                                     style="width: ${item.percentRemaining}%"></div>
                            </div>
                        </div>
                        
                        <div class="item-stats">
                            <div class="stat">
                                <div class="stat-label">Weight</div>
                                <div class="stat-value">${item.currentWeight.toFixed(1)}g</div>
                            </div>
                            <div class="stat">
                                <div class="stat-label">Initial</div>
                                <div class="stat-value">${item.initialWeight.toFixed(1)}g</div>
                            </div>
                        </div>
                        
                        <div class="item-footer">
                            <div class="item-status ${statusClass}">${statusText}</div>
                            <div class="sensor-label">${item.sensorLabel}</div>
                        </div>
                    </div>
                `;
            }).join('');

            if (data.sensors) {
                sensorData.innerHTML = data.sensors.map(sensor => `
                    <div class="sensor">
                        <div class="sensor-name">${sensor.label}</div>
                        <div class="sensor-weight">${sensor.currentWeight.toFixed(2)} g</div>
                    </div>
                `).join('');
            }
        }

        connectWebSocket();
        setInterval(() => {
            if (ws.readyState === WebSocket.OPEN) {
                ws.send('get_inventory');
            }
        }, 5000);
    </script>
</body>
</html>
)rawliteral";

// commands
class CommandHandler {
  LoadSensor* sensors;
  InventorySystem* inventory;
  CalibrationManager* calibration;

public:
  CommandHandler(LoadSensor* inputSensors, InventorySystem* inputInventory, 
                 CalibrationManager* inputCalibration) {
    sensors = inputSensors;
    inventory = inputInventory;
    calibration = inputCalibration;
  }

  void printHelp() {
    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║           AVAILABLE COMMANDS           ║");
    Serial.println("╠════════════════════════════════════════╣");
    Serial.println("║  1 - Tare sensor 1                     ║");
    Serial.println("║  2 - Tare sensor 2                     ║");
    Serial.println("║  3 - Calibrate sensor 1                ║");
    Serial.println("║  4 - Calibrate sensor 2                ║");
    Serial.println("║  w - Print current weights             ║");
    Serial.println("║  p - Print inventory database          ║");
    Serial.println("║  i - Print WiFi info                   ║");
    Serial.println("║  h - Show this help                    ║");
    Serial.println("╚════════════════════════════════════════╝\n");
  }

  void printWifiInfo() {
    Serial.print("Wifi Info")
    Serial.print("Status: ");
    Serial.println(WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Dashboard: http://");
    Serial.println(WiFi.localIP().toString());
    Serial.print("MAC Address: ");
    Serial.println(WiFi.macAddress());
  }

  void update() {
    if (!Serial.available()) return;

    char cmd = Serial.read();

    switch (cmd) {
      case '1':
        sensors[0].tare();
        notifyClients();
        break;

      case '2':
        sensors[1].tare();
        notifyClients();
        break;

      case '3':
        calibration->start(0);
        break;

      case '4':
        calibration->start(1);
        break;

      case 'w':
        Serial.println("CURRENT SENSOR WEIGHTS");
        for (int i = 0; i < inventory->getSensorCount(); i++) {
          sensors[i].printWeight();
        }
        Serial.println();
        break;

      case 'p':
        inventory->printDatabase();
        break;

      case 'i':
        printWifiInfo();
        break;

      case 'h':
        printHelp();
        break;

      default:
        Serial.println("ERROR: Unknown command. Type 'h' for help.");
        break;
    }
  }
};

CommandHandler commands(sensors, &inventory, &calibration);

void setup() {
  Serial.begin(115200);
  delay(1000); // Give serial time to initialize
  
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n");
  Serial.println("╔════════════════════════════════════════════════╗");
  Serial.println("║  SMART NAIL INVENTORY SYSTEM v2.0 - XIAO C3   ║");
  Serial.println("║  Starting up...                                ║");
  Serial.println("╚════════════════════════════════════════════════╝");

  // Initialize LED
  led.begin();
  led.setPattern(LED_WIFI_CONNECTING);

  // Connect to WiFi
  Serial.println("\nConnecting to WiFi...");
  Serial.print("SSID: ");
  Serial.println(WIFI_SSID);
  
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
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Dashboard URL: http://");
    Serial.println(WiFi.localIP());
    
    led.setPattern(LED_WIFI_CONNECTED);
    delay(1000);
    led.setPattern(LED_OFF);
  } else {
    Serial.println("ERROR: WiFi connection failed");
    Serial.println("System will continue in offline mode");
  }

  // Initialize hardware
  Serial.println("\nInitializing hardware...");
  rfid.begin();
  
  for (int i = 0; i < SENSOR_COUNT; i++) {
    sensors[i].begin();
    Serial.print("INIT: ");
    Serial.print(sensors[i].getLabel());
    Serial.println(" initialized");
  }

  // Start web server if WiFi connected
  if (WiFi.status() == WL_CONNECTED) {
    // WebSocket handler
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);
    
    // Serve main page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send_P(200, "text/html", index_html);
    });
    
    // API endpoint for JSON data
    server.on("/api/inventory", HTTP_GET, [](AsyncWebServerRequest *request){
      String json = inventory.getInventoryJson();
      request->send(200, "application/json", json);
    });
    
    // Start server
    server.begin();
    
    Serial.println("SUCCESS: Web server started");
    Serial.println("SUCCESS: WebSocket server started");
  }

  Serial.println("SYSTEM READY - SCAN AN ITEM");
  
  commands.printHelp();
}

void loop() {
  // Update components
  led.update();
  calibration.update();
  commands.update();
  
  // Clean up WebSocket clients
  ws.cleanupClients();

  // Handle RFID scans
  String uid;
  if (rfid.readTag(uid)) {
    inventory.handleScan(uid);
  }

  // Check for returned items
  inventory.checkForReturnedItems();

  delay(10);
}
