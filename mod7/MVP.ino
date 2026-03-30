#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>
#include <HX711.h>

// pins
const int HX1_DOUT = D0;
const int HX1_SCK  = D1;

const int HX2_DOUT = D2;
const int HX2_SCK  = D3;

const int RFID_SS  = D7;
const int RFID_RST = D6;

const int LED_PIN = D4;

// constants
float SENSOR_1_CAL_FACTOR = 271.65f;
float SENSOR_2_CAL_FACTOR = 349.0484f;

const float KNOWN_MASS_1 = 62.0f;
const float KNOWN_MASS_2 = 62.0f;

const float PLACED_THRESHOLD_G = 15.0f;

// led class
class StatusLed {
  int pin;
  bool timedOn = false;
  unsigned long offTime = 0;

public:
  StatusLed(int inputPin) {
    pin = inputPin;
  }

  void begin() {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }

  void on() {
    digitalWrite(pin, HIGH);
    timedOn = false;
  }

  void off() {
    digitalWrite(pin, LOW);
    timedOn = false;
  }

  void onFor(unsigned long ms) {
    digitalWrite(pin, HIGH);
    timedOn = true;
    offTime = millis() + ms;
  }

  void update() {
    if (timedOn && millis() >= offTime) {
      digitalWrite(pin, LOW);
      timedOn = false;
    }
  }
};

// load sensor class
class LoadSensor {
  HX711 scale;
  int doutPin;
  int sckPin;
  float calibrationFactor;
  const char* label;

public:
  LoadSensor(int inputDout, int inputSck, float inputCalFactor,
             const char* inputLabel) {
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

  const char* getLabel() {
    return label;
  }

  void printWeight() {
    Serial.print(label);
    Serial.print(": ");
    Serial.print(readWeight(10), 2);
    Serial.println(" g");
  }
};

// rfid sensor
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
    Serial.println("RFID ready.");
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

// items class
struct ItemRecord {
  String uid;
  String name;
  float currentWeight;
  bool inUse;
  bool waitingForReturn;
  int sensorIndex;
};

ItemRecord items[] = {
  {"1141733C", "Red Polish", 47.0f, false, false, 0},
  {"D14E733C", "Top Coat",   62.0f, false, false, 1}
};

const int ITEM_COUNT = sizeof(items) / sizeof(items[0]);

// inventory database
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

  void printCurrentWeights() {
    Serial.println("\n--- CURRENT SENSOR WEIGHTS ---");

    for (int i = 0; i < sensorCount; i++) {
      sensors[i].printWeight();
    }

    Serial.println("------------------------------\n");
  }

  void printDatabase() {
    Serial.println("\n--- ITEM DATABASE ---");

    for (int i = 0; i < itemCount; i++) {
      Serial.print(i);
      Serial.print(" | ");
      Serial.print(items[i].name);
      Serial.print(" | UID: ");
      Serial.print(items[i].uid);
      Serial.print(" | weight: ");
      Serial.print(items[i].currentWeight, 2);
      Serial.print(" g | inUse: ");
      Serial.print(items[i].inUse ? "true" : "false");
      Serial.print(" | waitingForReturn: ");
      Serial.print(items[i].waitingForReturn ? "true" : "false");
      Serial.print(" | sensor: ");
      Serial.println(items[i].sensorIndex);
    }

    Serial.println("---------------------\n");
  }

  void handleScan(const String &uid) {
    Serial.print("Scanned UID: ");
    Serial.println(uid);

    led->onFor(500);

    int itemIndex = findItemIndexByUid(uid);

    if (itemIndex == -1) {
      Serial.println("Tag not found in database.");
      return;
    }

    ItemRecord &item = items[itemIndex];

    if (!item.inUse && !item.waitingForReturn) {
      item.inUse = true;
      Serial.print(item.name);
      Serial.println(" marked as IN USE.");
      return;
    }

    if (item.inUse) {
      item.inUse = false;
      item.waitingForReturn = true;
      Serial.print(item.name);
      Serial.print(" marked as NOT IN USE. Waiting for return on ");
      Serial.println(sensors[item.sensorIndex].getLabel());
      return;
    }

    if (item.waitingForReturn) {
      Serial.print(item.name);
      Serial.println(" is already waiting to be placed on the sensor.");
    }
  }

  void checkForReturnedItems() {
    for (int i = 0; i < itemCount; i++) {
      ItemRecord &item = items[i];

      if (!item.waitingForReturn) continue;

      int sensorIdx = item.sensorIndex;
      if (sensorIdx < 0 || sensorIdx >= sensorCount) continue;

      if (sensors[sensorIdx].isObjectPresent()) {
        item.currentWeight = sensors[sensorIdx].readWeight(10);
        item.waitingForReturn = false;

        led->onFor(1500);

        Serial.print(item.name);
        Serial.print(" returned on ");
        Serial.print(sensors[sensorIdx].getLabel());
        Serial.print(". New weight recorded: ");
        Serial.print(item.currentWeight, 2);
        Serial.println(" g");
      }
    }
  }
};

// calibration -- should i combine with load sensor class?
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

    Serial.print("Remove all weight from ");
    Serial.print(sensors[sensorIndex].getLabel());
    Serial.println(".");
  }

  void update() {
    if (!active) return;

    unsigned long now = millis();

    if (step == 0 && now - stepStart >= 2000) {
      sensors[sensorIndex].tare();

      Serial.print("Place known mass on ");
      Serial.print(sensors[sensorIndex].getLabel());
      Serial.println(" now...");

      step = 1;
      stepStart = now;
    }
    else if (step == 1 && now - stepStart >= 3000) {
      if (sensorIndex == 0) {
        sensors[sensorIndex].calibrate(KNOWN_MASS_1);
      }
      else if (sensorIndex == 1) {
        sensors[sensorIndex].calibrate(KNOWN_MASS_2);
      }

      active = false;
      sensorIndex = -1;
      step = 0;
    }
  }

  bool isActive() {
    return active;
  }
};

// command system
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
    Serial.println("Commands:");
    Serial.println("  1 = tare sensor 1");
    Serial.println("  2 = tare sensor 2");
    Serial.println("  3 = calibrate sensor 1");
    Serial.println("  4 = calibrate sensor 2");
    Serial.println("  w = print current weights");
    Serial.println("  p = print database");
  }

  void update() {
    if (!Serial.available()) return;

    char cmd = Serial.read();

    switch (cmd) {
      case '1':
        sensors[0].tare();
        break;

      case '2':
        sensors[1].tare();
        break;

      case '3':
        calibration->start(0);
        break;

      case '4':
        calibration->start(1);
        break;

      case 'w':
        inventory->printCurrentWeights();
        break;

      case 'p':
        inventory->printDatabase();
        break;

      default:
        printHelp();
        break;
    }
  }
};

// objects
StatusLed led(LED_PIN);

LoadSensor sensors[] = {
  LoadSensor(HX1_DOUT, HX1_SCK, SENSOR_1_CAL_FACTOR, "Sensor 1"),
  LoadSensor(HX2_DOUT, HX2_SCK, SENSOR_2_CAL_FACTOR, "Sensor 2")
};

const int SENSOR_COUNT = sizeof(sensors) / sizeof(sensors[0]);

RfidSystem rfid(RFID_SS, RFID_RST);
InventorySystem inventory(items, ITEM_COUNT, sensors, SENSOR_COUNT, &led);
CalibrationManager calibration(sensors);
CommandHandler commands(sensors, &inventory, &calibration);

void setup() {
  Serial.begin(115200);

  led.begin();
  rfid.begin();

  for (int i = 0; i < SENSOR_COUNT; i++) {
    sensors[i].begin();
  }

  Serial.println("System ready.");
  Serial.println("Scan an item tag.");
  commands.printHelp();
}

void loop() {
  led.update();
  calibration.update();
  commands.update();

  String uid;
  if (rfid.readTag(uid)) {
    inventory.handleScan(uid);
  }

  inventory.checkForReturnedItems();
}