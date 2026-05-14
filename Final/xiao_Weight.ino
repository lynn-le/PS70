#include <HX711_ADC.h>

// Define WeightSerial as Serial1
#define WeightSerial Serial1

// pins
const int HX1_DOUT = D0;
const int HX1_SCK  = D1;

// constants
float SENSOR_CAL_FACTOR = 271.65f;
const float KNOWN_MASS = 62.0f;

HX711_ADC scale(HX1_DOUT, HX1_SCK);

void setup() {
  Serial.begin(115200);  // USB for debugging
  WeightSerial.begin(115200, SERIAL_8N1, D7, D6);
  
  delay(10);
  
  Serial.println("Weight Sensor Board Starting...");
  WeightSerial.println("Weight Sensor Board Starting...");
  
  scale.begin();
  unsigned long stabilizingtime = 500;
  boolean _tare = true;
  scale.start(stabilizingtime, _tare);
  
  if (scale.getTareTimeoutFlag() || scale.getSignalTimeoutFlag()) {
    Serial.println("ERROR: Timeout, check wiring!");
    WeightSerial.println("ERROR: Timeout, check wiring!");
    while (1);
  } else {
    scale.setCalFactor(SENSOR_CAL_FACTOR);
    Serial.println("Weight sensor ready.");
    WeightSerial.println("Weight sensor ready.");
  }
  
  while (!scale.update());
  
  Serial.println("READY");
  WeightSerial.println("READY");
}

void loop() {
  static boolean newDataReady = false;
  static unsigned long lastPing = 0;
  
  
  // Always update scale
  if (scale.update()) {
    newDataReady = true;
  }
  
  // Check for commands from Main Board
  if (WeightSerial.available() > 0) {
    String cmd = WeightSerial.readStringUntil('\n');
    cmd.trim();
    
    Serial.print("Received command: ");
    Serial.println(cmd);
    
    if (cmd == "w") {
      Serial.println("Getting weight...");
      
      // Take stable reading
      delay(200); // Settle time
      
      float sum = 0;
      int numReadings = 50;
      
      for (int i = 0; i < numReadings; i++) {
        while (!scale.update()) {
          delay(1);
        }
        sum += scale.getData();
        delay(10);
      }
      
      float weight = sum / numReadings;
      
      // Send back to Main Board
      WeightSerial.print("WEIGHT:");
      WeightSerial.println(weight, 2);
      
      Serial.print("Sent weight: ");
      Serial.println(weight, 2);
    }
    else if (cmd == "CHECK_PRESENT") {
      float weight = scale.getData();
      if (weight > 15.0f) {
        WeightSerial.println("PRESENT:YES");
        Serial.println("Sent: PRESENT:YES");
      } else {
        WeightSerial.println("PRESENT:NO");
        Serial.println("Sent: PRESENT:NO");
      }
    }
    else if (cmd == "t") {
      scale.tareNoDelay();
      WeightSerial.println("TARE_STARTED");
      Serial.println("Tare started");
    }
    else if (cmd == "c") {
      // Legacy single-char command: use hardcoded KNOWN_MASS
      scale.refreshDataSet();
      float newCal = scale.getNewCalibration(KNOWN_MASS);
      SENSOR_CAL_FACTOR = newCal;
      scale.setCalFactor(SENSOR_CAL_FACTOR);
      WeightSerial.print("NEW_CAL:");
      WeightSerial.println(SENSOR_CAL_FACTOR, 4);
      Serial.print("New cal factor: ");
      Serial.println(SENSOR_CAL_FACTOR, 4);
    }
    else if (cmd.startsWith("GET_NEW_CAL:")) {
      // Dynamic calibration: mass sent from main board
      float knownMass = cmd.substring(12).toFloat();
      if (knownMass > 0) {
        scale.refreshDataSet();
        float newCal = scale.getNewCalibration(knownMass);
        SENSOR_CAL_FACTOR = newCal;
        scale.setCalFactor(SENSOR_CAL_FACTOR);
        WeightSerial.print("NEW_CAL:");
        WeightSerial.println(SENSOR_CAL_FACTOR, 4);
        Serial.print("New cal factor (mass=");
        Serial.print(knownMass, 2);
        Serial.print("g): ");
        Serial.println(SENSOR_CAL_FACTOR, 4);
      } else {
        WeightSerial.println("NEW_CAL:0");
        Serial.println("ERROR: Invalid mass received");
      }
    }
    else if (cmd == "GET_CURRENT") {
      float current = scale.getData();
      WeightSerial.print("CURRENT:");
      WeightSerial.println(current, 2);
      Serial.print("Sent current: ");
      Serial.println(current, 2);
    }
    else if (cmd.startsWith("SET_CAL:")) {
      float newCal = cmd.substring(8).toFloat();
      if (newCal != 0) {
        SENSOR_CAL_FACTOR = newCal;
        scale.setCalFactor(SENSOR_CAL_FACTOR);
        Serial.print("Cal factor set to: ");
        Serial.println(SENSOR_CAL_FACTOR, 4);
      }
    }
    else if (cmd == "GET_CAL") {
      WeightSerial.print("CAL:");
      WeightSerial.println(SENSOR_CAL_FACTOR, 4);
      Serial.print("Sent cal factor: ");
      Serial.println(SENSOR_CAL_FACTOR, 4);
    }
    else if (cmd == "REFRESH_DATASET") {
      scale.refreshDataSet();
      Serial.println("Dataset refreshed");
    }
  }
  
  // Check if tare complete
  if (scale.getTareStatus() == true) {
    WeightSerial.println("TARE_COMPLETE");
    Serial.println("Tare complete");
  }
}