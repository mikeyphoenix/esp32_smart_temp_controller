#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneButton.h>

// Define the service UUID for the RAPT Pill
#define RAPT_PILL_SERVICE_UUID "0000fff0-0000-1000-8000-00805f9b34fb"

const uint16_t SCAN_INTERVAL = 100; // 1000 ms = 1 se
const uint16_t SCAN_WINDOW = 99;    // 1000 ms = 1 sec

//const unsigned long SCAN_INTERVAL_MS = 1 * 60 * 1000; // 1 minutes (in milliseconds)
const unsigned long SCAN_INTERVAL_MS = 5; // 1 minutes (in milliseconds)

unsigned long lastScanTime = 0;
bool isScanning = false;

//i2c LCD 20x4
LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

#define ONE_WIRE_BUS 2
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DS18B20(&oneWire);
float temp1;

//buttons
OneButton buttonOne(0, true);

// 90:38:0c:12:94:26

BLEScan* pBLEScan;


struct RaptPillInfo {
    float temperature;
    float specificGravity;
    int16_t accelerometerX;
    int16_t accelerometerY;
    int16_t accelerometerZ;
    float batterySOC;
};

std::map<String, String> raptPillDisplayNames;
std::map<String, RaptPillInfo> raptPillData;

// Method to print the raw advertisement data in hexadecimal format
void printRawAdvertisementData(const uint8_t* data, size_t length) {
    Serial.print("Advertisement Data (Hex): ");
    for (size_t i = 0; i < length; i++) {
        Serial.print(String(data[i] < 0x10 ? "0" : ""));
        Serial.print(data[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
}

// Function to convert a 4-byte big-endian hexadecimal value to a float
float hexToFloat(const uint8_t* hexData) {
  uint32_t intValue = (hexData[0] << 24) | (hexData[1] << 16) | (hexData[2] << 8) | hexData[3];
  return *reinterpret_cast<float*>(&intValue);
}

float calculateSpecificGravityBigEndian(uint8_t byteArray[4]) {
  // Convert big-endian byte array to IEEE 754 single-precision floating-point number
  union {
    float floatValue;
    uint8_t byteData[4];
  } converter;

  for (int i = 0; i < 4; i++) {
    converter.byteData[i] = byteArray[i];
  }

  // Extracting sign, exponent, and fraction from IEEE 754 format
  int sign = (converter.byteData[0] >> 7) & 0x1;
  int exponent = ((converter.byteData[0] & 0x7F) << 1) | ((converter.byteData[1] >> 7) & 0x1);
  uint32_t fraction = ((converter.byteData[1] & 0x7F) << 16) | (converter.byteData[2] << 8) | converter.byteData[3];

  // Calculate specific gravity using the formula
  float specificGravity = pow(-1, sign) * (1 + fraction / 8388608.0f) * pow(2, exponent - 127);
  
  // Convert float value to a formatted string with x.xxx format
  char formattedValue[10];
  snprintf(formattedValue, sizeof(formattedValue), "%.3f", (specificGravity/1000));

  Serial.println("Calculated Value:");
  Serial.print(formattedValue);
  Serial.println();

  return specificGravity/1000;
}

void addDisplayName(const String& macAddress, const String& displayName) {
    raptPillDisplayNames[macAddress] = displayName;
}

void processV1Format(const uint8_t* data) {
    // Extract data for v1 format
    const uint8_t* macAddress = &data[1];
    uint16_t temperature = (data[7] << 8) | data[8];
    float gravity = *((float*)&data[9]);
    int16_t accelerometerX = (data[13] << 8) | data[14];
    int16_t accelerometerY = (data[15] << 8) | data[16];
    int16_t accelerometerZ = (data[17] << 8) | data[18];
    uint16_t batterySOC = (data[19] << 8) | data[20];

    Serial.print(", MAC address: ");
    for (int i = 0; i < 6; i++) {
      Serial.print(macAddress[i], HEX);
      if (i < 5) Serial.print(":");
    }
    Serial.print(", Temperature (Celcius): ");
    Serial.print((temperature / 128.0) - 273.15);
    Serial.print(", Specific Gravity: ");
    Serial.print(gravity, 3);
    Serial.print(", Accelerometer (X, Y, Z): ");
    Serial.print(accelerometerX);
    Serial.print(", ");
    Serial.print(accelerometerY);
    Serial.print(", ");
    Serial.print(accelerometerZ);
    Serial.print(", Battery SOC (%): ");
    Serial.println(batterySOC / 256.0);
}
void processV2Format(const String macAddress, const uint8_t* data) {
    // Extract data for v2 format
    uint8_t gravityValid = data[5];
    float gravityVelocity = 0.0;
    float gravityVelocityPointsPerDay = 0.0;
    if (gravityValid == 0x01) {
      gravityVelocity = *((float*)&data[6]);
      uint32_t rawGravityVelocity = (data[7] << 24) | (data[8] << 16) | (data[9] << 8) | data[10];
      gravityVelocityPointsPerDay = *((float*)&rawGravityVelocity);
    }
    uint16_t temperatureInKelvin = (data[11] << 8) | data[12];
    uint16_t temperature = (temperatureInKelvin / 128.0) - 273.15;
    uint8_t specificGravityData[4];
    for (int i = 0; i < 4; i++) {
      specificGravityData[i] = data[13 + i];
    }

    // Convert specific gravity data to decimal
    //float gravity = hexToFloat(specificGravityData);
    float gravity = calculateSpecificGravityBigEndian(specificGravityData);

    int16_t accelerometerX = (data[17] << 8) | data[18];
    int16_t accelerometerY = (data[19] << 8) | data[20];
    int16_t accelerometerZ = (data[21] << 8) | data[22];
    uint16_t batterySOC = (data[23] << 8) | data[24];
    uint16_t batterySOCPerc = batterySOC / 256.0;
    raptPillData[macAddress] = {
        temperature,
        gravity,
        accelerometerX,
        accelerometerY,
        accelerometerZ,
        batterySOCPerc
    };

    Serial.print(", Gravity Velocity (points per day): ");
    Serial.print(gravityVelocity);
    Serial.print(", Temperature (Celcius): ");
    Serial.print(temperature);
    Serial.print(", Specific Gravity: ");
    Serial.print(gravity, 3);
    Serial.print(", Accelerometer (X, Y, Z): ");
    Serial.print(accelerometerX);
    Serial.print(", ");
    Serial.print(accelerometerY);
    Serial.print(", ");
    Serial.print(accelerometerZ);
    Serial.print(", Battery SOC (%): ");
    Serial.println(batterySOCPerc);
}

// BLEScan callback function to handle the scanned advertisements
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    //Serial.print("Advertised Device: ");
    //Serial.println(advertisedDevice.toString().c_str());
    // Get the advertisement data as a raw byte array
    //const uint8_t* advertisedDataRaw = advertisedDevice.getPayload();
    //size_t advertisedDataLength = advertisedDevice.getPayloadLength();
    // Print the raw advertisement data
    //printRawAdvertisementData(advertisedDataRaw, advertisedDataLength);
    // Get the advertised data
    std::string manufacturerData = advertisedDevice.getManufacturerData();
    // Print the manufacturer data in hexadecimal format
    Serial.print("Manufacturer Data: ");
    for (int i = 0; i < manufacturerData.length(); i++) {
      Serial.print(manufacturerData[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
    // Check if the advertisement has valid manufacturer data and is of RAPT Pill service
    if (!manufacturerData.empty() && manufacturerData.find("RAPT") == 0) {
      //Serial.print("Name: ");
      //Serial.print(advertisedDevice.getName().c_str());
      Serial.print("Address: ");
      Serial.println(advertisedDevice.getAddress().toString().c_str());

      // Convert the manufacturer data to a constant byte array
      const uint8_t* data = (const uint8_t*)manufacturerData.data();
      size_t dataLen = manufacturerData.length();

      // Check the format version (v1 or v2)
      uint8_t formatVersion = data[4];
      //Serial.print("formatVersion: ");
      //Serial.println(formatVersion);

      if (formatVersion == 0x01 && dataLen >= 0x1D) { // v1 format
        processV1Format(data);
      } else if (formatVersion == 0x02) { // && dataLen >= 0x19) { // v2 format
        processV2Format(advertisedDevice.getAddress().toString().c_str(), data);
      }
    }
  }
};

void startBLEScan() {
  // Start the BLE scanning process
  Serial.println("Start Scanning....");
  /*BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(SCAN_INTERVAL);
  pBLEScan->setWindow(SCAN_WINDOW);*/
  //pBLEScan->start(SCAN_INTERVAL_MS);
  pBLEScan->start(SCAN_INTERVAL_MS, false);
}

void stopBLEScan() {
  // Stop the BLE scanning process
  Serial.println("Stop Scanning!!!");
  //BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->stop();
}

void initBLE() {
  BLEDevice::init("SmartTempController");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(SCAN_INTERVAL);
  pBLEScan->setWindow(SCAN_WINDOW);
}

void initLCD() {
  lcd.init();
  lcd.backlight();      // Turn on the backlight.
  lcd.clear();
  /*lcd.setCursor(3, 0);  // Move the cursor at origin
  lcd.print("HELLO READERS,");
  lcd.setCursor(0, 1);
  lcd.print("Interfacing nodemcu");
  lcd.setCursor(0, 2);
  lcd.print("with LCD 20X4.");*/
}

void initTempProbes() {

}

void setup() {
  Serial.begin(115200);
  addDisplayName("macAddress", "PillOne_Yellow");
  addDisplayName("90:38:0c:12:94:26", "PillTwo_Green");
  initBLE();
  initLCD();

}

void loop() {
  unsigned long currentMillis = millis();
  //lcd.setCursor(5, 3);
  //lcd.print("GOOD LUCK!");
  Serial.println("Start loop");
  // Check if it's time to start a new scan cycle
  if (!isScanning && (currentMillis - lastScanTime >= SCAN_INTERVAL_MS)) {
      isScanning = true;
      lastScanTime = millis();
      startBLEScan();
  }

  // Check if the scan cycle is complete
  if (isScanning && (currentMillis - lastScanTime >= SCAN_INTERVAL_MS)) {
       isScanning = false;
       lastScanTime = millis();
       stopBLEScan();
  }
  //lcd.setCursor(5, 3);
  //lcd.clear();
  printToLCD();
  Serial.print("End loop");
  //delay(1000);
}

void printToLCD() {
    lcd.clear();
    lcd.setCursor(0, 0); // Start printing on the second line
    lcd.print("SmartTempController");

    for (const auto& entry : raptPillData) {
        /*lcd.setCursor(5, 1);
        lcd.print(entry.first); // Print MAC address */

        lcd.setCursor(0, 1); // Move to the third line
        lcd.print("Temp: ");
        lcd.print(entry.second.temperature, 1); // Print temperature

        lcd.setCursor(0, 2); // Move to the fourth line
        lcd.print("SG: ");
        lcd.print(entry.second.specificGravity, 3); // Print specific gravity

        lcd.setCursor(0, 3); // Move to the fourth line
        lcd.print("SOG%: ");
        lcd.print(entry.second.batterySOC); // Print specific gravity
    }

}
