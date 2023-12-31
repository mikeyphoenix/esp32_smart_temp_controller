#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>

// Define the service UUID for the RAPT Pill
#define RAPT_PILL_SERVICE_UUID "0000fff0-0000-1000-8000-00805f9b34fb"

const uint16_t SCAN_INTERVAL = 1600; // 1000 ms (1600 * 0.625 ms)
const uint16_t SCAN_WINDOW = 800;    // 500 ms (800 * 0.625 ms)

const unsigned long SCAN_INTERVAL_MS = 5 * 60 * 1000; // 5 minutes (in milliseconds)

unsigned long lastScanTime = 0;
bool isScanning = false;

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
  return *(float*)&intValue;
}

// BLEScan callback function to handle the scanned advertisements
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("Advertised Device: ");
        Serial.println(advertisedDevice.toString().c_str());
        // Get the advertisement data as a raw byte array
        const uint8_t* advertisedDataRaw = advertisedDevice.getPayload();
        size_t advertisedDataLength = advertisedDevice.getPayloadLength();

        // Print the raw advertisement data
        printRawAdvertisementData(advertisedDataRaw, advertisedDataLength);
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
      Serial.print("Name: ");
      Serial.print(advertisedDevice.getName().c_str());
      Serial.print(", Address: ");
      Serial.println(advertisedDevice.getAddress().toString().c_str());

      // Convert the manufacturer data to a constant byte array
      const uint8_t* data = (const uint8_t*)manufacturerData.data();
      size_t dataLen = manufacturerData.length();

      // Check the format version (v1 or v2)
      uint8_t formatVersion = data[4];
      Serial.print("formatVersion: ");
      Serial.println(formatVersion);

      if (formatVersion == 0x01 && dataLen >= 0x1D) { // v1 format
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
        Serial.print(", Temperature (Kelvin): ");
        Serial.print(temperature / 128.0);
        Serial.print(", Specific Gravity: ");
        Serial.print(gravity);
        Serial.print(", Accelerometer (X, Y, Z): ");
        Serial.print(accelerometerX);
        Serial.print(", ");
        Serial.print(accelerometerY);
        Serial.print(", ");
        Serial.print(accelerometerZ);
        Serial.print(", Battery SOC (%): ");
        Serial.println(batterySOC / 256.0);
      } else if (formatVersion == 0x02) { // && dataLen >= 0x19) { // v2 format
        // Extract data for v2 format
        uint8_t gravityValid = data[5];
        float gravityVelocity = 0.0;
        float gravityVelocityPointsPerDay = 0.0;
        if (gravityValid == 0x01) {
          gravityVelocity = *((float*)&data[6]);
          uint32_t rawGravityVelocity = (data[7] << 24) | (data[8] << 16) | (data[9] << 8) | data[10];
          gravityVelocityPointsPerDay = *((float*)&rawGravityVelocity);
        }
        uint16_t temperature = (data[11] << 8) | data[12];
        uint8_t specificGravityData[4];
        for (int i = 0; i < 4; i++) {
          specificGravityData[i] = data[13 + i];
        }

        // Convert specific gravity data to decimal
        float gravity = hexToFloat(specificGravityData);
        //float gravity = *((float*)&data[8]);
        int16_t accelerometerX = (data[17] << 8) | data[18];
        int16_t accelerometerY = (data[19] << 8) | data[20];
        int16_t accelerometerZ = (data[21] << 8) | data[22];
        uint16_t batterySOC = (data[23] << 8) | data[24];

        Serial.print(", Gravity Velocity (points per day): ");
        Serial.print(gravityVelocity);
        Serial.print(", Temperature (Kelvin): ");
        Serial.print((temperature / 128.0) - 273.15);
        Serial.print(", Specific Gravity: ");
        Serial.print(gravity);
        Serial.print(", Accelerometer (X, Y, Z): ");
        Serial.print(accelerometerX);
        Serial.print(", ");
        Serial.print(accelerometerY);
        Serial.print(", ");
        Serial.print(accelerometerZ);
        Serial.print(", Battery SOC (%): ");
        Serial.println(batterySOC / 256.0);
      }
      // Add more conditions here for other versions if required

      // You can add more code here to process the data as needed
    } else {
      Serial.println("NOT RAPT DEVICE");
    }
  }
};

void startBLEScan() {
  // Start the BLE scanning process
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(SCAN_INTERVAL);
  pBLEScan->setWindow(SCAN_WINDOW);
  pBLEScan->start(SCAN_INTERVAL_MS);
  isScanning = true;
  lastScanTime = millis();
}

void stopBLEScan() {
  // Stop the BLE scanning process
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->stop();
  isScanning = false;
}

void setup() {
  Serial.begin(115200);
  BLEDevice::init("");
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(1000, true);
}

void loop() {
  Serial.println("Start Loop");
  unsigned long currentMillis = millis();

  // Check if it's time to start a new scan cycle
  if (!isScanning && (currentMillis - lastScanTime >= SCAN_INTERVAL_MS)) {
    startBLEScan();
  }

  // Check if the scan cycle is complete
  if (isScanning && (currentMillis - lastScanTime >= SCAN_INTERVAL_MS)) {
    stopBLEScan();
  }
  delay(1000);
}
