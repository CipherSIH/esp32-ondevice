#include <Wire.h>
#include <MPU6050_light.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

MPU6050 mpu(Wire);

const float FALL_THRESHOLD = 2.0;
const int FALL_DURATION = 500;
const int BUZZER_PIN = 13;

float prevTotalAcc = 1.0; // Previous total acceleration
unsigned long fallStartTime = 0;
bool fallingDetected = false;

// Warning tune
const int WARNING_TUNE_LENGTH = 8;
const int WARNING_NOTES[] = {262, 330, 392, 523, 523, 392, 330, 262}; // C4, E4, G4, C5, C5, G4, E4, C4
const int WARNING_NOTE_DURATIONS[] = {200, 200, 200, 400, 400, 200, 200, 400};

// Bootup chime
const int BOOTUP_TUNE_LENGTH = 4;
const int BOOTUP_NOTES[] = {523, 659, 784, 1047}; // C5, E5, G5, C6
const int BOOTUP_NOTE_DURATIONS[] = {150, 150, 150, 450};

// BLE server name
#define bleServerName "FallDetector"

// UUID (for service and characteristic)
#define SERVICE_UUID "91bad492-b950-4226-aa2b-4ede9fa42f59"
#define CHARACTERISTIC_UUID "cba1d466-344c-4be3-ab3f-189f80dd7518"

// BLE Server
BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
bool deviceConnected = false;

// Callback for client connection and disconnection
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  pinMode(BUZZER_PIN, OUTPUT);
  
  // Play bootup chime
  playBootupChime();
  
  // Initialize BLE
  setupBLE();
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050 -- ERROR
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");
  
  // Send bootup message over BLE
  sendBLEMessage("Device booted up and ready");
}

void loop() {
  mpu.update();
  
  float accX = mpu.getAccX();
  float accY = mpu.getAccY();
  float accZ = mpu.getAccZ();
  
  // Calculate total acceleration
  float totalAcc = sqrt(accX*accX + accY*accY + accZ*accZ);
  
  // Detect sudden change in acceleration
  if (abs(totalAcc - prevTotalAcc) > FALL_THRESHOLD) {
    if (!fallingDetected) {
      fallingDetected = true;
      fallStartTime = millis();
    }
  } else if (fallingDetected && (millis() - fallStartTime > FALL_DURATION)) {
    Serial.println("Fall detected!");
    playWarningTune();
    sendBLEMessage("Fall detected!");
    fallingDetected = false;
  }
  
  prevTotalAcc = totalAcc;
  
  delay(10); // Short delay for stability
}

void playWarningTune() {
  for (int i = 0; i < WARNING_TUNE_LENGTH; i++) {
    tone(BUZZER_PIN, WARNING_NOTES[i], WARNING_NOTE_DURATIONS[i]);
    delay(WARNING_NOTE_DURATIONS[i] + 50); // Add a small pause between notes
    noTone(BUZZER_PIN);
  }
}

void playBootupChime() {
  for (int i = 0; i < BOOTUP_TUNE_LENGTH; i++) {
    tone(BUZZER_PIN, BOOTUP_NOTES[i], BOOTUP_NOTE_DURATIONS[i]);
    delay(BOOTUP_NOTE_DURATIONS[i] + 30); // Add a small pause between notes
    noTone(BUZZER_PIN);
  }
}

void setupBLE() {
  // Create the BLE Device
  BLEDevice::init(bleServerName);

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  // Create a BLE Descriptor
  pCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting for a client connection to notify...");
}

void sendBLEMessage(const char* message) {
  if (deviceConnected) {
    pCharacteristic->setValue(message);
    pCharacteristic->notify();
    Serial.print("Sent over BLE: ");
    Serial.println(message);
  }
}
