#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "MAX30105.h"
#include "heartRate.h"

#define DEVICE_NAME "ESP32_HeartBLE"
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

MAX30105 particleSensor;
BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// Beat detection variables
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg;

// NN intervals for HRV calculation
const int INTERVAL_BUFFER_SIZE = 30;
uint16_t nnIntervals[INTERVAL_BUFFER_SIZE];
int intervalIndex = 0;
int intervalCount = 0;

// Calculated metrics
float currentBPM = 0.0;
float currentIPM = 0.0;
float currentHRSTD = 0.0;
float currentRMSSD = 0.0;

bool fingerPresent = false;
const uint32_t MIN_INTERVAL = 400;
const uint32_t MAX_INTERVAL = 1500;

// Sensor health monitoring
uint32_t lastSensorCheck = 0;
uint32_t sensorCheckInterval = 5000;
bool sensorHealthy = false;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        Serial.println("BLE Device Connected");
    }
    
    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        Serial.println("BLE Device Disconnected");
        
        // Auto-restart advertising
        delay(500);
        pServer->startAdvertising();
        Serial.println("Advertising restarted");
    }
};

// Initialize sensor with better error handling
bool initializeSensor() {
    Serial.println("Initializing MAX30102...");
    
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
        Serial.println("MAX30102 not found on I2C bus");
        return false;
    }
    
    // Configure sensor with optimal settings
    byte ledBrightness = 0x1F;
    byte sampleAverage = 4;
    byte ledMode = 2;
    int sampleRate = 100;
    int pulseWidth = 411;
    int adcRange = 4096;
    
    particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
    particleSensor.setPulseAmplitudeRed(0x1F);
    particleSensor.setPulseAmplitudeGreen(0);
    particleSensor.clearFIFO();
    
    Serial.println("✓ MAX30102 initialized");
    return true;
}

// Check sensor health
bool checkSensorHealth() {
    long irValue = particleSensor.getIR();
    if (irValue == 0) return false;
    
    float temperature = particleSensor.readTemperature();
    if (temperature < -40 || temperature > 85) return false;
    
    return true;
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n\n=== ESP32 Heart Monitor Starting ===");
    
    // Initialize I2C
    Wire.begin();
    Wire.setClock(400000);
    
    // Initialize sensor with retry
    int retries = 0;
    while (!sensorHealthy && retries < 5) {
        sensorHealthy = initializeSensor();
        if (!sensorHealthy) {
            Serial.printf("Retry %d/5...\n", retries + 1);
            delay(1000);
        }
        retries++;
    }
    
    if (!sensorHealthy) {
        Serial.println("FATAL: Could not initialize sensor!");
        Serial.println("Check wiring and restart");
        while (1) delay(1000);
    }
    
    // Initialize arrays
    for (int i = 0; i < RATE_SIZE; i++) rates[i] = 0;
    for (int i = 0; i < INTERVAL_BUFFER_SIZE; i++) nnIntervals[i] = 0;
    
    // BLE Setup
    Serial.println("Initializing BLE...");
    
    BLEDevice::init(DEVICE_NAME);
    BLEDevice::setMTU(517);
    
    // Set TX power to maximum
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9);
    
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    
    BLEService *pService = pServer->createService(SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    
    pCharacteristic->addDescriptor(new BLE2902());
    pService->start();
    
    // Configure advertising for stability
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    
    // Use more stable connection parameters
    pAdvertising->setMinPreferred(0x20);  // 40ms
    pAdvertising->setMaxPreferred(0x40);  // 80ms
    
    BLEDevice::startAdvertising();
    
    Serial.println("✓ BLE Advertising started");
    Serial.println("Device Name: " + String(DEVICE_NAME));
    Serial.println("MAC Address: " + String(BLEDevice::getAddress().toString().c_str()));
    Serial.println("Ready for connection\n");
}

// Reset all sensor data
void resetSensorData() {
    intervalCount = 0;
    intervalIndex = 0;
    rateSpot = 0;
    lastBeat = 0;
    currentBPM = 0.0;
    currentIPM = 0.0;
    currentHRSTD = 0.0;
    currentRMSSD = 0.0;
    fingerPresent = false;
    
    for (int i = 0; i < RATE_SIZE; i++) rates[i] = 0;
    for (int i = 0; i < INTERVAL_BUFFER_SIZE; i++) nnIntervals[i] = 0;
    
    if (sensorHealthy) particleSensor.clearFIFO();
}

// Store NN interval
void storeInterval(uint16_t interval) {
    nnIntervals[intervalIndex] = interval;
    intervalIndex = (intervalIndex + 1) % INTERVAL_BUFFER_SIZE;
    if (intervalCount < INTERVAL_BUFFER_SIZE) intervalCount++;
}

// Calculate BPM from stored intervals
void calculateBPM() {
    if (intervalCount < 5) {
        currentBPM = 0.0;
        currentIPM = 0.0;
        return;
    }
    
    float sum = 0;
    int count = min(intervalCount, 10);
    
    for (int i = 0; i < count; i++) {
        int idx = (intervalIndex - 1 - i + INTERVAL_BUFFER_SIZE) % INTERVAL_BUFFER_SIZE;
        sum += 60000.0 / nnIntervals[idx];
    }
    
    currentBPM = sum / count;
    currentIPM = currentBPM;
}

// Calculate HRSTD
void calculateHRSTD() {
    if (intervalCount < 10) {
        currentHRSTD = 0.0;
        return;
    }
    
    int count = min(intervalCount, INTERVAL_BUFFER_SIZE);
    
    float mean = 0;
    for (int i = 0; i < count; i++) mean += nnIntervals[i];
    mean /= count;
    
    float sumSquaredDiff = 0;
    for (int i = 0; i < count; i++) {
        float diff = nnIntervals[i] - mean;
        sumSquaredDiff += diff * diff;
    }
    
    currentHRSTD = sqrt(sumSquaredDiff / count);
}

// Calculate RMSSD
void calculateRMSSD() {
    if (intervalCount < 10) {
        currentRMSSD = 0.0;
        return;
    }
    
    int count = min(intervalCount, INTERVAL_BUFFER_SIZE);
    
    float sumSquaredDiff = 0;
    int diffCount = 0;
    
    for (int i = 1; i < count; i++) {
        float diff = (float)nnIntervals[i] - (float)nnIntervals[i-1];
        sumSquaredDiff += diff * diff;
        diffCount++;
    }
    
    if (diffCount > 0) {
        currentRMSSD = sqrt(sumSquaredDiff / diffCount);
    } else {
        currentRMSSD = 0.0;
    }
}

// Send metrics via BLE
void sendMetrics() {
    if (!deviceConnected) return;
    
    uint8_t payload[16] = {0};
    
    uint16_t bpm_scaled = (uint16_t)(currentBPM * 10);
    uint16_t ipm_scaled = (uint16_t)(currentIPM * 10);
    uint16_t hrstd_scaled = (uint16_t)(currentHRSTD * 10);
    uint16_t rmssd_scaled = (uint16_t)(currentRMSSD * 10);
    
    payload[0] = bpm_scaled & 0xFF;
    payload[1] = (bpm_scaled >> 8) & 0xFF;
    payload[2] = ipm_scaled & 0xFF;
    payload[3] = (ipm_scaled >> 8) & 0xFF;
    payload[4] = hrstd_scaled & 0xFF;
    payload[5] = (hrstd_scaled >> 8) & 0xFF;
    payload[6] = rmssd_scaled & 0xFF;
    payload[7] = (rmssd_scaled >> 8) & 0xFF;
    payload[8] = intervalCount;
    payload[9] = fingerPresent ? 1 : 0;
    
    pCharacteristic->setValue(payload, 16);
    pCharacteristic->notify();
    
    Serial.printf("BPM: %5.1f | IPM: %5.1f | HRSTD: %5.1f | RMSSD: %5.1f | Count: %2d\n",
                  currentBPM, currentIPM, currentHRSTD, currentRMSSD, intervalCount);
}

// Process heart rate sensor data
void processSensorData() {
    static uint32_t lastMetricsSend = 0;
    uint32_t now = millis();
    
    // Periodic sensor health check
    if (now - lastSensorCheck > sensorCheckInterval) {
        lastSensorCheck = now;
        if (!checkSensorHealth()) {
            Serial.println("Sensor health check failed, reinitializing...");
            sensorHealthy = initializeSensor();
            if (!sensorHealthy) {
                Serial.println("Sensor reinitialization failed");
                return;
            }
        }
    }
    
    long irValue = particleSensor.getIR();
    
    // Check for finger with hysteresis
    if (irValue < 50000) {
        if (fingerPresent) {
            fingerPresent = false;
            Serial.println("\nFinger removed\n");
            resetSensorData();
        }
        return;
    }
    
    if (!fingerPresent) {
        fingerPresent = true;
        Serial.println("\nFinger detected\n");
    }
    
    // Check for beat using SparkFun's algorithm
    if (checkForBeat(irValue) == true) {
        long delta = now - lastBeat;
        
        if (lastBeat != 0 && delta >= MIN_INTERVAL && delta <= MAX_INTERVAL) {
            storeInterval((uint16_t)delta);
            
            beatsPerMinute = 60000.0 / delta;
            
            rates[rateSpot++] = (byte)beatsPerMinute;
            rateSpot %= RATE_SIZE;
            
            beatAvg = 0;
            for (byte x = 0; x < RATE_SIZE; x++) beatAvg += rates[x];
            beatAvg /= RATE_SIZE;
            
            calculateBPM();
            calculateHRSTD();
            calculateRMSSD();
        }
        
        lastBeat = now;
    }
    
    // Send metrics via BLE every 2 seconds
    if (fingerPresent && deviceConnected && (now - lastMetricsSend >= 2000)) {
        lastMetricsSend = now;
        if (intervalCount >= 5) sendMetrics();
    }
}

void loop() {
    // Handle disconnection
    if (!deviceConnected && oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
        Serial.println("Ready for new connection");
        resetSensorData();
    }
    
    // Handle connection
    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
        Serial.println("Connection stable");
        Serial.println("Place finger on sensor\n");
        resetSensorData();
    }
    
    // Process sensor data
    if (sensorHealthy) {
        processSensorData();
    } else {
        delay(100);
    }
    
    delay(10);
}
