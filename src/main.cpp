#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h> // Needed for notifications
#include <Arduino.h>
#include <esp_sleep.h> // Needed for deep sleep

// --- Configuration ---
#define bleServerName "ESP32_RightFoot" // Name seen during BLE scan

// *** CRITICAL: VERIFY PINS FOR YOUR BOARD ***
// Use ADC1 pins. Common choices: 32, 33, 34, 35, 36, 39.
// Avoid ADC2 pins if using WiFi (not used here, but good practice).
// Avoid pins used for Flash (usually 6, 7, 8, 9, 10, 11).
// Avoid pins used for default Serial (1, 3).
const int analogPins[] = {1,2,3,4,5,6,7,8}; // EXAMPLE: Using 4 common ADC1 pins
// const int analogPins[] = {34, 35, 32, 33, 36, 39}; // EXAMPLE: Using 6 common ADC1 pins
const int numPins = sizeof(analogPins) / sizeof(analogPins[0]);

// --- Power Saving Configuration ---
#define DEBUG // Comment out to disable Serial prints for max power saving
const uint64_t WAKE_INTERVAL_US = 1 * 1000 * 1000; // Wake up every 1 second (in microseconds)
const uint32_t INACTIVITY_TIMEOUT_MS = 60 * 1000; // Go to sleep after 1 minute of inactivity (in milliseconds)
const uint16_t CHANGE_THRESHOLD = 20; // Minimum change in ADC value to be considered "significant"
// Optional: Reduce CPU Frequency for power saving (80MHz is often lowest stable)
// Requires #include <soc/soc.h>, #include <soc/rtc_cntl_reg.h>
// Set in setup(): WRITE_PERI_REG(RTC_CNTL_CLK_CONF_REG, (READ_PERI_REG(RTC_CNTL_CLK_CONF_REG) & 0xffffff00) | 0x01); // 80MHz XTAL

// --- BLE Configuration ---
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914c" // Keep same UUIDs if client expects them
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a9"

// --- Global Variables ---
BLECharacteristic *pAnalogCharacteristic = nullptr; // Initialize pointer
BLEServer *pServer = nullptr;                     // Initialize pointer
bool deviceConnected = false;
bool bleInitialized = false; // Track if BLE stack needs full init
uint32_t lastSignificantChangeTime = 0;
uint32_t lastSendTime = 0;
const long sendInterval = 1000; // Send data interval when connected and active (ms)

// Store the last readings that were considered significant
uint16_t lastSignificantReadings[numPins];

// --- BLE Server Callbacks ---
class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServerInstance) { // Renamed pServer to avoid conflict
        deviceConnected = true;
        #ifdef DEBUG
        Serial.println("Device connected");
        #endif
        // Optional: Update connection parameters for lower power when connected
        // Example: Slower interval, higher latency tolerance
        // pServerInstance->updateConnParams(pServerInstance->getConnId(), 0x30, 0x30, 0, 400); // 60ms interval, 4s timeout
    }

    void onDisconnect(BLEServer* pServerInstance) {
        deviceConnected = false;
        #ifdef DEBUG
        Serial.println("Device disconnected");
        #endif
        // Advertising should restart automatically when we wake up if needed
        // If woken by timer and were disconnected, loop logic will restart it.
    }
};

// --- Function Declarations ---
void initBLE();
void goToDeepSleep();
bool checkSignificantChange(uint16_t currentReadings[]);

// --- Setup ---
void setup() {
    #ifdef DEBUG
    Serial.begin(115200);
    while (!Serial); // Wait for serial monitor
    Serial.println("Starting ESP32 BLE Analog Server (Optimized)...");
    #endif

    // Optional: Reduce CPU frequency (uncomment include directives above too)
    // WRITE_PERI_REG(RTC_CNTL_CLK_CONF_REG, (READ_PERI_REG(RTC_CNTL_CLK_CONF_REG) & 0xffffff00) | 0x01); // Set CPU to 80MHz

    // Configure ADC
    // analogReadResolution(12); // Default is 12-bit
    // analogSetAttenuation(ADC_11db); // Default is 11dB

    #ifdef DEBUG
    Serial.print("Configured ADC Pins: ");
    for(int i=0; i<numPins; i++) {
        Serial.print(analogPins[i]);
        if (i < numPins - 1) Serial.print(", ");
    }
    Serial.println();
    #endif

    // Initialize last readings (read once)
    for (int i = 0; i < numPins; i++) {
        lastSignificantReadings[i] = analogRead(analogPins[i]);
    }
    lastSignificantChangeTime = millis(); // Assume initial state is "active"

    // Check wake-up reason
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

    if (wakeup_reason != ESP_SLEEP_WAKEUP_TIMER) {
        // First boot or wake from other source (not timer)
        #ifdef DEBUG
        Serial.println("Woke up from reset or non-timer event. Initializing BLE.");
        #endif
        initBLE(); // Full BLE initialization
    } else {
        // Woke up from timer (deep sleep)
        #ifdef DEBUG
        Serial.println("Woke up from timer deep sleep.");
        #endif
        // BLE might still be advertising or connected if the sleep was short,
        // but often the connection is lost. We'll check connection status
        // and restart advertising if needed in the loop.
        // Re-init BLE objects, but not device itself.
        initBLE(); // Re-initialize server/service/characteristic pointers and start service
    }

    // Configure deep sleep timer wake-up
    esp_sleep_enable_timer_wakeup(WAKE_INTERVAL_US);
    #ifdef DEBUG
    Serial.printf("Wake timer set to %llu us.\n", WAKE_INTERVAL_US);
    Serial.printf("Inactivity timeout set to %lu ms.\n", INACTIVITY_TIMEOUT_MS);
    Serial.printf("Change threshold set to %u.\n", CHANGE_THRESHOLD);
    #endif
}

// --- Main Loop ---
void loop() {
    unsigned long currentMillis = millis();
    uint16_t currentReadings[numPins];
    bool significantChangeDetected = false;

    // 1. Read current sensor values
    for (int i = 0; i < numPins; i++) {
        currentReadings[i] = analogRead(analogPins[i]);
    }

    // 2. Check for significant change
    significantChangeDetected = checkSignificantChange(currentReadings);

    if (significantChangeDetected) {
        #ifdef DEBUG
        Serial.print("Significant change detected. Readings: ");
        for(int i=0; i<numPins; i++) Serial.printf("[%d]:%d ", analogPins[i], currentReadings[i]);
        Serial.println();
        #endif
        // Update last significant readings and reset inactivity timer
        memcpy(lastSignificantReadings, currentReadings, sizeof(lastSignificantReadings));
        lastSignificantChangeTime = currentMillis;
    }

    // 3. Handle BLE Connection and Sending Data
    if (deviceConnected) {
        // Send data periodically ONLY if connected
        if (currentMillis - lastSendTime >= sendInterval) {
            lastSendTime = currentMillis;

            // Prepare buffer (use last significant readings)
            uint8_t dataBuffer[numPins * 2];
            for (int i = 0; i < numPins; i++) {
                dataBuffer[i * 2]     = lastSignificantReadings[i] & 0xFF;        // Low byte
                dataBuffer[i * 2 + 1] = (lastSignificantReadings[i] >> 8) & 0xFF; // High byte
            }

            if (pAnalogCharacteristic != nullptr) { // Check if characteristic exists
                 pAnalogCharacteristic->setValue(dataBuffer, sizeof(dataBuffer));
                 pAnalogCharacteristic->notify();
                 #ifdef DEBUG
                 Serial.printf("Sent %d bytes via BLE notification.\n", sizeof(dataBuffer));
                 #endif
            } else {
                 #ifdef DEBUG
                 Serial.println("Error: pAnalogCharacteristic is null!");
                 #endif
            }
        }
    } else {
        // Not connected. Ensure advertising is running if BLE is initialized.
        // The BLE stack might handle restarts, but explicit check is safer after sleep.
        if (bleInitialized && pServer != nullptr) {
            // Ensure advertising is running if BLE is initialized
            BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
            #ifdef DEBUG
            Serial.println("Device not connected, starting advertising...");
            #endif
            pAdvertising->start(); // Directly start advertising (idempotent)
        }
    }

    // 4. Check for Inactivity and Go to Sleep
    if (currentMillis - lastSignificantChangeTime > INACTIVITY_TIMEOUT_MS) {
        #ifdef DEBUG
        Serial.printf("No significant change for %lu ms. Entering deep sleep.\n", INACTIVITY_TIMEOUT_MS);
        #endif
        goToDeepSleep();
    }

    // Yield for background tasks (like BLE stack) - important!
    // A small delay can sometimes help stability, but yield() is preferred.
    yield();
    // delay(10); // Use yield() instead if possible
}

// --- Helper Functions ---

bool checkSignificantChange(uint16_t currentReadings[]) {
    for (int i = 0; i < numPins; i++) {
        // Use abs() for difference calculation
        if (abs((int16_t)currentReadings[i] - (int16_t)lastSignificantReadings[i]) > CHANGE_THRESHOLD) {
            return true; // Significant change detected
        }
    }
    return false; // No significant change
}

void goToDeepSleep() {
    #ifdef DEBUG
    Serial.flush(); // Ensure serial messages are sent before sleeping
    #endif

    // Stop advertising before sleeping
    if (bleInitialized) {
         BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
         pAdvertising->stop(); // Directly stop advertising
         #ifdef DEBUG
         Serial.println("Stopped BLE advertising.");
         #endif
         // Optional: Disconnect client explicitly? Deep sleep usually breaks it anyway.
         // if(deviceConnected && pServer != nullptr) {
         //    pServer->disconnect(pServer->getConnId());
         // }
    }

    #ifdef DEBUG
    Serial.println("Going to sleep now.");
    delay(100); // Short delay to allow serial flush again
    #endif

    esp_deep_sleep_start();
}

void initBLE() {
    if (bleInitialized) {
        // If already initialized (e.g., wake from sleep),
        // we might just need to restart services/advertising if needed.
        // However, pointers might be invalid after sleep. Re-creating seems safer.
        // Clean up previous objects if they exist (optional, depends on BLE library internals)
        // BLEDevice::deinit(true); // Full deinit - might be too aggressive
    }

    // Initialize BLE Device (only once ideally, but safe to call again)
    if (!BLEDevice::getInitialized()) { // Check if core BLE is initialized
         BLEDevice::init(bleServerName);
          // Optional: Set lower BLE transmit power (saves power, reduces range)
         // Options: ESP_PWR_LVL_N12, N9, N6, N3, N0, P3, P6, P9 (default P3)
         // BLEDevice::setPower(ESP_PWR_LVL_N6); // Example: -6dBm
    }


    // Create the BLE Server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Create the BLE Service
    BLEService *pService = pServer->createService(SERVICE_UUID);

    // Create a BLE Characteristic
    pAnalogCharacteristic = pService->createCharacteristic(
                              CHARACTERISTIC_UUID,
                              BLECharacteristic::PROPERTY_READ |
                              BLECharacteristic::PROPERTY_NOTIFY
                            );

    pAnalogCharacteristic->addDescriptor(new BLE2902()); // Essential for notifications

    // Start the service
    pService->start();

    // Configure Advertising
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06); // Helps iOS/connection reliability
    // Consider longer advertising intervals for power saving if discovery speed isn't critical
    // pAdvertising->setMinInterval(0x80); // Example: 80 * 0.625ms = 50ms
    // pAdvertising->setMaxInterval(0x100); // Example: 160 * 0.625ms = 100ms

    // Don't start advertising here directly, let the loop handle it based on connection status.
    bleInitialized = true;

    #ifdef DEBUG
    Serial.println("BLE Initialized/Re-initialized.");
    Serial.printf("Connect to '%s'\n", bleServerName);
    Serial.printf("Service UUID: %s\n", SERVICE_UUID);
    Serial.printf("Characteristic UUID: %s\n", CHARACTERISTIC_UUID);
    #endif
}
