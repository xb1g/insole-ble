#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h> // Needed for notifications
#include <Arduino.h>

// --- Configuration ---
#define bleServerName "ESP32_LeftFoot" // Name seen during BLE scan

// *** CRITICAL: VERIFY PINS FOR YOUR SPECIFIC ESP32 BOARD ***
// This maps common Arduino-style labels (A0-A5, A8, A9) to ESP32 GPIO numbers.
// Double-check your board's pinout diagram!
//
// COMMON MAPPINGS:
// A0 = GPIO 36 (ADC1_CH0) - Input only
// A1 = GPIO 37 (ADC1_CH1) - Input only, *often missing*
// A2 = GPIO 38 (ADC1_CH2) - Input only, *often missing*
// A3 = GPIO 39 (ADC1_CH3) - Input only
// A4 = GPIO 32 (ADC1_CH4)
// A5 = GPIO 33 (ADC1_CH5)
// A6 = GPIO 34 (ADC1_CH6) - Input only
// A7 = GPIO 35 (ADC1_CH7) - Input only
// A8 = GPIO 4  (ADC2_CH0) - !!! ADC2 !!!
// A9 = GPIO 0  (ADC2_CH1) - !!! ADC2 + BOOT PIN !!!
//
// !!! WARNINGS !!!
// 1. ADC2 PINS (GPIO 0, 2, 4, 12-15, 25-27): Cannot be used with analogRead()
//    while WiFi is active. This code doesn't use WiFi, but be aware.
// 2. GPIO 0 (A9 / ADC2_CH1): Is a strapping pin (BOOT). Connecting sensors
//    here can interfere with booting/flashing if it pulls the pin LOW.
//    *Strongly recommended to avoid GPIO 0 if possible.*
// 3. GPIOs 37, 38 (A1, A2): May not exist on your board. Check pinout.
//
// USING: A0, A3, A4, A5, A8, A9 (Skipping potentially missing A1, A2)
const int analogPins[] = {
    1, // A3 (ADC1)
    2, // A4 (ADC1)
    3, // A5 (ADC1)
    4,  // A8 (ADC2 - Warning: WiFi conflict potential)
    5,   // A9 (ADC2 - Warning: WiFi conflict + BOOT PIN)
    6,   // A9 (ADC2 - Warning: WiFi conflict + BOOT PIN)
    7,
    8,   // A9 (ADC2 - Warning: WiFi conflict + BOOT PIN)
};
const int numPins = sizeof(analogPins) / sizeof(analogPins[0]);

// --- Power Saving Configuration ---
// #define DEBUG // Comment out to disable Serial prints for max power saving
// Remove WAKE_INTERVAL_US and INACTIVITY_TIMEOUT_MS, no sleep needed
const uint16_t CHANGE_THRESHOLD = 20; // Minimum change in ADC value to be considered "significant"

// --- BLE Configuration ---
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914c" // Keep same UUIDs if client expects them
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a9"

// --- Buzzer Configuration ---
#define isRight  // Define this only for the right insole
#define BUZZER_PIN 9
#define BUZZER_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26aa"

#ifdef isRight
// --- Buzzer Callbacks ---
class BuzzerCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string rxValue = pCharacteristic->getValue();
        if (rxValue.length() > 0) {
            if (rxValue[0] == 0x01) {
                digitalWrite(BUZZER_PIN, HIGH);  // Turn buzzer on
            } else if (rxValue[0] == 0x00) {
                digitalWrite(BUZZER_PIN, LOW);   // Turn buzzer off
            }
        }
    }
};
#endif
// --- Global Variables ---
BLECharacteristic *pAnalogCharacteristic = nullptr; // Initialize pointer
BLEServer *pServer = nullptr;                     // Initialize pointer
bool deviceConnected = false;
bool bleInitialized = false; // Track if BLE stack needs full init
// Remove lastSignificantChangeTime and INACTIVITY_TIMEOUT_MS, no inactivity timer needed
uint32_t lastSendTime = 0;
const long sendInterval = 500; // Send data interval when connected and active (ms)
uint32_t lastAdvertiseTime = 0; // For BLE reconnect interval

// Store the last readings that were considered significant
uint16_t lastSignificantReadings[numPins];

// Add for LED breathing
bool breathing = false;
uint32_t lastBreathUpdate = 0;
int breathBrightness = 0;
int breathDirection = 1; // 1 = brighter, -1 = dimmer

// --- Queue Configuration ---
#define QUEUE_SIZE 15  // Number of readings to buffer for smoothing
typedef struct {
    uint16_t readings[numPins];
} SensorReading;

SensorReading readingQueue[QUEUE_SIZE];
int queueHead = 0;  // Index to read from
int queueTail = 0;  // Index to write to
int queueCount = 0; // Number of items currently in queue

// Helper functions for queue operations
void enqueueReading(const uint16_t* readings) {
    // Copy readings to the queue
    memcpy(readingQueue[queueTail].readings, readings, sizeof(uint16_t) * numPins);
    
    queueTail = (queueTail + 1) % QUEUE_SIZE;
    if (queueCount < QUEUE_SIZE) {
        queueCount++;
    } else {
        // Queue is full, move head to discard oldest reading
        queueHead = (queueHead + 1) % QUEUE_SIZE;
    }
}

void calculateAverageReadings(uint16_t* result) {
    if (queueCount == 0) {
        memset(result, 0, sizeof(uint16_t) * numPins);
        return;
    }

    // Initialize sums
    uint32_t sums[numPins] = {0};
    
    // Sum all readings in the queue
    int idx = queueHead;
    for (int i = 0; i < queueCount; i++) {
        for (int j = 0; j < numPins; j++) {
            sums[j] += readingQueue[idx].readings[j];
        }
        idx = (idx + 1) % QUEUE_SIZE;
    }
    
    // Calculate averages
    for (int i = 0; i < numPins; i++) {
        result[i] = sums[i] / queueCount;
    }
}

// --- BLE Server Callbacks ---
class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServerInstance) {
        deviceConnected = true;
        #ifdef DEBUG
        Serial.println("Device connected");
        #endif
        // Start breathing effect
        breathing = true;
        // Ensure LED is ON at start of breathing
        analogWrite(LED_BUILTIN, 0);
    }

    void onDisconnect(BLEServer* pServerInstance) {
        deviceConnected = false;
        #ifdef DEBUG
        Serial.println("Device disconnected");
        #endif
        // Stop breathing effect, turn off LED
        breathing = false;
        analogWrite(LED_BUILTIN, 0);
    }
};

// --- Function Declarations ---
void initBLE();
bool checkSignificantChange(uint16_t currentReadings[]);
int readAnalogPin(int pin); // Helper for potential ADC2 workaround

// --- Setup ---
void setup() {
    #ifdef DEBUG
    Serial.begin(115200);
    while (!Serial); // Wait for serial monitor
    Serial.println("Starting ESP32 BLE Analog Server (Optimized)...");
    #endif

    // Initialize LED_BUILTIN
    pinMode(LED_BUILTIN, OUTPUT);
    analogWrite(LED_BUILTIN, 0);

    if (bleServerName == "ESP32_RightFoot") {
        pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, HIGH);  // Start with buzzer off
    delay(100); // Short delay to ensure buzzer is off
    digitalWrite(BUZZER_PIN, LOW);  // Start with buzzer off
    } else {
        #ifdef DEBUG
        Serial.println("Left insole detected.");
        #endif
    }

    // Configure ADC
    // analogReadResolution(12); // Default is 12-bit
    // analogSetAttenuation(ADC_11db); // Default is 11dB (adjust per pin if needed)
    // Example: Set attenuation for specific pins if voltage range differs
    // adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11); // GPIO 36 (A0)
    // adc2_config_channel_atten(ADC2_CHANNEL_0, ADC_ATTEN_DB_11); // GPIO 4 (A8) - Note ADC2

    #ifdef DEBUG
    Serial.print("Configured ADC Pins (GPIO Numbers): ");
    for(int i=0; i<numPins; i++) {
        Serial.print(analogPins[i]);
        if (i < numPins - 1) Serial.print(", ");
    }
    Serial.println();
    Serial.println("!!! Verify these GPIO numbers match your board's A0-A5, A8, A9 labels !!!");
    Serial.println("!!! WARNING: Pins 4 (A8) and 0 (A9) are ADC2. May conflict with WiFi. Pin 0 is BOOT pin. !!!");
    #endif

    // Initialize last readings (read once)
    for (int i = 0; i < numPins; i++) {
        pinMode(analogPins[i], INPUT);
        lastSignificantReadings[i] = readAnalogPin(analogPins[i]);
    }
    // Remove lastSignificantChangeTime = millis();

    // Remove wake-up reason and deep sleep timer config
    // Always initialize BLE on boot
    initBLE();

    #ifdef DEBUG
    Serial.println("BLE initialized on boot.");
    Serial.printf("Change threshold set to %u.\n", CHANGE_THRESHOLD);
    #endif
}

// --- Main Loop ---
void loop() {
    unsigned long currentMillis = millis();
    uint16_t currentReadings[numPins];
    bool significantChangeDetected = false;

    // --- LED Handling ---
    if (!deviceConnected) {
        // Blink rapidly while advertising (trying to connect)
        static uint32_t lastBlink = 0;
        static bool ledState = false;
        if (currentMillis - lastBlink > 200) {
            ledState = !ledState;
            digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);
            lastBlink = currentMillis;
        }
    } else if (breathing) {
        // Slow breathing effect when connected
        if (currentMillis - lastBreathUpdate > 10) {
            // Breath period: ~3s up, ~3s down (adjust step for slower/faster)
            breathBrightness += breathDirection;
            if (breathBrightness >= 255) {
                breathBrightness = 255;
                breathDirection = -1;
            } else if (breathBrightness <= 0) {
                breathBrightness = 0;
                breathDirection = 1;
            }
            analogWrite(LED_BUILTIN, breathBrightness);
            lastBreathUpdate = currentMillis;
        }
    }

    // 1. Read current sensor values
    for (int i = 0; i < numPins; i++) {
        currentReadings[i] = readAnalogPin(analogPins[i]);
        if (currentReadings[i] == -1) {
             #ifdef DEBUG
             Serial.printf("Warning: Failed to read ADC pin %d (Likely ADC2 conflict if WiFi was active)\n", analogPins[i]);
             #endif
             currentReadings[i] = lastSignificantReadings[i];
        }
    }

    // Add current readings to the queue
    enqueueReading(currentReadings);

    // Get smoothed readings for comparison and sending
    uint16_t smoothedReadings[numPins];
    calculateAverageReadings(smoothedReadings);

    // 2. Check for significant change using smoothed readings
    significantChangeDetected = checkSignificantChange(smoothedReadings);

    if (significantChangeDetected) {
        #ifdef DEBUG
        Serial.print("Significant change detected. Readings (GPIO:Value): ");
        for(int i=0; i<numPins; i++) Serial.printf("[%d]:%d ", analogPins[i], currentReadings[i]);
        Serial.println();
        #endif
        memcpy(lastSignificantReadings, currentReadings, sizeof(lastSignificantReadings));
        // Remove lastSignificantChangeTime = currentMillis;
    }

    // 3. Handle BLE Connection and Sending Data
    if (deviceConnected) {
        if (currentMillis - lastSendTime >= sendInterval) {
            lastSendTime = currentMillis;
            uint8_t dataBuffer[numPins * 2];
            uint16_t smoothedReadings[numPins];
            calculateAverageReadings(smoothedReadings);
            for (int i = 0; i < numPins; i++) {
                dataBuffer[i * 2]     = smoothedReadings[i] & 0xFF;
                dataBuffer[i * 2 + 1] = (smoothedReadings[i] >> 8) & 0xFF;
            }
            if (pAnalogCharacteristic != nullptr) {
                 pAnalogCharacteristic->setValue(dataBuffer, sizeof(dataBuffer));
                 pAnalogCharacteristic->notify();
            } else {
                 #ifdef DEBUG
                 Serial.println("Error: pAnalogCharacteristic is null! Cannot send.");
                 #endif
            }
        }
    } else {
        // Not connected. Ensure advertising is running every second if BLE is initialized.
        if (bleInitialized && pServer != nullptr) {
            if (currentMillis - lastAdvertiseTime >= 1000) {
                BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
                #ifdef DEBUG
                Serial.println("Device not connected, (re)starting advertising...");
                #endif
                pAdvertising->start();
                lastAdvertiseTime = currentMillis;
            }
        } else if (!bleInitialized) {
             #ifdef DEBUG
             Serial.println("BLE not initialized, cannot advertise yet.");
             #endif
        }
    }

    // Remove inactivity/sleep logic

    yield();
}

// --- Helper Functions ---

// Wrapper for analogRead, handles ADC2 potential issues (returns -1 on error)
int readAnalogPin(int pin) {
    // Simple approach for this code (no WiFi): just use analogRead
     return analogRead(pin);

    // If WiFi WAS active, you'd need a more complex check using adc2_get_raw():
    /*
    int reading = -1;
    adc2_channel_t channel;
    if (digitalPinToAnalogChannel(pin) >= SOC_ADC_MAX_CHANNEL_NUM) { // Check if it's an ADC2 pin
        // Convert GPIO to ADC2 channel
        if (adc2_pad_get_io_num((adc2_channel_t)digitalPinToAnalogChannel(pin), &channel) == ESP_OK) {
             esp_err_t err = adc2_get_raw(channel, ADC_WIDTH_BIT_12, &reading);
             if (err == ESP_ERR_TIMEOUT || err == ESP_ERR_INVALID_STATE) {
                 // ADC2 is busy (likely WiFi) or not calibrated
                 return -1; // Indicate error
             } else if (err != ESP_OK) {
                 // Other ADC error
                 return -1;
             }
        } else {
            return -1; // Invalid GPIO for ADC2
        }
    } else {
        // It's an ADC1 pin
        reading = analogRead(pin);
    }
    return reading;
    */
}


bool checkSignificantChange(uint16_t currentReadings[]) {
    for (int i = 0; i < numPins; i++) {
        // Use abs() for difference calculation, cast to signed int to handle wrap-around correctly
        if (abs((int32_t)currentReadings[i] - (int32_t)lastSignificantReadings[i]) > CHANGE_THRESHOLD) {
            return true; // Significant change detected
        }
    }
    return false; // No significant change
}

void initBLE() {
    // Simple re-init approach for wake-from-sleep:
    // Assume previous objects are invalid/stale after sleep.
    // The BLEDevice state might persist partially, but server/service/char likely need recreation.

    // If BLE stack was truly running before sleep, a full deinit might be cleaner,
    // but can sometimes cause issues. Let's try re-creating objects directly.
    // if (bleInitialized && pServer != nullptr) {
        // Maybe clean up?
        // delete pServer; // Risky if callbacks are still somehow active
        // pServer = nullptr;
        // pAnalogCharacteristic = nullptr; // Pointer is now invalid anyway
        // BLEDevice::deinit(false); // Gentle deinit?
    // }

    // Ensure core BLE Device is initialized
    if (!BLEDevice::getInitialized()) { // Check if core BLE is initialized
         BLEDevice::init(bleServerName);
          // Optional: Set lower BLE transmit power (saves power, reduces range)
         // Options: ESP_PWR_LVL_N12, N9, N6, N3, N0, P3, P6, P9 (default P3)
         // BLEDevice::setPower(ESP_PWR_LVL_N6); // Example: -6dBm
         #ifdef DEBUG
         Serial.println("BLEDevice initialized.");
         #endif
    } else {
         #ifdef DEBUG
         Serial.println("BLEDevice already initialized.");
         #endif
    }

    // Create the BLE Server (or get existing if library handles persistence?)
    // Safer to assume we need to create it again after sleep.
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks()); // Create NEW callback object

    // Create the BLE Service
    BLEService *pService = pServer->createService(SERVICE_UUID);

    // Create a BLE Characteristic
    pAnalogCharacteristic = pService->createCharacteristic(
                              CHARACTERISTIC_UUID,
                              BLECharacteristic::PROPERTY_READ |
                              BLECharacteristic::PROPERTY_NOTIFY
                            );

    // Add the CCCD descriptor for notifications
    pAnalogCharacteristic->addDescriptor(new BLE2902()); // Essential for notifications

    #ifdef isRight
    // Create buzzer characteristic (only for right insole)
    BLECharacteristic *pBuzzerCharacteristic = pService->createCharacteristic(
        BUZZER_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );
    pBuzzerCharacteristic->setCallbacks(new BuzzerCallbacks());
    #endif

    // Start the service
    pService->start();

    // Configure Advertising
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06); // Helps iOS/connection reliability (Continuity Info)
    // Consider longer advertising intervals for power saving if discovery speed isn't critical
    // pAdvertising->setMinInterval(0x80); // Example: 80 * 0.625ms = 50ms
    // pAdvertising->setMaxInterval(0x100); // Example: 160 * 0.625ms = 100ms

    // Don't start advertising here. Let the loop handle it based on connection status.
    bleInitialized = true; // Mark BLE setup as complete for this wake cycle

    #ifdef DEBUG
    Serial.println("BLE Server/Service/Characteristic Initialized/Re-initialized.");
    Serial.printf("Connect to '%s'\n", bleServerName);
    Serial.printf("Service UUID: %s\n", SERVICE_UUID);
    Serial.printf("Characteristic UUID: %s\n", CHARACTERISTIC_UUID);
    #endif
}
