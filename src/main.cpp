#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h> // Needed for notifications
#include <Arduino.h>

// --- Configuration ---
const char* bleServerName = "ESP32_LeftFoot"; // Name seen during BLE scan


const int analogInputPins[] = {
    1, // A3 (ADC1)
    2, // A4 (ADC1)
    3, // A5 (ADC1)
    4,  // A8 (ADC2 - Warning: WiFi conflict potential)
    5,   // A9 (ADC2 - Warning: WiFi conflict + BOOT PIN)
    6,   // A9 (ADC2 - Warning: WiFi conflict + BOOT PIN)
    7,
    8,   // A9 (ADC2 - Warning: WiFi conflict + BOOT PIN)
};
const int numPins = sizeof(analogInputPins) / sizeof(analogInputPins[0]);

// --- Power Saving Configuration ---
#define DEBUG // Comment out to disable Serial prints for max power saving
const uint16_t CHANGE_THRESHOLD = 20; // Minimum change in ADC value to be considered "significant"

// --- BLE Configuration ---
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914c" // Keep same UUIDs if client expects them
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a9"
// --- Buzzer Configuration ---
#define BUZZER_PIN 9 // GPIO pin for the buzzer (Check your ESP32 board pinout)
#define BUZZER_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26aA" // Unique UUID for buzzer control
// --- Queue Configuration ---
#define QUEUE_SIZE 25 // Number of readings to buffer. Adjust as needed.
typedef uint16_t SensorReadingSet[numPins]; // Type definition for a set of readings

// --- Global Variables ---
BLECharacteristic *pAnalogCharacteristic = nullptr;
BLEServer *pServer = nullptr;
bool deviceConnected = false;
bool bleInitialized = false;
bool isAdvertising = false; // <-- Add this line
uint32_t lastSendTime = 0;
const long sendInterval = 100; // Send data interval (ms). CAN BE SHORTER NOW.
                               // Make this relatively short to drain the queue quickly when connected.
uint32_t lastAdvertiseTime = 0;

// Store the last readings *used for change detection*
uint16_t lastSignificantReadings[numPins];

// --- Queue Implementation (Circular Buffer) ---
SensorReadingSet readingQueue[QUEUE_SIZE];
volatile int queueHead = 0; // Index to read from
volatile int queueTail = 0; // Index to write to
volatile int queueCount = 0; // Number of items currently in the queue

// --- LED Breathing Variables ---
bool breathing = false;
uint32_t lastBreathUpdate = 0;
int breathBrightness = 0;
int breathDirection = 1;

// --- Analog Read Throttling ---
const unsigned long ANALOG_READ_INTERVAL_MS = 20; // Minimum interval between reads per pin
unsigned long lastAnalogReadTime[numPins] = {0};
uint16_t lastAnalogReadValue[numPins] = {0};
// --- Buzzer BLE Callback Handler ---
// Only define if this is the right foot
#if defined(BUZZER_PIN) && defined(BUZZER_CHARACTERISTIC_UUID)
class BuzzerCharacteristicCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string rxValue = pCharacteristic->getValue();
        if (rxValue.length() > 0) {
            #ifdef DEBUG
            Serial.print("Received Buzzer Command (HEX): ");
            for (int i = 0; i < rxValue.length(); i++)
                Serial.printf("%02X ", (int)rxValue[i]);
            Serial.println();
            #endif
            if (rxValue[0] == 0x01) {
                #ifdef DEBUG
                Serial.println("Turning Buzzer ON");
                #endif
                digitalWrite(BUZZER_PIN, HIGH);
            } else if (rxValue[0] == 0x00) {
                #ifdef DEBUG
                Serial.println("Turning Buzzer OFF");
                #endif
                digitalWrite(BUZZER_PIN, LOW);
            }
        }
    }
};
#endif
// --- BLE Server Callbacks ---
class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServerInstance) {
        deviceConnected = true;
        isAdvertising = false; // Stop tracking advertising on connect
        #ifdef DEBUG
        Serial.println("Device connected");
        #endif
        breathing = true;
        breathBrightness = 0; // Start bright
        breathDirection = 1;
        analogWrite(LED_BUILTIN, breathBrightness); // Start with LED off, will ramp up
    }

    void onDisconnect(BLEServer* pServerInstance) {
        deviceConnected = false;
        isAdvertising = false; // Will be restarted in loop
        #ifdef DEBUG
        Serial.println("Device disconnected");
        #endif
        breathing = false;
        digitalWrite(LED_BUILTIN, LOW); // Ensure LED is off
    }
};

// --- Function Declarations ---
void initBLE();
bool checkSignificantChange(const uint16_t currentReadings[]);
int readAnalogPin(int pin);
bool enqueueReading(const SensorReadingSet reading);
bool dequeueReading(SensorReadingSet destination);
bool isQueueEmpty();
bool isQueueFull();

// --- Setup ---
void setup() {
    #ifdef DEBUG
    Serial.begin(115200);
    while (!Serial);
    Serial.println("Starting ESP32 BLE Analog Server (Queue Version)...");
    #endif

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    // Only setup buzzer pin if this is the right foot
    if (strcmp(bleServerName, "ESP32_RightFoot") == 0) {
        pinMode(BUZZER_PIN, OUTPUT);
        digitalWrite(BUZZER_PIN, LOW);
    }

    // Configure ADC pins and read initial values
    #ifdef DEBUG
    Serial.print("Configured ADC Pins (GPIO Numbers): ");
    #endif
    for(int i=0; i<numPins; i++) {
        pinMode(analogInputPins[i], INPUT);
        // Consider adding analogSetAttenuation for specific pins if needed
        // Example: analogSetPinAttenuation(analogInputPins[i], ADC_11db);
        lastSignificantReadings[i] = readAnalogPin(analogInputPins[i]);
        #ifdef DEBUG
        Serial.print(analogInputPins[i]);
        if (i < numPins - 1) Serial.print(", ");
        #endif
    }
    #ifdef DEBUG
    Serial.println();
    Serial.println("!!! Verify these GPIO numbers match your board's labels !!!");
    Serial.println("!!! WARNING: Check for ADC2 conflicts (Pins 0, 2, 4, 12-15, 25-27) if using WiFi !!!");
    #endif

    initBLE();

    #ifdef DEBUG
    Serial.println("BLE initialized.");
    Serial.printf("Change threshold: %u, Send Interval: %ldms, Queue Size: %d\n",
                  CHANGE_THRESHOLD, sendInterval, QUEUE_SIZE);
    #endif
}

// --- Main Loop ---
void loop() {
    unsigned long currentMillis = millis();
    uint16_t currentReadings[numPins];
    bool significantChangeDetected = false;

    // --- LED Handling ---
    if (!deviceConnected) {
        // Blink rapidly while advertising
        static uint32_t lastBlink = 0;
        static bool ledState = false;
        if (currentMillis - lastBlink > 200) {
            ledState = !ledState;
            digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);
            lastBlink = currentMillis;
        }
    } else if (breathing) {
        // Slow breathing effect when connected
        if (currentMillis - lastBreathUpdate > 20) { // Slower breathing
            breathBrightness += breathDirection;
            if (breathBrightness >= 255) {
                breathBrightness = 255;
                breathDirection = -1;
            } else if (breathBrightness <= 0) {
                breathBrightness = 0;
                breathDirection = 1;
            }
            // Map linear brightness to perceived brightness (approx gamma correction)
            float perceivedBrightness = pow((float)breathBrightness / 255.0, 2.2) * 255.0;
             analogWrite(LED_BUILTIN, (int)perceivedBrightness); // Use analogWrite for ESP32 PWM
            lastBreathUpdate = currentMillis;
        }
    }

    // 1. Read current sensor values (with per-pin interval)
    for (int i = 0; i < numPins; i++) {
        if (currentMillis - lastAnalogReadTime[i] >= ANALOG_READ_INTERVAL_MS) {
            uint16_t val = readAnalogPin(analogInputPins[i]);
            if (val == 65535) {
                #ifdef DEBUG
                Serial.printf("Warning: Failed to read ADC pin %d (Likely ADC2 conflict)\n", analogInputPins[i]);
                #endif
                val = lastSignificantReadings[i];
            }
            lastAnalogReadValue[i] = val;
            lastAnalogReadTime[i] = currentMillis;
        }
        currentReadings[i] = lastAnalogReadValue[i];
    }

    // 2. Check for significant change compared to the last DETECTED change
    significantChangeDetected = checkSignificantChange(currentReadings);

    if (significantChangeDetected) {
        memcpy(lastSignificantReadings, currentReadings, sizeof(lastSignificantReadings));
        if (enqueueReading(currentReadings)) {
            #ifdef DEBUG
            Serial.printf("Significant change detected. Enqueued. Queue size: %d/%d\n", queueCount, QUEUE_SIZE);
            #endif
        } else {
            #ifdef DEBUG
            Serial.println("!!! Queue Full! Dropping latest reading. !!!");
            #endif
        }
    }

    // 3. Handle BLE Connection and Sending Data from Queue
    if (deviceConnected && pAnalogCharacteristic != nullptr) {
        // Try to send data from the queue if the interval has passed and the queue is not empty
        if (currentMillis - lastSendTime >= sendInterval && !isQueueEmpty()) {
            SensorReadingSet readingToSend;
            if (dequeueReading(readingToSend)) {
                uint8_t dataBuffer[numPins * 2];
                for (int i = 0; i < numPins; i++) {
                    dataBuffer[i * 2]     = readingToSend[i] & 0xFF;
                    dataBuffer[i * 2 + 1] = (readingToSend[i] >> 8) & 0xFF;
                }
                pAnalogCharacteristic->setValue(dataBuffer, sizeof(dataBuffer));
                pAnalogCharacteristic->notify();
                lastSendTime = currentMillis;
                #ifdef DEBUG
                Serial.printf("Sent reading from queue. Queue size: %d/%d\n", queueCount, QUEUE_SIZE);
                #endif
            }
            else {
                #ifdef DEBUG
                Serial.println("Warning: Dequeue failed despite queue not being empty?");
                #endif
            }
        }
        // --- Force send stagnant data if no queue and 1s passed ---
        else if (isQueueEmpty() && (currentMillis - lastSendTime >= 1000)) {
            // Send the lastSignificantReadings (stagnant data)
            uint8_t dataBuffer[numPins * 2];
            for (int i = 0; i < numPins; i++) {
                dataBuffer[i * 2]     = lastSignificantReadings[i] & 0xFF;
                dataBuffer[i * 2 + 1] = (lastSignificantReadings[i] >> 8) & 0xFF;
            }
            pAnalogCharacteristic->setValue(dataBuffer, sizeof(dataBuffer));
            pAnalogCharacteristic->notify();
            lastSendTime = currentMillis;
            #ifdef DEBUG
            Serial.println("Sent stagnant data (no change) due to 1s timeout.");
            #endif
        }
    } else if (!deviceConnected) {
        // Not connected. Ensure advertising is running periodically.
        if (bleInitialized && pServer != nullptr) {
            if (currentMillis - lastAdvertiseTime >= 1000) { // Check every second
                BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
                if (!isAdvertising) { // Only start if not already advertising
                    #ifdef DEBUG
                    Serial.println("Device not connected, starting advertising...");
                    #endif
                    pAdvertising->start();
                    isAdvertising = true;
                    lastAdvertiseTime = currentMillis;
                } else {
                    #ifdef DEBUG
                    // Serial.println("Advertising already active."); // Reduce verbose output
                     #endif
                }
            }
        } else if (!bleInitialized) {
            #ifdef DEBUG
            Serial.println("BLE not initialized, cannot advertise yet.");
            #endif
        }
    }

    yield(); // Allow background tasks (like BLE stack) to run
}

// --- Helper Functions ---

// Wrapper for analogRead, handles ADC2 potential issues (returns 65535 on error)
int readAnalogPin(int pin) {
    // Simple approach for this code (no WiFi assumed active during read): just use analogRead
    // Note: analogRead returns uint16_t (0-4095 for 12-bit)
    int reading = analogRead(pin);

    // Add basic check if ADC2 might be involved, though without WiFi active, it's less likely to fail.
    // GPIOs 0, 2, 4, 12, 13, 14, 15, 25, 26, 27 are ADC2 pins.
    // This check is rudimentary. A proper check involves esp-idf functions if WiFi is active.
    if (pin == 0 || pin == 2 || pin == 4 || (pin >= 12 && pin <= 15) || (pin >= 25 && pin <= 27)) {
        // If a read on an ADC2 pin consistently fails (returns 0 or 4095 unexpectedly),
        // it might indicate an issue, but analogRead itself doesn't directly signal errors like ESP_ERR_TIMEOUT.
        // We'll rely on checking for significant change logic rather than trying complex ADC2 error handling here.
    }
     // Simple check for potentially invalid readings (though 0 and 4095 are valid readings)
     // If you consistently get 0 or 4095 when you shouldn't, investigate the pin/circuit.
     // Returning a special value like 65535 (-1 cast to uint16_t) if error *could* be detected.
     // Since analogRead() doesn't provide error status, this function can't reliably return an error code.
     // We'll handle potential bad reads by comparing against last known good values in the main loop.
     return reading;

    // --- More complex ADC2 handling (if WiFi were active) ---
    /*
    int reading = -1; // Use -1 to indicate error, will cast to 65535 later
    adc_unit_t unit = ADC_UNIT_1; // Default to ADC1
    adc1_channel_t channel1;
    adc2_channel_t channel2;
    esp_err_t err = ESP_OK;

    // Determine ADC unit and channel from GPIO number
    err = adc_oneshot_io_to_channel(pin, &unit, (adc_channel_t*)&channel1); // Use channel1 as temp holder
    if (err != ESP_OK) {
        #ifdef DEBUG
        Serial.printf("Pin %d is not a valid ADC pin.\n", pin);
        #endif
        return 65535; // Error: Not an ADC pin
    }

    if (unit == ADC_UNIT_1) {
        channel1 = (adc1_channel_t)channel1; // Correct cast
        // For ADC1, analogRead is generally safe
        reading = analogRead(pin);
        // Could use adc1_get_raw for more control if needed
    } else { // ADC_UNIT_2
        channel2 = (adc2_channel_t)channel1; // Correct cast
        // For ADC2, analogRead *can* conflict with WiFi. Use adc2_get_raw for safety.
        // This requires calibration setup in `setup()` (adc2_config_channel_atten)
        // and error checking.
        err = adc2_get_raw(channel2, ADC_WIDTH_BIT_12, &reading);
        if (err == ESP_ERR_TIMEOUT) {
            // Most likely conflict with WiFi
            #ifdef DEBUG
            Serial.printf("ADC2 read timeout on pin %d (WiFi conflict?)\n", pin);
            #endif
            return 65535; // Indicate error
        } else if (err != ESP_OK) {
            #ifdef DEBUG
            Serial.printf("ADC2 read error on pin %d: %s\n", pin, esp_err_to_name(err));
            #endif
            return 65535; // Indicate other error
        }
        // Note: adc2_get_raw returns raw value, may need calibration/conversion depending on setup
    }

    if (reading < 0 || reading > 4095) { // Check bounds after potential raw read
        return 65535; // Indicate error/invalid reading
    }

    return (uint16_t)reading;
    */
}

// Checks if the current readings differ significantly from the last recorded ones
bool checkSignificantChange(const uint16_t currentReadings[]) {
    for (int i = 0; i < numPins; i++) {
        // Use abs() for difference calculation
        if (abs((int32_t)currentReadings[i] - (int32_t)lastSignificantReadings[i]) > CHANGE_THRESHOLD) {
            return true; // Significant change detected on at least one pin
        }
    }
    return false; // No significant change
}

// --- Queue Functions ---

bool isQueueEmpty() {
    return queueCount == 0;
}

bool isQueueFull() {
    return queueCount == QUEUE_SIZE;
}

// Adds a reading set to the queue
bool enqueueReading(const SensorReadingSet reading) {
    if (isQueueFull()) {
        return false; // Queue is full
    }
    // Copy data to the tail position
    memcpy(readingQueue[queueTail], reading, sizeof(SensorReadingSet));
    // Advance tail index (circularly)
    queueTail = (queueTail + 1) % QUEUE_SIZE;
    // Increment count (use noInterrupts/interrupts for safety if ISRs were involved)
    // taskENTER_CRITICAL(); // If using FreeRTOS tasks
    queueCount++;
    // taskEXIT_CRITICAL();
    return true;
}

// Removes a reading set from the queue
bool dequeueReading(SensorReadingSet destination) {
    if (isQueueEmpty()) {
        return false; // Queue is empty
    }
    // Copy data from the head position
    memcpy(destination, readingQueue[queueHead], sizeof(SensorReadingSet));
    // Advance head index (circularly)
    queueHead = (queueHead + 1) % QUEUE_SIZE;
    // Decrement count (use noInterrupts/interrupts for safety if ISRs were involved)
    // taskENTER_CRITICAL();
    queueCount--;
    // taskEXIT_CRITICAL();
    return true;
}


// --- BLE Initialization ---
void initBLE() {
    if (!BLEDevice::getInitialized()) {
        BLEDevice::init(bleServerName);
        #ifdef DEBUG
        Serial.println("BLEDevice initialized.");
        #endif
        // BLEDevice::setPower(ESP_PWR_LVL_N6);
    } else {
        #ifdef DEBUG
        Serial.println("BLEDevice already initialized.");
        #endif
    }
    if (pServer == nullptr) {
        pServer = BLEDevice::createServer();
        pServer->setCallbacks(new MyServerCallbacks());

        BLEService *pService = pServer->createService(SERVICE_UUID);

        pAnalogCharacteristic = pService->createCharacteristic(
            CHARACTERISTIC_UUID,
            BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_NOTIFY
        );
        pAnalogCharacteristic->addDescriptor(new BLE2902());
        #ifdef DEBUG
        Serial.printf("Analog Characteristic UUID: %s\n", CHARACTERISTIC_UUID);
        #endif

        // Only add buzzer characteristic if this is the right foot
        if (strcmp(bleServerName, "ESP32_RightFoot") == 0) {
            BLECharacteristic *pBuzzerCharacteristic = pService->createCharacteristic(
                BUZZER_CHARACTERISTIC_UUID,
                BLECharacteristic::PROPERTY_WRITE
            );
            pBuzzerCharacteristic->setCallbacks(new BuzzerCharacteristicCallbacks());
            #ifdef DEBUG
            Serial.printf("Buzzer Characteristic UUID: %s\n", BUZZER_CHARACTERISTIC_UUID);
            #endif
        }

        pService->start();

        BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
        // ...existing advertising setup...
        bleInitialized = true;
        #ifdef DEBUG
        Serial.println("BLE Server/Service/Characteristics Initialized.");
        #endif
    } else {
        #ifdef DEBUG
        Serial.println("BLE Server already exists.");
        #endif
    }

    // Ensure advertising starts if not connected initially
    if (!deviceConnected) {
        BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
        if (!isAdvertising) {
            pAdvertising->start();
            isAdvertising = true;
            #ifdef DEBUG
            Serial.println("Initial advertising started.");
            #endif
            lastAdvertiseTime = millis();
        }
    }
}