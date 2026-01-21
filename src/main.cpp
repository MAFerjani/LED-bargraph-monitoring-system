/**
 * Smart Environmental Monitoring System with BarGraph Display
 * and Low-Power Operation Using ESP32
 * 
 * Features:
 * - FreeRTOS task-based architecture
 * - Interrupt-driven button handling with debouncing
 * - DHT22 temperature/humidity sensor
 * - 5-LED bargraph display (Threshold and Absolute modes)
 * - Potentiometer-based threshold adjustment
 * - Serial command interface
 * - 10-second light sleep with GPIO wake sources
 */

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/timers.h>
#include "DHT.h"
#include "esp_sleep.h"
#include "driver/gpio.h"

// ============================================================================
//                              PIN CONFIGURATION
// ============================================================================
const int LED_PINS[5] = {33, 25, 26, 27, 14};  // Green, Green, Yellow, Orange, Red
const int POT_PIN = 34;
const int SENSOR_PIN = 23;
const int BUTTON_PINS[3] = {0, 4, 15};  // Mode, LED Mode, Power Mode

// ============================================================================
//                              MODE ENUMERATIONS
// ============================================================================
enum SensorMode  { TEMP_MODE, HUM_MODE };
enum LedMode     { THRESHOLD_MODE, ABSOLUTE_MODE };
enum PowerMode   { NORMAL_MODE, ENERGY_SAVING_MODE };

// ============================================================================
//                         THRESHOLD MAPPING CONSTANTS
// ============================================================================
// From report: Temperature threshold range 20-35°C, Humidity 30-80%
const float TEMP_THRESH_MIN = 20.0f;
const float TEMP_THRESH_MAX = 35.0f;
const float HUM_THRESH_MIN  = 30.0f;
const float HUM_THRESH_MAX  = 80.0f;

// Absolute mode ranges (from report)
const float TEMP_ABS_MIN = 20.0f;
const float TEMP_ABS_MAX = 40.0f;
const float HUM_ABS_MIN  = 30.0f;
const float HUM_ABS_MAX  = 80.0f;

// Threshold mode deltas (from report)
const float DELTA_NEAR_T = 2.0f;   // ±2°C for "near threshold"
const float DELTA_FAR_T  = 5.0f;   // ±5°C for "far from threshold"
const float DELTA_NEAR_H = 4.0f;   // ±4% for humidity
const float DELTA_FAR_H  = 10.0f;  // ±10% for humidity

// ============================================================================
//                              TIMING CONSTANTS
// ============================================================================
const TickType_t SENSOR_TASK_PERIOD   = pdMS_TO_TICKS(2000);  // DHT needs ~2s between reads
const TickType_t POT_TASK_PERIOD      = pdMS_TO_TICKS(100);   // Fast ADC polling
const TickType_t DISPLAY_TASK_PERIOD  = pdMS_TO_TICKS(1000);  // 1s display update
const TickType_t SERIAL_TASK_PERIOD   = pdMS_TO_TICKS(50);    // Fast serial polling
const uint32_t   DEBOUNCE_TIME_MS     = 50;                   // Button debounce
const uint64_t   SLEEP_DURATION_US    = 10000000;             // 10 seconds in microseconds

// ============================================================================
//                              TASK PRIORITIES
// ============================================================================
const UBaseType_t PRIORITY_SENSOR  = 2;
const UBaseType_t PRIORITY_POT     = 1;
const UBaseType_t PRIORITY_DISPLAY = 3;
const UBaseType_t PRIORITY_SERIAL  = 2;

// ============================================================================
//                           GLOBAL SHARED STATE
// ============================================================================
// Protected by mutex
typedef struct {
    float temperature;
    float humidity;
    float threshold;
    SensorMode sensorMode;
    LedMode ledMode;
    PowerMode powerMode;
    bool systemOn;
    int activeLedCount;
} SystemState;

SystemState gState = {
    .temperature = 0.0f,
    .humidity = 0.0f,
    .threshold = 25.0f,
    .sensorMode = TEMP_MODE,
    .ledMode = THRESHOLD_MODE,
    .powerMode = NORMAL_MODE,
    .systemOn = true,
    .activeLedCount = 1
};

// Synchronization primitives
SemaphoreHandle_t gStateMutex = NULL;

// Task handles (for suspend/resume during sleep)
TaskHandle_t hSensorTask  = NULL;
TaskHandle_t hPotTask     = NULL;
TaskHandle_t hDisplayTask = NULL;
TaskHandle_t hSerialTask  = NULL;

// DHT sensor instance
DHT dht(SENSOR_PIN, DHT22);

// ============================================================================
//                      BUTTON INTERRUPT HANDLING
// ============================================================================
// Volatile flags set by ISRs
volatile bool gButton1Pressed = false;
volatile bool gButton2Pressed = false;
volatile bool gButton3Pressed = false;

// Timestamps for debouncing (set in ISR)
volatile uint32_t gButton1Time = 0;
volatile uint32_t gButton2Time = 0;
volatile uint32_t gButton3Time = 0;

// Last processed time for debouncing
uint32_t gLastButton1Process = 0;
uint32_t gLastButton2Process = 0;
uint32_t gLastButton3Process = 0;

// ISR for Button 1 (GPIO 0) - Toggle Sensor Mode (TEMP/HUM)
void IRAM_ATTR isrButton1() {
    gButton1Time = millis();
    gButton1Pressed = true;
}

// ISR for Button 2 (GPIO 4) - Toggle LED Mode (THRESHOLD/ABSOLUTE)
void IRAM_ATTR isrButton2() {
    gButton2Time = millis();
    gButton2Pressed = true;
}

// ISR for Button 3 (GPIO 15) - Toggle Power Mode (NORMAL/ENERGY_SAVING)
void IRAM_ATTR isrButton3() {
    gButton3Time = millis();
    gButton3Pressed = true;
}

// ============================================================================
//                          FUNCTION PROTOTYPES
// ============================================================================
void vSensorTask(void *pvParameters);
void vPotTask(void *pvParameters);
void vDisplayTask(void *pvParameters);
void vSerialTask(void *pvParameters);

void processButtons();
void updateLEDs(int count);
void allLEDsOff();
int calculateThresholdLevel(float value, float threshold, float deltaNear, float deltaFar);
int calculateAbsoluteLevel(float value, float minVal, float maxVal);
void printStatus();
void enterLightSleep();
void configureWakeSources();

// ============================================================================
//                                 SETUP
// ============================================================================
void setup() {
    // Initialize Serial
    Serial.begin(115200);
    while (!Serial) { delay(10); }
    
    Serial.println();
    Serial.println("========================================");
    Serial.println(" Smart Environmental Monitoring System");
    Serial.println("     ESP32 + FreeRTOS Implementation");
    Serial.println("========================================");

    // Initialize LED pins
    for (int i = 0; i < 5; i++) {
        pinMode(LED_PINS[i], OUTPUT);
        digitalWrite(LED_PINS[i], LOW);
    }

    // Initialize button pins with internal pull-ups
    for (int i = 0; i < 3; i++) {
        pinMode(BUTTON_PINS[i], INPUT_PULLUP);
    }

    // Attach interrupts to buttons (FALLING edge = button press with pull-up)
    attachInterrupt(digitalPinToInterrupt(BUTTON_PINS[0]), isrButton1, FALLING);
    attachInterrupt(digitalPinToInterrupt(BUTTON_PINS[1]), isrButton2, FALLING);
    attachInterrupt(digitalPinToInterrupt(BUTTON_PINS[2]), isrButton3, FALLING);

    // Initialize potentiometer pin
    pinMode(POT_PIN, INPUT);

    // Initialize DHT sensor
    dht.begin();

    // Create mutex for state protection
    gStateMutex = xSemaphoreCreateMutex();
    if (gStateMutex == NULL) {
        Serial.println("ERROR: Failed to create mutex!");
        while (1) { delay(1000); }
    }

    // Create tasks
    BaseType_t result;

    result = xTaskCreatePinnedToCore(
        vSensorTask,
        "SensorTask",
        4096,
        NULL,
        PRIORITY_SENSOR,
        &hSensorTask,
        1  // Run on core 1
    );
    if (result != pdPASS) Serial.println("ERROR: Failed to create SensorTask");

    result = xTaskCreatePinnedToCore(
        vPotTask,
        "PotTask",
        2048,
        NULL,
        PRIORITY_POT,
        &hPotTask,
        1
    );
    if (result != pdPASS) Serial.println("ERROR: Failed to create PotTask");

    result = xTaskCreatePinnedToCore(
        vDisplayTask,
        "DisplayTask",
        4096,
        NULL,
        PRIORITY_DISPLAY,
        &hDisplayTask,
        1
    );
    if (result != pdPASS) Serial.println("ERROR: Failed to create DisplayTask");

    result = xTaskCreatePinnedToCore(
        vSerialTask,
        "SerialTask",
        4096,
        NULL,
        PRIORITY_SERIAL,
        &hSerialTask,
        0  // Run on core 0 (protocol core)
    );
    if (result != pdPASS) Serial.println("ERROR: Failed to create SerialTask");

    Serial.println("All tasks created successfully!");
    Serial.println("Commands: ON, OFF, MODE TEMP, MODE HUM, SLEEP");
    Serial.println("========================================");
    Serial.println();
}

// ============================================================================
//                              MAIN LOOP
// ============================================================================
void loop() {
    // Process button interrupts in main loop (debouncing)
    processButtons();

    // Check for energy saving mode
    bool shouldSleep = false;
    
    if (xSemaphoreTake(gStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        shouldSleep = (gState.powerMode == ENERGY_SAVING_MODE) && gState.systemOn;
        xSemaphoreGive(gStateMutex);
    }

    if (shouldSleep) {
        // Give tasks time to complete one cycle
        vTaskDelay(pdMS_TO_TICKS(100));
        enterLightSleep();
    }

    vTaskDelay(pdMS_TO_TICKS(10));  // Small delay to prevent tight loop
}

// ============================================================================
//                            BUTTON PROCESSING
// ============================================================================
void processButtons() {
    uint32_t now = millis();

    // Button 1: Toggle Sensor Mode (TEMP/HUM)
    if (gButton1Pressed) {
        if ((now - gLastButton1Process) > DEBOUNCE_TIME_MS) {
            gLastButton1Process = now;
            
            if (xSemaphoreTake(gStateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                gState.sensorMode = (gState.sensorMode == TEMP_MODE) ? HUM_MODE : TEMP_MODE;
                Serial.print("[BUTTON] Sensor mode: ");
                Serial.println((gState.sensorMode == TEMP_MODE) ? "TEMPERATURE" : "HUMIDITY");
                xSemaphoreGive(gStateMutex);
            }
        }
        gButton1Pressed = false;
    }

    // Button 2: Toggle LED Mode (THRESHOLD/ABSOLUTE)
    if (gButton2Pressed) {
        if ((now - gLastButton2Process) > DEBOUNCE_TIME_MS) {
            gLastButton2Process = now;
            
            if (xSemaphoreTake(gStateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                gState.ledMode = (gState.ledMode == THRESHOLD_MODE) ? ABSOLUTE_MODE : THRESHOLD_MODE;
                Serial.print("[BUTTON] LED mode: ");
                Serial.println((gState.ledMode == THRESHOLD_MODE) ? "THRESHOLD" : "ABSOLUTE");
                xSemaphoreGive(gStateMutex);
            }
        }
        gButton2Pressed = false;
    }

    // Button 3: Toggle Power Mode (NORMAL/ENERGY_SAVING)
    if (gButton3Pressed) {
        if ((now - gLastButton3Process) > DEBOUNCE_TIME_MS) {
            gLastButton3Process = now;
            
            if (xSemaphoreTake(gStateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                gState.powerMode = (gState.powerMode == NORMAL_MODE) ? ENERGY_SAVING_MODE : NORMAL_MODE;
                Serial.print("[BUTTON] Power mode: ");
                Serial.println((gState.powerMode == NORMAL_MODE) ? "NORMAL" : "ENERGY_SAVING");
                xSemaphoreGive(gStateMutex);
            }
        }
        gButton3Pressed = false;
    }
}

// ============================================================================
//                            SENSOR TASK
// ============================================================================
void vSensorTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    float t, h;

    while (true) {
        // Read both temperature and humidity
        t = dht.readTemperature();
        h = dht.readHumidity();

        // Validate and update state
        if (xSemaphoreTake(gStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            // Only update if readings are valid
            if (!isnan(t) && t >= -40.0f && t <= 80.0f) {
                gState.temperature = t;
            }
            if (!isnan(h) && h >= 0.0f && h <= 100.0f) {
                gState.humidity = h;
            }
            xSemaphoreGive(gStateMutex);
        }

        // Wait for next period
        vTaskDelayUntil(&xLastWakeTime, SENSOR_TASK_PERIOD);
    }
}

// ============================================================================
//                          POTENTIOMETER TASK
// ============================================================================
void vPotTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    int adcValue;
    float threshold;
    SensorMode currentMode;

    while (true) {
        // Read ADC
        adcValue = analogRead(POT_PIN);

        // Get current sensor mode
        if (xSemaphoreTake(gStateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            currentMode = gState.sensorMode;
            xSemaphoreGive(gStateMutex);
        }

        // Map ADC to threshold based on mode
        // Formula: threshold = min + (ADC/4095) * (max - min)
        if (currentMode == TEMP_MODE) {
            threshold = TEMP_THRESH_MIN + ((float)adcValue / 4095.0f) * (TEMP_THRESH_MAX - TEMP_THRESH_MIN);
        } else {
            threshold = HUM_THRESH_MIN + ((float)adcValue / 4095.0f) * (HUM_THRESH_MAX - HUM_THRESH_MIN);
        }

        // Update state
        if (xSemaphoreTake(gStateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            gState.threshold = threshold;
            xSemaphoreGive(gStateMutex);
        }

        vTaskDelayUntil(&xLastWakeTime, POT_TASK_PERIOD);
    }
}

// ============================================================================
//                            DISPLAY TASK
// ============================================================================
void vDisplayTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    // Local copies of state
    float temperature, humidity, threshold;
    SensorMode sensorMode;
    LedMode ledMode;
    bool systemOn;
    int ledCount;

    while (true) {
        // Get current state
        if (xSemaphoreTake(gStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            temperature = gState.temperature;
            humidity = gState.humidity;
            threshold = gState.threshold;
            sensorMode = gState.sensorMode;
            ledMode = gState.ledMode;
            systemOn = gState.systemOn;
            xSemaphoreGive(gStateMutex);
        } else {
            // Couldn't get mutex, skip this cycle
            vTaskDelayUntil(&xLastWakeTime, DISPLAY_TASK_PERIOD);
            continue;
        }

        if (!systemOn) {
            allLEDsOff();
            vTaskDelayUntil(&xLastWakeTime, DISPLAY_TASK_PERIOD);
            continue;
        }

        // Calculate LED count based on mode
        float currentValue = (sensorMode == TEMP_MODE) ? temperature : humidity;

        if (ledMode == THRESHOLD_MODE) {
            if (sensorMode == TEMP_MODE) {
                ledCount = calculateThresholdLevel(currentValue, threshold, DELTA_NEAR_T, DELTA_FAR_T);
            } else {
                ledCount = calculateThresholdLevel(currentValue, threshold, DELTA_NEAR_H, DELTA_FAR_H);
            }
        } else {  // ABSOLUTE_MODE
            if (sensorMode == TEMP_MODE) {
                ledCount = calculateAbsoluteLevel(currentValue, TEMP_ABS_MIN, TEMP_ABS_MAX);
            } else {
                ledCount = calculateAbsoluteLevel(currentValue, HUM_ABS_MIN, HUM_ABS_MAX);
            }
        }

        // Update LEDs
        updateLEDs(ledCount);

        // Update state with active LED count
        if (xSemaphoreTake(gStateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            gState.activeLedCount = ledCount;
            xSemaphoreGive(gStateMutex);
        }

        // Print status
        printStatus();

        vTaskDelayUntil(&xLastWakeTime, DISPLAY_TASK_PERIOD);
    }
}

// ============================================================================
//                            SERIAL TASK
// ============================================================================
void vSerialTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    String command;

    while (true) {
        if (Serial.available() > 0) {
            command = Serial.readStringUntil('\n');
            command.trim();
            command.toUpperCase();

            if (command.length() > 0) {
                if (xSemaphoreTake(gStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    
                    if (command == "ON") {
                        gState.systemOn = true;
                        Serial.println("[CMD] System ON - Full operation resumed");
                    }
                    else if (command == "OFF") {
                        gState.systemOn = false;
                        allLEDsOff();
                        Serial.println("[CMD] System OFF - All LEDs off, reading paused");
                    }
                    else if (command == "MODE TEMP") {
                        gState.sensorMode = TEMP_MODE;
                        Serial.println("[CMD] Forced to TEMPERATURE mode");
                    }
                    else if (command == "MODE HUM") {
                        gState.sensorMode = HUM_MODE;
                        Serial.println("[CMD] Forced to HUMIDITY mode");
                    }
                    else if (command == "SLEEP") {
                        Serial.println("[CMD] Immediate 10-second sleep requested");
                        xSemaphoreGive(gStateMutex);
                        
                        // Print status before sleep
                        printStatus();
                        Serial.println("Entering light sleep...");
                        delay(50);  // Let serial flush
                        
                        enterLightSleep();
                        
                        Serial.println("Woke up from sleep!");
                        xLastWakeTime = xTaskGetTickCount();
                        continue;  // Skip the normal delay
                    }
                    else {
                        Serial.print("[CMD] Unknown: ");
                        Serial.println(command);
                        Serial.println("Available: ON, OFF, MODE TEMP, MODE HUM, SLEEP");
                    }
                    
                    xSemaphoreGive(gStateMutex);
                }
            }
        }

        vTaskDelayUntil(&xLastWakeTime, SERIAL_TASK_PERIOD);
    }
}

// ============================================================================
//                      LED LEVEL CALCULATIONS
// ============================================================================

/**
 * Calculate LED count for THRESHOLD mode
 * Based on distance from threshold using near/far deltas
 * 
 * From report:
 * - Far below threshold (value <= thresh - deltaFar): 1 LED
 * - Below threshold (thresh - deltaFar < value <= thresh - deltaNear): 2 LEDs
 * - Near threshold (|value - thresh| <= deltaNear): 3 LEDs
 * - Above threshold (thresh + deltaNear < value <= thresh + deltaFar): 4 LEDs
 * - Much above threshold (value > thresh + deltaFar): 5 LEDs
 */
int calculateThresholdLevel(float value, float threshold, float deltaNear, float deltaFar) {
    float diff = value - threshold;

    if (diff <= -deltaFar) {
        return 1;  // Far below threshold
    } else if (diff <= -deltaNear) {
        return 2;  // Below threshold
    } else if (diff <= deltaNear) {
        return 3;  // Near threshold (within ±deltaNear)
    } else if (diff <= deltaFar) {
        return 4;  // Above threshold
    } else {
        return 5;  // Much above threshold
    }
}

/**
 * Calculate LED count for ABSOLUTE mode
 * Divides the range into 5 equal parts
 * 
 * From report (Temperature 20-40°C):
 * - 20-24°C: 1 LED
 * - 24-28°C: 2 LEDs
 * - 28-32°C: 3 LEDs
 * - 32-36°C: 4 LEDs
 * - 36-40°C: 5 LEDs
 */
int calculateAbsoluteLevel(float value, float minVal, float maxVal) {
    // Clamp value to range
    if (value < minVal) value = minVal;
    if (value > maxVal) value = maxVal;

    float rangeStep = (maxVal - minVal) / 5.0f;

    if (value < minVal + rangeStep) {
        return 1;
    } else if (value < minVal + 2 * rangeStep) {
        return 2;
    } else if (value < minVal + 3 * rangeStep) {
        return 3;
    } else if (value < minVal + 4 * rangeStep) {
        return 4;
    } else {
        return 5;
    }
}

// ============================================================================
//                            LED CONTROL
// ============================================================================
void updateLEDs(int count) {
    // Constrain to valid range
    if (count < 1) count = 1;
    if (count > 5) count = 5;

    // Turn on LEDs 0 to count-1, turn off the rest
    for (int i = 0; i < 5; i++) {
        digitalWrite(LED_PINS[i], (i < count) ? HIGH : LOW);
    }
}

void allLEDsOff() {
    for (int i = 0; i < 5; i++) {
        digitalWrite(LED_PINS[i], LOW);
    }
}

// ============================================================================
//                          STATUS PRINTING
// ============================================================================
void printStatus() {
    // Get state snapshot
    float temperature, humidity, threshold;
    SensorMode sensorMode;
    LedMode ledMode;
    PowerMode powerMode;
    int activeLeds;
    bool systemOn;

    if (xSemaphoreTake(gStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        temperature = gState.temperature;
        humidity = gState.humidity;
        threshold = gState.threshold;
        sensorMode = gState.sensorMode;
        ledMode = gState.ledMode;
        powerMode = gState.powerMode;
        activeLeds = gState.activeLedCount;
        systemOn = gState.systemOn;
        xSemaphoreGive(gStateMutex);
    } else {
        return;  // Couldn't get mutex
    }

    Serial.println("===== Smart Environment Monitor =====");
    
    Serial.print("Measurement : ");
    Serial.println((sensorMode == TEMP_MODE) ? "TEMP" : "HUMID");
    
    Serial.print("LED Mode    : ");
    Serial.println((ledMode == THRESHOLD_MODE) ? "THRESHOLD" : "ABSOLUTE");
    
    Serial.print("Power Mode  : ");
    Serial.println((powerMode == NORMAL_MODE) ? "NORMAL" : "ENERGY_SAVING");
    
    Serial.print("System      : ");
    Serial.println(systemOn ? "ON" : "OFF");
    
    Serial.print("Temperature : ");
    Serial.print(temperature, 1);
    Serial.println(" °C");
    
    Serial.print("Humidity    : ");
    Serial.print(humidity, 1);
    Serial.println(" %");
    
    Serial.print("Threshold   : ");
    Serial.print(threshold, 1);
    Serial.println((sensorMode == TEMP_MODE) ? " °C" : " %");
    
    Serial.print("LEDs Active : [");
    for (int i = 1; i <= 5; i++) {
        if (i <= activeLeds) {
            Serial.print(i);
        } else {
            Serial.print(" ");
        }
        if (i < 5) Serial.print(" ");
    }
    Serial.println("]");
    
    Serial.println("=====================================");
    Serial.println();
}

// ============================================================================
//                          LIGHT SLEEP
// ============================================================================
void configureWakeSources() {
    // Configure GPIO wake sources (buttons can wake the ESP32)
    // Note: Not all GPIOs support wake from light sleep
    // GPIO 0, 4, 15 should work on most ESP32 boards
    
    esp_sleep_enable_timer_wakeup(SLEEP_DURATION_US);
    
    // Enable GPIO wake for buttons (wake on LOW level since we use pull-ups)
    // This allows buttons to wake the ESP32 before the 10-second timer expires
    gpio_wakeup_enable((gpio_num_t)BUTTON_PINS[0], GPIO_INTR_LOW_LEVEL);
    gpio_wakeup_enable((gpio_num_t)BUTTON_PINS[1], GPIO_INTR_LOW_LEVEL);
    gpio_wakeup_enable((gpio_num_t)BUTTON_PINS[2], GPIO_INTR_LOW_LEVEL);
    esp_sleep_enable_gpio_wakeup();
}

void enterLightSleep() {
    Serial.println(">>> Entering 10-second light sleep...");
    Serial.println(">>> Press any button to wake early");
    Serial.flush();
    delay(50);  // Ensure serial output completes

    // Configure wake sources
    configureWakeSources();

    // Enter light sleep (all tasks automatically suspend)
    esp_light_sleep_start();

    // Execution resumes here after wake
    // Check wake cause
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    
    switch (wakeup_reason) {
        case ESP_SLEEP_WAKEUP_TIMER:
            Serial.println("<<< Woke up: Timer (10 seconds elapsed)");
            break;
        case ESP_SLEEP_WAKEUP_GPIO:
            Serial.println("<<< Woke up: Button press detected!");
            break;
        default:
            Serial.print("<<< Woke up: Reason code ");
            Serial.println(wakeup_reason);
            break;
    }
}
