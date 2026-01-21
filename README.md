# Smart Environmental Monitoring System

A professional ESP32-based environmental monitoring system featuring real-time temperature and humidity sensing, a 5-LED bargraph display, user-configurable thresholds, and low-power operation.

Built with **FreeRTOS** for concurrent task execution and **interrupt-driven button handling** for responsive user interaction.

---

## Features

- **Real-time Sensing**: DHT22 temperature and humidity readings with invalid data rejection
- **5-LED Bargraph Display**: Visual indicator with two display modes
  - *Threshold Mode*: Shows distance from user-set threshold
  - *Absolute Mode*: Shows measurement level within fixed ranges
- **User-Adjustable Threshold**: Potentiometer-based ADC input for setting thresholds
- **Three Push Buttons**: Toggle measurement mode, LED mode, and power mode
- **Serial Command Interface**: Remote control via USB serial connection
- **Energy-Saving Mode**: 10-second light sleep with button wake capability
- **Concurrent Architecture**: FreeRTOS tasks for parallel sensor reading and display updates

---

## Hardware Requirements

| Component | Quantity | Specification |
|-----------|----------|---------------|
| ESP32-WROOM-32 DevKit | 1 | 3.3V logic, USB powered |
| DHT22 Sensor | 1 | Temperature & humidity sensor |
| Potentiometer | 1 | 10kΩ recommended |
| LEDs | 5 | Green, Green, Yellow, Orange, Red |
| Resistors | 5 | 220-330Ω (max 500Ω) |
| Push Buttons | 3 | Momentary, normally open |
| Breadboard | 1 | With jumper wires |
| USB Cable | 1 | For programming and serial monitor |

---

## Pin Configuration

| Signal | GPIO | Description |
|--------|------|-------------|
| LED 1 (Green) | 33 | Bargraph level 1 (lowest) |
| LED 2 (Green) | 25 | Bargraph level 2 |
| LED 3 (Yellow) | 26 | Bargraph level 3 |
| LED 4 (Orange) | 27 | Bargraph level 4 |
| LED 5 (Red) | 14 | Bargraph level 5 (highest) |
| Potentiometer | 34 | ADC input for threshold |
| DHT22 Data | 23 | Sensor data pin |
| Button 1 | 0 | Toggle TEMP/HUM mode |
| Button 2 | 4 | Toggle THRESHOLD/ABSOLUTE mode |
| Button 3 | 15 | Toggle NORMAL/ENERGY_SAVING mode |

### Wiring Notes

- **LEDs**: Anode → GPIO (through resistor), Cathode → GND
- **Buttons**: One terminal → GPIO, other terminal → GND (internal pull-ups enabled)
- **Potentiometer**: One end → 3.3V, other end → GND, wiper → GPIO 34
- **DHT22**: VCC → 3.3V, GND → GND, Data → GPIO 23

---

## Software Requirements

- [PlatformIO](https://platformio.org/) (recommended) or Arduino IDE
- ESP32 board support package
- DHT sensor library (automatically installed via PlatformIO)

---

## Installation & Setup

### Using PlatformIO (Recommended)

1. **Clone the repository**
   ```bash
   git clone https://github.com/MAFerjani/LED-bargraph-monitoring-system.git
   cd LED-bargraph-monitoring-system
   ```

2. **Install dependencies**
   ```bash
   pio pkg install
   ```

3. **Build the project**
   ```bash
   pio run
   ```

4. **Upload to ESP32**
   ```bash
   pio run -t upload
   ```

5. **Open Serial Monitor**
   ```bash
   pio device monitor
   ```

### Using Arduino IDE

1. Install ESP32 board support via Board Manager
2. Install "DHT sensor library" by Adafruit via Library Manager
3. Open `src/main.cpp` and rename to `main.ino`
4. Select board: "ESP32 Dev Module"
5. Set upload speed: 115200
6. Upload and open Serial Monitor at 115200 baud

---

## Usage

### Button Controls

| Button | GPIO | Function | Action |
|--------|------|----------|--------|
| Button 1 | 0 | Measurement Mode | Toggle between TEMPERATURE and HUMIDITY |
| Button 2 | 4 | LED Display Mode | Toggle between THRESHOLD and ABSOLUTE |
| Button 3 | 15 | Power Mode | Toggle between NORMAL and ENERGY_SAVING |

### Serial Commands

Connect via USB and open a serial terminal at **115200 baud**.

| Command | Description |
|---------|-------------|
| `ON` | Resume full system operation |
| `OFF` | Turn system off (LEDs off, sensing paused) |
| `MODE TEMP` | Force temperature measurement mode |
| `MODE HUM` | Force humidity measurement mode |
| `SLEEP` | Immediately enter 10-second light sleep |

### Serial Output Format

```
===== Smart Environment Monitor =====
Measurement : TEMP
LED Mode    : THRESHOLD
Power Mode  : NORMAL
System      : ON
Temperature : 25.3 °C
Humidity    : 48.2 %
Threshold   : 27.5 °C
LEDs Active : [1 2 3    ]
=====================================
```

---

## LED Bargraph Mapping

### Threshold Mode

LEDs indicate how far the measurement is from the threshold.

**Temperature (Δnear = ±2°C, Δfar = ±5°C)**

| Condition | Range | LEDs ON |
|-----------|-------|---------|
| Far below threshold | T ≤ threshold - 5°C | LED 1 |
| Below threshold | threshold - 5°C < T ≤ threshold - 2°C | LED 1-2 |
| Near threshold | \|T - threshold\| ≤ 2°C | LED 1-3 |
| Above threshold | threshold + 2°C < T ≤ threshold + 5°C | LED 1-4 |
| Much above threshold | T > threshold + 5°C | LED 1-5 |

**Humidity (Δnear = ±4%, Δfar = ±10%)**

| Condition | Range | LEDs ON |
|-----------|-------|---------|
| Far below threshold | H ≤ threshold - 10% | LED 1 |
| Below threshold | threshold - 10% < H ≤ threshold - 4% | LED 1-2 |
| Near threshold | \|H - threshold\| ≤ 4% | LED 1-3 |
| Above threshold | threshold + 4% < H ≤ threshold + 10% | LED 1-4 |
| Much above threshold | H > threshold + 10% | LED 1-5 |

### Absolute Mode

LEDs represent the raw measurement within fixed ranges.

**Temperature (20-40°C)**

| Range | LEDs ON |
|-------|---------|
| 20-24°C | LED 1 |
| 24-28°C | LED 1-2 |
| 28-32°C | LED 1-3 |
| 32-36°C | LED 1-4 |
| 36-40°C | LED 1-5 |

**Humidity (30-80%)**

| Range | LEDs ON |
|-------|---------|
| 30-40% | LED 1 |
| 40-50% | LED 1-2 |
| 50-60% | LED 1-3 |
| 60-70% | LED 1-4 |
| 70-80% | LED 1-5 |

---

## Architecture

### Why FreeRTOS?

This implementation uses **FreeRTOS** (included with ESP32 Arduino core) for several reasons:

1. **Concurrent Execution**: Sensor reading, threshold calculation, LED updates, and serial handling run as independent tasks without blocking each other.

2. **Deterministic Timing**: `vTaskDelayUntil()` ensures consistent task periods regardless of execution time variations.

3. **Priority-Based Scheduling**: Higher-priority tasks (display updates) preempt lower-priority ones (potentiometer reading) when needed.

4. **Dual-Core Utilization**: Tasks are pinned to specific cores—serial handling on Core 0, sensor/display on Core 1—for optimal performance.

### Task Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                        ESP32 Dual Core                       │
├──────────────────────────┬──────────────────────────────────┤
│         Core 0           │            Core 1                 │
├──────────────────────────┼──────────────────────────────────┤
│   vSerialTask (50ms)     │   vSensorTask (2000ms)           │
│   - Handle commands      │   - Read DHT22                   │
│   - Parse input          │   - Validate readings            │
│                          │                                  │
│                          │   vPotTask (100ms)               │
│                          │   - Read ADC                     │
│                          │   - Calculate threshold          │
│                          │                                  │
│                          │   vDisplayTask (1000ms)          │
│                          │   - Update LEDs                  │
│                          │   - Print status                 │
└──────────────────────────┴──────────────────────────────────┘
                              │
                    ┌─────────┴─────────┐
                    │   gStateMutex     │
                    │  (Thread Safety)  │
                    └───────────────────┘
```

### Why Interrupt-Driven Buttons?

Buttons use hardware interrupts instead of polling for these advantages:

1. **Immediate Response**: Button presses are detected instantly, not on the next poll cycle.

2. **Works During Sleep**: GPIO interrupts are configured as wake sources, allowing buttons to wake the ESP32 from light sleep before the 10-second timer expires.

3. **Lower CPU Usage**: No continuous polling loop required.

4. **Proper Debouncing**: ISRs capture timestamps; debouncing logic in the main loop ensures clean state transitions without blocking.

```cpp
// ISR: Minimal, runs in IRAM for speed
void IRAM_ATTR isrButton1() {
    gButton1Time = millis();
    gButton1Pressed = true;
}

// Main loop: Handles debouncing safely
if (gButton1Pressed && (now - gLastButton1Process) > DEBOUNCE_TIME_MS) {
    // Process button action
    gButton1Pressed = false;
}
```

### Thread Safety

All shared state is protected by a FreeRTOS mutex:

```cpp
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

// Every access follows this pattern:
if (xSemaphoreTake(gStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    // Read or modify gState
    xSemaphoreGive(gStateMutex);
}
```

---

## Energy-Saving Mode

When enabled (Button 3 or serial command), the system:

1. Completes one full update cycle (read → calculate → display → print)
2. Configures wake sources:
   - **Timer**: 10-second automatic wake
   - **GPIO**: Any button press triggers early wake
3. Enters ESP32 light sleep
4. Wakes and reports wake reason (timer or button)

```
>>> Entering 10-second light sleep...
>>> Press any button to wake early
<<< Woke up: Button press detected!
```

---

## Project Structure

```
Micro_project_EE535/
├── src/
│   └── main.cpp          # Main firmware source
├── include/
│   └── README            # PlatformIO include folder docs
├── lib/
│   └── README            # PlatformIO library folder docs
├── test/
│   └── README            # PlatformIO test folder docs
├── platformio.ini        # PlatformIO configuration
├── micro_project.txt     # Project specification
├── micro_project_report.tex  # LaTeX technical report
└── README.md             # This file
```

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| DHT readings show 0 or NaN | Check wiring; ensure DHT data pin has pull-up resistor if required by module |
| Buttons don't respond | Verify button wiring (GPIO to GND when pressed); check internal pull-ups are enabled |
| LEDs don't light | Check polarity (anode to GPIO); verify resistor values (220-330Ω) |
| Serial monitor shows garbage | Set baud rate to 115200 |
| Upload fails | Hold BOOT button during upload; check USB cable supports data |

---

## License

This project was developed for the EE535 Microcontrollers course at the Faculty of Electrical and Electronic Engineering, Tripoli.

---

## Author

**Muhammed Ali Muftah**  
Student ID: 2200208982  
Instructor: Eng. Ramadan Altuiby
