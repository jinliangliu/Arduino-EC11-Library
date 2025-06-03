# EC11 Rotary Encoder Library for Arduino

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
![Platform](https://img.shields.io/badge/Platform-ESP32%20|%20ESP8266%20|%20AVR-blue)

Advanced driver for EC11 rotary encoders with robust event detection and hardware-level optimization. Designed for reliable operation in embedded systems requiring precise input handling.

```cpp
#include <ec11encoder.h>

EC11Encoder encoder(2, 3, 4); // SA:GPIO2, SB:GPIO3, SW:GPIO4

void setup() {
  encoder.begin();
  encoder.onRotation([](int8_t dir, bool fast) {
    Serial.printf("%s rotation: %s\n", 
      fast ? "Fast" : "Normal", 
      dir > 0 ? "CW" : "CCW");
  });
  encoder.onSwitch([](EC11Encoder::SwEvent event) {
    if(event == EC11Encoder::SwEvent::ShortClick) Serial.println("Click");
  });
}

void loop() {
  encoder.update();
}
```

## Key Features

- Quadrature Decoding - State-machine based phase analysis
- Dual Modes - Hardware interrupts (ESP) & optimized polling
- Multi-Instance - Up to 3 encoders with auto conflict detection
- Precise Timing - Configurable debounce (50ms) and long-press (2000ms)
- Cross-Platform - Unified API for AVR/ESP32/ESP8266

## Installation

**Using Arduino CLI**:
```bash
arduino-cli lib install "EC11Encoder"
```
**Using Arduino IDE**:
1. Open Arduino IDE.
2. Go to Sketch -> Include Library -> Manage Libraries...
3. Search for "EC11Encoder".
4. Click "Install".

**Manual Installation**:
1. [Download latest release](https://github.com/jinliangliu/Arduino-EC11-Library/releases/latest)  
   ![release badge](https://img.shields.io/github/v/release/jinliangliu/Arduino-EC11-Library?style=flat-square)
2. Extract ZIP to Arduino libraries folder  
   `~/Documents/Arduino/libraries/` (Windows/Mac)
3. Restart Arduino IDE

## Applications

- Industrial control interfaces
- Automotive dashboard controls
- Professional audio equipment
- IoT device navigation