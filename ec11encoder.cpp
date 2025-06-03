/******************************************************************************
 * @file ec11encoder.cpp
 * @author jinliang.liu@outlook.com
 * @brief EC11 Encoder Driver Implementation - Robust rotary & switch event handling
 *        * Key Functionality:
 *        - Quadrature decoding algorithm (4-phase state machine)
 *        - Switch debouncing with long-press detection (>2s)
 *        - Thread-safe design for single-threaded environments
 *        - Automatic rotation lock release mechanism
 *        * Typical Applications:
 *        - Human-Machine Interfaces (volume control, menu navigation)
 *        - Industrial control systems (parameter adjustment dials)
 *        - Motion sensing devices (position tracking)
 ******************************************************************************
 * @attention 
 * 
 * Copyright (c) 2025 Jinliang Liu. 
 * All rights reserved.
 * 
 * This code is licensed under the MIT License.
 * For details, see LICENSE file in project root.
 * 
 * @verbatim
 * ==============================================================================
 *                    ##### How to use this library #####
 * ==============================================================================
 * 
 * Basic Usage:
 * 1. Include header file:
 *    #include <ec11encoder.h>
 * 
 * 2. Create encoder instance (global scope):
 *    EC11Encoder encoder(PIN_SA, PIN_SB, PIN_SW); // Replace with actual pin numbers
 * 
 * 3. Initialize in setup():
 *    void setup() {
 *        encoder.begin();
 *        // Set event callbacks (optional)
 *        encoder.setRotationCallback(rotationHandler);
 *        encoder.setSwitchCallback(switchHandler);
 *    }
 * 
 * 4. Regular update in loop():
 *    void loop() {
 *        encoder.update();
 *        // Other logic...
 *    }
 * 
 * Event Handling Example:
 * // Rotation event handler
 * void rotationHandler(int8_t direction, bool fast) {
 *     Serial.print(fast ? "Fast " : "Slow ");
 *     Serial.println(direction > 0 ? "CW" : "CCW");
 * }
 * 
 * // Switch event handler
 * void switchHandler(EC11Encoder::SwEvent event) {
 *     switch(event) {
 *         case EC11Encoder::SwEventEnum::ShortClick:
 *             Serial.println("Short click");
 *             break;
 *         case EC11Encoder::SwEventEnum::LongPress:
 *             Serial.println("Long press");
 *             break;
 *     }
 * }
 * 
 * Important Notes:
 * - Supports hardware interrupts for ESP32/ESP8266, polling mode for other platforms
 * - Ensure no duplicate pin usage (auto-detected in constructor)
 * - Each encoder requires a separate instance
 * - Recommended to handle non-ISR logic in update()
 * 
 * @endverbatim
 *****************************************************************************/
#include "ec11encoder.h"

EC11Encoder::InstanceInfo EC11Encoder::_instances[EC11_MAX_INSTANCES];
uint8_t EC11Encoder::_instanceCount = 0;

EC11Encoder::EC11Encoder(uint8_t pinSA, uint8_t pinSB, uint8_t pinSW) 
    : _pinSA(pinSA), _pinSB(pinSB), _pinSW(pinSW)
{
    // Check pin conflicts
    for(uint8_t i=0; i<_instanceCount; i++){
        if(_instances[i].pinSA == pinSA || 
           _instances[i].pinSB == pinSB ||
           _instances[i].pinSW == pinSW){
            // TODO: Implement proper error handling
            Serial.println("[ERROR] EC11Encoder: Pin conflict detected (SA:" 
              + String(pinSA) + " SB:" + String(pinSB) + " SW:" + String(pinSW) + ")");
            // Consider throwing exception for Arduino framework compatibility
            while (1){ 
                delay(1000); 
            }
        }
    }
    registerInstance(pinSA, pinSB, pinSW, this);
}

EC11Encoder::~EC11Encoder() {
    end();

    // Remove from instances array if present
    for(uint8_t i=0; i<_instanceCount; i++){
        if(_instances[i].instance == this){
            _instances[i] = _instances[_instanceCount-1];
            _instanceCount--;
            break;
        }
    }
}

bool EC11Encoder::registerInstance(uint8_t sa, uint8_t sb, uint8_t sw, EC11Encoder* instance) {
    if (_instanceCount >= EC11_MAX_INSTANCES) return false;
    
    _instances[_instanceCount] = {sa, sb, sw, instance};
    _instanceCount++;
    return true;
}

EC11Encoder* EC11Encoder::findInstanceByEncoder(uint8_t sa, uint8_t sb) {
    for (uint8_t i = 0; i < _instanceCount; i++) {
        if (_instances[i].pinSA == sa && _instances[i].pinSB == sb) {
            return _instances[i].instance;
        }
    }
    return nullptr;
}

EC11Encoder* EC11Encoder::findInstanceBySwitch(uint8_t sw) {
    for (uint8_t i = 0; i < _instanceCount; i++) {
        if (_instances[i].pinSW == sw) {
            return _instances[i].instance;
        }
    }
    return nullptr;
}

#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
void IRAM_ATTR EC11Encoder::handleEncoderISR(void *arg){
    EC11Encoder* instance = static_cast<EC11Encoder*>(arg);
    uint8_t phase = (digitalRead(instance->_pinSB) << 1) | digitalRead(instance->_pinSA);
    instance->decode(phase);
}   

void IRAM_ATTR EC11Encoder::handleSwISR(void *arg){
    EC11Encoder* instance = static_cast<EC11Encoder*>(arg);
    instance->processSwitch();
}
#else 
void EC11Encoder::handleEncoderISR(){
    for(uint8_t i=0; i<_instanceCount; i++) {
        InstanceInfo& info = _instances[i];
        bool currentSA = digitalRead(info.pinSA);
        if(currentSA != (info.instance->_rotation.last_phase & 0x01)) {
            uint8_t phase = (digitalRead(info.pinSB) << 1) | currentSA;
            info.instance->decode(phase);
        }
    }
}   

void EC11Encoder::handleSwISR() {
    uint32_t now = millis();
    for(uint8_t i=0; i<_instanceCount; i++) {
        InstanceInfo& info = _instances[i];
        
        if(digitalPinToInterrupt(info.pinSW) == digitalPinToInterrupt(_instances[i].pinSW)) {
            info.instance->processSwitch();
        }
    }
}
#endif 

void EC11Encoder::processEncoder() {
    uint8_t phase = (digitalRead(_pinSB) << 1) | digitalRead(_pinSA); 
    decode(phase);
}

void EC11Encoder::processSwitch() {
    bool current = !digitalRead(_pinSW);
    if(current) {
        _sw.press_start = millis();
        _sw.longpress_flag = false;
    }
    _sw.last_press = millis();
}

/**
 * Hardware Initialization Sequence
 * 1. Configure GPIO modes with critical section protection
 * 2. Attach interrupt handlers (platform-specific)
 * 3. Initialize state machines
 * 4. Set initialization flag
 */
void EC11Encoder::begin() {
    if (!_initialized) {
        Serial.print(F("[EC11] Library Version: "));
        Serial.println(EC11ENCODER_VERSION_STR);
        
        // initialize pins as input with internal pull-up resistors
        enterCritical();
        pinMode(_pinSA, INPUT_PULLUP);
        pinMode(_pinSB, INPUT_PULLUP);
        pinMode(_pinSW, INPUT_PULLUP);
        exitCritical();

        _rotation.last = millis();
        _sw.state = (digitalRead(_pinSW)  == 0) ? SwStateEnum::Pressed : SwStateEnum::Idle;

    #if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
        attachInterruptArg(digitalPinToInterrupt(_pinSA), handleEncoderISR, this, CHANGE);
        attachInterruptArg(digitalPinToInterrupt(_pinSB), handleEncoderISR, this, CHANGE);
        attachInterruptArg(digitalPinToInterrupt(_pinSW), handleSwISR, this, CHANGE);
    #else
        attachInterrupt(digitalPinToInterrupt(_pinSA), handleEncoderISR, CHANGE);
        attachInterrupt(digitalPinToInterrupt(_pinSB), handleEncoderISR, CHANGE);
        attachInterrupt(digitalPinToInterrupt(_pinSW), handleSwISR, CHANGE);
    #endif
        _lastUpdate = millis();

        // initialize rotation state structure with default values
        _rotation.lock = false;
        _rotation.interval = 0;
        _rotation.last = millis();
        _rotation.last_phase = 0;
        _rotation.valid_count = 0;

        // initialize switch state structure with default values
        _sw.state = (digitalRead(_pinSW)  == 0) ? SwStateEnum::Pressed : SwStateEnum::Idle; 
        _sw.last_press = millis();
        _sw.press_start = millis();
        _sw.longpress_flag = false;

        // set initialized flag
        _initialized = true;
    }
}

void EC11Encoder::end() {
    if(_interrupts_attached) {
        detachInterrupt(digitalPinToInterrupt(_pinSA));
        detachInterrupt(digitalPinToInterrupt(_pinSB));
        detachInterrupt(digitalPinToInterrupt(_pinSW));
    }
    _initialized = false;
}

void IRAM_ATTR EC11Encoder::decode(uint8_t phase) {
#if defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32)
    static const int8_t DIR_TABLE[] PROGMEM = {
#else
    static constexpr int8_t DIR_TABLE[] = {
#endif
        0, -1, 1, 0,    /*  CW */
        1, 0, 0, -1,    /* CCW */
        -1, 0, 0, 1,    /* CCW */
        0, 1, -1, 0     /*  CW */
    };

    /** 
     * Quadrature Decoding Logic
     * ------------------------
     * 1. Check encoder lock status and phase change
     * 2. Validate rotation after 2 consecutive phase changes
     * 3. Calculate direction using DIR_TABLE lookup
     * 4. Handle fast/slow rotation differentiation
     * 5. Trigger callback with rotation data
     */
    if (!_rotation.lock && (phase != _rotation.last_phase)) {
        if(++_rotation.valid_count >= 2) {
#if defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32)
            const int8_t dir = pgm_read_byte(DIR_TABLE + ((_rotation.last_phase << 2) | phase));
#else
            const int8_t dir = DIR_TABLE[((_rotation.last_phase << 2) | phase)];
#endif
            if (dir != 0) {
                _rotation.lock = true;
                const uint32_t now = millis();
                _rotation.interval = now - _rotation.last;
                _rotation.last = now;

                // Determine the step value based on the interval.
                const bool fast = _rotation.interval < static_cast<uint16_t>(TimingEnum::RotationThreshold);
                const int8_t step = fast ? static_cast<int8_t>(StepEnum::Fast) 
                                         : static_cast<int8_t>(StepEnum::Normal);

                // Notify rotation event with direction and step values and reset valid count to 0.
                notifyRotationEvent(dir * step, fast);
            }
            _rotation.valid_count = 0;
            }
        _rotation.last_phase = phase;
        }
    else {
        _rotation.valid_count = 0;
    }
}

void EC11Encoder::checkSwState() {
    const uint32_t now = millis();    
    const bool current = !digitalRead(_pinSW);
    
    if(_rotation.lock) return;

    volatile SwState& sw = _sw;

    switch (sw.state) {
        case SwStateEnum::Idle:{
            // check if switch is pressed and debounce time has passed and if not already in pressed state.
            if (current && ((now - sw.last_press) > static_cast<uint16_t>(TimingEnum::Debounce))) {
                sw.press_start = now;
                sw.state = SwStateEnum::Pressed;
            }
            break;
        }
        case SwStateEnum::Pressed:{
            if (!current) {
                // check if switch is released and debounce time has passed and if not already in released state.                
                sw.last_press = now;
                if (((now - sw.press_start) < static_cast<uint16_t>(TimingEnum::LongPress)) && !sw.longpress_flag) {
                    notifySwEvent(SwEventEnum::ShortClick);
                }
                
                sw.state = SwStateEnum::Idle;
                sw.longpress_flag = false;
            } 
            // check if long press has occurred and if not already in long press state. 
            else if (!sw.longpress_flag && 
                (now - sw.press_start) > static_cast<uint16_t>(TimingEnum::LongPress) ) {
                notifySwEvent(SwEventEnum::LongPress);
                sw.longpress_flag = true;
            }
            break;
        }
        case SwStateEnum::Released:{
            // check if switch is released and debounce time has passed and if not already in released state.
            if (!current) {
                sw.state = SwStateEnum::Idle;
            }

           break; 
        }
        default:
            break;
    }
}
