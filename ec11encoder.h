/******************************************************************************
 * @file ec11encoder.h
 * @author jinliang.liu@outlook.com
 * @brief EC11 Quadrature Encoder Driver - Precise rotation detection with switch state machine
 *        * Core Features:
 *        - Automatic phase decoding (CW/CCW direction detection)
 *        - Velocity-sensitive event reporting (Fast/Slow rotation differentiation)
 *        - Multi-instance support with pin conflict prevention
 *        - Cross-platform interrupt handling (HW interrupts for ESP32/ESP8266)
 *******************************************************************************
 * @attention 
 * 
 * Copyright (c) 2025 Jinliang Liu. 
 * All rights reserved.
 * 
 * This code is licensed under the MIT License.
 * For details, see LICENSE file in project root.
 *
 * @version     1.0.0
 * @date        2025-06-03
 * @changelog
 * [1.0.0] - 2025-06-03
 * - Initial release.
 * - Implemented core encoder functionality.
 * - Added multi-platform interrupt support.
 * - Built-in state machine for switch detection.
 ******************************************************************************/

#ifndef EC11ENCODER_H
#define EC11ENCODER_H

#include <Arduino.h>

#if defined(ESP8266) && !defined(ARDUINO_ARCH_ESP8266)
#define ARDUINO_ARCH_ESP8266
#endif

#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
#define ARDUINO_ARCH_ESP32
#endif

#if defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32)
#include <functional>
#include <Ticker.h>
#define ISR_ATTR IRAM_ATTR
#else
#define ISR_ATTR
#endif

#if defined(ARDUINO_ARCH_ESP32)
#include <freertos/FreeRTOS.h>
#endif

#if defined(ARDUINO_ARCH_ESP8266)
  #define HAS_ATOMIC_EXTENSION 0
  #include <atomic>
  
  // Add inline keyword to prevent multiple definitions
  extern "C" {
    inline bool __atomic_compare_exchange_1(volatile void* ptr, void* expected, 
                                    uint8_t desired, bool weak, int success, int failure) {
      (void)weak; (void)success; (void)failure;
      uint8_t* p = reinterpret_cast<uint8_t*>(const_cast<void*>(ptr));
      uint8_t expect = *reinterpret_cast<uint8_t*>(expected);
      if (*p != expect) return false;
      *p = desired;
      return true;
    }
    
    inline uint8_t __atomic_fetch_add_1(volatile void* ptr, uint8_t value, int memorder) {
      (void)memorder;
      uint8_t* p = reinterpret_cast<uint8_t*>(const_cast<void*>(ptr));
      uint8_t old = *p;
      *p += value;
      return old;
    }
  }
#elif defined(ARDUINO_ARCH_ESP32)
  #include <atomic>
#endif

// Version macros (major.minor.patch)
#ifndef EC11ENCODER_VERSION
#define EC11ENCODER_VERSION_MAJOR 1
#define EC11ENCODER_VERSION_MINOR 0
#define EC11ENCODER_VERSION_PATCH 0
#define EC11ENCODER_VERSION 10000  // (MAJOR * 10000 + MINOR * 100 + PATCH)
#define EC11ENCODER_VERSION_STR "1.0.0"
#endif

// Maximum number of instances allowed to be created
#ifndef EC11_MAX_INSTANCES
#define EC11_MAX_INSTANCES 3
#endif

class EC11Encoder {
public:
    // Timing Enum(ms)
    enum class TimingEnum : uint16_t {
        Debounce = 50,         // debounce time for switch
        LongPress = 2000,      // long press time for switch
        RotationThreshold = 30,// fast rotation threshold
        RotationUnlock = 15    // rotation unlock time after lock
    };

    // Rotation Step Enum
    enum class StepEnum : int8_t {
        Normal = 1,            // normal step length
        Fast = 3               // fast step length
    };

    // Switch Event Enum
    enum class SwEventEnum : uint8_t {
        None        = 0,        // None event
        ShortClick  = 1,        // Click event
        LongPress   = 2         // Long press event
    };

    // Swtich State Enum
    enum class SwStateEnum : uint8_t {
        Idle     = 0,          // IDLE
        Pressed  = 1,          // Switch Pressed
        Released = 2           // Switch Released
    };

    // Callback Function Types for Rotation and Switch Events
    typedef void(*RotationCallback)(int8_t, bool);
    typedef void(*SwCallback)(SwEventEnum);
    //  Platform-safe callback type
#if defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32)
        using InterruptCallback = std::function<void()>;
#else
        using InterruptCallback = void(*)();
#endif
    /**
     * @brief Construct a new EC11Encoder object with pin numbers.
     * @param pinSA The pin number for the SA pin of the encoder.
     * @param pinSB The pin number for the SB pin of the encoder.
     * @param pinSW The pin number for the SW pin of the encoder.
     * @note The pin numbers should be connected to the encoder as follows:
     *       SA -> D2 (GPIO4)
     *       SB -> D1 (GPIO5)
     *       SW -> D3 (GPIO2)
     */
    EC11Encoder(uint8_t pinSA, uint8_t pinSB, uint8_t pinSW);
    ~EC11Encoder();
    
    /**
     * @brief Initialize the encoder.
     * @note This method should be called before using the encoder.
     *       It will set the interrupt handler for the encoder and switch.
     *       The interrupt handler will be called when the encoder is rotated or the switch is pressed or released.
     *       The interrupt handler will be called in the IRAM, so it will not block the main loop.
     */
    void begin(); 

    /**
     * @brief End the encoder.
     * @note This method will detach the interrupt handler for the encoder and switch.
     *       It will also stop the ticker for periodic checks.
     *       It should be called when the encoder is not used anymore.
     *       The interrupt handler will be called in the IRAM, so it will not block the main loop.
     */
    void end();


    /**
     * @brief Start the ticker to periodically update the encoder state.
     * @param interval_ms interval in milliseconds.
     * @note This method should be called before using the encoder.
     *       It will start the ticker to periodically update the encoder state.
     *       The ticker will call the update() method to update the encoder state.
     *       The update() method will be called in the IRAM, so it will not block the main loop.
     */
    void autoUpdate(bool useTicker, uint16_t interval_ms = 10) {
        _isUpdateInTicker = useTicker;
        if (_isUpdateInTicker) {
#if defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32)
            _ticker.attach_ms(interval_ms, [this](){ 
                checkSwState();

                if (_rotation.lock &&
                    ((millis() - _rotation.last) > static_cast<uint16_t>(TimingEnum::RotationUnlock))) {
                    _rotation.lock = false;
                }
            });
#else
            Serial.println("Auto update is not supported on this platform.");
            Serial.println("Please use begin() method to initialize the encoder.");
            Serial.println("Then, use update() method to update the encoder state manually.");
            while (1){ 
                delay(1000); 
            }
#endif
        }
    }

    /**
     * @brief Update the encoder state.
     * @note This method should be called periodically to update the encoder state.
     *       It will check the rotation and switch state and call the callback function if needed.
     *       The callback function will be called in the IRAM, so it will not block the main loop.
     *       The callback function will be called when the encoder is rotated or the switch is pressed or released.
     *       The callback function will be called in the IRAM, so it will not block the main loop.
     */
    void update(){
        if (_isUpdateInTicker == false){
            uint32_t now = millis();
            if(now - _lastUpdate >= 10) {
                checkSwState();
                if(_rotation.lock && 
                    ((now - _rotation.last) > static_cast<uint16_t>(TimingEnum::RotationUnlock))) {
                    _rotation.lock = false;
                }
                _lastUpdate = now;
            }
        }
    }

    /**
     * @brief Set the callback function for rotation event.
     * @param cb The callback function.
     * @note The callback function will be called when the rotation event is detected.
     */
    void onRotationCallback(RotationCallback cb) {
        onRotationEvent = cb;
    }

    /**
     * @brief Set the callback function for switch event.
     * @param cb The callback function.
     * @note The callback function will be called when the switch event is detected.
     */
    void onSwCallback(SwCallback cb) {
        onSwEvent = cb; 
    }

private:
    uint8_t _pinSA, _pinSB, _pinSW;
    bool _isUpdateInTicker = false;
    struct InstanceInfo {
        uint8_t pinSA;
        uint8_t pinSB;
        uint8_t pinSW;
        EC11Encoder* instance;
    };
    
    // Static member for storing instance info
    static InstanceInfo _instances[EC11_MAX_INSTANCES];
    static uint8_t _instanceCount;

    // Flag to indicate if the encoder is initialized
    bool _initialized = false;
    // Flag to indicate if the interrupts are attached to the encoder and switch pins.
    // This flag is used to prevent multiple interrupts from being attached to the same pins.
    bool _interrupts_attached = false;
    uint32_t _lastUpdate = 0;
    // the state of rotation struct
    struct RotationState {
        uint32_t last;          // the last rotation time
        uint16_t interval;       // the interval between two rotations
        bool lock;               // the lock flag for fast rotation
        uint8_t last_phase;  // the last phase of rotation
        uint8_t valid_count; // the valid count of rotation
    } _rotation;

    // the state of switch struct
    struct SwState {
        SwStateEnum state;      // the state of switch (Idle, Pressed, Released)
        uint32_t  last_press;   // the last press time
        uint32_t  press_start;  // the press start time for long press
        bool longpress_flag;    // the flag for long press detected
    } _sw;

    // Callback function for rotation event
    RotationCallback onRotationEvent = nullptr;
    // Callback function for switch event
    SwCallback onSwEvent = nullptr;

private:
    /**
     * Critical Section Management
     * ---------------------------
     * AVR       : Disable global interrupts
     * ESP32     : FreeRTOS critical section
     * ESP8266   : Disable interrupts
     * Generic   : No operation (fallback)
     */
    void enterCritical(){
#if defined(__AVR__)
        cli();
#elif defined(ARDUINO_ARCH_ESP32)
        taskENTER_CRITICAL(&mux);
#elif defined(ARDUINO_ARCH_ESP8266)
        noInterrupts();
#endif
    }

    /**
     * @brief Exit critical section.
     * @note This method is called by the sw_isr_handler method.
     */
    void exitCritical(){
#if defined(__AVR__)
        sei();
#elif defined(ARDUINO_ARCH_ESP32)
        taskEXIT_CRITICAL(&mux);
#elif defined(ARDUINO_ARCH_ESP8266)
        interrupts();
#endif
    }

#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
    static void IRAM_ATTR handleEncoderISR(void* arg);
    static void IRAM_ATTR handleSwISR(void* arg);
#else

    /**
     * @brief The interrupt handler for the switch. This method is called when the switch is pressed or released.
     * @note This method is called by the sw_isr_handler method.
     */
    static void handleSwISR();

    /**
     * @brief The interrupt handler for the encoder. This method is called when the encoder is rotated.
     * @note This method is called by the encoder_isr_handler method.
     */
    static void handleEncoderISR();
#endif

    void processEncoder();
    void processSwitch();

     /**
     * @brief Trigger button event and pass to registered callback function.
     * @param event The button event type to trigger, see SwEventEnum for details.
     * @note This method is called by the checkSwState method when a button event is detected.
     */
    void notifySwEvent(SwEventEnum event){
        if(onSwEvent) {
            onSwEvent(event);
        }
    }    

    /**
     * @brief Trigger rotation event and pass to registered callback function.
     *        This method checks if the onRotationEvent callback is registered:
     *          - If registered, call the callback function with the step and fast parameters
     *          - If not registered, silently ignore the event
     * @param step  The rotation step length, see StepEnum for details (1 for normal, 3 for fast) 
     * @param fast  The flag to indicate if the rotation is fast (true for fast, false for normal) 
     * @note This method is called by the decode method when a rotation event is detected.
     */
    void notifyRotationEvent(int8_t step, bool fast) {
        if(onRotationEvent) {
            onRotationEvent(step, fast);
        }
    }

    /**
     * @brief Decode the rotation event and trigger the rotation event.
     *        This method is called by the encoder_isr_handler method.
     * @param phase The current phase of the encoder, see PhaseEnum for details (0-3).
     * @note This method is called by the encoder_isr_handler method when the encoder is rotated.
     */
    void IRAM_ATTR decode(uint8_t phase);

    /**
     * @brief Check the switch state and trigger the switch event.
     * @note This method is called by the _ticker callback.
     */
    void checkSwState();

    static bool registerInstance(uint8_t sa, uint8_t sb, uint8_t sw, EC11Encoder* instance);
    static EC11Encoder* findInstanceByEncoder(uint8_t sa, uint8_t sb);
    static EC11Encoder* findInstanceBySwitch(uint8_t sw);

#if defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32)
    // Ticker for timing control in ESP8266 and ESP32 platforms.
    Ticker _ticker;
#endif
#if defined(ARDUINO_ARCH_ESP32)
    static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
#endif
};

#endif
