/**
 * @file ec11_encoder.h
 * @brief EC11 rotary encoder handler with switch support and interrupt-driven design.
 *        This library provides a simple and efficient way to handle EC11 rotary encoder hardware.
 *        It supports both rotation detection and switch press detection.
 *        The library is designed to be used in interrupt-driven environments, such as Arduino.
 *        The library is not thread-safe, and should not be used in multi-threaded environments.
 * @author jinliang.liu@outlook.com
 * @date 2025-05-27
 * @copyright Copyright (c) 2025 Jinliang Liu. All rights reserved.
 *            This code is licensed under the MIT License.
 *            For details, see LICENSE file in project root.
 * 
 * Features: 
 * - Encoder rotation detection with acceleration support
 * - Switch press detection (short/long press)
 * - Interrupt-driven design
 * - Callback event mechanism
 */

#ifndef EC11_ENCODER_H
#define EC11_ENCODER_H

#include <Arduino.h>
#include <Ticker.h>

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
    
    /**
     * @brief Initialize the encoder.
     * @note This method should be called before using the encoder.
     *       It will set the interrupt handler for the encoder and switch.
     *       The interrupt handler will be called when the encoder is rotated or the switch is pressed or released.
     *       The interrupt handler will be called in the IRAM, so it will not block the main loop.
     */
    void begin(); 

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
    // The instance of the encoder object
    static EC11Encoder* instance;

    // Pin numbers for the encoder
    uint8_t _pinSA, _pinSB, _pinSW;

    // Callback functions for rotation and switch events
    RotationCallback onRotationEvent;
    SwCallback onSwEvent;
    
    // Ticker for timing control
    Ticker _ticker;
    // Flag to indicate if the encoder is initialized
    bool _initialized = false; 

    // the state of rotation struct
    struct RotationState {
        uint32_t  last;         // the last rotation time
        uint16_t interval;      // the interval between two rotations
        bool lock;              // the lock flag for fast rotation
    } volatile _rotation;

    // the state of switch struct
    struct SwState {
        SwStateEnum state;      // the state of switch (Idle, Pressed, Released)
        uint32_t  last_press;   // the last press time
        uint32_t  press_start;  // the press start time for long press
        bool longpress_flag;    // the flag for long press detected
    } volatile _sw;

private:
    /**
     * @brief The interrupt handler for the switch. This method is called when the switch is pressed or released.
     * @note This method is called by the sw_isr_handler method.
     */
    static void IRAM_ATTR sw_isr_handler() {
        instance->_sw.last_press = millis();  
    }

    /**
     * @brief The interrupt handler for the encoder. This method is called when the encoder is rotated.
     * @note This method is called by the encoder_isr_handler method.
     */
    static void IRAM_ATTR encoder_isr_handler() {
        noInterrupts();
        uint8_t phase = (digitalRead(instance->_pinSB) << 1) | 
                        digitalRead(instance->_pinSA);
        interrupts();
        instance->decode(phase);
    }

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
    void decode(uint8_t phase); 

    /**
     * @brief Check the switch state and trigger the switch event.
     * @note This method is called by the _ticker callback.
     */
    void checkSwState();
};

#endif
