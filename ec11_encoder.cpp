/*******************************************************************
 * @file EC11Encoder.cpp
 * @brief Implementation of EC11Encoder class for handling EC11 encoder hardware interactions.
 * @author jinliang.liu@outlook.com
 * @date 2025-05-27
 * @copyright Copyright (c) 2025 Jinliang Liu. All rights reserved.
 *            This code is licensed under the MIT License.
 *            For details, see LICENSE file in project root.
 * 
 * @details
 * Handles EC11 encoder hardware interactions including:
 * - Quadrature decoding for rotation detection
 * - Switch state machine with debouncing
 * - Timing-sensitive event detection (long press/fast rotation)
 * 
 * @note
 * - Must call begin() before using encoder
 * - Not thread-safe (designed for single-threaded environments)
 * - Uses singleton pattern for ISR handling
 * 
 * @history：
 *********************************************************************
 *  2025-05-27   Initial version
 * 
 *******************************************************************/

 #include "ec11_encoder.h"

EC11Encoder* EC11Encoder::instance = nullptr;

EC11Encoder::EC11Encoder(uint8_t pinSA, uint8_t pinSB, uint8_t pinSW) 
    : _pinSA(pinSA), _pinSB(pinSB), _pinSW(pinSW),
    onRotationEvent(nullptr), 
    onSwEvent(nullptr)
{
    instance = this;
}

void EC11Encoder::begin() {
    // initialize pins as input with internal pull-up resistors
    noInterrupts();
    pinMode(_pinSA, INPUT_PULLUP);
    pinMode(_pinSB, INPUT_PULLUP);
    pinMode(_pinSW, INPUT_PULLUP);
    interrupts();

    _rotation.last = millis();
    _sw.state = (digitalRead(_pinSW)  == 0) ? SwStateEnum::Pressed : SwStateEnum::Idle;
    
    // attach interrupt handlers
    attachInterrupt(digitalPinToInterrupt(_pinSA), encoder_isr_handler, CHANGE);
    attachInterrupt(digitalPinToInterrupt(_pinSB), encoder_isr_handler, CHANGE);
    attachInterrupt(digitalPinToInterrupt(_pinSW), sw_isr_handler, CHANGE);
    
    // start ticker for periodic checks
    _ticker.attach_ms(10, [this](){
        // check switch state
        checkSwState();

        // unlock if no rotation for 15ms 
        if(_rotation.lock && ((millis() - _rotation.last) > static_cast<uint16_t>(TimingEnum::RotationUnlock))) {
            _rotation.lock = false;
        }
    });

    // initialize rotation state structure with default values
    _rotation.lock = false;
    _rotation.interval = 0;
    _rotation.last = millis();

    // initialize switch state structure with default values
    _sw.state = (digitalRead(_pinSW)  == 0) ? SwStateEnum::Pressed : SwStateEnum::Idle; 
    _sw.last_press = millis();
    _sw.press_start = millis();
    _sw.longpress_flag = false;

    // set initialized flag
    _initialized = true;
}

void EC11Encoder::decode(uint8_t phase) {
    static constexpr int8_t DIR_TABLE[] PROGMEM = {
        0, -1, 1, 0,    /*  CW */
        1, 0, 0, -1,    /* CCW */
        -1, 0, 0, 1,    /* CCW */
        0, 1, -1, 0     /*  CW */
    };
    static uint8_t last_phase = 0;
    static uint8_t valid_count = 0;

    /** 
     * Check if the encoder is locked or if the phase has changed since the 
     * last check and increment the valid count accordingly.If the valid count 
     * is greater than or equal to 2, calculate the direction of rotation based 
     * on the last and current phase using the DIR_TABLE array.  If the direction 
     * is not 0, lock the encoder, calculate the interval between the last and 
     * current rotation, and update the last rotation time. If the interval is 
     * less than the rotation threshold, set the step to fast, otherwise set it 
     * to normal. Notify the rotation event with the direction and step values, 
     * and reset the valid count to 0.If the phase has not changed, reset the 
     * valid count to 0 and return early.
    */
    if(!_rotation.lock && (phase != last_phase)) {
        if(++valid_count >= 2) {
            const int8_t dir = pgm_read_byte(DIR_TABLE + ((last_phase << 2) | phase));
            if (dir != 0) {
                _rotation.lock = true;
                const uint16_t now = millis();
                _rotation.interval = now - _rotation.last; 
                _rotation.last = now;
                
                // Determine the step value based on the interval.
                const bool fast = _rotation.interval < static_cast<uint16_t>(TimingEnum::RotationThreshold);
                const int8_t step = fast ? static_cast<int8_t>(StepEnum::Fast) 
                                         : static_cast<int8_t>(StepEnum::Normal);

                // Notify rotation event with direction and step values and reset valid count to 0.
                notifyRotationEvent(dir * step, fast);
            }
            valid_count = 0;
        }    

        last_phase = phase;
    }
    else {
        valid_count = 0;
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
            else if (!sw.longpress_flag && (now - sw.press_start) > static_cast<uint16_t>(TimingEnum::LongPress) ) {
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
