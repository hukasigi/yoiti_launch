#pragma once

#include <Arduino.h>
#include <cstdint>

using pin_t  = uint8_t;
using ch_t   = uint8_t;
using duty_t = int16_t;

class Motor {
    public:
        Motor(pin_t DIR, pin_t PWM) : DIR_PIN(DIR), PWM_PIN(PWM) {
            pinMode(DIR_PIN, OUTPUT);
            pinMode(PWM_PIN, OUTPUT);
            analogWriteFrequency(PWM_FREQ);
        }

        void run(duty_t duty, int8_t sign = 1) {
            const int16_t power = static_cast<int16_t>(duty) * sign;

            digitalWrite(DIR_PIN, power > 0 ? HIGH : LOW);
            const uint8_t output = static_cast<uint8_t>(abs(power));
            analogWrite(PWM_PIN, output);
        }
        void stop() { run(0); }

    private:
        pin_t DIR_PIN;
        pin_t PWM_PIN;

        static constexpr uint32_t PWM_FREQ = 20000;
};
