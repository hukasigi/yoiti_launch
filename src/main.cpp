#include "Pid.h"
#include "incremental_encoder.hpp"
#include "motor.hpp"
#include <Arduino.h>
extern "C" {
#include "stm32f3xx_hal.h"
}

const uint8_t A_PIN = PA6;
const uint8_t B_PIN = PA7;

const uint8_t LAUNCH_PWM_PIN = PA8;
const uint8_t LAUNCH_DIR_PIN = PB0;

const uint8_t LOAD_PWM_PIN = PA3;
const uint8_t LOAD_DIR_PIN = PA1;

const uint8_t LIM_SW = PB4;

// TODO:1回転あたりのカウント数 変更
const int32_t ENC_CPR = 4096;

const uint8_t DAC_PIN = PA4;

// 目標RPM
double target_rpm = -4000.0;

HardwareTimer encoderTimer(TIM3);

const uint8_t  PWM_LIMIT      = 255;
const uint16_t INTEGRAL_LIMIT = 1000;

// PID定数（要調整）
PID pid(1.0,                            // kp
        0.0,                            // ki
        0.0,                            // kd
        -PWM_LIMIT, PWM_LIMIT,          // 出力制限
        -INTEGRAL_LIMIT, INTEGRAL_LIMIT // 積分制限
);

IncrementalEncoder enc(encoderTimer.getHandle(), A_PIN, B_PIN);

Motor launch(LAUNCH_DIR_PIN, LAUNCH_PWM_PIN);

Motor load(LOAD_DIR_PIN, LOAD_PWM_PIN);

int64_t       prev_count = 0;
unsigned long prev_ms    = 0;

bool load_finished = false;

const uint16_t DAC_MAX_VALUE      = 4095;
const double   DAC_REFERENCE_VOLT = 3.3;
const double   SENSOR_ZERO_VOLT   = 1.65;
const double   SENSOR_MAX_VOLT    = 3.2;
const double   SENSOR_MAX_CURRENT = 32.2;

DAC_HandleTypeDef hdac1;

static void dac1_init() {
    __HAL_RCC_DAC1_CLK_ENABLE();

    hdac1.Instance = DAC1;

    if (HAL_DAC_Init(&hdac1) != HAL_OK) {
        while (true) {
            delay(1000);
        }
    }

    DAC_ChannelConfTypeDef config = {};
    config.DAC_Trigger            = DAC_TRIGGER_NONE;

    if (HAL_DAC_ConfigChannel(&hdac1, &config, DAC_CHANNEL_1) != HAL_OK) {
        while (true) {
            delay(1000);
        }
    }

    HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
}

static uint16_t voltage_to_dac_value(double voltage) {
    voltage = constrain(voltage, 0.0, DAC_REFERENCE_VOLT);

    return static_cast<uint16_t>((voltage / DAC_REFERENCE_VOLT) * DAC_MAX_VALUE);
}

static double current_to_voltage(double current) {
    current = constrain(current, 0.0, SENSOR_MAX_CURRENT);

    return SENSOR_ZERO_VOLT + (current / SENSOR_MAX_CURRENT) * (SENSOR_MAX_VOLT - SENSOR_ZERO_VOLT);
}

static void set_current_limit(double current) {
    const double   voltage = current_to_voltage(current);
    const uint16_t value   = voltage_to_dac_value(voltage);

    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, value);
}

void setup() {
    Serial.begin(115200);

    // DAC初期化と電流リミット設定
    dac1_init();
    set_current_limit(20.0);

    pinMode(LAUNCH_DIR_PIN, OUTPUT);
    pinMode(LAUNCH_PWM_PIN, OUTPUT);

    pinMode(LOAD_DIR_PIN, OUTPUT);
    pinMode(LOAD_PWM_PIN, OUTPUT);

    pinMode(LIM_SW, INPUT);

    if (!enc.begin()) {
        Serial.println("Encoder initialization failed");
        while (true) {
            delay(1000);
        }
    }

    enc.clear();
    pid.reset(0.0);

    prev_count = enc.getCount();
    prev_ms    = millis();
}

uint32_t last_printed = millis();

void loop() {
    const unsigned long now_ms = millis();
    const double        dt     = (now_ms - prev_ms) / 1000.0;

    if (dt < 0.01) return; // 10ms周期程度
    prev_ms = now_ms;

    const int64_t now_count   = enc.getCount();
    const int64_t delta_count = now_count - prev_count;
    prev_count                = now_count;

    // counts -> RPM
    const double measured_rpm = (delta_count / static_cast<double>(ENC_CPR)) / dt * 60.0;

    const double control = pid.update(target_rpm, measured_rpm, dt);

    launch.run(static_cast<int>(control));

    if (!load_finished) {
        if (digitalRead(LIM_SW) == LOW) {
            load.run(0);
            load_finished = true;
            Serial.println("Load limit reached");
        } else {
            load.run(100);
        }
    }

    if (millis() - last_printed >= 200) {
        Serial.print(digitalRead(LIM_SW));
        Serial.print("rpm: ");
        Serial.println(measured_rpm);
        last_printed = millis();
    }
}