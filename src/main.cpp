#include "Pid.h"
#include "incremental_encoder.hpp"
#include "message.h"
#include "motor.hpp"
#include <Arduino.h>
extern "C" {
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_can.h"
}

const uint8_t A_PIN = PA6;
const uint8_t B_PIN = PA7;

const uint8_t DEBUG_PIN = PA10;

const uint8_t LAUNCH_PWM_PIN = PA8;
const uint8_t LAUNCH_DIR_PIN = PB0;

const uint8_t LOAD_PWM_PIN = PA9;
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

CAN_HandleTypeDef   hcan      = {};
CAN_RxHeaderTypeDef rxHeader  = {};
uint8_t             rxData[8] = {0};

const uint16_t CAN_TX_ID_TARGET_REACHED = 0x102;
const double   TARGET_RPM_TOLERANCE     = 100.0;

double current_limit_amps = 20.0;

bool launch_started = false;
bool roller_started = false;
bool roller_stop    = false;

extern "C" void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle) {
    if (canHandle->Instance != CAN) return;

    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {};
    gpio.Pin              = GPIO_PIN_11 | GPIO_PIN_12;
    gpio.Mode             = GPIO_MODE_AF_PP;
    gpio.Pull             = GPIO_NOPULL;
    gpio.Speed            = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate        = GPIO_AF9_CAN;

    HAL_GPIO_Init(GPIOA, &gpio);
}

static void can_init() {
    hcan.Instance                  = CAN;
    hcan.Init.Prescaler            = 2;
    hcan.Init.Mode                 = CAN_MODE_NORMAL;
    hcan.Init.SyncJumpWidth        = CAN_SJW_1TQ;
    hcan.Init.TimeSeg1             = CAN_BS1_11TQ;
    hcan.Init.TimeSeg2             = CAN_BS2_4TQ;
    hcan.Init.TimeTriggeredMode    = DISABLE;
    hcan.Init.AutoBusOff           = ENABLE;
    hcan.Init.AutoWakeUp           = DISABLE;
    hcan.Init.AutoRetransmission   = ENABLE;
    hcan.Init.ReceiveFifoLocked    = DISABLE;
    hcan.Init.TransmitFifoPriority = DISABLE;

    if (HAL_CAN_Init(&hcan) != HAL_OK) {
        Serial.println("CAN init failed");
        while (true)
            delay(1000);
    }

    CAN_FilterTypeDef filter    = {};
    filter.FilterBank           = 0;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation     = ENABLE;

    if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK || HAL_CAN_Start(&hcan) != HAL_OK) {
        Serial.println("CAN start failed");
        while (true)
            delay(1000);
    }

    Serial.println("CAN started: 1 Mbps");
}

static void can_send_target_reached() {
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0) return;

    CAN_TxHeaderTypeDef tx_header = {};
    tx_header.StdId               = CAN_TX_ID_TARGET_REACHED;
    tx_header.IDE                 = CAN_ID_STD;
    tx_header.RTR                 = CAN_RTR_DATA;
    tx_header.DLC                 = 0;

    uint8_t  data[8] = {};
    uint32_t mailbox = 0;

    HAL_CAN_AddTxMessage(&hcan, &tx_header, data, &mailbox);
}

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

static void can_receive_command() {
    while (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0) {
        if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK) {
            return;
        }

        digitalWrite(DEBUG_PIN, !digitalRead(DEBUG_PIN));

        if (rxHeader.IDE != CAN_ID_STD) {
            continue;
        }
        Serial.println(rxHeader.StdId);

        switch (rxHeader.StdId) {
        case CAN_CMD_LAUNCH_START:
            launch_started = true;
            Serial.println("Launch started");
            break;

        case CAN_CMD_LAUNCH_ROLLER:
            roller_started = true;
            Serial.println("Roller started");
            break;

        case CAN_CMD_STOP_ROLLER:
            launch_started = false;
            Serial.println("Launch stop");
            break;

        default: break;
        }
    }
}

void setup() {
    Serial.begin(115200);

    can_init();

    // DAC初期化と電流リミット設定
    dac1_init();
    set_current_limit(20.0);

    pinMode(LAUNCH_DIR_PIN, OUTPUT);
    pinMode(LAUNCH_PWM_PIN, OUTPUT);

    pinMode(LOAD_DIR_PIN, OUTPUT);
    pinMode(LOAD_PWM_PIN, OUTPUT);

    pinMode(LIM_SW, INPUT);

    pinMode(DEBUG_PIN, OUTPUT);
    digitalWrite(DEBUG_PIN, LOW);

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

uint32_t last_printed  = millis();
uint32_t last_can_sent = 0;

void loop() {
    can_receive_command();
    const unsigned long now_ms = millis();
    const double        dt     = (now_ms - prev_ms) / 1000.0;

    if (dt < 0.01) return; // 10ms周期程度
    prev_ms = now_ms;

    const int64_t now_count   = enc.getCount();
    const int64_t delta_count = now_count - prev_count;
    prev_count                = now_count;

    // counts -> RPM
    const double measured_rpm = (delta_count / static_cast<double>(ENC_CPR)) / dt * 60.0;

    bool target_reached = false;

    if (launch_started) {
        const double control = pid.update(target_rpm, measured_rpm, dt);
        launch.run(static_cast<int>(control));

        target_reached = std::fabs(measured_rpm - target_rpm) <= TARGET_RPM_TOLERANCE;
    } else {
        launch.run(0);
    }

    if (launch_started) {
        if (!load_finished) {
            while (digitalRead(LIM_SW) == 0) {
                load.run(100);
            }

            if (digitalRead(LIM_SW) == 0) {
                load.run(0);
                load_finished  = true;
                launch_started = false;
                Serial.println("Load limit reached");
            } else {
                load.run(100);
            }
        }
    }

    // if (target_reached && now_ms - last_can_sent >= 200) {
    if (now_ms - last_can_sent >= 200) {
        can_send_target_reached();
        last_can_sent = now_ms;
    }

    if (millis() - last_printed >= 200) {
        Serial.print(digitalRead(LIM_SW));
        Serial.print("rpm: ");
        Serial.println(measured_rpm);
        last_printed = millis();
    }
}