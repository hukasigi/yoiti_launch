#pragma once

#include <Arduino.h>
#include <HardwareTimer.h>
#include <cstdint>

extern "C" {
#include "stm32f3xx_hal.h"
}

class IncrementalEncoder {
    public:
        IncrementalEncoder(TIM_HandleTypeDef* htim, uint32_t pinA, uint32_t pinB)
            : htim_(htim), pinA_(pinA), pinB_(pinB), last_count_(0), count_(0) {}

        bool begin() {
            __HAL_RCC_GPIOA_CLK_ENABLE();
            __HAL_RCC_TIM3_CLK_ENABLE();

            GPIO_InitTypeDef gpio{};
            gpio.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
            gpio.Mode      = GPIO_MODE_AF_PP;
            gpio.Pull      = GPIO_PULLUP;
            gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
            gpio.Alternate = GPIO_AF2_TIM3;

            HAL_GPIO_Init(GPIOA, &gpio);

            htim_->Init.Prescaler         = 0;
            htim_->Init.CounterMode       = TIM_COUNTERMODE_UP;
            htim_->Init.Period            = 0xFFFF;
            htim_->Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
            htim_->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

            TIM_Encoder_InitTypeDef encoderConfig{};

            encoderConfig.EncoderMode = TIM_ENCODERMODE_TI12;

            encoderConfig.IC1Polarity  = TIM_ICPOLARITY_RISING;
            encoderConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
            encoderConfig.IC1Prescaler = TIM_ICPSC_DIV1;
            encoderConfig.IC1Filter    = 4;

            encoderConfig.IC2Polarity  = TIM_ICPOLARITY_RISING;
            encoderConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
            encoderConfig.IC2Prescaler = TIM_ICPSC_DIV1;
            encoderConfig.IC2Filter    = 4;

            if (HAL_TIM_Encoder_Init(htim_, &encoderConfig) != HAL_OK) {
                return false;
            }

            __HAL_TIM_SET_COUNTER(htim_, 0);

            if (HAL_TIM_Encoder_Start(htim_, TIM_CHANNEL_ALL) != HAL_OK) {
                return false;
            }

            last_count_ = 0;
            count_      = 0;

            return true;
        }
        int64_t getCount() {
            const uint16_t current = static_cast<uint16_t>(__HAL_TIM_GET_COUNTER(htim_));

            const int16_t delta = static_cast<int16_t>(static_cast<uint16_t>(current - last_count_));

            count_ += delta;
            last_count_ = current;

            return count_;
        }

        void setCount(int64_t count) {
            __HAL_TIM_SET_COUNTER(htim_, 0);

            last_count_ = 0;
            count_      = count;
        }

        void clear() { setCount(0); }

    private:
        TIM_HandleTypeDef* htim_;

        uint32_t pinA_;
        uint32_t pinB_;

        uint16_t last_count_;
        int64_t  count_;
};
