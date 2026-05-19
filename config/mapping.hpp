#pragma once

#include <cstddef>

#include "mapping_types.hpp"

namespace ru::driver::stm32h5xx::cfg {

namespace gpio {
inline constexpr gpio_config DEBUG_LED{
    .port_base = GPIOE_BASE,
    .active_high = true,
    .init =
        {
            .Pin = GPIO_PIN_3,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW,
            .Alternate = 0U,
        },
};
}  // namespace gpio

namespace adc {
inline constexpr auto ADC_0 = adc_config::polling_config(
    ADC1_BASE, GPIOA_BASE,
    {
        .Pin = GPIO_PIN_1,
        .Mode = GPIO_MODE_ANALOG,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_LOW,
        .Alternate = 0U,
    },
    {
        .ClockPrescaler = ADC_CLOCK_ASYNC_DIV1,
        .Resolution = ADC_RESOLUTION_12B,
        .DataAlign = ADC_DATAALIGN_RIGHT,
        .ScanConvMode = ADC_SCAN_DISABLE,
        .EOCSelection = ADC_EOC_SINGLE_CONV,
        .LowPowerAutoWait = DISABLE,
        .ContinuousConvMode = DISABLE,
        .NbrOfConversion = 1U,
        .DiscontinuousConvMode = DISABLE,
        .NbrOfDiscConversion = 1U,
        .ExternalTrigConv = ADC_SOFTWARE_START,
        .ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE,
        .SamplingMode = ADC_SAMPLING_MODE_NORMAL,
        .DMAContinuousRequests = DISABLE,
        .Overrun = ADC_OVR_DATA_OVERWRITTEN,
        .OversamplingMode = DISABLE,
        .Oversampling = {},
    },
    {
        .Channel = ADC_CHANNEL_1,
        .Rank = ADC_REGULAR_RANK_1,
        .SamplingTime = ADC_SAMPLETIME_47CYCLES_5,
        .SingleDiff = ADC_SINGLE_ENDED,
        .OffsetNumber = ADC_OFFSET_NONE,
        .Offset = 0U,
        .OffsetSign = ADC_OFFSET_SIGN_NEGATIVE,
        .OffsetSaturation = DISABLE,
    });

inline constexpr auto ADC_1 = adc_config::dma_config(
    ADC2_BASE, GPIOA_BASE,
    {
        .Pin = GPIO_PIN_2,
        .Mode = GPIO_MODE_ANALOG,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_LOW,
        .Alternate = 0U,
    },
    {
        .ClockPrescaler = ADC_CLOCK_ASYNC_DIV4,
        .Resolution = ADC_RESOLUTION_12B,
        .DataAlign = ADC_DATAALIGN_RIGHT,
        .ScanConvMode = ADC_SCAN_DISABLE,
        .EOCSelection = ADC_EOC_SINGLE_CONV,
        .LowPowerAutoWait = DISABLE,
        .ContinuousConvMode = ENABLE,
        .NbrOfConversion = 1U,
        .DiscontinuousConvMode = DISABLE,
        .NbrOfDiscConversion = 1U,
        .ExternalTrigConv = ADC_SOFTWARE_START,
        .ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE,
        .SamplingMode = ADC_SAMPLING_MODE_NORMAL,
        .DMAContinuousRequests = ENABLE,
        .Overrun = ADC_OVR_DATA_OVERWRITTEN,
        .OversamplingMode = DISABLE,
        .Oversampling = {},
    },
    {
        .Channel = ADC_CHANNEL_2,
        .Rank = ADC_REGULAR_RANK_1,
        .SamplingTime = ADC_SAMPLETIME_92CYCLES_5,
        .SingleDiff = ADC_SINGLE_ENDED,
        .OffsetNumber = ADC_OFFSET_NONE,
        .Offset = 0U,
        .OffsetSign = ADC_OFFSET_SIGN_NEGATIVE,
        .OffsetSaturation = DISABLE,
    },
    256U, GPDMA1_REQUEST_ADC2, GPDMA1_Channel1_BASE, GPDMA1_Channel1_IRQn);
}  // namespace adc

namespace pwm {
inline constexpr auto PWM_0 = pwm_config::general_purpose_config(
    GPIOA_BASE, GPIO_PIN_3, GPIO_PULLUP, GPIO_AF1_TIM2,
    {
        .instance_base = TIM2_BASE,
        .channel = TIM_CHANNEL_4,
        .init =
            {
                .Prescaler = 0U,
                .CounterMode = TIM_COUNTERMODE_UP,
                .Period = 0U,
                .ClockDivision = TIM_CLOCKDIVISION_DIV1,
                .RepetitionCounter = 0U,
                .AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE,
            },
        .channel_init =
            {
                .OCMode = TIM_OCMODE_PWM1,
                .Pulse = 0U,
                .OCPolarity = TIM_OCPOLARITY_HIGH,
                .OCNPolarity = TIM_OCNPOLARITY_HIGH,
                .OCFastMode = TIM_OCFAST_DISABLE,
                .OCIdleState = TIM_OCIDLESTATE_RESET,
                .OCNIdleState = TIM_OCNIDLESTATE_RESET,
            },
    });
}  // namespace pwm

namespace bx_can {
inline constexpr can_config CAN{
    .instance_base = FDCAN1_BASE,
    .port_rx_base = GPIOD_BASE,
    .port_tx_base = GPIOD_BASE,
    .rx_pin_init =
        {
            .Pin = GPIO_PIN_0,
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_PULLUP,
            .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
            .Alternate = GPIO_AF9_FDCAN1,
        },
    .tx_pin_init =
        {
            .Pin = GPIO_PIN_1,
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_PULLUP,
            .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
            .Alternate = GPIO_AF9_FDCAN1,
        },
    .init =
        {
            .ClockDivider = FDCAN_CLOCK_DIV1,
            .FrameFormat = FDCAN_FRAME_CLASSIC,
            .Mode = FDCAN_MODE_NORMAL,
            .AutoRetransmission = ENABLE,
            .TransmitPause = DISABLE,
            .ProtocolException = DISABLE,
            .NominalPrescaler = 4U,
            .NominalSyncJumpWidth = 1U,
            .NominalTimeSeg1 = 8U,
            .NominalTimeSeg2 = 1U,
            .DataPrescaler = 4U,
            .DataSyncJumpWidth = 1U,
            .DataTimeSeg1 = 8U,
            .DataTimeSeg2 = 1U,
            .StdFiltersNbr = 0U,
            .ExtFiltersNbr = 0U,
            .TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION,
        },
};
}  // namespace bx_can
}  // namespace ru::driver::stm32h5xx::cfg

#define RU_STM32H5XX_GPIO_MAP(X) \
  X(DEBUG_LED, ::ru::driver::stm32h5xx::cfg::gpio::DEBUG_LED)

#define RU_STM32H5XX_ADC_MAP(X) \
  X(ADC_0, ::ru::driver::stm32h5xx::cfg::adc::ADC_0) \
  X(ADC_1, ::ru::driver::stm32h5xx::cfg::adc::ADC_1)

#define RU_STM32H5XX_PWM_MAP(X) \
  X(PWM_0, ::ru::driver::stm32h5xx::cfg::pwm::PWM_0)

#define RU_STM32H5XX_TIMER_MAP(X)

#define RU_STM32H5XX_SERIAL_MAP(X)

#define RU_STM32H5XX_M_CAN_MAP(X)

#define RU_STM32H5XX_BX_CAN_MAP(X) \
  X(CAN, ::ru::driver::stm32h5xx::cfg::bx_can::CAN)

#define RU_STM32H5XX_NV_MEMORY_MAP(X)

#define RU_STM32H5XX_WATCHDOG_MAP(X)
