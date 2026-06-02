#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "common.hpp"
#include "mapping.hpp"
#include "mapping_types.hpp"
#include "stm32h5xx_hal.h"  // IWYU pragma: keep

namespace ru::driver {

constexpr std::size_t kAdcDmaMaxSequenceLength = 16U;

class Adc;

struct opaque_adc {
 public:
  constexpr opaque_adc() noexcept = default;
  constexpr explicit opaque_adc(
      const stm32h5xx::cfg::adc_config* const p_config) noexcept
      : m_p_config(p_config) {}

  constexpr bool valid() const noexcept {
    return m_p_config != nullptr;
  }

  bool uses_dma() const noexcept;
  ADC_TypeDef* instance() const noexcept;
  GPIO_TypeDef* port() const noexcept;
  const GPIO_InitTypeDef& pin_init() const noexcept;
  const ADC_InitTypeDef& adc_init() const noexcept;
  const ADC_ChannelConfTypeDef& channel_init() const noexcept;
  const ADC_ChannelConfTypeDef* dma_channel_sequence() const noexcept;
  std::size_t dma_frame_count() const noexcept;
  std::size_t dma_sequence_length() const noexcept;
  std::size_t dma_sequence_index() const noexcept;
  stm32h5xx::cfg::adc_dma_backend dma_backend() const noexcept;
  std::size_t dma_window_width() const noexcept;
  uint32_t dma_request() const noexcept;
  DMA_Channel_TypeDef* dma_channel() const noexcept;
  IRQn_Type dma_irq() const noexcept;
  bool uses_timer_trigger() const noexcept;
  TIM_TypeDef* trigger_timer_instance() const noexcept;
  uint32_t trigger_counter_clock_hz() const noexcept;
  uint32_t trigger_frequency_hz() const noexcept;
  const TIM_Base_InitTypeDef& trigger_timer_init() const noexcept;
  const TIM_MasterConfigTypeDef& trigger_master_config() const noexcept;

 private:
  friend class Adc;

  std::optional<AdcId> id() const noexcept;
  bool initialized() const noexcept;
  static void start() noexcept;
  result init() noexcept;
  result stop() noexcept;
  result read(uint16_t& r_value) noexcept;
  result try_read(bool& r_has_value, uint16_t& r_value) noexcept;

 public:
  ADC_HandleTypeDef m_handle{};
  TIM_HandleTypeDef m_trigger_timer_handle{};
  DMA_NodeTypeDef m_dma_node{};
  DMA_QListTypeDef m_dma_queue{};
  DMA_HandleTypeDef m_dma_channel_handle{};
  const stm32h5xx::cfg::adc_config* m_p_config{nullptr};
};
}  // namespace ru::driver
