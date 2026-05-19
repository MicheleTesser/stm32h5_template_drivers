#pragma once

#include <cstdint>

#include "common.hpp"
#include "stm32h5xx_hal.h"

namespace ru::driver::stm32h5xx::cfg {
struct timer_config;
}

namespace ru::driver {
class Timer;

struct opaque_timer {
 public:
  constexpr opaque_timer() noexcept = default;
  constexpr explicit opaque_timer(TIM_HandleTypeDef* const p_handle) noexcept
      : m_p_handle(p_handle) {}

 private:
  friend class Timer;

  bool initialized() const noexcept;
  result init(const stm32h5xx::cfg::timer_config& config) const noexcept;
  result stop() const noexcept;
  expected::expected<uint64_t, result> time_now() const noexcept;

  TIM_HandleTypeDef* m_p_handle{nullptr};
  mutable uint32_t m_counter_clock_hz{0U};
};
}  // namespace ru::driver
