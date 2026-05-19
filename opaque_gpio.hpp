#pragma once

#include <cstdint>

#include "common.hpp"
#include "stm32h5xx_hal.h"

namespace ru::driver {
class Gpio;

struct opaque_gpio {
 public:
  constexpr opaque_gpio() noexcept = default;
  constexpr opaque_gpio(GPIO_TypeDef* const p_port, const uint16_t pin,
                        const bool active_high) noexcept
      : m_p_port(p_port), m_pin(pin), m_active_high(active_high) {}

  bool active_high() const noexcept { return m_active_high; }

  result init(const GPIO_InitTypeDef& init) const noexcept;
  result stop() const noexcept;
  bool is_active() const noexcept;
  bool is_high() const noexcept;
  result set_level(bool active) const noexcept;
  result toggle() const noexcept;

 private:
  GPIO_TypeDef* m_p_port{nullptr};
  uint16_t m_pin{0U};
  bool m_active_high{false};
};
}  // namespace ru::driver
