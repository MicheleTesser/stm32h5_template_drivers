#pragma once

#include <cstdint>

#include "common.hpp"
#include "stm32h5xx_hal.h"

namespace ru::driver {
class Watchdog;

struct opaque_watchdog {
 public:
  constexpr opaque_watchdog() noexcept = default;

 private:
  friend class Watchdog;

  result init(uint32_t timeout_ms) noexcept;
  result stop() noexcept;
  result kick() noexcept;

  IWDG_HandleTypeDef m_handle{};

  bool initialized() const noexcept { return m_handle.Instance != nullptr; }
};
}  // namespace ru::driver
