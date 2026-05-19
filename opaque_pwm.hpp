#pragma once

#include <cstdint>
#include <variant>

#include "common.hpp"
#include "stm32h5xx_hal.h"

namespace ru::driver::stm32h5xx::cfg {
struct pwm_config;
}

namespace ru::driver {
class Pwm;

struct general_purpose_pwm_runtime_state {
  TIM_HandleTypeDef* p_handle{nullptr};
};

struct low_power_pwm_runtime_state {
  LPTIM_HandleTypeDef handle{};
};

struct opaque_general_purpose_pwm {
  result init(const stm32h5xx::cfg::pwm_config& config,
              uint32_t frequency_hz) noexcept;
  result enable() noexcept;
  result disable() noexcept;
  result deinit() noexcept;
  bool is_enabled() const noexcept;
  result update_duty_cycle(uint16_t duty_cycle_permille) noexcept;
  expected::expected<uint16_t, result> get_duty_cycle() const noexcept;

  general_purpose_pwm_runtime_state* p_state{nullptr};
  uint32_t channel{0U};
};

struct opaque_low_power_pwm {
  result init(const stm32h5xx::cfg::pwm_config& config,
              uint32_t frequency_hz) noexcept;
  result enable() noexcept;
  result disable() noexcept;
  result deinit() noexcept;
  bool is_enabled() const noexcept;
  result update_duty_cycle(uint16_t duty_cycle_permille) noexcept;
  expected::expected<uint16_t, result> get_duty_cycle() const noexcept;

  low_power_pwm_runtime_state* p_state{nullptr};
  uint32_t channel{0U};
};

struct opaque_pwm {
 public:
  constexpr opaque_pwm() noexcept = default;
  constexpr explicit opaque_pwm(const opaque_general_purpose_pwm& backend) noexcept
      : m_backend(backend) {}
  constexpr explicit opaque_pwm(const opaque_low_power_pwm& backend) noexcept
      : m_backend(backend) {}

 private:
  friend class Pwm;

  using backend_variant =
      std::variant<std::monostate, opaque_general_purpose_pwm, opaque_low_power_pwm>;

  result init(const stm32h5xx::cfg::pwm_config& config,
              uint32_t frequency_hz) noexcept;
  bool initialized() const noexcept;
  void reset() noexcept;
  result enable() noexcept;
  result disable() noexcept;
  result deinit() noexcept;
  bool is_enabled() const noexcept;
  result update_duty_cycle(uint16_t duty_cycle_permille) noexcept;
  expected::expected<uint16_t, result> get_duty_cycle() const noexcept;

  backend_variant m_backend{};
};
}  // namespace ru::driver
