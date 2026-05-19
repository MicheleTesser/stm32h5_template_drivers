#include "gpio.hpp"

#include <cassert>

#include "mapping.hpp"
#include "stm_common.hpp"

using namespace ru::driver;

namespace ru::driver {
namespace {
const stm32h5xx::cfg::gpio_config* config_for(const GpioId id) noexcept {
  switch (id) {
#define RU_STM32H5XX_GPIO_CONFIG(name, config) \
    case GpioId::name:                         \
      return &config;
    RU_STM32H5XX_GPIO_MAP(RU_STM32H5XX_GPIO_CONFIG)
#undef RU_STM32H5XX_GPIO_CONFIG
    default:
      return nullptr;
  }
}

opaque_gpio make_opaque(const GpioId id) noexcept {
  const auto* const config = config_for(id);
  if (config == nullptr) {
    return opaque_gpio{};
  }

  return opaque_gpio{config->port(), config->pin(), config->active_high};
}

bool is_output_mode(GPIO_TypeDef* const p_port, const uint16_t pin) noexcept {
  if (p_port == nullptr || pin == 0U) {
    return false;
  }

  const auto pin_index = static_cast<uint32_t>(__builtin_ctz(pin));
  const auto mode = (p_port->MODER >> (pin_index * 2U)) & 0x3U;
  return mode == 0x1U;
}

bool is_output_init(const GPIO_InitTypeDef& init) noexcept {
  return init.Mode == GPIO_MODE_OUTPUT_PP || init.Mode == GPIO_MODE_OUTPUT_OD;
}

}  // namespace

Gpio::Gpio(const GpioId id) noexcept : m_id(id), m_opaque(make_opaque(id)) {
}

result Gpio::start() noexcept {
  return result::OK;
}

result Gpio::init() noexcept {
  const auto* const config = config_for(m_id);
  if (config == nullptr) {
    return result::UNRECOVERABLE_ERROR;
  }

  return m_opaque.init(config->init);
}

result Gpio::stop() noexcept {
  return m_opaque.stop();
}

GpioValue Gpio::active_value() const noexcept {
  return m_opaque.active_high() ? GpioValue::HIGH : GpioValue::LOW;
}

GpioPolarity Gpio::polarity() const noexcept {
  return m_opaque.active_high() ? GpioPolarity::ACTIVE_HIGH : GpioPolarity::ACTIVE_LOW;
}

expected::expected<bool, result> Gpio::is_active() const noexcept {
  return m_opaque.is_active();
}

expected::expected<bool, result> Gpio::is_inactive() const noexcept {
  const auto active = is_active();
  if (!active.has_value()) {
    return expected::unexpected(active.error());
  }

  return !active.value();
}

expected::expected<bool, result> Gpio::is_high() const noexcept {
  return m_opaque.is_high();
}

expected::expected<bool, result> Gpio::is_low() const noexcept {
  const auto high = is_high();
  if (!high.has_value()) {
    return expected::unexpected(high.error());
  }

  return !high.value();
}

result Gpio::set_active() noexcept {
  return m_opaque.set_level(true);
}

result Gpio::set_inactive() noexcept {
  return m_opaque.set_level(false);
}

result Gpio::set_level(const bool active) noexcept {
  return active ? set_active() : set_inactive();
}

result Gpio::toggle() noexcept {
  return m_opaque.toggle();
}

result opaque_gpio::init(const GPIO_InitTypeDef& init) const noexcept {
  if (m_p_port == nullptr || m_pin == 0U) {
    return result::UNRECOVERABLE_ERROR;
  }

  init_pin(m_p_port, init);

  if (is_output_init(init)) {
    return set_level(false);
  }

  return result::OK;
}

result opaque_gpio::stop() const noexcept {
  if (m_p_port == nullptr || m_pin == 0U) {
    return result::UNRECOVERABLE_ERROR;
  }

  HAL_GPIO_DeInit(m_p_port, m_pin);
  return result::OK;
}

bool opaque_gpio::is_active() const noexcept {
  assert(m_p_port);
  return (HAL_GPIO_ReadPin(m_p_port, m_pin) == GPIO_PIN_SET) == m_active_high;
}

bool opaque_gpio::is_high() const noexcept {
  assert(m_p_port);
  return HAL_GPIO_ReadPin(m_p_port, m_pin) == GPIO_PIN_SET;
}

result opaque_gpio::set_level(const bool active) const noexcept {
  assert(m_p_port);
  if (!is_output_mode(m_p_port, m_pin)) {
    return result::UNRECOVERABLE_ERROR;
  }

  HAL_GPIO_WritePin(m_p_port, m_pin,
                    active == m_active_high ? GPIO_PIN_SET : GPIO_PIN_RESET);
  return result::OK;
}

result opaque_gpio::toggle() const noexcept {
  assert(m_p_port);
  if (!is_output_mode(m_p_port, m_pin)) {
    return result::UNRECOVERABLE_ERROR;
  }

  HAL_GPIO_TogglePin(m_p_port, m_pin);
  return result::OK;
}
}  // namespace ru::driver
