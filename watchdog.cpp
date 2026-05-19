#include "watchdog.hpp"

#include <array>

#include "stm_common.hpp"

using namespace ru::driver;

namespace ru::driver {
namespace {
constexpr uint32_t k_default_watchdog_timeout_ms = 1000U;

constexpr uint32_t k_max_reload = 0x0FFFU;
constexpr uint32_t k_lsi_hz = LSI_VALUE;

struct PrescalerOption {
  uint32_t hal_value;
  uint32_t divider;
};

constexpr std::array<PrescalerOption, 9U> k_prescalers{{
    {IWDG_PRESCALER_4, 4U},
    {IWDG_PRESCALER_8, 8U},
    {IWDG_PRESCALER_16, 16U},
    {IWDG_PRESCALER_32, 32U},
    {IWDG_PRESCALER_64, 64U},
    {IWDG_PRESCALER_128, 128U},
    {IWDG_PRESCALER_256, 256U},
    {IWDG_PRESCALER_512, 512U},
    {IWDG_PRESCALER_1024, 1024U},
}};

bool choose_timeout(const uint32_t timeout_ms, uint32_t& prescaler,
                    uint32_t& reload) noexcept {
  if (timeout_ms == 0U) {
    return false;
  }

  for (const auto& option : k_prescalers) {
    const auto denominator = static_cast<uint64_t>(1000U) * option.divider;
    const auto ticks = (static_cast<uint64_t>(timeout_ms) * k_lsi_hz + denominator - 1U) /
                       denominator;

    if ((ticks - 1U) <= k_max_reload) {
      prescaler = option.hal_value;
      reload = static_cast<uint32_t>(ticks - 1U);
      return true;
    }
  }

  return false;
}
}  // namespace

Watchdog::Watchdog(const WatchdogId id) noexcept : m_id(id), m_opaque() {
}

result Watchdog::start() noexcept {
  return result::OK;
}

result Watchdog::init() noexcept {
  return m_opaque.init(k_default_watchdog_timeout_ms);
}

result Watchdog::stop() noexcept {
  return m_opaque.stop();
}

result Watchdog::kick() noexcept {
  return m_opaque.kick();
}

Timestamp Watchdog::timeout() const noexcept {
  return static_cast<Timestamp>(k_default_watchdog_timeout_ms) * 1000U;
}

result opaque_watchdog::init(const uint32_t timeout_ms) noexcept {
  if (initialized()) {
    return kick();
  }

  uint32_t prescaler{0U};
  uint32_t reload{0U};
  if (!choose_timeout(timeout_ms, prescaler, reload)) {
    return result::UNRECOVERABLE_ERROR;
  }

  m_handle = {};
  m_handle.Instance = IWDG;
  m_handle.Init.Prescaler = prescaler;
  m_handle.Init.Reload = reload;
  m_handle.Init.Window = IWDG_WINDOW_DISABLE;
  m_handle.Init.EWI = IWDG_EWI_DISABLE;

  const auto status = from_hal_status(HAL_IWDG_Init(&m_handle));
  if (status != result::OK) {
    m_handle.Instance = nullptr;
  }

  return status;
}

result opaque_watchdog::stop() noexcept {
  return initialized() ? result::RECOVERABLE_ERROR : result::OK;
}

result opaque_watchdog::kick() noexcept {
  if (!initialized()) {
    return result::UNRECOVERABLE_ERROR;
  }

  return from_hal_status(HAL_IWDG_Refresh(&m_handle));
}
}  // namespace ru::driver
