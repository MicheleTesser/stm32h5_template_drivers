#include <algorithm>
#include <array>
#include <limits>

#include "pwm.hpp"

#include "mapping.hpp"
#include "mapping_types.hpp"
#include "stm_common.hpp"

using namespace ru::driver;

namespace ru::driver {
namespace {
const stm32h5xx::cfg::pwm_config* config_for(const PwmId id) noexcept {
  switch (id) {
#define RU_STM32H5XX_PWM_CONFIG(name, config) \
    case PwmId::name:                         \
      return &config;
    RU_STM32H5XX_PWM_MAP(RU_STM32H5XX_PWM_CONFIG)
#undef RU_STM32H5XX_PWM_CONFIG
    default:
      return nullptr;
  }
}

TIM_HandleTypeDef* general_purpose_tim_handle(const PwmId id) noexcept {
  static std::array<TIM_HandleTypeDef, static_cast<std::size_t>(PwmId::COUNT)> handles{};
  const auto index = static_cast<std::size_t>(id);
  return index < handles.size() ? &handles[index] : nullptr;
}

general_purpose_pwm_runtime_state* pwm_runtime_state(const PwmId id) noexcept {
  static std::array<general_purpose_pwm_runtime_state,
                    static_cast<std::size_t>(PwmId::COUNT)>
      states{};
  const auto index = static_cast<std::size_t>(id);
  return index < states.size() ? &states[index] : nullptr;
}

low_power_pwm_runtime_state* lptim_pwm_state(const PwmId id) noexcept {
  static std::array<low_power_pwm_runtime_state,
                    static_cast<std::size_t>(PwmId::COUNT)>
      states{};
  const auto index = static_cast<std::size_t>(id);
  return index < states.size() ? &states[index] : nullptr;
}

template <class... Ts>
struct overloaded : Ts... {
  using Ts::operator()...;
};
template <class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

constexpr uint32_t pwm_pulse(const uint32_t period,
                             const uint16_t duty_cycle_permille) noexcept {
  const auto pulse = ((period + 1U) * duty_cycle_permille) / 1000U;
  return pulse > period ? period : pulse;
}

constexpr uint32_t pwm_channel_enable_mask(const uint32_t channel) noexcept {
  switch (channel) {
    case TIM_CHANNEL_1:
      return TIM_CCER_CC1E;
    case TIM_CHANNEL_2:
      return TIM_CCER_CC2E;
    case TIM_CHANNEL_3:
      return TIM_CCER_CC3E;
    case TIM_CHANNEL_4:
      return TIM_CCER_CC4E;
    default:
      return 0U;
  }
}

constexpr uint16_t pwm_duty_cycle_permille(
    const uint32_t period, const uint32_t compare_value) noexcept {
  if (period == 0U || compare_value == 0U) {
    return 0U;
  }

  if (compare_value >= period) {
    return 1000U;
  }

  return static_cast<uint16_t>((compare_value * 1000U) / (period + 1U));
}

uint32_t pwm_prescaler(const TIM_TypeDef* instance,
                       const uint32_t frequency_hz) noexcept {
  const auto timer_clock_hz = timer_input_clock_hz(instance);
  const auto target_period_ticks =
      std::max<uint64_t>(timer_clock_hz / frequency_hz, 1ULL);
  const auto max_period_ticks =
      static_cast<uint64_t>(timer_auto_reload_max(instance)) + 1ULL;
  const auto prescaler_div =
      std::max<uint64_t>((target_period_ticks + max_period_ticks - 1ULL) /
                             max_period_ticks,
                         1ULL);
  const auto bounded_prescaler_div = std::min<uint64_t>(
      prescaler_div,
      static_cast<uint64_t>(std::numeric_limits<uint16_t>::max()) + 1ULL);
  return static_cast<uint32_t>(bounded_prescaler_div - 1ULL);
}

uint32_t pwm_period(const TIM_TypeDef* instance, const uint32_t frequency_hz,
                    const uint32_t prescaler) noexcept {
  const auto timer_clock_hz = timer_input_clock_hz(instance);
  const auto target_period_ticks =
      std::max<uint64_t>(timer_clock_hz / frequency_hz, 1ULL);
  const auto prescaler_div = static_cast<uint64_t>(prescaler) + 1ULL;
  const auto max_period_ticks =
      static_cast<uint64_t>(timer_auto_reload_max(instance)) + 1ULL;
  const auto period_ticks = std::clamp<uint64_t>(
      target_period_ticks / prescaler_div, 1ULL, max_period_ticks);
  return static_cast<uint32_t>(period_ticks - 1ULL);
}

struct lptim_prescaler_config {
  uint32_t bits;
  uint32_t divider;
};

lptim_prescaler_config lptim_prescaler(const uint32_t source_clock_hz,
                                       const uint32_t frequency_hz) noexcept {
  constexpr std::array<lptim_prescaler_config, 8U> k_prescalers{{
      {0U, 1U},
      {LPTIM_PRESCALER_DIV2, 2U},
      {LPTIM_PRESCALER_DIV4, 4U},
      {LPTIM_PRESCALER_DIV8, 8U},
      {LPTIM_PRESCALER_DIV16, 16U},
      {LPTIM_PRESCALER_DIV32, 32U},
      {LPTIM_PRESCALER_DIV64, 64U},
      {LPTIM_PRESCALER_DIV128, 128U},
  }};

  for (const auto& prescaler : k_prescalers) {
    const auto counter_clock_hz = source_clock_hz / prescaler.divider;
    if ((counter_clock_hz / frequency_hz) <= 65536U) {
      return prescaler;
    }
  }

  return k_prescalers.back();
}

uint32_t lptim_period(const uint32_t source_clock_hz,
                      const uint32_t frequency_hz,
                      const uint32_t prescaler_divider) noexcept {
  const auto counter_clock_hz =
      source_clock_hz / std::max<uint32_t>(prescaler_divider, 1U);
  const auto period_ticks = std::clamp<uint64_t>(
      counter_clock_hz / frequency_hz, 2ULL, 65536ULL);
  return static_cast<uint32_t>(period_ticks - 1ULL);
}

uint32_t lptim_compare(const uint32_t period,
                       const uint16_t duty_cycle_permille) noexcept {
  if (duty_cycle_permille == 0U) {
    return period;
  }

  const auto compare_ticks =
      ((static_cast<uint64_t>(period) + 1ULL) * (1000U - duty_cycle_permille)) /
      1000ULL;
  return compare_ticks == 0ULL ? 0U : static_cast<uint32_t>(compare_ticks - 1ULL);
}

uint16_t lptim_duty_cycle_permille(const uint32_t period,
                                   const uint32_t compare_value) noexcept {
  if (compare_value >= period) {
    return 0U;
  }

  return static_cast<uint16_t>(
      1000U -
      (((static_cast<uint64_t>(compare_value) + 1ULL) * 1000ULL) /
       (static_cast<uint64_t>(period) + 1ULL)));
}

uint32_t lptim_channel_enable_mask(const uint32_t channel) noexcept {
  switch (channel) {
    case LPTIM_CHANNEL_1:
      return LPTIM_CCMR1_CC1E;
    case LPTIM_CHANNEL_2:
      return LPTIM_CCMR1_CC2E;
    default:
      return 0U;
  }
}

uint32_t lptim_compare_ok_flag(const uint32_t channel) noexcept {
  switch (channel) {
    case LPTIM_CHANNEL_1:
      return LPTIM_ISR_CMP1OK;
    case LPTIM_CHANNEL_2:
      return LPTIM_ISR_CMP2OK;
    default:
      return 0U;
  }
}

uint32_t lptim_compare_ok_clear_flag(const uint32_t channel) noexcept {
  switch (channel) {
    case LPTIM_CHANNEL_1:
      return LPTIM_ICR_CMP1OKCF;
    case LPTIM_CHANNEL_2:
      return LPTIM_ICR_CMP2OKCF;
    default:
      return 0U;
  }
}

bool wait_for_lptim_flag(LPTIM_TypeDef* const p_instance,
                         const uint32_t flag) noexcept {
  for (uint32_t attempt = 0U; attempt < 100000U; ++attempt) {
    if ((p_instance->ISR & flag) != 0U) {
      return true;
    }
  }

  return false;
}

bool lptim_write_compare(LPTIM_TypeDef* const p_instance, const uint32_t channel,
                         const uint32_t pulse) noexcept {
  if (channel != LPTIM_CHANNEL_1 && channel != LPTIM_CHANNEL_2) {
    return false;
  }

  const bool was_enabled = (p_instance->CR & LPTIM_CR_ENABLE) != 0U;
  if (!was_enabled) {
    p_instance->CR |= LPTIM_CR_ENABLE;
  }

  p_instance->ICR = lptim_compare_ok_clear_flag(channel);
  if (channel == LPTIM_CHANNEL_1) {
    p_instance->CCR1 = pulse;
  } else {
    p_instance->CCR2 = pulse;
  }

  const bool ok = wait_for_lptim_flag(p_instance, lptim_compare_ok_flag(channel));
  if (!was_enabled) {
    p_instance->CR &= ~LPTIM_CR_ENABLE;
  }

  return ok;
}
}  // namespace

opaque_pwm make_opaque_pwm(const PwmId id,
                           const stm32h5xx::cfg::pwm_config& config) noexcept {
  if (const auto* const general_purpose = config.general_purpose();
      general_purpose != nullptr) {
    auto* const runtime = pwm_runtime_state(id);
    auto* const p_handle = general_purpose_tim_handle(id);
    if (runtime == nullptr || p_handle == nullptr) {
      return {};
    }

    runtime->p_handle = p_handle;
    return opaque_pwm{opaque_general_purpose_pwm{
        .p_state = runtime,
        .channel = general_purpose->channel,
    }};
  }

  if (const auto* const low_power = config.low_power(); low_power != nullptr) {
    auto* const state = lptim_pwm_state(id);
    if (state == nullptr) {
      return {};
    }

    return opaque_pwm{opaque_low_power_pwm{
        .p_state = state,
        .channel = low_power->channel,
    }};
  }

  return {};
}

namespace {
opaque_pwm make_opaque(const PwmId id) noexcept {
  const auto* const config = config_for(id);
  return config != nullptr ? make_opaque_pwm(id, *config) : opaque_pwm{};
}
}  // namespace

Pwm::Pwm(const PwmId id) noexcept : m_id(id), m_opaque(make_opaque(id)) {
}

result Pwm::start() noexcept {
  return result::OK;
}

result Pwm::init() noexcept {
  return config_for(m_id) != nullptr ? result::OK : result::UNRECOVERABLE_ERROR;
}

result Pwm::stop() noexcept {
  const auto* const config = config_for(m_id);
  if (config == nullptr) {
    return result::UNRECOVERABLE_ERROR;
  }

  const auto stop_status = m_opaque.disable();
  if (stop_status != result::OK) {
    return stop_status;
  }

  const auto deinit_status = m_opaque.deinit();
  if (deinit_status == result::OK) {
    m_opaque.reset();
  }

  return deinit_status;
}

result Pwm::enable() noexcept {
  return m_opaque.enable();
}

result Pwm::disable() noexcept {
  return m_opaque.disable();
}

result Pwm::set_frequency(const uint32_t frequency_hz) noexcept {
  const auto* const config = config_for(m_id);
  if (config == nullptr) {
    return result::UNRECOVERABLE_ERROR;
  }

  if (!m_opaque.initialized()) {
    return result::UNRECOVERABLE_ERROR;
  }

  const auto duty_cycle = m_opaque.get_duty_cycle();
  if (!duty_cycle.has_value() && m_opaque.is_enabled()) {
    return duty_cycle.error();
  }

  const auto stop_status = m_opaque.disable();
  if (stop_status != result::OK) {
    return stop_status;
  }

  const auto deinit_status = m_opaque.deinit();
  if (deinit_status != result::OK) {
    return deinit_status;
  }

  m_opaque.reset();

  const auto init_status = m_opaque.init(*config, frequency_hz);
  if (init_status != result::OK) {
    return init_status;
  }

  if (duty_cycle.has_value()) {
    const auto duty_status = m_opaque.update_duty_cycle(duty_cycle.value());
    if (duty_status != result::OK) {
      return duty_status;
    }
  }

  return result::OK;
}

result Pwm::set_duty_cycle(const uint16_t duty_cycle_permille) noexcept {
  const auto* const config = config_for(m_id);
  if (config == nullptr) {
    return result::UNRECOVERABLE_ERROR;
  }

  const auto clamped_duty_cycle =
      static_cast<uint16_t>(std::min<uint16_t>(duty_cycle_permille, 1000U));
  if (!m_opaque.initialized()) {
    return result::RECOVERABLE_ERROR;
  }

  return m_opaque.update_duty_cycle(clamped_duty_cycle);
}

expected::expected<uint16_t, result> Pwm::get_duty_cycle() const noexcept {
  return m_opaque.get_duty_cycle();
}

result opaque_general_purpose_pwm::init(
    const stm32h5xx::cfg::pwm_config& config,
    const uint32_t frequency_hz) noexcept {
  if (p_state == nullptr || p_state->p_handle == nullptr || frequency_hz == 0U) {
    return result::UNRECOVERABLE_ERROR;
  }

  const auto* const backend = config.general_purpose();
  if (backend == nullptr) {
    return result::UNRECOVERABLE_ERROR;
  }

  auto* const p_handle = p_state->p_handle;
  auto& r_handle = *p_handle;
  auto* const p_instance = backend->instance();

  enable_tim_clock(p_instance);
  init_pin(config.port(), config.pin, GPIO_MODE_AF_PP, config.pull,
           GPIO_SPEED_FREQ_VERY_HIGH, config.alternate);

  r_handle.Instance = p_instance;
  r_handle.Init = backend->init;
  r_handle.Init.Prescaler = pwm_prescaler(p_instance, frequency_hz);
  r_handle.Init.Period =
      pwm_period(p_instance, frequency_hz, r_handle.Init.Prescaler);

  if (HAL_TIM_PWM_Init(&r_handle) != HAL_OK) {
    return result::RECOVERABLE_ERROR;
  }

  auto channel_config = backend->channel_init;
  return from_hal_status(
      HAL_TIM_PWM_ConfigChannel(&r_handle, &channel_config, channel));
}

result opaque_general_purpose_pwm::enable() noexcept {
  if (p_state == nullptr || p_state->p_handle == nullptr) {
    return result::UNRECOVERABLE_ERROR;
  }

  auto& r_handle = *p_state->p_handle;
  if (r_handle.Instance == nullptr) {
    return result::UNRECOVERABLE_ERROR;
  }

  return from_hal_status(HAL_TIM_PWM_Start(&r_handle, channel));
}

result opaque_general_purpose_pwm::disable() noexcept {
  if (p_state == nullptr || p_state->p_handle == nullptr) {
    return result::UNRECOVERABLE_ERROR;
  }

  auto& r_handle = *p_state->p_handle;
  if (r_handle.Instance == nullptr) {
    return result::OK;
  }

  return from_hal_status(HAL_TIM_PWM_Stop(&r_handle, channel));
}

result opaque_general_purpose_pwm::deinit() noexcept {
  if (p_state == nullptr || p_state->p_handle == nullptr) {
    return result::UNRECOVERABLE_ERROR;
  }

  auto& r_handle = *p_state->p_handle;
  if (r_handle.Instance == nullptr) {
    return result::OK;
  }

  return from_hal_status(HAL_TIM_PWM_DeInit(&r_handle));
}

bool opaque_general_purpose_pwm::is_enabled() const noexcept {
  if (p_state == nullptr || p_state->p_handle == nullptr) {
    return false;
  }

  const auto& r_handle = *p_state->p_handle;
  return r_handle.Instance != nullptr &&
         (r_handle.Instance->CCER & pwm_channel_enable_mask(channel)) != 0U;
}

result opaque_general_purpose_pwm::update_duty_cycle(
    const uint16_t duty_cycle_permille) noexcept {
  if (p_state == nullptr || p_state->p_handle == nullptr) {
    return result::UNRECOVERABLE_ERROR;
  }

  auto& r_handle = *p_state->p_handle;
  if (r_handle.Instance == nullptr) {
    return result::OK;
  }

  __HAL_TIM_SET_COMPARE(&r_handle, channel,
                        pwm_pulse(r_handle.Instance->ARR, duty_cycle_permille));
  return result::OK;
}

expected::expected<uint16_t, result> opaque_general_purpose_pwm::get_duty_cycle()
    const noexcept {
  if (p_state == nullptr || p_state->p_handle == nullptr) {
    return expected::unexpected(result::UNRECOVERABLE_ERROR);
  }

  const auto& r_handle = *p_state->p_handle;
  if (r_handle.Instance == nullptr) {
    return expected::unexpected(result::UNRECOVERABLE_ERROR);
  }

  return pwm_duty_cycle_permille(
      r_handle.Instance->ARR,
      __HAL_TIM_GET_COMPARE(const_cast<TIM_HandleTypeDef*>(&r_handle), channel));
}

result opaque_low_power_pwm::init(const stm32h5xx::cfg::pwm_config& config,
                                  const uint32_t frequency_hz) noexcept {
  if (p_state == nullptr || frequency_hz == 0U) {
    return result::UNRECOVERABLE_ERROR;
  }

  const auto* const backend = config.low_power();
  if (backend == nullptr || backend->source_clock_hz == 0U) {
    return result::UNRECOVERABLE_ERROR;
  }

  auto* const p_instance = backend->instance();
  enable_lptim_clock(p_instance);
  init_pin(config.port(), config.pin, GPIO_MODE_AF_PP, config.pull,
           GPIO_SPEED_FREQ_VERY_HIGH, config.alternate);

  const auto prescaler = lptim_prescaler(backend->source_clock_hz, frequency_hz);
  auto init = backend->init;
  init.Clock.Prescaler = prescaler.bits;
  init.Period =
      lptim_period(backend->source_clock_hz, frequency_hz, prescaler.divider);

  auto& handle = p_state->handle;
  handle = {};
  handle.Instance = p_instance;
  handle.Init = init;

  if (HAL_LPTIM_Init(&handle) != HAL_OK) {
    handle.Instance = nullptr;
    return result::RECOVERABLE_ERROR;
  }

  auto channel_config = backend->channel_init;
  channel_config.Pulse = std::min<uint32_t>(channel_config.Pulse, init.Period);
  if (HAL_LPTIM_OC_ConfigChannel(&handle, &channel_config, channel) != HAL_OK) {
    (void)HAL_LPTIM_DeInit(&handle);
    handle.Instance = nullptr;
    return result::RECOVERABLE_ERROR;
  }

  return result::OK;
}

result opaque_low_power_pwm::enable() noexcept {
  if (p_state == nullptr || p_state->handle.Instance == nullptr) {
    return result::UNRECOVERABLE_ERROR;
  }

  return from_hal_status(HAL_LPTIM_PWM_Start(&p_state->handle, channel));
}

result opaque_low_power_pwm::disable() noexcept {
  if (p_state == nullptr || p_state->handle.Instance == nullptr) {
    return result::OK;
  }

  return from_hal_status(HAL_LPTIM_PWM_Stop(&p_state->handle, channel));
}

result opaque_low_power_pwm::deinit() noexcept {
  if (p_state == nullptr || p_state->handle.Instance == nullptr) {
    return result::OK;
  }

  const auto status = from_hal_status(HAL_LPTIM_DeInit(&p_state->handle));
  if (status == result::OK) {
    p_state->handle.Instance = nullptr;
  }
  return status;
}

bool opaque_low_power_pwm::is_enabled() const noexcept {
  return p_state != nullptr && p_state->handle.Instance != nullptr &&
         (p_state->handle.Instance->CCMR1 & lptim_channel_enable_mask(channel)) != 0U;
}

result opaque_low_power_pwm::update_duty_cycle(
    const uint16_t duty_cycle_permille) noexcept {
  if (p_state == nullptr || p_state->handle.Instance == nullptr) {
    return result::OK;
  }

  auto* const p_instance = p_state->handle.Instance;
  const auto pulse = lptim_compare(p_instance->ARR, duty_cycle_permille);
  return lptim_write_compare(p_instance, channel, pulse) ? result::OK
                                                          : result::RECOVERABLE_ERROR;
}

expected::expected<uint16_t, result> opaque_low_power_pwm::get_duty_cycle()
    const noexcept {
  if (p_state == nullptr || p_state->handle.Instance == nullptr) {
    return expected::unexpected(result::UNRECOVERABLE_ERROR);
  }

  const auto* const p_instance = p_state->handle.Instance;
  const auto compare =
      channel == LPTIM_CHANNEL_1 ? p_instance->CCR1 : p_instance->CCR2;
  return lptim_duty_cycle_permille(p_instance->ARR, compare);
}

result opaque_pwm::init(const stm32h5xx::cfg::pwm_config& config,
                        const uint32_t frequency_hz) noexcept {
  return std::visit(
      overloaded{
          [&](std::monostate&) { return result::UNRECOVERABLE_ERROR; },
          [&](auto& backend) { return backend.init(config, frequency_hz); },
      },
      m_backend);
}

bool opaque_pwm::initialized() const noexcept {
  return std::visit(
      overloaded{
          [&](const std::monostate&) { return false; },
          [&](const opaque_general_purpose_pwm& backend) {
            return backend.p_state != nullptr && backend.p_state->p_handle != nullptr &&
                   backend.p_state->p_handle->Instance != nullptr;
          },
          [&](const opaque_low_power_pwm& backend) {
            return backend.p_state != nullptr && backend.p_state->handle.Instance != nullptr;
          },
      },
      m_backend);
}

void opaque_pwm::reset() noexcept {
  std::visit(
      overloaded{
          [&](std::monostate&) {},
          [&](opaque_general_purpose_pwm& backend) {
            if (backend.p_state != nullptr && backend.p_state->p_handle != nullptr) {
              backend.p_state->p_handle->Instance = nullptr;
            }
          },
          [&](opaque_low_power_pwm& backend) {
            if (backend.p_state != nullptr) {
              backend.p_state->handle.Instance = nullptr;
            }
          },
      },
      m_backend);
}

result opaque_pwm::enable() noexcept {
  return std::visit(
      overloaded{
          [&](std::monostate&) { return result::RECOVERABLE_ERROR; },
          [&](auto& backend) { return backend.enable(); },
      },
      m_backend);
}

result opaque_pwm::disable() noexcept {
  return std::visit(
      overloaded{
          [&](std::monostate&) { return result::RECOVERABLE_ERROR; },
          [&](auto& backend) { return backend.disable(); },
      },
      m_backend);
}

result opaque_pwm::deinit() noexcept {
  return std::visit(
      overloaded{
          [&](std::monostate&) { return result::RECOVERABLE_ERROR; },
          [&](auto& backend) { return backend.deinit(); },
      },
      m_backend);
}

bool opaque_pwm::is_enabled() const noexcept {
  return std::visit(
      overloaded{
          [&](const std::monostate&) { return false; },
          [&](const auto& backend) { return backend.is_enabled(); },
      },
      m_backend);
}

result opaque_pwm::update_duty_cycle(
    const uint16_t duty_cycle_permille) noexcept {
  return std::visit(
      overloaded{
          [&](std::monostate&) { return result::RECOVERABLE_ERROR; },
          [&](auto& backend) { return backend.update_duty_cycle(duty_cycle_permille); },
      },
      m_backend);
}

expected::expected<uint16_t, result> opaque_pwm::get_duty_cycle() const noexcept {
  return std::visit(
      overloaded{
          [&](const std::monostate&) -> expected::expected<uint16_t, result> {
            return expected::unexpected(result::RECOVERABLE_ERROR);
          },
          [&](const auto& backend) { return backend.get_duty_cycle(); },
      },
      m_backend);
}
}  // namespace ru::driver
