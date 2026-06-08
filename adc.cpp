#include <algorithm>
#include <optional>

#include "adc.hpp"

#include "mapping.hpp"
#include "stm_common.hpp"
#include "stm32h5xx_hal_adc_ex.h"
#include "stm32h5xx_hal_dma_ex.h"
#include "stm32h5xx_hal_tim.h"
#include "stm32h5xx_hal_tim_ex.h"

using namespace ru::driver;

namespace ru::driver {
namespace {
constexpr uint32_t DMA_FIXED_WINDOW_READY = 0x80000000UL;
constexpr uint32_t DMA_SAMPLE_COUNT_MASK = ~DMA_FIXED_WINDOW_READY;

constexpr std::size_t dma_transfer_length(
    const stm32h5xx::cfg::adc_config& config) noexcept {
  return config.dma_frame_count == 0U ? 1U : config.dma_frame_count;
}

template <const stm32h5xx::cfg::adc_config* Config>
uint16_t* dma_buffer_data() noexcept {
  if constexpr (Config->uses_dma) {
    static uint16_t buffer[dma_transfer_length(*Config)]{};
    return buffer;
  }

  return nullptr;
}

template <const stm32h5xx::cfg::adc_config* Config>
constexpr std::size_t dma_buffer_size() noexcept {
  if constexpr (Config->uses_dma) {
    return dma_transfer_length(*Config);
  }

  return 0U;
}

constexpr const stm32h5xx::cfg::adc_config* config_for(const AdcId id) noexcept {
  switch (id) {
#define RU_STM32H5XX_ADC_CONFIG(name, config) \
    case AdcId::name:                         \
      return &config;
    RU_STM32H5XX_ADC_MAP(RU_STM32H5XX_ADC_CONFIG)
#undef RU_STM32H5XX_ADC_CONFIG
    default:
      return nullptr;
  }
}

uint16_t* dma_buffer_for(
    const stm32h5xx::cfg::adc_config* const p_config) noexcept {
#define RU_STM32H5XX_ADC_BUFFER_DATA(name, config) \
  if (p_config == &config) {                       \
    return dma_buffer_data<&config>();             \
  }
  RU_STM32H5XX_ADC_MAP(RU_STM32H5XX_ADC_BUFFER_DATA)
#undef RU_STM32H5XX_ADC_BUFFER_DATA
  return nullptr;
}

std::size_t dma_buffer_size_for(
    const stm32h5xx::cfg::adc_config* const p_config) noexcept {
#define RU_STM32H5XX_ADC_BUFFER_SIZE(name, config) \
  if (p_config == &config) {                       \
    return dma_buffer_size<&config>();             \
  }
  RU_STM32H5XX_ADC_MAP(RU_STM32H5XX_ADC_BUFFER_SIZE)
#undef RU_STM32H5XX_ADC_BUFFER_SIZE
  return 0U;
}

opaque_adc make_opaque(const stm32h5xx::cfg::adc_config* const p_config) noexcept {
  return opaque_adc{p_config};
}

uint32_t timer_prescaler(const TIM_TypeDef* instance,
                         const uint32_t counter_clock_hz) noexcept {
  const auto timer_clock_hz = timer_input_clock_hz(instance);
  const auto target_counter_clock_hz = counter_clock_hz == 0U ? 1U : counter_clock_hz;
  return timer_clock_hz > target_counter_clock_hz
             ? (timer_clock_hz / target_counter_clock_hz) - 1U
             : 0U;
}

uint32_t timer_period(const uint32_t counter_clock_hz,
                      const uint32_t frequency_hz) noexcept {
  const auto target_counter_clock_hz = counter_clock_hz == 0U ? 1U : counter_clock_hz;
  const auto target_frequency_hz = frequency_hz == 0U ? 1U : frequency_hz;
  const auto period_ticks =
      std::max<uint32_t>(target_counter_clock_hz / target_frequency_hz, 1U);
  return period_ticks - 1U;
}

opaque_adc make_opaque(const AdcId id) noexcept {
  const auto* const config = config_for(id);
  return config != nullptr ? make_opaque(config) : opaque_adc{};
}
}  // namespace

result Adc::start() noexcept {
  opaque_adc::start();
  return result::OK;
}

Adc::Adc(const AdcId id) noexcept : m_id(id), m_opaque(make_opaque(id)) {
}

result Adc::init() noexcept {
  return m_opaque.init();
}

result Adc::stop() noexcept {
  return m_opaque.stop();
}

expected::expected<uint16_t, result> Adc::read() noexcept {
  uint16_t value{0U};
  const auto status = m_opaque.read(value);
  if (status != result::OK) {
    return expected::unexpected(status);
  }

  return value;
}

expected::expected<std::optional<uint16_t>, result> Adc::try_read() noexcept {
  bool has_value{false};
  uint16_t value{0U};
  const auto status = m_opaque.try_read(has_value, value);
  if (status != result::OK) {
    return expected::unexpected(status);
  }

  if (!has_value) {
    return std::optional<uint16_t>{};
  }

  return std::optional<uint16_t>{value};
}

uint16_t Adc::sample_max() const noexcept {
  return m_opaque.sample_max();
}

namespace {

std::size_t dma_transfer_length(const opaque_adc& config) noexcept {
  return config.dma_frame_count() == 0U ? 1U : config.dma_frame_count();
}

std::size_t dma_sequence_length(const opaque_adc& config) noexcept {
  const auto configured_length = config.dma_sequence_length();
  return configured_length == 0U ? 1U : configured_length;
}

std::size_t dma_window_width(const opaque_adc& config) noexcept {
  const auto configured_width = config.dma_window_width();
  return configured_width == 0U ? dma_transfer_length(config) : configured_width;
}

bool uses_fixed_window_average(const opaque_adc& config) noexcept {
  return config.dma_backend() == stm32h5xx::cfg::adc_dma_backend::fixed_window_average;
}

uint16_t* dma_buffer(const opaque_adc& runtime) noexcept {
  return dma_buffer_for(runtime.m_p_config);
}

opaque_adc*& dma_owner_slot(ADC_TypeDef* const p_instance) noexcept {
  static opaque_adc* adc1_owner{nullptr};
  static opaque_adc* adc2_owner{nullptr};
  static opaque_adc* invalid_owner{nullptr};

  switch (reinterpret_cast<uintptr_t>(p_instance)) {
    case ADC1_BASE:
      return adc1_owner;
    case ADC2_BASE:
      return adc2_owner;
    default:
      return invalid_owner;
  }
}

struct dma_shared_state {
  std::size_t processed_samples_in_cycle{0U};
  std::array<uint64_t, kAdcDmaMaxSequenceLength> average_sums{};
  std::array<uint32_t, kAdcDmaMaxSequenceLength> average_sample_counts{};
  uint64_t fixed_window_sum{0U};
  uint32_t fixed_window_count{0U};
};

dma_shared_state& dma_state_slot(ADC_TypeDef* const p_instance) noexcept {
  static dma_shared_state adc1_state{};
  static dma_shared_state adc2_state{};
  static dma_shared_state invalid_state{};

  switch (reinterpret_cast<uintptr_t>(p_instance)) {
    case ADC1_BASE:
      return adc1_state;
    case ADC2_BASE:
      return adc2_state;
    default:
      return invalid_state;
  }
}

opaque_adc* dma_owner_for_instance(ADC_TypeDef* const p_instance) noexcept {
  return p_instance != nullptr ? dma_owner_slot(p_instance) : nullptr;
}

void set_dma_owner(ADC_TypeDef* const p_instance, opaque_adc* const p_owner) noexcept {
  if (p_instance != nullptr) {
    dma_owner_slot(p_instance) = p_owner;
  }
}

bool dma_initialized(const opaque_adc& config) noexcept {
  return config.m_handle.Instance != nullptr && config.m_dma_channel_handle.Instance != nullptr;
}

opaque_adc* running_dma_owner_for(const opaque_adc& config) noexcept {
  if (!config.uses_dma()) {
    return nullptr;
  }

  auto* const owner = dma_owner_for_instance(config.instance());
  if (owner == nullptr || !dma_initialized(*owner)) {
    return nullptr;
  }

  return owner;
}

opaque_adc* running_injected_owner_for(const opaque_adc& config) noexcept {
  if (!config.uses_injected()) {
    return nullptr;
  }

  auto* const owner = dma_owner_for_instance(config.instance());
  if (owner == nullptr || !dma_initialized(*owner)) {
    return nullptr;
  }

  return owner;
}

opaque_adc* adc_from_handle(ADC_HandleTypeDef* const hadc) noexcept {
  return hadc != nullptr ? dma_owner_for_instance(hadc->Instance) : nullptr;
}

dma_shared_state& dma_state(opaque_adc& runtime) noexcept {
  return dma_state_slot(runtime.instance());
}

void accumulate_dma_samples(opaque_adc& runtime, const std::size_t begin_sample,
                            const std::size_t end_sample) noexcept {
  const auto active_buffer_length = dma_transfer_length(runtime);
  if (active_buffer_length == 0U) {
    (void)begin_sample;
    (void)end_sample;
    return;
  }

  const std::size_t begin = std::min(begin_sample, active_buffer_length);
  const std::size_t end = std::min(end_sample, active_buffer_length);
  const auto sequence_length = dma_sequence_length(runtime);
  auto& state = dma_state(runtime);
  auto* const buffer = dma_buffer(runtime);

  for (std::size_t sample = begin; sample < end; ++sample) {
    const auto channel_index = sample % sequence_length;
    state.average_sums[channel_index] += buffer[sample];
    state.average_sample_counts[channel_index] += 1U;
  }
}

void accumulate_dma_until(opaque_adc& runtime, const std::size_t boundary) noexcept {
  auto& state = dma_state(runtime);
  if (boundary <= state.processed_samples_in_cycle) {
    return;
  }

  accumulate_dma_samples(runtime, state.processed_samples_in_cycle, boundary);
  state.processed_samples_in_cycle = boundary;
}

uint64_t sum_fixed_dma_window(const opaque_adc& runtime,
                              const std::size_t end_sample) noexcept {
  const auto active_buffer_length = dma_transfer_length(runtime);
  const auto window_width = dma_window_width(runtime);
  auto* const buffer = dma_buffer(runtime);
  uint64_t sum{0U};

  const auto sum_range = [buffer, &sum](const std::size_t begin,
                                        const std::size_t end) noexcept {
    for (std::size_t sample = begin; sample < end; ++sample) {
      sum += buffer[sample];
    }
  };

  if (window_width <= end_sample) {
    sum_range(end_sample - window_width, end_sample);
    return sum;
  }

  const auto wrapped_count = window_width - end_sample;
  sum_range(active_buffer_length - wrapped_count, active_buffer_length);
  sum_range(0U, end_sample);
  return sum;
}

void complete_fixed_dma_window(opaque_adc& runtime,
                               const std::size_t boundary) noexcept {
  const auto active_buffer_length = dma_transfer_length(runtime);
  auto& state = dma_state(runtime);
  if (active_buffer_length == 0U || boundary > active_buffer_length ||
      boundary <= state.processed_samples_in_cycle) {
    return;
  }

  const auto window_width = dma_window_width(runtime);
  const auto completed_count = boundary - state.processed_samples_in_cycle;
  const auto previous_valid_count = state.fixed_window_count & DMA_SAMPLE_COUNT_MASK;
  const auto valid_count =
      std::min<uint32_t>(static_cast<uint32_t>(window_width),
                         previous_valid_count + static_cast<uint32_t>(completed_count));

  state.processed_samples_in_cycle = boundary;
  if (valid_count < window_width) {
    state.fixed_window_count = valid_count;
    return;
  }

  state.fixed_window_sum = sum_fixed_dma_window(runtime, boundary);
  state.fixed_window_count = static_cast<uint32_t>(window_width) | DMA_FIXED_WINDOW_READY;
}

bool prepare_dma_runtime(opaque_adc& runtime) noexcept {
  auto& state = dma_state(runtime);
  state.average_sums.fill(0U);
  state.average_sample_counts.fill(0U);
  state.fixed_window_sum = 0U;
  state.fixed_window_count = 0U;
  state.processed_samples_in_cycle = 0U;
  const auto active_buffer_length = dma_transfer_length(runtime);
  const auto sequence_length = dma_sequence_length(runtime);
  const auto window_width = dma_window_width(runtime);
  return dma_buffer(runtime) != nullptr &&
         active_buffer_length <= dma_buffer_size_for(runtime.m_p_config) &&
         sequence_length > 0U && sequence_length <= kAdcDmaMaxSequenceLength &&
         (!uses_fixed_window_average(runtime) || sequence_length == 1U) &&
         window_width > 0U && window_width <= active_buffer_length &&
         window_width <= DMA_SAMPLE_COUNT_MASK;
}

bool configure_adc_channels(opaque_adc& runtime) noexcept {
  const auto sequence_length = dma_sequence_length(runtime);
  if (sequence_length == 1U) {
    auto channel_config = runtime.channel_init();
    channel_config.Rank = ADC_REGULAR_RANK_1;
    return HAL_ADC_ConfigChannel(&runtime.m_handle, &channel_config) == HAL_OK;
  }

  const auto* const channel_sequence = runtime.dma_channel_sequence();
  if (channel_sequence == nullptr) {
    return false;
  }

  for (std::size_t index = 0U; index < sequence_length; ++index) {
    auto channel_config = channel_sequence[index];
    if (HAL_ADC_ConfigChannel(&runtime.m_handle, &channel_config) != HAL_OK) {
      return false;
    }
  }

  return true;
}

bool configure_injected_channels(opaque_adc& runtime) noexcept {
#define RU_STM32H5XX_ADC_CONFIGURE_INJECTED(name, config)                         \
  if (config.uses_injected && config.instance() == runtime.instance()) {           \
    init_pin(config.port(), config.pin_init);                                      \
    auto injected_channel_config = config.injected_channel_init;                   \
    if (HAL_ADCEx_InjectedConfigChannel(                                           \
            &runtime.m_handle, &injected_channel_config) != HAL_OK) {              \
      return false;                                                                \
    }                                                                              \
  }
  RU_STM32H5XX_ADC_MAP(RU_STM32H5XX_ADC_CONFIGURE_INJECTED)
#undef RU_STM32H5XX_ADC_CONFIGURE_INJECTED
  return true;
}

bool init_trigger_timer(opaque_adc& runtime) noexcept {
  enable_tim_clock(runtime.trigger_timer_instance());

  auto& timer_handle = runtime.m_trigger_timer_handle;
  timer_handle = {};
  timer_handle.Instance = runtime.trigger_timer_instance();
  timer_handle.Init = runtime.trigger_timer_init();
  timer_handle.Init.Prescaler =
      timer_prescaler(runtime.trigger_timer_instance(), runtime.trigger_counter_clock_hz());
  timer_handle.Init.Period =
      timer_period(runtime.trigger_counter_clock_hz(), runtime.trigger_frequency_hz());

  if (HAL_TIM_Base_Init(&timer_handle) != HAL_OK) {
    timer_handle.Instance = nullptr;
    return false;
  }

  auto master_config = runtime.trigger_master_config();
  if (HAL_TIMEx_MasterConfigSynchronization(&timer_handle, &master_config) != HAL_OK) {
    (void)HAL_TIM_Base_DeInit(&timer_handle);
    timer_handle.Instance = nullptr;
    return false;
  }

  __HAL_TIM_SET_COUNTER(&timer_handle, 0U);
  return true;
}

bool start_trigger_timer(opaque_adc& runtime) noexcept {
  auto& timer_handle = runtime.m_trigger_timer_handle;
  if (timer_handle.Instance == nullptr) {
    return true;
  }

  return HAL_TIM_Base_Start(&timer_handle) == HAL_OK;
}

bool start_dma_backend_impl(opaque_adc& runtime) noexcept {
  if (!runtime.uses_dma()) {
    return false;
  }

  auto* const active_owner = dma_owner_for_instance(runtime.instance());
  if (active_owner != nullptr && active_owner != &runtime) {
    return false;
  }

  if (dma_initialized(runtime)) {
    return true;
  }

  const auto* const config = runtime.m_p_config;
  runtime = make_opaque(config);
  set_dma_owner(runtime.instance(), &runtime);

  const auto fail = [&runtime, config]() noexcept {
    set_dma_owner(runtime.instance(), nullptr);
    runtime = make_opaque(config);
    return false;
  };

  __HAL_RCC_ADC_CLK_ENABLE();
  __HAL_RCC_GPDMA1_CLK_ENABLE();

  if (!prepare_dma_runtime(runtime)) {
    return fail();
  }

  if (runtime.uses_timer_trigger() && !init_trigger_timer(runtime)) {
    return fail();
  }

  DMA_NodeConfTypeDef node_config{};
  node_config.NodeType = DMA_GPDMA_LINEAR_NODE;
  node_config.Init.Request = runtime.dma_request();
  node_config.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
  node_config.Init.Direction = DMA_PERIPH_TO_MEMORY;
  node_config.Init.SrcInc = DMA_SINC_FIXED;
  node_config.Init.DestInc = DMA_DINC_INCREMENTED;
  node_config.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_HALFWORD;
  node_config.Init.DestDataWidth = DMA_DEST_DATAWIDTH_HALFWORD;
  node_config.Init.SrcBurstLength = 1U;
  node_config.Init.DestBurstLength = 1U;
  node_config.Init.TransferAllocatedPort =
      DMA_SRC_ALLOCATED_PORT0 | DMA_DEST_ALLOCATED_PORT0;
  node_config.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
  node_config.Init.Mode = DMA_NORMAL;
  node_config.TriggerConfig.TriggerPolarity = DMA_TRIG_POLARITY_MASKED;
  node_config.DataHandlingConfig.DataExchange = DMA_EXCHANGE_NONE;
  node_config.DataHandlingConfig.DataAlignment = DMA_DATA_RIGHTALIGN_ZEROPADDED;

  if (HAL_DMAEx_List_BuildNode(&node_config, &runtime.m_dma_node) != HAL_OK ||
      HAL_DMAEx_List_InsertNode(&runtime.m_dma_queue, nullptr, &runtime.m_dma_node) != HAL_OK ||
      HAL_DMAEx_List_SetCircularMode(&runtime.m_dma_queue) != HAL_OK) {
    return fail();
  }

  runtime.m_dma_channel_handle.Instance = runtime.dma_channel();
  runtime.m_dma_channel_handle.InitLinkedList.Priority = DMA_LOW_PRIORITY_LOW_WEIGHT;
  runtime.m_dma_channel_handle.InitLinkedList.LinkStepMode = DMA_LSM_FULL_EXECUTION;
  runtime.m_dma_channel_handle.InitLinkedList.LinkAllocatedPort = DMA_LINK_ALLOCATED_PORT0;
  runtime.m_dma_channel_handle.InitLinkedList.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
  runtime.m_dma_channel_handle.InitLinkedList.LinkedListMode = DMA_LINKEDLIST_CIRCULAR;

  if (HAL_DMAEx_List_Init(&runtime.m_dma_channel_handle) != HAL_OK ||
      HAL_DMAEx_List_LinkQ(&runtime.m_dma_channel_handle, &runtime.m_dma_queue) != HAL_OK ||
      HAL_DMA_ConfigChannelAttributes(&runtime.m_dma_channel_handle, DMA_CHANNEL_NPRIV) !=
          HAL_OK) {
    return fail();
  }

  HAL_NVIC_SetPriority(runtime.dma_irq(), 6U, 0U);
  HAL_NVIC_EnableIRQ(runtime.dma_irq());

  runtime.m_handle.Instance = runtime.instance();
  runtime.m_handle.Init = runtime.adc_init();
  __HAL_LINKDMA(&runtime.m_handle, DMA_Handle, runtime.m_dma_channel_handle);

  if (HAL_ADC_Init(&runtime.m_handle) != HAL_OK) {
    return fail();
  }

  if (!configure_adc_channels(runtime)) {
    return fail();
  }

  auto* const buffer = dma_buffer(runtime);
  std::fill_n(buffer, dma_transfer_length(runtime), 0U);

  if (HAL_ADCEx_Calibration_Start(&runtime.m_handle, ADC_SINGLE_ENDED) != HAL_OK) {
    return fail();
  }

  if (!configure_injected_channels(runtime)) {
    return fail();
  }

  if (HAL_ADC_Start_DMA(&runtime.m_handle,
                        reinterpret_cast<uint32_t*>(buffer),
                        static_cast<uint32_t>(dma_transfer_length(runtime))) != HAL_OK) {
    return fail();
  }

  if (!start_trigger_timer(runtime)) {
    return fail();
  }

  return true;
}

uint32_t lock_irq() noexcept {
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

void unlock_irq(const uint32_t primask) noexcept {
  __set_PRIMASK(primask);
}

expected::expected<uint16_t, result> read_dma_average(const opaque_adc& config) noexcept {
  auto* const runtime = running_dma_owner_for(config);
  if (runtime == nullptr) {
    return expected::unexpected(result::RECOVERABLE_ERROR);
  }

  const auto channel_index = config.dma_sequence_index();
  if (channel_index >= dma_sequence_length(*runtime)) {
    return expected::unexpected(result::RECOVERABLE_ERROR);
  }

  auto& state = dma_state(*runtime);
  const uint32_t primask = lock_irq();
  const uint64_t sum = state.average_sums[channel_index];
  const uint32_t count = state.average_sample_counts[channel_index];
  state.average_sums[channel_index] = 0U;
  state.average_sample_counts[channel_index] = 0U;
  unlock_irq(primask);

  if (count == 0U) {
    return expected::unexpected(result::RECOVERABLE_ERROR);
  }

  return static_cast<uint16_t>(sum / count);
}

expected::expected<uint16_t, result> read_dma_fixed_window_average(
    const opaque_adc& config) noexcept {
  auto* const runtime = running_dma_owner_for(config);
  if (runtime == nullptr || runtime != &config) {
    return expected::unexpected(result::RECOVERABLE_ERROR);
  }

  const auto window_width = dma_window_width(*runtime);
  auto& state = dma_state(*runtime);
  const uint32_t primask = lock_irq();
  const uint64_t sum = state.fixed_window_sum;
  const uint32_t count = state.fixed_window_count;
  state.fixed_window_count = count & DMA_SAMPLE_COUNT_MASK;
  unlock_irq(primask);

  if ((count & DMA_FIXED_WINDOW_READY) == 0U) {
    return expected::unexpected(result::RECOVERABLE_ERROR);
  }

  return static_cast<uint16_t>(sum / window_width);
}

expected::expected<std::optional<uint16_t>, result> try_read_dma_average(
    const opaque_adc& config) noexcept {
  auto* const runtime = running_dma_owner_for(config);
  if (runtime == nullptr) {
    return expected::unexpected(result::RECOVERABLE_ERROR);
  }

  const auto channel_index = config.dma_sequence_index();
  if (channel_index >= dma_sequence_length(*runtime)) {
    return expected::unexpected(result::RECOVERABLE_ERROR);
  }

  auto& state = dma_state(*runtime);
  const uint32_t primask = lock_irq();
  const uint64_t sum = state.average_sums[channel_index];
  const uint32_t count = state.average_sample_counts[channel_index];
  state.average_sums[channel_index] = 0U;
  state.average_sample_counts[channel_index] = 0U;
  unlock_irq(primask);

  if (count == 0U) {
    return std::optional<uint16_t>{};
  }

  return std::optional<uint16_t>{static_cast<uint16_t>(sum / count)};
}

expected::expected<std::optional<uint16_t>, result> try_read_dma_fixed_window_average(
    const opaque_adc& config) noexcept {
  auto* const runtime = running_dma_owner_for(config);
  if (runtime == nullptr || runtime != &config) {
    return expected::unexpected(result::RECOVERABLE_ERROR);
  }

  const auto window_width = dma_window_width(*runtime);
  auto& state = dma_state(*runtime);
  const uint32_t primask = lock_irq();
  const uint64_t sum = state.fixed_window_sum;
  const uint32_t count = state.fixed_window_count;
  state.fixed_window_count = count & DMA_SAMPLE_COUNT_MASK;
  unlock_irq(primask);

  if ((count & DMA_FIXED_WINDOW_READY) == 0U) {
    return std::optional<uint16_t>{};
  }

  return std::optional<uint16_t>{static_cast<uint16_t>(sum / window_width)};
}

expected::expected<uint16_t, result> read_injected_conversion(
    const opaque_adc& config) noexcept {
  auto* const runtime = running_injected_owner_for(config);
  if (runtime == nullptr) {
    return expected::unexpected(result::RECOVERABLE_ERROR);
  }

  if (HAL_ADCEx_InjectedStart(&runtime->m_handle) != HAL_OK) {
    return expected::unexpected(result::RECOVERABLE_ERROR);
  }

  if (HAL_ADCEx_InjectedPollForConversion(
          &runtime->m_handle, config.polling_timeout_ms()) != HAL_OK) {
    (void)HAL_ADCEx_InjectedStop(&runtime->m_handle);
    return expected::unexpected(result::RECOVERABLE_ERROR);
  }

  const auto value = static_cast<uint16_t>(
      HAL_ADCEx_InjectedGetValue(
          &runtime->m_handle, config.injected_channel_init().InjectedRank));
  (void)HAL_ADCEx_InjectedStop(&runtime->m_handle);
  return value;
}

expected::expected<std::optional<uint16_t>, result> try_read_injected_conversion(
    const opaque_adc& config) noexcept {
  auto* const runtime = running_injected_owner_for(config);
  if (runtime == nullptr) {
    return expected::unexpected(result::RECOVERABLE_ERROR);
  }

  if (HAL_ADCEx_InjectedStart(&runtime->m_handle) != HAL_OK) {
    return expected::unexpected(result::RECOVERABLE_ERROR);
  }

  const auto poll_status =
      HAL_ADCEx_InjectedPollForConversion(&runtime->m_handle, 0U);
  if (poll_status == HAL_TIMEOUT) {
    (void)HAL_ADCEx_InjectedStop(&runtime->m_handle);
    return std::optional<uint16_t>{};
  }

  if (poll_status != HAL_OK) {
    (void)HAL_ADCEx_InjectedStop(&runtime->m_handle);
    return expected::unexpected(result::RECOVERABLE_ERROR);
  }

  const auto value = static_cast<uint16_t>(
      HAL_ADCEx_InjectedGetValue(
          &runtime->m_handle, config.injected_channel_init().InjectedRank));
  (void)HAL_ADCEx_InjectedStop(&runtime->m_handle);
  return std::optional<uint16_t>{value};
}
}  // namespace

void adc_dma_half_transfer_callback(ADC_HandleTypeDef* hadc) noexcept {
  auto* const runtime = adc_from_handle(hadc);
  if (runtime == nullptr) {
    return;
  }

  const auto half_buffer_length = dma_transfer_length(*runtime) / 2U;
  if (uses_fixed_window_average(*runtime)) {
    complete_fixed_dma_window(*runtime, half_buffer_length);
  } else {
    accumulate_dma_until(*runtime, half_buffer_length);
  }
}

void adc_dma_full_transfer_callback(ADC_HandleTypeDef* hadc) noexcept {
  auto* const runtime = adc_from_handle(hadc);
  if (runtime == nullptr) {
    return;
  }

  if (uses_fixed_window_average(*runtime)) {
    complete_fixed_dma_window(*runtime, dma_transfer_length(*runtime));
  } else {
    accumulate_dma_until(*runtime, dma_transfer_length(*runtime));
  }
  dma_state(*runtime).processed_samples_in_cycle = 0U;
}

void adc_dma_irq_handler(ADC_TypeDef* const p_instance) noexcept {
  auto* const runtime = dma_owner_for_instance(p_instance);
  if (runtime == nullptr) {
    return;
  }

  HAL_DMA_IRQHandler(&runtime->m_dma_channel_handle);
}

bool opaque_adc::initialized() const noexcept {
  if (uses_dma()) {
    return running_dma_owner_for(*this) != nullptr;
  }

  if (uses_injected()) {
    return running_injected_owner_for(*this) != nullptr;
  }

  return m_handle.Instance != nullptr;
}

std::optional<AdcId> opaque_adc::id() const noexcept {
  for (std::size_t index = 0U; index < static_cast<std::size_t>(AdcId::COUNT); ++index) {
    const auto id = static_cast<AdcId>(index);
    if (config_for(id) == m_p_config) {
      return id;
    }
  }

  return std::nullopt;
}

bool opaque_adc::uses_dma() const noexcept {
  return m_p_config != nullptr && m_p_config->uses_dma;
}

bool opaque_adc::uses_injected() const noexcept {
  return m_p_config != nullptr && m_p_config->uses_injected;
}

ADC_TypeDef* opaque_adc::instance() const noexcept {
  return m_p_config != nullptr ? m_p_config->instance() : nullptr;
}

GPIO_TypeDef* opaque_adc::port() const noexcept {
  return m_p_config != nullptr ? m_p_config->port() : nullptr;
}

const GPIO_InitTypeDef& opaque_adc::pin_init() const noexcept {
  return m_p_config->pin_init;
}

const ADC_InitTypeDef& opaque_adc::adc_init() const noexcept {
  return m_p_config->init;
}

const ADC_ChannelConfTypeDef& opaque_adc::channel_init() const noexcept {
  return m_p_config->channel_init;
}

const ADC_InjectionConfTypeDef& opaque_adc::injected_channel_init() const noexcept {
  return m_p_config->injected_channel_init;
}

uint32_t opaque_adc::polling_timeout_ms() const noexcept {
  return m_p_config != nullptr ? m_p_config->polling_timeout_ms
                               : stm32h5xx::cfg::kDefaultAdcPollTimeoutMs;
}

uint16_t opaque_adc::sample_max() const noexcept {
  return m_p_config != nullptr ? m_p_config->sample_max
                               : stm32h5xx::cfg::kDefaultAdcSampleMax;
}

const ADC_ChannelConfTypeDef* opaque_adc::dma_channel_sequence() const noexcept {
  return m_p_config != nullptr ? m_p_config->dma_channel_sequence : nullptr;
}

std::size_t opaque_adc::dma_frame_count() const noexcept {
  return m_p_config != nullptr ? m_p_config->dma_frame_count : 0U;
}

std::size_t opaque_adc::dma_sequence_length() const noexcept {
  return m_p_config != nullptr ? m_p_config->dma_channel_sequence_length : 0U;
}

std::size_t opaque_adc::dma_sequence_index() const noexcept {
  return m_p_config != nullptr ? m_p_config->dma_sequence_index : 0U;
}

stm32h5xx::cfg::adc_dma_backend opaque_adc::dma_backend() const noexcept {
  return m_p_config != nullptr ? m_p_config->dma_backend
                               : stm32h5xx::cfg::adc_dma_backend::average_since_read;
}

std::size_t opaque_adc::dma_window_width() const noexcept {
  return m_p_config != nullptr ? m_p_config->dma_window_width : 0U;
}

uint32_t opaque_adc::dma_request() const noexcept {
  return m_p_config != nullptr ? m_p_config->dma_request : 0U;
}

DMA_Channel_TypeDef* opaque_adc::dma_channel() const noexcept {
  return m_p_config != nullptr ? m_p_config->dma_channel() : nullptr;
}

IRQn_Type opaque_adc::dma_irq() const noexcept {
  return m_p_config != nullptr ? m_p_config->dma_irq : static_cast<IRQn_Type>(0);
}

bool opaque_adc::uses_timer_trigger() const noexcept {
  return m_p_config != nullptr && m_p_config->uses_timer_trigger();
}

TIM_TypeDef* opaque_adc::trigger_timer_instance() const noexcept {
  return m_p_config != nullptr ? m_p_config->trigger_timer_instance() : nullptr;
}

uint32_t opaque_adc::trigger_counter_clock_hz() const noexcept {
  return m_p_config != nullptr ? m_p_config->trigger_counter_clock_hz : 0U;
}

uint32_t opaque_adc::trigger_frequency_hz() const noexcept {
  return m_p_config != nullptr ? m_p_config->trigger_frequency_hz : 0U;
}

const TIM_Base_InitTypeDef& opaque_adc::trigger_timer_init() const noexcept {
  return m_p_config->trigger_timer_init;
}

const TIM_MasterConfigTypeDef& opaque_adc::trigger_master_config() const noexcept {
  return m_p_config->trigger_master_config;
}

void opaque_adc::start() noexcept {
  __HAL_RCC_ADC_CLK_ENABLE();
}

result opaque_adc::init() noexcept {
  if (!valid()) {
    return result::UNRECOVERABLE_ERROR;
  }

  init_pin(port(), pin_init());

  if (uses_injected()) {
    return initialized() ? result::OK : result::RECOVERABLE_ERROR;
  }

  if (uses_dma()) {
    if (initialized()) {
      return result::OK;
    }

    return start_dma_backend_impl(*this) ? result::OK : result::RECOVERABLE_ERROR;
  }

  if (initialized()) {
    return result::OK;
  }

  m_handle.Instance = instance();
  m_handle.Init = adc_init();

  if (HAL_ADC_Init(&m_handle) != HAL_OK) {
    m_handle.Instance = nullptr;
    return result::RECOVERABLE_ERROR;
  }

  auto channel = channel_init();

  return from_hal_status(HAL_ADC_ConfigChannel(&m_handle, &channel));
}

result opaque_adc::stop() noexcept {
  if (uses_dma()) {
    if (!dma_initialized(*this)) {
      return result::OK;
    }

    result status = result::OK;
    if (m_trigger_timer_handle.Instance != nullptr) {
      if (HAL_TIM_Base_Stop(&m_trigger_timer_handle) != HAL_OK ||
          HAL_TIM_Base_DeInit(&m_trigger_timer_handle) != HAL_OK) {
        status = result::RECOVERABLE_ERROR;
      }
    }

    if (HAL_ADC_Stop_DMA(&m_handle) != HAL_OK || HAL_ADC_DeInit(&m_handle) != HAL_OK) {
      status = result::RECOVERABLE_ERROR;
    }

    set_dma_owner(instance(), nullptr);
    const auto* const config = m_p_config;
    *this = make_opaque(config);
    return status;
  }

  if (uses_injected()) {
    return result::OK;
  }

  (void)HAL_ADC_Stop(&m_handle);
  const auto status = from_hal_status(HAL_ADC_DeInit(&m_handle));
  if (status == result::OK) {
    m_handle.Instance = nullptr;
  }
  return status;
}

result opaque_adc::read(uint16_t& r_value) noexcept {
  if (uses_dma()) {
    const auto dma_value = uses_fixed_window_average(*this)
                               ? read_dma_fixed_window_average(*this)
                               : read_dma_average(*this);
    if (!dma_value.has_value()) {
      return dma_value.error();
    }

    r_value = *dma_value;
    return result::OK;
  }

  if (uses_injected()) {
    const auto injected_value = read_injected_conversion(*this);
    if (!injected_value.has_value()) {
      return injected_value.error();
    }

    r_value = *injected_value;
    return result::OK;
  }

  if (HAL_ADC_Start(&m_handle) != HAL_OK) {
    return result::RECOVERABLE_ERROR;
  }

  if (HAL_ADC_PollForConversion(&m_handle, polling_timeout_ms()) != HAL_OK) {
    (void)HAL_ADC_Stop(&m_handle);
    return result::RECOVERABLE_ERROR;
  }

  r_value = static_cast<uint16_t>(HAL_ADC_GetValue(&m_handle));
  (void)HAL_ADC_Stop(&m_handle);
  return result::OK;
}

result opaque_adc::try_read(bool& r_has_value, uint16_t& r_value) noexcept {
  r_has_value = false;
  if (uses_dma()) {
    const auto dma_value = uses_fixed_window_average(*this)
                               ? try_read_dma_fixed_window_average(*this)
                               : try_read_dma_average(*this);
    if (!dma_value.has_value()) {
      return dma_value.error();
    }

    if (!dma_value->has_value()) {
      return result::OK;
    }

    r_has_value = true;
    r_value = **dma_value;
    return result::OK;
  }

  if (uses_injected()) {
    const auto injected_value = try_read_injected_conversion(*this);
    if (!injected_value.has_value()) {
      return injected_value.error();
    }

    if (!injected_value->has_value()) {
      return result::OK;
    }

    r_has_value = true;
    r_value = **injected_value;
    return result::OK;
  }

  if (HAL_ADC_Start(&m_handle) != HAL_OK) {
    return result::RECOVERABLE_ERROR;
  }

  const auto poll_status = HAL_ADC_PollForConversion(&m_handle, 0U);
  if (poll_status == HAL_TIMEOUT) {
    (void)HAL_ADC_Stop(&m_handle);
    return result::OK;
  }

  if (poll_status != HAL_OK) {
    (void)HAL_ADC_Stop(&m_handle);
    return result::RECOVERABLE_ERROR;
  }

  r_has_value = true;
  r_value = static_cast<uint16_t>(HAL_ADC_GetValue(&m_handle));
  (void)HAL_ADC_Stop(&m_handle);
  return result::OK;
}
}  // namespace ru::driver

extern "C" void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
  ru::driver::adc_dma_half_transfer_callback(hadc);
}

extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  ru::driver::adc_dma_full_transfer_callback(hadc);
}

extern "C" void GPDMA1_Channel0_IRQHandler(void) {
  ru::driver::adc_dma_irq_handler(ADC1);
}

extern "C" void GPDMA1_Channel1_IRQHandler(void) {
  ru::driver::adc_dma_irq_handler(ADC2);
}
