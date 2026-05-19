#include "nv_memory.hpp"

#include <cstddef>
#include <cstdint>

#include "eeprom_emul_core.h"
#include "mapping.hpp"

using namespace ru::driver;

namespace ru::driver {
namespace {
const stm32h5xx::cfg::nv_memory_config* config_for(const NvMemoryId id) noexcept {
  switch (id) {
#define RU_STM32H5XX_NV_MEMORY_CONFIG(name, config) \
    case NvMemoryId::name:                          \
      return &config;
    RU_STM32H5XX_NV_MEMORY_MAP(RU_STM32H5XX_NV_MEMORY_CONFIG)
#undef RU_STM32H5XX_NV_MEMORY_CONFIG
    default:
      return nullptr;
  }
}

opaque_nv_memory make_opaque(const NvMemoryId id) noexcept {
  const auto* const config = config_for(id);
  return config != nullptr ? opaque_nv_memory{config->backend, config->arg0, config->size}
                           : opaque_nv_memory{};
}

constexpr uint16_t k_erased_word{0xFFFFU};

ee_object_t g_eeprom_object{};
bool g_eeprom_initialized{false};

bool range_valid(const uint32_t capacity, const uint32_t address,
                 const size_t len) noexcept {
  return static_cast<uint64_t>(address) + static_cast<uint64_t>(len) <=
         static_cast<uint64_t>(capacity);
}

bool eeprom_status_is_error(const ee_status status) noexcept {
  switch (status) {
    case EE_INVALID_PARAM:
    case EE_INVALID_VIRTUALADDRESS:
    case EE_ERROR_CORRUPTION:
    case EE_ERROR_ALGO:
    case EE_ERROR_ITF_FLASH:
    case EE_ERROR_ITF_CRC:
    case EE_ERROR_ITF_ECC:
      return true;

    default:
      return false;
  }
}

result finish_write_status(const ee_status status) noexcept {
  if (eeprom_status_is_error(status)) {
    return result::RECOVERABLE_ERROR;
  }

  if (status == EE_INFO_CLEANUP_REQUIRED) {
    const auto cleanup_status = EE_CleanUp();
    return eeprom_status_is_error(cleanup_status) ? result::RECOVERABLE_ERROR
                                                   : result::OK;
  }

  return result::OK;
}

result ensure_eeprom_initialized() noexcept {
  if (g_eeprom_initialized) {
    return result::OK;
  }

  const auto status = EE_Init(&g_eeprom_object, EE_CONDITIONAL_ERASE);
  if (eeprom_status_is_error(status)) {
    return result::RECOVERABLE_ERROR;
  }

  g_eeprom_initialized = true;
  return result::OK;
}

uint16_t word_count_for_capacity(const uint32_t capacity) noexcept {
  return static_cast<uint16_t>((capacity + 1U) / 2U);
}

bool config_is_emulated_eeprom(
    const stm32h5xx::cfg::nv_memory_config::backend_kind backend) noexcept {
  return backend == stm32h5xx::cfg::nv_memory_config::backend_kind::EmulatedEeprom;
}

uint16_t base_virtual_address(const uint32_t arg0) noexcept {
  return static_cast<uint16_t>(arg0);
}

bool eeprom_config_valid(const uint16_t base_virtual_address,
                         const uint32_t capacity) noexcept {
  const auto words = word_count_for_capacity(capacity);
  return base_virtual_address != 0U && capacity != 0U && words != 0U &&
         (static_cast<uint32_t>(base_virtual_address) + words - 1U) <=
             static_cast<uint32_t>(EE_NB_OF_VARIABLES);
}

uint16_t virtual_address_for(const uint16_t base_virtual_address,
                             const uint32_t byte_address) noexcept {
  return static_cast<uint16_t>(base_virtual_address + (byte_address / 2U));
}

result eeprom_read_word(const uint16_t virtual_address, uint16_t& word) noexcept {
  const auto init_status = ensure_eeprom_initialized();
  if (init_status != result::OK) {
    return init_status;
  }

  const auto status = EE_ReadVariable16bits(virtual_address, &word);
  if (status == EE_INFO_NODATA) {
    word = k_erased_word;
    return result::OK;
  }

  return status == EE_OK ? result::OK : result::RECOVERABLE_ERROR;
}

result eeprom_write_word(const uint16_t virtual_address, const uint16_t word) noexcept {
  const auto init_status = ensure_eeprom_initialized();
  if (init_status != result::OK) {
    return init_status;
  }

  return finish_write_status(EE_WriteVariable16bits(virtual_address, word));
}
}  // namespace

NvMemory::NvMemory(const NvMemoryId id) noexcept : m_id(id), m_opaque(make_opaque(id)) {
}

result NvMemory::start() noexcept {
  return result::OK;
}

result NvMemory::init() noexcept {
  return m_opaque.init();
}

result NvMemory::stop() noexcept {
  return m_opaque.stop();
}

uint32_t NvMemory::capacity() const noexcept {
  return m_opaque.capacity();
}

result NvMemory::clear() noexcept {
  return m_opaque.clear();
}

result NvMemory::read(const uint32_t address, uint8_t* const p_data,
                      const size_t len) noexcept {
  return m_opaque.read(address, p_data, len);
}

expected::expected<uint8_t, result> NvMemory::read(const uint32_t address) noexcept {
  uint8_t value{0U};
  const auto status = read(address, &value, sizeof(value));
  if (status != result::OK) {
    return expected::unexpected(status);
  }

  return value;
}

result NvMemory::write(const uint32_t address, const uint8_t* const p_data,
                       const size_t len) noexcept {
  return m_opaque.write(address, p_data, len);
}

result NvMemory::write(const uint32_t address, const uint8_t value) noexcept {
  return write(address, &value, sizeof(value));
}

result opaque_nv_memory::init() const noexcept {
  if (m_capacity == 0U) {
    return result::UNRECOVERABLE_ERROR;
  }

  if (config_is_emulated_eeprom(m_backend)) {
    return eeprom_config_valid(base_virtual_address(m_arg0), m_capacity)
               ? ensure_eeprom_initialized()
               : result::UNRECOVERABLE_ERROR;
  }

  return result::UNRECOVERABLE_ERROR;
}

result opaque_nv_memory::stop() const noexcept {
  return result::OK;
}

result opaque_nv_memory::clear() const noexcept {
  if (!config_is_emulated_eeprom(m_backend)) {
    return result::UNRECOVERABLE_ERROR;
  }
  const auto config_base_virtual_address = base_virtual_address(m_arg0);
  if (!eeprom_config_valid(config_base_virtual_address, m_capacity)) {
    return result::UNRECOVERABLE_ERROR;
  }

  const auto words = word_count_for_capacity(m_capacity);
  for (uint16_t word_index = 0U; word_index < words; ++word_index) {
    const auto virtual_address = static_cast<uint16_t>(config_base_virtual_address + word_index);

    uint16_t word{k_erased_word};
    auto status = eeprom_read_word(virtual_address, word);
    if (status != result::OK) {
      return status;
    }

    if (word == k_erased_word) {
      continue;
    }

    status = eeprom_write_word(virtual_address, k_erased_word);
    if (status != result::OK) {
      return status;
    }
  }

  return result::OK;
}

result opaque_nv_memory::read(const uint32_t address, uint8_t* const p_data,
                              const size_t len) const noexcept {
  if (!config_is_emulated_eeprom(m_backend)) {
    return result::UNRECOVERABLE_ERROR;
  }
  const auto config_base_virtual_address = base_virtual_address(m_arg0);
  if (!eeprom_config_valid(config_base_virtual_address, m_capacity) ||
      (len != 0U && p_data == nullptr)) {
    return result::UNRECOVERABLE_ERROR;
  }
  if (!range_valid(m_capacity, address, len)) {
    return result::RECOVERABLE_ERROR;
  }

  for (size_t index = 0U; index < len; ++index) {
    const auto byte_address = static_cast<uint32_t>(address + index);
    const auto virtual_address =
        virtual_address_for(config_base_virtual_address, byte_address);

    uint16_t word{k_erased_word};
    const auto status = eeprom_read_word(virtual_address, word);
    if (status != result::OK) {
      return status;
    }

    p_data[index] = (byte_address & 1U) == 0U
                        ? static_cast<uint8_t>(word & 0x00FFU)
                        : static_cast<uint8_t>((word >> 8U) & 0x00FFU);
  }

  return result::OK;
}

result opaque_nv_memory::write(const uint32_t address, const uint8_t* const p_data,
                               const size_t len) const noexcept {
  if (!config_is_emulated_eeprom(m_backend)) {
    return result::UNRECOVERABLE_ERROR;
  }
  const auto config_base_virtual_address = base_virtual_address(m_arg0);
  if (!eeprom_config_valid(config_base_virtual_address, m_capacity) ||
      (len != 0U && p_data == nullptr)) {
    return result::UNRECOVERABLE_ERROR;
  }
  if (!range_valid(m_capacity, address, len)) {
    return result::RECOVERABLE_ERROR;
  }

  for (size_t index = 0U; index < len; ++index) {
    const auto byte_address = static_cast<uint32_t>(address + index);
    const auto virtual_address =
        virtual_address_for(config_base_virtual_address, byte_address);

    uint16_t word{k_erased_word};
    auto status = eeprom_read_word(virtual_address, word);
    if (status != result::OK) {
      return status;
    }

    const auto next_word = (byte_address & 1U) == 0U
                               ? static_cast<uint16_t>((word & 0xFF00U) | p_data[index])
                               : static_cast<uint16_t>(
                                     (word & 0x00FFU) |
                                     (static_cast<uint16_t>(p_data[index]) << 8U));
    if (next_word == word) {
      continue;
    }

    status = eeprom_write_word(virtual_address, next_word);
    if (status != result::OK) {
      return status;
    }
  }

  return result::OK;
}
}  // namespace ru::driver
