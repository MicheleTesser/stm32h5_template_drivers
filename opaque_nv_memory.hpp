#pragma once

#include <cstddef>
#include <cstdint>

#include "common.hpp"
#include "mapping_types.hpp"

namespace ru::driver {
class NvMemory;

struct opaque_nv_memory {
 public:
  constexpr opaque_nv_memory() noexcept = default;
  constexpr opaque_nv_memory(const stm32h5xx::cfg::nv_memory_config::backend_kind backend,
                             const uint32_t arg0, const uint32_t capacity) noexcept
      : m_backend(backend), m_arg0(arg0), m_capacity(capacity) {}

 private:
  friend class NvMemory;

  result init() const noexcept;
  result stop() const noexcept;
  uint32_t capacity() const noexcept { return m_capacity; }
  result clear() const noexcept;
  result read(uint32_t address, uint8_t* p_data, size_t len) const noexcept;
  result write(uint32_t address, const uint8_t* p_data, size_t len) const noexcept;

  stm32h5xx::cfg::nv_memory_config::backend_kind m_backend{
      stm32h5xx::cfg::nv_memory_config::backend_kind::None};
  uint32_t m_arg0{0U};
  uint32_t m_capacity{0U};
};
}  // namespace ru::driver
