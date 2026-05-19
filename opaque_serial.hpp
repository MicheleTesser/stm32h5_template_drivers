#pragma once

#include <cstddef>
#include <cstdint>

#include "common.hpp"

namespace ru::driver {
class Serial;

struct opaque_serial {
 public:
  constexpr opaque_serial() noexcept = default;

 private:
  friend class Serial;

  result init() noexcept;
  result stop() noexcept;
  result write(const uint8_t* p_data, size_t len, Timestamp timeout_uS) noexcept;
  result read(uint8_t* p_data, size_t len, Timestamp timeout_uS) noexcept;
};
}  // namespace ru::driver
