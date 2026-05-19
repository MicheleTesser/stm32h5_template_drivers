#include "serial.hpp"
#include "usb/usb_cdc.h"

using namespace ru::driver;

namespace ru::driver {
Serial::Serial(const SerialId id) noexcept : m_id(id), m_opaque() {
}

result Serial::start() noexcept {
  return ru_stm32_usb_cdc_start() == 0 ? result::OK : result::RECOVERABLE_ERROR;
}

result Serial::init() noexcept {
  return m_opaque.init();
}

result Serial::stop() noexcept {
  return m_opaque.stop();
}

result Serial::write(const uint8_t* const p_data, const size_t len,
                     const Timestamp timeout_uS) noexcept {
  if (p_data == nullptr && len != 0U) {
    return result::UNRECOVERABLE_ERROR;
  }

  return m_opaque.write(p_data, len, timeout_uS);
}

result Serial::read(uint8_t* const p_data, const size_t len,
                    const Timestamp timeout_uS) noexcept {
  if (p_data == nullptr && len != 0U) {
    return result::UNRECOVERABLE_ERROR;
  }

  if (p_data == nullptr) {
    return result::OK;
  }

  return m_opaque.read(p_data, len, timeout_uS);
}

result opaque_serial::init() noexcept {
  return ru_stm32_usb_cdc_init() == 0 ? result::OK : result::RECOVERABLE_ERROR;
}

result opaque_serial::stop() noexcept {
  return ru_stm32_usb_cdc_stop() == 0 ? result::OK : result::RECOVERABLE_ERROR;
}

result opaque_serial::write(const uint8_t* const p_data, const size_t len,
                            const Timestamp timeout_uS) noexcept {
  return ru_stm32_usb_cdc_write(p_data, len, timeout_uS) == 0
             ? result::OK
             : result::RECOVERABLE_ERROR;
}

result opaque_serial::read(uint8_t* const p_data, const size_t len,
                           const Timestamp timeout_uS) noexcept {
  (void)p_data;
  (void)timeout_uS;

  return len == 0U ? result::OK : result::RECOVERABLE_ERROR;
}
}  // namespace ru::driver
