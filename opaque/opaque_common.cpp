#include "opaque_common.hpp"

#include "stm_common.hpp"

using namespace ru::driver;

namespace ru::driver {
static result configure_system_clock() noexcept;

result opaque_common::start() const noexcept {
  const auto hal_status = HAL_Init();
  if (hal_status != HAL_OK) {
    return from_hal_status(hal_status);
  }

  return configure_system_clock();
}

/*
* HSE     = 12 MHz
* SYSCLK  = 250 MHz
* HCLK    = 250 MHz
* PCLK1   = 250 MHz
* PCLK2   = 250 MHz
* PCLK3   = 250 MHz
*
* FDCAN   = 40 MHz
* ADCDAC  = 60 MHz
*
* HSI48   = 48 MHz (USB sync)
*/
static result configure_system_clock() noexcept {
  RCC_OscInitTypeDef osc_init{};
  RCC_ClkInitTypeDef clk_init{};
  RCC_PeriphCLKInitTypeDef periph_clk_init{};
  RCC_CRSInitTypeDef crs_init{};

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
  }

  osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI48;

  osc_init.HSEState = RCC_HSE_ON;
  osc_init.HSI48State = RCC_HSI48_ON;
  osc_init.PLL.PLLState = RCC_PLL_ON;
  osc_init.PLL.PLLSource = RCC_PLL1_SOURCE_HSE;
  osc_init.PLL.PLLM = 3;
  osc_init.PLL.PLLN = 125;

  osc_init.PLL.PLLP = 2;
  osc_init.PLL.PLLQ = 2;
  osc_init.PLL.PLLR = 2;

  osc_init.PLL.PLLRGE = RCC_PLL1_VCIRANGE_2;
  osc_init.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
  osc_init.PLL.PLLFRACN = 0;

  if (HAL_RCC_OscConfig(&osc_init) != HAL_OK) {
    return result::RECOVERABLE_ERROR;
  }

  clk_init.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 |
                       RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_PCLK3;
  clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk_init.APB1CLKDivider = RCC_HCLK_DIV1;
  clk_init.APB2CLKDivider = RCC_HCLK_DIV1;
  clk_init.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&clk_init, FLASH_LATENCY_5) != HAL_OK) {
    return result::RECOVERABLE_ERROR;
  }

  periph_clk_init.PeriphClockSelection =
      RCC_PERIPHCLK_ADCDAC | RCC_PERIPHCLK_FDCAN | RCC_PERIPHCLK_LPTIM2;
  periph_clk_init.PLL2.PLL2Source = RCC_PLL2_SOURCE_HSE;
  periph_clk_init.PLL2.PLL2M = 2U;
  periph_clk_init.PLL2.PLL2N = 60U;
  periph_clk_init.PLL2.PLL2P = 2U;
  periph_clk_init.PLL2.PLL2Q = 9U;
  periph_clk_init.PLL2.PLL2R = 6U;
  periph_clk_init.PLL2.PLL2RGE = RCC_PLL2_VCIRANGE_2;
  periph_clk_init.PLL2.PLL2VCOSEL = RCC_PLL2_VCORANGE_WIDE;
  periph_clk_init.PLL2.PLL2FRACN = 0U;
  periph_clk_init.PLL2.PLL2ClockOut = RCC_PLL2_DIVQ | RCC_PLL2_DIVR;
  periph_clk_init.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL2Q;
  periph_clk_init.AdcDacClockSelection = RCC_ADCDACCLKSOURCE_PLL2R;
  periph_clk_init.Lptim2ClockSelection = RCC_LPTIM2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&periph_clk_init) != HAL_OK) {
    return result::RECOVERABLE_ERROR;
  }

  __HAL_RCC_CRS_CLK_ENABLE();

  crs_init.Prescaler = RCC_CRS_SYNC_DIV1;
  crs_init.Source = RCC_CRS_SYNC_SOURCE_USB;
  crs_init.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  crs_init.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000U, 1000U);
  crs_init.ErrorLimitValue = 34U;
  crs_init.HSI48CalibrationValue = 32U;
  HAL_RCCEx_CRSConfig(&crs_init);

  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_2);
  return result::OK;
}

}  // namespace ru::driver
