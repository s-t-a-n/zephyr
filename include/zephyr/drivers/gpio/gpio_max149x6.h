/*
 * Copyright (c) 2026 Stan Verschuuren
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Diagnostics API for MAX149xx GPIO drivers
 * @ingroup gpio_max149x6_interface
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_GPIO_GPIO_MAX149X6_H_
#define ZEPHYR_INCLUDE_DRIVERS_GPIO_GPIO_MAX149X6_H_

/**
 * @defgroup gpio_max149x6_interface MAX149xx
 * @ingroup gpio_interface_ext
 * @brief Analog Devices MAX14906/MAX14915/MAX14916 industrial digital output
 * @{
 */

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief MAX14906 diagnostic status.
 */
struct max14906_diagnostics {
	uint8_t overload;
	uint8_t current_limit;
	uint8_t open_wire_off;
	uint8_t above_vdd;
	uint8_t short_to_vdd;
	uint8_t vdd_overvoltage;
	uint8_t safe_demag;
	uint8_t global_err;
	uint8_t interrupt;
};

/**
 * @brief MAX14915/MAX14916 diagnostic status.
 */
struct max14916_diagnostics {
	uint8_t overload;
	uint8_t current_limit;
	uint8_t open_wire_off;
	uint8_t open_wire_on;
	uint8_t short_to_vdd;
	uint8_t global_err;
	uint8_t interrupt;
};

/**
 * @brief Read all diagnostic registers from a MAX14906 device.
 *
 * @param dev - MAX14906 device.
 * @param diag - output struct to populate.
 * @return 0 in case of success, negative error code otherwise.
 */
int max14906_fetch_diagnostics(const struct device *dev, struct max14906_diagnostics *diag);

/**
 * @brief Check whether a MAX14906 has pending faults. Safe to call from ISR context.
 *
 * @param dev - MAX14906 device.
 * @return true if faults are pending, false otherwise.
 */
bool max14906_has_pending_faults(const struct device *dev);

/**
 * @brief Read all diagnostic registers from a MAX14916 device.
 *
 * @param dev - MAX14916 device.
 * @param diag - output struct to populate.
 * @return 0 in case of success, negative error code otherwise.
 */
int max14916_fetch_diagnostics(const struct device *dev, struct max14916_diagnostics *diag);

/**
 * @brief Check whether a MAX14916 has pending faults. Safe to call from ISR context.
 *
 * @param dev - MAX14916 device.
 * @return true if faults are pending, false otherwise.
 */
bool max14916_has_pending_faults(const struct device *dev);

#define max14915_diagnostics       max14916_diagnostics
#define max14915_fetch_diagnostics max14916_fetch_diagnostics
#define max14915_has_pending_faults max14916_has_pending_faults

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_DRIVERS_GPIO_GPIO_MAX149X6_H_ */
