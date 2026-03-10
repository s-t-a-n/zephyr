/*
 * Copyright (c) 2024 Analog Devices Inc.
 * Copyright (c) 2024 Baylibre SAS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>

#define LOG_LEVEL CONFIG_GPIO_LOG_LEVEL
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(gpio_max14916);

#include <zephyr/drivers/gpio/gpio_max149x6.h>
#include <zephyr/drivers/gpio/gpio_utils.h>

#include "gpio_max14916.h"
#include "gpio_max149x6.h"

static int max14916_reg_read(const struct device *dev, uint8_t addr)
{
	struct max14916_data *data = dev->data;
	uint8_t sdo[2];
	int ret = max149x6_reg_transceive(dev, addr, 0, sdo, MAX149x6_READ);

	if (ret >= 0) {
		data->reg_cache[addr] = ret;
		data->cached_sdo_summary |= sdo[0] & 0x3f;
	}

	return ret;
}

static int max14916_reg_write(const struct device *dev, uint8_t addr, uint8_t val)
{
	struct max14916_data *data = dev->data;
	uint8_t sdo[2];
	int ret = max149x6_reg_transceive(dev, addr, val, sdo, MAX149x6_WRITE);

	if (ret == 0) {
		data->reg_cache[addr] = val;
		data->cached_sdo_summary |= sdo[0] & 0x3f;
	}

	return ret;
}
static int gpio_max14916_port_set_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	struct max14916_data *data = dev->data;
	int ret;

	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	uint8_t reg_val = data->reg_cache[MAX14916_SETOUT_REG] | pins;

	ret = max14916_reg_write(dev, MAX14916_SETOUT_REG, reg_val);

	k_mutex_unlock(&data->lock);
	return ret;
}

static int gpio_max14916_port_clear_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	struct max14916_data *data = dev->data;
	int ret;

	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	uint8_t reg_val = data->reg_cache[MAX14916_SETOUT_REG] & ~pins;

	ret = max14916_reg_write(dev, MAX14916_SETOUT_REG, reg_val);

	k_mutex_unlock(&data->lock);
	return ret;
}

static int gpio_max14916_port_set_masked_raw(const struct device *dev,
					     gpio_port_pins_t mask,
					     gpio_port_value_t value)
{
	struct max14916_data *data = dev->data;
	int ret;

	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	uint8_t reg_val = (data->reg_cache[MAX14916_SETOUT_REG] & ~mask) |
			  (value & mask);

	ret = max14916_reg_write(dev, MAX14916_SETOUT_REG, reg_val);

	k_mutex_unlock(&data->lock);
	return ret;
}

static int gpio_max14916_config(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
	struct max14916_data *data = dev->data;
	int err = 0;

	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	if ((flags & (GPIO_INPUT | GPIO_OUTPUT)) == GPIO_DISCONNECTED) {
		return -ENOTSUP;
	}

	if ((flags & GPIO_SINGLE_ENDED) != 0) {
		return -ENOTSUP;
	}

	if ((flags & (GPIO_PULL_UP | GPIO_PULL_DOWN)) != 0) {
		return -ENOTSUP;
	}

	if (flags & GPIO_INT_ENABLE) {
		return -ENOTSUP;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	switch (flags & GPIO_DIR_MASK) {
	case GPIO_OUTPUT:
		if (flags & GPIO_OUTPUT_INIT_HIGH) {
			err = gpio_max14916_port_set_bits_raw(dev, BIT(pin));
		} else if (flags & GPIO_OUTPUT_INIT_LOW) {
			err = gpio_max14916_port_clear_bits_raw(dev, BIT(pin));
		}
		break;
	case GPIO_INPUT:
	default:
		LOG_ERR("NOT SUPPORTED OPTION!");
		err = -ENOTSUP;
		break;
	}

	k_mutex_unlock(&data->lock);
	return err;
}

static int gpio_max14916_port_get_raw(const struct device *dev, gpio_port_value_t *value)
{
	struct max14916_data *data = dev->data;
	int ret;

	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	ret = max14916_reg_read(dev, MAX14916_SETOUT_REG);
	if (ret < 0) {
		goto out;
	}

	*value = ret;
	ret = 0;

out:
	k_mutex_unlock(&data->lock);
	return ret;
}

static int gpio_max14916_port_toggle_bits(const struct device *dev, gpio_port_pins_t pins)
{
	struct max14916_data *data = dev->data;
	int ret;

	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	uint8_t reg_val = data->reg_cache[MAX14916_SETOUT_REG] ^ pins;

	ret = max14916_reg_write(dev, MAX14916_SETOUT_REG, reg_val);

	k_mutex_unlock(&data->lock);
	return ret;
}

int max14916_fetch_diagnostics(const struct device *dev, struct max14916_diagnostics *diag)
{
	struct max14916_data *data = dev->data;
	uint8_t regs[1 + MAX149x6_BURST_READ_NREGS];
	int ret;

	if (diag == NULL) {
		return -EINVAL;
	}

	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	*diag = (struct max14916_diagnostics){0};

	k_mutex_lock(&data->lock, K_FOREVER);

	/* burst starts at 0x04; reading Interrupt separately is required to de-assert FAULT (FLatchEn=1) */
	ret = max14916_reg_read(dev, MAX14916_INT_REG);
	if (ret < 0) {
		goto out;
	}
	diag->interrupt = ret;

	ret = max149x6_burst_read(dev, MAX14916_OVR_LD_REG, regs);
	if (ret < 0) {
		goto out;
	}

	diag->overload      = regs[1];
	diag->current_limit = regs[2];
	diag->open_wire_off = regs[3];
	diag->open_wire_on  = regs[4];
	diag->short_to_vdd  = regs[5];
	diag->global_err    = regs[6];

	data->cached_sdo_summary = 0;

out:
	k_mutex_unlock(&data->lock);
	return ret;
}

bool max14916_has_pending_faults(const struct device *dev)
{
	const struct max14916_data *data = dev->data;

	return data->cached_sdo_summary != 0;
}

static int gpio_max14916_clean_on_power(const struct device *dev)
{
	struct max14916_diagnostics diag;

	/* Clear the latched faults generated at power up */
	return max14916_fetch_diagnostics(dev, &diag);
}

static int gpio_max14916_init_registers(const struct device *dev)
{
	const struct max14916_config *config = dev->config;
	int ret;

	/* Configure global registers */
	ret = max14916_reg_write(dev, MAX14916_CONFIG1_REG, config->config1.reg_raw);
	if (ret < 0) {
		return ret;
	}

	ret = max14916_reg_write(dev, MAX14916_CONFIG2_REG, config->config2.reg_raw);
	if (ret < 0) {
		return ret;
	}

	ret = max14916_reg_write(dev, MAX14916_FAULT_MASK_REG, config->fault_mask.reg_raw);
	if (ret < 0) {
		return ret;
	}

	/* Configure per-channel registers */
	ret = max14916_reg_write(dev, MAX14916_OW_ON_EN_REG, config->ow_on_en.reg_raw);
	if (ret < 0) {
		return ret;
	}

	ret = max14916_reg_write(dev, MAX14916_OW_OFF_EN_REG, config->ow_off_en.reg_raw);
	if (ret < 0) {
		return ret;
	}

	ret = max14916_reg_write(dev, MAX14916_SHT_VDD_EN_REG, config->sht_vdd_en.reg_raw);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int gpio_max14916_init(const struct device *dev)
{
	const struct max14916_config *config = dev->config;
	struct max14916_data *data = dev->data;
	int ret = 0;

	LOG_DBG(" --- GPIO MAX14916 init IN ---");

	if (!spi_is_ready_dt(&config->spi)) {
		LOG_ERR("SPI bus is not ready\n");
		return -ENODEV;
	}

	ret = k_mutex_init(&data->lock);
	if (ret != 0) {
		LOG_ERR("unable to initialize mutex");
		return ret;
	}

	/* setup READY gpio - normal low */
	if (!gpio_is_ready_dt(&config->ready_gpio)) {
		LOG_ERR("READY GPIO device not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&config->ready_gpio, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Failed to configure reset GPIO");
		return ret;
	}

	/* setup FLT gpio - normal high */
	if (!gpio_is_ready_dt(&config->fault_gpio)) {
		LOG_ERR("FLT GPIO device not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&config->fault_gpio, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Failed to configure DC GPIO");
		return ret;
	}

	/* setup LATCH gpio - normal high */
	if (!gpio_is_ready_dt(&config->sync_gpio)) {
		LOG_ERR("SYNC GPIO device not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&config->sync_gpio, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure busy GPIO");
		return ret;
	}

	/* setup LATCH gpio - normal high */
	if (!gpio_is_ready_dt(&config->en_gpio)) {
		LOG_ERR("SYNC GPIO device not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&config->en_gpio, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure busy GPIO");
		return ret;
	}

	gpio_pin_set_dt(&config->en_gpio, 1);
	gpio_pin_set_dt(&config->sync_gpio, 1);

	LOG_ERR("[GPIO] FALUT - %d\n", gpio_pin_get_dt(&config->fault_gpio));
	LOG_ERR("[GPIO] READY - %d\n", gpio_pin_get_dt(&config->ready_gpio));
	LOG_ERR("[GPIO] SYNC  - %d\n", gpio_pin_get_dt(&config->sync_gpio));
	LOG_ERR("[GPIO] EN    - %d\n", gpio_pin_get_dt(&config->en_gpio));

	ret = gpio_max14916_clean_on_power(dev);
	if (ret < 0) {
		return ret;
	}

	ret = max14916_reg_write(dev, MAX14916_SETOUT_REG, 0);
	if (ret < 0) {
		return ret;
	}

	ret = gpio_max14916_init_registers(dev);
	if (ret < 0) {
		return ret;
	}

	LOG_DBG(" --- GPIO MAX14916 init OUT ---");

	return ret;
}

static DEVICE_API(gpio, gpio_max14916_api) = {
	.pin_configure = gpio_max14916_config,
	.port_get_raw = gpio_max14916_port_get_raw,
	.port_set_masked_raw = gpio_max14916_port_set_masked_raw,
	.port_set_bits_raw = gpio_max14916_port_set_bits_raw,
	.port_clear_bits_raw = gpio_max14916_port_clear_bits_raw,
	.port_toggle_bits = gpio_max14916_port_toggle_bits,
};

#define GPIO_MAX14916_DEVICE(id, model)                                                            \
	static const struct max14916_config max##model##_##id##_cfg = {                            \
		.common = GPIO_COMMON_CONFIG_FROM_DT_INST(id),                                     \
		.spi = SPI_DT_SPEC_INST_GET(id, SPI_OP_MODE_MASTER | SPI_WORD_SET(8U)),            \
		.ready_gpio = GPIO_DT_SPEC_INST_GET(id, drdy_gpios),                               \
		.fault_gpio = GPIO_DT_SPEC_INST_GET(id, fault_gpios),                              \
		.sync_gpio = GPIO_DT_SPEC_INST_GET(id, sync_gpios),                                \
		.en_gpio = GPIO_DT_SPEC_INST_GET(id, en_gpios),                                    \
		.crc_en = DT_INST_PROP(id, crc_en),                                                \
		.config1.reg_bits.FLED_SET = DT_INST_PROP(id, fled_set),                           \
		.config1.reg_bits.SLED_SET = DT_INST_PROP(id, sled_set),                           \
		.config1.reg_bits.FLED_STRETCH = DT_INST_PROP(id, fled_stretch),                   \
		.config1.reg_bits.FFILTER_EN = DT_INST_PROP(id, ffilter_en),                       \
		.config1.reg_bits.FILTER_LONG = DT_INST_PROP(id, filter_long),                     \
		.config1.reg_bits.FLATCH_EN = DT_INST_PROP(id, flatch_en),                         \
		.config1.reg_bits.LED_CURR_LIM = DT_INST_PROP(id, led_cur_lim),                    \
		.config2.reg_bits.VDD_ON_THR = DT_INST_PROP(id, vdd_on_thr),                       \
		.config2.reg_bits.SYNCH_WD_EN = DT_INST_PROP(id, synch_wd_en),                     \
		.config2.reg_bits.SHT_VDD_THR = DT_INST_PROP(id, sht_vdd_thr),                     \
		.config2.reg_bits.OW_OFF_CS = DT_INST_PROP(id, ow_off_cs),                         \
		.config2.reg_bits.WD_TO = DT_INST_PROP(id, wd_to),                                 \
		.pkt_size = (DT_INST_PROP(id, crc_en) & 0x1) ? 3 : 2,                              \
		.ow_on_en.reg_bits =                                                               \
			{                                                                          \
				.OW_ON_EN1 = DT_INST_PROP_BY_IDX(id, ow_on_en, 0),                 \
				.OW_ON_EN2 = DT_INST_PROP_BY_IDX(id, ow_on_en, 1),                 \
				.OW_ON_EN3 = DT_INST_PROP_BY_IDX(id, ow_on_en, 2),                 \
				.OW_ON_EN4 = DT_INST_PROP_BY_IDX(id, ow_on_en, 3),                 \
				.OW_ON_EN5 = DT_INST_PROP_BY_IDX(id, ow_on_en, 4),                 \
				.OW_ON_EN6 = DT_INST_PROP_BY_IDX(id, ow_on_en, 5),                 \
				.OW_ON_EN7 = DT_INST_PROP_BY_IDX(id, ow_on_en, 6),                 \
				.OW_ON_EN8 = DT_INST_PROP_BY_IDX(id, ow_on_en, 7),                 \
			},                                                                         \
		.ow_off_en.reg_bits =                                                              \
			{                                                                          \
				.OW_OFF_EN1 = DT_INST_PROP_BY_IDX(id, ow_off_en, 0),               \
				.OW_OFF_EN2 = DT_INST_PROP_BY_IDX(id, ow_off_en, 1),               \
				.OW_OFF_EN3 = DT_INST_PROP_BY_IDX(id, ow_off_en, 2),               \
				.OW_OFF_EN4 = DT_INST_PROP_BY_IDX(id, ow_off_en, 3),               \
				.OW_OFF_EN5 = DT_INST_PROP_BY_IDX(id, ow_off_en, 4),               \
				.OW_OFF_EN6 = DT_INST_PROP_BY_IDX(id, ow_off_en, 5),               \
				.OW_OFF_EN7 = DT_INST_PROP_BY_IDX(id, ow_off_en, 6),               \
				.OW_OFF_EN8 = DT_INST_PROP_BY_IDX(id, ow_off_en, 7),               \
			},                                                                         \
		.sht_vdd_en.reg_bits =                                                             \
			{                                                                          \
				.SH_VDD_EN1 = DT_INST_PROP_BY_IDX(id, sh_vdd_en, 0),               \
				.SH_VDD_EN2 = DT_INST_PROP_BY_IDX(id, sh_vdd_en, 1),               \
				.SH_VDD_EN3 = DT_INST_PROP_BY_IDX(id, sh_vdd_en, 2),               \
				.SH_VDD_EN4 = DT_INST_PROP_BY_IDX(id, sh_vdd_en, 3),               \
				.SH_VDD_EN5 = DT_INST_PROP_BY_IDX(id, sh_vdd_en, 4),               \
				.SH_VDD_EN6 = DT_INST_PROP_BY_IDX(id, sh_vdd_en, 5),               \
				.SH_VDD_EN7 = DT_INST_PROP_BY_IDX(id, sh_vdd_en, 6),               \
				.SH_VDD_EN8 = DT_INST_PROP_BY_IDX(id, sh_vdd_en, 7),               \
			},                                                                         \
		.fault_mask.reg_raw = DT_INST_PROP(id, fault_mask),                                \
		.spi_addr = DT_INST_PROP(id, spi_addr),                                            \
	};                                                                                         \
                                                                                                   \
	static struct max14916_data max##model##_##id##_data = {                                   \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(id, &gpio_max14916_init, NULL, &max##model##_##id##_data,            \
			      &max##model##_##id##_cfg, POST_KERNEL,                               \
			      CONFIG_GPIO_MAX14916_INIT_PRIORITY, &gpio_max14916_api);

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT adi_max14915_gpio
DT_INST_FOREACH_STATUS_OKAY_VARGS(GPIO_MAX14916_DEVICE, 14915)

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT adi_max14916_gpio
DT_INST_FOREACH_STATUS_OKAY_VARGS(GPIO_MAX14916_DEVICE, 14916)
