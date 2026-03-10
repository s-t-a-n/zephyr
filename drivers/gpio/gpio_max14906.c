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

LOG_MODULE_REGISTER(gpio_max14906);

#include <zephyr/drivers/gpio/gpio_max149x6.h>
#include <zephyr/drivers/gpio/gpio_utils.h>

#include "gpio_max14906.h"
#include "gpio_max149x6.h"

#define DT_DRV_COMPAT adi_max14906_gpio

static int max14906_reg_read(const struct device *dev, uint8_t addr)
{
	struct max14906_data *data = dev->data;
	uint8_t sdo[2];
	int ret = max149x6_reg_transceive(dev, addr, 0, sdo, MAX149x6_READ);

	if (ret >= 0) {
		data->reg_cache[addr] = ret;
		data->cached_sdo_summary |= sdo[0] & 0x3f;
	}

	return ret;
}

static int max14906_reg_write(const struct device *dev, uint8_t addr, uint8_t val)
{
	struct max14906_data *data = dev->data;
	uint8_t sdo[2];
	int ret = max149x6_reg_transceive(dev, addr, val, sdo, MAX149x6_WRITE);

	if (ret == 0) {
		data->reg_cache[addr] = val;
		data->cached_sdo_summary |= sdo[0] & 0x3f;
	}

	return ret;
}
/**
 * @brief Configure a channel's function.
 * @param desc - device descriptor for the MAX14906
 * @param ch - channel index (0 based).
 * @param function - channel configuration (input, output or high-z).
 * @return 0 in case of success, negative error code otherwise
 */
static int max14906_ch_func(const struct device *dev, uint32_t ch, enum max14906_function function)
{
	const struct max14906_config *config = dev->config;
	struct max14906_data *data = dev->data;
	uint8_t config_do = data->reg_cache[MAX14906_CONFIG_DO_REG];
	uint8_t setout = data->reg_cache[MAX14906_SETOUT_REG];
	int ret;

	switch (function) {
	case MAX14906_HIGH_Z:
		config_do = (config_do & ~MAX14906_DO_MASK(ch)) |
			    FIELD_PREP(MAX14906_DO_MASK(ch), MAX14906_PUSH_PULL);
		setout |= MAX14906_CH_DIR_MASK(ch);
		break;
	case MAX14906_IN:
		config_do = (config_do & ~MAX14906_DO_MASK(ch)) |
			    FIELD_PREP(MAX14906_DO_MASK(ch), MAX14906_HIGH_SIDE);
		setout |= MAX14906_CH_DIR_MASK(ch);
		break;
	case MAX14906_OUT:
		config_do = (config_do & ~MAX14906_DO_MASK(ch)) |
			    FIELD_PREP(MAX14906_DO_MASK(ch),
				       FIELD_GET(MAX14906_DO_MASK(ch), config->config_do.reg_raw));
		setout &= ~MAX14906_CH_DIR_MASK(ch);
		break;
	default:
		return -EINVAL;
	}

	ret = max14906_reg_write(dev, MAX14906_CONFIG_DO_REG, config_do);
	if (ret < 0) {
		return ret;
	}

	return max14906_reg_write(dev, MAX14906_SETOUT_REG, setout);
}

static int gpio_max14906_port_set_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	struct max14906_data *data = dev->data;
	int ret;

	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	uint8_t output_pins = MAX149x6_LOWER_NIBBLE(pins);
	uint8_t reg_val = data->reg_cache[MAX14906_SETOUT_REG] | output_pins;

	ret = max14906_reg_write(dev, MAX14906_SETOUT_REG, reg_val);

	k_mutex_unlock(&data->lock);
	return ret;
}

static int gpio_max14906_port_clear_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	struct max14906_data *data = dev->data;
	int ret;

	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	uint8_t output_pins = MAX149x6_LOWER_NIBBLE(pins);
	uint8_t reg_val = data->reg_cache[MAX14906_SETOUT_REG] & ~output_pins;

	ret = max14906_reg_write(dev, MAX14906_SETOUT_REG, reg_val);

	k_mutex_unlock(&data->lock);
	return ret;
}

static int gpio_max14906_port_set_masked_raw(const struct device *dev,
					     gpio_port_pins_t mask,
					     gpio_port_value_t value)
{
	struct max14906_data *data = dev->data;
	int ret;

	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	uint8_t output_mask = MAX149x6_LOWER_NIBBLE(mask);
	uint8_t reg_val = (data->reg_cache[MAX14906_SETOUT_REG] & ~output_mask) |
			  (value & output_mask);

	ret = max14906_reg_write(dev, MAX14906_SETOUT_REG, reg_val);

	k_mutex_unlock(&data->lock);
	return ret;
}

static int gpio_max14906_config(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
	struct max14906_data *data = dev->data;
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
	case GPIO_INPUT:
		err = max14906_ch_func(dev, (uint32_t)pin, MAX14906_IN);
		if (err < 0) {
			break;
		}
		LOG_DBG("SETUP AS INPUT %d", pin);
		break;
	case GPIO_OUTPUT:
		if (flags & GPIO_OUTPUT_INIT_HIGH) {
			err = gpio_max14906_port_set_bits_raw(dev, BIT(pin));
		} else if (flags & GPIO_OUTPUT_INIT_LOW) {
			err = gpio_max14906_port_clear_bits_raw(dev, BIT(pin));
		}
		if (err < 0) {
			break;
		}
		err = max14906_ch_func(dev, (uint32_t)pin, MAX14906_OUT);
		if (err < 0) {
			break;
		}
		LOG_DBG("SETUP AS OUTPUT %d", pin);
		break;
	default:
		LOG_ERR("On MAX14906 only input option is available!");
		err = -ENOTSUP;
		break;
	}

	k_mutex_unlock(&data->lock);
	return err;
}

static int gpio_max14906_port_get_raw(const struct device *dev, gpio_port_value_t *value)
{
	/* We care only for first 4 bits of the reg.
	 * Next set of bits is for direction.
	 * NOTE : in case and only if pin is INPUT DOILEVEL reg bits 0-4 shows PIN state.
	 * In case PIN is OUTPUT same bits show VDDOKFault state.
	 */

	struct max14906_data *data = dev->data;
	int ret;

	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	ret = max14906_reg_read(dev, MAX14906_DOILEVEL_REG);
	if (ret < 0) {
		goto out;
	}

	data->cached_safe_demag_reg |= MAX149x6_UPPER_NIBBLE(ret);
	*value = MAX149x6_LOWER_NIBBLE(ret);
	ret = 0;

out:
	k_mutex_unlock(&data->lock);
	return ret;
}

static int gpio_max14906_port_toggle_bits(const struct device *dev, gpio_port_pins_t pins)
{
	struct max14906_data *data = dev->data;
	int ret;

	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	uint8_t output_pins = MAX149x6_LOWER_NIBBLE(pins);
	uint8_t reg_val = data->reg_cache[MAX14906_SETOUT_REG] ^ output_pins;

	ret = max14906_reg_write(dev, MAX14906_SETOUT_REG, reg_val);

	k_mutex_unlock(&data->lock);
	return ret;
}

int max14906_fetch_diagnostics(const struct device *dev, struct max14906_diagnostics *diag)
{
	struct max14906_data *data = dev->data;
	uint8_t regs[1 + MAX149x6_BURST_READ_NREGS];
	int ret;

	if (diag == NULL) {
		return -EINVAL;
	}

	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	*diag = (struct max14906_diagnostics){0};

	k_mutex_lock(&data->lock, K_FOREVER);

	ret = max149x6_burst_read(dev, MAX14906_DOILEVEL_REG, regs);
	if (ret < 0) {
		goto out;
	}

	data->cached_safe_demag_reg |= MAX149x6_UPPER_NIBBLE(regs[1]);
	diag->safe_demag = data->cached_safe_demag_reg;
	diag->interrupt = regs[2];
	diag->overload = MAX149x6_LOWER_NIBBLE(regs[3]);
	diag->current_limit = MAX149x6_UPPER_NIBBLE(regs[3]);
	diag->open_wire_off = MAX149x6_LOWER_NIBBLE(regs[4]);
	diag->above_vdd = MAX149x6_UPPER_NIBBLE(regs[4]);
	diag->short_to_vdd = MAX149x6_LOWER_NIBBLE(regs[5]);
	diag->vdd_overvoltage = MAX149x6_UPPER_NIBBLE(regs[5]);
	diag->global_err = regs[6];

	data->cached_safe_demag_reg = 0;
	data->cached_sdo_summary = 0;

out:
	k_mutex_unlock(&data->lock);
	return ret;
}

bool max14906_has_pending_faults(const struct device *dev)
{
	const struct max14906_data *data = dev->data;

	return data->cached_sdo_summary != 0;
}

static int gpio_max14906_clean_on_power(const struct device *dev)
{
	struct max14906_diagnostics diag;

	/* Clear the latched faults generated at power up */
	return max14906_fetch_diagnostics(dev, &diag);
}

static int gpio_max14906_init_registers(const struct device *dev)
{
	const struct max14906_config *config = dev->config;
	int ret;

	/* Configure global registers */
	ret = max14906_reg_write(dev, MAX14906_CONFIG1_REG, config->config1.reg_raw);
	if (ret < 0) {
		return ret;
	}

	ret = max14906_reg_write(dev, MAX14906_CONFIG2_REG, config->config2.reg_raw);
	if (ret < 0) {
		return ret;
	}

	ret = max14906_reg_write(dev, MAX14906_CONFIG_DI_REG, config->config_di.reg_raw);
	if (ret < 0) {
		return ret;
	}

	ret = max14906_reg_write(dev, MAX14906_FAULT_MASK_REG, config->fault_mask.reg_raw);
	if (ret < 0) {
		return ret;
	}

	/* Configure per-channel registers */
	ret = max14906_reg_write(dev, MAX14906_CONFIG_DO_REG, config->config_do.reg_raw);
	if (ret < 0) {
		return ret;
	}

	ret = max14906_reg_write(dev, MAX14906_CURR_LIM_REG, config->curr_lim.reg_raw);
	if (ret < 0) {
		return ret;
	}

	ret = max14906_reg_write(dev, MAX14906_OPN_WR_EN_REG, config->opn_wr_en.reg_raw);
	if (ret < 0) {
		return ret;
	}

	ret = max14906_reg_write(dev, MAX14906_SHT_VDD_EN_REG, config->sht_vdd_en.reg_raw);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int gpio_max14906_init(const struct device *dev)
{
	const struct max14906_config *config = dev->config;
	struct max14906_data *data = dev->data;
	int ret = 0;

	LOG_DBG(" --- GPIO MAX14906 init IN ---");

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
	if (config->ready_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&config->ready_gpio)) {
			LOG_ERR("READY GPIO device not ready");
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&config->ready_gpio, GPIO_INPUT);
		if (ret < 0) {
			LOG_ERR("Failed to configure reset GPIO");
			return ret;
		}
	}

	/* setup FAULT gpio - normal high */
	if (config->fault_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&config->fault_gpio)) {
			LOG_ERR("FAULT GPIO device not ready");
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&config->fault_gpio, GPIO_INPUT);
		if (ret < 0) {
			LOG_ERR("Failed to configure DC GPIO");
			return ret;
		}
	}

	/* setup LATCH gpio - normal high */
	if (config->sync_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&config->sync_gpio)) {
			LOG_ERR("SYNC GPIO device not ready");
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&config->sync_gpio, GPIO_OUTPUT_INACTIVE);
		if (ret < 0) {
			LOG_ERR("Failed to configure busy GPIO");
			return ret;
		}

		gpio_pin_set_dt(&config->sync_gpio, 1);
	}

	/* setup LATCH gpio - normal high */
	if (config->en_gpio.port != NULL) {
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
	}

	if (config->fault_gpio.port != NULL) {
		LOG_DBG("[GPIO] FAULT - %d\n", gpio_pin_get_dt(&config->fault_gpio));
	}
	if (config->ready_gpio.port != NULL) {
		LOG_DBG("[GPIO] READY - %d\n", gpio_pin_get_dt(&config->ready_gpio));
	}
	if (config->sync_gpio.port != NULL) {
		LOG_DBG("[GPIO] SYNC  - %d\n", gpio_pin_get_dt(&config->sync_gpio));
	}
	if (config->en_gpio.port != NULL) {
		LOG_DBG("[GPIO] EN    - %d\n", gpio_pin_get_dt(&config->en_gpio));
	}

	ret = gpio_max14906_clean_on_power(dev);
	if (ret < 0) {
		return ret;
	}

	/* all outputs off at power-up */
	ret = max14906_reg_write(dev, MAX14906_SETOUT_REG, 0);
	if (ret < 0) {
		return ret;
	}

	ret = gpio_max14906_init_registers(dev);
	if (ret < 0) {
		return ret;
	}

	LOG_DBG(" --- GPIO MAX14906 init OUT ---");

	return ret;
}

static DEVICE_API(gpio, gpio_max14906_api) = {
	.pin_configure = gpio_max14906_config,
	.port_get_raw = gpio_max14906_port_get_raw,
	.port_set_masked_raw = gpio_max14906_port_set_masked_raw,
	.port_set_bits_raw = gpio_max14906_port_set_bits_raw,
	.port_clear_bits_raw = gpio_max14906_port_clear_bits_raw,
	.port_toggle_bits = gpio_max14906_port_toggle_bits,
};

#define GPIO_MAX14906_DEVICE(id)                                                                   \
	static const struct max14906_config max14906_##id##_cfg = {                                \
		.common = GPIO_COMMON_CONFIG_FROM_DT_INST(id),                                     \
		.spi = SPI_DT_SPEC_INST_GET(id, SPI_OP_MODE_MASTER | SPI_WORD_SET(8U)),            \
		.ready_gpio = GPIO_DT_SPEC_INST_GET_OR(id, drdy_gpios, {0}),                       \
		.fault_gpio = GPIO_DT_SPEC_INST_GET_OR(id, fault_gpios, {0}),                      \
		.sync_gpio = GPIO_DT_SPEC_INST_GET_OR(id, sync_gpios, {0}),                        \
		.en_gpio = GPIO_DT_SPEC_INST_GET_OR(id, en_gpios, {0}),                            \
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
		.curr_lim.reg_bits.CL1 = DT_INST_PROP_BY_IDX(id, curr_lim, 0),                     \
		.curr_lim.reg_bits.CL2 = DT_INST_PROP_BY_IDX(id, curr_lim, 1),                     \
		.curr_lim.reg_bits.CL3 = DT_INST_PROP_BY_IDX(id, curr_lim, 2),                     \
		.curr_lim.reg_bits.CL4 = DT_INST_PROP_BY_IDX(id, curr_lim, 3),                     \
		.config_do.reg_bits.DO_MODE1 = DT_INST_PROP_BY_IDX(id, do_mode, 0),                \
		.config_do.reg_bits.DO_MODE2 = DT_INST_PROP_BY_IDX(id, do_mode, 1),                \
		.config_do.reg_bits.DO_MODE3 = DT_INST_PROP_BY_IDX(id, do_mode, 2),                \
		.config_do.reg_bits.DO_MODE4 = DT_INST_PROP_BY_IDX(id, do_mode, 3),                \
		.config_di.reg_bits.OVL_BLANK = DT_INST_PROP(id, ovl_blank),                       \
		.config_di.reg_bits.OVL_STRETCH_EN = DT_INST_PROP(id, ovl_stretch_en),             \
		.config_di.reg_bits.ABOVE_VDD_PROT_EN =                                            \
			DT_INST_PROP(id, above_vdd_prot_en),                                       \
		/* VDD_FAULT_SEL omitted: muxes VDD faults into DoiLevel, breaks port_get_raw */ \
		.config_di.reg_bits.TYP_2_DI = DT_INST_PROP(id, typ2_di),                         \
		.opn_wr_en.reg_bits =                                                              \
			{                                                                          \
				.OW_OFF_EN1 = DT_INST_PROP_BY_IDX(id, ow_en, 0),                   \
				.OW_OFF_EN2 = DT_INST_PROP_BY_IDX(id, ow_en, 1),                   \
				.OW_OFF_EN3 = DT_INST_PROP_BY_IDX(id, ow_en, 2),                   \
				.OW_OFF_EN4 = DT_INST_PROP_BY_IDX(id, ow_en, 3),                   \
				.GDRV_EN1 = DT_INST_PROP_BY_IDX(id, gdrv_en, 0),                   \
				.GDRV_EN2 = DT_INST_PROP_BY_IDX(id, gdrv_en, 1),                   \
				.GDRV_EN3 = DT_INST_PROP_BY_IDX(id, gdrv_en, 2),                   \
				.GDRV_EN4 = DT_INST_PROP_BY_IDX(id, gdrv_en, 3),                   \
			},                                                                         \
		.sht_vdd_en.reg_bits =                                                             \
			{                                                                          \
				.VDD_OV_EN1 = DT_INST_PROP_BY_IDX(id, vdd_ov_en, 0),               \
				.VDD_OV_EN2 = DT_INST_PROP_BY_IDX(id, vdd_ov_en, 1),               \
				.VDD_OV_EN3 = DT_INST_PROP_BY_IDX(id, vdd_ov_en, 2),               \
				.VDD_OV_EN4 = DT_INST_PROP_BY_IDX(id, vdd_ov_en, 3),               \
				.SH_VDD_EN1 = DT_INST_PROP_BY_IDX(id, sh_vdd_en, 0),               \
				.SH_VDD_EN2 = DT_INST_PROP_BY_IDX(id, sh_vdd_en, 1),               \
				.SH_VDD_EN3 = DT_INST_PROP_BY_IDX(id, sh_vdd_en, 2),               \
				.SH_VDD_EN4 = DT_INST_PROP_BY_IDX(id, sh_vdd_en, 3),               \
			},                                                                         \
		.fault_mask.reg_raw = DT_INST_PROP(id, fault_mask),                                      \
		.spi_addr = DT_INST_PROP(id, spi_addr),                                            \
	};                                                                                         \
                                                                                                   \
	static struct max14906_data max14906_##id##_data = {                                       \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(id, &gpio_max14906_init, NULL, &max14906_##id##_data,                \
			      &max14906_##id##_cfg, POST_KERNEL,                                   \
			      CONFIG_GPIO_MAX14906_INIT_PRIORITY, &gpio_max14906_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_MAX14906_DEVICE)
