/*
 * Copyright (c) 2024 Analog Devices Inc.
 * Copyright (c) 2024 Baylibre SAS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_GPIO_GPIO_MAX149X6_H_
#define ZEPHYR_DRIVERS_GPIO_GPIO_MAX149X6_H_

#define MAX149x6_READ  0
#define MAX149x6_WRITE 1

#define MAX149x6_BRST_MASK        BIT(5)
#define MAX149x6_BURST_READ_NREGS 6

#define MAX149x6_LOWER_NIBBLE(reg) FIELD_GET(GENMASK(3, 0), (reg))
#define MAX149x6_UPPER_NIBBLE(reg) FIELD_GET(GENMASK(7, 4), (reg))

/**
 * @brief Compute the CRC5 value for an array of bytes when writing to MAX149X6
 * @param data - array of data to encode
 * @param len - number of bytes
 * @param encode - action to be performed - true(encode), false(decode)
 * @param check_byte - SDO check byte masked to top 3 bits (A1|A0|ThrErr); for encode pass 0
 * @return the resulted CRC5
 */
static uint8_t max149x6_crc(uint8_t *data, size_t len, bool encode, uint8_t check_byte)
{
	uint8_t crc5_start = 0x1f;
	uint8_t crc5_poly = 0x15;
	uint8_t crc5_result = crc5_start;
	uint8_t data_bit;
	uint8_t result_bit;
	int i;

	/*
	 * This is a custom implementation of a CRC5 algorithm, detailed here:
	 * https://www.analog.com/en/app-notes/how-to-program-the-max14906-quadchannel-
	 *					industrial-digital-output-digital-input.html
	 */

	for (i = (encode) ? 0 : 2; i < 8; i++) {
		data_bit = (data[0] >> (7 - i)) & 0x01;
		result_bit = (crc5_result & 0x10) >> 4;
		if (data_bit ^ result_bit) {
			crc5_result = crc5_poly ^ ((crc5_result << 1) & 0x1f);
		} else {
			crc5_result = (crc5_result << 1) & 0x1f;
		}
	}

	for (size_t n = 1; n < len; n++) {
		for (i = 0; i < 8; i++) {
			data_bit = (data[n] >> (7 - i)) & 0x01;
			result_bit = (crc5_result & 0x10) >> 4;
			if (data_bit ^ result_bit) {
				crc5_result = crc5_poly ^ ((crc5_result << 1) & 0x1f);
			} else {
				crc5_result = (crc5_result << 1) & 0x1f;
			}
		}
	}

	for (i = 0; i < 3; i++) {
		data_bit = (check_byte >> (7 - i)) & 0x01;
		result_bit = (crc5_result & 0x10) >> 4;
		if (data_bit ^ result_bit) {
			crc5_result = crc5_poly ^ ((crc5_result << 1) & 0x1f);
		} else {
			crc5_result = (crc5_result << 1) & 0x1f;
		}
	}

	return crc5_result;
}

/*
 * @brief Register read/write function for MAX149x6
 *
 * @param dev - MAX149x6 device config.
 * @param addr - Register value to which data is written.
 * @param val - Value which is to be written to requested register.
 * @param rx_diag_buff - Optional buffer for received diagnostic bytes.
 * @param rw - Transaction direction (MAX149x6_READ or MAX149x6_WRITE).
 * @return 0 in case of success, negative error code otherwise.
 */
static int max149x6_reg_transceive(const struct device *dev, uint8_t addr, uint8_t val,
				   uint8_t *rx_diag_buff, uint8_t rw)
{
	uint8_t crc;
	int ret;

	uint8_t local_rx_buff[MAX149x6_MAX_PKT_SIZE] = {0};
	uint8_t local_tx_buff[MAX149x6_MAX_PKT_SIZE] = {0};

	const struct max149x6_config *config = dev->config;

	struct spi_buf tx_buf = {
		.buf = &local_tx_buff,
		.len = config->pkt_size,
	};
	const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

	struct spi_buf rx_buf = {
		.buf = &local_rx_buff,
		.len = config->pkt_size,
	};
	const struct spi_buf_set rx = {.buffers = &rx_buf, .count = 1};

	local_tx_buff[0] = FIELD_PREP(MAX149x6_ADDR_MASK, addr) |
			   FIELD_PREP(MAX149x6_CHIP_ADDR_MASK, config->spi_addr) |
			   FIELD_PREP(MAX149x6_RW_MASK, rw & 0x1);
	local_tx_buff[1] = val;

	/* If CRC enabled calculate it */
	if (config->crc_en) {
		local_tx_buff[2] = max149x6_crc(local_tx_buff, 2, true, 0);
	}

	/* write cmd & read resp at once */
	ret = spi_transceive_dt(&config->spi, &tx, &rx);

	if (ret) {
		LOG_ERR("Err spi_transcieve_dt  [%d]\n", ret);
		return ret;
	}

	/* if CRC enabled check read */
	if (config->crc_en) {
		crc = max149x6_crc(local_rx_buff, 2, false, local_rx_buff[2] & 0xE0);
		if (crc != (local_rx_buff[2] & 0x1F)) {
			LOG_ERR("READ CRC ERR (%d)-(%d)\n", crc, (local_rx_buff[2] & 0x1F));
			return -EINVAL;
		}
	}

	if (rx_diag_buff != NULL) {
		/* byte0 is first diagnostic byte */
		rx_diag_buff[0] = local_rx_buff[0];

		/* byte1 for WRITE: second diagnostic byte */
		if (MAX149x6_WRITE == rw) {
			rx_diag_buff[1] = local_rx_buff[1];
		}
	}

	/* byte1 for READ: register value returned to caller */
	if (rw == MAX149x6_READ) {
		return local_rx_buff[1];
	}

	return ret;
}

/*
 * @brief Burst read consecutive diagnostic registers in a single SPI transaction.
 * Reads all 6 diagnostic registers -> MAX14906: 0x02-0x07, MAX14916: 0x04-0x09.
 *
 * @param dev - MAX149x6 device config.
 * @param start_addr - First diagnostic register address
 * @param regs - Output buffer, must be at least 1 + MAX149x6_BURST_READ_NREGS bytes.
 *               regs[0] = SDO summary byte, regs[1..6] = register data in address order.
 * @return 0 in case of success, negative error code otherwise.
 */
static int max149x6_burst_read(const struct device *dev, uint8_t start_addr,
			       uint8_t *regs)
{
	int ret;

	/* 1 cmd + 6 data + optional CRC */
	uint8_t tx[1 + MAX149x6_BURST_READ_NREGS + 1] = {0};
	uint8_t rx[1 + MAX149x6_BURST_READ_NREGS + 1] = {0};

	const struct max149x6_config *config = dev->config;
	size_t frame_len = 1 + MAX149x6_BURST_READ_NREGS + (config->crc_en ? 1 : 0);

	struct spi_buf tx_buf = {
		.buf = tx,
		.len = frame_len,
	};
	const struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1};

	struct spi_buf rx_buf = {
		.buf = rx,
		.len = frame_len,
	};
	const struct spi_buf_set rx_set = {.buffers = &rx_buf, .count = 1};

	tx[0] = FIELD_PREP(MAX149x6_CHIP_ADDR_MASK, config->spi_addr) |
		MAX149x6_BRST_MASK |
		FIELD_PREP(MAX149x6_ADDR_MASK, start_addr);

	if (config->crc_en) {
		tx[frame_len - 1] = max149x6_crc(tx, frame_len - 1, true, 0);
	}

	ret = spi_transceive_dt(&config->spi, &tx_set, &rx_set);
	if (ret) {
		LOG_ERR("Err spi_transceive_dt burst [%d]\n", ret);
		return ret;
	}

	if (config->crc_en) {
		uint8_t check_byte = rx[frame_len - 1];
		uint8_t crc = max149x6_crc(rx, 1 + MAX149x6_BURST_READ_NREGS,
					   false, check_byte & 0xE0);

		if (crc != (check_byte & 0x1F)) {
			LOG_ERR("BURST CRC ERR (%d)-(%d)\n", crc, (check_byte & 0x1F));
			return -EINVAL;
		}
	}

	memcpy(regs, rx, 1 + MAX149x6_BURST_READ_NREGS);

	return 0;
}

#endif
