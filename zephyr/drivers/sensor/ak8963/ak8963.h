/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_AK8963_AK8963_H_
#define ZEPHYR_DRIVERS_SENSOR_AK8963_AK8963_H_

#include <device.h>

#define AK8963_REG_CHIP_ID		0x00
#define AK8963_CHIP_ID			0x48

#define AK8963_REG_DATA_START		0x03

#define AK8963_REG_CNTL			0x0A
#define AK8963_MODE_MEASURE		0x01
#define AK8963_MODE_FUSE_ACCESS		0x0F

#define AK8963_REG_ADJ_DATA_START	0x10

#define AK8963_MEASURE_TIME_US		9000
#define AK8963_MICRO_GAUSS_PER_BIT	3000

#if DT_NODE_HAS_PROP(DT_INST(0, invensense_mpu9250), reg)
#if DT_REG_ADDR(DT_INST(0, asahi_kasei_ak8963)) != 0x0C
#error "I2C address must be 0x0C when AK8963 is part of a MPU9250 chip"
#endif

#define MPU9250_I2C_ADDR		DT_REG_ADDR(DT_INST(0, invensense_mpu9250))

#define MPU9250_REG_BYPASS_CFG		0x37
#define MPU9250_I2C_BYPASS_EN		BIT(1)

#define MPU9250_REG_PWR_MGMT1		0x6B
#define MPU9250_SLEEP_EN		BIT(6)

#endif /* DT_NODE_HAS_PROP(DT_INST(0, invensense_mpu9150), reg) */


struct ak8963_data {
	const struct device *i2c;

	int16_t x_sample;
	int16_t y_sample;
	int16_t z_sample;

	uint8_t x_adj;
	uint8_t y_adj;
	uint8_t z_adj;
};

#endif /* ZEPHYR_DRIVERS_SENSOR_AK8963_AK8963_H_ */
