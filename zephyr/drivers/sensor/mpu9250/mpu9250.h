/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_MPU9250_MPU9250_H_
#define ZEPHYR_DRIVERS_SENSOR_MPU9250_MPU9250_H_

#include <device.h>
#include <drivers/gpio.h>
#include <sys/util.h>
#include <zephyr/types.h>

#define MPU9250_REG_CHIP_ID		0x75
#define MPU9250_CHIP_ID			0x71

#define MPU9250_REG_GYRO_CFG		0x1B
#define MPU9250_GYRO_FS_SHIFT		3

#define MPU9250_REG_ACCEL_CFG		0x1C
#define MPU9250_ACCEL_FS_SHIFT		3

#define MPU9250_REG_INT_EN		0x38
#define MPU9250_DRDY_EN			BIT(0)

#define MPU9250_REG_DATA_START		0x3B

#define MPU9250_REG_PWR_MGMT1		0x6B
#define MPU9250_SLEEP_EN		BIT(6)

#define MPU9250_REG_PWR_MGMT2       0x6C

/* measured in degrees/sec x10 to avoid floating point */
static const uint16_t mpu9250_gyro_sensitivity_x10[] = {
	1310, 655, 328, 164
};

struct mpu9250_data {
	const struct device *i2c;

	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	uint16_t accel_sensitivity_shift;

	int16_t temp;

	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
	uint16_t gyro_sensitivity_x10;

#ifdef CONFIG_MPU9250_TRIGGER
	const struct device *dev;
	const struct device *gpio;
	struct gpio_callback gpio_cb;

	struct sensor_trigger data_ready_trigger;
	sensor_trigger_handler_t data_ready_handler;

#if defined(CONFIG_MPU9250_TRIGGER_OWN_THREAD)
	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_MPU9250_THREAD_STACK_SIZE);
	struct k_thread thread;
	struct k_sem gpio_sem;
#elif defined(CONFIG_MPU9250_TRIGGER_GLOBAL_THREAD)
	struct k_work work;
#endif

#endif /* CONFIG_MPU9250_TRIGGER */
};

struct mpu9250_config {
	const char *i2c_label;
	uint16_t i2c_addr;
#ifdef CONFIG_MPU9250_TRIGGER
	uint8_t int_pin;
	uint8_t int_flags;
	const char *int_label;
#endif /* CONFIG_MPU9250_TRIGGER */
};

#ifdef CONFIG_MPU9250_TRIGGER
int mpu9250_trigger_set(const struct device *dev,
			const struct sensor_trigger *trig,
			sensor_trigger_handler_t handler);

int mpu9250_init_interrupt(const struct device *dev);
#endif

#endif /* __SENSOR_MPU9250__ */
