/*
 * Copyright (c) 2020   oolon.org
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <sys/printk.h>
#include <drivers/i2c.h>
#include <drivers/sensor.h>
#include <stdio.h>

#include "9dof_hal.h"

/* Test i2c */
#define I2C_DEV DT_LABEL(DT_ALIAS(i2c_0))

static const char *now_str(void)
{
	static char buf[16]; /* ...HH:MM:SS.MMM */
	uint32_t now = k_uptime_get_32();
	unsigned int ms = now % MSEC_PER_SEC;
	unsigned int s;
	unsigned int min;
	unsigned int h;

	now /= MSEC_PER_SEC;
	s = now % 60U;
	now /= 60U;
	min = now % 60U;
	now /= 60U;
	h = now;

	snprintf(buf, sizeof(buf), "%u:%02u:%02u.%03u",
		 h, min, s, ms);
	return buf;
}

int razor_hal_init()
{
    printk("##Enter razor_hal_init\n");

	 /* Check for i2c */
	const struct device * i2c_dev;
	i2c_dev = device_get_binding(I2C_DEV);

	if (!i2c_dev) {
		printk("I2C: Device driver not found.\n");
		return -EIO;
	}

    /* Check for MPU9250 	*/
    const char *const label = DT_LABEL(DT_INST(0, invensense_mpu9250));
	const struct device *mpu9250 = device_get_binding(label);

	if (!mpu9250) {
		printk("Failed to find sensor %s\n", label);
		return -EIO;;
	}

    /* Check for AK8963 */
	const struct device *ak8963 = device_get_binding("AK8963");

    if (!ak8963) {
		printk("Failed to find sensor AK8963\n");
		return -EIO;;
	}

    return 0;
}

int process_razor(void)
{
	/*
	Get all sensor values 
	*/

	int rc;

	const char *const label = DT_LABEL(DT_INST(0, invensense_mpu9250));
	const struct device *mpu9250 = device_get_binding(label);

	const struct device *ak8963 = device_get_binding("AK8963");

	struct sensor_value temperature;
	struct sensor_value accel[3];
	struct sensor_value gyro[3];
	struct sensor_value mag[3];

	rc = sensor_sample_fetch(mpu9250);

	rc = sensor_sample_fetch(ak8963);

	if (rc == 0) {
		rc = sensor_channel_get(mpu9250, SENSOR_CHAN_ACCEL_XYZ, accel);
	}

	if (rc == 0) {
		rc = sensor_channel_get(mpu9250, SENSOR_CHAN_GYRO_XYZ, gyro);
	}

	if (rc == 0) {
		rc = sensor_channel_get(mpu9250, SENSOR_CHAN_AMBIENT_TEMP, &temperature);
	}
	
	if (rc == 0) {
		rc = sensor_channel_get(ak8963, SENSOR_CHAN_MAGN_XYZ,mag);
	}

	if (rc == 0) {
		printk("[%s]:%g Cel\n"
		       "  accel %f %f %f m/s/s\n"
		       "  gyro  %f %f %f rad/s\n"
			   "  mag 	%f %f %f \n",
		       now_str(),
		       sensor_value_to_double(&temperature),
		       sensor_value_to_double(&accel[0]),
		       sensor_value_to_double(&accel[1]),
		       sensor_value_to_double(&accel[2]),
		       sensor_value_to_double(&gyro[0]),
		       sensor_value_to_double(&gyro[1]),
		       sensor_value_to_double(&gyro[2]),
			   sensor_value_to_double(&mag[0]),
		       sensor_value_to_double(&mag[1]),
		       sensor_value_to_double(&mag[2]));
	} else {
		printk("sample fetch/get failed: %d\n", rc);
	}
	
	return 0;

}


int process_mpu9250(const struct device *dev)
{
	struct sensor_value temperature;
	struct sensor_value accel[3];
	struct sensor_value gyro[3];
	
	int rc = sensor_sample_fetch(dev);

	if (rc == 0) {
		rc = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ,
					accel);
	}
	if (rc == 0) {
		rc = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ,
					gyro);
	}
	if (rc == 0) {
		rc = sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP,
					&temperature);
	}
	if (rc == 0) {
		printk("[%s]:%g Cel\n"
		       "  accel %f %f %f m/s/s\n"
		       "  gyro  %f %f %f rad/s\n",
		       now_str(),
		       sensor_value_to_double(&temperature),
		       sensor_value_to_double(&accel[0]),
		       sensor_value_to_double(&accel[1]),
		       sensor_value_to_double(&accel[2]),
		       sensor_value_to_double(&gyro[0]),
		       sensor_value_to_double(&gyro[1]),
		       sensor_value_to_double(&gyro[2]));
	} else {
		printk("sample fetch/get failed: %d\n", rc);
	}

	return rc;
}


int process_ak8963(const struct device *dev)
{
	struct sensor_value mag[3];
	int rc = sensor_sample_fetch(dev);

	if (rc == 0) {
		rc = sensor_channel_get(dev, SENSOR_CHAN_MAGN_XYZ,mag);
	}

	if (rc == 0) {
		printk("[%s]: mag %f %f %f \n",
		       now_str(),
		       sensor_value_to_double(&mag[0]),
		       sensor_value_to_double(&mag[1]),
		       sensor_value_to_double(&mag[2]));
	} else {
		printk("sample fetch/get failed: %d\n", rc);
	}

	return rc;
}