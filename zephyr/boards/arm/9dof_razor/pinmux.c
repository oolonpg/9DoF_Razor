/*
 * Copyright (c) 2020 Google LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <init.h>
#include <drivers/pinmux.h>
#include <soc.h>

static int board_pinmux_init(const struct device *dev)
{
	const struct device *muxa =
		device_get_binding(DT_LABEL(DT_NODELABEL(pinmux_a)));
	const struct device *muxb =
		device_get_binding(DT_LABEL(DT_NODELABEL(pinmux_b)));

	ARG_UNUSED(dev);

#if defined(CONFIG_UART_SAM0)

#if ATMEL_SAM0_DT_SERCOM_CHECK(3, atmel_sam0_uart)
#warning Pin mapping may not be configured
#endif
#if ATMEL_SAM0_DT_SERCOM_CHECK(5, atmel_sam0_uart)
#warning Pin mapping may not be configured
#endif

#if ATMEL_SAM0_DT_SERCOM_CHECK(0, atmel_sam0_uart)
	/* SERCOM0on RX=PA11 3, TX=PA10 2*/
	pinmux_pin_set(muxa, 11, PINMUX_FUNC_C);
	pinmux_pin_set(muxa, 10, PINMUX_FUNC_C);
#endif

#if ATMEL_SAM0_DT_SERCOM_CHECK(1, atmel_sam0_uart)
#warning Pin mapping may not be configured
#endif

#if ATMEL_SAM0_DT_SERCOM_CHECK(2, atmel_sam0_uart)
#warning Pin mapping may not be configured
#endif

#if ATMEL_SAM0_DT_SERCOM_CHECK(4, atmel_sam0_uart)
#warning Pin mapping may not be configured
#endif

#endif

#if defined(CONFIG_SPI_SAM0)
#if ATMEL_SAM0_DT_SERCOM_CHECK(1, atmel_sam0_spi)

#endif
#if ATMEL_SAM0_DT_SERCOM_CHECK(2, atmel_sam0_spi)

#endif
#if ATMEL_SAM0_DT_SERCOM_CHECK(0, atmel_sam0_spi)
#warning Pin mapping may not be configured
#endif
#if ATMEL_SAM0_DT_SERCOM_CHECK(3, atmel_sam0_spi)
#warning Pin mapping may not be configured
#endif
#if ATMEL_SAM0_DT_SERCOM_CHECK(4, atmel_sam0_spi)
/* SPI SERCOM4 on MISO=PA12/pad 0, MOSI=PB10/pad 2, SCK=PB11/pad 3 */
	pinmux_pin_set(muxa, 12, PINMUX_FUNC_D);
	pinmux_pin_set(muxb, 10, PINMUX_FUNC_D);
	pinmux_pin_set(muxb, 11, PINMUX_FUNC_D);
#endif
#if ATMEL_SAM0_DT_SERCOM_CHECK(5, atmel_sam0_spi)
#warning Pin mapping may not be configured
#endif
#endif /* CONFIG_SPI_SAM0 */

#if defined(CONFIG_I2C_SAM0)
#if ATMEL_SAM0_DT_SERCOM_CHECK(4, atmel_sam0_i2c)

#endif
#if ATMEL_SAM0_DT_SERCOM_CHECK(0, atmel_sam0_i2c)
#warning Pin mapping may not be configured
#endif
#if ATMEL_SAM0_DT_SERCOM_CHECK(1, atmel_sam0_i2c)
#warning Pin mapping may not be configured
#endif
#if ATMEL_SAM0_DT_SERCOM_CHECK(2, atmel_sam0_i2c)
#warning Pin mapping may not be configured
#endif
#if ATMEL_SAM0_DT_SERCOM_CHECK(3, atmel_sam0_i2c)
/* SERCOM3 on SDA=PA22, SCL=PA23 */
	pinmux_pin_set(muxa, 22, PINMUX_FUNC_C);
	pinmux_pin_set(muxa, 23, PINMUX_FUNC_C);
#endif
#if ATMEL_SAM0_DT_SERCOM_CHECK(5, atmel_sam0_i2c)
#warning Pin mapping may not be configured
#endif
#endif

#if defined(CONFIG_PWM_SAM0_TCC)
#if ATMEL_SAM0_DT_TCC_CHECK(2, atmel_sam0_tcc_pwm)

#endif
#endif

	if (IS_ENABLED(CONFIG_USB_DC_SAM0)) {
		/* USB DP on PA25, USB DM on PA24 */
		pinmux_pin_set(muxa, 25, PINMUX_FUNC_G);
		pinmux_pin_set(muxa, 24, PINMUX_FUNC_G);
	}

	return 0;
}

SYS_INIT(board_pinmux_init, PRE_KERNEL_1, CONFIG_PINMUX_INIT_PRIORITY);
