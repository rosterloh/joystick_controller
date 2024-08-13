#define DT_DRV_COMPAT adafruit_seesaw

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "seesaw.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(seesaw, CONFIG_SEESAW_LOG_LEVEL);

int seesaw_read(const struct device *dev, uint8_t reg_high, uint8_t reg_low, uint8_t *buf,
		uint8_t num, uint16_t delay)
{
	const struct seesaw_config *config = dev->config;
	int ret;

	struct i2c_msg write_msg, read_msg;
	uint8_t reg[2] = {reg_high, reg_low};

	write_msg.buf = reg;
	write_msg.len = sizeof(reg);
	write_msg.flags = I2C_MSG_WRITE;

	read_msg.buf = buf;
	read_msg.len = num;
	read_msg.flags = I2C_MSG_RESTART | I2C_MSG_READ | I2C_MSG_STOP;

	ret = i2c_transfer_dt(&config->i2c, &write_msg, 1);
	LOG_HEXDUMP_DBG(reg, sizeof(reg), "write");

	k_busy_wait(delay);

	ret = i2c_transfer_dt(&config->i2c, &read_msg, 1);
	LOG_HEXDUMP_DBG(buf, num, "read");

	return ret;
}

int seesaw_write(const struct device *dev, uint8_t reg_high, uint8_t reg_low, uint8_t *buf,
		 uint8_t num)
{
	const struct seesaw_config *config = dev->config;

	uint8_t reg[6] = {reg_high, reg_low};

	memcpy(&reg[2], buf, num);

	LOG_HEXDUMP_DBG(reg, sizeof(reg), "write");

	return i2c_write_dt(&config->i2c, reg, num + 2);
}

static int seesaw_set_pin_mode(const struct device *dev, uint32_t pins, uint8_t mode)
{
	int ret;
	uint8_t cmd[4];
	sys_put_be32(pins, cmd);

	switch (mode) {
	case SEESAW_OUTPUT:
		ret = seesaw_write(dev, SEESAW_GPIO_BASE, SEESAW_GPIO_DIRSET_BULK, cmd,
				   sizeof(cmd));
		break;
	case SEESAW_INPUT:
		ret = seesaw_write(dev, SEESAW_GPIO_BASE, SEESAW_GPIO_DIRCLR_BULK, cmd,
				   sizeof(cmd));
		break;
	case SEESAW_INPUT_PULLUP:
		ret = seesaw_write(dev, SEESAW_GPIO_BASE, SEESAW_GPIO_DIRCLR_BULK, cmd,
				   sizeof(cmd));
		ret |= seesaw_write(dev, SEESAW_GPIO_BASE, SEESAW_GPIO_PULLENSET, cmd, sizeof(cmd));
		ret |= seesaw_write(dev, SEESAW_GPIO_BASE, SEESAW_GPIO_BULK_SET, cmd, sizeof(cmd));
		break;
	case SEESAW_INPUT_PULLDOWN:
		ret = seesaw_write(dev, SEESAW_GPIO_BASE, SEESAW_GPIO_DIRCLR_BULK, cmd,
				   sizeof(cmd));
		ret |= seesaw_write(dev, SEESAW_GPIO_BASE, SEESAW_GPIO_PULLENSET, cmd, sizeof(cmd));
		ret |= seesaw_write(dev, SEESAW_GPIO_BASE, SEESAW_GPIO_BULK_CLR, cmd, sizeof(cmd));
		break;
	}

	return ret;
}

static int seesaw_get_digital(const struct device *dev, uint32_t pins, uint32_t *val)
{
	const struct seesaw_config *config = dev->config;
	uint8_t buf[4];
	int ret;

	ret = seesaw_read(dev, SEESAW_GPIO_BASE, SEESAW_GPIO_BULK, buf, sizeof(buf), config->delay);

	*val = sys_get_be32(buf) & pins;

	return ret;
}

static int seesaw_get_analog(const struct device *dev, uint8_t pin, uint16_t *val)
{
	int ret;
	uint8_t buf[2];
	uint8_t p;

	switch (pin) {
	case ADC_INPUT_0_PIN:
		p = 0;
		break;
	case ADC_INPUT_1_PIN:
		p = 1;
		break;
	case ADC_INPUT_2_PIN:
		p = 2;
		break;
	case ADC_INPUT_3_PIN:
		p = 3;
		break;
	default:
		LOG_WRN("Unknown analog pin read requested (%d)", pin);
		return 0;
		break;
	}

	ret = seesaw_read(dev, SEESAW_ADC_BASE, SEESAW_ADC_CHANNEL_OFFSET + p, buf, sizeof(buf),
			  500);

	*val = sys_get_be16(buf);

	return ret;
}

static int seesaw_set_gpio_interrupts(const struct device *dev, uint32_t pins, uint8_t en)
{
	int ret;
	uint8_t cmd[4];
	sys_put_be32(pins, cmd);

	if (en) {
		ret = seesaw_write(dev, SEESAW_GPIO_BASE, SEESAW_GPIO_INTENSET, cmd, sizeof(cmd));
	} else {
		ret = seesaw_write(dev, SEESAW_GPIO_BASE, SEESAW_GPIO_INTENCLR, cmd, sizeof(cmd));
	}

	return ret;
}

static int seesaw_sw_reset(const struct device *dev)
{
	uint8_t rst = 0xFF;
	return seesaw_write(dev, SEESAW_STATUS_BASE, SEESAW_STATUS_SWRST, &rst, sizeof(rst));
}

static int seesaw_init_device(const struct device *dev)
{
	const struct seesaw_config *config = dev->config;
	struct seesaw_data *data = dev->data;
	uint8_t buf[4];

	k_busy_wait(SEESAW_WAIT_STARTUP_US);
	if (seesaw_sw_reset(dev) < 0) {
		LOG_ERR("Failed to software reset device");
		return -EIO;
	}
	k_busy_wait(SEESAW_WAIT_INITIAL_RESET_US);

	if (seesaw_read(dev, SEESAW_STATUS_BASE, SEESAW_STATUS_HW_ID, &data->hw_id,
			sizeof(data->hw_id), config->delay) < 0) {
		LOG_ERR("Cannot obtain hardware ID");
		return -EIO;
	}

	if ((data->hw_id != SEESAW_HW_ID_CODE_SAMD09) &&
	    (data->hw_id != SEESAW_HW_ID_CODE_TINY817) &&
	    (data->hw_id != SEESAW_HW_ID_CODE_TINY807) &&
	    (data->hw_id != SEESAW_HW_ID_CODE_TINY816) &&
	    (data->hw_id != SEESAW_HW_ID_CODE_TINY806) &&
	    (data->hw_id != SEESAW_HW_ID_CODE_TINY1616) &&
	    (data->hw_id != SEESAW_HW_ID_CODE_TINY1617)) {
		LOG_ERR("Unknown device deteced: 0x%x", data->hw_id);
		return -EFAULT;
	}

	if (seesaw_read(dev, SEESAW_STATUS_BASE, SEESAW_STATUS_VERSION, buf, sizeof(buf),
			config->delay) < 0) {
		LOG_ERR("Cannot obtain version information");
		return -EIO;
	}

	uint32_t version = sys_get_be32(buf);
	data->pid = version >> 16;
	LOG_DBG("Product ID: %d, Compiled: %d-%d-%d", data->pid, (version >> 11) & 0x1F,
		(version >> 7) & 0xF, version & 0x3F);

	if (seesaw_read(dev, SEESAW_STATUS_BASE, SEESAW_STATUS_OPTIONS, buf, sizeof(buf),
			config->delay) < 0) {
		LOG_ERR("Cannot obtain version information");
		return -EIO;
	}

	data->options = sys_get_be32(buf);
	LOG_DBG("Modules found:");

	if ((data->options & BIT(SEESAW_TIMER_BASE)) > 0) {
		LOG_DBG("\tTIMER");
	}
	if ((data->options & BIT(SEESAW_ADC_BASE)) > 0) {
		LOG_DBG("\tADC");
	}
	if ((data->options & BIT(SEESAW_DAC_BASE)) > 0) {
		LOG_DBG("\tDAC");
	}
	if ((data->options & BIT(SEESAW_INTERRUPT_BASE)) > 0) {
		LOG_DBG("\tINTERRUPT");
	}
	if ((data->options & BIT(SEESAW_DAP_BASE)) > 0) {
		LOG_DBG("\tDAP");
	}
	if ((data->options & BIT(SEESAW_EEPROM_BASE)) > 0) {
		LOG_DBG("\tEEPROM");
	}
	if ((data->options & BIT(SEESAW_NEOPIXEL_BASE)) > 0) {
		LOG_DBG("\tNEOPIXEL");
	}
	if ((data->options & BIT(SEESAW_TOUCH_BASE)) > 0) {
		LOG_DBG("\tTOUCH");
	}
	if ((data->options & BIT(SEESAW_KEYPAD_BASE)) > 0) {
		LOG_DBG("\tKEYPAD");
	}
	if ((data->options & BIT(SEESAW_ENCODER_BASE)) > 0) {
		LOG_DBG("\tENCODER");
	}
	if ((data->options & BIT(SEESAW_SPECTRUM_BASE)) > 0) {
		LOG_DBG("\tSPECTRUM");
	}
	if ((data->options & BIT(SEESAW_SOIL_BASE)) > 0) {
		LOG_DBG("\tSOIL");
	}
	return 0;
}

int seesaw_init(const struct device *dev)
{
	const struct seesaw_config *config = dev->config;

	k_sleep(K_SECONDS(2));

	if (!device_is_ready(config->i2c.bus)) {
		LOG_ERR("Bus device is not ready");
		return -EINVAL;
	}

	if (seesaw_init_device(dev) < 0) {
		LOG_ERR("Failed to initialise device!");
		return -EIO;
	}

#ifdef CONFIG_SEESAW_TRIGGER
	if (seesaw_init_interrupt(dev) < 0) {
		LOG_ERR("Failed to initialise interrupt!");
		return -EIO;
	}
#endif
	return 0;
}

static const struct seesaw_driver_api seesaw_driver_api = {
	.write_pin_mode = seesaw_set_pin_mode,
	.read_digital = seesaw_get_digital,
	.read_analog = seesaw_get_analog,
	.gpio_interrupts = seesaw_set_gpio_interrupts,
#ifdef CONFIG_SEESAW_TRIGGER
	.int_set = seesaw_set_int_callback,
#endif
};

#define SEESAW_INIT(index)                                                                         \
	static struct seesaw_data seesaw_data_##index;                                             \
                                                                                                   \
	static const struct seesaw_config seesaw_config_##index = {                                \
		.i2c = I2C_DT_SPEC_INST_GET(index),                                                \
		.delay = DT_INST_PROP(index, read_delay_ms),                                       \
		IF_ENABLED(CONFIG_SEESAW_TRIGGER,                                                  \
			   (.int_gpio = GPIO_DT_SPEC_INST_GET_OR(index, int_gpios, {0}), ))};      \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(index, &seesaw_init, NULL, &seesaw_data_##index,                     \
			      &seesaw_config_##index, POST_KERNEL, CONFIG_SEESAW_INIT_PRIORITY,    \
			      &seesaw_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SEESAW_INIT)