#ifndef INCLUDE_DRIVERS_SEESAW_H_
#define INCLUDE_DRIVERS_SEESAW_H_

/**
 * @brief Seesaw Interface
 * @defgroup seesaw_interface Seesaw Interface
 * @ingroup io_interfaces
 * @{
 */

#include <zephyr/device.h>
#include <zephyr/toolchain.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

enum {
	SEESAW_INPUT = 0x0,
	SEESAW_OUTPUT = 0x1,
	SEESAW_INPUT_PULLUP = 0x2,
	SEESAW_INPUT_PULLDOWN = 0x3,
};

#define NEOKEY_1X4_BUTTONA 4
#define NEOKEY_1X4_BUTTONB 5
#define NEOKEY_1X4_BUTTONC 6
#define NEOKEY_1X4_BUTTOND 7
#define NEOKEY_1X4_BUTTONMASK                                                                      \
	(BIT(NEOKEY_1X4_BUTTONA) | BIT(NEOKEY_1X4_BUTTONB) | BIT(NEOKEY_1X4_BUTTONC) |             \
	 BIT(NEOKEY_1X4_BUTTOND))

/**
 * @typedef seesaw_write_pin_mode_t
 * @brief Callback API to configure seesaw gpio.
 *
 * See seesaw_write_pin_mode() for argument description
 */
typedef int (*seesaw_write_pin_mode_t)(const struct device *dev, uint32_t pins, uint8_t mode);

/**
 * @typedef seesaw_read_digital_t
 * @brief Callback API for reading seesaw gpio pins.
 *
 * See seesaw_read_digital() for argument description
 */
typedef int (*seesaw_read_digital_t)(const struct device *dev, uint32_t pins, uint32_t *val);

/**
 * @typedef seesaw_read_analog_t
 * @brief Callback API for reading an seesaw analog pin.
 *
 * See seesaw_read_analog() for argument description
 */
typedef int (*seesaw_read_analog_t)(const struct device *dev, uint8_t pin, uint16_t *val);

/**
 * @typedef seesaw_interrupts_gpio_t
 * @brief Callback API to configure seesaw gpio interrupts.
 *
 * See seesaw_gpio_interrupts() for argument description
 */
typedef int (*seesaw_interrupts_gpio_t)(const struct device *dev, uint32_t pins, uint8_t enable);

/**
 * @typedef seesaw_int_callback_t
 * @brief Define the callback function for interrupts
 *
 * @param "struct device *dev" Pointer to the seesaw device
 */
typedef void (*seesaw_int_callback_t)(const struct device *dev);

/**
 * @typedef seesaw_int_set_t
 * @brief Callback API for setting a seesaw's interrupt handler
 *
 * See seesaw_int_set() for argument description
 */
typedef int (*seesaw_int_set_t)(const struct device *dev, seesaw_int_callback_t handler);

/** @brief Seesaw driver class operations */
__subsystem struct seesaw_driver_api {
	seesaw_write_pin_mode_t write_pin_mode;
	seesaw_read_digital_t read_digital;
	seesaw_read_analog_t read_analog;
	seesaw_interrupts_gpio_t gpio_interrupts;
	seesaw_int_set_t int_set;
};

/**
 * @brief Configures the gpio of a seesaw device.
 *
 * @param dev Pointer to the seesaw device.
 * @param pins Button mask of pins to configure.
 * @param mode Mode to configure pins as.
 *
 * @return 0 if successful, negative errno code of first failing property
 */
__syscall int seesaw_write_pin_mode(const struct device *dev, uint32_t pins, uint8_t mode);

static inline int z_impl_seesaw_write_pin_mode(const struct device *dev, uint32_t pins,
					       uint8_t mode)
{
	const struct seesaw_driver_api *api = (const struct seesaw_driver_api *)dev->api;

	if (api->write_pin_mode == NULL) {
		return -ENOSYS;
	}

	return api->write_pin_mode(dev, pins, mode);
}

/**
 * @brief Reads the state of the gpios of a seesaw device.
 *
 * @param dev Pointer to the seesaw device.
 * @param pins Button mask of pins to read.
 * @param val Pointer to where values are read into
 *
 * @return 0 if successful, negative errno code of first failing property
 */
__syscall int seesaw_read_digital(const struct device *dev, uint32_t pins, uint32_t *val);

static inline int z_impl_seesaw_read_digital(const struct device *dev, uint32_t pins, uint32_t *val)
{
	const struct seesaw_driver_api *api = (const struct seesaw_driver_api *)dev->api;

	if (api->read_digital == NULL) {
		return -ENOSYS;
	}

	return api->read_digital(dev, pins, val);
}

/**
 * @brief Reads the value of a seesaw analog pin.
 *
 * @param dev Pointer to the seesaw device.
 * @param pin Analog pin to read.
 * @param val Pointer to where value is read into
 *
 * @return 0 if successful, negative errno code of first failing property
 */
__syscall int seesaw_read_analog(const struct device *dev, uint8_t pin, uint16_t *val);

static inline int z_impl_seesaw_read_analog(const struct device *dev, uint8_t pin, uint16_t *val)
{
	const struct seesaw_driver_api *api = (const struct seesaw_driver_api *)dev->api;

	if (api->read_analog == NULL) {
		return -ENOSYS;
	}

	return api->read_analog(dev, pin, val);
}

/**
 * @brief Configures the gpio interrupts of a seesaw device.
 *
 * @param dev Pointer to the seesaw device.
 * @param pins Button mask of pins to configure.
 * @param enabled 0 to disable or 1 to enable interrupts.
 *
 * @return 0 if successful, negative errno code of first failing property
 */
__syscall int seesaw_gpio_interrupts(const struct device *dev, uint32_t pins, uint8_t enabled);

static inline int z_impl_seesaw_gpio_interrupts(const struct device *dev, uint32_t pins,
						uint8_t enabled)
{
	const struct seesaw_driver_api *api = (const struct seesaw_driver_api *)dev->api;

	if (api->gpio_interrupts == NULL) {
		return -ENOSYS;
	}

	return api->gpio_interrupts(dev, pins, enabled);
}

/**
 * @brief Set the INT callback function pointer.
 *
 * This sets up the callback for INT. When an IRQ is triggered,
 * the specified function will be called with the device pointer.
 *
 * @param dev Seesaw device structure.
 * @param cb Pointer to the callback function.
 *
 * @return N/A
 */
__syscall void seesaw_int_set(const struct device *dev, seesaw_int_callback_t cb);

static inline void z_impl_seesaw_int_set(const struct device *dev, seesaw_int_callback_t cb)
{
	const struct seesaw_driver_api *api = (const struct seesaw_driver_api *)dev->api;

	if ((api != NULL) && (api->int_set != NULL)) {
		api->int_set(dev, cb);
	}
}

/**
 * @}
 */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#include <syscalls/seesaw.h>

#endif /* INCLUDE_DRIVERS_SEESAW_H_ */