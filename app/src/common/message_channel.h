#ifndef _MESSAGE_CHANNEL_H_
#define _MESSAGE_CHANNEL_H_

#include <zephyr/zbus/zbus.h>

#define DEFAULT_OBS_PRIO 1

/**
 * @brief System State.
 */
enum sys_states {
	/**
	 * @brief System was active and is transitioning to sleep now.
	 */
	SYS_SLEEP,

	/**
	 * @brief System is in standby.
	 */
	SYS_STANDBY,
};

/**
 * @brief System Events.
 */
enum sys_events {
	/**
	 * @brief Button Pressed event
	 */
	SYS_BUTTON_PRESSED,

	BUTTON_A_PRESSED,
	BUTTON_B_PRESSED,
	BUTTON_C_PRESSED,
	BUTTON_D_PRESSED,
	BUTTON_A_RELEASED,
	BUTTON_B_RELEASED,
	BUTTON_C_RELEASED,
	BUTTON_D_RELEASED,
};

ZBUS_CHAN_DECLARE(button_ch);
ZBUS_CHAN_DECLARE(led_ch);

#endif /* _MESSAGE_CHANNEL_H_ */