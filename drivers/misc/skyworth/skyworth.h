#ifndef __SKYWORTH_H__ 
#define __SKYWORTH_H__

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif

#include <dt-bindings/gpio/gpio.h>

struct td1030_platform_data {
        int BD_NRESET_gpio;
	int BD_NRESET_gpio_status;
	int BD_EN_gpio;
	int BD_EN_gpio_status;
	int BD_LAN_gpio;
	int BD_LAN_gpio_status;
};

struct led-control_platform_data {
        int LED_R_gpio;
        int LED_R_gpio_status;
        int LED_G_gpio;
        int LED_G_gpio_status;
        int LED_B_gpio;
        int LED_B_gpio_status;
};

#endif
