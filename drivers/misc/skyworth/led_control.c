/* 
 * drivers/misc/skyworth/led_control.c - driver for led_control
 * add by cuikai
 * 2017.05.23
 */

#define DEBUG
#define pr_fmt(fmt) "cuikai: %s: " fmt, __func__

#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/fcntl.h>
#include <linux/input.h>
#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/uaccess.h>

#include <linux/of_gpio.h>
#include <linux/rk_fb.h>


#include <linux/wakelock.h>
//#include "skyworth.h"


#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif

#include <dt-bindings/gpio/gpio.h>

#define	MAX_SIZE(a)			sizeof(a)/sizeof(a[0])

#define BDGPS_ON                                0x4800
#define BDGPS_RST                               0x4801
#define BDGPS_STOP                              0x4802

//static struct wake_lock usb_wakelock;

#define GPIO_JIFFIES            (1000 / (MSEC_PER_SEC / HZ))      /* 80ms */

struct led_control_platform_data {
	int USB_DRV_gpio;
	int USB_DRV_gpio_status;
	int SDMMC_PWR_gpio;
	int SDMMC_PWR_gpio_status;
	int HUB_PWR_gpio;
	int HUB_PWR_gpio_status;
	int LED_CTL_gpio;
	int LED_CTL_gpio_status;
	int sys_sta_gpio;
	int sys_sta_gpio_status;
	int WIFI_PWR_gpio;
	int WIFI_PWR_gpio_status;
	int DVP_CTL_gpio;
	int DVP_CTL_gpio_status;
	struct timer_list mytimer;
	struct delayed_work mywork;
};

struct led_control_platform_data *mpdata;

struct gpio_state {
	struct mutex		mutex; /* mutual excl. when accessing chip */
	void __iomem *base;
	int			irq;
	struct timer_list timer;
};


#ifdef CONFIG_OF
static struct led_control_platform_data *led_control_parse_dt(struct device *dev)
{
	enum of_gpio_flags flags;
	int gpio;
        struct led_control_platform_data *pdata;

        pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
        if (!pdata) {
                dev_err(dev, "memory allocation for led_control_data failed_control\n");
                return ERR_PTR(-ENOMEM);
        }
	if (of_find_property(dev->of_node, "USB_DRV_gpio", NULL)) 
	{
        	gpio = of_get_named_gpio_flags(dev->of_node, "USB_DRV_gpio", 0, &flags);
        	if (gpio_is_valid(gpio))
		{
            		pdata->USB_DRV_gpio = gpio;
            		pdata->USB_DRV_gpio_status = (flags == GPIO_ACTIVE_HIGH)? 1:0;
        	}
    	}
	if (of_find_property(dev->of_node, "SDMMC_PWR_gpio", NULL)) 
	{
        	gpio = of_get_named_gpio_flags(dev->of_node, "SDMMC_PWR_gpio", 0, &flags);
        	if (gpio_is_valid(gpio))
		{
            		pdata->SDMMC_PWR_gpio = gpio;
            		pdata->SDMMC_PWR_gpio_status = (flags == GPIO_ACTIVE_HIGH)? 1:0;
        	}
    	}
	if (of_find_property(dev->of_node, "HUB_PWR_gpio", NULL)) 
	{
        	gpio = of_get_named_gpio_flags(dev->of_node, "HUB_PWR_gpio", 0, &flags);
        	if (gpio_is_valid(gpio))
		{
            		pdata->HUB_PWR_gpio = gpio;
            		pdata->HUB_PWR_gpio_status = (flags == GPIO_ACTIVE_HIGH)? 1:0;
        	}
    	}
	if (of_find_property(dev->of_node, "LED_CTL_gpio", NULL)) 
	{
        	gpio = of_get_named_gpio_flags(dev->of_node, "LED_CTL_gpio", 0, &flags);
        	if (gpio_is_valid(gpio))
		{
            		pdata->LED_CTL_gpio = gpio;
            		pdata->LED_CTL_gpio_status = (flags == GPIO_ACTIVE_HIGH)? 1:0;
        	}
    	}
	if (of_find_property(dev->of_node, "sys_sta_gpio", NULL)) 
	{
        	gpio = of_get_named_gpio_flags(dev->of_node, "sys_sta_gpio", 0, &flags);
        	if (gpio_is_valid(gpio))
		{
            		pdata->sys_sta_gpio = gpio;
            		pdata->sys_sta_gpio_status = (flags == GPIO_ACTIVE_HIGH)? 1:0;
        	}
    	}
	if (of_find_property(dev->of_node, "WIFI_PWR_gpio", NULL)) 
	{
        	gpio = of_get_named_gpio_flags(dev->of_node, "WIFI_PWR_gpio", 0, &flags);
        	if (gpio_is_valid(gpio))
		{
            		pdata->WIFI_PWR_gpio = gpio;
            		pdata->WIFI_PWR_gpio_status = (flags == GPIO_ACTIVE_HIGH)? 1:0;
        	}
    	}
	if (of_find_property(dev->of_node, "DVP_CTL_gpio", NULL)) 
	{
        	gpio = of_get_named_gpio_flags(dev->of_node, "DVP_CTL_gpio", 0, &flags);
        	if (gpio_is_valid(gpio))
		{
            		pdata->DVP_CTL_gpio = gpio;
            		pdata->DVP_CTL_gpio_status = (flags == GPIO_ACTIVE_HIGH)? 1:0;
        	}
    	}
	return pdata;
}
#endif

int led_early_suspend()
{
        gpio_direction_output(mpdata->LED_CTL_gpio,!mpdata->LED_CTL_gpio_status);
        gpio_set_value(mpdata->LED_CTL_gpio,!mpdata->LED_CTL_gpio_status);
        printk("%s\n",__FUNCTION__);
        return 0;
}

int led_early_resume()
{
        gpio_direction_output(mpdata->LED_CTL_gpio,mpdata->LED_CTL_gpio_status);
        gpio_set_value(mpdata->LED_CTL_gpio,mpdata->LED_CTL_gpio_status);
        printk("%s\n",__FUNCTION__);
        return 0;
}

static int rockchip_led_fb_event_notify(struct notifier_block *self,
                                           unsigned long action,
                                           void *data)
{
        struct fb_event *event = data;
        int blank_mode = *((int *)event->data);

        if (action == FB_EARLY_EVENT_BLANK) {
                switch (blank_mode) {
                case FB_BLANK_UNBLANK:
                        break;
                default:
			printk("lcdc suspend\n");
			led_early_suspend();
                        break;
                }
        } else if (action == FB_EVENT_BLANK) {
                switch (blank_mode) {
                case FB_BLANK_UNBLANK:
			printk("lcdc resume\n");
			led_early_resume();
                        break;
                default:
                        break;
                }
        }

        return NOTIFY_OK;
}

static struct notifier_block rockchip_led_fb_notifier = {
        .notifier_call = rockchip_led_fb_event_notify,
};

static void gpio_timer(struct work_struct *work)
{
        int ret;
        struct led_control_platform_data *pdata = container_of(work, struct led_control_platform_data, mywork.work);
	schedule_delayed_work(&pdata->mywork,GPIO_JIFFIES);
	static int mcu_cnt=0;
	mcu_cnt++;
	if(mcu_cnt%2==0)
		gpio_set_value(pdata->sys_sta_gpio,pdata->sys_sta_gpio_status);
	else
		gpio_set_value(pdata->sys_sta_gpio,!pdata->sys_sta_gpio_status);
}
static int led_control_setup_gpio(struct led_control_platform_data *pdata)
{
	int ret;

	if (gpio_is_valid(pdata->USB_DRV_gpio))
	{
        	int ret = gpio_request(pdata->USB_DRV_gpio, "USB_DRV_gpio");
		 if (ret) 
            		printk("Failed_control to get %s gpio.\n", pdata->USB_DRV_gpio);
        	else
		{
			gpio_direction_output(pdata->USB_DRV_gpio,pdata->USB_DRV_gpio_status);
			gpio_set_value(pdata->USB_DRV_gpio,pdata->USB_DRV_gpio_status);
		}
        }
	if (gpio_is_valid(pdata->SDMMC_PWR_gpio))
	{
        	int ret = gpio_request(pdata->SDMMC_PWR_gpio, "SDMMC_PWR_gpio");
		 if (ret) 
            		printk("Failed_control to get %s gpio.\n", pdata->SDMMC_PWR_gpio);
        	else
		{
			gpio_direction_output(pdata->SDMMC_PWR_gpio,pdata->SDMMC_PWR_gpio_status);
			gpio_set_value(pdata->SDMMC_PWR_gpio,pdata->SDMMC_PWR_gpio_status);
		}
        }
	if (gpio_is_valid(pdata->HUB_PWR_gpio))
	{
        	int ret = gpio_request(pdata->HUB_PWR_gpio, "HUB_PWR_gpio");
		 if (ret) 
            		printk("Failed_control to get %s gpio.\n", pdata->HUB_PWR_gpio);
        	else
		{
			gpio_direction_output(pdata->HUB_PWR_gpio,pdata->HUB_PWR_gpio_status);
			gpio_set_value(pdata->HUB_PWR_gpio,pdata->HUB_PWR_gpio_status);
		}
        }
	if (gpio_is_valid(pdata->LED_CTL_gpio))
	{
        	int ret = gpio_request(pdata->LED_CTL_gpio, "LED_CTL_gpio");
		 if (ret) 
            		printk("Failed_control to get %s gpio.\n", pdata->LED_CTL_gpio);
        	else
		{
			gpio_direction_output(pdata->LED_CTL_gpio,pdata->LED_CTL_gpio_status);
			gpio_set_value(pdata->LED_CTL_gpio,pdata->LED_CTL_gpio_status);
		}
        }
	if (gpio_is_valid(pdata->sys_sta_gpio))
	{
        	int ret = gpio_request(pdata->sys_sta_gpio, "sys_sta_gpio");
		 if (ret) 
            		printk("Failed_control to get %s gpio.\n", pdata->sys_sta_gpio);
        	else
		{
			INIT_DELAYED_WORK(&pdata->mywork, gpio_timer);
        		schedule_delayed_work(&pdata->mywork,GPIO_JIFFIES);
			gpio_direction_output(pdata->sys_sta_gpio,pdata->sys_sta_gpio_status);
			gpio_set_value(pdata->sys_sta_gpio,pdata->sys_sta_gpio_status);
		}
        }
	if (gpio_is_valid(pdata->WIFI_PWR_gpio))
	{
        	int ret = gpio_request(pdata->WIFI_PWR_gpio, "WIFI_PWR_gpio");
		 if (ret) 
            		printk("Failed_control to get %s gpio.\n", pdata->WIFI_PWR_gpio);
        	else
		{
			gpio_direction_output(pdata->WIFI_PWR_gpio,pdata->WIFI_PWR_gpio_status);
			gpio_set_value(pdata->WIFI_PWR_gpio,pdata->WIFI_PWR_gpio_status);
		}
        }
	if (gpio_is_valid(pdata->DVP_CTL_gpio))
	{
        	int ret = gpio_request(pdata->DVP_CTL_gpio, "DVP_CTL_gpio");
		 if (ret) 
            		printk("Failed_control to get %s gpio.\n", pdata->DVP_CTL_gpio);
        	else
		{
			gpio_direction_output(pdata->DVP_CTL_gpio,pdata->DVP_CTL_gpio_status);
			gpio_set_value(pdata->DVP_CTL_gpio,pdata->DVP_CTL_gpio_status);
		}
        }
}

static int  led_control_probe(struct platform_device *pdev)
{
	int ret = 0;
	int i;
	struct gpio_state *state;

	struct led_control_platform_data *pdata;//struct for io control

	state = kzalloc(sizeof(struct gpio_state), GFP_KERNEL);
	if (state == NULL) {
		ret = -ENOMEM;
		goto err;
	}	
	platform_set_drvdata(pdev, state);

	if (!pdata && pdev->dev.of_node) {
                pdata = led_control_parse_dt(&pdev->dev);
                if (IS_ERR(pdata))
                return -ENOMEM;
        }

	mpdata = pdata;
	led_control_setup_gpio(pdata);
        fb_register_client(&rockchip_led_fb_notifier);

	//wake_lock_init(&usb_wakelock, WAKE_LOCK_SUSPEND, "usb_detect_3G");      
        //wake_lock_timeout(&usb_wakelock, 2000000000000000* HZ);

	
	printk("%s\n",__FUNCTION__);
	return 0;
	
err:
	kfree(state);
	printk("Error gpio%s\n",__FUNCTION__);
	return ret;
}

int led_control_suspend(struct platform_device *pdev,  pm_message_t state)
{
        struct led_control_platform_data *pdata = platform_get_drvdata(pdev);
	gpio_direction_output(pdata->LED_CTL_gpio,!pdata->LED_CTL_gpio_status);
        gpio_set_value(pdata->LED_CTL_gpio,!pdata->LED_CTL_gpio_status);
	printk("%s\n",__FUNCTION__);
	return 0;	
}

int led_control_resume(struct platform_device *pdev)
{
	struct led_control_platform_data *pdata = platform_get_drvdata(pdev);
	gpio_direction_output(pdata->LED_CTL_gpio,pdata->LED_CTL_gpio_status);
        gpio_set_value(pdata->LED_CTL_gpio,pdata->LED_CTL_gpio_status);
	printk("%s\n",__FUNCTION__);
	return 0;
}

void led_control_shutdown(struct platform_device *pdev)
{
	printk("%s\n",__FUNCTION__);
	//misc_deregister(&led_control_gps_dev);
}

static const struct of_device_id led_control_dt_ids[] = {
	{ .compatible = "led_control" },
	{},
};

struct platform_driver led_control_driver = {
	.driver		= {
		.name	= "led_control",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(led_control_dt_ids),
	},
	.probe	  = led_control_probe,
	.suspend  = led_control_suspend,
	.resume   = led_control_resume,
	//.remove		= led_control_remove,
	.shutdown = led_control_shutdown,
};

static int __init led_control_init(void)
{
	int ret;
	printk("%s\n",__FUNCTION__);
	return platform_driver_register(&led_control_driver);
}

static void __exit led_control_exit(void)
{
	printk("%s\n",__FUNCTION__);
	platform_driver_unregister(&led_control_driver);
}

MODULE_DESCRIPTION("led_control driver");
MODULE_LICENSE("GPL");

module_init(led_control_init);
module_exit(led_control_exit);

