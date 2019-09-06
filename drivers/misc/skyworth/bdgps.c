/* 
 * drivers/misc/skyworth/gps-td1030.c - driver for td1030
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
#include "bdgps.h"

#define	MAX_SIZE(a)			sizeof(a)/sizeof(a[0])

#define BDGPS_ON                                0x4800
#define BDGPS_OFF                               0x4801

struct td1030_platform_data *pdata;//struct for io control

struct gpio_state {
	struct mutex		mutex; /* mutual excl. when accessing chip */
	void __iomem *base;
	int			irq;
	struct timer_list timer;
};


static long td1030_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	printk("%s cmd = %d arg = %ld\n",__FUNCTION__,cmd, arg);

	switch (cmd){
		case BDGPS_ON:
			gpio_direction_output(pdata->BD_EN_gpio,0);
				
			gpio_direction_output(pdata->BD_NRESET_gpio,0);
			gpio_direction_output(pdata->BD_NRESET_gpio,1);
			//usleep(5000);
			gpio_direction_output(pdata->BD_NRESET_gpio,0);
			break;
			
		case BDGPS_OFF:
                        break;
		default:
			break;
	}
	return 0;
}


static struct file_operations td1030_fops = {
	.owner   = THIS_MODULE,
	//.open    = NULL,
	//.read    = td1030_read,
	//.write   = td1030_write,
	.unlocked_ioctl   = td1030_ioctl,
	//.release = NULL,
};

static struct miscdevice td1030_misc = 
{
    .minor = MISC_DYNAMIC_MINOR,
    .name = "td1030",
    .fops = &td1030_fops,
};

#ifdef CONFIG_OF
static struct td1030_platform_data *td1030_parse_dt(struct device *dev)
{
        u32 temp;
	enum of_gpio_flags flags;
	int gpio,ret;
        struct td1030_platform_data *pdata;

        pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
        if (!pdata) {
                dev_err(dev, "memory allocation for td1030_data failed\n");
                return ERR_PTR(-ENOMEM);
        }

	if (of_find_property(dev->of_node, "BD_NRESET_gpio", NULL)) 
	{
        	gpio = of_get_named_gpio_flags(dev->of_node, "BD_NRESET_gpio", 0, &flags);
        	if (gpio_is_valid(gpio))
		{
            		pdata->BD_NRESET_gpio = gpio;
            		pdata->BD_NRESET_gpio_status = (flags == GPIO_ACTIVE_HIGH)? 1:0;
        	}
    	}
	if (of_find_property(dev->of_node, "BD_EN_gpio", NULL)) 
	{
        	gpio = of_get_named_gpio_flags(dev->of_node, "BD_EN_gpio", 0, &flags);
        	if (gpio_is_valid(gpio))
		{
            		pdata->BD_EN_gpio = gpio;
            		pdata->BD_EN_gpio_status = (flags == GPIO_ACTIVE_HIGH)? 1:0;
        	}
    	}
	if (of_find_property(dev->of_node, "BD_LAN_gpio", NULL)) 
	{
        	gpio = of_get_named_gpio_flags(dev->of_node, "BD_LAN_gpio", 0, &flags);
        	if (gpio_is_valid(gpio))
		{
            		pdata->BD_LAN_gpio = gpio;
            		pdata->BD_LAN_gpio_status = (flags == GPIO_ACTIVE_HIGH)? 1:0;
        	}
    	}
   return pdata;
}
#endif

static int td1030_setup_gpio(struct td1030_platform_data *pdata)
{
	int ret;

	if (gpio_is_valid(pdata->BD_NRESET_gpio))
	{
        	int ret = gpio_request(pdata->BD_NRESET_gpio, "BD_NRESET_gpio");
		 if (ret) 
            		printk("Failed to get %s gpio.\n", pdata->BD_NRESET_gpio);
        	else
			gpio_direction_output(pdata->BD_NRESET_gpio,pdata->BD_NRESET_gpio_status);
        }
	if (gpio_is_valid(pdata->BD_EN_gpio))
	{
        	int ret = gpio_request(pdata->BD_EN_gpio, "BD_EN_gpio");
		 if (ret) 
            		printk("Failed to get %s gpio.\n", pdata->BD_EN_gpio);
        	else
			gpio_direction_output(pdata->BD_EN_gpio,pdata->BD_EN_gpio_status);
        }
	if (gpio_is_valid(pdata->BD_LAN_gpio))
	{
        	int ret = gpio_request(pdata->BD_LAN_gpio, "BD_LAN_gpio");
		 if (ret) 
            		printk("Failed to get %s gpio.\n", pdata->BD_LAN_gpio);
        	else
			gpio_direction_output(pdata->BD_LAN_gpio,pdata->BD_LAN_gpio_status);
        }
}

static int  td1030_probe(struct platform_device *pdev)
{
	int ret = 0;
	int i;
	struct gpio_state *state;

	state = kzalloc(sizeof(struct gpio_state), GFP_KERNEL);
	if (state == NULL) {
		ret = -ENOMEM;
		goto err;
	}	
	platform_set_drvdata(pdev, state);

	if (!pdata && pdev->dev.of_node) {
                pdata = td1030_parse_dt(&pdev->dev);
                if (IS_ERR(pdata))
                return -ENOMEM;
        }

	td1030_setup_gpio(pdata);
	
	printk("%s\n",__FUNCTION__);
	return 0;
	
err:
	kfree(state);
	printk("Error gpio%s\n",__FUNCTION__);
	return ret;
}

int td1030_suspend(struct platform_device *pdev,  pm_message_t state)
{
  
	printk("%s\n",__FUNCTION__);
	return 0;	
}

int td1030_resume(struct platform_device *pdev)
{
  
	printk("%s\n",__FUNCTION__);
	return 0;
}

void td1030_shutdown(struct platform_device *pdev)
{
	printk("%s\n",__FUNCTION__);
	//misc_deregister(&td1030_gps_dev);
}

static const struct of_device_id td1030_dt_ids[] = {
	{ .compatible = "td1030" },
	{},
};

struct platform_driver td1030_driver = {
	.driver		= {
		.name	= "td1030",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(td1030_dt_ids),
	},
	.probe	  = td1030_probe,
	.suspend  = td1030_suspend,
	.resume   = td1030_resume,
	//.remove		= td1030_remove,
	.shutdown = td1030_shutdown,
};

static int __init td1030_init(void)
{
	int ret;
//	ret = misc_register(&td1030_misc);
	printk("%s\n",__FUNCTION__);
	return platform_driver_register(&td1030_driver);
}

static void __exit td1030_exit(void)
{
//	misc_deregister(&td1030_misc);
//	printk("%s\n",__FUNCTION__);
	platform_driver_unregister(&td1030_driver);
}

MODULE_DESCRIPTION("td1030 driver");
MODULE_LICENSE("GPL");

module_init(td1030_init);
module_exit(td1030_exit);

