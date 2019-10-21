
#include <linux/vmalloc.h>
#include <linux/rk_fb.h>
#include <linux/device.h>
#include "lcd.h"
#include "../hdmi/rockchip-hdmi.h"

#define __DBG

#ifdef __DBG
#	define __srn_dbg(format, ...) do { printk(format, __VA_ARGS__); } while(0)
#else
#	define __srn_dbg(format, ...)
#endif


static struct rk_screen *rk_screen;
static char *local_boot_command_line = NULL;

/* support update the lcd param from sdcard */
#define LCD_PARAM_UPDATE_FROM_SDCARD

#if defined (LCD_PARAM_UPDATE_FROM_SDCARD)

extern char GetVendor0InfoBeforeNandInit(char * pbuf);

#endif

enum {
    SCREEN_TYPE_INDEX = 0,
    SCREEN_LVDS_FORMAT_INDEX,
    SCREEN_FACE_INDEX,
    SCREEN_X_RES_INDEX,
    SCREEN_Y_RES_INDEX,
    SCREEN_WIDTH_INDEX,
    SCREEN_HEIGHT_INDEX,

    SCREEN_LCDC_ACLK_INDEX,
    SCREEN_PIXCLOCK_INDEX,
    SCREEN_LEFT_MARGIN_INDEX,
    SCREEN_RIGHT_MARGIN_INDEX,
    SCREEN_HSYNC_LEN_INDEX,

    SCREEN_UPPER_MARGIN_INDEX,
    SCREEN_LOWER_MARGIN_INDEX,
    SCREEN_VSYNC_LEN_INDEX,

    SCREEN_PIN_VSYNC_INDEX,
    SCREEN_PIN_HSYNC_INDEX,
    SCREEN_PIN_DEN_INDEX,
    SCREEN_PIN_DCLK_INDEX,

    SCREEN_SWAP_RB_INDEX,
    SCREEN_SWAP_RG_INDEX,
    SCREEN_SWAP_GB_INDEX,
    SCREEN_SWAP_DELTA_INDEX,
    SCREEN_SWAP_DUMY_INDEX,
    SCREEN_PARAM_MAX,
};
static int lcdParam[SCREEN_PARAM_MAX] = {0};

int rk_fb_get_extern_screen(struct rk_screen *screen)
{
	if (unlikely(!rk_screen) || unlikely(!screen))
		return -1;

	memcpy(screen, rk_screen, sizeof(struct rk_screen));
	screen->dsp_lut = NULL;
	screen->cabc_lut = NULL;
	screen->type = SCREEN_NULL;

	return 0;
}
#if defined (LCD_PARAM_UPDATE_FROM_SDCARD)
int convert_lcd_param_to_screen(struct rk_screen *screen){
	
	 int result;
	 static char param_buf_temp[504];
	 int i ;

	  memset(param_buf_temp,0,504);
    GetVendor0InfoBeforeNandInit(param_buf_temp);
    for(i = 0; i < SCREEN_PARAM_MAX; i++)
    {
        lcdParam[i] = param_buf_temp[i * 4];
        lcdParam[i] =  (lcdParam[i] << 8) + param_buf_temp[i * 4 + 1];
        lcdParam[i] =  (lcdParam[i] << 8) + param_buf_temp[i * 4 + 2];
        lcdParam[i] =  (lcdParam[i] << 8) + param_buf_temp[i * 4 + 3];
        __srn_dbg("[clc] [%d] [%s] lcd_param %d\n", __LINE__, __FUNCTION__, lcdParam[i]);
    }
    if(lcdParam[3] > 0 && lcdParam[4] > 0 && lcdParam[8] > 0)
    {
				screen->type = lcdParam[SCREEN_TYPE_INDEX];
				screen->face = lcdParam[SCREEN_FACE_INDEX];
				screen->lvds_format = lcdParam[SCREEN_LVDS_FORMAT_INDEX];  //lvds data format
				
				screen->mode.xres = lcdParam[SCREEN_X_RES_INDEX];		//screen resolution
				screen->mode.yres = lcdParam[SCREEN_Y_RES_INDEX];
				
				screen->width = lcdParam[SCREEN_WIDTH_INDEX];
				screen->height = lcdParam[SCREEN_HEIGHT_INDEX];
				
				// LCDC_ACLK 为 LCDC 的 AXI 总线时钟，是 LCDC 通过 DMA 搬运 fb 中数据的时钟，
				// 这个在 RK30XX 以后的平台上，由 clock 管理代码统一锁定，外部驱动无法设置，所
				// 以这个宏定义多少都没关系。不过注意兼容老的驱动
				//screen->lcdc_aclk = lcdParam[SCREEN_LCDC_ACLK_INDEX]; // Timing
				screen->mode.pixclock = lcdParam[SCREEN_PIXCLOCK_INDEX];
				screen->mode.left_margin = lcdParam[SCREEN_LEFT_MARGIN_INDEX];
				screen->mode.right_margin = lcdParam[SCREEN_RIGHT_MARGIN_INDEX];
				screen->mode.hsync_len = lcdParam[SCREEN_HSYNC_LEN_INDEX];
				screen->mode.upper_margin = lcdParam[SCREEN_UPPER_MARGIN_INDEX];
				screen->mode.lower_margin = lcdParam[SCREEN_LOWER_MARGIN_INDEX];
				screen->mode.vsync_len = lcdParam[SCREEN_VSYNC_LEN_INDEX];
				
				screen->pin_hsync = lcdParam[SCREEN_PIN_HSYNC_INDEX]; //Pin polarity
				screen->pin_vsync = lcdParam[SCREEN_PIN_VSYNC_INDEX];
				screen->pin_den = lcdParam[SCREEN_PIN_DEN_INDEX];
				screen->pin_dclk = lcdParam[SCREEN_PIN_DCLK_INDEX];
				
				screen->swap_rb = lcdParam[SCREEN_SWAP_RB_INDEX]; // Swap rule
				screen->swap_rg = lcdParam[SCREEN_SWAP_RG_INDEX];
				screen->swap_gb = lcdParam[SCREEN_SWAP_GB_INDEX];
				screen->swap_delta = lcdParam[SCREEN_SWAP_DELTA_INDEX];
				screen->swap_dumy = lcdParam[SCREEN_SWAP_DUMY_INDEX];
				
	}else{
		
		result = -1;
		printk("read parameter failed!\n");
	}
	return result;
	
}
#endif
int  rk_fb_get_prmry_screen(struct rk_screen *screen)
{

	if (unlikely(!rk_screen) || unlikely(!screen))
		return -1;
	memcpy(screen, rk_screen, sizeof(struct rk_screen));

	return 0;
}

int rk_fb_set_prmry_screen(struct rk_screen *screen)
{
	if (unlikely(!rk_screen) || unlikely(!screen))
		return -1;

	rk_screen->lcdc_id = screen->lcdc_id;
	rk_screen->screen_id = screen->screen_id;
	rk_screen->x_mirror = screen->x_mirror;
	rk_screen->y_mirror = screen->y_mirror;
	rk_screen->overscan.left = screen->overscan.left;
	rk_screen->overscan.top = screen->overscan.left;
	rk_screen->overscan.right = screen->overscan.left;
	rk_screen->overscan.bottom = screen->overscan.left;
	return 0;
}

size_t get_fb_size(u8 reserved_fb)
{
	size_t size = 0;
	u32 xres = 0;
	u32 yres = 0;

	if (unlikely(!rk_screen))
		return 0;

	xres = rk_screen->mode.xres;
	yres = rk_screen->mode.yres;

	/* align as 64 bytes(16*4) in an odd number of times */
	xres = ALIGN_64BYTE_ODD_TIMES(xres, ALIGN_PIXEL_64BYTE_RGB8888);
  if (reserved_fb == 1) {
          size = (xres * yres << 2) << 1;/*two buffer*/
  } else {

	#if defined(CONFIG_THREE_FB_BUFFER)
		size = (xres * yres << 2) * 3;	/* three buffer */
	#else
		size = (xres * yres << 2) << 1; /* two buffer */
	#endif

	}
	return ALIGN(size, SZ_1M);
}

static int rk_screen_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int ret;

	if (!np) {
		dev_err(&pdev->dev, "Missing device tree node.\n");
		return -EINVAL;
	}
	rk_screen = devm_kzalloc(&pdev->dev,
			sizeof(struct rk_screen), GFP_KERNEL);
	if (!rk_screen) {
		dev_err(&pdev->dev, "kmalloc for rk screen fail!");
		return  -ENOMEM;
	}
	ret = rk_fb_prase_timing_dt(np, rk_screen);
	convert_lcd_param_to_screen(rk_screen);
	dev_info(&pdev->dev, "rockchip screen probe %s\n",
				ret ? "failed" : "success");
	return ret;
}

static const struct of_device_id rk_screen_dt_ids[] = {
	{ .compatible = "rockchip,screen", },
	{}
};

static struct platform_driver rk_screen_driver = {
	.probe		= rk_screen_probe,
	.driver		= {
		.name	= "rk-screen",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(rk_screen_dt_ids),
	},
};

static int __init rk_screen_init(void)
{
	return platform_driver_register(&rk_screen_driver);
}

static void __exit rk_screen_exit(void)
{
	if (NULL != local_boot_command_line)
	{
		vfree(local_boot_command_line);
	}

	platform_driver_unregister(&rk_screen_driver);
}

fs_initcall(rk_screen_init);
module_exit(rk_screen_exit);

