/*
 * Platform Code for Unicron BBP controller
 *
 * Copyright (C) 2011 Texas Instruments, Inc. - http://www.ti.com/
 * Copyright (C) 2014 Truby Zong <truby.zong@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/i2c/at24.h>
#include <linux/phy.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/ethtool.h>
#include <linux/mfd/tps65217.h>
#include <linux/pwm_backlight.h>
#include <linux/input/ti_tsc.h>
#include <linux/input/lmsw.h>
#include <linux/input/pwr_button.h>
#include <linux/input/adxl34x.h>
#include <linux/platform_data/ti_adc.h>
#include <linux/mfd/ti_tscadc.h>
#include <linux/reboot.h>
#include <linux/pwm/pwm.h>
#include <linux/rtc/rtc-omap.h>
#include <linux/opp.h>
#include <linux/skbuff.h>
#include <linux/lis3lv02d.h>
#include <linux/w1-gpio.h>
#include <linux/i2c/ads1015.h>
#include <linux/stepper.h>

#include <video/da8xx-fb.h>

#include <mach/hardware.h>
#include <mach/board-am335xevm.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/hardware/asp.h>

#include <plat/omap_device.h>
#include <plat/omap-pm.h>
#include <plat/irqs.h>
#include <plat/board.h>
#include <plat/common.h>
#include <plat/lcdc.h>
#include <plat/usb.h>
#include <plat/mmc.h>
#include <plat/emif.h>
#include <plat/nand.h>
#include <plat/dma-33xx.h>

#include "board-flash.h"
#include "cpuidle33xx.h"
#include "mux.h"
#include "devices.h"
#include "hsmmc.h"

#include "control.h"
#include "common.h"

/* bit 3: 0 - enable, 1 - disable for pull enable */
#define AM33XX_PULL_DISA		(1 << 3)
#define AM33XX_PULL_ENBL		(0 << 3)

/* Convert GPIO signal to GPIO pin number */
#define GPIO_TO_PIN(bank, gpio) (32 * (bank) + (gpio))

struct evm_dev_cfg {
	void (*device_init)(int evm_id, int profile);

    /*
    * If the device is required on both baseboard & daughter board (ex i2c),
    * specify DEV_ON_BASEBOARD
    */
    #define DEV_ON_BASEBOARD	0
    #define DEV_ON_DGHTR_BRD	1
	u32 device_on;

	u32 profile;	/* Profiles (0-7) in which the module is present */
};

static u32 am335x_evm_id;
static struct omap_board_config_kernel am335x_evm_config[] __initdata = {
};
/*
* EVM Config held in On-Board eeprom device.
*
* Header Format
*
*  Name			Size	Contents
*			(Bytes)
*-------------------------------------------------------------
*  Header		 4	0xAA, 0x55, 0x33, 0xEE
*
*  Board Name	 8	Name for board in ASCII.
*				 Example "A33515BB" = "AM335x 15x15 Base Board"
*
*  Version		 4	Hardware version code for board	in ASCII.
*				 "1.0A" = rev.01.0A
*
*  Serial Number 12	Serial number of the board. This is a 12
*				 character string which is WWYY4P16nnnn, where
*				 WW = 2 digit week of the year of production
*				 YY = 2 digit year of production
*				 nnnn = incrementing board number
*
*  Configuration option	32 Codes(TBD) to show the configuration
*				 setup on this board.
*
*  Available	 32720	Available space for other non-volatile data.
*/
struct am335x_evm_eeprom_config {
	u32	 header;
	u8	 name[8];
	char version[4];
	u8	 serial[12];
	u8	 opt[32];
};

static void setup_fastbot_bbp1(void);
static void setup_fastbot_bbp1s(void);
static void am335x_evm_setup(struct memory_accessor *mem_acc, void *context);

static struct am335x_evm_eeprom_config config;

static bool daughter_brd_detected;

#define EEPROM_MAC_ADDRESS_OFFSET	60 /* 4+8+4+12+32 */
#define EEPROM_NO_OF_MAC_ADDR		3
static char am335x_mac_addr[EEPROM_NO_OF_MAC_ADDR][ETH_ALEN];

#define AM335X_EEPROM_HEADER		0xEE3355AA

enum { BBP1=1, BBP1S=2, BBP_UNKNOW};
int bbp_board_type = BBP1;

int get_board_type()
{
	return bbp_board_type;
}

static void get_eeprom_board_type()
{
	int ret = 0; 
	if(!strncasecmp("bbp1s", config.name, 5)) {
		ret = BBP1S;
	} else if(!strncasecmp("bbp1", config.name, 4)){
		ret = BBP1;
	}

	bbp_board_type = ret;
	printk("board_type:0x%x\n", ret);
}

static int am33xx_evmid = -EINVAL;

/*
* am335x_evm_set_id - set up board evmid
* @evmid - evm id which needs to be configured
*
* This function is called to configure board evm id.
*/
void am335x_evm_set_id(unsigned int evmid)
{
	am33xx_evmid = evmid;
	return;
}
/*
* am335x_evm_get_id - returns Board Type (EVM/BB/EVM-SK ...)
*
* Note:
*	returns -EINVAL if Board detection hasn't happened yet.
*/
int am335x_evm_get_id(void)
{
	return am33xx_evmid;
}
EXPORT_SYMBOL(am335x_evm_get_id);
/*
* @evm_id - evm id which needs to be configured
* @dev_cfg - single evm structure which includes
*				all module inits, pin-mux defines
* @profile - if present, else PROFILE_NONE
* @dghtr_brd_flg - Whether Daughter board is present or not
*/
static void _configure_device(int evm_id, struct evm_dev_cfg *dev_cfg,
	int profile)
{
	int i;

	am335x_evm_set_id(evm_id);

	/*
	* Only General Purpose & Industrial Auto Motro Control
	* EVM has profiles. So check if this evm has profile.
	* If not, ignore the profile comparison
	*/

	/*
	* If the device is on baseboard, directly configure it. Else (device on
	* Daughter board), check if the daughter card is detected.
	*/
	if (profile == PROFILE_NONE) {
		for (i = 0; dev_cfg->device_init != NULL; dev_cfg++) {
			if (dev_cfg->device_on == DEV_ON_BASEBOARD)
				dev_cfg->device_init(evm_id, profile);
			else if (daughter_brd_detected == true)
				dev_cfg->device_init(evm_id, profile);
		}
	} else {
		for (i = 0; dev_cfg->device_init != NULL; dev_cfg++) {
			if (dev_cfg->profile & profile) {
				if (dev_cfg->device_on == DEV_ON_BASEBOARD)
					dev_cfg->device_init(evm_id, profile);
				else if (daughter_brd_detected == true)
					dev_cfg->device_init(evm_id, profile);
			}
		}
	}
}

/* module pin mux structure */
struct pinmux_config {
	const char *string_name; /* signal name format */
	int val;                 /* Options for the mux register value */
};

/*
 * @pin_mux - single module pin-mux structure which defines pin-mux
 *			  details for all its pins.
 */
static void setup_pin_mux(struct pinmux_config *pin_mux)
{
	int i;

	for (i = 0; pin_mux->string_name != NULL; pin_mux++) {
		omap_mux_init_signal(pin_mux->string_name, pin_mux->val);
    }
}
/*----------------------------------------------------------- 
 * W1-gpio for DS18B20 
 -----------------------------------------------------------*/
#define UNICRON_W1_GPIO GPIO_TO_PIN(1, 6)

static struct w1_gpio_platform_data w1_gpio_pdata = {
	.pin		   = UNICRON_W1_GPIO,
	.is_open_drain = 0,
};

static struct platform_device bone_w1_device = {
	.name		   = "w1-gpio",
	.id			   = -1,
	.dev.platform_data = &w1_gpio_pdata,
};

static void w1_gpio_init(int evm_id, int profile)
{
	int err;
	err = platform_device_register(&bone_w1_device);
	if (err) {
		pr_err("failed to register w1-gpio\n");
    } else {
		pr_info("w1-gpio connected to P8_3\n");
    }
}
/*----------------------------------------------------------- 
 * PWM
 * Heater: gpio0_2    heater_ext1    EHRPWM0A
 *         gpio0_3    heater_ext2    EHRPWM0B
 *         gpio1_18   heater_hbp     EHRPWM1A
 * Fans:   gpio1_19   fan1           EHRPWM1B
 *         gpio0_7    fan2           ECAP0_IN_PWM0_OUT
 -----------------------------------------------------------*/
/* pin mux for ecap0 */
static struct pinmux_config ecap0_pin_mux[] = {
	{"ecap0_in_pwm0_out.ecap0_in_pwm0_out", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

/* pin mux for ehrpwm0 */
static struct pinmux_config ehrpwm0_pin_mux[] = {
	{"spi0_sclk.ehrpwm0A", OMAP_MUX_MODE3 | AM33XX_PIN_OUTPUT},
	{"spi0_d0.ehrpwm0B",   OMAP_MUX_MODE3 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

/* pin mux for ehrpwm1 */
static struct pinmux_config ehrpwm1A_pin_mux[] = {
	{"gpmc_a2.ehrpwm1A", OMAP_MUX_MODE6 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

static struct pinmux_config ehrpwm1B_pin_mux[] = {
	{"gpmc_a3.ehrpwm1B", OMAP_MUX_MODE6 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

static struct pwmss_platform_data pwm_pdata[6] = {
    /* ecap0 */
	{
		.version = PWM_VERSION_1,
	},
    /* ecap1 */
	{
		.version = PWM_VERSION_1,
	},
    /* ecap2 */
	{
		.version = PWM_VERSION_1,
	},
    /* ehrpwm0 */
	{
		.version = PWM_VERSION_1,
	},
    /* ehrpwm1 */
	{
		.version = PWM_VERSION_1,
	},
    /* ehrpwm2 */
	{
		.version = PWM_VERSION_1,
	},
};

static void ecap0_init(int evm_id, int profile)
{
	setup_pin_mux(ecap0_pin_mux);
    am33xx_register_ecap(0, &pwm_pdata[0]);
}

static void ehrpwm0_init(int evm_id, int profile)
{
	setup_pin_mux(ehrpwm0_pin_mux);

	pwm_pdata[3].chan_attrib[0].max_freq = 4096;
	pwm_pdata[3].chan_attrib[0].inverse_pol = false;

	pwm_pdata[3].chan_attrib[1].max_freq = 4096;
	pwm_pdata[3].chan_attrib[1].inverse_pol = false;

	am33xx_register_ehrpwm(0, &pwm_pdata[3]);
}

static void ehrpwm1_init(int evm_id, int profile)
{
    if (bbp_board_type == BBP1S) {
	    setup_pin_mux(ehrpwm1B_pin_mux);
    } else {
	    setup_pin_mux(ehrpwm1A_pin_mux);
	    setup_pin_mux(ehrpwm1B_pin_mux);
    }

	pwm_pdata[4].chan_attrib[0].max_freq = 4096;
	pwm_pdata[4].chan_attrib[0].inverse_pol = false;

    if (bbp_board_type == BBP1S) {
        pwm_pdata[4].chan_attrib[1].max_freq = 4096;
        pwm_pdata[4].chan_attrib[1].inverse_pol = true;
    } else {
        pwm_pdata[4].chan_attrib[1].max_freq = 4096;
        pwm_pdata[4].chan_attrib[1].inverse_pol = false;
    }

	am33xx_register_ehrpwm(1, &pwm_pdata[4]);
}
/*----------------------------------------------------------- 
 * Fan&Light control (BBP 1 only)
 * Enable or disable fan&light power
 * Fan3 : gpio1_9
 * Fan4 : gpio1_8
 * Fan5 : gpio0_14
 -----------------------------------------------------------*/
static struct pinmux_config fan_pin_mux[] = {
	{"uart0_rtsn.gpio1_9", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
	{"uart0_ctsn.gpio1_8", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
	{"uart1_rxd.gpio0_14", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

#define FAN3_GPIO     GPIO_TO_PIN(1, 9)
#define FAN4_GPIO     GPIO_TO_PIN(1, 8)
#define FAN5_GPIO     GPIO_TO_PIN(0, 14)
static void fan_init(int evm_id, int profile)
{
	setup_pin_mux(fan_pin_mux);
}
/*----------------------------------------------------------- 
 * power button
 -----------------------------------------------------------*/
#define PWR_IRQ_GPIO     GPIO_TO_PIN(3, 16)
static struct pinmux_config pwr_button_pin_mux[] = {
	{"mcasp0_axr0.gpio3_16", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLDOWN},
	{NULL, 0},
};

static int pwr_button_gpio_init(void)
{
	if (gpio_request(PWR_IRQ_GPIO, "irq") < 0) {
		printk(KERN_ERR "Failed to request GPIO%d for POWER BUTTON IRQ\n", 
                PWR_IRQ_GPIO);
		return -ENXIO;
	}
	gpio_direction_input(PWR_IRQ_GPIO);

    return 0;
}

static void pwr_button_gpio_exit(void)
{
    gpio_free(PWR_IRQ_GPIO);
}

static int pwr_button_get_status(void)
{
    return gpio_get_value(PWR_IRQ_GPIO) ? 0 : 1;
}

static struct pwr_button_platform_data pwr_button_data = {
    .irq        = OMAP_GPIO_IRQ(PWR_IRQ_GPIO),
    .init       = pwr_button_gpio_init,
    .exit       = pwr_button_gpio_exit,
    .get_status = pwr_button_get_status,
};

static struct platform_device pwr_button_device = {
	.name	= "pwr_button",
	.id	    = -1,
	.dev	= {
		.platform_data	= &pwr_button_data,
	},
};

static void pwr_button_init(int evm_id, int profile)
{
	setup_pin_mux(pwr_button_pin_mux);

    platform_device_register(&pwr_button_device);
}
/*----------------------------------------------------------- 
 * LMSW
 * min_x -> gpio2_1     
 * min_y -> gpio1_17
 * min_z -> gpio0_31
 * max_x -> gpio0_30
 * max_y -> gpio1_29
 * max_z -> gpio0_15
 -----------------------------------------------------------*/
#define MIN_X_GPIO     GPIO_TO_PIN(2, 1)
#define MIN_Y_GPIO     GPIO_TO_PIN(1, 17)
#define MIN_Z_GPIO     GPIO_TO_PIN(0, 31)

#define MAX_X_GPIO     GPIO_TO_PIN(0, 30)
#define MAX_Y_GPIO     GPIO_TO_PIN(1, 29)
#define MAX_Z_GPIO     GPIO_TO_PIN(0, 15)
#define AUTOLEVEL_Z_GPIO	GPIO_TO_PIN(0, 14)

static struct pinmux_config lmsw_pin_mux[] = {
	{"gpmc_clk.gpio2_1",    OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_a1.gpio1_17",    OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_wpn.gpio0_31",   OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},

	{"gpmc_wait0.gpio0_30", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_csn0.gpio1_29",  OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
	{"uart1_txd.gpio0_15",  OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},

	{NULL, 0},
};

static struct lmsw_platform_data min_x_data = {
    .gpio  = MIN_X_GPIO,
    .level = 0,
    .value = KEY_F1,
};

static struct lmsw_platform_data min_y_data = {
    .gpio  = MIN_Y_GPIO,
    .level = 0,
    .value = KEY_F2,
};

static struct lmsw_platform_data min_z_data = {
    .gpio  = MIN_Z_GPIO,
    .level = 0,
    .value = KEY_F3,
};

static struct lmsw_platform_data max_x_data = {
    .gpio  = MAX_X_GPIO,
    .level = 0,
    .value = KEY_F4,
};

static struct lmsw_platform_data max_y_data = {
    .gpio  = MAX_Y_GPIO,
    .level = 0,
    .value = KEY_F5,
};

static struct lmsw_platform_data max_z_data = {
    .gpio  = MAX_Z_GPIO,
    .level = 0,
    .value = KEY_F6,
};

static struct platform_device min_x_device = {
    .name = "lmsw_min_x",
    .id   = -1,
    .dev = {
        .platform_data = &min_x_data,
    },
};

static struct platform_device min_y_device = {
    .name = "lmsw_min_y",
    .id   = -1,
    .dev = {
        .platform_data = &min_y_data,
    },
};

static struct platform_device min_z_device = {
    .name = "lmsw_min_z",
    .id   = -1,
    .dev = {
        .platform_data = &min_z_data,
    },
};

static struct platform_device max_x_device = {
    .name = "lmsw_max_x",
    .id   = -1,
    .dev = {
        .platform_data = &max_x_data,
    },
};

static struct platform_device max_y_device = {
    .name = "lmsw_max_y",
    .id   = -1,
    .dev = {
        .platform_data = &max_y_data,
    },
};

static struct platform_device max_z_device = {
    .name = "lmsw_max_z",
    .id   = -1,
    .dev = {
        .platform_data = &max_z_data,
    },
};

static void lmsw_init(int evm_id, int profile)
{
	setup_pin_mux(lmsw_pin_mux);

    platform_device_register(&min_x_device);
    platform_device_register(&min_y_device);
    platform_device_register(&min_z_device);
    platform_device_register(&max_x_device);
    platform_device_register(&max_y_device);
    platform_device_register(&max_z_device);
}

static int lmsw_check_min_x(void)
{
    return gpio_get_value(MIN_X_GPIO) ? 0 : 1;
}

static int lmsw_check_min_y(void)
{
    return gpio_get_value(MIN_Y_GPIO) ? 0 : 1;
}

static int lmsw_check_min_z(void)
{
    return gpio_get_value(MIN_Z_GPIO) ? 0 : 1;
}

static int lmsw_check_max_x(void)
{
    return gpio_get_value(MAX_X_GPIO) ? 0 : 1;
}

static int lmsw_check_max_y(void)
{
    return gpio_get_value(MAX_Y_GPIO) ? 0 : 1;
}

static int lmsw_check_max_z(void)
{
    return gpio_get_value(MAX_Z_GPIO) ? 0 : 1;
}

static int check_autoLevel_z(void)
{
    return gpio_get_value(AUTOLEVEL_Z_GPIO) ? 0 : 1;
}
/*----------------------------------------------------------- 
 * Stepper Motor
 * step_x    -> gpio1_21  BBP 1/1S
 * step_y    -> gpio1_22  BBP 1/1S
 * step_z    -> gpio1_23  BBP 1/1S
 * step_ext1 -> gpio1_28  BBP 1/1S
 * step_ext2 -> gpio1_24  BBP 1/1S
 * step_ext3 -> gpio1_25  BBP 1S
 * step_user -> gpio1_27  BBP 1S
 * step_exp  -> gpio1_18  BBP 1S
 *
 * dir_x     -> gpio2_0   BBP 1/1S
 * dir_y     -> gpio2_4   BBP 1/1S
 * dir_z     -> gpio2_3   BBP 1/1S
 * dir_ext1  -> gpio2_2   BBP 1/1S
 * dir_ext2  -> gpio2_5   BBP 1/1S
 * dir_ext3  -> gpio3_7   BBP 1S
 * dir_user  -> gpio3_8   BBP 1S
 * dir_exp   -> gpio0_20  BBP 1S
 *
 * fault_x   -> gpio0_20  BBP 1
 * fault_y   -> gpio3_7   BBP 1
 * fault_z   -> gpio3_8   BBP 1
 * fault_ext1-> gpio1_25  BBP 1
 * fault_ext2-> gpio1_27  BBP 1
 -----------------------------------------------------------*/
#define STEP_X_GPIO     GPIO_TO_PIN(1, 21)
#define STEP_Y_GPIO     GPIO_TO_PIN(1, 22)
#define STEP_Z_GPIO     GPIO_TO_PIN(1, 23)
#define STEP_EXT1_GPIO  GPIO_TO_PIN(1, 28)
#define STEP_EXT2_GPIO  GPIO_TO_PIN(1, 24)
#define STEP_EXT3_GPIO  GPIO_TO_PIN(1, 25)
#define STEP_USER_GPIO  GPIO_TO_PIN(1, 27)
#define STEP_EXP_GPIO   GPIO_TO_PIN(1, 18)

#define DIR_X_GPIO      GPIO_TO_PIN(2, 0)
#define DIR_Y_GPIO      GPIO_TO_PIN(2, 4)
#define DIR_Z_GPIO      GPIO_TO_PIN(2, 3)
#define DIR_EXT1_GPIO   GPIO_TO_PIN(2, 2)
#define DIR_EXT2_GPIO   GPIO_TO_PIN(2, 5)
#define DIR_EXT3_GPIO   GPIO_TO_PIN(3, 7)
#define DIR_USER_GPIO   GPIO_TO_PIN(3, 8)
#define DIR_EXP_GPIO    GPIO_TO_PIN(0, 20)

#define FAULT_X_GPIO    GPIO_TO_PIN(0, 20)
#define FAULT_Y_GPIO    GPIO_TO_PIN(3, 7)
#define FAULT_Z_GPIO    GPIO_TO_PIN(3, 8)
#define FAULT_EXT1_GPIO GPIO_TO_PIN(1, 25)
#define FAULT_EXT2_GPIO GPIO_TO_PIN(1, 27)

/* 12V power enable */
#define PWR_EN_GPIO     GPIO_TO_PIN(0, 12)

/* pin mux for stepper motor */
static struct pinmux_config stepper_pin_mux_bbp1[] = {
	{"gpmc_a5.gpio1_21",      OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT}, //step x
	{"gpmc_a6.gpio1_22",      OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT}, //step y
	{"gpmc_a7.gpio1_23",      OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT}, //step z
	{"gpmc_ben1.gpio1_28",    OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT}, //step ext1
	{"gpmc_a8.gpio1_24",      OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT}, //step ext2

	{"gpmc_csn3.gpio2_0",     OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT}, //dir x
	{"gpmc_wen.gpio2_4",      OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT}, //dir y
	{"gpmc_oen_ren.gpio2_3",  OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT}, //dir z
	{"gpmc_advn_ale.gpio2_2", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT}, //dir ext1
	{"gpmc_ben0_cle.gpio2_5", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT}, //dir ext2
    
    {"xdma_event_intr1.gpio0_20",  OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP}, //fault x
    {"emu0.gpio3_7",          OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},      //fault y
    {"emu1.gpio3_8",          OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},      //fault z
    {"gpmc_a9.gpio1_25",      OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},      //fault ext1
    {"gpmc_a11.gpio1_27",     OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},      //fault ext2

	{"uart1_ctsn.gpio0_12",   OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT}, //12V-EN

	{NULL, 0},
};

static struct pinmux_config stepper_pin_mux_bbp1s[] = {
	{"gpmc_a5.gpio1_21",      OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},      //step x
	{"gpmc_a6.gpio1_22",      OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},      //step y
	{"gpmc_a7.gpio1_23",      OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},      //step z
	{"gpmc_ben1.gpio1_28",    OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},      //step ext1
	{"gpmc_a8.gpio1_24",      OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},      //step ext2
    {"gpmc_a9.gpio1_25",      OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},      //step ext3
    {"gpmc_a11.gpio1_27",     OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},      //step user
	{"gpmc_a2.gpio1_18",      OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},      //step exp

	{"gpmc_csn3.gpio2_0",     OMAP_MUX_MODE3 | AM33XX_PIN_OUTPUT},      //dir x
	{"gpmc_wen.gpio2_4",      OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT},      //dir y
	{"gpmc_oen_ren.gpio2_3",  OMAP_MUX_MODE3 | AM33XX_PIN_OUTPUT},      //dir z
	{"gpmc_advn_ale.gpio2_2", OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT},      //dir ext1
	{"gpmc_ben0_cle.gpio2_5", OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT},      //dir ext2
    {"emu0.gpio3_7",          OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},      //dir ext3
    {"emu1.gpio3_8",          OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},      //dir user
    {"xdma_event_intr1.gpio0_20",  OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT}, //dir exp

	{"uart1_ctsn.gpio0_12",   OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},      //12V-EN

	{NULL, 0},
};

static void stepper_init(int evm_id, int profile)
{
    if (bbp_board_type == BBP1) {
	    setup_pin_mux(stepper_pin_mux_bbp1);
    } else if(bbp_board_type == BBP1S) {
	    setup_pin_mux(stepper_pin_mux_bbp1s);
    }

    if(bbp_board_type == BBP1S) {
     	gpio_request(14, "autoLevel"); //gpio0_14;
    	gpio_direction_output(14, 1);
    }

    /* stepper step gpios */
    gpio_request(STEP_X_GPIO, "step x");
    gpio_direction_output(STEP_X_GPIO, 0);

    gpio_request(STEP_Y_GPIO, "step y");
    gpio_direction_output(STEP_Y_GPIO, 0);

    gpio_request(STEP_Z_GPIO, "step z");
    gpio_direction_output(STEP_Z_GPIO, 0);

    gpio_request(STEP_EXT1_GPIO, "step ext1");
    gpio_direction_output(STEP_EXT1_GPIO, 0);

    gpio_request(STEP_EXT2_GPIO, "step ext2");
    gpio_direction_output(STEP_EXT2_GPIO, 0);

    if (bbp_board_type == BBP1S) {
        gpio_request(STEP_EXT3_GPIO, "step ext3");
        gpio_direction_output(STEP_EXT3_GPIO, 0);

        gpio_request(STEP_USER_GPIO, "step user");
        gpio_direction_output(STEP_USER_GPIO, 0);

        gpio_request(STEP_EXP_GPIO, "step exp");
        gpio_direction_output(STEP_EXP_GPIO, 0);
    }

    /* stepper dir gpios */
    gpio_request(DIR_X_GPIO, "dir x");
    gpio_direction_output(DIR_X_GPIO, 0);

    gpio_request(DIR_Y_GPIO, "dir y");
    gpio_direction_output(DIR_Y_GPIO, 0);

    gpio_request(DIR_Z_GPIO, "dir z");
    gpio_direction_output(DIR_Z_GPIO, 0);

    gpio_request(DIR_EXT1_GPIO, "dir ext1");
    gpio_direction_output(DIR_EXT1_GPIO, 0);

    gpio_request(DIR_EXT2_GPIO, "dir ext2");
    gpio_direction_output(DIR_EXT2_GPIO, 0);
    
    if (bbp_board_type == BBP1S) {
        gpio_request(DIR_EXT3_GPIO, "dir ext3");
        gpio_direction_output(DIR_EXT3_GPIO, 0);

        gpio_request(DIR_USER_GPIO, "dir user");
        gpio_direction_output(DIR_USER_GPIO, 0);

        gpio_request(DIR_EXP_GPIO, "dir exp");
        gpio_direction_output(DIR_EXP_GPIO, 0);
    }

    if (bbp_board_type == BBP1) {
        /* stepper  fault gpios */
        gpio_request(FAULT_X_GPIO, "fault x");
        gpio_direction_input(FAULT_X_GPIO);

        gpio_request(FAULT_Y_GPIO, "fault y");
        gpio_direction_input(FAULT_X_GPIO);

        gpio_request(FAULT_Z_GPIO, "fault z");
        gpio_direction_input(FAULT_X_GPIO);

        gpio_request(FAULT_EXT1_GPIO, "fault ext1");
        gpio_direction_input(FAULT_EXT1_GPIO);

        gpio_request(FAULT_EXT2_GPIO, "fault ext2");
        gpio_direction_input(FAULT_EXT2_GPIO);
    }

    /* 12V-EN */
    gpio_request(PWR_EN_GPIO, "12V en");
    gpio_direction_output(PWR_EN_GPIO, 1);
}
/*----------------------------------------------------------- 
 * SPI
 *
 *         --> hc595 -> drv8825 cs1 (stepper motor mode control)
 * spi1 --| 
 *         --> dac088           cs0 (DAC register setting)
 *
 * spi1_sclk -> gpio3_14
 * spi1_do   -> gpio3_15
 * spi1_d1   -> gpio3_16
 * spi1_cs0  -> gpio3_17
 * spi1_cs1  -> gpio0_13
 *
 * spi0 --> max6675 (temperature sensor, BBP 1S only)
 *
 -----------------------------------------------------------*/
/* pin mux for SPI0 */
static struct pinmux_config spi0_pin_mux[] = {
#if 0 
	{"spi0_sclk.spi0_sclk", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"spi0_d0.spi0_d0",     OMAP_MUX_MODE0 | AM33XX_INPUT_EN},
	{"spi0_d1.spi0_d1",     OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"spi0_cs0.spi0_cs0",   OMAP_MUX_MODE0 | AM33XX_PULL_UP 
                            | AM33XX_PIN_OUTPUT},
#else
    {"spi0_sclk.spi0_sclk", OMAP_MUX_MODE0 | AM33XX_PULL_ENBL
                            | AM33XX_INPUT_EN},       
    {"spi0_d0.spi0_d0",     OMAP_MUX_MODE0 | AM33XX_PULL_ENBL | AM33XX_PULL_UP
                            | AM33XX_INPUT_EN},       
    {"spi0_d1.spi0_d1", OMAP_MUX_MODE0 | AM33XX_PULL_ENBL
                            | AM33XX_INPUT_EN},       
    {"spi0_cs0.spi0_cs0", OMAP_MUX_MODE0 | AM33XX_PULL_ENBL | AM33XX_PULL_UP
                            | AM33XX_INPUT_EN},   
#endif
	{NULL, 0},
};

/* pin mux for SPI1 */
static struct pinmux_config spi1_pin_mux[] = {
	{"mcasp0_aclkx.spi1_sclk", OMAP_MUX_MODE3 | AM33XX_PIN_OUTPUT},
	{"mcasp0_fsx.spi1_d0",     OMAP_MUX_MODE3 | AM33XX_INPUT_EN},
	{"mcasp0_axr0.spi1_d1",    OMAP_MUX_MODE3 | AM33XX_PIN_OUTPUT},
	{"mcasp0_ahclkr.spi1_cs0", OMAP_MUX_MODE3 | AM33XX_PULL_UP 
                               | AM33XX_PIN_OUTPUT},
	{"uart1_rtsn.spi1_cs1",    OMAP_MUX_MODE4 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

static struct stepper_info stepper_infos_bbp1[] = {
    {
        .name  = "stepper_x",
        .step  = STEP_X_GPIO,
        .dir   = DIR_X_GPIO,
        .fault = FAULT_X_GPIO,
    },
    {
        .name  = "stepper_y",
        .step  = STEP_Y_GPIO,
        .dir   = DIR_Y_GPIO,
        .fault = FAULT_Y_GPIO,
    },
    {
        .name  = "stepper_z",
        .step  = STEP_Z_GPIO,
        .dir   = DIR_Z_GPIO,
        .fault = FAULT_Z_GPIO,
    },
    {
        .name  = "stepper_ext1",
        .step  = STEP_EXT1_GPIO,
        .dir   = DIR_EXT1_GPIO,
        .fault = FAULT_EXT1_GPIO,
    },
    {
        .name  = "stepper_ext2",
        .step  = STEP_EXT2_GPIO,
        .dir   = DIR_EXT2_GPIO,
        .fault = FAULT_EXT2_GPIO,
    },
};

/* Stepper Motor platform data */
static struct stepper_info stepper_infos_bbp1s[] = {
    {
        .name  = "stepper_x",
        .step  = STEP_X_GPIO,
        .dir   = DIR_X_GPIO,
    },
    {
        .name  = "stepper_y",
        .step  = STEP_Y_GPIO,
        .dir   = DIR_Y_GPIO,
    },
    {
        .name  = "stepper_z",
        .step  = STEP_Z_GPIO,
        .dir   = DIR_Z_GPIO,
    },
    {
        .name  = "stepper_ext1",
        .step  = STEP_EXT1_GPIO,
        .dir   = DIR_EXT1_GPIO,
    },
    {
        .name  = "stepper_ext2",
        .step  = STEP_EXT2_GPIO,
        .dir   = DIR_EXT2_GPIO,
    },
    {
        .name  = "stepper_ext3",
        .step  = STEP_EXT3_GPIO,
        .dir   = DIR_EXT3_GPIO,
    },
    {
        .name  = "stepper_user",
        .step  = STEP_USER_GPIO,
        .dir   = DIR_USER_GPIO,
    },
    {
        .name  = "stepper_exp",
        .step  = STEP_EXP_GPIO,
        .dir   = DIR_EXP_GPIO,
    },
};

static struct stepper_platform_data stepper_pdata_bbp1 = {
    .steppers    = stepper_infos_bbp1,
    .nsteppers   = ARRAY_SIZE(stepper_infos_bbp1),
    .check_min_x = lmsw_check_min_x,
    .check_min_y = lmsw_check_min_y,
    .check_min_z = lmsw_check_min_z,
    .check_max_x = lmsw_check_max_x,
    .check_max_y = lmsw_check_max_y,
    .check_max_z = lmsw_check_max_z,
};

static struct stepper_platform_data stepper_pdata_bbp1s = {
    .steppers    = stepper_infos_bbp1s,
    .nsteppers   = ARRAY_SIZE(stepper_infos_bbp1s),
    .check_min_x = lmsw_check_min_x,
    .check_min_y = lmsw_check_min_y,
    .check_min_z = lmsw_check_min_z,
    .check_max_x = lmsw_check_max_x,
    .check_max_y = lmsw_check_max_y,
    .check_max_z = lmsw_check_max_z,
    .check_autoLevel_z = check_autoLevel_z,
};

/* DAC088 voltage regulator support */
static struct regulator_consumer_supply dac088_vouta_consumers[] = {
    {
        .supply = "vref_x",
    },
};

static struct regulator_consumer_supply dac088_voutb_consumers[] = {
    {
        .supply = "vref_y",
    },
};

static struct regulator_consumer_supply dac088_voutc_consumers[] = {
    {
        .supply = "vref_z",
    },
};

static struct regulator_consumer_supply dac088_voutd_consumers[] = {
    {
        .supply = "vref_ext1",
    },
};

static struct regulator_consumer_supply dac088_voute_consumers[] = {
    {
        .supply = "vref_ext2",
    },
};

static struct regulator_consumer_supply dac088_voutf_consumers[] = {
    {
        .supply = "vref_ext3",
    },
};

static struct regulator_consumer_supply dac088_voutg_consumers[] = {
    {
        .supply = "vref_ext4",
    },
};

static struct regulator_consumer_supply dac088_vouth_consumers[] = {
    {
        .supply = "vref_ext5",
    },
};

static struct regulator_init_data dac088_regulator_data[] = {
	/* VoutA -> VREF_X */
	{
		.constraints = {
			.min_uV = 10000,
			.max_uV = 3300000,
			.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE |
				               REGULATOR_CHANGE_STATUS),
			.boot_on = 0,
			.always_on = 0,
		},
		.num_consumer_supplies = ARRAY_SIZE(dac088_vouta_consumers),
		.consumer_supplies = dac088_vouta_consumers,
	},
	/* VoutB -> VREF_Y */
	{
		.constraints = {
			.min_uV = 10000,
			.max_uV = 3300000,
			.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE |
				               REGULATOR_CHANGE_STATUS),
			.boot_on = 0,
			.always_on = 0,
		},
		.num_consumer_supplies = ARRAY_SIZE(dac088_voutb_consumers),
		.consumer_supplies = dac088_voutb_consumers,
	},
	/* VoutC -> VREF_Z */
	{
		.constraints = {
			.min_uV = 10000,
			.max_uV = 3300000,
			.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE |
				               REGULATOR_CHANGE_STATUS),
			.boot_on = 0,
			.always_on = 0,
		},
		.num_consumer_supplies = ARRAY_SIZE(dac088_voutc_consumers),
		.consumer_supplies = dac088_voutc_consumers,
	},
	/* VoutD -> VREF_EXT1 */
	{
		.constraints = {
			.min_uV = 10000,
			.max_uV = 3300000,
			.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE |
				               REGULATOR_CHANGE_STATUS),
			.boot_on = 0,
			.always_on = 0,
		},
		.num_consumer_supplies = ARRAY_SIZE(dac088_voutd_consumers),
		.consumer_supplies = dac088_voutd_consumers,
	},
	/* VoutE -> VREF_EXT2 */
	{
		.constraints = {
			.min_uV = 10000,
			.max_uV = 3300000,
			.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE |
				               REGULATOR_CHANGE_STATUS),
			.boot_on = 0,
			.always_on = 0,
		},
		.num_consumer_supplies = ARRAY_SIZE(dac088_voute_consumers),
		.consumer_supplies = dac088_voute_consumers,
	},
	/* VoutF -> VREF_EXT3 */
	{
		.constraints = {
			.min_uV = 10000,
			.max_uV = 3300000,
			.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE |
				               REGULATOR_CHANGE_STATUS),
			.boot_on = 0,
			.always_on = 0,
		},
		.num_consumer_supplies = ARRAY_SIZE(dac088_voutf_consumers),
		.consumer_supplies = dac088_voutf_consumers,
	},
	/* VoutG -> VREF_EXT4 */
	{
		.constraints = {
			.min_uV = 10000,
			.max_uV = 3300000,
			.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE |
				               REGULATOR_CHANGE_STATUS),
			.boot_on = 0,
			.always_on = 0,
		},
		.num_consumer_supplies = ARRAY_SIZE(dac088_voutg_consumers),
		.consumer_supplies = dac088_voutg_consumers,
	},
	/* VoutH -> VREF_EXT5 */
	{
		.constraints = {
			.min_uV = 10000,
			.max_uV = 3300000,
			.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE |
				               REGULATOR_CHANGE_STATUS),
			.boot_on = 0,
			.always_on = 0,
		},
		.num_consumer_supplies = ARRAY_SIZE(dac088_vouth_consumers),
		.consumer_supplies = dac088_vouth_consumers,
	},
};

static struct spi_board_info spi_slave_info_bbp1[] = {
	{
		.modalias      = "stepper_spi",
		.irq           = -1,
		.max_speed_hz  = 2000000,
		.bus_num       = 2,
		.chip_select   = 1,
        .platform_data = &stepper_pdata_bbp1,
	},
	{
		.modalias      = "dac088_regulator",
		.irq           = -1,
		.max_speed_hz  = 2000000,
		.bus_num       = 2,
		.chip_select   = 0,
		.platform_data = &dac088_regulator_data[0],
	},
};

static struct spi_board_info spi_slave_info_bbp1s[] = {
	{
		.modalias      = "stepper_spi",
		.irq           = -1,
		.max_speed_hz  = 2000000,
		.bus_num       = 2,
		.chip_select   = 1,
        .platform_data = &stepper_pdata_bbp1s,
	},
	{
		.modalias      = "dac088_regulator",
		.irq           = -1,
		.max_speed_hz  = 2000000,
		.bus_num       = 2,
		.chip_select   = 0,
		.platform_data = &dac088_regulator_data[0],
	},
	{
		.modalias      = "max6675",
		.irq           = -1,
		.max_speed_hz  = 2000000,
		.bus_num       = 1,
		.chip_select   = 0,
	},
};

/* setup spi */
static void spi_init(int evm_id, int profile)
{
	setup_pin_mux(spi1_pin_mux);

    if (bbp_board_type == BBP1S) {
	    setup_pin_mux(spi0_pin_mux);
    }

    if (bbp_board_type == BBP1) {
	    spi_register_board_info(spi_slave_info_bbp1, 
                                ARRAY_SIZE(spi_slave_info_bbp1));
    } else if (bbp_board_type == BBP1S) {
	    spi_register_board_info(spi_slave_info_bbp1s, 
                                ARRAY_SIZE(spi_slave_info_bbp1s));
    }
}

/* vref consumer devices */
static struct vref_platform_data vref_x_data = {
    .name = "vref_x",
    .def_uV = 1600000,
};

static struct platform_device vref_consumer_x = {
    .name = "vref_consumer_x",
    .id   = -1,
    .dev = {
        .platform_data = &vref_x_data,
    },
};

static struct vref_platform_data vref_y_data = {
    .name = "vref_y",
    .def_uV = 1600000,
};

static struct platform_device vref_consumer_y = {
    .name = "vref_consumer_y",
    .id   = -1,
    .dev = {
        .platform_data = &vref_y_data,
    },
};

static struct vref_platform_data vref_z_data = {
    .name = "vref_z",
    .def_uV = 1600000,
};

static struct platform_device vref_consumer_z = {
    .name = "vref_consumer_z",
    .id   = -1,
    .dev = {
        .platform_data = &vref_z_data,
    },
};

static struct vref_platform_data vref_ext1_data = {
    .name = "vref_ext1",
    .def_uV = 1600000,
};

static struct platform_device vref_consumer_ext1 = {
    .name = "vref_consumer_ext1",
    .id   = -1,
    .dev = {
        .platform_data = &vref_ext1_data,
    },
};

static struct vref_platform_data vref_ext2_data = {
    .name = "vref_ext2",
    .def_uV = 1600000,
};

static struct platform_device vref_consumer_ext2 = {
    .name = "vref_consumer_ext2",
    .id   = -1,
    .dev = {
        .platform_data = &vref_ext2_data,
    },
};

static struct vref_platform_data vref_ext3_data = {
    .name = "vref_ext3",
    .def_uV = 1600000,
};

static struct platform_device vref_consumer_ext3 = {
    .name = "vref_consumer_ext3",
    .id   = -1,
    .dev = {
        .platform_data = &vref_ext3_data,
    },
};

static struct vref_platform_data vref_ext4_data = {
    .name = "vref_ext4",
    .def_uV = 1600000,
};

static struct platform_device vref_consumer_ext4 = {
    .name = "vref_consumer_ext4",
    .id   = -1,
    .dev = {
        .platform_data = &vref_ext4_data,
    },
};


static struct vref_platform_data vref_ext5_data = {
    .name = "vref_ext5",
    .def_uV = 1600000,
};

static struct platform_device vref_consumer_ext5 = {
    .name = "vref_consumer_ext5",
    .id   = -1,
    .dev = {
        .platform_data = &vref_ext5_data,
    },
};

static struct platform_device *vref_consumers[] = {
    &vref_consumer_x,
    &vref_consumer_y,
    &vref_consumer_z,
    &vref_consumer_ext1,
    &vref_consumer_ext2,
    &vref_consumer_ext3,
    &vref_consumer_ext4,
    &vref_consumer_ext5,
};

static void dac_init(int evm_id, int profile)
{
    platform_add_devices(vref_consumers, ARRAY_SIZE(vref_consumers));
}
/*----------------------------------------------------------- 
 * ADC
 * Using tsadc to meassure temp voltage: 
 * ain4 -> temp_ext1
 * ain5 -> temp_ext2
 * ain6 -> temp_hbp
 -----------------------------------------------------------*/
static struct adc_data am335x_adc_data = {
	.adc_channels = 8,
};

static struct mfd_tscadc_board tscadc = {
	.adc_init = &am335x_adc_data,
};

static void adc_init(int evm_id, int profile)
{
	int err;

	err = am33xx_register_mfd_tscadc(&tscadc);
	if (err) {
		pr_err("failed to register touchscreen device\n");
    }
}
/*----------------------------------------------------------- 
 * I2C
 *           |--> TPS65217C
 *    I2C0 --|--> cat24c256 0x57  eeprom
 *
 *    I2C1 --|--> FT5x06    0x38  cap ts
 *
 -----------------------------------------------------------*/
static struct at24_platform_data eeprom_info = {
	.byte_len       = (256*1024) / 8,
	.page_size      = 64,
	.flags          = AT24_FLAG_ADDR16,
	.setup          = am335x_evm_setup,
	.context        = (void *)NULL,
};

static struct i2c_board_info __initdata am335x_i2c0_boardinfo[] = {
	{
		/* Baseboard board EEPROM */
		I2C_BOARD_INFO("24c256", 0x57),
		.platform_data  = &eeprom_info,
	},
};

static void __init i2c0_init(void)
{
	am335x_evm_id = BEAGLE_BONE_BLACK;

	omap_register_i2c_bus(1, 100, am335x_i2c0_boardinfo,
				ARRAY_SIZE(am335x_i2c0_boardinfo));
}
/*
 * ts ft5x06
 */
#define TSC_RST_GPIO GPIO_TO_PIN(3, 20)
#define TSC_IRQ_GPIO GPIO_TO_PIN(3, 19)
static struct pinmux_config ts_pin_mux[] = {
   //{"mcasp0_fsr.gpio3_19", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLDOWN},
   {"mcasp0_fsr.gpio3_19", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLDOWN},
   {"mcasp0_axr1.gpio3_20", OMAP_MUX_MODE7 | AM33XX_PULL_UP | AM33XX_PIN_OUTPUT},
   {NULL, 0},
};

static void ts_init(int evm_id, int profile)
{
	setup_pin_mux(ts_pin_mux);

	if (gpio_request(TSC_IRQ_GPIO, "ts_irq") < 0) {
		printk(KERN_ERR "Failed to request GPIO%d for ts IRQ\n", 
                TSC_IRQ_GPIO);
		return;
	}

	gpio_direction_input(TSC_IRQ_GPIO);

	if (gpio_request(TSC_RST_GPIO, "ts_rst") < 0) {
		printk(KERN_ERR "Failed to request GPIO%d for ts Reset\n", 
                TSC_RST_GPIO);
		return;
	}
	gpio_direction_output(TSC_RST_GPIO, 1);
}

/* pinmux for i2c1 */
static struct pinmux_config i2c1_pin_mux_bbp1[] = {
	{"spi0_d1.i2c1_sda",    OMAP_MUX_MODE2 | AM33XX_SLEWCTRL_SLOW |
					        AM33XX_PULL_ENBL | AM33XX_INPUT_EN},
	{"spi0_cs0.i2c1_scl",   OMAP_MUX_MODE2 | AM33XX_SLEWCTRL_SLOW |
					        AM33XX_PULL_ENBL | AM33XX_INPUT_EN},
	{NULL, 0},
};

static struct pinmux_config i2c1_pin_mux_bbp1s[] = {
	{"uart0_ctsn.i2c1_sda", OMAP_MUX_MODE3 | AM33XX_SLEWCTRL_SLOW |
					        AM33XX_PULL_ENBL | AM33XX_INPUT_EN},
	{"uart0_rtsn.i2c1_scl", OMAP_MUX_MODE3 | AM33XX_SLEWCTRL_SLOW |
					        AM33XX_PULL_ENBL | AM33XX_INPUT_EN},
	{NULL, 0},
};

static struct i2c_board_info i2c1_boardinfo_bbp1[] = {
    {
        I2C_BOARD_INFO("ft5x06_ts", 0x38),
        //I2C_BOARD_INFO("gslx680", 0x40),
        .irq = OMAP_GPIO_IRQ(TSC_IRQ_GPIO),
    },
};

/* ads1015 */
static struct ads1015_platform_data ads1015_info = {
    .channel_data[0] =  
    {
        .enabled = true,
        .pga = 2,
        .data_rate = 4,
    },
    .channel_data[1] =  
    {
        .enabled = true,
        .pga = 2,
        .data_rate = 4,
    },
    .channel_data[2] =  
    {
        .enabled = true,
        .pga = 2,
        .data_rate = 4,
    },
    .channel_data[3] =  
    {
        .enabled = true,
        .pga = 2,
        .data_rate = 4,
    },
};



static struct i2c_board_info i2c1_boardinfo_bbp1s[] = {
    {
        I2C_BOARD_INFO("pca9685-pwm", 0x7F),
    },
    {
        I2C_BOARD_INFO("ft5x06_ts", 0x38),
        //I2C_BOARD_INFO("gslx680", 0x40),
        .irq = OMAP_GPIO_IRQ(TSC_IRQ_GPIO),
    },
#if 0
 {
        /* ADDR -> Ground 0x48 */
        /* ADDR -> VDD    0x49 */
        /* ADDR -> SDA    0x4A */
        /* ADDR -> SDL    0x4B */
        I2C_BOARD_INFO("ads1015", 0x49),
        .platform_data = &ads1015_info,
    },
#endif
};

static void i2c1_init(int evm_id, int profile)
{
#if 0
    printk("[Truby]: Set I2C 1 for TP\n");
    setup_pin_mux(i2c1_pin_mux_bbp1);
    //omap_register_i2c_bus(2, 100, i2c1_boardinfo_bbp1,
    omap_register_i2c_bus(2, 200, i2c1_boardinfo_bbp1,
            ARRAY_SIZE(i2c1_boardinfo_bbp1));
#else
    if (bbp_board_type == BBP1) {
        printk("[BBP1]: Set I2C 1 for TP\n");
	    setup_pin_mux(i2c1_pin_mux_bbp1);
	    omap_register_i2c_bus(2, 1, i2c1_boardinfo_bbp1,
			    ARRAY_SIZE(i2c1_boardinfo_bbp1));
    } else {
        printk("[BBP1]: Set I2C 2 for TP\n");
	    setup_pin_mux(i2c1_pin_mux_bbp1s);
	    omap_register_i2c_bus(2, 1, i2c1_boardinfo_bbp1s,
			    ARRAY_SIZE(i2c1_boardinfo_bbp1s));
    }
#endif
	return;
}
/*----------------------------------------------------------- 
 * Display
 -----------------------------------------------------------*/
/* LCDC */
/* pin mux for LCDC */
static struct pinmux_config lcdc_pin_mux[] = {
	{"lcd_data0.lcd_data0",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data1.lcd_data1",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data2.lcd_data2",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data3.lcd_data3",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data4.lcd_data4",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data5.lcd_data5",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data6.lcd_data6",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data7.lcd_data7",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data8.lcd_data8",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data9.lcd_data9",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data10.lcd_data10",	OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data11.lcd_data11",	OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data12.lcd_data12",	OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data13.lcd_data13",	OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data14.lcd_data14",	OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data15.lcd_data15",	OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},

	{"gpmc_ad8.lcd_data16",		OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"gpmc_ad9.lcd_data17",		OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"gpmc_ad10.lcd_data18",	OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"gpmc_ad11.lcd_data19",	OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"gpmc_ad12.lcd_data20",	OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"gpmc_ad13.lcd_data21",	OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"gpmc_ad14.lcd_data22",	OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"gpmc_ad15.lcd_data23",	OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},

	{"lcd_vsync.lcd_vsync",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"lcd_hsync.lcd_hsync",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"lcd_pclk.lcd_pclk",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"lcd_ac_bias_en.lcd_ac_bias_en", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

static const struct display_panel disp_panel = {
	WVGA,
	32,
	32,
	COLOR_ACTIVE,
};

static struct lcd_ctrl_config lcd_cfg = {
	&disp_panel,
	.ac_bias		    = 255,
	.ac_bias_intrpt		= 0,
	.dma_burst_sz		= 16,
	.bpp			    = 32,
	.fdd			    = 0x80,
	.tft_alt_mode		= 0,
	.stn_565_mode		= 0,
	.mono_8bit_mode		= 0,
	.invert_line_clock	= 1,
	.invert_frm_clock	= 1,
	.sync_edge		    = 0,
	.sync_ctrl		    = 1,
	.raster_order		= 0,
};

struct da8xx_lcdc_platform_data HJ070NA13A_pdata = {
       .manu_name              = "INNOLUX",
       .controller_data        = &lcd_cfg,
       .type                   = "HJ070NA13A",
};

static int __init conf_disp_pll(int rate)
{
	struct clk *disp_pll;
	int ret = -EINVAL;

	disp_pll = clk_get(NULL, "dpll_disp_ck");
	if (IS_ERR(disp_pll)) {
		pr_err("Cannot clk_get disp_pll\n");
		goto out;
	}

	ret = clk_set_rate(disp_pll, rate);
	clk_put(disp_pll);
out:
	return ret;
}

static void lcdc_init(int evm_id, int profile)
{
	struct da8xx_lcdc_platform_data *lcdc_pdata;
	setup_pin_mux(lcdc_pin_mux);
    
	if (conf_disp_pll(300000000)) {
		printk("Failed configure display PLL, not attempting to"
				"register LCDC\n");
		return;
	}

	switch (evm_id) {
	case BEAGLE_BONE_BLACK:
		lcdc_pdata = &HJ070NA13A_pdata;
		break;
	default:
		printk("LCDC not supported on this evm (%d)\n",evm_id);
		return;
	}

	lcdc_pdata->get_context_loss_count = omap_pm_get_dev_context_loss_count;

	if (am33xx_register_lcdc(lcdc_pdata)) {
		printk("Failed to register LCDC device\n");
    }

	return;
}

/* HDMI */
static const struct display_panel dvi_panel = {
	WVGA,
	16,
	16,
	COLOR_ACTIVE,
};

static struct lcd_ctrl_config dvi_cfg = {
	&dvi_panel,
	.ac_bias           = 255,
	.ac_bias_intrpt    = 0,
	.dma_burst_sz      = 16,
	.bpp               = 16,
	.fdd               = 0x80,
	.tft_alt_mode      = 0,
	.stn_565_mode      = 0,
	.mono_8bit_mode    = 0,
	.invert_line_clock = 1,
	.invert_frm_clock  = 1,
	.sync_edge         = 0,
	.sync_ctrl         = 1,
	.raster_order      = 0,
};

struct da8xx_lcdc_platform_data hdmi_pdata = {
	.manu_name       = "NXP HDMI",
	.controller_data = &dvi_cfg,
	.type            = "nxp-1280x720@60",
};

static struct i2c_board_info tda99X_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("tda998x",  0x70),
	},
};

static void hdmi_init(int evm_id, int profile)
{
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	unsigned int i2c_instance;

	if (conf_disp_pll(371000000)) {
		pr_info("Failed to set pixclock to 371000000, not attempting to"
				"register DVI adapter\n");
		return;
	}

	if (am33xx_register_lcdc(&hdmi_pdata))
		pr_info("Failed to register BeagleBoardToys HDMI adapter\n");


	if (evm_id == BEAGLE_BONE_BLACK) {
		i2c_instance = 1;
    }

	/* I2C adapter request */
	adapter = i2c_get_adapter(i2c_instance);
	if (!adapter) {
		pr_err("Failed to get adapter i2c%u\n", i2c_instance);
		return;
	}

	client = i2c_new_device(adapter, tda99X_i2c_boardinfo);
	if (!client)
		pr_err("Failed to register HDMI tda998x to i2c%u\n", i2c_instance);

	i2c_put_adapter(adapter);

	pr_info("Setup HDMI display complete\n");

	return;
}

/* Backlight : EHRPWM1B */
#define AM335X_BACKLIGHT_MAX_BRIGHTNESS        100
#define AM335X_BACKLIGHT_DEFAULT_BRIGHTNESS    90
#define AM335X_PWM_PERIOD_NANO_SECONDS        (5000 * 10)

#define BL_EN_GPIO          GPIO_TO_PIN(0, 19)
#define LCD_PWR_EN_GPIO     GPIO_TO_PIN(3, 21)

/* pin mux for backlight control */
static struct pinmux_config backlight_pin_mux[] = {
	{"xdma_event_intr0.gpio0_19",   OMAP_MUX_MODE7 | AM33XX_PULL_UP, AM33XX_PIN_OUTPUT}, //BL-EN
    {"mcasp0_ahclkx.gpio3_21", OMAP_MUX_MODE7 | AM33XX_PULL_UP | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};
static struct platform_pwm_backlight_data backlight_pdata = {
	.pwm_id         = "ehrpwm.1",
	.ch             = 1,
	.lth_brightness	= 1,
	.max_brightness = AM335X_BACKLIGHT_MAX_BRIGHTNESS,
	.dft_brightness = AM335X_BACKLIGHT_DEFAULT_BRIGHTNESS,
	.pwm_period_ns  = AM335X_PWM_PERIOD_NANO_SECONDS,
};

/* Setup pwm-backlight */
static struct platform_device backlight_device = {
	.name           = "pwm-backlight",
	.id             = -1,
	.dev            = {
		.platform_data  = &backlight_pdata,
	}
};

static int __init backlight_init(void)
{
	setup_pin_mux(backlight_pin_mux);

	if (gpio_request(BL_EN_GPIO, "bl_en") < 0) {
		printk(KERN_ERR "Failed to request GPIO%d for bl enable\n", 
                TSC_IRQ_GPIO);
		return 0;
	}
	gpio_direction_output(BL_EN_GPIO, 1);
	gpio_set_value(BL_EN_GPIO, 1);

	if (gpio_request(LCD_PWR_EN_GPIO, "lcd_pwr_en") < 0) {
		printk(KERN_ERR "Failed to request GPIO%d for lcd pwr enable\n", 
                TSC_RST_GPIO);
		return 0;
	}

//printk("<0> lkj test lcd power 1\n"); 
	gpio_direction_output(LCD_PWR_EN_GPIO, 1);
	gpio_set_value(LCD_PWR_EN_GPIO, 0);

    platform_device_register(&backlight_device);
	return 0;
}
late_initcall(backlight_init);
/*----------------------------------------------------------- 
 * MMC
 * mmc0 -> micro sd card
 * mmc1 -> emmc
 -----------------------------------------------------------*/
/* Module pin mux for mmc0 */
static struct pinmux_config mmc0_common_pin_mux[] = {
	{"mmc0_dat3.mmc0_dat3",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_dat2.mmc0_dat2",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_dat1.mmc0_dat1",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_dat0.mmc0_dat0",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_clk.mmc0_clk",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_cmd.mmc0_cmd",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};

static struct pinmux_config mmc0_cd_only_pin_mux[] = {
	{"spi0_cs1.gpio0_6",  OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};

/* Module pin mux for mmc1 */
static struct pinmux_config mmc1_common_pin_mux[] = {
	{"gpmc_ad3.mmc1_dat3",	OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad2.mmc1_dat2",	OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad1.mmc1_dat1",	OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad0.mmc1_dat0",	OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_csn1.mmc1_clk",	OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_csn2.mmc1_cmd",	OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};

static struct pinmux_config mmc1_dat4_7_pin_mux[] = {
	{"gpmc_ad7.mmc1_dat7",	OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad6.mmc1_dat6",	OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad5.mmc1_dat5",	OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad4.mmc1_dat4",	OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};

static struct omap2_hsmmc_info am335x_mmc[] __initdata = {
	{
		.mmc            = 0,	/* will be set at runtime */
	},
	{
		.mmc            = 1,
		.caps           = MMC_CAP_4_BIT_DATA,
		.gpio_cd        = GPIO_TO_PIN(0, 6),
        .gpio_wp        = -EINVAL,
		.ocr_mask       = MMC_VDD_32_33 | MMC_VDD_33_34, /* 3V3 */
	},
	{
		.mmc            = 0,	/* will be set at runtime */
	},
	{}
};

static void mmc1_init(int evm_id, int profile)
{
    setup_pin_mux(mmc1_common_pin_mux);
    setup_pin_mux(mmc1_dat4_7_pin_mux);
    am335x_mmc[0].mmc		   = 2;
    am335x_mmc[0].caps		   = MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA;
    am335x_mmc[0].nonremovable = true;
    am335x_mmc[0].gpio_cd	   = -EINVAL;
    am335x_mmc[0].gpio_wp	   = -EINVAL;
    am335x_mmc[0].ocr_mask	   = MMC_VDD_32_33 | MMC_VDD_33_34; /* 3V3 */
	return;
}

static void mmc0_init(int evm_id, int profile)
{
    setup_pin_mux(mmc0_common_pin_mux);
    setup_pin_mux(mmc0_cd_only_pin_mux);

	omap2_hsmmc_init(am335x_mmc);
	return;
}
/*----------------------------------------------------------- 
 * USB
 -----------------------------------------------------------*/
/* pinmux for usb0 drvvbus */
static struct pinmux_config usb0_pin_mux[] = {
	{"usb0_drvvbus.usb0_drvvbus",    OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

/* pinmux for usb1 drvvbus */
static struct pinmux_config usb1_pin_mux[] = {
	{"usb1_drvvbus.usb1_drvvbus",    OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

static void usb0_init(int evm_id, int profile)
{
	setup_pin_mux(usb0_pin_mux);
	return;
}

static void usb1_init(int evm_id, int profile)
{
	setup_pin_mux(usb1_pin_mux);
	return;
}
/*----------------------------------------------------------- 
 * MII
 -----------------------------------------------------------*/
/* Module pin mux for mii1 */
static struct pinmux_config mii1_pin_mux[] = {
	{"mii1_rxerr.mii1_rxerr", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_txen.mii1_txen",   OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"mii1_rxdv.mii1_rxdv",   OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_txd3.mii1_txd3",   OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"mii1_txd2.mii1_txd2",   OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"mii1_txd1.mii1_txd1",   OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"mii1_txd0.mii1_txd0",   OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"mii1_txclk.mii1_txclk", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxclk.mii1_rxclk", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxd3.mii1_rxd3",   OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxd2.mii1_rxd2",   OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxd1.mii1_rxd1",   OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxd0.mii1_rxd0",   OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mdio_data.mdio_data",   OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mdio_clk.mdio_clk",     OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT_PULLUP},
	{NULL, 0},
};

static void mii1_init(int evm_id, int profile)
{
	setup_pin_mux(mii1_pin_mux);
	return;
}
/*----------------------------------------------------------- 
 * GPIO led
 -----------------------------------------------------------*/
/* pinmux for gpio based led */
static struct pinmux_config gpio_led_mux[] = {
	{"mcasp0_aclkr.gpio3_18",      OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
    {NULL, 0},
};

static struct gpio_led gpio_leds[] = {
	{
		.name			        = "usr1",
		.gpio			        = GPIO_TO_PIN(3, 18),
        .default_trigger        = "default-on",
	},
};

static struct gpio_led_platform_data gpio_led_info = {
    .leds           = gpio_leds,
    .num_leds       = ARRAY_SIZE(gpio_leds),
};

static struct platform_device leds_gpio = {
    .name   = "leds-gpio",
    .id     = -1,
    .dev    = {
            .platform_data  = &gpio_led_info,
    },
};

static void gpio_leds_init(int evm_id, int profile)
{
	setup_pin_mux(gpio_led_mux);
    platform_device_register(&leds_gpio);
}
/*----------------------------------------------------------- 
 * PMIC TPS65217
 -----------------------------------------------------------*/
/* TPS65217 voltage regulator support */
/* 1.8V */
static struct regulator_consumer_supply tps65217_dcdc1_consumers[] = {
	{
		.supply = "vdds_osc",
	},
	{
		.supply = "vdds_pll_ddr",
	},
	{
		.supply = "vdds_pll_mpu",
	},
	{
		.supply = "vdds_pll_core_lcd",
	},
	{
		.supply = "vdds_sram_mpu_bb",
	},
	{
		.supply = "vdds_sram_core_bg",
	},
	{
		.supply = "vdda_usb0_1p8v",
	},
	{
		.supply = "vdds_ddr",
	},
	{
		.supply = "vdds",
	},
	{
		.supply = "vdds_hvx_1p8v",
	},
	{
		.supply = "vdda_adc",
	},
	{
		.supply = "ddr2",
	},
};

/* 1.1V */
static struct regulator_consumer_supply tps65217_dcdc2_consumers[] = {
	{
		.supply = "vdd_mpu",
	},
};

/* 1.1V */
static struct regulator_consumer_supply tps65217_dcdc3_consumers[] = {
	{
		.supply = "vdd_core",
	},
};

/* 1.8V LDO */
static struct regulator_consumer_supply tps65217_ldo1_consumers[] = {
	{
		.supply = "vdds_rtc",
	},
};

/* 3.3V LDO */
static struct regulator_consumer_supply tps65217_ldo2_consumers[] = {
	{
		.supply = "vdds_any_pn",
	},
};

/* 3.3V LDO */
static struct regulator_consumer_supply tps65217_ldo3_consumers[] = {
	{
		.supply = "vdds_hvx_ldo3_3p3v",
	},
	{
		.supply = "vdda_usb0_3p3v",
	},
};

/* 3.3V LDO */
static struct regulator_consumer_supply tps65217_ldo4_consumers[] = {
	{
		.supply = "vdds_hvx_ldo4_3p3v",
	},
};

/*
 * FIXME: Some BeagleBones reuire a ramp_delay to settle down the set
 * voltage from 0.95v to 1.25v. By default a minimum of 70msec is set
 * based on experimentation. This will be removed/modified to exact
 * value, once the root cause is known.
 *
 * The reason for extended ramp time requirement on BeagleBone is not
 * known and the delay varies from board - board, if the board hangs
 * with this 70msec delay then try to increase the value.
 */
static struct tps65217_rdelay dcdc2_ramp_delay = {
	.ramp_delay = 70000,
};

static struct regulator_init_data tps65217_regulator_data[] = {
	/* dcdc1 */
	{
		.constraints = {
			.min_uV = 100000,
			.max_uV = 1800000,
			.boot_on = 1,
			.always_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65217_dcdc1_consumers),
		.consumer_supplies = tps65217_dcdc1_consumers,
	},

	/* dcdc2 */
	{
		.constraints = {
			.min_uV = 100000,
			.max_uV = 3300000,
			.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS),
			.boot_on = 1,
			.always_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65217_dcdc2_consumers),
		.consumer_supplies = tps65217_dcdc2_consumers,
		.driver_data = &dcdc2_ramp_delay,
	},

	/* dcdc3 */
	{
		.constraints = {
			.min_uV = 100000,
			.max_uV = 1500000,
			.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS),
			.boot_on = 1,
			.always_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65217_dcdc3_consumers),
		.consumer_supplies = tps65217_dcdc3_consumers,
	},

	/* ldo1 */
	{
		.constraints = {
			.min_uV = 1000000,
			.max_uV = 3300000,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.boot_on = 1,
			.always_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65217_ldo1_consumers),
		.consumer_supplies = tps65217_ldo1_consumers,
	},

	/* ldo2 */
	{
		.constraints = {
			.min_uV = 100000,
			.max_uV = 3300000,
			.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS),
			.boot_on = 1,
			.always_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65217_ldo2_consumers),
		.consumer_supplies = tps65217_ldo2_consumers,
	},

	/* ldo3 */
	{
		.constraints = {
			.min_uV = 1800000,
			.max_uV = 3300000,
			.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS),
			.boot_on = 1,
			.always_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65217_ldo3_consumers),
		.consumer_supplies = tps65217_ldo3_consumers,
	},

	/* ldo4 */
	{
		.constraints = {
			.min_uV = 1800000,
			.max_uV = 3300000,
			.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS),
			.boot_on = 1,
			.always_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65217_ldo4_consumers),
		.consumer_supplies = tps65217_ldo4_consumers,
	},
};

struct tps65217_bl_pdata bone_lcd3_bl_pdata[] = {
	{
		.isel = TPS65217_BL_ISET1,
		.fdim = TPS65217_BL_FDIM_200HZ,
	},
};

static struct tps65217_board beaglebone_tps65217_info = {
	.tps65217_init_data = &tps65217_regulator_data[0],
	.bl_pdata = bone_lcd3_bl_pdata,
	.status_off = true,
};

static struct i2c_board_info tps65217_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("tps65217", TPS65217_I2C_ID),
		.platform_data  = &beaglebone_tps65217_info,
	},
};

static void tps65217_init(int evm_id, int profile)
{
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	struct device *mpu_dev;
	struct tps65217 *tps;
	unsigned int val;
	int ret;

	mpu_dev = omap_device_get_by_hwmod_name("mpu");
	if (!mpu_dev)
		pr_warning("%s: unable to get the mpu device\n", __func__);

	/* I2C1 adapter request */
	adapter = i2c_get_adapter(1);
	if (!adapter) {
		pr_err("failed to get adapter i2c1\n");
		return;
	}

	client = i2c_new_device(adapter, tps65217_i2c_boardinfo);
	if (!client)
		pr_err("failed to register tps65217 to i2c1\n");

	i2c_put_adapter(adapter);

	tps = (struct tps65217 *)i2c_get_clientdata(client);

	ret = tps65217_reg_read(tps, TPS65217_REG_STATUS, &val);
	if (ret) {
		pr_err("failed to read tps65217 status reg\n");
		return;
	}

#if 0 
    //FIXME: Allow power on by OTG to run at high frequency.
	if (!(val & TPS65217_STATUS_ACPWR)) {
		/* If powered by USB then disable OPP120, OPPTURBO and OPPNITRO*/
		/*	
		pr_info("Maximum current provided by the USB port is 500mA"
			" which is not sufficient\nwhen operating @OPP120, OPPTURBO and"
			" OPPNITRO. The current requirement for some\nuse-cases"
			" using OPP100 might also exceed the maximum current"
			" that the\nUSB port can provide. Unless you are fully"
			" confident that the current\nrequirements for OPP100"
			" use-case don't exceed the USB limits, switching\nto"
			" AC power is recommended.\n");
		
		opp_disable(mpu_dev, 600000000);
		opp_disable(mpu_dev, 720000000);
		opp_disable(mpu_dev, 800000000);
		opp_disable(mpu_dev, 1000000000);
		*/
	}
#endif
}
/*----------------------------------------------------------- 
 * RTC
 -----------------------------------------------------------*/
static struct omap_rtc_pdata am335x_rtc_info = {
	.pm_off		= false,
	.wakeup_capable	= 0,
};

static void rtc_init(int evm_id, int profile)
{
	void __iomem *base;
	struct clk *clk;
	struct omap_hwmod *oh;
	struct platform_device *pdev;
	char *dev_name = "am33xx-rtc";

	clk = clk_get(NULL, "rtc_fck");
	if (IS_ERR(clk)) {
		pr_err("rtc : Failed to get RTC clock\n");
		return;
	}

	if (clk_enable(clk)) {
		pr_err("rtc: Clock Enable Failed\n");
		return;
	}

	base = ioremap(AM33XX_RTC_BASE, SZ_4K);

	if (WARN_ON(!base))
		return;

	/* Unlock the rtc's registers */
	writel(0x83e70b13, base + 0x6c);
	writel(0x95a4f1e0, base + 0x70);

	/*
	 * Enable the 32K OSc
	 * TODO: Need a better way to handle this
	 * Since we want the clock to be running before mmc init
	 * we need to do it before the rtc probe happens
	 */
	writel(0x48, base + 0x54);

	iounmap(base);
    
	am335x_rtc_info.pm_off = true;

	clk_disable(clk);
	clk_put(clk);

	if (omap_rev() >= AM335X_REV_ES2_0)
		am335x_rtc_info.wakeup_capable = 1;

	oh = omap_hwmod_lookup("rtc");
	if (!oh) {
		pr_err("could not look up %s\n", "rtc");
		return;
	}

	pdev = omap_device_build(dev_name, -1, oh, &am335x_rtc_info,
			sizeof(struct omap_rtc_pdata), NULL, 0, 0);
	WARN(IS_ERR(pdev), "Can't build omap_device for %s:%s.\n",
			dev_name, oh->name);
}
/*----------------------------------------------------------- 
 * GPU
 -----------------------------------------------------------*/
static void sgx_init(int evm_id, int profile)
{
	if (omap3_has_sgx()) {
		am33xx_gpu_init();
	}
}

/* FastBot BBP 1 */
static struct evm_dev_cfg bbp1_dev_cfg[] = {
	{rtc_init,        DEV_ON_BASEBOARD, PROFILE_NONE},
	{tps65217_init,	  DEV_ON_BASEBOARD, PROFILE_NONE},
	{mii1_init,	      DEV_ON_BASEBOARD, PROFILE_NONE},
	{adc_init,        DEV_ON_BASEBOARD, PROFILE_NONE},
	{usb0_init,	      DEV_ON_BASEBOARD, PROFILE_NONE},
	{usb1_init,	      DEV_ON_BASEBOARD, PROFILE_NONE},
	{mmc1_init,	      DEV_ON_BASEBOARD, PROFILE_NONE},
	{mmc0_init,	      DEV_ON_BASEBOARD, PROFILE_NONE},
    {spi_init,        DEV_ON_BASEBOARD, PROFILE_NONE},
	{ecap0_init,      DEV_ON_BASEBOARD, PROFILE_NONE},
	{ehrpwm0_init,    DEV_ON_BASEBOARD, PROFILE_NONE},
	{ehrpwm1_init,    DEV_ON_BASEBOARD, PROFILE_NONE},
    {i2c1_init,	      DEV_ON_BASEBOARD, PROFILE_NONE},
	{fan_init,        DEV_ON_BASEBOARD, PROFILE_NONE},
	{sgx_init,	      DEV_ON_BASEBOARD, PROFILE_NONE},
    {dac_init,        DEV_ON_BASEBOARD, PROFILE_NONE},
    {stepper_init,    DEV_ON_BASEBOARD, PROFILE_NONE},
    {lmsw_init,       DEV_ON_BASEBOARD, PROFILE_NONE},
	{lcdc_init,       DEV_ON_BASEBOARD, PROFILE_NONE},
	{gpio_leds_init,  DEV_ON_BASEBOARD, PROFILE_NONE},
	{ts_init,         DEV_ON_BASEBOARD, PROFILE_NONE},
    //{pwr_button_init, DEV_ON_BASEBOARD, PROFILE_NONE},
    //{hdmi_init,	    DEV_ON_BASEBOARD, PROFILE_NONE},
    //{w1_gpio_init,    DEV_ON_BASEBOARD, PROFILE_NONE},
	{NULL, 0, 0},
};

/* FastBot BBP 1S */
static struct evm_dev_cfg bbp1s_dev_cfg[] = {
	{rtc_init,        DEV_ON_BASEBOARD, PROFILE_NONE},
	{tps65217_init,	  DEV_ON_BASEBOARD, PROFILE_NONE},
	{mii1_init,	      DEV_ON_BASEBOARD, PROFILE_NONE},
	{adc_init,        DEV_ON_BASEBOARD, PROFILE_NONE},
	{usb0_init,	      DEV_ON_BASEBOARD, PROFILE_NONE},
	{usb1_init,	      DEV_ON_BASEBOARD, PROFILE_NONE},
	{mmc1_init,	      DEV_ON_BASEBOARD, PROFILE_NONE},
	{mmc0_init,	      DEV_ON_BASEBOARD, PROFILE_NONE},
    {spi_init,        DEV_ON_BASEBOARD, PROFILE_NONE},
	{ecap0_init,      DEV_ON_BASEBOARD, PROFILE_NONE},
	{ehrpwm1_init,    DEV_ON_BASEBOARD, PROFILE_NONE},
    {i2c1_init,	      DEV_ON_BASEBOARD, PROFILE_NONE},
	{sgx_init,	      DEV_ON_BASEBOARD, PROFILE_NONE},
    {dac_init,        DEV_ON_BASEBOARD, PROFILE_NONE},
    {stepper_init,    DEV_ON_BASEBOARD, PROFILE_NONE},
    {lmsw_init,       DEV_ON_BASEBOARD, PROFILE_NONE},
	{lcdc_init,       DEV_ON_BASEBOARD, PROFILE_NONE},
	{gpio_leds_init,  DEV_ON_BASEBOARD, PROFILE_NONE},
	{ts_init,         DEV_ON_BASEBOARD, PROFILE_NONE},
    //{pwr_button_init, DEV_ON_BASEBOARD, PROFILE_NONE},
    //{hdmi_init,	    DEV_ON_BASEBOARD, PROFILE_NONE},
    //{w1_gpio_init,    DEV_ON_BASEBOARD, PROFILE_NONE},
	{NULL, 0, 0},
};

/* BeagleBone Black */
static void setup_fastbot_bbp1(void)
{
	pr_info("The board is FastBot BBP 1\n");
	_configure_device(BEAGLE_BONE_BLACK, bbp1_dev_cfg, PROFILE_NONE);

	/* TPS65217 regulator has full constraints */
    //FIXME:  Please check it out!!!!!!
	//regulator_has_full_constraints();

	am33xx_cpsw_init(AM33XX_CPSW_MODE_MII, NULL, NULL);
}

static void setup_fastbot_bbp1s(void)
{
	pr_info("The board is FastBot BBP 1S\n");
	_configure_device(BEAGLE_BONE_BLACK, bbp1s_dev_cfg, PROFILE_NONE);

	/* TPS65217 regulator has full constraints */
    //FIXME:  Please check it out!!!!!!
	//regulator_has_full_constraints();

	am33xx_cpsw_init(AM33XX_CPSW_MODE_MII, NULL, NULL);
}

#define AM33XX_VDD_CORE_OPP50_UV		1100000
#define AM33XX_OPP120_FREQ		        600000000
#define AM33XX_OPPTURBO_FREQ		    720000000

#define AM33XX_ES2_0_VDD_CORE_OPP50_UV	950000
#define AM33XX_ES2_0_OPP120_FREQ	    720000000
#define AM33XX_ES2_0_OPPTURBO_FREQ	    800000000
#define AM33XX_ES2_0_OPPNITRO_FREQ	    1000000000

#define AM33XX_ES2_1_VDD_CORE_OPP50_UV	950000
#define AM33XX_ES2_1_OPP120_FREQ	    720000000
#define AM33XX_ES2_1_OPPTURBO_FREQ	    800000000
#define AM33XX_ES2_1_OPPNITRO_FREQ	    1000000000

static void am335x_opp_update(void)
{
	u32 rev;
	int voltage_uv = 0;
	struct device *core_dev, *mpu_dev;
	struct regulator *core_reg;

	core_dev = omap_device_get_by_hwmod_name("l3_main");
	mpu_dev = omap_device_get_by_hwmod_name("mpu");

	if (!mpu_dev || !core_dev) {
		pr_err("%s: Aiee.. no mpu/core devices? %p %p\n", __func__,
		       mpu_dev, core_dev);
		return;
	}

	core_reg = regulator_get(core_dev, "vdd_core");
	if (IS_ERR(core_reg)) {
		pr_err("%s: unable to get core regulator\n", __func__);
		return;
	}

	/*
	 * Ensure physical regulator is present.
	 * (e.g. could be dummy regulator.)
	 */
	voltage_uv = regulator_get_voltage(core_reg);
	if (voltage_uv < 0) {
		pr_err("%s: physical regulator not present for core" \
		       "(%d)\n", __func__, voltage_uv);
		regulator_put(core_reg);
		return;
	}

	pr_debug("%s: core regulator value %d\n", __func__, voltage_uv);
	if (voltage_uv > 0) {
		rev = omap_rev();
		switch (rev) {
		case AM335X_REV_ES1_0:
			if (voltage_uv <= AM33XX_VDD_CORE_OPP50_UV) {
				/*
				 * disable the higher freqs - we dont care about
				 * the results
				 */
				opp_disable(mpu_dev, AM33XX_OPP120_FREQ);
				opp_disable(mpu_dev, AM33XX_OPPTURBO_FREQ);
			}
			break;
		case AM335X_REV_ES2_0:
			if (voltage_uv <= AM33XX_ES2_0_VDD_CORE_OPP50_UV) {
				/*
				 * disable the higher freqs - we dont care about
				 * the results
				 */
				opp_disable(mpu_dev,
					    AM33XX_ES2_0_OPP120_FREQ);
				opp_disable(mpu_dev,
					    AM33XX_ES2_0_OPPTURBO_FREQ);
				opp_disable(mpu_dev,
					    AM33XX_ES2_0_OPPNITRO_FREQ);
			}
			break;
		case AM335X_REV_ES2_1:
		/* FALLTHROUGH */
		default:
			if (voltage_uv <= AM33XX_ES2_1_VDD_CORE_OPP50_UV) {
				/*
				 * disable the higher freqs - we dont care about
				 * the results
				 */
				opp_disable(mpu_dev,
					    AM33XX_ES2_1_OPP120_FREQ);
				opp_disable(mpu_dev,
					    AM33XX_ES2_1_OPPTURBO_FREQ);
				opp_disable(mpu_dev,
					    AM33XX_ES2_1_OPPNITRO_FREQ);
			}
			break;
		}
	}
}

static void am335x_evm_setup(struct memory_accessor *mem_acc, void *context)
{
	int ret;
	char tmp[10];
	struct device *mpu_dev;

    //FIXME: Mac Addr
#if 0
	/* 1st get the MAC address from EEPROM */
	ret = mem_acc->read(mem_acc, (char *)&am335x_mac_addr,
		EEPROM_MAC_ADDRESS_OFFSET, sizeof(am335x_mac_addr));

	if (ret != sizeof(am335x_mac_addr)) {
		pr_warning("AM335X: EVM Config read fail: %d\n", ret);
		return;
	}

	/* Fillup global mac id */
	am33xx_cpsw_macidfillup(&am335x_mac_addr[0][0],
				&am335x_mac_addr[1][0]);
#endif

	/* get board specific data from eeprom */
	ret = mem_acc->read(mem_acc, (char *)&config, 0, sizeof(config));
	if (ret != sizeof(config)) {
		pr_err("BBP board config read fail, read %d bytes\n", ret);
		pr_err("This likely means that there either is no/or a failed EEPROM\n");
		goto out;
	}

	snprintf(tmp, sizeof(config.name) + 1, "%s", config.name);
	pr_info("Board name: %s\n", tmp);
	snprintf(tmp, sizeof(config.version) + 1, "%s", config.version);
	pr_info("Board version: %s\n", tmp);

	get_eeprom_board_type();

    //FIXME: Please check it out!
    //mpu_dev = omap_device_get_by_hwmod_name("mpu");
    //opp_enable(mpu_dev, AM33XX_ES2_0_OPPNITRO_FREQ);

    if (bbp_board_type == BBP1S) {
	    setup_fastbot_bbp1s();
    } else {
	    setup_fastbot_bbp1();
    }

	am335x_opp_update();

#if 0
	if (config.header != AM335X_EEPROM_HEADER) {
		pr_err("AM335X: wrong header 0x%x, expected 0x%x\n",
			config.header, AM335X_EEPROM_HEADER);
		goto out;
	}

	if (strncmp("A335", config.name, 4)) {
		pr_err("Board %s\ndoesn't look like an AM335x board\n",
			config.name);
		goto out;
	}
#endif

#if 0
	if (!strncmp("A335BNLT", config.name, 8)) {
		daughter_brd_detected = false;
		if(!strncmp("0A5A", config.version, 4) ||
		   !strncmp("0A5B", config.version, 4) ||
		   !strncmp("0A5C", config.version, 4)) {
			mpu_dev = omap_device_get_by_hwmod_name("mpu");
			opp_enable(mpu_dev, AM33XX_ES2_0_OPPNITRO_FREQ);
		}
		setup_beaglebone_black();
	} else {
		goto out;
	}
#endif

	return;

out:
	/*
	 * If the EEPROM hasn't been programed or an incorrect header
	 * or board name are read then the hardware details are unknown.
	 * Notify the user and call machine_halt to stop the boot process.
	 */
	pr_err("The error message above indicates that there is an issue with\n"
		   "the EEPROM or the EEPROM contents.  After verifying the EEPROM\n"
		   "contents, if any, refer to the %s function in the\n"
		   "%s file to modify the board\n"
		   "initialization code to match the hardware configuration\n",
		   __func__ , __FILE__);
	machine_halt();
}

/*
 * ioremap
 */
void __iomem *am33xx_emif_base;
void __iomem * __init am33xx_get_mem_ctlr(void)
{
	am33xx_emif_base = ioremap(AM33XX_EMIF0_BASE, SZ_32K);

	if (!am33xx_emif_base) {
		pr_warning("%s: Unable to map DDR2 controller",	__func__);
    }

	return am33xx_emif_base;
}

void __iomem *am33xx_get_ram_base(void)
{
	return am33xx_emif_base;
}

void __iomem *am33xx_gpio0_base;
void __iomem *am33xx_get_gpio0_base(void)
{
	am33xx_gpio0_base = ioremap(AM33XX_GPIO0_BASE, SZ_4K);

	return am33xx_gpio0_base;
}

/* CPU Idle
 * AM33XX devices support DDR2 power down 
 */
static struct resource am33xx_cpuidle_resources[] = {
	{
		.start		= AM33XX_EMIF0_BASE,
		.end		= AM33XX_EMIF0_BASE + SZ_32K - 1,
		.flags		= IORESOURCE_MEM,
	},
};

static struct am33xx_cpuidle_config am33xx_cpuidle_pdata = {
	.ddr2_pdown	= 1,
};

static struct platform_device am33xx_cpuidle_device = {
	.name			= "cpuidle-am33xx",
	.num_resources	= ARRAY_SIZE(am33xx_cpuidle_resources),
	.resource		= am33xx_cpuidle_resources,
	.dev = {
		.platform_data	= &am33xx_cpuidle_pdata,
	},
};

static void __init am33xx_cpuidle_init(void)
{
	int ret;

	am33xx_cpuidle_pdata.emif_base = am33xx_get_mem_ctlr();

	ret = platform_device_register(&am33xx_cpuidle_device);
	if (ret) {
		pr_warning("AM33XX cpuidle registration failed\n");
    }
}

static struct omap_musb_board_data musb_board_data = {
	.interface_type	= MUSB_INTERFACE_ULPI,
	/*
	 * mode[0:3] = USB0PORT's mode
	 * mode[4:7] = USB1PORT's mode
	 * AM335X beta EVM has USB0 in OTG mode and USB1 in host mode.
	 */
	.mode       = (MUSB_HOST << 4) | MUSB_OTG,
	.power		= 500,
	.instances	= 1,
};

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	/*
	 * Setting SYSBOOT[5] should set xdma_event_intr0 pin to mode 3 thereby
	 * allowing clkout1 to be available on xdma_event_intr0.
	 * However, on some boards (like EVM-SK), SYSBOOT[5] isn't properly
	 * latched.
	 * To be extra cautious, setup the pin-mux manually.
	 * If any modules/usecase requries it in different mode, then subsequent
	 * module init call will change the mux accordingly.
	 */
	AM33XX_MUX(XDMA_EVENT_INTR0, OMAP_MUX_MODE3 | AM33XX_PIN_OUTPUT),
	AM33XX_MUX(I2C0_SDA, OMAP_MUX_MODE0 | AM33XX_SLEWCTRL_SLOW |
			AM33XX_INPUT_EN | AM33XX_PIN_OUTPUT),
	AM33XX_MUX(I2C0_SCL, OMAP_MUX_MODE0 | AM33XX_SLEWCTRL_SLOW |
			AM33XX_INPUT_EN | AM33XX_PIN_OUTPUT),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define	board_mux	NULL
#endif

static void __init am335x_unicron_init(void)
{
	am33xx_cpuidle_init();
	am33xx_mux_init(board_mux);
	omap_serial_init();
	i2c0_init();
	omap_sdrc_init(NULL, NULL);
	usb_musb_init(&musb_board_data);

	omap_board_config = am335x_evm_config;
	omap_board_config_size = ARRAY_SIZE(am335x_evm_config);
    
	/* Create an alias for icss clock */
	if (clk_add_alias("pruss", NULL, "pruss_uart_gclk", NULL)) {
		pr_warn("failed to create an alias: icss_uart_gclk --> pruss\n");
    }
	/* Create an alias for gfx/sgx clock */
	if (clk_add_alias("sgx_ck", NULL, "gfx_fclk", NULL)) {
		pr_warn("failed to create an alias: gfx_fclk --> sgx_ck\n");
    }
}

static void __init am335x_unicron_map_io(void)
{
	omap2_set_globals_am33xx();
	omapam33xx_map_common_io();
}

extern void __init omap_pru_reserve_sdram_memblock(void);
static void __init unicorn_omap_reserve(void)
{
    omap_pru_reserve_sdram_memblock();
}

//TODO: change uboot Machine ID to use UNICRON
MACHINE_START(AM335XEVM, "am335xevm")
	.atag_offset	= 0x100,
	.reserve		= unicorn_omap_reserve,
	.map_io		    = am335x_unicron_map_io,
	.init_early	    = am33xx_init_early,
	.init_irq	    = ti81xx_init_irq,
	.handle_irq     = omap3_intc_handle_irq,
	.timer		    = &omap3_am33xx_timer,
	.init_machine	= am335x_unicron_init,
MACHINE_END
