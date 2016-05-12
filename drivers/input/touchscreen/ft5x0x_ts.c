/* 
 * drivers/input/touchscreen/ft5x0x_ts.c
 *
 * FocalTech ft5x0x TouchScreen driver. 
 *
 * Copyright (c) 2010  Focal tech Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *
 *	note: only support mulititouch	Wenfs 2010-10-01
 *
 *	modified for robust device probing  Chipsee 2012-10-30
 *
 */

#include <linux/module.h>
#include <linux/hrtimer.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/earlysuspend.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/gpio.h>

#include "ft5x0x_ts.h"

static struct i2c_client *this_client;
static struct ft5x0x_ts_platform_data *pdata;

//#define CONFIG_FT5X0X_MULTITOUCH 1
#undef	CONFIG_FT5X0X_MULTITOUCH 
#define FT5X06_ID 0x79

#define TSC_RST_GPIO 116 

struct ts_event
{
	u16 x1;
	u16 y1;
	u16 x2;
	u16 y2;
	u16 x3;
	u16 y3;
	u16 x4;
	u16 y4;
	u16 x5;
	u16 y5;
	u16 pressure;
	s16 touch_ID1;
	s16 touch_ID2;
	s16 touch_ID3;
	s16 touch_ID4;
	s16 touch_ID5;
	u8  touch_point;
};

struct ft5x0x_ts_data
{
	struct input_dev	*input_dev;
	struct ts_event		event;
	struct work_struct 	pen_event_work;
	struct workqueue_struct *ts_workqueue;
	struct early_suspend	early_suspend;
};

static int ft5x0x_i2c_rxdata(char *rxdata, int length)
{
	int ret;

	struct i2c_msg msgs[] =
	{
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rxdata,
		},
		{
			.addr	= this_client->addr,
			.flags	= I2C_M_RD,
			.len	= length,
			.buf	= rxdata,
		},
	};

	ret = i2c_transfer(this_client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);
	
	return ret;
}

static int ft5x0x_i2c_txdata(char *txdata, int length)
{
	int ret;

	struct i2c_msg msg[] =
	{
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= length,
			.buf	= txdata,
		},
	};

	ret = i2c_transfer(this_client->adapter, msg, 1);
	if (ret < 0)
		pr_err("%s i2c write error: %d\n", __func__, ret);

	return ret;
}

static int ft5x0x_set_reg(u8 addr, u8 para)
{
	u8 buf[3];
	int ret = -1;

	buf[0] = addr;
	buf[1] = para;
	ret = ft5x0x_i2c_txdata(buf, 2);
	if (ret < 0)
	{
		pr_err("write reg failed! %#x ret: %d", buf[0], ret);
		return -1;
	}

	return 0;
}

static void ft5x0x_ts_release(void)
{
	struct ft5x0x_ts_data *data = i2c_get_clientdata(this_client);

#ifdef CONFIG_FT5X0X_MULTITOUCH	
	input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 0);
	input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 0);
#else
	input_report_abs(data->input_dev, ABS_PRESSURE, 0);
	input_report_key(data->input_dev, BTN_TOUCH, 0);
#endif
	input_sync(data->input_dev);
}

static void ft5x0x_ts_inactivate(void)
{
	struct ft5x0x_ts_data *data = i2c_get_clientdata(this_client);

#ifdef CONFIG_FT5X0X_MULTITOUCH
	input_mt_sync(data->input_dev);
#else
	input_report_abs(data->input_dev, ABS_PRESSURE, 0);
	input_report_key(data->input_dev, BTN_TOUCH, 0);
#endif
	input_sync(data->input_dev);
}

static int ft5x0x_read_data(void)
{
	struct ft5x0x_ts_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;
	u8 buf[32] = {0};
	int ret = -1;
	int status = 0;

#ifdef CONFIG_FT5X0X_MULTITOUCH
	ret = ft5x0x_i2c_rxdata(buf, 31);
#else
	ret = ft5x0x_i2c_rxdata(buf, 7);
#endif
	if (ret < 0)
	{
		printk("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
		return ret;
	}

	memset(event, 0, sizeof(struct ts_event));
	event->touch_point = buf[2] & 0x07;

	if (event->touch_point == 0)
	{
		ft5x0x_ts_inactivate();
		return 1; 
	}

#ifdef CONFIG_FT5X0X_MULTITOUCH
	switch (event->touch_point)
	{
		case 5:
			event->x5 = (s16)(buf[0x1b] & 0x0F)<<8 | (s16)buf[0x1c];
			event->y5 = (s16)(buf[0x1d] & 0x0F)<<8 | (s16)buf[0x1e];
			status = (s16)((buf[0x1b] & 0xc0) >> 6);
			event->touch_ID5=(s16)(buf[0x1D] & 0xF0)>>4;
			if (status == 1) ft5x0x_ts_release();

		case 4:
			event->x4 = (s16)(buf[0x15] & 0x0F)<<8 | (s16)buf[0x16];
			event->y4 = (s16)(buf[0x17] & 0x0F)<<8 | (s16)buf[0x18];
			status = (s16)((buf[0x15] & 0xc0) >> 6);
			event->touch_ID4=(s16)(buf[0x17] & 0xF0)>>4;
			if (status == 1) ft5x0x_ts_release();

		case 3:
			event->x3 = (s16)(buf[0x0f] & 0x0F)<<8 | (s16)buf[0x10];
			event->y3 = (s16)(buf[0x11] & 0x0F)<<8 | (s16)buf[0x12];
			status = (s16)((buf[0x0f] & 0xc0) >> 6);
			event->touch_ID3=(s16)(buf[0x11] & 0xF0)>>4;
			if (status == 1) ft5x0x_ts_release();

		case 2:
			event->x2 = (s16)(buf[9] & 0x0F)<<8 | (s16)buf[10];
			event->y2 = (s16)(buf[11] & 0x0F)<<8 | (s16)buf[12];
			status = (s16)((buf[0x9] & 0xc0) >> 6);
			event->touch_ID2=(s16)(buf[0x0b] & 0xF0)>>4;
			if (status == 1) ft5x0x_ts_release();

		case 1:
			event->x1 = (s16)(buf[3] & 0x0F)<<8 | (s16)buf[4];
			event->y1 = (s16)(buf[5] & 0x0F)<<8 | (s16)buf[6];
			status = (s16)((buf[0x3] & 0xc0) >> 6);
			event->touch_ID1=(s16)(buf[0x05] & 0xF0)>>4;
			if (status == 1) ft5x0x_ts_release();
            		break;

		default:
			return -1;
	}
#else
	if (event->touch_point == 1)
	{
		event->x1 = (s16)(buf[3] & 0x0F)<<8 | (s16)buf[4];
		event->y1 = (s16)(buf[5] & 0x0F)<<8 | (s16)buf[6];
	}
#endif
	event->pressure = 10;

	dev_dbg(&this_client->dev, "%s: 1:%d %d 2:%d %d \n", __func__,
		event->x1, event->y1, event->x2, event->y2);

	return 0;
}

static int last_point[2]= {0};
static int filter_point(u32 x, u32 y)
{
	int diff_x = abs(last_point[0] - x);
	int diff_y = abs(last_point[1] - y);
	if (diff_x <= 2 || diff_y <= 2) {
			return 1;
	}
	last_point[0] = x;
	last_point[1] = y;
	return 0;
}

static void ft5x0x_report_value(void)
{
	struct ft5x0x_ts_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;

#ifdef CONFIG_FT5X0X_MULTITOUCH
	switch(event->touch_point)
	{
		case 5:
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->touch_ID5);
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, SCREEN_MAX_Y - event->x5);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, SCREEN_MAX_X - event->y5);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_mt_sync(data->input_dev);
		case 4:
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->touch_ID4);
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, SCREEN_MAX_Y - event->x4);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, SCREEN_MAX_X - event->y4);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_mt_sync(data->input_dev);
		case 3:
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->touch_ID3);
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, SCREEN_MAX_Y - event->x3);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, SCREEN_MAX_X - event->y3);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_mt_sync(data->input_dev);
		case 2:
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->touch_ID2);
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, SCREEN_MAX_Y - event->x2);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, SCREEN_MAX_X - event->y2);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_mt_sync(data->input_dev);
		case 1:
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->touch_ID1);
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, SCREEN_MAX_Y - event->x1);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, SCREEN_MAX_X - event->y1);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_mt_sync(data->input_dev);
		default:
			break;
	}
#else	/* CONFIG_FT5X0X_MULTITOUCH*/
	if (event->touch_point == 1)
	{
		//input_report_abs(data->input_dev, ABS_Y, SCREEN_MAX_Y - event->x1);
		//input_report_abs(data->input_dev, ABS_X, event->y1);
		//lkj 1024 * 800.0f  == 5243 /4096 
		//600 * 480.0f  == 5120 /4096 
	#if 0
		int x1 = (event->x1 * 5243)>>12;
		int y1 = (event->y1 * 5120)>>12;
	#else
		int x1 = event->y1;
		int y1 = event->x1;
		//int x1 = event->x1;
		//int y1 = event->y1;
	#endif
#if 0
		if(filter_point(x1, y1)){
			printk("lkj filter ok\n");
			return;
		}
#endif
		input_report_abs(data->input_dev, ABS_X, x1);
		input_report_abs(data->input_dev, ABS_Y, y1);
		input_report_abs(data->input_dev, ABS_PRESSURE, event->pressure);
		input_report_key(data->input_dev, BTN_TOUCH, 1);
		input_sync(data->input_dev);
	}
#endif	/* CONFIG_FT5X0X_MULTITOUCH*/
	//input_sync(data->input_dev);

	//dev_info(&this_client->dev, "%s: 1:%d %d \n", __func__,
	//	event->x1 , event->y1);
	//dev_dbg(&this_client->dev, "%s: 1:%d %d 2:%d %d \n", __func__,
	//	(event->x1 * 5243)>>12, (event->y1 * 5120)>>12, event->x2, event->y2);
}	/*end ft5x0x_report_value*/

static void ft5x0x_ts_pen_irq_work(struct work_struct *work)
{
	int ret = -1;

	ret = ft5x0x_read_data();	

	if (ret == 0) {
		ft5x0x_report_value(); 	 
	}
}

static irqreturn_t ft5x0x_ts_interrupt(int irq, void *dev_id)
{
	struct ft5x0x_ts_data *ft5x0x_ts = dev_id;

	if (!work_pending(&ft5x0x_ts->pen_event_work)) {
		queue_work(ft5x0x_ts->ts_workqueue, &ft5x0x_ts->pen_event_work);
	}

	return IRQ_HANDLED;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ft5x0x_ts_suspend(struct early_suspend *handler)
{
}

static void ft5x0x_ts_resume(struct early_suspend *handler)
{
}
#endif  //CONFIG_HAS_EARLYSUSPEND

static int ft5x0x_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ft5x0x_ts_data *ft5x0x_ts;
	struct input_dev *input_dev;
	int err = 0;
	int rev_id = 0;


	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto err_out;
	}

	ft5x0x_ts = kzalloc(sizeof(*ft5x0x_ts), GFP_KERNEL);
	if (!ft5x0x_ts)	{
		err = -ENOMEM;
		goto err_free_mem;
	}

	this_client = client;
	i2c_set_clientdata(client, ft5x0x_ts);

	rev_id = i2c_smbus_read_byte_data(client, FT5X0X_REG_FT5201ID);
	if (rev_id != FT5X06_ID)
	{
		err = -ENODEV;
		dev_err(&client->dev, "failed to probe FT5X0X touchscreen device\n");
		goto err_free_mem;
	}

	INIT_WORK(&ft5x0x_ts->pen_event_work, ft5x0x_ts_pen_irq_work);
	ft5x0x_ts->ts_workqueue = create_singlethread_workqueue(dev_name(&client->dev));
	if (!ft5x0x_ts->ts_workqueue)
	{
		err = -ESRCH;
		goto err_free_thread;
	}
 	err = request_irq(client->irq, ft5x0x_ts_interrupt, IRQF_DISABLED | IRQF_TRIGGER_RISING, "ft5x0x_ts", ft5x0x_ts);
	if (err < 0)
	{
		dev_err(&client->dev, "request irq failed\n");
		goto err_free_irq;
	}

	input_dev = input_allocate_device();
	if (!input_dev)
	{
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto err_free_input;
	}
	
	ft5x0x_ts->input_dev = input_dev;

  #ifdef CONFIG_FT5X0X_MULTITOUCH
	set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);

	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_Y, 0, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);
#else
	set_bit(ABS_X, input_dev->absbit);
	set_bit(ABS_Y, input_dev->absbit);
	set_bit(ABS_PRESSURE, input_dev->absbit);
	set_bit(BTN_TOUCH, input_dev->keybit);

	input_set_abs_params(input_dev, ABS_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, PRESS_MAX, 0 , 0);
#endif

	set_bit(EV_ABS, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);

	input_dev->name	= FT5X0X_NAME;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;
	err = input_register_device(input_dev);
	if (err)
	{
		dev_err(&client->dev, "failed to register input device: %s\n",
		dev_name(&client->dev));
		goto err_free_input;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	ft5x0x_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ft5x0x_ts->early_suspend.suspend = ft5x0x_ts_suspend;
	ft5x0x_ts->early_suspend.resume	= ft5x0x_ts_resume;
	register_early_suspend(&ft5x0x_ts->early_suspend);
#endif

	return 0;

err_free_input:
	input_free_device(input_dev);
err_free_irq:
	free_irq(client->irq, ft5x0x_ts);
err_free_thread:
	cancel_work_sync(&ft5x0x_ts->pen_event_work);
	destroy_workqueue(ft5x0x_ts->ts_workqueue);
	i2c_set_clientdata(client, NULL);
err_free_mem:
	kfree(ft5x0x_ts);
err_out:
	return err;
}

static int __devexit ft5x0x_ts_remove(struct i2c_client *client)
{
	struct ft5x0x_ts_data *ft5x0x_ts = i2c_get_clientdata(client);

	unregister_early_suspend(&ft5x0x_ts->early_suspend);
	free_irq(client->irq, ft5x0x_ts);
	input_unregister_device(ft5x0x_ts->input_dev);
	kfree(ft5x0x_ts);
	cancel_work_sync(&ft5x0x_ts->pen_event_work);
	destroy_workqueue(ft5x0x_ts->ts_workqueue);
	i2c_set_clientdata(client, NULL);

	return 0;
}

static const struct i2c_device_id ft5x0x_ts_id[] =
{
	{ FT5X0X_NAME, 0 },{ }
};
MODULE_DEVICE_TABLE(i2c, ft5x0x_ts_id);

static struct i2c_driver ft5x0x_ts_driver =
{
	.probe		= ft5x0x_ts_probe,
	.remove		= __devexit_p(ft5x0x_ts_remove),
	.id_table	= ft5x0x_ts_id,
	.driver	=
	{
		.name	= FT5X0X_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init ft5x0x_ts_init(void)
{
	gpio_set_value(TSC_RST_GPIO, 1);
	msleep(20);
	gpio_set_value(TSC_RST_GPIO, 0);
	msleep(10);
	gpio_set_value(TSC_RST_GPIO, 1);
	msleep(360);

	return i2c_add_driver(&ft5x0x_ts_driver);
}

static void __exit ft5x0x_ts_exit(void)
{
	i2c_del_driver(&ft5x0x_ts_driver);
}

module_init(ft5x0x_ts_init);
module_exit(ft5x0x_ts_exit);

MODULE_AUTHOR("<wenfs@Focaltech-systems.com>");
MODULE_DESCRIPTION("FocalTech ft5x0x TouchScreen driver");
MODULE_LICENSE("GPL");
