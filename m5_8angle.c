// SPDX-License-Identifier: GPL-2.0-or-later
/*
    m5_8angle.c - M5 Stack, 8angle in-tree device driver

    Copyright (C) 2024 Kevin Ceresa <kevin.ceresa@gmail.com>

*/

#include "linux/delay.h"
#include "linux/dev_printk.h"
#include "linux/device.h"
#include "linux/input-event-codes.h"
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/mutex.h>
#include <linux/led-class-multicolor.h>
#include <linux/input.h>

struct m5_8angle_led {
	struct led_classdev_mc cdev;
	struct m5_8angle_data *parent;
	struct mc_subled subled_info[3];
	u8 base_address;
};

struct m5_8angle_analog {
	struct input_dev *cdev;
	struct m5_8angle_data *parent;
	u8 base_address;
};

struct m5_8angle_data {
	struct i2c_client *client;

	struct mutex lock;

	struct m5_8angle_led cdev_led0;
	struct m5_8angle_led cdev_led1;
	struct m5_8angle_led cdev_led2;
	struct m5_8angle_led cdev_led3;
	struct m5_8angle_led cdev_led4;
	struct m5_8angle_led cdev_led5;
	struct m5_8angle_led cdev_led6;
	struct m5_8angle_led cdev_led7;
	struct m5_8angle_led cdev_led8;

	// Analog inputs
	struct m5_8angle_analog vr0;
};

#define M5_8ANGLE_DRIVER "m5stack_8angle"

#define M5_8ANGLE_LED0_REG 0x30
#define M5_8ANGLE_LED1_REG 0x34
#define M5_8ANGLE_LED2_REG 0x38
#define M5_8ANGLE_LED3_REG 0x3C
#define M5_8ANGLE_LED4_REG 0x40
#define M5_8ANGLE_LED5_REG 0x44
#define M5_8ANGLE_LED6_REG 0x48
#define M5_8ANGLE_LED7_REG 0x4C
#define M5_8ANGLE_LED8_REG 0x50

#define M5_8ANGLE_LED0_NAME "m5_8angle_0:rgb:indicator"
#define M5_8ANGLE_LED1_NAME "m5_8angle_1:rgb:indicator"
#define M5_8ANGLE_LED2_NAME "m5_8angle_2:rgb:indicator"
#define M5_8ANGLE_LED3_NAME "m5_8angle_3:rgb:indicator"
#define M5_8ANGLE_LED4_NAME "m5_8angle_4:rgb:indicator"
#define M5_8ANGLE_LED5_NAME "m5_8angle_5:rgb:indicator"
#define M5_8ANGLE_LED6_NAME "m5_8angle_6:rgb:indicator"
#define M5_8ANGLE_LED7_NAME "m5_8angle_7:rgb:indicator"
#define M5_8ANGLE_LED8_NAME "m5_8angle_8:rgb:indicator"

#define M5_8ANGLE_VR0_REG_L 0x00

/* declate the probe and remove functions */
static int rotary_probe(struct i2c_client *client,
			const struct i2c_device_id *device_id);
static void rotary_remove(struct i2c_client *client);

static int m5_8angle_brightness_set_blocking(struct led_classdev *led_cdev,
					     enum led_brightness brt_val)
{
	int err;
	struct led_classdev_mc *mccdev = lcdev_to_mccdev(led_cdev);
	struct m5_8angle_led *cled =
		container_of(mccdev, struct m5_8angle_led, cdev);
	struct m5_8angle_data *drvdata = cled->parent;

	int i;

	mutex_lock(&drvdata->lock);

	//err = led_mc_calc_color_components(mccdev, brt_val);

	//if (err)
	//	dev_err(&drvdata->client->dev,
	//		"Unable to calculate sub-led intensity: %d\n", err);

	/* set the brightness in brightness control registers*/
	for (i = 0; i < mccdev->num_colors; i++) {
		err = i2c_smbus_write_byte_data(
			drvdata->client, cled->base_address + i,
			mccdev->subled_info[i].intensity);
		if (err)
			dev_err(&drvdata->client->dev,
				"Unable to set intensity to color idx %d: %d\n",
				i, err);
	}
	err = i2c_smbus_write_byte_data(drvdata->client, cled->base_address + 3,
					brt_val);

	if (err)
		dev_err(&drvdata->client->dev, "Unable to set brightness: %d\n",
			err);

	mutex_unlock(&drvdata->lock);

	return err;
}

/**
 * @brief init a m5_8angle_led
 */
static void init_m5_8angle_led(struct m5_8angle_led *led, const char *name,
			       const u16 addr)
{
	const struct mc_subled def_subled_red = {
		.color_index = LED_COLOR_ID_RED,
		.brightness = 0,
		.intensity = 0,
		.channel = 0,
	};
	const struct mc_subled def_subled_green = {
		.color_index = LED_COLOR_ID_GREEN,
		.brightness = 0,
		.intensity = 0,
		.channel = 1,
	};
	const struct mc_subled def_subled_blue = {
		.color_index = LED_COLOR_ID_BLUE,
		.brightness = 0,
		.intensity = 0,
		.channel = 2,
	};

	led->cdev.led_cdev.name = name;
	led->cdev.led_cdev.max_brightness = 255;
	led->cdev.led_cdev.brightness_set_blocking =
		m5_8angle_brightness_set_blocking;
	led->cdev.num_colors = 3;
	led->subled_info[0] = def_subled_red;
	led->subled_info[1] = def_subled_green;
	led->subled_info[2] = def_subled_blue;
	led->cdev.subled_info = led->subled_info;
	led->base_address = addr;
}

void poll_analog_input(struct input_dev *dev)
{
	struct m5_8angle_data *pDrv =
		input_get_drvdata(dev); // Retrieve the driver data
	u16 value[8];

	mutex_lock(&pDrv->lock);

	for (size_t i = 0; i < 8; ++i) {
		s32 ret;
		u8 data[2];
		ret = i2c_smbus_write_byte(pDrv->client, i * 2);
		if (ret < 0) {
			dev_err(&pDrv->client->dev,
				"[M5Stack][Rotary] I2C write error");
			mutex_unlock(&pDrv->lock);
			return;
		}

		// Write and read was split due to required ADC timing
		ret = i2c_master_recv(pDrv->client, data, 2);
		if (ret != 2) {
			dev_err(&pDrv->client->dev,
				"[M5Stack][Rotary] I2C read error");
			mutex_unlock(&pDrv->lock);
			return;
		}
		value[i] = (data[1] << 8) | data[0];
		msleep(1);
	}

	input_event(dev, EV_ABS, ABS_X, value[0]);
	input_event(dev, EV_ABS, ABS_Y, value[1]);
	input_event(dev, EV_ABS, ABS_Z, value[2]);
	input_event(dev, EV_ABS, ABS_RX, value[3]);
	input_event(dev, EV_ABS, ABS_RY, value[4]);
	input_event(dev, EV_ABS, ABS_RZ, value[5]);
	input_event(dev, EV_ABS, ABS_GAS, value[6]);
	input_event(dev, EV_ABS, ABS_BRAKE, value[7]);

	input_sync(dev);

	mutex_unlock(&pDrv->lock);
}

/**
 * @brief This function is called on loading the driver
 */
static int rotary_probe(struct i2c_client *client,
			const struct i2c_device_id *device_id)
{
	struct m5_8angle_data *drvdata;
	struct input_dev *idev_0;

	printk("[M5Stack][Rotary] i2c - Probing address 0x43\n");

	if (client->addr != 0x43) {
		printk("[M5Stack][Rotary] i2c - Wrong I2C address!\n");
		return -1;
	}

	drvdata = devm_kzalloc(&client->dev, sizeof(struct m5_8angle_data),
			       GFP_KERNEL);
	if (drvdata == NULL) {
		printk("[M5Stack][Rotary] Failed to allocate driver data");
		return -ENOMEM;
	}

	mutex_init(&drvdata->lock);

	i2c_set_clientdata(client, drvdata);

	idev_0 = devm_input_allocate_device(&client->dev);
	if (idev_0 == NULL) {
		printk("[M5Stack][Rotary] Failed to allocate input device");
		return -ENOMEM;
	}

	input_set_drvdata(idev_0, drvdata);
	drvdata->vr0.cdev = idev_0;

	drvdata->vr0.base_address = M5_8ANGLE_VR0_REG_L;
	idev_0->name = M5_8ANGLE_DRIVER;
	idev_0->phys = M5_8ANGLE_DRIVER "/input0";
	idev_0->id.bustype = BUS_I2C;

	input_set_abs_params(idev_0, ABS_X, 0, 4095, 4, 0);
	input_set_abs_params(idev_0, ABS_Y, 0, 4095, 4, 0);
	input_set_abs_params(idev_0, ABS_Z, 0, 4095, 4, 0);
	input_set_abs_params(idev_0, ABS_RX, 0, 4095, 4, 0);
	input_set_abs_params(idev_0, ABS_RY, 0, 4095, 4, 0);
	input_set_abs_params(idev_0, ABS_RZ, 0, 4095, 4, 0);
	input_set_abs_params(idev_0, ABS_GAS, 0, 4095, 4, 0);
	input_set_abs_params(idev_0, ABS_BRAKE, 0, 4095, 4, 0);

	drvdata->vr0.parent = drvdata;

	// We do not have interrupt, so we must poll the data
	if (input_setup_polling(drvdata->vr0.cdev, poll_analog_input)) {
		dev_err(&client->dev,
			"[M5Stack][Rotary] Failed to setup polling for VR0");
		return -EINVAL;
	}

	input_set_poll_interval(idev_0, 100);
	input_set_min_poll_interval(idev_0, 50);
	input_set_max_poll_interval(idev_0, 200);

	if (input_register_device(idev_0)) {
		dev_err(&client->dev,
			"[M5Stack][Rotary] Failed to register VR0");
		return -EINVAL;
	}
	printk("[M5Stack][Rotary] Registered VR0 successfully");

	drvdata->client = client;
	drvdata->cdev_led0.parent = drvdata;
	drvdata->cdev_led1.parent = drvdata;
	drvdata->cdev_led2.parent = drvdata;
	drvdata->cdev_led3.parent = drvdata;
	drvdata->cdev_led4.parent = drvdata;
	drvdata->cdev_led5.parent = drvdata;
	drvdata->cdev_led6.parent = drvdata;
	drvdata->cdev_led7.parent = drvdata;
	drvdata->cdev_led8.parent = drvdata;

	init_m5_8angle_led(&drvdata->cdev_led0, M5_8ANGLE_LED0_NAME,
			   M5_8ANGLE_LED0_REG);
	init_m5_8angle_led(&drvdata->cdev_led1, M5_8ANGLE_LED1_NAME,
			   M5_8ANGLE_LED1_REG);
	init_m5_8angle_led(&drvdata->cdev_led2, M5_8ANGLE_LED2_NAME,
			   M5_8ANGLE_LED2_REG);
	init_m5_8angle_led(&drvdata->cdev_led3, M5_8ANGLE_LED3_NAME,
			   M5_8ANGLE_LED3_REG);
	init_m5_8angle_led(&drvdata->cdev_led4, M5_8ANGLE_LED4_NAME,
			   M5_8ANGLE_LED4_REG);
	init_m5_8angle_led(&drvdata->cdev_led5, M5_8ANGLE_LED5_NAME,
			   M5_8ANGLE_LED5_REG);
	init_m5_8angle_led(&drvdata->cdev_led6, M5_8ANGLE_LED6_NAME,
			   M5_8ANGLE_LED6_REG);
	init_m5_8angle_led(&drvdata->cdev_led7, M5_8ANGLE_LED7_NAME,
			   M5_8ANGLE_LED7_REG);
	init_m5_8angle_led(&drvdata->cdev_led8, M5_8ANGLE_LED8_NAME,
			   M5_8ANGLE_LED8_REG);

	if (devm_led_classdev_multicolor_register(&client->dev,
						  &drvdata->cdev_led0.cdev))
		return -EINVAL;
	printk("[M5Stack][Rotary] Registered LED 0 successfully");

	if (devm_led_classdev_multicolor_register(&client->dev,
						  &drvdata->cdev_led1.cdev))
		return -EINVAL;
	printk("[M5Stack][Rotary] Registered LED 1 successfully");

	if (devm_led_classdev_multicolor_register(&client->dev,
						  &drvdata->cdev_led2.cdev))
		return -EINVAL;
	printk("[M5Stack][Rotary] Registered LED 2 successfully");

	if (devm_led_classdev_multicolor_register(&client->dev,
						  &drvdata->cdev_led3.cdev))
		return -EINVAL;
	printk("[M5Stack][Rotary] Registered LED 3 successfully");

	if (devm_led_classdev_multicolor_register(&client->dev,
						  &drvdata->cdev_led4.cdev))
		return -EINVAL;
	printk("[M5Stack][Rotary] Registered LED 4 successfully");

	if (devm_led_classdev_multicolor_register(&client->dev,
						  &drvdata->cdev_led5.cdev))
		return -EINVAL;
	printk("[M5Stack][Rotary] Registered LED 5 successfully");

	if (devm_led_classdev_multicolor_register(&client->dev,
						  &drvdata->cdev_led6.cdev))
		return -EINVAL;
	printk("[M5Stack][Rotary] Registered LED 6 successfully");

	if (devm_led_classdev_multicolor_register(&client->dev,
						  &drvdata->cdev_led7.cdev))
		return -EINVAL;
	printk("[M5Stack][Rotary] Registered LED 7 successfully");

	if (devm_led_classdev_multicolor_register(&client->dev,
						  &drvdata->cdev_led8.cdev))
		return -EINVAL;
	printk("[M5Stack][Rotary] Registered LED 8 successfully");

	return 0;
}

/**
 * @brief This function is called on unloading the driver
 */
static void rotary_remove(struct i2c_client *client)
{
	struct m5_8angle_data *drvdata = i2c_get_clientdata(client);

	printk("[M5Stack][Rotary] i2c - Removing driver\n");

	devm_led_classdev_multicolor_unregister(&client->dev,
						&drvdata->cdev_led0.cdev);
	devm_led_classdev_multicolor_unregister(&client->dev,
						&drvdata->cdev_led1.cdev);
	devm_led_classdev_multicolor_unregister(&client->dev,
						&drvdata->cdev_led2.cdev);
	devm_led_classdev_multicolor_unregister(&client->dev,
						&drvdata->cdev_led3.cdev);
	devm_led_classdev_multicolor_unregister(&client->dev,
						&drvdata->cdev_led4.cdev);
	devm_led_classdev_multicolor_unregister(&client->dev,
						&drvdata->cdev_led5.cdev);
	devm_led_classdev_multicolor_unregister(&client->dev,
						&drvdata->cdev_led6.cdev);
	devm_led_classdev_multicolor_unregister(&client->dev,
						&drvdata->cdev_led7.cdev);
	devm_led_classdev_multicolor_unregister(&client->dev,
						&drvdata->cdev_led8.cdev);
}

static struct of_device_id rotary_ids[] = { {
						    .compatible =
							    "m5stack,8angle",
					    },
					    { /* sentinel */ } };
MODULE_DEVICE_TABLE(of, rotary_ids);

static struct i2c_device_id rotary[] = {
	{ "angle8", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, rotary);

static struct i2c_driver rotary_driver = {
    .probe = rotary_probe,
    .remove = rotary_remove,
    .id_table = rotary,
    .driver =
        {
            .name = M5_8ANGLE_DRIVER,
            .of_match_table = rotary_ids,
        },
};

/* This will create the init and exit function automatically */
module_i2c_driver(rotary_driver);

/* Meta Information */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kevin Ceresa");
MODULE_DESCRIPTION("I2C driver for M5Stack 8Angle device");
