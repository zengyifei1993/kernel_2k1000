/*
 * Copyright (c) 2018 Loongson Technology Co., Ltd.
 * Authors:
 *	Zhu Chen <zhuchen@loongson.cn>
 *	Fang Yaling <fangyaling@loongson.cn>
 *	Zhang Dandan <zhangdandan@loongson.cn>
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/export.h>
#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>
#include <linux/pm_runtime.h>
#include <drm/drmP.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_edid.h>

#include "loongson_drv.h"
#include "config-ch7034.h"

#define adapter_to_i2c_client(d) container_of(d, struct i2c_client, adapter)

static struct eep_info{
	struct i2c_adapter *adapter;
	unsigned short addr;
}eeprom_info[2];


/**
 * loongson_connector_best_encoder
 *
 * @connector: point to the drm_connector structure
 *
 * Select the best encoder for the given connector.Used by both the helpers(in the
 * drm_atomic_helper_check_modeset() function)and in the legacy CRTC helpers
 */
static struct drm_encoder *loongson_connector_best_encoder(struct drm_connector
						  *connector)
{
	int enc_id = connector->encoder_ids[0];
	/* pick the encoder ids */
	if (enc_id)
		return drm_encoder_find(connector->dev, enc_id);
	return NULL;
}


/**
 * loongson_do_probe_ddc_edid
 *
 * @adapter: I2C device adapter
 *
 * Try to fetch EDID information by calling I2C driver functions
 */
static unsigned char *loongson_do_probe_ddc_edid(struct i2c_adapter *adapter,unsigned int id)
{
	unsigned char start = 0x0;
	unsigned char *buf = kmalloc(EDID_LENGTH * 2, GFP_KERNEL);
	unsigned int che_tmp = 0,i;
	struct i2c_msg msgs[] = {
		{
			.addr = 0x50,
			.flags = 0,
			.len = 1,
			.buf = &start,
		},{
			.addr = 0x50,
			.flags = I2C_M_RD,
			.len = EDID_LENGTH * 2,
			.buf = buf,
		}
	};
	if (!buf){
		dev_warn(&adapter->dev, "unable to allocate memory for EDID "
			"block.\n");
		return NULL;
	}
	if (i2c_transfer(adapter, msgs, 2) == 2){
		if(buf[126] != 0){
			buf[126] = 0;
			che_tmp = 0;
			for(i = 0;i < 127;i++){
				che_tmp += buf[i];
			}
			buf[127] = 256-(che_tmp)%256;
		}
		return buf;
	}
	dev_warn(&adapter->dev, "unable to read EDID block.\n");
	kfree(buf);
	return NULL;
}


/**
 * loongson_do_probe_ddc_ch7034
 *
 * @adapter: point to i2c_adapter structure
 *
 * Get ch7034 information via I2C
 */
static void loongson_do_probe_ddc_ch7034(struct i2c_adapter *adapter)
{
	/*mtf modify for 9022 config(getting edid preconfig)*/
	int cnt = 100;
	unsigned char old;
	unsigned char rbuf;
	unsigned char *wbuf = kmalloc(2*sizeof(unsigned char), GFP_KERNEL);
	struct i2c_msg msgs = {
		.addr = eeprom_info[0].addr,
		.flags = 1,
		.len = 1,
		.buf = &rbuf,
	};

	if (!wbuf) {
		dev_warn(&adapter->dev, "unable to allocate memory for SIL9022"
			"block.\n");
		return;
	}
	if (i2c_transfer(adapter, &msgs, 1) != 1) {
		printk("read 9022 sys control register err\n");
		return;
	}
	old = rbuf;

	/* Step 1: host requests DDC bus */
	msgs.flags = 0;/*write flag*/
	msgs.len = 2;
	/* 0x1A is 9022's SYS CONTROL register */
	wbuf[0] = 0x1A;
	/* Host requests to use DDC */
	wbuf[1] = old | (1 << 2);
	msgs.buf = wbuf;
	if (i2c_transfer(adapter, &msgs, 1) != 1) {
		printk("exe DDC bus request err\n");
		return;
	}

	/* Step 2: wait the request complete */
	msgs.flags = 1;/*read*/
	msgs.len = 1;
	msgs.buf = &rbuf;
	do {
		cnt--;
		msleep(20);
		if (i2c_transfer(adapter, &msgs, 1) != 1) {
			printk("2 read 9022 sys control register err\n");
			return;
		}

	}while((!(rbuf & 0x2)) && cnt);
	if (!cnt) {
		printk("DDC bus can't be used by host\n");
		return;
	}

	/* Step 3: write 0x1a = 0x06 */
	msgs.flags = 0;/*write*/
	msgs.len = 2;
	wbuf[0] = 0x1A;
	wbuf[1] = (1 << 2) | (1 << 1);
	msgs.buf = wbuf;
	if (i2c_transfer(adapter, &msgs, 1) != 1) {
		printk("write 0x1a = 0x06 err\n");
		return;
	}

	printk("getting edid preconfig success\n");
	return;
}


/**
 * I2C devices supported by I2C device driver
 */
static const struct i2c_device_id dvi_eep_ids[] = {
	{ "dvi-eeprom-edid", 0 },
	{ /* END OF LIST */ }
};

static const struct i2c_device_id vga_eep_ids[] = {
	{ "eeprom-edid1", 2 },
	{ /* END OF LIST */ }
};

static const struct i2c_device_id hdmi_eep_ids[] = {
	{ "eeprom-edid0", 0 },
	{ /* END OF LIST */ }
};

MODULE_DEVICE_TABLE(i2c, eep_ids);


/**
 * hdmi_eep_probe
 *
 * @client: represent an I2C slave device
 * @id: point to i2c_device_id structure
 *
 * Binding hdmi driver
 */
static int hdmi_eep_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	eeprom_info[0].adapter = client->adapter;
	eeprom_info[0].addr = client->addr;
	return 0;
}


/**
 * hdmi_eep_remove
 *
 * @client: identify a single device connected to an I2C bus
 *
 * Unregister hdmi driver from system
 */
static int hdmi_eep_remove(struct i2c_client *client)
{
	i2c_unregister_device(client);
	return 0;
}


/**
 * vga_eep_probe
 *
 * @client: represent an I2C slave device
 * @id: point to i2c_device_id structure
 *
 * Binding vga driver
 */
static int vga_eep_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	eeprom_info[1].adapter = client->adapter;
	eeprom_info[1].addr = client->addr;
	return 0;
}


/**
 * vga_eep_remove
 *
 * @client: identify a single device connected to an I2C bus
 *
 * Unregister vga driver from system
 */
static int vga_eep_remove(struct i2c_client *client)
{
	i2c_unregister_device(client);
	return 0;
}


/**
 * struct i2c_driver can represent an I2C device driver
 *
 * @drive: device driver model driver
 *         represent I2C device driver info,including device model's name and device mode'owner
 * @probe: bind device
 * @remove: unbind device
 * @id_table: list of I2C devices supported by this driver
 *
 * VGA device driver
 */
static struct i2c_driver vga_eep_driver = {
	.driver = {
		.name = "vga_eep-edid",
		.owner = THIS_MODULE,
	},
	.probe = vga_eep_probe,
	.remove = vga_eep_remove,
	.id_table = vga_eep_ids,
};


/**
 * struct i2c_driver can represent an I2C device driver
 *
 * HDMI device driver
 */
static struct i2c_driver hdmi_eep_driver = {
	.driver = {
		.name = "hdmi_eep-edid",
		.owner = THIS_MODULE,
	},
	.probe = hdmi_eep_probe,
	.remove = hdmi_eep_remove,
	.id_table = hdmi_eep_ids,
};


/*
static struct i2c_driver dvi_eep_driver = {
	.driver = {
		.name = "dvi_eep-edid",
		.owner = THIS_MODULE,
	},
	.probe = eep_probe,
	.remove = eep_remove,
	.id_table = dvi_eep_ids,
};
*/


/**
 * loongson_i2c_connector
 *
 * According to i2c bus,acquire screen information
 */
static unsigned char *loongson_i2c_connector(unsigned int id)
{
	unsigned char *edid = NULL;

	DRM_INFO("edid entry\n");
	if (!edid) {
		if (eeprom_info[id].adapter){
			edid = loongson_do_probe_ddc_edid(eeprom_info[id].adapter,id);
		}
	}

	return edid;
}


/**
 * loongson_vga_get_modes
 *
 * @connetcor: central DRM connector control structure
 *
 * Fill in all modes currently valid for the sink into the connector->probed_modes list.
 * It should also update the EDID property by calling drm_mode_connector_update_edid_property().
 */
static int loongson_vga_get_modes(struct drm_connector *connector)
{
	struct loongson_connector *loongson_connector = to_loongson_connector(connector);
	struct edid *edid;
	int ret = 0;

	DRM_INFO("connecotro_id = %d\n",connector->connector_id);
	edid = (struct edid *)loongson_i2c_connector(connector->connector_id);
	if (edid) {
		drm_mode_connector_update_edid_property(connector, edid);
		ret = drm_add_edid_modes(connector, edid);
		kfree(edid);
	}
	DRM_INFO("the vga get modes ret is %d\n",ret);
	return ret;
}

/*
 * loongson_vga_mode_valid
 *
 * @connector: point to the drm connector
 * @mode: point to the drm_connector structure
 *
 * Validate a mode for a connector, irrespective of the specific display configuration
 */
static int loongson_vga_mode_valid(struct drm_connector *connector,
				 struct drm_display_mode *mode)
{
	struct drm_device *dev = connector->dev;
	struct loongson_drm_device *ldev = (struct loongson_drm_device*)dev->dev_private;
	int bpp = 32;
        if(mode->hdisplay % 64)
		return MODE_BAD;
	return MODE_OK;
}

/**
 * loongson_i2c_create
 *
 * @dev: point to drm_device structure
 *
 * Create i2c adapter
 */
struct loongson_i2c_chan *loongson_i2c_create(struct drm_device *dev)
{
	struct loongson_drm_device *ldev = dev->dev_private;
	struct loongson_i2c_chan *i2c;
	int ret;
	int data, clock;

	return i2c;
}


/**
 * loongson_i2c_destroy
 *
 * @i2c: point to loongson_i2c_chan
 *
 * Destroy i2c adapter
 */
void loongson_i2c_destroy(struct loongson_i2c_chan *i2c)
{
	if (!i2c)
		return;
	i2c_del_adapter(&i2c->adapter);
	kfree(i2c);
}


/**
 * loongson_vga_detect
 *
 * @connector: point to drm_connector
 * @force: bool
 *
 * Check to see if anything is attached to the connector.
 * The parameter force is set to false whilst polling,
 * true when checking the connector due to a user request
 */
static enum drm_connector_status loongson_vga_detect(struct drm_connector
						   *connector, bool force)
{
	struct drm_device *dev = connector->dev;
	struct loongson_connector *loongson_connector = to_loongson_connector(connector);
        enum drm_connector_status ret = connector_status_disconnected;
        int r;

	DRM_INFO("loongson_vga_detect\n");
        r = pm_runtime_get_sync(connector->dev->dev);
        if (r < 0)
		ret = connector_status_disconnected;
        r = loongson_vga_get_modes(connector);
        if (r)
	{
		DRM_INFO("loongson_vga_detect: connected");
		ret = connector_status_connected;
	}

	pm_runtime_mark_last_busy(connector->dev->dev);
	pm_runtime_put_autosuspend(connector->dev->dev);

	return ret;
}

/**
 * loongson_connector_destroy
 *
 * @connector: point to the drm_connector structure
 *
 * Clean up connector resources
 */
static void loongson_connector_destroy(struct drm_connector *connector)
{
	struct loongson_connector *loongson_connector = to_loongson_connector(connector);
	loongson_i2c_destroy(loongson_connector->i2c);
	drm_connector_cleanup(connector);
	kfree(connector);
}


/**
 * These provide the minimum set of functions required to handle a connector
 *
 * Helper operations for connectors.These functions are used
 * by the atomic and legacy modeset helpers and by the probe helpers.
 */
static const struct drm_connector_helper_funcs loongson_vga_connector_helper_funcs = {
        .get_modes = loongson_vga_get_modes,
        .mode_valid = loongson_vga_mode_valid,
        .best_encoder = loongson_connector_best_encoder,
};


/**
 * These provide the minimum set of functions required to handle a connector
 *
 * Control connectors on a given device.
 * The functions below allow the core DRM code to control connectors,
 * enumerate available modes and so on.
 */
static const struct drm_connector_funcs loongson_vga_connector_funcs = {
        .dpms = drm_helper_connector_dpms,
        .detect = loongson_vga_detect,
        .fill_modes = drm_helper_probe_single_connector_modes,
        .destroy = loongson_connector_destroy,
};


/**
 * loongson_vga_init
 *
 * @dev: drm device
 * @connector_id:
 *
 * Vga is the interface between host and monitor
 * This function is to init vga
 */
struct drm_connector *loongson_vga_init(struct drm_device *dev,unsigned int connector_id)
{
	struct drm_connector *connector;
	struct loongson_connector *loongson_connector;

	loongson_connector = kzalloc(sizeof(struct loongson_connector), GFP_KERNEL);
	if (!loongson_connector)
		return NULL;

	connector = &loongson_connector->base;

	drm_connector_init(dev, connector,
			   &loongson_vga_connector_funcs, DRM_MODE_CONNECTOR_VGA);

	drm_connector_helper_add(connector, &loongson_vga_connector_helper_funcs);

	drm_connector_register(connector);
	if(connector_id == 0){
		if (i2c_add_driver(&hdmi_eep_driver)) {
			pr_err("i2c-%d No eeprom device register!",(int)hdmi_eep_driver.id_table->driver_data);
		}
	}else{
		if (i2c_add_driver(&vga_eep_driver)) {
			pr_err("i2c-%d No eeprom device register!",(int)vga_eep_driver.id_table->driver_data);
		}
	}

	return connector;
}
