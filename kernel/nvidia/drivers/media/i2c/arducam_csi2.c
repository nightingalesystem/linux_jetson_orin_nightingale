/*
 * Copyright (C) 2019 - 2026 Arducam.  All Rights Reserved.
 *
 * This program is free software; you may redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.

 * THE SOFTWARE IS PRELIMINARY AND STILL IN TESTING AND VERIFICATION PHASE AND IS PROVIDED ON AN “AS IS” AND “AS AVAILABLE” BASIS AND IS BELIEVED TO CONTAIN DEFECTS.
 * A PRIMARY PURPOSE OF THIS EARLY ACCESS IS TO OBTAIN FEEDBACK ON PERFORMANCE AND THE IDENTIFICATION OF DEFECT SOFTWARE, HARDWARE AND DOCUMENTATION.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/lcm.h>
#include <linux/crc32.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
//#include <media/v4l2-of.h>
#include <media/v4l2-subdev.h>
#include <media/tegra-v4l2-camera.h>
#include <media/camera_common.h>
#include <media/mc_common.h>

#include <linux/videodev2.h>
#include <media/tegracam_core.h>

#include "arducam.h"
// #include "arducam_ov9281_regs.h"
// #include "arducam_sensor_regs.h"
// #include "arducam_virtual_sensor/virtual_imx219.h"

static int debug = 0;
MODULE_PARM_DESC(debug, "debug");
module_param(debug, int, 0600); /* S_IRUGO */

struct arducam_csi2 {
	struct v4l2_subdev *subdev;
	struct media_pad pad;
	struct i2c_client *client;
	struct arducam_format *supported_formats;
	int num_supported_formats;
	int current_format_idx;
	int current_resolution_idx;
	int lanes;

	struct v4l2_captureparm streamcap;
	struct v4l2_ctrl_handler hdl;
	struct camera_common_data *s_data;
	struct v4l2_ctrl *ctrls[32];
};

static const struct of_device_id arducam_of_match[] = {
	{
		.compatible = "arducam,arducam-csi2",
	},
	{},
};
MODULE_DEVICE_TABLE(of, arducam_of_match);

static int show_csi_params(struct v4l2_subdev *sd);

static struct arducam_csi2 *arducam_get_priv(struct v4l2_subdev *sd)
{
	struct camera_common_data *s_data = 
		container_of(sd, struct camera_common_data, subdev);
	return s_data->priv;
}

static int arducam_csi2_subscribe_event(
	struct v4l2_subdev *sd, struct v4l2_fh *fh,
	struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_EVENT_SOURCE_CHANGE:
		return v4l2_src_change_event_subdev_subscribe(sd, fh, sub);
	case V4L2_EVENT_CTRL:
		return v4l2_ctrl_subdev_subscribe_event(sd, fh, sub);
	default:
		return -EINVAL;
	}
}

static int arducam_readl_reg(struct i2c_client *client,
								   u16 addr, u32 *val)
{
    u16 buf = htons(addr);
    u32 data;
    struct i2c_msg msgs[2] = {
		{
			.addr = client->addr,
			.flags= 0,
			.len = 2,
			.buf = (u8 *)&buf,
		},
		{
			.addr = client->addr,
			.flags= I2C_M_RD,
			.len = 4,
			.buf = (u8 *)&data,
		},
	};

	if(i2c_transfer(client->adapter, msgs, 2) != 2){
		return -1;
	}

	*val = ntohl(data);

	return 0;
}

static int arducam_writel_reg(struct i2c_client *client,
									u16 addr, u32 val)
{
	u8 data[6];
	struct i2c_msg msgs[2] = {
		{
			.addr = client->addr,
			.flags= 0,
			.len = 6,
			.buf = data,
		},
	};
	addr = htons(addr);
	val = htonl(val);
	memcpy(data, &addr, 2);
	memcpy(data + 2, &val, 4);

	if(i2c_transfer(client->adapter, msgs, 1) != 1)
		return -1;
	usleep_range(500, 500);
	return 0;
}

int arducam_read(struct i2c_client *client, u16 addr, u32 *value)
{
	int ret;
	int count = 0;
	while (count++ < I2C_READ_RETRY_COUNT) {
		ret = arducam_readl_reg(client, addr, value);
		if(!ret) {
			v4l2_dbg(1, debug, client, "%s: 0x%02x 0x%04x\n",
				__func__, addr, *value);
			return ret;
		}
	}
	
	v4l2_err(client, "%s: Reading register 0x%02x failed\n",
			 __func__, addr);
	return ret;
}

int arducam_write(struct i2c_client *client, u16 addr, u32 value)
{
	int ret;
	int count = 0;
	while (count++ < I2C_WRITE_RETRY_COUNT) {
		ret = arducam_writel_reg(client, addr, value);
		if(!ret)
			return ret;
	}
	v4l2_err(client, "%s: Write 0x%04x to register 0x%02x failed\n",
			 __func__, value, addr);
	return ret;
}

int arducam_power_on(struct camera_common_data *s_data)
{
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;

	dev_dbg(dev, "%s: power on\n", __func__);

	if (pdata->reset_gpio) {
		if (gpio_cansleep(pdata->reset_gpio))
			gpio_set_value_cansleep(pdata->reset_gpio, 0);
		else
			gpio_set_value(pdata->reset_gpio, 0);
	}

	if (pdata->reset_gpio) {
		if (gpio_cansleep(pdata->reset_gpio))
			gpio_set_value_cansleep(pdata->reset_gpio, 1);
		else
			gpio_set_value(pdata->reset_gpio, 1);
	}

	/* Need to wait for t4 + t5 + t9 time as per the data sheet */
	/* t4 - 200us, t5 - 21.2ms, t9 - 1.2ms */
	usleep_range(23000, 23100);
	return 0;
}

int arducam_power_off(struct camera_common_data *s_data)
{
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;

	dev_dbg(dev, "%s: power off\n", __func__);

	if (pdata->reset_gpio) {
		if (gpio_cansleep(pdata->reset_gpio))
			gpio_set_value_cansleep(pdata->reset_gpio, 0);
		else
			gpio_set_value(pdata->reset_gpio, 0);
	}

	usleep_range(10, 10);

	return 0;
}

static int s_power(struct v4l2_subdev *sd, int on)
{
	struct camera_common_data *s_data = 
		container_of(sd, struct camera_common_data, subdev);
	// struct arducam_csi2 *priv = s_data->priv;
	if (on)
		arducam_power_on(s_data);
	else
		arducam_power_off(s_data);
	return 0;
}

static void set_channel_timeout(struct v4l2_subdev *sd, unsigned long timeout)
{
	struct tegra_channel *tch;
	struct media_pad *pad_csi, *pad_vi;
	struct v4l2_subdev *sd_csi;//, *sd_vi;
	// struct video_device *vdev_vi;

	if (!sd->entity.pads)
		return;

	pad_csi = media_entity_remote_pad(&sd->entity.pads[0]);
	sd_csi = media_entity_to_v4l2_subdev(pad_csi->entity);
	pad_vi = media_entity_remote_pad(&sd_csi->entity.pads[1]);
	// sd_vi = media_entity_to_v4l2_subdev(pad_vi->entity);
	// vdev_vi = media_entity_to_video_device(pad_vi->entity);
	// tch = container_of(vdev_vi, struct tegra_channel, video);
	tch = container_of(pad_vi, struct tegra_channel, pad);
	tch->timeout = msecs_to_jiffies(timeout);
}

#if 1
static void set_channel_lanes(struct v4l2_subdev *sd, u32 lanes)
{
	int i;
	struct media_pad *pad_csi;
	struct v4l2_subdev *sd_csi;
	struct tegra_csi_channel *chan;
	struct tegra_channel *tegra_chan;
	struct tegra_csi_port *port;

	if (!sd->entity.pads)
		return;

	pad_csi = media_entity_remote_pad(&sd->entity.pads[0]);
	sd_csi = media_entity_to_v4l2_subdev(pad_csi->entity);
	chan = to_csi_chan(sd_csi);
	tegra_chan = v4l2_get_subdev_hostdata(sd_csi);

	for (i = 0; i < tegra_chan->valid_ports; i++) {
		port = &chan->ports[i];
		port->lanes = lanes;
		v4l2_dbg(1, debug, sd, "%s: csi port lanes: %d\n", __func__, port->lanes);
	}
}
#endif

static int arducam_restore_ctrls(struct arducam_csi2 *priv)
{
	int i = 0;
	struct v4l2_ctrl *ctrl;
	while ((ctrl = priv->ctrls[i++]) != NULL) {
		ctrl->val = ctrl->default_value;
		ctrl->cur.val = ctrl->default_value;
	}
	return 0;
}

static int arducam_csi2_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct arducam_csi2 *priv = arducam_get_priv(sd);
	v4l2_dbg(1, debug, sd, "%s: stream %s\n",
		__func__, enable ? "on" : "off");

	set_channel_timeout(sd, 2000);
	// set_channel_timeout(sd, -1);
	set_channel_lanes(sd, priv->lanes);
	show_csi_params(sd);
	arducam_restore_ctrls(priv);
	return arducam_write(priv->client, STREAM_ON, enable);
}

long arducam_csi2_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg) {
	struct arducam_csi2 *priv = arducam_get_priv(sd);
	struct arducam_i2c *i2c_data;
	struct arducam_dev *dev_data;
	int ret;
	u32 val;

	v4l2_dbg(1, debug, sd, "%s: ioctl cmd: 0x%x\n", __func__, cmd);
	switch (cmd) {
	case VIDIOC_R_I2C:
		i2c_data = (struct arducam_i2c *)arg;

		if (!i2c_data)
			return -EINVAL;
		
		ret = arducam_read(priv->client,
				i2c_data->reg | READ_SENSOR_I2C_MASK, &val);
		i2c_data->val = (__u16)val;

		return ret;

	case VIDIOC_W_I2C:
		i2c_data = (struct arducam_i2c *)arg;

		if (!i2c_data)
			return -EINVAL;

		ret = arducam_write(priv->client, SENSOR_WR_REG, 
					i2c_data->reg << 16 | i2c_data->val);
		// usleep_range(500, 500);
		return ret;

	case VIDIOC_R_DEV:
		dev_data = (struct arducam_dev *)arg;

		if (!dev_data)
			return -EINVAL;

		ret = arducam_read(priv->client, dev_data->reg, &val);
		dev_data->val = val;

		return ret;

	case VIDIOC_W_DEV:
		dev_data = (struct arducam_dev *)arg;

		if (!dev_data)
			return -EINVAL;

		ret = arducam_write(priv->client,
				dev_data->reg, dev_data->val);

		return ret;
	}
	return 0;
}

static const struct v4l2_subdev_core_ops arducam_csi2_core_ops = {
	.subscribe_event = arducam_csi2_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
	.s_power = s_power,
	.ioctl = arducam_csi2_ioctl,
};

static int show_csi_params(struct v4l2_subdev *sd)
{
	struct media_pad *pad_csi;
	struct v4l2_subdev *sd_csi;
	struct tegra_csi_channel *chan;
	struct tegra_channel *tegra_chan;
	struct tegra_csi_port *port; // = &chan->ports[port_idx];
	int i;

	pad_csi = media_entity_remote_pad(&sd->entity.pads[0]);
	sd_csi = media_entity_to_v4l2_subdev(pad_csi->entity);
	chan = to_csi_chan(sd_csi);
	tegra_chan = v4l2_get_subdev_hostdata(sd_csi);

	for (i = 0; i < tegra_chan->valid_ports; i++) {
		port = &chan->ports[i];
		// port->lanes = 1;
		v4l2_dbg(1, debug, sd, "%s: csi port lanes: %d\n", __func__, port->lanes);
	}
	return 0;
}

static int arducam_csi2_enum_framesizes(
			struct v4l2_subdev *sd,
			struct v4l2_subdev_pad_config *cfg,
			struct v4l2_subdev_frame_size_enum *fse)
{
	int i;
	struct arducam_csi2 *priv = arducam_get_priv(sd);
	struct arducam_format *supported_formats = priv->supported_formats;
	int num_supported_formats = priv->num_supported_formats;
	v4l2_dbg(1, debug, sd, "%s: code = (0x%X), index = (%d)\n",
			 __func__, fse->code, fse->index);
	for (i = 0; i < num_supported_formats; i++) {
		if (fse->code == supported_formats[i].mbus_code) {
			if (fse->index >= supported_formats[i].num_resolution_set)
				return -EINVAL;
			fse->min_width = fse->max_width =
				supported_formats[i].resolution_set[fse->index].width;
			fse->min_height = fse->max_height =
				supported_formats[i].resolution_set[fse->index].height;
			return 0;
		}
	}
	return -EINVAL;
}

static int arducam_csi2_enum_mbus_code(
			struct v4l2_subdev *sd,
			struct v4l2_subdev_pad_config *cfg,
			struct v4l2_subdev_mbus_code_enum *code)
{
	struct arducam_csi2 *priv = arducam_get_priv(sd);
	struct arducam_format *supported_formats = priv->supported_formats;
	int num_supported_formats = priv->num_supported_formats;
	v4l2_dbg(1, debug, sd, "%s: index = (%d)\n", __func__, code->index);
	if (code->index >= num_supported_formats)
		return -EINVAL;

	// code->code = MEDIA_BUS_FMT_Y8_1X8;
	code->code = supported_formats[code->index].mbus_code;

	return 0;
}

static int arducam_csi2_get_fmt_idx_by_code(struct arducam_csi2 *priv,
											u32 mbus_code)
{
	int i;
	struct arducam_format *formats = priv->supported_formats;
	for (i = 0; i < priv->num_supported_formats; i++) {
		if (formats[i].mbus_code == mbus_code)
			return i; 
	}
	return -EINVAL;
}

static int arducam_csi2_set_fmt(struct v4l2_subdev *sd,
								struct v4l2_subdev_pad_config *cfg,
								struct v4l2_subdev_format *format)
{
	int i, j;
	struct arducam_csi2 *priv = arducam_get_priv(sd);
	struct arducam_format *supported_formats = priv->supported_formats;

	show_csi_params(sd);
	format->format.colorspace = V4L2_COLORSPACE_SRGB;
	format->format.field = V4L2_FIELD_NONE;

	v4l2_dbg(1, debug, sd, "%s: code: 0x%X, width: %d, height: %d\n",
			 __func__, format->format.code, format->format.width,
			 	format->format.height);

	i = arducam_csi2_get_fmt_idx_by_code(priv, format->format.code);
	if (i < 0)
		return -EINVAL;

	for (j = 0; j < supported_formats[i].num_resolution_set; j++) {
		if (supported_formats[i].resolution_set[j].width 
				== format->format.width && 
			supported_formats[i].resolution_set[j].height
				== format->format.height) {

			v4l2_dbg(1, debug, sd, "%s: format match.\n", __func__);
			v4l2_dbg(1, debug, sd, "%s: set format to device: %d %d.\n",
				__func__, supported_formats[i].index, j);

			arducam_write(priv->client, PIXFORMAT_INDEX_REG,
				supported_formats[i].index);
			arducam_write(priv->client, RESOLUTION_INDEX_REG, j);

			priv->current_format_idx = i;
			priv->current_resolution_idx = j;

			return 0;
		}
	}
	format->format.width = supported_formats[i].resolution_set[0].width;
	format->format.height = supported_formats[i].resolution_set[0].height;

	arducam_write(priv->client, PIXFORMAT_INDEX_REG,
		supported_formats[i].index);
	arducam_write(priv->client, RESOLUTION_INDEX_REG, 0);

	priv->current_format_idx = i;
	priv->current_resolution_idx = 0;

	return 0;
}

static int arducam_csi2_get_fmt(struct v4l2_subdev *sd,
								struct v4l2_subdev_pad_config *cfg,
								struct v4l2_subdev_format *format)
{
	struct arducam_csi2 *priv = arducam_get_priv(sd);
	struct arducam_format *current_format = 
		&priv->supported_formats[priv->current_format_idx];

	format->format.width =
		current_format->resolution_set[priv->current_resolution_idx].width;
	format->format.height =
		current_format->resolution_set[priv->current_resolution_idx].height;

	format->format.code = current_format->mbus_code;
	format->format.field = V4L2_FIELD_NONE;
	format->format.colorspace = V4L2_COLORSPACE_SRGB;

	v4l2_dbg(1, debug, sd, "%s: width: (%d) height: (%d) code: (0x%X)\n",
		__func__, format->format.width,format->format.height,
			format->format.code);
	return 0;
}

static const struct v4l2_subdev_video_ops arducam_csi2_video_ops = {
	.s_stream = arducam_csi2_s_stream,
};

static const struct v4l2_subdev_pad_ops arducam_csi2_pad_ops = {
	.set_fmt = arducam_csi2_set_fmt,
	.get_fmt = arducam_csi2_get_fmt,
	.enum_mbus_code = arducam_csi2_enum_mbus_code,
	.enum_frame_size = arducam_csi2_enum_framesizes,
};

static const struct v4l2_subdev_ops arducam_csi2_subdev_ops = {
	.core = &arducam_csi2_core_ops,
	.video = &arducam_csi2_video_ops,
	.pad = &arducam_csi2_pad_ops,
};

static const struct media_entity_operations arducam_csi2_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static const struct regmap_config sensor_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
	.use_single_read = true,
	.use_single_write = true,
};

static struct camera_common_pdata *arducam_parse_dt(
	struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct camera_common_pdata *board_priv_pdata;
	const struct of_device_id *match;
	struct camera_common_pdata *ret = NULL;
	int err = 0;
	int gpio;

	if (!np)
		return NULL;

	match = of_match_device(arducam_of_match, dev);
	if (!match) {
		dev_err(dev, "Failed to find matching dt id\n");
		return NULL;
	}

	board_priv_pdata = devm_kzalloc(dev,
									sizeof(*board_priv_pdata), GFP_KERNEL);
	if (!board_priv_pdata)
		return NULL;

	gpio = of_get_named_gpio(np, "reset-gpios", 0);
	if (gpio < 0) {
		if (gpio == -EPROBE_DEFER)
			ret = ERR_PTR(-EPROBE_DEFER);
		dev_err(dev, "reset-gpios not found\n");
		goto error;
	}
	board_priv_pdata->reset_gpio = (unsigned int)gpio;

	err = of_property_read_string(np, "mclk", &board_priv_pdata->mclk_name);
	if (err)
		dev_dbg(dev, "mclk name not present, "
					 "assume sensor driven externally\n");

	err = of_property_read_string(np, "avdd-reg",
								  &board_priv_pdata->regulators.avdd);
	err |= of_property_read_string(np, "iovdd-reg",
								   &board_priv_pdata->regulators.iovdd);
	err |= of_property_read_string(np, "dvdd-reg",
								   &board_priv_pdata->regulators.dvdd);
	if (err)
		dev_dbg(dev, "avdd, iovdd and/or dvdd reglrs. not present, "
					 "assume sensor powered independently\n");

	board_priv_pdata->has_eeprom =
		of_property_read_bool(np, "has-eeprom");

	return board_priv_pdata;

error:
	devm_kfree(dev, board_priv_pdata);

	return ret;
}

static int arducam_s_ctrl(struct v4l2_ctrl *ctrl)
{
	int ret;
	int i = 0;
	unsigned long timeout;
	struct v4l2_ctrl *tmp_ctrl;
	struct arducam_csi2 *priv = 
		container_of(ctrl->handler, struct arducam_csi2, hdl);

	v4l2_dbg(1, debug, priv->client, "%s: cid = (0x%X), value = (%d).\n",
			 __func__, ctrl->id, ctrl->val);

	switch (ctrl->id) {
	case TEGRA_CAMERA_CID_GAIN:
		dev_info(&priv->client->dev, "Gain\n");
		break;
	case TEGRA_CAMERA_CID_EXPOSURE:
		dev_info(&priv->client->dev, "Exposure\n");
		break;
	case TEGRA_CAMERA_CID_FRAME_RATE:
		dev_info(&priv->client->dev, "Frame Rate\n");
		break;
	case TEGRA_CAMERA_CID_GROUP_HOLD:
		dev_info(&priv->client->dev, "Group Hold\n");
		break;
	case TEGRA_CAMERA_CID_SENSOR_MODE_ID:
		dev_info(&priv->client->dev, "Sensor Mode\n");
		break;
	case V4L2_CID_ARDUCAM_TEGRA_DISABLE_TIMEOUT:
		if (ctrl->val == 1)
			set_channel_timeout(priv->subdev, -1);
		else {
			while ((tmp_ctrl = priv->ctrls[i++]) != NULL) {
				if (tmp_ctrl->id == V4L2_CID_ARDUCAM_TEGRA_TIMEOUT) {
					timeout = tmp_ctrl->val;
					set_channel_timeout(priv->subdev, timeout);
					return 0;
				}
			}
		}
		break;
	case V4L2_CID_ARDUCAM_TEGRA_TIMEOUT:
		while ((tmp_ctrl = priv->ctrls[i++]) != NULL) {
			/* First check if the timouet is not disabled */
			if (tmp_ctrl->id == -1) {
				if (tmp_ctrl->val)
					return 0;
				else
					break;
			}
		}

		/* If it is not disabled, set it in HW */
		set_channel_timeout(priv->subdev, ctrl->val);
		return 0;

	default:
		ret = arducam_write(priv->client, CTRL_ID_REG, ctrl->id);
		ret += arducam_write(priv->client, CTRL_VALUE_REG, ctrl->val);
		if (ret < 0)
			return -EINVAL;
		break;
	}
	return 0;
}

static const struct v4l2_ctrl_ops arducam_ctrl_ops = {
	.s_ctrl = arducam_s_ctrl,
};

static int is_raw(int pixformat)
{
	return pixformat >= 0x28 && pixformat <= 0x2D;
}

static u32 bayer_to_mbus_code(int data_type, int bayer_order)
{
	const uint32_t depth8[] = {
        MEDIA_BUS_FMT_SBGGR8_1X8,
		MEDIA_BUS_FMT_SGBRG8_1X8,
        MEDIA_BUS_FMT_SGRBG8_1X8,
		MEDIA_BUS_FMT_SRGGB8_1X8,
		MEDIA_BUS_FMT_Y8_1X8,
	};
    const uint32_t depth10[] = {
        MEDIA_BUS_FMT_SBGGR10_1X10,
		MEDIA_BUS_FMT_SGBRG10_1X10,
        MEDIA_BUS_FMT_SGRBG10_1X10,
		MEDIA_BUS_FMT_SRGGB10_1X10,
		MEDIA_BUS_FMT_Y10_1X10,
	};
    const uint32_t depth12[] = {
        MEDIA_BUS_FMT_SBGGR12_1X12,
		MEDIA_BUS_FMT_SGBRG12_1X12,
        MEDIA_BUS_FMT_SGRBG12_1X12,
		MEDIA_BUS_FMT_SRGGB12_1X12,
		MEDIA_BUS_FMT_Y12_1X12,
	};
    // const uint32_t depth16[] = {
	// 	MEDIA_BUS_FMT_SBGGR16_1X16,
	// 	MEDIA_BUS_FMT_SGBRG16_1X16,
    //     MEDIA_BUS_FMT_SGRBG16_1X16,
	// 	MEDIA_BUS_FMT_SRGGB16_1X16,
    // };
    if (bayer_order < 0 || bayer_order > 4) {
        return 0;
    }

    switch (data_type) {
    case IMAGE_DT_RAW8:
        return depth8[bayer_order];
    case IMAGE_DT_RAW10:
        return depth10[bayer_order];
    case IMAGE_DT_RAW12:
        return depth12[bayer_order];
    }
    return 0;
}

static u32 yuv422_to_mbus_code(int data_type, int order)
{
	const uint32_t depth8[] = {
        MEDIA_BUS_FMT_YUYV8_1X16,
		MEDIA_BUS_FMT_YVYU8_1X16,
        MEDIA_BUS_FMT_UYVY8_1X16,
		MEDIA_BUS_FMT_VYUY8_1X16,
	};

	const uint32_t depth10[] = {
        MEDIA_BUS_FMT_YUYV10_1X20,
		MEDIA_BUS_FMT_YVYU10_1X20,
		MEDIA_BUS_FMT_UYVY10_1X20,
		MEDIA_BUS_FMT_VYUY10_1X20,
	};

	if (order < 0 || order > 3) {
        return 0;
    }

	switch(data_type) {
	case IMAGE_DT_YUV422_8:
		return depth8[order];
	case IMAGE_DT_YUV422_10:
		return depth10[order];
	}
    return 0;
}

static u32 data_type_to_mbus_code(int data_type, int order)
{
    if(is_raw(data_type)) {
		return bayer_to_mbus_code(data_type, order);
	}

	switch(data_type) {
	case IMAGE_DT_YUV422_8:
	case IMAGE_DT_YUV422_10:
		return yuv422_to_mbus_code(data_type, order);
	}
	return 0;
}

static int arducam_get_length_of_set(struct i2c_client *client,
									u16 idx_reg, u16 val_reg)
{
	int ret;
	int index = 0;
	u32 val;
	while (1) {
		ret = arducam_write(client, idx_reg, index);
		ret += arducam_read(client, val_reg, &val);

		if (ret < 0)
			return -1;

		if (val == NO_DATA_AVAILABLE)
			break;
		index++;
	}
	arducam_write(client, idx_reg, 0);
	return index;
}

static int arducam_enum_resolution(struct i2c_client *client,
								struct arducam_format *format)
{
	int index = 0;
	u32 width, height;
	int num_resolution = 0;
	int ret;
	
	num_resolution = arducam_get_length_of_set(client,
						RESOLUTION_INDEX_REG, FORMAT_WIDTH_REG);
	if (num_resolution < 0)
		goto err;

	format->resolution_set = devm_kzalloc(&client->dev,
			sizeof(*(format->resolution_set)) * num_resolution, GFP_KERNEL);
	while (1) {
		ret = arducam_write(client, RESOLUTION_INDEX_REG, index);
		ret += arducam_read(client, FORMAT_WIDTH_REG, &width);
		ret += arducam_read(client, FORMAT_HEIGHT_REG, &height);

		if (ret < 0)
			goto err;

		if (width == NO_DATA_AVAILABLE || height == NO_DATA_AVAILABLE)
			break;

		format->resolution_set[index].width = width;
		format->resolution_set[index].height= height;

		index++;
	}
	format->num_resolution_set = index;
	arducam_write(client, RESOLUTION_INDEX_REG, 0);
	return 0;
err:
	return -ENODEV;
}

/**
 * Reference:
 * https://github.com/alliedvision/linux_nvidia_jetson/blob/
 * c9fff51e08022ca78b1c96bd3835b6ec1273e8d0/kerneltree/kernel/
 * kernel-4.9/drivers/media/i2c/avt_csi2.c#L1146
 * */
static const struct v4l2_ctrl_config tegra_ctrl[] = {
	{
		.ops = &arducam_ctrl_ops,
		.id = V4L2_CID_ARDUCAM_TEGRA_DISABLE_TIMEOUT,
		.name = "Disable frame timeout",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.def = 0,
		.min = 0,
		.max = 1,
		.step = 1,
	},
	{
		.ops = &arducam_ctrl_ops,
		.id = V4L2_CID_ARDUCAM_TEGRA_TIMEOUT,
		.name = "Frame timeout",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 100,
		.max = 12000,
		.step = 1,
		.def = 2000,
	},
};

static const char *arducam_ctrl_get_name(u32 id) {
	switch(id) {
	case V4L2_CID_ARDUCAM_EXT_TRI:
		return "trigger_mode";
	case V4L2_CID_ARDUCAM_FACE_DETECTION:
		return "face_detection";
	case V4L2_CID_EXPOSURE_AUTO:
		return "exposure_auto";
	case V4L2_CID_ARDUCAM_IRCUT:
		return "ircut";
	case V4L2_CID_ARDUCAM_FRAME_RATE:
		return "frame_rate";
	case V4L2_CID_ARDUCAM_EFFECTS:
		return "effects";
	case V4L2_CID_PAN_ABSOLUTE:
		return "pan";
	case V4L2_CID_ZOOM_ABSOLUTE:
		return "zoom";
	case V4L2_CID_ARDUCAM_PAN_X_ABSOLUTE:
		return "Pan Horizontal";
	case V4L2_CID_ARDUCAM_PAN_Y_ABSOLUTE:
		return "Pan Vertical";
	case V4L2_CID_ARDUCAM_ZOOM_PAN_SPEED:
		return "pan_zoom_speed";
	case V4L2_CID_ARDUCAM_HDR:
		return "hdr";
	case V4L2_CID_ARDUCAM_DENOISE:
		return "denoise";
	default:
		return NULL;
	}
}

enum v4l2_ctrl_type arducam_get_v4l2_ctrl_type(u32 id) {
	switch(id) {
	case V4L2_CID_ARDUCAM_EXT_TRI:
		return V4L2_CTRL_TYPE_BOOLEAN;
	case V4L2_CID_ARDUCAM_FACE_DETECTION:
		return V4L2_CTRL_TYPE_BOOLEAN;
	case V4L2_CID_EXPOSURE_AUTO:
		return V4L2_CTRL_TYPE_BOOLEAN;
	case V4L2_CID_ARDUCAM_IRCUT:
		return V4L2_CTRL_TYPE_BOOLEAN;
	case V4L2_CID_ARDUCAM_HDR:
		return V4L2_CTRL_TYPE_BOOLEAN;
	case V4L2_CID_ARDUCAM_FRAME_RATE:
		return V4L2_CTRL_TYPE_INTEGER;
	case V4L2_CID_ARDUCAM_EFFECTS:
		return V4L2_CTRL_TYPE_MENU;
	case V4L2_CID_PAN_ABSOLUTE:
		return V4L2_CTRL_TYPE_MENU;
	case V4L2_CID_ZOOM_ABSOLUTE:
		return V4L2_CTRL_TYPE_INTEGER;
	case V4L2_CID_ARDUCAM_PAN_X_ABSOLUTE:
		return V4L2_CTRL_TYPE_INTEGER;
	case V4L2_CID_ARDUCAM_PAN_Y_ABSOLUTE:
		return V4L2_CTRL_TYPE_INTEGER;
	case V4L2_CID_ARDUCAM_ZOOM_PAN_SPEED:
		return V4L2_CTRL_TYPE_MENU;
	case V4L2_CID_ARDUCAM_DENOISE:
		return V4L2_CTRL_TYPE_MENU;
	default:
		return V4L2_CTRL_TYPE_INTEGER;
	}
}

static const char * const arducam_effect_menu[] = {
	"Normal",
	"Alien",
	"Antique",
	"Black/White",
	"Emboss",
	"Emboss/Color",
	"Grayscale",
	"Negative",
	"Blueish",
	"Greenish",
	"Redish",
	"Posterize 1",
	"Posterize 2",
	"Sepia 1",
	"Sepia 2",
	"Sketch",
	"Solarize",
	"Foggy",
};

static const char * const arducam_pan_menu[] = {
	"Center",
	"Top Left",
	"Top Right",
	"Bottom Left",
	"Bottom Right",
};
static const char * const arducam_zoom_menu[] = {
	"1X",
	"2X",
	"3X",
	"4X",
};
static const char * const arducam_pan_zoom_speed_menu[] = {
	"Immediate",
	"slow",
	"fast",
};

static const char * const arducam_denoise_menu[] = {
	"denoise = -8",
	"denoise = -4",
	"denoise = -2",
	"denoise = -1",
	"denoise = -0.5",
	"denoise = 0",
	"denoise = 0.5",
	"denoise = 1",
	"denoise = 2",
	"denoise = 4",
	"denoise = 8",
};

static const char * const arducam_test_pattern_menu[] = {
	"Disabled",
	"Color Bars",
	"Solid Color",
	"Grey Color Bars",
	"PN9"
};

const char * const* arducam_get_v4l2_ctrl_menu(u32 id) {
	switch(id) {
	case V4L2_CID_ARDUCAM_EFFECTS:
		return arducam_effect_menu;
	case V4L2_CID_PAN_ABSOLUTE:
		return arducam_pan_menu;
	case V4L2_CID_ARDUCAM_ZOOM_PAN_SPEED:
		return arducam_pan_zoom_speed_menu;
	case V4L2_CID_ARDUCAM_DENOISE:
		return arducam_denoise_menu;
	default:
		return NULL;
	}
}

static struct v4l2_ctrl *v4l2_ctrl_new_arducam(struct v4l2_ctrl_handler *hdl,
			const struct v4l2_ctrl_ops *ops,
			u32 id, s64 min, s64 max, u64 step, s64 def)
{
	struct v4l2_ctrl_config cfg = {
		.ops = ops,
		.id = id,
		.name = NULL,
		// .type = V4L2_CTRL_TYPE_INTEGER,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.flags = 0,
		.min = min,
		.max = max,
		.def = def,
		.step = step,
	};
	cfg.name = arducam_ctrl_get_name(id);
	cfg.type = arducam_get_v4l2_ctrl_type(id);
	cfg.qmenu = arducam_get_v4l2_ctrl_menu(id);
	return v4l2_ctrl_new_custom(hdl, &cfg, NULL);
}

static int arducam_enum_controls(struct arducam_csi2 *priv)
{
	int ret;
	int index = 0;
	int i = 0;
	int num_ctrls = 0;
	u32 id, min, max, def, step;
	struct i2c_client *client;
	client = priv->client;
	
	num_ctrls = arducam_get_length_of_set(client,
					CTRL_INDEX_REG, CTRL_ID_REG);

	if (num_ctrls < 0)
		goto err;

	num_ctrls += ARRAY_SIZE(tegra_ctrl);
	priv->s_data->numctrls = num_ctrls;
	v4l2_ctrl_handler_init(&priv->hdl, num_ctrls);

	index = 0;
	while (1) {
		ret = arducam_write(client, CTRL_INDEX_REG, index);
		ret += arducam_read(client, CTRL_ID_REG, &id);
		ret += arducam_read(client, CTRL_MAX_REG, &max);
		ret += arducam_read(client, CTRL_MIN_REG, &min);
		ret += arducam_read(client, CTRL_DEF_REG, &def);
		ret += arducam_read(client, CTRL_STEP_REG, &step);

		if (ret < 0)
			goto err;

		if (id == NO_DATA_AVAILABLE || max == NO_DATA_AVAILABLE ||
			min == NO_DATA_AVAILABLE || def == NO_DATA_AVAILABLE ||
			step == NO_DATA_AVAILABLE)
			break;
		// priv->ctrls[index] = v4l2_ctrl_new_custom(&priv->hdl, & NULL);
		if (arducam_ctrl_get_name(id) != NULL) {
			priv->ctrls[index] = v4l2_ctrl_new_arducam(&priv->hdl,
						&arducam_ctrl_ops, id, min, max, step, def);
			v4l2_dbg(1, debug, priv->client, "%s: new custom ctrl, ctrl: %p.\n",
				__func__, priv->ctrls[index]);
		} else {
			priv->ctrls[index] = v4l2_ctrl_new_std(&priv->hdl,
						&arducam_ctrl_ops, id,
						min, max, step, def);
		}
		index++;
	}

	for (i = 0; i < ARRAY_SIZE(tegra_ctrl); i++) {
		priv->ctrls[index++] =
			v4l2_ctrl_new_custom(&priv->hdl, &tegra_ctrl[i], NULL);
	}

	v4l2_ctrl_handler_setup(&priv->hdl);
	arducam_write(client, CTRL_INDEX_REG, 0);
	return 0;

err:
	return -ENODEV;
}

static int arducam_add_extension_pixformat(struct arducam_csi2 *priv)
{
	int i;
	struct arducam_format *formats = priv->supported_formats;
	for (i = 0; i < priv->num_supported_formats; i++) {
		switch (formats[i].mbus_code){
		case MEDIA_BUS_FMT_SBGGR10_1X10:
		case MEDIA_BUS_FMT_SGBRG10_1X10:
        case MEDIA_BUS_FMT_SGRBG10_1X10:
		case MEDIA_BUS_FMT_SRGGB10_1X10:
		case MEDIA_BUS_FMT_Y10_1X10:
			formats[priv->num_supported_formats] = formats[i];
			formats[priv->num_supported_formats].mbus_code = 
				MEDIA_BUS_FMT_ARDUCAM_Y102Y16_1x16;
			priv->num_supported_formats++;
			return 0;
        case MEDIA_BUS_FMT_SBGGR12_1X12:
		case MEDIA_BUS_FMT_SGBRG12_1X12:
        case MEDIA_BUS_FMT_SGRBG12_1X12:
		case MEDIA_BUS_FMT_SRGGB12_1X12:
		case MEDIA_BUS_FMT_Y12_1X12:
			formats[priv->num_supported_formats] = formats[i];
			formats[priv->num_supported_formats].mbus_code = 
				MEDIA_BUS_FMT_ARDUCAM_Y122Y16_1x16;
			priv->num_supported_formats++;
			return 0;
		}
	}
	return -1;
}

static int arducam_enum_pixformat(struct arducam_csi2 *priv)
{
	int ret = 0;
	u32 mbus_code = 0;
	int pixformat_type;
	int bayer_order;
	int lanes;
	int index = 0;
	int num_pixformat = 0;
	struct i2c_client *client = priv->client;

	num_pixformat = arducam_get_length_of_set(client,
						PIXFORMAT_INDEX_REG, PIXFORMAT_TYPE_REG);

	if (num_pixformat < 0)
		goto err;

	priv->supported_formats = devm_kzalloc(&client->dev,
		sizeof(*(priv->supported_formats)) * (num_pixformat + 1), GFP_KERNEL);

	while (1) {
		ret = arducam_write(client, PIXFORMAT_INDEX_REG, index);
		ret += arducam_read(client, PIXFORMAT_TYPE_REG, &pixformat_type);

		if (pixformat_type == NO_DATA_AVAILABLE)
			break;

		ret += arducam_read(client, MIPI_LANES_REG, &lanes);
		// TODO 什么时候设置lanes比较好?probe的时候估计media link还没有创建
		if (lanes == NO_DATA_AVAILABLE)
			break;

		// if (is_raw(pixformat_type))
		ret += arducam_read(client, PIXFORMAT_ORDER_REG, &bayer_order);

		if (ret < 0)
			goto err;

		mbus_code = data_type_to_mbus_code(pixformat_type, bayer_order);
		priv->supported_formats[index].index = index;
		priv->supported_formats[index].mbus_code = mbus_code;
		priv->supported_formats[index].data_type = pixformat_type;
		if (arducam_enum_resolution(client,
				&priv->supported_formats[index]))
			goto err;

		index++;
	}
	arducam_write(client, PIXFORMAT_INDEX_REG, 0);
	priv->num_supported_formats = index;
	priv->current_format_idx = 0;
	priv->current_resolution_idx = 0;
	priv->lanes = lanes;
	arducam_add_extension_pixformat(priv);
	return 0;

err:
	return -ENODEV;
}

static int arducam_csi2_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	v4l2_dbg(1, debug, sd, "%s:\n", __func__);
	return 0;
}

static int subdev_registered(struct v4l2_subdev *sd)
{
	v4l2_dbg(1, debug, sd, "%s: v4l2 subde registered.\n", __func__);
	return 0;
}

static const struct v4l2_subdev_internal_ops arducam_csi2_int_ops = {
	.open = arducam_csi2_open,
	.registered = &subdev_registered,
};

static int arducam_setup_board(struct arducam_csi2 *priv) {
	int ret;
	u32 device_id;
	u32 sensor_id;
	u32 firmware_version;
	u32 retry = 0;
	struct device *dev = &priv->client->dev;
	arducam_power_on(priv->s_data);
	ret = arducam_read(priv->client, DEVICE_ID_REG, &device_id);
	if (ret || device_id != DEVICE_ID) {
		dev_err(dev, "probe failed\n");
		goto err;
	}

	ret = arducam_read(priv->client, FIRMWARE_VERSION_REG, &firmware_version);
	if (ret)
		dev_err(dev, "failed to read version number.\n");
	else
		dev_info(dev, "firmware version: %d\n", firmware_version);
	
	do {
		ret = arducam_read(priv->client, SENSOR_ID_REG, &sensor_id);
		if (!ret && sensor_id != NO_DATA_AVAILABLE)
			break;
		usleep_range(3000, 3000);
	} while (retry++ < 3);

	if (ret || sensor_id == NO_DATA_AVAILABLE) {
		dev_err(dev, "read sensor id failed\n");

		ret = arducam_read(priv->client, FIRMWARE_SENSOR_ID_REG, &sensor_id);
		dev_err(dev, "Expected Sensor ID: 0x%04X\n", sensor_id);

		goto err;
	}
	
	dev_info(dev, "Sensor ID: 0x%04X\n", sensor_id);

	if (arducam_enum_pixformat(priv)) {
		dev_err(dev, "enum pixformat failed.\n");
		goto err;
	}

	if (arducam_enum_controls(priv)) {
		dev_err(dev, "enum controls failed.\n");
		goto err;
	}

	return 0;
err:
	arducam_power_off(priv->s_data);
	return -ENODEV;
}

static int arducam_csi2_probe(struct i2c_client *client,
							  const struct i2c_device_id *id)
{
	int ret;
	struct camera_common_data *common_data;
	struct arducam_csi2 *priv;
#ifdef ARDUCAM_TEST
	v4l2_dbg(1, debug, client, 
		"this is debug message from arducam-csi2 driver.\n");
#endif
	v4l2_dbg(1, debug, client, "chip found @ 0x%x (%s)\n",
			 client->addr << 1, client->adapter->name);

	priv = devm_kzalloc(&client->dev,
						sizeof(struct arducam_csi2), GFP_KERNEL);

	common_data = devm_kzalloc(&client->dev,
			sizeof(struct camera_common_data), GFP_KERNEL);

	priv->subdev = &common_data->subdev;
	priv->s_data = common_data;
	priv->client = client;

	priv->subdev->ctrl_handler = &priv->hdl;
	priv->subdev->dev = &client->dev;

	common_data->priv = priv;
	common_data->dev = &client->dev;
	common_data->ctrl_handler = &priv->hdl;
	common_data->ctrls = priv->ctrls;

	v4l2_i2c_subdev_init(priv->subdev, client, &arducam_csi2_subdev_ops);
	priv->subdev->internal_ops = &arducam_csi2_int_ops;

	ret = camera_common_initialize(common_data, "arducam");
	if (ret) {
		dev_err(&client->dev,
				"Failed to initialize tegra common for arducam.\n");
		return ret;
	}

	common_data->pdata = arducam_parse_dt(&client->dev);
	common_data->regmap = devm_regmap_init_i2c(client,
											   &sensor_regmap_config);
	if (IS_ERR(common_data->regmap)) {
		dev_err(&client->dev,
				"regmap init failed: %ld\n", PTR_ERR(common_data->regmap));
		return -ENODEV;
	}

	if (arducam_setup_board(priv)) {
		dev_err(&client->dev,
				"Failed to setup board.\n");
		return -ENODEV;
	}

	priv->pad.flags = MEDIA_PAD_FL_SOURCE;
	priv->subdev->entity.ops = &arducam_csi2_media_ops;
	ret = tegra_media_entity_init(&priv->subdev->entity, 1,
								  &priv->pad, true, true);
	if (ret < 0)
		return ret;

	ret = v4l2_async_register_subdev(priv->subdev);
	if (ret < 0)
		return ret;

	dev_info(&client->dev, "sensor %s registered\n",
			 priv->subdev->name);

	return 0;
}

static int arducam_csi2_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	v4l2_dbg(1, debug, client, "driver removed.\n");
	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	return 0;
}

static const struct i2c_device_id arducam_id[] = {
	{"arducam-csi2", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, arducam_id);

static struct i2c_driver arducam_i2c_driver = {
	.driver = {
		.name = "arducam-csi2",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(arducam_of_match),
	},
	.probe = arducam_csi2_probe,
	.remove = arducam_csi2_remove,
	.id_table = arducam_id,
};
module_i2c_driver(arducam_i2c_driver);

MODULE_DESCRIPTION("Arducam CSI-2 Driver");
MODULE_AUTHOR("Arducam");
MODULE_LICENSE("GPL v2");
