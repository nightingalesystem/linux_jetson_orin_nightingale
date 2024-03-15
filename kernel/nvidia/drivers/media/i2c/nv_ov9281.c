/*
 * ov9281.c - ov9281 sensor driver
 *
 * Copyright (c) 2016-2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#define DEVEL

#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/module.h>

#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <media/tegra_v4l2_camera.h>
#include <media/tegracam_core.h>
#include <media/camera_common.h>
#include "../platform/tegra/camera/camera_gpio.h"

#include "ov9281_mode_tbls.h"

/* OV9281 Registers */
#define OV9281_SC_MODE_SELECT_ADDR	0x0100
#define OV9281_SC_MODE_SELECT_STREAMING	0x01
#define OV9281_SC_CHIP_ID_HIGH_ADDR	0x300A
#define OV9281_SC_CHIP_ID_LOW_ADDR	0x300B
#define OV9281_SC_CTRL_SCCB_ID_ADDR	0x302B
#define OV9281_SC_CTRL_3B_ADDR		0x303B
#define OV9281_SC_CTRL_3B_SCCB_ID2_NACK_EN	(1 << 0)
#define OV9281_SC_CTRL_3B_SCCB_PGM_ID_EN	(1 << 1)

#define OV9281_GROUP_HOLD_ADDR		0x3208
#define OV9281_GROUP_HOLD_START		0x00
#define OV9281_GROUP_HOLD_END		0x10
#define OV9281_GROUP_HOLD_LAUNCH_LBLANK	0x60
#define OV9281_GROUP_HOLD_LAUNCH_VBLANK	0xA0
#define OV9281_GROUP_HOLD_LAUNCH_IMMED	0xE0
#define OV9281_GROUP_HOLD_BANK_0	0x00
#define OV9281_GROUP_HOLD_BANK_1	0x01

#define OV9281_EXPO_HIGH_ADDR		0x3500
#define OV9281_EXPO_MID_ADDR		0x3501
#define OV9281_EXPO_LOW_ADDR		0x3502

#define OV9281_GAIN_SHIFT_ADDR		0x3507
#define OV9281_GAIN_HIGH_ADDR		0x3508
#define OV9281_GAIN_LOW_ADDR		0x3509

#define OV9281_TIMING_VTS_HIGH_ADDR	0x380E
#define OV9281_TIMING_VTS_LOW_ADDR	0x380F
#define OV9281_TIMING_FORMAT1		0x3820
#define OV9281_TIMING_FORMAT1_VBIN	(1 << 1)
#define OV9281_TIMING_FORMAT1_FLIP	(1 << 2)
#define OV9281_TIMING_FORMAT2		0x3821
#define OV9281_TIMING_FORMAT2_HBIN	(1 << 0)
#define OV9281_TIMING_FORMAT2_MIRROR	(1 << 2)
#define OV9281_TIMING_RST_FSIN_HIGH_ADDR	0x3826
#define OV9281_TIMING_RST_FSIN_LOW_ADDR	0x3827

#define OV9281_OTP_BUFFER_ADDR		0x3D00
#define OV9281_OTP_BUFFER_SIZE		32
#define OV9281_OTP_STR_SIZE		(OV9281_OTP_BUFFER_SIZE * 2)
#define OV9281_FUSE_ID_OTP_BUFFER_ADDR	0x3D00
#define OV9281_FUSE_ID_OTP_BUFFER_SIZE	16
#define OV9281_FUSE_ID_STR_SIZE		(OV9281_FUSE_ID_OTP_BUFFER_SIZE * 2)
#define OV9281_OTP_PROGRAM_CTRL_ADDR	0x3D80
#define OV9281_OTP_LOAD_CTRL_ADDR	0x3D81
#define OV9281_OTP_LOAD_CTRL_OTP_RD	0x01

#define OV9281_PRE_CTRL00_ADDR		0x5E00
#define OV9281_PRE_CTRL00_TEST_PATTERN_EN	(1 << 7)

/* OV9281 Other Stuffs */
#define OV9281_DEFAULT_GAIN		0x0010 /* 1.0x real gain */
#define OV9281_MIN_GAIN			0x0001
#define OV9281_MAX_GAIN			0x1FFF

#define OV9281_DEFAULT_FRAME_LENGTH	0x071C
#define OV9281_MIN_FRAME_LENGTH		0x0001
#define OV9281_MAX_FRAME_LENGTH		0xFFFF
#define OV9281_FRAME_LENGTH_1SEC	(0x40d * 120) /* TODO: try to calc */

#define OV9281_MIN_EXPOSURE_COARSE	0x00000001
#define OV9281_MAX_EXPOSURE_COARSE	0x000FFFFF
#define OV9281_DEFAULT_EXPOSURE_COARSE	0x00002A90

#define OV9281_MAX_WIDTH		1280
#define OV9281_MAX_HEIGHT		800

#define OV9281_DEFAULT_MODE		OV9281_MODE_1280X800
#define OV9281_DEFAULT_WIDTH		OV9281_MAX_WIDTH
#define OV9281_DEFAULT_HEIGHT		OV9281_MAX_HEIGHT
#define OV9281_DEFAULT_DATAFMT		MEDIA_BUS_FMT_SBGGR10_1X10
#define OV9281_DEFAULT_CLK_FREQ		26000000

#define OV9281_DEFAULT_I2C_ADDRESS_C0		(0xc0 >> 1)
#define OV9281_DEFAULT_I2C_ADDRESS_20		(0x20 >> 1)
#define OV9281_DEFAULT_I2C_ADDRESS_PROGRAMMABLE	(0xe0 >> 1)


/*static const ov9281_reg op_10bit[] = {
	{0x030d, 0x50},
	{0x3662, 0x05},
	{OV9281_TABLE_END, 0x00},
};

static const ov9281_reg op_8bit[] = {
	{0x030d, 0x60},
	{0x3662, 0x07},
	{OV9281_TABLE_END, 0x00},
};
*/

static const struct of_device_id ov9281_of_match[] = {
	{ .compatible = "nvidia,ov9281", },
	{ },
};
MODULE_DEVICE_TABLE(of, ov9281_of_match);

static const u32 ctrl_cid_list[] = {
	TEGRA_CAMERA_CID_GAIN,
	TEGRA_CAMERA_CID_EXPOSURE,
	TEGRA_CAMERA_CID_FRAME_RATE,
	TEGRA_CAMERA_CID_SENSOR_MODE_ID,
};

struct ov9281 {
	struct i2c_client		*i2c_client;
	struct v4l2_subdev		*subdev;
	u16				fine_integ_time;
	u16				fsync;
	u32				frame_length;	
	struct camera_common_data	*s_data;
	struct tegracam_device		*tc_dev;		
};

static const struct regmap_config ov9281_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
	.use_single_read = true,
	.use_single_write = true,

};

/* Register/regmap stuff */
static int ov9281_read_reg(struct camera_common_data *s_data, u16 addr, u8 *val)
{
	int err = 0;
	u32 reg_val = 0;

	err = regmap_read(s_data->regmap, addr, &reg_val);
	*val = reg_val & 0xff;

	return err;
}

static int ov9281_write_reg(struct camera_common_data *s_data, u16 addr, u8 val)
{
	int err = 0;

	err = regmap_write(s_data->regmap, addr, val);
	if (err)
		dev_err(s_data->dev, "%s: i2c write failed, 0x%x = %x",
			__func__, addr, val);

	return err;
}

static int ov9281_write_table(struct ov9281 *priv, const ov9281_reg table[])
{
	return regmap_util_write_table_8(priv->s_data->regmap, table, NULL, 0,
					 OV9281_TABLE_WAIT_MS,
					 OV9281_TABLE_END);
}

/*CAMERA CONTROLS*/
static int ov9281_set_group_hold(struct tegracam_device *tc_dev, bool val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = s_data->dev;	

	int err;

	dev_dbg(dev, "%s: Setting group hold control to: %u\n", __func__, val);

	if (val) 
	{
		// group hold start 
		err = ov9281_write_reg(s_data, OV9281_GROUP_HOLD_ADDR,
				       (OV9281_GROUP_HOLD_START |
					OV9281_GROUP_HOLD_BANK_0));
		if (err)
			goto fail;	
	} 
	else
	{
		// group hold end 
		err = ov9281_write_reg(s_data, OV9281_GROUP_HOLD_ADDR,
				       (OV9281_GROUP_HOLD_END |
					OV9281_GROUP_HOLD_BANK_0));
		// quick launch 
		err |= ov9281_write_reg(s_data,
				       OV9281_GROUP_HOLD_ADDR,
				       (OV9281_GROUP_HOLD_LAUNCH_VBLANK |
					OV9281_GROUP_HOLD_BANK_0));
		if (err)
			goto fail;		
	}
	return 0;

fail:
	dev_err(dev, "%s: Group hold control error\n", __func__);
	return err;
}

static int ov9281_set_gain(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = s_data->dev;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode_prop_idx];
	struct ov9281 *priv = (struct ov9281 *)tegracam_get_privdata(tc_dev);
	ov9281_reg regs[4];
	u16 gain;
	int err;

	if (val < mode->control_properties.min_gain_val)
	{
		gain = mode->control_properties.min_gain_val;
		dev_dbg(dev, "%s: value: %d%s\n", __func__, (u32)val," is below minimum gain for this mode ! Set to minimum.");
	}
	else if (val > mode->control_properties.max_gain_val)
	{
		gain = mode->control_properties.max_gain_val;
		dev_dbg(dev, "%s: value: %d%s\n", __func__, (u32)val," is above maximum gain for this mode ! Set to maximum.");
	}
	else
		gain = val;		

	/*If 0x3503[2] = 0, gain[7:0] is real gain format,
	where low 4 bits are fraction bits (e.g., 0x10
	is 1x gain, 0x28 is 2.5x gain)*/

	regs[0].addr = OV9281_GAIN_SHIFT_ADDR; 
	regs[0].val = 0x03; 
	/*00: No shift
	01: Left shift 1 bit
	10: Left shift 2 bit
	11: Left shift 3 bit*/
	regs[1].addr = OV9281_GAIN_HIGH_ADDR;
	regs[1].val = (gain >> 8) & 0x1f;
	regs[2].addr = OV9281_GAIN_LOW_ADDR;
	regs[2].val = gain & 0xff;
	regs[3].addr = OV9281_TABLE_END;
	regs[3].val = 0;
	
	err = ov9281_write_table(priv, regs);
	err=0;
	

	if (err)
		goto fail;

	dev_dbg(dev, "%s: camera gain set to: %d\n", __func__, gain);
	return 0;

fail:
	dev_err(dev, "%s: GAIN control error\n", __func__);
	return err;
}

static int ov9281_set_exposure(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = s_data->dev;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode_prop_idx];
	struct ov9281 *priv = (struct ov9281 *)tegracam_get_privdata(tc_dev);
	ov9281_reg regs[4];
	u32 coarse_time;
	int err;

	if(val < mode->control_properties.min_exp_time.val)
	{
		coarse_time = (u32)mode->control_properties.min_exp_time.val;
		dev_dbg(dev, "%s: value: %d%s\n", __func__, (u32)val," is below minimum exposure for this mode ! Set to minimum.");
	}
	else if(val > mode->control_properties.max_exp_time.val)
	{
		coarse_time = (u32)mode->control_properties.max_exp_time.val;
		dev_dbg(dev, "%s: value: %d%s\n", __func__, (u32)val," is above maximum exposure for this mode ! Set to maximum.");
	}
	else
		coarse_time = (u32)val;
		

	regs[0].addr = OV9281_EXPO_HIGH_ADDR;
	regs[0].val = (coarse_time >> 16) & 0x0f;
	regs[1].addr = OV9281_EXPO_MID_ADDR;
	regs[1].val = (coarse_time >> 8) & 0xff;
	regs[2].addr = OV9281_EXPO_LOW_ADDR;
	regs[2].val = (coarse_time & 0xff);
	regs[3].addr = OV9281_TABLE_END;
	regs[3].val = 0;

	//ov9281_set_group_hold(priv); important ?
	err = ov9281_write_table(priv, regs);
	err=0;


	if (err)
		goto fail;

	dev_dbg(dev, "%s: camera exposure set to: %d\n", __func__, coarse_time);
	return 0;

fail:
	dev_err(dev, "%s: EXPOSURE control error\n", __func__);
	return err;
}

static int ov9281_set_frame_rate(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = s_data->dev;
	struct ov9281 *priv = (struct ov9281 *)tegracam_get_privdata(tc_dev);
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode_prop_idx];

	ov9281_reg regs[5];
	u16 frame_length;	
	int err;	

	u8 act_vts_hi;
	u8 act_vts_lo;
	u16 act_vts;
	int fps = -1;
	int i;
	/*Break in non free run mode*/    
	if (priv->fsync != OV9281_FSYNC_NONE)
	{
		dev_dbg(dev, "%s: frame length control executed, but in trigger mode has no effect, because frame rate is controlled by trigger frequency.\n", __func__);

		ov9281_read_reg(s_data, OV9281_TIMING_VTS_HIGH_ADDR, &act_vts_hi);
		ov9281_read_reg(s_data, OV9281_TIMING_VTS_LOW_ADDR, &act_vts_lo);
		act_vts = (act_vts_hi<<8)  + act_vts_lo;
		// find value from lookup table
		for (i = 0; i < sizeof(fps_lookup_table) / sizeof(u16); i++) {
		    if(act_vts == fps_lookup_table[i]) {
			fps = i;
			break;
		    }
		}
		dev_dbg(dev, "%s:TIMING_VTS: 0x%hhx:%hhx (%hu), fps = %d.\n", __func__,act_vts_hi,act_vts_lo,act_vts, fps );
		//return 0;		
	}
	
	if(val < mode->control_properties.min_framerate)
	{
		dev_dbg(dev, "%s: value: %d%s\n", __func__, (u16)val," is below minimum fps for this mode ! Set to minimum.");		
		frame_length = fps_lookup_table[mode->control_properties.min_framerate];
		val = mode->control_properties.min_framerate;		
	}
	else if(val > mode->control_properties.max_framerate)
	{
		dev_err(dev, "%s: value: %d%s\n", __func__, (u16)val," is above maximum fps for this mode ! Set to maximum.");
		frame_length = fps_lookup_table[mode->control_properties.max_framerate];
		val = mode->control_properties.max_framerate;
	}
	else {
		frame_length = fps_lookup_table[val];
	}


	regs[0].addr = OV9281_TIMING_VTS_HIGH_ADDR;
	regs[0].val = (frame_length >> 8) & 0xff;
	regs[1].addr = OV9281_TIMING_VTS_LOW_ADDR;
	regs[1].val = (frame_length) & 0xff;
	regs[2].addr = OV9281_TABLE_END;
	regs[2].val = 0;


	//ov9281_set_group_hold(priv);
	err = ov9281_write_table(priv, regs);
	if (err)
		goto fail;	

	dev_info(dev, "%s: camera fps set to: %d  => TIMING_VTS: %hu\n", __func__, (u32)val, frame_length);
	dev_info(dev, "new driver version");

	return 0;

fail:
	dev_err(dev, "%s: FRAME_RATE control error\n", __func__);
	return err;
}


static struct tegracam_ctrl_ops ov9281_ctrl_ops = {
	.numctrls = ARRAY_SIZE(ctrl_cid_list),
	.ctrl_cid_list = ctrl_cid_list,
	.set_gain = ov9281_set_gain,
	.set_exposure = ov9281_set_exposure,
	.set_frame_rate = ov9281_set_frame_rate,
	.set_group_hold = ov9281_set_group_hold,
};

/* NVIDIA camera_common stuff */
static int ov9281_power_on(struct camera_common_data *s_data)
{
	int err = 0;	
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;	

	dev_dbg(dev, "%s: power on\n", __func__);

	if (pdata && pdata->power_on) {
		err = pdata->power_on(pw);
		if (err)
			dev_err(dev, "%s failed.\n", __func__);
		else
			pw->state = SWITCH_ON;
		return err;
	}

	if (pw->reset_gpio) {
		if (gpio_cansleep(pw->reset_gpio))
			gpio_set_value_cansleep(pw->reset_gpio, 0);
		else
			gpio_set_value(pw->reset_gpio, 0);
	}

	if (unlikely(!(pw->avdd || pw->iovdd || pw->dvdd)))
		goto skip_power_seqn;

	usleep_range(10, 20);	

	if (pw->avdd) {
		err = regulator_enable(pw->avdd);
		if (err)
			goto avdd_fail;
	}

	if (pw->dvdd) {
		err = regulator_enable(pw->dvdd);
		if (err)
			goto dvdd_fail;
	}

	if (pw->iovdd) {
		err = regulator_enable(pw->iovdd);
		if (err)
			goto iovdd_fail;
	}

	usleep_range(5350, 5360);

	skip_power_seqn:
	if (pw->reset_gpio) {
		if (gpio_cansleep(pw->reset_gpio))
			gpio_set_value_cansleep(pw->reset_gpio, 1);
		else
			gpio_set_value(pw->reset_gpio, 1);
	}

	/*err = ov9281_i2c_addr_assign(priv, priv->i2c_client->addr);
	if (err)
		goto addr_assign_fail;
	*/

	usleep_range(66000, 67000);

	pw->state = SWITCH_ON;
	return 0;

/*addr_assign_fail:
	if (pw->iovdd)
		regulator_disable(pw->iovdd);*/

iovdd_fail:
	if (pw->dvdd)
		regulator_disable(pw->dvdd);

dvdd_fail:
	if (pw->avdd)
		regulator_disable(pw->avdd);

avdd_fail:
	dev_err(dev, "%s failed.\n", __func__);

	return -ENODEV;
}

static int ov9281_power_off(struct camera_common_data *s_data)
{
	int err = 0;
	//struct ov9281 *priv = (struct ov9281 *)s_data->priv;	
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;
	
	dev_dbg(dev, "%s: power off\n", __func__);
	//ov9281_write_table(priv, ov9281_mode_table[OV9281_MODE_STOP_STREAM]);

	if (pdata && pdata->power_off) {
		err = pdata->power_off(pw);
		if (err) {
			dev_err(dev, "%s failed.\n", __func__);
			return err;
		}
	} else {
		if (pw->reset_gpio) {
			if (gpio_cansleep(pw->reset_gpio))
				gpio_set_value_cansleep(pw->reset_gpio, 0);
			else
				gpio_set_value(pw->reset_gpio, 0);
		}

		usleep_range(10, 10);

		if (pw->dvdd)
			regulator_disable(pw->dvdd);
		if (pw->iovdd)
			regulator_disable(pw->iovdd);
		if (pw->avdd)
			regulator_disable(pw->avdd);
	}

	pw->state = SWITCH_OFF;
	return 0;
}

static int ov9281_power_put(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;

	if (unlikely(!pw))
		return -EFAULT;

	if (likely(pw->dvdd))
		devm_regulator_put(pw->dvdd);

	if (likely(pw->avdd))
		devm_regulator_put(pw->avdd);

	if (likely(pw->iovdd))
		devm_regulator_put(pw->iovdd);

	pw->dvdd = NULL;
	pw->avdd = NULL;
	pw->iovdd = NULL;

	if (likely(pw->reset_gpio))
		gpio_free(pw->reset_gpio);

	return 0;
}

static int ov9281_power_get(struct tegracam_device *tc_dev)
{
	/*struct camera_common_power_rail *pw = &priv->power;
	struct camera_common_pdata *pdata = priv->pdata;
	struct device *dev = &priv->i2c_client->dev;
	const char *mclk_name;*/

	struct device *dev = tc_dev->dev;
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct clk *parent;
	int err = 0;

	if (!pdata) {
		dev_err(dev, "pdata missing\n");
		return -EFAULT;
	}

	if (pdata->mclk_name) {
		pw->mclk = devm_clk_get(dev, pdata->mclk_name);
		if (IS_ERR(pw->mclk)) {
			dev_err(dev, "unable to get clock %s\n",
				pdata->mclk_name);
			return PTR_ERR(pw->mclk);
		}

		if (pdata->parentclk_name) {
			parent = devm_clk_get(dev, pdata->parentclk_name);
			if (IS_ERR(parent)) {
				dev_err(dev, "unable to get parent clock %s",
					pdata->parentclk_name);
			} else
				clk_set_parent(pw->mclk, parent);
		}
	}

	/* analog 2.8v */
	if (pdata->regulators.avdd)
		err |= camera_common_regulator_get(dev,
				&pw->avdd, pdata->regulators.avdd);
	/* IO 1.8v */
	if (pdata->regulators.iovdd)
		err |= camera_common_regulator_get(dev,
				&pw->iovdd, pdata->regulators.iovdd);
	/* dig 1.2v */
	if (pdata->regulators.dvdd)
		err |= camera_common_regulator_get(dev,
				&pw->dvdd, pdata->regulators.dvdd);
	if (err) {
		dev_err(dev, "%s: unable to get regulator(s)\n", __func__);
		goto done;
	}

	/* Reset or ENABLE GPIO */
	pw->reset_gpio = pdata->reset_gpio;
	err = gpio_request(pw->reset_gpio, "cam_reset_gpio");
	if (err < 0) {
		dev_err(dev, "%s: unable to request reset_gpio (%d)\n",
			__func__, err);
		goto done;
	}

done:
	pw->state = SWITCH_OFF;
	return err;
}

static struct camera_common_pdata *ov9281_parse_dt(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct device_node *np = dev->of_node;
	struct camera_common_pdata *board_priv_pdata;
	const struct of_device_id *match;
	struct camera_common_pdata *ret = NULL;	
	int err = 0;
	int gpio;

    match = of_match_device(ov9281_of_match, dev);
	if (!match) {
		dev_err(dev, "Failed to find ov9281 matching dt id\n");
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

	return board_priv_pdata;
}

static int ov9281_set_mode(struct tegracam_device *tc_dev)
{
   	struct ov9281 *priv = (struct ov9281 *)tegracam_get_privdata(tc_dev);
	struct camera_common_data *s_data = tc_dev->s_data;		
	struct device *dev = s_data->dev;
	

	int err = 0; 

    dev_dbg(dev, "%s: writting camera mode: %d\n", __func__, s_data->mode);
	err = ov9281_write_table(priv, mode_table[s_data->mode]);
	
	/*WRITE FSYNC TABLE */
	if(priv->fsync > OV9281_FSYNC_NONE)
	{
		dev_dbg(dev, "%s: writting fsync mode: %d\n", __func__, priv->fsync);
		err |= ov9281_write_table(priv, ov9281_fsync_table[priv->fsync]);		
	}	

	if (err)
		return err;

	return 0;
}

static int ov9281_start_streaming(struct tegracam_device *tc_dev)
{
	int err;
	struct ov9281 *priv = (struct ov9281 *)tegracam_get_privdata(tc_dev);	
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = s_data->dev;	

	
	if(priv->fsync == OV9281_FSYNC_SLAVE_TRIGGER)
	{
		dev_dbg(dev, "%s: camera started in trigger mode.\n", __func__);	
		/*camera stream stopped and chip waits in standby for trigger*/	
		err = ov9281_write_table(priv, mode_table[OV9281_STOP_STREAM]);	
	}
	else
	{
		dev_dbg(dev, "%s: camera started in continuous streaming mode.\n", __func__);
		err = ov9281_write_table(priv, mode_table[OV9281_START_STREAM]);		
	}
	
	return err;
}

static int ov9281_stop_streaming(struct tegracam_device *tc_dev)
{
	int err;
	struct ov9281 *priv = (struct ov9281 *)tegracam_get_privdata(tc_dev);

	err = ov9281_write_table(priv, mode_table[OV9281_STOP_STREAM]);

	usleep_range(50000, 51000);

	return err;
}

static struct camera_common_sensor_ops ov9281_common_ops = {
	.numfrmfmts = ARRAY_SIZE(ov9281_frmfmt),
	.frmfmt_table = ov9281_frmfmt,
	.power_on =  ov9281_power_on,
	.power_off = ov9281_power_off,
	.write_reg = ov9281_write_reg,
	.read_reg =  ov9281_read_reg,
	.parse_dt =  ov9281_parse_dt, 
	.power_get = ov9281_power_get,
	.power_put = ov9281_power_put,
	.set_mode =  ov9281_set_mode,
	.start_streaming = ov9281_start_streaming, 
	.stop_streaming = ov9281_stop_streaming,  
};


static int ov9281_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_dbg(&client->dev, "%s:\n", __func__);

	return 0;
}

static const struct v4l2_subdev_internal_ops ov9281_subdev_internal_ops = {
	.open = ov9281_open,
};



/* Driver probe helper stuff */
static int ov9281_verify_chip_id(struct ov9281 *priv)
{
	struct i2c_client *client = priv->i2c_client;
	struct camera_common_data *s_data = priv->s_data;
	u8 chip_id_hi, chip_id_lo;
	u16 chip_id;
	int err;

	err = ov9281_read_reg(s_data, OV9281_SC_CHIP_ID_HIGH_ADDR, &chip_id_hi);
	if (err) {
		dev_dbg(&client->dev, "Failed to read chip ID\n");
		return err;
	}
	err = ov9281_read_reg(s_data, OV9281_SC_CHIP_ID_LOW_ADDR, &chip_id_lo);
	if (err) {
		dev_dbg(&client->dev, "Failed to read chip ID\n");
		return err;
	}

	chip_id = (chip_id_hi << 8) | chip_id_lo;
	if (chip_id != 0x9281) {
		dev_err(&client->dev, "Read unknown chip ID 0x%04x\n", chip_id);
		return -EINVAL;
	}

	return 0;
}

static int ov9281_board_setup(struct ov9281 *priv)
{
	struct camera_common_data *s_data = priv->s_data;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;	
	struct device_node *np = dev->of_node;
	int err = 0;
	const char *fsync_str;

	if (pdata->mclk_name) {
		err = camera_common_mclk_enable(s_data);
		if (err) {
			dev_err(dev, "error turning on mclk (%d)\n", err);
			goto done;
		}
	}

	err = ov9281_power_on(s_data);
	if (err) {
		dev_err(dev, "error during power on sensor (%d)\n", err);
		goto err_power_on;
	}


	/* FSYNC property read */  
    err = of_property_read_string(np, "fsync", &fsync_str);
    	if (!err && fsync_str && (strcmp(fsync_str, "master") == 0)) {
		priv->fsync = OV9281_FSYNC_MASTER;
		dev_info(dev, "ov9281 camera configured in fsync master mode \n");
	} else if (!err && fsync_str && (strcmp(fsync_str, "slave-trigger") == 0)) {
		priv->fsync = OV9281_FSYNC_SLAVE_TRIGGER;
		dev_info(dev, "ov9281 camera configured in fsync slave-trigger mode \n");
	} else {
		priv->fsync = OV9281_FSYNC_NONE;
		dev_info(dev, "ov9281 camera configured in free-run mode \n");
	}

	/* Probe sensor model id registers */
	err = ov9281_verify_chip_id(priv);
	if (err == 0) {
		dev_info(dev, "ov9281 sensor ID matches\n");
		goto done;
	}

	ov9281_power_off(s_data);

err_power_on:
	if (pdata->mclk_name)
		camera_common_mclk_disable(s_data);

done:
	return err;
}

static int ov9281_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct tegracam_device *tc_dev;
	struct ov9281 *priv;	
	int err;

	dev_info(dev, "probing ov9281 v4l2 sensor at addr 0x%0x\n", client->addr);

	if (!IS_ENABLED(CONFIG_OF) || !client->dev.of_node)
		return -EINVAL;

	priv = devm_kzalloc(dev,
			sizeof(struct ov9281), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	tc_dev = devm_kzalloc(dev,
			sizeof(struct tegracam_device), GFP_KERNEL);
	if (!tc_dev)
		return -ENOMEM;

	priv->i2c_client = tc_dev->client = client;
	tc_dev->dev = dev;
	strncpy(tc_dev->name, "ov9281", sizeof(tc_dev->name));
	tc_dev->dev_regmap_config = &ov9281_regmap_config;
	tc_dev->sensor_ops = &ov9281_common_ops;
	tc_dev->v4l2sd_internal_ops = &ov9281_subdev_internal_ops;
	tc_dev->tcctrl_ops = &ov9281_ctrl_ops;

	err = tegracam_device_register(tc_dev);
	if (err) {
		dev_err(dev, "ov9281 tegra camera driver registration failed\n");
		return err;
	}
	priv->tc_dev = tc_dev;
	priv->s_data = tc_dev->s_data;
	priv->subdev = &tc_dev->s_data->subdev;
	tegracam_set_privdata(tc_dev, (void *)priv);

	err = ov9281_board_setup(priv);
	if (err) {
		tegracam_device_unregister(tc_dev);
		dev_err(dev, "board setup failed\n");
		return err;
	}

	err = tegracam_v4l2subdev_register(tc_dev, true);
	if (err) {
		dev_err(dev, "tegra camera subdev registration failed\n");
		return err;
	}

	dev_info(dev, "detected ov9281 sensor\n");

	return 0;
}

static int ov9281_remove(struct i2c_client *client)
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct ov9281 *priv = (struct ov9281 *)s_data->priv;

	tegracam_v4l2subdev_unregister(priv->tc_dev);
	tegracam_device_unregister(priv->tc_dev);

	return 0;
}





static const struct i2c_device_id ov9281_id[] = {
#ifdef DEVEL
        { "ov9281_devel", 0 },
#else    
        { "ov9281", 0 },
#endif 
	{ }
};

MODULE_DEVICE_TABLE(i2c, ov9281_id);

static struct i2c_driver ov9281_i2c_driver = {
	.driver = {
#ifdef DEVEL
		.name = "ov9281_devel",
#else    
        .name = "ov9281",
#endif            
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ov9281_of_match),
	},
	.probe = ov9281_probe,
	.remove = ov9281_remove,
	.id_table = ov9281_id,
};

module_i2c_driver(ov9281_i2c_driver);

MODULE_DESCRIPTION("SoC Camera driver for Omnivison OV9281");
MODULE_AUTHOR("NVIDIA Corporation");
MODULE_LICENSE("GPL v2");


