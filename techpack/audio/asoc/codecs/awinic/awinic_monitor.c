/*
 * awinic_monitor.c monitor_module
 *
 * Version: v0.1.17
 *
 * Copyright (c) 2019 AWINIC Technology CO., LTD
 *
 *  Author: Nick Li <liweilei@awinic.com.cn>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#include <linux/module.h>
#include <linux/i2c.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/of.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include "aw882xx.h"
#include "aw882xx_reg.h"
#include "awinic_cali.h"
#include "awinic_monitor.h"


/*monitor  voltage and temperature table*/
static struct aw882xx_low_vol vol_down_table[] = {
	{3500, IPEAK_2P50_A, GAIN_NEG_1P5_DB},
	{3700, IPEAK_2P75_A, GAIN_NEG_1P0_DB},
	{3900, IPEAK_3P00_A, GAIN_NEG_0P5_DB},
};
static struct aw882xx_low_vol vol_up_table[] = {
	{4000, IPEAK_3P50_A, GAIN_NEG_0P0_DB},
	{3800, IPEAK_3P00_A, GAIN_NEG_0P5_DB},
	{3600, IPEAK_2P75_A, GAIN_NEG_1P0_DB},
};
static struct aw882xx_low_temp temp_down_table[] = {
	{-5, IPEAK_2P50_A, GAIN_NEG_6P0_DB, VMAX_063_PERCENTAGE},
	{ 0, IPEAK_2P75_A, GAIN_NEG_4P5_DB, VMAX_075_PERCENTAGE},
	{ 5, IPEAK_3P00_A, GAIN_NEG_3P0_DB, VMAX_086_PERCENTAGE},
};
static struct aw882xx_low_temp temp_up_table[] = {
	{ 7, IPEAK_3P50_A, GAIN_NEG_0P0_DB, VMAX_100_PERCENTAGE},
	{ 2, IPEAK_3P00_A, GAIN_NEG_3P0_DB, VMAX_086_PERCENTAGE},
	{-2, IPEAK_2P75_A, GAIN_NEG_4P5_DB, VMAX_075_PERCENTAGE},
};

/*monitor*/
int aw882xx_monitor_start(struct aw882xx_monitor *monitor)
{
	struct aw882xx *aw882xx = container_of(monitor,
				struct aw882xx, monitor);

	aw_dev_info(aw882xx->dev, "%s: monitor is_enable %d,scene_mode %d\n",
		__func__, aw882xx->monitor.is_enable, aw882xx->scene_mode);

	if (aw882xx->monitor.is_enable &&
		(aw882xx->scene_mode == AW882XX_SPEAKER_MODE)) {
		if (!hrtimer_active(&monitor->timer)) {
			aw_dev_info(aw882xx->dev, "%s: start monitor\n",
				__func__);
			hrtimer_start(&monitor->timer,
				ktime_set(monitor->timer_val/1000,
				(monitor->timer_val%1000)*1000000),
				HRTIMER_MODE_REL);
		}
	}

	return 0;
}

int aw882xx_monitor_stop(struct aw882xx_monitor *monitor)
{
	struct aw882xx *aw882xx = container_of(monitor,
				struct aw882xx, monitor);
	aw_dev_info(aw882xx->dev, "%s: enter\n", __func__);

	if (aw882xx->monitor.is_enable) {
		if (hrtimer_active(&monitor->timer)) {
			aw_dev_info(aw882xx->dev, "%s: stop monitor\n",
				__func__);
			hrtimer_cancel(&monitor->timer);
		}
	}
	return 0;
}

static enum hrtimer_restart
	aw882xx_monitor_timer_func(struct hrtimer *timer)
{
	struct aw882xx_monitor *monitor =
		container_of(timer, struct aw882xx_monitor, timer);
	struct aw882xx *aw882xx = container_of(monitor,
				struct aw882xx, monitor);

	aw_dev_dbg(aw882xx->dev, "%s : enter\n", __func__);

	if (monitor->is_enable)
		schedule_work(&monitor->work);

	return HRTIMER_NORESTART;
}
static int aw882xx_monitor_get_voltage(struct aw882xx *aw882xx,
						unsigned int *vol)
{
	int ret = -1;
	uint16_t local_vol = 0;

	ret = aw882xx_i2c_read(aw882xx, AW882XX_VBAT_REG, vol);
	if (ret < 0) {
		aw_dev_err(aw882xx->dev, "%s: read voltage failed !\n",
			__func__);
		return ret;
	}
	local_vol = (*vol) * AW882XX_MONITOR_VBAT_RANGE
				 / AW882XX_MONITOR_INT_10BIT;

	*vol = local_vol;
	aw_dev_dbg(aw882xx->dev, "%s: chip voltage is %d\n",
		__func__, local_vol);
	return 0;
}

static int aw882xx_monitor_voltage(struct aw882xx *aw882xx,
				struct aw882xx_low_vol *vol_cfg)
{
	int ret = -1;
	int i;
	unsigned int voltage = 0;
	struct aw882xx_monitor *monitor = NULL;

	if (aw882xx == NULL || vol_cfg == NULL) {
		pr_err("%s: pointer is NULL\n", __func__);
		return ret;
	}
	monitor = &aw882xx->monitor;
#ifdef AW_DEBUG
	if (monitor->test_vol == 0) {
		ret = aw882xx_monitor_get_voltage(aw882xx, &voltage);
		if (ret < 0)
			return ret;
	} else {
		voltage = monitor->test_vol;
	}
#else
	ret = aw882xx_monitor_get_voltage(aw882xx, &voltage);
	if (ret < 0)
		return ret;
#endif
	if (monitor->pre_vol > voltage) {
		/* vol down*/
		for (i = 0; i < 3; i++) {
			if (voltage < vol_down_table[i].vol) {
				*vol_cfg = vol_down_table[i];
				break;
			}
		}
		if (i == 3) {
			vol_cfg->ipeak = IPEAK_NONE;
			vol_cfg->gain  = GAIN_NONE;
		}
	} else if (monitor->pre_vol < voltage) {
		/*vol up*/
		for (i = 0; i < 3; i++) {
			if (voltage > vol_up_table[i].vol) {
				*vol_cfg = vol_up_table[i];
				break;
			}
		}
		if (i == 3) {
			vol_cfg->ipeak = IPEAK_NONE;
			vol_cfg->gain  = GAIN_NONE;
		}
	} else {
		/*vol no change*/
		vol_cfg->ipeak = IPEAK_NONE;
		vol_cfg->gain  = GAIN_NONE;
	}
	monitor->pre_vol = voltage;
	return 0;
}
static int aw882xx_monitor_get_temperature(struct aw882xx *aw882xx,  int *temp)
{
	int ret = -1;
	unsigned int reg_val = 0;
	uint16_t local_temp;

	ret = aw882xx_i2c_read(aw882xx, AW882XX_TEMP_REG, &reg_val);
	if (ret < 0) {
		aw_dev_err(aw882xx->dev, "%s: get temperature failed !\n",
			__func__);
		return ret;
	}

	local_temp = reg_val;
	if (local_temp & AW882XX_MONITOR_TEMP_SIGN_MASK)
		local_temp = local_temp | AW882XX_MONITOR_TEMP_NEG_MASK;

	*temp = (int)local_temp;
	aw_dev_dbg(aw882xx->dev, "%s: chip temperature = %d\n",
		__func__, local_temp);
	return 0;
}

static int aw882xx_monitor_temperature(struct aw882xx *aw882xx,
				struct aw882xx_low_temp *temp_cfg)
{
	int ret;
	int i;
	struct aw882xx_monitor *monitor = NULL;
	int  current_temp = 0;

	monitor = &aw882xx->monitor;
#ifdef AW_DEBUG
	if (monitor->test_temp == 0) {
		ret = aw882xx_monitor_get_temperature(aw882xx, &current_temp);
		if (ret)
			return ret;
	} else {
		current_temp = monitor->test_temp;
	}
#else
	ret = aw882xx_monitor_get_temperature(aw882xx, &current_temp);
	if (ret < 0)
		return ret;
#endif
	if (monitor->pre_temp > current_temp) {
		/*temp down*/
		for (i = 0; i < 3; i++) {
			if (current_temp < temp_down_table[i].temp) {
				temp_cfg->ipeak = temp_down_table[i].ipeak;
				temp_cfg->gain = temp_down_table[i].gain;
				temp_cfg->vmax = temp_down_table[i].vmax;
				break;
			}
		}

		if (i == 3) {
			temp_cfg->ipeak = IPEAK_NONE;
			temp_cfg->gain  = GAIN_NONE;
			temp_cfg->vmax  = VMAX_NONE;
		}
	} else if (monitor->pre_temp < current_temp) {
		/*temp up*/
		for (i = 0; i < 3; i++) {
			if (current_temp > temp_up_table[i].temp) {
				temp_cfg->ipeak = temp_up_table[i].ipeak;
				temp_cfg->gain  = temp_up_table[i].gain;
				temp_cfg->vmax  = temp_up_table[i].vmax;
				break;
			}
		}
		if (i == 3) {
			temp_cfg->ipeak = IPEAK_NONE;
			temp_cfg->gain  = GAIN_NONE;
			temp_cfg->vmax  = VMAX_NONE;
		}
	} else {
		/*temp no change*/
		temp_cfg->ipeak = IPEAK_NONE;
		temp_cfg->gain  = GAIN_NONE;
		temp_cfg->vmax  = VMAX_NONE;
	}
	monitor->pre_temp = current_temp;
	return 0;
}

static void aw882xx_monitor_get_cfg(struct aw882xx_low_temp *temp,
					struct aw882xx_low_vol *vol)
{
	if (vol->ipeak == IPEAK_NONE)
		return;

	if (temp->ipeak == IPEAK_NONE) {
		temp->ipeak = vol->ipeak;
		temp->gain  = vol->gain;
		return;
	}

	/*get min ipeak*/
	if (temp->ipeak > vol->ipeak)
		temp->ipeak = vol->ipeak;

	/*get min gain*/
	if (temp->gain < vol->gain)
		temp->gain = vol->gain;

}
static void aw882xx_monitor_set_ipeak(struct aw882xx *aw882xx, uint8_t ipeak)
{
	unsigned int reg_val = 0;
	unsigned int read_reg_val;
	int ret;

	if (ipeak == IPEAK_NONE)
		return;

	ret = aw882xx_i2c_read(aw882xx, AW882XX_SYSCTRL2_REG, &reg_val);
	if (ret < 0) {
		aw_dev_err(aw882xx->dev, "%s: read ipeak failed\n", __func__);
		return;
	}

	read_reg_val = reg_val;
	read_reg_val &= AW882XX_BIT_SYSCTRL2_BST_IPEAK_MASK;

	if (read_reg_val == ipeak) {
		aw_dev_dbg(aw882xx->dev, "%s: ipeak = 0x%x, no change\n",
					__func__, read_reg_val);
		return;
	}
	reg_val &= (~AW882XX_BIT_SYSCTRL2_BST_IPEAK_MASK);
	read_reg_val = ipeak;
	reg_val |= read_reg_val;

	ret = aw882xx_i2c_write(aw882xx, AW882XX_SYSCTRL2_REG, reg_val);
	if (ret < 0) {
		aw_dev_err(aw882xx->dev, "%s: write ipeak failed\n", __func__);
		return;
	}
	aw_dev_dbg(aw882xx->dev, "%s: set reg val = 0x%x, ipeak = 0x%x\n",
					__func__, reg_val, ipeak);
}
static void aw882xx_monitor_set_gain(struct aw882xx *aw882xx, uint8_t gain)
{
	unsigned int reg_val = 0;
	unsigned int read_reg_val;
	int ret;

	if (gain == GAIN_NONE)
		return;

	ret = aw882xx_i2c_read(aw882xx, AW882XX_HAGCCFG4_REG, &reg_val);
	if (ret < 0) {
		aw_dev_err(aw882xx->dev, "%s: read gain failed\n", __func__);
		return;
	}

	read_reg_val = reg_val;
	read_reg_val = read_reg_val >> AW882XX_BIT_HAGCCFG4_GAIN_SHIFT;

	if (read_reg_val == gain) {
		aw_dev_dbg(aw882xx->dev, "%s: gain = 0x%x, no change\n",
				__func__, read_reg_val);
		return;
	}
	reg_val &= AW882XX_BIT_HAGCCFG4_GAIN_MASK;
	read_reg_val = gain;
	reg_val |= (read_reg_val << AW882XX_BIT_HAGCCFG4_GAIN_SHIFT);

	ret = aw882xx_i2c_write(aw882xx, AW882XX_HAGCCFG4_REG, reg_val);
	if (ret < 0) {
		aw_dev_err(aw882xx->dev, "%s: write gain failed\n", __func__);
		return;
	}
	aw_dev_dbg(aw882xx->dev, "%s: set reg val = 0x%x, gain = 0x%x\n",
			__func__, reg_val, gain);
}
static void aw882xx_monitor_set_vmax(struct aw882xx *aw882xx, uint32_t vmax)
{
	uint32_t local_vmax = vmax;
	int ret;
	struct aw882xx_chan_info *chan_info = &aw882xx->chan_info;

	if (vmax == VMAX_NONE)
		return;

	ret = aw_write_data_to_dsp(INDEX_PARAMS_ID_RX_VMAX,
				&local_vmax, sizeof(uint32_t),
				chan_info->channel);
	if (ret)
		aw_dev_err(aw882xx->dev, "%s: dsp_msg_write error\n",
			__func__);

	aw_dev_dbg(aw882xx->dev, "%s: set vmax = 0x%x\n", __func__, vmax);
}
static void aw882xx_monitor_work(struct aw882xx *aw882xx)
{
	struct aw882xx_low_vol vol_cfg;
	struct aw882xx_low_temp temp_cfg;
	int ret;

	if (aw882xx == NULL) {
		pr_err("%s: pointer is NULL\n", __func__);
		return;
	}
	if (aw882xx->cali.status != 0) {
		aw_dev_info(aw882xx->dev, "%s: done nothing while start cali",
			__func__);
		return;
	}

	ret = aw882xx_monitor_voltage(aw882xx, &vol_cfg);
	if (ret < 0) {
		aw_dev_err(aw882xx->dev, "%s: monitor voltage failed\n",
			__func__);
		return;
	}

	ret = aw882xx_monitor_temperature(aw882xx, &temp_cfg);
	if (ret < 0) {
		aw_dev_err(aw882xx->dev, "%s: monitor temperature failed\n",
			__func__);
		return;
	}
	aw_dev_dbg(aw882xx->dev, "%s: vol: ipeak = 0x%x, gain = 0x%x\n",
			__func__, vol_cfg.ipeak, vol_cfg.gain);
	aw_dev_dbg(aw882xx->dev, "%s: temp: ipeak = 0x%x, gain = 0x%x, vmax = 0x%x\n",
		__func__, temp_cfg.ipeak, temp_cfg.gain, temp_cfg.vmax);

	aw882xx_monitor_get_cfg(&temp_cfg, &vol_cfg);

	aw882xx_monitor_set_ipeak(aw882xx, temp_cfg.ipeak);

	aw882xx_monitor_set_gain(aw882xx, temp_cfg.gain);

	aw882xx_monitor_set_vmax(aw882xx, temp_cfg.vmax);
}

static int aw882xx_get_hmute(struct aw882xx *aw882xx)
{
	unsigned int reg_val = 0;
	int ret;

	aw_dev_dbg(aw882xx->dev, "%s: enter\n", __func__);

	aw882xx_i2c_read(aw882xx, AW882XX_SYSCTRL2_REG, &reg_val);
	if ((~AW882XX_HMUTE_MASK) & reg_val)
		ret = 1;
	else
		ret = 0;

	return ret;
}
static void aw882xx_monitor_work_func(struct work_struct *work)
{
	struct aw882xx_monitor *monitor = container_of(work,
				struct aw882xx_monitor, work);
	struct aw882xx *aw882xx = container_of(monitor,
				struct aw882xx, monitor);

	aw_dev_info(aw882xx->dev, "%s: enter\n", __func__);
	mutex_lock(&aw882xx->lock);
	if (!aw882xx_get_hmute(aw882xx)) {
		aw882xx_monitor_work(aw882xx);
		aw882xx_monitor_start(monitor);
	}
	mutex_unlock(&aw882xx->lock);
}


#ifdef AW_DEBUG
static ssize_t aw882xx_vol_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct aw882xx *aw882xx = dev_get_drvdata(dev);
	uint32_t vol = 0;
	int ret = -1;

	if (count == 0)
		return 0;

	ret = kstrtouint(buf, 0, &vol);
	if (ret < 0)
		return ret;

	aw_dev_info(aw882xx->dev, "%s: vol set =%d\n", __func__, vol);
	aw882xx->monitor.test_vol = vol;

	return count;
}

static ssize_t aw882xx_vol_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aw882xx *aw882xx = dev_get_drvdata(dev);
	ssize_t len = 0;
	uint32_t local_vol = aw882xx->monitor.test_vol;

	len += snprintf(buf+len, PAGE_SIZE-len,
		"aw882xx vol: %d\n", local_vol);
	return len;
}
static ssize_t aw882xx_temp_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct aw882xx *aw882xx = dev_get_drvdata(dev);
	int32_t temp = 0;
	int ret = -1;

	if (count == 0)
		return 0;

	ret = kstrtoint(buf, 0, &temp);
	if (ret < 0)
		return ret;

	aw_dev_info(aw882xx->dev, "%s: temp set =%d\n", __func__, temp);
	aw882xx->monitor.test_temp = temp;

	return count;
}

static ssize_t aw882xx_temp_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aw882xx *aw882xx = dev_get_drvdata(dev);
	ssize_t len = 0;
	int32_t local_temp = aw882xx->monitor.test_temp;

	len += snprintf(buf+len, PAGE_SIZE-len,
		"aw882xx vol: %d\n", local_temp);
	return len;
}

static DEVICE_ATTR(vol, S_IWUSR | S_IRUGO,
	aw882xx_vol_show, aw882xx_vol_store);
static DEVICE_ATTR(temp, S_IWUSR | S_IRUGO,
	aw882xx_temp_show, aw882xx_temp_store);
#endif

static ssize_t aw882xx_monitor_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct aw882xx *aw882xx = dev_get_drvdata(dev);
	uint32_t enable = 0;
	int ret = -1;

	if (count == 0)
		return 0;

	ret = kstrtouint(buf, 0, &enable);
	if (ret < 0)
		return ret;

	aw_dev_info(aw882xx->dev, "%s:monitor  enable set =%d\n",
		__func__, enable);
	aw882xx->monitor.is_enable = enable;
	if (enable)
		schedule_work(&aw882xx->monitor.work);

	return count;
}

static ssize_t aw882xx_monitor_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aw882xx *aw882xx = dev_get_drvdata(dev);
	ssize_t len = 0;
	uint32_t local_enable;

	local_enable = aw882xx->monitor.is_enable;
	len += snprintf(buf+len, PAGE_SIZE-len,
		"aw882xx monitor enable: %d\n", local_enable);
	return len;
}

static DEVICE_ATTR(monitor, S_IWUSR | S_IRUGO,
	aw882xx_monitor_show, aw882xx_monitor_store);


static struct attribute *aw882xx_monitor_attr[] = {
	&dev_attr_monitor.attr,
#ifdef AW_DEBUG
	&dev_attr_vol.attr,
	&dev_attr_temp.attr,
#endif
	NULL
};

static struct attribute_group aw882xx_monitor_attr_group = {
	.attrs = aw882xx_monitor_attr,
};

void init_aw882xx_monitor(struct aw882xx_monitor *monitor)
{
	int ret;
	struct aw882xx *aw882xx = container_of(monitor,
				struct aw882xx, monitor);
	aw_dev_info(aw882xx->dev, "%s: enter\n", __func__);
	hrtimer_init(&monitor->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	monitor->timer.function = aw882xx_monitor_timer_func;
	INIT_WORK(&monitor->work, aw882xx_monitor_work_func);
	monitor->pre_vol = 0;
	monitor->pre_temp = 0;
#ifdef AW_DEBUG
	monitor->test_vol = 0;
	monitor->test_temp = 0;
#endif
	ret = sysfs_create_group(&aw882xx->dev->kobj,
				&aw882xx_monitor_attr_group);
	if (ret < 0) {
		aw_dev_err(aw882xx->dev, "%s error creating sysfs attr files\n",
			__func__);
	}
}

/*****************************************************
 * device tree parse monitor param
 *****************************************************/
void aw882xx_parse_monitor_dt(struct aw882xx_monitor *monitor)
{
	int ret;
	struct aw882xx *aw882xx = container_of(monitor,
				struct aw882xx, monitor);
	struct device_node *np = aw882xx->dev->of_node;

	ret = of_property_read_u32(np, "monitor-flag", &monitor->is_enable);
	if (ret) {
		monitor->is_enable = AW882XX_MONITOR_DEFAULT_FLAG;
		dev_err(aw882xx->dev,
			"%s: monitor-flag get failed ,user default value!\n",
			__func__);
	} else {
		dev_info(aw882xx->dev, "%s: monitor-flag = %d\n",
			__func__, monitor->is_enable);
	}

	ret = of_property_read_u32(np, "monitor-timer-val",
				&monitor->timer_val);
	if (ret) {
		monitor->timer_val = AW882XX_MONITOR_DEFAULT_TIMER_VAL;
		dev_err(aw882xx->dev,
			"%s: monitor-timer-val get failed,user default value!\n",
			__func__);
	} else {
		dev_info(aw882xx->dev, "%s: monitor-timer-val = %d\n",
			__func__, monitor->timer_val);
	}

}


