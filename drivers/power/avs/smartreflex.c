/*
 * OMAP SmartReflex Voltage Control
 *
 * Author: Thara Gopinath	<thara@ti.com>
 *
 * Copyright (C) 2012 Texas Instruments, Inc.
 * Thara Gopinath <thara@ti.com>
 *
 * Copyright (C) 2008 Nokia Corporation
 * Kalle Jokiniemi
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 * Lesly A M <x0080970@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/power/smartreflex.h>
#include <plat/voltage.h>

#define SMARTREFLEX_NAME_LEN	16
#define NVALUE_NAME_LEN		40
#define SR_DISABLE_TIMEOUT	200

/* sr_list contains all the instances of smartreflex module */
static LIST_HEAD(sr_list);

static struct omap_sr_class_data *sr_class;
static struct omap_sr_pmic_data *sr_pmic_data;
static struct dentry		*sr_dbg_dir;
static atomic_t sr_driver_ready;

static inline void sr_write_reg(struct omap_sr *sr, unsigned offset, u32 value)
{
	__raw_writel(value, (sr->base + offset));
}

static inline void sr_modify_reg(struct omap_sr *sr, unsigned offset, u32 mask,
					u32 value)
{
	u32 reg_val;

	/*
	 * Smartreflex error config register is special as it contains
	 * certain status bits which if written a 1 into means a clear
	 * of those bits. So in order to make sure no accidental write of
	 * 1 happens to those status bits, do a clear of them in the read
	 * value. This mean this API doesn't rewrite values in these bits
	 * if they are currently set, but does allow the caller to write
	 * those bits.
	 */
	if (sr->ip_type == SR_TYPE_V1 && offset == ERRCONFIG_V1)
		mask |= ERRCONFIG_STATUS_V1_MASK;
	else if (sr->ip_type == SR_TYPE_V2 && offset == ERRCONFIG_V2)
		mask |= ERRCONFIG_VPBOUNDINTST_V2;

	reg_val = __raw_readl(sr->base + offset);
	reg_val &= ~mask;

	value &= mask;

	reg_val |= value;

	__raw_writel(reg_val, (sr->base + offset));
}

static inline u32 sr_read_reg(struct omap_sr *sr, unsigned offset)
{
	return __raw_readl(sr->base + offset);
}

static struct omap_sr *_sr_lookup(struct voltagedomain *voltdm)
{
	struct omap_sr *sr_info;

	if (!voltdm) {
		pr_err("%s: Null voltage domain passed!\n", __func__);
		return ERR_PTR(-EINVAL);
	}

	list_for_each_entry(sr_info, &sr_list, node) {
		if (voltdm == sr_info->voltdm)
			return sr_info;
	}

	return ERR_PTR(-ENODATA);
}

static irqreturn_t sr_interrupt(int irq, void *data)
{
	struct omap_sr *sr_info = data;
	u32 status = 0;

	switch (sr_info->ip_type) {
	case SR_TYPE_V1:
		/* Read the status bits */
		status = sr_read_reg(sr_info, ERRCONFIG_V1);

		/* Clear them by writing back */
		sr_write_reg(sr_info, ERRCONFIG_V1, status);
		break;
	case SR_TYPE_V2:
		/* Read the status bits */
		status = sr_read_reg(sr_info, IRQSTATUS);

		/* Clear them by writing back */
		sr_write_reg(sr_info, IRQSTATUS, status);
		break;
	default:
		dev_err(&sr_info->pdev->dev, "UNKNOWN IP type %d\n",
			sr_info->ip_type);
		return IRQ_NONE;
	}

	if (sr_class->notify)
		sr_class->notify(sr_info, status);

	return IRQ_HANDLED;
}

static void sr_set_clk_length(struct omap_sr *sr)
{
	struct clk *sys_ck = NULL;
	u32 sys_clk_speed;

	if (cpu_is_omap34xx())
		sys_ck = clk_get(NULL, "sys_ck");
	else if (cpu_is_omap44xx())
		sys_ck = clk_get(NULL, "sys_clkin_ck");
	else if (cpu_is_omap54xx())
		sys_ck = clk_get(NULL, "sys_clkin");
	/* DONOT populate a default sysclk name unless we know about it */

	if (IS_ERR_OR_NULL(sys_ck)) {
		dev_err(&sr->pdev->dev, "%s: unable to get sys clk\n",
			__func__);
		return;
	}

	sys_clk_speed = clk_get_rate(sys_ck);
	clk_put(sys_ck);

	switch (sys_clk_speed) {
	case 12000000:
		sr->clk_length = SRCLKLENGTH_12MHZ_SYSCLK;
		break;
	case 13000000:
		sr->clk_length = SRCLKLENGTH_13MHZ_SYSCLK;
		break;
	case 19200000:
		sr->clk_length = SRCLKLENGTH_19MHZ_SYSCLK;
		break;
	case 26000000:
		sr->clk_length = SRCLKLENGTH_26MHZ_SYSCLK;
		break;
	case 38400000:
		sr->clk_length = SRCLKLENGTH_38MHZ_SYSCLK;
		break;
	default:
		dev_err(&sr->pdev->dev, "%s: Invalid sysclk value: %d\n",
			__func__, sys_clk_speed);
		break;
	}
}

static void sr_set_regfields(struct omap_sr *sr)
{
	/*
	 * For time being these values are defined in smartreflex.h
	 * and populated during init. May be they can be moved to board
	 * file or pmic specific data structure. In that case these structure
	 * fields will have to be populated using the pdata or pmic structure.
	 */
	if (cpu_is_omap34xx() || cpu_is_omap44xx()) {
		sr->err_weight = OMAP3430_SR_ERRWEIGHT;
		sr->err_maxlimit = OMAP3430_SR_ERRMAXLIMIT;
		sr->accum_data = OMAP3430_SR_ACCUMDATA;
		if (!(strcmp(sr->name, "smartreflex_mpu_iva"))) {
			sr->senn_avgweight = OMAP3430_SR1_SENNAVGWEIGHT;
			sr->senp_avgweight = OMAP3430_SR1_SENPAVGWEIGHT;
		} else {
			sr->senn_avgweight = OMAP3430_SR2_SENNAVGWEIGHT;
			sr->senp_avgweight = OMAP3430_SR2_SENPAVGWEIGHT;
		}
	} else if (cpu_is_omap54xx()) {
		/*
		 * Keeping this duplicated until GS80 characterization is
		 * complete
		 */
		sr->err_weight = OMAP3430_SR_ERRWEIGHT;
		sr->err_maxlimit = OMAP3430_SR_ERRMAXLIMIT;
		sr->accum_data = OMAP54XX_SR_ACCUMDATA;
		sr->senn_avgweight = OMAP54XX_SR2_SENNAVGWEIGHT;
		sr->senp_avgweight = OMAP54XX_SR2_SENPAVGWEIGHT;
	}
}

static void sr_start_vddautocomp(struct omap_sr *sr)
{
	if (!sr_class || !(sr_class->enable) || !(sr_class->configure)) {
		dev_warn(&sr->pdev->dev,
			 "%s: smartreflex class driver not registered\n",
			 __func__);
		return;
	}

	if (!sr_class->enable(sr))
		sr->autocomp_active = true;
}

static void sr_stop_vddautocomp(struct omap_sr *sr)
{
	if (!sr_class || !(sr_class->disable)) {
		dev_warn(&sr->pdev->dev,
			 "%s: smartreflex class driver not registered\n",
			 __func__);
		return;
	}

	if (sr->autocomp_active) {
		sr_class->disable(sr, 1);
		sr->autocomp_active = false;
	}
}

/*
 * This function handles the intializations which have to be done
 * only when both sr device and class driver regiter has
 * completed. This will be attempted to be called from both sr class
 * driver register and sr device intializtion API's. Only one call
 * will ultimately succeed.
 *
 * Currently this function registers interrupt handler for a particular SR
 * if smartreflex class driver is already registered and has
 * requested for interrupts and the SR interrupt line in present.
 */
static int sr_late_init(struct omap_sr *sr_info)
{
	struct omap_sr_data *pdata = sr_info->pdev->dev.platform_data;
	struct resource *mem;
	int ret = 0;

	if (sr_class->notify && sr_class->notify_flags && sr_info->irq) {
		ret = request_irq(sr_info->irq, sr_interrupt,
				  0, sr_info->name, sr_info);
		if (ret)
			goto error;
		disable_irq(sr_info->irq);
	}

	if (pdata && pdata->enable_on_init)
		sr_start_vddautocomp(sr_info);

	return ret;

error:
	iounmap(sr_info->base);
	mem = platform_get_resource(sr_info->pdev, IORESOURCE_MEM, 0);
	release_mem_region(mem->start, resource_size(mem));
	list_del(&sr_info->node);
	dev_err(&sr_info->pdev->dev, "%s: ERROR in registering"\
		"interrupt handler. Smartreflex will"\
		"not function as desired\n", __func__);
	kfree(sr_info);

	return ret;
}

static void sr_v1_disable(struct omap_sr *sr)
{
	int timeout = 0;
	int errconf_val = ERRCONFIG_MCUACCUMINTST | ERRCONFIG_MCUVALIDINTST |
			ERRCONFIG_MCUBOUNDINTST;

	/* Enable MCUDisableAcknowledge interrupt */
	sr_modify_reg(sr, ERRCONFIG_V1,
		      ERRCONFIG_MCUDISACKINTEN, ERRCONFIG_MCUDISACKINTEN);

	/* SRCONFIG - disable SR */
	sr_modify_reg(sr, SRCONFIG, SRCONFIG_SRENABLE, 0x0);

	/* Disable all other SR interrupts and clear the status as needed */
	if (sr_read_reg(sr, ERRCONFIG_V1) & ERRCONFIG_VPBOUNDINTST_V1)
		errconf_val |= ERRCONFIG_VPBOUNDINTST_V1;
	sr_modify_reg(sr, ERRCONFIG_V1,
		      (ERRCONFIG_MCUACCUMINTEN | ERRCONFIG_MCUVALIDINTEN |
		      ERRCONFIG_MCUBOUNDINTEN | ERRCONFIG_VPBOUNDINTEN_V1),
		      errconf_val);

	/*
	 * Wait for SR to be disabled.
	 * wait until ERRCONFIG.MCUDISACKINTST = 1. Typical latency is 1us.
	 */
	sr_test_cond_timeout((sr_read_reg(sr, ERRCONFIG_V1) &
			     ERRCONFIG_MCUDISACKINTST), SR_DISABLE_TIMEOUT,
			     timeout);

	if (timeout >= SR_DISABLE_TIMEOUT)
		dev_warn(&sr->pdev->dev, "%s: Smartreflex disable timedout\n",
			 __func__);

	/* Disable MCUDisableAcknowledge interrupt & clear pending interrupt */
	sr_modify_reg(sr, ERRCONFIG_V1, ERRCONFIG_MCUDISACKINTEN,
		      ERRCONFIG_MCUDISACKINTST);
}

static void sr_v2_disable(struct omap_sr *sr)
{
	int timeout = 0;

	/* Enable MCUDisableAcknowledge interrupt */
	sr_write_reg(sr, IRQENABLE_SET, IRQENABLE_MCUDISABLEACKINT);

	/* SRCONFIG - disable SR */
	sr_modify_reg(sr, SRCONFIG, SRCONFIG_SRENABLE, 0x0);

	/*
	 * Disable all other SR interrupts and clear the status
	 * write to status register ONLY on need basis - only if status
	 * is set.
	 */
	if (sr_read_reg(sr, ERRCONFIG_V2) & ERRCONFIG_VPBOUNDINTST_V2)
		sr_modify_reg(sr, ERRCONFIG_V2, ERRCONFIG_VPBOUNDINTEN_V2,
			      ERRCONFIG_VPBOUNDINTST_V2);
	else
		sr_modify_reg(sr, ERRCONFIG_V2, ERRCONFIG_VPBOUNDINTEN_V2,
			      0x0);
	sr_write_reg(sr, IRQENABLE_CLR, (IRQENABLE_MCUACCUMINT |
			IRQENABLE_MCUVALIDINT |
			IRQENABLE_MCUBOUNDSINT));
	sr_write_reg(sr, IRQSTATUS, (IRQSTATUS_MCUACCUMINT |
			IRQSTATUS_MCVALIDINT |
			IRQSTATUS_MCBOUNDSINT));

	/*
	 * Wait for SR to be disabled.
	 * wait until IRQSTATUS.MCUDISACKINTST = 1. Typical latency is 1us.
	 */
	sr_test_cond_timeout((sr_read_reg(sr, IRQSTATUS) &
			     IRQSTATUS_MCUDISABLEACKINT), SR_DISABLE_TIMEOUT,
			     timeout);

	if (timeout >= SR_DISABLE_TIMEOUT)
		dev_warn(&sr->pdev->dev, "%s: Smartreflex disable timedout\n",
			 __func__);

	/* Disable MCUDisableAcknowledge interrupt & clear pending interrupt */
	sr_write_reg(sr, IRQENABLE_CLR, IRQENABLE_MCUDISABLEACKINT);
	sr_write_reg(sr, IRQSTATUS, IRQSTATUS_MCUDISABLEACKINT);
}

static u32 sr_retrieve_nvalue(struct omap_sr *sr, u32 efuse_offs)
{
	int i;

	if (!sr->nvalue_table) {
		dev_warn(&sr->pdev->dev, "%s: Missing ntarget value table\n",
			 __func__);
		return 0;
	}

	for (i = 0; i < sr->nvalue_count; i++) {
		if (sr->nvalue_table[i].efuse_offs == efuse_offs)
			return sr->nvalue_table[i].nvalue;
	}

	return 0;
}

/**
 * sr_retrieve_lvt_nvalue() - Retrieves Nvalues from Efuse offsets.
 * @sr:		SR Instance Pointer
 * @efuse_offs:	Efuses Address Offsets
 *
 * This API is called from sr_enable. It retrieves
 * Nvalues corrsponding to efuse offset
 * for a SR instance. It looks
 * up in Nvalue Table with efuse
 * offset and returns corresponding Ntarget.
 */
static u32 sr_retrieve_lvt_nvalue(struct omap_sr *sr, u32 efuse_offs)
{
	int i = 0;

	if (!sr->lvt_nvalue_table) {
		dev_warn(&sr->pdev->dev, "%s: Missing LVT Sensor ntarget value table\n",
			 __func__);
		return 0;
	}

	while (i < sr->lvt_nvalue_count) {
		if (sr->lvt_nvalue_table[i].efuse_offs == efuse_offs)
			return sr->lvt_nvalue_table[i].nvalue;
		i++;
	}

	return 0;
}

/* Public Functions */

/**
 * sr_configure_errgen() - Configures the smrtreflex to perform AVS using the
 *			 error generator module.
 * @voltdm:	VDD pointer to which the SR module to be configured belongs to.
 *
 * This API is to be called from the smartreflex class driver to
 * configure the error generator module inside the smartreflex module.
 * SR settings if using the ERROR module inside Smartreflex.
 * SR CLASS 3 by default uses only the ERROR module where as
 * SR CLASS 2 can choose between ERROR module and MINMAXAVG
 * module. Returns 0 on success and error value in case of failure.
 */
int sr_configure_errgen(struct voltagedomain *voltdm)
{
	u32 sr_config, sr_errconfig, errconfig_offs;
	u32 vpboundint_en, vpboundint_st;
	u32 senp_en = 0, senn_en = 0;
	u8 senp_shift, senn_shift;
	struct omap_sr *sr = _sr_lookup(voltdm);

	if (IS_ERR(sr)) {
		pr_warning("%s: omap_sr struct for voltdm not found\n",
			   __func__);
		return PTR_ERR(sr);
	}

	if (!sr->clk_length)
		sr_set_clk_length(sr);

	senp_en = sr->senp_mod;
	senn_en = sr->senn_mod;

	sr_config = (sr->clk_length << SRCONFIG_SRCLKLENGTH_SHIFT) |
		SRCONFIG_SENENABLE | SRCONFIG_ERRGEN_EN;

	switch (sr->ip_type) {
	case SR_TYPE_V1:
		sr_config |= SRCONFIG_DELAYCTRL;
		senn_shift = SRCONFIG_SENNENABLE_V1_SHIFT;
		senp_shift = SRCONFIG_SENPENABLE_V1_SHIFT;
		errconfig_offs = ERRCONFIG_V1;
		vpboundint_en = ERRCONFIG_VPBOUNDINTEN_V1;
		vpboundint_st = ERRCONFIG_VPBOUNDINTST_V1;
		break;
	case SR_TYPE_V2:
		senn_shift = SRCONFIG_SENNENABLE_V2_SHIFT;
		senp_shift = SRCONFIG_SENPENABLE_V2_SHIFT;
		errconfig_offs = ERRCONFIG_V2;
		vpboundint_en = ERRCONFIG_VPBOUNDINTEN_V2;
		vpboundint_st = ERRCONFIG_VPBOUNDINTST_V2;
		break;
	default:
		dev_err(&sr->pdev->dev, "%s: Trying to Configure smartreflex"\
			"module without specifying the ip\n", __func__);
		return -EINVAL;
	}

	if (sr->lvt_sensor) {
		sr_config |= SRCONFIG_LVTSENNENABLE | SRCONFIG_LVTSENPENABLE;
		sr_config |= SRCONFIG_LVTSENENABLE;
	}
	sr_config |= ((senn_en << senn_shift) | (senp_en << senp_shift));
	sr_write_reg(sr, SRCONFIG, sr_config);
	sr_errconfig = (sr->err_weight << ERRCONFIG_ERRWEIGHT_SHIFT) |
		(sr->err_maxlimit << ERRCONFIG_ERRMAXLIMIT_SHIFT) |
		(sr->err_minlimit <<  ERRCONFIG_ERRMINLIMIT_SHIFT);
	sr_modify_reg(sr, errconfig_offs, (SR_ERRWEIGHT_MASK |
		SR_ERRMAXLIMIT_MASK | SR_ERRMINLIMIT_MASK),
		sr_errconfig);

	/* Enabling the interrupts if the ERROR module is used */
	sr_modify_reg(sr, errconfig_offs, (vpboundint_en | vpboundint_st),
		      vpboundint_en);

	return 0;
}

/**
 * sr_disable_errgen() - Disables SmartReflex AVS module's errgen component
 * @voltdm:	VDD pointer to which the SR module to be configured belongs to.
 *
 * This API is to be called from the smartreflex class driver to
 * disable the error generator module inside the smartreflex module.
 *
 * Returns 0 on success and error value in case of failure.
 */
int sr_disable_errgen(struct voltagedomain *voltdm)
{
	u32 errconfig_offs;
	u32 vpboundint_en, vpboundint_st;
	struct omap_sr *sr = _sr_lookup(voltdm);

	if (IS_ERR(sr)) {
		pr_warning("%s: omap_sr struct for voltdm not found\n",
			   __func__);
		return PTR_ERR(sr);
	}

	switch (sr->ip_type) {
	case SR_TYPE_V1:
		errconfig_offs = ERRCONFIG_V1;
		vpboundint_en = ERRCONFIG_VPBOUNDINTEN_V1;
		vpboundint_st = ERRCONFIG_VPBOUNDINTST_V1;
		break;
	case SR_TYPE_V2:
		errconfig_offs = ERRCONFIG_V2;
		vpboundint_en = ERRCONFIG_VPBOUNDINTEN_V2;
		vpboundint_st = ERRCONFIG_VPBOUNDINTST_V2;
		break;
	default:
		dev_err(&sr->pdev->dev, "%s: Trying to Configure smartreflex"\
			"module without specifying the ip\n", __func__);
		return -EINVAL;
	}

	/* Disable the interrupts of ERROR module */
	sr_modify_reg(sr, errconfig_offs, vpboundint_en | vpboundint_st, 0);

	/* Disable the Sensor and errorgen */
	sr_modify_reg(sr, SRCONFIG, SRCONFIG_SENENABLE | SRCONFIG_ERRGEN_EN, 0);

	return 0;
}

/**
 * sr_configure_minmax() - Configures the smrtreflex to perform AVS using the
 *			 minmaxavg module.
 * @voltdm:	VDD pointer to which the SR module to be configured belongs to.
 *
 * This API is to be called from the smartreflex class driver to
 * configure the minmaxavg module inside the smartreflex module.
 * SR settings if using the ERROR module inside Smartreflex.
 * SR CLASS 3 by default uses only the ERROR module where as
 * SR CLASS 2 can choose between ERROR module and MINMAXAVG
 * module. Returns 0 on success and error value in case of failure.
 */
int sr_configure_minmax(struct voltagedomain *voltdm)
{
	u32 sr_config, sr_avgwt;
	u32 senp_en = 0, senn_en = 0;
	u8 senp_shift, senn_shift;
	struct omap_sr *sr = _sr_lookup(voltdm);

	if (IS_ERR(sr)) {
		pr_warning("%s: omap_sr struct for voltdm not found\n",
			   __func__);
		return PTR_ERR(sr);
	}

	if (!sr->clk_length)
		sr_set_clk_length(sr);

	senp_en = sr->senp_mod;
	senn_en = sr->senn_mod;

	sr_config = (sr->clk_length << SRCONFIG_SRCLKLENGTH_SHIFT) |
		SRCONFIG_SENENABLE |
		(sr->accum_data << SRCONFIG_ACCUMDATA_SHIFT);

	switch (sr->ip_type) {
	case SR_TYPE_V1:
		sr_config |= SRCONFIG_DELAYCTRL;
		senn_shift = SRCONFIG_SENNENABLE_V1_SHIFT;
		senp_shift = SRCONFIG_SENPENABLE_V1_SHIFT;
		break;
	case SR_TYPE_V2:
		senn_shift = SRCONFIG_SENNENABLE_V2_SHIFT;
		senp_shift = SRCONFIG_SENPENABLE_V2_SHIFT;
		break;
	default:
		dev_err(&sr->pdev->dev, "%s: Trying to Configure smartreflex"\
			"module without specifying the ip\n", __func__);
		return -EINVAL;
	}

	if (sr->lvt_sensor) {
		sr_config |= SRCONFIG_LVTSENNENABLE | SRCONFIG_LVTSENPENABLE;
		sr_config |= SRCONFIG_LVTSENENABLE;
	}
	sr_config |= ((senn_en << senn_shift) | (senp_en << senp_shift));
	sr_write_reg(sr, SRCONFIG, sr_config);
	sr_avgwt = (sr->senp_avgweight << AVGWEIGHT_SENPAVGWEIGHT_SHIFT) |
		(sr->senn_avgweight << AVGWEIGHT_SENNAVGWEIGHT_SHIFT);
	sr_write_reg(sr, AVGWEIGHT, sr_avgwt);

	/*
	 * Enabling the interrupts if MINMAXAVG module is used.
	 * TODO: check if all the interrupts are mandatory
	 */
	switch (sr->ip_type) {
	case SR_TYPE_V1:
		sr_modify_reg(sr, ERRCONFIG_V1,
			      (ERRCONFIG_MCUACCUMINTEN |
			      ERRCONFIG_MCUVALIDINTEN |
			      ERRCONFIG_MCUBOUNDINTEN),
			      (ERRCONFIG_MCUACCUMINTEN |
			      ERRCONFIG_MCUACCUMINTST |
			      ERRCONFIG_MCUVALIDINTEN |
			      ERRCONFIG_MCUVALIDINTST |
			      ERRCONFIG_MCUBOUNDINTEN |
			      ERRCONFIG_MCUBOUNDINTST));
		break;
	case SR_TYPE_V2:
		sr_write_reg(sr, IRQSTATUS,
			     IRQSTATUS_MCUACCUMINT |
			     IRQSTATUS_MCVALIDINT |
			     IRQSTATUS_MCBOUNDSINT |
			     IRQSTATUS_MCUDISABLEACKINT);
		sr_write_reg(sr, IRQENABLE_SET,
			     IRQENABLE_MCUACCUMINT |
			     IRQENABLE_MCUVALIDINT |
			     IRQENABLE_MCUBOUNDSINT |
			     IRQENABLE_MCUDISABLEACKINT);
		break;
	default:
		dev_err(&sr->pdev->dev, "%s: Trying to Configure smartreflex"\
			"module without specifying the ip\n", __func__);
		return -EINVAL;
	}

	return 0;
}

/**
 * sr_enable() - Enables the smartreflex module.
 * @voltdm:	VDD pointer to which the SR module to be configured belongs to.
 * @volt:	The voltage at which the Voltage domain associated with
 *		the smartreflex module is operating at.
 *		This is required only to program the correct Ntarget value.
 *
 * This API is to be called from the smartreflex class driver to
 * enable a smartreflex module. Returns 0 on success. Returns error
 * value if the voltage passed is wrong or if ntarget value is wrong.
 */
int sr_enable(struct voltagedomain *voltdm, unsigned long volt)
{
	struct omap_volt_data *volt_data = NULL;
	struct omap_sr *sr = _sr_lookup(voltdm);
	u32 nvalue_reciprocal;
	int ret;
	u32 lvt_nvalue_reciprocal = 0;

	if (IS_ERR(sr)) {
		pr_warning("%s: omap_sr struct for voltdm not found\n",
			   __func__);
		return PTR_ERR(sr);
	}

	volt_data = omap_voltage_get_curr_vdata(voltdm);

	if (IS_ERR_OR_NULL(volt_data)) {
		dev_warn(&sr->pdev->dev, "%s: bad voltage data\n", __func__);
		return -EINVAL;
	}

	nvalue_reciprocal = sr_retrieve_nvalue(sr, volt_data->sr_efuse_offs);
	if (!nvalue_reciprocal) {
		dev_warn(&sr->pdev->dev, "%s: NVALUE = 0 at voltage %ld\n",
			 __func__, volt);
		return -ENODATA;
	}

	if (sr->lvt_sensor) {
		lvt_nvalue_reciprocal = sr_retrieve_lvt_nvalue(sr,
						volt_data->lvt_sr_efuse_offs);
		if (!lvt_nvalue_reciprocal) {
			dev_err(&sr->pdev->dev,
				"%s: LVT SENSOR NVALUE = 0 at voltage %ld\n",
				__func__, volt);
			return -ENODATA;
		}
	}

	/* errminlimit is opp dependent and hence linked to voltage */
	sr->err_minlimit = volt_data->sr_errminlimit;

	sr->ops->get(sr);

	/* Check if SR is already enabled. If yes do nothing */
	if (sr_read_reg(sr, SRCONFIG) & SRCONFIG_SRENABLE)
		return 0;

	/* Configure SR */
	ret = sr_class->configure(sr);
	if (ret)
		return ret;

	sr_write_reg(sr, NVALUERECIPROCAL, nvalue_reciprocal);

	if (sr->lvt_sensor)
		sr_write_reg(sr, LVTNVALUERECIPROCAL, lvt_nvalue_reciprocal);

	/* SRCONFIG - enable SR */
	sr_modify_reg(sr, SRCONFIG, SRCONFIG_SRENABLE, SRCONFIG_SRENABLE);
	return 0;
}

/**
 * sr_disable() - Disables the smartreflex module.
 * @voltdm:	VDD pointer to which the SR module to be configured belongs to.
 *
 * This API is to be called from the smartreflex class driver to
 * disable a smartreflex module.
 */
void sr_disable(struct voltagedomain *voltdm)
{
	struct omap_sr *sr = _sr_lookup(voltdm);

	if (IS_ERR(sr)) {
		pr_warning("%s: omap_sr struct for voltdm not found\n",
			   __func__);
		return;
	}

	/* Check if SR clocks are already disabled. If yes do nothing */
	if (pm_runtime_suspended(&sr->pdev->dev))
		return;

	/*
	 * Disable SR if only it is indeed enabled. Else just
	 * disable the clocks.
	 */
	if (sr_read_reg(sr, SRCONFIG) & SRCONFIG_SRENABLE) {
		switch (sr->ip_type) {
		case SR_TYPE_V1:
			sr_v1_disable(sr);
			break;
		case SR_TYPE_V2:
			sr_v2_disable(sr);
			break;
		default:
			dev_err(&sr->pdev->dev, "UNKNOWN IP type %d\n",
				sr->ip_type);
		}
	}

	sr->ops->put(sr);
}

/**
 * sr_register_class() - API to register a smartreflex class parameters.
 * @class_data:	The structure containing various sr class specific data.
 *
 * This API is to be called by the smartreflex class driver to register itself
 * with the smartreflex driver during init. Returns 0 on success else the
 * error value.
 */
int sr_register_class(struct omap_sr_class_data *class_data)
{
	if (!class_data) {
		pr_warning("%s:, Smartreflex class data passed is NULL\n",
			   __func__);
		return -EINVAL;
	}

	if (sr_class) {
		pr_warning("%s: Smartreflex class driver already registered\n",
			   __func__);
		return -EBUSY;
	}

	sr_class = class_data;

	return 0;
}

/**
 * omap_sr_enable() -  API to enable SR clocks and to call into the
 *			registered smartreflex class enable API.
 * @voltdm:	VDD pointer to which the SR module to be configured belongs to.
 *
 * This API is to be called from the kernel in order to enable
 * a particular smartreflex module. This API will do the initial
 * configurations to turn on the smartreflex module and in turn call
 * into the registered smartreflex class enable API.
 */
void omap_sr_enable(struct voltagedomain *voltdm)
{
	struct omap_sr *sr;

	if (!atomic_read(&sr_driver_ready))
		return;

	sr = _sr_lookup(voltdm);
	if (IS_ERR(sr)) {
		pr_warning("%s: omap_sr struct for voltdm not found\n",
			   __func__);
		return;
	}

	if (!sr->autocomp_active)
		return;

	if (!sr_class || !(sr_class->enable) || !(sr_class->configure)) {
		dev_warn(&sr->pdev->dev, "%s: smartreflex class driver not"\
			"registered\n", __func__);
		return;
	}

	sr_class->enable(sr);
}

/**
 * omap_sr_disable() - API to disable SR without resetting the voltage
 *			processor voltage
 * @voltdm:	VDD pointer to which the SR module to be configured belongs to.
 *
 * This API is to be called from the kernel in order to disable
 * a particular smartreflex module. This API will in turn call
 * into the registered smartreflex class disable API. This API will tell
 * the smartreflex class disable not to reset the VP voltage after
 * disabling smartreflex.
 */
void omap_sr_disable(struct voltagedomain *voltdm)
{
	struct omap_sr *sr;

	if (!atomic_read(&sr_driver_ready))
		return;

	sr = _sr_lookup(voltdm);
	if (IS_ERR(sr)) {
		pr_warning("%s: omap_sr struct for voltdm not found\n",
			   __func__);
		return;
	}

	if (!sr->autocomp_active)
		return;

	if (!sr_class || !(sr_class->disable)) {
		dev_warn(&sr->pdev->dev, "%s: smartreflex class driver not"\
			"registered\n", __func__);
		return;
	}

	sr_class->disable(sr, 0);
}

/**
 * omap_sr_disable_reset_volt() - API to disable SR and reset the
 *				voltage processor voltage
 * @voltdm:	VDD pointer to which the SR module to be configured belongs to.
 *
 * This API is to be called from the kernel in order to disable
 * a particular smartreflex module. This API will in turn call
 * into the registered smartreflex class disable API. This API will tell
 * the smartreflex class disable to reset the VP voltage after
 * disabling smartreflex.
 */
void omap_sr_disable_reset_volt(struct voltagedomain *voltdm)
{
	struct omap_sr *sr;

	if (!atomic_read(&sr_driver_ready))
		return;

	sr = _sr_lookup(voltdm);
	if (IS_ERR(sr)) {
		pr_warning("%s: omap_sr struct for voltdm not found\n",
			   __func__);
		return;
	}

	if (!sr->autocomp_active)
		return;

	if (!sr_class || !(sr_class->disable)) {
		dev_warn(&sr->pdev->dev, "%s: smartreflex class driver not"\
			"registered\n", __func__);
		return;
	}

	sr_class->disable(sr, 1);
}

/**
 * omap_sr_register_pmic() - API to register pmic specific info.
 * @pmic_data:	The structure containing pmic specific data.
 *
 * This API is to be called from the PMIC specific code to register with
 * smartreflex driver pmic specific info. Currently the only info required
 * is the smartreflex init on the PMIC side.
 */
void omap_sr_register_pmic(struct omap_sr_pmic_data *pmic_data)
{
	if (!pmic_data) {
		pr_warning("%s: Trying to register NULL PMIC data structure"\
			"with smartreflex\n", __func__);
		return;
	}

	sr_pmic_data = pmic_data;
}

/* PM Debug FS entries to enable and disable smartreflex. */
static int omap_sr_autocomp_show(void *data, u64 *val)
{
	struct omap_sr *sr_info = data;

	if (!sr_info) {
		pr_warning("%s: omap_sr struct not found\n", __func__);
		return -EINVAL;
	}

	*val = sr_info->autocomp_active;

	return 0;
}

static int omap_sr_autocomp_store(void *data, u64 val)
{
	struct omap_sr *sr_info = data;

	if (!sr_info) {
		pr_warning("%s: omap_sr struct not found\n", __func__);
		return -EINVAL;
	}

	/* Sanity check */
	if (val > 1) {
		pr_warning("%s: Invalid argument %lld\n", __func__, val);
		return -EINVAL;
	}

	/* control enable/disable only if there is a delta in value */
	if (sr_info->autocomp_active != val) {
		if (!val)
			sr_stop_vddautocomp(sr_info);
		else
			sr_start_vddautocomp(sr_info);
	}

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(pm_sr_fops, omap_sr_autocomp_show,
			omap_sr_autocomp_store, "%llu\n");

static int __init omap_sr_probe(struct platform_device *pdev)
{
	struct omap_sr *sr_info;
	struct omap_sr_data *pdata = pdev->dev.platform_data;
	struct resource *mem, *irq;
	struct dentry *nvalue_dir;
	struct dentry *lvt_nvalue_dir;
	int i, ret = 0;

	sr_info = kzalloc(sizeof(struct omap_sr), GFP_KERNEL);
	if (!sr_info) {
		dev_err(&pdev->dev, "%s: unable to allocate sr_info\n",
			__func__);
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, sr_info);

	if (!pdata) {
		dev_err(&pdev->dev, "%s: platform data missing\n", __func__);
		ret = -EINVAL;
		goto err_free_devinfo;
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "%s: no mem resource\n", __func__);
		ret = -ENODEV;
		goto err_free_devinfo;
	}

	mem = request_mem_region(mem->start, resource_size(mem),
					dev_name(&pdev->dev));
	if (!mem) {
		dev_err(&pdev->dev, "%s: no mem region\n", __func__);
		ret = -EBUSY;
		goto err_free_devinfo;
	}

	irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);

	pm_runtime_enable(&pdev->dev);
	pm_runtime_irq_safe(&pdev->dev);

	sr_info->name = kasprintf(GFP_KERNEL, "%s", pdata->name);
	if (!sr_info->name) {
		dev_err(&pdev->dev, "%s: Unable to alloc SR instance name\n",
			__func__);
		ret = -ENOMEM;
		goto err_release_region;
	}

	if (!pdata->ops || !pdata->ops->get || !pdata->ops->put) {
		dev_err(&pdev->dev, "%s: Missing fops!!\n", __func__);
		ret = -EINVAL;
		goto err_release_region;
	}

	sr_info->ops = kmalloc(sizeof(struct omap_sr_ops), GFP_KERNEL);
	if (!sr_info->ops) {
		dev_err(&pdev->dev, "%s: Could'nt alloc ops mem!!\n",
			__func__);
		ret = -ENOMEM;
		goto err_release_region;
	}
	memcpy(sr_info->ops, pdata->ops, sizeof(struct omap_sr_ops));

	sr_info->pdev = pdev;
	sr_info->srid = pdev->id;
	sr_info->voltdm = pdata->voltdm;
	sr_info->lvt_sensor = pdata->lvt_sensor;
	sr_info->nvalue_table = pdata->nvalue_table;
	sr_info->nvalue_count = pdata->nvalue_count;
	sr_info->lvt_nvalue_table = pdata->lvt_nvalue_table;
	sr_info->lvt_nvalue_count = pdata->lvt_nvalue_count;
	sr_info->senn_mod = pdata->senn_mod;
	sr_info->senp_mod = pdata->senp_mod;
	sr_info->autocomp_active = false;
	sr_info->ip_type = pdata->ip_type;
	sr_info->base = ioremap(mem->start, resource_size(mem));
	if (!sr_info->base) {
		dev_err(&pdev->dev, "%s: ioremap fail\n", __func__);
		ret = -ENOMEM;
		goto err_release_region2;
	}

	if (irq)
		sr_info->irq = irq->start;

	sr_set_clk_length(sr_info);
	sr_set_regfields(sr_info);

	list_add(&sr_info->node, &sr_list);

	/*
	 * Call into late init to do intializations that require
	 * both sr driver and sr class driver to be initiallized.
	 */
	if (sr_class) {
		ret = sr_late_init(sr_info);
		if (ret) {
			pr_warning("%s: Error in SR late init\n", __func__);
			goto err_iounmap;
		}
	}

	dev_info(&pdev->dev, "%s: SmartReflex driver initialized\n", __func__);
	if (!sr_dbg_dir) {
		sr_dbg_dir = debugfs_create_dir("smartreflex", NULL);
		if (IS_ERR_OR_NULL(sr_dbg_dir)) {
			ret = PTR_ERR(sr_dbg_dir);
			pr_err("%s:sr debugfs dir creation failed(%d)\n",
			       __func__, ret);
			goto err_iounmap;
		}
	}

	sr_info->dbg_dir = debugfs_create_dir(sr_info->name, sr_dbg_dir);
	if (IS_ERR_OR_NULL(sr_info->dbg_dir)) {
		dev_err(&pdev->dev, "%s: Unable to create debugfs directory\n",
			__func__);
		ret = PTR_ERR(sr_info->dbg_dir);
		goto err_free_name;
	}

	(void) debugfs_create_file("autocomp", S_IRUGO | S_IWUSR,
			sr_info->dbg_dir, (void *)sr_info, &pm_sr_fops);
	(void) debugfs_create_x32("errweight", S_IRUGO, sr_info->dbg_dir,
			&sr_info->err_weight);
	(void) debugfs_create_x32("errmaxlimit", S_IRUGO, sr_info->dbg_dir,
			&sr_info->err_maxlimit);

	nvalue_dir = debugfs_create_dir("nvalue", sr_info->dbg_dir);
	if (IS_ERR_OR_NULL(nvalue_dir)) {
		dev_err(&pdev->dev, "%s: Unable to create debugfs directory"\
			"for n-values\n", __func__);
		ret = PTR_ERR(nvalue_dir);
		goto err_debugfs;
	}

	if (sr_info->nvalue_count == 0 || !sr_info->nvalue_table) {
		dev_warn(&pdev->dev, "%s: %s: No NVALUES available."\
					"Cannot create debugfs entries\n",
			 __func__, sr_info->name);
		ret = -ENODATA;
		goto err_debugfs;
	}

	for (i = 0; i < sr_info->nvalue_count; i++) {
		char name[NVALUE_NAME_LEN + 1];

		snprintf(name, sizeof(name), "volt_%lu",
			 sr_info->nvalue_table[i].volt_nominal);
		(void) debugfs_create_x32(name, S_IRUGO | S_IWUSR, nvalue_dir,
				&(sr_info->nvalue_table[i].nvalue));
		snprintf(name, sizeof(name), "errminlimit_%lu",
			 sr_info->nvalue_table[i].volt_nominal);
		(void) debugfs_create_x32(name, S_IRUGO | S_IWUSR, nvalue_dir,
				&(sr_info->nvalue_table[i].errminlimit));

	}

	if (!sr_info->lvt_sensor)
		goto skip_lvt;

	lvt_nvalue_dir = debugfs_create_dir("lvt_nvalue", sr_info->dbg_dir);
	if (IS_ERR_OR_NULL(lvt_nvalue_dir)) {
		dev_err(&pdev->dev, "%s: Unable to create debugfs directory"
			"for lvt sensor's n-values\n", __func__);
		ret = PTR_ERR(lvt_nvalue_dir);
		goto err_debugfs;
	}

	for (i = 0; i < sr_info->lvt_nvalue_count; i++) {
		char name[NVALUE_NAME_LEN + 1];

		snprintf(name, sizeof(name), "volt_%lu",
			 sr_info->lvt_nvalue_table[i].volt_nominal);
		(void) debugfs_create_x32(name, S_IRUGO | S_IWUSR,
				lvt_nvalue_dir,
				&(sr_info->lvt_nvalue_table[i].nvalue));
	}

skip_lvt:

	return ret;

err_debugfs:
	debugfs_remove_recursive(sr_info->dbg_dir);
err_free_name:
	kfree(sr_info->name);
err_iounmap:
	list_del(&sr_info->node);
	iounmap(sr_info->base);
err_release_region2:
	kfree(sr_info->ops);
err_release_region:
	release_mem_region(mem->start, resource_size(mem));
err_free_devinfo:
	kfree(sr_info);

	return ret;
}

static int __devexit omap_sr_remove(struct platform_device *pdev)
{
	struct omap_sr_data *pdata = pdev->dev.platform_data;
	struct omap_sr *sr_info;
	struct resource *mem;

	if (!pdata) {
		dev_err(&pdev->dev, "%s: platform data missing\n", __func__);
		return -EINVAL;
	}

	sr_info = _sr_lookup(pdata->voltdm);
	if (IS_ERR(sr_info)) {
		dev_warn(&pdev->dev, "%s: omap_sr struct not found\n",
			 __func__);
		return PTR_ERR(sr_info);
	}

	if (sr_info->autocomp_active)
		sr_stop_vddautocomp(sr_info);
	if (sr_info->dbg_dir)
		debugfs_remove_recursive(sr_info->dbg_dir);

	list_del(&sr_info->node);
	iounmap(sr_info->base);
	kfree(sr_info->name);
	kfree(sr_info->ops);
	kfree(sr_info);
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(mem->start, resource_size(mem));

	return 0;
}

static void __devexit omap_sr_shutdown(struct platform_device *pdev)
{
	struct omap_sr_data *pdata = pdev->dev.platform_data;
	struct omap_sr *sr_info;

	if (!pdata) {
		dev_err(&pdev->dev, "%s: platform data missing\n", __func__);
		return;
	}

	sr_info = _sr_lookup(pdata->voltdm);
	if (IS_ERR(sr_info)) {
		dev_warn(&pdev->dev, "%s: omap_sr struct not found\n",
			 __func__);
		return;
	}

	if (sr_info->autocomp_active)
		sr_stop_vddautocomp(sr_info);

	return;
}

static int sr_suspend(struct device *dev)
{
	struct omap_sr_data *pdata;
	struct omap_sr *sr_info;

	pdata = dev_get_platdata(dev);
	if (!pdata) {
		dev_err(dev, "%s: platform data missing\n", __func__);
		return -EINVAL;
	}

	sr_info = _sr_lookup(pdata->voltdm);
	if (IS_ERR_OR_NULL(sr_info)) {
		dev_warn(dev, "%s: omap_sr struct not found\n", __func__);
		return -EINVAL;
	}

	if (!sr_info->autocomp_active)
		return 0;

	if (sr_info->suspended)
		return 0;

	sr_info->suspended =  true;
	/* Flag the same info to the other CPUs */
	smp_wmb();

	return 0;
}

static int sr_resume(struct device *dev)
{

	struct omap_sr_data *pdata;
	struct omap_sr *sr_info;

	pdata = dev_get_platdata(dev);
	if (!pdata) {
		dev_err(dev, "%s: platform data missing\n", __func__);
		return -EINVAL;
	}

	sr_info = _sr_lookup(pdata->voltdm);
	if (IS_ERR_OR_NULL(sr_info)) {
		dev_warn(dev, "%s: omap_sr struct not found\n", __func__);
		return -EINVAL;
	}

	if (!sr_info->autocomp_active)
		return 0;

	if (!sr_info->suspended)
		return 0;

	sr_info->suspended =  false;
	/* Flag the same info to the other CPUs */
	smp_wmb();

	return 0;
}

static SIMPLE_DEV_PM_OPS(serial_omap_dev_pm_ops, sr_suspend, sr_resume);

static struct platform_driver smartreflex_driver = {
	.remove         = __devexit_p(omap_sr_remove),
	.shutdown	= __devexit_p(omap_sr_shutdown),
	.driver		= {
		.name	= "smartreflex",
		.pm	= &serial_omap_dev_pm_ops,
	},
};

static int __init sr_init(void)
{
	int ret = 0;

	/*
	 * sr_init is a late init. If by then a pmic specific API is not
	 * registered either there is no need for anything to be done on
	 * the PMIC side or somebody has forgotten to register a PMIC
	 * handler. Warn for the second condition.
	 */
	if (sr_pmic_data && sr_pmic_data->sr_pmic_init)
		sr_pmic_data->sr_pmic_init();
	else
		pr_warning("%s: No PMIC hook to init smartreflex\n", __func__);

	ret = platform_driver_probe(&smartreflex_driver, omap_sr_probe);
	if (ret) {
		pr_err("%s: platform driver register failed for SR\n",
		       __func__);
		return ret;
	}

	atomic_set(&sr_driver_ready, 1);

	return 0;
}
late_initcall(sr_init);

static void __exit sr_exit(void)
{
	atomic_set(&sr_driver_ready, 0);
	platform_driver_unregister(&smartreflex_driver);
}
module_exit(sr_exit);

MODULE_DESCRIPTION("OMAP Smartreflex Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_AUTHOR("Texas Instruments Inc");
