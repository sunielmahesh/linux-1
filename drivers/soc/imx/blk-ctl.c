// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2021 NXP.
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/pm_domain.h>
#include <linux/reset-controller.h>

#include "blk-ctl.h"

static inline struct imx_blk_ctl_domain *to_imx_blk_ctl_pd(struct generic_pm_domain *genpd)
{
	return container_of(genpd, struct imx_blk_ctl_domain, pd);
}

static int imx_blk_ctl_enable_hsk(struct device *dev)
{
	struct imx_blk_ctl *blk_ctl = dev_get_drvdata(dev);
	const struct imx_blk_ctl_hw *hw = &blk_ctl->dev_data->hw_hsk;
	struct regmap *regmap = blk_ctl->regmap;
	int ret;

	if (hw->flags & IMX_BLK_CTL_PD_RESET) {
		ret = regmap_update_bits(regmap, hw->rst_offset, hw->rst_mask, hw->rst_mask);
		if (ret)
			return ret;
	}

	ret = regmap_update_bits(regmap, hw->offset, hw->mask, hw->mask);

	/* Wait for handshake */
	udelay(5);

	return ret;
}

int imx_blk_ctl_power_off(struct generic_pm_domain *domain)
{
	struct imx_blk_ctl_domain *pd = to_imx_blk_ctl_pd(domain);
	struct imx_blk_ctl *blk_ctl = pd->blk_ctl;
	struct regmap *regmap = blk_ctl->regmap;
	const struct imx_blk_ctl_hw *hw = &blk_ctl->dev_data->pds[pd->id];
	int ret;

	mutex_lock(&blk_ctl->lock);

	ret = clk_bulk_prepare_enable(blk_ctl->num_clks, blk_ctl->clks);
	if (ret) {
		mutex_unlock(&blk_ctl->lock);
		return ret;
	}

	ret = regmap_clear_bits(regmap, hw->offset, hw->mask);
	if (ret)
		goto hsk_fail;

	if (hw->flags & IMX_BLK_CTL_PD_RESET) {
		ret = regmap_clear_bits(regmap, hw->rst_offset, hw->rst_mask);
		if (ret)
			goto hsk_fail;
	}

	blk_ctl->power_count--;

	if (!blk_ctl->power_count) {
		ret = imx_blk_ctl_enable_hsk(blk_ctl->dev);
		if (ret)
			dev_err(blk_ctl->dev, "Handshake failed when power off\n");
	}

hsk_fail:
	clk_bulk_disable_unprepare(blk_ctl->num_clks, blk_ctl->clks);

	mutex_unlock(&blk_ctl->lock);

	return ret;
}

int imx_blk_ctl_power_on(struct generic_pm_domain *domain)
{
	struct imx_blk_ctl_domain *pd = to_imx_blk_ctl_pd(domain);
	struct imx_blk_ctl *blk_ctl = pd->blk_ctl;
	struct regmap *regmap = blk_ctl->regmap;
	const struct imx_blk_ctl_hw *hw = &blk_ctl->dev_data->pds[pd->id];
	int ret;

	mutex_lock(&blk_ctl->lock);

	ret = clk_bulk_prepare_enable(blk_ctl->num_clks, blk_ctl->clks);
	if (ret) {
		mutex_unlock(&blk_ctl->lock);
		return ret;
	}

	if (!blk_ctl->power_count) {
		ret = imx_blk_ctl_enable_hsk(blk_ctl->dev);
		if (ret) {
			dev_err(blk_ctl->dev, "Handshake failed when power on\n");
			goto disable_clk;
		}
	}

	if (hw->flags & IMX_BLK_CTL_PD_RESET) {
		ret = regmap_clear_bits(regmap, hw->rst_offset, hw->rst_mask);
		if (ret)
			goto disable_clk;
	}

	/* Wait for reset propagate */
	udelay(5);

	if (hw->flags & IMX_BLK_CTL_PD_RESET) {
		ret = regmap_update_bits(regmap, hw->rst_offset, hw->rst_mask, hw->rst_mask);
		if (ret)
			goto disable_clk;
	}

	ret = regmap_update_bits(regmap, hw->offset, hw->mask, hw->mask);
	if (ret)
		goto disable_clk;

	blk_ctl->power_count++;

disable_clk:
	clk_bulk_disable_unprepare(blk_ctl->num_clks, blk_ctl->clks);

	mutex_unlock(&blk_ctl->lock);

	return ret;
}

static int imx_blk_ctl_attach_pd(struct device *dev, struct device **devs, char **pd_names,
				 u32 num_pds)
{
	int i, ret;

	if (!pd_names)
		return -EINVAL;

	if (dev->pm_domain) {
		devs[0] = dev;
		pm_runtime_enable(dev);
		return 0;
	}

	for (i = 0; i < num_pds; i++) {
		devs[i] = dev_pm_domain_attach_by_name(dev, pd_names[i]);
		if (IS_ERR_OR_NULL(devs[i])) {
			ret = PTR_ERR(devs[i]) ? : -ENODATA;
			goto detach_pm;
		}
	}

	return 0;

detach_pm:
	for (i--; i >= 0; i--)
		dev_pm_domain_detach(devs[i], false);

	return ret;
}

static int imx_blk_ctl_register_pd(struct device *dev)
{
	struct imx_blk_ctl *blk_ctl = dev_get_drvdata(dev);
	const struct imx_blk_ctl_dev_data *dev_data = blk_ctl->dev_data;
	int num = dev_data->pds_num;
	struct imx_blk_ctl_domain *domain;
	int i, ret;

	blk_ctl->onecell_data.num_domains = num;
	blk_ctl->onecell_data.domains = devm_kcalloc(dev, num,
						     sizeof(struct generic_pm_domain *),
						     GFP_KERNEL);

	if (!blk_ctl->onecell_data.domains)
		return -ENOMEM;

	for (i = 0; i < num; i++) {
		domain = devm_kzalloc(dev, sizeof(*domain), GFP_KERNEL);
		if (!domain) {
			ret = -ENOMEM;
			goto remove_genpd;
		}
		domain->pd.name = dev_data->pds[i].name;
		domain->pd.power_off = imx_blk_ctl_power_off;
		domain->pd.power_on = imx_blk_ctl_power_on;
		domain->blk_ctl = blk_ctl;
		domain->id = i;

		ret = pm_genpd_init(&domain->pd, NULL, true);
		if (ret)
			goto remove_genpd;

		blk_ctl->onecell_data.domains[i] = &domain->pd;
	}

	return 0;

remove_genpd:
	for (i = i - 1; i >= 0; i--)
		pm_genpd_remove(blk_ctl->onecell_data.domains[i]);

	return ret;
}

static int imx_blk_ctl_hook_pd(struct device *dev)
{
	struct imx_blk_ctl *blk_ctl = dev_get_drvdata(dev);
	const struct imx_blk_ctl_dev_data *dev_data = blk_ctl->dev_data;
	const struct imx_blk_ctl_hw *pds = dev_data->pds;
	int num_active_pd = dev_data->num_active_pd;
	int num = dev_data->pds_num;
	struct generic_pm_domain *genpd, *child_genpd;
	int ret;
	int i, j;

	blk_ctl->active_pds = devm_kcalloc(dev, num_active_pd, sizeof(struct device *), GFP_KERNEL);
	if (!blk_ctl->active_pds)
		return -ENOMEM;

	ret = imx_blk_ctl_attach_pd(dev, blk_ctl->active_pds, dev_data->active_pd_names,
				    num_active_pd);
	if (ret) {
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Failed to attach active pd: %d\n", ret);
		return ret;
	}

	for (i = 0; i < num; i++) {
		for (j = 0; j < num_active_pd; j++) {
			genpd = pd_to_genpd(blk_ctl->active_pds[j]->pm_domain);
			if (!strcmp(genpd->name, pds[i].parent_name))
				break;
		}

		child_genpd = blk_ctl->onecell_data.domains[i];
		if (pm_genpd_add_subdomain(genpd, child_genpd))
			pr_warn("failed to add subdomain: %s\n", child_genpd->name);
	}

	return 0;
}

int imx_blk_ctl_register(struct device *dev)
{
	struct imx_blk_ctl *blk_ctl = dev_get_drvdata(dev);
	const struct imx_blk_ctl_dev_data *dev_data = blk_ctl->dev_data;
	int num = dev_data->pds_num;
	int i, ret;

	if (!blk_ctl)
		return -ENODEV;

	ret = imx_blk_ctl_register_pd(dev);
	if (ret)
		return ret;

	ret = imx_blk_ctl_hook_pd(dev);
	if (ret)
		goto unregister_pd;

	ret = of_genpd_add_provider_onecell(dev->of_node, &blk_ctl->onecell_data);
	if (ret)
		goto detach_pd;

	pm_runtime_get_noresume(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	pm_runtime_put(dev);

	return 0;

detach_pd:
	for (i = blk_ctl->dev_data->num_active_pd; i >= 0; i--)
		dev_pm_domain_detach(blk_ctl->active_pds[i], false);
unregister_pd:
	for (i = num - 1; i >= 0; i--)
		pm_genpd_remove(blk_ctl->onecell_data.domains[i]);

	if (dev->pm_domain)
		pm_runtime_disable(dev);

	return ret;
}
EXPORT_SYMBOL_GPL(imx_blk_ctl_register);

const struct dev_pm_ops imx_blk_ctl_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
			   pm_runtime_force_resume)
};
EXPORT_SYMBOL_GPL(imx_blk_ctl_pm_ops);
