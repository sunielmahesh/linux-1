// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2018 NXP
 * Copyright (C) 2021 Amarula Solutions(India)
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/phy/phy.h>
#include <linux/regmap.h>

#define DSI_PHYCTRL_B1			0x00
#define DSI_PHYCTRL_B2			0x04
#define DSI_PHYCTRL_M1			0x08
#define DSI_PHYCTRL_M2			0x0c
#define DSI_PHYTIMING			0x10
#define DSI_PHYTIMING1			0x14
#define DSI_PHYTIMING2			0x18

/* phytiming */
#define M_TLPXCTL_MASK			GENMASK(15, 8)
#define M_TLPXCTL(x)			FIELD_PREP(M_TLPXCTL_MASK, (x))
#define M_THSEXITCTL_MASK		GENMASK(7, 0)
#define M_THSEXITCTL(x)			FIELD_PREP(M_THSEXITCTL_MASK, (x))

/* phytiming1 */
#define M_TCLKPRPRCTL_MASK		GENMASK(31, 24)
#define M_TCLKPRPRCTL(x)		FIELD_PREP(M_TCLKPRPRCTL_MASK, (x))
#define M_TCLKZEROCTL_MASK		GENMASK(23, 16)
#define M_TCLKZEROCTL(x)		FIELD_PREP(M_TCLKZEROCTL_MASK, (x))
#define M_TCLKPOSTCTL_MASK		GENMASK(15, 8)
#define M_TCLKPOSTCTL(x)		FIELD_PREP(M_TCLKPOSTCTL_MASK, (x))
#define M_TCLKTRAILCTL_MASK		GENMASK(7, 0)
#define M_TCLKTRAILCTL(x)		FIELD_PREP(M_TCLKTRAILCTL_MASK, (x))

/* phytiming2 */
#define M_THSPRPRCTL_MASK		GENMASK(23, 16)
#define M_THSPRPRCTL(x)			FIELD_PREP(M_THSPRPRCTL_MASK, (x))
#define M_THSZEROCTL_MASK		GENMASK(15, 8)
#define M_THSZEROCTL(x)			FIELD_PREP(M_THSZEROCTL_MASK, (x))
#define M_THSTRAILCTL_MASK		GENMASK(7, 0)
#define M_THSTRAILCTL(x)		FIELD_PREP(M_THSTRAILCTL_MASK, (x))

struct dsim_dphy_plat_data {
	unsigned int m_tlpxctl;
	unsigned int m_thsexitctl;
	unsigned int m_tclkprprctl;
	unsigned int m_tclkzeroctl;
	unsigned int m_tclkpostctl;
	unsigned int m_tclktrailctl;
	unsigned int m_thsprprctl;
	unsigned int m_thszeroctl;
	unsigned int m_thstrailctl;
};

struct dsim_dphy {
	struct regmap *regmap;
	struct clk *phy_ref_clk;
	const struct dsim_dphy_plat_data *pdata;
};

static const struct regmap_config dsim_dphy_regmap_config = {
	.reg_bits = 8,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = DSI_PHYTIMING2,
	.name = "mipi-dphy",
};

static int dsim_dphy_init(struct phy *phy)
{
	struct dsim_dphy *dphy = phy_get_drvdata(phy);
	const struct dsim_dphy_plat_data *pdata = dphy->pdata;
	u32 reg;

	/* phytiming */
	regmap_read(dphy->regmap, DSI_PHYTIMING, &reg);

	reg &= ~M_TLPXCTL_MASK;
	reg |= M_TLPXCTL(pdata->m_tlpxctl);
	reg &= ~M_THSEXITCTL_MASK;
	reg |= M_THSEXITCTL(pdata->m_thsexitctl);
	regmap_write(dphy->regmap, DSI_PHYTIMING, reg);

	/* phytiming1 */
	regmap_read(dphy->regmap, DSI_PHYTIMING1, &reg);

	reg &= ~M_TCLKPRPRCTL_MASK;
	reg |= M_TCLKPRPRCTL(pdata->m_tclkprprctl);
	reg &= ~M_TCLKZEROCTL_MASK;
	reg |= M_TCLKZEROCTL(pdata->m_tclkzeroctl);
	reg &= ~M_TCLKPOSTCTL_MASK;
	reg |= M_TCLKPOSTCTL(pdata->m_tclkpostctl);
	reg &= ~M_TCLKTRAILCTL_MASK;
	reg |= M_TCLKTRAILCTL(pdata->m_tclktrailctl);
	regmap_write(dphy->regmap, DSI_PHYTIMING1, reg);

	/* phytiming2 */
	regmap_read(dphy->regmap, DSI_PHYTIMING2, &reg);

	reg &= ~M_THSPRPRCTL_MASK;
	reg |= M_THSPRPRCTL(pdata->m_thsprprctl);
	reg &= ~M_THSZEROCTL_MASK;
	reg |= M_THSZEROCTL(pdata->m_thszeroctl);
	reg &= ~M_THSTRAILCTL_MASK;
	reg |= M_THSTRAILCTL(pdata->m_thstrailctl);
	regmap_write(dphy->regmap, DSI_PHYTIMING2, reg);

	return 0;
}

static int dsim_dphy_exit(struct phy *phy)
{
	return 0;
}

static int dsim_dphy_power_on(struct phy *phy)
{
	struct dsim_dphy *dphy = phy_get_drvdata(phy);
	int ret;

	ret = clk_prepare_enable(dphy->phy_ref_clk);
	if (ret < 0)
		return ret;

	return ret;
}

static int dsim_dphy_power_off(struct phy *phy)
{
	struct dsim_dphy *dphy = phy_get_drvdata(phy);

	clk_disable_unprepare(dphy->phy_ref_clk);

	return 0;
}

static const struct phy_ops dsim_dphy_phy_ops = {
	.init = dsim_dphy_init,
	.exit = dsim_dphy_exit,
	.power_on = dsim_dphy_power_on,
	.power_off = dsim_dphy_power_off,
	.owner = THIS_MODULE,
};

static const struct dsim_dphy_plat_data imx8mm_dphy_plat_data = {
	/* phytiming */
	.m_tlpxctl	= 0x06,
	.m_thsexitctl	= 0x0b,
	/* phytiming1 */
	.m_tclkprprctl	= 0x07,
	.m_tclkzeroctl	= 0x26,
	.m_tclkpostctl	= 0x0d,
	.m_tclktrailctl	= 0x08,
	/* phytimings2 */
	.m_thsprprctl	= 0x08,
	.m_thszeroctl	= 0x0d,
	.m_thstrailctl	= 0x0b,
};

static const struct of_device_id dsim_dphy_of_match[] = {
	{
		.compatible = "fsl,imx8mm-sec-dsim-dphy",
		.data = &imx8mm_dphy_plat_data,
	},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, dsim_dphy_of_match);

static int dsim_dphy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct phy_provider *phy_provider;
	struct dsim_dphy *dphy;
	struct phy *phy;
	void __iomem *base;

	if (!np)
		return -ENODEV;

	dphy = devm_kzalloc(dev, sizeof(*dphy), GFP_KERNEL);
	if (!dphy)
		return -ENOMEM;

	dphy->pdata = of_device_get_match_data(&pdev->dev);
	if (!dphy->pdata)
		return -EINVAL;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	dphy->regmap = devm_regmap_init_mmio(&pdev->dev, base,
					     &dsim_dphy_regmap_config);
	if (IS_ERR(dphy->regmap)) {
		dev_err(dev, "failed create the DPHY regmap\n");
		return PTR_ERR(dphy->regmap);
	}

	dphy->phy_ref_clk = devm_clk_get(&pdev->dev, "phy_ref");
	if (IS_ERR(dphy->phy_ref_clk)) {
		dev_err(dev, "failed to get phy_ref clock\n");
		return PTR_ERR(dphy->phy_ref_clk);
	}

	dev_set_drvdata(dev, dphy);

	phy = devm_phy_create(dev, np, &dsim_dphy_phy_ops);
	if (IS_ERR(phy)) {
		dev_err(dev, "failed to create phy %ld\n", PTR_ERR(phy));
		return PTR_ERR(phy);
	}
	phy_set_drvdata(phy, dphy);

	phy_provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);

	return PTR_ERR_OR_ZERO(phy_provider);
}

static struct platform_driver dsim_dphy_driver = {
	.probe	= dsim_dphy_probe,
	.driver = {
		.name = "sec-dsim-dphy",
		.of_match_table	= dsim_dphy_of_match,
	}
};
module_platform_driver(dsim_dphy_driver);

MODULE_AUTHOR("Jagan Teki <jagan@amarulasolutions.com>");
MODULE_DESCRIPTION("Samsung SEC MIPI DSIM DPHY driver");
MODULE_LICENSE("GPL");
