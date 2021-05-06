/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __SOC_IMX_BLK_CTL_H
#define __SOC_IMX_BLK_CTL_H

enum imx_blk_ctl_pd_type {
	BLK_CTL_PD,
};

struct imx_blk_ctl_hw {
	int type;
	char *name;
	char *parent_name;
	u32 offset;
	u32 mask;
	u32 flags;
	u32 id;
	u32 rst_offset;
	u32 rst_mask;
};

struct imx_blk_ctl_domain {
	struct generic_pm_domain pd;
	struct imx_blk_ctl *blk_ctl;
	u32 id;
};

struct imx_blk_ctl_dev_data {
	struct regmap_config config;
	struct imx_blk_ctl_hw *pds;
	struct imx_blk_ctl_hw hw_hsk;
	u32 pds_num;
	char **active_pd_names;
	u32 num_active_pd;
};

struct imx_blk_ctl {
	struct device *dev;
	struct regmap *regmap;
	struct device **active_pds;
	u32 pds_num;
	u32 active_pd_count;
	struct genpd_onecell_data onecell_data;
	const struct imx_blk_ctl_dev_data *dev_data;
	struct clk_bulk_data *clks;
	u32 num_clks;

	struct mutex lock;
	int power_count;
};

#define IMX_BLK_CTL(_type, _name, _parent_name, _id, _offset, _mask, _rst_offset, _rst_mask,	\
		    _flags)								\
	{										\
		.type = _type,								\
		.name = _name,								\
		.parent_name = _parent_name,						\
		.id = _id,								\
		.offset = _offset,							\
		.mask = _mask,								\
		.flags = _flags,							\
		.rst_offset = _rst_offset,						\
		.rst_mask = _rst_mask,							\
	}

#define IMX_BLK_CTL_PD(_name, _parent_name, _id, _offset, _mask, _rst_offset, _rst_mask, _flags) \
	IMX_BLK_CTL(BLK_CTL_PD, _name, _parent_name, _id, _offset, _mask, _rst_offset,		\
		    _rst_mask, _flags)

int imx_blk_ctl_register(struct device *dev);

#define IMX_BLK_CTL_PD_HANDSHAKE	BIT(0)
#define IMX_BLK_CTL_PD_RESET		BIT(1)
#define IMX_BLK_CTL_PD_BUS		BIT(2)

const extern struct dev_pm_ops imx_blk_ctl_pm_ops;

#endif
