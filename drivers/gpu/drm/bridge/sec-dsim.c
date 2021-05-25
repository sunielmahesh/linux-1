/*
 * Samsung MIPI DSIM Bridge
 *
 * Copyright 2018-2019 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <asm/unaligned.h>
#include <linux/bsearch.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/gcd.h>
#include <linux/log2.h>
#include <linux/module.h>
#include <linux/busfreq-imx.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/phy/phy.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_connector.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_encoder.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <drm/drm_of.h>
#include <drm/drm_simple_kms_helper.h>
#include <video/videomode.h>
#include <video/mipi_display.h>

/* dsim registers */
#define DSIM_VERSION			0x00
#define DSIM_STATUS			0x04
#define DSIM_RGB_STATUS			0x08
#define DSIM_SWRST			0x0c
#define DSIM_CLKCTRL			0x10
#define DSIM_TIMEOUT			0x14
#define DSIM_CONFIG			0x18
#define DSIM_ESCMODE			0x1c
#define DSIM_MDRESOL			0x20
#define DSIM_MVPORCH			0x24
#define DSIM_MHPORCH			0x28
#define DSIM_MSYNC			0x2c
#define DSIM_SDRESOL			0x30
#define DSIM_INTSRC			0x34
#define DSIM_INTMSK			0x38

/* packet */
#define DSIM_PKTHDR			0x3c
#define DSIM_PAYLOAD			0x40
#define DSIM_RXFIFO			0x44
#define DSIM_FIFOTHLD			0x48
#define DSIM_FIFOCTRL			0x4c
#define DSIM_MEMACCHR			0x50
#define DSIM_MULTI_PKT			0x78

/* pll control */
#define DSIM_PLLCTRL_1G			0x90
#define DSIM_PLLCTRL			0x94
#define DSIM_PLLCTRL1			0x98
#define DSIM_PLLCTRL2			0x9c
#define DSIM_PLLTMR			0xa0

/* dphy */
#define DSIM_PHYTIMING			0xb4
#define DSIM_PHYTIMING1			0xb8
#define DSIM_PHYTIMING2			0xbc

/* reg bit manipulation */
#define REG_MASK(e, s) (((1 << ((e) - (s) + 1)) - 1) << (s))
#define REG_PUT(x, e, s) (((x) << (s)) & REG_MASK(e, s))
#define REG_GET(x, e, s) (((x) & REG_MASK(e, s)) >> (s))

/* register bit fields */
#define STATUS_PLLSTABLE		BIT(31)
#define STATUS_SWRSTRLS			BIT(20)
#define STATUS_TXREADYHSCLK		BIT(10)
#define STATUS_ULPSCLK			BIT(9)
#define STATUS_STOPSTATECLK		BIT(8)
#define STATUS_GET_ULPSDAT(x)		REG_GET(x,  7,  4)
#define STATUS_GET_STOPSTATEDAT(x)	REG_GET(x,  3,  0)

#define RGB_STATUS_CMDMODE_INSEL	BIT(31)
#define RGB_STATUS_GET_RGBSTATE(x)	REG_GET(x, 12,  0)

#define CLKCTRL_TXREQUESTHSCLK		BIT(31)
#define CLKCTRL_DPHY_SEL_1G		BIT(29)
#define CLKCTRL_DPHY_SEL_1P5G		(0x0 << 29)
#define CLKCTRL_ESCCLKEN		BIT(28)
#define CLKCTRL_PLLBYPASS		BIT(29)
#define CLKCTRL_BYTECLKSRC_DPHY_PLL	REG_PUT(0, 26, 25)
#define CLKCTRL_BYTECLKEN		BIT(24)
#define CLKCTRL_SET_LANEESCCLKEN(x)	REG_PUT(x, 23, 19)
#define CLKCTRL_SET_ESCPRESCALER(x)	REG_PUT(x, 15,  0)

#define TIMEOUT_SET_BTAOUT(x)		REG_PUT(x, 23, 16)
#define TIMEOUT_SET_LPDRTOUT(x)		REG_PUT(x, 15,  0)

#define CONFIG_NON_CONTINUOUS_CLOCK_LANE	BIT(31)
#define CONFIG_CLKLANE_STOP_START	BIT(30)
#define CONFIG_MFLUSH_VS		BIT(29)
#define CONFIG_EOT_R03			BIT(28)
#define CONFIG_SYNCINFORM		BIT(27)
#define CONFIG_BURSTMODE		BIT(26)
#define CONFIG_VIDEOMODE		BIT(25)
#define CONFIG_AUTOMODE			BIT(24)
#define CONFIG_HSEDISABLEMODE		BIT(23)
#define CONFIG_HFPDISABLEMODE		BIT(22)
#define CONFIG_HBPDISABLEMODE		BIT(21)
#define CONFIG_HSADISABLEMODE		BIT(20)
#define CONFIG_SET_MAINVC(x)		REG_PUT(x, 19, 18)
#define CONFIG_SET_SUBVC(x)		REG_PUT(x, 17, 16)
#define CONFIG_SET_MAINPIXFORMAT(x)	REG_PUT(x, 14, 12)
#define CONFIG_SET_SUBPIXFORMAT(x)	REG_PUT(x, 10,  8)
#define CONFIG_SET_NUMOFDATLANE(x)	REG_PUT(x,  6,  5)
#define CONFIG_SET_LANEEN(x)		REG_PUT(x,  4,  0)

#define ESCMODE_SET_STOPSTATE_CNT(X)	REG_PUT(x, 31, 21)
#define ESCMODE_SET_STOPSTATE_CNT_MASK	(0x7ff << 21)
#define ESCMODE_FORCESTOPSTATE		BIT(20)
#define ESCMODE_FORCEBTA		BIT(16)
#define ESCMODE_CMDLPDT			BIT(7)
#define ESCMODE_TXLPDT			BIT(6)
#define ESCMODE_TXTRIGGERRST		BIT(5)

#define MDRESOL_MAINSTANDBY		BIT(31)
#define MDRESOL_SET_MAINVRESOL(x)	REG_PUT(x, 27, 16)
#define MDRESOL_SET_MAINHRESOL(x)	REG_PUT(x, 11,  0)

#define MVPORCH_SET_CMDALLOW(x)		REG_PUT(x, 31, 28)
#define MVPORCH_SET_STABLEVFP(x)	REG_PUT(x, 26, 16)
#define MVPORCH_SET_MAINVBP(x)		REG_PUT(x, 10,  0)

#define MHPORCH_SET_MAINHFP(x)		REG_PUT(x, 31, 16)
#define MHPORCH_SET_MAINHBP(x)		REG_PUT(x, 15,  0)

#define MSYNC_SET_MAINVSA(x)		REG_PUT(x, 31, 22)
#define MSYNC_SET_MAINHSA(x)		REG_PUT(x, 15,  0)

#define INTSRC_PLLSTABLE		BIT(31)
#define INTSRC_SWRSTRELEASE		BIT(30)
#define INTSRC_SFRPLFIFOEMPTY		BIT(29)
#define INTSRC_SFRPHFIFOEMPTY		BIT(28)
#define INTSRC_FRAMEDONE		BIT(24)
#define INTSRC_LPDRTOUT			BIT(21)
#define INTSRC_TATOUT			BIT(20)
#define INTSRC_RXDATDONE		BIT(18)
#define INTSRC_RXTE			BIT(17)
#define INTSRC_RXACK			BIT(16)
#define INTSRC_MASK			(INTSRC_PLLSTABLE	|	\
					 INTSRC_SWRSTRELEASE	|	\
					 INTSRC_SFRPLFIFOEMPTY	|	\
					 INTSRC_SFRPHFIFOEMPTY	|	\
					 INTSRC_FRAMEDONE	|	\
					 INTSRC_LPDRTOUT	|	\
					 INTSRC_TATOUT		|	\
					 INTSRC_RXDATDONE	|	\
					 INTSRC_RXTE		|	\
					 INTSRC_RXACK)

#define INTMSK_MSKPLLSTABLE		BIT(31)
#define INTMSK_MSKSWRELEASE		BIT(30)
#define INTMSK_MSKSFRPLFIFOEMPTY	BIT(29)
#define INTMSK_MSKSFRPHFIFOEMPTY	BIT(28)
#define INTMSK_MSKFRAMEDONE		BIT(24)
#define INTMSK_MSKLPDRTOUT		BIT(21)
#define INTMSK_MSKTATOUT		BIT(20)
#define INTMSK_MSKRXDATDONE		BIT(18)
#define INTMSK_MSKRXTE			BIT(17)
#define INTMSK_MSKRXACK			BIT(16)

#define PKTHDR_SET_DATA1(x)		REG_PUT(x, 23, 16)
#define PKTHDR_GET_DATA1(x)		REG_GET(x, 23, 16)
#define PKTHDR_SET_DATA0(x)		REG_PUT(x, 15,  8)
#define PKTHDR_GET_DATA0(x)		REG_GET(x, 15,  8)
#define PKTHDR_GET_WC(x)		REG_GET(x, 23,  8)
#define PKTHDR_SET_DI(x)		REG_PUT(x,  7,  0)
#define PKTHDR_GET_DI(x)		REG_GET(x,  7,  0)
#define PKTHDR_SET_DT(x)		REG_PUT(x,  5,  0)
#define PKTHDR_GET_DT(x)		REG_GET(x,  5,  0)
#define PKTHDR_SET_VC(x)		REG_PUT(x,  7,  6)
#define PKTHDR_GET_VC(x)		REG_GET(x,  7,  6)

#define FIFOCTRL_FULLRX			BIT(25)
#define FIFOCTRL_EMPTYRX		BIT(24)
#define FIFOCTRL_FULLHSFR		BIT(23)
#define FIFOCTRL_EMPTYHSFR		BIT(22)
#define FIFOCTRL_FULLLSFR		BIT(21)
#define FIFOCTRL_EMPTYLSFR		BIT(20)
#define FIFOCTRL_FULLHMAIN		BIT(11)
#define FIFOCTRL_EMPTYHMAIN		BIT(10)
#define FIFOCTRL_FULLLMAIN		BIT(9)
#define FIFOCTRL_EMPTYLMAIN		BIT(8)
#define FIFOCTRL_NINITRX		BIT(4)
#define FIFOCTRL_NINITSFR		BIT(3)
#define FIFOCTRL_NINITI80		BIT(2)
#define FIFOCTRL_NINITSUB		BIT(1)
#define FIFOCTRL_NINITMAIN		BIT(0)

#define PLLCTRL_DPDNSWAP_CLK		BIT(25)
#define PLLCTRL_DPDNSWAP_DAT		BIT(24)
#define PLLCTRL_PLLEN			BIT(23)
#define PLLCTRL_SET_PMS(x)		REG_PUT(x, 19,  1)
   #define PLLCTRL_SET_P(x)		REG_PUT(x, 18, 13)
   #define PLLCTRL_SET_M(x)		REG_PUT(x, 12,  3)
   #define PLLCTRL_SET_S(x)		REG_PUT(x,  2,  0)

#define PHYTIMING_SET_M_TLPXCTL(x)	REG_PUT(x, 15,  8)
#define PHYTIMING_SET_M_THSEXITCTL(x)	REG_PUT(x,  7,  0)

#define PHYTIMING1_SET_M_TCLKPRPRCTL(x)	 REG_PUT(x, 31, 24)
#define PHYTIMING1_SET_M_TCLKZEROCTL(x)	 REG_PUT(x, 23, 16)
#define PHYTIMING1_SET_M_TCLKPOSTCTL(x)	 REG_PUT(x, 15,  8)
#define PHYTIMING1_SET_M_TCLKTRAILCTL(x) REG_PUT(x,  7,  0)

#define PHYTIMING2_SET_M_THSPRPRCTL(x)	REG_PUT(x, 23, 16)
#define PHYTIMING2_SET_M_THSZEROCTL(x)	REG_PUT(x, 15,  8)
#define PHYTIMING2_SET_M_THSTRAILCTL(x)	REG_PUT(x,  7,  0)

/* fixed phy ref clk rate */
#define PHY_REF_CLK		12000

#define MAX_MAIN_HRESOL		2047
#define MAX_MAIN_VRESOL		2047
#define MAX_SUB_HRESOL		1024
#define MAX_SUB_VRESOL		1024

/* in KHZ */
#define MAX_ESC_CLK_FREQ	20000

/* dsim all irqs index */
#define PLLSTABLE		1
#define SWRSTRELEASE		2
#define SFRPLFIFOEMPTY		3
#define SFRPHFIFOEMPTY		4
#define SYNCOVERRIDE		5
#define BUSTURNOVER		6
#define FRAMEDONE		7
#define LPDRTOUT		8
#define TATOUT			9
#define RXDATDONE		10
#define RXTE			11
#define RXACK			12
#define ERRRXECC		13
#define ERRRXCRC		14
#define ERRESC3			15
#define ERRESC2			16
#define ERRESC1			17
#define ERRESC0			18
#define ERRSYNC3		19
#define ERRSYNC2		20
#define ERRSYNC1		21
#define ERRSYNC0		22
#define ERRCONTROL3		23
#define ERRCONTROL2		24
#define ERRCONTROL1		25
#define ERRCONTROL0		26

#define MIPI_FIFO_TIMEOUT	msecs_to_jiffies(250)

#define MIPI_HFP_PKT_OVERHEAD	6
#define MIPI_HBP_PKT_OVERHEAD	6
#define MIPI_HSA_PKT_OVERHEAD	6

/* DPHY PLL structure */
struct sec_dsim_range {
	uint32_t min;
	uint32_t max;
};

struct sec_dsim_pll {
	struct sec_dsim_range p;
	struct sec_dsim_range m;
	struct sec_dsim_range s;
	struct sec_dsim_range k;
	struct sec_dsim_range fin;
	struct sec_dsim_range fpref;
	struct sec_dsim_range fvco;
};

/* DPHY timings structure */
struct sec_dsim_dphy_timing {
	uint32_t bit_clk;	/* MHz */

	uint32_t clk_prepare;
	uint32_t clk_zero;
	uint32_t clk_post;
	uint32_t clk_trail;

	uint32_t hs_prepare;
	uint32_t hs_zero;
	uint32_t hs_trail;

	uint32_t lpx;
	uint32_t hs_exit;
};

#define DSIM_DPHY_TIMING(bclk, cpre, czero, cpost, ctrail,	\
			 hpre, hzero, htrail, lp, hexit)	\
	.bit_clk	= bclk,					\
	.clk_prepare	= cpre,					\
	.clk_zero	= czero,				\
	.clk_post	= cpost,				\
	.clk_trail	= ctrail,				\
	.hs_prepare	= hpre,					\
	.hs_zero	= hzero,				\
	.hs_trail	= htrail,				\
	.lpx		= lp,					\
	.hs_exit	= hexit

struct sec_dsim_plat_data {
	uint32_t version;
	uint32_t max_data_lanes;
	uint64_t max_data_rate;
	const struct sec_dsim_dphy_timing *dphy_timing;
	uint32_t num_dphy_timing;
	const struct sec_dsim_pll *dphy_pll;
	int (*dphy_timing_cmp)(const void *key, const void *elt);
};

#include "sec_mipi_dphy_ln14lpp.h"
#include "sec_mipi_pll_1432x.h"

#define DRIVER_NAME		"sec-dsim"

/* used for CEA standard modes */
struct dsim_hblank_par {
	char *name;		/* drm display mode name */
	int vrefresh;
	int hfp_wc;
	int hbp_wc;
	int hsa_wc;
	int lanes;
};

struct dsim_pll_pms {
	uint32_t bit_clk;	/* kHz */
	uint32_t p;
	uint32_t m;
	uint32_t s;
	uint32_t k;
};

struct sec_dsim {
	struct mipi_dsi_host host;
	struct drm_bridge bridge;
	struct drm_bridge *panel_bridge;
	struct device *dev;

	struct regmap *regmap;
	void __iomem *base;
	int irq;

	struct phy *phy;
	struct clk *clk_cfg;
	struct clk *clk_pllref;
	struct clk *pclk;			/* pixel clock */

	/* kHz clocks */
	uint32_t pix_clk;
	uint32_t bit_clk;
	uint32_t pref_clk;			/* phy ref clock rate in KHz */

	unsigned int lanes;
	unsigned int channel;			/* virtual channel */
	enum mipi_dsi_pixel_format format;
	unsigned long mode_flags;
	const struct dsim_hblank_par *hpar;
	unsigned int pms;
	unsigned int p;
	unsigned int m;
	unsigned int s;
	unsigned long long lp_data_rate;
	unsigned long long hs_data_rate;
	struct videomode vmode;

	struct completion pll_stable;
	struct completion ph_tx_done;
	struct completion pl_tx_done;
	struct completion rx_done;
	const struct sec_dsim_plat_data *pdata;
};

static inline struct sec_dsim *host_to_dsim(struct mipi_dsi_host *host)
{
	return container_of(host, struct sec_dsim, host);
}

static inline struct sec_dsim *bridge_to_dsim(struct drm_bridge *bridge)
{
	return container_of(bridge, struct sec_dsim, bridge);
}

static void dsim_write(struct sec_dsim *dsim, u32 val, unsigned int reg)
{
	int ret;

	ret = regmap_write(dsim->regmap, reg, val);
	if (ret < 0)
		dev_err(dsim->dev, "failed to write SEC DSIM reg 0x%x: %d\n",
			reg, ret);
}

static u32 dsim_read(struct sec_dsim *dsim, u32 reg)
{
	unsigned int val;
	int ret;

	printk("1. %s: 0x%x\n", __func__, reg);
	ret = regmap_read(dsim->regmap, reg, &val);
	if (ret < 0)
		dev_err(dsim->dev, "failed to read SEC DSIM reg 0x%x: %d\n",
			reg, ret);

	printk("%s\n", __func__);
	return val;
}

static const struct regmap_config sec_dsim_regmap_config = {
	.reg_bits = 8,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = 0xbc,
	.name = DRIVER_NAME,
};

static void __maybe_unused sec_dsim_irq_mask(struct sec_dsim *dsim,
						  int irq_idx)
{
	uint32_t intmsk;

	intmsk = dsim_read(dsim, DSIM_INTMSK);

	switch (irq_idx) {
	case PLLSTABLE:
		intmsk |= INTMSK_MSKPLLSTABLE;
		break;
	case SWRSTRELEASE:
		intmsk |= INTMSK_MSKSWRELEASE;
		break;
	case SFRPLFIFOEMPTY:
		intmsk |= INTMSK_MSKSFRPLFIFOEMPTY;
		break;
	case SFRPHFIFOEMPTY:
		intmsk |= INTMSK_MSKSFRPHFIFOEMPTY;
		break;
	case FRAMEDONE:
		intmsk |= INTMSK_MSKFRAMEDONE;
		break;
	case LPDRTOUT:
		intmsk |= INTMSK_MSKLPDRTOUT;
		break;
	case TATOUT:
		intmsk |= INTMSK_MSKTATOUT;
		break;
	case RXDATDONE:
		intmsk |= INTMSK_MSKRXDATDONE;
		break;
	case RXTE:
		intmsk |= INTMSK_MSKRXTE;
		break;
	case RXACK:
		intmsk |= INTMSK_MSKRXACK;
		break;
	default:
		/* unsupported irq */
		return;
	}

	dsim_write(dsim, intmsk, DSIM_INTMSK);
}

static void sec_dsim_irq_unmask(struct sec_dsim *dsim,
				     int irq_idx)
{
	uint32_t intmsk;

	intmsk = dsim_read(dsim, DSIM_INTMSK);

	switch (irq_idx) {
	case PLLSTABLE:
		intmsk &= ~INTMSK_MSKPLLSTABLE;
		break;
	case SWRSTRELEASE:
		intmsk &= ~INTMSK_MSKSWRELEASE;
		break;
	case SFRPLFIFOEMPTY:
		intmsk &= ~INTMSK_MSKSFRPLFIFOEMPTY;
		break;
	case SFRPHFIFOEMPTY:
		intmsk &= ~INTMSK_MSKSFRPHFIFOEMPTY;
		break;
	case FRAMEDONE:
		intmsk &= ~INTMSK_MSKFRAMEDONE;
		break;
	case LPDRTOUT:
		intmsk &= ~INTMSK_MSKLPDRTOUT;
		break;
	case TATOUT:
		intmsk &= ~INTMSK_MSKTATOUT;
		break;
	case RXDATDONE:
		intmsk &= ~INTMSK_MSKRXDATDONE;
		break;
	case RXTE:
		intmsk &= ~INTMSK_MSKRXTE;
		break;
	case RXACK:
		intmsk &= ~INTMSK_MSKRXACK;
		break;
	default:
		/* unsupported irq */
		return;
	}

	dsim_write(dsim, intmsk, DSIM_INTMSK);
}

/* write 1 clear irq */
static void sec_dsim_irq_clear(struct sec_dsim *dsim,
				    int irq_idx)
{
	uint32_t intsrc = 0;

	switch (irq_idx) {
	case PLLSTABLE:
		intsrc |= INTSRC_PLLSTABLE;
		break;
	case SWRSTRELEASE:
		intsrc |= INTSRC_SWRSTRELEASE;
		break;
	case SFRPLFIFOEMPTY:
		intsrc |= INTSRC_SFRPLFIFOEMPTY;
		break;
	case SFRPHFIFOEMPTY:
		intsrc |= INTSRC_SFRPHFIFOEMPTY;
		break;
	case FRAMEDONE:
		intsrc |= INTSRC_FRAMEDONE;
		break;
	case LPDRTOUT:
		intsrc |= INTSRC_LPDRTOUT;
		break;
	case TATOUT:
		intsrc |= INTSRC_TATOUT;
		break;
	case RXDATDONE:
		intsrc |= INTSRC_RXDATDONE;
		break;
	case RXTE:
		intsrc |= INTSRC_RXTE;
		break;
	case RXACK:
		intsrc |= INTSRC_RXACK;
		break;
	default:
		/* unsupported irq */
		return;
	}

	dsim_write(dsim, intsrc, DSIM_INTSRC);
}

static void sec_dsim_irq_init(struct sec_dsim *dsim)
{
	sec_dsim_irq_unmask(dsim, PLLSTABLE);
	sec_dsim_irq_unmask(dsim, SWRSTRELEASE);
}

static irqreturn_t sec_dsim_irq_handler(int irq, void *data)
{
	uint32_t intsrc, status;
	struct sec_dsim *dsim = data;

	intsrc = dsim_read(dsim, DSIM_INTSRC);
	status = dsim_read(dsim, DSIM_STATUS);

	if (WARN_ON(!intsrc)) {
		dev_err(dsim->dev, "interrupt is not from dsim\n");
		return IRQ_NONE;
	}

	if (WARN_ON(!(intsrc & INTSRC_MASK))) {
		dev_warn(dsim->dev, "unenable irq happens: %#x\n", intsrc);
		/* just clear irqs */
		dsim_write(dsim, intsrc, DSIM_INTSRC);
		return IRQ_NONE;
	}

	if (intsrc & INTSRC_PLLSTABLE) {
		WARN_ON(!(status & STATUS_PLLSTABLE));
		sec_dsim_irq_clear(dsim, PLLSTABLE);
		complete(&dsim->pll_stable);
	}

	if (intsrc & INTSRC_SWRSTRELEASE)
		sec_dsim_irq_clear(dsim, SWRSTRELEASE);

	if (intsrc & INTSRC_SFRPLFIFOEMPTY) {
		sec_dsim_irq_clear(dsim, SFRPLFIFOEMPTY);
		complete(&dsim->pl_tx_done);
	}

	if (intsrc & INTSRC_SFRPHFIFOEMPTY) {
		sec_dsim_irq_clear(dsim, SFRPHFIFOEMPTY);
		complete(&dsim->ph_tx_done);
	}

	if (WARN_ON(intsrc & INTSRC_LPDRTOUT)) {
		sec_dsim_irq_clear(dsim, LPDRTOUT);
		dev_warn(dsim->dev, "LP RX timeout\n");
	}

	if (WARN_ON(intsrc & INTSRC_TATOUT)) {
		sec_dsim_irq_clear(dsim, TATOUT);
		dev_warn(dsim->dev, "Turns around Acknowledge timeout\n");
	}

	if (intsrc & INTSRC_RXDATDONE) {
		sec_dsim_irq_clear(dsim, RXDATDONE);
		complete(&dsim->rx_done);
	}

	if (intsrc & INTSRC_RXTE) {
		sec_dsim_irq_clear(dsim, RXTE);
		dev_dbg(dsim->dev, "TE Rx trigger received\n");
	}

	if (intsrc & INTSRC_RXACK) {
		sec_dsim_irq_clear(dsim, RXACK);
		dev_dbg(dsim->dev, "ACK Rx trigger received\n");
	}

	return IRQ_HANDLED;
}

#define DSIM_HBLANK_PARAM(nm, vf, hfp, hbp, hsa, num)	\
	.name	  = (nm),				\
	.vrefresh = (vf),				\
	.hfp_wc   = (hfp),				\
	.hbp_wc   = (hbp),				\
	.hsa_wc   = (hsa),				\
	.lanes	  = (num)

static const struct dsim_hblank_par hblank_4lanes[] = {
	/* {  88, 148, 44 } */
	{ DSIM_HBLANK_PARAM("1920x1080", 60,  60, 105,  27, 4), },
	/* { 528, 148, 44 } */
	{ DSIM_HBLANK_PARAM("1920x1080", 50, 390, 105,  27, 4), },
	/* {  88, 148, 44 } */
	{ DSIM_HBLANK_PARAM("1920x1080", 30,  60, 105,  27, 4), },
	/* { 110, 220, 40 } */
	{ DSIM_HBLANK_PARAM("1280x720" , 60,  78, 159,  24, 4), },
	/* { 440, 220, 40 } */
	{ DSIM_HBLANK_PARAM("1280x720" , 50, 324, 159,  24, 4), },
	/* {  16,  60, 62 } */
	{ DSIM_HBLANK_PARAM("720x480"  , 60,   6,  39,  40, 4), },
	/* {  12,  68, 64 } */
	{ DSIM_HBLANK_PARAM("720x576"  , 50,   3,  45,  42, 4), },
	/* {  16,  48, 96 } */
	{ DSIM_HBLANK_PARAM("640x480"  , 60,   6,  30,  66, 4), },
};

static const struct dsim_hblank_par hblank_2lanes[] = {
	/* {  88, 148, 44 } */
	{ DSIM_HBLANK_PARAM("1920x1080", 30, 114, 210,  60, 2), },
	/* { 110, 220, 40 } */
	{ DSIM_HBLANK_PARAM("1280x720" , 60, 159, 320,  40, 2), },
	/* { 440, 220, 40 } */
	{ DSIM_HBLANK_PARAM("1280x720" , 50, 654, 320,  40, 2), },
	/* {  16,  60, 62 } */
	{ DSIM_HBLANK_PARAM("720x480"  , 60,  16,  66,  88, 2), },
	/* {  12,  68, 64 } */
	{ DSIM_HBLANK_PARAM("720x576"  , 50,  12,  96,  72, 2), },
	/* {  16,  48, 96 } */
	{ DSIM_HBLANK_PARAM("640x480"  , 60,  18,  66, 138, 2), },
};

static const struct dsim_hblank_par *sec_dsim_get_hblank_par(const char *name,
								  int vrefresh,
								  int lanes)
{
	int i, size;
	const struct dsim_hblank_par *hpar, *hblank;

	if (unlikely(!name))
		return NULL;

	switch (lanes) {
	case 2:
		hblank = hblank_2lanes;
		size   = ARRAY_SIZE(hblank_2lanes);
		break;
	case 4:
		hblank = hblank_4lanes;
		size   = ARRAY_SIZE(hblank_4lanes);
		break;
	default:
		pr_err("No hblank data for mode %s with %d lanes\n",
		       name, lanes);
		return NULL;
	}

	for (i = 0; i < size; i++) {
		hpar = &hblank[i];

		if (!strcmp(name, hpar->name)) {
			if (vrefresh != hpar->vrefresh)
				continue;

			/* found */
			return hpar;
		}
	}

	return NULL;
}

/* For now, dsim only support one device attached */
static int sec_dsim_host_attach(struct mipi_dsi_host *host,
				     struct mipi_dsi_device *dsi)
{
	struct sec_dsim *dsim = host_to_dsim(host);

	dsim->lanes	 = dsi->lanes;
	dsim->channel	 = dsi->channel;
	dsim->format	 = dsi->format;
	dsim->mode_flags = dsi->mode_flags;

	dev_info(dsim->dev, "%s end!\n", __func__);
	return 0;
}

static int sec_dsim_host_detach(struct mipi_dsi_host *host,
				     struct mipi_dsi_device *dsi)
{
	struct sec_dsim *dsim = host_to_dsim(host);

	/* clear the saved dsi parameters */
	dsim->lanes	 = 0;
	dsim->channel	 = 0;
	dsim->format	 = 0;
	dsim->mode_flags = 0;

	return 0;
}

static void sec_dsim_config_cmd_lpm(struct sec_dsim *dsim,
					 bool enable)
{
	uint32_t escmode;

	escmode = dsim_read(dsim, DSIM_ESCMODE);

	if (enable)
		escmode |= ESCMODE_CMDLPDT;
	else
		escmode &= ~ESCMODE_CMDLPDT;

	dsim_write(dsim, escmode, DSIM_ESCMODE);
}

static void sec_dsim_write_pl_to_sfr_fifo(struct sec_dsim *dsim,
					       const void *payload,
					       size_t length)
{
	uint32_t pl_data;

	if (!length)
		return;

	while (length >= 4) {
		pl_data = get_unaligned_le32(payload);
		dsim_write(dsim, pl_data, DSIM_PAYLOAD);
		payload += 4;
		length -= 4;
	}

	pl_data = 0;
	switch (length) {
	case 3:
		pl_data |= ((u8 *)payload)[2] << 16;
		/* fall through */
	case 2:
		pl_data |= ((u8 *)payload)[1] << 8;
		/* fall through */
	case 1:
		pl_data |= ((u8 *)payload)[0];
		dsim_write(dsim, pl_data, DSIM_PAYLOAD);
		break;
	}
}

static void sec_dsim_write_ph_to_sfr_fifo(struct sec_dsim *dsim,
					       void *header,
					       bool use_lpm)
{
	uint32_t pkthdr;

	pkthdr = PKTHDR_SET_DATA1(((u8 *)header)[2])	| /* WC MSB  */
		 PKTHDR_SET_DATA0(((u8 *)header)[1])	| /* WC LSB  */
		 PKTHDR_SET_DI(((u8 *)header)[0]);	  /* Data ID */

	dsim_write(dsim, pkthdr, DSIM_PKTHDR);
}

static int sec_dsim_read_pl_from_sfr_fifo(struct sec_dsim *dsim,
					       void *payload,
					       size_t length)
{
	uint8_t data_type;
	uint16_t word_count = 0;
	uint32_t fifoctrl, ph, pl;

	fifoctrl = dsim_read(dsim, DSIM_FIFOCTRL);

	if (WARN_ON(fifoctrl & FIFOCTRL_EMPTYRX))
		return -EINVAL;

	ph = dsim_read(dsim, DSIM_RXFIFO);
	data_type = PKTHDR_GET_DT(ph);
	switch (data_type) {
	case MIPI_DSI_RX_ACKNOWLEDGE_AND_ERROR_REPORT:
		dev_err(dsim->dev, "peripheral report error: (0-7)%x, (8-15)%x\n",
			PKTHDR_GET_DATA0(ph), PKTHDR_GET_DATA1(ph));
		return -EPROTO;
	case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_2BYTE:
	case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_2BYTE:
		if (!WARN_ON(length < 2)) {
			((u8 *)payload)[1] = PKTHDR_GET_DATA1(ph);
			word_count++;
		}
		/* fall through */
	case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_1BYTE:
	case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_1BYTE:
		((u8 *)payload)[0] = PKTHDR_GET_DATA0(ph);
		word_count++;
		length = word_count;
		break;
	case MIPI_DSI_RX_DCS_LONG_READ_RESPONSE:
	case MIPI_DSI_RX_GENERIC_LONG_READ_RESPONSE:
		word_count = PKTHDR_GET_WC(ph);
		if (word_count > length) {
			dev_err(dsim->dev, "invalid receive buffer length\n");
			return -EINVAL;
		}

		length = word_count;

		while (word_count >= 4) {
			pl = dsim_read(dsim, DSIM_RXFIFO);
			((u8 *)payload)[0] = pl & 0xff;
			((u8 *)payload)[1] = (pl >> 8)  & 0xff;
			((u8 *)payload)[2] = (pl >> 16) & 0xff;
			((u8 *)payload)[3] = (pl >> 24) & 0xff;
			payload += 4;
			word_count -= 4;
		}

		if (word_count > 0) {
			pl = dsim_read(dsim, DSIM_RXFIFO);

			switch (word_count) {
			case 3:
				((u8 *)payload)[2] = (pl >> 16) & 0xff;
				/* fall through */
			case 2:
				((u8 *)payload)[1] = (pl >> 8) & 0xff;
				/* fall through */
			case 1:
				((u8 *)payload)[0] = pl & 0xff;
				break;
			}
		}

		break;
	default:
		return -EINVAL;
	}

	return length;
}

static ssize_t sec_dsim_host_transfer(struct mipi_dsi_host *host,
					   const struct mipi_dsi_msg *msg)
{
	int ret;
	bool use_lpm;
	struct mipi_dsi_packet packet;
	struct sec_dsim *dsim = host_to_dsim(host);

	if ((msg->rx_buf && !msg->rx_len) || (msg->rx_len && !msg->rx_buf))
		return -EINVAL;

	ret = mipi_dsi_create_packet(&packet, msg);
	if (ret) {
		dev_err(dsim->dev, "failed to create dsi packet: %d\n", ret);
		return ret;
	}

	/* need to read data from peripheral */
	if (unlikely(msg->rx_buf))
		reinit_completion(&dsim->rx_done);

	/* config LPM for CMD TX */
	use_lpm = msg->flags & MIPI_DSI_MSG_USE_LPM ? true : false;
	sec_dsim_config_cmd_lpm(dsim, use_lpm);

	if (packet.payload_length) {		/* Long Packet case */
		reinit_completion(&dsim->pl_tx_done);

		/* write packet payload */
		sec_dsim_write_pl_to_sfr_fifo(dsim,
						   packet.payload,
						   packet.payload_length);

		/* write packet header */
		sec_dsim_write_ph_to_sfr_fifo(dsim,
						   packet.header,
						   use_lpm);

		ret = wait_for_completion_timeout(&dsim->ph_tx_done,
						  MIPI_FIFO_TIMEOUT);
		if (!ret) {
			dev_err(dsim->dev, "wait payload tx done time out\n");
			return -EBUSY;
		}
	} else {
		reinit_completion(&dsim->ph_tx_done);

		/* write packet header */
		sec_dsim_write_ph_to_sfr_fifo(dsim,
						   packet.header,
						   use_lpm);

		ret = wait_for_completion_timeout(&dsim->ph_tx_done,
						  MIPI_FIFO_TIMEOUT);
		if (!ret) {
			dev_err(dsim->dev, "wait pkthdr tx done time out\n");
			return -EBUSY;
		}
	}

	/* read packet payload */
	if (unlikely(msg->rx_buf)) {
		ret = wait_for_completion_timeout(&dsim->rx_done,
						  MIPI_FIFO_TIMEOUT);
		if (!ret) {
			dev_err(dsim->dev, "wait rx done time out\n");
			return -EBUSY;
		}

		ret = sec_dsim_read_pl_from_sfr_fifo(dsim,
							  msg->rx_buf,
							  msg->rx_len);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static const struct mipi_dsi_host_ops sec_dsim_host_ops = {
	.attach   = sec_dsim_host_attach,
	.detach   = sec_dsim_host_detach,
	.transfer = sec_dsim_host_transfer,
};

static int sec_dsim_bridge_attach(struct drm_bridge *bridge,
				       enum drm_bridge_attach_flags flags)
{
	struct sec_dsim *dsim = bridge_to_dsim(bridge);
	struct drm_bridge *panel_bridge;
	struct drm_panel *panel;
	int ret;

	dev_info(dsim->dev, "%s In!\n", __func__);
	ret = drm_of_find_panel_or_bridge(dsim->dev->of_node, 1, 0, &panel,
					  &panel_bridge);
	if (ret)
		return ret;

	if (panel) {
		panel_bridge = drm_panel_bridge_add(panel);
		if (IS_ERR(panel_bridge))
			return PTR_ERR(panel_bridge);
	}
	dsim->panel_bridge = panel_bridge;

	if (!dsim->panel_bridge)
		return -EPROBE_DEFER;

	return drm_bridge_attach(bridge->encoder, dsim->panel_bridge, bridge,
				 flags);
}

static int sec_dsim_config_pll(struct sec_dsim *dsim)
{
	int ret;
	uint32_t pllctrl = 0, status, data_lanes_en, stop;

	dsim_write(dsim, 0x1f4, DSIM_PLLTMR);

	/* TODO: config dp/dn swap if requires */

	pllctrl |= PLLCTRL_SET_PMS(dsim->pms) | PLLCTRL_PLLEN;
	pllctrl = 0x0080c630;
	dsim_write(dsim, pllctrl, DSIM_PLLCTRL);

	ret = wait_for_completion_timeout(&dsim->pll_stable, HZ / 10);
	if (!ret) {
		dev_err(dsim->dev, "wait for pll stable time out\n");
		return -EBUSY;
	}

	/* wait for clk & data lanes to go to stop state */
	mdelay(1);

	data_lanes_en = (0x1 << dsim->lanes) - 1;
	status = dsim_read(dsim, DSIM_STATUS);
	if (!(status & STATUS_STOPSTATECLK)) {
		dev_err(dsim->dev, "clock is not in stop state\n");
		return -EBUSY;
	}

	stop = STATUS_GET_STOPSTATEDAT(status);
	if ((stop & data_lanes_en) != data_lanes_en) {
		dev_err(dsim->dev,
			"one or more data lanes is not in stop state\n");
		return -EBUSY;
	}

	return 0;
}

static void sec_dsim_set_main_mode(struct sec_dsim *dsim)
{
	uint32_t bpp, hfp_wc, hbp_wc, hsa_wc, wc;
	uint32_t mdresol = 0, mvporch = 0, mhporch = 0, msync = 0;
	struct videomode *vmode = &dsim->vmode;

	mdresol |= MDRESOL_SET_MAINVRESOL(vmode->vactive) |
		   MDRESOL_SET_MAINHRESOL(vmode->hactive);
	dsim_write(dsim, mdresol, DSIM_MDRESOL);

	mvporch |= MVPORCH_SET_MAINVBP(vmode->vback_porch)    |
		   MVPORCH_SET_STABLEVFP(vmode->vfront_porch) |
		   MVPORCH_SET_CMDALLOW(0xf);
	dsim_write(dsim, mvporch, DSIM_MVPORCH);

	bpp = mipi_dsi_pixel_format_to_bpp(dsim->format);

	/* calculate hfp & hbp word counts */
	if (!dsim->hpar) {
		wc = DIV_ROUND_UP(vmode->hfront_porch * (bpp >> 3),
				  dsim->lanes);
		hfp_wc = wc > MIPI_HFP_PKT_OVERHEAD ?
			 wc - MIPI_HFP_PKT_OVERHEAD : vmode->hfront_porch;
		wc = DIV_ROUND_UP(vmode->hback_porch * (bpp >> 3),
				  dsim->lanes);
		hbp_wc = wc > MIPI_HBP_PKT_OVERHEAD ?
			 wc - MIPI_HBP_PKT_OVERHEAD : vmode->hback_porch;
	} else {
		hfp_wc = dsim->hpar->hfp_wc;
		hbp_wc = dsim->hpar->hbp_wc;
	}

	mhporch |= MHPORCH_SET_MAINHFP(hfp_wc) |
		   MHPORCH_SET_MAINHBP(hbp_wc);

	dsim_write(dsim, mhporch, DSIM_MHPORCH);

	/* calculate hsa word counts */
	if (!dsim->hpar) {
		wc = DIV_ROUND_UP(vmode->hsync_len * (bpp >> 3),
				  dsim->lanes);
		hsa_wc = wc > MIPI_HSA_PKT_OVERHEAD ?
			 wc - MIPI_HSA_PKT_OVERHEAD : vmode->hsync_len;
	} else
		hsa_wc = dsim->hpar->hsa_wc;

	msync |= MSYNC_SET_MAINVSA(vmode->vsync_len) |
		 MSYNC_SET_MAINHSA(hsa_wc);

	dsim_write(dsim, msync, DSIM_MSYNC);
}

static void sec_dsim_config_dpi(struct sec_dsim *dsim)
{
	uint32_t config = 0, rgb_status = 0, data_lanes_en;

	if (dsim->mode_flags & MIPI_DSI_MODE_VIDEO)
		rgb_status &= ~RGB_STATUS_CMDMODE_INSEL;
	else
		rgb_status |= RGB_STATUS_CMDMODE_INSEL;

	//dsim_write(dsim, rgb_status, DSIM_RGB_STATUS);

	if (dsim->mode_flags & MIPI_DSI_CLOCK_NON_CONTINUOUS) {
		config |= CONFIG_NON_CONTINUOUS_CLOCK_LANE;
		config |= CONFIG_CLKLANE_STOP_START;
	}

	if (!(dsim->mode_flags & MIPI_DSI_MODE_VSYNC_FLUSH))
		config |= CONFIG_MFLUSH_VS;

	/* disable EoT packets in HS mode */
	if (!(dsim->mode_flags & MIPI_DSI_MODE_EOT_PACKET))
		config |= CONFIG_EOT_R03;

	if (dsim->mode_flags & MIPI_DSI_MODE_VIDEO) {
		config |= CONFIG_VIDEOMODE;

		if (dsim->mode_flags & MIPI_DSI_MODE_VIDEO_BURST)
			config |= CONFIG_BURSTMODE;

		else if (dsim->mode_flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE)
			config |= CONFIG_SYNCINFORM;

		if (dsim->mode_flags & MIPI_DSI_MODE_VIDEO_AUTO_VERT)
			config |= CONFIG_AUTOMODE;

		if (dsim->mode_flags & MIPI_DSI_MODE_VIDEO_HSE)
			config |= CONFIG_HSEDISABLEMODE;

		if (!(dsim->mode_flags & MIPI_DSI_MODE_VIDEO_HFP))
			config |= CONFIG_HFPDISABLEMODE;

		if (!(dsim->mode_flags & MIPI_DSI_MODE_VIDEO_HBP))
			config |= CONFIG_HBPDISABLEMODE;

		if (!(dsim->mode_flags & MIPI_DSI_MODE_VIDEO_HSA))
			config |= CONFIG_HSADISABLEMODE;
	}

	config |= CONFIG_SET_MAINVC(dsim->channel);

	if (dsim->mode_flags & MIPI_DSI_MODE_VIDEO) {
		switch (dsim->format) {
		case MIPI_DSI_FMT_RGB565:
			config |= CONFIG_SET_MAINPIXFORMAT(0x4);
			break;
		case MIPI_DSI_FMT_RGB666_PACKED:
			config |= CONFIG_SET_MAINPIXFORMAT(0x5);
			break;
		case MIPI_DSI_FMT_RGB666:
			config |= CONFIG_SET_MAINPIXFORMAT(0x6);
			break;
		case MIPI_DSI_FMT_RGB888:
			config |= CONFIG_SET_MAINPIXFORMAT(0x7);
			break;
		default:
			config |= CONFIG_SET_MAINPIXFORMAT(0x7);
			break;
		}
	}

	/* config data lanes number and enable lanes */
	data_lanes_en = (0x1 << dsim->lanes) - 1;
	config |= CONFIG_SET_NUMOFDATLANE(dsim->lanes - 1);
	config |= CONFIG_SET_LANEEN(0x1 | data_lanes_en << 1);

	dsim_write(dsim, config, DSIM_CONFIG);

	config = 0x01e00000;
	dsim_write(dsim, config, DSIM_ESCMODE);
}

static void sec_dsim_config_dphy(struct sec_dsim *dsim)
{
	struct sec_dsim_dphy_timing key = { 0 };
	const struct sec_dsim_dphy_timing *match = NULL;
	const struct sec_dsim_plat_data *pdata = dsim->pdata;
	uint32_t phytiming = 0, phytiming1 = 0, phytiming2 = 0, timeout = 0;
	uint32_t hactive, vactive;
	struct videomode *vmode = &dsim->vmode;
	struct drm_display_mode mode;

	dev_info(dsim->dev, "%s In!\n", __func__);
	key.bit_clk = DIV_ROUND_CLOSEST_ULL(dsim->bit_clk, 1000);

	/* '1280x720@60Hz' mode with 2 data lanes
	 * requires special fine tuning for DPHY
	 * TIMING config according to the tests.
	 */
	if (dsim->lanes == 2) {
		hactive = vmode->hactive;
		vactive = vmode->vactive;

		if (hactive == 1280 && vactive == 720) {
			memset(&mode, 0x0, sizeof(mode));
			drm_display_mode_from_videomode(vmode, &mode);

			if (drm_mode_vrefresh(&mode) == 60)
				key.bit_clk >>= 1;
		}
	}

	match = bsearch(&key, pdata->dphy_timing, pdata->num_dphy_timing,
			sizeof(struct sec_dsim_dphy_timing),
			pdata->dphy_timing_cmp);
	if (WARN_ON(!match))
		return;

	phytiming  |= PHYTIMING_SET_M_TLPXCTL(match->lpx)	|
		      PHYTIMING_SET_M_THSEXITCTL(match->hs_exit);
	phytiming = 0x0000060b;
	dsim_write(dsim, phytiming, DSIM_PHYTIMING);

	phytiming1 |= PHYTIMING1_SET_M_TCLKPRPRCTL(match->clk_prepare)	|
		      PHYTIMING1_SET_M_TCLKZEROCTL(match->clk_zero)	|
		      PHYTIMING1_SET_M_TCLKPOSTCTL(match->clk_post)	|
		      PHYTIMING1_SET_M_TCLKTRAILCTL(match->clk_trail);
	phytiming1 = 0x07260d08;
	dsim_write(dsim, phytiming1, DSIM_PHYTIMING1);

	phytiming2 |= PHYTIMING2_SET_M_THSPRPRCTL(match->hs_prepare)	|
		      PHYTIMING2_SET_M_THSZEROCTL(match->hs_zero)	|
		      PHYTIMING2_SET_M_THSTRAILCTL(match->hs_trail);
	phytiming2 = 0x00080d0b;
	dsim_write(dsim, phytiming2, DSIM_PHYTIMING2);

	timeout |= TIMEOUT_SET_BTAOUT(0xff)	|
		   TIMEOUT_SET_LPDRTOUT(0xffff);
	dsim_write(dsim, timeout, DSIM_TIMEOUT);
}

static void sec_dsim_init_fifo_pointers(struct sec_dsim *dsim)
{
	uint32_t fifoctrl, fifo_ptrs;

	fifoctrl = dsim_read(dsim, DSIM_FIFOCTRL);

	fifo_ptrs = FIFOCTRL_NINITRX	|
		    FIFOCTRL_NINITSFR	|
		    FIFOCTRL_NINITI80	|
		    FIFOCTRL_NINITSUB	|
		    FIFOCTRL_NINITMAIN;

	fifoctrl &= ~fifo_ptrs;
	dsim_write(dsim, fifoctrl, DSIM_FIFOCTRL);
	udelay(500);

	fifoctrl |= fifo_ptrs;
	dsim_write(dsim, fifoctrl, DSIM_FIFOCTRL);
	udelay(500);
}

static void sec_dsim_config_clkctrl(struct sec_dsim *dsim)
{
	uint32_t clkctrl = 0, data_lanes_en;
	uint32_t byte_clk, esc_prescaler;

	clkctrl |= CLKCTRL_TXREQUESTHSCLK;

	/* using 1.5Gbps PHY */
	clkctrl |= CLKCTRL_DPHY_SEL_1P5G;

	clkctrl |= CLKCTRL_ESCCLKEN;

	clkctrl &= ~CLKCTRL_PLLBYPASS;

	clkctrl |= CLKCTRL_BYTECLKSRC_DPHY_PLL;

	clkctrl |= CLKCTRL_BYTECLKEN;

	data_lanes_en = (0x1 << dsim->lanes) - 1;
	clkctrl |= CLKCTRL_SET_LANEESCCLKEN(0x1 | data_lanes_en << 1);

	/* calculate esc prescaler from byte clock:
	 * EscClk = ByteClk / EscPrescaler;
	 */
	byte_clk = dsim->bit_clk >> 3;
	esc_prescaler = DIV_ROUND_UP(byte_clk, MAX_ESC_CLK_FREQ);
	clkctrl |= CLKCTRL_SET_ESCPRESCALER(esc_prescaler);

	dsim_write(dsim, clkctrl, DSIM_CLKCTRL);
}

static void sec_dsim_set_standby(struct sec_dsim *dsim,
				      bool standby)
{
	uint32_t mdresol = 0;

	mdresol = dsim_read(dsim, DSIM_MDRESOL);

	if (standby)
		mdresol |= MDRESOL_MAINSTANDBY;
	else
		mdresol &= ~MDRESOL_MAINSTANDBY;

	dsim_write(dsim, mdresol, DSIM_MDRESOL);
}

struct dsim_pll_pms *sec_dsim_calc_pmsk(struct sec_dsim *dsim)
{
	uint32_t p, m, s;
	uint32_t best_p = 0, best_m = 0, best_s = 0;
	uint32_t fin, fout;
	uint32_t s_pow_2, raw_s;
	uint64_t mfin, pfvco, pfout, psfout;
	uint32_t delta, best_delta = ~0U;
	struct dsim_pll_pms *pll_pms;
	struct device *dev = dsim->dev;
	const struct sec_dsim_plat_data *pdata = dsim->pdata;
	struct sec_dsim_pll dpll = *pdata->dphy_pll;
	struct sec_dsim_range *prange = &dpll.p;
	struct sec_dsim_range *mrange = &dpll.m;
	struct sec_dsim_range *srange = &dpll.s;
	struct sec_dsim_range *krange = &dpll.k;
	struct sec_dsim_range *fvco_range  = &dpll.fvco;
	struct sec_dsim_range *fpref_range = &dpll.fpref;
	struct sec_dsim_range pr_new = *prange;
	struct sec_dsim_range sr_new = *srange;

	pll_pms = devm_kzalloc(dev, sizeof(*pll_pms), GFP_KERNEL);
	if (!pll_pms) {
		dev_err(dev, "Unable to allocate 'pll_pms'\n");
		return ERR_PTR(-ENOMEM);
	}

	fout = dsim->bit_clk;
	fin  = dsim->pref_clk;

	/* TODO: ignore 'k' for PMS calculation,
	 * only use 'p', 'm' and 's' to generate
	 * the requested PLL output clock.
	 */
	krange->min = 0;
	krange->max = 0;

	dev_info(dev,
        "prange->min %d, fin %d, fpref_range->min %d, prange->max %d, fpref_range->max %d\n",
        prange->min, fin, fpref_range->min, prange->max, fpref_range->max);

        dev_info(dev,
        "mrange->min %d, pfvco %lld, fvco_range->min %d, mrange->max %d, fvco_range->max %d\n",
        mrange->min, pfvco, fvco_range->min, mrange->max, fvco_range->max);

	/* narrow 'p' range via 'Fpref' limitation:
	 * Fpref : [2MHz ~ 30MHz] (Fpref = Fin / p)
	 */
	prange->min = max(prange->min, DIV_ROUND_UP(fin, fpref_range->max));
	prange->max = min(prange->max, fin / fpref_range->min);

	/* narrow 'm' range via 'Fvco' limitation:
	 * Fvco: [1050MHz ~ 2100MHz] (Fvco = ((m + k / 65536) * Fin) / p)
	 * So, m = Fvco * p / Fin and Fvco > Fin;
	 */
	pfvco = (uint64_t)fvco_range->min * prange->min;
	mrange->min = max_t(uint32_t, mrange->min,
			    DIV_ROUND_UP_ULL(pfvco, fin));
	pfvco = (uint64_t)fvco_range->max * prange->max;
	mrange->max = min_t(uint32_t, mrange->max,
			    DIV_ROUND_UP_ULL(pfvco, fin));

	dev_info(dev, "p: min = %u, max = %u, "
		     "m: min = %u, max = %u, "
		     "s: min = %u, max = %u\n",
		prange->min, prange->max, mrange->min,
		mrange->max, srange->min, srange->max);

	/* first determine 'm', then can determine 'p', last determine 's' */
	for (m = mrange->min; m <= mrange->max; m++) {
		/* p = m * Fin / Fvco */
		mfin = (uint64_t)m * fin;
		pr_new.min = max_t(uint32_t, prange->min,
				   DIV_ROUND_UP_ULL(mfin, fvco_range->max));
		pr_new.max = min_t(uint32_t, prange->max,
				   (mfin / fvco_range->min));

		if (pr_new.max < pr_new.min || pr_new.min < prange->min)
			continue;

		for (p = pr_new.min; p <= pr_new.max; p++) {
			/* s = order_pow_of_two((m * Fin) / (p * Fout)) */
			pfout = (uint64_t)p * fout;
			raw_s = DIV_ROUND_CLOSEST_ULL(mfin, pfout);

			s_pow_2 = rounddown_pow_of_two(raw_s);
			sr_new.min = max_t(uint32_t, srange->min,
					   order_base_2(s_pow_2));

			s_pow_2 = roundup_pow_of_two(DIV_ROUND_CLOSEST_ULL(mfin, pfout));
			sr_new.max = min_t(uint32_t, srange->max,
					   order_base_2(s_pow_2));

			if (sr_new.max < sr_new.min || sr_new.min < srange->min)
				continue;

			for (s = sr_new.min; s <= sr_new.max; s++) {
				/* fout = m * Fin / (p * 2^s) */
				psfout = pfout * (1 << s);
				delta = abs(psfout - mfin);
				if (delta < best_delta) {
					best_p = p;
					best_m = m;
					best_s = s;
					best_delta = delta;
				}
			}
		}
	}

	if (best_delta == ~0U) {
		devm_kfree(dev, pll_pms);
		return ERR_PTR(-EINVAL);
	}

	pll_pms->p = best_p;
	pll_pms->m = best_m;
	pll_pms->s = best_s;

	dev_info(dev, "fout = %u, fin = %u, m = %u, "
		     "p = %u, s = %u, best_delta = %u\n",
		fout, fin, pll_pms->m, pll_pms->p, pll_pms->s, best_delta);

	return pll_pms;
}

static int sec_dsim_check_pll_out(struct sec_dsim *dsim,
				       const struct drm_display_mode *mode)
{
	int bpp;
	uint32_t pix_clk, bit_clk;
	const struct sec_dsim_plat_data *pdata = dsim->pdata;
	const struct dsim_hblank_par *hpar;
	const struct dsim_pll_pms *pmsk;

	bpp = mipi_dsi_pixel_format_to_bpp(dsim->format);
	if (bpp < 0)
		return -EINVAL;

	pix_clk = mode->clock;
	bit_clk = DIV_ROUND_UP(pix_clk * bpp, dsim->lanes);

	dev_info(dsim->dev,
		 "bpp %d, clock %d, lane %d, pix_clk %d, bit_clk %d, max_rate %lld\n",
		 bpp, mode->clock, dsim->lanes, pix_clk, bit_clk * 1000, pdata->max_data_rate);
	if (bit_clk * 1000 > pdata->max_data_rate) {
		dev_err(dsim->dev,
			"reuest bit clk freq exceeds lane's maximum value\n");
		return -EINVAL;
	}

	dsim->pix_clk = pix_clk;
	dsim->bit_clk = bit_clk;
	dsim->hpar = NULL;

	dev_info(dsim->dev, "pix_clk %d, bit_clk %d\n", dsim->pix_clk, dsim->bit_clk);
	pmsk = sec_dsim_calc_pmsk(dsim);
	if (IS_ERR(pmsk)) {
		dev_err(dsim->dev,
			"failed to get pmsk for: fin = %u, fout = %u\n",
			dsim->pref_clk, dsim->bit_clk);
		return -EINVAL;
	}

	dsim->pms = PLLCTRL_SET_P(pmsk->p) |
		    PLLCTRL_SET_M(pmsk->m) |
		    PLLCTRL_SET_S(pmsk->s);

	/* free 'dsim_pll_pms' structure data which is
	 * allocated in 'sec_dsim_calc_pmsk()'.
	 */
	devm_kfree(dsim->dev, (void *)pmsk);

	if (dsim->mode_flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE) {
		hpar = sec_dsim_get_hblank_par(mode->name,
						    drm_mode_vrefresh(mode),
						    dsim->lanes);
		dsim->hpar = hpar;
		if (!hpar)
			dev_dbg(dsim->dev, "no pre-exist hpar can be used\n");
	}

	return 0;
}

static void sec_dsim_bridge_enable(struct drm_bridge *bridge)
{
	int ret;
	struct sec_dsim *dsim = bridge_to_dsim(bridge);

	dev_info(dsim->dev, "%s In!\n", __func__);

	request_bus_freq(BUS_FREQ_HIGH);

	clk_prepare_enable(dsim->clk_pllref);

	clk_prepare_enable(dsim->clk_cfg);

	ret = phy_power_on(dsim->phy);
	if (ret) {
		dev_err(dsim->dev, "cannot enable phy %d\n", ret);
		return;
	}

	sec_dsim_irq_init(dsim);

	/* At this moment, the dsim bridge's preceding encoder has
	 * already been enabled. So the dsim can be configed here
	 */

	/* config main display mode */
	sec_dsim_set_main_mode(dsim);

	/* config dsim dpi */
	sec_dsim_config_dpi(dsim);

	/* config dsim pll */
	ret = sec_dsim_config_pll(dsim);
	if (ret) {
		dev_err(dsim->dev, "dsim pll config failed: %d\n", ret);
		return;
	}

	/* config dphy timings */
	sec_dsim_config_dphy(dsim);

	/* initialize FIFO pointers */
	sec_dsim_init_fifo_pointers(dsim);

	/* config esc clock, byte clock and etc */
	sec_dsim_config_clkctrl(dsim);

	/* enable data transfer of dsim */
	sec_dsim_set_standby(dsim, true);

	return;
}

static void sec_dsim_disable_clkctrl(struct sec_dsim *dsim)
{
	uint32_t clkctrl;

	clkctrl = dsim_read(dsim, DSIM_CLKCTRL);

	clkctrl &= ~CLKCTRL_TXREQUESTHSCLK;

	clkctrl &= ~CLKCTRL_ESCCLKEN;

	clkctrl &= ~CLKCTRL_BYTECLKEN;

	dsim_write(dsim, clkctrl, DSIM_CLKCTRL);
}

static void sec_dsim_disable_pll(struct sec_dsim *dsim)
{
	uint32_t pllctrl;

	pllctrl  = dsim_read(dsim, DSIM_PLLCTRL);

	pllctrl &= ~PLLCTRL_PLLEN;

	dsim_write(dsim, pllctrl, DSIM_PLLCTRL);
}

static void sec_dsim_bridge_disable(struct drm_bridge *bridge)
{
	struct sec_dsim *dsim = bridge_to_dsim(bridge);

	dev_info(dsim->dev, "%s In!\n", __func__);
	/* disable data transfer of dsim */
	sec_dsim_set_standby(dsim, false);

	/* disable esc clock & byte clock */
	sec_dsim_disable_clkctrl(dsim);

	/* disable dsim pll */
	sec_dsim_disable_pll(dsim);

	clk_disable_unprepare(dsim->clk_cfg);

	clk_disable_unprepare(dsim->clk_pllref);

	release_bus_freq(BUS_FREQ_HIGH);
}

static bool sec_dsim_bridge_mode_fixup(struct drm_bridge *bridge,
					    const struct drm_display_mode *mode,
					    struct drm_display_mode *adjusted_mode)
{
	struct sec_dsim *dsim = bridge_to_dsim(bridge);
	dev_info(dsim->dev, "%s In!\n", __func__);

	adjusted_mode->flags |= (DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC);
	adjusted_mode->flags &= ~(DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC);

	return true;
}

static void sec_dsim_bridge_mode_set(struct drm_bridge *bridge,
					  const struct drm_display_mode *mode,
					  const struct drm_display_mode *adjusted_mode)
{
	struct sec_dsim *dsim = bridge_to_dsim(bridge);
	int ret;

	dev_info(dsim->dev, "%s In!\n", __func__);
	/* This hook is called when the display pipe is completely
	 * off. And since the pm runtime is implemented, the dsim
	 * hardware cannot be accessed at this moment. So move all
	 * the mode_set config to ->enable() hook.
	 * And this hook is called only when 'mode_changed' is true,
	 * so it is called not every time atomic commit.
	 */

	ret = sec_dsim_check_pll_out(dsim, adjusted_mode);
	if (ret) {
		dev_err(dsim->dev, "failed to pll out (ret=%d)\n", ret);
		return;
	}

	drm_display_mode_to_videomode(adjusted_mode, &dsim->vmode);
}

static const struct drm_bridge_funcs sec_dsim_bridge_funcs = {
	.attach     = sec_dsim_bridge_attach,
	.enable     = sec_dsim_bridge_enable,
	.disable    = sec_dsim_bridge_disable,
	.mode_set   = sec_dsim_bridge_mode_set,
	.mode_fixup = sec_dsim_bridge_mode_fixup,
};

static const struct drm_bridge_timings sec_dsim_bridge_timings = {
	.input_bus_flags = DRM_BUS_FLAG_DE_LOW,
};

static inline int dphy_timing_default_cmp(const void *key, const void *elt)
{
	const struct sec_dsim_dphy_timing *_key = key;
	const struct sec_dsim_dphy_timing *_elt = elt;

	/* find an element whose 'bit_clk' is equal to the
	 * the key's 'bit_clk' value or, the difference
	 * between them is less than 5.
	 */
	if (abs((int)(_elt->bit_clk - _key->bit_clk)) <= 5)
		return 0;

	if (_key->bit_clk < _elt->bit_clk)
		/* search bottom half */
		return 1;
	else
		/* search top half */
		return -1;
}

static const struct sec_dsim_plat_data imx8mm_mipi_dsim_plat_data = {
	.version	= 0x1060200,
	.max_data_lanes = 4,
	.max_data_rate  = 1500000000ULL,
	.dphy_pll	= &pll_1432x,
	.dphy_timing	= dphy_timing_ln14lpp_v1p2,
	.num_dphy_timing = ARRAY_SIZE(dphy_timing_ln14lpp_v1p2),
	.dphy_timing_cmp = dphy_timing_default_cmp,
};

static const struct of_device_id sec_dsim_dt_ids[] = {
	{
		.compatible = "fsl,imx8mm-mipi-dsim",
		.data = &imx8mm_mipi_dsim_plat_data,
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, sec_dsim_dt_ids);

static int sec_dsim_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct of_device_id *of_id;
	struct sec_dsim *dsim;
	struct resource *res;
	int irq, version;
	int ret;

	dsim = devm_kzalloc(dev, sizeof(*dsim), GFP_KERNEL);
	if (!dsim) {
		dev_err(dev, "Unable to allocate 'dsim'\n");
		return -ENOMEM;
	}

	of_id = of_match_device(sec_dsim_dt_ids, dev);
	if (!of_id)
		return -ENODEV;

	dsim->pdata = of_id->data;
	dsim->dev = dev;

	init_completion(&dsim->pll_stable);
	init_completion(&dsim->ph_tx_done);
	init_completion(&dsim->pl_tx_done);
	init_completion(&dsim->rx_done);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	dsim->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(dsim->base))
		return PTR_ERR(dsim->base);

	dsim->regmap =
		devm_regmap_init_mmio(dev, dsim->base, &sec_dsim_regmap_config);
	if (IS_ERR(dsim->regmap)) {
		ret = PTR_ERR(dsim->regmap);
		dev_err(dsim->dev, "failed to create SEC DSIM regmap: %d\n", ret);
		return ret;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return -ENODEV;

	ret = devm_request_irq(dev, irq, sec_dsim_irq_handler,
			       0, dev_name(dev), dsim);
	if (ret) {
		dev_err(dev, "failed to request dsim irq: %d\n", ret);
		return ret;
	}

	dsim->phy = devm_phy_get(dev, "dsim");
        if (IS_ERR(dsim->phy)) {
                dev_info(dev, "failed to get dsim phy\n");
                return PTR_ERR(dsim->phy);
        }

	dsim->clk_pllref = devm_clk_get(dev, "bus_clk");
	if (IS_ERR(dsim->clk_pllref)) {
		ret = PTR_ERR(dsim->clk_pllref);
		dev_err(dev, "Unable to get phy pll reference clock: %d\n", ret);
		return ret;
	}

	dev_info(dev, "%s: 7\n", __func__);
	dsim->clk_cfg = devm_clk_get(dev, "sclk_mipi");
	if (IS_ERR(dsim->clk_cfg)) {
		ret = PTR_ERR(dsim->clk_cfg);
		dev_err(dev, "Unable to get configuration clock: %d\n", ret);
		return ret;
	}

	dsim->pref_clk = PHY_REF_CLK;

	dev_info(dev, "%s: 8\n", __func__);
//	version = dsim_read(dsim, DSIM_VERSION);
//	WARN_ON(version != dsim->pdata->version);
//	dev_info(dev, "version number is %#x\n", version);

	dsim->host.ops = &sec_dsim_host_ops;
	dsim->host.dev = dsim->dev;

	ret = mipi_dsi_host_register(&dsim->host);
	if (ret) {
		dev_err(dev, "Unable to register mipi dsi host: %d\n", ret);
		return ret;
	}

	dsim->bridge.driver_private = dsim;
	dsim->bridge.funcs = &sec_dsim_bridge_funcs;
	dsim->bridge.of_node = dev->of_node;
	dsim->bridge.timings = &sec_dsim_bridge_timings,
	dev_set_drvdata(dev, dsim);

	drm_bridge_add(&dsim->bridge);

	dev_info(dev, "%s: end!\n", __func__);
	return 0;
}

static int sec_dsim_remove(struct platform_device *pdev)
{
	struct sec_dsim *dsim = platform_get_drvdata(pdev);;

	mipi_dsi_host_unregister(&dsim->host);

	return 0;
}

struct platform_driver sec_dsim_driver = {
	.probe    = sec_dsim_probe,
	.remove   = sec_dsim_remove,
	.driver   = {
		.name = DRIVER_NAME,
		.of_match_table = sec_dsim_dt_ids,
	},
};

module_platform_driver(sec_dsim_driver);

MODULE_DESCRIPTION("Samsung MIPI DSI Host Controller bridge driver");
MODULE_AUTHOR("Fancy Fang <chen.fang@nxp.com>");
MODULE_LICENSE("GPL");
