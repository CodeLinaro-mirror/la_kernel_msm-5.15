// SPDX-License-Identifier: GPL-2.0-only

// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/kthread.h>
#include <linux/of.h>
#include <linux/sysfs.h>
#include <linux/string.h>

#include "dwmac-qcom-serdes.h"

#define SKIP 0xFFFFFFFF

enum serdes_mode_index {
	SERDES_MODE_SGMII_1G,
	SERDES_MODE_SGMII_2P5G,
	SERDES_MODE_USXGMII_2P5G,
	SERDES_MODE_USXGMII_5G,
	SERDES_MODE_USXGMII_10G,
	SERDES_MODE_NUM_MODES,
};

struct reg_value_per_mode {
	unsigned int reg;
	u32 value[SERDES_MODE_NUM_MODES];
};

#define REG_SEQ(reg_name, v0, v1, v2, v3, v4) \
	{ .reg = (reg_name), .value = {(v0), (v1), (v2), (v3), (v4)} }

/* Union of the QSERDES_* register sequences written by
 * qcom_ethqos_serdes_sgmii_1Gb(), _sgmii_2p5Gb(), _usxgmii_2p5Gb(),
 * _usxgmii_5Gb() and _usxgmii_10Gb_1Gb(), aligned row-by-row per mode
 * (SKIP where a given mode does not write that register at that step).
 * Only covers the non-v3 (EMAC_HW_v3_1_0) register set.
 */
static struct reg_value_per_mode serdes_prog_table[] = {
	REG_SEQ(QSERDES_PCS_SW_RESET, 0x01, 0x01, 0x01, 0x01, 0x01),
	REG_SEQ(QSERDES_PCS_POWER_DOWN_CONTROL, 0x01, 0x01, 0x01, 0x01, 0x01),
	REG_SEQ(QSERDES_COM_BG_TIMER, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E),
	REG_SEQ(QSERDES_COM_BIAS_EN_CLKBUFLR_EN, 0x07, 0x07, 0x07, 0x07, 0x07),
	REG_SEQ(QSERDES_COM_CLK_ENABLE1, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F),
	REG_SEQ(QSERDES_COM_PLL_IVCO, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F),
	REG_SEQ(QSERDES_COM_CP_CTRL_MODE0, 0x08, 0x08, 0x08, 0x08, 0x08),
	REG_SEQ(QSERDES_COM_PLL_RCTRL_MODE0, 0x16, 0x16, 0x16, 0x16, 0x16),
	REG_SEQ(QSERDES_COM_PLL_CCTRL_MODE0, 0x36, 0x36, 0x36, 0x36, 0x36),
	REG_SEQ(QSERDES_COM_INTEGLOOP_GAIN0_MODE0, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F),
	REG_SEQ(QSERDES_COM_INTEGLOOP_GAIN1_MODE0, 0x00, 0x00, 0x00, 0x00, 0x00),
	REG_SEQ(QSERDES_COM_PLL_EN, 0x03, 0x03, 0x03, 0x03, 0x03),
	REG_SEQ(QSERDES_COM_SYSCLK_EN_SEL, 0x1A, 0x1A, 0x1A, 0x1A, 0x1A),
	REG_SEQ(QSERDES_COM_LOCK_CMP1_MODE0, 0x0A, 0x1A, 0x91, 0x23, 0x46),
	REG_SEQ(QSERDES_COM_LOCK_CMP2_MODE0, 0x1A, 0x41, 0x21, 0x43, 0x86),
	REG_SEQ(QSERDES_COM_DEC_START_MODE0, 0x82, 0x7A, 0x86, 0x86, 0x86),
	REG_SEQ(QSERDES_COM_DIV_FRAC_START1_MODE0, 0x55, 0x00, 0x00, 0x00, 0x00),
	REG_SEQ(QSERDES_COM_DIV_FRAC_START2_MODE0, 0x55, 0x20, 0x70, 0x70, 0x70),
	REG_SEQ(QSERDES_COM_DIV_FRAC_START3_MODE0, 0x03, 0x01, 0x04, 0x04, 0x04),
	REG_SEQ(QSERDES_COM_VCO_TUNE1_MODE0, 0x84, 0x01, 0x46, 0x46, 0x46),
	REG_SEQ(QSERDES_COM_VCO_TUNE2_MODE0, 0x01, 0x02, 0x01, 0x01, 0x01),
	REG_SEQ(QSERDES_COM_VCO_TUNE_INITVAL2, 0x00, 0x00, 0x00, 0x00, 0x00),
	REG_SEQ(QSERDES_COM_HSCLK_SEL_1, 0x04, 0x03, 0x04, 0x01, 0x00),
	REG_SEQ(QSERDES_COM_HSCLK_HS_SWITCH_SEL_1, 0x04, 0x03, 0x00, 0x01, 0x00),
	REG_SEQ(QSERDES_COM_CORECLK_DIV_MODE0, 0x0A, 0x05, 0x04, 0x04, 0x04),
	REG_SEQ(QSERDES_COM_CORE_CLK_EN, 0x00, 0x00, 0x10, 0x10, 0x10),
	REG_SEQ(QSERDES_COM_CMN_CONFIG_1, 0x16, 0x16, 0x16, 0x16, 0x16),
	REG_SEQ(QSERDES_COM_BIN_VCOCAL_CMP_CODE1_MODE0, 0xB9, 0xCD, 0xAE, 0xAE, 0xAE),
	REG_SEQ(QSERDES_COM_BIN_VCOCAL_CMP_CODE2_MODE0, 0x1E, 0x1C, 0x1F, 0x1F, 0x1F),
	REG_SEQ(QSERDES_COM_BIN_VCOCAL_HSCLK_SEL_1, 0x11, 0x11, 0x11, 0x11, 0x11),
	REG_SEQ(QSERDES_TX_CLKBUF_ENABLE, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D),
	REG_SEQ(QSERDES_TX0_TX_BAND, 0x05, 0x04, 0x04, 0x04, 0x04),
	REG_SEQ(QSERDES_TX0_SLEW_CNTL, 0x0A, 0x0A, 0x08, 0x08, 0x08),
	REG_SEQ(QSERDES_TX0_RES_CODE_LANE_OFFSET_TX, 0x09, 0x09, 0x09, 0x09, 0x09),
	REG_SEQ(QSERDES_TX0_RES_CODE_LANE_OFFSET_RX, 0x09, 0x09, 0x09, 0x09, 0x09),
	REG_SEQ(QSERDES_TX0_LANE_MODE_1, 0xF5, 0xF5, 0xF5, 0xF5, 0xF5),
	REG_SEQ(QSERDES_TX0_LANE_MODE_2, 0x02, 0x02, 0x06, 0x06, 0x06),
	REG_SEQ(QSERDES_TX0_LANE_MODE_3, 0x3F, 0x00, 0x3F, 0x3F, 0x3F),
	REG_SEQ(QSERDES_TX_LANE_MODE_4, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F),
	REG_SEQ(QSERDES_TX_LANE_MODE_5, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F),
	REG_SEQ(QSERDES_TX0_RCV_DETECT_LVL_2, 0x12, 0x12, 0x12, 0x12, 0x12),
	REG_SEQ(QSERDES_TX0_TRAN_DRVR_EMP_EN, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F),
	REG_SEQ(QSERDES_TX_TX_EMP_POST1_LVL, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B),
	REG_SEQ(QSERDES_RX0_UCDR_FO_GAIN, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D),
	REG_SEQ(QSERDES_RX0_UCDR_SO_GAIN, 0x03, 0x03, 0x03, 0x03, 0x03),
	REG_SEQ(QSERDES_RX0_UCDR_FASTLOCK_FO_GAIN, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A),
	REG_SEQ(QSERDES_RX0_UCDR_SO_SATURATION_AND_ENABLE, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F),
	REG_SEQ(QSERDES_RX0_UCDR_FASTLOCK_COUNT_LOW, 0x00, 0x00, 0x00, 0x00, 0x00),
	REG_SEQ(QSERDES_RX0_UCDR_FASTLOCK_COUNT_HIGH, 0x01, 0x01, 0x01, 0x01, 0x01),
	REG_SEQ(QSERDES_RX0_UCDR_PI_CONTROLS, 0x81, 0x81, 0x81, 0x81, 0x81),
	REG_SEQ(QSERDES_RX0_UCDR_PI_CTRL2, 0x81, 0x81, 0x80, 0x81, 0x81),
	REG_SEQ(QSERDES_RX_UCDR_SB2_THRESH1, 0x00, 0x00, 0x00, 0x00, 0x00),
	REG_SEQ(QSERDES_RX_UCDR_SB2_THRESH2, 0x00, 0x00, 0x00, 0x00, 0x00),
	REG_SEQ(QSERDES_RX0_RX_TERM_BW, 0x00, 0x00, 0x03, 0x03, 0x03),
	REG_SEQ(QSERDES_RX0_VGA_CAL_CNTRL2, 0x08, 0x08, 0x08, 0x08, 0x08),
	REG_SEQ(QSERDES_RX0_GM_CAL, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F),
	REG_SEQ(QSERDES_RX0_RX_EQU_ADAPTOR_CNTRL1, 0x04, 0x04, 0x04, 0x04, 0x04),
	REG_SEQ(QSERDES_RX0_RX_EQU_ADAPTOR_CNTRL2, 0x00, 0x00, 0x00, 0x00, 0x00),
	REG_SEQ(QSERDES_RX0_RX_EQU_ADAPTOR_CNTRL3, 0x4A, 0x4A, 0x4A, 0x4A, 0x4A),
	REG_SEQ(QSERDES_RX0_RX_EQU_ADAPTOR_CNTRL4, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A),
	REG_SEQ(QSERDES_RX0_RX_IDAC_TSETTLE_LOW, 0x80, 0x80, 0x80, 0x80, 0x80),
	REG_SEQ(QSERDES_RX0_RX_IDAC_TSETTLE_HIGH, 0x01, 0x01, 0x01, 0x01, 0x01),
	REG_SEQ(QSERDES_RX0_RX_IDAC_MEASURE_TIME, 0x20, 0x20, 0x20, 0x20, 0x20),
	REG_SEQ(QSERDES_RX0_RX_EQ_OFFSET_ADAPTOR_CNTRL1, 0x17, 0x17, 0x17, 0x17, 0x17),
	REG_SEQ(QSERDES_RX0_RX_OFFSET_ADAPTOR_CNTRL2, 0x00, 0x00, 0x00, 0x00, 0x00),
	REG_SEQ(QSERDES_RX0_SIGDET_CNTRL, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F),
	REG_SEQ(QSERDES_RX0_SIGDET_DEGLITCH_CNTRL, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E),
	REG_SEQ(QSERDES_RX0_RX_BAND, 0x05, 0x18, 0x18, 0x18, 0x18),
	REG_SEQ(QSERDES_RX0_RX_MODE_00_LOW, 0x18, 0x18, 0x18, 0x1F, 0x1F),
	REG_SEQ(QSERDES_RX0_RX_MODE_00_HIGH, 0xC8, 0xC8, 0xC8, 0xBF, 0xBF),
	REG_SEQ(QSERDES_RX0_RX_MODE_00_HIGH2, 0xC8, 0xC8, 0xC8, 0xFF, 0xFF),
	REG_SEQ(QSERDES_RX0_RX_MODE_00_HIGH3, 0x0C, 0x0C, 0x0C, 0xDF, 0xDF),
	REG_SEQ(QSERDES_RX0_RX_MODE_00_HIGH4, 0xB8, 0xB8, 0xB8, 0xED, 0xED),
	REG_SEQ(QSERDES_RX0_RX_MODE_01_LOW, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0),
	REG_SEQ(QSERDES_RX0_RX_MODE_01_HIGH, 0xC8, 0xC8, 0xC8, 0xC8, 0xC8),
	REG_SEQ(QSERDES_RX0_RX_MODE_01_HIGH2, 0xC8, 0xC8, 0xC8, 0xC8, 0xC8),
	REG_SEQ(QSERDES_RX0_RX_MODE_01_HIGH3, 0x09, 0x09, 0x09, 0x09, 0x09),
	REG_SEQ(QSERDES_RX0_RX_MODE_01_HIGH4, 0xB1, 0xB1, 0xB1, 0xB1, 0xB1),
	REG_SEQ(QSERDES_RX0_RX_MODE_10_LOW, SKIP, SKIP, SKIP, 0xE0, 0xE0),
	REG_SEQ(QSERDES_RX0_RX_MODE_10_HIGH, SKIP, SKIP, SKIP, 0xC8, 0xC8),
	REG_SEQ(QSERDES_RX0_RX_MODE_10_HIGH2, SKIP, SKIP, SKIP, 0xC8, 0xC8),
	REG_SEQ(QSERDES_RX0_RX_MODE_10_HIGH3, SKIP, SKIP, SKIP, 0x3B, 0x3B),
	REG_SEQ(QSERDES_RX0_RX_MODE_10_HIGH4, SKIP, SKIP, SKIP, 0xB7, 0xB7),
	REG_SEQ(QSERDES_RX0_DCC_CTRL1, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C),
	REG_SEQ(QSERDES_PCS_LINE_RESET_TIME, 0x0C, 0x0C, SKIP, 0x0C, 0x0C),
	REG_SEQ(QSERDES_PCS_TX_SMALL_AMP_DRV_LVL, 0x03, 0x03, 0x03, 0x03, 0x03),
	REG_SEQ(QSERDES_PCS_TX_LARGE_AMP_DRV_LVL, 0x1A, 0x1A, 0x1A, 0x1A, 0x1A),
	REG_SEQ(QSERDES_PCS_TX_LARGE_AMP_POST_EMP_LVL, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B),
	REG_SEQ(QSERDES_PCS_TX_SMALL_AMP_POST_EMP_LVL, 0x00, 0x00, 0x2F, 0x00, 0x00),
	REG_SEQ(QSERDES_PCS_TX_MID_TERM_CTRL1, 0x83, 0x83, 0x83, 0x83, 0x83),
	REG_SEQ(QSERDES_PCS_TX_MID_TERM_CTRL2, 0x08, 0x08, 0x08, 0x08, 0x08),
	REG_SEQ(QSERDES_RX0_SIGDET_CAL_CTRL1, 0x10, 0x10, 0x00, 0x00, 0x00),
	REG_SEQ(QSERDES_PCS_RX_SIGDET_CTRL2, 0x77, 0x77, 0xCC, 0xCC, 0xCC),
	REG_SEQ(QSERDES_RX0_SIGDET_CAL_CTRL2_AND_CDR_LOCK_EDGE, 0x00, 0x00, 0x00, 0x00, 0x00),
	REG_SEQ(QSERDES_PCS_SGMII_MISC_CTRL7, 0x00, 0x00, 0x00, 0x00, 0x00),
	REG_SEQ(QSERDES_PCS_SGMII_MISC_CTRL8, 0x0C, 0x8C, 0x14, 0x14, 0x14),
	REG_SEQ(QSERDES_PCS_RETIME_BUFFER_EN, 0x01, 0x01, 0x01, 0x01, 0x01),
	REG_SEQ(QSERDES_PCS_SW_RESET, 0x00, 0x00, 0x00, 0x00, 0x00),
	REG_SEQ(QSERDES_PCS_PHY_START, 0x01, 0x01, 0x01, 0x01, 0x01),
	REG_SEQ(QSERDES_PCS_PHY_START, 0x01, 0x01, SKIP, 0x01, 0x01),
};

static const size_t serdes_prog_table_size = ARRAY_SIZE(serdes_prog_table);

#if IS_ENABLED(CONFIG_ETHQOS_QCOM_VER4)
void qcom_ethqos_serdes_power_down(struct qcom_ethqos *ethqos)
{
	writel_relaxed(0x08, ethqos->sgmii_base + QSERDES_PCS_TX_MID_TERM_CTRL2);
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES_PCS_SW_RESET);
	usleep_range(100, 200);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_PCS_SW_RESET);
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES_PCS_PHY_START);
	writel_relaxed(0x0, ethqos->sgmii_base + QSERDES_PCS_POWER_DOWN_CONTROL);

	/* Set current speed to 0 so that serdes will be reprogrammed on next link up. */
	ethqos->curr_serdes_speed = 0;
}
#endif

void qcom_ethqos_serdes_phy_soft_reset(struct qcom_ethqos *ethqos)
{
	int ret = 0;
	int retry = 500;
	unsigned int val;

	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES_PCS_SW_RESET);
	usleep_range(100, 200);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_PCS_SW_RESET);

	do {
		val = readl_relaxed(ethqos->sgmii_base + QSERDES_COM_CMN_STATUS);
		val &= BIT(1);
		if (val)
			break;
		usleep_range(1000, 1500);
		retry--;
	} while (retry > 0);
	if (!retry) {
		ETHQOSERR("PLL Lock Status timedout with retry = %d\n", retry);
		ret = -1;
		goto err_ret;
	}

	retry = 500;
	do {
		val = readl_relaxed(ethqos->sgmii_base + QSERDES_COM_C_READY_STATUS);
		val &= BIT(0);
		if (val)
			break;
		usleep_range(1000, 1500);
		retry--;
	} while (retry > 0);
	if (!retry) {
		ETHQOSERR("C_READY_STATUS timedout with retry = %d\n", retry);
		ret = -1;
		goto err_ret;
	}

	retry = 500;
	do {
		val = readl_relaxed(ethqos->sgmii_base + QSERDES_PCS_READY_STATUS);
		val &= BIT(7);
		if (val)
			break;
		usleep_range(1000, 1500);
		retry--;
	} while (retry > 0);
	if (!retry) {
		ETHQOSERR("PCS_READY timedout with retry = %d\n", retry);
		ret = -1;
		goto err_ret;
	}

	retry = 500;
	do {
		val = readl_relaxed(ethqos->sgmii_base + QSERDES_PCS_READY_STATUS);
		val &= BIT(0);
		if (val)
			break;
		usleep_range(1000, 1500);
		retry--;
	} while (retry > 0);
	if (!retry) {
		ETHQOSERR("SGMIIPHY_READY timedout with retry = %d\n", retry);
		ret = -1;
		goto err_ret;
	}

	return;

err_ret:
	ETHQOSERR("Serdes phy soft reset failed\n");
}
EXPORT_SYMBOL_GPL(qcom_ethqos_serdes_phy_soft_reset);

void qcom_ethqos_serdes_soft_reset(struct qcom_ethqos *ethqos)
{
	int ret = 0;
	int retry = 500;
	unsigned int val;

	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES_PCS_SW_RESET);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_PCS_SW_RESET);
	usleep_range(3000, 5000);
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES_PCS_PHY_START);
	usleep_range(3000, 5000);

	do {
		val = readl_relaxed(ethqos->sgmii_base + QSERDES_COM_CMN_STATUS);
		val &= BIT(1);
		if (val)
			break;
		usleep_range(1000, 1500);
		retry--;
	} while (retry > 0);
	if (!retry) {
		ETHQOSERR("PLL Lock Status timedout with retry = %d\n", retry);
		ret = -1;
		goto err_ret;
	}

	retry = 500;
	do {
		val = readl_relaxed(ethqos->sgmii_base + QSERDES_COM_C_READY_STATUS);
		val &= BIT(0);
		if (val)
			break;
		usleep_range(1000, 1500);
		retry--;
	} while (retry > 0);
	if (!retry) {
		ETHQOSERR("C_READY_STATUS timedout with retry = %d\n", retry);
		ret = -1;
		goto err_ret;
	}

	retry = 500;
	do {
		val = readl_relaxed(ethqos->sgmii_base + QSERDES_PCS_READY_STATUS);
		val &= BIT(7);
		if (val)
			break;
		usleep_range(1000, 1500);
		retry--;
	} while (retry > 0);
	if (!retry) {
		ETHQOSERR("PCS_READY timedout with retry = %d\n", retry);
		ret = -1;
		goto err_ret;
	}

	retry = 500;
	do {
		val = readl_relaxed(ethqos->sgmii_base + QSERDES_PCS_READY_STATUS);
		val &= BIT(0);
		if (val)
			break;
		usleep_range(1000, 1500);
		retry--;
	} while (retry > 0);
	if (!retry) {
		ETHQOSERR("SGMIIPHY_READY timedout with retry = %d\n", retry);
		ret = -1;
		goto err_ret;
	}

	return;

err_ret:
	ETHQOSERR("Serdes soft reset failed\n");

}
EXPORT_SYMBOL(qcom_ethqos_serdes_soft_reset);

void qcom_ethqos_disable_serdes_clocks(struct qcom_ethqos *ethqos)
{
	if (ethqos->phyaux_clk)
		clk_disable_unprepare(ethqos->phyaux_clk);

	if (ethqos->sgmiref_clk)
		clk_disable_unprepare(ethqos->sgmiref_clk);
}
EXPORT_SYMBOL(qcom_ethqos_disable_serdes_clocks);

int qcom_ethqos_enable_serdes_clocks(struct qcom_ethqos *ethqos)
{
	int ret = 0;

	if (ethqos->phyaux_clk) {
		ret = clk_prepare_enable(ethqos->phyaux_clk);
		if (ret) {
			ETHQOSERR("Failed to enable phyaux clock: %d\n", ret);
			return ret;
		}
	}

	if (ethqos->sgmiref_clk) {
		ret = clk_prepare_enable(ethqos->sgmiref_clk);
		if (ret) {
			ETHQOSERR("Failed to enable SerDes reference clock: %d\n", ret);
			return ret;
		}
	}

	return ret;
}
EXPORT_SYMBOL(qcom_ethqos_enable_serdes_clocks);

static int qcom_ethqos_serdes_wait_ready(struct qcom_ethqos *ethqos)
{
	int retry = 500;
	unsigned int val;

	do {
		val = readl_relaxed(ethqos->sgmii_base + QSERDES_COM_CMN_STATUS);
		val &= BIT(1);
		if (val)
			break;
		usleep_range(1000, 1500);
		retry--;
	} while (retry > 0);
	if (!retry) {
		ETHQOSERR("PLL Lock Status timedout with retry = %d\n", retry);
		return -ETIMEDOUT;
	}

	retry = 500;
	do {
		val = readl_relaxed(ethqos->sgmii_base + QSERDES_COM_C_READY_STATUS);
		val &= BIT(0);
		if (val)
			break;
		usleep_range(1000, 1500);
		retry--;
	} while (retry > 0);
	if (!retry) {
		ETHQOSERR("C_READY_STATUS timedout with retry = %d\n", retry);
		return -ETIMEDOUT;
	}

	retry = 500;
	do {
		val = readl_relaxed(ethqos->sgmii_base + QSERDES_PCS_READY_STATUS);
		val &= BIT(7);
		if (val)
			break;
		usleep_range(1000, 1500);
		retry--;
	} while (retry > 0);
	if (!retry) {
		ETHQOSERR("PCS_READY timedout with retry = %d\n", retry);
		return -ETIMEDOUT;
	}

	retry = 500;
	do {
		val = readl_relaxed(ethqos->sgmii_base + QSERDES_PCS_READY_STATUS);
		val &= BIT(0);
		if (val)
			break;
		usleep_range(1000, 1500);
		retry--;
	} while (retry > 0);
	if (!retry) {
		ETHQOSERR("SGMIIPHY_READY timedout with retry = %d\n", retry);
		return -ETIMEDOUT;
	}

	return 0;
}

static int qcom_ethqos_serdes_prog_tbl(struct qcom_ethqos *ethqos,
				       enum serdes_mode_index mode)
{
	unsigned int i;

	for (i = 0; i < serdes_prog_table_size; i++) {
		if (serdes_prog_table[i].value[mode] == SKIP)
			continue;
		writel_relaxed(serdes_prog_table[i].value[mode],
			       ethqos->sgmii_base + serdes_prog_table[i].reg);
	}

	usleep_range(3000, 5000);
	return qcom_ethqos_serdes_wait_ready(ethqos);
}

static int qcom_ethqos_serdes3_sgmii_1Gb(struct qcom_ethqos *ethqos)
{
	int retry = 500;
	unsigned int val;
	int ret = 0;

	/****************MODULE: SGMII_PHY_SGMII_PCS**********************************/
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES3_PCS_SW_RESET);
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES3_PCS_POWER_DOWN_CONTROL);

	/***************** MODULE: QSERDES3_COM_SGMII_QMP_PLL*********/
	writel_relaxed(0x0F, ethqos->sgmii_base + QSERDES3_COM_PLL_IVCO);
	writel_relaxed(0x06, ethqos->sgmii_base + QSERDES3_COM_CP_CTRL_MODE0);
	writel_relaxed(0x16, ethqos->sgmii_base + QSERDES3_COM_PLL_RCTRL_MODE0);
	writel_relaxed(0x36, ethqos->sgmii_base + QSERDES3_COM_PLL_CCTRL_MODE0);
	writel_relaxed(0x1A, ethqos->sgmii_base + QSERDES3_COM_SYSCLK_EN_SEL);
	writel_relaxed(0x0A, ethqos->sgmii_base + QSERDES3_COM_LOCK_CMP1_MODE0);
	writel_relaxed(0x1A, ethqos->sgmii_base + QSERDES3_COM_LOCK_CMP2_MODE0);
	writel_relaxed(0x82, ethqos->sgmii_base + QSERDES3_COM_DEC_START_MODE0);
	writel_relaxed(0x55, ethqos->sgmii_base + QSERDES3_COM_DIV_FRAC_START1_MODE0);
	writel_relaxed(0x55, ethqos->sgmii_base + QSERDES3_COM_DIV_FRAC_START2_MODE0);
	writel_relaxed(0x03, ethqos->sgmii_base + QSERDES3_COM_DIV_FRAC_START3_MODE0);
	writel_relaxed(0x24, ethqos->sgmii_base + QSERDES3_COM_VCO_TUNE1_MODE0);

	writel_relaxed(0x02, ethqos->sgmii_base + QSERDES3_COM_VCO_TUNE2_MODE0);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES3_COM_VCO_TUNE_INITVAL2);
	writel_relaxed(0x04, ethqos->sgmii_base + QSERDES3_COM_HSCLK_SEL);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES3_COM_HSCLK_HS_SWITCH_SEL);
	writel_relaxed(0x0A, ethqos->sgmii_base + QSERDES3_COM_CORECLK_DIV_MODE0);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES3_COM_CORE_CLK_EN);
	writel_relaxed(0xB9, ethqos->sgmii_base + QSERDES3_COM_BIN_VCOCAL_CMP_CODE1_MODE0);
	writel_relaxed(0x1E, ethqos->sgmii_base + QSERDES3_COM_BIN_VCOCAL_CMP_CODE2_MODE0);
	writel_relaxed(0x11, ethqos->sgmii_base + QSERDES3_COM_BIN_VCOCAL_HSCLK_SEL);

	/******************MODULE: QSERDES3_TX0_SGMII_QMP_TX***********************/
	writel_relaxed(0x05, ethqos->sgmii_base + QSERDES3_TX_TX_BAND);
	writel_relaxed(0x0A, ethqos->sgmii_base + QSERDES3_TX_SLEW_CNTL);
	writel_relaxed(0x09, ethqos->sgmii_base + QSERDES3_TX_RES_CODE_LANE_OFFSET_TX);
	writel_relaxed(0x09, ethqos->sgmii_base + QSERDES3_TX_RES_CODE_LANE_OFFSET_RX);
	writel_relaxed(0x05, ethqos->sgmii_base + QSERDES3_TX_LANE_MODE_1);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES3_TX_LANE_MODE_3);
	writel_relaxed(0x12, ethqos->sgmii_base + QSERDES3_TX_RCV_DETECT_LVL_2);
	writel_relaxed(0x0C, ethqos->sgmii_base + QSERDES3_TX_TRAN_DRVR_EMP_EN);

	/*****************MODULE: QSERDES3_RX0_SGMII_QMP_RX*******************/
	writel_relaxed(0x0A, ethqos->sgmii_base + QSERDES3_RX_UCDR_FO_GAIN);
	writel_relaxed(0x06, ethqos->sgmii_base + QSERDES3_RX_UCDR_SO_GAIN);
	writel_relaxed(0x0A, ethqos->sgmii_base + QSERDES3_RX_UCDR_FASTLOCK_FO_GAIN);
	writel_relaxed(0x7F, ethqos->sgmii_base + QSERDES3_RX_UCDR_SO_SATURATION_AND_ENABLE);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES3_RX_UCDR_FASTLOCK_COUNT_LOW);
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES3_RX_UCDR_FASTLOCK_COUNT_HIGH);
	writel_relaxed(0x81, ethqos->sgmii_base + QSERDES3_RX_UCDR_PI_CONTROLS);
	writel_relaxed(0x80, ethqos->sgmii_base + QSERDES3_RX_UCDR_PI_CTRL2);
	writel_relaxed(0x04, ethqos->sgmii_base + QSERDES3_RX_RX_TERM_BW);
	writel_relaxed(0x08, ethqos->sgmii_base + QSERDES3_RX_VGA_CAL_CNTRL2);
	writel_relaxed(0x0F, ethqos->sgmii_base + QSERDES3_RX_GM_CAL);
	writel_relaxed(0x04, ethqos->sgmii_base + QSERDES3_RX_RX_EQU_ADAPTOR_CNTRL1);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES3_RX_RX_EQU_ADAPTOR_CNTRL2);
	writel_relaxed(0x4A, ethqos->sgmii_base + QSERDES3_RX_RX_EQU_ADAPTOR_CNTRL3);
	writel_relaxed(0x0A, ethqos->sgmii_base + QSERDES3_RX_RX_EQU_ADAPTOR_CNTRL4);
	writel_relaxed(0x80, ethqos->sgmii_base + QSERDES3_RX_RX_IDAC_TSETTLE_LOW);
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES3_RX_RX_IDAC_TSETTLE_HIGH);
	writel_relaxed(0x20, ethqos->sgmii_base + QSERDES3_RX_RX_IDAC_MEASURE_TIME);
	writel_relaxed(0x17, ethqos->sgmii_base + QSERDES3_RX_RX_EQ_OFFSET_ADAPTOR_CNTRL1);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES3_RX_RX_OFFSET_ADAPTOR_CNTRL2);
	writel_relaxed(0x0F, ethqos->sgmii_base + QSERDES3_RX_SIGDET_CNTRL);
	writel_relaxed(0x1E, ethqos->sgmii_base + QSERDES3_RX_SIGDET_DEGLITCH_CNTRL);
	writel_relaxed(0x05, ethqos->sgmii_base + QSERDES3_RX_RX_BAND);
	writel_relaxed(0xE0, ethqos->sgmii_base + QSERDES3_RX_RX_MODE_00_LOW);
	writel_relaxed(0xC8, ethqos->sgmii_base + QSERDES3_RX_RX_MODE_00_HIGH);
	writel_relaxed(0xC8, ethqos->sgmii_base + QSERDES3_RX_RX_MODE_00_HIGH2);
	writel_relaxed(0x09, ethqos->sgmii_base + QSERDES3_RX_RX_MODE_00_HIGH3);
	writel_relaxed(0xB1, ethqos->sgmii_base + QSERDES3_RX_RX_MODE_00_HIGH4);
	writel_relaxed(0xE0, ethqos->sgmii_base + QSERDES3_RX_RX_MODE_01_LOW);
	writel_relaxed(0xC8, ethqos->sgmii_base + QSERDES3_RX_RX_MODE_01_HIGH);
	writel_relaxed(0xC8, ethqos->sgmii_base + QSERDES3_RX_RX_MODE_01_HIGH2);
	writel_relaxed(0x09, ethqos->sgmii_base + QSERDES3_RX_RX_MODE_01_HIGH3);
	writel_relaxed(0xB1, ethqos->sgmii_base + QSERDES3_RX_RX_MODE_01_HIGH4);
	writel_relaxed(0xE0, ethqos->sgmii_base + QSERDES3_RX_RX_MODE_10_LOW);
	writel_relaxed(0xC8, ethqos->sgmii_base + QSERDES3_RX_RX_MODE_10_HIGH);
	writel_relaxed(0xC8, ethqos->sgmii_base + QSERDES3_RX_RX_MODE_10_HIGH2);
	writel_relaxed(0x3B, ethqos->sgmii_base + QSERDES3_RX_RX_MODE_10_HIGH3);
	writel_relaxed(0xB7, ethqos->sgmii_base + QSERDES3_RX_RX_MODE_10_HIGH4);
	writel_relaxed(0x0C, ethqos->sgmii_base + QSERDES3_RX_DCC_CTRL1);

	/****************MODULE: SGMII_PHY_SGMII_PCS**********************************/
	writel_relaxed(0x0C, ethqos->sgmii_base + QSERDES3_PCS_LINE_RESET_TIME);
	writel_relaxed(0x1F, ethqos->sgmii_base + QSERDES3_PCS_TX_LARGE_AMP_DRV_LVL);
	writel_relaxed(0x03, ethqos->sgmii_base + QSERDES3_PCS_TX_SMALL_AMP_DRV_LVL);
	writel_relaxed(0x83, ethqos->sgmii_base + QSERDES3_PCS_TX_MID_TERM_CTRL1);
	writel_relaxed(0x08, ethqos->sgmii_base + QSERDES3_PCS_TX_MID_TERM_CTRL2);
	writel_relaxed(0x0C, ethqos->sgmii_base + QSERDES3_PCS_SGMII_MISC_CTRL8);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES3_PCS_SW_RESET);

	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES3_PCS_PHY_START);

	do {
		val = readl_relaxed(ethqos->sgmii_base + QSERDES3_COM_C_READY_STATUS);
		val &= QSERDES3_COM_C_READY;
		if (val)
			break;
		usleep_range(1000, 1500);
		retry--;
	} while (retry > 0);
	if (!retry) {
		ETHQOSERR("QSERDES3_COM_C_READY_STATUS timedout with retry = %d\n", retry);
		ret = -1;
		goto err_ret;
	}

	retry = 500;
	do {
		val = readl_relaxed(ethqos->sgmii_base + QSERDES3_PCS_PCS_READY_STATUS);
		val &= QSERDES3_PCS_READY;
		if (val)
			break;
		usleep_range(1000, 1500);
		retry--;
	} while (retry > 0);
	if (!retry) {
		ETHQOSERR("PCS_READY timedout with retry = %d\n", retry);
		ret = -1;
		goto err_ret;
	}

	retry = 500;
	do {
		val = readl_relaxed(ethqos->sgmii_base + QSERDES3_PCS_PCS_READY_STATUS);
		val &= QSERDES3_PCS_SGMIIPHY_READY;
		if (val)
			break;
		usleep_range(1000, 1500);
		retry--;
	} while (retry > 0);
	if (!retry) {
		ETHQOSERR("SGMIIPHY_READY timedout with retry = %d\n", retry);
		ret = -1;
		goto err_ret;
	}

	retry = 5000;
	do {
		val = readl_relaxed(ethqos->sgmii_base + QSERDES3_COM_CMN_STATUS);
		val &= QSERDES3_COM_C_PLL_LOCKED;
		if (val)
			break;
		usleep_range(1000, 1500);
		retry--;
	} while (retry > 0);
	if (!retry) {
		ETHQOSERR("PLL Lock Status timedout with retry = %d\n", retry);
		ret = -1;
		goto err_ret;
	}

err_ret:
		return ret;
}

static int qcom_ethqos_serdes3_sgmii_2_5Gb(struct qcom_ethqos *ethqos)
{
	int retry = 500;
	unsigned int val;
	int ret = 0;

	/****************MODULE: SGMII_PHY_SGMII_PCS**********************************/
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES3_PCS_SW_RESET);
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES3_PCS_POWER_DOWN_CONTROL);

	/***************** MODULE: QSERDES_COM_SGMII_QMP_PLL*********/
	writel_relaxed(0x0F, ethqos->sgmii_base + QSERDES3_COM_PLL_IVCO);
	writel_relaxed(0x06, ethqos->sgmii_base + QSERDES3_COM_CP_CTRL_MODE0);
	writel_relaxed(0x16, ethqos->sgmii_base + QSERDES3_COM_PLL_RCTRL_MODE0);
	writel_relaxed(0x36, ethqos->sgmii_base + QSERDES3_COM_PLL_CCTRL_MODE0);
	writel_relaxed(0x1A, ethqos->sgmii_base + QSERDES3_COM_SYSCLK_EN_SEL);
	writel_relaxed(0x1A, ethqos->sgmii_base + QSERDES3_COM_LOCK_CMP1_MODE0);
	writel_relaxed(0x41, ethqos->sgmii_base + QSERDES3_COM_LOCK_CMP2_MODE0);
	writel_relaxed(0x7A, ethqos->sgmii_base + QSERDES3_COM_DEC_START_MODE0);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES3_COM_DIV_FRAC_START1_MODE0);
	writel_relaxed(0x20, ethqos->sgmii_base + QSERDES3_COM_DIV_FRAC_START2_MODE0);
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES3_COM_DIV_FRAC_START3_MODE0);
	writel_relaxed(0xA1, ethqos->sgmii_base + QSERDES3_COM_VCO_TUNE1_MODE0);

	writel_relaxed(0x02, ethqos->sgmii_base + QSERDES3_COM_VCO_TUNE2_MODE0);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES3_COM_VCO_TUNE_INITVAL2);
	writel_relaxed(0x03, ethqos->sgmii_base + QSERDES3_COM_HSCLK_SEL);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES3_COM_HSCLK_HS_SWITCH_SEL);
	writel_relaxed(0x05, ethqos->sgmii_base + QSERDES3_COM_CORECLK_DIV_MODE0);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES3_COM_CORE_CLK_EN);
	writel_relaxed(0xCD, ethqos->sgmii_base + QSERDES3_COM_BIN_VCOCAL_CMP_CODE1_MODE0);
	writel_relaxed(0x1C, ethqos->sgmii_base + QSERDES3_COM_BIN_VCOCAL_CMP_CODE2_MODE0);
	writel_relaxed(0x11, ethqos->sgmii_base + QSERDES3_COM_BIN_VCOCAL_HSCLK_SEL);

	/******************MODULE: QSERDES_TX0_SGMII_QMP_TX***********************/
	writel_relaxed(0x04, ethqos->sgmii_base + QSERDES3_TX_TX_BAND);
	writel_relaxed(0x0A, ethqos->sgmii_base + QSERDES3_TX_SLEW_CNTL);
	writel_relaxed(0x09, ethqos->sgmii_base + QSERDES3_TX_RES_CODE_LANE_OFFSET_TX);
	writel_relaxed(0x02, ethqos->sgmii_base + QSERDES3_TX_RES_CODE_LANE_OFFSET_RX);
	writel_relaxed(0x05, ethqos->sgmii_base + QSERDES3_TX_LANE_MODE_1);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES3_TX_LANE_MODE_3);
	writel_relaxed(0x12, ethqos->sgmii_base + QSERDES3_TX_RCV_DETECT_LVL_2);
	writel_relaxed(0x0C, ethqos->sgmii_base + QSERDES3_TX_TRAN_DRVR_EMP_EN);

	/*****************MODULE: QSERDES_RX0_SGMII_QMP_RX*******************/
	writel_relaxed(0x0A, ethqos->sgmii_base + QSERDES3_RX_UCDR_FO_GAIN);
	writel_relaxed(0x06, ethqos->sgmii_base + QSERDES3_RX_UCDR_SO_GAIN);
	writel_relaxed(0x0A, ethqos->sgmii_base + QSERDES3_RX_UCDR_FASTLOCK_FO_GAIN);
	writel_relaxed(0x7F, ethqos->sgmii_base + QSERDES3_RX_UCDR_SO_SATURATION_AND_ENABLE);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES3_RX_UCDR_FASTLOCK_COUNT_LOW);
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES3_RX_UCDR_FASTLOCK_COUNT_HIGH);
	writel_relaxed(0x81, ethqos->sgmii_base + QSERDES3_RX_UCDR_PI_CONTROLS);
	writel_relaxed(0x80, ethqos->sgmii_base + QSERDES3_RX_UCDR_PI_CTRL2);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES3_RX_RX_TERM_BW);
	writel_relaxed(0x08, ethqos->sgmii_base + QSERDES3_RX_VGA_CAL_CNTRL2);
	writel_relaxed(0x0F, ethqos->sgmii_base + QSERDES3_RX_GM_CAL);
	writel_relaxed(0x04, ethqos->sgmii_base + QSERDES3_RX_RX_EQU_ADAPTOR_CNTRL1);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES3_RX_RX_EQU_ADAPTOR_CNTRL2);
	writel_relaxed(0x4A, ethqos->sgmii_base + QSERDES3_RX_RX_EQU_ADAPTOR_CNTRL3);
	writel_relaxed(0x0A, ethqos->sgmii_base + QSERDES3_RX_RX_EQU_ADAPTOR_CNTRL4);
	writel_relaxed(0x80, ethqos->sgmii_base + QSERDES3_RX_RX_IDAC_TSETTLE_LOW);
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES3_RX_RX_IDAC_TSETTLE_HIGH);
	writel_relaxed(0x20, ethqos->sgmii_base + QSERDES3_RX_RX_IDAC_MEASURE_TIME);
	writel_relaxed(0x17, ethqos->sgmii_base + QSERDES3_RX_RX_EQ_OFFSET_ADAPTOR_CNTRL1);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES3_RX_RX_OFFSET_ADAPTOR_CNTRL2);
	writel_relaxed(0x0F, ethqos->sgmii_base + QSERDES3_RX_SIGDET_CNTRL);
	writel_relaxed(0x1E, ethqos->sgmii_base + QSERDES3_RX_SIGDET_DEGLITCH_CNTRL);
	writel_relaxed(0x18, ethqos->sgmii_base + QSERDES3_RX_RX_BAND);
	writel_relaxed(0x18, ethqos->sgmii_base + QSERDES3_RX_RX_MODE_00_LOW);
	writel_relaxed(0xC8, ethqos->sgmii_base + QSERDES3_RX_RX_MODE_00_HIGH);
	writel_relaxed(0xC8, ethqos->sgmii_base + QSERDES3_RX_RX_MODE_00_HIGH2);
	writel_relaxed(0x0C, ethqos->sgmii_base + QSERDES3_RX_RX_MODE_00_HIGH3);
	writel_relaxed(0xB8, ethqos->sgmii_base + QSERDES3_RX_RX_MODE_00_HIGH4);
	writel_relaxed(0xE0, ethqos->sgmii_base + QSERDES3_RX_RX_MODE_01_LOW);
	writel_relaxed(0xC8, ethqos->sgmii_base + QSERDES3_RX_RX_MODE_01_HIGH);
	writel_relaxed(0xC8, ethqos->sgmii_base + QSERDES3_RX_RX_MODE_01_HIGH2);
	writel_relaxed(0x09, ethqos->sgmii_base + QSERDES3_RX_RX_MODE_01_HIGH3);
	writel_relaxed(0xB1, ethqos->sgmii_base + QSERDES3_RX_RX_MODE_01_HIGH4);
	writel_relaxed(0xE0, ethqos->sgmii_base + QSERDES3_RX_RX_MODE_10_LOW);
	writel_relaxed(0xC8, ethqos->sgmii_base + QSERDES3_RX_RX_MODE_10_HIGH);
	writel_relaxed(0xC8, ethqos->sgmii_base + QSERDES3_RX_RX_MODE_10_HIGH2);
	writel_relaxed(0x3B, ethqos->sgmii_base + QSERDES3_RX_RX_MODE_10_HIGH3);
	writel_relaxed(0xB7, ethqos->sgmii_base + QSERDES3_RX_RX_MODE_10_HIGH4);
	writel_relaxed(0x0C, ethqos->sgmii_base + QSERDES3_RX_DCC_CTRL1);

	/****************MODULE: SGMII_PHY_SGMII_PCS**********************************/
	writel_relaxed(0x0C, ethqos->sgmii_base + QSERDES3_PCS_LINE_RESET_TIME);
	writel_relaxed(0x1F, ethqos->sgmii_base + QSERDES3_PCS_TX_LARGE_AMP_DRV_LVL);
	writel_relaxed(0x03, ethqos->sgmii_base + QSERDES3_PCS_TX_SMALL_AMP_DRV_LVL);
	writel_relaxed(0x83, ethqos->sgmii_base + QSERDES3_PCS_TX_MID_TERM_CTRL1);
	writel_relaxed(0x08, ethqos->sgmii_base + QSERDES3_PCS_TX_MID_TERM_CTRL2);
	writel_relaxed(0x8C, ethqos->sgmii_base + QSERDES3_PCS_SGMII_MISC_CTRL8);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES3_PCS_SW_RESET);

	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES3_PCS_PHY_START);

	do {
		val = readl_relaxed(ethqos->sgmii_base + QSERDES3_COM_C_READY_STATUS);
		val &= QSERDES3_COM_C_READY;
		if (val)
			break;
		usleep_range(1000, 1500);
		retry--;
	} while (retry > 0);
	if (!retry) {
		ETHQOSERR("QSERDES3_COM_C_READY_STATUS timedout with retry = %d\n", retry);
		ret = -1;
		goto err_ret;
	}

	retry = 500;
	do {
		val = readl_relaxed(ethqos->sgmii_base + QSERDES3_PCS_PCS_READY_STATUS);
		val &= QSERDES3_PCS_READY;
		if (val)
			break;
		usleep_range(1000, 1500);
		retry--;
	} while (retry > 0);
	if (!retry) {
		ETHQOSERR("PCS_READY timedout with retry = %d\n", retry);
		ret = -1;
		goto err_ret;
	}

	retry = 500;
	do {
		val = readl_relaxed(ethqos->sgmii_base + QSERDES3_PCS_PCS_READY_STATUS);
		val &= QSERDES3_PCS_SGMIIPHY_READY;
		if (val)
			break;
		usleep_range(1000, 1500);
		retry--;
	} while (retry > 0);
	if (!retry) {
		ETHQOSERR("SGMIIPHY_READY timedout with retry = %d\n", retry);
		ret = -1;
		goto err_ret;
	}

	retry = 5000;
	do {
		val = readl_relaxed(ethqos->sgmii_base + QSERDES3_COM_CMN_STATUS);
		val &= QSERDES3_COM_C_PLL_LOCKED;
		if (val)
			break;
		usleep_range(1000, 1500);
		retry--;
	} while (retry > 0);
	if (!retry) {
		ETHQOSERR("PLL Lock Status timedout with retry = %d\n", retry);
		ret = -1;
		goto err_ret;
	}

err_ret:
		return ret;
}

static int qcom_ethqos_serdes_sgmii_1Gb(struct qcom_ethqos *ethqos)
{
	int ret = 0;
	int retry = 500;
	unsigned int val;

	if (ethqos->emac_ver == EMAC_HW_v3_1_0)
		return qcom_ethqos_serdes3_sgmii_1Gb(ethqos);

	/****************MODULE: SGMII_PHY_SGMII_PCS**********************************/
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES_PCS_SW_RESET);
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES_PCS_POWER_DOWN_CONTROL);

	/***************** MODULE: QSERDES_COM_SGMII_QMP_PLL*********/
	writel_relaxed(0x0E, ethqos->sgmii_base + QSERDES_COM_BG_TIMER);
	writel_relaxed(0x07, ethqos->sgmii_base + QSERDES_COM_BIAS_EN_CLKBUFLR_EN);
	writel_relaxed(0x0F, ethqos->sgmii_base + QSERDES_COM_CLK_ENABLE1);
	writel_relaxed(0x1F, ethqos->sgmii_base + QSERDES_COM_PLL_IVCO);
	writel_relaxed(0x08, ethqos->sgmii_base + QSERDES_COM_CP_CTRL_MODE0);
	writel_relaxed(0x16, ethqos->sgmii_base + QSERDES_COM_PLL_RCTRL_MODE0);
	writel_relaxed(0x36, ethqos->sgmii_base + QSERDES_COM_PLL_CCTRL_MODE0);
	writel_relaxed(0X1F, ethqos->sgmii_base + QSERDES_COM_INTEGLOOP_GAIN0_MODE0);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_COM_INTEGLOOP_GAIN1_MODE0);
	writel_relaxed(0x03, ethqos->sgmii_base + QSERDES_COM_PLL_EN);
	writel_relaxed(0x1A, ethqos->sgmii_base + QSERDES_COM_SYSCLK_EN_SEL);
	writel_relaxed(0x0A, ethqos->sgmii_base + QSERDES_COM_LOCK_CMP1_MODE0);
	writel_relaxed(0x1A, ethqos->sgmii_base + QSERDES_COM_LOCK_CMP2_MODE0);
	writel_relaxed(0x82, ethqos->sgmii_base + QSERDES_COM_DEC_START_MODE0);
	writel_relaxed(0x55, ethqos->sgmii_base + QSERDES_COM_DIV_FRAC_START1_MODE0);
	writel_relaxed(0x55, ethqos->sgmii_base + QSERDES_COM_DIV_FRAC_START2_MODE0);
	writel_relaxed(0x03, ethqos->sgmii_base + QSERDES_COM_DIV_FRAC_START3_MODE0);
	writel_relaxed(0x84, ethqos->sgmii_base + QSERDES_COM_VCO_TUNE1_MODE0);
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES_COM_VCO_TUNE2_MODE0);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_COM_VCO_TUNE_INITVAL2);
	writel_relaxed(0x04, ethqos->sgmii_base + QSERDES_COM_HSCLK_SEL_1);
	writel_relaxed(0x04, ethqos->sgmii_base + QSERDES_COM_HSCLK_HS_SWITCH_SEL_1);
	writel_relaxed(0x0A, ethqos->sgmii_base + QSERDES_COM_CORECLK_DIV_MODE0);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_COM_CORE_CLK_EN);
	writel_relaxed(0x16, ethqos->sgmii_base + QSERDES_COM_CMN_CONFIG_1);
	writel_relaxed(0xB9, ethqos->sgmii_base + QSERDES_COM_BIN_VCOCAL_CMP_CODE1_MODE0);
	writel_relaxed(0x1E, ethqos->sgmii_base + QSERDES_COM_BIN_VCOCAL_CMP_CODE2_MODE0);
	writel_relaxed(0x11, ethqos->sgmii_base + QSERDES_COM_BIN_VCOCAL_HSCLK_SEL_1);

	/******************MODULE: QSERDES_TX0_SGMII_QMP_TX***********************/
	writel_relaxed(0x0D, ethqos->sgmii_base + QSERDES_TX_CLKBUF_ENABLE);
	writel_relaxed(0x05, ethqos->sgmii_base + QSERDES_TX0_TX_BAND);
	writel_relaxed(0x0A, ethqos->sgmii_base + QSERDES_TX0_SLEW_CNTL);
	writel_relaxed(0x09, ethqos->sgmii_base + QSERDES_TX0_RES_CODE_LANE_OFFSET_TX);
	writel_relaxed(0x09, ethqos->sgmii_base + QSERDES_TX0_RES_CODE_LANE_OFFSET_RX);
	writel_relaxed(0xF5, ethqos->sgmii_base + QSERDES_TX0_LANE_MODE_1);
	writel_relaxed(0x02, ethqos->sgmii_base + QSERDES_TX0_LANE_MODE_2);
	writel_relaxed(0x3F, ethqos->sgmii_base + QSERDES_TX0_LANE_MODE_3);
	writel_relaxed(0x3F, ethqos->sgmii_base + QSERDES_TX_LANE_MODE_4);
	writel_relaxed(0x5F, ethqos->sgmii_base + QSERDES_TX_LANE_MODE_5);
	writel_relaxed(0x12, ethqos->sgmii_base + QSERDES_TX0_RCV_DETECT_LVL_2);
	writel_relaxed(0x0F, ethqos->sgmii_base + QSERDES_TX0_TRAN_DRVR_EMP_EN);
	writel_relaxed(0x2B, ethqos->sgmii_base + QSERDES_TX_TX_EMP_POST1_LVL);

	/*****************MODULE: QSERDES_RX0_SGMII_QMP_RX*******************/
	writel_relaxed(0x0D, ethqos->sgmii_base + QSERDES_RX0_UCDR_FO_GAIN);
	writel_relaxed(0x03, ethqos->sgmii_base + QSERDES_RX0_UCDR_SO_GAIN);
	writel_relaxed(0x0A, ethqos->sgmii_base + QSERDES_RX0_UCDR_FASTLOCK_FO_GAIN);
	writel_relaxed(0x7F, ethqos->sgmii_base + QSERDES_RX0_UCDR_SO_SATURATION_AND_ENABLE);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_RX0_UCDR_FASTLOCK_COUNT_LOW);
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES_RX0_UCDR_FASTLOCK_COUNT_HIGH);
	writel_relaxed(0x81, ethqos->sgmii_base + QSERDES_RX0_UCDR_PI_CONTROLS);
	writel_relaxed(0x81, ethqos->sgmii_base + QSERDES_RX0_UCDR_PI_CTRL2);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_RX_UCDR_SB2_THRESH1);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_RX_UCDR_SB2_THRESH2);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_RX0_RX_TERM_BW);
	writel_relaxed(0x08, ethqos->sgmii_base + QSERDES_RX0_VGA_CAL_CNTRL2);
	writel_relaxed(0x0F, ethqos->sgmii_base + QSERDES_RX0_GM_CAL);
	writel_relaxed(0x04, ethqos->sgmii_base + QSERDES_RX0_RX_EQU_ADAPTOR_CNTRL1);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_RX0_RX_EQU_ADAPTOR_CNTRL2);
	writel_relaxed(0x4A, ethqos->sgmii_base + QSERDES_RX0_RX_EQU_ADAPTOR_CNTRL3);
	writel_relaxed(0x5A, ethqos->sgmii_base + QSERDES_RX0_RX_EQU_ADAPTOR_CNTRL4);
	writel_relaxed(0x80, ethqos->sgmii_base + QSERDES_RX0_RX_IDAC_TSETTLE_LOW);
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES_RX0_RX_IDAC_TSETTLE_HIGH);
	writel_relaxed(0x20, ethqos->sgmii_base + QSERDES_RX0_RX_IDAC_MEASURE_TIME);
	writel_relaxed(0x17, ethqos->sgmii_base + QSERDES_RX0_RX_EQ_OFFSET_ADAPTOR_CNTRL1);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_RX0_RX_OFFSET_ADAPTOR_CNTRL2);
	writel_relaxed(0x0F, ethqos->sgmii_base + QSERDES_RX0_SIGDET_CNTRL);
	writel_relaxed(0x1E, ethqos->sgmii_base + QSERDES_RX0_SIGDET_DEGLITCH_CNTRL);
	writel_relaxed(0x05, ethqos->sgmii_base + QSERDES_RX0_RX_BAND);
	writel_relaxed(0x18, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_00_LOW);
	writel_relaxed(0xC8, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_00_HIGH);
	writel_relaxed(0xC8, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_00_HIGH2);
	writel_relaxed(0x0C, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_00_HIGH3);
	writel_relaxed(0xB8, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_00_HIGH4);
	writel_relaxed(0xE0, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_01_LOW);
	writel_relaxed(0xC8, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_01_HIGH);
	writel_relaxed(0xC8, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_01_HIGH2);
	writel_relaxed(0x09, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_01_HIGH3);
	writel_relaxed(0xB1, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_01_HIGH4);
	writel_relaxed(0x0C, ethqos->sgmii_base + QSERDES_RX0_DCC_CTRL1);
	writel_relaxed(0x0C, ethqos->sgmii_base + QSERDES_PCS_LINE_RESET_TIME);

	/****************MODULE: SGMII_PHY_SGMII_PCS**********************************/
	writel_relaxed(0x03, ethqos->sgmii_base + QSERDES_PCS_TX_SMALL_AMP_DRV_LVL);
	writel_relaxed(0x1A, ethqos->sgmii_base + QSERDES_PCS_TX_LARGE_AMP_DRV_LVL);
	writel_relaxed(0x0B, ethqos->sgmii_base + QSERDES_PCS_TX_LARGE_AMP_POST_EMP_LVL);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_PCS_TX_SMALL_AMP_POST_EMP_LVL);
	writel_relaxed(0x83, ethqos->sgmii_base + QSERDES_PCS_TX_MID_TERM_CTRL1);
	writel_relaxed(0x08, ethqos->sgmii_base + QSERDES_PCS_TX_MID_TERM_CTRL2);
	writel_relaxed(0x10, ethqos->sgmii_base + QSERDES_RX0_SIGDET_CAL_CTRL1);
	writel_relaxed(0x77, ethqos->sgmii_base + QSERDES_PCS_RX_SIGDET_CTRL2);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_RX0_SIGDET_CAL_CTRL2_AND_CDR_LOCK_EDGE);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_PCS_SGMII_MISC_CTRL7);
	writel_relaxed(0x0C, ethqos->sgmii_base + QSERDES_PCS_SGMII_MISC_CTRL8);
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES_PCS_RETIME_BUFFER_EN);

	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_PCS_SW_RESET);
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES_PCS_PHY_START);
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES_PCS_PHY_START);
	usleep_range(3000, 5000);

	do {
		val = readl_relaxed(ethqos->sgmii_base + QSERDES_COM_CMN_STATUS);
		val &= BIT(1);
		if (val)
			break;
		usleep_range(1000, 1500);
		retry--;
	} while (retry > 0);
	if (!retry) {
		ETHQOSERR(" %s : PLL Lock Status timedout with retry = %d\n",
			  __func__,
			  retry);
		ret = -1;
		goto err_ret;
	}

	retry = 500;
	do {
		val = readl_relaxed(ethqos->sgmii_base + QSERDES_COM_C_READY_STATUS);
		val &= BIT(0);
		if (val)
			break;
		usleep_range(1000, 1500);
		retry--;
	} while (retry > 0);
	if (!retry) {
		ETHQOSERR("%s : C_READY_STATUS timedout with retry = %d\n",
			  __func__,
			  retry);
		ret = -1;
		goto err_ret;
	}

	retry = 500;
	do {
		val = readl_relaxed(ethqos->sgmii_base + QSERDES_PCS_READY_STATUS);
		val &= BIT(7);
		if (val)
			break;
		usleep_range(1000, 1500);
		retry--;
	} while (retry > 0);
	if (!retry) {
		ETHQOSERR("%s : PCS_READY timedout with retry = %d\n",
			  __func__,
			  retry);
		ret = -1;
		goto err_ret;
	}

	retry = 500;
	do {
		val = readl_relaxed(ethqos->sgmii_base + QSERDES_PCS_READY_STATUS);
		val &= BIT(0);
		if (val)
			break;
		usleep_range(1000, 1500);
		retry--;
	} while (retry > 0);
	if (!retry) {
		ETHQOSERR("%s : SGMIIPHY_READY timedout with retry = %d\n",
			  __func__,
			  retry);
		ret = -1;
		goto err_ret;
	}

	return ret;

err_ret:
	return ret;
}

static int qcom_ethqos_serdes_sgmii_2p5Gb(struct qcom_ethqos *ethqos)
{
	int ret = 0;
	int retry = 500;
	unsigned int val;

	if (ethqos->emac_ver == EMAC_HW_v3_1_0)
		return qcom_ethqos_serdes3_sgmii_2_5Gb(ethqos);

	/****************MODULE: SGMII_PHY_SGMII_PCS**********************************/
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES_PCS_SW_RESET);
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES_PCS_POWER_DOWN_CONTROL);

	/***************** MODULE: QSERDES_COM_SGMII_QMP_PLL*********/
	writel_relaxed(0x0E, ethqos->sgmii_base + QSERDES_COM_BG_TIMER);
	writel_relaxed(0x07, ethqos->sgmii_base + QSERDES_COM_BIAS_EN_CLKBUFLR_EN);
	writel_relaxed(0x0F, ethqos->sgmii_base + QSERDES_COM_CLK_ENABLE1);
	writel_relaxed(0x1F, ethqos->sgmii_base + QSERDES_COM_PLL_IVCO);
	writel_relaxed(0x08, ethqos->sgmii_base + QSERDES_COM_CP_CTRL_MODE0);
	writel_relaxed(0x16, ethqos->sgmii_base + QSERDES_COM_PLL_RCTRL_MODE0);
	writel_relaxed(0x36, ethqos->sgmii_base + QSERDES_COM_PLL_CCTRL_MODE0);
	writel_relaxed(0X1F, ethqos->sgmii_base + QSERDES_COM_INTEGLOOP_GAIN0_MODE0);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_COM_INTEGLOOP_GAIN1_MODE0);
	writel_relaxed(0x03, ethqos->sgmii_base + QSERDES_COM_PLL_EN);
	writel_relaxed(0x1A, ethqos->sgmii_base + QSERDES_COM_SYSCLK_EN_SEL);
	writel_relaxed(0x1A, ethqos->sgmii_base + QSERDES_COM_LOCK_CMP1_MODE0);
	writel_relaxed(0x41, ethqos->sgmii_base + QSERDES_COM_LOCK_CMP2_MODE0);
	writel_relaxed(0x7A, ethqos->sgmii_base + QSERDES_COM_DEC_START_MODE0);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_COM_DIV_FRAC_START1_MODE0);
	writel_relaxed(0x20, ethqos->sgmii_base + QSERDES_COM_DIV_FRAC_START2_MODE0);
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES_COM_DIV_FRAC_START3_MODE0);
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES_COM_VCO_TUNE1_MODE0);
	writel_relaxed(0x02, ethqos->sgmii_base + QSERDES_COM_VCO_TUNE2_MODE0);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_COM_VCO_TUNE_INITVAL2);
	writel_relaxed(0x03, ethqos->sgmii_base + QSERDES_COM_HSCLK_SEL_1);
	writel_relaxed(0x03, ethqos->sgmii_base + QSERDES_COM_HSCLK_HS_SWITCH_SEL_1);
	writel_relaxed(0x05, ethqos->sgmii_base + QSERDES_COM_CORECLK_DIV_MODE0);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_COM_CORE_CLK_EN);
	writel_relaxed(0x16, ethqos->sgmii_base + QSERDES_COM_CMN_CONFIG_1);
	writel_relaxed(0xCD, ethqos->sgmii_base + QSERDES_COM_BIN_VCOCAL_CMP_CODE1_MODE0);
	writel_relaxed(0x1C, ethqos->sgmii_base + QSERDES_COM_BIN_VCOCAL_CMP_CODE2_MODE0);
	writel_relaxed(0x11, ethqos->sgmii_base + QSERDES_COM_BIN_VCOCAL_HSCLK_SEL_1);

	/******************MODULE: QSERDES_TX0_SGMII_QMP_TX***********************/
	writel_relaxed(0x0D, ethqos->sgmii_base + QSERDES_TX_CLKBUF_ENABLE);
	writel_relaxed(0x04, ethqos->sgmii_base + QSERDES_TX0_TX_BAND);
	writel_relaxed(0x0A, ethqos->sgmii_base + QSERDES_TX0_SLEW_CNTL);
	writel_relaxed(0x09, ethqos->sgmii_base + QSERDES_TX0_RES_CODE_LANE_OFFSET_TX);
	writel_relaxed(0x09, ethqos->sgmii_base + QSERDES_TX0_RES_CODE_LANE_OFFSET_RX);
	writel_relaxed(0xF5, ethqos->sgmii_base + QSERDES_TX0_LANE_MODE_1);
	writel_relaxed(0x02, ethqos->sgmii_base + QSERDES_TX0_LANE_MODE_2);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_TX0_LANE_MODE_3);
	writel_relaxed(0x3F, ethqos->sgmii_base + QSERDES_TX_LANE_MODE_4);
	writel_relaxed(0x5F, ethqos->sgmii_base + QSERDES_TX_LANE_MODE_5);
	writel_relaxed(0x12, ethqos->sgmii_base + QSERDES_TX0_RCV_DETECT_LVL_2);
	writel_relaxed(0x0F, ethqos->sgmii_base + QSERDES_TX0_TRAN_DRVR_EMP_EN);
	writel_relaxed(0x2B, ethqos->sgmii_base + QSERDES_TX_TX_EMP_POST1_LVL);

	/*****************MODULE: QSERDES_RX0_SGMII_QMP_RX*******************/
	writel_relaxed(0x0D, ethqos->sgmii_base + QSERDES_RX0_UCDR_FO_GAIN);
	writel_relaxed(0x03, ethqos->sgmii_base + QSERDES_RX0_UCDR_SO_GAIN);
	writel_relaxed(0x0A, ethqos->sgmii_base + QSERDES_RX0_UCDR_FASTLOCK_FO_GAIN);
	writel_relaxed(0x7F, ethqos->sgmii_base + QSERDES_RX0_UCDR_SO_SATURATION_AND_ENABLE);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_RX0_UCDR_FASTLOCK_COUNT_LOW);
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES_RX0_UCDR_FASTLOCK_COUNT_HIGH);
	writel_relaxed(0x81, ethqos->sgmii_base + QSERDES_RX0_UCDR_PI_CONTROLS);
	writel_relaxed(0x81, ethqos->sgmii_base + QSERDES_RX0_UCDR_PI_CTRL2);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_RX_UCDR_SB2_THRESH1);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_RX_UCDR_SB2_THRESH2);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_RX0_RX_TERM_BW);
	writel_relaxed(0x08, ethqos->sgmii_base + QSERDES_RX0_VGA_CAL_CNTRL2);
	writel_relaxed(0x0F, ethqos->sgmii_base + QSERDES_RX0_GM_CAL);
	writel_relaxed(0x04, ethqos->sgmii_base + QSERDES_RX0_RX_EQU_ADAPTOR_CNTRL1);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_RX0_RX_EQU_ADAPTOR_CNTRL2);
	writel_relaxed(0x4A, ethqos->sgmii_base + QSERDES_RX0_RX_EQU_ADAPTOR_CNTRL3);
	writel_relaxed(0x5A, ethqos->sgmii_base + QSERDES_RX0_RX_EQU_ADAPTOR_CNTRL4);
	writel_relaxed(0x80, ethqos->sgmii_base + QSERDES_RX0_RX_IDAC_TSETTLE_LOW);
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES_RX0_RX_IDAC_TSETTLE_HIGH);
	writel_relaxed(0x20, ethqos->sgmii_base + QSERDES_RX0_RX_IDAC_MEASURE_TIME);
	writel_relaxed(0x17, ethqos->sgmii_base + QSERDES_RX0_RX_EQ_OFFSET_ADAPTOR_CNTRL1);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_RX0_RX_OFFSET_ADAPTOR_CNTRL2);
	writel_relaxed(0x0F, ethqos->sgmii_base + QSERDES_RX0_SIGDET_CNTRL);
	writel_relaxed(0x1E, ethqos->sgmii_base + QSERDES_RX0_SIGDET_DEGLITCH_CNTRL);
	writel_relaxed(0x18, ethqos->sgmii_base + QSERDES_RX0_RX_BAND);
	writel_relaxed(0x18, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_00_LOW);
	writel_relaxed(0xC8, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_00_HIGH);
	writel_relaxed(0xC8, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_00_HIGH2);
	writel_relaxed(0x0C, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_00_HIGH3);
	writel_relaxed(0xB8, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_00_HIGH4);
	writel_relaxed(0xE0, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_01_LOW);
	writel_relaxed(0xC8, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_01_HIGH);
	writel_relaxed(0xC8, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_01_HIGH2);
	writel_relaxed(0x09, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_01_HIGH3);
	writel_relaxed(0xB1, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_01_HIGH4);
	writel_relaxed(0x0C, ethqos->sgmii_base + QSERDES_RX0_DCC_CTRL1);
	writel_relaxed(0x0C, ethqos->sgmii_base + QSERDES_PCS_LINE_RESET_TIME);

	/****************MODULE: SGMII_PHY_SGMII_PCS**********************************/
	writel_relaxed(0x03, ethqos->sgmii_base + QSERDES_PCS_TX_SMALL_AMP_DRV_LVL);
	writel_relaxed(0x1A, ethqos->sgmii_base + QSERDES_PCS_TX_LARGE_AMP_DRV_LVL);
	writel_relaxed(0x0B, ethqos->sgmii_base + QSERDES_PCS_TX_LARGE_AMP_POST_EMP_LVL);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_PCS_TX_SMALL_AMP_POST_EMP_LVL);
	writel_relaxed(0x83, ethqos->sgmii_base + QSERDES_PCS_TX_MID_TERM_CTRL1);
	writel_relaxed(0x08, ethqos->sgmii_base + QSERDES_PCS_TX_MID_TERM_CTRL2);

	writel_relaxed(0x10, ethqos->sgmii_base + QSERDES_RX0_SIGDET_CAL_CTRL1);
	writel_relaxed(0x77, ethqos->sgmii_base + QSERDES_PCS_RX_SIGDET_CTRL2);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_RX0_SIGDET_CAL_CTRL2_AND_CDR_LOCK_EDGE);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_PCS_SGMII_MISC_CTRL7);
	writel_relaxed(0x8c, ethqos->sgmii_base + QSERDES_PCS_SGMII_MISC_CTRL8);
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES_PCS_RETIME_BUFFER_EN);

	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_PCS_SW_RESET);
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES_PCS_PHY_START);
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES_PCS_PHY_START);
	usleep_range(3000, 5000);
	do {
		val = readl_relaxed(ethqos->sgmii_base + QSERDES_COM_CMN_STATUS);
		val &= BIT(1);
		if (val)
			break;
		usleep_range(1000, 1500);
		retry--;
	} while (retry > 0);
	if (!retry) {
		ETHQOSERR("%s : PLL Lock Status timedout with retry = %d\n",
			  __func__,
			  retry);
		ret = -1;
		goto err_ret;
	}

	retry = 500;
	do {
		val = readl_relaxed(ethqos->sgmii_base + QSERDES_COM_C_READY_STATUS);
		val &= BIT(0);
		if (val)
			break;
		usleep_range(1000, 1500);
		retry--;
	} while (retry > 0);
	if (!retry) {
		ETHQOSERR("%s : C_READY_STATUS timedout with retry = %d\n",
			  __func__,
			  retry);
		ret = -1;
		goto err_ret;
	}

	retry = 500;
	do {
		val = readl_relaxed(ethqos->sgmii_base + QSERDES_PCS_READY_STATUS);
		val &= BIT(7);
		if (val)
			break;
		usleep_range(1000, 1500);
		retry--;
	} while (retry > 0);
	if (!retry) {
		ETHQOSERR("%s : PCS_READY timedout with retry = %d\n",
			  __func__,
			  retry);
		ret = -1;
		goto err_ret;
	}

	retry = 500;
	do {
		val = readl_relaxed(ethqos->sgmii_base + QSERDES_PCS_READY_STATUS);
		val &= BIT(0);
		if (val)
			break;
		usleep_range(1000, 1500);
		retry--;
	} while (retry > 0);
	if (!retry) {
		ETHQOSERR("%s : SGMIIPHY_READY timedout with retry = %d\n",
			  __func__,
			  retry);
		ret = -1;
		goto err_ret;
	}

	return ret;

err_ret:
	return ret;
}

static int qcom_ethqos_serdes_usxgmii_2p5Gb(struct qcom_ethqos *ethqos)
{
	int ret = 0;
	int retry = 500;
	unsigned int val;

	/****************MODULE: SGMII_PHY_SGMII_PCS**********************************/
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES_PCS_SW_RESET);
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES_PCS_POWER_DOWN_CONTROL);

	/***************** MODULE: QSERDES_COM_SGMII_QMP_PLL*********/
	writel_relaxed(0x0E, ethqos->sgmii_base + QSERDES_COM_BG_TIMER);
	writel_relaxed(0x07, ethqos->sgmii_base + QSERDES_COM_BIAS_EN_CLKBUFLR_EN);
	writel_relaxed(0x0F, ethqos->sgmii_base + QSERDES_COM_CLK_ENABLE1);
	writel_relaxed(0x1F, ethqos->sgmii_base + QSERDES_COM_PLL_IVCO);
	writel_relaxed(0x08, ethqos->sgmii_base + QSERDES_COM_CP_CTRL_MODE0);
	writel_relaxed(0x16, ethqos->sgmii_base + QSERDES_COM_PLL_RCTRL_MODE0);
	writel_relaxed(0x36, ethqos->sgmii_base + QSERDES_COM_PLL_CCTRL_MODE0);
	writel_relaxed(0X1F, ethqos->sgmii_base + QSERDES_COM_INTEGLOOP_GAIN0_MODE0);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_COM_INTEGLOOP_GAIN1_MODE0);
	writel_relaxed(0x03, ethqos->sgmii_base + QSERDES_COM_PLL_EN);
	writel_relaxed(0x1A, ethqos->sgmii_base + QSERDES_COM_SYSCLK_EN_SEL);
	writel_relaxed(0x91, ethqos->sgmii_base + QSERDES_COM_LOCK_CMP1_MODE0);
	writel_relaxed(0x21, ethqos->sgmii_base + QSERDES_COM_LOCK_CMP2_MODE0);
	writel_relaxed(0x86, ethqos->sgmii_base + QSERDES_COM_DEC_START_MODE0);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_COM_DIV_FRAC_START1_MODE0);
	writel_relaxed(0x70, ethqos->sgmii_base + QSERDES_COM_DIV_FRAC_START2_MODE0);
	writel_relaxed(0x04, ethqos->sgmii_base + QSERDES_COM_DIV_FRAC_START3_MODE0);
	writel_relaxed(0x46, ethqos->sgmii_base + QSERDES_COM_VCO_TUNE1_MODE0);
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES_COM_VCO_TUNE2_MODE0);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_COM_VCO_TUNE_INITVAL2);
	writel_relaxed(0x04, ethqos->sgmii_base + QSERDES_COM_HSCLK_SEL_1);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_COM_HSCLK_HS_SWITCH_SEL_1);
	writel_relaxed(0x04, ethqos->sgmii_base + QSERDES_COM_CORECLK_DIV_MODE0);
	writel_relaxed(0x10, ethqos->sgmii_base + QSERDES_COM_CORE_CLK_EN);
	writel_relaxed(0x16, ethqos->sgmii_base + QSERDES_COM_CMN_CONFIG_1);
	writel_relaxed(0xAE, ethqos->sgmii_base + QSERDES_COM_BIN_VCOCAL_CMP_CODE1_MODE0);
	writel_relaxed(0x1F, ethqos->sgmii_base + QSERDES_COM_BIN_VCOCAL_CMP_CODE2_MODE0);
	writel_relaxed(0x11, ethqos->sgmii_base + QSERDES_COM_BIN_VCOCAL_HSCLK_SEL_1);

	/******************MODULE: QSERDES_TX0_SGMII_QMP_TX***********************/
	writel_relaxed(0x0D, ethqos->sgmii_base + QSERDES_TX_CLKBUF_ENABLE);
	writel_relaxed(0x04, ethqos->sgmii_base + QSERDES_TX0_TX_BAND);
	writel_relaxed(0x08, ethqos->sgmii_base + QSERDES_TX0_SLEW_CNTL);
	writel_relaxed(0x09, ethqos->sgmii_base + QSERDES_TX0_RES_CODE_LANE_OFFSET_TX);
	writel_relaxed(0x09, ethqos->sgmii_base + QSERDES_TX0_RES_CODE_LANE_OFFSET_RX);
	writel_relaxed(0xF5, ethqos->sgmii_base + QSERDES_TX0_LANE_MODE_1);
	writel_relaxed(0x06, ethqos->sgmii_base + QSERDES_TX0_LANE_MODE_2);
	writel_relaxed(0x3F, ethqos->sgmii_base + QSERDES_TX0_LANE_MODE_3);
	writel_relaxed(0x3F, ethqos->sgmii_base + QSERDES_TX_LANE_MODE_4);
	writel_relaxed(0x5F, ethqos->sgmii_base + QSERDES_TX_LANE_MODE_5);
	writel_relaxed(0x12, ethqos->sgmii_base + QSERDES_TX0_RCV_DETECT_LVL_2);
	writel_relaxed(0x0F, ethqos->sgmii_base + QSERDES_TX0_TRAN_DRVR_EMP_EN);
	writel_relaxed(0x2B, ethqos->sgmii_base + QSERDES_TX_TX_EMP_POST1_LVL);

	/*****************MODULE: QSERDES_RX0_SGMII_QMP_RX*******************/
	writel_relaxed(0x0D, ethqos->sgmii_base + QSERDES_RX0_UCDR_FO_GAIN);
	writel_relaxed(0x03, ethqos->sgmii_base + QSERDES_RX0_UCDR_SO_GAIN);
	writel_relaxed(0x0A, ethqos->sgmii_base + QSERDES_RX0_UCDR_FASTLOCK_FO_GAIN);
	writel_relaxed(0x7F, ethqos->sgmii_base + QSERDES_RX0_UCDR_SO_SATURATION_AND_ENABLE);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_RX0_UCDR_FASTLOCK_COUNT_LOW);
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES_RX0_UCDR_FASTLOCK_COUNT_HIGH);
	writel_relaxed(0x81, ethqos->sgmii_base + QSERDES_RX0_UCDR_PI_CONTROLS);
	writel_relaxed(0x80, ethqos->sgmii_base + QSERDES_RX0_UCDR_PI_CTRL2);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_RX_UCDR_SB2_THRESH1);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_RX_UCDR_SB2_THRESH2);
	writel_relaxed(0x03, ethqos->sgmii_base + QSERDES_RX0_RX_TERM_BW);
	writel_relaxed(0x08, ethqos->sgmii_base + QSERDES_RX0_VGA_CAL_CNTRL2);
	writel_relaxed(0x0F, ethqos->sgmii_base + QSERDES_RX0_GM_CAL);
	writel_relaxed(0x04, ethqos->sgmii_base + QSERDES_RX0_RX_EQU_ADAPTOR_CNTRL1);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_RX0_RX_EQU_ADAPTOR_CNTRL2);
	writel_relaxed(0x4A, ethqos->sgmii_base + QSERDES_RX0_RX_EQU_ADAPTOR_CNTRL3);
	writel_relaxed(0x5A, ethqos->sgmii_base + QSERDES_RX0_RX_EQU_ADAPTOR_CNTRL4);
	writel_relaxed(0x80, ethqos->sgmii_base + QSERDES_RX0_RX_IDAC_TSETTLE_LOW);
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES_RX0_RX_IDAC_TSETTLE_HIGH);
	writel_relaxed(0x20, ethqos->sgmii_base + QSERDES_RX0_RX_IDAC_MEASURE_TIME);
	writel_relaxed(0x17, ethqos->sgmii_base + QSERDES_RX0_RX_EQ_OFFSET_ADAPTOR_CNTRL1);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_RX0_RX_OFFSET_ADAPTOR_CNTRL2);
	writel_relaxed(0x0F, ethqos->sgmii_base + QSERDES_RX0_SIGDET_CNTRL);
	writel_relaxed(0x1E, ethqos->sgmii_base + QSERDES_RX0_SIGDET_DEGLITCH_CNTRL);
	writel_relaxed(0x18, ethqos->sgmii_base + QSERDES_RX0_RX_BAND);
	writel_relaxed(0x18, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_00_LOW);
	writel_relaxed(0xC8, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_00_HIGH);
	writel_relaxed(0xC8, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_00_HIGH2);
	writel_relaxed(0x0C, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_00_HIGH3);
	writel_relaxed(0xB8, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_00_HIGH4);
	writel_relaxed(0xE0, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_01_LOW);
	writel_relaxed(0xC8, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_01_HIGH);
	writel_relaxed(0xC8, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_01_HIGH2);
	writel_relaxed(0x09, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_01_HIGH3);
	writel_relaxed(0xB1, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_01_HIGH4);
	writel_relaxed(0x0C, ethqos->sgmii_base + QSERDES_RX0_DCC_CTRL1);

	/****************MODULE: SGMII_PHY_SGMII_PCS**********************************/
	writel_relaxed(0x03, ethqos->sgmii_base + QSERDES_PCS_TX_SMALL_AMP_DRV_LVL);
	writel_relaxed(0x1A, ethqos->sgmii_base + QSERDES_PCS_TX_LARGE_AMP_DRV_LVL);
	writel_relaxed(0x0B, ethqos->sgmii_base + QSERDES_PCS_TX_LARGE_AMP_POST_EMP_LVL);
	writel_relaxed(0x2F, ethqos->sgmii_base + QSERDES_PCS_TX_SMALL_AMP_POST_EMP_LVL);
	writel_relaxed(0x83, ethqos->sgmii_base + QSERDES_PCS_TX_MID_TERM_CTRL1);
	writel_relaxed(0x08, ethqos->sgmii_base + QSERDES_PCS_TX_MID_TERM_CTRL2);

	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_RX0_SIGDET_CAL_CTRL1);
	writel_relaxed(0xCC, ethqos->sgmii_base + QSERDES_PCS_RX_SIGDET_CTRL2);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_RX0_SIGDET_CAL_CTRL2_AND_CDR_LOCK_EDGE);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_PCS_SGMII_MISC_CTRL7);
	writel_relaxed(0x14, ethqos->sgmii_base + QSERDES_PCS_SGMII_MISC_CTRL8);
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES_PCS_RETIME_BUFFER_EN);

	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_PCS_SW_RESET);
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES_PCS_PHY_START);
	usleep_range(3000, 5000);

	do {
		val = readl_relaxed(ethqos->sgmii_base + QSERDES_COM_CMN_STATUS);
		val &= BIT(1);
		if (val)
			break;
		usleep_range(1000, 1500);
		retry--;
	} while (retry > 0);
	if (!retry) {
		ETHQOSERR("PLL Lock Status timedout with retry = %d\n", retry);
		ret = -1;
		goto err_ret;
	}

	retry = 500;
	do {
		val = readl_relaxed(ethqos->sgmii_base + QSERDES_COM_C_READY_STATUS);
		val &= BIT(0);
		if (val)
			break;
		usleep_range(1000, 1500);
		retry--;
	} while (retry > 0);
	if (!retry) {
		ETHQOSERR("C_READY_STATUS timedout with retry = %d\n", retry);
		ret = -1;
		goto err_ret;
	}

	retry = 500;
	do {
		val = readl_relaxed(ethqos->sgmii_base + QSERDES_PCS_READY_STATUS);
		val &= BIT(7);
		if (val)
			break;
		usleep_range(1000, 1500);
		retry--;
	} while (retry > 0);
	if (!retry) {
		ETHQOSERR("PCS_READY timedout with retry = %d\n", retry);
		ret = -1;
		goto err_ret;
	}

	retry = 500;
	do {
		val = readl_relaxed(ethqos->sgmii_base + QSERDES_PCS_READY_STATUS);
		val &= BIT(0);
		if (val)
			break;
		usleep_range(1000, 1500);
		retry--;
	} while (retry > 0);
	if (!retry) {
		ETHQOSERR("SGMIIPHY_READY timedout with retry = %d\n", retry);
		ret = -1;
		goto err_ret;
	}

	ETHQOSINFO("%s : Serdes is up on USXGMII 2.5 Gbps mode", __func__);
	return ret;

err_ret:
	return ret;
}

static int qcom_ethqos_serdes_usxgmii_5Gb(struct qcom_ethqos *ethqos)
{
	int ret = 0;
	int retry = 500;
	unsigned int val;

	/****************MODULE: SGMII_PHY_SGMII_PCS**********************************/
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES_PCS_SW_RESET);
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES_PCS_POWER_DOWN_CONTROL);

	/***************** MODULE: QSERDES_COM_SGMII_QMP_PLL*********/
	writel_relaxed(0x0E, ethqos->sgmii_base + QSERDES_COM_BG_TIMER);
	writel_relaxed(0x07, ethqos->sgmii_base + QSERDES_COM_BIAS_EN_CLKBUFLR_EN);
	writel_relaxed(0x0F, ethqos->sgmii_base + QSERDES_COM_CLK_ENABLE1);
	writel_relaxed(0x1F, ethqos->sgmii_base + QSERDES_COM_PLL_IVCO);
	writel_relaxed(0x08, ethqos->sgmii_base + QSERDES_COM_CP_CTRL_MODE0);
	writel_relaxed(0x16, ethqos->sgmii_base + QSERDES_COM_PLL_RCTRL_MODE0);
	writel_relaxed(0x36, ethqos->sgmii_base + QSERDES_COM_PLL_CCTRL_MODE0);
	writel_relaxed(0X1F, ethqos->sgmii_base + QSERDES_COM_INTEGLOOP_GAIN0_MODE0);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_COM_INTEGLOOP_GAIN1_MODE0);
	writel_relaxed(0x03, ethqos->sgmii_base + QSERDES_COM_PLL_EN);
	writel_relaxed(0x1A, ethqos->sgmii_base + QSERDES_COM_SYSCLK_EN_SEL);
	writel_relaxed(0x23, ethqos->sgmii_base + QSERDES_COM_LOCK_CMP1_MODE0);
	writel_relaxed(0x43, ethqos->sgmii_base + QSERDES_COM_LOCK_CMP2_MODE0);
	writel_relaxed(0x86, ethqos->sgmii_base + QSERDES_COM_DEC_START_MODE0);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_COM_DIV_FRAC_START1_MODE0);
	writel_relaxed(0x70, ethqos->sgmii_base + QSERDES_COM_DIV_FRAC_START2_MODE0);
	writel_relaxed(0x04, ethqos->sgmii_base + QSERDES_COM_DIV_FRAC_START3_MODE0);
	writel_relaxed(0x46, ethqos->sgmii_base + QSERDES_COM_VCO_TUNE1_MODE0);
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES_COM_VCO_TUNE2_MODE0);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_COM_VCO_TUNE_INITVAL2);
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES_COM_HSCLK_SEL_1);
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES_COM_HSCLK_HS_SWITCH_SEL_1);
	writel_relaxed(0x04, ethqos->sgmii_base + QSERDES_COM_CORECLK_DIV_MODE0);
	writel_relaxed(0x10, ethqos->sgmii_base + QSERDES_COM_CORE_CLK_EN);
	writel_relaxed(0x16, ethqos->sgmii_base + QSERDES_COM_CMN_CONFIG_1);
	writel_relaxed(0xAE, ethqos->sgmii_base + QSERDES_COM_BIN_VCOCAL_CMP_CODE1_MODE0);
	writel_relaxed(0x1F, ethqos->sgmii_base + QSERDES_COM_BIN_VCOCAL_CMP_CODE2_MODE0);
	writel_relaxed(0x11, ethqos->sgmii_base + QSERDES_COM_BIN_VCOCAL_HSCLK_SEL_1);

	/******************MODULE: QSERDES_TX0_SGMII_QMP_TX***********************/
	writel_relaxed(0x0D, ethqos->sgmii_base + QSERDES_TX_CLKBUF_ENABLE);
	writel_relaxed(0x04, ethqos->sgmii_base + QSERDES_TX0_TX_BAND);
	writel_relaxed(0x08, ethqos->sgmii_base + QSERDES_TX0_SLEW_CNTL);
	writel_relaxed(0x09, ethqos->sgmii_base + QSERDES_TX0_RES_CODE_LANE_OFFSET_TX);
	writel_relaxed(0x09, ethqos->sgmii_base + QSERDES_TX0_RES_CODE_LANE_OFFSET_RX);
	writel_relaxed(0xF5, ethqos->sgmii_base + QSERDES_TX0_LANE_MODE_1);
	writel_relaxed(0x06, ethqos->sgmii_base + QSERDES_TX0_LANE_MODE_2);
	writel_relaxed(0x3F, ethqos->sgmii_base + QSERDES_TX0_LANE_MODE_3);
	writel_relaxed(0x3F, ethqos->sgmii_base + QSERDES_TX_LANE_MODE_4);
	writel_relaxed(0x5F, ethqos->sgmii_base + QSERDES_TX_LANE_MODE_5);
	writel_relaxed(0x12, ethqos->sgmii_base + QSERDES_TX0_RCV_DETECT_LVL_2);
	writel_relaxed(0x0F, ethqos->sgmii_base + QSERDES_TX0_TRAN_DRVR_EMP_EN);
	writel_relaxed(0x2B, ethqos->sgmii_base + QSERDES_TX_TX_EMP_POST1_LVL);

	/*****************MODULE: QSERDES_RX0_SGMII_QMP_RX*******************/
	writel_relaxed(0x0D, ethqos->sgmii_base + QSERDES_RX0_UCDR_FO_GAIN);
	writel_relaxed(0x03, ethqos->sgmii_base + QSERDES_RX0_UCDR_SO_GAIN);
	writel_relaxed(0x0A, ethqos->sgmii_base + QSERDES_RX0_UCDR_FASTLOCK_FO_GAIN);
	writel_relaxed(0x7F, ethqos->sgmii_base + QSERDES_RX0_UCDR_SO_SATURATION_AND_ENABLE);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_RX0_UCDR_FASTLOCK_COUNT_LOW);
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES_RX0_UCDR_FASTLOCK_COUNT_HIGH);
	writel_relaxed(0x81, ethqos->sgmii_base + QSERDES_RX0_UCDR_PI_CONTROLS);
	writel_relaxed(0x81, ethqos->sgmii_base + QSERDES_RX0_UCDR_PI_CTRL2);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_RX_UCDR_SB2_THRESH1);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_RX_UCDR_SB2_THRESH2);
	writel_relaxed(0x03, ethqos->sgmii_base + QSERDES_RX0_RX_TERM_BW);
	writel_relaxed(0x08, ethqos->sgmii_base + QSERDES_RX0_VGA_CAL_CNTRL2);
	writel_relaxed(0x0F, ethqos->sgmii_base + QSERDES_RX0_GM_CAL);
	writel_relaxed(0x04, ethqos->sgmii_base + QSERDES_RX0_RX_EQU_ADAPTOR_CNTRL1);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_RX0_RX_EQU_ADAPTOR_CNTRL2);
	writel_relaxed(0x4A, ethqos->sgmii_base + QSERDES_RX0_RX_EQU_ADAPTOR_CNTRL3);
	writel_relaxed(0x5A, ethqos->sgmii_base + QSERDES_RX0_RX_EQU_ADAPTOR_CNTRL4);
	writel_relaxed(0x80, ethqos->sgmii_base + QSERDES_RX0_RX_IDAC_TSETTLE_LOW);
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES_RX0_RX_IDAC_TSETTLE_HIGH);
	writel_relaxed(0x20, ethqos->sgmii_base + QSERDES_RX0_RX_IDAC_MEASURE_TIME);
	writel_relaxed(0x17, ethqos->sgmii_base + QSERDES_RX0_RX_EQ_OFFSET_ADAPTOR_CNTRL1);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_RX0_RX_OFFSET_ADAPTOR_CNTRL2);
	writel_relaxed(0x0F, ethqos->sgmii_base + QSERDES_RX0_SIGDET_CNTRL);
	writel_relaxed(0x1E, ethqos->sgmii_base + QSERDES_RX0_SIGDET_DEGLITCH_CNTRL);
	writel_relaxed(0x18, ethqos->sgmii_base + QSERDES_RX0_RX_BAND);
	writel_relaxed(0x1F, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_00_LOW);
	writel_relaxed(0xBF, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_00_HIGH);
	writel_relaxed(0xFF, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_00_HIGH2);
	writel_relaxed(0xDF, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_00_HIGH3);
	writel_relaxed(0xED, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_00_HIGH4);
	writel_relaxed(0xE0, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_01_LOW);
	writel_relaxed(0xC8, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_01_HIGH);
	writel_relaxed(0xC8, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_01_HIGH2);
	writel_relaxed(0x09, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_01_HIGH3);
	writel_relaxed(0xB1, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_01_HIGH4);
	writel_relaxed(0xE0, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_10_LOW);
	writel_relaxed(0xC8, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_10_HIGH);
	writel_relaxed(0xC8, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_10_HIGH2);
	writel_relaxed(0x3B, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_10_HIGH3);
	writel_relaxed(0xB7, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_10_HIGH4);
	writel_relaxed(0x0C, ethqos->sgmii_base + QSERDES_RX0_DCC_CTRL1);
	writel_relaxed(0x0C, ethqos->sgmii_base + QSERDES_PCS_LINE_RESET_TIME);
	/****************MODULE: SGMII_PHY_SGMII_PCS**********************************/
	writel_relaxed(0x03, ethqos->sgmii_base + QSERDES_PCS_TX_SMALL_AMP_DRV_LVL);
	writel_relaxed(0x1A, ethqos->sgmii_base + QSERDES_PCS_TX_LARGE_AMP_DRV_LVL);
	writel_relaxed(0x0B, ethqos->sgmii_base + QSERDES_PCS_TX_LARGE_AMP_POST_EMP_LVL);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_PCS_TX_SMALL_AMP_POST_EMP_LVL);
	writel_relaxed(0x83, ethqos->sgmii_base + QSERDES_PCS_TX_MID_TERM_CTRL1);
	writel_relaxed(0x08, ethqos->sgmii_base + QSERDES_PCS_TX_MID_TERM_CTRL2);

	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_RX0_SIGDET_CAL_CTRL1);
	writel_relaxed(0xCC, ethqos->sgmii_base + QSERDES_PCS_RX_SIGDET_CTRL2);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_RX0_SIGDET_CAL_CTRL2_AND_CDR_LOCK_EDGE);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_PCS_SGMII_MISC_CTRL7);
	writel_relaxed(0x14, ethqos->sgmii_base + QSERDES_PCS_SGMII_MISC_CTRL8);
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES_PCS_RETIME_BUFFER_EN);

	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_PCS_SW_RESET);
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES_PCS_PHY_START);
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES_PCS_PHY_START);
	usleep_range(3000, 5000);

	do {
		val = readl_relaxed(ethqos->sgmii_base + QSERDES_COM_CMN_STATUS);
		val &= BIT(1);
		if (val)
			break;
		usleep_range(1000, 1500);
		retry--;
	} while (retry > 0);
	if (!retry) {
		ETHQOSERR("PLL Lock Status timedout with retry = %d\n", retry);
		ret = -1;
		goto err_ret;
	}

	retry = 500;
	do {
		val = readl_relaxed(ethqos->sgmii_base + QSERDES_COM_C_READY_STATUS);
		val &= BIT(0);
		if (val)
			break;
		usleep_range(1000, 1500);
		retry--;
	} while (retry > 0);
	if (!retry) {
		ETHQOSERR("C_READY_STATUS timedout with retry = %d\n", retry);
		ret = -1;
		goto err_ret;
	}

	retry = 500;
	do {
		val = readl_relaxed(ethqos->sgmii_base + QSERDES_PCS_READY_STATUS);
		val &= BIT(7);
		if (val)
			break;
		usleep_range(1000, 1500);
		retry--;
	} while (retry > 0);
	if (!retry) {
		ETHQOSERR("PCS_READY timedout with retry = %d\n", retry);
		ret = -1;
		goto err_ret;
	}

	retry = 500;
	do {
		val = readl_relaxed(ethqos->sgmii_base + QSERDES_PCS_READY_STATUS);
		val &= BIT(0);
		if (val)
			break;
		usleep_range(1000, 1500);
		retry--;
	} while (retry > 0);
	if (!retry) {
		ETHQOSERR("SGMIIPHY_READY timedout with retry = %d\n", retry);
		ret = -1;
		goto err_ret;
	}

	return ret;

err_ret:
	return ret;
}

static int qcom_ethqos_serdes_usxgmii_10Gb_1Gb(struct qcom_ethqos *ethqos)
{
	int ret = 0;
	int retry = 500;
	unsigned int val;

	/****************MODULE: SGMII_PHY_SGMII_PCS**********************************/
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES_PCS_SW_RESET);
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES_PCS_POWER_DOWN_CONTROL);

	/***************** MODULE: QSERDES_COM_SGMII_QMP_PLL*********/
	writel_relaxed(0x0E, ethqos->sgmii_base + QSERDES_COM_BG_TIMER);
	writel_relaxed(0x07, ethqos->sgmii_base + QSERDES_COM_BIAS_EN_CLKBUFLR_EN);
	writel_relaxed(0x0F, ethqos->sgmii_base + QSERDES_COM_CLK_ENABLE1);
	writel_relaxed(0x1F, ethqos->sgmii_base + QSERDES_COM_PLL_IVCO);
	writel_relaxed(0x08, ethqos->sgmii_base + QSERDES_COM_CP_CTRL_MODE0);
	writel_relaxed(0x16, ethqos->sgmii_base + QSERDES_COM_PLL_RCTRL_MODE0);
	writel_relaxed(0x36, ethqos->sgmii_base + QSERDES_COM_PLL_CCTRL_MODE0);
	writel_relaxed(0X1F, ethqos->sgmii_base + QSERDES_COM_INTEGLOOP_GAIN0_MODE0);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_COM_INTEGLOOP_GAIN1_MODE0);
	writel_relaxed(0x03, ethqos->sgmii_base + QSERDES_COM_PLL_EN);
	writel_relaxed(0x1A, ethqos->sgmii_base + QSERDES_COM_SYSCLK_EN_SEL);
	writel_relaxed(0x46, ethqos->sgmii_base + QSERDES_COM_LOCK_CMP1_MODE0);
	writel_relaxed(0x86, ethqos->sgmii_base + QSERDES_COM_LOCK_CMP2_MODE0);
	writel_relaxed(0x86, ethqos->sgmii_base + QSERDES_COM_DEC_START_MODE0);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_COM_DIV_FRAC_START1_MODE0);
	writel_relaxed(0x70, ethqos->sgmii_base + QSERDES_COM_DIV_FRAC_START2_MODE0);
	writel_relaxed(0x04, ethqos->sgmii_base + QSERDES_COM_DIV_FRAC_START3_MODE0);
	writel_relaxed(0x46, ethqos->sgmii_base + QSERDES_COM_VCO_TUNE1_MODE0);
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES_COM_VCO_TUNE2_MODE0);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_COM_VCO_TUNE_INITVAL2);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_COM_HSCLK_SEL_1);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_COM_HSCLK_HS_SWITCH_SEL_1);
	writel_relaxed(0x04, ethqos->sgmii_base + QSERDES_COM_CORECLK_DIV_MODE0);
	writel_relaxed(0x10, ethqos->sgmii_base + QSERDES_COM_CORE_CLK_EN);
	writel_relaxed(0x16, ethqos->sgmii_base + QSERDES_COM_CMN_CONFIG_1);
	writel_relaxed(0xAE, ethqos->sgmii_base + QSERDES_COM_BIN_VCOCAL_CMP_CODE1_MODE0);
	writel_relaxed(0x1F, ethqos->sgmii_base + QSERDES_COM_BIN_VCOCAL_CMP_CODE2_MODE0);
	writel_relaxed(0x11, ethqos->sgmii_base + QSERDES_COM_BIN_VCOCAL_HSCLK_SEL_1);

	/******************MODULE: QSERDES_TX0_SGMII_QMP_TX***********************/
	writel_relaxed(0x0D, ethqos->sgmii_base + QSERDES_TX_CLKBUF_ENABLE);
	writel_relaxed(0x04, ethqos->sgmii_base + QSERDES_TX0_TX_BAND);
	writel_relaxed(0x08, ethqos->sgmii_base + QSERDES_TX0_SLEW_CNTL);
	writel_relaxed(0x09, ethqos->sgmii_base + QSERDES_TX0_RES_CODE_LANE_OFFSET_TX);
	writel_relaxed(0x09, ethqos->sgmii_base + QSERDES_TX0_RES_CODE_LANE_OFFSET_RX);
	writel_relaxed(0xF5, ethqos->sgmii_base + QSERDES_TX0_LANE_MODE_1);
	writel_relaxed(0x06, ethqos->sgmii_base + QSERDES_TX0_LANE_MODE_2);
	writel_relaxed(0x3F, ethqos->sgmii_base + QSERDES_TX0_LANE_MODE_3);
	writel_relaxed(0x3F, ethqos->sgmii_base + QSERDES_TX_LANE_MODE_4);
	writel_relaxed(0x5F, ethqos->sgmii_base + QSERDES_TX_LANE_MODE_5);
	writel_relaxed(0x12, ethqos->sgmii_base + QSERDES_TX0_RCV_DETECT_LVL_2);
	writel_relaxed(0x0F, ethqos->sgmii_base + QSERDES_TX0_TRAN_DRVR_EMP_EN);
	writel_relaxed(0x2B, ethqos->sgmii_base + QSERDES_TX_TX_EMP_POST1_LVL);

	/*****************MODULE: QSERDES_RX0_SGMII_QMP_RX*******************/
	writel_relaxed(0x0D, ethqos->sgmii_base + QSERDES_RX0_UCDR_FO_GAIN);
	writel_relaxed(0x03, ethqos->sgmii_base + QSERDES_RX0_UCDR_SO_GAIN);
	writel_relaxed(0x0A, ethqos->sgmii_base + QSERDES_RX0_UCDR_FASTLOCK_FO_GAIN);
	writel_relaxed(0x7F, ethqos->sgmii_base + QSERDES_RX0_UCDR_SO_SATURATION_AND_ENABLE);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_RX0_UCDR_FASTLOCK_COUNT_LOW);
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES_RX0_UCDR_FASTLOCK_COUNT_HIGH);
	writel_relaxed(0x81, ethqos->sgmii_base + QSERDES_RX0_UCDR_PI_CONTROLS);
	writel_relaxed(0x81, ethqos->sgmii_base + QSERDES_RX0_UCDR_PI_CTRL2);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_RX_UCDR_SB2_THRESH1);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_RX_UCDR_SB2_THRESH2);
	writel_relaxed(0x03, ethqos->sgmii_base + QSERDES_RX0_RX_TERM_BW);
	writel_relaxed(0x08, ethqos->sgmii_base + QSERDES_RX0_VGA_CAL_CNTRL2);
	writel_relaxed(0x0F, ethqos->sgmii_base + QSERDES_RX0_GM_CAL);
	writel_relaxed(0x04, ethqos->sgmii_base + QSERDES_RX0_RX_EQU_ADAPTOR_CNTRL1);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_RX0_RX_EQU_ADAPTOR_CNTRL2);
	writel_relaxed(0x4A, ethqos->sgmii_base + QSERDES_RX0_RX_EQU_ADAPTOR_CNTRL3);
	writel_relaxed(0x5A, ethqos->sgmii_base + QSERDES_RX0_RX_EQU_ADAPTOR_CNTRL4);
	writel_relaxed(0x80, ethqos->sgmii_base + QSERDES_RX0_RX_IDAC_TSETTLE_LOW);
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES_RX0_RX_IDAC_TSETTLE_HIGH);
	writel_relaxed(0x20, ethqos->sgmii_base + QSERDES_RX0_RX_IDAC_MEASURE_TIME);
	writel_relaxed(0x17, ethqos->sgmii_base + QSERDES_RX0_RX_EQ_OFFSET_ADAPTOR_CNTRL1);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_RX0_RX_OFFSET_ADAPTOR_CNTRL2);
	writel_relaxed(0x0F, ethqos->sgmii_base + QSERDES_RX0_SIGDET_CNTRL);
	writel_relaxed(0x1E, ethqos->sgmii_base + QSERDES_RX0_SIGDET_DEGLITCH_CNTRL);
	writel_relaxed(0x18, ethqos->sgmii_base + QSERDES_RX0_RX_BAND);
	writel_relaxed(0x1F, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_00_LOW);
	writel_relaxed(0xBF, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_00_HIGH);
	writel_relaxed(0xFF, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_00_HIGH2);
	writel_relaxed(0xDF, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_00_HIGH3);
	writel_relaxed(0xED, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_00_HIGH4);
	writel_relaxed(0xE0, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_01_LOW);
	writel_relaxed(0xC8, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_01_HIGH);
	writel_relaxed(0xC8, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_01_HIGH2);
	writel_relaxed(0x09, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_01_HIGH3);
	writel_relaxed(0xB1, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_01_HIGH4);
	writel_relaxed(0xE0, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_10_LOW);
	writel_relaxed(0xC8, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_10_HIGH);
	writel_relaxed(0xC8, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_10_HIGH2);
	writel_relaxed(0x3B, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_10_HIGH3);
	writel_relaxed(0xB7, ethqos->sgmii_base + QSERDES_RX0_RX_MODE_10_HIGH4);
	writel_relaxed(0x0C, ethqos->sgmii_base + QSERDES_RX0_DCC_CTRL1);
	writel_relaxed(0x0C, ethqos->sgmii_base + QSERDES_PCS_LINE_RESET_TIME);
	/****************MODULE: SGMII_PHY_SGMII_PCS**********************************/
	writel_relaxed(0x03, ethqos->sgmii_base + QSERDES_PCS_TX_SMALL_AMP_DRV_LVL);
	writel_relaxed(0x1A, ethqos->sgmii_base + QSERDES_PCS_TX_LARGE_AMP_DRV_LVL);
	writel_relaxed(0x0B, ethqos->sgmii_base + QSERDES_PCS_TX_LARGE_AMP_POST_EMP_LVL);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_PCS_TX_SMALL_AMP_POST_EMP_LVL);
	writel_relaxed(0x83, ethqos->sgmii_base + QSERDES_PCS_TX_MID_TERM_CTRL1);
	writel_relaxed(0x08, ethqos->sgmii_base + QSERDES_PCS_TX_MID_TERM_CTRL2);

	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_RX0_SIGDET_CAL_CTRL1);
	writel_relaxed(0xCC, ethqos->sgmii_base + QSERDES_PCS_RX_SIGDET_CTRL2);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_RX0_SIGDET_CAL_CTRL2_AND_CDR_LOCK_EDGE);
	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_PCS_SGMII_MISC_CTRL7);
	writel_relaxed(0x14, ethqos->sgmii_base + QSERDES_PCS_SGMII_MISC_CTRL8);
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES_PCS_RETIME_BUFFER_EN);

	writel_relaxed(0x00, ethqos->sgmii_base + QSERDES_PCS_SW_RESET);
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES_PCS_PHY_START);
	writel_relaxed(0x01, ethqos->sgmii_base + QSERDES_PCS_PHY_START);
	usleep_range(3000, 5000);

	do {
		val = readl_relaxed(ethqos->sgmii_base + QSERDES_COM_CMN_STATUS);
		val &= BIT(1);
		if (val)
			break;
		usleep_range(1000, 1500);
		retry--;
	} while (retry > 0);
	if (!retry) {
		ETHQOSERR("PLL Lock Status timedout with retry = %d\n", retry);
		ret = -1;
		goto err_ret;
	}

	retry = 500;
	do {
		val = readl_relaxed(ethqos->sgmii_base + QSERDES_COM_C_READY_STATUS);
		val &= BIT(0);
		if (val)
			break;
		usleep_range(1000, 1500);
		retry--;
	} while (retry > 0);
	if (!retry) {
		ETHQOSERR("C_READY_STATUS timedout with retry = %d\n", retry);
		ret = -1;
		goto err_ret;
	}

	retry = 500;
	do {
		val = readl_relaxed(ethqos->sgmii_base + QSERDES_PCS_READY_STATUS);
		val &= BIT(7);
		if (val)
			break;
		usleep_range(1000, 1500);
		retry--;
	} while (retry > 0);
	if (!retry) {
		ETHQOSERR("PCS_READY timedout with retry = %d\n", retry);
		ret = -1;
		goto err_ret;
	}

	retry = 500;
	do {
		val = readl_relaxed(ethqos->sgmii_base + QSERDES_PCS_READY_STATUS);
		val &= BIT(0);
		if (val)
			break;
		usleep_range(1000, 1500);
		retry--;
	} while (retry > 0);
	if (!retry) {
		ETHQOSERR("SGMIIPHY_READY timedout with retry = %d\n", retry);
		ret = -1;
		goto err_ret;
	}

	return ret;

err_ret:
	return ret;
}

static int qcom_ethqos_serdes_update_sgmii(struct qcom_ethqos *ethqos,
					   int speed)
{
	int ret = 0;
	bool use_prog_tbl = ethqos->enable_debug_serdes_mode &&
			    ethqos->emac_ver != EMAC_HW_v3_1_0;

	switch (speed) {
	case SPEED_1000:
		if (ethqos->curr_serdes_speed != SPEED_1000) {
			if (use_prog_tbl)
				ret = qcom_ethqos_serdes_prog_tbl(ethqos, SERDES_MODE_SGMII_1G);
			else
				ret = qcom_ethqos_serdes_sgmii_1Gb(ethqos);
		}

		ethqos->curr_serdes_speed = SPEED_1000;
		break;
	case SPEED_2500:
		if (ethqos->curr_serdes_speed != SPEED_2500) {
			if (use_prog_tbl)
				ret = qcom_ethqos_serdes_prog_tbl(ethqos, SERDES_MODE_SGMII_2P5G);
			else
				ret = qcom_ethqos_serdes_sgmii_2p5Gb(ethqos);
		}

		ethqos->curr_serdes_speed = SPEED_2500;
		break;
	case SPEED_100:
	case SPEED_10:
		if (ethqos->curr_serdes_speed != SPEED_1000) {
			ETHQOSINFO("%s : Serdes Speed set to 1GB speed", __func__);
			if (use_prog_tbl)
				ret = qcom_ethqos_serdes_prog_tbl(ethqos, SERDES_MODE_SGMII_1G);
			else
				ret = qcom_ethqos_serdes_sgmii_1Gb(ethqos);
			ethqos->curr_serdes_speed = SPEED_1000;
		}
		break;
	default:
		ETHQOSERR("%s : Serdes Speed not set for speed = %d\n",
			  __func__,
			   speed);
		ethqos->curr_serdes_speed = 0;
		break;
	}

	ETHQOSINFO("%s : Exit serdes speed = %d",
		   __func__,
		   ethqos->curr_serdes_speed);
	return ret;
}

static int qcom_ethqos_serdes_update_usxgmii(struct qcom_ethqos *ethqos,
					     int speed)
{
	int ret = 0;
	bool use_prog_tbl = ethqos->enable_debug_serdes_mode &&
			    ethqos->emac_ver != EMAC_HW_v3_1_0;

	switch (ethqos->usxgmii_mode) {
	case USXGMII_MODE_10G:
		if (ethqos->curr_serdes_speed != SPEED_10000) {
			if (use_prog_tbl)
				ret = qcom_ethqos_serdes_prog_tbl(ethqos, SERDES_MODE_USXGMII_10G);
			else
				ret = qcom_ethqos_serdes_usxgmii_10Gb_1Gb(ethqos);
		}

		ethqos->curr_serdes_speed = SPEED_10000;
		break;
	case USXGMII_MODE_5G:
		if (ethqos->curr_serdes_speed != SPEED_5000) {
			if (use_prog_tbl)
				ret = qcom_ethqos_serdes_prog_tbl(ethqos, SERDES_MODE_USXGMII_5G);
			else
				ret = qcom_ethqos_serdes_usxgmii_5Gb(ethqos);
		}

		ethqos->curr_serdes_speed = SPEED_5000;
		break;
	case USXGMII_MODE_2P5G:
		if (ethqos->curr_serdes_speed != SPEED_2500) {
			if (use_prog_tbl)
				ret = qcom_ethqos_serdes_prog_tbl(ethqos, SERDES_MODE_USXGMII_2P5G);
			else
				ret = qcom_ethqos_serdes_usxgmii_2p5Gb(ethqos);
		}

		ethqos->curr_serdes_speed = SPEED_2500;
		break;
	default:
		ETHQOSERR("%s : Serdes clock not set for USXGMII PHY speed = %d\n",
			  __func__,
			  ethqos->usxgmii_mode);
		ethqos->curr_serdes_speed = 0;
		break;
	}

	ETHQOSINFO("%s : exit ret = %d", __func__, ret);
	return ret;
}

int qcom_ethqos_serdes_update(struct qcom_ethqos *ethqos,
			      int speed,
			      int interface)
{
	int ret = 0;

#ifdef CONFIG_MSM_BOOT_TIME_MARKER
	update_marker("M -Ethernet Serdes power up start");
#endif

	switch (interface) {
	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_2500BASEX:
		ret = qcom_ethqos_serdes_update_sgmii(ethqos, speed);
		break;
	case PHY_INTERFACE_MODE_5GBASER:
	case PHY_INTERFACE_MODE_USXGMII:
		if (interface == PHY_INTERFACE_MODE_USXGMII)
			ethqos->usxgmii_mode = USXGMII_MODE_10G;
		else if (interface == PHY_INTERFACE_MODE_5GBASER)
			ethqos->usxgmii_mode = USXGMII_MODE_5G;

		ret = qcom_ethqos_serdes_update_usxgmii(ethqos, speed);
		break;
	default:
		ETHQOSINFO("%s : PHY interface not supported", __func__);
		ret = EINVAL;
		break;
	}
#ifdef CONFIG_MSM_BOOT_TIME_MARKER
	update_marker("M - Ethernet Serdes power up end");
#endif

	return ret;
}
EXPORT_SYMBOL(qcom_ethqos_serdes_update);

int qcom_ethqos_serdes_configure_dt(struct qcom_ethqos *ethqos, int interface)
{
	struct resource *res = NULL;
	struct platform_device *pdev = ethqos->pdev;
	int ret;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "serdes");
	ethqos->sgmii_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(ethqos->sgmii_base)) {
		dev_err(&pdev->dev, "Can't get sgmii base\n");
		ret = PTR_ERR(ethqos->sgmii_base);
		goto err_mem;
	}

	ethqos->sgmiref_clk = devm_clk_get(&pdev->dev, "sgmi_ref");
	if (IS_ERR(ethqos->sgmiref_clk)) {
		ETHQOSERR("%s : configure_serdes_dt Failed sgmi_ref", __func__);
		ret = PTR_ERR(ethqos->sgmiref_clk);
		goto err_mem;
	}
	ethqos->phyaux_clk = devm_clk_get(&pdev->dev, "phyaux");
	if (IS_ERR(ethqos->phyaux_clk)) {
		ETHQOSERR("%s : configure_serdes_dt Failed phyaux", __func__);
		ret = PTR_ERR(ethqos->phyaux_clk);
		goto err_mem;
	}

	ret = qcom_ethqos_enable_serdes_clocks(ethqos);
	if (ret)
		goto err_mem;

	return 0;

err_mem:
	ETHQOSERR("%s : configure_serdes_dt Failed", __func__);
	return -1;
}
EXPORT_SYMBOL(qcom_ethqos_serdes_configure_dt);

static ssize_t enable_debug_serdes_mode_show(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	struct net_device *netdev = to_net_dev(dev);
	struct stmmac_priv *priv;
	struct qcom_ethqos *ethqos;

	if (!netdev) {
		ETHQOSERR("netdev is NULL\n");
		return -EINVAL;
	}
	priv = netdev_priv(netdev);
	if (!priv) {
		ETHQOSERR("priv is NULL\n");
		return -EINVAL;
	}
	ethqos = priv->plat->bsp_priv;
	if (!ethqos) {
		ETHQOSERR("ethqos is NULL\n");
		return -EINVAL;
	}

	return scnprintf(buf, PAGE_SIZE, "%d\n", ethqos->enable_debug_serdes_mode);
}

static ssize_t enable_debug_serdes_mode_store(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t size)
{
	struct net_device *netdev = to_net_dev(dev);
	struct stmmac_priv *priv;
	struct qcom_ethqos *ethqos;
	bool val;

	if (!netdev) {
		ETHQOSERR("netdev is NULL\n");
		return -EINVAL;
	}
	priv = netdev_priv(netdev);
	if (!priv) {
		ETHQOSERR("priv is NULL\n");
		return -EINVAL;
	}
	ethqos = priv->plat->bsp_priv;
	if (!ethqos) {
		ETHQOSERR("ethqos is NULL\n");
		return -EINVAL;
	}

	if (kstrtobool(buf, &val)) {
		ETHQOSERR("Error in reading option from user\n");
		return -EINVAL;
	}

	ethqos->enable_debug_serdes_mode = val;

	return size;
}

static DEVICE_ATTR_RW(enable_debug_serdes_mode);

static ssize_t serdes_prog_table_read(struct file *filp, struct kobject *kobj,
				      struct bin_attribute *battr, char *buf,
				      loff_t off, size_t count)
{
	char *tmp;
	size_t len = 0, tmp_size;
	unsigned int i, j;
	ssize_t ret;

	tmp_size = serdes_prog_table_size * 48 + 1;
	tmp = kmalloc(tmp_size, GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	for (i = 0; i < serdes_prog_table_size; i++) {
		len += scnprintf(tmp + len, tmp_size - len, "0x%04x",
				 serdes_prog_table[i].reg);
		for (j = 0; j < SERDES_MODE_NUM_MODES; j++) {
			if (serdes_prog_table[i].value[j] == SKIP)
				len += scnprintf(tmp + len, tmp_size - len,
						 "   0xffffffff");
			else
				len += scnprintf(tmp + len, tmp_size - len,
						 "   0x%02x", serdes_prog_table[i].value[j]);
		}
		len += scnprintf(tmp + len, tmp_size - len, "\n");
	}

	ret = memory_read_from_buffer(buf, count, &off, tmp, len);
	kfree(tmp);
	return ret;
}

static ssize_t serdes_prog_table_write(struct file *filp, struct kobject *kobj,
				       struct bin_attribute *battr, char *buf,
				       loff_t off, size_t count)
{
	char *line, *rest, *tmp;
	size_t tbl_idx = 0;

	/* The whole table is small enough to always arrive in a single write(2). */
	if (off != 0)
		return -EINVAL;

	tmp = kmemdup_nul(buf, count, GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	rest = tmp;
	while ((line = strsep(&rest, "\n")) != NULL) {
		unsigned int reg, val[SERDES_MODE_NUM_MODES];
		int parsed;

		if (*line == '\0' || *line == '#')
			continue;

		if (tbl_idx >= serdes_prog_table_size) {
			ETHQOSERR("prog_table: input has more entries than table\n");
			break;
		}

		parsed = sscanf(line, "%x %x %x %x %x %x",
				&reg, &val[0], &val[1], &val[2], &val[3], &val[4]);
		if (parsed != 1 + SERDES_MODE_NUM_MODES) {
			ETHQOSERR("prog_table: invalid line \"%s\", expected 0xREG 0xV0..V4\n",
				  line);
			kfree(tmp);
			return -EINVAL;
		}

		serdes_prog_table[tbl_idx].reg = reg;
		memcpy(serdes_prog_table[tbl_idx].value, val, sizeof(val));
		tbl_idx++;
	}

	kfree(tmp);
	return count;
}

static struct bin_attribute bin_attr_prog_table =
	__BIN_ATTR(prog_table, 0644, serdes_prog_table_read, serdes_prog_table_write, 8192);

int qcom_ethqos_serdes_create_sysfs(struct qcom_ethqos *ethqos)
{
	struct net_device *net_dev;
	int ret;

	net_dev = platform_get_drvdata(ethqos->pdev);
	if (!net_dev) {
		ETHQOSERR("netdev is NULL\n");
		return -EINVAL;
	}

	ret = sysfs_create_file(&net_dev->dev.kobj, &dev_attr_enable_debug_serdes_mode.attr);
	if (ret) {
		ETHQOSERR("unable to create enable_debug_serdes_mode sysfs node\n");
		return ret;
	}

	ret = sysfs_create_bin_file(&net_dev->dev.kobj, &bin_attr_prog_table);
	if (ret) {
		ETHQOSERR("unable to create prog_table sysfs node\n");
		sysfs_remove_file(&net_dev->dev.kobj, &dev_attr_enable_debug_serdes_mode.attr);
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(qcom_ethqos_serdes_create_sysfs);

int qcom_ethqos_serdes_remove_sysfs(struct qcom_ethqos *ethqos)
{
	struct net_device *net_dev;

	net_dev = platform_get_drvdata(ethqos->pdev);
	if (!net_dev) {
		ETHQOSERR("netdev is NULL\n");
		return -EINVAL;
	}

	sysfs_remove_bin_file(&net_dev->dev.kobj, &bin_attr_prog_table);
	sysfs_remove_file(&net_dev->dev.kobj, &dev_attr_enable_debug_serdes_mode.attr);

	return 0;
}
EXPORT_SYMBOL_GPL(qcom_ethqos_serdes_remove_sysfs);
