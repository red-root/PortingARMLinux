/*
 * Copyright (C) 2012-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>
#include <linux/ata.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/regulator/consumer.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <linux/fec.h>
#include <linux/memblock.h>
#include <linux/gpio.h>
#include <linux/etherdevice.h>
#include <linux/regulator/anatop-regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/mxc-hdmi-core.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/mxc_dvfs.h>
#include <mach/memory.h>
#include <mach/iomux-mx6q.h>
#include <mach/imx-uart.h>
#include <mach/viv_gpu.h>
#include <mach/ahci_sata.h>
#include <mach/ipu-v3.h>
#include <mach/mxc_hdmi.h>
#include <mach/mxc_asrc.h>
#include <mach/mipi_dsi.h>

#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include "usb.h"
#include "devices-imx6q.h"
#include "crm_regs.h"
#include "cpu_op-mx6.h"
#include "board-mx6q_zeus.h"
#include "board-mx6dl_zeus.h"

#define ZEUS_VOLUME_UP		IMX_GPIO_NR(1, 2)
#define ZEUS_VOLUME_DN		IMX_GPIO_NR(1, 4)
#define ZEUS_MICROPHONE_DET	IMX_GPIO_NR(3, 31)
#define ZEUS_HDMI_IN_INT	IMX_GPIO_NR(1, 16)
#define ZEUS_HDMI_IN_RST_B	IMX_GPIO_NR(1, 17)
#define ZEUS_MIPICSI_PWN	IMX_GPIO_NR(1, 19)
#define ZEUS_MIPICSI_RST	IMX_GPIO_NR(1, 20)
#define ZEUS_SD3_CD		IMX_GPIO_NR(3, 19)
#define ZEUS_SD3_WP		IMX_GPIO_NR(3, 20)

#define ZEUS_PCIE_PWR_EN	IMX_GPIO_NR(3, 24)
#define ZEUS_USB_OTG_PWR	IMX_GPIO_NR(3, 22)
#define ZEUS_USB_H1_PWR		IMX_GPIO_NR(1, 29)
#define ZEUS_POWER_OFF		IMX_GPIO_NR(1, 5)

#define ZEUS_CAN1_STBY		IMX_GPIO_NR(4, 5)
#define ZEUS_HDMI_CEC_IN	IMX_GPIO_NR(4, 11)
#define ZEUS_PCIE_DIS_B		IMX_GPIO_NR(3, 25)

#define ZEUS_CAP_TCH_INT1	IMX_GPIO_NR(3, 17)
#define ZEUS_CAP_TCH_INT0	IMX_GPIO_NR(3, 16)
#define ZEUS_DISP_RST_B		IMX_GPIO_NR(3, 28)
#define ZEUS_DISP_PWR_EN	IMX_GPIO_NR(3, 29)
#define ZEUS_CABC_EN0		IMX_GPIO_NR(3, 26)
#define ZEUS_CABC_EN1		IMX_GPIO_NR(3, 27)
#define ZEUS_LVDS_5V_PWREN	IMX_GPIO_NR(7, 11)
#define ZEUS_LVDS_3V3_PWREN	IMX_GPIO_NR(7, 12)

#define ZEUS_HEADPHONE_DET	IMX_GPIO_NR(7, 8)
#define ZEUS_PCIE_RST_B_REVB	IMX_GPIO_NR(3, 23)
#define ZEUS_PMIC_INT_B		IMX_GPIO_NR(4, 10)
#define ZEUS_PFUZE_INT		IMX_GPIO_NR(4, 10)

#ifdef CONFIG_MX6_ENET_IRQ_TO_GPIO
#define MX6_ENET_IRQ		IMX_GPIO_NR(1, 6)
#define IOMUX_OBSRV_MUX1_OFFSET	0x3c
#define OBSRV_MUX1_MASK			0x3f
#define OBSRV_MUX1_ENET_IRQ		0x9
#endif

static struct clk *sata_clk;
static int caam_enabled;

extern char *gp_reg_id;
extern char *soc_reg_id;
extern char *pu_reg_id;

static const struct esdhc_platform_data mx6q_zeus_sd1_data __initconst = {
	//.cd_gpio = ZEUS_SD1_CD,
	//.wp_gpio = ZEUS_SD1_WP,
	.keep_power_at_suspend = 1,
	.support_8bit = 0,
	.delay_line = 0,
	.cd_type = ESDHC_CD_CONTROLLER,
};

static const struct esdhc_platform_data mx6q_zeus_sd3_data __initconst = {
	.cd_gpio = ZEUS_SD3_CD,
	.wp_gpio = ZEUS_SD3_WP,
	.keep_power_at_suspend = 1,
	.support_8bit = 1,
	.delay_line = 0,
	.cd_type = ESDHC_CD_CONTROLLER,
};

static const struct esdhc_platform_data mx6q_zeus_sd4_data __initconst = {
	.always_present = 1,
	.keep_power_at_suspend = 1,
	.support_8bit = 1,
	.delay_line = 0,
	.cd_type = ESDHC_CD_PERMANENT,
};

#ifdef CONFIG_IMX_ZEUS_NAND
static int __init gpmi_nand_platform_init(void)
{
        iomux_v3_cfg_t *nand_pads = NULL;
        u32 nand_pads_cnt;

        nand_pads = mx6q_gpmi_nand;
        nand_pads_cnt = ARRAY_SIZE(mx6q_gpmi_nand);
        BUG_ON(!nand_pads);
        return mxc_iomux_v3_setup_multiple_pads(nand_pads, nand_pads_cnt);
}

static const struct gpmi_nand_platform_data
mx6q_gpmi_nand_platform_data __initconst = {
        .platform_init           = gpmi_nand_platform_init,
        .min_prop_delay_in_ns    = 5,
        .max_prop_delay_in_ns    = 9,
        .max_chip_count          = 1,
};
#endif

static const struct anatop_thermal_platform_data
	mx6q_zeus_anatop_thermal_data __initconst = {
		.name = "anatop_thermal",
};

static inline void mx6q_zeus_init_uart(void)
{
	imx6q_add_imx_uart(3, NULL);
	imx6q_add_imx_uart(4, NULL);
}

static int mx6q_zeus_fec_phy_init(struct phy_device *phydev)
{
	unsigned short val;

	/* Ar8031 phy SmartEEE feature cause link status generates glitch,
	 * which cause ethernet link down/up issue, so disable SmartEEE
	 */
	phy_write(phydev, 0xd, 0x3);
	phy_write(phydev, 0xe, 0x805d);
	phy_write(phydev, 0xd, 0x4003);
	val = phy_read(phydev, 0xe);
	val &= ~(0x1 << 8);
	phy_write(phydev, 0xe, val);

	/* To enable AR8031 ouput a 125MHz clk from CLK_25M */
	phy_write(phydev, 0xd, 0x7);
	phy_write(phydev, 0xe, 0x8016);
	phy_write(phydev, 0xd, 0x4007);
	val = phy_read(phydev, 0xe);

	val &= 0xffe3;
	val |= 0x18;
	phy_write(phydev, 0xe, val);

	/* Introduce tx clock delay */
	phy_write(phydev, 0x1d, 0x5);
	val = phy_read(phydev, 0x1e);
	val |= 0x0100;
	phy_write(phydev, 0x1e, val);

	/*check phy power*/
	val = phy_read(phydev, 0x0);

	if (val & BMCR_PDOWN)
		phy_write(phydev, 0x0, (val & ~BMCR_PDOWN));

	return 0;
}

static struct fec_platform_data fec_data __initdata = {
	.init = mx6q_zeus_fec_phy_init,
	.phy = PHY_INTERFACE_MODE_RGMII,
#ifdef CONFIG_MX6_ENET_IRQ_TO_GPIO
	.gpio_irq = MX6_ENET_IRQ,
#endif
};

static struct mxc_audio_platform_data mx6_zeus_audio_data;

static int mx6_zeus_sgtl5000_init(void)
{
	struct clk *clko;
	struct clk *new_parent;
	int rate;

	clko = clk_get(NULL, "clko_clk");
	if (IS_ERR(clko)) {
		pr_err("can't get CLKO clock.\n");
		return PTR_ERR(clko);
	}
	new_parent = clk_get(NULL, "ahb");
	if (!IS_ERR(new_parent)) {
		clk_set_parent(clko, new_parent);
		clk_put(new_parent);
	}
	rate = clk_round_rate(clko, 16000000);
	if (rate < 8000000 || rate > 27000000) {
		pr_err("Error:SGTL5000 mclk freq %d out of range!\n",rate);
		clk_put(clko);
		return -1;
	}

	mx6_zeus_audio_data.sysclk = rate;
	clk_set_rate(clko, rate);
	clk_enable(clko);
	return 0;
}

static struct imx_ssi_platform_data mx6_zeus_ssi_pdata = {
	.flags = IMX_SSI_DMA | IMX_SSI_SYN,
};

static struct mxc_audio_platform_data mx6_zeus_audio_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 4,
	.init = mx6_zeus_sgtl5000_init,
	.hp_gpio = -1,
};

static struct platform_device mx6_zeus_audio_device = {
	.name = "imx-sgtl5000",
};

static void mx6q_mipi_powerdown(int powerdown)
{
	if (powerdown)
		gpio_set_value(ZEUS_MIPICSI_PWN, 1);
	else
		gpio_set_value(ZEUS_MIPICSI_PWN, 0);

	msleep(2);
}

static void mx6q_mipi_sensor_io_init(void)
{
	if (cpu_is_mx6q())
		mxc_iomux_v3_setup_multiple_pads(mx6q_zeus_mipi_sensor_pads,
			ARRAY_SIZE(mx6q_zeus_mipi_sensor_pads));
	else if (cpu_is_mx6dl())
		mxc_iomux_v3_setup_multiple_pads(mx6dl_zeus_mipi_sensor_pads,
			ARRAY_SIZE(mx6dl_zeus_mipi_sensor_pads));

	/* Camera reset */
	gpio_request(ZEUS_MIPICSI_RST, "cam-reset");
	gpio_direction_output(ZEUS_MIPICSI_RST, 1);

	/* Camera power down */
	gpio_request(ZEUS_MIPICSI_PWN, "cam-pwdn");
	gpio_direction_output(ZEUS_MIPICSI_PWN, 1);
	msleep(5);
	gpio_set_value(ZEUS_MIPICSI_PWN, 0);
	msleep(5);
	gpio_set_value(ZEUS_MIPICSI_RST, 0);
	msleep(1);
	gpio_set_value(ZEUS_MIPICSI_RST, 1);
	msleep(5);
	gpio_set_value(ZEUS_MIPICSI_PWN, 1);

	/*for mx6dl, mipi virtual channel 1 connect to csi 1*/
	if (cpu_is_mx6dl())
		mxc_iomux_set_gpr_register(13, 3, 3, 1);
}

static struct fsl_mxc_camera_platform_data mipi_csi2_data = {
	.mclk = 24000000,
	.mclk_source = 0,
	.csi = 1,
	.io_init = mx6q_mipi_sensor_io_init,
	.pwdn = mx6q_mipi_powerdown,
};

static struct imxi2c_platform_data mx6q_zeus_i2c_data = {
	.bitrate = 100000,
};

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("mxc_hdmi_i2c", 0x50),
	},
	{
		I2C_BOARD_INFO("ov5640_mipi", 0x3c),
		.platform_data = (void *)&mipi_csi2_data,
	},
	{
		I2C_BOARD_INFO("egalax_ts", 0x4),
		.irq = gpio_to_irq(ZEUS_CAP_TCH_INT0),
	},
	{
		I2C_BOARD_INFO("sgtl5000", 0x0a),
	},
};

static struct i2c_board_info mxc_i2c2_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("egalax_ts", 0x4),
		.irq = gpio_to_irq(ZEUS_CAP_TCH_INT1),
	},
};

static void imx6q_zeus_usbotg_vbus(bool on)
{
	if (on)
		gpio_set_value(ZEUS_USB_OTG_PWR, 1);
	else
		gpio_set_value(ZEUS_USB_OTG_PWR, 0);
}

static void imx6q_zeus_host1_vbus(bool on)
{
	if (on)
		gpio_set_value(ZEUS_USB_H1_PWR, 1);
	else
		gpio_set_value(ZEUS_USB_H1_PWR, 0);
}

static void __init imx6q_zeus_init_usb(void)
{
	int ret = 0;

	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);
	/* disable external charger detect,
	 * or it will affect signal quality at dp .
	 */
	ret = gpio_request(ZEUS_USB_OTG_PWR, "usb-pwr");
	if (ret) {
		pr_err("failed to get GPIO ZEUS_USB_OTG_PWR: %d\n",
			ret);
		return;
	}
	gpio_direction_output(ZEUS_USB_OTG_PWR, 0);
	/* keep USB host1 VBUS always on */
	ret = gpio_request(ZEUS_USB_H1_PWR, "usb-h1-pwr");
	if (ret) {
		pr_err("failed to get GPIO ZEUS_USB_H1_PWR: %d\n",
			ret);
		return;
	}
	gpio_direction_output(ZEUS_USB_H1_PWR, 0);
	if (board_is_mx6_reva())
		mxc_iomux_set_gpr_register(1, 13, 1, 1);
	else
		mxc_iomux_set_gpr_register(1, 13, 1, 0);

	mx6_set_otghost_vbus_func(imx6q_zeus_usbotg_vbus);
	mx6_set_host1_vbus_func(imx6q_zeus_host1_vbus);

}

/* HW Initialization, if return 0, initialization is successful. */
static int mx6q_zeus_sata_init(struct device *dev, void __iomem *addr)
{
	u32 tmpdata;
	int ret = 0;
	struct clk *clk;

	sata_clk = clk_get(dev, "imx_sata_clk");
	if (IS_ERR(sata_clk)) {
		dev_err(dev, "no sata clock.\n");
		return PTR_ERR(sata_clk);
	}
	ret = clk_enable(sata_clk);
	if (ret) {
		dev_err(dev, "can't enable sata clock.\n");
		goto put_sata_clk;
	}

	/* Set PHY Paremeters, two steps to configure the GPR13,
	 * one write for rest of parameters, mask of first write is 0x07FFFFFD,
	 * and the other one write for setting the mpll_clk_off_b
	 *.rx_eq_val_0(iomuxc_gpr13[26:24]),
	 *.los_lvl(iomuxc_gpr13[23:19]),
	 *.rx_dpll_mode_0(iomuxc_gpr13[18:16]),
	 *.sata_speed(iomuxc_gpr13[15]),
	 *.mpll_ss_en(iomuxc_gpr13[14]),
	 *.tx_atten_0(iomuxc_gpr13[13:11]),
	 *.tx_boost_0(iomuxc_gpr13[10:7]),
	 *.tx_lvl(iomuxc_gpr13[6:2]),
	 *.mpll_ck_off(iomuxc_gpr13[1]),
	 *.tx_edgerate_0(iomuxc_gpr13[0]),
	 */
	tmpdata = readl(IOMUXC_GPR13);
	writel(((tmpdata & ~0x07FFFFFD) | 0x0593A044), IOMUXC_GPR13);

	/* enable SATA_PHY PLL */
	tmpdata = readl(IOMUXC_GPR13);
	writel(((tmpdata & ~0x2) | 0x2), IOMUXC_GPR13);

	/* Get the AHB clock rate, and configure the TIMER1MS reg later */
	clk = clk_get(NULL, "ahb");
	if (IS_ERR(clk)) {
		dev_err(dev, "no ahb clock.\n");
		ret = PTR_ERR(clk);
		goto release_sata_clk;
	}
	tmpdata = clk_get_rate(clk) / 1000;
	clk_put(clk);

#ifdef CONFIG_SATA_AHCI_PLATFORM
	ret = sata_init(addr, tmpdata);
	if (ret == 0)
		return ret;
#else
	usleep_range(1000, 2000);
	/* AHCI PHY enter into PDDQ mode if the AHCI module is not enabled */
	tmpdata = readl(addr + PORT_PHY_CTL);
	writel(tmpdata | PORT_PHY_CTL_PDDQ_LOC, addr + PORT_PHY_CTL);
	pr_info("No AHCI save PWR: PDDQ %s\n", ((readl(addr + PORT_PHY_CTL)
					>> 20) & 1) ? "enabled" : "disabled");
#endif

release_sata_clk:
	/* disable SATA_PHY PLL */
	writel((readl(IOMUXC_GPR13) & ~0x2), IOMUXC_GPR13);
	clk_disable(sata_clk);
put_sata_clk:
	clk_put(sata_clk);

	return ret;
}

#ifdef CONFIG_SATA_AHCI_PLATFORM
static void mx6q_zeus_sata_exit(struct device *dev)
{
	clk_disable(sata_clk);
	clk_put(sata_clk);
}

static struct ahci_platform_data mx6q_zeus_sata_data = {
	.init = mx6q_zeus_sata_init,
	.exit = mx6q_zeus_sata_exit,
};
#endif

static void mx6q_zeus_flexcan0_switch(int enable)
{
	if (enable) {
		gpio_set_value(ZEUS_CAN1_STBY, 1);
	} else {
		gpio_set_value(ZEUS_CAN1_STBY, 0);
	}
}

static const struct flexcan_platform_data
	mx6q_zeus_flexcan0_pdata __initconst = {
	.transceiver_switch = mx6q_zeus_flexcan0_switch,
};

static struct viv_gpu_platform_data imx6q_gpu_pdata __initdata = {
	.reserved_mem_size = SZ_128M,
};

static struct imx_asrc_platform_data imx_asrc_data = {
	.channel_bits = 4,
	.clk_map_ver = 2,
};

static void mx6_reset_mipi_dsi(void)
{
	gpio_set_value(ZEUS_DISP_PWR_EN, 1);
	gpio_set_value(ZEUS_DISP_RST_B, 1);
	udelay(10);
	gpio_set_value(ZEUS_DISP_RST_B, 0);
	udelay(50);
	gpio_set_value(ZEUS_DISP_RST_B, 1);

	/*
	 * it needs to delay 120ms minimum for reset complete
	 */
	msleep(120);
}

static struct mipi_dsi_platform_data mipi_dsi_pdata = {
	.ipu_id		= 0,
	.disp_id	= 1,
	.lcd_panel	= "TRULY-WVGA",
	.reset		= mx6_reset_mipi_dsi,
};

static struct ipuv3_fb_platform_data zeus_fb_data[] = {
	{ /*fb0*/
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "LDB-XGA",
	.default_bpp = 16,
	.int_clk = false,
	.late_init = false,
	}, {
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "LDB-XGA",
	.default_bpp = 16,
	.int_clk = false,
	}, {
	.disp_dev = "lcd",
	.interface_pix_fmt = IPU_PIX_FMT_RGB565,
	.mode_str = "CLAA-WVGA",
	.default_bpp = 16,
	.int_clk = false,
	.late_init = false,
	}, {
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "LDB-VGA",
	.default_bpp = 16,
	.int_clk = false,
	.late_init = false,
	},
};

static void hdmi_init(int ipu_id, int disp_id)
{
	int hdmi_mux_setting;

	if ((ipu_id > 1) || (ipu_id < 0)) {
		pr_err("Invalid IPU select for HDMI: %d. Set to 0\n", ipu_id);
		ipu_id = 0;
	}

	if ((disp_id > 1) || (disp_id < 0)) {
		pr_err("Invalid DI select for HDMI: %d. Set to 0\n", disp_id);
		disp_id = 0;
	}

	/* Configure the connection between IPU1/2 and HDMI */
	hdmi_mux_setting = 2*ipu_id + disp_id;

	/* GPR3, bits 2-3 = HDMI_MUX_CTL */
	mxc_iomux_set_gpr_register(3, 2, 2, hdmi_mux_setting);

	/* Set HDMI event as SDMA event2 while Chip version later than TO1.2 */
	if (hdmi_SDMA_check())
		mxc_iomux_set_gpr_register(0, 0, 1, 1);
}

/* On mx6x zeus board i2c2 iomux with hdmi ddc,
 * the pins default work at i2c2 function,
 when hdcp enable, the pins should work at ddc function */

static void hdmi_enable_ddc_pin(void)
{
	if (cpu_is_mx6dl())
		mxc_iomux_v3_setup_multiple_pads(mx6dl_zeus_hdmi_ddc_pads,
			ARRAY_SIZE(mx6dl_zeus_hdmi_ddc_pads));
	else
		mxc_iomux_v3_setup_multiple_pads(mx6q_zeus_hdmi_ddc_pads,
			ARRAY_SIZE(mx6q_zeus_hdmi_ddc_pads));
}

static void hdmi_disable_ddc_pin(void)
{
	if (cpu_is_mx6dl())
		mxc_iomux_v3_setup_multiple_pads(mx6dl_zeus_i2c2_pads,
			ARRAY_SIZE(mx6dl_zeus_i2c2_pads));
	else
		mxc_iomux_v3_setup_multiple_pads(mx6q_zeus_i2c2_pads,
			ARRAY_SIZE(mx6q_zeus_i2c2_pads));
}

static struct fsl_mxc_hdmi_platform_data hdmi_data = {
	.init = hdmi_init,
	.enable_pins = hdmi_enable_ddc_pin,
	.disable_pins = hdmi_disable_ddc_pin,
};

static struct fsl_mxc_hdmi_core_platform_data hdmi_core_data = {
	.ipu_id = 0,
	.disp_id = 0,
};

static struct fsl_mxc_lcd_platform_data lcdif_data = {
	.ipu_id = 0,
	.disp_id = 0,
	.default_ifmt = IPU_PIX_FMT_RGB565,
};

static struct fsl_mxc_ldb_platform_data ldb_data = {
	.ipu_id = 1,
	.disp_id = 1,
	.ext_ref = 1,
	.mode = LDB_SEP1,
	.sec_ipu_id = 1,
	.sec_disp_id = 0,
};

static struct imx_ipuv3_platform_data ipu_data[] = {
	{
	.rev = 4,
	.csi_clk[0] = "clko_clk",
	.bypass_reset = false,
	}, {
	.rev = 4,
	.csi_clk[0] = "clko_clk",
	.bypass_reset = false,
	},
};

static struct fsl_mxc_capture_platform_data capture_data[] = {
	{
		.csi = 0,
		.ipu = 0,
		.mclk_source = 0,
		.is_mipi = 0,
	}, {
		.csi = 1,
		.ipu = 0,
		.mclk_source = 0,
		.is_mipi = 1,
	},
};


static void zeus_suspend_enter(void)
{
	/* suspend preparation */
	/* Disable LVDS 5V */
	gpio_set_value(ZEUS_LVDS_5V_PWREN, 0);
	//gpio_set_value(ZEUS_LVDS_3V3_PWREN, 0);
}

static void zeus_suspend_exit(void)
{
	/* resume restore */
	/* Enable LVDS 5V */
	gpio_set_value(ZEUS_LVDS_5V_PWREN, 1);
	//gpio_set_value(ZEUS_LVDS_3V3_PWREN, 1);
}
static const struct pm_platform_data mx6q_zeus_pm_data __initconst = {
	.name = "imx_pm",
	.suspend_enter = zeus_suspend_enter,
	.suspend_exit = zeus_suspend_exit,
};

static struct regulator_consumer_supply zeus_vmmc_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.1"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.2"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.3"),
};

static struct regulator_init_data zeus_vmmc_init = {
	.num_consumer_supplies = ARRAY_SIZE(zeus_vmmc_consumers),
	.consumer_supplies = zeus_vmmc_consumers,
};

static struct fixed_voltage_config zeus_vmmc_reg_config = {
	.supply_name		= "vmmc",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &zeus_vmmc_init,
};

static struct platform_device zeus_vmmc_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 3,
	.dev	= {
		.platform_data = &zeus_vmmc_reg_config,
	},
};

#ifdef CONFIG_SND_SOC_SGTL5000

static struct regulator_consumer_supply sgtl5000_zeus_consumer_vdda = {
	.supply = "VDDA",
	.dev_name = "1-000a",
};

static struct regulator_consumer_supply sgtl5000_zeus_consumer_vddio = {
	.supply = "VDDIO",
	.dev_name = "1-000a",
};

static struct regulator_consumer_supply sgtl5000_zeus_consumer_vddd = {
	.supply = "VDDD",
	.dev_name = "1-000a",
};

static struct regulator_init_data sgtl5000_zeus_vdda_reg_initdata = {
	.num_consumer_supplies = 1,
	.consumer_supplies = &sgtl5000_zeus_consumer_vdda,
};

static struct regulator_init_data sgtl5000_zeus_vddio_reg_initdata = {
	.num_consumer_supplies = 1,
	.consumer_supplies = &sgtl5000_zeus_consumer_vddio,
};

static struct regulator_init_data sgtl5000_zeus_vddd_reg_initdata = {
	.num_consumer_supplies = 1,
	.consumer_supplies = &sgtl5000_zeus_consumer_vddd,
};

static struct fixed_voltage_config sgtl5000_zeus_vdda_reg_config = {
	.supply_name	= "VDDA",
	.microvolts	= 2500000,
	.gpio		= -1,
	.init_data	= &sgtl5000_zeus_vdda_reg_initdata,
};

static struct fixed_voltage_config sgtl5000_zeus_vddio_reg_config = {
	.supply_name	= "VDDIO",
	.microvolts	= 3300000,
	.gpio		= -1,
	.init_data	= &sgtl5000_zeus_vddio_reg_initdata,
};

static struct fixed_voltage_config sgtl5000_zeus_vddd_reg_config = {
	.supply_name	= "VDDD",
	.microvolts	= 0,
	.gpio		= -1,
	.init_data	= &sgtl5000_zeus_vddd_reg_initdata,
};

static struct platform_device sgtl5000_zeus_vdda_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 0,
	.dev	= {
		.platform_data = &sgtl5000_zeus_vdda_reg_config,
	},
};

static struct platform_device sgtl5000_zeus_vddio_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 1,
	.dev	= {
		.platform_data = &sgtl5000_zeus_vddio_reg_config,
	},
};

static struct platform_device sgtl5000_zeus_vddd_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 2,
	.dev	= {
		.platform_data = &sgtl5000_zeus_vddd_reg_config,
	},
};

#endif /* CONFIG_SND_SOC_SGTL5000 */

static int imx6q_init_audio(void)
{
	mxc_register_device(&mx6_zeus_audio_device, 
			    &mx6_zeus_audio_data);
	imx6q_add_imx_ssi(1, &mx6_zeus_ssi_pdata);
#ifdef CONFIG_SND_SOC_SGTL5000
	platform_device_register(&sgtl5000_zeus_vdda_reg_devices);
	platform_device_register(&sgtl5000_zeus_vddio_reg_devices);
	platform_device_register(&sgtl5000_zeus_vddd_reg_devices);
#endif
	return 0;
}
	
#ifndef CONFIG_IMX_PCIE
static void pcie_3v3_power(void)
{
	/* disable PCIE_3V3 first */
	gpio_request(ZEUS_PCIE_PWR_EN, "pcie_3v3_en");
	gpio_direction_output(ZEUS_PCIE_PWR_EN, 0);
	mdelay(10);
	/* enable PCIE_3V3 again */
	gpio_set_value(ZEUS_PCIE_PWR_EN, 1);
	gpio_free(ZEUS_PCIE_PWR_EN);
}

static void pcie_3v3_reset(void)
{
	/* reset miniPCIe */
	gpio_request(ZEUS_PCIE_RST_B_REVB, "pcie_reset_rebB");
	gpio_direction_output(ZEUS_PCIE_RST_B_REVB, 0);
	/* The PCI Express Mini CEM specification states that PREST# is
	deasserted minimum 1ms after 3.3vVaux has been applied and stable*/
	mdelay(1);
	gpio_set_value(ZEUS_PCIE_RST_B_REVB, 1);
	gpio_free(ZEUS_PCIE_RST_B_REVB);
}
#endif

#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
#define GPIO_BUTTON(gpio_num, ev_code, act_low, descr, wake, debounce)	\
{								\
	.gpio		= gpio_num,				\
	.type		= EV_KEY,				\
	.code		= ev_code,				\
	.active_low	= act_low,				\
	.desc		= "btn " descr,				\
	.wakeup		= wake,					\
	.debounce_interval = debounce,				\
}

static struct gpio_keys_button imx6q_buttons[] = {
	GPIO_BUTTON(ZEUS_VOLUME_UP, KEY_VOLUMEUP, 1, "volume-up", 0, 1),
	GPIO_BUTTON(ZEUS_VOLUME_DN, KEY_VOLUMEDOWN, 1, "volume-down", 0, 1),
	GPIO_BUTTON(ZEUS_POWER_OFF, KEY_POWER, 1, "power", 1, 1),
};

static struct gpio_keys_platform_data imx6q_button_data = {
	.buttons	= imx6q_buttons,
	.nbuttons	= ARRAY_SIZE(imx6q_buttons),
};

static struct platform_device imx6q_button_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.num_resources  = 0,
	.dev		= {
		.platform_data = &imx6q_button_data,
	}
};

static void __init imx6q_add_device_buttons(void)
{
	platform_device_register(&imx6q_button_device);
}
#else
static void __init imx6q_add_device_buttons(void) {}
#endif

static struct platform_pwm_backlight_data mx6_zeus_pwm_backlight_data = {
	.pwm_id = 0,
	.max_brightness = 248,
	.dft_brightness = 128,
	.pwm_period_ns = 50000,
};

static struct mxc_dvfs_platform_data zeus_dvfscore_data = {
	.reg_id = "VDDCORE",
	.soc_id	= "VDDSOC",
	.clk1_id = "cpu_clk",
	.clk2_id = "gpc_dvfs_clk",
	.gpc_cntr_offset = MXC_GPC_CNTR_OFFSET,
	.ccm_cdcr_offset = MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset = MXC_CCM_CACRR_OFFSET,
	.ccm_cdhipr_offset = MXC_CCM_CDHIPR_OFFSET,
	.prediv_mask = 0x1F800,
	.prediv_offset = 11,
	.prediv_val = 3,
	.div3ck_mask = 0xE0000000,
	.div3ck_offset = 29,
	.div3ck_val = 2,
	.emac_val = 0x08,
	.upthr_val = 25,
	.dnthr_val = 9,
	.pncthr_val = 33,
	.upcnt_val = 10,
	.dncnt_val = 10,
	.delay_time = 80,
};

static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
}

static struct mipi_csi2_platform_data mipi_csi2_pdata = {
	.ipu_id	 = 0,
	.csi_id = 1,
	.v_channel = 0,
	.lanes = 2,
	.dphy_clk = "mipi_pllref_clk",
	.pixel_clk = "emi_clk",
};

static int __init caam_setup(char *__unused)
{
	caam_enabled = 1;
	return 1;
}
early_param("caam", caam_setup);

#define SNVS_LPCR 0x38
static void mx6_snvs_poweroff(void)
{

	void __iomem *mx6_snvs_base =  MX6_IO_ADDRESS(MX6Q_SNVS_BASE_ADDR);
	u32 value;
	value = readl(mx6_snvs_base + SNVS_LPCR);
	/*set TOP and DP_EN bit*/
	writel(value | 0x60, mx6_snvs_base + SNVS_LPCR);
}

static const struct imx_pcie_platform_data mx6_zeus_pcie_data __initconst = {
	.pcie_pwr_en	= ZEUS_PCIE_PWR_EN,
	.pcie_rst	= ZEUS_PCIE_RST_B_REVB,
	//.pcie_wake_up	= ZEUS_PCIE_WAKE_B,
	.pcie_dis	= ZEUS_PCIE_DIS_B,
};

/*!
 * Board specific initialization.
 */
static void __init mx6_zeus_board_init(void)
{
	int i;
	int ret;
	struct clk *clko, *clko2;
	struct clk *new_parent;
	int rate;

	if (cpu_is_mx6q())
		mxc_iomux_v3_setup_multiple_pads(mx6q_zeus_pads,
			ARRAY_SIZE(mx6q_zeus_pads));
	else if (cpu_is_mx6dl()) {
		mxc_iomux_v3_setup_multiple_pads(mx6dl_zeus_pads,
			ARRAY_SIZE(mx6dl_zeus_pads));
	}

#ifdef CONFIG_FEC_1588
	/* Set GPIO_16 input for IEEE-1588 ts_clk and RMII reference clock
	 * For MX6 GPR1 bit21 meaning:
	 * Bit21:       0 - GPIO_16 pad output
	 *              1 - GPIO_16 pad input
	 */
	 mxc_iomux_set_gpr_register(1, 21, 1, 1);
#endif

	gp_reg_id = zeus_dvfscore_data.reg_id;
	soc_reg_id = zeus_dvfscore_data.soc_id;
	mx6q_zeus_init_uart();

	/*
	 * MX6DL/Solo only supports single IPU
	 * The following codes are used to change ipu id
	 * and display id information for MX6DL/Solo. Then
	 * register 1 IPU device and up to 2 displays for
	 * MX6DL/Solo
	 */
	if (cpu_is_mx6dl()) {
		ldb_data.ipu_id = 0;
		ldb_data.sec_ipu_id = 0;
	}
	imx6q_add_mxc_hdmi_core(&hdmi_core_data);

	imx6q_add_ipuv3(0, &ipu_data[0]);
	if (cpu_is_mx6q()) {
		imx6q_add_ipuv3(1, &ipu_data[1]);
		for (i = 0; i < 4 && i < ARRAY_SIZE(zeus_fb_data); i++)
			imx6q_add_ipuv3fb(i, &zeus_fb_data[i]);
	} else
		for (i = 0; i < 2 && i < ARRAY_SIZE(zeus_fb_data); i++)
			imx6q_add_ipuv3fb(i, &zeus_fb_data[i]);

	imx6q_add_vdoa();
	imx6q_add_mipi_dsi(&mipi_dsi_pdata);
	imx6q_add_lcdif(&lcdif_data);
	imx6q_add_ldb(&ldb_data);
	imx6q_add_v4l2_output(0);
	imx6q_add_v4l2_capture(0, &capture_data[0]);
	imx6q_add_v4l2_capture(1, &capture_data[1]);
	imx6q_add_mipi_csi2(&mipi_csi2_pdata);
	imx6q_add_imx_snvs_rtc();

	if (1 == caam_enabled)
		imx6q_add_imx_caam();

	imx6q_add_imx_i2c(1, &mx6q_zeus_i2c_data);
	imx6q_add_imx_i2c(2, &mx6q_zeus_i2c_data);
	i2c_register_board_info(1, mxc_i2c1_board_info,
			ARRAY_SIZE(mxc_i2c1_board_info));
	i2c_register_board_info(2, mxc_i2c2_board_info,
			ARRAY_SIZE(mxc_i2c2_board_info));
	ret = gpio_request(ZEUS_PFUZE_INT, "pFUZE-int");
	if (ret) {
		printk(KERN_ERR"request pFUZE-int error!!\n");
		return;
	} else {
		gpio_direction_input(ZEUS_PFUZE_INT);
		mx6q_sabresd_init_pfuze100(ZEUS_PFUZE_INT);
	}

	imx6q_add_mxc_hdmi(&hdmi_data);

	imx6q_add_anatop_thermal_imx(1, &mx6q_zeus_anatop_thermal_data);
	imx6_init_fec(fec_data);
#ifdef CONFIG_MX6_ENET_IRQ_TO_GPIO
	/* Make sure the IOMUX_OBSRV_MUX1 is set to ENET_IRQ. */
	mxc_iomux_set_specialbits_register(IOMUX_OBSRV_MUX1_OFFSET,
		OBSRV_MUX1_ENET_IRQ, OBSRV_MUX1_MASK);
#endif

	imx6q_add_pm_imx(0, &mx6q_zeus_pm_data);

	/* Move sd4 to first because sd4 connect to emmc.
	   Mfgtools want emmc is mmcblk0 and other sd card is mmcblk1.
	*/
#ifndef CONFIG_IMX_ZEUS_NAND	// disable SDHC which conflicts with NAND
	imx6q_add_sdhci_usdhc_imx(3, &mx6q_zeus_sd4_data);
#endif
	imx6q_add_sdhci_usdhc_imx(0, &mx6q_zeus_sd1_data);
	imx6q_add_sdhci_usdhc_imx(2, &mx6q_zeus_sd3_data);
	imx_add_viv_gpu(&imx6_gpu_data, &imx6q_gpu_pdata);
	imx6q_zeus_init_usb();
	/* SATA is not supported by MX6DL/Solo */
	if (cpu_is_mx6q()) {
#ifdef CONFIG_SATA_AHCI_PLATFORM
		imx6q_add_ahci(0, &mx6q_zeus_sata_data);
#else
		mx6q_zeus_sata_init(NULL,
			(void __iomem *)ioremap(MX6Q_SATA_BASE_ADDR, SZ_4K));
#endif
	}
	imx6q_add_vpu();
	imx6q_init_audio();
	platform_device_register(&zeus_vmmc_reg_devices);
	imx_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
	imx_asrc_data.asrc_audio_clk = clk_get(NULL, "asrc_serial_clk");
	imx6q_add_asrc(&imx_asrc_data);

	/*
	 * Disable HannStar touch panel CABC function,
	 * this function turns the panel's backlight automatically
	 * according to the content shown on the panel which
	 * may cause annoying unstable backlight issue.
	 */
	gpio_request(ZEUS_CABC_EN0, "cabc-en0");
	gpio_direction_output(ZEUS_CABC_EN0, 0);
	gpio_request(ZEUS_CABC_EN1, "cabc-en1");
	gpio_direction_output(ZEUS_CABC_EN1, 0);

	imx6q_add_mxc_pwm(0);
	imx6q_add_mxc_pwm(1);
	imx6q_add_mxc_pwm(2);
	imx6q_add_mxc_pwm(3);
	imx6q_add_mxc_pwm_backlight(0, &mx6_zeus_pwm_backlight_data);

	imx6q_add_otp();
	imx6q_add_viim();
	imx6q_add_imx2_wdt(0, NULL);
	imx6q_add_dma();

#ifdef CONFIG_IMX_ZEUS_NAND
	imx6q_add_gpmi(&mx6q_gpmi_nand_platform_data);
#endif

	imx6q_add_dvfs_core(&zeus_dvfscore_data);
	imx6q_add_device_buttons();

	imx6q_add_hdmi_soc();
	imx6q_add_hdmi_soc_dai();

	if (cpu_is_mx6dl()) {
		imx6dl_add_imx_pxp();
		imx6dl_add_imx_pxp_client();
	}

	clko2 = clk_get(NULL, "clko2_clk");
	if (IS_ERR(clko2))
		pr_err("can't get CLKO2 clock.\n");

	new_parent = clk_get(NULL, "osc_clk");
	if (!IS_ERR(new_parent)) {
		clk_set_parent(clko2, new_parent);
		clk_put(new_parent);
	}
	rate = clk_round_rate(clko2, 24000000);
	clk_set_rate(clko2, rate);
	clk_enable(clko2);

	/* Camera and audio use osc clock */
	clko = clk_get(NULL, "clko_clk");
	if (!IS_ERR(clko))
		clk_set_parent(clko, clko2);

	/* Enable LVDS_5V */
	gpio_request(ZEUS_LVDS_5V_PWREN, "lvds_5v_pwren");
	gpio_direction_output(ZEUS_LVDS_5V_PWREN, 1);
	gpio_set_value(ZEUS_LVDS_5V_PWREN, 1);

#ifndef CONFIG_IMX_PCIE
	/* enable pcie 3v3 power without pcie driver */
	pcie_3v3_power();
	mdelay(10);
	pcie_3v3_reset();
#endif

	pm_power_off = mx6_snvs_poweroff;
	imx6q_add_busfreq();

	/* Add PCIe RC interface support */
	imx6q_add_pcie(&mx6_zeus_pcie_data);

	imx6_add_armpmu();
	imx6q_add_perfmon(0);
	imx6q_add_perfmon(1);
	imx6q_add_perfmon(2);
}

extern void __iomem *twd_base;
static void __init mx6_zeus_timer_init(void)
{
	struct clk *uart_clk;
#ifdef CONFIG_LOCAL_TIMERS
	twd_base = ioremap(LOCAL_TWD_ADDR, SZ_256);
	BUG_ON(!twd_base);
#endif
	mx6_clocks_init(32768, 24000000, 0, 0);

	uart_clk = clk_get_sys("imx-uart.0", NULL);
	early_console_setup(UART4_BASE_ADDR, uart_clk);
}

static struct sys_timer mx6_zeus_timer = {
	.init   = mx6_zeus_timer_init,
};

static void __init mx6q_zeus_reserve(void)
{
#if defined(CONFIG_MXC_GPU_VIV) || defined(CONFIG_MXC_GPU_VIV_MODULE)
	phys_addr_t phys;

	if (imx6q_gpu_pdata.reserved_mem_size) {
		phys = memblock_alloc_base(imx6q_gpu_pdata.reserved_mem_size,
					   SZ_4K, SZ_1G);
		memblock_remove(phys, imx6q_gpu_pdata.reserved_mem_size);
		imx6q_gpu_pdata.reserved_mem_base = phys;
	}
#endif
}

/*
 * initialize __mach_desc_MX6Q_ZEUS data structure.
 */
MACHINE_START(MX6Q_ZEUS, "Freescale i.MX 6Quad/DualLite/Solo ZEUS Board")
	/* Maintainer: Freescale Semiconductor, Inc. */
	.boot_params = MX6_PHYS_OFFSET + 0x100,
	.fixup = fixup_mxc_board,
	.map_io = mx6_map_io,
	.init_irq = mx6_init_irq,
	.init_machine = mx6_zeus_board_init,
	.timer = &mx6_zeus_timer,
	.reserve = mx6q_zeus_reserve,
MACHINE_END
