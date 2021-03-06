/*
 *  Copyright (C) 2013, Loongson Technology Corporation Limited, Inc.
 *
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 *
 */
#ifndef _LOONGSON_PCH_H
#define _LOONGSON_PCH_H

#include <linux/types.h>
#include <linux/pci.h>
#include <asm/addrspace.h>
#include <linux/msi.h>

#define LS2H_PCH_REG_BASE		0x1b000000

/* CHIP CONFIG regs */
#define LS2H_CHIPCFG_REG_BASE		(LS2H_PCH_REG_BASE + 0x00d00000)

#define LS2H_INT_REG_BASE		(LS2H_CHIPCFG_REG_BASE + 0x0040)

#define LS2H_INT_ISR0_REG		(LS2H_CHIPCFG_REG_BASE + 0x0040)
#define LS2H_INT_IEN0_REG		(LS2H_CHIPCFG_REG_BASE + 0x0044)
#define LS2H_INT_SET0_REG		(LS2H_CHIPCFG_REG_BASE + 0x0048)
#define LS2H_INT_CLR0_REG		(LS2H_CHIPCFG_REG_BASE + 0x004c)
#define LS2H_INT_POL0_REG		(LS2H_CHIPCFG_REG_BASE + 0x0050)
#define LS2H_INT_EDGE0_REG		(LS2H_CHIPCFG_REG_BASE + 0x0054)

#define LS2H_GPIO_CFG_REG		(LS2H_CHIPCFG_REG_BASE + 0x00c0)
#define LS2H_GPIO_OE_REG		(LS2H_CHIPCFG_REG_BASE + 0x00c4)
#define LS2H_GPIO_IN_REG		(LS2H_CHIPCFG_REG_BASE + 0x00c8)
#define LS2H_GPIO_OUT_REG		(LS2H_CHIPCFG_REG_BASE + 0x00cc)

#define LS2H_DMA_ORDER_REG		(LS2H_CHIPCFG_REG_BASE + 0x0100)
#define LS2H_CHIP_CFG0_REG		(LS2H_CHIPCFG_REG_BASE + 0x0200)
#define LS2H_CHIP_CFG1_REG		(LS2H_CHIPCFG_REG_BASE + 0x0204)
#define LS2H_CHIP_CFG2_REG		(LS2H_CHIPCFG_REG_BASE + 0x0208)
#define LS2H_CHIP_CFG3_REG		(LS2H_CHIPCFG_REG_BASE + 0x020c)
#define LS2H_CHIP_SAMP0_REG		(LS2H_CHIPCFG_REG_BASE + 0x0210)
#define LS2H_CHIP_SAMP1_REG		(LS2H_CHIPCFG_REG_BASE + 0x0214)
#define LS2H_CHIP_SAMP2_REG		(LS2H_CHIPCFG_REG_BASE + 0x0218)
#define LS2H_CHIP_SAMP3_REG		(LS2H_CHIPCFG_REG_BASE + 0x021c)
#define LS2H_CLK_CTRL0_REG		(LS2H_CHIPCFG_REG_BASE + 0x0220)
#define LS2H_CLK_CTRL1_REG		(LS2H_CHIPCFG_REG_BASE + 0x0224)
#define LS2H_CLK_CTRL2_REG		(LS2H_CHIPCFG_REG_BASE + 0x0228)
#define LS2H_CLK_CTRL3_REG		(LS2H_CHIPCFG_REG_BASE + 0x022c)
#define LS2H_PIXCLK0_CTRL0_REG		(LS2H_CHIPCFG_REG_BASE + 0x0230)
#define LS2H_PIXCLK0_CTRL1_REG		(LS2H_CHIPCFG_REG_BASE + 0x0234)
#define LS2H_PIXCLK1_CTRL0_REG		(LS2H_CHIPCFG_REG_BASE + 0x0238)
#define LS2H_PIXCLK1_CTRL1_REG		(LS2H_CHIPCFG_REG_BASE + 0x023c)

#define LS2H_WIN_CFG_BASE		(LS2H_CHIPCFG_REG_BASE + 0x80000)
#define LS2H_M1_WIN4_BASE_REG		(LS2H_WIN_CFG_BASE + 0x0120)
#define LS2H_M1_WIN4_MASK_REG		(LS2H_WIN_CFG_BASE + 0x0160)
#define LS2H_M1_WIN4_MMAP_REG		(LS2H_WIN_CFG_BASE + 0x01a0)
#define LS2H_M1_WIN6_BASE_REG		(LS2H_WIN_CFG_BASE + 0x0130)
#define LS2H_M1_WIN6_MASK_REG		(LS2H_WIN_CFG_BASE + 0x0170)
#define LS2H_M1_WIN6_MMAP_REG		(LS2H_WIN_CFG_BASE + 0x01b0)
#define LS2H_M4_WIN0_BASE_REG		(LS2H_WIN_CFG_BASE + 0x0400)
#define LS2H_M4_WIN0_MASK_REG		(LS2H_WIN_CFG_BASE + 0x0440)
#define LS2H_M4_WIN0_MMAP_REG		(LS2H_WIN_CFG_BASE + 0x0480)

/* USB regs */
#define LS2H_EHCI_REG_BASE		(LS2H_PCH_REG_BASE + 0x00e00000)
#define LS2H_OHCI_REG_BASE		(LS2H_PCH_REG_BASE + 0x00e08000)

/* GMAC regs */
#define LS2H_GMAC0_REG_BASE		(LS2H_PCH_REG_BASE + 0x00e10000)
#define LS2H_GMAC1_REG_BASE		(LS2H_PCH_REG_BASE + 0x00e18000)

/* HDA regs */
#define LS2H_HDA_REG_BASE		(LS2H_PCH_REG_BASE + 0x00e20000)

/* SATAregs */
#define LS2H_SATA_REG_BASE		(LS2H_PCH_REG_BASE + 0x00e30000)

/* GPU regs */
#define LS2H_GPU_REG_BASE		(LS2H_PCH_REG_BASE + 0x00e40000)

/* DC regs */
#define LS2H_DC_REG_BASE		(LS2H_PCH_REG_BASE + 0x00e50000)

#define LS2H_FB_CFG_DVO_REG		(LS2H_DC_REG_BASE + 0x1240)
#define LS2H_FB_CFG_VGA_REG		(LS2H_DC_REG_BASE + 0x1250)
#define LS2H_FB_ADDR0_DVO_REG		(LS2H_DC_REG_BASE + 0x1260)
#define LS2H_FB_ADDR0_VGA_REG		(LS2H_DC_REG_BASE + 0x1270)
#define LS2H_FB_STRI_DVO_REG		(LS2H_DC_REG_BASE + 0x1280)
#define LS2H_FB_STRI_VGA_REG		(LS2H_DC_REG_BASE + 0x1290)

#define LS2H_FB_DITCFG_DVO_REG		(LS2H_DC_REG_BASE + 0x1360)
#define LS2H_FB_DITCFG_VGA_REG		(LS2H_DC_REG_BASE + 0x1370)
#define LS2H_FB_DITTAB_LO_DVO_REG	(LS2H_DC_REG_BASE + 0x1380)
#define LS2H_FB_DITTAB_LO_VGA_REG	(LS2H_DC_REG_BASE + 0x1390)
#define LS2H_FB_DITTAB_HI_DVO_REG	(LS2H_DC_REG_BASE + 0x13a0)
#define LS2H_FB_DITTAB_HI_VGA_REG	(LS2H_DC_REG_BASE + 0x13b0)
#define LS2H_FB_PANCFG_DVO_REG		(LS2H_DC_REG_BASE + 0x13c0)
#define LS2H_FB_PANCFG_VGA_REG		(LS2H_DC_REG_BASE + 0x13d0)
#define LS2H_FB_PANTIM_DVO_REG		(LS2H_DC_REG_BASE + 0x13e0)
#define LS2H_FB_PANTIM_VGA_REG		(LS2H_DC_REG_BASE + 0x13f0)

#define LS2H_FB_HDISPLAY_DVO_REG	(LS2H_DC_REG_BASE + 0x1400)
#define LS2H_FB_HDISPLAY_VGA_REG	(LS2H_DC_REG_BASE + 0x1410)
#define LS2H_FB_HSYNC_DVO_REG		(LS2H_DC_REG_BASE + 0x1420)
#define LS2H_FB_HSYNC_VGA_REG		(LS2H_DC_REG_BASE + 0x1430)

#define LS2H_FB_VDISPLAY_DVO_REG	(LS2H_DC_REG_BASE + 0x1480)
#define LS2H_FB_VDISPLAY_VGA_REG	(LS2H_DC_REG_BASE + 0x1490)
#define LS2H_FB_VSYNC_DVO_REG		(LS2H_DC_REG_BASE + 0x14a0)
#define LS2H_FB_VSYNC_VGA_REG		(LS2H_DC_REG_BASE + 0x14b0)

#define LS2H_FB_GAMINDEX_DVO_REG	(LS2H_DC_REG_BASE + 0x14e0)
#define LS2H_FB_GAMINDEX_VGA_REG	(LS2H_DC_REG_BASE + 0x14f0)
#define LS2H_FB_GAMDATA_DVO_REG		(LS2H_DC_REG_BASE + 0x1500)
#define LS2H_FB_GAMDATA_VGA_REG		(LS2H_DC_REG_BASE + 0x1510)

#define LS2H_FB_CUR_CFG_REG		(LS2H_DC_REG_BASE + 0x1520)
#define LS2H_FB_CUR_ADDR_REG		(LS2H_DC_REG_BASE + 0x1530)
#define LS2H_FB_CUR_LOC_ADDR_REG	(LS2H_DC_REG_BASE + 0x1540)
#define LS2H_FB_CUR_BACK_REG		(LS2H_DC_REG_BASE + 0x1550)
#define LS2H_FB_CUR_FORE_REG		(LS2H_DC_REG_BASE + 0x1560)

#define LS2H_FB_INT_REG			(LS2H_DC_REG_BASE + 0x1570)

#define LS2H_FB_ADDR1_DVO_REG		(LS2H_DC_REG_BASE + 0x1580)
#define LS2H_FB_ADDR1_VGA_REG		(LS2H_DC_REG_BASE + 0x1590)

#define LS2H_FB_DAC_CTRL_REG		(LS2H_DC_REG_BASE + 0x1600)
#define LS2H_FB_DVO_OUTPUT_REG		(LS2H_DC_REG_BASE + 0x1630)

/* OTG regs */
#define LS2H_OTG_REG_BASE		(LS2H_PCH_REG_BASE + 0x00e60000)

/* SPI regs */
#define LS2H_SPI_REG_BASE		(LS2H_PCH_REG_BASE + 0x00e70000)

/* UART regs */
#define LS2H_UART0_REG_BASE		(LS2H_PCH_REG_BASE + 0x00e80000)
#define LS2H_UART1_REG_BASE		(LS2H_PCH_REG_BASE + 0x00e81000)
#define LS2H_UART2_REG_BASE		(LS2H_PCH_REG_BASE + 0x00e82000)
#define LS2H_UART3_REG_BASE		(LS2H_PCH_REG_BASE + 0x00e83000)

/* I2C regs */
#define LS2H_I2C0_REG_BASE		(LS2H_PCH_REG_BASE + 0x00e90000)
#define LS2H_I2C0_PRER_LO_REG		(LS2H_I2C0_REG_BASE + 0x0)
#define LS2H_I2C0_PRER_HI_REG		(LS2H_I2C0_REG_BASE + 0x1)
#define LS2H_I2C0_CTR_REG   		(LS2H_I2C0_REG_BASE + 0x2)
#define LS2H_I2C0_TXR_REG   		(LS2H_I2C0_REG_BASE + 0x3)
#define LS2H_I2C0_RXR_REG    		(LS2H_I2C0_REG_BASE + 0x3)
#define LS2H_I2C0_CR_REG     		(LS2H_I2C0_REG_BASE + 0x4)
#define LS2H_I2C0_SR_REG     		(LS2H_I2C0_REG_BASE + 0x4)

#define LS2H_I2C1_REG_BASE		(LS2H_PCH_REG_BASE + 0x00e91000)
#define LS2H_I2C1_PRER_LO_REG		(LS2H_I2C1_REG_BASE + 0x0)
#define LS2H_I2C1_PRER_HI_REG		(LS2H_I2C1_REG_BASE + 0x1)
#define LS2H_I2C1_CTR_REG    		(LS2H_I2C1_REG_BASE + 0x2)
#define LS2H_I2C1_TXR_REG    		(LS2H_I2C1_REG_BASE + 0x3)
#define LS2H_I2C1_RXR_REG    		(LS2H_I2C1_REG_BASE + 0x3)
#define LS2H_I2C1_CR_REG     		(LS2H_I2C1_REG_BASE + 0x4)
#define LS2H_I2C1_SR_REG     		(LS2H_I2C1_REG_BASE + 0x4)

#define	CR_START			0x80
#define	CR_STOP				0x40
#define	CR_READ				0x20
#define	CR_WRITE			0x10
#define	CR_ACK				0x8
#define	CR_IACK				0x1

#define	SR_NOACK			0x80
#define	SR_BUSY				0x40
#define	SR_AL				0x20
#define	SR_TIP				0x2
#define	SR_IF				0x1

/* PWM regs */
#define LS2H_PWM_REG_BASE		(LS2H_PCH_REG_BASE + 0x00ea0000)

/* HPET regs */
#define LS2H_HPET_REG_BASE		(LS2H_PCH_REG_BASE + 0x00ec0000)

/* AC97 regs */
#define LS2H_AC97_REG_BASE		(LS2H_PCH_REG_BASE + 0x00ed0000)

/* NAND regs */
#define LS2H_NAND_REG_BASE		(LS2H_PCH_REG_BASE + 0x00ee0000)
#define LS2H_NAND_CMD_REG		(LS2H_NAND_REG_BASE + 0x0000)
#define LS2H_NAND_ADDR_C_REG		(LS2H_NAND_REG_BASE + 0x0004)
#define LS2H_NAND_ADDR_R_REG		(LS2H_NAND_REG_BASE + 0x0008)
#define LS2H_NAND_TIMING_REG		(LS2H_NAND_REG_BASE + 0x000c)
#define LS2H_NAND_IDL_REG		(LS2H_NAND_REG_BASE + 0x0010)
#define LS2H_NAND_STA_IDH_REG		(LS2H_NAND_REG_BASE + 0x0014)
#define LS2H_NAND_PARAM_REG		(LS2H_NAND_REG_BASE + 0x0018)
#define LS2H_NAND_OP_NUM_REG		(LS2H_NAND_REG_BASE + 0x001c)
#define LS2H_NAND_CSRDY_MAP_REG		(LS2H_NAND_REG_BASE + 0x0020)
#define LS2H_NAND_DMA_ACC_REG		(LS2H_NAND_REG_BASE + 0x0040)

/* ACPI regs */
#define LS2H_ACPI_REG_BASE		(LS2H_PCH_REG_BASE + 0x00ef0000)
#define LS2H_PM_SOC_REG			(LS2H_ACPI_REG_BASE + 0x0000)
#define LS2H_PM_RESUME_REG		(LS2H_ACPI_REG_BASE + 0x0004)
#define LS2H_PM_RTC_REG			(LS2H_ACPI_REG_BASE + 0x0008)
#define LS2H_PM1_STS_REG		(LS2H_ACPI_REG_BASE + 0x000c)
#define LS2H_PM1_EN_REG			(LS2H_ACPI_REG_BASE + 0x0010)
#define LS2H_PM1_CNT_REG		(LS2H_ACPI_REG_BASE + 0x0014)
#define LS2H_PM1_TMR_REG		(LS2H_ACPI_REG_BASE + 0x0018)
#define LS2H_P_CNT_REG			(LS2H_ACPI_REG_BASE + 0x001c)
#define LS2H_P_LVL2_REG			(LS2H_ACPI_REG_BASE + 0x0020)
#define LS2H_P_LVL3_REG			(LS2H_ACPI_REG_BASE + 0x0024)
#define LS2H_GPE0_STS_REG		(LS2H_ACPI_REG_BASE + 0x0028)
#define LS2H_GPE0_EN_REG		(LS2H_ACPI_REG_BASE + 0x002c)
#define LS2H_RST_CNT_REG		(LS2H_ACPI_REG_BASE + 0x0030)
#define LS2H_WD_SET_REG			(LS2H_ACPI_REG_BASE + 0x0034)
#define LS2H_WD_TIMER_REG		(LS2H_ACPI_REG_BASE + 0x0038)
#define LS2H_DVFS_CNT_REG		(LS2H_ACPI_REG_BASE + 0x003c)
#define LS2H_DVFS_STS_REG		(LS2H_ACPI_REG_BASE + 0x0040)
#define LS2H_MS_CNT_REG			(LS2H_ACPI_REG_BASE + 0x0044)
#define LS2H_MS_THT_REG			(LS2H_ACPI_REG_BASE + 0x0048)
#define	LS2H_THSENS_CNT_REG		(LS2H_ACPI_REG_BASE + 0x004c)
#define LS2H_GEN_RTC1_REG		(LS2H_ACPI_REG_BASE + 0x0050)
#define LS2H_GEN_RTC2_REG		(LS2H_ACPI_REG_BASE + 0x0054)

/* RTC regs */
#define LS2H_RTC_REG_BASE		(LS2H_PCH_REG_BASE + 0x00ef8000)
#define	LS2H_TOY_TRIM_REG		(LS2H_RTC_REG_BASE + 0x0020)
#define	LS2H_TOY_WRITE0_REG		(LS2H_RTC_REG_BASE + 0x0024)
#define	LS2H_TOY_WRITE1_REG		(LS2H_RTC_REG_BASE + 0x0028)
#define	LS2H_TOY_READ0_REG		(LS2H_RTC_REG_BASE + 0x002c)
#define	LS2H_TOY_READ1_REG		(LS2H_RTC_REG_BASE + 0x0030)
#define	LS2H_TOY_MATCH0_REG		(LS2H_RTC_REG_BASE + 0x0034)
#define	LS2H_TOY_MATCH1_REG		(LS2H_RTC_REG_BASE + 0x0038)
#define	LS2H_TOY_MATCH2_REG		(LS2H_RTC_REG_BASE + 0x003c)
#define	LS2H_RTC_CTRL_REG		(LS2H_RTC_REG_BASE + 0x0040)
#define	LS2H_RTC_TRIM_REG		(LS2H_RTC_REG_BASE + 0x0060)
#define	LS2H_RTC_WRITE0_REG		(LS2H_RTC_REG_BASE + 0x0064)
#define	LS2H_RTC_READ0_REG		(LS2H_RTC_REG_BASE + 0x0068)
#define	LS2H_RTC_MATCH0_REG		(LS2H_RTC_REG_BASE + 0x006c)
#define	LS2H_RTC_MATCH1_REG		(LS2H_RTC_REG_BASE + 0x0070)
#define	LS2H_RTC_MATCH2_REG		(LS2H_RTC_REG_BASE + 0x0074)

/* LPC regs */
#define LS3_LPC_REG_BASE		0x1fe00200
#define LS2H_LPC_IO_BASE		(LS2H_PCH_REG_BASE + 0x00f00000)
#define LS2H_LPC_REG_BASE		(LS2H_PCH_REG_BASE + 0x00f10000)
extern u64 ls_lpc_reg_base;
#define LS_LPC_CFG0_REG			(ls_lpc_reg_base + 0x0)
#define LS_LPC_CFG1_REG			(ls_lpc_reg_base + 0x4)
#define LS_LPC_CFG2_REG			(ls_lpc_reg_base + 0x8)
#define LS_LPC_CFG3_REG			(ls_lpc_reg_base + 0xc)
#define LS_LPC_CFG4_REG			(ls_lpc_reg_base + 0x10)
#define LS_LPC_INT_CTL			LS_LPC_CFG0_REG
#define LS_LPC_INT_ENA			LS_LPC_CFG1_REG
#define LS_LPC_INT_STS			LS_LPC_CFG2_REG
#define LS_LPC_INT_CLR			LS_LPC_CFG3_REG
#define LS_LPC_INT_POL			LS_LPC_CFG4_REG
#define LS2H_LPC_CFG0_REG		(LS2H_LPC_REG_BASE + 0x0)
#define LS2H_LPC_CFG1_REG		(LS2H_LPC_REG_BASE + 0x4)
#define LS2H_LPC_CFG2_REG		(LS2H_LPC_REG_BASE + 0x8)
#define LS2H_LPC_CFG3_REG		(LS2H_LPC_REG_BASE + 0xc)
#define LS2H_LPC_INT_CTL		LS2H_LPC_CFG0_REG
#define LS2H_LPC_INT_ENA		LS2H_LPC_CFG1_REG
#define LS2H_LPC_INT_STS		LS2H_LPC_CFG2_REG
#define LS2H_LPC_INT_CLR		LS2H_LPC_CFG3_REG

/* REG ACCESS*/
#define ls2h_readb(addr)		(*(volatile u8 *)CKSEG1ADDR(addr))
#define ls2h_readw(addr)		(*(volatile u16 *)CKSEG1ADDR(addr))
#define ls2h_readl(addr)		(*(volatile u32 *)CKSEG1ADDR(addr))
#define ls2h_writeb(val, addr)		*(volatile u8 *)CKSEG1ADDR(addr) = (val)
#define ls2h_writew(val, addr)		*(volatile u16 *)CKSEG1ADDR(addr) = (val)
#define ls2h_writel(val, addr)		*(volatile u32 *)CKSEG1ADDR(addr) = (val)
/* Board Version Number */
enum {
	LS2H_BOARD_VER_2_2 = 0x4,
	LS2H_BOARD_VER_OLD = 0xf,
};

enum {
	LS3A2H_BOARD_VER_2_2 = 0x4,
	LS3A2H_BOARD_VER_OLD = 0xf,
};


#define LS2H_PCIE_PORT0             0
#define LS2H_PCIE_PORT1             1
#define LS2H_PCIE_PORT2             2
#define LS2H_PCIE_PORT3             3
#define LS2H_PCIE_MAX_PORTNUM       3

#define LS2H_CHIP_CFG_REG_CLK_CTRL3     0x22c
#define LS2H_CLK_CTRL3_BIT_PEREF_EN(portnum) (1 << (24 + portnum))

#define LS2H_PCIE_MEM0_BASE(portnum)       (0x10000000 + (portnum << 25))
#define LS2H_PCIE_MEM1_BASE(portnum)       (0x40000000 + (portnum << 28))
#define LS2H_PCIE_IO_BASE(portnum)         (0x18100000 + (portnum << 22))
#define LS2H_PCIE_PORT_HEAD_BASE(portnum)  (0x18114000 + (portnum << 22))
#define LS2H_PCIE_DEV_HEAD_BASE(portnum)   (0x18116000 + (portnum << 22))
#define LS2H_PCIE_PORT_REG_BASE(portnum)   (0x18118000 + (portnum << 22))
#define LS2H_PCIE_PORT_REG_CTR0			0x0
#define  LS2H_PCIE_REG_CTR0_BIT_LTSSM_EN	(1 << 3)
#define  LS2H_PCIE_REG_CTR0_BIT_REQ_L1		(1 << 12)
#define  LS2H_PCIE_REG_CTR0_BIT_RDY_L23		(1 << 13)
#define LS2H_PCIE_PORT_REG_CTR1			0x4
#define LS2H_PCIE_PORT_REG_STAT0		0x8
#define LS2H_PCIE_PORT_REG_STAT1		0xc
#define  LS2H_PCIE_REG_STAT1_MASK_LTSSM		0x0000003f
#define  LS2H_PCIE_REG_STAT1_BIT_LINKUP		(1 << 6)
#define LS2H_PCIE_PORT_REG_INTSTS		0x18
#define LS2H_PCIE_PORT_REG_INTCLR		0x1c
#define LS2H_PCIE_PORT_REG_INTMSK		0x20
#define LS2H_PCIE_PORT_REG_CFGADDR		0x24
#define LS2H_PCIE_PORT_REG_CTR_STAT		0x28
#define  LS2H_PCIE_REG_CTR_STAT_BIT_ISX4	(1 << 26)
#define  LS2H_PCIE_REG_CTR_STAT_BIT_ISRC	(1 << 27)
#define LS2H_PCI_EXP_LNKCAP			0x7c


/* LS7A PCH Registers (Misc, Confreg) */

#define LS7A_HT1_BASE 0x90000e0000000000
#define LS7A_PCH_REG_BASE			0x10000000
/* CHIPCFG regs */
#define LS7A_CHIPCFG_REG_BASE 	(LS7A_PCH_REG_BASE + 0x0a000000)
/* MISC reg base */
#define LS7A_MISC_REG_BASE		(LS7A_PCH_REG_BASE + 0x00080000)
/* ACPI regs */
#define LS7A_ACPI_REG_BASE      (LS7A_MISC_REG_BASE + 0x00050000)

/* UART regs */
#define LS7A_UART0_REG_BASE		(LS7A_MISC_REG_BASE + 0x00000000)
#define LS7A_UART1_REG_BASE		(LS7A_MISC_REG_BASE + 0x00000100)
#define LS7A_UART2_REG_BASE		(LS7A_MISC_REG_BASE + 0x00000200)
#define LS7A_UART3_REG_BASE		(LS7A_MISC_REG_BASE + 0x00000300)

/* RTC ram addr */
#define LS7A_RTC_RAM	(LS7A_ACPI_REG_BASE + 0x00000050)

/* I2C regs */
#define LS7A_I2C0_REG_BASE		(LS7A_MISC_REG_BASE + 0x00010000)
#define LS7A_I2C1_REG_BASE		(LS7A_MISC_REG_BASE + 0x00010100)
#define LS7A_I2C2_REG_BASE		(LS7A_MISC_REG_BASE + 0x00010200)
#define LS7A_I2C3_REG_BASE		(LS7A_MISC_REG_BASE + 0x00010300)
#define LS7A_I2C4_REG_BASE		(LS7A_MISC_REG_BASE + 0x00010400)
#define LS7A_I2C5_REG_BASE		(LS7A_MISC_REG_BASE + 0x00010500)

/* HPET */
#define LS7A_HPET_REG_BASE    (LS7A_MISC_REG_BASE + 0x00040000)

/* RTC regs */
#define LS7A_RTC_REG_BASE		(LS7A_MISC_REG_BASE + 0x00050100)

/* DC regs */
#define LS7A_DC_REG_BASE		(LS7A_MISC_REG_BASE + 0x00050000)

#define LS7A_FB_CFG_DVO0_REG		(0x1240)
#define LS7A_FB_CFG_DVO1_REG		(0x1250)
#define LS7A_FB_ADDR0_DVO0_REG		(0x1260)
#define LS7A_FB_ADDR0_DVO1_REG		(0x1270)
#define LS7A_FB_STRI_DVO0_REG		(0x1280)
#define LS7A_FB_STRI_DVO1_REG		(0x1290)

#define LS7A_FB_DITCFG_DVO0_REG		(0x1360)
#define LS7A_FB_DITCFG_DVO1_REG		(0x1370)
#define LS7A_FB_DITTAB_LO_DVO0_REG	(0x1380)
#define LS7A_FB_DITTAB_LO_DVO1_REG	(0x1390)
#define LS7A_FB_DITTAB_HI_DVO0_REG	(0x13a0)
#define LS7A_FB_DITTAB_HI_DVO1_REG	(0x13b0)
#define LS7A_FB_PANCFG_DVO0_REG		(0x13c0)
#define LS7A_FB_PANCFG_DVO1_REG		(0x13d0)
#define LS7A_FB_PANTIM_DVO0_REG		(0x13e0)
#define LS7A_FB_PANTIM_DVO1_REG		(0x13f0)

#define LS7A_FB_HDISPLAY_DVO0_REG	(0x1400)
#define LS7A_FB_HDISPLAY_DVO1_REG	(0x1410)
#define LS7A_FB_HSYNC_DVO0_REG		(0x1420)
#define LS7A_FB_HSYNC_DVO1_REG		(0x1430)

#define LS7A_FB_VDISPLAY_DVO0_REG	(0x1480)
#define LS7A_FB_VDISPLAY_DVO1_REG	(0x1490)
#define LS7A_FB_VSYNC_DVO0_REG		(0x14a0)
#define LS7A_FB_VSYNC_DVO1_REG		(0x14b0)

#define LS7A_FB_GAMINDEX_DVO0_REG	(0x14e0)
#define LS7A_FB_GAMINDEX_DVO1_REG	(0x14f0)
#define LS7A_FB_GAMDATA_DVO0_REG	(0x1500)
#define LS7A_FB_GAMDATA_DVO1_REG	(0x1510)

#define LS7A_FB_CUR_CFG_REG		(0x1520)
#define LS7A_FB_CUR_ADDR_REG		(0x1530)
#define LS7A_FB_CUR_LOC_ADDR_REG	(0x1540)
#define LS7A_FB_CUR_BACK_REG		(0x1550)
#define LS7A_FB_CUR_FORE_REG		(0x1560)

#define LS7A_FB_INT_REG			(0x1570)

#define LS7A_FB_ADDR1_DVO0_REG		(0x1580)
#define LS7A_FB_ADDR1_DVO1_REG		(0x1590)

#define LS7A_FB_DAC_CTRL_REG		(0x1600)
#define LS7A_FB_DVO_OUTPUT_REG		(0x1630)

#define LS7A_PMCON_SOC_REG      (void *)TO_UNCAC(LS7A_ACPI_REG_BASE + 0x000)
#define LS7A_PMCON_RESUME_REG   (void *)TO_UNCAC(LS7A_ACPI_REG_BASE + 0x004)
#define LS7A_PMCON_RTC_REG      (void *)TO_UNCAC(LS7A_ACPI_REG_BASE + 0x008)
#define LS7A_PM1_EVT_REG        (void *)TO_UNCAC(LS7A_ACPI_REG_BASE + 0x00c)
#define LS7A_PM1_ENA_REG        (void *)TO_UNCAC(LS7A_ACPI_REG_BASE + 0x010)
#define LS7A_PM1_CNT_REG        (void *)TO_UNCAC(LS7A_ACPI_REG_BASE + 0x014)
#define LS7A_PM1_TMR_REG        (void *)TO_UNCAC(LS7A_ACPI_REG_BASE + 0x018)
#define LS7A_P_CNT_REG          (void *)TO_UNCAC(LS7A_ACPI_REG_BASE + 0x01c)
#define LS7A_GPE0_STS_REG       (void *)TO_UNCAC(LS7A_ACPI_REG_BASE + 0x028)
#define LS7A_GPE0_ENA_REG       (void *)TO_UNCAC(LS7A_ACPI_REG_BASE + 0x02c)
#define LS7A_RST_CNT_REG        (void *)TO_UNCAC(LS7A_ACPI_REG_BASE + 0x030)
#define LS7A_WD_SET_REG         (void *)TO_UNCAC(LS7A_ACPI_REG_BASE + 0x034)
#define LS7A_WD_TIMER_REG       (void *)TO_UNCAC(LS7A_ACPI_REG_BASE + 0x038)
#define LS7A_THSENS_CNT_REG     (void *)TO_UNCAC(LS7A_ACPI_REG_BASE + 0x04c)
#define LS7A_GEN_RTC_1_REG      (void *)TO_UNCAC(LS7A_ACPI_REG_BASE + 0x050)
#define LS7A_GEN_RTC_2_REG      (void *)TO_UNCAC(LS7A_ACPI_REG_BASE + 0x054)
#define LS7A_DPM_CFG_REG        (void *)TO_UNCAC(LS7A_ACPI_REG_BASE + 0x400)                                                                                                
#define LS7A_DPM_STS_REG        (void *)TO_UNCAC(LS7A_ACPI_REG_BASE + 0x404)
#define LS7A_DPM_CNT_REG        (void *)TO_UNCAC(LS7A_ACPI_REG_BASE + 0x408)

#define LS7A_PCIE_BAR_BASE(bus, dev, func) \
	readl((void *)TO_UNCAC(LS7A_CHIPCFG_REG_BASE | (bus << 16) | (dev << 11) | (func << 8) | 0x10))

/* 7A bridge has a gpio controller in DC space */
#define LS7A_DC_CNT_REG_BASE	(LS7A_PCIE_BAR_BASE(0x0, 0x6, 0x1) & 0xfffffff0)

/* REG ACCESS*/
#define ls7a_readb(addr)			  (*(volatile unsigned char  *)TO_UNCAC(addr))
#define ls7a_readw(addr)			  (*(volatile unsigned short *)TO_UNCAC(addr))
#define ls7a_readl(addr)			  (*(volatile unsigned int   *)TO_UNCAC(addr))
#define ls7a_readq(addr)			  (*(volatile unsigned long  *)TO_UNCAC(addr))
#define ls7a_writeb(val, addr)		*(volatile unsigned char  *)TO_UNCAC(addr) = (val)
#define ls7a_writew(val, addr)		*(volatile unsigned short *)TO_UNCAC(addr) = (val)
#define ls7a_writel(val, addr)		ls7a_write_type(val, addr, uint32_t)
#define ls7a_writeq(val, addr)		ls7a_write_type(val, addr, uint64_t)
#define ls7a_write(val, addr)		ls7a_write_type(val, addr, uint64_t)

extern unsigned long ls7a_dc_writeflags;
extern spinlock_t ls7a_dc_writelock;
extern unsigned long ls7a_rwflags;
extern rwlock_t ls7a_rwlock;

#define ls7a_dc_write(val, addr)          \
    do {                                \
        spin_lock_irqsave(&ls7a_dc_writelock,ls7a_dc_writeflags);          \
        *(volatile unsigned long __force *)TO_UNCAC(addr) = (val);          \
        spin_unlock_irqrestore(&ls7a_dc_writelock,ls7a_dc_writeflags);        \
    }while(0)

#define ls7a_read(val, addr)        					  \
    do {                                				  \
        read_lock_irqsave(&ls7a_rwlock,flags); 			          \
        val = *(volatile unsigned long __force *)TO_UNCAC(addr);          \
        read_unlock_irqrestore(&ls7a_rwlock,flags); 		          \
    }while(0)

#define ls7a_write_type(val, addr, type)          					  \
    do {                                				  \
        write_lock_irqsave(&ls7a_rwlock,ls7a_rwflags);          	  \
        *(volatile type __force *)TO_UNCAC(addr) = (val);        \
        write_unlock_irqrestore(&ls7a_rwlock,ls7a_rwflags);               \
    }while(0)

static inline int pcie_get_portnum(void *sysdata)
{
	u64 memstart = ((struct pci_controller *)(sysdata))->mem_resource->start;
	if (memstart < LS2H_PCIE_MEM1_BASE(0))
		return (memstart - LS2H_PCIE_MEM0_BASE(0)) >> 25;
	else
		return (memstart - LS2H_PCIE_MEM1_BASE(0)) >> 28;
}
#define LS2H_PCIE_GET_PORTNUM pcie_get_portnum

/* ============== RS780/SBX00 registers =============== */

#define SBX00_ACPI_IO_BASE 0x800
#define SBX00_ACPI_IO_SIZE 0x100

#define SBX00_PM_EVT_BLK       (SBX00_ACPI_IO_BASE + 0x00) /* 4 bytes */
#define SBX00_PM_CNT_BLK       (SBX00_ACPI_IO_BASE + 0x04) /* 2 bytes */
#define SBX00_PMA_CNT_BLK      (SBX00_ACPI_IO_BASE + 0x0F) /* 1 byte */
#define SBX00_PM_TMR_BLK       (SBX00_ACPI_IO_BASE + 0x18) /* 4 bytes */
#define SBX00_GPE0_BLK         (SBX00_ACPI_IO_BASE + 0x10) /* 8 bytes */
#define SBX00_PM_END           (SBX00_ACPI_IO_BASE + 0x80)

#define PM_INDEX        0xCD6
#define PM_DATA         0xCD7
#define PM2_INDEX       0xCD0
#define PM2_DATA        0xCD1


enum board_type {
	LS2H,
	RS780E,
	LS7A
};

struct platform_controller_hub {
	int	board_type;
	int 	pcidev_max_funcs;
	void	(*early_config)(void);
	void	(*init_irq)(void);
	void	(*init_swiotlb)(void);
	void	(*irq_dispatch)(void);
	int	(*pcibios_map_irq)(const struct pci_dev *dev, u8 slot, u8 pin);
	int	(*pcibios_dev_init)(struct pci_dev *dev);
	void	(*pch_arch_initcall)(void);
	void	(*pch_device_initcall)(void);
#ifdef CONFIG_PCI_MSI
	int	(*pch_setup_msi_irq)(struct pci_dev *pdev, struct msi_desc *desc);
	void	(*pch_teardown_msi_irq)(unsigned int irq);
#endif
};

extern struct platform_controller_hub *loongson_pch;

enum virtdev_irq {
	VIRTDEV_QXL_IRQ = 3,
	VIRTDEV_BLK_VIRTIO_IRQ,
	VIRTDEV_BALLOON_VIRTIO_IRQ,
	VIRTDEV_NET_VIRTIO_IRQ,
	VIRTDEV_ACPI_RESERVED_IRQ = 7,
	VIRTDEV_RTC_RESERVED_IRQ = 8,
	VIRTDEV_SERIAL_VIRTIO_IRQ,
	VIRTDEV_SCSI_VIRTIO_IRQ,
	VIRTDEV_GPU_VIRTIO_IRQ,
	VIRTDEV_IRQ_DEFAULT = 14,
	VIRTDEV_IRQ_MAX = 15,
};

/* gpio data */
struct platform_gpio_data {
	u32 gpio_conf;
	u32 gpio_out;
	u32 gpio_in;
	int gpio_base;
	int ngpio;
};

#ifndef CONFIG_KVM_GUEST_LS3A3000
#define HT_cache_enable_reg1	*(volatile unsigned int *)(0x90000EFDFB000000 + 0x68)
#define HT_cache_base_reg1	    *(volatile unsigned int *)(0x90000EFDFB000000 + 0x6c)
#define HT_uncache_enable_reg0	*(volatile unsigned int *)(0x90000EFDFB000000 + 0xF0)
#define HT_uncache_base_reg0	*(volatile unsigned int *)(0x90000EFDFB000000 + 0xF4)
#define HT_uncache_enable_reg1	*(volatile unsigned int *)(0x90000EFDFB000000 + 0xF8)
#define HT_uncache_base_reg1	*(volatile unsigned int *)(0x90000EFDFB000000 + 0xFC)
#define HT_uncache_enable_reg2	*(volatile unsigned int *)(0x90000EFDFB000000 + 0x168)
#define HT_uncache_base_reg2	*(volatile unsigned int *)(0x90000EFDFB000000 + 0x16C)
#define HT_uncache_enable_reg3	*(volatile unsigned int *)(0x90000EFDFB000000 + 0x170)
#define HT_uncache_base_reg3	*(volatile unsigned int *)(0x90000EFDFB000000 + 0x174)
#else
#define HT_cache_enable_reg1	*(volatile unsigned int *)(0x900000EFFB000000 + 0x68)
#define HT_cache_base_reg1	    *(volatile unsigned int *)(0x900000EFFB000000 + 0x6c)
#define HT_uncache_enable_reg0	*(volatile unsigned int *)(0x900000EFFB000000 + 0xF0)
#define HT_uncache_base_reg0	*(volatile unsigned int *)(0x900000EFFB000000 + 0xF4)
#define HT_uncache_enable_reg1	*(volatile unsigned int *)(0x900000EFFB000000 + 0xF8)
#define HT_uncache_base_reg1	*(volatile unsigned int *)(0x900000EFFB000000 + 0xFC)
#define HT_uncache_enable_reg2	*(volatile unsigned int *)(0x900000EFFB000000 + 0x168)
#define HT_uncache_base_reg2	*(volatile unsigned int *)(0x900000EFFB000000 + 0x16C)
#define HT_uncache_enable_reg3	*(volatile unsigned int *)(0x900000EFFB000000 + 0x170)
#define HT_uncache_base_reg3	*(volatile unsigned int *)(0x900000EFFB000000 + 0x174)
#endif

extern void uncache_resume(void);
#endif
