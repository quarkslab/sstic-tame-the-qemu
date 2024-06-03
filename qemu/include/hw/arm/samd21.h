/*
 * Atmel SAMD21 common defines
 *
 * This file hosts generic defines used in various SAMD21 peripheral devices.
 *
 * Product Datasheet: https://ww1.microchip.com/downloads/en/DeviceDoc/SAM_D21_DA1_Family_DataSheet_DS40001882F.pdf
 *
 * Copyright 2024 Damien Cauquil <dcauquil@quarkslab.com>
 *
 * This code is licensed under the GPL version 2 or later.  See
 * the COPYING file in the top-level directory.
 */

#ifndef SAMD21_H
#define SAMD21_H

/* Flash base address and variant sizes. */
#define SAMD21_FLASH_BASE       0x00000000
#define SAMD21_X18_FLASH_SIZE   0x00040000
#define SAMD21_X17_FLASH_SIZE   0x00020000
#define SAMD21_X16_FLASH_SIZE   0x00010000
#define SAMD21_X15_FLASH_SIZE   0x00008000
#define SAMD21_X14_FLASH_SIZE   0x00004000

/* SRAM base address and variant sizes. */
#define SAMD21_SRAM_BASE        0x20000000
#define SAMD21_X18_SRAM_SIZE    0x00008000
#define SAMD21_X17_SRAM_SIZE    0x00004000
#define SAMD21_X16_SRAM_SIZE    0x00002000
#define SAMD21_X15_SRAM_SIZE    0x00001000
#define SAMD21_X14_SRAM_SIZE    0x00001000

/* IOMEM description. */
#define SAMD21_IOMEM_BASE       0x40000000
#define SAMD21_IOMEM_SIZE       0x20100000

/* AHP-APB Bridge A */
#define SAMD21_PAC0_BASE       0x40000000
#define SAMD21_PM_BASE         0x40000400
#define SAMD21_SYSCTRL_BASE    0x40000800
#define SAMD21_GCLK_BASE       0x40000C00
#define SAMD21_WDT_BASE        0x40001000
#define SAMD21_RTC_BASE        0x40001400
#define SAMD21_EIC_BASE        0x40001800

/* AHB-APB bridge B */
#define SAMD21_PAC1_BASE       0x41000000
#define SAMD21_DSU_BASE        0x41002000
#define SAMD21_NVMCTRL_BASE    0x41004000
#define SAMD21_PORT_BASE       0x41004400
#define SAMD21_DMAC_BASE       0x41004800
#define SAMD21_USB_BASE        0x41005000
#define SAMD21_MTB_BASE        0x41006000

/* AHB-APB Bridge C */
#define SAMD21_PAC2_BASE       0x42000000
#define SAMD21_EVSYS_BASE      0x42000400
#define SAMD21_SERCOM_BASE     0x42000800
#define SAMD21_SERCOM0_BASE    0x42000800
#define SAMD21_SERCOM1_BASE    0x42000C00
#define SAMD21_SERCOM2_BASE    0x42001000
#define SAMD21_SERCOM3_BASE    0x42001400
#define SAMD21_SERCOM4_BASE    0x42001800
#define SAMD21_SERCOM5_BASE    0x42001C00
#define SAMD21_TCC_BASE        0x42002000
#define SAMD21_TCC0_BASE       0x42002000
#define SAMD21_TCC1_BASE       0x42002400
#define SAMD21_TCC2_BASE       0x42002800
#define SAMD21_TC3_BASE        0x42002C00
#define SAMD21_TC4_BASE        0x42003000
#define SAMD21_TC5_BASE        0x42003400
#define SAMD21_TC6_BASE        0x42003800
#define SAMD21_TC7_BASE        0x42003C00
#define SAMD21_ADC_BASE        0x42004000
#define SAMD21_AC_BASE         0x42004400
#define SAMD21_DAC_BASE        0x42004800
#define SAMD21_PTC_BASE        0x42004C00
#define SAMD21_I2S_BASE        0x42005000
#define SAMD21_AC1_BASE        0x42005400
#define SAMD21_TCC3_BASE       0x42006000

/* GCLK registers offsets */
#define SAMD21_GCLK_CTRL        0x00
#define SAMD21_GCLK_STATUS      0x01
#define SAMD21_GCLK_CLKCTRL     0x02
#define SAMD21_GCLK_GENCTRL     0x04
#define SAMD21_GCLK_GENDIV      0x08
#define SAMD21_GCLK_PERIPH_SIZE 0x0C 

/* SYSCTRL regisyers offsets */
#define SAMD21_SYSCTRL_INTENCLR     0x00
#define SAMD21_SYSCTRL_INTENSET     0x04
#define SAMD21_SYSCTRL_INTENFLAG    0x08
#define SAMD21_SYSCTRL_PCLKSR       0x0C
#define SAMD21_SYSCTRL_XOSC         0x10
#define SAMD21_SYSCTRL_XOSC32K      0x14
#define SAMD21_SYSCTRL_OSC32K       0x18
#define SAMD21_SYSCTRL_OSCULP32K    0x1C
#define SAMD21_SYSCTRL_OSC8M        0x20
#define SAMD21_SYSCTRL_DFLLCTRL     0x24
#define SAMD21_SYSCTRL_DFLLVAL      0x28
#define SAMD21_SYSCTRL_DFLLMUL      0x2C
#define SAMD21_SYSCTRL_DFLLSYNC     0x30
#define SAMD21_SYSCTRL_BOD33        0x34
#define SAMD21_SYSCTRL_VREG         0x3C
#define SAMD21_SYSCTRL_VREF         0x40
#define SAMD21_SYSCTRL_DPLLCTRLA    0x44
#define SAMD21_SYSCTRL_DPLLRATIO    0x48
#define SAMD21_SYSCTRL_DPLLCTRLB    0x4C
#define SAMD21_SYSCTRL_DPLLSTATUS   0x50
#define SAMD21_SYSCTRL_PERIPH_SIZE  0x54

/* AUX1 */
#define SAMD21_AUX_BASE            0x00800000
#define SAMD21_AUX_SIZE            0x00006040

/* SERCOM */
#define SAMD21_SERCOM_CTRLA        0x00
#define SAMD21_SERCOM_CTRLB        0x04
#define SAMD21_SERCOM_BAUD         0x0C
#define SAMD21_SERCOM_RXPL         0x0E
#define SAMD21_SERCOM_INTENCLR     0x14
#define SAMD21_SERCOM_INTENSET     0x16
#define SAMD21_SERCOM_INTFLAG      0x18
#define SAMD21_SERCOM_STATUS       0x1A
#define SAMD21_SERCOM_SYNCBUSY     0x1C
#define SAMD21_SERCOM_ADDR         0x24
#define SAMD21_SERCOM_DATA         0x28
#define SAMD21_SERCOM_DBGCTRL      0x30

#endif /* SAMD21_H */
