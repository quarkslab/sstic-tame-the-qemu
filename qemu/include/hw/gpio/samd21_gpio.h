/*
 * SAMD21 System-on-Chip general purpose input/output register definition
 *
 * QEMU interface:
 * + sysbus MMIO regions 0: GPIO registers
 * + Unnamed GPIO inputs 0-31: Set tri-state input level for GPIO pin.
 *   Level -1: Externally Disconnected/Floating; Pull-up/down will be regarded
 *   Level 0: Input externally driven LOW
 *   Level 1: Input externally driven HIGH
 * + Unnamed GPIO outputs 0-31:
 *   Level -1: Disconnected/Floating
 *   Level 0: Driven LOW
 *   Level 1: Driven HIGH
 *
 * Accuracy of the peripheral model:
 * + The nRF51 GPIO output driver supports two modes, standard and high-current
 *   mode. These different drive modes are not modeled and handled the same.
 * + Pin SENSEing is not modeled/implemented.
 *
 * Copyright 2018 Steffen Görtz <contrib@steffen-goertz.de>
 *
 * This code is licensed under the GPL version 2 or later.  See
 * the COPYING file in the top-level directory.
 *
 */
#ifndef SAMD21_PORT_H
#define SAMD21_PORT_H

#include "hw/sysbus.h"
#include "qom/object.h"
#define TYPE_SAMD21_PORT "samd21_soc.port"
OBJECT_DECLARE_SIMPLE_TYPE(SAMD21GPIOState, SAMD21_PORT)

#define SAMD21_PORT_PINS 32

#define SAMD21_PORT_SIZE 0x1000

#define SAMD21_PORT_REG_DIR          0x00
#define SAMD21_PORT_REG_DIRCLR       0x04
#define SAMD21_PORT_REG_DIRSET       0x08
#define SAMD21_PORT_REG_DIRTGL       0x0C
#define SAMD21_PORT_REG_OUT          0x10
#define SAMD21_PORT_REG_OUTCLR       0x14
#define SAMD21_PORT_REG_OUTSET       0x18
#define SAMD21_PORT_REG_OUTTGL       0x1C
#define SAMD21_PORT_REG_IN           0x20
#define SAMD21_PORT_REG_CONTROL      0x24
#define SAMD21_PORT_REG_WRCONFIG     0x28
#define SAMD21_PORT_REG_PMUX         0x30
#define SAMD21_PORT_REG_PINCFG_START 0x40
#define SAMD21_PORT_REG_PINCFG_STOP  0x60

struct SAMD21GPIOState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    qemu_irq irq;

    
    uint32_t out;
    uint32_t old_out;
    uint32_t old_out_connected;
    
    uint32_t in;
    uint32_t in_mask;
    uint32_t dir;
    uint32_t control;
    uint32_t cnf[SAMD21_PORT_PINS];

   qemu_irq output[SAMD21_PORT_PINS];
   qemu_irq detect;
};


#endif
