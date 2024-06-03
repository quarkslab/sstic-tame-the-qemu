/*
 * ATMEL SAMD21 MCU
 *
 * Copyright 2024 Damien Cauquil <dcauquil@quarkslab.com>
 *
 * This code is licensed under the GPL version 2 or later.  See
 * the COPYING file in the top-level directory.
 */

#ifndef SAMD21_MCU_H
#define SAMD21_MCU_H

#include "hw/sysbus.h"
#include "hw/arm/armv7m.h"
#include "hw/clock.h"
#include "qom/object.h"
#include "hw/arm/samd21.h"
#include "hw/misc/samd21_sercom.h"
#include "hw/gpio/samd21_gpio.h"

/**
 * The following lines define our MCU class name, "samd21-mcu",
 * and its associated state structure `SAMD21State`.
 * 
 * The `OBJECT_DECLARE_SIMPLE_TYPE` macro creates a series of other
 * macros including `SAMD21_MCU()` that will cast an opaque pointer
 * to a pointer to a `SAMD21State` structure. This macro is intensively
 * used in the MCU callbacks.
 */

#define TYPE_SAMD21_MCU "samd21-mcu"
OBJECT_DECLARE_SIMPLE_TYPE(SAMD21State, SAMD21_MCU)

/**
 * We then declare the SYSCTRL registers offsets based on the datasheet.
 * 
 * These macros will define for each register a constant `A_<REGISTER_NAME>`
 * and a constant `R_<REGISTER_NAME>` that contains respectively the register
 * memory offset and the register array index (computed from register size).
 * 
 * For instance, the following `SYSCTRL_INTENSET` register is used with the
 * `REG32()` macro creating a `A_SYSCTRL_INTENSET` constant of value 0x04 and
 * a `R_SYSCTRL_INTENSET` constant of value 0x01 (= 0x04 / 4, since the register
 * contains a 32-bit value).
 * 
 * In our state structure, we define a `sysctrl_regs` array that will used to
 * hold the registers value. The `SYSCTRL_INTENSET` register index in  this
 * array is given by `R_SYSCTRL_INTENSET`.
 * 
 * The `REG8()`, `REG16()` and `REG32()` macros are specifically designed for
 * this usage.
 * 
 */

REG32(SYSCTRL_INTENCLR, 0x00)
REG32(SYSCTRL_INTENSET, 0x04)
REG32(SYSCTRL_INTFLAG, 0x08)
REG32(SYSCTRL_PCLKSR, 0x0C)
REG32(SYSCTRL_XOSC, 0x10)
REG32(SYSCTRL_XOSC32K, 0x14)
REG32(SYSCTRL_OSC32K, 0x18)
REG32(SYSCTRL_OSCULP32K, 0x1C)
REG32(SYSCTRL_OSC8M, 0x20)
REG32(SYSCTRL_DFLLCTRL, 0x24)
REG32(SYSCTRL_DFLLVAL, 0x28)
REG32(SYSCTRL_DFLLMUL, 0x2C)
REG32(SYSCTRL_DFLLSYNC, 0x30)
REG32(SYSCTRL_BOD33, 0x34)
REG32(SYSCTRL_VREG, 0x3C)
REG32(SYSCTRL_VREF, 0x40)
REG32(SYSCTRL_DPLLCTRLA, 0x44)
REG32(SYSCTRL_DPLLRATIO, 0x48)
REG32(SYSCTRL_DPLLCTRLB, 0x4C)
REG32(SYSCTRL_DPLLSTATUS, 0x50)

/* Define GCLK registers offsets. */
REG8(GCLK_CTRL, 0x00)
REG8(GCLK_STATUS, 0x01)
REG16(GCLK_CLKCTRL, 0x02)
REG32(GCLK_GENCTRL, 0x04)
REG32(GCLK_GENDIV, 0x08)

/**
 * We then define our own device state structure, `SAMD21State`.
 * 
 * This structure is critical as it will be used to store any instanciated
 * object state in memory. When QEMU creates an object, it retrieves the
 * object type (or class) information and allocate memory for the class
 * instance it created. QEMU has no idea of the structure to use for this
 * newly created object, but the type information provides the size required
 * to store this structure, in bytes.
 * 
 * Therefore, every QEMU functions handling generic objects use *opaque*
 * pointers that point to an instance of each object state structure. Specific
 * callbacks that are expected this structure must convert the provided *opaque*
 * pointer to the correct type (see `OBJECT_DECLARE_SIMPLE_TYPE` above).
 * 
 * This structure starts with the state structure of its parent class, in our
 * case a System Bus Device state structure. This allows our instance structure
 * to be used with any System Bus Device related functions, as they will use
 * this `parent_obj` structure to read and store the parent object state.
 * 
 * Our object properties will be stored right after the parent object state
 * structure, allowing us to add our own members without interfering with the
 * parent state structure.
 * 
 */

struct SAMD21State {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/

    /* We need an ARMv7M CPU object. */
    ARMv7MState cpu;

    /* And also some memory region for our various controllers. */
    MemoryRegion iomem;
    MemoryRegion sram;
    MemoryRegion flash;
    MemoryRegion clock;
    MemoryRegion sys;
    MemoryRegion pm;
    MemoryRegion aux;

    /* These two members are required to store our device properties. */
    uint32_t sram_size;         /* SRAM size. */
    uint32_t flash_size;        /* Flash memory size */
    MemoryRegion *board_memory; /* Board memory region we will load into our
                                   container memory region. */

    /**
     * Hardware peripherals
     */

    /* Clock controller registers (GCLK). */
    uint32_t gclk_regs[3];

    /* SYSCTRL registers. */
    uint32_t sysctrl_regs[0x15];

    /* AUX memory space. */
    uint32_t aux_mem[SAMD21_AUX_SIZE/4];

    /* PORT controller. */
    SAMD21GPIOState port;

    /*
     * SERCOM child object, defined and implemented
     * in /hw/misc/samd21_sercom.c
     */

    SAMD21SERCOMState sercom[6];


    /* Our main MCU memory container. */
    MemoryRegion container;

    /* Main CPU clock. */
    Clock *sysclk;
};

#endif /* SAMD21_H */
