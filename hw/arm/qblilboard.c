/*
 * Qb Lil'board emulation
 *
 * Copyright (C) 2024 Damien Cauquil
 * Written by Damien Cauquil <dcauquil@quarkslab.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/qdev-properties.h"
#include "hw/boards.h"
#include "hw/arm/boot.h"
#include "sysemu/sysemu.h"
#include "exec/address-spaces.h"

#include "hw/arm/samd21_mcu.h"
#include "hw/i2c/microbit_i2c.h"
#include "hw/qdev-properties.h"
#include "qom/object.h"

/**
 * Like our SAMD21 SERCOM device class or our MCU device class,
 * we need to define a specific device class for our board, as well
 * as a device state structure:
 */

struct LilboardMachineState {
    /* Parent machine state. */
    MachineState parent;

    /* MCU state. */
    SAMD21State samd21;
};

/**
 * We define a simple device type declared as a machine named "qb-lilboard",
 * as well as the associated state structure `LilboardMachineState` and the
 * associated macro `LILBOARD_MACHINE()` to transparently cast a machine opaque
 * pointer into a pointer to our state structure.
 */

#define TYPE_LILBOARD_MACHINE MACHINE_TYPE_NAME("qb-lilboard")
OBJECT_DECLARE_SIMPLE_TYPE(LilboardMachineState, LILBOARD_MACHINE)

/**
 * The `lilboard_init` callback initialize the machine, including its
 * child objects, memory and firmware.
 */

static void lilboard_init(MachineState *machine)
{
    DeviceState *dev;
    DriveInfo *di;
    BlockBackend *blk;
    qemu_irq cs_line;
    
    /* We cast the machine opaque pointer to a `LilboardMachineState` pointer. */
    LilboardMachineState *s = LILBOARD_MACHINE(machine);

    /*
     * We retrieve the system memory region allocated to this machine.
     * QEMU allocates by itself the machine' system memory (in fact a
     * structure representing the system memory space, not pre-allocating
     * a huge chunk of memory).
     */
    MemoryRegion *system_memory = get_system_memory();

    /**
     * We initialize our MCU, by asking QEMU to instanciate a device of
     * type `TYPE_SAMD21_MCU` and store its state into the `samd21` member
     * of our machine state structure.
     */
    object_initialize_child(OBJECT(machine), "samd21", &s->samd21,
                            TYPE_SAMD21_MCU);

    /**
     * We then set the MCU "serial0" property in order to use the
     * first defined serial hardware device specified in command
     * line. 
     */
    qdev_prop_set_chr(DEVICE(&s->samd21), "serial0", serial_hd(0));

    /**
     * After that, we link our MCU "property" to the board system memory,
     * in order for our MCU to be able to access our board memory map.
     */
    object_property_set_link(OBJECT(&s->samd21), "memory",
                             OBJECT(system_memory), &error_fatal);

    /**
     * When all the properties have been set, we can ask QEMU to realize
     * our MCU: it will call the object `realize` callback in charge of
     * setting the object state and child objects based on the properties
     * we defined.
     */
    sysbus_realize(SYS_BUS_DEVICE(&s->samd21), &error_fatal);

    /**
     * Once our MCU initialized, we declare a new SSI bus and connect our
     * SERCOM SSI buses on it.
     * 
     * 
     * 
     */

    /* Look for a block device that represents our SPI flash device content. */
    di = drive_get(IF_NONE, 0, 0);
    blk = di ? blk_by_legacy_dinfo(di) : NULL;
    if (blk != NULL)
    {
        /* Create an SPI flash device and attach to SERCOM4 SSI interface. */
        dev = qdev_new("sst25vf016b");
        qdev_prop_set_drive(dev, "drive", blk);
        qdev_prop_set_uint8(dev, "cs", 0);
        qdev_realize_and_unref(dev, BUS(s->samd21.sercom[4].ssi), &error_fatal);

        /* Connect Flash CS line to our GPIO. */
        cs_line = qdev_get_gpio_in_named(dev, SSI_GPIO_CS, 0);
        qdev_connect_gpio_out(DEVICE(&s->samd21.port), 21, cs_line);
    }

    /**
     * Our MCU is ready to execute some code, but we still need to load this
     * code into memory. In our case, we load a binary or hex firmware at
     * the beginning of the Flash memory (mapped to 0x00000000) thanks to
     * the `armv7m_load_kernel` function.
     * 
     * The `kernel_filename` file path is provided through the `-kernel`
     * option of QEMU, or through the `-device loader` option.
     * 
     * Once the firmware loaded into memory, we let QEMU start our machine. 
     */

    armv7m_load_kernel(ARM_CPU(first_cpu), machine->kernel_filename,
                       0, s->samd21.flash_size);
}

/**
 * QbLilboard machine class initialization callback.
 * 
 * As usual, we set the machine description, instance initialization callback
 * and maximum number of CPUs.
 */

static void lilboard_machine_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "Qb Lil'board (SAMD21)";
    mc->init = lilboard_init;
    mc->max_cpus = 1;
}

/**
 * The following structure describes our machine class type:
 * 
 * - It sets the machine name ("qb-lilboard").
 * - It sets the parent class type (a QEMU machine).
 * - It sets the instance size (the size of our state structure).
 * - It sets the class initialization callback.
 * 
 */

static const TypeInfo lilboard_info = {
    .name = TYPE_LILBOARD_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(LilboardMachineState),
    .class_init = lilboard_machine_class_init,
};

/**
 * The following function is in charge of declaring every types related
 * to our class.
 * 
 * In fact, it only registers our machine type.
 */

static void lilboard_machine_init(void)
{
    type_register_static(&lilboard_info);
}

/* Tells QEMU to register our module type registration callback. */
type_init(lilboard_machine_init);