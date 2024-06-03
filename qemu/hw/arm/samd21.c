/*
 * ATMEL SAMD21/SAMDA1 MCU

 * https://ww1.microchip.com/downloads/en/DeviceDoc/SAM_D21_DA1_Family_DataSheet_DS40001882F.pdf
 *
 * Copyright 2024 Damien Cauquil <dcauquil@quarkslab.com>
 *
 * This code is licensed under the GPL version 2 or later.  See
 * the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/arm/boot.h"
#include "hw/sysbus.h"
#include "hw/qdev-clock.h"
#include "hw/misc/unimp.h"
#include "qemu/log.h"

/* Include our SAMD21 header files. */
#include "hw/arm/samd21.h"
#include "hw/arm/samd21_mcu.h"

/* Default SAMD21 clock frequency (48MHz). */
#define HCLK_FRQ 48000000

/******************************************************************************
 * SAMD21 System Control (SYSCTRL)
 * 
 * The `sysctrl_read` callback function is called whenever a memory-mapped IO
 * (MMIO) register is read by the CPU (or a debugger). `hwaddr` contains the
 * offset of the memory region that needs to be read and `size` the size of the
 * read operation.
 * 
 * This `size` parameter can generally be 1, 2, 4 or 8 depending on the guest
 * architecture and the instruction being executed. On a ARM-based system for
 * instance, a `ldrb r1, [r0]` instruction will load a single byte located at
 * the address specified in r0 into r1, therefore using a size of 1.
 * 
 * The `size` parameter can be restricted to some specific values by setting
 * QEMU's memory operation structure `impl` field with a default minimal and
 * maximal access sizes as shown below:
 * 
 * ``` c
 * static const MemoryRegionOps sysctrl_ops = {
 *   .read = sysctrl_read,
 *   .write = sysctrl_write,
 *   .endianness = DEVICE_NATIVE_ENDIAN,
 *   .impl.min_access_size = 4,
 *   .impl.max_access_size = 4
 * };
 * ``` 
 * 
 * In this specific case, we are not implementing any specific behavior when
 * a register is read because we only need to provide the correct values to
 * make the application code believe everything is running fine.
 * 
 * This also applies to our `sysctrl_write` callback function that will only
 * write the provided value in our array containing the registers values. This
 * is required because the application code may check later that this value has
 * correctly been modified.
 * 
 * Note that the memory space allocated to the SYSCTRL hardware peripheral is
 * 100% emulated and not fully allocated. We only stick to the defined registers
 * and their associated offsets.
 * 
 * The first parameter given to our callbacks, `opaque`, is a pointer to a
 * structure provided during the initialization of the memory operations
 * associated to this specific region. Since these callbacks are generic,
 * their declarations cannot specify the expected type of this first parameter.
 * This is why the first operation we do in these functions is to call the
 * SAMD21_MCU() macro (defined in samd21_mcu.h) that will perform a transparent
 * cast of this `opaque` pointer and return a pointer to a `SAMD21State`
 * structure containing our MCU state.
 *****************************************************************************/

static uint64_t sysctrl_read(void *opaque, hwaddr addr, unsigned int size)
{
    SAMD21State *s = SAMD21_MCU(opaque);

    /* Return register value. */
    return s->sysctrl_regs[addr/4];
}

static void sysctrl_write(void *opaque, hwaddr addr, uint64_t data,
                        unsigned int size)
{
    SAMD21State *s = SAMD21_MCU(opaque);
    s->sysctrl_regs[addr/4] = (data & 0xffffffff);
}

static const MemoryRegionOps sysctrl_ops = {
    .read = sysctrl_read,
    .write = sysctrl_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4
};


/******************************************************************************
 * SAMD21 Generic Clock controller (GCLK)
 * 
 * In this controller, we are using the same mechanisms as for the SYSCTRL
 * MMIO registers.
 * 
 * We don't need to setup anything as the application code is expected to write
 * values in these MMIO registers to configure and trigger the different clock
 * sources, we just reflect the various values in memory.
 * 
 * It may be interesting to act upon some write operations later, depending on
 * the hardware peripherals we might want to add.
 * 
 *****************************************************************************/

static uint64_t clock_read(void *opaque, hwaddr addr, unsigned int size)
{
    SAMD21State *s = SAMD21_MCU(opaque);

    /* Return register value. */
    return s->gclk_regs[addr/4];
}

static void clock_write(void *opaque, hwaddr addr, uint64_t data,
                        unsigned int size)
{
    SAMD21State *s = SAMD21_MCU(opaque);
    s->gclk_regs[addr/4] = (data & 0xffffffff);
}

static const MemoryRegionOps clock_ops = {
    .read = clock_read,
    .write = clock_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4
};


/******************************************************************************
 * SAMD21 AUX1 memory
 * 
 * This memory region is used to store some non-volatile data related to the
 * SAMD21, therefore we need to emulate this region and keep track of the
 * different values stored in it.
 * 
 * Basically, the bootstrap code does not really care about the values stored
 * in here, so we will fill it with zeroes and emulate the read and write
 * operations.
 *****************************************************************************/

static uint64_t aux_read(void *opaque, hwaddr addr, unsigned int size)
{
    SAMD21State *s = SAMD21_MCU(opaque);

    /* Return register value. */
    return s->aux_mem[addr/4];
}

static void aux_write(void *opaque, hwaddr addr, uint64_t data,
                        unsigned int size)
{
    SAMD21State *s = SAMD21_MCU(opaque);
    s->aux_mem[addr/4] = (data & 0xffffffff);
}

static const MemoryRegionOps aux_ops = {
    .read = aux_read,
    .write = aux_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4
};

/******************************************************************************
 * SAMD21 MCU implementation
 *****************************************************************************/

/******************************************************************************
 * The following `samd21_realize` function is called whenever a SAMD21 MCU is
 * instanciated by QEMU. 
 * 
 * The first parameter provides the device state structure, which again is a
 * generic type because this function is a generic callback. Our `SAMD21State`
 * is defined with its first member being a `SysBusDevice` structure, which has
 * its first member declared as a `DeviceState` structure.
 * 
 * We dynamically cast this structure pointer to a pointer to our `SAMD21State`
 * as we need to access our additional members by using the `SAMD21_MCU()`
 * macro. 
 * 
 * The second parameter is a pointer to a pointer onto an `Error` structure
 * that must be used in this function to report any error that may occurs.
 * 
 * What the following function does is quite simple. An MCU state structure
 * has been allocated by QEMU (representing an instance of our SAMD21 MCU),
 * initialized through a previous call to `samd21_init` and needs to be
 * "realized". Realization in QEMU is a specific step that comes after
 * an object initialization and configuration, since any object normally offers
 * a set of properties that can be accessed by the code that created this
 * object. So once these properties have been set, the object is "realized" in
 * order for these properties to be taken into account and the object finally
 * configured and ready to be used.
 * 
 * In our case, that means:
 * 
 *  - configuring the main CPU clock source (required by the ARM CPU)
 *  - setting the main CPU memory container (again required by the CPU)
 *  - initializing the various additional memory regions: our SRAM, Flash and
 *    other MMIO registers used by our hardware controllers
 *  - realize all of our internal QEMU objects (in our case SERCOM0)
 *  - connect our hardware peripherals IRQ lines to the correct inputs of the
 *    CPU Nested Vector Interrupt Controller (NVIC) in order for these IRQs to
 *    be correctly propagated to the CPU and handled as it should be
 * 
 * Once done, the SAMD21 object is ready and can be used by QEMU.
 * 
 *****************************************************************************/

static void samd21_realize(DeviceState *dev_mcu, Error **errp)
{
    SAMD21State *s = SAMD21_MCU(dev_mcu);
    Error *err = NULL;
    MemoryRegion *mr;
    int i;

    /* SAMD21 object memory property must be set by the code using this
       object before calling the object realization callback. */

    if (!s->board_memory) {
        error_setg(errp, "memory property was not set");
        return;
    }

    /*
     * HCLK on this SoC is fixed, so we set up sysclk ourselves and
     * the board shouldn't connect it.
     */
    if (clock_has_source(s->sysclk)) {
        error_setg(errp, "sysclk clock must not be wired up by the board code");
        return;
    }

    /* This clock doesn't need migration because it is fixed-frequency */
    clock_set_hz(s->sysclk, HCLK_FRQ);
    qdev_connect_clock_in(DEVICE(&s->cpu), "cpuclk", s->sysclk);

    /* Link container memory to system memory. */
    object_property_set_link(OBJECT(&s->cpu), "memory", OBJECT(&s->container),
                             &error_abort);
    
    /* CPU has been correctly configured, we call its realize callback and
       exit if an error occurs during this operation. */
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->cpu), errp)) {
        return;
    }

    /* We map the board memory inside our memory container. */
    memory_region_add_subregion_overlap(&s->container, 0, s->board_memory, -1);

    /* Initialize SRAM memory region. */
    memory_region_init_ram(&s->sram, OBJECT(s), "samd21.sram", s->sram_size,
                           &err);
    if (err) {
        error_propagate(errp, err);
        return;
    }

    /* Map the SRAM memory region in our memory container at the correct
       base address (provided in the SAMD21 datasheet). */
    memory_region_add_subregion(&s->container, SAMD21_SRAM_BASE, &s->sram);


    /*
     * Initialize our generic clock controller (GCLK) MMIO.
     *
     * We first initialize our MMIO registers to zero and then
     * create a dedicated MMIO region with specific read/write
     * handlers. Then this region is mapped into our memory
     * container at the expected address (again as defined in the
     * datasheet).
     */

    memset(s->gclk_regs, 0, sizeof(s->gclk_regs));
    memory_region_init_io(&s->clock, OBJECT(dev_mcu), &clock_ops, (void *)s,
                        "samd21.clock", SAMD21_GCLK_PERIPH_SIZE);

    /* Map our GCLK MMIO registers into our memory container. */
    memory_region_add_subregion_overlap(&s->container,
                                    SAMD21_GCLK_BASE, &s->clock, -1);

    /*
     * Initialize our system controller (GCLK) MMIO.
     *
     * We first initialize our MMIO registers to their default values (as
     * specified in the SAMD21 datasheet) and then create a dedicated MMIO
     * region with specific read/write handlers. Then this region is mapped
     * into our memory container at the expected address (again as defined in
     * the datasheet).
     * 
     * By the way, we mark all the clocks as configured and ready to be used
     * in order for the bootstrap code to consider is fine and to jump as fast
     * as possible to the application code. Since no real clocks are defined
     * in this implementation, clocks will not be reliable but will go as fast
     * as possible. Clock dividers and sources will have no impact on this.
     */

    memset(s->sysctrl_regs, 0, sizeof(s->sysctrl_regs));

    /* Reset registers to their initial value. */
    s->sysctrl_regs[R_SYSCTRL_XOSC] = 0x0080;
    s->sysctrl_regs[R_SYSCTRL_XOSC32K] = 0x0080;
    s->sysctrl_regs[R_SYSCTRL_OSC32K] = 0x003F0080;
    s->sysctrl_regs[R_SYSCTRL_OSC8M] = 0x00000382;
    s->sysctrl_regs[R_SYSCTRL_DFLLCTRL] = 0x0080;
    s->sysctrl_regs[R_SYSCTRL_VREG] = 0x0002;
    s->sysctrl_regs[R_SYSCTRL_DPLLCTRLA] = 0x80;

    /*
     * Mark all clocks as enabled and ready by default (initialization bypass)
     *
     * We do this by setting some bits in the SYSCTRL XOSC32K register and the
     * PCLKSR status register to indicate our clocks are configured and ready.
     */

    s->sysctrl_regs[R_SYSCTRL_XOSC32K] |= 0x06; /* EN32K=1 and ENABLE=1 */
    s->sysctrl_regs[R_SYSCTRL_PCLKSR] = 0xDE; /* DFLLRDY=1, OSC32KRDY=1 and OSC8MRDY=1 */
    
    /* We then initialize a MMIO region for our registers. */
    memory_region_init_io(&s->sys, OBJECT(dev_mcu), &sysctrl_ops, (void *)s,
                        "samd21.sysctrl", SAMD21_SYSCTRL_PERIPH_SIZE);

    /* And we map them into our container at the correct base address. */
    memory_region_add_subregion_overlap(&s->container,
                                    SAMD21_SYSCTRL_BASE, &s->sys, -1);




    /*
     * Initialize our AUX memory region.
     *
     * Nothing fancy here, we just create an MMIO region and mapped it at the
     * correct address (SAMD21_AUX_BASE).
     */


    memset(&s->aux_mem, 0, SAMD21_AUX_SIZE);
    memory_region_init_io(&s->aux, OBJECT(dev_mcu), &aux_ops, (void *)s,
                        "samd21.aux", SAMD21_AUX_SIZE);
    memory_region_add_subregion_overlap(&s->container,
                                    SAMD21_AUX_BASE, &s->aux, -1);


    /*
     * Initialize our Flash memory region.
     *
     * The Flash memory region is located at address SAMD21_FLASH_BASE and will
     * contain the application code. We need to declare it in order to make this
     * memory region available and load some firmware in it later.
     * 
     * We are using `memory_region_init_rom` as it is basically read-only memory
     * (well, the SAMD21 NVM controller has not been implemented at this point
     * so the code cannot update the Flash memory).
     */

    /* Create the Flash memory ROM region. */
    memory_region_init_rom(&s->flash, OBJECT(s), "samd21.flash", SAMD21_X18_FLASH_SIZE,
                           &err);
    if (err) {
        error_propagate(errp, err);
        return;
    }

    /* Map the Flash memory at the correct base address (SAMD21_FLASH_BASE). */
    memory_region_add_subregion_overlap(&s->container,
                                SAMD21_FLASH_BASE, &s->flash, -1);


    /*
     * Initialize PORT controller.
     */
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->port), errp)) {
        return;
    }

    /* We retrieve our PORT controller MMIO region into `mr`. */
    mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->port), 0);

    /* And we map it to the correct base address inside our container. */
    memory_region_add_subregion_overlap(&s->container, SAMD21_PORT_BASE, mr, 0);

    /*
     * Initialize SERCOM0.
     *
     * Unlike the different controllers initialized above, the SERCOM peripheral
     * is a real QEMU object that has been created in the `samd21_init` callback.
     * 
     * Therefore, we need QEMU to realize this object (remember, it has been
     * configured in the `samd21_init` function). Once realized, we also need
     * to map its own MMIO memory regions into our container at the correct base
     * address.
     * 
     * To do this, we first need to retrieve the first MMIO region configured
     * in our SERCOM peripheral by using the `sysbus_mmio_get_region` function.
     * This function will find the n-th MMIO region associated with a device
     * system bus, meaning the n-th MMIO region associated with our SERCOM
     * device. Of course, a device can register more than one MMIO region (this
     * SAMD21 device implementation is a good example :D), so we need to give
     * the expected MMIO number. MMIO regions are numbered in the exact order
     * they have been declared and associated to a device system bus. In our
     * case, the SERCOM device has a single MMIO region so we ask QEMU to
     * retrieve the MMIO region number 0. 
     * 
     * It may be difficult to remember exactly which MMMIO region has which
     * number, so please refer to the device implementation to find it out.
     * Yes, that is a limitation set up by QEMU.
     * 
     */

    for (i=0; i<6; i++)
    {

        if (!sysbus_realize(SYS_BUS_DEVICE(&s->sercom[i]), errp)) {
            return;
        }

        /* We retrieve our SERCOM peripheral MMIO region into `mr`. */
        mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->sercom[i]), 0);

        /* And we map it to the correct base address inside our container. */
        memory_region_add_subregion_overlap(&s->container, SAMD21_SERCOM0_BASE + 0x400*i, mr, 0);


        /*
        * Last but not least, we need to connect our SERCOM IRQ line to the
        * corresponding NVIC input IRQ line, using the `sysbus_connect_irq`
        * function. We need to know the IRQ line number used in the SERCOM
        * peripheral implementation (again, identified by a sequential number
        * that is incremented in the exact order these IRQ lines have been
        * created) and connect it to the correct GPIO exposed by the CPU NVIC.
        * 
        * Here we are connecting SERCOM's IRQ line #0 to NVIC input GPIO #9,
        * as described in the SAMD21 documentation (section 11.2.2).
        */
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->sercom[i]), 0,
                        qdev_get_gpio_in(DEVICE(&s->cpu),
                        9 + i));
    }



    /*
     * All the remaining IO memory must be considered as not implemented,
     * and is created using the `create_unimplemented_device` function.
     * 
     * This helper function will create a new device with an associated
     * MMIO region that starts at the provided base address (SAMD21_IOMEM_BASE)
     * with a provided size (SAMD21_IOMEM_SIZE). This MMIO region will
     * have a very low priority (-1000) and any overlapping region with higher
     * priority will catch any read/write operation. The region is filled with
     * zeroes by default.
     * 
     * This device will ensure any read/write access to any unimplemented
     * controller MMIO registers will be allowed and logged with LOG_UNIMP.
     */

    create_unimplemented_device("samd21_mcu.io", SAMD21_IOMEM_BASE,
                                SAMD21_IOMEM_SIZE);
}

/*
 * SAMD21 object initialization callback
 *
 * This callback function (`samd21_init`) is called right after QEMU created a new
 * SAMD21 MCU object. The single parameter `obj` points to the allocated object
 * structure, in our case our `SAMD21State` structure.
 * 
 * The objectives of this function are the following:
 * 
 *  - To initialize the device main container.
 *  - To create the correct CPU for this MCU (ARM Cortex-M0+) and set its
 *    properties (CPU type, number of IRQ lines)
 *  - To create the SERCOM0 peripheral and set its properties (the character
 *    device to use for USART)
 *  - To create the system clock that will be later used to feed the CPU
 *    in `samd21_realize`.
 * 
 * Don't get it wrong: this function creates objects that are required by the
 * current device (SAMD21 MCU) but let the `samd21_realize` function handle
 * the realization of all the different objects (and sometimes MMIO mapping).
 * 
 */

static void samd21_init(Object *obj)
{
    SAMD21State *s = SAMD21_MCU(obj);
    int i;


    /* Initialize the MCU memory container for this device. */
    memory_region_init(&s->container, obj, "samd21-container", UINT64_MAX);

    /* 
     * Initialize our CPU (this will call the associated CPU initialization
     * function) and set its properties (based on those exposed in its class
     * definition).
     * 
     * In our case we specify the type of CPU we need and the number of IRQs
     * it should handle through its NVIC.
     */

    object_initialize_child(OBJECT(s), "armv7m", &s->cpu, TYPE_ARMV7M);
    qdev_prop_set_string(DEVICE(&s->cpu), "cpu-type",
                         ARM_CPU_TYPE_NAME("cortex-m0"));
    qdev_prop_set_uint32(DEVICE(&s->cpu), "num-irq", 32);

    /* 
     * Initialize our 6 SERCOM interfaces (this will call the corresponding
     * initialization callback implemented in `samd21_sercom.c`).
     * 
     * In our case we specify the character device to use when dealing with
     * an USART device (SERCOM supports multiple protocols including USART),
     * but only for SERCOM0.
     * 
     * We also create an alias for this character device called "serial0",
     * that will appear as a property of the SAMD21 device, linked to this
     * SERCOM0 interface.
     * 
     * Note: no need to use any API defined in the SERCOM peripheral
     * implementation nor including the `samd21_sercom.h`header file,
     * types are directly managed by QEMU and instanciated thanks to the
     * QEMU Object Model.
     */

    for (i=0; i<6; i++)
    {
        /* Create an instance of SERCOM peripheral. */
        object_initialize_child(obj, "sercom[*]", &s->sercom[i], TYPE_SAMD21_SERCOM);

        /* Special case for SERCOM0: we want to add an alias 'serial0' for it. */
        if (i == 0)
        {
            object_property_add_alias(obj, "serial0", OBJECT(&s->sercom[i]), "chardev");
        }
    }

    /*
     * Initialize our PORT (GPIO) controller.
     */
    object_initialize_child(obj, "port", &s->port, TYPE_SAMD21_PORT);

    /* 
     * Initialize a clock for this device called "sysclk".
     */

    s->sysclk = qdev_init_clock_in(DEVICE(s), "sysclk", NULL, NULL, 0);
}

/*
 * The following structure defines the properties associated with our SAMD21
 * device class:
 * 
 *  - `memory`: this is a link to the `board_memory` member of our `SAMD21State`
 *              structure, which is a pointer to a memory region
 *  - `sram-size`: Size of the MCU SRAM in bytes, configured with a default
 *                 value of SAMD21_X18_SRAM_SIZE if not provided.
 *  - `flash-size`: Size of the Flash memory in bytes, configured with a default
 *                  value of SAMD21_X18_FLASH_SIZE if not provided.
 * 
 * Instead of providing the SRAM and Flash sizes, we could also provide a
 * property to set the expected variant of the SAMD21 chip and then deduce
 * in `samd21_init` the corresponding SRAM and Flash size to use.
*/

static Property samd21_properties[] = {
    DEFINE_PROP_LINK("memory", SAMD21State, board_memory, TYPE_MEMORY_REGION,
                     MemoryRegion *),
    DEFINE_PROP_UINT32("sram-size", SAMD21State, sram_size, SAMD21_X18_SRAM_SIZE),
    DEFINE_PROP_UINT32("flash-size", SAMD21State, flash_size,
                       SAMD21_X18_FLASH_SIZE),
    DEFINE_PROP_END_OF_LIST(),
};

/**
 * This `samd21_class_init` function is the class initialization callback.
 * 
 * This function is called when an object of our device class is instanciated
 * and used to configure its properties and related callbacks.
 * 
 * In our case, we set the realization callback to `samd21_realize` and
 * the object properties to the previously defined properties. 
 */

static void samd21_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = samd21_realize;
    device_class_set_props(dc, samd21_properties);
}

/*
 * The `samd21_info` structure describes our new class. It provides
 * the following information:
 * 
 *  - The `name` member specifies the name associated with this class, in our
 *    case "samd21-mcu".
 * 
 *  - The `parent` member describes the parent class, in our case a System Bus
 *    Device ("sys-bus-device"). This parent class must be correlated to the
 *    associated structure `parent_obj` member type.
 * 
 *  - The `instance_size` member specifies the number of bytes to allocate for
 *    the associated device state structure, in our case the size of our
 *    `SAMD21State` structure.
 * 
 *  - The `instance_init` member is set to point to our class instance initialization
 *    function, in our case `samd21_init`.
 * 
 *  - The `class_init` member is set to point to our class initialization
 *    function, in our case `samd21_class_init`.
 */
static const TypeInfo samd21_info = {
    .name          = TYPE_SAMD21_MCU,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SAMD21State),
    .instance_init = samd21_init,
    .class_init    = samd21_class_init,
};


/*
 * The following function is a callback that will be used to register all the
 * new types implemented in this module.
 * 
 * It simply calls QEMU `type_register_static()` with the type description
 * structure as parameter to make QEMU aware of this class.
 */

static void samd21_types(void)
{
    type_register_static(&samd21_info);
}

/*
 * We tell QEMU to register our new module initialization callback, it will
 * be called at the right time to register the required classes and make them
 * available to the user.
 */

type_init(samd21_types)