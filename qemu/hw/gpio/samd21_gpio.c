/*
 * SAMD21 general purpose input/output peripheral definition.
 *
 * This hardware peripheral implementation follows SAMD21's datasheet
 * and provide a tri-state input and output support. Pins with level 0 (low)
 * or 1 (high) are considered as driven by this controller while unconnected
 * inputs will show a level of -1 (high-impedance).
 *
 * This code is licensed under the GPL version 2 or later.  See
 * the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/gpio/samd21_gpio.h"
#include "hw/irq.h"
#include "migration/vmstate.h"
#include "trace.h"

/**
 * Determine if a GPIO pin has its internal pull-up resistor enabled.
 *
 * This function returns 1 if the pin's internal pull-up resistor is enabled
 * and -1 if disabled. -1 specifies the pin has high-impedance.
 */

/**
 * @brief Determine if a GPIO pin has its internal pull-up resistor enabled.
 * 
 * This function returns 1 if the pin's internal pull-up resistor is enabled
 * and -1 if disabled. -1 specifies the pin has high-impedance.
 * 
 * @param config Pin configuration (CNF) register value
 * @return int 
 */

static int pull_value(uint32_t config)
{
    int pull = extract32(config, 2, 1);
    if (!pull) {
        /* Pull-up enabled. */
        return 1;
    } else {
        /* high impedance if pull-up disabled. */
        return -1;
    }
}

/**
 * @brief Update GPIO IRQ state based on its current level and state.
 * 
 * If a pin is set as output and is connected to something, the corresponding
 * outgoing IRQ value is set to 0 or 1 depending the configured level.
 * 
 * If this pin is not connected to anything, its corresponding IRQ value is
 * set to -1 to notify that it is disconnected.
 * 
 * Pin IRQ is updated only on pin level change, we keep track of previous levels
 * in our custom state structure SAMD21GPIOState's old_out_connected member.
 * 
 * @param s Custon SAMD21 GPIO state structure (object)
 * @param i Pin number
 * @param connected True if pin is connected, false otherwise
 * @param level Pin level (0 or 1)
 */

static void update_output_irq(SAMD21GPIOState *s, size_t i,
                              bool connected, bool level)
{
    /* If pin is not connected, reflect its floating state by setting the IRQ
       level to -1. */
    int64_t irq_level = connected ? level : -1;
    bool old_connected = extract32(s->old_out_connected, i, 1);
    bool old_level = extract32(s->old_out, i, 1);

    /* If output has been disconnected/connected or level changed, notify. */
    if ((old_connected != connected) || (old_level != level)) {
        qemu_set_irq(s->output[i], irq_level);
    }

    /* Save current pin state. */
    s->old_out = deposit32(s->old_out, i, 1, level);
    s->old_out_connected = deposit32(s->old_out_connected, i, 1, connected);
}


/**
 * @brief Update SAMD21 GPIO controller state
 * 
 * This function retrieves the controller status and configuration registers
 * (OUT and PINCFG, as described in section 23.8 of SAMD21's datasheet), and
 * updates our state structure accordingly.
 * 
 * Moreover, if any pin marked as output has its level changed then the
 * corresponding IRQ (that can be connected to any other device's input) is
 * updated accordingly, causing QEMU to propagate this change to any other
 * connected device. This is how we can drive other devices' input with QEMU,
 * and we don't even have to know to what a GPIO is connected.
 * 
 * @param s SAMD21 GPIO controller state structure
 */

static void update_state(SAMD21GPIOState *s)
{
    int pull;
    size_t i;
    bool connected_out, dir, connected_in, out, in, input;

    for (i = 0; i < SAMD21_PORT_PINS; i++) {
        /* Retrieve the pin pull-up configuration. */
        pull = pull_value(s->cnf[i]);

        /* Retrieve the pin direction. */
        dir = extract32(s->cnf[i], 1, 1);

        /* Determine if the pin is externally driven. */
        connected_in = extract32(s->in_mask, i, 1);

        /* Retrieve the OUT register value. */
        out = extract32(s->out, i, 1);

        /* Retrieve the IN register value. */
        in = extract32(s->in, i, 1);

        /* Determine if pin is configured as input. */
        input = extract32(s->cnf[i], 1, 1);

        /* Determine if pin is configured as output. */
        connected_out = (dir==0);

        /* If pin is configured as output and pull-up enabled, reflect pull-up
           configuration in the IN register. */
        if (!input) {
            if (pull > 0) {
                s->in = deposit32(s->in, i, 1, pull);
            }
        } else {
            /* If pin is configured as input and connected to an output IRQ, then
               we have an issue ! */
            if (connected_out && connected_in && out != in) {
                /* Pin both driven externally and internally */
                qemu_log_mask(LOG_GUEST_ERROR,
                              "GPIO pin %zu short circuited\n", i);
            }

            /* If pin is configured as input and externally driven */
            if (connected_in) {
                /* If pull-up is enabled and pin not connected to an ouput IRQ. */
                if (pull > 0 && !connected_out) {
                    /* Consider pin as connected out (temporarily). */
                    connected_out = true;
 *

                    /* Set pin level to 1 if pull-up enabled, 0 otherwise. */
                    out = pull;
                }
                if (connected_out) {
                    /* Update IN value if pull-up enabled. */
                    s->in = deposit32(s->in, i, 1, out);
                }
            }
        }

        /* Update output GPIO levels based on state. */
        update_output_irq(s, i, connected_out, out);
    }
}

/**
 * @brief Update pin configuration register PINCNF to reflect the direction bit
 * 
 * This function propagates the direction bit set in our GPIO controller DIR
 * register to each pin configuration register (PINCNF), as it is supposed to
 * work in the target hardware.
 * 
 * @param s SAMD21 GPIO state structure
 */

static void reflect_dir_bit_in_cnf(SAMD21GPIOState *s)
{
    size_t i;

    uint32_t value = s->dir;

    for (i = 0; i < SAMD21_PORT_PINS; i++) {
        s->cnf[i] = (s->cnf[i] & ~(2UL)) | ((~((value >> i) & 0x01)) << 1);
    }
}


/**
 * @brief Read operation handler for GPIO controller.
 * 
 * We use this specific handler to emulate our MMIO registers and emulate the
 * corresponding memory region from our controller state structure members.
 * 
 * @param opaque Pointer to an opaque structure representing our object.
 * @param offset Memory offset starting from the device base address
 * @param size Size of data read
 * @return uint64_t Value read from memory
 */

static uint64_t samd21_port_read(void *opaque, hwaddr offset, unsigned int size)
{
    SAMD21GPIOState *s = SAMD21_PORT(opaque);
    size_t idx;
    uint64_t r = 0;

    switch (offset)
    {
        /* Pin output level read. */
        case SAMD21_PORT_REG_OUT ... SAMD21_PORT_REG_OUTCLR:
            r = s->out;
            break;

        /* Pin input level read. */
        case SAMD21_PORT_REG_IN:
            r = s->in;
            break;

        /* Pin WRCONFIG register, not supposed to be read. */
        case SAMD21_PORT_REG_WRCONFIG:
            r = 0;
            break;

        /* Not used, return 0. */
        case SAMD21_PORT_REG_CONTROL:
            r = 0;
            break;

        /* Pin direction register read. */
        case SAMD21_PORT_REG_DIR ... SAMD21_PORT_REG_DIRCLR:
            r = s->dir;
            break;

        /* Pin configuration register read, compute array index and return value. */
        case SAMD21_PORT_REG_PINCFG_START ... SAMD21_PORT_REG_PINCFG_STOP:
            idx = (offset - SAMD21_PORT_REG_PINCFG_START) / 4;
            r = s->cnf[idx];
            break;

        /* Other offsets are not supported, log as error. */
        default:
            qemu_log_mask(LOG_GUEST_ERROR,
                    "%s: bad read offset 0x%" HWADDR_PRIx "\n",
                        __func__, offset);
            break;
    }

    return r;
}


/**
 * @brief Write operation handler for GPIO controller
 * 
 * This handler is called whenever a write operation is performed in memory.
 * It allows us to catch any modification made to our MMIO registers and to
 * update our internal state accordingly.
 * 
 * @param opaque Pointer to an opaque structure representing our object.
 * @param offset Memory offset starting from the device base address
 * @param size Size of data written
 */

static void samd21_port_write(void *opaque, hwaddr offset,
                       uint64_t value, unsigned int size)
{
    SAMD21GPIOState *s = SAMD21_PORT(opaque);
    size_t idx;

    switch (offset)
    {
        /* Pin output level needs to be updated. */
        case SAMD21_PORT_REG_OUT:
        {
            s->out = value;
        }
        break;

        /* Writing to OUTSET sets all bits set to 1 in OUTSET in OUT register (logical OR). */
        case SAMD21_PORT_REG_OUTSET:
        {
            s->out |= value;
        }
        break;

        /* Writing to OUTCLR clears any bit set to 1 in OUTCLR in OUT register (logical AND). */
        case SAMD21_PORT_REG_OUTCLR:
        {
            s->out &= ~(value);
        }
        break;

        /* Writing to OUTTGL toggles any bit set to 1 in OUTTGL in OUT register (logical XOR). */
        case SAMD21_PORT_REG_OUTTGL:
        {
            s->out ^= value;
        }
        break;


        /* Writing to DIR register changes one or more pin direction (input or output). */
        case SAMD21_PORT_REG_DIR:
        {
            s->dir = value;

            /* Reflect direction bit in PINCNF register. */
            reflect_dir_bit_in_cnf(s);
        }
        break;

        /* Set direction bit in DIR register, same behavior as OUTSET. */
        case SAMD21_PORT_REG_DIRSET:
        {
            s->dir |= value;
            reflect_dir_bit_in_cnf(s);
        }
        break;

        /* Clear direction bit in DIR register, same behavior as OUTCLR. */
        case SAMD21_PORT_REG_DIRCLR:
        {
            s->dir &= ~(value);

            /* Reflect direction bit in PINCNF register. */
            reflect_dir_bit_in_cnf(s);
        }
        break;


        /* Toggle direction bit in DIR register, same behavior as OUTTGL. */
        case SAMD21_PORT_REG_DIRTGL:
        {
            s->dir ^= value;
            reflect_dir_bit_in_cnf(s);
        }
        break;

        /* Write value to CONTROL register, not supported yet. */
        case SAMD21_PORT_REG_CONTROL:
        {
            s->control = value;
            qemu_log_mask(LOG_UNIMP,
                      "%s: CONTROL register support not yet implemented !\n",
                      __func__);
        }
        break;

        /* Change multiple pins configuration, not supported yet. */
        case SAMD21_PORT_REG_WRCONFIG:
        {
            /* Determine which half to update. */
             qemu_log_mask(LOG_UNIMP,
                      "%s: WRCONFIG register support not yet implemented !\n",
                      __func__);
        }
        break;

        /* Writing to PMUX, not supported. */
        case SAMD21_PORT_REG_PMUX:
        {
            /* Determine which half to update. */
             qemu_log_mask(LOG_UNIMP,
                      "%s: PMUX register support not yet implemented !\n",
                      __func__);
        }
        break;

        /* Writing to PINCFG, update pin configuration and direction. */
        case SAMD21_PORT_REG_PINCFG_START ... SAMD21_PORT_REG_PINCFG_STOP:
        {
            idx = (offset - SAMD21_PORT_REG_PINCFG_START) / 4;
            s->cnf[idx] = value;
            s->dir = (s->dir & ~(1UL << idx)) | ((~((value & 0x02)>>1)) << idx);
        }
        break;


        /* Other offsets are unsupported. */
        default:
        {
            qemu_log_mask(LOG_GUEST_ERROR,
                        "%s: bad write offset 0x%" HWADDR_PRIx "\n",
                        __func__, offset);
        }
        break;
    } 

    update_state(s);
}


/**
 * @brief Defines our GPIO controller MMIO registers read/write handlers
 */

static const MemoryRegionOps gpio_ops = {
    .read =  samd21_port_read,
    .write = samd21_port_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};


/**
 * @brief Set a GPIO level.
 * 
 * @param opaque Opaque pointer to our GPIO controller state structure
 * @param line GPIO number
 * @param value GPIO level
 */

static void samd21_gpio_set(void *opaque, int line, int value)
{
    SAMD21GPIOState *s = SAMD21_PORT(opaque);

    assert(line >= 0 && line < SAMD21_PORT_PINS);

    s->in_mask = deposit32(s->in_mask, line, 1, value >= 0);
    if (value >= 0) {
        s->in = deposit32(s->in, line, 1, value != 0);
    }
}


/**
 * @brief SAMD21 GPIO controller hardware reset
 * 
 * This function sets the different registers to their reset value, as stated
 * in SAMD21's datasheet.
 * 
 * @param dev Pointer to our device state structure
 */

static void samd21_port_reset(DeviceState *dev)
{
    SAMD21GPIOState *s = SAMD21_PORT(dev);
    size_t i;

    s->out = 0;
    s->in = 0;
    s->old_out = 0;
    s->old_out_connected = 0;
    s->in_mask = 0;
    s->dir = 0;
    s->control = 0;

    for (i = 0; i < SAMD21_PORT_PINS; i++) {
        s->cnf[i] = 0x00000000;
    }
}


/**
 * @brief SAMD21 GPIO controller initialization.
 * 
 * This function initialize the SAMD21 GPIO MMIO register memory region and its
 * related read/write handlers. It also tells QEMU that we provide a specific
 * callback function that must be called whenever one of our input pins' level
 * changes, and that we have a array of output pins that can be connected to
 * any other component.
 * 
 * @param obj Opaque pointer to our object state structure.
 */
static void samd21_port_init(Object *obj)
{
    SAMD21GPIOState *s = SAMD21_PORT(obj);

    /* Tells QEMU to declare a memory region of size SAMD21_PORT_SIZE at offset 0
       and to use our read/writer callbacks to handle these operations. */
    memory_region_init_io(&s->mmio, obj, &gpio_ops, s,
            TYPE_SAMD21_PORT, SAMD21_PORT_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);

    /* Tells QEMU that our device has SAMD21_PORT_PINS input pins and that QEMU
       must call our `samd21_gpio_set()` callback whenever one of this input pin
       level changes. */
    qdev_init_gpio_in(DEVICE(s), samd21_gpio_set, SAMD21_PORT_PINS);

    /* Eventually, we tells QEMU that we expose a set of GPIO as outputs,
       that can be connected to other devices */
    qdev_init_gpio_out(DEVICE(s), s->output, SAMD21_PORT_PINS);
}

/**
 * @brief SAMD21 GPIO class initialization.
 * 
 * @param klass Pointer to our device class structure.
 * @param data Optional data pointer.
 */
static void samd21_port_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    /* We just set our reset callback */
    dc->reset = samd21_port_reset;

    /* And give a name to our controller (SAMD21 PORT peripheral). */
    dc->desc = "SAMD21 PORT";
}

/**
 * @brief Defines our hardware peripheral type.
 * 
 * We specify the parent type (a system bus device), the name of our peripheral
 * ("samd21.port"), the size of our device state structure (SAMD21GPIOState) and
 * our class initialization callback and our class instance initialization callback.
 */
static const TypeInfo samd21_port_info = {
    .name = TYPE_SAMD21_PORT,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SAMD21GPIOState),
    .instance_init = samd21_port_init,
    .class_init = samd21_port_class_init
};

/**
 * @brief Register our SAMD21 GPIO controller type.
 */
static void samd21_port_register_types(void)
{
    type_register_static(&samd21_port_info);
}

/* Register our new type. This code will be executed on QEMU initialization, and
   will add our new device type to QEMU's list of supported device types. */
type_init(samd21_port_register_types)
