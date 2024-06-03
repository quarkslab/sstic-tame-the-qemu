/*
 * SAMD21 general purpose input/output register definition
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

/*
 * Direction is exposed in both the DIR register and the DIR bit
 * of each PINs CNF configuration register. Reflect bits for pins in DIR
 * to individual pin configuration registers.
 */
static void reflect_dir_bit_in_cnf(SAMD21GPIOState *s)
{
    size_t i;

    uint32_t value = s->dir;

    for (i = 0; i < SAMD21_PORT_PINS; i++) {
        s->cnf[i] = (s->cnf[i] & ~(2UL)) | ((~((value >> i) & 0x01)) << 1);
    }
}

static uint64_t samd21_port_read(void *opaque, hwaddr offset, unsigned int size)
{
    SAMD21GPIOState *s = SAMD21_PORT(opaque);
    size_t idx;
    uint64_t r = 0;

    switch (offset)
    {
        case SAMD21_PORT_REG_OUT ... SAMD21_PORT_REG_OUTCLR:
            r = s->out;
            break;

        case SAMD21_PORT_REG_IN:
            r = s->in;
            break;

        case SAMD21_PORT_REG_WRCONFIG:
            r = 0;
            break;

        case SAMD21_PORT_REG_CONTROL:
            r = 0;
            break;

        case SAMD21_PORT_REG_DIR ... SAMD21_PORT_REG_DIRCLR:
            r = s->dir;
            break;

        case SAMD21_PORT_REG_PINCFG_START ... SAMD21_PORT_REG_PINCFG_STOP:
            idx = (offset - SAMD21_PORT_REG_PINCFG_START) / 4;
            r = s->cnf[idx];
            break;

        default:
            qemu_log_mask(LOG_GUEST_ERROR,
                    "%s: bad read offset 0x%" HWADDR_PRIx "\n",
                        __func__, offset);
            break;
    }

    return r;
}

static void samd21_port_write(void *opaque, hwaddr offset,
                       uint64_t value, unsigned int size)
{
    SAMD21GPIOState *s = SAMD21_PORT(opaque);
    size_t idx;

    switch (offset)
    {
        case SAMD21_PORT_REG_OUT:
        {
            s->out = value;
        }
        break;

        case SAMD21_PORT_REG_OUTSET:
        {
            s->out |= value;
        }
        break;

        case SAMD21_PORT_REG_OUTCLR:
        {
            s->out &= ~(value);
        }
        break;

        case SAMD21_PORT_REG_OUTTGL:
        {
            s->out ^= value;
        }
        break;

        case SAMD21_PORT_REG_DIR:
        {
            s->dir = value;
            reflect_dir_bit_in_cnf(s);
        }
        break;

        case SAMD21_PORT_REG_DIRSET:
        {
            s->dir |= value;
            reflect_dir_bit_in_cnf(s);
        }
        break;

        case SAMD21_PORT_REG_DIRCLR:
        {
            s->dir &= ~(value);
            reflect_dir_bit_in_cnf(s);
        }
        break;

        case SAMD21_PORT_REG_DIRTGL:
        {
            s->dir ^= value;
            reflect_dir_bit_in_cnf(s);
        }
        break;

        case SAMD21_PORT_REG_CONTROL:
        {
            s->control = value;
            qemu_log_mask(LOG_UNIMP,
                      "%s: CONTROL register support not yet implemented !\n",
                      __func__);
        }
        break;

        case SAMD21_PORT_REG_WRCONFIG:
        {
            /* Determine which half to update. */
             qemu_log_mask(LOG_UNIMP,
                      "%s: WRCONFIG register support not yet implemented !\n",
                      __func__);
        }
        break;

        case SAMD21_PORT_REG_PMUX:
        {
            /* Determine which half to update. */
             qemu_log_mask(LOG_UNIMP,
                      "%s: PMUX register support not yet implemented !\n",
                      __func__);
        }
        break;

        case SAMD21_PORT_REG_PINCFG_START ... SAMD21_PORT_REG_PINCFG_STOP:
        {
            idx = (offset - SAMD21_PORT_REG_PINCFG_START) / 4;
            s->cnf[idx] = value;
            s->dir = (s->dir & ~(1UL << idx)) | ((~((value & 0x02)>>1)) << idx);
        }
        break;

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

static const MemoryRegionOps gpio_ops = {
    .read =  samd21_port_read,
    .write = samd21_port_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void samd21_gpio_set(void *opaque, int line, int value)
{
    SAMD21GPIOState *s = SAMD21_PORT(opaque);

    assert(line >= 0 && line < SAMD21_PORT_PINS);

    s->in_mask = deposit32(s->in_mask, line, 1, value >= 0);
    if (value >= 0) {
        s->in = deposit32(s->in, line, 1, value != 0);
    }
}

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

static void samd21_port_init(Object *obj)
{
    SAMD21GPIOState *s = SAMD21_PORT(obj);

    memory_region_init_io(&s->mmio, obj, &gpio_ops, s,
            TYPE_SAMD21_PORT, SAMD21_PORT_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);

    qdev_init_gpio_in(DEVICE(s), samd21_gpio_set, SAMD21_PORT_PINS);
    qdev_init_gpio_out(DEVICE(s), s->output, SAMD21_PORT_PINS);
}

static void samd21_port_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = samd21_port_reset;
    dc->desc = "SAMD21 PORT";
}

static const TypeInfo samd21_port_info = {
    .name = TYPE_SAMD21_PORT,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SAMD21GPIOState),
    .instance_init = samd21_port_init,
    .class_init = samd21_port_class_init
};

static void samd21_port_register_types(void)
{
    type_register_static(&samd21_port_info);
}

type_init(samd21_port_register_types)
