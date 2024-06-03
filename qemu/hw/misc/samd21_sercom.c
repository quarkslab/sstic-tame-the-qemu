/*
 * SAMD21 SERCOM peripheral (USART/SPI)
 *
 * Copyright 2024 Damien Cauquil <dcauquil@quarkslab.com>
 *
 * This code is licensed under the GPL version 2 or later.  See
 * the COPYING file in the top-level directory.
 * 
 * 
 * The SERCOM interface implemented in the SAMD21 microcontroller supports
 * different protocols including USART and SPI. This is why this file is
 * present in the /hw/misc/ folder.
 * 
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "hw/arm/samd21.h"
#include "hw/misc/samd21_sercom.h"
#include "hw/irq.h"
#include "hw/qdev-properties-system.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"

/******************************************************************************
 * The following `samd21_update_irq` function sets the SERCOM interface IRQ
 * level based on:
 * 
 *  - the event flags set in this SERCOM INTFLAG register: DRE, TXC, RXC, RXS,
 *    CTSIC, RXBRK and ERROR
 *  - the interrupt events enabled by the software through the INTENSET register
 * 
 * The IRQ is raised if at least one event flag is set and the corresponding
 * interrupt event enabled. If an interrupt flag is set but not enabled then
 * no IRQ will be raised.
 * 
 *****************************************************************************/
static void samd21_update_irq(SAMD21SERCOMState *s)
{
    uint8_t intflag = s->reg8[R_USART_INTFLAG];
    uint8_t intenset = s->interrupts;
    bool irq = false;

    /* DRE */
    irq |= (intflag &&
            (intflag & R_USART_INTFLAG_DRE_MASK) &&
            (intenset & R_USART_INTFLAG_DRE_MASK));

    /* TXC */
    irq |= (intflag &&
            (intflag & R_USART_INTFLAG_TXC_MASK) &&
            (intenset & R_USART_INTFLAG_TXC_MASK));

    /* RXC */
    irq |= (intflag &&
            (intflag & R_USART_INTFLAG_RXC_MASK) &&
            (intenset & R_USART_INTFLAG_RXC_MASK));

    /* RXS */
    irq |= (intflag &&
            (intflag & R_USART_INTFLAG_RXS_MASK) &&
            (intenset & R_USART_INTFLAG_RXS_MASK));

    /* CTSIC */
    irq |= (intflag &&
            (intflag & R_USART_INTFLAG_CTSIC_MASK) &&
            (intenset & R_USART_INTFLAG_CTSIC_MASK));

    /* RXBRK */
    irq |= (intflag &&
            (intflag & R_USART_INTFLAG_RXBRK_MASK) &&
            (intenset & R_USART_INTFLAG_RXBRK_MASK));

    /* ERROR */
    irq |= (intflag &&
            (intflag & R_USART_INTFLAG_ERROR_MASK) &&
            (intenset & R_USART_INTFLAG_ERROR_MASK));

    /* Set IRQ level (IRQ is triggered if level > 0). */
    qemu_set_irq(s->irq, irq);
}

static void sercom_spi_transfer(SAMD21SERCOMState *s)
{
    /* Send and receive byte. */
    s->reg16[R_SERCOM_DATA] = ssi_transfer(s->ssi, s->reg16[R_SERCOM_DATA] & 0xFF);

    /* Set DRE, RXC and TXC flag in the INTFLAG register, to tell the software that
       more bytes can be sent after this function call. */
    s->reg8[R_SPI_INTFLAG] |= (1 << R_SPI_INTFLAG_DRE_SHIFT);
    s->reg8[R_SPI_INTFLAG] |= (1 << R_SPI_INTFLAG_TXC_SHIFT);
    s->reg8[R_SPI_INTFLAG] |= (1 << R_SPI_INTFLAG_RXC_SHIFT);
}

/******************************************************************************
 * The `sercom_read` callback is in charge of handling any read operation
 * performed on SERCOM MMIO registers. Based on the datasheet, we must emulate
 * some internal operations when the application code reads some specific
 * registers like the DATA register used for data reception.
 * 
 * When the DATA register is read, we need to retrieve the next read byte from
 * our reception queue, motify QEMU that one byte has been read from the
 * associated character device (if current mode is USART) in order to keep
 * the emulated serial device backend working as expected.
 * 
 * Once read, we also set the interrupt flag and trigger another IRQ if more
 * data is waiting to be read in our reception queue, allowing the software
 * to loop on the received data until the reception queue is empty.
 * 
 * When the STATUS register is read, we provide its 16-bit value as stored in
 * the corresponding register bank.
 * 
 * Other registers are mostly 32-bits and can be read through the default
 * register handler.
 *****************************************************************************/

static uint64_t sercom_read(void *opaque, hwaddr addr, unsigned int size)
{
    SAMD21SERCOMState *s = SAMD21_SERCOM(opaque);
    uint64_t db;

    /* Process registers differently when required. */
    switch (addr)
    {
        /**
         * SERCOM DATA register needs to be handled in a very specific way,
         * as it is fully emulated (we will not write to the register in our
         * register bank). When this register is read and the SERCOM interface
         * is in USART mode, we grab the next character received from the MCU
         * `serial0` character device and return it as the register content.
         * 
         * If more data is waiting into our reception queue, we notify the
         * software that more data is waiting by setting the RXC flag (receive
         * complete event) and generating a corresponding IRQ if we have to.
         * 
         */
        case A_SERCOM_DATA:
            {
                /* Are we in USART mode ? */
                if (s->current_mode == SERCOM_USART)
                {
                    /* Do we have at least one character to retrieve ? */
                    if (s->rx_fifo_len > 0)
                    {
                        /* Read character from FIFO. */
                        db = s->rx_fifo[s->rx_fifo_pos];
                        if (s->rx_enabled && s->rx_fifo_len) {
                            s->rx_fifo_pos = (s->rx_fifo_pos + 1) % UART_FIFO_LENGTH;
                            s->rx_fifo_len--;
                            if (s->rx_fifo_len) {

                                /* If we have more data, set RXC bit of INTFLAG register to 1. */
                                s->reg8[R_USART_INTFLAG] |= (1 << R_USART_INTFLAG_RXC_SHIFT);

                                /* Generate an IRQ if required (RXC bit set and RXC interrupt event enabled). */
                                samd21_update_irq(s);
                            }
                            else
                            {
                                /* No more data, set RXC to 0. */
                                s->reg8[R_USART_INTFLAG] &= ~(1 << R_USART_INTFLAG_RXC_SHIFT);
                            }

                            /* Tell QEMU we have read one byte from our character device. */
                            qemu_chr_fe_accept_input(&s->chr);
                        }

                        /* Return the received byte to the application software. */
                        return db;
                    }
                }
                else if (s->current_mode == SERCOM_SPI)
                {
                    /* In SPI mode, we simply read the last received byte store in SERCOM_DATA. */
                    return (s->reg16[R_SERCOM_DATA] & 0xFF);
                }

                /* Not in USART mode or empty FIFO. */
                return 0;
            }
            break;

        /* SERCOM status register read operation. */
        case A_SERCOM_STATUS:
            {
                /* Return the 16-bit value corresponding to the STATUS register. */
                return s->reg16[R_SERCOM_STATUS];
            }
            break;

        /**
         * Other registers are simply read from our registers bank, based on
         * 4-byte aligned read operations.
         */

        default:
            return s->reg[addr/4];
            break;
    }
}

/******************************************************************************
 * The `uart_transmit` function is in charge of sending a byte to QEMU
 * serial character device. This function is triggered by software when the
 * SERCOM USART interface is used to send a byte, and will read the byte
 * sent by the software and forward it to the corresponding QEMU character\
 * device backend to let QEMU process it (for instance, if the character
 * device backend is a serial console bound to stdio, this character will be
 * displayed directly in the console).
 *****************************************************************************/

static gboolean uart_transmit(void *do_not_use, GIOCondition cond, void *opaque)
{
    SAMD21SERCOMState *s = SAMD21_SERCOM(opaque);
    int r;

    /* First, read the 16-bit DATA register and keep only the low 8 bits. */
    uint8_t c = s->reg16[R_SERCOM_DATA];

    /* Set DRE and TXC flag in the INTFLAG register, to tell the software that
       more bytes can be sent after this function call. */
    s->reg8[R_USART_INTFLAG] &= (~(1 << R_USART_INTFLAG_DRE_SHIFT) | (1 << R_USART_INTFLAG_TXC_SHIFT));

    /* Reset watch tag. Watch tags are used for asynchronous character device
       backends that may take some time to forward a byte. */
    s->watch_tag = 0;

    /* Ask QEMU character device backend to write our character to it. */
    r = qemu_chr_fe_write(&s->chr, &c, 1);
    if (r <= 0) {
        /*
         * Write operation to character device failed, we set up a watch in
         * order for this function to be called again and send the same byte
         * later.
         */
        s->watch_tag = qemu_chr_fe_add_watch(&s->chr, G_IO_OUT | G_IO_HUP,
                                             uart_transmit, s);

        /* Could not create a watch tag, abort transmission. */
        if (!s->watch_tag) {
            goto buffer_drained;
        }

        /* Success. */
        return G_SOURCE_REMOVE;
    }

/*
 * Byte has been processed, we set the DRE flag to allow another byte to be
 * written (datasheet states that this flag has to be set to write a byte into
 * the DATA register) and the TXC flag to tell the software we successfully
 * sent one byte onto the physical serial interface.
 * 
 * We then set the DATA register to 0.
 */

buffer_drained:
    s->reg8[R_USART_INTFLAG] |= (1 << R_USART_INTFLAG_DRE_SHIFT);
    s->reg8[R_USART_INTFLAG] |= (1 << R_USART_INTFLAG_TXC_SHIFT);
    s->reg16[R_SERCOM_DATA] = 0;
    s->pending_tx_byte = false;
    return G_SOURCE_REMOVE;
}

/**
 * Cancel USART transmission, especially when the SERCOM peripheral is
 * reset.
 */
static void uart_cancel_transmit(SAMD21SERCOMState *s)
{
    /*
     * If we were expecting the character device backend to call our
     * `uart_transmit` later, then we need to remove the corresponding
     * watch.
     */
    if (s->watch_tag) {
        g_source_remove(s->watch_tag);
        s->watch_tag = 0;
    }
}

/******************************************************************************
 * The `sercom_write` callback process any SERCOM register write operation. We
 * need to process specifically certain registers to correctly emulate the
 * peripheral behavior.
 * 
 * SERCOM CTRLA and CTRLB registers are used to:
 *  - Configure the communication protocol to use: USART, SPI, I2C and I2S. Only
 *    USART and SPI will are supported.
 *  - Enable or disable the reception and transmission operations on an interface
 * 
 * The `INTENSET` and `INTENCLR` registers are used to enable or disable
 * specific interrupt events. These events will then be used by the
 * `samd21_update_irq` function to generate (or not) an IRQ.
 * 
 * When the DATA register is written, we set the provided value into memory
 * and call the `uart_transmit` function if the current mode is set to USART,
 * emulating a real USART character transmit onto the associated character
 * device backend.
 * 
 *****************************************************************************/

static void sercom_write(void *opaque, hwaddr addr,
                       uint64_t value, unsigned int size)
{
    SAMD21SERCOMState *s = SAMD21_SERCOM(opaque);
    uint8_t mode, enabled;

    switch (addr)
    {
        /**
         * Handle CTRLA register write operations. This register is mostly
         * used to configure the protocol used by the SERCOM interface or
         * to reset the peripheral.
         */
        case A_USART_CTRLA:
            {
                /* Do we need to reset this peripheral ? */
                if (value & R_USART_CTRLA_SWRST_MASK)
                {
                    /* Reset required. */
                    s->pending_tx_byte = false;

                    /* Cancel any ongoing byte transmission. */
                    uart_cancel_transmit(s);

                    /* Reset registers values. */
                    memset(s->reg, 0, sizeof(s->reg));

                    /* Flush FIFO, disable RX and TX and disable interface. */
                    s->rx_fifo_len = 0;
                    s->rx_fifo_pos = 0;
                    s->rx_enabled = false;
                    s->tx_enabled = false;
                    s->enabled = false;

                    /* No mode specified. */
                    s->current_mode = SERCOM_UNSUPPORTED;
                }
                else
                {
                    /*
                     * Configuration has been changed, take into account the
                     * requested mode and enable this interface if required.
                     */
                    mode = (value & R_USART_CTRLA_MODE_MASK) >> R_USART_CTRLA_MODE_SHIFT;
                    enabled = (value & R_USART_CTRLA_ENABLE_MASK) >> R_USART_CTRLA_ENABLE_SHIFT;

                    /* Is mode updated ? */
                    switch (mode)
                    {
                        /* USART */
                        case 0:
                        case 1:
                            {
                                /* Current mode is USART. */
                                s->current_mode = SERCOM_USART;

                                /*
                                 * When USART protocol is enabled, we set the
                                 * DRE bit of INTFLAG to 1 to tell the software
                                 * that we are ready to transmit a byte, and
                                 * TXC to 1.
                                 */
                                /* Set DRE to 1 and TXC to 1. */
                                s->reg8[R_USART_INTFLAG] |= 3;
                            }
                            break;

                        case 2:
                        case 3:
                            {
                                /*
                                 * SPI mode selected. Not implemented yet, but
                                 * it will soon allow the software to interact
                                 * with our board SSI bus.
                                 */
                                s->current_mode = SERCOM_SPI;

                                /* Set DRE to 1 and TXC to 1. */
                                s->reg8[R_USART_INTFLAG] |= 3;
                            }
                            break;

                        default:
                            /* Other modes are not supported. */
                            s->current_mode = SERCOM_UNSUPPORTED;
                            break;
                    }

                    /* Enable/disable interface based on value. */
                    s->enabled = (enabled == 1);

                    /* Store value. */
                    s->reg[addr / 4] = value & 0xffffffff;
                }
            }
            break;

        /**
         * Handle CTRLB register write operations. This register is mostly
         * used to enable/disable TX and RX operations on this interface.
         */
        case A_USART_CTRLB:
            {
                /* Keep track of RX/TX status. */
                s->rx_enabled = ((value & R_USART_CTRLB_RXEN_MASK) != 0);
                s->tx_enabled = ((value & R_USART_CTRLB_TXEN_MASK) != 0);

                /* Update register. */
                s->reg[addr / 4] = value & 0xffffffff;
            }
            break;

        /**
         * The DATA register is used to send a byte over USART and SPI.
         * For now, only USART is supported.
         * 
         * When a value is written into the DATA register and the current
         * mode is set to USART, we set this register value and we call
         * the `uart_transmit` function in order to send this byte to the
         * associated QEMU character device.
         */
        case A_SERCOM_DATA:
            {
                switch (s->current_mode)
                {
                    case SERCOM_USART:
                        {
                            /* Write register. */
                            s->reg16[R_SERCOM_DATA] = (value & 0xffff);

                            /* Mark byte as sent. */
                            s->pending_tx_byte = true;
                            uart_transmit(NULL, G_IO_OUT, s);
                        }
                        break;

                    case SERCOM_SPI:
                        {
                            /* Write register. */
                            s->reg16[R_SERCOM_DATA] = (value & 0xffff);

                            /* Transfer through SSI. */
                            sercom_spi_transfer(s);
                        }
                        break;

                    default:
                        break;
                }
            }
            break;


        /**
         * The INTENSET register is used to enable specific interrupt events.
         * 
         * Only the bits set to 1 are of interest, as each bit set will enable
         * the corresponding interrupt event (see section 26.8.6 of the
         * datasheet). 
         */

        case A_USART_INTENSET:
            {
                /* Enable corresponding interrupts. */
                s->interrupts |= (value & 0xff);
            }
            break;

        /**
         * The INTENCLR register is used to disable specific interrupt events.
         * 
         * Only the bits set to 1 are of interest, as each bit set will disable
         * the corresponding interrupt event (see section 26.8.5 of the
         * datasheet).
         * 
         * We just need to apply a negative mask to the interrupts events value
         * to clear the corresponding interrupt events.
         */

        case A_USART_INTENCLR:
            {       
                /* Force selected bits to 0. */
                s->interrupts &= ~(value & 0xff);
            }
            break;

        default:
            /* Update register. */
            s->reg[addr / 4] = value & 0xffffffff;
            break;
    }
    
    samd21_update_irq(s);
}

/**
 * The following `sercom_ops` structure defines the MMIO region read/write
 * handlers as well as some constraints regarding their implementation.
 * 
 * This structure is used when initializing the SERCOM MMIO registers region.
 */

static const MemoryRegionOps sercom_ops = {
    .read =  sercom_read,
    .write = sercom_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4
};


/**
 * Reset the SERCOM peripheral
 */

static void samd21_sercom_reset(DeviceState *dev)
{
    SAMD21SERCOMState *s = SAMD21_SERCOM(dev);

    /* No more pending byte. */
    s->pending_tx_byte = false;

    /* Cancel current transmission, if pending. */
    uart_cancel_transmit(s);

    /* Reset our registers. */
    memset(s->reg, 0, sizeof(s->reg));

    /* Clear FIFO, disabled TX/RX and disabled the interface. */
    s->rx_fifo_len = 0;
    s->rx_fifo_pos = 0;
    s->rx_enabled = false;
    s->tx_enabled = false;
    s->enabled = false;

    /* No mode specified. */
    s->current_mode = SERCOM_UNSUPPORTED;
}


/******************************************************************************
 * QEMU character device callbacks
 *****************************************************************************/

/**
 * The following `uart_receive` function is called by QEMU's character device
 * backend when a byte is received from the corresponding character device.
 * 
 * This byte is then stored in our RX FIFO, and some flags are set into the
 * INTFLAG register to tell the application code that we have just received
 * a byte. An interrupt is generated if conditions are met.
 */

static void uart_receive(void *opaque, const uint8_t *buf, int size)
{
    SAMD21SERCOMState *s = SAMD21_SERCOM(opaque);
    int i;

    /* Sanity checks. */
    if (size == 0 || s->rx_fifo_len >= UART_FIFO_LENGTH) {
        return;
    }

    /* Loop on received bytes and load them into our RX FIFO. */
    for (i = 0; i < size; i++) {
        uint32_t pos = (s->rx_fifo_pos + s->rx_fifo_len) % UART_FIFO_LENGTH;
        s->rx_fifo[pos] = buf[i];
        s->rx_fifo_len++;
    }

    /* Set the RXC bit of INTFLAG to notify the software. */
    s->reg8[R_USART_INTFLAG] |= (1 << R_USART_INTFLAG_RXC_SHIFT);

    /* Generate an IRQ if required. */
    samd21_update_irq(s);
}


/**
 * The following `uart_can_receive` function is called by QEMU's character device
 * backend to check if we can receive a byte.
 * 
 * We simply check that our RX FIFO is not empty and respond with the number of
 * bytes available if some space left, or 0 if our RX FIFO is full.
 */

static int uart_can_receive(void *opaque)
{
    SAMD21SERCOMState *s = SAMD21_SERCOM(opaque);

    return s->rx_enabled ? (UART_FIFO_LENGTH - s->rx_fifo_len) : 0;
}


/**
 * The following `uart_event` function is called by QEMU's character device
 * backend when a specific event occured on QEMU' side, like a closed serial
 * device or a transmission error.
 * 
 * We will handle the CHR_EVENT_BREAK event that notifies an error related to
 * the QEMU character device and reflects this error into our INTFLAG register.
 * 
 * We also generate an IRQ if required.
 */

static void uart_event(void *opaque, QEMUChrEvent event)
{
    SAMD21SERCOMState *s = SAMD21_SERCOM(opaque);

    /* Handle CHR_EVENT_BREAK only. */
    if (event == CHR_EVENT_BREAK) {
        /* Set the ERROR bit of INTFLAG to report the error. */
        s->reg8[R_USART_INTFLAG] |= (1 << R_USART_INTFLAG_ERROR_SHIFT);

        /* Send an IRQ if required. */
        samd21_update_irq(s);
    }
}

/******************************************************************************
 * SAMD21 SERCOM QOM class definition
 *****************************************************************************/

/**
 * @brief This function realizes a SAMD21 SERCOM object by setting the correct
 *        UART callbacks for the associated character device.
 * 
 * @param dev   Pointer to the SAMD21 SERCOM instance state structure
 * @param errp  Pointer to an Error structure to report any error
 */

static void samd21_sercom_realize(DeviceState *dev, Error **errp)
{
    SAMD21SERCOMState *s = SAMD21_SERCOM(dev);

    /* Set QEMU character device backend callbacks to handle UART events. */
    qemu_chr_fe_set_handlers(&s->chr, uart_can_receive, uart_receive,
                            uart_event, NULL, s, NULL, true);
}


/**
 * The `samd21_sercom_init` function initializes a SAMD21 SERCOM peripheral.
 * 
 * It creates the associated MMIO region to deal with registers read/write
 * operations, creates the single IRQ line for this peripheral (that will
 * be later connected to a CPU NVIC GPIO) and sets all the registers default
 * values as well as the peripheral internal state.
 */

static void samd21_sercom_init(Object *obj)
{
    SAMD21SERCOMState *s = SAMD21_SERCOM(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    DeviceState *dev = DEVICE(obj);

    /* Initialize the SERCOM MMIO operation handlers. */
    memory_region_init_io(&s->iomem, obj, &sercom_ops, s,
                          "samd21.sercom", SERCOM_SIZE);

    /* Associate the MMIO region with the device system bus. */
    sysbus_init_mmio(sbd, &s->iomem);

    /* Initialize a new IRQ associated with the device system bus. */
    sysbus_init_irq(sbd, &s->irq);

    /* Initialize a new SSI bus. */
    s->ssi = ssi_create_bus(dev, "ssi");

    /* Initialize 16-bit and 8-bit registers arrays. */
    s->reg16 = (uint16_t *)s->reg;
    s->reg8 = (uint8_t *)s->reg;

    /* Reset controller status. */
    s->reg16[R_SERCOM_STATUS] = 0x0000;
    s->pending_tx_byte = false;

    /* Cancel transmission. */
    uart_cancel_transmit(s);

    /* Reset registers. */
    memset(s->reg, 0, sizeof(s->reg));

    /* Flush FIFO, disable RX/TX, disable the device. */
    s->rx_fifo_len = 0;
    s->rx_fifo_pos = 0;
    s->rx_enabled = false;
    s->tx_enabled = false;
    s->enabled = false;

    /* No mode specified. */
    s->current_mode = SERCOM_UNSUPPORTED;
}

/**
 * SAMD21 SERCOM class properties.
 * 
 * This structure defines the only property exposed by this class,
 * the "chardev" property that allows the calling code to set the
 * character device to use when dealing with USART.
 * 
 */

static Property samd21_sercom_properties[] = {
    DEFINE_PROP_CHR("chardev", SAMD21SERCOMState, chr),
    DEFINE_PROP_END_OF_LIST(),
};

/**
 * The `samd21_sercom_class_init` function is in charge of initializing
 * our class object:
 * 
 *  - It sets the class `reset` and `realize` callbacks.
 *  - It configures the properties associated with the class.
 * 
 * These callbacks will be later called when an object of the SAMD21 SERCOM
 * class is instanciated.
 */

static void samd21_sercom_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = samd21_sercom_reset;
    dc->realize = samd21_sercom_realize;
    device_class_set_props(dc, samd21_sercom_properties);
}

/**
 * The following structure defines our class definition:
 * 
 *  - It defines our type name ("samd21.sercom").
 *  - It defines the parent class type (a system bus device).
 *  - It defines the instance size, basically the size of our state structure.
 *  - It defines the class init callback to `samd21_sercom_class_init` and the
 *    instance init callback (`samd21_sercom_init`).
 * 
 */
static const TypeInfo samd21_sercom_info = {
    .name = TYPE_SAMD21_SERCOM,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SAMD21SERCOMState),
    .instance_init = samd21_sercom_init,
    .class_init = samd21_sercom_class_init
};


/**
 * The following `samd21_sercom_register_types` function is in charge of
 * registering all the types required by this module.
 * 
 * In fact, it only registers our class based on its type description.
 */

static void samd21_sercom_register_types(void)
{
    /* Register our "samd21.sercom" device type. */
    type_register_static(&samd21_sercom_info);
}

/* Register this module types registration callback. */
type_init(samd21_sercom_register_types)