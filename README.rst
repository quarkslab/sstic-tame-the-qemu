Tame the (Q)emu - SSTIC 2024
============================

*QEMU's original README.rst has been renamed to `README_ORIGINAL.rst <README_ORIGINAL.rst>`_*.

Introduction
------------

This repository is based on QEMU 8.2 stable branch and adds support for a custom
board based on a Microchip ATSAMD21G18A microcontroller (MCU) with heavily documented
source code. The goal of this project is to provide an example of implementation
of a new MCU and its dedicated peripherals, as well as a specific electronic board
with external components.

This project is part of a short talk given at the French *Symposium sur la Sécurité des 
Technologies de l'Information et de Communication* event in 2024. Slides, video recording
and article (PDF) of this talk are available `on the talk's dedicated page <https://www.sstic.org/2024/presentation/tame_the_qemu_debug_firmware_on_custom_emulated_board/>`_ (slides and article in English, talk in French).

Files added by our implementation
---------------------------------

* `hw/arm/samd21.c`
* `hw/arm/qblilboard.c`
* `hw/gpio/samd21_gpio.c`
* `hw/misc/samd21_sercom.c`
* `include/hw/samd21_mcu.h`
* `include/hw/samd21.h`
* `include/gpio/samd21_gpio.h`
* `include/misc/samd21_sercom.h`

Important note
--------------

This repository being part of a tutorial published in the context of a conference, the provided
code will not be maintained and is given as an example implementation compatible with QEMU
version 8.2. It will not be updated/adapted to any future version of QEMU and will not reflect
the future changes that may occur in QEMU's API.

Nevertheless, QEMU basic principles (QEMU Object Model, memory management, GPIO and IRQs) are not
expected to change a lot in future QEMU development.

