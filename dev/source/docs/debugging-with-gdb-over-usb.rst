.. _debugging-with-gdb-over-usb:

=====================================
Debugging with GDB over USB (STM32H7)
=====================================

ArduPilot 4.8 and later can include an experimental GDB debugger that runs over the autopilot's USB port, so firmware can be debugged on the bench without a :ref:`debug probe <debugging-with-gdb-on-stm32>`. It is disabled by default and must be enabled when building the firmware.

The full reference, including VS Code and WSL setup, is in `Tools/debug/README.md <https://github.com/ArduPilot/ardupilot/blob/master/Tools/debug/README.md>`__ in the ArduPilot source.

.. warning:: This is for bench debugging only. Interrupts, telemetry and command processing stop while the target is halted, and IOMCU and peripheral timeouts may occur. Reboot the autopilot after a debug session before using the vehicle.

Requirements
============

- An STM32H7 autopilot that provides two USB CDC interfaces on USB OTG1. The second interface is reserved for GDB.
- Boards with external flash or an external watchdog are not supported.
- Python 3 with ``pyserial``, and ``arm-none-eabi-gdb``.

Building the Firmware
=====================

Configure with ``--enable-USB-debug`` and debug symbols, then build and upload as normal. For example:

::

    ./waf configure --board CubeOrangePlus --enable-USB-debug --debug-symbols
    ./waf plane --upload

By default the debugger can only attach once the vehicle's main loop is running. To debug vehicle setup, also add ``--enable-USB-debug-startup-wait``. The firmware then waits, with no timeout, for the debugger to attach after USB and the scheduler start. Set breakpoints, then ``continue`` to run setup. Failures before USB starts still need a debug probe or a :ref:`crash dump <copter:crash_dump>`.

Attaching GDB
=============

Run the launcher with the exact ELF file that was uploaded and the autopilot's second USB interface. On Linux use the persistent ``by-id`` path ending in ``-if02``. On Windows use the second COM port, and on macOS the second ``/dev/cu.usbmodem...`` device.

::

    python3 Tools/debug/gdb_usb.py build/CubeOrangePlus/bin/arduplane \
        --port /dev/serial/by-id/usb-CubePilot_CubeOrange+_<SERIAL>-if02

``python3 -m serial.tools.list_ports -v`` lists the available ports.

The launcher asks the firmware to attach, waits for USB to re-enumerate, and starts GDB. Use ``--no-break`` to reconnect to a session that is already running.

The first USB interface stays available for a GCS while the target runs. At a breakpoint its connection stays open, but telemetry pauses until ``continue``. Attaching and detaching re-enumerate both USB interfaces, so the GCS must reconnect after each.

An attached hardware debug probe prevents USB debugger attachment.

What Works
==========

- Integer and floating-point registers, RAM reads and writes, and internal flash reads
- ChibiOS thread lists and backtraces (``info threads``, ``thread apply all bt``)
- Hardware breakpoints in flash, software breakpoints in RAM, and data watchpoints (``watch``, ``rwatch``, ``awatch``)
- Single instruction stepping (``stepi``) and Ctrl-C to stop the running firmware
- Simple function calls such as ``p AP_HAL::millis()``
- Stopping at HardFault, MemManage, BusFault and UsageFault after attachment. ``monitor fault`` shows the fault registers.
- Debugging while armed, including breakpoints in code that only runs when armed

``detach`` resumes the firmware and restores normal USB. ``monitor reset`` reboots the autopilot.

Limitations
===========

- Flash cannot be programmed through the debugger.
- Do not place breakpoints in NMI or HardFault handlers, or in code that runs with interrupts disabled. The debug event can escalate to a HardFault.
- To continue from a breakpoint in an interrupt handler, delete that breakpoint first.
- Stepping into interrupt handlers or fault stops is not supported.
- Optimised builds may not keep every local variable.
- If the debugger stops with no valid GDB connection, the autopilot reboots after 30 seconds.

VS Code
=======

``.vscode/usb-debug.code-workspace`` in the ArduPilot source provides a **USB: Attach** debug configuration for Linux, Windows and macOS. It prompts for the ELF, USB port, Python and GDB. Use **Detach** to end a session. VS Code's **Stop** button resets the board instead.

``.vscode/usb-debug-wsl.code-workspace`` supports running GDB in WSL while the autopilot's USB stays connected to Windows, so a Windows GCS can still use the first USB interface. See the `README <https://github.com/ArduPilot/ardupilot/blob/master/Tools/debug/README.md>`__ for the setup steps.
