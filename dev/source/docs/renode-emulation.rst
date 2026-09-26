.. _renode-emulation:

=====================
Renode MCU Emulation
=====================

`Renode <https://renode.io>`__ runs **unmodified** ArduPilot STM32 firmware - the
same ``.elf``/``.apj`` you would flash to a board - on an emulated MCU on your
development machine. Unlike :ref:`SITL <sitl-simulator-software-in-the-loop>`,
which compiles ArduPilot as a native host program, Renode executes the real
ChibiOS build, so the real driver code, DMA setup and bus handling are
exercised rather than replaced by host stubs. The board's own bootloader and the
independent watchdog can be brought into the picture as well, each behind its own
option.

This makes it possible to reproduce and debug board-specific problems without
owning the board.

.. note::

   Renode support is a developer tool. It is not intended for flight testing or
   for tuning, and its peripheral models are functional rather than
   physically accurate.

What is emulated
================

Renode board descriptions are **generated from the board's normal**
``hwdef.dat`` (and ``hwdef-bl.dat`` where the target has one) at launch time,
using the same hwdef compiler that ``./waf configure`` uses. AP_Periph targets
without a bootloader hwdef reuse their application hwdef. There are no
hand-maintained per-board Renode descriptions to keep in sync - only one
hand-maintained base platform per MCU family, which every board of that family
shares.

Support is therefore driven by MCU family rather than by a board allowlist, and
covers STM32 F1, F3, F4, F7, G4, H7 and L4 targets. All 109 STM32 AP_Periph
hwdefs pass the generation audit, which checks that their expanded configuration
translates into a valid Renode platform; booting and running is verified
separately on representative targets rather than on every board.

The generated model provides:

- MCU, flash layout, serial order, timers, buses, DMA, chip selects and SDMMC,
  taken from the compiled hwdef
- representative IMU, barometer, compass, GPS, airspeed, rangefinder, battery,
  IOMCU, RAMTRON, dataflash and SD card models
- persistent internal flash, FRAM/RAMTRON, crash dump data and microSD contents
  across restarts
- UART/MAVLink, DroneCAN multicast and, on H743/H757 boards that declare an
  ``ETH1`` hwdef entry, Ethernet connectivity

Run ``Tools/renode/run.py --list`` to see the targets the generator accepts.

Installing Renode
=================

ArduPilot uses a patched Renode build; the stock release is usable but
substantially slower. A prebuilt package for Linux x86_64 and aarch64 can be
fetched with:

.. code-block:: bash

    Tools/renode/tests/fetch_renode.sh build/renode

The graphical launcher can also download and update it for you with its
**Download Renode** button, caching it under ``~/.cache/ardupilot/renode/``.

``run.py`` finds the emulator from ``--renode``, the ``RENODE`` environment
variable, a gitignored ``Tools/renode/renode`` symlink, ``PATH``, and finally
``/opt/renode/renode`` or ``/usr/local/bin/renode``. The symlink is the usual way
to pin a locally built, patched Renode without any environment setup. It also downloads the SVD file matching the board's exact
MCU from ``firmware.ardupilot.org`` and caches it alongside.

To build Renode yourself, clone the ArduPilot performance branch
(``pr-arudpilot-am32-perf``), run ``git submodule update --init --recursive``,
apply ``./perf_patches/apply.sh`` and build with ``./build.sh --net`` using a
.NET 10 SDK.

Running firmware
================

Build the board's firmware normally, then launch it:

.. code-block:: bash

    ./waf configure --board KakuteF4 && ./waf copter
    Tools/renode/run.py KakuteF4

This drops you into the interactive Renode monitor with the firmware running.
The first hardware UART in the board's ``SERIAL_ORDER`` is served on
``tcp:localhost:5762``, so a GCS connects with:

.. code-block:: bash

    mavproxy.py --master tcp:localhost:5762

That socket serves one client only; to reconnect a GCS, restart Renode.

Useful options:

- ``--serial N`` selects a different ``SERIAL_ORDER`` entry
- ``--uds`` uses a Unix domain socket instead of a TCP port; ``run.py`` prints
  the exact MAVProxy endpoint to use
- ``--elf`` (or ``--firmware``) overrides the firmware; APJ, raw BIN, Intel HEX
  and ELF are accepted
- ``--bootloader`` loads a real ArduPilot bootloader and starts from its vector
  table
- ``--port N`` gives a telnet monitor instead of the console, and ``--exec``
  appends monitor commands

.. note::

   Exposing a UART does not change firmware parameters. The selected serial port
   must still have the desired ``SERIALn_PROTOCOL`` configured.

An ELF can also be downloaded from the official firmware server instead of being
built locally:

.. code-block:: bash

    Tools/renode/renode_firmware.py CubeBlack --vehicle Copter --channel latest

Graphical launcher
==================

``Tools/renode/launch.py`` is a graphical chooser around ``run.py``:

.. code-block:: bash

    Tools/renode/launch.py

The **Target** tab discovers supported boards and their built ELF images, picks
a matching bootloader, and exposes CPU pinning, real IOMCU, UDS, USB/DFU, CAN
and Ethernet TAP options.

The **Config** tab expands the selected board's ``hwdef.dat`` and lists its
``SERIAL_ORDER``, ``I2C_ORDER`` and ``CAN_ORDER`` ports, so simulated devices can
be attached to the logical ArduPilot port rather than to a raw peripheral. The
initial device catalog includes u-blox GPS and Benewake/LightWare rangefinders on
UARTs, an IST8310 compass and MS4525/AUAV/ASP5033 airspeed sensors on I2C, and a
DroneCAN airspeed node on CAN.

Devices can be attached, removed or switched off **while the firmware is
running**, which allows testing driver probing, hot-plug and runtime
initialisation without rebooting the flight controller. The attachment list and
all tab selections are saved to ``launch-settings.json`` in the directory the
launcher was started from.

The runtime panel reports the current PC, configured and executed MIPS, and the
virtual-time speedup; paced execution targets ``1.00x realtime``.

Flight dynamics
===============

A standalone physics sidecar reuses the existing ``libraries/SITL`` model factory
and dynamics, exchanging actuator and sensor state with Renode over a localhost
lockstep protocol:

.. code-block:: bash

    ./waf configure --board sitl
    ./waf --targets tool/renode-physics
    build/sitl/tool/renode-physics --model quad --physics-port 9002

Quadcopter, plane, rover and quadplane models are offered, with latitude,
longitude, altitude, heading and rate controls in the launcher's **Physics** tab.
Physics can be connected and disconnected while the firmware runs, so resets and
runtime sensor initialisation can be tested without resetting the vehicle state.

Debugging with GDB
==================

Build with debug symbols and pass ``--gdb``:

.. code-block:: bash

    ./waf configure --board CubeBlack -g
    ./waf copter
    Tools/renode/run.py CubeBlack --gdb

Renode starts the machine when GDB attaches. A ChibiOS-aware adapter sits in
front of Renode's GDB server, so once ``chSysInit()`` has run, ``info threads``,
``thread N`` and ``thread apply all bt`` show the real guest threads and their
saved Cortex-M contexts. ``--gdb-port`` selects the adapter port (3333 by
default), and ``--no-xterm`` prints the attach script to run in another terminal
instead of opening one.

Adding ``--reverse-debug`` enables bounded reverse execution, so GDB's
``reverse-step``, ``reverse-stepi`` and ``reverse-continue`` (``rs``, ``rsi``,
``rc``) can be used to run backwards from a fault. ``--reverse-gdb-limit`` bounds
the retained history by guest instruction count and defaults to 1000; zero
selects Renode's own unlimited history. The bounded mode requires the patched
ArduPilot Renode branch - on a stock build, use zero. Reverse execution is single core, and ``rs`` may
require several restore/replay cycles - use ``rsi`` to step back exactly one
machine instruction.

See also :ref:`debugging-with-gdb`.

Logic analyser output
=====================

``--sigrok`` exposes a continuous logic-analyser stream that
`PulseView <https://sigrok.org/wiki/PulseView>`__ can read:

.. code-block:: bash

    Tools/renode/run.py Pixhawk6X --sigrok
    pulseview -d renode-la:conn=tcp/127.0.0.1/4242

The capture contains the main MAVLink UART's TX and RX, the first SPI bus
(SCK, MOSI, MISO and every chip select) and every pin with a ``GPIO(n)``
assignment in the compiled hwdef, named with their physical pin, hwdef signal and
logical GPIO names. ``--sigrok-channels`` limits the advertised channels using
shell wildcards, ``--sigrok-sample-rate`` changes the 10 MHz default, and
``--sigrok-port`` changes the port.

Every hwdef ``GPIO(n)`` is also wired to an on-demand stimulus, which can generate
RPM pulse trains or quadrature wheel-encoder signals from the Renode monitor:

.. code-block:: text

    sysbus.gpioStimulus StartPulse 1 25
    sysbus.gpioStimulus StartQuadrature 1 2 10 false
    sysbus.gpioStimulus StopAll

CAN and DroneCAN
================

``--can`` connects the generated CAN1 and CAN2 peripherals to separate ArduPilot
multicast buses, available to DroneCAN tooling as ``mcast:0`` and ``mcast:1``:

.. code-block:: bash

    Tools/renode/run.py CubeOrange --can

Configure the CAN drivers as usual and reboot:

.. code-block:: text

    param set CAN_P1_DRIVER 1
    param set CAN_D1_PROTOCOL 1
    param set CAN_P2_DRIVER 2
    param set CAN_D2_PROTOCOL 1
    reboot

AP_Periph targets open their CAN buses by default, since CAN is their primary
transport. ``--can-base N`` moves CAN1 to ``mcast:N``, which is useful when
running tests in parallel.

USB and firmware upload
=======================

On Linux, ``--usb`` exports the firmware-driven F4 or H7 USB controller through
Renode's USB/IP server, so the emulated board appears as a real
``/dev/ttyACM*`` device with entries under ``/dev/serial/by-id/``. Run the helper
in a second terminal:

.. code-block:: bash

    Tools/renode/run.py CubeOrange --usb \
        --bootloader Tools/bootloaders/CubeOrange_bl.bin --hold-bootloader
    Tools/renode/usbip_attach.py

Perform the one-time udev setup first so the helper can run as a normal user:

.. code-block:: bash

    sudo Tools/renode/usbip_attach.py --install-rules

Firmware can then be uploaded through the same host path used for real hardware:

.. code-block:: bash

    Tools/scripts/uploader.py \
        --port /dev/serial/by-id/usb-Hex_ProfiCNC_CubeOrange-BL_*-if00 \
        build/CubeOrange/bin/arducopter.apj

Leave execution paced while uploading. Under ``--unthrottled``, guest time can
run past the bootloader's receive timeout while the host is between USB writes.

Passing ``--usb --dfu`` exposes an STM32 factory-ROM-compatible DfuSe device
instead, which writes the board's persistent flash image before Renode starts.
``--dfu`` requires ``--usb`` and cannot be combined with ``--bootloader`` or
``--gdb``.

.. warning::

   Renode's USB/IP server listens on all host interfaces. Keep its TCP port
   firewalled from untrusted networks.

Ethernet
========

Pixhawk6X is the reference :ref:`Ethernet <copter:common-network>` target. Enable
networking on the firmware and reboot:

.. code-block:: text

    param set NET_ENABLE 1
    reboot

``--ethernet-tap`` attaches the emulated MAC to a named host TAP interface,
which can be bridged to the physical LAN:

.. code-block:: bash

    sudo ip tuntap add dev tap-renode mode tap user "$USER"
    sudo ip link set tap-renode master br0
    sudo ip link set tap-renode up
    Tools/renode/run.py Pixhawk6X --ethernet-tap tap-renode

.. warning::

   Bridging exposes the emulated autopilot directly to the local network. Use the
   same firewall precautions as for physical flight controller hardware.

Real IOMCU
==========

H7 boards with an ``IOMCU_UART`` can run the real STM32F100 IOMCU bootloader and
firmware in a second emulated machine, connected to the FMU through a paced UART
hub:

.. code-block:: bash

    Tools/renode/run.py CubeOrangePlus --real-iomcu \
        --bootloader Tools/bootloaders/CubeOrangePlus_bl.bin

The IOMCU application comes from the FMU ELF's embedded ``io_firmware.bin``, and
its full 64 KiB flash persists between runs. Safety switch operation can then be
exercised over the normal FMU-to-IOMCU register path with MAVProxy's
``arm safetyoff`` and ``arm safetyon``, and the FMU's ``IOMC`` log message records
the real IOMCU status. The faster synthetic IOMCU remains the default.

Automated tests
===============

``test_all.py`` discovers firmware already built under ``build/*/bin`` and runs a
boot and parameter-download check against each:

.. code-block:: bash

    Tools/renode/test_all.py --parallel 4
    Tools/renode/test_all.py 'HolybroG4*' --parallel 2
    Tools/renode/test_all.py 'CubeBlack/arducopter'

Flight controller tests wait for a heartbeat and require a complete ``param ftp``
download; AP_Periph tests allocate a DroneCAN node ID and walk the parameter
table. Each worker gets private flash, SD card, Renode configuration and TCP
ports, and AP_Periph workers additionally get their own multicast CAN buses.

A GitHub Actions job (``.github/workflows/test_renode.yml``) runs the
``Tools/renode/tests`` unit tests and then flies a CubeOrangePlus quadplane
mission against the physics sidecar. The CubeOrange mission test that downloads
a DataFlash log over MAVLink is run locally rather than in CI.

Limitations
===========

- STM32 targets only; there is no emulation of Linux, ESP32 or QURT boards
- peripheral models are functional, not high-fidelity electrical or physical
  simulations
- USB/IP, the DFU device and TAP networking are Linux only
- a TCP-exposed UART serves a single client; reconnecting means restarting Renode
- execution is paced to wall-clock time by default, and a demanding board or
  workload may not keep up. ``--unthrottled`` removes the pacing for benchmarks
  and batch testing, but must not be used for firmware upload over USB

Further information
===================

``Tools/renode/README.md`` in the ArduPilot source tree is the authoritative
reference, and includes the full option list, the peripheral model inventory,
performance analysis and the list of Renode bugs found during bring-up.
