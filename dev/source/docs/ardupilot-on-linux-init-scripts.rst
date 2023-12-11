.. _ardupilot-on-linux-init-scripts:

============================
Linux Initialization Scripts
============================


Example Systemd Unit Files
==========================

Adjust as needed and place in /etc/defaults/ardupilot

::

    # ArduPilot configuration options

    # Serial port configuration
    # Uncomment and update as needed
    # See "Startup Options" under "ArduPilot on Linux" in dev wiki for more information.
    #SERIAL0=--serial0 tcp:192.168.1.26:12345:nowait
    #SERIAL1=--serial1 /dev/ttyS0
    #SERIAL2=
    #SERIAL3=
    #SERIAL4=
    #SERIAL5=
    #SERIAL6=
    #SERIAL7=
    #SERIAL8=
    #SERIAL9=

    # change log directory (Optional)
    #LOG_DIRECTORY=--log-directory /var/APM/logs

    # change terrain directory (Optional)
    #TERRAIN_DIRECTORY=--terrain-directory /var/APM/terrain

    # change storage path (Optional)
    #STORAGE_DIRECTORY=--storage-directory /var/APM/storage

    # Configure CPU affinity (Optional)
    #CPU_AFFINITY=--cpu-affinity 1 #a single CPU, or range(1-3)

This board-specific one-shot script should contain any system-level
configuration required for running ardupilot. Adjust as needed, and
place in /etc/systemd/system/ardupilot-sysconfig.service .

::

    [Unit]
    Description=Configure system for realtime operation

    [Service]
    Type=oneshot

    # Lock CPU governor to top speed for reduced jitter caused by clock tree changes
    # For more information, https://www.kernel.org/doc/Documentation/cpu-freq/governors.txt
    ExecStart=/bin/sh -c "/bin/echo performance | tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor"

This build-type specific script contains the startup options.
Adjust as needed(and per-build), and place in /etc/systemd/system/arduplane.service

::

    [Unit]
    Description=ArduPlane Service
    After=ardupilot-sysconfig.service
    Requires=ardupilot-sysconfig.service

    # Do not attempt to run multiple ardupilot services at a time,
    # as they will almost certainly conflict.
    Conflicts=arducopter.service ardupilot.service ardurover.service

    [Service]
    #Run arduplane using environment configured in /etc/default/ardupilot
    EnvironmentFile=/etc/default/ardupilot

    #  See "Startup Options" under "ArduPilot on Linux" in dev wiki for more information.
    #  Configure the serial ports according to board.
    #  Configure cpu-affinity according to board or remove.
    ExecStart=/usr/local/bin/arduplane \
        $SERIAL0 \
        $SERIAL1 \
        $SERIAL2 \
        $SERIAL3 \
        $SERIAL4 \
        $SERIAL5 \
        $SERIAL6 \
        $SERIAL7 \
        $SERIAL8 \
        $SERIAL9 \
        $LOG_DIRECTORY \
        $TERRAIN_DIRECTORY \
        $STORAGE_DIRECTORY \
        $CPU_AFFINITY

    Restart=on-failure

    [Install]
    WantedBy=multi-user.target

To start, ``sudo systemctl start arduplane``.
To check on it, ``sudo systemctl status arduplane``.
To restart, ``sudo systemctl restart arduplane``.
To stop, ``sudo systemctl stop arduplane``.
To enable service startup at boot, ``sudo systemctl enable arduplane``
Note that only one build must be running at a time.
