.. _debugging:

=========
Debugging
=========

This section is for topics related to debugging ArduPilot source code.

There is a series of videos introducing some debugging techniques for ArduPilot code:

.. youtube:: 5nQiY_q_ZVA

.. youtube:: 8fIndjwc_-Y

Debug Output
------------

There are three ways to easily get debug output from your code - so called "printf debugging".

MAVLink messages
................

These messages will appear on Ground Control Stations (and will also appear in onboard log logs).

Most places in the code you can insert a single statement to send a text message to the GCS, using the GCS singleton:

::

   GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Best number ever: %f", 42.4242424242);


Onboard Log messages
....................

The AP_Logger singleton can be used to put messages to the onboard log:

::

    AP::logger().Write_MessageF("u-blox %d HW: %s SW: %s",
                                       state.instance+1,
                                       _version.hwVersion,
                                       _version.swVersion);

More complicated messages (with multiple fields of differing types) can be created with Log_Write:

::

    AP::logger().Write("CTRL", "TimeUS,RMSRollP,RMSRollD,RMSPitchP,RMSPitchD,RMSYaw", "Qfffff",
                                           AP_HAL::micros64(),
                                           (double)safe_sqrt(_control_monitor.rms_roll_P),
                                           (double)safe_sqrt(_control_monitor.rms_roll_D),
                                           (double)safe_sqrt(_control_monitor.rms_pitch_P),
                                           (double)safe_sqrt(_control_monitor.rms_pitch_D),
                                           (double)safe_sqrt(_control_monitor.rms_yaw));

See AP_Logger.h for the definition of this method.  Generally, it's a 4-character name followed by a comma-separated list of column names, followed by a list of list of format specifiers for each column, followed by the data.  Similar definitions allow addition of multiplier and unit information.

printf(...) and friends
.......................

Everybody's favourite go-to debugging tool.

Availability and where the output of this command goes is dependent on which hardware you are running on.

In SITL you can see stderr and stdout output in the xterm Window running the ArduPilot binary - or, on Windows, in the log file created in place of that window.

On Linux this output will appear on ArduPilot's controlling TTY - so in whatever terminal you started ArduPilot in.

The stderr and stdout file handles are not available on ChibiOS, so fprintf is not an option.  However, printf(...) output is either sent to a specified serial device (see "STDOUT_SERIAL" in hwdef.dat), or to the console (the first-defined SERIAL device).


hal.console->printf(...);
.........................


This output appears on the console.  For ChibiOS-based boards this is the first-defined SERIAL device.

Catching memory overruns
........................

You can place guard regions around memory areas in the code to catch overruns by configuring the build using the ----enable-malloc-guard option on "waf configure" command prior to building the code for SITL

GPIO Assert Debugging
.....................

It is possible to output debug assertions from ChibiOS (so the debug errors that you get if you build with --enable-asserts) using a GPIO pin. To do this you need to set "FAULT" in hwdef.dat to the GPIO pin you want to use and then connect a logic analyzer to the pin. You will also need to configure your logic analyzer to interpret the serial stream and print readable information but most modern logic analyzers are able to do this automatically.

You can also use this pin defintion to output general debug messages using fault_printf(). This works in exactly the same way as printf() but outputs to the GPIO pin. Both of these methods are extremely useful for debugging situations where general IO is not available.


.. toctree::
    :maxdepth: 1

    Logs <common-logs>
    DroneCAN Debugging <https://www.youtube.com/watch?v=F0H4Y5V3lfo>
    Debugging with send_text <debug-with-send-text>
    Using MAVExplorer for log analysis <using-mavexplorer-for-log-analysis>
    Finding the Particular Commit which Introduced a Bug <finding-the-particular-commit-which-introduced-a-bug>
    Debugging with GDB <debugging-with-gdb>
    Watchdog Debugging <watchdog-debugging>
    LTTng Tracing <using-linux-trace-toolkit-ng-lttng-to-trace-ardupilot-in-realtime>
    Static Analysis <static-analysis>
