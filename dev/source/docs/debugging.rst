.. _debugging:

=========
Debugging
=========

This section is for topics related to debugging ArduPilot source code.

Debug Output
------------

There are three ways to easily get debug output from your code - so called "printf debugging".

MAVLink messages
................

These messages will appear on Ground Control Stations (and will also appear in DataFlash logs).

Most places in the code you can insert a single statement to send a text message to the GCS, using the GCS singleton:

::

   gcs().send_text(MAV_SEVERITY_WARNING, "Best number ever: %f", 42.4242424242);


DataFlash Log messages
......................

The DataFlash singleton can be used to put messages to the DataFlash log:

::

   DataFlash_Class::instance()->Log_Write_MessageF("u-blox %d HW: %s SW: %s",
                                                   state.instance+1,
                                                   _version.hwVersion,
                                                   _version.swVersion);

More complicated messages (with multiple fields of differing types) can be created with Log_Write:

::

    DataFlash_Class::instance()->Log_Write("CTRL", "TimeUS,RMSRollP,RMSRollD,RMSPitchP,RMSPitchD,RMSYaw", "Qfffff",
                                           AP_HAL::micros64(),
                                           (double)sqrtf(_control_monitor.rms_roll_P),
                                           (double)sqrtf(_control_monitor.rms_roll_D),
                                           (double)sqrtf(_control_monitor.rms_pitch_P),
                                           (double)sqrtf(_control_monitor.rms_pitch_D),
                                           (double)sqrtf(_control_monitor.rms_yaw));

See DataFlash.h for the definition of this method.  Generally, it's a 4-character name followed by a comma-separated list of column names, followed by a list of list of format specifiers for each column, followed by the data.  Similar definitions allow addition of multiplier and unit information.

fprintf(stderr, ...)
....................

Everybody's favourite go-to debugging tool.

Where the output of this command goes is dependendant on which hardware you are running on.

In SITL you can see this output in the xterm Window running the ArduPilot binary - or, on Windows, in the log file created in place of that window.

On Linux this output will appear on ArduPilot's controlling TTY - so in whatever terminal you started ArduPilot in.

On NuttX-based ports, the output will appear 


hal.console->printf(...);
.........................

...


.. toctree::
    :maxdepth: 1

    Logs <common-logs>
    Debugging with send_text <debug-with-send-text>
    Using MAVExplorer for log analysis <using-mavexplorer-for-log-analysis>
    Finding the Particular Commit which Introduced a Bug <finding-the-particular-commit-which-introduced-a-bug>
    Debugging with GDB <debugging-with-gdb>
    Interfacing with Pixhawk using the NSH <interfacing-with-pixhawk-using-the-nsh>
    LTTng Tracing <using-linux-trace-toolkit-ng-lttng-to-trace-ardupilot-in-realtime>
