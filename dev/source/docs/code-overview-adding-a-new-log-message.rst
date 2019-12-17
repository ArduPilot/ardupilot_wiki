.. _code-overview-adding-a-new-log-message:

========================
Adding a new Log Message
========================

:ref:`DataFlash Logs <common-logs>` are stored on the flight controller's
onboard dataflash memory and can be downloaded after a flight.  The logs
provide detailed information about each flight which can be important
especially when trying to :ref:`diagnose why something went wrong <common-diagnosing-problems-using-logs>`.

This page explains how to write a new dataflash log message.

The Easy Way
------------

Use `AP::logger().Write(...) <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Logger/AP_Logger.h#L278>`__:

   .. code-block:: python

       void Write(const char *name, const char *labels, const char *fmt, ...);
       void Write(const char *name, const char *labels, const char *units, const char *mults, const char *fmt, ...);

An example of the top function being used can be found in `Compass_learn.cpp <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Compass/Compass_learn.cpp#L99>`__:

   .. code-block:: python

       AP::logger().Write("COFS", "TimeUS,OfsX,OfsY,OfsZ,Var,Yaw,WVar,N", "QffffffI",
                                               AP_HAL::micros64(),
                                               (double)best_offsets.x,
                                               (double)best_offsets.y,
                                               (double)best_offsets.z,
                                               (double)best_error,
                                               (double)best_yaw_deg,
                                               (double)worst_error,
                                               num_samples);

- the 1st argument is the message name.  This should be 4 characters or less and be unique
- the 2nd argument is a comma separated list of up to 16 field names; with a limit of 64 characters in total
- the 3rd argument is a format string (maximum of 16 characters) with each letter holding the format for the corresponding field.  The meaning of each letter can be found `here in AP_Logger/LogStructure.h <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Logger/LogStructure.h#L6>`__
- the remaining arguments are the actual values that will be logged.
  You may notice in the example above, some fields have a format of float ("f") but are cast to ``(double)`` this is correct and necessary to avoid a compiler warning.

The 2nd Log_Write function is the same as the first except that it accepts two additional string arguments,
`"units" <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Logger/LogStructure.h#L42>`__ and
`"mults" <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Logger/LogStructure.h#L86>`__.
Similar to the "format" argument, each character in these arguments specifies the
`units <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Logger/LogStructure.h#L42>`__ (i.e. "d" for degrees) or
`multiplier <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Logger/LogStructure.h#L86>`__ (i.e. "2" for \*100, "B" for \*0.01) for the following fields.
These help the graphing tools scale the output correctly when displaying to the user.

For example, below is a "TEST" log message which outputs the current system time and altitude.

   .. code-block:: python

       AP::logger().Write("TEST", "TimeUS,Alt",
                          "sm", // units: seconds, meters
                          "FB", // mult: 1e-6, 1e-2
                          "Qf", // format: uint64_t, float
                          AP_HAL::micros64(),
                          (double)alt_in_cm);

- the time has units of seconds ("s"), multiplier of "F" to indicate the value should be divided by 1 million and format of "Q" to indicate it is output as a 64 bit unsigned integer.
- the altitude has units of meters ("m"), multiplier of "B" to indicate the value should be divided by 100 (to convert from centimeters to meters) and format of "f" because the value is a float.

The Harder Way
--------------

For commonly used messages, especially those which are output at a relatively high rate (50hz or more) a slightly more efficient logging method can be used.

- decide if the logging will come from a library (i.e. the call to Log_Write will from somewhere within
  the `libraries directory <https://github.com/ArduPilot/ardupilot/tree/master/libraries>`__) or
  the vehicle code (i.e. from within `ArduCopter <https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter>`__, `ArduPlane <https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane>`__, etc)
- add a new entry to the `vehicle's #define list or enum in defines.h <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/defines.h#L222>`__ or `AP_Logger/LogStructure.h's LogMessages enum <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Logger/LogStructure.h#L1566>`__.
- define a structure to hold the values to be logged in either the vehicle's `Log.cpp <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/Log.cpp>`__ file
  or in `AP_Logger/LogStructure.h <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Logger/LogStructure.h>`__.  All log file messages should have time_us as their first field.

   .. code-block:: python

       struct PACKED log_Test {
           LOG_PACKET_HEADER;
           uint64_t time_us;
           float a_value;
       }

- add the log message's name, `units <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Logger/LogStructure.h#L42>`__,
  `multiplier <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Logger/LogStructure.h#L86>`__ and `format <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Logger/LogStructure.h#L6>`__ strings into the `vehicle's LogStructure array <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/Log.cpp#L454>`__ or `AP_Logger/LogStructure.h's LOG_EXTRA_STRUCTURES array <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Logger/LogStructure.h#L1360>`__
- add a new method to the vehicle code or AP_Logger library called Write_<something-or-other> which fills in the structure and then calls `AP_Logger/WriteBlock() <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Logger/AP_Logger.h#L197>`__

   .. code-block:: python

       void Copter::Log_Write_Test()
       {
           struct log_Test pkt = {
               LOG_PACKET_HEADER_INIT(LOG_TEST_MSG),
               time_us  : AP_HAL::micros64(),
               a_value  : 1234
           };
           logger.WriteBlock(&pkt, sizeof(pkt));
       }

- call this new function from the :ref:`scheduler <code-overview-scheduling-your-new-code-to-run-intermittently>` or from some other place in the code at the moment you wish to log the values
