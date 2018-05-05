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

Use `DataFlash_Class:instance()->Log_Write(...) <https://github.com/ArduPilot/ardupilot/blob/master/libraries/DataFlash/DataFlash.h#L162>`__:

   .. code-block:: python

       void Log_Write(const char *name, const char *labels, const char *fmt, ...);
       void Log_Write(const char *name, const char *labels, const char *units, const char *mults, const char *fmt, ...);

An example of the top function being used can be found in `Compass_learn.cpp <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Compass/Compass_learn.cpp#L101>`__:

   .. code-block:: python

       DataFlash_Class::instance()->Log_Write("COFS", "TimeUS,OfsX,OfsY,OfsZ,Var,Yaw,WVar,N", "QffffffI",
                                              AP_HAL::micros64(),
                                              (double)best_offsets.x,
                                              (double)best_offsets.y,
                                              (double)best_offsets.z,
                                              (double)best_error,
                                              (double)best_yaw_deg,
                                              (double)worst_error,
                                              num_samples);

- the 1st argument is the message name.  This should be 4 characters or less and be unique
- the 2nd argument is a comma separated list of field names
- the 3rd argument is a format string with each letter holding the format for the corresponding field.  The meaning of each letter can be found `here in DataFlash/LogStructure.h <https://github.com/ArduPilot/ardupilot/blob/master/libraries/DataFlash/LogStructure.h#L1144>`__
- the remaining arguments are the actual values that will be logged.
  You may notice in the example above, some fields have a format of float ("f") but are cast to ``(double)`` this is correct and necessary to avoid a compiler warning.

The 2nd Log_Write function is the same as the first except that it accepts two additional string arguments,
`"units" <https://github.com/ArduPilot/ardupilot/blob/master/libraries/DataFlash/LogStructure.h#L74>`__ and
`"mults" <https://github.com/ArduPilot/ardupilot/blob/master/libraries/DataFlash/LogStructure.h#L115>`__.
Similar to the "format" argument, each character in these arguments specifies the
`units <https://github.com/ArduPilot/ardupilot/blob/master/libraries/DataFlash/LogStructure.h#L74>`__ (i.e. "d" for degrees) or
`multiplier <https://github.com/ArduPilot/ardupilot/blob/master/libraries/DataFlash/LogStructure.h#L115>`__ (i.e. "2" for \*100, "B" for \*0.01) for the following fields.
These help the graphing tools scale the output correctly when displaying to the user.

For example, below is a "TEST" log message which outputs the current system time and altitude.

   .. code-block:: python

       DataFlash_Class::instance()->Log_Write("TEST", "TimeUS,Alt",
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
- add a new entry to the `vehicle's #define list or enum in defines.h <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/defines.h#L313>`__ or `DataFlash/LogStructure.h's LogMessages enum <https://github.com/ArduPilot/ardupilot/blob/master/libraries/DataFlash/LogStructure.h#L1418>`__.
- define a structure to hold the values to be logged in either the vehicle's `Log.cpp <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/Log.cpp>`__ file
  or in `DataFlash/LogStructure.h <https://github.com/ArduPilot/ardupilot/blob/master/libraries/DataFlash/LogStructure.h>`__.  All log file messages should have time_us as their first field.

   .. code-block:: python

       struct PACKED log_Test {
           LOG_PACKET_HEADER;
           uint64_t time_us;
           float a_value;
       }

- add the log message's name, `units <https://github.com/ArduPilot/ardupilot/blob/master/libraries/DataFlash/LogStructure.h#L74>`__,
  `multiplier <https://github.com/ArduPilot/ardupilot/blob/master/libraries/DataFlash/LogStructure.h#L115>`__ and `format <https://github.com/ArduPilot/ardupilot/blob/master/libraries/DataFlash/LogStructure.h#L1144>`__ strings into the `vehicle's LogStructure array <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/Log.cpp#L578>`__ or `DataFlash/LogStructure.h's LOG_EXTRA_STRUCTURES array <https://github.com/ArduPilot/ardupilot/blob/master/libraries/DataFlash/LogStructure.h#L1243>`__
- add a new method to the vehicle code or DataFlash library called Log_Write_<something-or-other> which fills in the structure and then calls `DataFlash/WriteBlock() <https://github.com/ArduPilot/ardupilot/blob/master/libraries/DataFlash/DataFlash.h#L78>`__

   .. code-block:: python

       void Copter::Log_Write_Test()
       {
           struct log_Test pkt = {
               LOG_PACKET_HEADER_INIT(LOG_TEST_MSG),
               time_us  : AP_HAL::micros64(),
               a_value  : 1234
           };
           DataFlash.WriteBlock(&pkt, sizeof(pkt));
       }

- call this new function from the :ref:`scheduler <code-overview-scheduling-your-new-code-to-run-intermittently>` or from some other place in the code at the moment you wish to log the values
