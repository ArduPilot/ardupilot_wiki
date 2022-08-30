.. _code-overview-scheduling-your-new-code-to-run-intermittently:

=====================================
Scheduling Code to Run Intermittently
=====================================

This page describes how you can schedule some new piece of code you have
written to run intermittently.

Running your code with the scheduler
====================================

The most flexible way to run your code at a given interval is to use the
scheduler.  This can be done by adding your new function to the
`scheduler_tasks <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/Copter.cpp#L91>`__
array in
`Copter.cpp <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/Copter.cpp>`__. 
Note that there are actually two task lists, `the upper list <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/Copter.cpp#L788>`__
is for high speed CPUs (i.e. Pixhawk) and the `lower list <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/Copter.cpp#L856>`__
is for slow CPUs (i.e. APM2).

Adding a task is fairly simple, just create a new row in the list
(higher in the list means high priority).  The first column holds the
function name, the 2nd is a number of 2.5ms units (or 10ms units in case
of APM2).  So if you wanted the function executed at 400Hz this column
would contain "1", if you wanted 50Hz it would contain "8".  The final
column holds the number of microseconds (i.e. millions of a second) the
function is expected to take.  This helps the scheduler guess whether or
not there is enough time to run your function before the main loop
starts the next iteration.

::

    /*
      scheduler table - all regular tasks apart from the fast_loop()
      should be listed here, along with how often they should be called
      (in 10ms units) and the maximum time they are expected to take (in
      microseconds)
     */
    static const AP_Scheduler::Task scheduler_tasks[] PROGMEM = {
        { update_GPS,            2,     900 },
        { update_nav_mode,       1,     400 },
        { medium_loop,           2,     700 },
        { update_altitude,      10,    1000 },
        { fifty_hz_loop,         2,     950 },
        { run_nav_updates,      10,     800 },
        { slow_loop,            10,     500 },
        { gcs_check_input,       2,     700 },
        { gcs_send_heartbeat,  100,     700 },
        { gcs_data_stream_send,  2,    1500 },
        { gcs_send_deferred,     2,    1200 },
        { compass_accumulate,    2,     700 },
        { barometer_accumulate,  2,     900 },
        { super_slow_loop,     100,    1100 },
          { my_new_function,      10,     200 },
        { perf_update,        1000,     500 }
    };

Running your code as part of one of the loops
=============================================

Instead of creating a new entry in the scheduler task list, you can add
your function to one of the existing timed loops.  There is no real
advantage to this approach over the above approach except in the case of
the fast-loop.  Adding your function to the fast-loop will mean that it
runs at the highest possible priority (i.e. it is nearly 100% guaranteed
to run at 400Hz).

-  `fast_loop <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/Copter.cpp#L229>`__
   : runs at 100Hz on APM2, 400Hz on Pixhawk
-  `fifty_hz_loop <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/Copter.cpp#L370>`__
   : runs at 50Hz
-  `ten_hz_logging_loop <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/Copter.cpp#L333>`__:
   runs at 10Hz
-  `three_hz_loop <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/Copter.cpp#L398>`__:
   runs at 3.3Hz
-  `one_hz_loop <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/Copter.cpp#L417>`__
   : runs at 1Hz

So for example if you want your new code to run at 10Hz you could add it
to one of the case statements in the
`ten_hz_logging_loop() <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/Copter.cpp#L333>`__
function found in
`Copter.cpp <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/Copter.cpp>`__.

::

    // ten_hz_logging_loop
    // should be run at 10hz
    static void ten_hz_logging_loop()
    {
        if (g.log_bitmask & MASK_LOG_ATTITUDE_MED) {
            Log_Write_Attitude();
        }
        if (g.log_bitmask & MASK_LOG_RCIN) {
            DataFlash.Log_Write_RCIN();
        }
        if (g.log_bitmask & MASK_LOG_RCOUT) {
            DataFlash.Log_Write_RCOUT();
        }
        if ((g.log_bitmask & MASK_LOG_NTUN) && mode_requires_GPS(control_mode)) {
            Log_Write_Nav_Tuning();
        }

            // your new function call here
            my_new_function();
    }
