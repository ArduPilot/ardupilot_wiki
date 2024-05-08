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

Helpers are provided to create entries in the scheduler table; see the documentation referenced above.

::

    const AP_Scheduler::Task Copter::scheduler_tasks[] = {
        // update INS immediately to get current gyro data populated
        FAST_TASK_CLASS(AP_InertialSensor, &copter.ins, update),
        // run low level rate controllers that only require IMU data
        FAST_TASK(run_rate_controller),
    #if AC_CUSTOMCONTROL_MULTI_ENABLED == ENABLED
        FAST_TASK(run_custom_controller),
    #endif
    .
    .
        SCHED_TASK_CLASS(GCS,                  (GCS*)&copter._gcs,          update_receive, 400, 180, 102),
        SCHED_TASK_CLASS(GCS,                  (GCS*)&copter._gcs,          update_send,    400, 550, 105),
    #if HAL_MOUNT_ENABLED
        SCHED_TASK_CLASS(AP_Mount,             &copter.camera_mount,        update,          50,  75, 108),
    #endif
    #if AP_CAMERA_ENABLED
        SCHED_TASK_CLASS(AP_Camera,            &copter.camera,              update,          50,  75, 111),
    .
    .

Running your code as part of one of the loops
=============================================

Instead of creating a new entry in the scheduler task list, you can
add your function to one of the existing timed loops.  There is no
real advantage to this approach over the above approach.

-  `fifty_hz_loop <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/Copter.cpp#L370>`__
   : runs at 50Hz
-  `ten_hz_logging_loop <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/Copter.cpp#L333>`__:
   runs at 10Hz
-  `three_hz_loop <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/Copter.cpp#L398>`__:
   runs at 3Hz
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
