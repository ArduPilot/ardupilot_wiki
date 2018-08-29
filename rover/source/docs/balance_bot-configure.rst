.. _balance_bot-configure:

=======================
Configuration and Setup
=======================

We have assumed that the radio, accelerometer and compass calibrations have been completed prior to this step. If not, follow the instructions :ref:`here<rover-code-configuration>` to do that before proceeding.


Motor and ESC configuration
---------------------------
Balance Bots use **skid steering** by default. The motor drive/ESC configuration have to done based on the components you have used. Follow the instructions :ref:`here<rover-motor-and-servo-configuration-skid>` to configure it.

Parameter Configuration
-----------------------
The following parameters need to be set correctly for your balance bot to function properly. The parameters can be accessed from Mission planner -> Config/Tuning-> Full Parameter List. After making the changes click on 'Write Params' to save your changes.

* SERVO1_FUNCTION =  73 (Throttle left)
* SERVO3_FUNCTION =  74 (Throttle right)
* FRAME_CLASS     =   3 (Balance Bot)
* SCHED_LOOP_RATE = 200 (Scheduler loop frequency)
* MOT_SLEWRATE    =   0 (Do not account for motor slew)
* FS_CRASH_CHECK  =   1 (Enable Crash Checks)

Minimum Throttle
----------------
Many motors and ESCs have a dead zone. That is the zone between the zero throttle value and the throttle value where the motor actually starts rotating. A large dead zone can bring down balancing performance. 

.. tip:: Remove wheels before proceeding

To fix the dead zone, open the motor test window in Mission Planner, as mentioned :ref:`here<rover-motor-and-servo-configuration-testing>`.  Find the minimum throttle value at which both motors turn on and set the parameter MOT_THR_MIN to that value. Now your motors should start at 1% throttle.

















