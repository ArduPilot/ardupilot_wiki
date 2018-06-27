.. _balance_bot-configure:

=======================
Configuration and Setup
=======================

We have assumed that the radio, accelerometer and compass calibrations have been completed prior to this step. If not, follow the instructions :ref:`here<rover-code-configuration>` to do that before proceeding.

Leveling the accelerometer
--------------------------
Changes in cabling or crashes can displace the flight controller slightly on your balancebot. This can cause it to report an incorrect pitch which can completely upset the balancing mechanism. Make sure you repeat this step, every time you make any major change to the balance bot hardware.

Remove both the wheels and place your balance bot level on the ground. Open the accelerometer calibration window in mission planner and press the calibrate level button. The leveling should complete in a few seconds.

Motor and ESC configuration
---------------------------
Balance Bots use **skid steering** by default. The motor drive/ESC configuration have to done based on the components you have used. Follow the instructions :ref:`here<rover-motor-and-servo-configuration-skid>` to configure it.

Minimum Throttle
----------------
Many motors and ESCs have a dead zone. That is the zone between the zero throttle value and the throttle value where the motor actually starts rotating. A large dead zone can bring down balancing performance. Another issue could be that both motors respond differently to the same throttle, causing the vehicle to yaw continuosly.

.. tip:: Remove wheels before proceeding

To fix the dead zone, open the motor test window in Mission Planner, as mentioned :ref:`here<rover-motor-and-servo-configuration-testing>`.  Set the parameter MOT_THR_MIN to that value. Follow the intructions in the next step for changing parameters. Now your motors should start at 1% throttle.

Parameter Configuration
-----------------------
The following parameters need to be set correctly for your balance bot to function properly. The parameters can be accessed from Mission planner -> Config/Tuning-> Full Parameter List. After making the changes click on 'Write Params' to save your changes.

* SERVO1_FUNCTION =  73 (Throttle left)
* SERVO3_FUNCTION =  74 (Throttle right)
* FRAME_CLASS     =   3 (Balance Bot)
* SCHED_LOOP_RATE = 200 (Scheduler loop frequency)
* MOT_SLEWRATE    =   0 (Do not account for motor slew)
* FS_CRASH_CHECK  =   1 (Enable Crash Checks)

We have assumed you have already set MOT_THR_MIN as mentioned in the previous step.

PID configuration
-----------------
These parameters are the most crucial for the balance bot. If you are using SITL, the default parameters should do fine. If you are trying on a real balance bot, the default params are certain to crash it! So change them as mentioned below.

.. hint:: The right PID values for your vehicle will have to be determined through trial and error. We recommend you do a fresh PID tuning and use these values only as reference. 

These are the P, I and D we used and the increment factor for tuning:

* ATC_BAL_P = 1.6  (increment by 0.01)
* ATC_BAL_I = 1.4  (increment by 0.01)
* ATC_BAL_D = 0.04 (increment by 0.001)















