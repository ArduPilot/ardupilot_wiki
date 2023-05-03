.. _common -gryoflow:

===============================================
Video Stabilization Using IMU data and Gyroflow
===============================================

ArduPilot provides the means to produce the log data needed for use with `Gyroflow Video Stabilization software <https://gyroflow.xyz/>`_. This software allows the post processing of video taken during a flight with the IMU data logged during the flight to obtain stabilized video.

.. youtube:: R4qHfM-Vwy0

ArduPilot Setup
===============

The following parameters are required:

The Video Stabilization bit in :ref:`LOG_BITMASK<LOG_BITMASK>` (bit 20 for Copter/Rover, 21 for Plane) must be set (1048576 (Copter/Rover) or 2097152 (Plane) added to present value if not already set) to enable the proper logging

:ref:`SCHED_LOOP_RATE<SCHED_LOOP_RATE>` should be set to 300Hz or higher. Planes and Rovers have 50Hz as a default and this should be raised. QuadPlanes and Copters have it already set by default.

Optional: for ease of use, set :ref:`LOG_FILE_DSRMROT<LOG_FILE_DSRMROT>` = 1 to stop logging when disarmed. This allows multiple logs to be generated for multiple flights during a session and makes syncing the logs to the videos easier. Otherwise you would have one big log for multiple flights and videos while powered up.

Be sure to start the video and arm to start logging at very close to the same time, otherwise you will need to determine the time offset between them and use it for the synchronization dialog's "Rough gryo offset" value in Gyroflow.

Gyroflow Use
============

Many tutorials exist on YouTube on how to use and adjust Gyroflow, but here is a simplified quickstart for use with ArduPilot, assuming you have installed Gyroflow:

1. Download the flight log corresponding the video you wish to stabilize. In Mission Planner, under the DataFlash tab, use the .bin to .log button to convert the log .bin file to a .log file.
2. Start Gyroflow program.
3. Open video file in Gryoflow
4. Select the lens profile for your camera. If it is not listed (unlikely) the there are tutorials  on YouTube as to how to calibrate for your camera.
5. Open the .log file you created in the first step above.
6. Be sure to enter the camera's rotation with respect to the autopilot. Its very common to tilt the camera toward the earth for a better view on a vehicle and this rotation MUST be precisely entered in the "Rotation" input boxes in the log data section....typically -10 to -20 degrees.
7. use the AutoSync button to synchronize the data and video...the number of points used depends on the video and data, but is easily experimented with. Starting with 4, for example.

You should be able to play a preview of stabilized video at this point and experiment with the other stabilization settings...

.. tip:: if you are getting "jello" (CMOS cameras) that you cant remove in the Gyroflow processing, it might be better to hard mount your camera, and maybe get more even more jello in the raw video. The reason is that when soft mounting the camera, it experiences different G-forces than what the autopilot is recording and, therefore Gyroflow, cant compensate. Of course, in some cases the end product can be even worse, so mounting experimentation for your particular setup may be required.
