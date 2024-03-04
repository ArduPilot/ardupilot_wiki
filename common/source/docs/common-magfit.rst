.. _common-magfit:

==================================================
Refining Calibration Parameters using a Flight Log
==================================================

The compass offsets, scales, diagonals, and even motor compensation can be determined from a flight dataflash log of the vehicle using the web browser-based WebTools MagFit utility. This utility is available `here <https://firmware.ardupilot.org/Tools/WebTools/MAGFit/>`__.

Using a flight log, the utility uses the GPS location, the vehicle attitude, and the World Magnetic Model database
to compare the logged magnetic field components from the vehicle compass and compare the x/y/z values to that expected which are graphed. It also will run a calibration routine to correct the Ardupilot compass parameter values to improve the compass performance. Offsets (OFS), diagonals(DIA), ellipticals(ODI), motor current compensation, and scale parameters can be corrected.

It can detect incorrect orientation, also.

Using WebTools MagFit
=====================
- load flight log
- select relevant portion of log to analyze (eliminate on the ground portions)
- select which parameters to optimize( unless there is significant motor current interference or large nearby ferro-magnetic objects like batteries to the compass, offset optimization  will usually be sufficient)
- check if the reduction in heading deviation is acceptable with the new param values(usually <10deg is okay)
- save the new values to a file and load into the autopilot.

Here is a video explaining the utility's options and use:

.. youtube:: mfbBjxte55c


Using MAVExplorer's integrated Magfit utility
=============================================

Now MAVExplorer has a Tools menu which includes this utility and allows it to be applied to any portion of the log by simply opening any Graph and zooming (magnifying glass icon) to the area to be analyzed. If no graph is opened, and area selected, then the entire log will be used.

Below is a video of the process.

.. note:: selecting CMOT No Change box, will calculate the CMOT correction values, but not apply them to the corrected graph.

.. youtube:: mfbBjxte55c


Hints and Tips
--------------

- Be sure you have several complete turns during the flight and at least a few minutes of log.
- Be sure you put conditions on the analysis so you do not include data while on the ground or being carried.
- Be sure you have already calibrated the compasses prior to the flight. You can use the tool on very poorly calibrated compasses, but that could be a flight hazard (unless its a plane or vehicle not dependent on compass).
- If you have large COMPASS_MOT values indicating strong motor/battery system interference, be sure that none of those components can shift in flight, resulting in invalidating the compass motor interference corrections you entered.
- The EKF's sole purpose is to filter out sensor inaccuracies and system disturbances. If the ATT yaw change is very large( > 20 degrees), this means that something else is possibly very wrong in the other sensors, or vehicle setup, and needs to be investigated before using the new compass calibration parameter values.



