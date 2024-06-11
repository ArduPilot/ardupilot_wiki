.. _tilt-vectored-yaw-tuning:

===============================================
Yaw Tuning for Tilt Vectored Yaw Configurations
===============================================

Tuning the yaw axis is a bit different if tilt vectored yaw is utilized. Tilt Vectored Tailsitters and Quadplanes like Tilt-Tricopter configurations use tilting of the motors to accomplish yaw will need a slightly different tuning process since the yaw response is not being driven by motor rotation speed differentials. These systems will need the FF term determined and set before tuning D and P terms for yaw. (This information is repeated in the :ref:`Tailsitter tuning <tailsitter-tuning-guide>` page)

- First, follow Steps 1 - 4 of the normal :ref:`QuadPlane Tuning Process Setup<quadplane-vtol-tuning-process>`.
- Normally, the vehicle is stable enough with the default PIDs to do a first test hover in QSTABILIZE or QHOVER. Do a short hover flight with logging enabled such that the :ref:`Harmonic Notch Filters<common-imu-notch-filtering>` can be setup. Its important to remove as much motor noise as possible in order to get a good tune using a throttle based harmonic notch filter.
- Once the notch filters are set, do another hover flight but briefly do short, but sharp yaw stick movements in both directions. Do not endanger the vehicle, but try to get several short full stick movements right and left. You should be able to hold full yaw stick each direction for a second or so, returning to neutral for a second in-between direction reversals.
- Download the :ref:`dataflash log<common-downloading-and-analyzing-data-logs-in-mission-planner>` for analysis. In order to determine the :ref:`Q_A_RAT_YAW_FF<Q_A_RAT_YAW_FF>` term, use a log analyzer like https://plot.ardupilot.org and setup the following plots on the same scale, using the "Add Expression" button:

.. code:: bash

   PIQY.FF+PIQY.P+PIQY.D   
   X * (PIQY.Act) , where X is 0.2 to start

- Next adjust the X value above until the magnitudes of both plots are about equal. This will now be value for the :ref:`Q_A_RAT_YAW_FF<Q_A_RAT_YAW_FF>` term. Set the :ref:`Q_A_RAT_YAW_I<Q_A_RAT_YAW_I>` term to equal this.

Below is an image of this where the X value is adjusted until the two curves are the same amplitude, with a value of x = 0.1. This is then set as the :ref:`Q_A_RAT_YAW_FF<Q_A_RAT_YAW_FF>` and :ref:`Q_A_RAT_YAW_I<Q_A_RAT_YAW_I>` values.

.. image:: ../../../images/FF-calculation.png
   :target: ../_images/FF-calculation.png

- Now you can hover again, and begin increasing the :ref:`Q_A_RAT_YAW_D<Q_A_RAT_YAW_D>` term, either iteratively, or using :ref:`common-transmitter-tuning`, until it oscillates and then reduce it to 1/2 to 1/3 that value.
- Then increase the :ref:`Q_A_RAT_YAW_P<Q_A_RAT_YAW_P>` term, until it oscillates and then reduce it  1/2 to 1/3 that value.