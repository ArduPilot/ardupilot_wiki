.. _common-optitrack:

================================
Optitrack for Non-GPS Navigation
================================

[copywiki destination="copter"]

This article explains how a `OptiTrack <https://optitrack.com/>`__ motion capture system can be used as a short-range substitute for a GPS allowing position control modes like Loiter, Guided, RTL, and Auto indoors.

.. note::

   You will need a recent version of ArduPilot on your copter (Copter-4.0 or above).

.. youtube:: IocykCXJmhw
   :width: 100%

Required hardware
=================

Please refer to OptiTrack `build your own <https://optitrack.com/systems/>`__ tool for all hardware required to setup a motion capture system.

.. note::

   In order to track the both location and orientation of a drone, you need at least 4 markers.


Motion capture system setup
===========================

Please refer to OptiTrack `quick start guides <https://v20.wiki.optitrack.com/index.php?title=Quick_Start_Guide:_Getting_Started>`__ for hardware and software setup. When you set the ground plane, do not forget to mark the origin and the axis. The positive X will point to the "north" of our indoor flight environment (because we do not use compass, it do not need pointing to the real/magnetic north)

.. figure:: ../../../images/optitrack_mark_ground_plane.jpg
   :target: ../_images/optitrack_mark_ground_plane.jpg

   Mark motion capture system coordinate system using tape


Required softwares
==================

* `Motive 2 Tracker <https://optitrack.com/products/motive/tracker/>`__
* `MAVProxy <https://github.com/ArduPilot/MAVProxy>`__

.. note::

   Motive 3 is not supported

Prepare the drone
=================

.. warning::

   It is highly recommended that use small drone for indoor flight and deploy cage in your test flight enviorment.

.. tip::

   If you are looking for a small drone for indoor flight test, Skyviper V2450 GPS drone or its successor journey GPS drone is a good choice. It is very affordable and running ArduPilot out-of-box. You can easily flash it wit custom build ArduPilot. If you perfer custom build small drone, there is a very good discuss `here <https://discuss.ardupilot.org/t/microarducopter-3-props-omnibus-nano-success/32568?u=chobitsfan>`__. The RTF quadcopter frame used in another example video is available from `sdmodel <https://goods.ruten.com.tw/item/show?21806678027603>`__.

First, you need to place markers on the drone. It is very important to place markers so that they form a stereoscopic, asymmetrical shape. Please refer to OptiTrack `rigid body marker placement <https://v20.wiki.optitrack.com/index.php?title=Rigid_Body_Tracking#Rigid_Body_Marker_Placement>`__ for details.

.. figure:: ../../../images/optitrack_place_markers.jpg
   :target: ../_images/optitrack_place_markers.jpg

   Place markers on the drone

Then, put the drone in the ground plane and align it with X axis of motion capture system. The drone should heading to positive X direction.

.. figure:: ../../../images/optitrack_drone_align_x.jpg
   :target: ../_images/optitrack_drone_align_x.jpg

   Align the drone with X axis

Select all markers in Motive and create a rigid body from them. Please refer to OptiTrack `creating rigid body <https://v20.wiki.optitrack.com/index.php?title=Rigid_Body_Tracking#Creating_Rigid_Body>`__ for details

Configuration the drone
=======================

- set :ref:`AHRS_EKF_TYPE <AHRS_EKF_TYPE>` to 3 , :ref:`EK3_ENABLE <EK3_ENABLE>` to 1 and :ref:`EK2_ENABLE <EK2_ENABLE>` to 0
- set :ref:`COMPASS_USE <COMPASS_USE>`, :ref:`COMPASS_USE2 <COMPASS_USE2>`, :ref:`COMPASS_USE3 <COMPASS_USE3>` to 0. It prevents ArduPilot from using compass, because there are many sources causing electromagnetic interference in indoor environment.
- set :ref:`VISO_TYPE <VISO_TYPE>` to 1
- set :ref:`VISO_POS_M_NSE <VISO_POS_M_NSE>` to 0.3 or lower to increase the weighting of position measurements from motion capture system.
- set :ref:`VISO_YAW_M_NSE <VISO_YAW_M_NSE>` to 0.2 or lower
- set :ref:`EK3_SRC1_POSXY <EK3_SRC1_POSXY>` to 6
- set :ref:`EK3_SRC1_POSZ <EK3_SRC1_POSZ>` to 6
- set :ref:`EK3_SRC1_YAW <EK3_SRC1_YAW>` to 6
- set :ref:`EK3_SRC1_VELXY <EK3_SRC1_VELXY>` to 0
- set :ref:`EK3_SRC1_VELZ <EK3_SRC1_VELZ>` to 0


Send data to the drone
======================

Start MAVProxy and connect to your copter. Inside MAVProxy load optitrack module with:

.. code:: bash

    module load optitrack
    
You need to set tracking rigid body id to match your setting in Motive:

.. code:: bash

    optitrack set obj_id RIGID_BODY_STREAMING_ID

If you set Motive data streaming local interface to other than loopback , it is required to configuare optitrack module with:

.. code:: bash

    optitrack set server SERVER_IP_ADDRESS
    optitrack set client CLIENT_IP_ADDRESS


.. note::

   The coordinate system of both Motive and ArduPilot are right-handed. While Z axis of ArduPilot is pointing down, Y axis of Motive is pointing up.  

After all parameters is set, start sending pose to ardupilot:

.. code:: bash

   optitrack start

Ground testing
==============

- Connect the drone to a ground station
- Start Motive and make sure `data streaming <https://v20.wiki.optitrack.com/index.php?title=Data_Streaming>`__ is turned on.
- If you see following message in ground station console (initial pos may vary), then the drone should be ready for flight test

*EKF2 IMU0 is using external nav data
EKF2 IMU0 initial pos NED = 0.1,-0.2,0.0 (m)
EKF2 IMU0 ext nav yaw alignment complete*

Flight testing
==============

Take off in AltHold mode and maintain a stable hover. Switch to Loiter but be ready to switch back to AltHold or Stabilize if the vehicle's position or altitude becomes unstable.

.. note::

   In order to take off in guided or auto mode, you need to use `GPS_GLOBAL_ORIGIN <https://mavlink.io/en/messages/common.html#SET_GPS_GLOBAL_ORIGIN>`__ to set the GPS location of motion capture system origin. It is not need to be accurate, any valid lat/lng is ok.

.. youtube:: JKzuaVQZclI
   :width: 100%
