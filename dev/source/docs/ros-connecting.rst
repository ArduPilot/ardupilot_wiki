.. _ros-connecting:

===================
Connecting with ROS
===================

Before attempting to connect with ROS, ensure that the physical serial cable connection and ArduPilot side parameters have been set as described on the appropriate companion computer page (:ref:`RPi3 <raspberry-pi-via-mavlink>`, :ref:`NVidia TX1 <companion-computer-nvidia-tx1>`, :ref:`NVidia TX2 <companion-computer-nvidia-tx2>` or :ref:`Intel Edison <intel-edison>`).

The connection between ArduPilot running on the flight controller and ROS/MAVROS can be established with the following commands on the companion computer:

- open up a terminal and type, "roscore"
- open another terminal and start mavros as described `here on the ros/mavros wiki <http://wiki.ros.org/mavros#Usage>`__.  For example, the following instruction is appropriate for a TX2/APSync install with port 14855 opened (see "How flight controller data is routed to various programs" at the bottom of :ref:`this page <apsync-intro>`):

::

    roslaunch mavros apm.launch fcu_url:=udp://:14855@

Test the flight controller is responding to `mavros commands <http://wiki.ros.org/mavros#Utility_commands>`__ like `mavsys <http://wiki.ros.org/mavros#mavsys>`__ (which can set the flight mode) and `mavsafety <http://wiki.ros.org/mavros#mavsafety>`__ (which can arm/disarm the vehicle):

::

    rosrun mavros mavsys mode -c 0 (sets the vehicle to mode "0")
    rosrun mavros mavsafety arm (to arm the vehicle)

.. note::

   We are keen to improve ArduPilot's support of ROS so if you find issues (such as commands that do not seem to be supported), please report them in the `ArduPilot issues list <https://github.com/ArduPilot/ardupilot/issues>`__ with a title that includes "ROS" and we will attempt to resolve them as quickly as possible.
