.. _ros-connecting:

===================
Connecting with ROS
===================

The connection between ArduPilot running on the flight controller and ROS/MAVROS can be tested with the following commands on the companion computer:

- open up a terminal and type, "roscore"
- open another terminal and start mavros as described `here on the ros/mavros wiki <http://wiki.ros.org/mavros#Usage>`__.  For example, the following instruction is appropriate for a TX2/APSync install with port 14855 opened (see "How flight controller data is routed to various programs" at the bottom of :ref:`this page <apsync-intro>`):

::

    roslaunch mavros apm.launch fcu_url:=udp://@127.0.0.1:14855

.. note::

   We are keen to improve ArduPilot's support of ROS so if you find issues (such as commands that do not seem to be supported), please report them in the `ArduPilot issues list <https://github.com/ArduPilot/ardupilot/issues>`__ with a title that includes "ROS" and we will attempt to resolve them as quickly as possible.
