.. _takeoff-mode:

============
TAKEOFF Mode
============

Automatic takeoff can be accomplished either as a mission control command or by directly changing into the TAKEOFF mode. See also the :ref:`automatic-takeoff` topic for general setup information for automatic takeoffs.

TAKEOFF Mission Command
=======================

The takeoff mission command specifies a takeoff pitch and a target altitude. During auto-takeoff
Plane will use the maximum throttle set by the :ref:`TKOFF_THR_MAX<TKOFF_THR_MAX>` parameter.
The takeoff mission item is considered complete when the plane has
reached the target altitude specified in the mission. The mission will then execute its normal end of mission behavior if it runs out of commands in the mission list. For Plane, this is RTL.

TAKEOFF Flight Mode
===================

In Arduplane 4.0 and later, Automatic Takeoff is also a mode itself. When entered, the plane will use maximum throttle as set by the :ref:`TKOFF_THR_MAX<TKOFF_THR_MAX>` parameter, climbing with :ref:`TKOFF_LVL_PITCH<TKOFF_LVL_PITCH>` maximum to the :ref:`TKOFF_ALT<TKOFF_ALT>` altitude and proceeding at the initial launch heading until :ref:`TKOFF_DIST<TKOFF_DIST>` from the point where the mode is entered. It will then loiter, as in LOITER mode, at :ref:`TKOFF_ALT<TKOFF_ALT>` altitude until the mode is changed.

Once :ref:`TKOFF_LVL_ALT<TKOFF_LVL_ALT>` is reached, or the loiter point distance is reached, maximum throttle and takeoff roll limits (:ref:`LEVEL_ROLL_LIMIT<LEVEL_ROLL_LIMIT>` ) are stopped and normal navigation begins to the loiter point and altitude.

If the mode is entered while already flying, it will immediately begin loitering as in LOITER mode.

TAKEOFF mode can also be entered via a switch using an RCx_OPTION = 77, as well as via normal selection by the flight mode channel.

TAKEOFF Heading
===============

Before takeoff it is important that the plane be pointing into the wind,
and be aligned with the runway (if a wheeled takeoff is used). The plane
will try to hold its heading during takeoff, with the initial heading
set by the direction the plane is facing when the takeoff starts. It is
highly recommended that a compass be enabled and properly configured for
auto takeoff, as takeoff with a GPS heading can lead to poor initial heading
control such that heading can different from the initial heading by tens of degrees during the climb. While this may not be an issue for hand launchs, runway takeoffs require a compass for adequate heading control during the takeoff rollout.


If you are using a wheeled aircraft then you should look at the
``WHEELSTEER_*`` PID settings for controlling ground steering. If you
are hand launching or using a catapult you should look at the
``TKOFF_THR_MINACC`` and ``TKOFF_THR_MINSPD`` parameters.
