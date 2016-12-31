.. _takeoff-mode:

============
TAKEOFF Mode
============

Auto takeoff is set by the mission control scripting only. The takeoff
mission specifies a takeoff pitch and a target altitude. During takeoff
Plane will use the maximum throttle set by the ``THR_MAX`` parameter.
The takeoff mission item is considered complete when the plane has
reached the target altitude specified in the mission.

Before takeoff it is important that the plane be pointing into the wind,
and be aligned with the runway (if a wheeled takeoff is used). The plane
will try to hold its heading during takeoff, with the initial heading
set by the direction the plane is facing when the takeoff starts. It is
highly recommended that a compass be enabled and properly configured for
auto takeoff, as takeoff with a GPS heading can lead to poor heading
control.

If you are using a wheeled aircraft then you should look at the
``WHEELSTEER_*`` PID settings for controlling ground steering. If you
are hand launching or using a catapult you should look at the
``TKOFF_THR_MINACC`` and ``TKOFF_THR_MINSPD`` parameters.
