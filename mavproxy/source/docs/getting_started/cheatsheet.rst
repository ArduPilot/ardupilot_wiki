.. _mavproxy-cheetsheet:

===================
MAVProxy Cheatsheet
===================

The following table is intended as a printable quick reference to
commonly used MAVProxy commands.

=======================  ======================================================
Command                  Description                                           
=======================  ======================================================
link list                List all links
link set N               Set link N to primary
link add X               Add new link
link remove N            Remove link
wp list                  Download and show waypoints
fence list               Download and show geofence
fence enable             Enable geofence
fence disable            Disable geofence
arm throttle             Arm vehicle
disarm                   Disarm vehicle
disarm force             Force vehicle disarm (heli)
Engage auto mode         auto
wp set N                 Set current auto waypoint to N
setspeed N               Override auto speed to N m/s
param show X             Show current value of parameter X
param set X N            Set parameter X to value N
param download           Download parameter definitions from ArduPilot website
param help X             Display definition of parameter X
mode loiter              Engage loiter mode
mode rtl                 Engage rtl mode
mode manual              Engage manual mode
mode guided LAT LON ALT  Engage guided mode and move to specified location
relay set N [0|1]        Set relay N to open or closed
servo set N PWM          Override servo N to PWM value
terrain check LAT LON    Get terrain height at specified location
=======================  ======================================================



