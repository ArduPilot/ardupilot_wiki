.. _can-bus-mavlink:

===================
Mavlink over UAVCAN
===================

Mavlink over CAN support is found on `Tom's CAN/mavlink-over-uavcan_upstream2 branch <https://github.com/magicrub/ardupilot/tree/CAN/mavlink-over-uavcan_upstream2>`.

Start by cloning the repository and switching to the branch.
::
    git clone git@github.com:magicrub/ardupilot.git
    git checkout -b CAN/mavlink-over-uavcan_upstream2

Configure your board using waf.
::
    ./waf configure --board px4-v4

Upload ArduPilot to your board
::
    ./waf --targets bin/arducopter --upload

Open mavproxy.py
::
    mavproxy.py

Enable the CAN driver
::
    set CAN_P1_DRIVER = 1
    reboot

Set CAN_D1_UC_VSER_P to 1
::
    set CAN_D1_UC_VSER_P = 1

Set SYSID_THISMAV to a unique value (for example 72)
::
    set SYSID_THISMAV = 72

Your board is now ready to accept Mavlink over UAVCAN!