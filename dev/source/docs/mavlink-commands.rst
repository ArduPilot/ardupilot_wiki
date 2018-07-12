.. _mavlink-commands:

================
MAVLink Commands
================

ArduPilot has adopted a subset of the MAVLink protocol command set.
Important links for working with commands are listed below:

-  MAVLINK Common Message Set in `HTML <https://mavlink.io/en/messages/common.html>`__ and
   `XML <https://github.com/mavlink/mavlink/blob/master/message_definitions/v1.0/common.xml>`__
   (Protocol Definition).
-  `MavLink Tutorial for Absolute Dummies (Part–1) <http://api.ning.com/files/i*tFWQTF2R*7Mmw7hksAU-u9IABKNDO9apguOiSOCfvi2znk1tXhur0Bt00jTOldFvob-Sczg3*lDcgChG26QaHZpzEcISM5/MAVLINK_FOR_DUMMIESPart1_v.1.1.pdf>`__
   (from `Shyam Balasubramanian <http://api.ning.com/files/i*tFWQTF2R*7Mmw7hksAU-u9IABKNDO9apguOiSOCfvi2znk1tXhur0Bt00jTOldFvob-Sczg3*lDcgChG26QaHZpzEcISM5/MAVLINK_FOR_DUMMIESPart1_v.1.1.pdf>`__).
-  :ref:`MAVLink Mission Command Messages (MAV_CMD) <planner:common-mavlink-mission-command-messages-mav_cmd>`
   (ArduPilot supported subset).
-  User-modifiable MAVLink parameters: 
   :ref:`Copter Parameters <copter:parameters>`,
   :ref:`Plane Parameters <plane:parameters>`,
   :ref:`Rover Parameters <rover:parameters>`.

Other relevant topics on this wiki include:

.. toctree::
    :maxdepth: 1

    Command Package Format <ardupilot-mavlink-command-package-format>
    Copter Commands (Guided Mode) <copter-commands-in-guided-mode>
    Plane Commands (Guided Mode) <plane-commands-in-guided-mode>
    MAVLink Routing <mavlink-routing-in-ardupilot>
