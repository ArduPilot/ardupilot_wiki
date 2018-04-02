.. _mavlink-commands:

================
MAVLink Commands
================

ArduPilot uses a dialect of the MAVLink protocol, which includes elements of the baseline Common message set,
plus unique messages for ArduPilot, uAvionix (ADS-B), and the NASA Icarous project.  Whilst attempts are made to keep
the ArduPilot dialect aligned with basic MAVLink, as the MAVLink repository is not controlled by the ArduPilot Project,
the branch located in the ArduPilot GitHub repository should be used to ensure compatibility with the ArduPilot ecosystem.

Important links for working with commands are listed below:

-  `MAVLink ArduPilot Message Set in HTML <http://mavlink.org/messages/ardupilotmega>`__ and
   `XML <https://github.com/ArduPilot/mavlink/blob/master/message_definitions/v1.0/ardupilotmega.xml>`__
   (Protocol Definition).
-  `MavLink Tutorial for Absolute Dummies (Part–1) <http://api.ning.com/files/i*tFWQTF2R*7Mmw7hksAU-u9IABKNDO9apguOiSOCfvi2znk1tXhur0Bt00jTOldFvob-Sczg3*lDcgChG26QaHZpzEcISM5/MAVLINK_FOR_DUMMIESPart1_v.1.1.pdf>`__
   (from `Shyam Balasubramanian <http://api.ning.com/files/i*tFWQTF2R*7Mmw7hksAU-u9IABKNDO9apguOiSOCfvi2znk1tXhur0Bt00jTOldFvob-Sczg3*lDcgChG26QaHZpzEcISM5/MAVLINK_FOR_DUMMIESPart1_v.1.1.pdf>`__).
-  `MAVLink communications between Pixhawk and Arduino <https://discuss.ardupilot.org/t/mavlink-and-arduino-step-by-step/25566>`__ (example from `Juan Pedro López <https://discuss.ardupilot.org/t/mavlink-and-arduino-step-by-step/25566>`__).
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
    Adding a new MAVLink Message <code-overview-adding-a-new-mavlink-message>
    Copter Commands (Guided Mode) <copter-commands-in-guided-mode>
    Plane Commands (Guided Mode) <plane-commands-in-guided-mode>
    MAVLink Routing <mavlink-routing-in-ardupilot>
