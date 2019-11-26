.. _mavlink-commands:

=================
MAVLink Interface
=================

ArduPilot supports the MAVLink protocol for communication with Ground Stations and :ref:`Companion Computers <companion-computers>`.  These pages explain the details of this interface

.. toctree::
    :maxdepth: 1

    MAVLink Basics <mavlink-basics>
    Requesting Data From The AutoPilot <mavlink-requesting-data>
    Getting and Setting Parameters <mavlink-get-set-params>
    Copter Commands (Guided Mode) <copter-commands-in-guided-mode>
    Plane Commands (Guided Mode) <plane-commands-in-guided-mode>
    MAVLink Routing <mavlink-routing-in-ardupilot>

External References
-------------------

-  `MAVLink Common Message Set (HTML) <https://mavlink.io/en/messages/common.html>`__ and `XML <https://github.com/mavlink/mavlink/blob/master/message_definitions/v1.0/common.xml>`__ (Protocol Definition)
-  `MAVLink ArduPilot Message Set (HTML) <https://mavlink.io/en/messages/ardupilotmega.html>`__ and `XML <https://github.com/ArduPilot/mavlink/blob/master/message_definitions/v1.0/ardupilotmega.xml>`__
-  `MAVLink Tutorial for Absolute Dummies (Part–1) <http://api.ning.com/files/i*tFWQTF2R*7Mmw7hksAU-u9IABKNDO9apguOiSOCfvi2znk1tXhur0Bt00jTOldFvob-Sczg3*lDcgChG26QaHZpzEcISM5/MAVLINK_FOR_DUMMIESPart1_v.1.1.pdf>`__ by Shyam Balasubramanian

Autonomous Mission Commands
---------------------------

Mission commands are stored on the flight controller in eeprom and executed one-by-one when the vehicle is switched into Auto mode.  Although not directly related to the real-time MAVLink interface linked above, the available commands are a subset of the `MAVLink MAV_CMD list <https://mavlink.io/en/messages/common.html#mavlink-commands-mavcmd>`__

.. toctree::
    :maxdepth: 1

    Mission Command List <common-mavlink-mission-command-messages-mav_cmd>
    Mission Command Package Format <ardupilot-mavlink-command-package-format>

See also: :ref:`Copter Mission Command List <copter:mission-command-list>`

Vehicle Parameter References
----------------------------

- :ref:`Copter Parameters <copter:parameters>`
- :ref:`Plane Parameters <plane:parameters>`
- :ref:`Rover Parameters <rover:parameters>`
