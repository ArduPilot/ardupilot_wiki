.. _mavlink-commands:

=================
MAVLink Interface
=================

ArduPilot supports the MAVLink protocol for communication with Ground Stations and :ref:`Companion Computers <companion-computers>`.  These pages explain the details of this interface

.. toctree::
    :maxdepth: 1

    MAVLink Basics <mavlink-basics>
    Request Data From The AutoPilot <mavlink-requesting-data>
    Get and Set Parameters <mavlink-get-set-params>
    Copter Commands (Guided Mode) <copter-commands-in-guided-mode>
    Plane Commands (Guided Mode) <plane-commands-in-guided-mode>
    Rover Commands (Guided Mode) <mavlink-rover-commands>
    Get and Set Home and/or EKF origin <mavlink-get-set-home-and-origin>
    Arm and Disarm <mavlink-arming-and-disarming>
    Get and Set FlightMode <mavlink-get-set-flightmode>
    Camera Commands <mavlink-camera>
    Gimbal / Camera Mount <mavlink-gimbal-mount>
    Mission Upload/Download <mavlink-mission-upload-download>
    Move a Servo <mavlink-move-servo>
    Winch Commands <mavlink-winch>
    MAVLink Routing <mavlink-routing-in-ardupilot>
    Other Commands <mavlink-other-commands>

External References
-------------------

-  `MAVLink Common Message Set (HTML) <https://mavlink.io/en/messages/common.html>`__ and `XML <https://github.com/mavlink/mavlink/blob/master/message_definitions/v1.0/common.xml>`__ (Protocol Definition)
-  `MAVLink ArduPilot Message Set (HTML) <https://mavlink.io/en/messages/ardupilotmega.html>`__ and `XML <https://github.com/ArduPilot/mavlink/blob/master/message_definitions/v1.0/ardupilotmega.xml>`__
-  `MAVLink Tutorial for Absolute Dummies (Part–1) <https://discuss.ardupilot.org/uploads/short-url/vS0JJd3BQfN9uF4DkY7bAeb6Svd.pdf>`__ by Shyam Balasubramanian

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
- `Sub Parameters <http://www.ardusub.com/developers/full-parameter-list.html>`_
