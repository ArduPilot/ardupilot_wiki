.. _common-MAVLink-high-latency:
[copywiki destination="plane,copter,rover,blimp"]
=========================
MAVLink High Latency Mode
=========================

ArduPilot supports the `MAVLink High Latency <https://mavlink.io/en/services/high_latency.html>`__ protocol. This designed for very low bandwidth or high cost links, such as satellite or LoRA.

Within this protocol, only the `HIGH_LATENCY2 <https://mavlink.io/en/messages/common.html#HIGH_LATENCY2>`__ MAVLink message is sent once per 5 seconds. Any responses to MAVLink commands or requests (such as command acknowledgements or parameter requests) will be sent too.

MAVLink High Latency is available as an option on any telemetry port. A typical use case is to have a standard MAVLink telemetry port for initial configuration and system checkout - usually a radio modem or 4G link. A low-bandwidth modem will be connected on a 2nd telemetry port, which the user can activate 

Configuration
=============

- Make sure the desired telemetry port protocol is set to option MAVLink High Latency (``SERIALn_PROTOCOL = 43``)

- In MAVProxy use the ``link hl on|off`` to enable or disable sending telemetry on the High Latency port

- In QGroundControl connect using the ``High Latency`` option to enable sending telemetry on the High Latency port

- In Mission Planner, use ``Actions -> Do Action`` to enable or disable sending telemetry on the High Latency port

Notes
=====

MAVLink High Latency will only send very basic, high level information of the vehicle's position and status.

The bandwidth requirements of MAVLink High Latency are very minimal - less than 100 bytes/sec in most cases.

The enable/disable of a High Latency link can be sent over any telemetry link and will enable/disable all High Latency links on the vehicle.
