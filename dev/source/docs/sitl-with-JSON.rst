.. _sitl-with-JSON:

==========================
JSON interface
==========================

The JSON interface is designed to be easy to implement for those developing new physics backend. It provides a plain text return to ArduPilot 
with easily configurable fields. The interface uses UDP communication and is launched using ``sim_vehichle.py -f JSON:127.0.0.1`` or with the vehicle executable ``--model json:127.0.0.1`` where, if necessary, ``127.0.0.1`` is replaced by the IP of the machine where the physics backend is running.

The JSON interface has examples in Python, :ref:`MATLAB with Simulink <sitl-with-MATLAB>`, and C++. These along with a detailed description of the protocol can be found in the 
`ArduPilot repository <https://github.com/ArduPilot/ardupilot/tree/master/libraries/SITL/examples/JSON>`__.

Global position and free-running (non-lockstep) simulators
==========================================================

The physics backend can send these optional fields in addition to the ones described in the repository:

- ``latitude`` (degrees), ``longitude`` (degrees) and ``altitude`` (meters, AMSL): the vehicle's global position. All three must be sent together. When they are present, they are used instead of ``position``, and ``position`` is then no longer a required field. The first position received sets the simulation's start location.
- ``no_lockstep`` (boolean): set to ``true`` when the physics backend runs in real time and cannot be stepped by ArduPilot. ArduPilot then does not wait for each sensor packet; if none has arrived, it advances its own time at the ``SIM_RATE_HZ`` loop rate. It also does not adjust its frame rate to match the backend.

These fields are useful for connecting existing simulators, such as flight simulators, that keep their own world position and time.

For example:

.. code-block:: json

    {
      "timestamp": 123.456,
      "latitude": -35.363261,
      "longitude": 149.165230,
      "altitude": 584.0,
      "no_lockstep": true,
      "imu": {"gyro": [0.0, 0.0, 0.0], "accel_body": [0.0, 0.0, -9.81]},
      "attitude": [0.0, 0.0, 0.0],
      "velocity": [0.0, 0.0, 0.0]
    }
