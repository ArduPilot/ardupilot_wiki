.. _sitl-with-JSON:

==========================
JSON interface
==========================

The JSON interface is designed to be easy to implement for those developing new physics backend. It provides a plain text return to ArduPilot 
with easily configurable fields. The interface uses UDP communication and is launched using ``-f JSON:127.0.0.1`` where, if necessary, ``127.0.0.1`` is replaced 
by the IP of the machine where the physics backend is running.

The JSON interface has examples in Python, :ref:`MATLAB and Simulink <sitl-with-MATLAB>`. These along with a detailed description of the protocol can be found in the 
`ArduPilot repository <https://github.com/ArduPilot/ardupilot/tree/master/libraries/SITL/examples/JSON>`__.
