.. _MATLAB-Simulink-Simulation:

==========================
Simulink Simulation
==========================

.. youtube:: hTFyMrjwQlI
    :width: 100%

`Connection blocks <https://github.com/ArduPilot/ardupilot/blob/master/libraries/SITL/examples/JSON/MATLAB/AP_Conector.slx>`__ 
are provided to read and write to ArduPilot SITL, along with examples for a `basic rover <https://github.com/ArduPilot/ardupilot/tree/master/libraries/SITL/examples/JSON/MATLAB/Rover>`__ and a `complex helicopter <https://github.com/ArduPilot/ardupilot/tree/master/libraries/SITL/examples/JSON/MATLAB/Heli>`__ 
model. Both examples require additional MATLAB toolboxes. It is recommended to run the :ref:`MATLAB example <MATLAB-Simulation>` 
to test the connection before proceeding to the Simulink examples, the Simulink connection is made using the same ``-f JSON:127.0.0.1`` command.

.. note::
  The Simulink connection library and examples were made using 2020a, older versions of the software can't use these files. Someone 
  with access to 2020a can export for use with a older version, however it may not run as expected. Join the Discord simulation chanel for help.
