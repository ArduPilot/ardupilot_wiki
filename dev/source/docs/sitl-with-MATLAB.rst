.. _sitl-with-MATLAB:

==========================
MATLAB and Simulink
==========================

All Matlab and Simulink functions rely on the `TCP/UDP/IP Toolbox 2.0.6 <https://www.mathworks.com/matlabcentral/fileexchange/345-tcp-udp-ip-toolbox-2-0-6>`__. 
A modified version of the toolbox is provided in the ArduPilot `repository <https://github.com/ArduPilot/ardupilot/tree/master/libraries/SITL/examples/JSON/MATLAB/tcp_udp_ip_2.0.6>`__.
The toolbox is compiled into a `MEX file <https://www.mathworks.com/help/matlab/call-mex-file-functions.html>`__ allowing for fast connection speeds,
also removing the need for a licence for the official TCP/UDP tools. Pre-built MEX files are provided for Windows and Linux however it may be necessary
to compile locally. The `basic MATLAB Simulation copter <https://github.com/ArduPilot/ardupilot/blob/master/libraries/SITL/examples/JSON/MATLAB/Copter/SIM_multicopter.m>`__
example will try to do this if no MEX file is found. This should be run before trying the other examples.

.. toctree::
    :maxdepth: 1

    MATLAB Simulation <MATLAB-Simulation>
    Simulink Simulation <MATLAB-Simulink-Simulation>
    Plotting in real time <MATLAB-Plotting>
    MATLAB serial driver testing<MATLAB-Serial-driver>

.. tip::
  For advice using MATLAB and Simulink with ArduPilot please join the simulation channel on :ref:`Discord<ardupilot-discord-server>`.
