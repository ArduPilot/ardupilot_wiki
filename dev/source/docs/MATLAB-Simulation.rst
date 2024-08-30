.. _MATLAB-Simulation:

==========================
MATLAB Simulation
==========================

.. youtube:: sYCU2ch7oFE
    :width: 100%

MATLAB can be connected directly to ArduPilot using the JSON SITL interface. A connection function is provided along with a copter example. 
The connection function deals with the UDP communications with ArduPilot. The copter example also contains a `function <https://github.com/ArduPilot/ardupilot/blob/afa153fb6fb569419455eb37384a1889971bd5bf/libraries/SITL/examples/JSON/MATLAB/Copter/SIM_multicopter.m#L127>`__
to deal with the 6 dof dynamics of the system using the ArduPilot reference frames and conventions. This allows new vehicle types to only 
output a calculated force and moment given the vehicles state and PWM inputs. The example code is thoroughly commented and can be found in 
the `main repository <https://github.com/ArduPilot/ardupilot/tree/master/libraries/SITL/examples/JSON/MATLAB/Copter>`__.

In order to connect MATLAB run the standard SITL command followed with ``-f JSON:127.0.0.1`` where, if necessary, ``127.0.0.1`` is replaced 
by the IP of the machine where MATLAB is running. There is no requirement for MATLAB and SITL to be on the same system, however firewall 
exceptions may need to be added for both. Both MATLAB and SITL can be stopped and restarted and the other should re-connect. Break-points 
will work as normal.

.. note::
    If you're running Matlab (within windows) and SITL (within WSL2) in the same machine:

    1. If you've enabled mirrored mode networking in WSL2 you have to disable it first. You can read more about this in this article: https://learn.microsoft.com/en-us/windows/wsl/networking#mirrored-mode-networking. (Note: To apply changes you've made to the .wslconfig file, you'll have to restart the WSL.)
    2. You can find the ip address that you should replace 127.0.0.1 with by running ``ipconfig`` command in windows PowerShell. (You have to use the ip address under the Ethernet Adapter vEthernet (WSL (Hyper-V 
       firewall). eg: 172.xx.xx.x)

